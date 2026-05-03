# CAN 总线调试指南 —— DM4340 右电机排查实录

本文档记录在 Zephyr RTOS 上调试 DM4340 关节电机的完整过程，适合初学 CAN 总线与电机驱动的同学参考。

## 1. 背景

**硬件平台：**
- 主控：DJI RoboMaster Development Board Type C (STM32F407IGH6)
- 关节电机：两个 DM4340 减速电机，挂在 CAN1 总线上（PD0/PD1）
- 轮毂电机：两个 M3508/VESC，挂在 CAN2 总线上（PB5/PB6）
- 烧录：ST-LINK V2 (SWD)
- 串口：CH340 USB-TTL 连接 USART1 (PA9 TX, PB7 RX, 115200 8N1)

**软件平台：**
- Zephyr RTOS v4.1.0
- CAN 总线速率：1 Mbps
- 日志级别：INFO，立即模式 (`CONFIG_LOG_MODE_IMMEDIATE`)

**DM4340 通信协议要点：**
```
发送（控制器 → 电机）：
  - enable:  CAN ID = 电机ID (左=0x01, 右=0x02), data[7]=0xFC
  - disable: CAN ID = 电机ID, data[7]=0xFD
  - 位置+速度: CAN ID = 0x100 + 电机ID, 8字节（两个float32）
  - 速度模式: CAN ID = 0x200 + 电机ID, 4字节（一个float32）
  - MIT模式:  CAN ID = 电机ID, 8字节（uint16+uint12+uint12+uint12+uint12）

接收（电机 → 控制器）：
  - 左电机反馈 CAN ID = 0x00
  - 右电机反馈 CAN ID = 0x03
  - 帧格式：8字节，包含位置(uint16)、速度(uint12)、扭矩(uint12)、温度×2
  - data[0] bit3-0 = 电机ID, data[0] bit7-4 = 错误码
```

## 2. 排查过程

### 阶段 1：左电机正常，右电机无反馈

**现象：**
```
uart:~$ motor dm status all
DM4340 id=1 err=1 age=1ms  pos=2.2185 rad vel=-0.0110 rad/s torque=0.004 Nm temp=32/30C
DM4340 id=2 no feedback
```

**排查步骤：**

1. 确认两个电机都在 CAN1 总线上 ✓
2. 确认右电机 CAN ID 已设为 02（通过达妙上位机） ✓
3. 确认右电机硬件正常（用达妙上位机直连可以工作） ✓
4. 检查代码：CAN 滤波器只注册了 CAN ID 0x00 —— **发现问题！**

**根因：** 代码只过滤左电机的反馈 ID (0x00)，没有为右电机的反馈 ID (0x03) 注册滤波器。

### 阶段 2：添加第二个滤波器

**代码改动（dm4340.c）：**
```c
// 原来只过滤 0x00
const struct can_filter filter_0x00 = {
    .flags = 0,
    .id = 0x000,
    .mask = 0x7ff,  // 完全匹配
};

// 添加第二个滤波器
const struct can_filter filter_0x03 = {
    .flags = 0,
    .id = 0x003,
    .mask = 0x7ff,
};
can_add_rx_filter(can, dm4340_rx_cb, bus, &filter_0x03);
```

**结果：** 仍然无反馈。改为 accept-all 滤波器（mask=0x000），仍然无反馈。

### 阶段 3：CAN 总线 bus-off 问题

重新上电后，CAN 状态变为：

```
joint/CAN1 state=bus-off tx_err=0 rx_err=95
wheel/CAN2 state=bus-off tx_err=0 rx_err=120
```

**什么是 bus-off？**

CAN 控制器有一个错误计数机制：
- `tx_err`：发送错误计数器，每次发送失败 +8
- `rx_err`：接收错误计数器，每次接收错误 +1
- 当 `tx_err > 255` 时，控制器进入 **bus-off** 状态，完全断开与总线的连接
- 恢复需要：连续检测到 128 次 11 个隐性位（总线空闲）

**为什么会进入 bus-off？**

代码中 `control_thread` 以 1000Hz 频率向两个 DM4340 电机发送位置指令（见 `main.c:205-213`）。如果电机未上电/未连接，没有 CAN 节点来 ACK 这些帧，每次发送失败 +8，约 32 次发送后就会达到 bus-off（32 × 8 = 256 > 255）。

即使电机后来上电，bus-off 状态下控制器也无法通信。

**经验教训：** CAN 总线必须有至少 2 个节点才能正常工作（一个发、一个 ACK）。单节点发送必然进入 bus-off。

### 阶段 4：修复 bus-off 问题

**方案 A：自动恢复（ABOM）**

STM32 的 CAN 外设有 ABOM (Automatic Bus-Off Management) 位。启用后，bus-off 会自动恢复（约 1.4ms @ 1Mbps）。

Zephyr 驱动在初始化时已经启用了 ABOM（`can_stm32_bxcan.c:660`）：
```c
can->MCR |= CAN_MCR_ABOM;
```

但自动恢复需要总线空闲（128 次 × 11 个隐性位）。如果总线有噪声或没有终端电阻，总线永远不会空闲，恢复不会发生。

**方案 B：手动恢复**

`can_stop()` + `can_start()` 可以强制退出 bus-off：
```c
can_stop(dev);   // 进入初始化模式，重置错误计数器
can_start(dev);  // 退出初始化模式，回到 error-active
```

我们添加了 shell 命令 `motor can recover <bus>` 用于手动恢复。

**方案 C：预防 bus-off**

在 DM4340 发送函数中添加 CAN 状态检查，bus-off 时跳过发送：
```c
static bool can_ok(const struct device *can)
{
    enum can_state state;
    struct can_bus_err_cnt err_cnt;
    if (can_get_state(can, &state, &err_cnt) != 0) return false;
    return state != CAN_STATE_BUS_OFF && state != CAN_STATE_STOPPED;
}

// 在每个发送函数开头：
if (!can_ok(bus->can)) {
    return -ENETDOWN;
}
```

## 3. 添加的调试工具

### 3.1 CAN 状态查看
```bash
motor can status all        # 查看 CAN1/CAN2 状态和错误计数器
motor can status can1       # 仅查看 CAN1
```

### 3.2 CAN 总线恢复
```bash
motor can recover all       # 恢复所有 CAN 总线
motor can recover can1      # 仅恢复 CAN1（can_stop + can_start）
```

### 3.3 原始 CAN 帧捕获

`dm4340_rx_cb` 会将所有收到的 CAN 帧（含非 DM4340 格式的帧）记录到环形缓冲区（128 条）。每条记录去重合并（相同 ID+数据 只计数）。

通过 shell 查看：
```bash
motor dm rxlog              # 打印 CAN1 上收到的所有帧
```

### 3.4 RAW CAN 发送
```bash
motor can raw can1 2 255 255 255 255 255 255 255 252   # 向 CAN1 上 ID=2 发送 enable 命令
motor can raw can1 2 255 255 255 255 255 255 255 253   # 发送 disable 命令
```

## 4. 后续排查方向

截至本文档撰写时，右电机（ID=2）仍未收到反馈。已知：

1. **左电机完全正常** —— CAN 通信、反馈、控制都工作
2. **CAN 总线健康** —— error-active, tx_err=0, rx_err=0
3. **右电机 enable 返回成功** (ret=0) —— 命令已发送到 CAN 总线
4. **右电机硬件正常** —— 用达妙上位机直连可以工作

**待排查的假设：**
- [ ] 右电机可能需要额外的初始化命令（不同于左电机）
- [ ] 右电机可能使用不同的反馈帧格式（非标准 DM4340 格式）
- [ ] 右电机反馈 CAN ID 可能不是 0x03（需通过 `motor dm rxlog` 确认）
- [ ] 右电机可能需要特定 CAN ID 的 MIT/位置命令才能激活反馈
- [ ] 两个电机不能同时挂在同一 CAN 总线上使用标准 DM4340 协议（ID 冲突？）

## 5. 关键代码文件

| 文件 | 作用 |
|------|------|
| [src/dm4340.c](../src/dm4340.c) | DM4340 驱动：CAN 发送/接收/反馈解析 |
| [src/dm4340.h](../src/dm4340.h) | DM4340 数据结构定义 |
| [src/main.c](../src/main.c) | 主程序：CAN 初始化、控制线程 |
| [src/shell_commands.c](../src/shell_commands.c) | Shell 命令：motor dm/can 等调试命令 |
| [src/app_config.h](../src/app_config.h) | 配置：电机 ID、CAN ID 定义 |
| [boards/arm/dji_f407igh6_c/dji_f407igh6_c.dts](../boards/arm/dji_f407igh6_c/dji_f407igh6_c.dts) | 设备树：CAN 引脚、波特率 |

## 6. 常用操作（速查）

```bash
# ---- 系统 ----
robot enable 0             # 关闭平衡控制（停止控制线程发送 DM 指令）
robot status               # 查看机器人状态

# ---- DM 电机 ----
motor dm status all        # 查看所有 DM 电机状态
motor dm enable left       # 使能左电机 (ID=1)
motor dm enable right      # 使能右电机 (ID=2)
motor dm disable left      # 禁用左电机
motor dm vel left 1.0 500  # 左电机以 1.0 rad/s 转 500ms
motor dm pos left 1.0 2.0  # 左电机转到 1.0 rad, 速度上限 2.0 rad/s
motor dm stop left         # 停止左电机的调试指令

# ---- CAN 总线 ----
motor can status all       # 查看 CAN 状态
motor can recover all      # 恢复所有 CAN 总线
motor dm rxlog             # 查看 CAN1 原始帧捕获

# ---- Wheel 电机 ----
motor wheel status all     # 查看所有轮毂电机状态
motor wheel rpm left 1000  # 左轮毂 1000 ERPM
```

## 7. 关键概念总结

1. **CAN 总线是差分信号**，需要至少 2 个节点 + 两端各 120Ω 终端电阻
2. **bus-off** 是 CAN 控制器的保护机制，进入后完全断开总线。需等待恢复或手动 `can_stop/can_start`
3. **CAN 滤波器** 在硬件层面过滤帧。`mask=0x000` 表示接受所有帧（不匹配任何位）
4. **DM4340 的 CAN ID** 分为命令 ID（发）和反馈 ID（收），两者不同
5. **can_send 返回 0** 只表示帧已写入邮箱，不表示对方收到。需要检查反馈帧来确认通信
6. **Zephyr 的 printk 在 shell 模式下可能不显示**，推荐用环形缓冲区 + shell 命令来调试
