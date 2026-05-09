# Ascento 串口命令完整参考

> 版本：2026-05-05
> 用途：所有串口调试命令的完整语法、参数说明和使用示例

---

## 0. 快速开始

```bash
# 连接串口
./scripts/serial.sh

# 退出：Ctrl+A 然后 Ctrl+X (picocom) 或 Ctrl+A \ (minicom)
```

所有命令在串口中直接输入，回车执行。支持 backspace 编辑。

---

## 1. robot — 机器人控制

### 1.1 robot enable — 开启/关闭控制

```text
robot enable <0|1|on|off|true|false|enable|disable>
```

| 参数 | 说明 |
|------|------|
| `1`, `on`, `true`, `enable` | 开启平衡控制 |
| `0`, `off`, `false`, `disable` | 关闭控制（轮电流归零） |

```text
# 例
robot enable 1        # 开启控制
robot enable 0        # 关闭控制（安全停止）
robot status          # 确认 enable=1/0
```

**注意**：命令有 700ms 超时。如果 `robot enable 1` 后 `robot status` 显示 `enable=0`，命令已被老化，重新发送。

---

### 1.2 robot status — 查看完整状态（最常用）

```text
robot status
```

输出格式：
```text
enable=1 wheels=1 fault=0 height=38 joy=(0.0,0.0)
pitch=-0.80 roll=1.20 yaw=0.5 speed=0.0 lqr=1.5 yaw_out=0.0
current=(579,-579) joint=(2.381,1.435) jump=0
```

| 字段 | 含义 | 正常范围 |
|------|------|---------|
| `enable` | 使能状态 (1=已开) | 0/1 |
| `wheels` | 轮输出是否放行 | 0/1 |
| `fault` | pitch 倾角保护触发 | 0/1 |
| `pitch` | 当前 pitch 角度 (°) | −45~45 |
| `roll` | 当前 roll 角度 (°) | −45~45 |
| `yaw` | 偏航角累计 (°) | −180~180 |
| `speed` | 车身速度 (rad/s) | −5~5 |
| `lqr` | 平衡力矩输出 (Nm) | −5~5 |
| `current` | **门检查后**实际发送的左右轮电流 (mA) | −1500~1500 |
| `joint` | 左右关节目标角度 (rad) | 左 2.05~2.90, 右 0.95~1.83 |
| `jump` | 跳跃阶段 (0=未触发) | 0~N |

**诊断速查**：
- `current=(0,0)` 但 `enable=1` → 看 `fault=` 和 `wheels=`
- `fault=1` → 把机器人扶正等待恢复
- `wheels=0` → 轮反馈丢失（`motor wheel status all`）
- pitch/roll 读数异常 → IMU 问题

---

### 1.3 robot pid — 读取/设置 PID 参数

```text
# 读取当前 PID
robot pid

# 设置 PID
robot pid <angle_p> <gyro_p> [distance_p] [speed_p] [limit_mA]
```

```text
# 例
robot pid                           # 读取
robot pid 900.0 35.0                # 只改 angle_p 和 gyro_p
robot pid 900.0 35.0 50.0 1000.0 1500  # 全部参数
```

**注意**：此命令设置的是 PID 平衡控制器的参数。当 `APP_USE_ASCENTO_BALANCE_CONTROLLER=1` 时，实际使用的是 LQR 增益表中的值，PID 参数仅作为 fallback。

---

### 1.4 robot param — 读取/设置 LQR 平衡参数（实时调参）

```text
robot param                          # 显示所有当前参数
robot param list                     # 列出可调参数名
robot param <name> <value>           # 设置单个参数（立即生效，断电丢失）
robot param save                     # 保存到 Flash（永久，断电保留）
robot param reset                    # 恢复代码默认值，清除 Flash 存储
```

**可调参数列表**：

| 参数名 | 说明 | 当前值 |
|--------|------|--------|
| `theta_eq` | 平衡参考角 (rad)，前倾为正 | 0.314159 (18°) |
| `k_pitch` | 固定 K_pitch，姿态 P | -11.75 |
| `k_pitch_rate` | 固定 K_pitch_rate，姿态阻尼 | -4.40 |
| `k_position` | 固定 K_position，位置回正 | 0.90 |
| `k_velocity` | 固定 K_velocity，速度阻尼 | 1.05 |
| `k_yaw_rate` | 偏航角速率增益 | 0.0 |
| `stiction_ma` | Stiction 补偿电流 (mA) | 1900 |
| `stiction_start` | Stiction 起始角度 (deg) | 0.18 |
| `stiction_full` | Stiction 满幅角度 (deg) | 2.40 |
| `current_limit` | 轮子电流上限 (mA) | 3000 |
| `current_scale` | 电流缩放系数 | 1.45 |
| `sync_gain` | 轮子同步增益 (mA/(rad/s)) | 0 |
| `sync_limit` | 轮子同步电流上限 (mA) | 500 |

**示例**：

```text
# 查看所有参数
robot param

# 查看可调参数列表
robot param list

# 调整平衡参考角（例如设置为前倾 18°）
robot param theta_eq 0.314159

# 增大 pitch 增益（更激进）
robot param k_pitch -13.0

# 增大位置回复力（防漂移）
robot param k_position 1.10

# 提高速度阻尼
robot param k_velocity 1.20

# 禁用 stiction 补偿
robot param stiction_ma 0

# 调整后立即生效，无需重启

# 满意后永久保存（断电不丢失）
robot param save

# 不满意？恢复默认值
robot param reset
```

**调参建议**（机器人无法保持平衡时）：

1. 先用 `robot param` 查看当前参数
2. 调 `theta_eq`：扶住机器人找到自然平衡点，让 `err≈0`
3. 调 `k_pitch`：振荡→减小绝对值；倒下→增大绝对值
4. 调 `k_position` / `k_velocity`：漂移→增大
5. `current_limit` 当前保持 `3000 mA`，不要继续加
6. 调 `stiction_ma`：小角度不响应→增大；振荡→减小或设 0
7. 调好后 `robot param save` 永久保存

**持久化**：
- `robot param <name> <value>` — 立即生效，断电丢失
- `robot param save` — 保存到 Flash，断电保留，下次开机自动加载
- `robot param reset` — 恢复代码默认值，清除 Flash 存储
- Flash 存储在 STM32F407 最后 128KB 扇区（NVS），不会覆盖固件

---

### 1.5 robot motion — 运动命令

```text
robot motion <forward|back|left|right|jump|stop>
```

```text
# 例
robot motion forward
robot motion stop
robot stop            # 等价于 robot motion stop
```

---

### 1.6 robot zero — 设置平衡零点

```text
robot zero <deg>      # 手动设置零点角度
robot zero now        # 用当前倾角作为零点（限制 ±8° 内）
```

```text
# 例
robot zero 0.70       # 设零点为 0.70°
robot zero now        # 当前姿态设为零点
```

---

### 1.7 robot height — 设置目标高度

```text
robot height <32..80>
```

---

### 1.8 robot joy — 摇杆输入

```text
robot joy <x:-100..100> <y:-100..100>
```

---

### 1.9 robot jump — 触发跳跃

```text
robot jump
```

---

## 2. motor wheel — 轮毂电机 (VESC/M3508, CAN2)

### 2.1 motor wheel status — 查看轮电机状态

```text
motor wheel status all          # 左右轮一起看
motor wheel status left         # 只看左轮
motor wheel status right        # 只看右轮
motor wheel status <101|100>    # 按 ID
```

输出：
```text
VESC/M3508 id=101 age=23ms erpm=123 motor_rpm=5 angle=0.456 rad speed=2.34 rad/s
cmd=500 mA motor_current=480 mA input=100 mA vin=23.40 V temp=35.2/38.1C
tach=12345 torque_k=0.003457 torque_est=1.234 Nm duty=0.050
s4_age=45ms s5_age=45ms
```

| 字段 | 含义 | 说明 |
|------|------|------|
| `age` | 距上次反馈的毫秒数 | 应 < 50ms |
| `erpm` | 电机电转速 | — |
| `speed` | 输出转速 (rad/s) | — |
| `cmd` | 发送的电流命令 (mA) | **重点：看命令是否发出** |
| `motor_current` | VESC 报告的电机电流 (mA) | **重点：看电机是否执行** |
| `input` | VESC 输入电流 (mA) | 电池侧电流 |
| `vin` | VESC 输入电压 (V) | — |
| `torque_est` | 估算轮端扭矩 (Nm) | — |
| `duty` | VESC PWM 占空比 | 0~1 |
| `s4_age` | Status4 帧的年龄 | 温度/电压数据新鲜度 |
| `s5_age` | Status5 帧的年龄 | 转速/tach 数据新鲜度 |

如果显示 `no feedback`：VESC 不在线。检查 VESC 电源、CAN2 接线、VESC ID 配置。

---

### 2.2 motor wheel current — 手动发送电流

```text
motor wheel current <left|right|id> <current_mA> [duration_ms]
```

```text
# 例
motor wheel current left 100 3000    # 左轮 100mA, 持续 3s
motor wheel current right -200       # 右轮 −200mA, 默认 1s
motor wheel current 101 500 5000     # ID=101, 500mA, 5s
```

**安全限制**：电流被钳位到 `APP_WHEEL_CURRENT_SAFE` (800 mA)。

---

### 2.3 motor wheel pair — 左右轮同时送电流

```text
motor wheel pair <left_mA> <right_mA> [duration_ms]
```

```text
# 例
motor wheel pair 100 -100 3000    # 左前 100mA, 右后 100mA, 3s (原地旋转)
motor wheel pair 200 200 2000     # 两轮同时 200mA, 2s (直线)
```

---

### 2.4 motor wheel rpm — 手动转速控制

```text
motor wheel rpm <left|right|101|100> <target_erpm> [duration_ms]
```

```text
# 例
motor wheel rpm left 500 5000     # 左轮 500 erpm, 5s
```

---

### 2.5 motor wheel stop — 停止手动轮电流

```text
motor wheel stop
```

---

## 3. motor dm — 关节电机 (DM4340, CAN1)

### 3.1 motor dm status — 查看关节电机状态

```text
motor dm status all          # 左右关节一起看
motor dm status left         # 只看左关节
motor dm status right        # 只看右关节
```

输出：
```text
DM4340 id=1 err=0 age=12ms pos=2.3806 rad vel=0.05 rad/s torque=0.123 Nm temp=35/38C
```

| 字段 | 含义 |
|------|------|
| `err` | 错误码 (0=无错误) |
| `age` | 距上次反馈的毫秒数 |
| `pos` | 关节角度 (rad) |
| `vel` | 关节角速度 (rad/s) |
| `torque` | 扭矩估算 (Nm) |
| `temp` | MOS/转子温度 (°C) |

---

### 3.2 motor dm enable / disable — 使能/失能

```text
motor dm enable <left|right|id>
motor dm disable <left|right|id>
```

```text
# 例
motor dm enable left
motor dm disable right
```

---

### 3.3 motor dm zero — 保存机械零点

```text
motor dm zero <left|right|id>
```

将当前位置保存为 DM 电机的机械零点（写入 EEPROM）。**谨慎使用**——会影响四连杆标定。

---

### 3.4 motor dm pos — 位置控制

```text
motor dm pos <left|right|id> <target_rad> [vel_rad_s] [duration_ms]
```

```text
# 例
motor dm pos left 2.381 1.0 3000    # 左关节到 2.381 rad, 速度 1 rad/s, 3s
motor dm pos right 1.435            # 右关节到 1.435 rad, 默认速度 1 rad/s
```

参考值（站立高度 0.115m）：
- 左关节角 ≈ 2.3 rad
- 右关节角 ≈ 1.5 rad

---

### 3.5 motor dm vel — 速度控制

```text
motor dm vel <left|right|id> <rad_s> [duration_ms]
```

```text
# 例
motor dm vel left 2.0 5000     # 左关节 2 rad/s, 5s
```

---

### 3.6 motor dm mit — MIT 模式直接控制

```text
motor dm mit <left|right|id> <pos_rad> <vel_rad_s> <kp> <kd> <torque_nm> [duration_ms]
```

```text
# 例
# 零力模式 (电机浮空)
motor dm mit left 0 0 0 0 0

# 位置伺服 (kp=50 相当于 ~50 N/m 刚度)
motor dm mit left 2.381 0 50.0 8.0 0 5000
```

---

### 3.7 motor dm nudge — 微动当前位置

```text
motor dm nudge <left|right|id> <delta_rad> [kp] [kd] [duration_ms]
```

从当前位置微动 `delta_rad`（±0.10 rad 内），使用 MIT 模式。

```text
# 例
motor dm nudge left 0.05        # 左关节从当前位置 +0.05 rad, 默认 kp=3 kd=0.2
motor dm nudge right -0.03 5.0 0.5 2000  # 右关节 −0.03 rad, kp=5, 2s
```

---

### 3.8 motor dm wiggle — 正弦摆动测试

```text
motor dm wiggle <left|right|id> <amp_rad> [period_ms] [kp] [kd] [duration_ms]
```

```text
# 例
motor dm wiggle left 0.05 2000 12.0 0.5 5000
# 左关节以 ±0.05 rad 摆动, 周期 2s, 持续 5s
```

**用途**：验证关节响应、CAN 延迟、机械间隙。

---

### 3.9 motor dm reg — 读取 DM 寄存器

```text
motor dm reg <left|right|id> <rid>
```

常用寄存器 ID：

| rid | 名称 | 说明 |
|-----|------|------|
| 0x01 | FAULT | 故障码 |
| 0x02 | WARNING | 警告码 |
| 0x04 | STATUS | 运行状态 |
| 0x09 | TIMEOUT | CAN 超时时间 (ms) |
| 0x0a | CTRL_MODE | 控制模式 (1=MIT, 2=pos_vel, 3=velocity) |
| 0x15 | PMAX | 最大位置 (rad) |
| 0x16 | VMAX | 最大速度 (rad/s) |
| 0x17 | TMAX | 最大扭矩 (Nm) |
| 0x3b | Imax | 最大电流 (A) |
| 0x3c | VBus | 母线电压 (V) |

---

### 3.10 motor dm diag — 完整诊断

```text
motor dm diag <left|right|id>
```

一次性读取所有关键诊断寄存器（FAULT/WARNING/STATUS/CAN_ERR/MOTOR_ERR/TIMEOUT/CTRL_MODE/PMAX/VMAX/TMAX/Imax/VBus）。

**用途**：排查 DM 电机配置或故障。

---

### 3.11 motor dm stop — 停止手动调试

```text
motor dm stop <left|right|id>
```

发送零力 MIT 命令使关节失力。

---

### 3.12 motor dm rxlog — 打印 DM CAN 接收日志

```text
motor dm rxlog
```

---

## 4. motor can — CAN 总线

### 4.1 motor can status — CAN 总线状态

```text
motor can status all                 # 两条 CAN 总线
motor can status joint               # 只看 CAN1 (关节)
motor can status wheel               # 只看 CAN2 (轮)
```

输出：
```text
joint/CAN1 state=error-active tx_err=0 rx_err=0
wheel/CAN2 state=error-active tx_err=0 rx_err=0
```

| 状态 | 含义 |
|------|------|
| `error-active` | 正常 |
| `error-warning` | 有错误但可通信 |
| `error-passive` | 错误较多，发送受限 |
| `bus-off` | **总线断开** — 检查终端电阻和接线 |

---

### 4.2 motor can raw / rawx — 手动发送 CAN 帧

```text
# 标准帧 (11-bit ID)
motor can raw <joint|wheel|can1|can2> <std_id> [byte0..byte7]

# 扩展帧 (29-bit ID)
motor can rawx <joint|wheel|can1|can2> <ext_id> [byte0..byte7]
```

```text
# 例
# DM4340 使能命令 (ID=1, 扩展帧, data=FF FF FF FF FF FF FF FC)
motor can rawx joint 1 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFC
```

---

### 4.3 motor can recover — CAN 总线恢复

```text
motor can recover all           # 两条总线重新启动
motor can recover joint         # 只恢复 CAN1
motor can recover wheel         # 只恢复 CAN2
```

当 CAN 进入 bus-off 状态时使用。

---

## 5. motor debug — 调试模式管理

### 5.1 motor debug status — 查看调试状态

```text
motor debug status
```

显示当前是否有手动调试命令在运行。

---

### 5.2 motor debug stop — 停止所有调试

```text
motor debug stop
```

停止所有手动电机调试命令（轮 + 关节），恢复正常控制循环。

**重要**：用完后必须 `motor debug stop`，否则机器人不会响应 `robot enable 1`。如果关节无力，按复位键重新走一遍使能流程。

---

## 6. 常用命令序列

### 6.1 启动检查

```text
robot status                     # 看 firmware 版本/状态
motor can status all             # CAN 总线是否正常
motor wheel status all           # 轮电机反馈是否在线
motor dm status all              # 关节电机反馈是否在线
```

预期：
- CAN1/CAN2: `error-active`, tx_err=0, rx_err=0
- 轮电机 age < 50ms, 无 "no feedback"
- 关节电机 age < 20ms, err=0

---

### 6.2 架空方向测试

```text
# 0. 前置：确认 IMU 轴正确
robot status                     # 直立 → 记 pitch/roll
# 前倾 15° → robot status        # 预期：pitch 变, roll 不变
# 左倾 15° → robot status        # 预期：roll 变, pitch 不变

# 1. 开启控制
robot enable 1
robot status                     # 确认 enable=1

# 2. 观察自动日志 (~1Hz)
# [ascento] active | pitch=... err=... cur=... L=... K=...

# 3. 前倾 → 观察轮子方向
# 前倾 → cur 正值 → 轮子向前转 ✓
# 后仰 → cur 负值 → 轮子向后转 ✓

# 4. 关闭
robot enable 0
```

---

### 6.3 轮子不转排查

```text
# 看模型输出
# [ascento] 日志中 cur=%d/%d → 模型算出的电流

# 看实际发送
robot status | grep current → current=(左,右)

# 如果 cur 非零但 current=0/0 → 门关掉了
#   检查 enable=/wheels=/fault=

# 如果 current 非零但轮子不动 → VESC 侧
motor wheel status all | grep cmd/motor_current
#   如果 cmd 非零, motor_current≈0 → VESC 未执行 (断线/故障)
#   如果 cmd 非零, motor_current 非零 → 机械卡死
```

---

### 6.4 关节无力排查

```text
motor dm status all                  # 看 pos 是否有读数
# 如果无反馈 → 检查 DM 电源/CAN1 接线

motor dm diag left                   # 看 CTRL_MODE 是否为 MIT(1)
# 如果是 unknown(0) → DM 未使能

# 按复位键重启（推荐，走完整使能流程）
# 或手动使能：
motor dm enable left
motor dm enable right
```

---

### 6.5 安全停止

```text
robot enable 0              # 关闭平衡控制
motor debug stop            # 停止所有手动调试
motor wheel stop            # 轮电流归零
motor dm stop left          # 左关节失力
motor dm stop right         # 右关节失力
```

---

### 6.6 轮子手动方向/极性测试

```text
# 左轮前进方向测试
motor wheel current left 100 3000
# 观察轮子转向

# 右轮前进方向测试
motor wheel current right 100 3000
# 观察转向是否与左轮一致（应相反，因为对侧安装）

# 原地旋转测试
motor wheel pair 100 -100 3000

# 停止
motor wheel stop
```

---

### 6.7 关节角度调整（测量/标定用）

```text
# 安全停止
robot enable 0
motor debug stop

# 读取当前角度
motor dm status all

# 微动调整
motor dm nudge left 0.05          # 左关节 +0.05 rad
motor dm nudge right -0.03        # 右关节 −0.03 rad

# 直接指定位置
motor dm pos left 2.381 1.0 5000
motor dm pos right 1.435 1.0 5000
```

---

## 7. 串口日志解读

### 7.1 [ascento] 日志（~1Hz，模型内部）

```
[ascento] active | pitch=0.03 err=-0.40 torque=-2.00 cur=-579/-579 L=0.100 K=-4.95/-0.88/-0.63/-0.94
```

| 字段 | 含义 |
|------|------|
| `pitch` | 模型坐标系 pitch (rad)，正=前倾 |
| `err` | pitch − theta_eq，正=偏前 |
| `torque` | 平衡力矩 (Nm) |
| `cur` | **模型输出的**左右轮电流 (mA) |
| `L` | 当前平均腿长 (m) |
| `K` | 增益 K_pitch/K_prate/K_pos/K_vel |

---

### 7.2 [ascento] model blocked 日志

```
[ascento] model blocked: params_ready=1 enable_req=0 wheel_fb_ok=1 calib=1
```

| 字段 | 为 0 时的含义 |
|------|-------------|
| `enable_req=0` | 未执行 `robot enable 1` |
| `wheel_fb_ok=0` | VESC 反馈超时 |
| `params_ready=0` | 物理参数未标定 |
| `calib=0` | `APP_ASCENTO_PARAMS_CALIBRATED` 为 0 |

---

### 7.3 [ascento] faulted 日志

```
[ascento] faulted | pitch=-0.80 theta_eq=0.00 err_deg=45.8 recover=0/200
```

| 字段 | 含义 |
|------|------|
| `err_deg` | 当前倾角偏差绝对值 (°) |
| `recover` | 恢复计数器（需在恢复阈值内持续 >200 tick 才恢复）|

---

### 7.4 [app] 日志（2Hz，实际状态）

```
pitch -0.80 roll -1.03 yaw -0.2 gy 0.1 gz -0.1 current 0/0 height 38 batt 23.20 V
```

| 字段 | 含义 |
|------|------|
| `pitch` | 原始 IMU pitch (°)，**未**经模型符号修正 |
| `roll` | 原始 IMU roll (°) |
| `current` | **门检查后**实际发送的电流 (mA) |
| `batt` | 电池电压 (V) |

**诊断技巧**：对比 `[ascento]` 的 `cur=` 和 `[app]` 的 `current=`：
- 两者一致 → 门正常放行
- `cur` 非零, `current=0,0` → 门关掉了（查 `robot status` 的 enable=/wheels=/fault=）

---

## 8. 三层诊断数据对照

| 数据层 | 来源 | 频率 | 看什么 |
|--------|------|------|--------|
| 模型内部 | `[ascento]` 日志 | ~1 Hz | 模型算了什么电流 (`cur=`) |
| 控制器状态 | `robot status` | 按需 | 哪个门关掉了 (`enable/wheels/fault`) |
| 实际电机 | `motor wheel/dm status` | 按需 | 反馈是否在线、cmd vs motor_current |

排查顺序：**模型内部 → 门状态 → 电机实际**。跳过任一层都会导致误判。

---

## 9. 关联文档

- [ASCENTO_WHEEL_NOT_ROTATE_DEBUG_MANUAL_ZH.md](ASCENTO_WHEEL_NOT_ROTATE_DEBUG_MANUAL_ZH.md) — 轮子不转排查手册
- [MOTOR_DEBUG_README_ZH.md](MOTOR_DEBUG_README_ZH.md) — 电机接线、调试、VESC 协议和力矩系数
