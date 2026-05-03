# DM4340 右关节电机 CAN 调试记录

这份记录面向刚开始调试 CAN 电机的同学。目标不是只给结论，而是把“怎么一步一步判断问题在哪里”写清楚。

## 1. 本次硬件和现象

- 主控：DJI RoboMaster Development Board Type C，STM32F407IGH6。
- 关节电机：两个达妙 DM4340，挂在 CAN1。
- 串口：`/dev/ttyUSB0`，115200 8N1。
- 烧录：ST-LINK，SWD 4 线。
- 初始现象：左关节电机能回传，右关节电机通信异常，怀疑两个电机 ID 相同、反馈 ID 配错，或 CAN 总线干扰。

本次确认的正确 ID 是：

| 电机 | 控制器发给电机的 CAN ID | 电机回传给控制器的 CAN ID |
|------|--------------------------|----------------------------|
| 左关节 | `0x01` | `0x11` |
| 右关节 | `0x02` | `0x12` |

注意：达妙电机的“控制 ID”和“Master/反馈 ID”不是同一个概念。控制命令发到电机 ID，反馈帧从 Master ID 回来。本次右电机不是 `0x03`，而是 `0x12`。

## 2. 代码改动

### 2.1 固定正确的电机 ID

在 [src/app_config.h](../src/app_config.h) 中配置：

```c
#define APP_DM_LEFT_ID 1
#define APP_DM_RIGHT_ID 2
#define APP_DM_LEFT_FEEDBACK_ID 0x11
#define APP_DM_RIGHT_FEEDBACK_ID 0x12
```

### 2.2 按反馈帧 CAN ID 区分左右电机

在 [src/dm4340.c](../src/dm4340.c) 中，反馈解析先检查标准帧 ID：

- `0x11` 解析成左电机，也就是 `id = 1`
- `0x12` 解析成右电机，也就是 `id = 2`

不要只看 `data[0]`，因为现场排查时最先要确认的是“这帧到底来自哪个反馈 CAN ID”。

### 2.3 不再扫描 0 到 15

早期为了找右电机 ID，曾经把 enable 和控制指令扫过 `0..15`。确认 ID 后已经改回只控制：

- `APP_DM_LEFT_ID`
- `APP_DM_RIGHT_ID`

这样更安全，也能减少 CAN 总线上的无关帧。

### 2.4 增加调试命令

新增或使用了这些 shell 命令：

```text
motor can status all
motor dm status all
motor dm status left
motor dm status right
motor dm rxlog
motor dm nudge <left|right|id> <delta_rad> [kp] [kd] [ms]
motor dm stop <left|right|id>
motor debug stop
```

其中 `motor dm nudge` 会读取当前反馈位置，再在当前位置附近给一个小角度 MIT 目标，避免直接跳到一个危险的绝对角度。

## 3. CAN 原始帧到物理量的换算

达妙 DM4340 反馈帧是 8 字节。以本项目代码中的范围为准：

- 位置范围：`[-12.5, 12.5] rad`
- 速度范围：`[-45, 45] rad/s`
- 扭矩范围：`[-18, 18] Nm`

反馈帧格式：

```text
data[0] 高 4 bit = 电机状态/错误码
data[0] 低 4 bit = 电机 ID
data[1:2] = 位置 uint16
data[3] + data[4] 高 4 bit = 速度 uint12
data[4] 低 4 bit + data[5] = 扭矩 uint12
data[6] = MOS 温度
data[7] = 转子温度
```

代码中的解析方式：

```c
p_int = (data[1] << 8) | data[2];
v_int = (data[3] << 4) | (data[4] >> 4);
t_int = ((data[4] & 0x0f) << 8) | data[5];
```

整数转物理量的通用公式：

```text
physical = raw * (max - min) / ((1 << bits) - 1) + min
```

对应到本项目：

```text
position_rad   = p_int * 25.0 / 65535 + (-12.5)
velocity_rad_s = v_int * 90.0 / 4095 + (-45.0)
torque_nm      = t_int * 36.0 / 4095 + (-18.0)
```

本次用 `motor dm status all` 已经看到两台电机都能把 CAN 反馈转成具体物理数据：

```text
DM4340 id=1 err=1 age=1ms pos=2.2185 rad vel=-0.0110 rad/s torque=-0.004 Nm temp=32/30C
DM4340 id=2 err=1 age=1ms pos=0.6754 rad vel=-0.0110 rad/s torque=-0.004 Nm temp=32/30C
```

结论：反馈解析链路是通的，能从 CAN 帧得到位置、速度、扭矩、MOS 温度、转子温度。

## 4. 编译和烧录

README 中的常规命令是：

```bash
./scripts/build.sh && ./scripts/flash.sh
```

本次现场 `flash.sh` 和 `openocd_flash_fast_4pin.sh` 都遇到过 OpenOCD flash algorithm timeout。最终稳定成功的办法是整片擦除后写入并校验：

```bash
./scripts/build.sh
./scripts/openocd_mass_erase_flash.sh
```

成功日志里能看到类似信息：

```text
stm32x mass erase complete
wrote 146388 bytes from file build/zephyr/zephyr.hex
verified 146327 bytes
```

如果只是想单独校验当前板子里的固件，可以用 OpenOCD：

```bash
source ./zephyr-env.sh >/dev/null
OPENOCD="${ZEPHYR_SDK_INSTALL_DIR}/sysroots/x86_64-pokysdk-linux/usr/bin/openocd"
SCRIPTS="${ZEPHYR_SDK_INSTALL_DIR}/sysroots/x86_64-pokysdk-linux/usr/share/openocd/scripts"
"$OPENOCD" -s "$SCRIPTS" -f interface/stlink.cfg \
  -c 'transport select hla_swd' \
  -c 'set WORKAREASIZE 0x1000' \
  -f target/stm32f4x.cfg \
  -c 'adapter speed 100' \
  -c init -c targets -c halt \
  -c 'verify_image build/zephyr/zephyr.bin 0x08000000 bin' \
  -c 'reset run' -c shutdown
```

## 5. 串口连接和权限

如果串口没有权限，本次环境的 sudo 密码为 `1`，可以执行：

```bash
printf '1\n' | sudo -S chmod a+rw /dev/ttyUSB0
```

打开串口：

```bash
picocom --echo -b 115200 /dev/ttyUSB0
```

退出 `picocom`：按 `Ctrl-A`，再按 `Ctrl-X`。

串口中不建议一次粘贴太多命令。现场曾出现：

```text
<wrn> shell_uart: RX ring buffer full.
```

所以调试时尽量一条命令执行完再发下一条。

## 6. 本次实际测试步骤

### 6.1 看 CAN 总线状态

```text
motor can status all
```

正常结果：

```text
joint/CAN1 state=error-active tx_err=0 rx_err=0
wheel/CAN2 state=error-active tx_err=0 rx_err=0
```

含义：

- `error-active` 是正常可通信状态。
- `tx_err=0 rx_err=0` 说明当前没有明显总线干扰、ACK 失败或收发错误累计。

### 6.2 看两台 DM4340 的物理反馈

```text
motor dm status all
```

本次结果：

```text
DM4340 id=1 err=1 age=1ms pos=2.2185 rad vel=-0.0110 rad/s torque=-0.004 Nm temp=32/30C
DM4340 id=2 err=1 age=1ms pos=0.6754 rad vel=-0.0110 rad/s torque=-0.004 Nm temp=32/30C
```

含义：

- 左右电机都有反馈，不再是右电机 `no feedback`。
- `age` 只有 0 到 2 ms，说明反馈在持续刷新。
- `pos/vel/torque/temp` 都是从 CAN 原始帧换算出的物理量。

### 6.3 看原始 CAN 帧

```text
motor dm rxlog
```

本次看到持续出现标准帧：

```text
id=0x011 ... data=1196b77ff7...
id=0x012 ... data=1286ea7ff7...
```

判断方法：

- `id=0x011` 是左电机反馈帧。
- `id=0x012` 是右电机反馈帧。
- `data[0] = 0x11` 时，高 4 bit 为状态 `1`，低 4 bit 为电机 ID `1`。
- `data[0] = 0x12` 时，高 4 bit 为状态 `1`，低 4 bit 为电机 ID `2`。

这一步可以同时排除“两个电机反馈 ID 相同”的问题。

### 6.4 小幅度测试关节电机

先用很小的角度和较低刚度测试。本次最终复测使用了 `0.01 rad`，大约 0.57 度：

```text
motor dm nudge left 0.01 3.0 0.2 500
motor dm status left
motor dm nudge right 0.01 3.0 0.2 500
motor dm status right
```

如果确认机械结构安全、没有卡死，也可以做 `0.02 rad` 的反方向小幅测试：

```text
motor dm nudge left -0.02 15.0 0.5 1000
motor dm status left
motor dm nudge right -0.02 15.0 0.5 1000
motor dm status right
```

本次测试结果：

```text
motor dm nudge left 0.01 3.0 0.2 500
DM4340 id=1 nudge from 2.2181 to 2.2281 rad delta=0.0100 kp=3.00 kd=0.20 for 500 ms
DM4340 id=1 err=1 age=0ms pos=2.2185 rad vel=-0.0110 rad/s torque=0.022 Nm temp=33/30C

motor dm nudge right 0.01 3.0 0.2 500
DM4340 id=2 nudge from 0.6754 to 0.6854 rad delta=0.0100 kp=3.00 kd=0.20 for 500 ms
DM4340 id=2 err=1 age=1ms pos=0.6754 rad vel=-0.0110 rad/s torque=0.013 Nm temp=33/31C

joint/CAN1 state=error-active tx_err=0 rx_err=0
```

结论：

- CAN 通信稳定，右电机不是因为反馈 ID 或总线干扰导致离线。
- 两台电机能持续反馈物理量，说明接收和解析无误。
- 小幅 MIT 指令发出后扭矩有变化，但位置没有明显变化。这个现象更像是控制模式、上位机参数、机械限位/负载、增益过低或电机内部模式配置问题，而不是 CAN ID 问题。

测试结束后执行：

```text
motor debug stop
motor dm stop left
motor dm stop right
motor can status all
```

## 7. 初学者排查流程

### 第一步：只看总线是否健康

```text
motor can status all
```

如果出现 `bus-off`，先不要继续发控制命令。检查：

- 电机是否上电。
- CANH/CANL 是否接反。
- 终端电阻是否合理。
- 总线上是否至少有一个节点能 ACK。
- 波特率是否一致。

### 第二步：看有没有反馈物理量

```text
motor dm status all
```

如果某个电机 `no feedback`，优先检查：

- 控制 ID 是否正确。
- 反馈/Master ID 是否正确。
- 代码里的反馈 ID 是否和上位机配置一致。
- CAN 滤波器是否把该反馈 ID 过滤掉了。

### 第三步：看原始帧

```text
motor dm rxlog
```

原始帧能告诉你两件事：

- 电机到底用哪个 CAN ID 回传。
- `data[0]` 低 4 bit 里的电机 ID 是否符合预期。

本次右电机从 `0x12` 回传，所以代码必须接受并解析 `0x12`。

### 第四步：只做当前位置附近的小动作

优先用：

```text
motor dm nudge right 0.02 3.0 0.2 1000
```

不要一上来用绝对位置命令，例如直接让电机去 `0 rad`。如果当前机械角度离 `0 rad` 很远，电机会突然大幅动作。

### 第五步：小动作没动时怎么判断

如果 `nudge` 后：

- CAN 仍然 `tx_err=0 rx_err=0`
- 反馈仍然持续刷新
- 扭矩有变化
- 位置基本不变

通常优先怀疑控制层或机械层：

- 达妙上位机里的控制模式是否允许 MIT 控制。
- 电机是否处于使能状态。
- 机械关节是否被限位、卡住或负载过大。
- `kp/kd` 是否太低。
- 电机内部参数、零点或方向配置是否不符合当前机构。

这时不要马上把增益拉很高。先拆小负载或抬空机构，再逐步增加角度和增益。

## 8. 本次结论

1. 右关节电机异常的主要问题不是总线干扰；测试时 CAN1 为 `error-active`，`tx_err=0`，`rx_err=0`。
2. 右关节电机异常的主要问题也不是两个电机 ID 相同；原始帧中左电机为 `0x11`，右电机为 `0x12`。
3. 代码已经按 `0x11/0x12` 解析反馈，并能把 CAN 原始数据转换为位置、速度、扭矩和温度。
4. 小幅测试中两台电机反馈稳定，扭矩响应存在，但关节位置未明显变化，后续应重点检查达妙电机控制模式、上位机参数、机械约束和 MIT 增益设置。

## 9. 右关节空载但电源电流不变时的进一步排查

用户确认右关节电机处于空载状态，执行 MIT 小幅往复和 torque-only 测试时，供电电源电流没有变化。这个现象比“关节不动”更关键：

- 空载 MIT 力矩控制下，哪怕很小的力矩也应让电机有加速趋势。
- 如果电源电流完全不变，说明驱动器很可能没有真正给三相功率级输出电流。
- 此时反馈帧里的 `torque` 不能单独当作真实出力证据，它可能是驱动器内部估计值、命令相关值，或在错误量程下被误解析的值。

本轮新增了达妙寄存器读取命令：

```text
motor dm reg <left|right|id> <rid>
```

常用寄存器：

```text
motor dm reg right 0x0a   # CTRL_MODE，1 表示 MIT
motor dm reg right 0x07   # MST_ID，反馈 ID
motor dm reg right 0x08   # ESC_ID，接收 ID
motor dm reg right 0x15   # PMAX
motor dm reg right 0x16   # VMAX
motor dm reg right 0x17   # TMAX
motor dm reg right 0x3c   # VBus
motor dm reg right 0x3b   # Imax
motor dm reg right 0x03   # OC_Value
```

实测右关节：

```text
CTRL_MODE(0x0a) u32=1 mode=MIT
MST_ID(0x07) u32=18 raw=0x00000012
ESC_ID(0x08) u32=2 raw=0x00000002
VBus(0x3c) f=23.875048
Imax(0x3b) f=20.522388
OC_Value(0x03) f=0.800000
PMAX(0x15) f=12.500000
VMAX(0x16) f=10.000000
TMAX(0x17) f=28.000000
```

左关节也读到 `VMAX=10.0`、`TMAX=28.0`。原固件里 MIT 换算使用的是 `VMAX=45.0`、`TMAX=18.0`，这会导致速度和扭矩字段的物理换算不准。本轮已经把 [src/dm4340.c](../src/dm4340.c) 中的 MIT 量程修正为：

```c
.p_min = -12.5f, .p_max = 12.5f,
.v_min = -10.0f, .v_max = 10.0f,
.t_min = -28.0f, .t_max = 28.0f,
```

同时修正了两个安全点：

- `motor dm stop` 优先发送 MIT 零力矩，再兼容发送速度零命令。
- 手动调试命令到期后，主循环会主动补发左右关节 MIT 零力矩，不再只依赖电机 CAN timeout。

修正量程并重新烧录后，右关节测试：

```text
motor dm wiggle right 0.02 2000 12 0.5 2000
motor dm wiggle right 0.04 2000 20 0.8 5000
motor dm mit right 0 0 0 0 0.10 300
```

串口侧仍显示 CAN 正常、反馈在线、状态为使能：

```text
joint/CAN1 state=error-active tx_err=0 rx_err=0
DM4340 id=2 err=1 age=1ms pos=0.6758 rad vel=-0.0024 rad/s torque=0.007 Nm temp=35/33C
```

如果电源侧仍然没有任何电流变化，则下一步不要继续盲目加大力矩。优先检查：

- 达妙驱动器是否有保护/锁定/零力矩/拖动类状态没有解除。
- 上位机里是否能单独用 MIT 力矩模式让这只电机空载转动。
- 电机三相线、驱动功率板、相电流采样或门极驱动是否异常。
- 驱动器指示灯是否真的处于绿色使能状态，而不是仅 CAN 状态位显示使能。
- 同样参数下左电机是否能出力；如果左能右不能，基本可定位到右电机或右驱动器硬件/参数。

## 10. 电机内部驱动与使能深度排查

本章的排查目标是：**确定电机反馈在线、CAN 通信正常、MIT 指令已发送的情况下，为什么电机不转动且电源电流不变。**

### 10.1 诊断思路框架

电机 "收到 MIT 命令但不转动" 的可能原因可以按链路分层排查：

```
MIT 命令帧 → CAN 收发器 → 电机 MCU → FOC/PID 控制器 → 门极驱动 → MOS 功率级 → 电机三相绕组
                                                          ↑
                                                    使能/保护逻辑
```

| 层级 | 检查项 | 方法 |
|------|--------|------|
| CAN 收发 | 帧是否被电机正确接收 | 读 ESC_ID(0x08) 确认接收 ID |
| 电机 MCU | 控制模式是否为 MIT | 读 CTRL_MODE(0x0a) 已确认=1 |
| 电机 MCU | 是否有故障锁存 | 读 FAULT(0x01), WARNING(0x02) |
| 电机 MCU | 电机是否处于运行状态 | 读 STATUS(0x04), MOTOR_ERR(0x06) |
| 电机 MCU | CAN 看门狗超时是否过短 | 读 TIMEOUT(0x09) |
| FOC 控制 | Iq/Id 电流是否为 0 | 如寄存器支持，读 Iq 参考值 |
| 功率级 | 母线电压是否正常 | 读 VBus(0x3c) 已确认=23.9V |
| 功率级 | 驱动器指示灯状态 | 目视检查 LED 颜色 |
| 功率级 | 三相线是否连接 | 万用表通断测试 |

### 10.2 新增诊断命令

本轮为固件添加了以下能力：

#### motor dm diag — 一键诊断

```text
motor dm diag left
motor dm diag right
```

一次性读取以下寄存器，输出到串口：

| RID | 名称 | 含义 | 期望值 |
|-----|------|------|--------|
| 0x01 | FAULT | 故障锁存寄存器 | **必须为 0** |
| 0x02 | WARNING | 警告寄存器 | 应为 0 |
| 0x04 | STATUS | 电机运行状态 | 使能后应为运行态 |
| 0x05 | CAN_ERR | CAN 收发错误计数 | 应为 0 |
| 0x06 | MOTOR_ERR | 电机侧错误 | 应为 0 |
| 0x09 | TIMEOUT | CAN 指令看门狗(ms) | 建议 ≥ 5ms |
| 0x0a | CTRL_MODE | 控制模式 | 1 = MIT |
| 0x15 | PMAX | MIT 位置量程(rad) | 12.5 |
| 0x16 | VMAX | MIT 速度量程(rad/s) | 10.0 |
| 0x17 | TMAX | MIT 力矩量程(Nm) | 28.0 |
| 0x3b | Imax | 最大电流(A) | 20.5 |
| 0x3c | VBus | 母线电压(V) | ~24 |

#### 扩展的寄存器读取

`motor dm reg` 命令现在支持更多寄存器，包括：

```text
motor dm reg right 0x01   # FAULT — 最关键：如果非零，驱动器已锁存故障
motor dm reg right 0x04   # STATUS — 确认电机是否真正在运行
motor dm reg right 0x05   # CAN_ERR — 从电机侧看 CAN 错误
motor dm reg right 0x06   # MOTOR_ERR — 电机本体故障（霍尔/编码器/缺相）
motor dm reg right 0x09   # TIMEOUT — 如果太小(<2ms)，1kHz 命令间隔可能导致断续
```

### 10.3 MIT 帧格式逐字节验证

当前固件发送的 MIT 帧格式为 **16/12/12/12/12 位压缩格式**（共 64 位 = 8 字节），与反馈帧的 16/12/12 编码风格一致。

以 `motor dm nudge right 0.01 3.0 0.2 500` 为例展开一次实际 CAN 帧：

**已知参数：**
- 当前位置 `fb.position_rad` ≈ 0.6754
- 目标位置 = 0.6754 + 0.01 = 0.6854 rad
- kp=3.0, kd=0.2, torque_nm=0.0

**编码计算：**

```text
p_int = (0.6854 - (-12.5)) * 65535 / 25.0 = 13.1854 * 2621.4 ≈ 34562 = 0x8702
v_int = (0 - (-10.0)) * 4095 / 20.0 = 10.0 * 204.75 = 2048 = 0x800
kp_u  = (3.0 - 0) * 4095 / 500.0 = 3.0 * 8.19 = 25 = 0x019
kd_u  = (0.2 - 0) * 4095 / 5.0   = 0.2 * 819 = 164 = 0x0A4
t_int = (0.0 - (-28.0)) * 4095 / 56.0 = 28.0 * 73.125 = 2048 = 0x800
```

**构造 CAN 帧：**

```text
data[0] = 0x87   (p >> 8)
data[1] = 0x02   (p & 0xff)
data[2] = 0x80   (v >> 4, v=0x800 → 0x80)
data[3] = 0x01   ((v & 0x0f) << 4 | kp >> 8) = (0 << 4) | 0 = 0x00? 

Wait, 重新计算:
v_int=2048=0x800, v>>4=0x80, v&0x0f=0
kp_u=25=0x19, kp_u>>8=0

data[3] = (0 << 4) | 0 = 0x00
data[4] = 0x19   (kp_u & 0xff)
data[5] = 0x0A   (kd_u >> 4, kd_u=0xA4 → 0x0A)
data[6] = 0x48   ((kd_u & 0x0f) << 4 | t >> 8) = (0x4 << 4) | (0x800 >> 8) = 0x40 | 0x08 = 0x48
data[7] = 0x00   (t & 0xff, t=0x800 → 0x00)
```

**最终 CAN 帧：**
```text
CAN ID = 0x02 (右电机)
DLC = 8
DATA = 87 02 80 00 19 0A 48 00
```

如果要验证帧格式是否正确，可以在达妙上位机中用 MIT 模式发一条类似的指令，在 CAN 总线上抓包对比字节排列。如果上位机发出的字节排列和我们算的不同，就需要修正 MIT 编码函数。

### 10.4 常见故障寄存器位定义

> 以下为达妙 DM43xx 系列通用定义，以实际读出的 FAULT 和 STATUS 寄存器为准。

**FAULT(0x01) 常见位：**

| 位 | 含义 |
|----|------|
| 0 | 过流 (OC) |
| 1 | 过压 (OV) |
| 2 | 欠压 (UV) |
| 3 | 过温 (OT) |
| 4 | 编码器故障 |
| 5 | 缺相 |
| 6 | CAN 超时 |
| 7 | 启动失败 |

**STATUS(0x04) 常见位：**

| 位 | 含义 |
|----|------|
| 0 | 使能状态 (1=使能) |
| 1 | 运行状态 (1=正在运行) |
| 2 | 抱闸/刹车状态 |
| 3 | 零力矩模式 |

### 10.5 推荐的现场排查步骤

#### 步骤一：先读故障寄存器（最关键）

```text
motor dm diag left
motor dm diag right
```

**判断：**
- 如果 FAULT(0x01) ≠ 0：驱动器已锁存故障，必须**先清除故障**才能让电机出力。通常是**重新上电**或发送故障清除命令。
- 如果 STATUS(0x04) 显示未运行：说明使能成功但 FOC 未启动，可能是 TIMEOUT 太短或模式切换未生效。
- 如果 TIMEOUT(0x09) 很小（如 1ms）：1kHz 控制循环下稍有抖动就触发看门狗停止输出。达妙电机典型 TIMEOUT 为 5-10ms。

#### 步骤二：对比左右电机

如果左电机也不能转动（同样的 MIT 命令下位置不变、电源电流不变），**基本可以排除单个电机硬件故障**。更可能是：

1. **MIT 帧格式与电机固件协议不匹配**——这是当前最高概率的假设。需要用达妙上位机抓包对比。
2. **电机需要先通过上位机进行一次 MIT 使能**——有些达妙电机在上位机里点过 MIT 控制后，CAN 命令才开始生效。
3. **使能时序问题**——某些达妙电机需要：设置 CTRL_MODE → 保存 → 重新上电 → 发送使能 → 等待 100ms → 发送 MIT 命令。

如果左电机能转动但右电机不能，则问题在右电机或右驱动器硬件。

#### 步骤三：尝试 pose-vel 模式

MIT 模式是最复杂的控制模式，涉及位置环 + 速度前馈 + 力矩前馈。如果 MIT 不工作，可以用更简单的模式隔离问题：

```text
# 用位置-速度模式，以当前位置为目标，低速移动
motor dm pos right 0.7 1.0 2000
motor dm status right
motor dm pos right 0.65 1.0 2000
motor dm status right
motor dm stop right
```

如果 pos-vel 模式电机能转动，说明**问题出在 MIT 模式层**（帧格式、参数映射或模式切换）。如果 pos-vel 模式也不转，说明**问题在使能/保护/硬件层**。

#### 步骤四：检查 TIMEOUT 寄存器

```text
motor dm reg right 0x09
```

如果 TIMEOUT 值太小（如 1ms 或 2ms），可以通过达妙上位机将其改为 10ms 或更大。`main.c` 中控制线程以 1kHz 发送 MIT 命令，所以 TIMEOUT 至少需要 > 1ms。

#### 步骤五：用上位机验证

这是绕不开的一步。用达妙调试上位机（Damiao CAN Tool）连接 CAN 总线，在同一电机上：

1. 确认 CTRL_MODE=1 (MIT)
2. 手动发送 MIT 命令让电机转动
3. 如果上位机能控制电机转动，抓包保存上位机发出的 CAN 帧，与第 10.3 节我们的编码结果逐字节对比
4. 如果上位机也不能控制电机转动，问题在驱动器/电机硬件

### 10.6 如果左右电机都不动：MIT 帧格式备选方案

如果确认 FAULT=0、STATUS=运行态、TIMEOUT 合理，但电机仍不转动，并且达妙上位机能正常控制，则问题基本锁定在 **MIT 帧格式**。

DM43xx 系列不同固件版本可能有不同的 MIT 帧定义。已知的备选格式：

**格式 B：全部 uint16，无扭矩前馈（8 字节 = 4 × uint16）**

```text
data[0:1] = position uint16 (big-endian, 0-65535 → -12.5~12.5 rad)
data[2:3] = velocity uint16 (big-endian, 0-65535 → -VMAX~VMAX rad/s)
data[4:5] = kp uint16 (big-endian, 0-65535 → 0~500)
data[6:7] = kd uint16 (big-endian, 0-65535 → 0~5)
```

扭矩通过位置偏差 × kp 隐式产生，不单独发送。这种格式在某些老版本达妙固件中常见。

**格式 C：带浮点编码的 MIT**

如果达妙上位机使用的是 IEEE 754 float 编码（直接发 4 字节 float），则需要将每个物理量以 `memcpy` 方式填入 CAN 帧，完全改变编码逻辑。

### 10.7 本次结论与下一步行动 (2026-05-03 初版)

> 注：以下为首次诊断报告。实际烧录并现场测试后，发现了更根本的问题，详见第 11 章。

**已完成的分析：**

1. CAN 通信双向正常，反馈帧持续更新，参数寄存器可读可确认。
2. 电机 CTRL_MODE=1 (MIT)，ESC_ID 和 MST_ID 与代码一致。
3. MIT 量程 (PMAX/VMAX/TMAX) 已按实测值修正到代码。
4. 电机显示使能状态 (error=1)，但**不排除驱动器内部有故障锁存或保护状态未解除**。
5. 固件已新增 `motor dm diag` 命令，可一次性读取关键诊断寄存器。

## 11. 现场实测：电机能转动，问题定位为增益参数

**测试时间：** 2026-05-03，烧录新版固件后。  
**测试条件：** 右关节电机空载，CAN 通信正常，VBus=23.88V，CTRL_MODE=MIT。

### 11.1 核心结论

**电机完全正常！** CAN 通信、MIT 控制模式、功率输出、位置反馈全部工作正常。

**初始"电机不转"的根因：** kp=3.0 配合 delta=0.01 rad 的位置误差，仅产生约 0.03 Nm 的 MIT 扭矩。减速箱（约 9:1 减速比）静摩擦力远超此值，导致电机无法从静止状态启动。

### 11.2 逐步加大控制参数后的电机响应

| 测试 | 命令参数 | 位置变化 | 结果 |
|------|---------|----------|------|
| 低增益位置 | kp=3.0, kd=0.2, delta=0.01 | 0.6758 → 0.6758 | 扭矩=0.03Nm，未克服静摩擦 |
| 低增益位置 | kp=15.0, kd=0.5, delta=0.04 | 0.6758 → 0.6758 | 扭矩=0.6Nm，接近但未突破 |
| **速度模式** | **kp=0, kd=2.0, v_des=4.0** | **0.6754 → 1.1538** | **电机转动！Δ=+0.48 rad** |
| 速度模式 | kp=0, kd=1.5, v_des=3.0 | -0.0769 → 5.2379 | 从限位移出，Δ=+5.31 rad |
| 速度模式 | kp=0, kd=1.0, v_des=1.5 | -0.0517 → 0.9901 | 稳定移动，Δ=+1.04 rad |
| 高增益位置 | kp=50, kd=2.0, delta=0.04 | 0.9813 → 0.9966 | 移动+0.015 rad，但未到目标 |
| 高增益位置 | kp=80, kd=2.0, p_des=0.70 | 0.9813 → **-0.064** | **过冲撞到机械限位！** |

### 11.3 关键发现

**A. 减速箱静摩擦是主因**

MIT 扭矩公式：`torque = kp*(p_des - p_actual) + kd*(v_des - v_actual) + t_ff`

| kp | 位置误差 | 计算扭矩 | 结果 |
|----|---------|---------|------|
| 3.0 | 0.01 rad | 0.03 Nm | < 静摩擦，**不动** |
| 15.0 | 0.04 rad | 0.60 Nm | ≈ 静摩擦边界 |
| 50.0 | 0.04 rad | 2.00 Nm | > 静摩擦，**能动** |
| 80.0 | 0.28 rad | 22.4 Nm | >> 静摩擦，**猛冲** |

速度模式（kp=0, kd=2.0, v_des=4.0）在静止时产生扭矩 = 2.0 × 4.0 = **8.0 Nm**，远大于静摩擦，因此能可靠启动。

**B. 位置控制需要足够阻尼（kd/kp 比值）**

| kp | kd | kd/kp | 表现 |
|----|----|-------|------|
| 15 | 0.5 | 0.03 | 不动（不足以破冰） |
| 20 | 2.0 | 0.10 | 大幅过冲→撞限位 |
| 25 | 3.0 | 0.12 | 过冲→撞限位 |
| 50 | 2.0 | 0.04 | 过冲严重 |
| 80 | 2.0 | 0.025 | 过冲极其严重 |

数据手册明确警告 `kd不能赋0`，典型示例使用 kd=1 N·m·s/rad。初步判断 kd/kp 比值需 ≥ 0.3 才能有效阻尼。

**C. 寄存器映射已从数据手册确认**

| RID | 实际名称 | 实测值 | 说明 |
|-----|---------|--------|------|
| 0x01 | KT_Value (扭矩系数) | 0.0 | =0 表示使用内部标定参数，属正确值 |
| 0x02 | OT_Value (过温保护) | 100.0 | 100°C 过温阈值 |
| 0x04 | ACC (加速度) | 2.0 | rad/s² |
| 0x05 | DEC (减速度) | -2.0 | rad/s² |
| 0x06 | MAX_SPD (最大速度) | 600.0 | rad/s |
| 0x09 | TIMEOUT | 0 | CAN看门狗禁用 |
| 0x0a | CTRL_MODE | 1 | MIT模式 |
| 0x0b | Damp (粘滞系数) | — | 只读 |
| 0x0c | Inertia (转动惯量) | — | 只读 |

> **修正：** 之前将 0x01-0x06 假设为 FAULT/WARNING/STATUS 等位域寄存器是错误的。DM4340 的故障信息通过反馈帧 ERR 字段报告（error=1=使能）。

### 11.4 推荐的调试参数

```text
# 第一步：确认通信在线
motor dm status right

# 第二步：速度模式"破冰"（克服静摩擦）
motor dm mit right 0 1.5 0 1.0 0 1000

# 第三步：高阻尼位置控制（kd/kp ≈ 0.3-0.4）
motor dm mit right 0.675 0 15 5.0 0 5000

# 第四步：小幅 wiggle 验证位置跟随
motor dm wiggle right 0.02 2000 18 6.0 5000

# 停止
motor dm stop right
```

**安全注意：**
1. 初次测试用 kp ≤ 20, kd ≥ 5，避免过冲撞限位
2. 速度模式优先——不累积位置误差，比位置控制安全
3. 随时用 `motor dm stop right` 紧急停止
4. 注意机械限位：RIGHT_LEG_MIN_RAD=-0.70, RIGHT_LEG_MAX_RAD=-0.07

### 11.5 下一步优化建议

1. **增益调优：** 在达妙上位机中精确调 kp/kd。建议范围：kp=15-25, kd=5-8
2. **轨迹规划：** 在固件中实现 S 曲线/S 型加减速，避免位置阶跃冲击
3. **静摩擦补偿：** 位置控制中加入力矩前馈 (t_ff)，估计并抵消减速箱静摩擦
4. **软限位保护：** 在 `motor_debug_set_dm_mit_raw` 中检查命令目标是否超出 APP_LEG 范围
