# 电机接线与单电机调试 README

适用工程：

```bash
/home/h/code_leg/zephyr_ascento_f407_wheel_leg
```

本文主要讲三件事：

1. 每一根线怎么接。
2. 烧录成功后怎么进入串口 shell。
3. 调试电机时，速度、电流、位置、时间这些参数怎么一点点调。

调试原则：先只让一个电机用 `100 mA` 动 `3 s`，确认反馈、方向和停止都正常，再进入下一步。

## 0. 烧录成功后先做什么

你刚刚看到下面信息，说明固件已经烧进 C 板：

```text
Flash written and verified! jolly good!
Go to Thumb mode
```

烧录成功后，电机调试不要急着上电。先按这个顺序：

1. 断开电机动力电，只保留 C 板供电。
2. ST-LINK 可以继续插着，也可以拔掉；它只负责烧录/调试，不负责电机命令。
3. 插上 CH340/USB-TTL 串口线。
4. 打开串口 shell。
5. 确认能看到 `uart:~$`。
6. 再一个总线、一个电机地上电调试。

打开串口：

```bash
cd /home/h/code_leg/zephyr_ascento_f407_wheel_leg
./scripts/serial.sh
```

看到：

```text
uart:~$
```

才说明你进入了 Zephyr 固件里的命令行。后面所有 `motor ...` 和 `robot ...` 命令都在这里输入，不是在 Linux 终端输入。

建议先输入：

```text
help
robot status
motor debug status
motor can status all
```

### 0.1 非常重要：当前固件启动后的默认行为

当前固件启动后会做两件事：

1. 自动发送达妙关节 `enable`。
2. 控制循环会持续给达妙关节发送默认腿部位置。

也就是说，如果 CAN1 上的达妙关节已经上电、ID 正确、机械连杆也装好了，关节可能会进入保持/移动状态。轮电机 VESC/M3508 不会在默认状态下主动输出电流，除非你执行 `robot enable 1` 或 `motor wheel ...` 调试命令。

所以第一次调试关节时建议：

1. 关节不要带重负载。
2. 连杆附近不要放手。
3. 先只接一个达妙电机。
4. 如果要让达妙完全失能，输入：

```text
motor dm disable left
motor dm disable right
```

注意：如果达妙没有上电，`disable` 命令可能没有实际效果；它不是 Linux 里的停止程序命令，而是通过 CAN 发给电机的命令。

### 0.2 电机调试的“急停命令”

只要串口还可以输入，优先输入：

```text
robot enable 0
robot stop
motor debug stop
motor wheel stop
motor dm stop left
motor dm stop right
```

如果串口卡住、电机动作异常、关节接近机械限位，直接断电，不要等命令。

## 1. 整体硬件连接

默认双 CAN：

| 总线 | 接什么 | 默认 ID | 波特率 |
| --- | --- | --- | --- |
| CAN1 | 达妙 DM4340/DM4310 关节电机 | 左 1，右 2 | 1 Mbps |
| CAN2 | VESC + DJI M3508 轮电机 | 左 101，右 100 | 1 Mbps |

控制板：

| 功能 | C 板接口/引脚 |
| --- | --- |
| ST-LINK SWDIO | C 板 SWDIO |
| ST-LINK SWCLK | C 板 SWCLK |
| ST-LINK GND | C 板 GND |
| ST-LINK 3V3 sense | C 板 3V3，只做检测，不给整车供电 |
| 串口 TX | PA9 / USART1_TX |
| 串口 RX | PB7 / USART1_RX |
| CAN1 | PD0/PD1 对应 C 板 CAN1 接口 |
| CAN2 | PB5/PB6 对应 C 板 CAN2 接口 |

## 2. 电源接线

必须共地：

```text
电池负极 / 电源 GND
  ├── C 板 GND
  ├── 达妙电机 GND
  └── VESC 电调 GND
```

推荐上电顺序：

1. 先只给 C 板供电，确认能烧录和看串口。
2. 再给达妙关节上电，调 CAN1。
3. 再给 VESC/M3508 上电，调 CAN2。
4. 最后两条 CAN 都接上，调整机。

注意：

- ST-LINK 的 3V3 不要拿来给整车供电。
- USB-TTL 只能接 3.3 V TTL 串口，不能接 RS232 电平。
- 电机大电源和控制信号线都要固定好，避免调试时松动。

## 3. CAN1 达妙关节接线

CAN1 只接达妙关节：

| C 板 CAN1 | 左达妙 | 右达妙 |
| --- | --- | --- |
| CAN1_H | CAN_H | CAN_H |
| CAN1_L | CAN_L | CAN_L |
| GND | GND | GND |

达妙 ID：

| 电机 | ID |
| --- | --- |
| 左关节 | 1 |
| 右关节 | 2 |

达妙 Master ID 默认：

```c
#define APP_DM_MASTER_ID 0x00
```

如果你的达妙上位机里 Master ID 不是 `0x00`，要改 `src/app_config.h` 后重新编译烧录。

CAN1 总线要求：

- CAN_H 和 CAN_L 不要接反。
- 总线两端各 120 欧终端电阻。
- 两个关节都必须和 C 板共地。
- 如果只接一个达妙电机测试，也要保证总线有合适终端。

## 4. CAN2 VESC/M3508 接线

CAN2 只接 VESC：

| C 板 CAN2 | 左 VESC | 右 VESC |
| --- | --- | --- |
| CAN2_H | CAN_H | CAN_H |
| CAN2_L | CAN_L | CAN_L |
| GND | GND | GND |

VESC ID：

| 电机 | VESC Controller ID | 主要反馈扩展帧 |
| --- | --- | --- |
| 左轮 M3508 | 101 | `0x965`，也就是 `(CAN_PACKET_STATUS 9 << 8) | 101` |
| 右轮 M3508 | 100 | `0x964`，也就是 `(CAN_PACKET_STATUS 9 << 8) | 100` |

CAN2 总线要求和 CAN1 一样：

- CAN_H/CAN_L 不接反。
- 两端 120 欧。
- C 板和 VESC 共地。
- 轮子必须架空后再输出电流命令。

VESC Tool 里至少确认这些设置：

| 项目 | 推荐值 |
| --- | --- |
| 左 VESC Controller ID | `101` |
| 右 VESC Controller ID | `100` |
| CAN Baud | `1 Mbps` |
| CAN Mode | `VESC` |
| CAN Status | 开启 Status 1，设置 `200 Hz` |

如果 `motor wheel status left/right` 一直 `no feedback`，优先检查 VESC Tool 里的 CAN ID、CAN 波特率和 CAN Status 发送是否开启。

本工程按 `docs/2.28修订版说明书.pdf` 的 VESC CAN 说明实现：

| 手册内容 | 本工程对应 |
| --- | --- |
| VESC CAN 使用 29 bit 扩展帧 | 代码发送/接收都使用 `CAN_FRAME_IDE` |
| CAN 波特率可改为 RoboMaster 常用 `1 Mbps` | `APP_CAN_BITRATE 1000000U` |
| Status 1 反馈帧范围 `0x900` 到 `0x9ff` | 代码过滤 `(9 << 8) | id` |
| 右轮 ID 100 的反馈帧为 `0x964` | `motor wheel status right` 接收 `0x964` |
| 左轮 ID 101 的反馈帧为 `0x965` | `motor wheel status left` 接收 `0x965` |
| 反馈数据 0-3 字节为 ERPM | 状态打印 `erpm=...` |
| 反馈数据 4-5 字节为实际电流 x10 | 状态打印换算后的 `current=... mA` |
| 反馈数据 6-7 字节为占空比 x1000 | 状态打印 `duty=...` |
| `CAN_PACKET_SET_RPM = 3`，ID 100 命令帧 `0x364` | `motor wheel rpm right ...` 或 `motor can rawx can2 0x364 ...` |

注意：VESC Tool 里的 `RPM` 通常对应电机 ERPM。M3508 机械转速大致为：

```text
电机本体机械 rpm = ERPM / 7
减速箱输出轴 rpm = ERPM / 7 / 19.203
```

## 5. 串口和烧录接线

ST-LINK：

| ST-LINK | C 板 |
| --- | --- |
| SWDIO | SWDIO |
| SWCLK | SWCLK |
| GND | GND |
| 3V3 sense | 3V3 |
| NRST，可选 | NRST |

USB-TTL：

| USB-TTL | C 板 |
| --- | --- |
| RX | PA9 / USART1_TX |
| TX | PB7 / USART1_RX |
| GND | GND |

打开串口：

```bash
cd /home/h/code_leg/zephyr_ascento_f407_wheel_leg
./scripts/serial.sh
```

如果自动找不到串口，先看设备名：

```bash
./scripts/check_devices.sh
```

串口接线再次确认：

| USB-TTL | C 板 |
| --- | --- |
| TXD | PB7 / USART1_RX |
| RXD | PA9 / USART1_TX |
| GND | GND |

也就是 TX 接 RX，RX 接 TX，GND 必须共地。CH340/USB-TTL 要用 `3.3 V TTL` 电平，不要用 RS232 电平。

如果串口打开后没有 `uart:~$`，按回车试一下；如果仍然没有，检查：

| 现象 | 常见原因 | 处理 |
| --- | --- | --- |
| 找不到 `/dev/ttyUSB0` | CH340 没识别或设备号变了 | 运行 `./scripts/check_devices.sh` |
| 有串口但无输出 | TX/RX 接反、波特率不对、C 板没运行 | 交换 TX/RX，确认 115200 |
| 乱码 | 波特率不对或地线没接好 | 使用 115200，检查 GND |
| shell 不响应 | 接到了别的串口或固件没启动 | 重新烧录，检查 PA9/PB7 |

## 6. 上电后第一步：看 CAN 状态

在串口 shell 输入：

```text
motor can status all
```

正常结果：

```text
joint/CAN1 state=error-active tx_err=0 rx_err=0
wheel/CAN2 state=error-active tx_err=0 rx_err=0
```

这里检查的是 C 板上的 CAN 控制器状态，不等于电机已经在线。判断电机是否在线，还要看：

```text
motor dm status all
motor wheel status all
```

如果状态显示 `no feedback`，说明当前固件还没有收到对应电机的反馈帧。先检查电机是否上电、ID 是否正确、CAN_H/CAN_L 是否接反、终端电阻是否正确。

如果不是 `error-active`：

| 现象 | 常见原因 | 处理 |
| --- | --- | --- |
| `bus-off` | CAN_H/L 接反、没终端、波特率不一致 | 断电检查，复位 C 板 |
| `error-warning` | 线太长、终端不对、接触不良 | 检查接线和终端电阻 |
| CAN1 异常 | 达妙线、电源或 ID 设置问题 | 先不要测关节 |
| CAN2 异常 | VESC 线、电源或 ID 设置问题 | 先不要测轮子 |

高级原始 CAN 发送命令：

```text
motor can raw <joint|wheel|can1|can2> <std_id> [byte0..byte7]
motor can rawx <joint|wheel|can1|can2> <ext_id> [byte0..byte7]
```

例子：

```text
motor can raw can1 0x165 0 0 0 0
motor can rawx can2 0x364 0 0 3 232 0 0 0 0
```

`raw` 发标准帧，`rawx` 发扩展帧。VESC 使用扩展帧，所以手册里的 ID 100、目标 1000 ERPM 示例就是 `0x364` 和数据 `00 00 03 E8 00 00 00 00`。这个命令会直接往 CAN 总线发原始帧，新手调电机时优先用 `motor wheel current/rpm`，只有在确认具体协议帧含义时才使用原始帧。

## 7. 达妙关节调试命令

达妙相关命令：

```text
motor dm status [left|right|id|all]
motor dm enable <left|right|id>
motor dm disable <left|right|id>
motor dm zero <left|right|id>
motor dm pos <left|right|id> <pos_rad> [vel_rad_s] [ms]
motor dm vel <left|right|id> <vel_rad_s> [ms]
motor dm mit <left|right|id> <pos_rad> <vel_rad_s> <kp> <kd> <torque_nm> [ms]
motor dm stop <left|right|id>
```

参数含义：

| 参数 | 含义 | 新手建议 |
| --- | --- | --- |
| `pos_rad` | 目标角度，单位 rad | 从 `0.10` 或 `-0.10` 开始 |
| `vel_rad_s` | 速度限制，单位 rad/s | 从 `0.2` 到 `1.0` 开始 |
| `ms` | 命令持续时间，单位 ms | 从 `300` 到 `800` 开始 |
| `kp` | 位置刚度 | MIT 模式先从 `5` 到 `20` |
| `kd` | 阻尼 | MIT 模式先从 `0.1` 到 `0.5` |
| `torque_nm` | 前馈力矩 | 新手先用 `0.0` |

代码里的调试限幅：

```c
#define APP_DM_DEBUG_VEL_LIMIT_RAD_S 4.0f
#define APP_DM_DEBUG_KP_LIMIT 80.0f
#define APP_DM_DEBUG_KD_LIMIT 2.0f
#define APP_DM_DEBUG_TORQUE_LIMIT_NM 2.0f
```

也就是说，串口里输入更大的速度、kp、kd、力矩，也会被代码限制住。

持续时间规则：

```c
#define APP_MOTOR_DEBUG_DEFAULT_TIMEOUT_MS 1000
#define APP_MOTOR_DEBUG_MAX_TIMEOUT_MS 5000
```

如果命令不写 `[ms]`，默认持续 `1000 ms`。如果写得超过 `5000 ms`，代码也会限制到 `5000 ms`。轮子方向调试建议明确写 `3000 ms`，也就是 `3 s`。

只要执行 `motor dm ...` 调试命令，固件会自动退出平衡控制：

```text
robot enable 0
robot stop
```

但是调试命令时间结束后，控制循环仍可能继续给关节发送默认腿部位置。如果你希望关节彻底不出力，用：

```text
motor dm disable left
motor dm disable right
```

`motor dm zero` 是保存电机零点命令，新手不要随便用。只有在你已经确认机械零位、达妙电机方向、连杆安装都正确时，才可以保存零点；错误保存零点会让后续位置控制全部偏掉。

### 7.1 达妙左关节最小测试

先确认左达妙电机 ID 是 `1`，只给左达妙上电，右达妙先不接或不供电。

```text
robot enable 0
motor dm enable left
motor dm status left
motor dm pos left 0.10 0.5 500
motor dm status left
motor dm stop left
motor debug status
```

怎么调速度：

| 想要的效果 | 改哪个参数 | 示例 |
| --- | --- | --- |
| 转慢一点 | 降低 `vel_rad_s` | `motor dm pos left 0.10 0.2 500` |
| 转快一点 | 提高 `vel_rad_s` | `motor dm pos left 0.10 1.0 500` |
| 动作时间更短 | 降低 `ms` | `motor dm pos left 0.10 0.5 300` |
| 角度更小 | 降低 `pos_rad` 绝对值 | `motor dm pos left 0.07 0.5 500` |

左关节默认位置范围：

```c
#define APP_LEFT_LEG_MIN_RAD 0.07f
#define APP_LEFT_LEG_MAX_RAD 0.70f
```

所以 `left` 位置命令会被限制在 `0.07` 到 `0.70` rad。

通过标准：

1. `enable ret=0` 或没有明显 CAN 错误。
2. `motor dm status left` 不是 `no feedback`。
3. 电机只小幅移动，没有冲向机械限位。
4. `motor dm stop left` 后不再继续执行调试速度命令。

如果 `pos left 0.10` 看起来方向不对，不要立刻加大角度，先记录实际方向。

### 7.2 达妙右关节最小测试

先确认右达妙电机 ID 是 `2`。

```text
robot enable 0
motor dm enable right
motor dm status right
motor dm pos right -0.10 0.5 500
motor dm status right
motor dm stop right
motor debug status
```

右关节默认位置范围：

```c
#define APP_RIGHT_LEG_MIN_RAD -0.70f
#define APP_RIGHT_LEG_MAX_RAD -0.07f
```

所以右关节测试通常用负角度，比如 `-0.10`、`-0.20`。

### 7.3 达妙速度模式

速度模式直接给目标速度，不给目标位置：

```text
motor dm vel left 0.2 500
motor dm stop left
```

参数调法：

| 命令 | 效果 |
| --- | --- |
| `motor dm vel left 0.1 300` | 很慢转 0.3 秒 |
| `motor dm vel left 0.5 500` | 中等速度转 0.5 秒 |
| `motor dm vel left -0.5 500` | 反向转 0.5 秒 |

速度模式更容易撞机械限位，新手只短时间测试。

### 7.4 达妙 MIT 模式

MIT 模式参数最多，新手最后再用：

```text
motor dm mit left 0.10 0.0 10.0 0.3 0.0 500
```

含义：

| 参数 | 当前例子 | 意义 |
| --- | --- | --- |
| `pos_rad` | `0.10` | 目标角度 |
| `vel_rad_s` | `0.0` | 目标速度 |
| `kp` | `10.0` | 位置刚度 |
| `kd` | `0.3` | 阻尼 |
| `torque_nm` | `0.0` | 前馈力矩 |
| `ms` | `500` | 持续 0.5 秒 |

调参顺序：

1. `torque_nm` 先保持 `0.0`。
2. `kp` 从 `5`、`10`、`20` 慢慢加。
3. 抖动就加一点 `kd`，比如 `0.2` 到 `0.5`。
4. 不要一开始用大 kp 或大 torque。

### 7.5 达妙零点命令

命令：

```text
motor dm zero left
motor dm zero right
```

这个命令会向达妙电机发送保存零点命令。新手调试电机能不能转时，不需要执行它。

只有满足下面条件才考虑保存零点：

1. 机械腿已经装好。
2. 你已经定义好“站立零位”是哪一个姿态。
3. 确认左右电机 ID 没接反。
4. 确认达妙正方向和代码里的左右范围一致。
5. 保存前已经断开平衡控制，且机器人被支撑住。

保存错零点后，后续 `motor dm pos` 和 `robot height` 都会偏，所以不要把 `zero` 当成普通停止命令。

## 8. VESC/M3508 轮电机调试命令

M3508 相关命令：

```text
motor wheel status [left|right|100|101|all]
motor wheel current <left|right|100|101> <current_mA> [ms]
motor wheel rpm <left|right|100|101> <target_erpm> [ms]
motor wheel pair <left_current_mA> <right_current_mA> [ms]
motor wheel stop
```

重要区别：

- `motor wheel current` 是 VESC 电流/力矩命令，单位按毫安理解，适合平衡控制和试力矩。
- `motor wheel rpm` 是按 VESC 手册 `CAN_PACKET_SET_RPM = 3` 发送的转速命令，参数是目标 ERPM。
- 轮子的实际反馈要通过 `motor wheel status` 看，重点看 `erpm`、`motor_rpm`、`speed`、`cmd`、`motor_current`、`input`、`vin`、`temp`、`torque_k`、`torque_est`、`duty` 和 `age`。
- 想让轮子转慢一点，电流模式就减小 `current_mA` 或缩短 `ms`；转速模式就减小 `target_erpm`。

调试限幅：

```c
#define APP_M3508_DEBUG_CURRENT_LIMIT APP_WHEEL_CURRENT_SAFE
#define APP_WHEEL_CURRENT_SAFE 2500
#define APP_VESC_DEBUG_ERPM_LIMIT 8000
```

也就是说，电流调试命令最大会被限制在 `-2500 mA` 到 `2500 mA`，转速调试命令最大会被限制在 `-8000 ERPM` 到 `8000 ERPM`。

`current_mA` 不是目标转速。它是发给 VESC 的电流/力矩命令。空载时很小的电流就可能让轮子转起来；落地、带负载或轮子卡住时，同样电流产生的速度会完全不同。

轮电机调试前必须确认：

1. 轮子架空。
2. VESC/M3508 有独立动力电。
3. VESC 和 C 板共地。
4. VESC ID 正确：左轮 `101`，右轮 `100`。
5. VESC CAN Baud 是 `1 Mbps`。
6. VESC Status 1 回传频率设置为 `200 Hz`。
7. 没有执行 `robot enable 1`。

### 8.1 左轮最小测试

轮子必须架空。

```text
robot enable 0
motor wheel stop
motor wheel status left
motor wheel current left 100 3000
motor wheel status left
motor wheel stop
```

如果 `100` 不动，再试：

```text
motor wheel current left 300 500
motor wheel stop
```

参数含义：

| 参数 | 含义 |
| --- | --- |
| `left` | 左轮，默认 VESC ID 101 |
| `100` 或 `300` | 电流命令，单位 mA，越大力矩越大 |
| `3000` 或 `500` | 持续时间，单位 ms |

### 8.2 右轮最小测试

```text
robot enable 0
motor wheel stop
motor wheel status right
motor wheel current right 100 3000
motor wheel status right
motor wheel stop
```

### 8.3 调 M3508 力矩和速度

先用电流模式试力矩，最安全：

| 目标 | 命令 |
| --- | --- |
| 最轻微试转 | `motor wheel current left 100 3000` |
| 稍微明显一点 | `motor wheel current left 300 500` |
| 更有力一点 | `motor wheel current left 600 500` |
| 反向试转 | `motor wheel current left -300 500` |
| 右轮同样测试 | 把 `left` 改成 `right` |

确认反馈和停止都正常后，再用 VESC 手册里的 RPM 命令试速度：

| 目标 | 命令 |
| --- | --- |
| 低速正转 | `motor wheel rpm left 500 300` |
| 稍高一点 | `motor wheel rpm left 1000 500` |
| 反向 | `motor wheel rpm left -500 300` |

ERPM 和实际输出轴转速差很多。按 M3508 极对数 7、减速比约 19.203 来估算：

| ERPM | 输出轴转速约 |
| --- | --- |
| `500` | `500 / 7 / 19.203 = 3.7 rpm` |
| `1000` | `7.4 rpm` |
| `8000` | `59.5 rpm` |

每次命令后看反馈：

```text
motor wheel status left
```

状态里重点看：

| 字段 | 含义 |
| --- | --- |
| `erpm=xxx` | VESC 状态帧中的 ERPM |
| `motor_rpm=xxx` | 代码按 `ERPM / 7` 换算出的电机本体机械转速 |
| `speed=xxx rad/s` | 电机本体角速度，后续控制里会再除以减速比估算轮速 |
| `cmd=xxx mA` | F407 最近一次发给 VESC 的电流命令 |
| `motor_current=xxx mA` | VESC Status 1 回传的实测电机电流 |
| `input=xxx mA` | VESC Status 4 回传的输入电流，需要在 VESC Tool 开启 |
| `vin=xxx V` | VESC Status 5 回传的输入电压，需要在 VESC Tool 开启 |
| `temp=a/bC` | FET/电机温度，需要 Status 4 |
| `torque_k=xxx` | 当前 VESC ID 使用的 `Nm/mA` 力矩系数 |
| `torque_est=xxx Nm` | `motor_current * torque_k` 估算的轮端力矩 |
| `duty=xxx` | VESC 占空比 |

当前固件默认不再使用一个固定占位力矩系数，而是按 M3508 理论力矩常数和减速比计算：

```text
torque_k = Kt * reduction_ratio * gearbox_efficiency * 0.001
```

左右轮可以分别修正：

```c
#define APP_ASCENTO_LEFT_CURRENT_MA_TO_WHEEL_TORQUE_NM ...
#define APP_ASCENTO_RIGHT_CURRENT_MA_TO_WHEEL_TORQUE_NM ...
```
| `age=xxms` | 反馈多久前收到，越小越好 |

VESC Status 1 设置为 `200 Hz` 时，正常 `age` 通常应该小于 `20 ms`；代码里按 `50 ms` 作为平衡控制的反馈超时保护。如果 `age` 一直变大，说明之前收到过反馈，但现在反馈断了；如果直接显示 `no feedback`，说明开机后还没收到过该 ID 的反馈。

### 8.4 同时测试左右轮

左右同向：

```text
motor wheel pair 100 100 3000
motor wheel stop
```

左右反向：

```text
motor wheel pair 100 -100 3000
motor wheel stop
```

反向同向都要记录：

| 命令 | 左轮实际方向 | 右轮实际方向 | 是否符合机械安装 |
| --- | --- | --- | --- |
| `100 100` | 已测：正转 | 已测：反转 | 左右相反 |
| `-100 -100` |  |  |  |
| `100 -100` |  |  |  |

当前实测结果已经写入代码：

```c
#define APP_WHEEL_LEFT_FORWARD_CURRENT_SIGN 1
#define APP_WHEEL_RIGHT_FORWARD_CURRENT_SIGN -1
```

含义是控制器内部使用“前进为正”的逻辑电流，真正发给 VESC 时右轮会自动取反。`motor wheel ...` 调试命令仍是 VESC 原始电流方向。

## 9. 调试时怎么逐步加参数

建议顺序：

1. 只看 CAN 状态。
2. 只看电机反馈，不发运动命令。
3. 发最小命令，持续 `300 ms`。
4. 确认自动停止。
5. 再把持续时间加到 `500 ms`。
6. 再把速度或电流加一点。
7. 每次只改一个参数。
8. 每次测试都记录方向。

达妙推荐递增：

```text
motor dm pos left 0.10 0.2 300
motor dm pos left 0.10 0.5 500
motor dm pos left 0.15 0.5 500
motor dm pos left 0.20 1.0 800
```

M3508 推荐递增：

```text
motor wheel current left 100 3000
motor wheel current left 300 500
motor wheel current left 600 500
motor wheel current left -300 500
motor wheel rpm left 500 300
motor wheel rpm left 1000 500
```

不要这样做：

```text
motor wheel current left 2500 5000
motor dm vel left 4.0 5000
motor dm mit left 0.5 0 80 2 2 5000
```

这些命令虽然在代码限幅内，但对初调太激进。

建议记录表：

| 项目 | 命令 | 实际现象 | 是否通过 |
| --- | --- | --- | --- |
| 左达妙小位置 | `motor dm pos left 0.10 0.5 500` |  |  |
| 右达妙小位置 | `motor dm pos right -0.10 0.5 500` |  |  |
| 左轮小电流 | `motor wheel current left 100 3000` | 正转 | 通过 |
| 右轮小电流 | `motor wheel current right 100 3000` | 反转 | 通过 |

## 10. 方向反了怎么处理

先不要马上改代码，先记录方向。

达妙方向反了：

1. 确认电机 ID 没左右接反。
2. 确认连杆安装方向。
3. 再考虑改 `src/app_config.h` 的关节零位和角度范围。
4. 腿高方向反了，再考虑 `APP_LEG_RAD_PER_HEIGHT_UNIT` 的符号。

VESC/M3508 方向反了：

1. 确认左右轮 ID 没接反。
2. 记录 `current_mA` 正负号对应的实际方向。
3. 后续平衡方向不对时，优先检查 `src/app_config.h` 的 `APP_WHEEL_*_FORWARD_CURRENT_SIGN`。

## 11. 常见故障

| 现象 | 可能原因 | 处理 |
| --- | --- | --- |
| `motor can status all` 正常但电机无反馈 | CAN 控制器正常，不代表电机在线 | 继续查 `motor dm status all` / `motor wheel status all` |
| `motor dm status` 无反馈 | 达妙 ID/Master ID/CAN1 接线/供电问题 | 查 ID，查 CAN1 H/L/GND，确认达妙上电 |
| `motor wheel status` 无反馈 | VESC ID/CAN2 接线/供电/回传设置问题 | 查 ID，查 CAN2 H/L/GND，确认 VESC 上电，Status 1 为 200 Hz |
| `age` 一直变大 | 曾经收到反馈，现在断了 | 查电源、CAN 插头、端子压接 |
| 命令打印 `ret=0` 但电机不动 | 只代表 CAN 发送成功，不代表电机执行 | 看反馈、看电机使能、电源和 ID |
| 命令打印负数返回值 | CAN 发送失败或参数错误 | 先停，查 CAN 状态和命令格式 |
| 一发命令 CAN bus-off | H/L 接反或终端电阻问题 | 断电检查 CAN |
| 达妙抖动 | kp 太大、kd 太小、机械卡滞 | 降低 kp，增大 kd，检查机械 |
| 达妙一上电就想动 | 当前固件启动会 enable 达妙并发送默认腿位 | 先支撑机器人，必要时执行 `motor dm disable left/right` |
| 达妙向机械限位冲 | 角度正负、零位或左右 ID 错 | 断电，检查 ID、零位、连杆方向 |
| M3508 不动但有反馈 | 电流太小、轮子卡住、VESC 未正确配置 | 从 100 mA 加到 300/600 mA，检查 VESC Tool 和机械 |
| M3508 转一下不停 | 没执行 stop 或调试持续时间太长 | `motor wheel stop`、`motor debug stop` |
| M3508 一启用平衡就狂转 | IMU 符号、轮子方向或左右轮符号不对 | 不要落地，先只做架空测试 |
| 电机发热快 | 长时间堵转或参数太大 | 停止调试，降低参数 |

## 12. 每次调试结束必须清零

串口输入：

```text
robot enable 0
robot stop
motor debug stop
motor wheel stop
motor dm stop left
motor dm stop right
```

如果要确保达妙关节不再出力，再输入：

```text
motor dm disable left
motor dm disable right
```

然后再断电。断电顺序建议先断电机动力电，再断 C 板控制电。

## 13. 修改限幅后重新编译烧录

限幅在：

```text
src/app_config.h
```

常改参数：

```c
APP_M3508_DEBUG_CURRENT_LIMIT
APP_DM_DEBUG_VEL_LIMIT_RAD_S
APP_DM_DEBUG_KP_LIMIT
APP_DM_DEBUG_KD_LIMIT
APP_DM_DEBUG_TORQUE_LIMIT_NM
APP_LEFT_LEG_MIN_RAD
APP_LEFT_LEG_MAX_RAD
APP_RIGHT_LEG_MIN_RAD
APP_RIGHT_LEG_MAX_RAD
```

改完执行：

```bash
cd /home/h/code_leg/zephyr_ascento_f407_wheel_leg
./scripts/build.sh
./scripts/stlink_flash.sh
```

如果你想继续用 OpenOCD 烧录，参考：

```text
docs/FLASH_TROUBLESHOOTING_README_ZH.md
```

## 14. 最小完整调试流程

按这个顺序从头走。不要跳步，不要一开始就两条 CAN、四个电机全上电。

### 14.1 阶段 A：只验证 C 板和串口

电机动力电断开，只保留 C 板供电：

```text
help
robot status
motor debug status
motor can status all
```

通过标准：

1. 能看到 `uart:~$`。
2. `robot status` 有输出。
3. `motor can status all` 能打印 CAN1/CAN2 状态。

### 14.2 阶段 B：只测 CAN1 达妙

只给一个达妙上电，先测左关节：

```text
robot enable 0

motor dm enable left
motor dm status left
motor dm pos left 0.10 0.5 500
motor dm status left
motor dm stop left
```

左关节通过后，再只测右关节：

```text
motor dm enable right
motor dm status right
motor dm pos right -0.10 0.5 500
motor dm status right
motor dm stop right
```

通过标准：

1. `motor dm status` 不是 `no feedback`。
2. 电机只短暂小幅动作。
3. 没有撞机械限位。
4. 方向已经记录。

### 14.3 阶段 C：只测 CAN2 M3508

轮子架空，只给 VESC/M3508 上电：

```text
robot enable 0
motor wheel stop
motor wheel status all

motor wheel current left 100 3000
motor wheel status left
motor wheel stop

motor wheel current right 100 3000
motor wheel status right
motor wheel stop
```

如果 `100` 不动，再把单次命令改成 `300 500`，不要直接上大电流。

通过标准：

1. `motor wheel status` 不是 `no feedback`。
2. 轮子短暂转动并自动停止。
3. 左右轮方向已经记录。
4. `age` 通常小于 `20 ms`，`current` 和 `duty` 没有异常飙高。

### 14.4 阶段 D：收尾清零

```text
robot enable 0
robot stop
motor debug stop
motor wheel stop
motor dm stop left
motor dm stop right
motor dm disable left
motor dm disable right
```

这套流程通过后，才能继续调 `robot height 38`、`robot height 60` 和架空平衡。没有完成前，不要执行：

```text
robot enable 1
robot jump
robot motion forward
```

## 15. VESC CAN 协议核对

根据 VESC 手册（`docs/2.28修订版说明书.pdf`）的 CAN 通信说明，当前代码已对齐的内容：

| 手册/需求 | 当前工程 |
| --- | --- |
| VESC CAN 使用扩展帧 29 bit ID | `src/dji_m3508.c` 中发送和接收都使用 `CAN_FRAME_IDE` |
| CAN 波特率使用 `1 Mbps` | `src/app_config.h` 中 `APP_CAN_BITRATE 1000000U` |
| 左轮 VESC ID 为 `101` | `APP_WHEEL_LEFT_ID 101` |
| 右轮 VESC ID 为 `100` | `APP_WHEEL_RIGHT_ID 100` |
| Status 1 反馈帧为 `CAN_PACKET_STATUS = 9` | 代码过滤 `(9 << 8) | id` |
| 右轮 ID100 状态帧为 `0x964` | `motor wheel status right` 接收该帧 |
| 左轮 ID101 状态帧为 `0x965` | `motor wheel status left` 接收该帧 |
| 反馈字节 0-3 为 ERPM | 状态打印 `erpm=...` |
| 反馈字节 4-5 为实际电流 x10，单位 0.1A | 状态打印换算后的 `current=... mA` |
| 反馈字节 6-7 为占空比 x1000 | 状态打印 `duty=...` |
| RPM 命令为 `CAN_PACKET_SET_RPM = 3` | 已提供 `motor wheel rpm ...` |
| ID100 的 RPM 命令扩展帧为 `0x364` | 可用 `motor wheel rpm right ...` 或 `motor can rawx can2 0x364 ...` |

注意：`200 Hz` 回传频率必须在 VESC Tool 里设置并保存，C 板代码不能凭空让 VESC 改频率。当前代码用 `50 ms` 作为 VESC 反馈超时保护；如果 Status 1 是 200 Hz，正常 `age` 通常应小于 `20 ms`。

按手册手动发 VESC 原始帧示例：

```text
# 右轮 ID100，目标 1000 ERPM
motor can rawx can2 0x364 0 0 3 232 0 0 0 0

# 左轮 ID101，目标 1000 ERPM
motor can rawx can2 0x365 0 0 3 232 0 0 0 0
```

解释：`0x364 = (CAN_PACKET_SET_RPM 3 << 8) | 0x64`，`1000 = 0x000003E8 = 0 0 3 232`。

## 16. VESC 电流反馈与力矩系数

固件通过 VESC CAN Status 帧读取实时数据。当前读取的主要状态帧：

| VESC 状态帧 | CAN packet | 固件读取内容 |
| --- | --- | --- |
| Status 1 | `CAN_PACKET_STATUS = 9` | ERPM、电机电流、duty |
| Status 4 | `CAN_PACKET_STATUS_4 = 16` | 输入电流、FET 温度、电机温度 |
| Status 5 | `CAN_PACKET_STATUS_5 = 27` | 输入电压、tachometer |

状态字段含义：

| 字段 | 含义 |
| --- | --- |
| `cmd` | F407 最近一次通过 CAN 发给 VESC 的电流命令 |
| `motor_current` | VESC Status 1 回传的实测电机电流，用于力矩估算 |
| `input` | VESC Status 4 回传的输入电流（电池侧），需要在 VESC Tool 开启 Status 4 |
| `vin` | VESC Status 5 回传的输入电压，需要在 VESC Tool 开启 Status 5 |
| `temp` | VESC Status 4 回传的 FET/电机温度 |
| `tach` | VESC Status 5 回传的 tachometer |
| `torque_k` | 当前固件用于该 VESC ID 的 `Nm/mA` 系数 |
| `torque_est` | `motor_current * torque_k` 得到的轮端力矩估计 |
| `duty` | VESC 占空比 |
| `s4_age/s5_age` | Status 4/5 距离上次更新的时间，`-1ms` 表示还没有收到 |

力矩系数计算：

```text
torque_k = Kt * reduction_ratio * gearbox_efficiency * 0.001
```

当前默认参数：

```c
#define APP_M3508_MOTOR_KT_NM_PER_A 0.180f
#define APP_M3508_REDUCTION_RATIO 19.203208f
#define APP_M3508_GEARBOX_EFFICIENCY 1.000f
```

即 `torque_k = 0.180 * 19.203208 * 1.000 * 0.001 = 0.003456577 Nm/mA`。这是理论值，适合先用于调试和模型初值。左右轮可以分别配置：

```c
#define APP_ASCENTO_LEFT_CURRENT_MA_TO_WHEEL_TORQUE_NM ...
#define APP_ASCENTO_RIGHT_CURRENT_MA_TO_WHEEL_TORQUE_NM ...
```

VESC 上位机完成 FOC 检测后，`motor_current` 才能作为可信的电流反馈。

## 17. 轮端力矩 Nm/mA 标定方案

目标：测得或修正左右轮各自的 `Nm/mA` 系数。

### 静态力臂测量法

所需工具：弹簧秤/电子秤/拉力计、卷尺/游标卡尺、固定夹具/绳索。

步骤：

1. 测量力臂 r（轮胎有效半径，轮轴中心到轮胎外缘接地点距离，单位 m）。
2. 在轮子外缘固定细绳，绳另一端接弹簧秤，弹簧秤另一端固定在机架上。
3. 发送电流命令（如 `motor wheel current left 300 5000`），等电流稳定后记录弹簧秤读数。
4. 至少 4 个电流档位（含正负），每档测 2-3 次取平均。
5. 按公式计算：`τ (Nm) = F (N) × r (m)`，`k (Nm/mA) = τ / I_cmd`。
6. 对所有有效 k 值取平均或线性回归，填入 `src/app_config.h`。

注意事项：

- VESC 默认有堵转检测，5000ms 超时足够读数。
- 代码中电流调试限幅 2500 mA，测 1000 mA 以下足够。
- 如果 F407 和 VESC Tool 测出的 k 值差异 > 5%，优先用 F407 的（平衡控制器走 F407 → CAN → VESC 路径）。
- 左右轮可能因减速器个体差异有 2-5% 差异，建议分别标定。

## 18. 串口终端使用

串口设备（如 `/dev/ttyUSB0`）属于 `dialout` 组，当前用户需要加入该组：

```bash
sudo usermod -a -G dialout $USER
```

然后注销重新登录使其生效。临时可用 `newgrp dialout`。

启动串口：

```bash
./scripts/serial.sh
```

默认自动查找 `/dev/serial/by-id/*` 下的设备，波特率 115200。也可手动指定：

```bash
./scripts/serial.sh /dev/ttyUSB0 115200
```

picocom 基本操作：直接打字发送，回车换行，`Ctrl+A` `Ctrl+X` 退出。
