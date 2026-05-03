# VESC 手册核对、编译烧录和轮电机调试 README

工程目录：

```bash
cd /home/h/code_leg/zephyr_ascento_f407_wheel_leg
```

参考手册：

```text
docs/2.28修订版说明书.pdf
```

本文只针对 CAN2 上的 VESC + DJI M3508 轮电机。CAN1 达妙关节调试见 `docs/MOTOR_DEBUG_README_ZH.md`。

## 1. 本次代码核对结论

根据 VESC 手册里的 CAN 通信说明，当前代码不需要再改 VESC 协议层。已经对齐的内容如下：

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

## 2. VESC Tool 必须确认的参数

两个 VESC 都要设置：

| 项目 | 左 VESC | 右 VESC |
| --- | --- | --- |
| Controller ID | `101` | `100` |
| CAN Baud | `1 Mbps` | `1 Mbps` |
| CAN Mode | `VESC` | `VESC` |
| CAN Status 1 | `200 Hz` | `200 Hz` |

M3508 速度换算：

```text
电机本体机械 rpm = ERPM / 7
减速箱输出轴 rpm = ERPM / 7 / 19.203
```

例如 `target_erpm=1000` 时，减速箱输出轴大约 `7.4 rpm`。

## 3. 编译指令

推荐直接用工程脚本：

```bash
cd /home/h/code_leg/zephyr_ascento_f407_wheel_leg
./scripts/build.sh
```

编译成功后会生成：

```text
build/zephyr/zephyr.elf
build/zephyr/zephyr.bin
build/zephyr/zephyr.hex
```

如果想手动执行 west：

```bash
cd /home/h/code_leg/zephyr_ascento_f407_wheel_leg
source ./zephyr-env.sh
west build -b dji_f407igh6_c -p always .
```

## 4. 烧录指令

你之前已经用 `st-flash` 成功烧录过，所以优先用：

```bash
cd /home/h/code_leg/zephyr_ascento_f407_wheel_leg
./scripts/stlink_flash.sh
```

如果想用 Zephyr/OpenOCD：

```bash
./scripts/flash.sh
```

OpenOCD 失败时再尝试：

```bash
./scripts/openocd_flash.sh
./scripts/openocd_flash_bin_slow.sh
```

如果 OpenOCD 报 `timed out while waiting for target halted`，建议把 ST-LINK 的 `NRST` 接到 C 板 `NRST` 后再烧。更完整的烧录排错见：

```text
docs/FLASH_TROUBLESHOOTING_README_ZH.md
```

## 5. 打开串口 shell

烧录后打开串口：

```bash
cd /home/h/code_leg/zephyr_ascento_f407_wheel_leg
./scripts/serial.sh
```

如果脚本找不到串口，先查设备：

```bash
./scripts/check_devices.sh
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

看到下面提示后，后续命令都在串口 shell 输入：

```text
uart:~$
```

不是在 Linux 终端里输入 `motor ...` 命令。

## 6. 上电和接线检查

CAN2 接线：

| C 板 CAN2 | 左 VESC | 右 VESC |
| --- | --- | --- |
| CAN2_H | CAN_H | CAN_H |
| CAN2_L | CAN_L | CAN_L |
| GND | GND | GND |

必须满足：

1. 轮子架空。
2. VESC 和 C 板共地。
3. CAN_H/CAN_L 不接反。
4. CAN2 总线两端有 120 欧终端电阻。
5. 左 VESC ID 是 `101`，右 VESC ID 是 `100`。
6. 两个 VESC 都设置为 `1 Mbps` 和 Status 1 `200 Hz`。

## 7. VESC 反馈检查

先停止所有运动输出：

```text
robot enable 0
robot stop
motor debug stop
motor wheel stop
```

查看 CAN 状态：

```text
motor can status all
```

正常应看到 CAN2 是 `error-active`，例如：

```text
wheel/CAN2 state=error-active tx_err=0 rx_err=0
```

查看 VESC 反馈：

```text
motor wheel status all
```

正常应类似：

```text
VESC/M3508 id=101 age=5ms erpm=0 motor_rpm=0 angle=0.000 rad speed=0.000 rad/s cmd=0 mA motor_current=0 mA input=0 mA vin=0.00 V temp=0.0/0.0C tach=0 torque_k=0.003457 torque_est=0.000 Nm duty=0.000 s4_age=-1ms s5_age=-1ms
VESC/M3508 id=100 age=5ms erpm=0 motor_rpm=0 angle=0.000 rad speed=0.000 rad/s cmd=0 mA motor_current=0 mA input=0 mA vin=0.00 V temp=0.0/0.0C tach=0 torque_k=0.003457 torque_est=0.000 Nm duty=0.000 s4_age=-1ms s5_age=-1ms
```

其中：

| 字段 | 含义 |
| --- | --- |
| `cmd` | F407 最近一次通过 CAN 发给 VESC 的电流命令 |
| `motor_current` | VESC Status 1 回传的实测电机电流 |
| `input` | VESC Status 4 回传的输入电流，需要在 VESC Tool 开启 Status 4 |
| `vin` | VESC Status 5 回传的输入电压，需要在 VESC Tool 开启 Status 5 |
| `temp` | VESC Status 4 回传的 FET/电机温度 |
| `tach` | VESC Status 5 回传的 tachometer |
| `torque_k` | 当前固件用于该 VESC ID 的 `Nm/mA` 系数 |
| `torque_est` | `motor_current * torque_k` 得到的轮端力矩估计 |
| `duty` | VESC 占空比 |
| `s4_age/s5_age` | Status 4/5 距离上次更新的时间，`-1ms` 表示还没有收到 |

VESC 上位机完成 FOC 检测后，`motor_current` 才能作为可信的电流反馈。当前固件按 M3508 理论力矩常数、减速比和效率计算左右轮默认 `torque_k`。如果左右 VESC 或电机需要不同修正，改 `APP_ASCENTO_LEFT_CURRENT_MA_TO_WHEEL_TORQUE_NM` 和 `APP_ASCENTO_RIGHT_CURRENT_MA_TO_WHEEL_TORQUE_NM`。

默认计算公式：

```text
torque_k = Kt * reduction_ratio * gearbox_efficiency * 0.001
```

对应代码：

```c
#define APP_M3508_MOTOR_KT_NM_PER_A 0.180f
#define APP_M3508_REDUCTION_RATIO 19.203208f
#define APP_M3508_GEARBOX_EFFICIENCY 1.000f
```

判断：

| 现象 | 处理 |
| --- | --- |
| `no feedback` | 查 VESC 是否上电、ID、CAN2 H/L/GND、终端电阻、CAN Baud、Status 1 |
| `age` 大于 `50 ms` 或一直变大 | 反馈曾经来过但现在断了，查接线、电源和 VESC 回传频率 |
| CAN2 `bus-off` | 先断电，重点查 H/L 是否接反、终端电阻和波特率 |

## 8. 电流模式调试

电流模式适合先试轮子有没有力矩，也适合后续平衡控制。单位按 `mA` 理解。

左轮：

```text
robot enable 0
motor wheel stop
motor wheel current left 100 3000
motor wheel status left
motor wheel stop
```

右轮：

```text
robot enable 0
motor wheel stop
motor wheel current right 100 3000
motor wheel status right
motor wheel stop
```

如果 `100 mA` 不动，再试：

```text
motor wheel current left 300 500
motor wheel stop
```

不要一开始就用大电流。当前调试限幅是：

```text
-2500 mA 到 2500 mA
```

## 9. RPM 模式调试

RPM 模式对应手册里的 `CAN_PACKET_SET_RPM = 3`。这里的参数是 `ERPM`，不是减速箱输出轴 rpm。

左轮低速：

```text
motor wheel rpm left 500 300
motor wheel status left
motor wheel stop
```

右轮低速：

```text
motor wheel rpm right 500 300
motor wheel status right
motor wheel stop
```

反向测试：

```text
motor wheel rpm left -500 300
motor wheel rpm right -500 300
motor wheel stop
```

当前 RPM 调试限幅是：

```text
-8000 ERPM 到 8000 ERPM
```

## 10. 按手册手动发 VESC 原始帧

新手优先用 `motor wheel current/rpm`。如果你想按手册验证原始扩展帧，可以用 `rawx`。

右轮 ID100，目标 `1000 ERPM`：

```text
motor can rawx can2 0x364 0 0 3 232 0 0 0 0
```

左轮 ID101，目标 `1000 ERPM`：

```text
motor can rawx can2 0x365 0 0 3 232 0 0 0 0
```

解释：

```text
0x364 = (CAN_PACKET_SET_RPM 3 << 8) | 0x64
0x365 = (CAN_PACKET_SET_RPM 3 << 8) | 0x65
1000 = 0x000003E8 = 0 0 3 232
```

停止：

```text
motor wheel stop
motor debug stop
```

## 11. 左右轮同时调试

同向：

```text
motor wheel pair 100 100 3000
motor wheel status all
motor wheel stop
```

反向：

```text
motor wheel pair 100 -100 3000
motor wheel status all
motor wheel stop
```

记录方向：

| 命令 | 左轮实际方向 | 右轮实际方向 | 是否符合机器人前进方向 |
| --- | --- | --- | --- |
| `motor wheel pair 100 100 3000` | 已测：正转 | 已测：反转 | 左右相反 |
| `motor wheel pair -100 -100 3000` |  |  |  |
| `motor wheel pair 100 -100 3000` |  |  |  |

## 12. 每次调试结束

必须清零输出：

```text
robot enable 0
robot stop
motor debug stop
motor wheel stop
```

然后先断 VESC/M3508 动力电，再断 C 板控制电。

## 13. 如果需要改代码的位置

常用参数在：

```text
src/app_config.h
```

关键值：

```c
#define APP_WHEEL_LEFT_ID 101
#define APP_WHEEL_RIGHT_ID 100
#define APP_CAN_BITRATE 1000000U
#define APP_VESC_STATUS_RATE_HZ 200
#define APP_VESC_FEEDBACK_TIMEOUT_MS 50
#define APP_VESC_DEBUG_ERPM_LIMIT 8000
#define APP_WHEEL_CURRENT_SAFE 2500
```

VESC CAN 协议实现：

```text
src/dji_m3508.c
```

串口 shell 命令：

```text
src/shell_commands.c
```

改完代码后重新：

```bash
./scripts/build.sh
./scripts/stlink_flash.sh
./scripts/serial.sh
```
