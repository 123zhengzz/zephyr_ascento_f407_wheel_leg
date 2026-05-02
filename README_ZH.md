# DJI F407 C 板 Ascento 轮腿机器人调试手册

工程目录：

```bash
cd /home/h/code_leg/zephyr_ascento_f407_wheel_leg
```

本工程使用双 CAN：

| 总线 | 设备 | 默认 ID | 用途 |
| --- | --- | --- | --- |
| CAN1 | 达妙 DM4340/DM43xx 关节电机 | 左 1，右 2 | 腿部关节 |
| CAN2 | VESC + DJI M3508 | 左 101，右 100 | 左右轮 |

串口 shell：USART1，115200 波特率。

所有调试命令都在串口 shell 里输入，也就是看到 `uart:~$` 以后再输入，不是在 Linux 终端里输入。

更细的电机接线和参数调试说明见：

```text
docs/MOTOR_DEBUG_README_ZH.md
```

按 VESC 手册核对代码、编译烧录和 CAN2 轮电机调试指令见：

```text
docs/VESC_CODE_CHECK_AND_DEBUG_README_ZH.md
```

烧录报错、ST-LINK/OpenOCD/st-flash 排错、RST/NRST 接线说明见：

```text
docs/FLASH_TROUBLESHOOTING_README_ZH.md
```

如果 C 板只有 4pin SWD、没有 NRST，优先看本次成功记录：

```text
docs/STLINK_4PIN_FLASH_README_ZH.md
```

暂不启用的 Ascento 串联腿平衡控制代码和所需物理数据见：

```text
docs/ASCENTO_BALANCE_REQUIREMENTS_ZH.md
```

## 0. 安全准备

第一次调试一定按下面做：

1. 轮子架空，或者先把轮子拆下。
2. 达妙关节不要带重负载，连杆附近不要放手。
3. 电池旁边准备断电开关。
4. 所有电机先用小命令试，不要一上来 `robot enable 1`。
5. 任何时候不对劲，先输入：

```text
robot enable 0
motor debug stop
```

如果串口已经卡住或电机异常，直接断电。

## 1. 当前文件夹的 Zephyr 环境

我已经把 Zephyr 环境配置到当前工程目录，根目录有这些脚本：

```text
zephyr-env.sh              # 手动进入 Zephyr 环境
scripts/build.sh           # 编译
scripts/flash.sh           # west + OpenOCD 烧录
scripts/openocd_flash.sh   # 直接 OpenOCD 烧录
scripts/debug.sh           # GDB 调试
scripts/serial.sh          # 打开串口终端
scripts/check_devices.sh   # 检查 ST-LINK 和串口是否被电脑识别
```

手动进入环境：

```bash
cd /home/h/code_leg/zephyr_ascento_f407_wheel_leg
source ./zephyr-env.sh
```

看到类似下面说明环境正常：

```text
Zephyr environment ready.
Project: /home/h/code_leg/zephyr_ascento_f407_wheel_leg
Zephyr:  /home/h/code/zephyr_robot/zephyr
SDK:     /home/h/zephyr-sdk-0.17.0
```

以后最常用的是直接运行：

```bash
./scripts/build.sh
```

## 2. 编译固件

在 Linux 终端执行：

```bash
cd /home/h/code_leg/zephyr_ascento_f407_wheel_leg
./scripts/build.sh
```

编译成功会生成：

```text
build/zephyr/zephyr.elf
build/zephyr/zephyr.bin
build/zephyr/zephyr.hex
```

如果提示 `west: command not found`，说明没有使用本工程脚本，改用：

```bash
source ./zephyr-env.sh
west build -b dji_f407igh6_c -p always .
```

## 3. 烧录固件

接线：

| ST-LINK | C 板 |
| --- | --- |
| SWDIO | SWDIO |
| SWCLK | SWCLK |
| GND | GND |
| 3V3 sense | 3V3 |

烧录：

```bash
./scripts/flash.sh
```

如果出现下面错误：

```text
Error: open failed
```

这说明 OpenOCD 没有打开 ST-LINK。先执行：

```bash
./scripts/check_devices.sh
```

然后按结果判断：

| 现象 | 原因 | 处理 |
| --- | --- | --- |
| `No ST-LINK found` | Linux 没识别到 ST-LINK | 重新插 ST-LINK，检查 USB 线，虚拟机/WSL 要把 USB 设备透传进 Linux |
| `ST-LINK appears in lsusb` 但仍 `open failed` | 可能是权限问题 | 试 `sudo ./scripts/openocd_flash.sh`，或配置 udev 权限 |
| ST-LINK 正常但连不上目标 | C 板没上电或 SWD 接线错 | 检查 SWDIO、SWCLK、GND、3V3 sense，必要时接 NRST |
| `timed out while waiting for target halted` | ST-LINK 已连上，但 OpenOCD 复位后没有停住 MCU；常见原因是 NRST 没接，或复位方式不匹配 | 先用新版 `./scripts/flash.sh`；它已改为 `halt` 后烧录。如果还失败，接 ST-LINK 的 NRST 到 C 板 NRST，再试 `./scripts/openocd_flash.sh` |

如果 `west flash` 找不到 OpenOCD runner，可以用直接烧录脚本：

```bash
./scripts/openocd_flash.sh
```

如果 ST-LINK 能识别、CPU 也能 halt，但写入时报：

```text
flash write algorithm aborted by target
```

按这个顺序处理：

```bash
./scripts/openocd_probe_flash.sh
./scripts/openocd_flash.sh
```

如果还是失败，先接上 `NRST`，确认 C 板独立稳定供电，再尝试整片擦除后写入：

```bash
./scripts/openocd_mass_erase_flash.sh
```

如果 C 板只有 4pin SWD、没有 NRST，优先试本次成功的快速 4pin 脚本：

```bash
./scripts/openocd_flash_fast_4pin.sh
```

如果它仍然失败，或者你暂时不想多接 `NRST`，再退回慢速脚本：

```bash
./scripts/openocd_flash_bin_slow.sh
```

慢速脚本会使用 `zephyr.bin`、`50 kHz` SWD，并关闭 OpenOCD 的 RAM 工作区，速度较慢，但有时能绕开 `flash write algorithm aborted by target`。

如果系统安装了 `stlink-tools`，也可以不用 OpenOCD，直接试 ST-LINK 工具：

```bash
sudo apt install stlink-tools
./scripts/stlink_flash.sh
```

`openocd_mass_erase_flash.sh` 会擦除 STM32 内部 Flash 里的旧程序，然后写入当前工程固件。烧录调试时建议先断开电机动力电，只保留 C 板供电和 ST-LINK/SWD。

看到 `verified`、`reset`、`shutdown command invoked` 一类信息，表示烧录完成。

## 4. 打开串口 shell

先找串口设备：

```bash
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

假设看到 `/dev/ttyUSB0`，打开串口：

```bash
./scripts/serial.sh /dev/ttyUSB0
```

也可以让脚本自动选第一个串口：

```bash
./scripts/serial.sh
```

如果出现下面错误：

```text
FATAL: cannot open /dev/ttyUSB0: No such file or directory
```

说明当前没有 `/dev/ttyUSB0` 这个设备。先执行：

```bash
./scripts/check_devices.sh
```

然后按结果处理：

| 现象 | 原因 | 处理 |
| --- | --- | --- |
| 没有 `/dev/ttyUSB*` 或 `/dev/ttyACM*` | USB-TTL 没被识别 | 重新插 USB-TTL，换 USB 线或换 USB 口 |
| 有 `/dev/ttyACM0` | 串口号不是 `/dev/ttyUSB0` | 用 `./scripts/serial.sh /dev/ttyACM0` |
| 有 `/dev/ttyUSB1` | 串口号变了 | 用 `./scripts/serial.sh /dev/ttyUSB1` |
| 虚拟机/WSL 中看不到串口 | USB 没透传 | 在宿主机设置 USB 透传 |

串口参数：

| USB-TTL | C 板 / STM32 |
| --- | --- |
| RX | PA9 / USART1_TX |
| TX | PB7 / USART1_RX |
| GND | GND |
| 波特率 | 115200 |

按一下回车，应该看到：

```text
uart:~$
```

如果看不到 shell：

1. 检查 USB-TTL 的 TX/RX 是否交叉。
2. 检查 GND 是否共地。
3. 检查波特率是不是 115200。
4. 按 C 板复位键，看是否有启动日志。

## 5. 上电前检查 CAN 接线

CAN1 接达妙关节：

| C 板 CAN1 | 连接 |
| --- | --- |
| CAN1_H | 达妙左关节、右关节 CAN_H |
| CAN1_L | 达妙左关节、右关节 CAN_L |
| GND | 达妙电机 GND |

CAN2 接 VESC/M3508：

| C 板 CAN2 | 连接 |
| --- | --- |
| CAN2_H | 左右 VESC CAN_H |
| CAN2_L | 左右 VESC CAN_L |
| GND | VESC GND |

每条 CAN 都要有：

1. CAN_H 和 CAN_L 没接反。
2. 总线两端各一个 120 欧终端电阻。
3. C 板、电调、达妙电机共地。
4. 电机电源已经打开。

## 6. 检查两条 CAN 是否启动

在串口 shell 输入：

```text
motor can status all
```

正常情况类似：

```text
joint/CAN1 state=error-active tx_err=0 rx_err=0
wheel/CAN2 state=error-active tx_err=0 rx_err=0
```

判断：

| 现象 | 意义 | 处理 |
| --- | --- | --- |
| `error-active` | CAN 控制器状态正常 | 继续下一步 |
| `error-warning` 或 `error-passive` | 总线上有错误 | 检查 H/L、终端电阻、波特率 |
| `bus-off` | 总线严重错误 | 先断电检查接线，再复位 C 板 |
| 只有 CAN1 正常 | CAN2 接线或 VESC 电源可能有问题 | 先不要测轮子 |
| 只有 CAN2 正常 | CAN1 接线或达妙电源可能有问题 | 先不要测关节 |

## 7. 单独测试达妙关节电机

先只测 CAN1 上的达妙电机，不要启用整机平衡。

### 7.1 看达妙反馈

输入：

```text
motor dm status all
```

如果显示 `no feedback`，先不要慌。达妙有些模式需要 enable 或发命令后才稳定回反馈。

### 7.2 使能左关节

输入：

```text
motor dm enable left
```

再看状态：

```text
motor dm status left
```

正常时能看到位置、速度、温度，例如：

```text
DM4340 id=1 err=0 age=20ms pos=0.0000 rad vel=0.0000 rad/s ...
```

如果还是 `no feedback`：

1. 检查左关节 ID 是否是 1。
2. 检查达妙 Master ID 是否和 `src/app_config.h` 里的 `APP_DM_MASTER_ID` 一样，默认是 `0x00`。
3. 检查 CAN1 H/L/GND。
4. 检查达妙电机供电。

### 7.3 小角度转左关节

输入：

```text
motor dm pos left 0.10 1.0 800
```

含义：

| 参数 | 意义 |
| --- | --- |
| `left` | 左关节 |
| `0.10` | 目标位置 0.10 rad，大约 5.7 度 |
| `1.0` | 速度限制 1.0 rad/s |
| `800` | 持续 800 ms |

通过标准：

1. 左关节轻微转动。
2. 没有剧烈抖动。
3. `motor dm status left` 能看到位置变化。

停止左关节调试输出：

```text
motor dm stop left
```

### 7.4 测右关节

输入：

```text
motor dm enable right
motor dm pos right -0.10 1.0 800
motor dm status right
motor dm stop right
```

通过标准同左关节。

如果方向和你机械期望相反，先记录下来，不要急着改 PID。后面主要改 `src/app_config.h` 里的关节零位、最小/最大角度，或者改腿高映射符号。

## 8. 单独测试 M3508 轮子

轮子必须架空。

### 8.1 看 VESC/M3508 反馈

输入：

```text
motor wheel status all
```

正常时类似：

```text
VESC/M3508 id=101 age=5ms erpm=0 motor_rpm=0 angle=0.000 rad speed=0.000 rad/s current=0 mA duty=0.000
VESC/M3508 id=100 age=5ms erpm=0 motor_rpm=0 angle=0.000 rad speed=0.000 rad/s current=0 mA duty=0.000
```

如果 `no feedback`：

1. 检查 VESC ID，左轮应为 `101`，右轮应为 `100`。
2. 检查 VESC 是否上电。
3. 检查 CAN2 H/L/GND。
4. 确认 CAN2 总线终端电阻。
5. 确认 VESC CAN Baud 为 `1 Mbps`。
6. 确认 VESC Status 1 回传频率为 `200 Hz`。

### 8.2 小电流转左轮

输入：

```text
motor wheel current left 300 500
```

含义：

| 参数 | 意义 |
| --- | --- |
| `left` | 左轮 |
| `300` | VESC 电流命令，单位 mA，较小 |
| `500` | 持续 500 ms |

通过标准：

1. 左轮短暂转动。
2. 500 ms 后自动停止。
3. `motor wheel status left` 能看到 `erpm`、`speed`、`current` 或 `duty` 变化。

如果转得太猛，把 `300` 改成 `100`。如果不转，可以试 `500`，但不要长时间输出。

停止轮子：

```text
motor wheel stop
```

### 8.3 小电流转右轮

输入：

```text
motor wheel current right 300 500
motor wheel status right
motor wheel stop
```

通过标准同左轮。

### 8.4 同时测试左右轮

输入：

```text
motor wheel pair 300 300 500
motor wheel stop
```

然后反向测试：

```text
motor wheel pair -300 -300 500
motor wheel stop
```

记录：

| 命令 | 左轮方向 | 右轮方向 |
| --- | --- | --- |
| `300 300` | 自己填 | 自己填 |
| `-300 -300` | 自己填 | 自己填 |

如果某个轮方向和整机前进方向相反，后面要在控制输出符号或电机安装方向上修正。

## 9. 判断“电机能不能用”

每个电机都按这张表打勾：

| 电机 | 反馈正常 | 小命令会动 | 自动停止 | 方向已记录 |
| --- | --- | --- | --- | --- |
| 左达妙关节 ID1/CAN1 | [ ] | [ ] | [ ] | [ ] |
| 右达妙关节 ID2/CAN1 | [ ] | [ ] | [ ] | [ ] |
| 左 VESC/M3508 ID101/CAN2 | [ ] | [ ] | [ ] | [ ] |
| 右 VESC/M3508 ID100/CAN2 | [ ] | [ ] | [ ] | [ ] |

四个电机都满足后，才进入整机高度和平衡测试。

## 10. 测腿高控制

确认两个达妙关节都能动以后，输入：

```text
robot enable 0
motor debug stop
robot height 38
```

观察腿部是否到低位安全高度。

再输入：

```text
robot height 60
```

通过标准：

1. 左右腿一起变化。
2. 没有顶死机械限位。
3. 没有明显抖动。

如果方向反了，先停止：

```text
robot enable 0
motor debug stop
```

然后修改 `src/app_config.h` 里的这些参数：

```c
APP_LEFT_LEG_ZERO_RAD
APP_RIGHT_LEG_ZERO_RAD
APP_LEFT_LEG_MIN_RAD
APP_LEFT_LEG_MAX_RAD
APP_RIGHT_LEG_MIN_RAD
APP_RIGHT_LEG_MAX_RAD
APP_LEG_RAD_PER_HEIGHT_UNIT
```

改完重新：

```bash
./scripts/build.sh
./scripts/flash.sh
```

## 11. 架空测试平衡轮输出

这一步仍然不要落地，轮子保持架空。

输入：

```text
robot status
robot enable 1
```

轻轻前后倾斜车身，观察轮子是否有反应。

立刻停止：

```text
robot enable 0
```

通过标准：

1. 倾斜时轮子有响应。
2. 放平后输出变小。
3. 红灯不持续报警。
4. 轮子没有越倒越加速。

如果越倒越加速，说明方向或 IMU 符号不对。先不要落地，优先检查：

1. 左右轮是否接反。
2. M3508 正负电流方向是否和记录一致。
3. `src/bmi088.c` 里的 pitch 轴方向。
4. `src/control.c` 中左右轮电流输出符号。

## 12. 最小落地测试

只有前面全部通过，才进入落地。

1. 把 `APP_WHEEL_CURRENT_LIMIT` 保守设置，比如先不超过 `2500`。
2. 车旁边有人准备断电。
3. 输入：

```text
robot height 38
robot enable 1
```

4. 只扶着让它短时间站一下。
5. 立刻输入：

```text
robot enable 0
```

不要一开始就遥控、跳跃或高速移动。

## 13. 常用命令速查

CAN：

```text
motor can status all
motor can raw can1 0x001
motor can rawx can2 0x364 0 0 3 232 0 0 0 0   # 右轮 ID100，目标 1000 ERPM
motor can rawx can2 0x365 0 0 3 232 0 0 0 0   # 左轮 ID101，目标 1000 ERPM
```

达妙：

```text
motor dm status all
motor dm enable left
motor dm enable right
motor dm pos left 0.10 1.0 800
motor dm vel right 0.5 800
motor dm mit left 0.0 0.0 20.0 0.5 0.0 800
motor dm stop left
motor dm stop right
```

M3508：

```text
motor wheel status all
motor wheel current left 300 500
motor wheel current right 300 500
motor wheel rpm left 500 300
motor wheel rpm right 500 300
motor wheel pair 300 300 500
motor wheel stop
```

整机：

```text
robot status
robot height 38
robot height 60
robot joy 0 30
robot enable 1
robot enable 0
robot stop
robot jump
```

急停：

```text
robot enable 0
motor debug stop
```

## 14. 主要参数位置

电机 ID、限幅、PID 初值在：

```text
src/app_config.h
```

默认值：

```c
#define APP_WHEEL_LEFT_ID 101
#define APP_WHEEL_RIGHT_ID 100
#define APP_DM_LEFT_ID 1
#define APP_DM_RIGHT_ID 2
#define APP_DM_MASTER_ID 0x00
#define APP_CAN_BITRATE 1000000U
#define APP_VESC_STATUS_RATE_HZ 200
```

双 CAN 设备树在：

```text
boards/arm/dji_f407igh6_c/dji_f407igh6_c.dts
```

关键别名：

```dts
can-joint = &can1;
can-dm = &can1;
can-wheel = &can2;
can-m3508 = &can2;
```

## 15. 重新编译烧录的完整流程

每次改代码后，在 Linux 终端执行：

```bash
cd /home/h/code_leg/zephyr_ascento_f407_wheel_leg
./scripts/build.sh
./scripts/flash.sh
```

然后打开串口：

```bash
./scripts/serial.sh /dev/ttyUSB0
```

如果串口号不是 `/dev/ttyUSB0`，先运行：

```bash
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```
