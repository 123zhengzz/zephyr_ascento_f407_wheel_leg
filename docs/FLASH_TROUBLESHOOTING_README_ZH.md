# DJI F407 C 板 Zephyr 烧录排错详细 README

本文记录本工程从编译、OpenOCD/st-flash 烧录失败，到 4pin SWD 无 NRST 快速 OpenOCD 烧录成功的完整排错过程，适合初学者按步骤检查。

如果 C 板只有 4pin SWD、没有 NRST，优先看本次成功记录：

```text
docs/STLINK_4PIN_FLASH_README_ZH.md
```

工程目录：

```bash
cd /home/h/code_leg/zephyr_ascento_f407_wheel_leg
```

最终本次 4pin SWD 成功使用的是：

```bash
./scripts/openocd_flash_fast_4pin.sh
```

成功标志：

```text
wrote 262144 bytes from file build/zephyr/zephyr.bin
shutdown command invoked
```

看到这两行且中间没有 `Error`，说明固件已经写入 STM32F407 内部 Flash。若脚本使用默认 `OPENOCD_VERIFY=0`，这是写入成功但未校验；断电重上电后再通过串口日志或 LED 状态确认固件运行。

## 1. 烧录前安全检查

烧录和第一次电机调试时，建议这样做：

1. 断开电机动力电，只给 C 板供电。
2. 轮子架空，或者先不装轮子。
3. 达妙关节不要带负载，手不要放在连杆和关节附近。
4. 先完成烧录，再接串口 shell，再单独测试每个电机。
5. 电机调试时一旦异常，先断电。

注意：ST-LINK 的 `3.3V/VTref` 线通常只是检测目标板电压，不建议用 ST-LINK 给整块 C 板和外设供电。

## 2. 推荐接线

### 2.1 最小可尝试接线

有时只接下面 4 根也能烧录：

| ST-LINK | DJI F407 C 板 | 作用 |
| --- | --- | --- |
| SWDIO | SWDIO | SWD 数据 |
| SWCLK | SWCLK | SWD 时钟 |
| GND | GND | 共地 |
| 3.3V / VTref | 3.3V | 目标板电压检测 |

这个接法可以识别芯片，但是遇到 `timed out while waiting for target halted` 或 Flash 写入失败时，稳定性不够。

如果 C 板 SWD 口只有 4pin，没有引出 NRST，就不要使用 `STLINK_MODE=under-reset`。这种情况用“断电冷启动 + 热连接 + 降低 SWD 速度”的方式恢复：

```bash
pkill -9 openocd 2>/dev/null || true
st-info --probe
STLINK_MODE=hotplug STLINK_FREQ_KHZ=50 ./scripts/stlink_flash.sh
```

如果 `st-info --probe` 看到 `chipid: 0x000` 或 `dev-type: unknown`，不要继续烧录。先拔掉 ST-LINK USB、断开 C 板电源，等待几秒后只保留 C 板稳定供电和 SWD 四根线，再重新执行 `st-info --probe`。

### 2.2 推荐稳定接线：加 RST/NRST

推荐接 5 根：

| ST-LINK | DJI F407 C 板 | 作用 |
| --- | --- | --- |
| SWDIO | SWDIO | SWD 数据 |
| SWCLK | SWCLK | SWD 时钟 |
| GND | GND | 共地 |
| 3.3V / VTref | 3.3V | 目标板电压检测 |
| NRST / RST / RESET | NRST / RST | 复位 MCU，方便烧录器稳定 halt |

不同 ST-LINK 外壳上可能写法不同：

```text
NRST = RST = RESET
```

接法就是：

```text
ST-LINK NRST/RST/RESET  ->  C板 NRST/RST
```

不要把 RST 接到 BOOT0，不要接到 5V，不要接到电机电源。

### 2.3 为什么要接 RST/NRST

OpenOCD 烧录时经常需要让 STM32 复位并停在可调试状态。如果没有接 NRST，OpenOCD 只能靠软件 halt，有时会出现：

```text
Error: timed out while waiting for target halted
TARGET: stm32f4x.cpu - Not halted
```

接上 NRST 后，ST-LINK 可以硬件复位 MCU，烧录会更稳定。

## 3. 编译固件

每次改代码后先编译：

```bash
cd /home/h/code_leg/zephyr_ascento_f407_wheel_leg
./scripts/build.sh
```

编译成功后会生成：

```text
build/zephyr/zephyr.elf
build/zephyr/zephyr.hex
build/zephyr/zephyr.bin
```

这三个文件的区别：

| 文件 | 用途 |
| --- | --- |
| `zephyr.elf` | 带符号信息，适合 GDB 调试 |
| `zephyr.hex` | OpenOCD 常用烧录文件 |
| `zephyr.bin` | st-flash 常用烧录文件 |

## 4. 推荐烧录流程

### 4.1 先检查 ST-LINK 是否被电脑识别

```bash
lsusb | grep -i -E "st-link|stlink|0483:374"
```

正常会看到类似：

```text
Bus 003 Device 007: ID 0483:3748 STMicroelectronics ST-LINK/V2
```

也可以用工程脚本：

```bash
./scripts/check_devices.sh
```

### 4.2 优先推荐：stlink-tools 烧录

如果没有安装：

```bash
sudo apt install stlink-tools
```

烧录：

```bash
./scripts/stlink_flash.sh
```

脚本默认使用 100 kHz SWD，并在写入后复位目标板。遇到写入阶段不稳定时，可以切换连接模式：

```bash
STLINK_MODE=hotplug ./scripts/stlink_flash.sh
STLINK_MODE=under-reset ./scripts/stlink_flash.sh   # 需要 ST-LINK NRST 接到 C 板 NRST
```

这个脚本会先执行：

```bash
st-info --probe
```

如果正常，会看到类似：

```text
Found 1 stlink programmers
chipid:     0x413
dev-type:   STM32F4x5_F4x7
flash:      1048576
```

然后写入：

```text
Attempting to write 142352 bytes to stm32 address: 0x8000000
Flash written and verified! jolly good!
```

### 4.3 备用：OpenOCD 烧录

```bash
./scripts/flash.sh
```

或者：

```bash
./scripts/openocd_flash.sh
```

如果 OpenOCD 不稳定，或者 C 板只有 4pin SWD、没有 NRST，可以试慢速脚本：

先试 4pin 快速模式。它使用较小 SRAM 工作区，通常比完全慢速模式快：

```bash
./scripts/openocd_flash_fast_4pin.sh
```

如果仍然 USB 超时或 Flash loader 失败，再退回完全慢速模式：

```bash
./scripts/openocd_flash_bin_slow.sh
```

这个脚本使用：

```text
50 kHz SWD
zephyr.bin
禁用 OpenOCD RAM 工作区
```

它可能很慢。看到下面提示不是报错，后面可能安静 5-15 分钟：

```text
falling back to single memory accesses
```

这表示 OpenOCD 改成慢速单次写入。不要按 `Ctrl-C`，除非你明确要中断烧录。

如果只是想先确认能写进去、暂时跳过校验来减少等待时间，可以用：

```bash
OPENOCD_VERIFY=0 ./scripts/openocd_flash_bin_slow.sh
```

## 5. 本次遇到的报错和解决方案

### 5.1 `Error: open failed`

报错示例：

```text
Error: open failed
FATAL ERROR: command exited with status 1: openocd ...
```

含义：

OpenOCD 没有成功打开 ST-LINK。

常见原因：

1. ST-LINK 没插好。
2. USB 线不是数据线。
3. Linux 没识别到 ST-LINK。
4. 虚拟机或 WSL 没有透传 USB。
5. 权限不够。
6. 另一个 OpenOCD 或 st-flash 正在占用 ST-LINK。

检查：

```bash
lsusb | grep -i -E "st-link|stlink|0483:374"
pgrep -a openocd
```

解决：

```bash
pkill -9 openocd
拔插 ST-LINK USB
./scripts/check_devices.sh
```

如果是权限问题，可以临时试：

```bash
sudo ./scripts/openocd_flash.sh
```

或者使用 `stlink-tools`：

```bash
./scripts/stlink_flash.sh
```

### 5.2 `timed out while waiting for target halted`

报错示例：

```text
Info : STLINK V2J37S7
Info : Target voltage: 3.213639
Info : Cortex-M4 r0p1 processor detected
Error: timed out while waiting for target halted
TARGET: stm32f4x.cpu - Not halted
```

含义：

ST-LINK 已经识别到了板子，目标电压也正常，但是 OpenOCD 复位后没能让 MCU 停下来。

常见原因：

1. 没接 NRST/RST。
2. OpenOCD 使用的复位方式不适合当前接线。
3. SWD 速度偏快。
4. 板子上旧程序运行后影响调试接口。

本工程做过的处理：

1. 把 OpenOCD 配置改为不强制 `srst_only`。
2. 把 SWD 速度降到 `500 kHz`。
3. `scripts/flash.sh` 改为使用 `halt` 而不是强依赖 `reset init`。

推荐解决方案：

```bash
./scripts/flash.sh
```

如果还不行，接 RST：

```text
ST-LINK NRST/RST/RESET -> C板 NRST/RST
```

然后再烧录：

```bash
./scripts/openocd_flash.sh
```

### 5.3 `flash write algorithm aborted by target`

报错示例：

```text
Info : flash size = 1024 kbytes
flash 'stm32f2x' found at 0x08000000
stm32f2x user_options 0xEC
Error: flash write algorithm aborted by target
Error: error executing stm32x flash write algorithm
Error: flash write failed = 0x000000c0
Error: error writing to flash at address 0x08000000 at offset 0x00000000
auto erase enabled
auto unlock enabled
```

含义：

OpenOCD 已经能连接 ST-LINK、识别 STM32F407、halt CPU、读到 Flash 信息，但是执行写 Flash 的小程序时失败。

这说明 SWD 基本接线大概率没问题，问题发生在 Flash 写入阶段。

常见原因：

1. OpenOCD 的 Flash loader 在目标 SRAM 中运行失败。
2. 复位不干净，建议接 NRST。
3. 板子供电不稳。
4. SWD 速度或 OpenOCD 工作区设置不适合。
5. Flash 保护状态或旧程序影响烧录。

排查命令：

```bash
./scripts/openocd_probe_flash.sh
```

如果能看到：

```text
flash 'stm32f2x' found at 0x08000000
stm32f2x user_options 0xEC
```

说明 OpenOCD 能读到 Flash 信息。

优先解决方案：

```bash
sudo apt install stlink-tools
./scripts/stlink_flash.sh
```

本次就是用这个方法成功的。

如果还想继续用 OpenOCD，可以试：

```bash
./scripts/openocd_flash_bin_slow.sh
```

或者接上 NRST 后：

```bash
./scripts/openocd_mass_erase_flash.sh
```

注意：`openocd_mass_erase_flash.sh` 会擦除 STM32 内部 Flash 旧程序。

### 5.4 `Warn : no working area available`

报错示例：

```text
Warn : not enough working area available(requested 76)
Warn : no working area available, can't do block memory writes
Warn : couldn't use block writes, falling back to single memory accesses
```

含义：

这不是致命错误。OpenOCD 只是说不能使用 RAM 工作区做高速块写入，于是退回到很慢的单次写入。

正确处理：

等它继续跑，不要立刻按 `Ctrl-C`。

如果你按了 `Ctrl-C`，OpenOCD 可能残留在后台，占用 ST-LINK。清理：

```bash
pgrep -a openocd
pkill -9 openocd
```

然后重新检查：

```bash
st-info --probe
```

### 5.5 `Flash loader write error`

报错示例：

```text
ERROR flash_loader.c: Flash loader write error
WARN flash_loader.c: Loader state: R2 0x8000 R15 0xFFFFFFFE
WARN flash_loader.c: MCU state: DHCSR 0x3000B DFSR 0x8 CFSR 0x1 HFSR 0x40000000
ERROR flash_loader.c: stlink_flash_loader_run(0x8000000) failed! == -1
stlink_fwrite_flash() == -1
```

含义：

ST-LINK 已经识别到 STM32F407，也已经把写 Flash 的小程序加载到 SRAM，但这个小程序运行时 HardFault 了。它通常不是 Zephyr 固件格式问题，而是烧录链路在写入阶段不稳定。

优先尝试：

```bash
./scripts/stlink_flash.sh
STLINK_MODE=hotplug ./scripts/stlink_flash.sh
```

如果仍失败，接上 NRST 后尝试：

```bash
STLINK_MODE=under-reset ./scripts/stlink_flash.sh
```

还不行时，换 OpenOCD 慢速无 RAM 工作区写入：

```bash
./scripts/openocd_flash_bin_slow.sh
```

同时检查 C 板供电、SWDIO/SWCLK/GND 线长和接触，烧录时建议断开电机动力电，只保留 C 板供电和 ST-LINK/SWD。

### 5.6 `Couldn't find any ST-Link devices`

报错示例：

```text
st-flash 1.8.0
WARN usb.c: Couldn't find any ST-Link devices
```

含义：

`st-flash` 没有拿到 ST-LINK 设备。

本次实际原因：

前面慢速 OpenOCD 被 `Ctrl-C` 中断后，OpenOCD 进程还残留着，占用了 ST-LINK。

检查：

```bash
pgrep -a openocd
lsusb | grep -i -E "st-link|stlink|0483:374"
```

解决：

```bash
pkill -9 openocd
拔插 ST-LINK USB
st-info --probe
./scripts/stlink_flash.sh
```

如果 `st-info --probe` 能看到：

```text
Found 1 stlink programmers
```

就可以继续烧录。

### 5.7 `FATAL: cannot open /dev/ttyUSB0`

报错示例：

```text
FATAL: cannot open /dev/ttyUSB0: No such file or directory
```

含义：

这不是烧录失败，而是串口设备不存在。

常见原因：

1. CH340 没插。
2. USB 转串口线不是数据线。
3. 设备名不是 `/dev/ttyUSB0`，可能是 `/dev/ttyUSB1` 或 `/dev/ttyACM0`。
4. 权限不足。

检查：

```bash
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
ls /dev/serial/by-id/ 2>/dev/null
```

推荐打开串口：

```bash
./scripts/serial.sh
```

或者指定设备：

```bash
./scripts/serial.sh /dev/ttyUSB0
```

注意：串口 shell 是烧录成功后用来发电机调试命令的，不是用来烧录固件的。

## 6. 初学者分层判断法

遇到问题时，按层判断：

| 层级 | 正常现象 | 如果异常 |
| --- | --- | --- |
| USB 层 | `lsusb` 能看到 `0483:3748 ST-LINK/V2` | 检查 USB 线、拔插、虚拟机透传 |
| ST-LINK 层 | `st-info --probe` 能看到 `Found 1 stlink programmers` | 关闭 OpenOCD 残留进程，检查权限 |
| SWD 层 | 能看到 `Cortex-M4 r0p1 processor detected` | 检查 SWDIO/SWCLK/GND/VTref |
| 复位层 | 能看到 `target halted` | 接 NRST/RST，降低 SWD 速度 |
| Flash 层 | 能看到 `Flash written and verified` | 换 `stlink_flash.sh`，必要时 mass erase |
| 串口层 | 能打开 `uart:~$` | 检查 CH340、串口号、TX/RX/GND |

## 7. 本工程相关脚本说明

| 脚本 | 用途 | 何时使用 |
| --- | --- | --- |
| `scripts/build.sh` | 编译 Zephyr 固件 | 每次改代码后 |
| `scripts/check_devices.sh` | 检查 ST-LINK 和串口 | 烧录或串口打不开时 |
| `scripts/flash.sh` | west + OpenOCD 烧录 | 常规 OpenOCD 烧录 |
| `scripts/openocd_flash.sh` | 直接 OpenOCD 烧录 | `west flash` 不稳定时 |
| `scripts/openocd_probe_flash.sh` | 只探测 Flash，不写入 | 判断 OpenOCD 是否能读到 Flash |
| `scripts/openocd_flash_bin_slow.sh` | 慢速 OpenOCD 烧录 | 不想接 NRST 且 OpenOCD 写入失败时 |
| `scripts/openocd_mass_erase_flash.sh` | 整片擦除后写入 | 旧程序或 Flash 状态异常时 |
| `scripts/stlink_flash.sh` | stlink-tools 烧录 | 本次最终成功方案，推荐 |
| `scripts/serial.sh` | 打开串口 shell | 烧录成功后调试电机 |

## 8. 推荐最终流程

以后从零开始，建议按这个顺序：

```bash
cd /home/h/code_leg/zephyr_ascento_f407_wheel_leg
./scripts/build.sh
./scripts/check_devices.sh
./scripts/stlink_flash.sh
```

如果 `stlink_flash.sh` 找不到 ST-LINK：

```bash
pkill -9 openocd
拔插 ST-LINK USB
st-info --probe
./scripts/stlink_flash.sh
```

如果 OpenOCD 报复位或 halt 问题：

```text
ST-LINK NRST/RST/RESET -> C板 NRST/RST
```

然后再试：

```bash
./scripts/openocd_flash.sh
```

烧录成功后再打开串口：

```bash
./scripts/serial.sh
```

看到 `uart:~$` 后，才是在固件里发调试命令的时候。

## 9. 本次成功记录

本次 `stlink_flash.sh` 的关键成功输出：

```text
Found 1 stlink programmers
chipid:     0x413
dev-type:   STM32F4x5_F4x7
Attempting to write 142352 bytes to stm32 address: 0x8000000
Successfully loaded flash loader in sram
Flash written and verified! jolly good!
Go to Thumb mode
```

这个结果说明：

1. ST-LINK 正常。
2. C 板供电正常。
3. STM32F407 被识别。
4. Flash loader 成功加载进 SRAM。
5. 固件已经写入并校验通过。
