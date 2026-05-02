# DJI F407 C 板 4pin SWD 烧录排错记录

适用场景：

- C 板 SWD 接口只有 4pin：`SWDIO / SWCLK / GND / 3V3(VTref)`。
- 没有引出 `NRST/RST/RESET`，不能使用真正的 `connect-under-reset`。
- ST-LINK 能识别 STM32F407，但烧录过程中偶发超时、HardFault 或写入失败。

## 1. 本次成功结果

这段输出表示已经写入成功：

```text
target halted due to debug-request, current mode: Handler HardFault
Info : flash size = 1024 kbytes
flash 'stm32f2x' found at 0x08000000
auto erase enabled
auto unlock enabled
wrote 262144 bytes from file build/zephyr/zephyr.bin in 45.352230s (5.645 KiB/s)
shutdown command invoked
```

判断标准：

1. 能识别到 `stm32f2x` Flash。
2. 出现 `auto erase enabled` 和 `auto unlock enabled`。
3. 出现 `wrote ... bytes from file build/zephyr/zephyr.bin`。
4. 最后是 `shutdown command invoked`，中间没有 `Error`。

本次使用的是 4pin 快速 OpenOCD 脚本：

```bash
./scripts/openocd_flash_fast_4pin.sh
```

注意：这个脚本默认 `OPENOCD_VERIFY=0`，也就是写入后不做校验，用来节省时间。写入成功后请手动断电重上电，再看串口日志或 LED 状态确认固件运行。

`wrote 262144 bytes` 比当前 `zephyr.bin` 文件大小更大时不用紧张。STM32F407 的 Flash 扇区比较大，OpenOCD 在擦写和对齐处理时可能按更大的块统计写入量。判断成功主要看是否出现 `wrote ...` 和 `shutdown command invoked`，且中间没有 `Error`。

如果需要写入后校验：

```bash
OPENOCD_VERIFY=1 ./scripts/openocd_flash_fast_4pin.sh
```

## 2. 最推荐的 4pin 烧录流程

每次烧录前先清理残留 OpenOCD：

```bash
pkill -9 openocd 2>/dev/null || true
```

物理连接只保留：

| ST-LINK | C 板 |
| --- | --- |
| SWDIO | SWDIO |
| SWCLK | SWCLK |
| GND | GND |
| 3V3 / VTref | 3V3 |

烧录时建议断开电机动力电，只保留 C 板稳定供电和 SWD。

先确认芯片识别正常：

```bash
st-info --probe
```

必须看到：

```text
chipid:     0x413
flash:      1048576
dev-type:   STM32F4x5_F4x7
```

然后快速烧录：

```bash
./scripts/openocd_flash_fast_4pin.sh
```

成功后手动断电重上电。

## 3. 速度调节

默认速度：

```bash
OPENOCD_ADAPTER_KHZ=100
OPENOCD_WORKAREASIZE=0x1000
OPENOCD_VERIFY=0
```

如果稳定，可以尝试加速：

```bash
OPENOCD_ADAPTER_KHZ=200 ./scripts/openocd_flash_fast_4pin.sh
```

如果出现 USB 超时或写入失败，降低速度和工作区：

```bash
OPENOCD_ADAPTER_KHZ=50 OPENOCD_WORKAREASIZE=0x800 ./scripts/openocd_flash_fast_4pin.sh
```

如果仍不稳定，退回完全慢速模式：

```bash
OPENOCD_VERIFY=0 ./scripts/openocd_flash_bin_slow.sh
```

完全慢速模式看到下面提示不是失败：

```text
falling back to single memory accesses
```

它表示 OpenOCD 改成单次写入，可能需要等待 5-15 分钟。

## 4. 本次失败点和含义

### 4.1 `Flash loader write error`

典型输出：

```text
ERROR flash_loader.c: Flash loader write error
WARN flash_loader.c: Loader state: R2 0x8000 R15 0xFFFFFFFE
WARN flash_loader.c: MCU state: DHCSR 0x3000B DFSR 0x8 CFSR 0x1 HFSR 0x40000000
ERROR flash_loader.c: stlink_flash_loader_run(0x8000000) failed! == -1
```

含义：

ST-LINK 已经识别到芯片，也把写 Flash 的小程序加载到了 SRAM，但这个小程序运行时异常了。常见原因是目标板状态不干净、SWD 链路不稳、供电不稳，或者 ST-LINK 工具的高速 loader 与当前目标状态不兼容。

处理：

```bash
pkill -9 openocd 2>/dev/null || true
```

然后拔插 ST-LINK、C 板断电重上电，再改用：

```bash
./scripts/openocd_flash_fast_4pin.sh
```

### 4.2 `LIBUSB_ERROR_TIMEOUT` / `LIBUSB_ERROR_IO`

典型输出：

```text
WRITEMEM_32BIT send request failed: LIBUSB_ERROR_TIMEOUT
write_buffer_to_sram() == -1
READDEBUGREG read reply failed: LIBUSB_ERROR_TIMEOUT
WRITEDEBUGREG send request failed: LIBUSB_ERROR_IO
```

含义：

烧录过程中电脑、ST-LINK、目标板之间的通信断续或超时。它不一定是固件问题，更像 USB/ST-LINK/SWD/供电链路问题。

处理顺序：

1. 换 USB 口，优先插电脑本机 USB 口，不走扩展坞。
2. 换一根短 USB 数据线。
3. 缩短 SWDIO/SWCLK/GND/3V3 线。
4. 断开电机动力电，只保留 C 板稳定供电。
5. 降低速度：

```bash
OPENOCD_ADAPTER_KHZ=50 OPENOCD_WORKAREASIZE=0x800 ./scripts/openocd_flash_fast_4pin.sh
```

### 4.3 `chipid: 0x000` / `dev-type: unknown`

典型输出：

```text
flash:      0
sram:       0
chipid:     0x000
dev-type:   unknown
```

含义：

此时 ST-LINK 本身可能还在，但没有正确连上 STM32F407。不要继续烧录。

处理：

1. 拔掉 ST-LINK USB。
2. C 板断电。
3. 等 5 秒。
4. 重新插 ST-LINK，C 板重新上电。
5. 重新执行：

```bash
st-info --probe
```

只有重新看到 `chipid: 0x413` 后再烧录。

### 4.4 `target halted ... Handler HardFault`

典型输出：

```text
target halted due to debug-request, current mode: Handler HardFault
xPSR: 0x01000003 pc: 0x0800dd60 msp: 0x200096e0
```

含义：

MCU 当前运行的旧程序已经进入 HardFault。对于烧录来说，这不一定是失败；只要 OpenOCD 能 halt CPU、识别 Flash，就仍然可以写入新固件。

本次成功日志里也出现了 `Handler HardFault`，但后面继续完成了：

```text
wrote 262144 bytes from file build/zephyr/zephyr.bin
shutdown command invoked
```

### 4.5 `falling back to single memory accesses`

典型输出：

```text
Warn : no working area available, can't do block memory writes
Warn : couldn't use block writes, falling back to single memory accesses
```

含义：

这不是失败。它表示 OpenOCD 不使用 SRAM 工作区做高速块写入，而是用很慢的单次访问写 Flash。

处理：

- 如果使用 `openocd_flash_bin_slow.sh`，看到它以后继续等待。
- 如果想更快，优先改用：

```bash
./scripts/openocd_flash_fast_4pin.sh
```

## 5. 为什么第一次很快，后面变慢

第一次快，是因为烧录器成功使用了 SRAM 中的高速 Flash loader。

后面失败后，MCU 可能停在 HardFault，ST-LINK 或 USB 通信也可能进入异常状态。没有 `NRST` 时，烧录器不能硬件复位 MCU，只能靠软件 halt 或热连接，所以高速 loader 更容易失败。

完全慢速模式之所以可靠，是因为它不用 SRAM block write，但代价是速度很慢。`openocd_flash_fast_4pin.sh` 是折中方案：仍使用一个较小 SRAM 工作区，所以比完全慢速模式快，同时比 `st-flash` 的高速写入更保守。

## 6. 快速决策表

| 现象 | 意义 | 下一步 |
| --- | --- | --- |
| `chipid: 0x413` | STM32F407 识别正常 | 可以烧录 |
| `chipid: 0x000` | 没连上目标 MCU | 拔插 ST-LINK，C 板断电重上电，检查 SWD |
| `LIBUSB_ERROR_TIMEOUT` | USB/ST-LINK/SWD 通信超时 | 换 USB 线/口，降速，断开电机动力电 |
| `Flash loader write error` | SRAM loader 运行失败 | 断电恢复，改用 OpenOCD 4pin 快速脚本 |
| `Handler HardFault` | 旧程序崩溃或 CPU 在异常态 | 只要 Flash 能识别，可以继续烧录 |
| `falling back to single memory accesses` | 进入超慢写入模式 | 等待，不要中断 |
| `wrote ... bytes ... shutdown` | 写入完成 | 断电重上电验证固件 |
