# 烧录脚本测试报告

测试环境：ST-LINK V2 (SWD 4线)、STM32F407IGH6、Linux

## 测试结果汇总

| # | 脚本 | 结果 | 耗时 | 说明 |
|---|------|------|------|------|
| 1 | `flash.sh` | ✅ 成功 | ~13s | west flash，含验证和自动复位 |
| 2 | `stlink_flash.sh` (normal) | ❌ 失败 | - | USB 写入超时 |
| 3 | `stlink_flash.sh` (hotplug) | ❌ 失败 | - | 无法进入 SWD 模式 |
| 4 | `stlink_flash.sh` (under-reset) | ❌ 失败 | - | NRST 引脚未连接 |
| 5 | `openocd_flash.sh` | ❌ 失败 | - | Flash write 算法超时 (workarea 不足) |
| 6 | `openocd_flash_bin_slow.sh` | ❌ 失败 | - | 脚本有参数 bug + 慢速写入被 SIGKILL |
| 7 | `openocd_flash_fast_4pin.sh` | ✅ 成功 | ~45s | 需手动断电重启，无验证 |
| 8 | `openocd_mass_erase_flash.sh` | ✅ 成功 | ~6s | 全片擦除后写入，有验证和复位 |
| 9 | `openocd_probe_flash.sh` | ✅ 探测 | ~2s | 仅探测芯片信息，不烧录 |

## 推荐

**首选：`./scripts/flash.sh`** — 最快(13s)、有验证、自动复位。

**备选：`./scripts/openocd_mass_erase_flash.sh`** — 需要全片擦除时使用。

**不推荐 stlink_flash.sh 系列** — 当前 ST-LINK V2 固件在 USB 批量传输阶段会超时，正常/hotplug/under-reset 三种模式全部失败。

## 各脚本详细测试记录

### 1. flash.sh ✅

```
$ ./scripts/flash.sh
Zephyr environment ready.
-- west flash: using runner openocd
adapter speed: 500 kHz
Info : STLINK V2J37S7 (API v2) VID:PID 0483:3748
Info : [stm32f4x.cpu] Cortex-M4 r0p1 processor detected
auto erase enabled
wrote 262144 bytes ... in 13.010781s (19.676 KiB/s)
verified 144687 bytes in 0.491545s (287.453 KiB/s)
shutdown command invoked
```

### 2. stlink_flash.sh (normal) ❌

```
$ ./scripts/stlink_flash.sh
Flashing with st-flash mode=normal, freq=100 kHz
EraseFlash ... success (6 sectors)
INFO flash_loader.c: Successfully loaded flash loader in sram
ERROR usb.c: WRITEMEM_32BIT send request failed: LIBUSB_ERROR_TIMEOUT
ERROR flash_loader.c: write_buffer_to_sram() == -1
ERROR flash_loader.c: stlink_flash_loader_run(0x8000000) failed! == -1
st-flash failed.
```

### 3. stlink_flash.sh (hotplug) ❌

```
$ STLINK_MODE=hotplug ./scripts/stlink_flash.sh
Flashing with st-flash mode=hotplug, freq=100 kHz
ERROR usb.c: GET_VERSION read reply failed: LIBUSB_ERROR_TIMEOUT
Failed to enter SWD mode
Failed to connect to target
```

### 4. stlink_flash.sh (under-reset) ❌

```
$ STLINK_MODE=under-reset ./scripts/stlink_flash.sh
WARN common.c: NRST is not connected
Flashing with st-flash mode=under-reset, freq=100 kHz
...erase OK...
ERROR usb.c: WRITEMEM_32BIT send request failed: LIBUSB_ERROR_TIMEOUT
ERROR flash_loader.c: stlink_flash_loader_run(0x8000000) failed! == -1
```

### 5. openocd_flash.sh ❌

```
$ ./scripts/openocd_flash.sh
adapter speed: 500 kHz
Info : STLINK V2J37S7 (API v2) VID:PID 0483:3748
Info : [stm32f4x.cpu] Cortex-M4 r0p1 processor detected
Error: timeout waiting for algorithm, a target reset is recommended
Error: error executing stm32x flash write algorithm
Error: flash write failed = 0x00000040
```

原因：`openocd.cfg` 没有配置 workarea，导致 flash write 算法在 SRAM 中无法运行。

### 6. openocd_flash_bin_slow.sh ❌

```
$ ./scripts/openocd_flash_bin_slow.sh
Slow OpenOCD flashing mode.
SWD speed: 50 kHz
Unexpected command line argument: verify_image build/zephyr/zephyr.bin 0x08000000 bin
```

两个问题：
1. 脚本构造参数时数组切片逻辑有 bug，verify_image 参数拼接到上一行末尾
2. 即使修复 bug 手动执行，50kHz 慢速模式下写入 144KB 耗时极长，最终被 SIGKILL

### 7. openocd_flash_fast_4pin.sh ✅

```
$ ./scripts/openocd_flash_fast_4pin.sh
Fast 4-pin SWD OpenOCD flashing mode.
SWD speed: 100 kHz
Work area: 0x1000
hla_swd
Info : STLINK V2J37S7 (API v2) VID:PID 0483:3748
Info : [stm32f4x.cpu] Cortex-M4 r0p1 processor detected
auto erase enabled
wrote 262144 bytes from file build/zephyr/zephyr.bin in 45.247734s (5.658 KiB/s)
shutdown command invoked
```

注意：烧录后需要手动断电重启（或按复位键），脚本不自动复位。

### 8. openocd_mass_erase_flash.sh ✅

```
$ ./scripts/openocd_mass_erase_flash.sh
adapter speed: 500 kHz
Info : STLINK V2J37S7 (API v2) VID:PID 0483:3748
stm32x mass erase complete
wrote 144748 bytes from file build/zephyr/zephyr.hex in 4.827632s (29.280 KiB/s)
verified 144687 bytes in 1.604426s (88.066 KiB/s)
shutdown command invoked
```

### 9. openocd_probe_flash.sh ✅ (仅探测)

```
$ ./scripts/openocd_probe_flash.sh
adapter speed: 500 kHz
Info : STLINK V2J37S7 (API v2) VID:PID 0483:3748
Info : Target voltage: 3.233400
Info : [stm32f4x.cpu] Cortex-M4 r0p1 processor detected
device id = 0x101f6413
flash size = 1024 kbytes
flash 'stm32f2x' found at 0x08000000
#0 : stm32f2x at 0x08000000, size 0x00100000 ... not protected
STM32F4xx - Rev: unknown (0x101f)
```

## 根因分析

| 失败原因 | 影响脚本 |
|----------|---------|
| `st-flash` USB 批量传输超时 | stlink_flash.sh (全部三种模式) |
| `openocd.cfg` 没配置 workarea | openocd_flash.sh |
| 脚本参数数组拼接 bug | openocd_flash_bin_slow.sh |
| 单字节写入速度太慢被 OOM | openocd_flash_bin_slow.sh (手动) |

## 快速参考

```bash
# 编译 + 烧录 (推荐)
./scripts/build.sh && ./scripts/flash.sh

# 仅烧录 (已编译过)
./scripts/flash.sh

# 串口连接
./scripts/serial.sh
```
