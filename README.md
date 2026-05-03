# DJI F407 C Board Ascento Wheel-Leg Robot

中文初学者调试手册见 [README_ZH.md](README_ZH.md)。

Quick start on this machine:

```bash
cd /home/h/code_leg/zephyr_ascento_f407_wheel_leg
./scripts/build.sh
./scripts/flash.sh
./scripts/serial.sh /dev/ttyUSB0
```

Default bus routing:

- CAN1: Damiao DM4340/DM43xx joint motors, IDs 1 and 2
- CAN2: DJI C620 + M3508 wheel motors, IDs 1 and 2

## DM4340 MIT Small Wiggle

达妙 DM4340 有位置速度模式和速度模式，帧 ID 分别是 `CAN ID + 0x100` 和 `CAN ID + 0x200`。本机调试只采用 MIT 模式，MIT 帧直接发到电机 CAN ID：左关节 `0x01`，右关节 `0x02`。

在串口 shell 中执行下面的命令，可以让关节电机围绕当前反馈位置做小幅 MIT 往复。单次调试时长会被固件限制在 5000 ms，测试结束后务必停止：

```text
# 先确认 CAN 和反馈正常
motor can status all
motor dm status all

# 右关节 MIT 小幅往复：振幅 0.02 rad，周期 2000 ms，kp=12，kd=0.5，持续 5000 ms
motor dm wiggle right 0.02 2000 12 0.5 5000
motor dm status right
motor dm stop right

# 左关节同样测试
motor dm wiggle left 0.02 2000 12 0.5 5000
motor dm status left
motor dm stop left
```

如果 0.02 rad 没有明显动作，可以在确认机械安全后试 `0.04 rad`；如果扭矩有变化但位置仍不变，优先检查达妙上位机模式、机械限位和电机使能状态。

如果右关节仍不动，按下面顺序排查：

```text
# 1. 确认右电机还在线，CAN 没有错误
motor can status all
motor dm status right

# 2. 稍微提高 MIT 小幅往复的刚度和幅度，仍然只在当前位置附近摆动
motor dm wiggle right 0.04 2000 20 0.8 5000
motor dm status right
motor dm stop right

# 3. torque-only MIT 小力矩测试：kp=0、kd=0，位置字段不起作用
motor dm mit right 0 0 0 0 0.20 800
motor dm status right
motor dm mit right 0 0 0 0 -0.20 800
motor dm status right
motor dm stop right
```

判断方法：如果 torque-only 时反馈扭矩有变化但位置不变，先看电源电流是否同步变化；如果空载电机没有转动且电源电流完全不变，反馈扭矩不能当作真实相电流证据，应优先排查驱动器功率级、相线/电机本体、保护/锁定状态或达妙驱动器内部输出配置。如果反馈扭矩也没有变化，多半是达妙上位机里控制模式/使能状态不接受 MIT 指令。

当前固件还支持直接读取达妙寄存器，用于确认 ID、模式和 MIT 映射范围：

```text
# 右关节：控制模式、反馈 ID、接收 ID
motor dm reg right 0x0a   # CTRL_MODE: 1=MIT, 2=位置速度, 3=速度, 4=力位混控
motor dm reg right 0x07   # MST_ID / 反馈 ID，右关节应为 0x12
motor dm reg right 0x08   # ESC_ID / 接收 ID，右关节应为 0x02

# MIT 量化范围，必须和 src/dm4340.c 里的 limit 一致
motor dm reg right 0x15   # PMAX，本机实测 12.5
motor dm reg right 0x16   # VMAX，本机实测 10.0
motor dm reg right 0x17   # TMAX，本机实测 28.0

# 供电和限流
motor dm reg right 0x3c   # VBus
motor dm reg right 0x3b   # Imax
motor dm reg right 0x03   # OC_Value
```

本机 2026-05-03 实测两台 4340 的 MIT 量程为 `PMAX=12.5`、`VMAX=10.0`、`TMAX=28.0`，固件已按这组值打包和解析 MIT 帧。`motor dm stop` 和调试命令超时后都会主动补发 MIT 零力矩。

新增一键诊断命令，一次性读取所有关键寄存器：

```text
motor dm diag left       # 左关节全寄存器诊断
motor dm diag right      # 右关节全寄存器诊断
```

读取的寄存器包括：FAULT(0x01), WARNING(0x02), STATUS(0x04), CAN_ERR(0x05), MOTOR_ERR(0x06), TIMEOUT(0x09), CTRL_MODE(0x0a), PMAX(0x15), VMAX(0x16), TMAX(0x17), Imax(0x3b), VBus(0x3c)。

`motor dm reg` 也扩展支持 FAULT/STATUS/CAN_ERR/MOTOR_ERR 等诊断寄存器。

电机不动的深层排查过程详见 [docs/deepseek_4340_debug.md](docs/deepseek_4340_debug.md) 第 10 章。

Motor wiring and smoke-test commands are documented in:

- `README_ZH.md`
- `docs/MOTOR_DEBUG_README_ZH.md`
- `docs/ASCENTO_BALANCE_REQUIREMENTS_ZH.md`

## Build & Flash

```bash
# 编译
./scripts/build.sh

# 烧录 (推荐，13s，含验证和自动复位)
./scripts/flash.sh

# 备选：全片擦除后烧录 (6s)
./scripts/openocd_mass_erase_flash.sh

# 备选：4线 SWD 烧录 (45s，需要手动复位)
./scripts/openocd_flash_fast_4pin.sh

# 串口连接
./scripts/serial.sh
```

不可用的烧录脚本已禁用，详见 [docs/FLASH_SCRIPT_TEST_REPORT.md](docs/FLASH_SCRIPT_TEST_REPORT.md).
