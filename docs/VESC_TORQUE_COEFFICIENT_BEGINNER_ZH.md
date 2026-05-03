# VESC 电流反馈和轮端力矩系数入门说明

本文解释当前固件如何使用 VESC 上位机已经校准好的数据，把 M3508 的电流反馈换算成轮端力矩估计。

适用硬件：

- 主控：DJI F407 C 板
- 轮电机：DJI M3508
- 轮电调：VESC
- CAN 总线：CAN2，1 Mbps
- 左 VESC ID：101
- 右 VESC ID：100

## 1. 现在是否已经有电流到力矩系数？

有。

当前固件里已经有左右轮各自的 `Nm/mA` 系数：

```c
#define APP_ASCENTO_LEFT_CURRENT_MA_TO_WHEEL_TORQUE_NM ...
#define APP_ASCENTO_RIGHT_CURRENT_MA_TO_WHEEL_TORQUE_NM ...
```

它们在 `src/app_config.h` 中定义。默认情况下，左右轮先使用同一个理论计算值：

```text
torque_k = Kt * reduction_ratio * gearbox_efficiency * 0.001
```

当前默认参数是：

```c
#define APP_M3508_MOTOR_KT_NM_PER_A 0.180f
#define APP_M3508_REDUCTION_RATIO 19.203208f
#define APP_M3508_GEARBOX_EFFICIENCY 1.000f
#define APP_VESC_CURRENT_CMD_TO_AMP 0.001f
```

所以：

```text
torque_k = 0.180 * 19.203208 * 1.000 * 0.001
         = 0.003456577 Nm/mA
```

意思是：如果 VESC 反馈电机电流为 `1000 mA`，固件会估算轮端力矩约为：

```text
1000 * 0.003456577 = 3.456577 Nm
```

这是理论值，不是外部拉力计实测值。它适合先用于调试和模型初值。

## 2. 为什么 VESC 上位机校准很重要？

VESC Tool 做 FOC 检测后，VESC 内部会知道如何正确控制这个电机，例如电流环、电机参数、传感器方向等。

对 F407 固件来说，最重要的是：

```text
VESC 回传的 motor_current 更可信了。
```

F407 不需要重新校准 VESC，它只需要通过 CAN 读取 VESC 发来的状态帧。

当前固件读取的主要 CAN 状态帧：

| VESC 状态帧 | CAN packet | 固件读取内容 |
| --- | --- | --- |
| Status 1 | `CAN_PACKET_STATUS = 9` | ERPM、电机电流、duty |
| Status 4 | `CAN_PACKET_STATUS_4 = 16` | 输入电流、FET 温度、电机温度 |
| Status 5 | `CAN_PACKET_STATUS_5 = 27` | 输入电压、tachometer |

其中最关键的是 Status 1 里的 `motor_current`。

## 3. 几个容易混淆的量

### 3.1 `cmd`

`cmd` 是 F407 最近一次发给 VESC 的电流命令。

例如执行：

```text
motor wheel current left 100 3000
```

意思是给左 VESC 发送约 `100 mA = 0.1 A` 的电流命令，持续 `3 s`。

状态里会看到类似：

```text
cmd=100 mA
```

`cmd` 是命令，不代表 VESC 实际做到了多少电流。

### 3.2 `motor_current`

`motor_current` 是 VESC Status 1 回传的实测电机电流。

这才是固件用来估算轮端力矩的电流。

例如：

```text
motor_current=285 mA
```

说明 VESC 实测电机电流约为 `0.285 A`。

### 3.3 `input`

`input` 是 VESC Status 4 回传的输入电流，也就是电池侧电流。

它和 `motor_current` 不是同一个东西。平衡控制和轮端力矩估算主要看 `motor_current`。

### 3.4 `torque_k`

`torque_k` 是当前这个轮子使用的电流到轮端力矩系数，单位是：

```text
Nm/mA
```

左轮和右轮可以不同。

### 3.5 `torque_est`

`torque_est` 是固件根据 VESC 实测电流估算出的轮端力矩：

```text
torque_est = motor_current * torque_k
```

例如：

```text
motor_current=285 mA
torque_k=0.003457 Nm/mA
```

则：

```text
torque_est = 285 * 0.003457 = 0.985 Nm
```

## 4. 串口状态输出怎么看？

烧录当前固件后，打开串口 shell，执行：

```text
motor wheel status all
```

正常输出类似：

```text
VESC/M3508 id=101 age=5ms erpm=0 motor_rpm=0 angle=0.000 rad speed=0.000 rad/s cmd=0 mA motor_current=0 mA input=0 mA vin=24.00 V temp=35.0/34.0C tach=0 torque_k=0.003457 torque_est=0.000 Nm duty=0.000 s4_age=8ms s5_age=10ms
VESC/M3508 id=100 age=5ms erpm=0 motor_rpm=0 angle=0.000 rad speed=0.000 rad/s cmd=0 mA motor_current=0 mA input=0 mA vin=24.00 V temp=35.0/34.0C tach=0 torque_k=0.003457 torque_est=0.000 Nm duty=0.000 s4_age=8ms s5_age=10ms
```

重点看：

| 字段 | 正常说明 |
| --- | --- |
| `age` | Status 1 更新时间，正常应小于 50 ms |
| `cmd` | F407 发送的电流命令 |
| `motor_current` | VESC 实测电机电流 |
| `vin` | VESC 输入电压 |
| `temp` | FET/电机温度 |
| `torque_k` | 当前轮子的力矩系数 |
| `torque_est` | 当前估算轮端力矩 |
| `s4_age` | Status 4 更新时间 |
| `s5_age` | Status 5 更新时间 |

如果 `s4_age=-1ms`，说明还没收到 Status 4。  
如果 `s5_age=-1ms`，说明还没收到 Status 5。

需要在 VESC Tool 里开启 Status 4 和 Status 5 的 CAN 回传。

## 5. 测试步骤

先确保轮子架空，然后执行：

```text
robot enable 0
robot stop
motor debug stop
motor wheel stop
motor wheel status all
```

如果左右 VESC 都有反馈，再测试左轮：

```text
motor wheel current left 100 3000
motor wheel status left
motor wheel stop
```

再测试右轮：

```text
motor wheel current right 100 3000
motor wheel status right
motor wheel stop
```

你应该能看到：

- `cmd` 接近刚刚发送的命令。
- `motor_current` 有变化。
- `torque_est` 随 `motor_current` 变化。
- `duty` 有变化。
- `age` 没有持续变大。

## 6. 两个 VESC 数据不同怎么办？

这是正常的。两个 VESC、电机、减速箱、轮胎都可能有差异。

当前代码允许左右轮分别配置力矩系数：

```c
#define APP_ASCENTO_LEFT_CURRENT_MA_TO_WHEEL_TORQUE_NM \
	APP_M3508_CALC_CURRENT_MA_TO_WHEEL_TORQUE_NM

#define APP_ASCENTO_RIGHT_CURRENT_MA_TO_WHEEL_TORQUE_NM \
	APP_M3508_CALC_CURRENT_MA_TO_WHEEL_TORQUE_NM
```

如果你根据 VESC 上位机、实测数据或经验确认左右轮需要不同系数，可以改成：

```c
#define APP_ASCENTO_LEFT_CURRENT_MA_TO_WHEEL_TORQUE_NM 0.00340f
#define APP_ASCENTO_RIGHT_CURRENT_MA_TO_WHEEL_TORQUE_NM 0.00352f
```

改完后重新编译烧录：

```bash
./scripts/build.sh
./scripts/flash.sh
```

然后用：

```text
motor wheel status all
```

确认左右轮显示的 `torque_k` 已经不同。

## 7. 这次代码做了什么？

### 7.1 配置层

文件：

```text
src/app_config.h
```

新增了 M3508 理论力矩计算和左右轮单独系数。

### 7.2 VESC 驱动层

文件：

```text
src/dji_m3508.c
src/dji_m3508.h
```

新增读取：

- `motor_current`
- `input_current`
- `input_voltage`
- FET 温度
- 电机温度
- tachometer
- 左右轮各自 `torque_k`
- `torque_est`

### 7.3 Shell 调试层

文件：

```text
src/shell_commands.c
```

`motor wheel status` 现在会打印更多 VESC 数据，便于初学者确认 CAN 数据是否真的进入固件。

### 7.4 模型控制层

文件：

```text
src/ascento_balance.c
src/ascento_balance.h
```

模型控制器已经支持左右轮不同 `Nm/mA` 系数。后续模型输出轮端力矩时，左轮和右轮会分别换算成自己的 VESC 电流命令。

## 8. 当前方案的边界

当前固件通过 VESC CAN Status 帧读取实时数据，这是稳定、适合实机调试的方式。

但普通 Status 帧不包含完整的 FOC 检测配置。完整配置读取需要 VESC 的分包通信和 `COMM_GET_MCCONF`，与 VESC 固件版本强相关，当前没有加入。

所以当前结论是：

```text
VESC 上位机负责电机校准。
F407 通过 CAN 读取 VESC 实时状态。
固件用 motor_current 和 torque_k 估算轮端力矩。
左右轮 torque_k 可以分别配置。
```

这是现阶段最稳妥的接入方式。
