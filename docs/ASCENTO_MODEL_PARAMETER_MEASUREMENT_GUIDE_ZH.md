# Ascento 控制模型参数填写和测量指南

本文说明当前已经写入控制模型的参数、还缺哪些参数，以及每个缺失参数怎么测。

适用文件：

- 参数配置：`src/app_config.h`
- 模型参数结构：`src/ascento_balance.h`
- 模型参数默认值：`src/ascento_balance.c`
- 当前传统 PID 控制：`src/control.c`

注意：模型控制器目前仍是禁用状态：

```c
#define APP_USE_ASCENTO_BALANCE_CONTROLLER 0
#define APP_ASCENTO_PARAMS_CALIBRATED 0
```

这很重要。参数没测完前，不要把它们改成 `1`。

## 1. 已经写入模型的参数

当前 `src/ascento_balance.c` 里的 `ascento_balance_default_params` 已经从 `src/app_config.h` 读取这些参数：

| 参数 | 当前来源 | 状态 |
| --- | --- | --- |
| `wheel_radius_m` | `APP_ASCENTO_WHEEL_RADIUS_M` | 需要按实物确认 |
| `wheel_base_m` | `APP_ASCENTO_WHEEL_BASE_M` | 需要按实物确认 |
| `total_mass_kg` | `APP_ASCENTO_TOTAL_MASS_KG` | 需要实测 |
| `body_com_height_m` | `APP_ASCENTO_BODY_COM_HEIGHT_M` | 需要实测 |
| `body_pitch_inertia_kg_m2` | `APP_ASCENTO_BODY_PITCH_INERTIA_KG_M2` | 需要估算或实测 |
| `wheel_inertia_kg_m2` | `APP_ASCENTO_WHEEL_INERTIA_KG_M2` | 需要估算或实测 |
| 左轮 `Nm/mA` | `APP_ASCENTO_LEFT_CURRENT_MA_TO_WHEEL_TORQUE_NM` | 已有理论计算值 |
| 右轮 `Nm/mA` | `APP_ASCENTO_RIGHT_CURRENT_MA_TO_WHEEL_TORQUE_NM` | 已有理论计算值 |
| 腿长范围 | `APP_ASCENTO_LEG_LENGTH_*` | 需要实测 |
| 关节角到腿长映射端点 | `APP_ASCENTO_*_JOINT_AT_*_LEG_RAD` | 需要实测 |
| LQR/PID 模型增益 | `APP_ASCENTO_K_*` | 还未计算，当前为 0 |

目前最可靠的是轮电机相关参数：

```c
#define APP_M3508_REDUCTION_RATIO 19.203208f
#define APP_M3508_MOTOR_KT_NM_PER_A 0.180f
#define APP_M3508_GEARBOX_EFFICIENCY 1.000f
#define APP_ASCENTO_LEFT_CURRENT_MA_TO_WHEEL_TORQUE_NM \
	APP_M3508_CALC_CURRENT_MA_TO_WHEEL_TORQUE_NM
#define APP_ASCENTO_RIGHT_CURRENT_MA_TO_WHEEL_TORQUE_NM \
	APP_M3508_CALC_CURRENT_MA_TO_WHEEL_TORQUE_NM
```

默认计算结果约为：

```text
0.003456577 Nm/mA
```

模型输出轮端力矩时，左轮和右轮会分别用自己的 `Nm/mA` 系数换算成 VESC 电流命令。

## 2. 还需要测哪些参数？

最低限度还需要这些：

| 优先级 | 参数 | 单位 | 为什么需要 |
| --- | --- | --- | --- |
| 1 | 轮半径 `r_wheel` | m | 把轮角速度换成车体速度 |       30mm
| 1 | 左右轮距 `track_width` | m | 转向和 yaw 力矩分配 |   201mm
| 1 | 整机质量 `m_total` | kg | 倒立摆模型主质量 |      2315g
| 1 | 质心高度 `h_com` | m | 平衡增益最敏感参数 |       98mm
| 1 | 质心前后偏移 `x_com` | m | 决定直立零点和静态偏置 | 向后偏移40mm
| 1 | IMU pitch 零点 | deg | 零点错会导致机器人一直加速 | 
| 1 | 轮子正电流方向 | - | 符号错会越控越倒 |
| 2 | 腿长最小/最大值 | m | 防止关节撞限位 |            60.49mm 176.81mm
| 2 | 关节角 `q` 到腿长 `L` 的关系 | rad/m | 目标腿长转达妙关节角 | 
| 2 | 车体 pitch 转动惯量 `I_pitch` | kg*m^2 | 用于 LQR/仿真 |
| 2 | 轮子转动惯量 `I_wheel` | kg*m^2 | 用于更准确模型 |
| 3 | 左右轮各自力矩系数修正 | Nm/mA | 补偿两个 VESC/电机差异 |
| 3 | 腿部质量和惯量 | kg, kg*m^2 | 高精度多体模型需要 |

## 3. 参数怎么测？

### 3.1 轮半径 `APP_ASCENTO_WHEEL_RADIUS_M`

最简单测法：

1. 机器人正常装轮，放在地面。
2. 测量轮轴中心到地面的垂直距离。
3. 单位换成 m。

如果轮胎比较软，用机器人实际重量压在地上测，不要悬空测轮胎外径。

填入：

```c
#define APP_ASCENTO_WHEEL_RADIUS_M ...
```

### 3.2 左右轮距 `APP_ASCENTO_WHEEL_BASE_M`

这里的 wheel base 在当前模型里按左右轮中心距理解。

测法：

1. 从左轮接地点中心量到右轮接地点中心。
2. 或从左轮轴中心量到右轮轴中心。
3. 单位 m。

填入：

```c
#define APP_ASCENTO_WHEEL_BASE_M ...
```

### 3.3 整机质量 `APP_ASCENTO_TOTAL_MASS_KG`

测法：

1. 装上电池、控制板、外壳、所有电机和连杆。
2. 使用电子秤称整机。
3. 单位 kg。

填入：

```c
#define APP_ASCENTO_TOTAL_MASS_KG ...
```

### 3.4 质心高度 `APP_ASCENTO_BODY_COM_HEIGHT_M`

推荐用“平衡边法”，不需要 CAD。

步骤：

1. 把机器人断电，固定腿长到目标站立高度。
2. 拆掉容易晃动的线，保持电池在真实安装位置。
3. 找一根圆杆、桌边或窄木条作为支点。
4. 让机器人侧面放在支点上，慢慢移动，找到刚好能平衡的位置。
5. 这个位置就是整机质心在侧视图中的投影。
6. 测量这个点相对轮轴的高度，单位 m。

填入：

```c
#define APP_ASCENTO_BODY_COM_HEIGHT_M ...
```

注意：如果腿长会变化，质心高度也会变化。初期先只测默认站立腿长。

### 3.5 质心前后偏移 `x_com`

同样用平衡边法。

步骤：

1. 保持默认站立腿长。
2. 从侧面找质心投影。
3. 测量质心投影相对轮轴的前后距离。
4. 约定向前为正，向后为负。

这个值暂时没有单独宏，但它会影响 `APP_ANGLE_ZERO_DEG` 和模型静态平衡点。初期可以通过机械直立时的 pitch 零点补偿。

### 3.6 IMU pitch 零点 `APP_ANGLE_ZERO_DEG`

步骤：

1. 机器人断电或轮子架空，机械上扶到直立。
2. 打开串口。
3. 执行：

```text
robot status
```

4. 记录机械直立时 `pitch` 的读数。
5. 多读几次取平均。

填入：

```c
#define APP_ANGLE_ZERO_DEG ...
```

如果机械直立时显示 `pitch=-2.25`，就填：

```c
#define APP_ANGLE_ZERO_DEG -2.25f
```

### 3.7 轮子正电流方向

步骤：

1. 轮子架空。
2. 停止所有控制：

```text
robot enable 0
robot stop
motor debug stop
motor wheel stop
```

3. 左轮测试：

```text
motor wheel current left 300 500
motor wheel status left
motor wheel stop
```

4. 右轮测试：

```text
motor wheel current right 300 500
motor wheel status right
motor wheel stop
```

记录：

| 电机 | 正电流时方向 |
| --- | --- |
| 左轮 | 前进 / 后退 |
| 右轮 | 前进 / 后退 |

如果符号错，控制器可能会越控越倒。先记录，后面统一改控制输出符号。

### 3.8 腿长范围

目标是得到：

```c
#define APP_ASCENTO_LEG_LENGTH_MIN_M ...
#define APP_ASCENTO_LEG_LENGTH_MAX_M ...
#define APP_ASCENTO_LEG_LENGTH_DEFAULT_M ...
```

测法：

1. 关节断电或使用很小速度移动。
2. 找到机械不干涉、不顶死的最短腿长。
3. 测量轮轴到车体参考点，或足端到车体固定参考点的距离。
4. 找到最长安全腿长，同样测量。
5. 默认站立腿长取中间偏稳定的位置。

建议不要把物理极限当软件极限，至少留 5-10 mm 安全余量。

### 3.9 关节角 `q` 到腿长 `L` 的关系

当前代码还是线性映射，只适合临时占位。四连杆真实关系通常不是线性的。

推荐实测表格法：

1. 让达妙反馈在线：

```text
motor dm status all
```

2. 在安全范围内选 10-15 个关节角。
3. 每个位置记录：

```text
left_q_rad, left_leg_length_m
right_q_rad, right_leg_length_m
```

4. 做成表格。

表格模板：

| 点 | 左关节 q(rad) | 左腿长 L(m) | 右关节 q(rad) | 右腿长 L(m) |
| --- | --- | --- | --- | --- |
| 1 |  |  |  |  |
| 2 |  |  |  |  |
| 3 |  |  |  |  |
| 4 |  |  |  |  |
| 5 |  |  |  |  |

最少先填四个端点：

```c
#define APP_ASCENTO_LEFT_JOINT_AT_MIN_LEG_RAD ...
#define APP_ASCENTO_LEFT_JOINT_AT_MAX_LEG_RAD ...
#define APP_ASCENTO_RIGHT_JOINT_AT_MIN_LEG_RAD ...
#define APP_ASCENTO_RIGHT_JOINT_AT_MAX_LEG_RAD ...
```

之后再把线性映射替换为查表插值或四连杆解析反解。

### 3.10 车体 pitch 转动惯量 `APP_ASCENTO_BODY_PITCH_INERTIA_KG_M2`

初学者优先用 CAD 估算。

如果没有 CAD，可以先用粗略盒体估算：

```text
I_pitch ≈ m * (height^2 + length^2) / 12
```

其中：

- `m` 是车体质量，不含轮子时更好。
- `height` 是车体高度。
- `length` 是车体前后长度。

填入：

```c
#define APP_ASCENTO_BODY_PITCH_INERTIA_KG_M2 ...
```

这个值先用于仿真和 LQR 初值，不建议一开始追求很精确。

### 3.11 轮子转动惯量 `APP_ASCENTO_WHEEL_INERTIA_KG_M2`

粗略估算：

```text
I_wheel ≈ 0.5 * m_wheel * r_wheel^2
```

其中：

- `m_wheel` 是单个轮子总质量，包括 M3508、轮胎、轮毂。
- `r_wheel` 是轮半径。

填入：

```c
#define APP_ASCENTO_WHEEL_INERTIA_KG_M2 ...
```

### 3.12 左右轮力矩系数修正

当前已有理论值：

```text
torque_k ≈ 0.003456577 Nm/mA
```

如果只用计算值，暂时不需要再测。

如果你发现左右轮在同样电流下力矩差异明显，可以分别修正：

```c
#define APP_ASCENTO_LEFT_CURRENT_MA_TO_WHEEL_TORQUE_NM ...
#define APP_ASCENTO_RIGHT_CURRENT_MA_TO_WHEEL_TORQUE_NM ...
```

最可靠方法仍然是力臂法：

```text
tau = F * r
torque_k = tau / motor_current_mA
```

这里建议用 `motor_current_mA`，不是 `cmd`，因为 VESC 已校准后的实测电流更接近真实电机电流。

## 4. 填完参数后还缺什么？

填完物理参数后，还缺控制增益：

```c
#define APP_ASCENTO_K_PITCH 0.0f
#define APP_ASCENTO_K_PITCH_RATE 0.0f
#define APP_ASCENTO_K_POSITION 0.0f
#define APP_ASCENTO_K_VELOCITY 0.0f
#define APP_ASCENTO_K_YAW_RATE 0.0f
#define APP_ASCENTO_K_ROLL_TO_LEG_M_PER_RAD 0.0f
```

这些不能靠量尺直接得到，需要通过仿真或逐步调参得到。

推荐顺序：

1. 固定腿长，先只做两轮倒立摆仿真。
2. 根据 `m_total`、`h_com`、`r_wheel`、`torque_k` 算第一版 LQR/PID。
3. 在仿真里加电流限幅和延迟。
4. 轮子架空验证输出方向。
5. 手扶落地，小电流限幅测试扶正趋势。
6. 再逐步增加增益。

## 5. 最小数据表

你可以按下面格式把数据发回来，我就能帮你把 `src/app_config.h` 里的模型参数填成第一版：

| 参数 | 数值 |
| --- | --- |
| 轮半径 m |  |
| 左右轮中心距 m |  |
| 整机质量 kg |  |
| 默认腿长时质心高度 m |  |
| 默认腿长时质心前后偏移 m |  |
| 机械直立 pitch 读数 deg |  |
| 左轮正电流方向 |  |
| 右轮正电流方向 |  |
| 最短安全腿长 m |  |
| 最长安全腿长 m |  |
| 默认站立腿长 m |  |
| 左腿最短腿长关节角 rad |  |
| 左腿最长腿长关节角 rad |  |
| 右腿最短腿长关节角 rad |  |
| 右腿最长腿长关节角 rad |  |
| 车体 pitch 转动惯量 kg*m^2 |  |
| 单个轮子转动惯量 kg*m^2 |  |
| 左轮 torque_k Nm/mA，可选 |  |
| 右轮 torque_k Nm/mA，可选 |  |

## 6. 当前结论

现在已经写入模型的可靠参数是：

- VESC 左右 ID。
- M3508 减速比。
- M3508 理论 `Kt`。
- VESC 电流命令单位换算。
- 左右轮各自 `Nm/mA` 系数。

还必须实测后再相信的参数是：

- 轮半径。
- 左右轮距。
- 整机质量。
- 质心高度和前后偏移。
- 四连杆腿长曲线。
- IMU 零点和方向。
- 转动惯量。
- 控制增益。

这些完成前，模型控制器应继续保持禁用。

