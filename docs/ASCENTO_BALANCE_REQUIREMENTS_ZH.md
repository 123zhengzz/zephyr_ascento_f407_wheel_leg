# Ascento 串联腿平衡行走控制：需要提供的物理数据

工程里已经加入了一个暂不启用的模型控制模块：

```text
src/ascento_balance.h
src/ascento_balance.c
```

它已加入编译，但默认不会参与控制：

```c
#define APP_USE_ASCENTO_BALANCE_CONTROLLER 0
#define APP_ASCENTO_PARAMS_CALIBRATED 0
```

也就是说，现在单电机调试和原来的 `robot` 控制不会被它影响。要真正让它接管平衡，必须先把下面数据测出来，再计算控制增益。

## 1. 为什么不能直接靠代码平衡

两轮轮腿机器人本质上是倒立摆。能否平衡，主要取决于：

1. 质心在哪里。
2. 整机多重。
3. 轮子半径和轮距。
4. 轮电机输出电流能产生多少轮端力矩。
5. 腿长变化时，质心高度和腿机构角度怎么变化。
6. IMU 安装方向和零点。
7. LQR/PID 增益是否按真实机械参数算过。

如果这些数据错了，代码能编译也不代表能站住。

## 2. 必须提供的数据清单

### 2.1 整机质量和质心

| 数据 | 符号 | 单位 | 说明 |
| --- | --- | --- | --- |
| 整机总质量 | `m_total` | kg | 带电池、C 板、电机、轮子、外壳的总质量 |
| 车体质量 | `m_body` | kg | 不含轮子、可动腿的主体质量，做模型时更好 |
| 单个轮子质量 | `m_wheel` | kg | M3508 + 轮胎 + 轮毂等 |
| 左腿质量 | `m_leg_l` | kg | 包含连杆和关节输出侧 |
| 右腿质量 | `m_leg_r` | kg | 同上 |
| 质心到轮轴高度 | `h_com` | m | 站立目标腿长时，整机质心高于轮轴多少 |
| 质心前后偏移 | `x_com` | m | 质心相对轮轴向前为正 |

最重要的是：

```text
整机总质量 m_total
站立时质心高度 h_com
质心前后偏移 x_com
```

没有这三个，平衡增益只能瞎猜。

### 2.2 转动惯量

| 数据 | 符号 | 单位 | 说明 |
| --- | --- | --- | --- |
| 车体俯仰转动惯量 | `I_pitch` | kg*m^2 | 绕轮轴左右方向的转动惯量 |
| 单个轮子转动惯量 | `I_wheel` | kg*m^2 | 绕轮轴转动 |
| 腿部等效转动惯量 | `I_leg` | kg*m^2 | 可先忽略，后续提高模型精度 |

最低要求：

```text
I_pitch
I_wheel
```

如果不会测，可以先用 CAD 质量属性估算；如果没有 CAD，则用实物摆动法估算。

### 2.3 轮子和底盘几何

| 数据 | 符号 | 单位 | 说明 |
| --- | --- | --- | --- |
| 轮半径 | `r_wheel` | m | 轮胎接地半径 |
| 左右轮中心距 | `track_width` | m | 两轮接地点左右距离 |
| M3508 减速比 | `gear_ratio` | - | 当前代码默认 `19.203208` |

对应当前代码：

```c
#define APP_ASCENTO_WHEEL_RADIUS_M 0.060f
#define APP_ASCENTO_WHEEL_BASE_M 0.250f
#define APP_M3508_REDUCTION_RATIO 19.203208f
```

这些默认值只是占位，必须按你的实物改。

### 2.4 轮电机力矩常数

需要知道：

| 数据 | 符号 | 单位 | 说明 |
| --- | --- | --- | --- |
| VESC 电流命令到轮端力矩 | `tau_per_current_cmd` | Nm/mA | 输入 `motor wheel current ... 1000`，也就是约 1A 时轮端约多少 Nm |
| 最大安全电流命令 | `current_safe` | mA | 初调建议很小 |

当前默认按 M3508 理论力矩常数、减速比和效率计算，左右轮也可以分别覆盖：

```c
#define APP_ASCENTO_LEFT_CURRENT_MA_TO_WHEEL_TORQUE_NM ...
#define APP_ASCENTO_RIGHT_CURRENT_MA_TO_WHEEL_TORQUE_NM ...
#define APP_WHEEL_CURRENT_SAFE 2500
```

这些值在当前工程里按 `Nm/mA` 理解，会直接影响平衡输出大小。太小会站不住，太大可能一启用就猛冲。

建议测法：

1. 架空轮子。
2. 给小电流命令，比如 `300`。
3. 看轮子加速度。
4. 或用测力臂/弹簧秤测轮端力矩。

### 2.5 腿机构几何

你的腿是 Ascento 风格串联/四连杆腿，至少要给下面数据：

| 数据 | 单位 | 说明 |
| --- | --- | --- |
| 每根连杆长度 | m | 四连杆所有杆长 |
| 固定铰点坐标 | m | 相对车体坐标系 |
| 电机输出角零点 | rad | 达妙反馈 `0 rad` 对应机械位置 |
| 电机角度正方向 | - | 正角度是伸腿还是收腿 |
| 腿长最小值 | m | 不顶死机械的最短腿长 |
| 腿长最大值 | m | 不顶死机械的最长腿长 |
| 关节角到腿长曲线 | `L(q)` | 用来把目标腿长转成电机角 |
| 腿长到关节角曲线 | `q(L)` | 控制时实际用这个 |

当前代码里先用线性占位：

```c
#define APP_ASCENTO_LEG_LENGTH_MIN_M 0.120f
#define APP_ASCENTO_LEG_LENGTH_MAX_M 0.360f
#define APP_ASCENTO_LEG_LENGTH_DEFAULT_M 0.220f

#define APP_ASCENTO_LEFT_JOINT_AT_MIN_LEG_RAD APP_LEFT_LEG_MIN_RAD
#define APP_ASCENTO_LEFT_JOINT_AT_MAX_LEG_RAD APP_LEFT_LEG_MAX_RAD
#define APP_ASCENTO_RIGHT_JOINT_AT_MIN_LEG_RAD APP_RIGHT_LEG_MAX_RAD
#define APP_ASCENTO_RIGHT_JOINT_AT_MAX_LEG_RAD APP_RIGHT_LEG_MIN_RAD
```

真实四连杆通常不是线性的，所以要实测或用几何推导替换。

### 2.6 传感器方向和零点

必须提供：

| 数据 | 说明 |
| --- | --- |
| IMU 安装方向 | X/Y/Z 轴对应车体前/左/上哪一轴 |
| pitch 正方向 | 车体前倾时 pitch 是正还是负 |
| roll 正方向 | 左倾/右倾符号 |
| gyro 正方向 | `gx/gy/gz` 和欧拉角方向是否一致 |
| 站立零点 | 机器人机械直立时 pitch 角 |

当前零点：

```c
#define APP_ANGLE_ZERO_DEG -2.25f
```

这个必须用实机标定。零点错 2 到 3 度，车就可能一直加速。

### 2.7 控制目标和安全范围

需要确定：

| 数据 | 单位 | 说明 |
| --- | --- | --- |
| 目标站立腿长 | m | 初调固定腿长 |
| 最大俯仰角 | deg | 超过就停轮 |
| 最大轮电流 | mA | 初调限流 |
| 最大腿速度 | rad/s | 防止腿突然冲到限位 |
| 最大行走速度 | m/s | 初期设很低 |

当前安全参数：

```c
#define APP_PITCH_FAULT_DEG 25.0f
#define APP_WHEEL_CURRENT_LIMIT 6500
#define APP_LEG_VEL_LIMIT_RAD_S 12.0f
```

初次落地建议把轮电流限制降到更保守。

## 3. 需要计算的控制增益

当前 dormant 控制器预留了这些增益：

```c
#define APP_ASCENTO_K_PITCH 0.0f
#define APP_ASCENTO_K_PITCH_RATE 0.0f
#define APP_ASCENTO_K_POSITION 0.0f
#define APP_ASCENTO_K_VELOCITY 0.0f
#define APP_ASCENTO_K_YAW_RATE 0.0f
#define APP_ASCENTO_K_ROLL_TO_LEG_M_PER_RAD 0.0f
```

实际要根据模型算出：

| 增益 | 用途 |
| --- | --- |
| `K_PITCH` | 车体俯仰角反馈 |
| `K_PITCH_RATE` | 俯仰角速度反馈 |
| `K_POSITION` | 轮子位移反馈，防止越跑越远 |
| `K_VELOCITY` | 前后速度反馈 |
| `K_YAW_RATE` | 转向角速度阻尼 |
| `K_ROLL_TO_LEG` | 左右腿长差补偿 roll |

更完整的 LQR 状态建议：

```text
x = [
  body_pitch_rad,
  body_pitch_rate_rad_s,
  wheel_position_m,
  wheel_velocity_m_s,
  leg_angle_rad,
  leg_angle_rate_rad_s
]
```

对应输出：

```text
u = [
  left_wheel_torque_Nm,
  right_wheel_torque_Nm,
  left_leg_target_or_torque,
  right_leg_target_or_torque
]
```

如果目前每条腿只有一个关节电机，腿控制先用“目标腿长/目标关节角”，轮子负责主平衡，是更稳妥的起步方案。

## 4. 数据填到哪里

先填：

```text
src/app_config.h
```

这些宏：

```c
APP_ASCENTO_WHEEL_RADIUS_M
APP_ASCENTO_WHEEL_BASE_M
APP_ASCENTO_TOTAL_MASS_KG
APP_ASCENTO_BODY_COM_HEIGHT_M
APP_ASCENTO_BODY_PITCH_INERTIA_KG_M2
APP_ASCENTO_WHEEL_INERTIA_KG_M2
APP_ASCENTO_LEFT_CURRENT_MA_TO_WHEEL_TORQUE_NM
APP_ASCENTO_RIGHT_CURRENT_MA_TO_WHEEL_TORQUE_NM
APP_ASCENTO_LEG_LENGTH_MIN_M
APP_ASCENTO_LEG_LENGTH_MAX_M
APP_ASCENTO_LEG_LENGTH_DEFAULT_M
APP_ASCENTO_LEFT_JOINT_AT_MIN_LEG_RAD
APP_ASCENTO_LEFT_JOINT_AT_MAX_LEG_RAD
APP_ASCENTO_RIGHT_JOINT_AT_MIN_LEG_RAD
APP_ASCENTO_RIGHT_JOINT_AT_MAX_LEG_RAD
APP_ASCENTO_K_PITCH
APP_ASCENTO_K_PITCH_RATE
APP_ASCENTO_K_POSITION
APP_ASCENTO_K_VELOCITY
APP_ASCENTO_K_YAW_RATE
APP_ASCENTO_K_ROLL_TO_LEG_M_PER_RAD
```

确认数据和增益合理后，最后才改：

```c
#define APP_ASCENTO_PARAMS_CALIBRATED 1
#define APP_USE_ASCENTO_BALANCE_CONTROLLER 1
```

现在它们保持 `0`，所以不会使用。

## 5. 最小可平衡数据表

你可以按这个表给我数据，我就能帮你把第一版可试站立参数填进去：

| 项目 | 你的数值 |
| --- | --- |
| 轮半径 m |  |
| 左右轮中心距 m |  |
| 整机总质量 kg |  |
| 站立时轮轴到质心高度 m |  |
| 质心相对轮轴前后偏移 m |  |
| 车体 pitch 转动惯量 kg*m^2 |  |
| 单个轮子转动惯量 kg*m^2 |  |
| M3508 减速比 |  |
| 左 VESC 电流命令到轮端力矩 Nm/mA |  |
| 右 VESC 电流命令到轮端力矩 Nm/mA |  |
| 最短腿长 m |  |
| 最长腿长 m |  |
| 默认站立腿长 m |  |
| 左腿最短腿长对应关节角 rad |  |
| 左腿最长腿长对应关节角 rad |  |
| 右腿最短腿长对应关节角 rad |  |
| 右腿最长腿长对应关节角 rad |  |
| IMU pitch 正方向 |  |
| 机械直立时 pitch 读数 deg |  |
| 正电流时左轮实际方向 |  |
| 正电流时右轮实际方向 |  |

## 6. 启用前测试顺序

不要直接启用新控制器。必须先完成：

1. `motor dm pos left/right` 单关节测试通过。
2. `motor wheel current left/right` 单轮测试通过。
3. `robot height 38/60` 腿高方向正确。
4. 架空时原有轮子输出方向正确。
5. 所有物理参数填完。
6. LQR/PID 增益先在仿真或 MATLAB/Python 里验证。
7. 把轮子架空，启用新控制器只看输出方向。
8. 限流落地，旁边准备断电。

## 7. 当前代码做了什么

`src/ascento_balance.c` 目前包含：

1. 物理参数结构体。
2. 关节角到腿长的临时线性映射。
3. 腿长到关节角的临时线性映射。
4. 轮子位移/速度估计。
5. pitch、pitch rate、位移、速度的 LQR 形式输出。
6. yaw rate 转向修正。
7. roll 到左右腿长差的补偿接口。
8. 电机力矩到 VESC 电流命令 mA 的换算。

但它现在没有接入主控制输出，不会影响当前调试。
