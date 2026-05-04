# Ascento 控制模型总 README

本文把 Ascento 轮腿控制模型相关的说明合并到一处：当前已经写入哪些参数、还缺哪些数据、如何测量、需要做哪些仿真，以及什么时候才能把模型控制真正接入实机。

适用硬件：

- 主控：DJI F407 C 板
- 关节电机：达妙 DM4340，CAN1，左 ID 1，右 ID 2
- 轮电机：DJI M3508 + VESC，CAN2，左 VESC ID 101，右 VESC ID 100

相关代码：

- 参数配置：[src/app_config.h](/home/h/code_leg/zephyr_ascento_f407_wheel_leg/src/app_config.h)
- Ascento 模型结构：[src/ascento_balance.h](/home/h/code_leg/zephyr_ascento_f407_wheel_leg/src/ascento_balance.h)
- Ascento 模型实现：[src/ascento_balance.c](/home/h/code_leg/zephyr_ascento_f407_wheel_leg/src/ascento_balance.c)
- 当前主控制循环：[src/control.c](/home/h/code_leg/zephyr_ascento_f407_wheel_leg/src/control.c)

## 0. 当前结论

模型控制器现在仍然是禁用状态：

```c
#define APP_USE_ASCENTO_BALANCE_CONTROLLER 0
#define APP_ASCENTO_PARAMS_CALIBRATED 0
```

不要现在把它们改成 `1`。原因很直接：物理参数已有一部分，但四连杆腿部运动学、转动惯量和控制增益还没有完成。现在可以继续做电机、IMU、VESC 和参数测量；不能直接导入运动模型让机器人落地自平衡。

## 1. 已经写入代码的参数

这些值已经写在 [src/app_config.h](/home/h/code_leg/zephyr_ascento_f407_wheel_leg/src/app_config.h)：

| 参数 | 当前值 | 状态 |
| --- | --- | --- |
| 左轮 VESC ID | `101` | 已确认 |
| 右轮 VESC ID | `100` | 已确认 |
| 左轮正电流方向 | `APP_WHEEL_LEFT_FORWARD_CURRENT_SIGN 1` | 已确认，正电流为前进方向 |
| 右轮正电流方向 | `APP_WHEEL_RIGHT_FORWARD_CURRENT_SIGN -1` | 已确认，正电流为后退方向，控制输出会取反 |
| 方向调试电流 | `100 mA / 3000 ms` | 已按你的要求写入 |
| M3508 减速比 | `19.203208` | 理论值 |
| M3508 理论力矩常数 | `0.180 Nm/A` | 理论值 |
| VESC 电流命令单位 | `0.001 A/mA` | 已写入 |
| 轮端力矩系数 | 约 `0.003456577 Nm/mA` | 理论计算值，左右可单独修正 |
| 轮半径 | `0.030 m` | 你已提供 |
| 左右轮距 | `0.201 m` | 你已提供 |
| 整机质量 | `2.315 kg` | 你已提供 |
| 质心高度 | `0.098 m` | 你已提供 |
| 质心前后偏移 | `-0.040 m` | 你已提供，向前为正，所以向后为负 |
| IMU pitch 零点 | `-1.225 deg` | 已实测写入 |
| 最短腿长 | `0.06049 m` | 你已提供 |
| 最长腿长 | `0.17681 m` | 你已提供 |
| 默认腿长 | `0.11865 m` | 当前取最短和最长中点 |
| 车体 pitch 转动惯量 | `0.060 kg*m^2` | 暂时估算，占位待验证 |
| 单个轮子转动惯量 | `0.00035 kg*m^2` | 暂时估算，占位待验证 |
| Ascento 控制增益 | 全部 `0.0` | 尚未计算，模型不能启用 |

当前最危险的占位是三类：

- `q <-> L` 腿长映射仍是线性占位，真实四连杆通常不是线性。
- `APP_ASCENTO_K_*` 控制增益全是 0，模型还不会产生有效平衡控制。
- 模型代码中**没有任何 IMU 符号修正**，直接使用原始 `pitch_deg`/`gy_dps`/`gz_dps`/`roll_deg`。如果任何一个通道符号与模型假设相反，会导致正反馈失控。

## 2. 现在还需要哪些数据

按优先级看，当前还缺这些。

| 优先级 | 数据 | 作用 | 当前状态 |
| --- | --- | --- | --- |
| **0** | 模型级 IMU 轴方向与符号修正 | 模型直接读 `pitch_deg`/`gy_dps`/`gz_dps`/`roll_deg`，**没有任何符号修正**，符号反了会直接失控 | **未确认，见第 5.4 节** |
| 1 | 左右达妙关节角正方向 | 判断正角度是伸腿还是收腿 | 未完成 |
| 1 | 左右关节安全软限位 | 防止四连杆撞机械限位 | 当前只有占位范围，**实测方法见第 4.5 节** |
| 1 | `q -> L` 腿长表 | 把达妙反馈角换成真实腿长 | 未完成 |
| 1 | `L -> q` 反解表或函数 | 模型输出目标腿长后转换成关节目标角 | 未完成 |
| 1 | IMU pitch/roll/gyro 正方向 | 确认前倾时 pitch 增大还是减小 | pitch 零点已有，轴方向仍要确认 |
| 1 | 达妙 MIT 模式 KP/KD（模型态） | 模型输出关节目标角后，DM 自身 MIT 控制器需要合适的刚度/阻尼 | 当前 PID 控制器的 KP=18/KD=4 是在 roll 轴平衡下调的，pitch 轴平衡可能需要不同值 |
| 2 | 车体 pitch 转动惯量 `I_pitch` | 计算 LQR/仿真 | 只有占位估算 `0.060` |
| 2 | 单轮转动惯量 `I_wheel` | 更准确的轮-地动力学 | 只有占位估算 `0.00035` |
| 2 | 左右轮实际 `torque_k` 修正 | 补偿两个 VESC/电机差异 | 已有理论值，可后续实测 |
| 2 | 固定腿长倒立摆控制增益 | 让机器人能站住 | 未计算，**计算步骤见第 8.5 节** |
| 2 | 默认站立腿长验证 | 确认 `0.11865 m`（最短最长中点）确实是合适的默认站立高度 | 未验证 |
| 3 | 腿部质量、质心和惯量 | 更高精度多体模型 | 可后续测 |
| 3 | 腿长变化时质心高度曲线 | 变腿长时更稳 | 可后续测 |
| 3 | roll 到左右腿长差增益 | 左右倾斜补偿 | 未调 |
| 3 | VESC 电流环响应 | 确认 VESC 侧电流 PI 已调好，电流阶跃响应无超调/振荡 | 未验证，可能已默认调好 |

> **优先级 0（模型 IMU 符号）是关键差异点**：当前 PID 控制器有 `APP_PID_BALANCE_AXIS_SIGN` 和 `APP_PID_BALANCE_USE_ROLL_AXIS` 两个宏来修正轴方向和符号，但 [src/ascento_balance.c](/home/h/code_leg/zephyr_ascento_f407_wheel_leg/src/ascento_balance.c) 的模型控制器直接使用原始 IMU 值，没有任何符号修正。这意味着即使 PID 控制器能正常平衡，模型控制器可能因为 IMU 符号相反导致正反馈失控。必须在启用模型前单独确认模型使用的 IMU 通道符号。

最低限度要完成优先级 0 和优先级 1 的全部 7 项，再加一组很保守的固定腿长平衡增益，才可以开始架空测试模型输出方向。

## 3. 测量前安全命令

每次测试前先在串口输入：

```text
robot enable 0
robot stop
motor debug stop
motor wheel stop
motor dm stop left
motor dm stop right
```

轮子方向调试统一使用：

```text
motor wheel current left 100 3000
motor wheel current right 100 3000
```

这里 `100` 是 `0.1 A`，`3000` 是 `3 s`。

## 4. 如何测四连杆腿部运动学

目标是得到两张表：

```text
left_q_rad  -> left_leg_length_m
right_q_rad -> right_leg_length_m
```

以及反向关系：

```text
left_leg_length_m  -> left_q_rad
right_leg_length_m -> right_q_rad
```

### 4.1 先定义腿长

建议统一用：

```text
腿长 L = 车体侧固定参考点到轮轴中心的直线距离
```

参考点可以选：

- 左右腿各自的主固定铰点中心。
- 或车体上一个固定孔位中心。

关键不是选哪个点，而是每次都用同一个点。左右腿要对称地选点。

### 4.2 测关节角正方向

轮子架空，连杆旁边不要放手。先看当前反馈：

```text
motor dm status left
motor dm status right
```

小幅动作：

```text
motor dm wiggle left 0.02 2000 18 0.8 3000
motor dm stop left

motor dm wiggle right 0.02 2000 18 0.8 3000
motor dm stop right
```

记录：

| 项目 | 记录 |
| --- | --- |
| 左关节 `q` 增大时 | 伸腿 / 收腿 |
| 右关节 `q` 增大时 | 伸腿 / 收腿 |
| 是否接近机械限位 | 是 / 否 |

如果出现冲限位趋势，立即断电或执行停止命令，不要继续加大幅度。

### 4.3 测 `q -> L` 表

推荐先手动或低速移动到 10 到 15 个姿态，每个姿态记录一次。

每个点记录：

```text
motor dm status left
motor dm status right
```

然后用尺子测：

```text
左腿 L = 左车体参考点到左轮轴中心距离
右腿 L = 右车体参考点到右轮轴中心距离
```

表格模板：

| 点 | 左关节 q(rad) | 左腿长 L(m) | 右关节 q(rad) | 右腿长 L(m) | 备注 |
| --- | --- | --- | --- | --- | --- |
| 1 |  |  |  |  | 最短附近，留安全余量 |
| 2 |  |  |  |  |  |
| 3 |  |  |  |  |  |
| 4 |  |  |  |  |  |
| 5 |  |  |  |  | 默认站立附近 |
| 6 |  |  |  |  |  |
| 7 |  |  |  |  |  |
| 8 |  |  |  |  |  |
| 9 |  |  |  |  |  |
| 10 |  |  |  |  | 最长附近，留安全余量 |

最少先确定四个端点：

```c
#define APP_ASCENTO_LEFT_JOINT_AT_MIN_LEG_RAD ...
#define APP_ASCENTO_LEFT_JOINT_AT_MAX_LEG_RAD ...
#define APP_ASCENTO_RIGHT_JOINT_AT_MIN_LEG_RAD ...
#define APP_ASCENTO_RIGHT_JOINT_AT_MAX_LEG_RAD ...
```

但是只填端点仍然是假设线性。真实四连杆建议后续改成查表插值。

### 4.4 怎么替换当前线性映射

当前 [src/ascento_balance.c](/home/h/code_leg/zephyr_ascento_f407_wheel_leg/src/ascento_balance.c) 里有两个函数：

```c
ascento_balance_leg_length_from_joint(...)
ascento_balance_joint_from_leg_length(...)
```

现在它们按端点线性插值。等你测完表格后，下一步应改成：

- `q -> L`：用实测表查表插值。
- `L -> q`：用同一张表反向查表插值。
- 超出表格范围时强制限幅，不允许命令跑到机械极限外。

### 4.5 测关节安全软限位

目标：找到左右 DM4340 关节的机械安全范围，给 `APP_ASCENTO_LEFT_JOINT_AT_MIN_LEG_RAD` / `APP_ASCENTO_LEFT_JOINT_AT_MAX_LEG_RAD` 等宏填真实值。

**关键**：这个测试必须比 `q -> L` 表更早做，因为如果不知道安全范围，测腿长表时可能撞机械限位。

步骤：

1. 轮子架空，确保四连杆不会碰地。

2. 用达妙调试命令手动小幅移动关节，向缩短腿的方向走，直到感觉即将碰到机械限位：

   ```text
   motor dm wiggle left 0.01 2000 18 0.4 3000
   motor dm wiggle right 0.01 2000 18 0.4 3000
   ```

   调整幅度 `0.01` 为更小值（如 `0.005`）来缓慢逼近。

3. 每次移动后执行：

   ```text
   motor dm status left
   motor dm status right
   ```

   记录当前关节角，并观察连杆是否即将碰到硬止挡。

4. 对伸腿方向重复同样的操作。

5. **在记录到的机械极限角上各留至少 0.03 到 0.05 rad 的安全余量**，作为软限位。

6. 填入表格：

| 参数 | 值 | 说明 |
| --- | --- | --- |
| 左关节最小安全角 rad | 待测 | 收腿最短，留余量 |
| 左关节最大安全角 rad | 待测 | 伸腿最长，留余量 |
| 右关节最小安全角 rad | 待测 | 收腿最短，留余量 |
| 右关节最大安全角 rad | 待测 | 伸腿最长，留余量 |

**注意左右不对称**：左右腿的四连杆构型可能是镜像的，关节角范围和方向可能不同（例如左腿 `q` 增大 = 伸腿，右腿 `q` 增大 = 收腿）。必须分别测，不要假设对称。

## 5. 如何确认 IMU 方向

pitch 零点已经测得：

```c
#define APP_ANGLE_ZERO_DEG -1.225f
```

还需要确认轴方向。

### 5.1 pitch 方向

轮子架空或机器人固定住，执行：

```text
robot status
```

手动让车体轻微前倾，记录：

| 动作 | pitch 变化 |
| --- | --- |
| 机械直立 | 接近 `0 deg`，因为已减零点 |
| 车体前倾 | pitch 变大 / 变小 |
| 车体后仰 | pitch 变大 / 变小 |

如果前倾和后仰的符号不符合控制模型假设，后面要在 IMU 坐标或控制符号里修正。不要靠增益硬顶。

### 5.2 roll 方向

同样执行：

```text
robot status
```

轻微左倾、右倾，记录：

| 动作 | roll 变化 |
| --- | --- |
| 左倾 | roll 变大 / 变小 |
| 右倾 | roll 变大 / 变小 |

### 5.3 yaw gyro 方向

原地轻微向左转、向右转，观察 `yaw` 或 `gz` 的变化。后续 `APP_ASCENTO_K_YAW_RATE` 需要用这个方向。

### 5.4 模型级 IMU 符号确认（关键差异点）

**这是最容易导致模型控制器首次启用时失控的问题。**

当前 PID 平衡控制器 (`pid_balance_control.c`) 和 Ascento 模型控制器 (`ascento_balance.c`) **使用的是完全不同的 IMU 路径**：

| 项目 | PID 控制器 | Ascento 模型 |
| --- | --- | --- |
| 平衡轴 | `APP_PID_BALANCE_USE_ROLL_AXIS` 可选 pitch/roll | **固定用 pitch** |
| 符号修正 | `APP_PID_BALANCE_AXIS_SIGN` 可翻转 | **无符号修正** |
| pitch 来源 | 通过 `balance_angle_deg()` 间接使用 | 直接读 `imu.pitch_deg` |
| pitch rate 来源 | 通过 `balance_gyro_dps()` 间接使用 | 直接读 `imu.gy_dps` |
| yaw rate 来源 | 不使用 | 直接读 `imu.gz_dps` |
| roll 来源 | 不使用 | 直接读 `imu.roll_deg`（用于 roll 补偿） |

也就是说，即使 PID 平衡控制器已经能站住，**模型控制器可能因为有符号差异而导致正反馈**。

确认方法：

1. 先确认 PID 控制器当前使用的轴配置：
   ```text
   // 在 app_config.h 中查看当前值
   #define APP_PID_BALANCE_USE_ROLL_AXIS 1   // 1=roll轴, 0=pitch轴
   #define APP_PID_BALANCE_AXIS_SIGN 1.0f       // 1=不反号, -1=反号
   ```

2. 在串口观察 `robot status` 的实时 pitch 值。

3. 手动让车体前倾，记录 `pitch_deg` 变化方向：
   - 如果前倾时 `pitch_deg` **增大**（正），则模型需要的符号为 `pitch * (+1)` → 不需要反号。
   - 如果前倾时 `pitch_deg` **减小**（负），则模型需要在代码中加入 `-pitch` 或 `APP_ASCENTO_IMU_PITCH_SIGN -1`。

4. 同样检查 `gy_dps`（pitch 角速度）：
   - 前倾转动时，`gy_dps` 应与 pitch 变化同号。
   - 如果 `gy_dps` 符号与 pitch 变化方向相反，模型也需要反号。

5. 检查 `gz_dps`（yaw 角速度）：
   - 从上方看，顺时针转动机器人，`gz_dps` 应为正（右手定则：z 轴向上）。
   - 模型代码 `yaw_rate_rad_s = imu.gz_dps * DEG_TO_RAD`，然后 `-params->k_yaw_rate * yaw_rate_rad_s` 产生负反馈。
   - 如果 `gz_dps` 与你预期的方向相反，需要在代码中加 `APP_ASCENTO_IMU_GZ_SIGN -1`。

6. 检查 `roll_deg`：
   - 左倾时 `roll_deg` 应为正（右手定则：x 轴向前）。
   - 模型中 `roll_leg_delta = roll * k_roll_to_leg_m_per_rad`，左倾正 roll 应该让左腿伸长、右腿缩短来补偿。
   - 如果 roll 符号反了，补偿方向也会反。

测试后填入：

| 项目 | 测试结果 | 模型需要修正 |
| --- | --- | --- |
| 前倾 pitch | 增大 / 减小 | 是否需要 `-1` |
| 前倾 gy_dps | 正 / 负 | 是否需要 `-1` |
| 顺时针 gz_dps | 正 / 负 | 是否需要 `-1` |
| 左倾 roll | 正 / 负 | 是否需要 `-1` |

**当前模型代码中这四个通道都没有符号修正宏**。如果任何一个通道需要反号，必须先修改 [src/ascento_balance.c](/home/h/code_leg/zephyr_ascento_f407_wheel_leg/src/ascento_balance.c) 中对应的读取行（第 188-193 行），才能安全启用模型。

## 6. 如何测转动惯量

初学者可以先用估算，再用实机小扰动修正。不要为了第一版平衡把转动惯量测得过度复杂。

### 6.1 车体 pitch 转动惯量 `I_pitch`

优先级：

1. 有 CAD：直接用 CAD 质量属性，轴选“左右方向，过轮轴或过质心的 pitch 轴”。
2. 没 CAD：用盒体粗估。
3. 想更准：用摆动法。

盒体粗估：

```text
I_body_com ≈ m_body * (height^2 + length^2) / 12
```

如果要换算到轮轴 pitch 轴：

```text
I_pitch_about_wheel_axis ≈ I_body_com + m_body * (h_com^2 + x_com^2)
```

其中：

- `m_body` 是车体主体质量。没有拆分数据时先用 `m_total` 做保守估算。
- `height` 是车体高度。
- `length` 是车体前后长度。
- `h_com` 是质心到轮轴高度。
- `x_com` 是质心到轮轴前后偏移。

摆动法：

1. 把车体从一根水平杆上悬挂，悬挂轴尽量平行于轮轴。
2. 测悬挂轴到质心的距离 `d`。
3. 小角度摆动，测 10 次周期取平均，得到周期 `T`。
4. 计算：

```text
I_about_pivot = m * g * d * (T / (2*pi))^2
I_about_com = I_about_pivot - m * d^2
```

然后按需要用平行轴定理换算到轮轴。

### 6.2 单轮转动惯量 `I_wheel`

粗估公式：

```text
I_wheel ≈ 0.5 * m_wheel * r_wheel^2
```

步骤：

1. 拆下一个轮子总成，包含 M3508、轮胎、轮毂。
2. 称质量 `m_wheel`，单位 kg。
3. 使用接地半径 `r_wheel = 0.030 m`。
4. 代入公式。

这个值对第一版站立不是最敏感，先估算可以接受。

## 7. 如何测 VESC 电流到轮端力矩

当前已经有理论系数：

```text
torque_k ≈ 0.003456577 Nm/mA
```

可以先用它做仿真和低功率测试。后续如果左右轮表现不一致，再实测修正。

静态力臂法：

1. 轮子架空并固定车体，确保不会跑走。
2. 在轮缘切向挂拉力计，力臂半径用 `r_wheel = 0.030 m`。
3. 发送小电流，例如：

```text
motor wheel current left 300 5000
motor wheel status left
motor wheel stop
```

4. 读取拉力 `F`，单位 N。如果电子秤读 kg，换算：

```text
F_N = kg * 9.81
```

5. 计算：

```text
tau_Nm = F_N * r_wheel
k_Nm_per_mA = tau_Nm / motor_current_mA
```

这里优先用 `motor wheel status` 里的 `motor_current`，不是 `cmd`。因为 `motor_current` 是 VESC 电流环实际反馈。

左右轮分别测，最后可以填：

```c
#define APP_ASCENTO_LEFT_CURRENT_MA_TO_WHEEL_TORQUE_NM  ...
#define APP_ASCENTO_RIGHT_CURRENT_MA_TO_WHEEL_TORQUE_NM ...
```

## 8. 需要做哪些仿真

不要直接上实机调 LQR。推荐按这个顺序做。

### 8.1 四连杆运动学仿真

目标：

- 输入达妙关节角 `q`，输出腿长 `L`。
- 输入目标腿长 `L`，反解达妙关节角 `q`。
- 检查机械限位、死点、奇异点和连杆干涉。

最低实现：

- 先用实测表格做 Python 插值。
- 画 `q-L` 曲线。
- 检查曲线是否单调。如果不单调，不能简单用一张反解表。

输出：

```text
left_q_to_L_table
right_q_to_L_table
safe_q_min / safe_q_max
safe_L_min / safe_L_max
```

### 8.2 固定腿长两轮倒立摆仿真

目标：

- 先假设腿长固定在 `0.11865 m`。
- 把机器人当成两轮倒立摆。
- 状态至少包含：

```text
pitch_rad
pitch_rate_rad_s
wheel_position_m
wheel_velocity_mps
```

输出：

```text
left_wheel_torque_Nm
right_wheel_torque_Nm
```

在仿真里加入：

- 轮半径 `0.030 m`
- 质量 `2.315 kg`
- 质心高度 `0.098 m`
- 质心后偏 `-0.040 m`
- 电流到轮端力矩系数
- 轮端力矩限幅
- 1 kHz 控制周期
- 20 到 50 ms 反馈超时保护

输出第一版：

```c
#define APP_ASCENTO_K_PITCH ...
#define APP_ASCENTO_K_PITCH_RATE ...
#define APP_ASCENTO_K_POSITION ...
#define APP_ASCENTO_K_VELOCITY ...
```

### 8.3 加入执行器限制

仿真里必须加入：

- VESC 最大电流限制。
- 轮端力矩饱和。
- 死区和静摩擦。
- CAN 反馈延迟。
- IMU 噪声和零点误差。

如果仿真里不加限幅，算出来的增益通常会在实机上一启用就过猛。

### 8.4 多体仿真

等固定腿长能站住后，再做：

- 腿长变化时保持平衡。
- 慢速前进、后退。
- 左右转向。
- roll 到左右腿长差补偿。

工具可选：

- Python 自写简化模型：最快，适合算第一版 LQR。
- MATLAB/Simulink：适合控制建模。
- MuJoCo 或 PyBullet：适合检查多体运动和四连杆干涉。

### 8.5 如何从物理参数计算 LQR 增益

对于固定腿长的两轮倒立摆，可以手工推导状态空间模型，然后在 Python 中求解 LQR 得到第一版增益。

#### 状态定义

```
x = [θ, θ̇, φ, φ̇]ᵀ
θ  = pitch 角 (rad)，车体前倾为正
θ̇ = pitch 角速度 (rad/s)
φ  = 轮子平均角位置 (rad)，前进为正
φ̇ = 轮子平均角速度 (rad/s)
```

全部取离平衡点的偏差量。

#### 输入

```
u = τ_wheel  (Nm)  轮端总力矩（左右轮之和的半值，即单轮等效驱动力矩）
实际左右轮力矩分配: τ_L = u - τ_yaw, τ_R = u + τ_yaw
```

#### 物理参数

| 符号 | 含义 | 当前值 | 单位 |
| --- | --- | --- | --- |
| m_b | 车体质量 | 2.315 | kg |
| I_b | 车体绕轮轴 pitch 转动惯量 | `APP_ASCENTO_BODY_PITCH_INERTIA_KG_M2` | kg·m² |
| I_w | 单轮转动惯量 | `APP_ASCENTO_WHEEL_INERTIA_KG_M2` | kg·m² |
| r | 轮半径 | 0.030 | m |
| L | 质心到轮轴距离 (腿长 + 质心高度) | 0.098 | m |
| g | 重力加速度 | 9.81 | m/s² |

#### 推导线性化动力学

取轮-地间纯滚动无滑移假设。拉格朗日推导后在 θ≈0 线性化：

运动方程（两轮倒立摆标准形式）：

```
(m_b·L² + I_b)·θ̈ + m_b·L·r·φ̈ - m_b·g·L·θ = -τ_wheel
m_b·L·r·θ̈ + (m_b·r² + 2·I_w)·φ̈ = τ_wheel
```

整理为标准状态空间 `ẋ = A·x + B·u`：

定义辅助常数：
```
a = m_b·L² + I_b
b = m_b·L·r
c = m_b·r² + 2·I_w
d = m_b·g·L
det = a·c - b²
```

则：
```
A = [[0, 1, 0, 0],
     [c·d/det, 0, 0, 0],
     [0, 0, 0, 1],
     [-b·d/det, 0, 0, 0]]

B = [[0],
     [-(c+b)/det],
     [0],
     [(a+b)/det]]
```

#### Python LQR 求解脚本模板

```python
import numpy as np
import scipy.linalg

# 填入实测参数
m_b = 2.315       # 车体质量 kg
I_b = 0.060       # 车体 pitch 惯量 kg*m²
I_w = 0.00035     # 单轮惯量 kg*m²
r   = 0.030       # 轮半径 m
L   = 0.098       # 质心到轮轴垂直距离 m (当前 = COM height，假设轮轴在 COM 正下方)
g   = 9.81

# 辅助常数 (注意: L 这里指质心到轮轴的垂直距离，不是腿长)
a = m_b * L**2 + I_b
b = m_b * L * r
c = m_b * r**2 + 2.0 * I_w
d = m_b * g * L
det = a * c - b**2

A = np.array([
    [0, 1, 0, 0],
    [c*d/det, 0, 0, 0],
    [0, 0, 0, 1],
    [-b*d/det, 0, 0, 0]
])

B = np.array([
    [0],
    [-(c+b)/det],
    [0],
    [(a+b)/det]
])

# LQR 权重矩阵 —— 需要根据实际调试
# Q 增大 = 对该状态惩罚更大 = 增益更高
Q = np.diag([
    500.0,    # θ  pitch 角权重
    50.0,     # θ̇ pitch 角速度权重
    10.0,     # φ  位置权重
    30.0      # φ̇ 速度权重
])

R = np.array([[0.1]])  # 控制力矩权重 (越小 → 增益越高 → 力矩越大)

# 求解 LQR
P = scipy.linalg.solve_continuous_are(A, B, Q, R)
K = np.linalg.solve(R, B.T @ P)

print("K =", K)
# K[0] = K_pitch, K[1] = K_pitch_rate, K[2] = K_position, K[3] = K_velocity
print(f"APP_ASCENTO_K_PITCH      = {K[0,0]:.3f}")
print(f"APP_ASCENTO_K_PITCH_RATE = {K[0,1]:.3f}")
print(f"APP_ASCENTO_K_POSITION   = {K[0,2]:.3f}")
print(f"APP_ASCENTO_K_VELOCITY   = {K[0,3]:.3f}")

# 检查闭环极点
eigvals = np.linalg.eigvals(A - B @ K)
print(f"闭环极点: {eigvals}")
print(f"主导极点实部: {np.max(np.real(eigvals)):.4f} (应 < 0)")
```

#### 增益调试顺序

**不要一次把所有增益都填进去**：

1. **先只调 K_pitch 和 K_pitch_rate**（K_position = K_velocity = 0）。让机器人能原地站立不倒。
2. 站立稳定后，**再加入 K_velocity**（小值开始，如 10-50），让机器人能对抗微小推搡。
3. 最后加入 **K_position**（小值开始），让机器人能回到原位。

#### K_yaw_rate 计算

Yaw 控制是独立的，不需要 LQR。从轮距和车轮动力学推导：

```
τ_yaw = K_yaw_rate * (yaw_rate_target - yaw_rate_actual)

最大 yaw 力矩 ≈ K_yaw_rate * 1.0 rad/s  (设计点)

τ_yaw_max = F_wheel * (wheel_base / 2)
          = (τ_wheel / r) * (wheel_base / 2)
          = (I_lim * torque_k / r) * (0.201 / 2)
```

取保守值：从 0.5 Nm/(rad/s) 开始，逐步上调。

#### K_roll_to_leg_m_per_rad

这个增益目前模型中设为 0。它决定倾斜时左右腿长差：

```
left_leg_length  = target_leg - roll * K_roll_to_leg
right_leg_length = target_leg + roll * K_roll_to_leg
```

初始建议：取 0.05 m/rad，即 1 度 roll 约产生 0.87 mm 腿长差。等 pitch 平衡调稳定后再加这个补偿。

## 9. 模型接入前的软件任务

接入前至少完成：

- **确认模型级 IMU 四个通道符号**（pitch/gy_dps/gz_dps/roll），如需要则给 [src/ascento_balance.c](/home/h/code_leg/zephyr_ascento_f407_wheel_leg/src/ascento_balance.c) 第 188-193 行添加 `APP_ASCENTO_IMU_*_SIGN` 宏。
- 把 `q <-> L` 线性映射改成查表插值或解析反解。
- 把转动惯量估算值更新到 `APP_ASCENTO_BODY_PITCH_INERTIA_KG_M2` 和 `APP_ASCENTO_WHEEL_INERTIA_KG_M2`。
- 用 LQR 脚本计算并填入 `APP_ASCENTO_K_*` 第一版增益。
- 确认 DM MIT 控制参数（KP/KD）适合 pitch 轴平衡（当前 18/4 是在 roll 轴下调的）。
- 把 `APP_WHEEL_CURRENT_LIMIT` 临时降到保守值，例如先不超过 `1000` 到 `1500 mA`。
- 确认 `APP_PITCH_FAULT_DEG` 足够保守。
- 确认启用模型前参数检查生效。

暂时不要改：

```c
#define APP_USE_ASCENTO_BALANCE_CONTROLLER 0
#define APP_ASCENTO_PARAMS_CALIBRATED 0
```

等上面完成后，先架空启用，只看输出方向，不落地。

## 10. 第一版实机验证顺序

1. 只接 C 板，确认串口、IMU、电池电压正常。
2. 只接 CAN1 达妙，确认左右关节小幅动作和停止正常。
3. 只接 CAN2 VESC，确认左右轮 `100 mA / 3 s` 动作和停止正常。
4. 轮子架空，启用很小的模型输出，确认前倾时轮子趋势是扶正。
5. 手扶落地，轮电流限幅很小，只看扶正趋势。
6. 每次只测试几秒，随时断电。
7. 逐步调 `K_PITCH` 和 `K_PITCH_RATE`。
8. 能原地站住后，再加入位置、速度和 yaw。
9. 最后再加入腿长变化和 roll 补偿。

## 11. 最小数据回填表

你后续可以按这张表把数据发回来，我就能继续帮你写进模型或生成仿真脚本。

| 项目 | 当前值 / 待填 |
| --- | --- |
| 模型级 pitch 符号（前倾时 pitch 增大/减小） | 待测 |
| 模型级 gy_dps 符号（前倾转动时正/负） | 待测 |
| 模型级 gz_dps 符号（顺时针转时正/负） | 待测 |
| 模型级 roll 符号（左倾时正/负） | 待测 |
| 左关节正角度方向（增大 = 伸腿/收腿） | 待测 |
| 右关节正角度方向（增大 = 伸腿/收腿） | 待测 |
| 左关节安全最小角 rad（收腿方向，留余量） | 待测 |
| 左关节安全最大角 rad（伸腿方向，留余量） | 待测 |
| 右关节安全最小角 rad（收腿方向，留余量） | 待测 |
| 右关节安全最大角 rad（伸腿方向，留余量） | 待测 |
| 左腿 `q -> L` 表（10-15 点） | 待测 |
| 右腿 `q -> L` 表（10-15 点） | 待测 |
| IMU 前倾时 pitch 符号 | 待测 |
| IMU 左倾时 roll 符号 | 待测 |
| yaw gyro 正方向 | 待测 |
| 车体 pitch 转动惯量 `I_pitch` | 待估算/实测 |
| 单个轮子转动惯量 `I_wheel` | 待估算/实测 |
| 左轮实测 `torque_k` | 可选，当前用理论值 |
| 右轮实测 `torque_k` | 可选，当前用理论值 |
| DM MIT 模型态 KP（pitch 轴平衡用） | 当前 PID 用 18，模型可能需要不同值 |
| DM MIT 模型态 KD（pitch 轴平衡用） | 当前 PID 用 4，模型可能需要不同值 |
| `APP_ASCENTO_K_PITCH` | 待 LQR 求解/调参 |
| `APP_ASCENTO_K_PITCH_RATE` | 待 LQR 求解/调参 |
| `APP_ASCENTO_K_POSITION` | 待仿真/调参 |
| `APP_ASCENTO_K_VELOCITY` | 待仿真/调参 |
| `APP_ASCENTO_K_YAW_RATE` | 待仿真/调参 |
| `APP_ASCENTO_K_ROLL_TO_LEG_M_PER_RAD` | 待仿真/调参 |

## 12. 每次测试记录模板

| 项目 | 记录 |
| --- | --- |
| 日期 |  |
| 固件版本或 git diff |  |
| 电池电压 |  |
| 测试命令 |  |
| 左轮 cmd / motor_current / erpm |  |
| 右轮 cmd / motor_current / erpm |  |
| 左达妙 q / 速度 / 温度 |  |
| 右达妙 q / 速度 / 温度 |  |
| IMU pitch / roll / yaw |  |
| 机器人实际动作 |  |
| 是否触发 fault |  |
| 需要修改的参数 |  |

## 13. 最重要的提醒

当前你已经完成了很关键的底层工作：VESC 路径确认、轮子方向、IMU pitch 零点、基本质量和几何参数。下一步不要急着启用模型，先把四连杆 `q <-> L` 和 IMU 符号测清楚。对轮腿倒立摆来说，符号错比增益小更危险，符号错会让机器人越控越倒。
