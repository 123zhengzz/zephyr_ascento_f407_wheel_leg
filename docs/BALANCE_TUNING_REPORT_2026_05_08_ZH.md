# 平衡调试报告 2026-05-08

本文记录 Ascento 轮腿机器人平衡控制的系统性调试过程，包括符号验证、日志分析、根因定位和代码修改。

## 1. 问题现象

机器人在以下参数下可以短暂平衡，但会出现两种故障模式：

```
theta_eq       = 0.3250 rad (18.6 deg)
k_pitch        = -3.00
k_pitch_rate   = -2.80
k_position     = 0.04
k_velocity     = 0.45
```

**故障模式 A**：平衡一段时间后突然向后摔倒
**故障模式 B**：在平衡点附近剧烈振荡（7° ~ 78°），反复摔倒

## 2. 符号验证

### 2.1 控制律

控制律公式（`ascento_balance.c:1132`）：

```
balance_torque = -(K_pitch * pitch_error + K_pitch_rate * pitch_rate + K_position * x_error + K_velocity * v_error)
```

其中 `pitch_error = pitch_rad - theta_eq - target_pitch_rad`。

### 2.2 IMU 符号校正链

完整的信号链路：

```
BMI088 原始 pitch_deg
    ↓ × APP_ASCENTO_IMU_PITCH_SIGN (-1.0)
pitch_rad（控制律使用的俯仰角）
    ↓
pitch_error = pitch_rad - theta_eq
    ↓ × K_pitch (-3.0)
    ↓ × 外层负号 (-1)
有效恢复力矩方向
```

关键符号因子（`app_config.h:660-663`）：

| 符号因子 | 值 | 含义 |
|---------|------|------|
| `APP_ASCENTO_IMU_PITCH_SIGN` | -1.0 | BMI088 正 pitch → 控制模型负 pitch |
| `APP_ASCENTO_IMU_GY_SIGN` | +1.0 | BMI088 正 gx → 正 pitch rate |
| `APP_WHEEL_LEFT_FORWARD_CURRENT_SIGN` | +1 | VESC 正电流 → 左轮前进 |
| `APP_WHEEL_RIGHT_FORWARD_CURRENT_SIGN` | -1 | VESC 负电流 → 右轮前进 |

### 2.3 恢复力矩方向推导

假设机器人前倾（物理 pitch > theta_eq）：

```
BMI088 pitch_deg > 0（前倾）
    → pitch_rad = -1.0 × pitch_deg < 0（符号校正后）
    → pitch_error = pitch_rad - theta_eq < 0（因为 pitch_rad 是负值，theta_eq 是正值）
    → K_pitch × pitch_error = (-3.0) × (负值) = 正值
    → balance_torque = -(正值) = 负值
    → 左轮: apply_wheel_forward_sign(负值, +1) = 负值 → VESC 负电流 → 后退
    → 右轮: apply_wheel_forward_sign(负值, -1) = 正值 → VESC 正电流 → 后退
    → 两轮后退 → 轮子移到质心下方 → 纠正前倾 ✓
```

**结论：符号方向正确，不是正反馈。**

如果符号完全错误（纯正反馈），机器人永远不可能平衡。实际观察到机器人可以短暂平衡，证实了负反馈方向正确。

### 2.4 阻尼力矩方向推导

假设机器人前倾且正在向前旋转（pitch_rate > 0）：

```
pitch_rate_rad_s > 0
    → K_pitch_rate × pitch_rate = (-2.8) × (正值) = 负值
    → balance_torque 的 pr_term 贡献 = -(负值) = 正值
    → 正值力矩 → 两轮前进 → 轮子追到质心下方 → 阻尼前向旋转 ✓
```

**阻尼方向也正确。**

## 3. 日志分析

### 3.1 关键数据点

从运行日志中提取的关键时刻：

```
t=22.265s: pitch=14.76° err=-3.86° p_term=-0.202 pr_term=+0.427 torque=+0.266  ← 向平衡点恢复
t=22.653s: pitch=16.75° err=-1.87° p_term=-0.098 pr_term=-0.128 torque=-0.187  ← 接近平衡点
t=23.051s: pitch=18.74° err=+0.12° p_term=+0.006 pr_term=-0.754 torque=-0.694  ← 在平衡点！但力矩向后
t=23.266s: pitch=50.62° ← 0.2秒内从18°冲到50°，失控
t=23.449s: pitch=64.25° ← 继续冲到64°
```

### 3.2 平衡点处的力矩分解

在 t=23.051s（pitch=18.74°，几乎在 theta_eq=18.6°）：

| 项 | 值 (Nm) | 占比 |
|------|---------|------|
| p_term（恢复力） | +0.006 | **0.8%** |
| pr_term（阻尼力） | -0.754 | **99.2%** |
| pos_term（位置） | +0.010 | ~1% |
| vel_term（速度） | +0.045 | ~6% |
| **总力矩** | **-0.694** | — |

**问题**：在平衡点处，阻尼力矩是恢复力矩的 **125 倍**。

### 3.3 物理解释

1. 机器人从 14.76° 向 18.6° 恢复（pitch_rate > 0，向前旋转）
2. 接近平衡点时，pitch_error ≈ 0，恢复力矩消失
3. 但 pitch_rate 仍然较大（约 +15°/s），阻尼项产生 -0.754 Nm 的反向力矩
4. 这个反向力矩使轮子减速 → 机器人惯性继续前冲 → 越过平衡点
5. 越过平衡点后 pitch_error 变正，但此刻 pitch_rate 也变正 → 两个正项相加 → 力矩猛增
6. 系统失控，振荡幅度越来越大

### 3.4 pitch_rate 噪声问题

从日志中提取的 pitch_rate 变化：

```
t=20.296s: pr_term=-1.065  (pitch_rate ≈ +21.9°/s)
t=20.684s: pr_term=-0.349  (pitch_rate ≈ +7.2°/s)
t=21.471s: pr_term=-0.453  (pitch_rate ≈ +9.3°/s)
t=21.868s: pr_term=+0.632  (pitch_rate ≈ -13.0°/s)  ← 0.4秒内从+9翻转到-13
t=22.265s: pr_term=+0.427  (pitch_rate ≈ -8.8°/s)
t=22.653s: pr_term=-0.128  (pitch_rate ≈ +2.6°/s)   ← 又翻转
t=23.051s: pr_term=-0.754  (pitch_rate ≈ +15.5°/s)  ← 再翻转
```

pitch_rate 在 ±15°/s 之间剧烈振荡，导致 pr_term 产生 ±0.7 Nm 的力矩尖峰。这些尖峰远大于 p_term 的恢复力矩（~0.01 Nm），主导了控制行为。

## 4. 根因总结

| 根因 | 说明 | 影响 |
|------|------|------|
| **K_pitch 相对太小** | p_term ≈ 0.01 Nm vs pr_term ≈ 0.7 Nm | 平衡点附近无恢复力，阻尼主导 |
| **pitch_rate 信号噪声大** | ±15°/s 振荡，无低通滤波 | 产生 ±0.7 Nm 力矩尖峰 |
| **位置积分器衰减** | 之前添加的 10s 衰减 | 引入额外漂移机制 |

注意：这不是符号问题。符号经验证是正确的。

## 5. 代码修改

### 5.1 添加 pitch_rate 低通滤波器

**目的**：滤除陀螺仪高频噪声，减少力矩尖峰。

**文件**：`ascento_balance.h`

```diff
  float body_velocity_lpf_mps;
  float yaw_rate_lpf_rad_s;
+ float pitch_rate_lpf_rad_s;
```

**文件**：`ascento_balance.c`（初始化部分）

```diff
  state->yaw_rate_lpf_rad_s = input->imu.gz_dps * DEG_TO_RAD;
+ state->pitch_rate_lpf_rad_s = APP_ASCENTO_IMU_GY_SIGN *
+                               input->imu.gx_dps * DEG_TO_RAD;
```

**文件**：`ascento_balance.c`（滤波应用）

```diff
- const float pitch_rate_rad_s = APP_ASCENTO_IMU_GY_SIGN *
-                                input->imu.gx_dps * DEG_TO_RAD;
+ const float pitch_rate_rad_s = APP_ASCENTO_IMU_GY_SIGN *
+                                app_lpf_update(input->imu.gx_dps * DEG_TO_RAD,
+                                               &state->pitch_rate_lpf_rad_s,
+                                               0.008f, dt_s);
```

滤波参数：
- 时间常数 τ = 0.008s
- 截止频率 f_c = 1/(2πτ) ≈ 20Hz
- 与现有 velocity LPF（τ=0.012s, ~13Hz）和 yaw_rate LPF（τ=0.012s）一致

### 5.2 移除位置积分器衰减

**目的**：之前添加的衰减会引入位置漂移，破坏平衡。

**文件**：`ascento_balance.c`

```diff
  state->body_position_m =
      wheel_distance_m - state->wheel_position_zero_m;
-
- /* 位置积分器衰减 */
- const float pos_decay = 1.0f - dt_s * 0.1f;
- state->body_position_m *= (pos_decay > 0.0f) ? pos_decay : 0.0f;
```

### 5.3 符号不变

之前误判符号反了（忽略了 `APP_ASCENTO_IMU_PITCH_SIGN = -1` 的作用），已恢复原始值：

```c
#define APP_ASCENTO_K_PITCH       -3.00f   /* 不变 */
#define APP_ASCENTO_K_PITCH_RATE  -2.80f   /* 不变 */
```

## 6. 调参建议

### 6.1 核心问题：K_pitch 太小

从力矩分解看，恢复力矩（p_term）只有阻尼力矩（pr_term）的 1/125。需要大幅增加 K_pitch。

二阶系统分析：
- 自然频率 ω_n ∝ √|K_pitch|
- 阻尼比 ζ ∝ |K_pitch_rate| / √|K_pitch|

增加 K_pitch 会：
- 提高自然频率 → 更快回到平衡点
- 降低阻尼比 → 从过阻尼变为适度欠阻尼
- 增大平衡点附近的恢复力矩

### 6.2 推荐参数

在 pitch_rate LPF 已添加的前提下，建议从以下参数开始测试：

```
robot param k_pitch -10
robot param k_pitch_rate -2.8
robot param k_position 0.04
robot param k_velocity 0.45
robot param theta_eq 0.325
```

调参路径：

| 步骤 | K_pitch | K_pitch_rate | 预期效果 |
|------|---------|-------------|---------|
| 1 | -5 | -2.8 | 比当前好，可能仍然振荡 |
| 2 | -8 | -2.8 | 接近临界阻尼 |
| 3 | -10 | -2.8 | 略欠阻尼，响应快 |
| 4 | -12 | -2.8 | 如果步骤3太慢 |
| 5 | -10 | -3.5 | 如果步骤3振荡太大 |

### 6.3 调参原则

1. **先调 K_pitch，固定 K_pitch_rate**：找到能稳定平衡的最小 K_pitch
2. **再调 K_pitch_rate**：如果振荡太大增加，太慢减小
3. **最后微调 theta_eq**：找到真正的平衡点
4. **不要同时改多个参数**

### 6.4 注意事项

- 新增的 pitch_rate LPF 会引入约 8ms 的相位滞后，在高频振荡时可能需要适当降低 K_pitch_rate
- 如果增大 K_pitch 后出现高频颤抖（>10Hz），说明 LPF 截止频率太高，需要增大时间常数（如从 0.008 改为 0.012）
- 位置积分器衰减已移除。如果之后再次出现长时间运行后位置累积导致的后仰摔倒，可以考虑用更温和的方案（如位置限幅 ±0.5m）替代衰减

## 7. 验证步骤

1. 编译烧录新固件
2. 执行 `ascento reset` 清除 Flash 中的旧参数
3. 设置推荐参数：
   ```
   robot param k_pitch -10
   robot param k_pitch_rate -2.8
   robot param k_position 0.04
   robot param k_velocity 0.45
   robot param theta_eq 0.325
   ```
4. `robot enable 1` 启动平衡
5. 观察日志中的 p_term 和 pr_term 比例，理想情况是 p_term 在平衡点附近占主导（>50%）
6. 如果仍然振荡，增加 K_pitch_rate 到 -3.5
7. 如果恢复太慢，增加 K_pitch 到 -12

## 8. 修改文件清单

| 文件 | 修改内容 |
|------|---------|
| `src/ascento_balance.h` | 添加 `pitch_rate_lpf_rad_s` 字段 |
| `src/ascento_balance.c` | 初始化 pitch_rate LPF；应用 LPF 滤波；移除位置积分器衰减 |
| `src/app_config.h` | 符号保持不变（-3.0 / -2.8） |
