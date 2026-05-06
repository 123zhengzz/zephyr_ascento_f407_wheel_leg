# Ascento 轮腿机器人站立调参指南

> 适用：Ascento 四连杆轮腿机器人，`APP_USE_ASCENTO_BALANCE_CONTROLLER=1`
> 更新：2026-05-05

---

## 1. 概述

本文档指导你将机器人从"站不起来"调整到"稳定站立"。所有参数均可通过串口实时修改，无需重新编译。

**核心原理**：机器人通过 LQR 控制器平衡，控制器输出轮子力矩来抵抗倾倒。关键是要让控制器在正确的平衡点工作，并有足够的增益和阻尼。

---

## 2. 调参前准备

### 2.1 硬件检查

```bash
# 连接串口
./scripts/serial.sh

# 检查系统状态
robot status

# 检查电机反馈
motor wheel status all
motor dm status all

# 检查 CAN 总线
motor can status all
```

**预期结果**：
- `batt` > 20V（电池电压充足）
- 轮电机 `age` < 50ms（反馈在线）
- 关节电机 `age` < 20ms，`err=0`
- CAN 状态 `error-active`

### 2.2 IMU 验证

```bash
# 机器人静止直立
robot status  # 记录 pitch, roll

# 前倾 15°
robot status  # pitch 应变化约 +15°

# 左倾 15°
robot status  # roll 应变化约 +15°
```

**如果 pitch/roll 方向相反或不变化**：IMU 轴映射有误，需修改 `bmi088.c` 中的轴映射。

### 2.3 电机方向验证

```bash
# 关闭控制
robot enable 0

# 测试左轮（前进方向）
motor wheel current left 100 3000

# 测试右轮（前进方向）
motor wheel current right 100 3000
```

**预期**：两个轮子都向前转（物理方向相反是正常的，因为对侧安装）。

---

## 3. 调参流程

### 3.1 第一步：确定平衡参考角 theta_eq

这是**最关键的参数**，决定了机器人在哪个角度达到平衡。

**测量方法**：
1. 手持机器人，缓慢倾斜
2. 找到机器人"自然平衡"的角度（不需要外力就能保持的角度）
3. 用 `robot status` 读取此时的 `pitch=` 值
4. 将该值设为 `theta_eq`

**设置命令**：
```bash
# 例如实测平衡点为前倾 17°
robot param theta_eq 0.2967

# 或者用角度制（需要手动换算：rad = deg × π / 180）
# 17° = 0.2967 rad
# 15° = 0.2618 rad
# 20° = 0.3491 rad
```

**验证**：
```bash
# 扶住机器人在平衡点
robot status
# 看 pitch= 值，应该接近 theta_eq
# 如果 err=0，说明 theta_eq 正确
```

**常见问题**：
| 现象 | 原因 | 解决 |
|------|------|------|
| 机器人向前倒 | theta_eq 太小 | 增大 theta_eq |
| 机器人向后倒 | theta_eq 太大 | 减小 theta_eq |
| 开启后立刻 fault | theta_eq 与实际姿态差距大 | 调整 theta_eq 使 err≈0 |

### 3.2 第二步：调整主增益 k_pitch_c

`k_pitch_c` 控制 pitch 角度误差的回复力矩。

**默认值**：-5.2007（在 `app_config.h` 中定义）

**调法**：
```bash
# 查看当前值
robot param

# 机器人无力倒下 → 增大绝对值（更激进）
robot param k_pitch_c -7.0

# 机器人振荡/抖动 → 减小绝对值（更柔和）
robot param k_pitch_c -4.0
```

**参考范围**：-3.0 到 -10.0

### 3.3 第三步：调整阻尼增益 k_pitch_rate_c

`k_pitch_rate_c` 控制角速度阻尼，防止振荡。

**默认值**：-1.4842

**调法**：
```bash
# 剧烈振荡 → 增大绝对值（更多阻尼）
robot param k_pitch_rate_c -2.5

# 响应迟钝 → 减小绝对值（更少阻尼）
robot param k_pitch_rate_c -1.0
```

**参考范围**：-0.5 到 -3.0

### 3.4 第四步：调整摩擦补偿 stiction_ma

VESC 电机有最小电流阈值（约 0.5-1A），小角度时 LQR 输出可能低于阈值导致轮子不转。

**默认值**：2800 mA

**调法**：
```bash
# 小角度不响应 → 增大
robot param stiction_ma 3500

# 振荡/过冲 → 减小或禁用
robot param stiction_ma 0
```

**参考范围**：0 到 4000 mA

### 3.5 第五步：调整电流限制 current_limit

**默认值**：1500 mA

**调法**：
```bash
# 力矩不足 → 增大
robot param current_limit 2500

# 轮子啸叫/饱和 → 减小
robot param current_limit 1000
```

**参考范围**：800 到 3000 mA

### 3.6 第六步：调整轮子同步 sync_gain

如果左右轮转速不同步（走直线跑偏）。

**默认值**：100 mA/(rad/s)

**调法**：
```bash
# 轮子不同步 → 增大
robot param sync_gain 200

# 振荡/抖动 → 减小
robot param sync_gain 50

# 不需要同步 → 禁用
robot param sync_gain 0
```

---

## 4. 完整调参序列示例

假设机器人站不起来，按以下顺序调整：

```bash
# 1. 查看当前参数
robot param

# 2. 调整平衡点（假设实测 17°）
robot param theta_eq 0.2967

# 3. 增大主增益（更激进）
robot param k_pitch_c -7.0

# 4. 增大阻尼（防振荡）
robot param k_pitch_rate_c -2.0

# 5. 增大摩擦补偿
robot param stiction_ma 3500

# 6. 增大电流限制
robot param current_limit 2000

# 7. 测试
robot enable 1

# 8. 根据表现微调...

# 9. 保存
robot param save
```

---

## 5. 故障排查速查表

| 现象 | 可能原因 | 调整命令 |
|------|---------|----------|
| **向前倒** | theta_eq 太小 | `robot param theta_eq 0.32` |
| **向后倒** | theta_eq 太大 | `robot param theta_eq 0.27` |
| **剧烈振荡** | 增益过大或阻尼不足 | `robot param k_pitch_c -4.0` 或 `robot param k_pitch_rate_c -2.5` |
| **缓慢漂移后倒** | K_position/K_velocity 不足 | `robot param k_position -1.5` |
| **小角度不响应** | stiction 不足 | `robot param stiction_ma 3500` |
| **使能后立刻 fault** | theta_eq 与实际姿态差距大 | 调整 theta_eq 使 err≈0 |
| **轮子单向转不停** | theta_eq 远离工作点 | 调整 theta_eq |
| **电流饱和（轮子啸叫）** | 增益过大或电流限制太低 | 减小增益或增大 current_limit |
| **轮子不同步** | 电机参数差异 | `robot param sync_gain 200` |
| **轮子不转** | VESC 反馈丢失 | 检查 `motor wheel status all` |
| **关节无力** | DM 电机未使能 | 检查 `motor dm status all` |

---

## 6. 高级调参

### 6.1 位置/速度增益

如果机器人能短暂站立但会漂移：

```bash
# 增大位置回复力
robot param k_position -1.5

# 增大速度阻尼
robot param k_velocity_c -2.0
```

### 6.2 增益调度（LQR 多项式）

增益按腿长 L (m) 进行调度：`K(L) = A*L^2 + B*L + C`

```bash
# 查看当前增益公式
robot param
# 输出：k_pitch = -8.0019*L^2 + 2.9106*L + -6.0000

# 调整常数项（最常用）
robot param k_pitch_c -7.0

# 调整 L 系数
robot param k_pitch_b 3.5

# 调整 L^2 系数
robot param k_pitch_a -9.0
```

### 6.3 故障保护阈值

```bash
# 放宽 fault 阈值（调试时避免频繁触发）
robot param fault_deg 90

# 放宽 recover 阈值
robot param recover_deg 20
```

### 6.4 Stiction 曲线

```bash
# 调整起始角度（更早开始补偿）
robot param stiction_start 0.05

# 调整满幅角度（更快达到满幅）
robot param stiction_full 1.0
```

---

## 7. 参数持久化

```bash
# 保存到 Flash（永久，断电保留）
robot param save

# 恢复默认值（清除 Flash 存储）
robot param reset

# 查看确认
robot param
```

**注意**：
- `robot param <name> <value>` 立即生效，但断电丢失
- `robot param save` 保存到 Flash，下次开机自动加载
- 重新烧录固件不会擦除参数（参数在独立的 Flash 分区）

---

## 8. 调试技巧

### 8.1 三层诊断

```bash
# 1. 模型内部（~1Hz）
# 看 [ascento] 日志中的 cur= 值

# 2. 控制器状态（按需）
robot status
# 看 enable=, wheels=, fault=

# 3. 电机实际（按需）
motor wheel status all
# 看 cmd= vs motor_current=
```

### 8.2 架空测试

```bash
# 将机器人架空（轮子离地）
robot enable 1

# 轻推机器人，观察轮子方向
# 前倾 → 轮子向前转 ✓
# 后仰 → 轮子向后转 ✓
```

### 8.3 增量调参

每次只调整一个参数，观察效果后再调下一个。避免同时改多个参数导致无法判断哪个有效。

### 8.4 记录参数

每次成功调参后，记录参数值：

```bash
robot param
# 复制输出保存
```

---

## 9. 物理参数确认

以下物理参数在 `app_config.h` 中定义，需要与实际机器人匹配：

| 参数 | 代码默认值 | 含义 | 如何测量 |
|------|-----------|------|---------|
| `APP_ASCENTO_TOTAL_MASS_KG` | 2.315 | 总质量 (kg) | 秤重 |
| `APP_ASCENTO_BODY_COM_HEIGHT_M` | 0.098 | 质心高度 (m) | 从轮轴到质心的垂直距离 |
| `APP_ASCENTO_BODY_COM_FORWARD_OFFSET_M` | -0.0245 | 质心前后偏移 (m) | 质心在轮轴前方为正，后方为负 |
| `APP_ASCENTO_WHEEL_RADIUS_M` | 0.030 | 轮半径 (m) | 测量轮子直径/2 |
| `APP_ASCENTO_WHEEL_BASE_M` | 0.201 | 轮距 (m) | 两轮中心距离 |
| `APP_ASCENTO_BODY_PITCH_INERTIA_KG_M2` | 0.060 | 转动惯量 (kg·m²) | 估算或摆动实验 |

**如果这些参数不准确**：LQR 增益计算会偏差，需要重新计算增益多项式。

---

## 10. 快速参考卡

### 常用命令

```bash
./scripts/serial.sh          # 打开串口
robot status                 # 查看状态
robot param                  # 查看参数
robot param <name> <value>   # 修改参数
robot param save             # 保存参数
robot enable 1               # 开启控制
robot enable 0               # 关闭控制
motor wheel status all       # 轮电机状态
motor dm status all          # 关节电机状态
```

### 参数调整方向

| 参数 | 增大效果 | 减小效果 |
|------|---------|---------|
| theta_eq | 平衡点前倾 | 平衡点后倾 |
| k_pitch_c | 更激进、更硬 | 更柔和、更软 |
| k_pitch_rate_c | 更多阻尼、更稳 | 更少阻尼、更快 |
| stiction_ma | 小角度更响应 | 减少过冲 |
| current_limit | 更大力矩 | 更安全 |
| sync_gain | 轮子更同步 | 减少同步 |

---

## 11. 关联文档

- [PARAM_TUNING_GUIDE.md](PARAM_TUNING_GUIDE.md) — 参数详细说明
- [SERIAL_COMMAND_REFERENCE.md](SERIAL_COMMAND_REFERENCE.md) — 串口命令完整参考
- [ASCENTO_README_ZH.md](ASCENTO_README_ZH.md) — Ascento 项目文档
- [VESC_TORQUE_COEFFICIENT_BEGINNER_ZH.md](VESC_TORQUE_COEFFICIENT_BEGINNER_ZH.md) — VESC 扭矩系数说明
