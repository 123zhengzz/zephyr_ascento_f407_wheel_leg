# LQR 平衡参数串口调参指南

> 适用：Ascento 四连杆轮腿机器人，`APP_USE_ASCENTO_BALANCE_CONTROLLER=1`
> 更新：2026-05-05

---

## 1. 概述

所有 LQR 平衡控制参数均可通过串口实时修改，无需重新编译固件。参数修改立即生效，支持保存到 Flash 实现断电持久化。

**调参命令**：

| 命令 | 作用 |
|------|------|
| `robot param` | 查看所有当前参数 |
| `robot param list` | 列出可调参数名及说明 |
| `robot param <name> <value>` | 修改单个参数（立即生效，断电丢失） |
| `robot param save` | 保存到 Flash（永久，断电保留） |
| `robot param reset` | 恢复代码默认值，清除 Flash 存储 |

---

## 2. 可调参数一览

### 2.1 平衡参考角

| 参数名 | 说明 | 默认值 |
|--------|------|--------|
| `theta_eq` | 平衡参考角 (rad)，前倾为正 | 0.245 (14°) |

**物理含义**：机器人自然静止时的前倾角度。COM 在轮轴后方约 24.5mm，需要前倾才能让 COM 对正轮轴。

**调法**：扶住机器人找到自然平衡点，用 `robot status` 看 `pitch=` 值，让该值接近 `theta_eq`。

### 2.2 LQR 增益多项式系数

增益按腿长 L (m) 进行调度：`K(L) = A*L^2 + B*L + C`

| 参数名 | 对应增益 | 说明 | 默认值 |
|--------|---------|------|--------|
| `k_pitch_a` | K_pitch | L^2 系数 | -8.0019 |
| `k_pitch_b` | K_pitch | L 系数 | 2.9106 |
| `k_pitch_c` | K_pitch | 常数项 | -5.2007 |
| `k_pitch_rate_a` | K_pitch_rate | L^2 系数 | -19.1727 |
| `k_pitch_rate_b` | K_pitch_rate | L 系数 | 7.0983 |
| `k_pitch_rate_c` | K_pitch_rate | 常数项 | -1.4842 |
| `k_position` | K_position | 常数（不随 L 变化） | -0.6325 |
| `k_velocity_a` | K_velocity | L^2 系数 | -6.1109 |
| `k_velocity_b` | K_velocity | L 系数 | 2.3991 |
| `k_velocity_c` | K_velocity | 常数项 | -1.1471 |
| `k_yaw_rate` | K_yaw_rate | 偏航角速率增益 | 0.0 |

**站立高度 L=0.115m 处的实际增益**：

| 增益 | 计算值 | 作用 |
|------|--------|------|
| K_pitch | -4.98 | 倾角误差回复力矩 |
| K_pitch_rate | -0.97 | 角速度阻尼 |
| K_position | -0.63 | 位置漂移回复 |
| K_velocity | -0.99 | 速度阻尼 |

### 2.3 Stiction 补偿

| 参数名 | 说明 | 默认值 |
|--------|------|--------|
| `stiction_ma` | 补偿电流幅值 (mA) | 2800 |
| `stiction_start` | 起始角度 (deg) | 0.10 |
| `stiction_full` | 满幅角度 (deg) | 1.50 |

**作用**：克服 VESC 电机控制器的最小电流死区（~0.5-1A）。倾角超过 `stiction_start` 后线性叠加电流，到 `stiction_full` 时满幅输出。

### 2.4 电流限制

| 参数名 | 说明 | 默认值 |
|--------|------|--------|
| `current_limit` | 轮子电流上限 (mA) | 1500 |
| `current_scale` | 电流缩放系数 | 1.0 |

### 2.5 轮子速度同步

| 参数名 | 说明 | 默认值 |
|--------|------|--------|
| `sync_gain` | 同步增益 (mA 每 rad/s 速度差) | 100 |
| `sync_limit` | 同步电流上限 (mA) | 500 |

**作用**：当左右轮转速不一致时（电机参数差异、摩擦不同、轮径差异），自动补偿电流使轮子同步。

**调法**：
- 轮子不同步（走直线跑偏）→ 增大 `sync_gain`，如 200
- 振荡/抖动 → 减小 `sync_gain`，如 50
- 不需要同步 → 设为 0 禁用

### 2.5 故障保护

| 参数名 | 说明 | 默认值 |
|--------|------|--------|
| `fault_deg` | 触发保护的角度阈值 (deg) | 30.0 |
| `recover_deg` | 恢复允许的角度范围 (deg) | 10.0 |

**作用**：机身绝对 pitch 超过 `fault_deg` 时切断电机输出，防止机器人剧烈翻转。代码还有不可被 Flash 参数放宽的硬限制：前倾最多 30 deg，后仰最多 22 deg，恢复必须回到 8 deg 内。保护不再以 `pitch - theta_eq` 为准。

---

## 3. 调参流程

### 3.1 准备

```text
# 连接串口
./scripts/serial.sh

# 确认机器人状态
robot status
```

### 3.2 快速调参步骤

```text
# 第一步：查看当前参数
robot param

# 第二步：调整平衡参考角
# 扶住机器人，看 robot status 的 pitch= 值
# 如果机器人自然前倾 14°，设置：
robot param theta_eq 0.245

# 第三步：调整主增益（最关键）
# 机器人振荡（左右晃）→ 减小绝对值
robot param k_pitch_c -4.0

# 机器人倒下（无力）→ 增大绝对值
robot param k_pitch_c -6.0

# 第四步：调整阻尼
# 振荡剧烈 → 增大 pitch_rate 绝对值（更多阻尼）
robot param k_pitch_rate_c -2.0

# 第五步：调整位置/速度增益
# 机器人漂移后倒 → 增大绝对值
robot param k_position -1.5
robot param k_velocity_c -1.5

# 第六步：调整电流限制（如果力矩不足）
robot param current_limit 2500

# 第七步：调整 stiction（如果小角度不响应）
robot param stiction_ma 3500
# 或者禁用 stiction 排除干扰
robot param stiction_ma 0

# 第八步：调整故障保护（调试时放宽）
robot param fault_deg 30
```

### 3.3 保存与恢复

```text
# 调好后永久保存（断电不丢失，下次开机自动加载）
robot param save

# 不满意？恢复默认值
robot param reset

# 查看确认
robot param
```

---

## 4. 故障排查速查表

| 现象 | 可能原因 | 调整 |
|------|---------|------|
| 向前倒 | theta_eq 太小 | `robot param theta_eq 0.40` |
| 向后倒 | theta_eq 太大 | `robot param theta_eq 0.33` |
| 剧烈振荡 | K_pitch 或 K_pitch_rate 过大 | 减小 `k_pitch_c` 绝对值 |
| 缓慢漂移后倒 | K_position/K_velocity 不足 | 增大绝对值 |
| 小角度不响应 | stiction 不足或电流太小 | 增大 `stiction_ma` 或 `current_limit` |
| 使能后立刻 fault | theta_eq 与实际姿态差距大 | 调整 `theta_eq` 使 err≈0 |
| 调试时频繁 fault | fault_deg 太小或姿态离工作点太远 | 扶回安全姿态后重新 enable |
| 轮子单向转不停 | theta_eq 远离工作点 | 调整 `theta_eq` |
| 电流饱和（轮子啸叫） | 增益过大或电流限制太低 | 减小增益或增大 `current_limit` |

---

## 5. 参数存储原理

- **运行时**：参数存在 RAM 中，通过 mutex 保护，控制循环每周期读取快照
- **Flash 存储**：直接读写 STM32F407 最后 128KB 扇区（0x0E0000-0x0FFFFF），使用 magic+version+data+CRC32 格式
- **启动加载**：`main()` 启动时自动从 Flash 读取参数，校验 CRC 后覆盖代码默认值
- **恢复默认**：`robot param reset` 擦除 Flash 扇区，恢复为 `app_config.h` 中的 `#define` 值
- **安全性**：Flash 分区与固件分区独立，重新烧录固件不会擦除参数分区
- **备注**：未使用 NVS，因为 STM32F407 的 128KB 扇区超过 NVS 的 uint16_t sector_size 限制（65535）

---

## 6. 永久修改默认值

如果需要将调好的参数写入代码（重新编译后作为新默认值），修改 `src/app_config.h`：

```c
#define APP_ASCENTO_THETA_EQ_STAND_RAD 0.245f  /* 14° */
#define APP_ASCENTO_GAIN_C0_C  -6.0f           /* K_pitch 常数项 */
#define APP_ASCENTO_GAIN_C1_C  -2.0f           /* K_pitch_rate 常数项 */
#define APP_ASCENTO_GAIN_C2    -1.5f           /* K_position */
#define APP_ASCENTO_GAIN_C3_C  -1.5f           /* K_velocity 常数项 */
#define APP_WHEEL_CURRENT_LIMIT 2500           /* 电流上限 mA */
#define APP_ASCENTO_STICTION_CURRENT_MA 0.0f   /* 禁用 stiction */
```

然后重新编译烧录：
```bash
source zephyr-env.sh && west build --pristine --board dji_f407igh6_c
west flash
```
