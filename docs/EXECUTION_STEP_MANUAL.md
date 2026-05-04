# 执行步骤手册：Ascento 轮腿机器人

> 版本：2026-05-05（架空测试完成，进入站立测试阶段）
> 用途：从当前状态到自平衡站立，每一步的具体操作和验证标准

---

## 0. 当前状态总览

```
四连杆关节角↔腿长映射  ✅ 已验证（全范围 ≤5mm）
连杆长度 (L1~L23)     ✅ 已确认
左右 offset 标定       ✅ 4.892 / 2.098 rad（3 点平均）
关节软限位             ✅ 左 2.055~2.87，右 1.00~1.80
IMU 四通道符号         ✅ pitch×(−1), gy×(−1), roll×(+1), gz×(+1)
IMU pitch/roll 轴      ✅ 已修复（bmi088.c 轴映射交换，2026-05-05）
轮子方向（修复后）     ✅ 左 −1, 右 +1（2026-05-05 架空验证通过）
质量/几何参数          ✅ 已写入代码
I_wheel 单轮惯量       ✅ 7.7×10⁻⁵ kg·m²（计算值）
I_pitch 车体惯量       ⚠ 占位值 0.060 kg·m²（待实测）
LQR 增益（多项式调度） ✅ 5 锚点全部闭环稳定，已接入运行时
目标腿长               ✅ 站立高度 0.115m（APP_ASCENTO_LEG_LENGTH_STAND_M）
theta_eq（架空测试）   ✅ 0.0f（直立平衡，方向验证用）
theta_eq（落地）       ✅ 0.367f（前倾 21°，COM 在轮轴后方需前倾补偿，见 REPORT_THETA_EQ_ANALYSIS.md）
电流缩放               ✅ 1.0x（落地测试用 LQR 原始增益；架空测试用 3.0x 突破 VESC 阈值）
Pitch 保护             ✅ fault 45°, recover 15°
DM 使能（debug 后）    ✅ 退出 debug 自动重新使能
架空测试               ✅ 已完成（2026-05-05）
站立测试               ← 当前步骤
```

**模型已活跃，LQR 增益调度已接入。当前任务：架空测试验证方向，然后落地。**

---

## 阶段 1：测量 I_pitch（车体俯仰转动惯量）

### 为什么这是下一步

LQR 增益对 I_pitch 敏感。I_pitch 差 30%，增益可能差 20-50%。先用占位值算增益再调参会浪费大量时间。

### 方法 A：CAD 质量属性（推荐，最快）

如果有 SolidWorks/Fusion 360 模型：
1. 打开装配体
2. 设置材料密度
3. 运行 Mass Properties
4. 选择轴：过轮轴中心，左右方向（pitch 轴）
5. 读取 `Ixx` 或 `Izz`（取决于 CAD 坐标），单位转为 kg·m²
6. 直接填入 `APP_ASCENTO_BODY_PITCH_INERTIA_KG_M2`

### 方法 B：摆动法（无 CAD 时）

**所需工具**：细绳/杆、秒表、卷尺

**步骤**：
1. 把机器人悬挂（挂钩或细绳），悬挂轴平行于轮轴
2. 测量悬挂轴到质心的距离 `d`（米）
   - 质心高度已知 0.098m，如果悬挂点在车体顶部，需要重新量悬挂点到质心距离
3. 小角度（<15°）推一下，让机器人像钟摆一样摆动
4. 用秒表测 10 次完整摆动的时间，取平均得周期 `T`（秒）
5. 计算：
   ```
   I_pivot = m × g × d × (T / (2π))²
       m = 2.315 kg（总质量）
       g = 9.81 m/s²
   I_com = I_pivot - m × d²
   I_pitch_about_wheel = I_com + m_body × (h_com² + x_com²)
   ```
6. 如果悬挂轴就是轮轴本身，`I_pivot` 直接就是所需的 I_pitch

### 方法 C：盒体粗估（最后手段）

```
h = 车体高度（m），量一下
d = 车体前后深度（m），量一下
m_body = 1.7514 kg
I_com ≈ m_body × (h² + d²) / 12
I_pitch ≈ I_com + m_body × (0.098² + 0.040²)
```

### 验证标准

- [ ] `APP_ASCENTO_BODY_PITCH_INERTIA_KG_M2` 从占位值 0.060 更新为实测/计算值
- [ ] 编译通过，0 warnings

---

## 阶段 2：计算 LQR 增益表

### 前置条件

- [x] `I_pitch` 已更新（阶段 1 完成）
- [x] 所有其他物理参数已在 `app_config.h` 中

### 步骤

**第 1 步：测量 5 个腿长点的质心高度**

```
安全命令（测量前）：
  robot enable 0
  motor debug stop
```

对每个腿长点，手动把关节移动到目标位置，用尺子量地面到质心标记的高度：

| 采样点 | 目标腿长 (m) | 左关节角 (rad) | 右关节角 (rad) | 质心离地高度 (m) |
|--------|-------------|---------------|---------------|------------------|
| min    | 0.064       | 2.0251        | 1.8324        | 待测 |
| low    | 0.090       | 待反解        | 待反解        | 待测 |
| mid    | 0.1345      | 2.3806        | 1.4353        | 待测 |
| high   | 0.160       | 待反解        | 待反解        | 待测 |
| max    | 0.205       | 2.9028        | 0.9676        | 待测 |

对于 low 和 high 腿长，用串口命令反解关节角：
```
motor dm pos left <angle>
motor dm pos right <angle>
```
从四连杆逆解获得角度（用 `fb_joint_from_leg_length()` 计算，或者直接用 min/mid/max 的关节角线性插值作为初值）。

计算 `L_com = 质心离地高度 - 0.030`（轮半径）。

**第 2 步：运行 LQR 脚本**

创建 Python 脚本 `tools/compute_lqr_gains.py`：

```python
#!/usr/bin/env python3
"""Compute LQR gain table for Ascento wheel-leg robot."""
import numpy as np
from scipy.linalg import solve_continuous_are

# ===== Physical parameters (update I_b after phase 1) =====
m_b   = 2.315       # total mass kg
I_b   = 0.060       # body pitch inertia kg*m² — UPDATE AFTER PHASE 1
I_w   = 0.000077    # single wheel inertia kg*m²
r     = 0.030       # wheel radius m
g     = 9.81

# ===== Fill after measuring COM heights =====
leg_points = [
    # (L_leg, L_com)
    (0.064,  0.000),   # min  — FILL ME
    (0.090,  0.000),   # low  — FILL ME
    (0.1345, 0.000),   # mid  — FILL ME
    (0.160,  0.000),   # high — FILL ME
    (0.205,  0.000),   # max  — FILL ME
]

# ===== LQR weights =====
Q = np.diag([500.0, 50.0, 10.0, 30.0])
R = np.array([[0.1]])

print(f"{'L_leg':>8s}  {'L_com':>8s}  {'K_pitch':>10s}  {'K_prate':>10s}  {'K_pos':>10s}  {'K_vel':>10s}")
print("-" * 70)

for L_leg, L_com in leg_points:
    a = m_b * L_com**2 + I_b
    b = m_b * L_com * r
    c = m_b * r**2 + 2.0 * I_w
    d = m_b * g * L_com
    det = a * c - b**2

    A = np.array([
        [0, 1, 0, 0],
        [c*d/det, 0, 0, 0],
        [0, 0, 0, 1],
        [-b*d/det, 0, 0, 0]
    ])
    B = np.array([[0], [-(c+b)/det], [0], [(a+b)/det]])

    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.solve(R, B.T @ P)

    eigvals = np.linalg.eigvals(A - B @ K)
    stable = "STABLE" if np.max(np.real(eigvals)) < 0 else "UNSTABLE!"

    print(f"{L_leg:8.4f}  {L_com:8.4f}  {K[0,0]:10.3f}  {K[0,1]:10.3f}  {K[0,2]:10.3f}  {K[0,3]:10.3f}  {stable}")
```

运行：
```bash
pip install numpy scipy
python3 tools/compute_lqr_gains.py
```

**第 3 步：检查输出**

- [ ] 所有 5 个点都标记 STABLE
- [ ] 增益随腿长单调变化（腿越长增益越小是正常的）
- [ ] 没有增益反号
- [ ] 没有突变（相邻点增益变化 <30% 是健康的）

如果某点 UNSTABLE：调大 Q 矩阵的权重（先调 Q[0,0]=pitch 权重 500→1000）。

**第 4 步：填入代码**

先用 mid 点（0.1345m）的增益作为单一增益填入 `app_config.h`：

```c
#define APP_ASCENTO_K_PITCH      <脚本输出的 K_pitch>
#define APP_ASCENTO_K_PITCH_RATE <脚本输出的 K_prate>
#define APP_ASCENTO_K_POSITION   <脚本输出的 K_pos>
#define APP_ASCENTO_K_VELOCITY   <脚本输出的 K_vel>
```

K_yaw_rate 和 K_roll_to_leg 先保持 0，后续再加。

---

## 阶段 3：代码准备（已完成 ✔）

> 以下配置已在第四轮修改中完成，无需再改。列在这里作为记录。

### 3.1 安全限制（已配置）

```c
#define APP_WHEEL_CURRENT_LIMIT 1500   // mA 安全钳位
#define APP_WHEEL_CURRENT_SAFE 800     // mA 手动调试限制
```

### 3.2 模型参数（已配置）

```c
#define APP_ASCENTO_PARAMS_CALIBRATED 1        // 参数已标定
#define APP_USE_ASCENTO_BALANCE_CONTROLLER 1   // 使用 Ascento 模型
#define APP_ASCENTO_THETA_EQ_STAND_RAD 0.0f    // 架空测试：直立平衡
#define APP_ASCENTO_WHEEL_CURRENT_SCALE 3.0f   // 架空测试：3x 电流
```

### 3.3 落地前切换（架空测试完成后执行）

在 [src/app_config.h](src/app_config.h) 中改两个值：

```c
// 改前（架空测试用）
#define APP_ASCENTO_THETA_EQ_STAND_RAD 0.0f
#define APP_ASCENTO_WHEEL_CURRENT_SCALE 3.0f

// 改后（落地测试用）
#define APP_ASCENTO_THETA_EQ_STAND_RAD 0.367f     // COM 偏心力矩平衡角
#define APP_ASCENTO_WHEEL_CURRENT_SCALE 1.0f      // 使用原始 LQR 增益
```

改完重新编译烧录。

### 3.4 编译烧录

```bash
cd /home/h/code_leg/zephyr_ascento_f407_wheel_leg/build
ninja
# 烧录 .elf 或 .bin 到 F407
```

### 3.5 验证启动

启动后连接串口，确认：
```
robot status
```
- [ ] 电池电压正常（6S 满电 ~25V，不低于 20V）
- [ ] IMU 数据正常（pitch/roll 读数合理）
- [ ] 关节电机有反馈（`motor dm status all`）
- [ ] 轮电机有反馈（`motor wheel status all`，无 "no feedback"）
- [ ] 没有 fault（`fault=0`）

---

## 阶段 4：架空测试（轮子不触地）← 当前步骤

### 4.0 前置检查

**固件版本确认**：必须使用第四轮修改后的固件。串口启动后确认：
```text
[ascento] model blocked: params_ready=1 enable_req=0 wheel_fb_ok=1 calib=1
```
如果 `params_ready=0` 或 `wheel_fb_ok=0`，先排查 CAN 总线。

**关键参数确认**（当前固件默认值）：
| 参数 | 值 | 作用 |
|------|------|------|
| `APP_ASCENTO_THETA_EQ_STAND_RAD` | 0.0f | 平衡参考 = 直立 |
| `APP_ASCENTO_WHEEL_CURRENT_SCALE` | 3.0f | 电流放大 3x，突破 VESC 阈值 |
| `APP_WHEEL_CURRENT_LIMIT` | 1500 mA | 安全钳位 |
| `APP_PITCH_FAULT_DEG` | 45.0f | 倾斜保护阈值 |
| 目标腿长 | 0.115m | 站立高度 |

### 4.1 架设机器人

1. 把机器人架起来（支架、泡沫块或手持），**轮子悬空**，车身可自由倾斜
2. 确认轮子无机械阻碍，可自由旋转
3. 连上串口：
   ```bash
   ./scripts/serial.sh
   ```

### 4.2 上电检查

上电后，**不要用任何 debug 命令**（`motor dm stop` 等会触发 DM 失能）。直接检查：

```text
# 1. 确认所有反馈在线
motor wheel status all     # 左右轮 age < 50ms，无 "no feedback"
motor dm status all        # 左右关节有位置读数
motor can status all       # CAN1/CAN2 状态正常
robot status               # fault=0, wheels=?, enable=?

# 2. 确认模型处于等待状态
#    串口应有 ~1Hz 的 "model blocked" 日志
#    enable_req=0 是正常的（还没开使能）
```

**新增：IMU 轴验证（在开控制之前）**（2026-05-05 代码已修复，但仍需烧录后验证一次）

```text
# 1. 直立，记基准
robot status          # 记下 pitch=  roll= 的值

# 2. 纯前倾 10-15°（保持左右水平），读
robot status
# 预期（修复后）：pitch 变化 ~10-15°, roll 不变

# 3. 纯左/右倾 10-15°（保持前后不倾），读
robot status
# 预期（修复后）：roll 变化 ~10-15°, pitch 不变
```

**如果 `motor wheel status` 显示 `no feedback`**：
- VESC 没上电 → 检查轮电机电源
- CAN2 接线问题 → 检查 CAN 线缆
- VESC 没开 Status1 回传 → 用 VESC Tool 配置

**如果 `motor dm status` 无反馈**：
- DM 电机没上电 → 检查关节电机电源（24V）
- CAN1 接线问题 → 检查 CAN 线缆

### 4.3 开启控制

```text
robot enable 1
```

**立刻观察串口**，你应该在 ~1 秒内看到：

```
[ascento] active | pitch=0.01 err=0.01 torque=0.05 cur=15/15 Lt=0.115 L=0.114 jL=2.28 jR=1.55 K=-5.0/-0.98/-0.63/-0.97
```

**逐字段检查**：

| 字段 | 期望值 | 如果不正常 |
|------|--------|-----------|
| `pitch` | −0.1 ~ +0.1 rad（架空） | 如果偏很大，theta_eq 可能不是 0（架空测试用 0） |
| `err` | −0.1 ~ +0.1 rad | 落地测试时，直立时 err≈+0.367（theta_eq=−0.367 前倾） |
| `cur` | 接近 0（直立时） | 如果始终为 0，检查 scale 是否已编译进去 |
| `Lt` | **0.115**（目标腿长） | 如果不是 0.115，固件用的还是旧的 DEFAULT_M |
| `L` | 跟随 Lt，最终 ~0.115 | L 是反馈腿长。如果 L 一直偏离 Lt，DM 没走到目标位置 |
| `jL/jR` | ~2.28 / ~1.55 rad | IK 对 0.115m 解算的关节目标角 |
| `K` | 约 −5.0/−0.98/−0.63/−0.97 | 站立高度 0.115m 的增益值 |

**如果看不到 `[ascento] active` 日志**：

```text
# 可能原因 1：门封锁
# 串口会显示 "model blocked: ..."，看各字段
# enable_req=0 → robot enable 1 没生效，再发一次
# wheel_fb_ok=0 → VESC 反馈丢失，检查 CAN2

# 可能原因 2：pitch fault 触发
# 串口会显示 "faulted | pitch=... err_deg=..."
# 把机器人扶正，等待 ~0.2 秒恢复

# 可能原因 3：enable 命令超时 (700ms)
# robot enable 1 后立刻 robot status 确认 enable=1 是否保持
```

### 4.4 关节力反馈验证

在 enable=1 状态下：

1. 用手轻轻推一条腿，**应该感到明显阻力**
2. 如果完全无力（腿可以被随意推动）：
   - 可能用了 debug 命令导致 DM 失能 → **按复位键**，等启动后直接 `robot enable 1`
   - 检查 `motor dm status` 看反馈是否正常

### 4.5 轮子方向测试

**测试前理解**：
- theta_eq=0，所以直立时误差≈0，轮子不转（或很慢）
- 3x 电流缩放：5° 倾斜 → ~380 mA，10° 倾斜 → ~760 mA
- 倾斜角度不够 → 电流不够 → 轮子可能不动。**建议至少倾斜 10°**

**测试步骤**：

```text
# 确认当前状态
robot status
# enable=1, wheels=1, fault=0, current=(接近 0)

# === 测试 1：直立 ===
# 保持机器人直立
# 观察 [ascento] active 日志：cur 应接近 0
# 观察轮子：不转或极慢

# === 测试 2：前倾 10-15° ===
# 缓慢前倾机器人（模拟向前倒）
# 观察 [ascento] active：err 应该变为正值，cur 应该变为正值
#   例：pitch=0.18 err=0.18 torque=0.90 cur=260/260 (1x) → 780/780 (3x)
# 观察轮子：向前转（把车体推回直立）
# app 日志：current 应跟随 ascento 的 cur 值

# === 测试 3：后仰 10-15° ===
# 缓慢后仰机器人（模拟向后倒）
# 观察 [ascento] active：err 应该变为负值，cur 应该变为负值
#   例：pitch=-0.18 err=-0.18 torque=-0.90 cur=-260/-260 (1x) → -780/-780 (3x)
# 观察轮子：向后转（把车体推回直立）

# === 测试 4：左右倾 ===
# 左倾/右倾不应引起轮子大幅动作（roll 补偿 K_roll_to_leg=0）
```

**每完成一个测试姿态，立刻回到直立位**，不要让机器人长时间处于大倾角（可能触发 fault）。

### 4.6 方向判断

| 现象 | 结论 | 行动 |
|------|------|------|
| 前倾→轮子前转，后仰→轮子后转 | **方向正确 ✓** | 进 4.7，准备落地 |
| 前倾→轮子后转，后仰→轮子前转 | **IMU pitch 符号反了** | `APP_ASCENTO_IMU_PITCH_SIGN` 改为 1.0f（当前 −1.0f），重新编译烧录 |
| 直立→轮子一直向一个方向转 | **theta_eq 不对** | 检查固件是否已更新。直立时 err 应接近 0 |
| 轮子不转，且 `cur` 接近 0 | **倾斜不够** | 增大倾角到 15-20°，或临时调大 `APP_ASCENTO_WHEEL_CURRENT_SCALE` 到 5.0f |
| 轮子不转，但 `cur` 非零，`app` 的 `current=0/0` | **门关掉了** | `robot status` 查 enable=, wheels=, fault= |
| 轮子不转，`cur` 非零，`app` 的 `current` 也非零 | **VESC/CAN2 问题** | `motor wheel status all` 看 cmd vs motor_current |
| 轮子抖动/暴走 | **增益太大或 IMU 噪声** | 减小 `APP_ASCENTO_WHEEL_CURRENT_SCALE` 到 1.0，如果还抖减半增益 |
| 关节完全无力 | **DM 失能** | 按复位键重试 |

### 4.7 测试完成

```text
robot enable 0     # 关闭控制
robot status       # 确认 enable=0, current=(0,0)
```

**如果方向确认正确**，接下来：
1. 修改 `APP_ASCENTO_THETA_EQ_STAND_RAD` 从 0.0f → 0.367f
2. 修改 `APP_ASCENTO_WHEEL_CURRENT_SCALE` 从 3.0f → 1.0f
3. 重新编译烧录
4. 进入阶段 5（手扶落地测试）

---

## 阶段 5：手扶落地测试

### 目标

在有人扶着的情况下，感受机器人的扶正力，确认方向和力度合理。

### 步骤

1. 机器人放在平整地面
2. 手扶车体顶部，保持机器人基本直立
3. 串口输入 `robot enable 1`
4. 轻轻推车体前倾 5-10°，感受轮子是否产生向前的力
5. 如果方向正确，松开一点手，看机器人是否有主动回到直立的趋势
6. 随时准备断电或 `robot enable 0`

### 调参

如果扶正力太小：
- `K_PITCH` 增大 30%
- `K_PITCH_RATE` 增大 30%

如果抖动/振荡：
- `K_PITCH_RATE` 减半（阻尼不够）
- 如果还振，`K_PITCH` 也减半

**每次只改一个参数，改完重新编译烧录。**

---

## 阶段 6：短时自立

### 目标

松手 1-2 秒，机器人能自己站住不倒。

### 步骤

1. 手扶机器人直立
2. `robot enable 1`
3. 松手，数 1-2 秒
4. 如果机器人开始倒，立刻扶住
5. `robot enable 0`

### 调试

如果前倾加速倒：
- `K_PITCH` 偏小，增大 50%
- 或 `K_PITCH_RATE` 偏小，增大 50%

如果前后振荡（摆来摆去）：
- `K_PITCH_RATE` 太大，减半
- 或 `K_PITCH` 太大，减半

如果朝一个方向匀速倒：
- 检查 `APP_ASCENTO_BODY_COM_FORWARD_OFFSET_M`（质心前后偏移），可能需要微调
- 或者加 `K_POSITION`（目前是 0）

**安全规则**：
- 松手前手指悬在串口回车键上
- 从头到尾保持可随时断电
- 振荡不停 → 立刻扶住 + `robot enable 0`

---

## 阶段 7：稳定站立

### 目标

机器人持续站立，不需要人扶。

### 调参顺序

站稳后逐步加功能：

1. **K_PITCH + K_PITCH_RATE**（已有，微调到最佳）
2. **K_VELOCITY**（阻尼平移漂移）→ 从脚本输出值的 50% 开始
3. **K_POSITION**（固定位置）→ 从脚本输出值的 50% 开始
4. **K_YAW_RATE**（防自转）→ 从 0.5 开始
5. **K_ROLL_TO_LEG**（roll 补偿）→ 从 0.005 开始

每加一个新增益，先减半测试，确认不引入振荡再逐步调大。

---

## 阶段 8：增益调度（变腿长）

### 前置条件

稳定站立后，才需要让腿长变化。

### 步骤

1. 用阶段 2 的脚本输出 5 组增益
2. 在 `ascento_balance_params_t` 中加增益表：
   ```c
   float gain_table_leg_lengths[5];
   float gain_table_K[5][4];  // [i][0]=pitch, [1]=prate, [2]=pos, [3]=vel
   int    gain_table_count;
   ```
3. 在 `ascento_balance_update()` 中根据当前平均腿长做线性插值
4. 先用 mid 腿长（0.1345m）验证与硬编码增益一致
5. 再测试 min 和 max 腿长下的稳定性

---

## 快速命令参考

完整串口命令手册：[SERIAL_COMMAND_REFERENCE.md](SERIAL_COMMAND_REFERENCE.md)

```bash
# 编译烧录
./scripts/flash_dap.sh build
./scripts/flash_dap.sh        # 只烧录（不编译）

# 连接串口
./scripts/serial.sh

# 安全失力（测量用）
robot enable 0
motor debug stop

# 状态查看（三件套）
robot status                  # 控制器状态
motor wheel status all        # 轮电机状态
motor dm status all           # 关节电机状态

# 轮子方向测试
motor wheel current left 100 3000
motor wheel current right 100 3000

# 关节微动
motor dm nudge left 0.05
motor dm nudge right -0.03

# 恢复控制
robot enable 1
```

---

## 故障排查

| 现象 | 可能原因 | 检查 |
|------|---------|------|
| `robot enable 1` 没反应 | PARAMS_CALIBRATED=0 | 确认阶段 3.2 已改为 1 |
| 轮子不动 | 电流限制太低 | 检查 APP_WHEEL_CURRENT_SAFE |
| 轮子方向反 | IMU 符号错 | 回阶段 4 重新确认 |
| 烧录后黑屏 | 固件 crash | 检查编译是否有 warning，串口是否有 panic 信息 |
| 关节暴走 | 软限位设错 | 检查 LEFT/RIGHT_JOINT_AT_MIN/MAX_LEG_RAD |
| 编译报错 | zephyr-env 未 source | `source ./zephyr-env.sh` |

---

## 关联文档

- [ASCENTO_README_ZH.md](ASCENTO_README_ZH.md) — 完整参数和测量方法
- [CURRENT_TASKS.md](CURRENT_TASKS.md) — 所有参数状态
- [REPORT_FOUR_BAR_ACTIVATION.md](REPORT_FOUR_BAR_ACTIVATION.md) — 四连杆标定详情
- [REPORT_SHORT_LEG_CORRECTION.md](REPORT_SHORT_LEG_CORRECTION.md) — 短腿数据修正过程
- [DEBUG_LEARNING_JOURNAL.md](DEBUG_LEARNING_JOURNAL.md) — 符号约定发现过程等调试经验
