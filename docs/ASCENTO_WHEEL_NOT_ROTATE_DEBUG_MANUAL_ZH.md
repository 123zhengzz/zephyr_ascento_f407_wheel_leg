# 倾斜时轮子不转：新手排查手册

这份手册对应一个很常见的现象：车体已经倾斜了，轮毂电机却不转。

先记住一句话：  
**先看控制器有没有允许输出，再看轮子有没有反馈，最后才怀疑电机本体。**

## 先搞清楚四个门

| 门 | 你要看什么 | 关掉时会发生什么 |
| --- | --- | --- |
| `robot enable` | `robot status` 里的 `enable=1` | 轮电流会被清零 |
| 轮反馈 | `motor wheel status all` | 没反馈时，控制器不会放心输出 |
| 倾斜保护 | `robot status` 里的 `fault=1` | 倾斜过大时，轮电流会主动归零 |
| 最终输出 | `robot status` 里的 `current=(...)` | 如果这里还是 0，前面一定有门没过 |

## 安全开始

机器人先架起来，别直接落地试。

```text
robot enable 0
motor debug stop
motor wheel stop
motor dm stop left
motor dm stop right
```

然后看状态：

```text
robot status
motor can status all
motor wheel status all
motor dm status all
```

你要先确认三件事：

1. `robot status` 能正常打印。
2. 左右轮 ID 是 `101` 和 `100`。
3. `motor wheel status` 不是一直 `no feedback`。

## 正常排查顺序

### 1. 先看 `enable`

如果 `robot status` 里 `enable=0`，那轮子不转是正常的。

先执行：

```text
robot enable 1
robot status
```

你要看到 `enable=1`，再继续下一步。

### 2. 再看 `wheels`

`robot status` 里还有个 `wheels` 字段。

- `wheels=1`：控制器认为轮子可以输出
- `wheels=0`：说明它主动挡住了输出

如果 `wheels=0`，先看是不是下面两种原因：

- 轮反馈断了
- 倾斜保护触发了

### 3. 看 `fault`

如果 `fault=1`，说明控制器已经进入保护。

这时轮子不转不是坏，是它故意不让你继续推。

处理方法很简单：

1. 把车体扶回接近直立。
2. 保持一小会儿。
3. 再看 `robot status`，确认 `fault` 是否清掉。

## 看轮反馈

轮子是否在线，主要看这条：

```text
motor wheel status all
```

重点看：

- 左轮 ID 是否是 `101`
- 右轮 ID 是否是 `100`
- `age` 是否很小
- 有没有 `no feedback`

经验上，`age` 一直很大，通常不是控制算法问题，而是：

- VESC 没上电
- ID 配错
- CAN2 接线有问题
- VESC 没开 Status 1 回传

## 什么时候算“保护正常”

你轻轻倾斜车体时，如果出现下面现象，通常说明保护在工作：

- `robot status` 里 `fault=1`
- `current=(0,0)`
- 红灯亮

这时不要继续加大倾角，先把车扶正。

## 什么时候像“真的出错了”

如果满足这几条：

- `enable=1`
- `fault=0`
- `motor wheel status` 有反馈
- 但 `current=(0,0)`

那就要重点查控制路径，而不是先怀疑电机坏了。

常见原因有三个：

1. 控制器没有真正读到 `enable`
2. 轮反馈超时，输出被清零
3. 代码路径和你以为的不一样

这个项目以前就踩过一个典型坑：  
**shell 里看起来已经 `robot enable 1` 了，但控制器没有真正用到这个状态。**  
所以排查时不要只看命令回显，一定要看 `robot status`。

## 一套最小实验

你可以按这个顺序做：

```text
robot enable 0
robot status
motor wheel status all

robot enable 1
robot status

轻轻前倾 5~10 度
robot status
motor wheel status left
motor wheel status right
```

你要观察三件事：

1. `enable` 有没有变成 `1`
2. `fault` 会不会变成 `1`
3. `current` 会不会开始变化

## 读数解释

| 现象 | 更像什么问题 |
| --- | --- |
| `enable=0` | 没允许输出 |
| `fault=1` | 倾斜保护触发 |
| `no feedback` | VESC / CAN2 / ID 问题 |
| `current=0,0` 但 `enable=1` | 控制路径没真正放行 |
| `motor_current` 有变化但轮子不动 | 机械或负载问题 |

## 用 `ascento` 日志看模型输出

串口日志里带 `[ascento]` 前缀的行是模型控制器自己打的状态。相比 `robot status` 的优势是：它直接显示模型内部算出的电流，不经过后续的门检查。

现在的日志格式是：

```
[ascento] active | pitch=0.03 err=-0.40 torque=-2.00 cur=-579/-579 L=0.100 K=-5.0/-0.9/-0.6/-0.9
```

重点看：
- `cur=%d/%d`：模型输出的左右轮电流（mA）。如果这里非零但 `robot status` 里 `current=(0,0)`，说明模型输出在后续被门关掉了（常见：`enable=0` 或 `wheels=0`）。
- `torque=%.2f`：平衡力矩（Nm），正 = 前进方向。
- `err=%.2f`：pitch 误差（rad），= 当前 pitch − theta_eq − 目标 pitch。

对比 `app` 日志（不带模块前缀，`pitch %.2f ... current %d/%d`）和 `ascento` 日志，能快速判断问题在模型内部还是在下游门逻辑。

## 对照代码看哪里

- [src/main.c](/home/h/code_leg/zephyr_ascento_f407_wheel_leg/src/main.c)  
  主循环，决定什么时候送轮电流。含 enable 和 wheel-feedback 两个门。
- [src/ascento_balance.c](/home/h/code_leg/zephyr_ascento_f407_wheel_leg/src/ascento_balance.c)  
  模型控制器，算轮电流和保护逻辑。`ascento` 日志从这里打出。
- [src/pid_balance_control.c](/home/h/code_leg/zephyr_ascento_f407_wheel_leg/src/pid_balance_control.c)  
  活跃的控制模块（`control_init`, `control_set_enable`, `control_publish_status` 等都在这里，不是 control.c）。
- [src/dji_m3508.c](/home/h/code_leg/zephyr_ascento_f407_wheel_leg/src/dji_m3508.c)  
  解析 VESC 反馈，判断轮子有没有在线。
- [src/control.h](/home/h/code_leg/zephyr_ascento_f407_wheel_leg/src/control.h)  
  状态字段定义，`robot status` 就靠它打印。
- [src/app_config.h](/home/h/code_leg/zephyr_ascento_f407_wheel_leg/src/app_config.h)  
  所有配置参数，包括增益、电流限制、几何尺寸等。

注意：`src/control.c` **没有被编译**（不在 CMakeLists.txt 中），所有控制函数实际来自 `pid_balance_control.c`。

## 一句话总结

轮子不转时，排查顺序永远是：

1. `enable` 有没有开
2. 轮反馈有没有来
3. `fault` 有没有触发
4. 最后才看电机和机械本体

## 进阶：常见「行为异常」排查

### 轮子方向不随角度改变

**现象**：前倾、后仰、直立，轮子都往同一个方向转。

**原因**：`theta_eq`（平衡参考角）离实际姿态太远。比如 theta_eq=21°（后仰），机器人处于直立 0°，误差始终为负，力矩方向不变。

**检查**：
```text
# 看 [ascento] active 日志的 err= 字段
# err = pitch − theta_eq
# 如果 err 始终同号 → theta_eq 与实际姿态不匹配
```

**解决**：架空测试时 `APP_ASCENTO_THETA_EQ_STAND_RAD` 应为 0。落地后再恢复。

### 关节电机没有力

**现象**：用手推拉腿，完全没有阻力。`[ascento]` 日志中 `L=` 始终是 0.060（最短腿长）。

**原因**：DM4340 电机未使能。常见于：
- 用过 `motor dm stop` 等 debug 命令后（debug 期间不发 MIT 命令，电机超时失能）
- 刚上电时（如果使能命令发送和 MIT 命令间隔太大）

**解决**：按复位键重启，让主循环走一遍完整使能流程（`dm4340_enable` → `send_control_joints`）。正常情况关节应有明显阻力（KP=50 对应约 50 N/m 刚度）。

### 架空测试 → 落地测试切换清单

架空测试和落地接触需要不同的平衡参考角。切换时改 [src/app_config.h](src/app_config.h) 中的这一个值：

```c
// 架空测试（当前）
#define APP_ASCENTO_THETA_EQ_STAND_RAD 0.0f

// 落地接触（测试完方向后改为此值）
#define APP_ASCENTO_THETA_EQ_STAND_RAD 0.367f  // atan(0.040/0.098) ≈ 21°
```

**为什么不同**：COM 在轮轴后方 40mm，落地时 gravity torque 会让机器人自然向后仰。theta_eq 必须匹配这个角度，否则 LQR 会持续输出单向力矩。架空时 theta_eq=0 能让方向验证直观（前倾→轮子前转，后仰→轮子后转）。

### 架空测试轮子不转（电流太小）

**现象**：关节有力，模型活跃（`[ascento] active`），但倾斜时轮子不动。

**原因**：theta_eq=0 后，5° 倾斜只有 ~126 mA 电流。VESC 有最小电流阈值（~0.5-1A），低于阈值不驱动电机。

**解决**：当前代码已加 3x 电流缩放（`APP_ASCENTO_WHEEL_CURRENT_SCALE 3.0f`）。5° 倾斜 → 378 mA，10° → 759 mA，轮子应明显转动。

如果仍然不转，逐步排查：
```text
# 1. 确认模型活跃且输出非零电流
#    [ascento] active | cur=-500/-500  ← 非零

# 2. 确认 app 日志 current 也非零（门没关）
#    [app] current -500/-500

# 3. 如果 app 日志 current=0/0，检查 robot status
robot status
#    看 enable=1, wheels=1, fault=0

# 4. 如果 current 非零但轮子不转，检查 VESC
motor wheel status all
#    看 cmd 和 motor_current 是否匹配
```

### 侧倾时轮子同向转动（IMU 轴怀疑交换）

**现象**：左倾时两轮都向前转，右倾时两轮都向后转（方向不随倾角换向，且两轮同向）。

**原因**：两轮同向 = pitch 控制器在响应。说明侧倾被 IMU 读成了 pitch 变化。最可能的原因是 BMI088 芯片坐标系与机器人坐标系不一致（芯片 X 轴对应机器人侧向，Y 轴对应机器人前后）。

**验证**（不需要改代码，用串口即可）：
```text
# 1. 机器人直立，记基准
robot status
# 记下 pitch= 和 roll= 的值

# 2. 纯前倾 15°（保持左右水平）
robot status
# 正常：pitch 变化 ~15°, roll 不变
# 异常（轴交换）：pitch 几乎不变, roll 变化 ~15°

# 3. 纯左倾/右倾 15°（保持前后不倾）
robot status
# 正常：roll 变化 ~15°, pitch 不变
# 异常（轴交换）：pitch 变化 ~15°, roll 几乎不变
```

**判断**：

| 前倾时谁变 | 侧倾时谁变 | 结论 |
|------------|------------|------|
| pitch 变 | roll 变 | IMU 轴正确，问题在其他地方 |
| roll 变 | pitch 变 | **pitch/roll 轴交换了** → 需修正 BMI088 驱动或交换映射 |

**临时验证修复**（确认轴交换后）：修改 [src/ascento_balance.c](src/ascento_balance.c) 的 IMU 字段映射，用 `roll_deg` 当 pitch、`pitch_deg` 当 roll、`gx_dps` 当 pitch rate。验证通过后再决定永久修复方案。

> **2026-05-05 已修复**：根本修复在 [src/bmi088.c](src/bmi088.c) 的 `update_attitude()` 和 `bmi088_init()` 中。修改内容：
> - roll ← `atan2(ax_g, az_g)` + `gy_dps` 陀螺积分（原为 ay + gx）
> - pitch ← `atan2(-ay_g, sqrt(ax²+az²))` + `gx_dps` 陀螺积分（原为 ax + gy）
> 
> 修复后所有下游消费者（Ascento 模型、app 日志）的 pitch/roll 均为机器人坐标系下的正确值。

