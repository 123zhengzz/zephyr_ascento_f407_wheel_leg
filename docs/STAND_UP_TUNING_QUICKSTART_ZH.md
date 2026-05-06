# Ascento 站立调参现场指令

适用：F407 C 板、DM4340 四连杆腿、VESC/M3508 轮、`APP_USE_ASCENTO_BALANCE_CONTROLLER=1`。

目标：先确认安全门都打开，再调 `theta_eq`，然后调轮子力矩和阻尼，让机器人能被扶着站住，再逐步放手。

## 0. 安全约定

现场调参必须有人扶机器人。第一次使能时轮子离地或半接地，手放在电源开关附近。

紧急停止：

```text
robot enable 0
motor wheel stop
```

没有确认稳定前不要执行 `robot param save`。

## 1. 连接和恢复基准

```bash
./scripts/serial.sh
```

串口里执行：

```text
robot enable 0
motor wheel stop
robot param reset
robot param
```

`robot param reset` 会恢复代码默认参数，并清除 Flash 中保存的旧调参结果。如果你要保留旧参数，只执行 `robot param` 查看，不要 reset。

当前建议初值：

```text
robot param theta_eq 0.245
robot param current_scale 1.0
robot param current_limit 1500
robot param stiction_ma 800
robot param stiction_start 0.10
robot param stiction_full 1.50
robot param k_pitch_c -5.2
robot param k_pitch_rate_c -1.5
robot param k_position -0.63
robot param k_velocity_c -1.15
robot param sync_gain 0
```

说明：先把 `sync_gain` 设为 0，避免左右轮同步补偿干扰站立判断。站稳后再打开。

## 2. 上电检查

```text
motor can status all
motor wheel status all
motor dm status all
robot status
```

合格标准：

| 项目 | 正常值 |
|------|--------|
| CAN | `error-active`，`tx_err/rx_err` 不持续增加 |
| VESC/M3508 | 左右轮都有反馈，`age < 50ms` |
| DM4340 | 左右关节都有反馈，`age < 20ms`，`err=0` |
| robot | `fault=0`，pitch/roll 数值稳定 |

如果 `motor wheel status all` 显示 `no feedback` 或 `age` 很大，先不要使能平衡。检查 VESC ID、CAN2、VESC Status 1 回传频率。

## 3. IMU 方向检查

机器人直立，执行：

```text
robot status
```

然后手扶机器人前倾约 10 到 15 度，再执行：

```text
robot status
```

合格标准：前倾时 `pitch` 增大，左倾时 `roll` 增大。现在 `robot status` 显示的是模型坐标，`pitch` 前倾为正。

如果前倾时 `pitch` 变负，先不要调参数，检查 `APP_ASCENTO_IMU_PITCH_SIGN` 或 BMI088 轴映射。

## 4. 轮子方向检查

单轮 debug 命令是 VESC 原始电流，不是模型的“前进正方向”。当前配置下：

```text
motor wheel current left 100 1000
motor wheel current right 100 1000
```

预期：左轮原始 `+100mA` 和右轮原始 `+100mA` 的物理方向可能相反，这是正常的。

检查模型前进方向，用这一条：

```text
motor wheel pair -100 100 1000
```

预期：两轮共同产生机器人前进方向的轮端力矩。如果不是，优先检查 `APP_WHEEL_LEFT_FORWARD_CURRENT_SIGN` 和 `APP_WHEEL_RIGHT_FORWARD_CURRENT_SIGN`。

## 5. 关节锁腿检查

关闭平衡时，主循环仍会让关节锁在站立腿长附近。

```text
robot enable 0
robot status
motor dm status all
```

预期 `robot status` 里的关节目标大约：

```text
joint=(2.28,1.55)
```

用手推腿应有明显阻力。如果腿完全软，执行：

```text
motor dm enable left
motor dm enable right
```

如果 debug 后关节又变软，复位控制板或重新执行上面的 enable。

## 5.1 左右腿机械调平

如果右腿或左腿实际略长，先做机械调平或重新检查四连杆标定，不使用软件腿长 trim。腿长不平会带来静态 roll 偏置，进而干扰 pitch 平衡判断。

判断方法：

```text
robot enable 0
robot status
```

扶住机器人，两轮接地、腿在站立高度附近，看 `roll`。左倾时 `roll` 变大。调机械或检查关节零点后，让直立附近 `roll` 接近 0，再继续做 pitch 调参。

如果 `roll` 长期偏在 2 到 3 度以上，先不要追 `theta_eq` 或增益。先确认：

| 项目 | 检查 |
|------|------|
| 左右腿机械长度 | 两侧连杆、轮轴到机身参考点高度是否一致 |
| DM 零点 | 左右关节反馈是否对应同一机械姿态 |
| 四连杆标定 | `joint=(2.339,1.573)` 附近时两腿实际长度是否一致 |

## 6. 第一次使能

把机器人扶在接近平衡角的位置，建议机身前倾约 14 度。然后：

```text
robot enable 1
robot status
```

正常状态应为：

```text
enable=1 wheels=1 fault=0
```

同时观察日志：

```text
[ascento] active | pitch=... err=... torque=... cur=.../...
```

关键判断：

| 现象 | 说明 |
|------|------|
| `wheels=0` | 轮反馈丢失、未 enable、或模型被挡住 |
| `fault=1` | 机身 pitch 超过保护角：前倾约 30 deg、后仰约 22 deg，扶回安全角度后重新 enable |
| `cur=0/0` | 误差太小、stiction 太低，或模型未 active |
| `cur` 非零但轮不动 | VESC 电流阈值、VESC 未使能、动力电不足 |

清 fault：

```text
robot enable 0
robot enable 1
```

## 7. 调 theta_eq

`theta_eq` 是最先调的参数。单位是 rad，前倾为正。

角度换算：

| 前倾角 | theta_eq |
|--------|----------|
| 10 deg | 0.175 |
| 12 deg | 0.209 |
| 14 deg | 0.245 |
| 16 deg | 0.279 |
| 18 deg | 0.314 |
| 20 deg | 0.349 |

扶住机器人，找到它接近自然平衡的位置，看 `robot status` 的 `pitch`，把 `theta_eq` 设成接近这个角度。

```text
robot param theta_eq 0.245
```

行为判断：

| 表现 | 调整 |
|------|------|
| 使能后向前倒 | 增大 `theta_eq`，每次加 `0.02` |
| 使能后向后倒 | 减小 `theta_eq`，每次减 `0.02` |
| 轮子一直单向转 | `theta_eq` 离实际平衡点太远 |
| 一使能就 fault | `theta_eq` 和当前姿态差太大，扶到接近平衡角再使能 |

## 8. 调力矩和阻尼

先只动这四个参数：

```text
robot param k_pitch_c -5.2
robot param k_pitch_rate_c -1.5
robot param stiction_ma 800
robot param current_limit 1500
```

按现象调：

| 表现 | 调整命令 |
|------|----------|
| 无力，慢慢倒下 | `robot param k_pitch_c -6.5` |
| 还是无力 | `robot param current_limit 2000` |
| 小角度轮子不响应 | `robot param stiction_ma 1200` |
| 轮子来回猛冲 | `robot param stiction_ma 400` |
| 前后振荡 | `robot param k_pitch_c -4.5` |
| 快速抖动 | `robot param k_pitch_rate_c -2.0` |
| 反应很钝 | `robot param k_pitch_rate_c -1.0` |

建议范围：

| 参数 | 常用范围 |
|------|----------|
| `k_pitch_c` | `-4.0` 到 `-8.0` |
| `k_pitch_rate_c` | `-1.0` 到 `-2.8` |
| `stiction_ma` | `0` 到 `2000` |
| `current_limit` | `1200` 到 `2500` |

## 9. 能站住后调漂移

如果能短暂站住，但会慢慢往一个方向跑：

```text
robot param k_position -0.8
robot param k_velocity_c -1.3
```

如果位置回复太强导致来回跑：

```text
robot param k_position -0.4
robot param k_velocity_c -1.0
```

如果左右轮速度不一致，站稳以后再开同步：

```text
robot param sync_gain 80
robot param sync_limit 400
```

跑偏仍明显：

```text
robot param sync_gain 150
```

同步导致抖动：

```text
robot param sync_gain 0
```

## 10. 保存参数

只有当机器人能重复站住，并且重启后你也想保留当前参数时：

```text
robot param
robot param save
```

保存后重新上电验证：

```text
robot param
robot enable 1
robot status
```

## 11. 推荐调参记录表

每轮只改一个参数，并记录现象。

```text
theta_eq=
k_pitch_c=
k_pitch_rate_c=
stiction_ma=
current_limit=
k_position=
k_velocity_c=
sync_gain=

现象：
下一步：
```
