# Ascento 站不起来时的调整策略

这份文档用于现场排查：机器人已经能通信、能输出轮电流，但仍然站不起来。

核心原则：不要一上来追某个固定角度。先确认控制链路工作，再把机器人扶到接近平衡点，最后按现象调参数。

## 1. 先判断是不是控制门问题

执行：

```text
robot status
motor wheel status all
motor dm status all
```

`robot status` 必须满足：

```text
enable=1 wheels=1 fault=0
```

如果不是：

| 现象 | 处理 |
|------|------|
| `enable=0` | 执行 `robot enable 1` |
| `wheels=0` | 查 `motor wheel status all`，左右轮反馈 age 必须小于 50ms |
| `fault=1` | 机身 pitch 超过保护角，尤其后仰超过约 22 deg 会切轮，扶回安全姿态后重新 enable |
| `current=(0,0)` | 模型未 active、轮反馈丢失、或输出太小 |

## 2. 确认方向对

前倾时 `pitch` 应变大。方向正确时：

| 姿态 | 轮子动作 |
|------|----------|
| 前倾 | 轮子向前追 |
| 后仰 | 轮子向后追 |

如果轮子追反，先修方向符号，不要调增益。

如果方向没反，继续下面步骤。

## 3. 先不要追 18 度

`theta_eq` 不是姿态命令，而是质心落在轮轴上方时的自然平衡角。当前估计初值是：

```text
robot param theta_eq 0.245
```

约等于前倾 14 度。只有当你手扶在 18 度时前后用力最小，才把它设为：

```text
robot param theta_eq 0.314
```

如果 18 度本身扶不住，说明它不是自然平衡点，不要硬追。

## 4. 正确的第一次使能姿态

先关控制：

```text
robot enable 0
```

扶住机器人，两轮接地，机身前倾到 `pitch` 接近 `theta_eq` 对应的角度。

如果 `theta_eq=0.245`，`robot status` 应看到：

```text
pitch=12 到 16
roll 接近 0
```

然后：

```text
robot enable 1
robot status
```

不要在 `pitch=-11` 这种明显后仰姿态下判断 14 度平衡参数。此时误差大约 25 度，控制器一定会大电流救姿态。

## 5. 推荐初值

```text
robot enable 0

robot param theta_eq 0.245
robot param sync_gain 0
robot param current_scale 1.0
robot param current_limit 2000
robot param stiction_ma 800
robot param stiction_start 0.10
robot param stiction_full 1.50
robot param k_pitch_c -5.8
robot param k_pitch_rate_c -1.8
robot param k_position -0.63
robot param k_velocity_c -1.15
```

然后扶到 `pitch=12 到 16`：

```text
robot enable 1
robot status
```

## 6. 按现象调

方向正确但慢慢倒下：

```text
robot param k_pitch_c -6.8
robot param current_limit 2400
robot param stiction_ma 1200
```

轮子小角度不动，倒到一定角度才突然动：

```text
robot param stiction_ma 1500
```

前后猛冲或抽搐：

```text
robot param k_pitch_c -4.8
robot param k_pitch_rate_c -2.3
robot param stiction_ma 400
```

能短暂站住但慢慢漂移：

```text
robot param k_position -0.8
robot param k_velocity_c -1.3
```

位置回复太强，来回跑：

```text
robot param k_position -0.4
robot param k_velocity_c -1.0
```

## 7. 左右腿不平的处理

不要用软件补偿掩盖左右腿实际长度差。先机械调平或重新检查标定。

检查：

```text
robot enable 0
robot status
```

扶正时 `roll` 应接近 0。左倾时 `roll` 变大。

如果 `roll` 长期偏 2 到 3 度以上，先查：

| 项目 | 说明 |
|------|------|
| 连杆装配 | 左右腿物理长度和安装孔位 |
| DM 零点 | 同一机械姿态下左右反馈是否合理 |
| 站立关节目标 | `robot status` 中 joint 是否接近期望 |
| 四连杆标定 | 左右关节角到腿长的映射是否一致 |

## 8. 保存条件

只有满足下面条件后才保存：

```text
enable=1 wheels=1 fault=0
```

机器人能被重复扶起并短时间站住，且参数不是临时试错值：

```text
robot param
robot param save
```

## 9. 现场记录模板

```text
theta_eq=
k_pitch_c=
k_pitch_rate_c=
stiction_ma=
current_limit=
k_position=
k_velocity_c=
sync_gain=

使能时 pitch=
使能时 roll=
现象：
下一步：
```
