# Ascento 轮腿机器人调参手册

更新日期：2026-05-06

适用固件：DJI F407 C 板，Ascento 四连杆串联腿构型，`APP_USE_ASCENTO_BALANCE_CONTROLLER=1`。

当前参数版本：`PARAMS_VERSION = 14`

重要约束：

- 轮毂电机电流上限保持 `3000 mA`。
- 当前不使用刹车命令，摔倒保护只发 `0 mA`。
- 前倾 pitch 为正，后仰 pitch 为负。
- 关节电机上电后保持站立高度，轮子只负责平衡。

## 1. 串口调参命令

连接串口：

```bash
./scripts/serial.sh
```

常用命令：

```text
robot status                  # 看机器人状态
robot param                   # 打印当前所有平衡参数
robot param list              # 列出可调参数名
robot param <name> <value>    # 修改单个参数，立即生效
robot param save              # 保存到 Flash，断电保留
robot param reset             # 清除 Flash 参数，恢复代码默认值
robot enable 0                # 关闭轮平衡输出
robot enable 1                # 开启轮平衡输出
```

调参前先执行：

```text
robot enable 0
robot param
robot status
```

调完确认稳定后再执行：

```text
robot param save
```

## 2. 控制器怎么算力

当前 Ascento 控制器核心输出是一个平衡力矩：

```text
pitch_error = pitch_rad - theta_eq - target_pitch_rad
x_error     = wheel_position - wheel_position_at_enable
v_error     = body_velocity - target_forward_speed

balance_torque =
  -(K_pitch * pitch_error
    + K_pitch_rate * pitch_rate
    + K_position * x_error
    + K_velocity * v_error)
```

然后力矩换算成左右轮电流，乘以 `current_scale`，叠加 `stiction` 起步补偿，最后夹到 `±current_limit`。

所以调参时要记住：

- `K_pitch` 主要决定倾倒时轮子追得快不快。
- `K_pitch_rate` 主要决定倾倒过程的阻尼。
- `K_position` 主要决定机器人跑远后是否回到使能位置附近。
- `K_velocity` 主要决定慢慢漂移时是否能刹住速度趋势。
- `current_scale` 和 `stiction_ma` 不改变控制逻辑，只让电流更早、更明显地打出来。
- 如果 `current` 已经长期顶到 `±3000`，继续加增益意义不大，优先查轮子方向、pitch 符号、机械摩擦和电池电压。

当前实现里，四个 K 已直接写死为固定参数，不再根据 `L` 或多项式计算。

## 3. 当前默认参数

这组是当前代码默认值。烧录后如果串口 `robot param` 不是这些值，说明 Flash 里保存过旧参数，可以执行 `robot param reset`。

```text
theta_eq       = 0.314159 rad  # 18 deg
k_pitch        = -11.7500
k_pitch_rate   = -4.4000
k_position     = 0.9000
k_velocity     = 1.0500
k_yaw_rate     = 0.0000
stiction_ma    = 1900 mA
stiction_start = 0.18 deg
stiction_full  = 2.40 deg
current_limit  = 3000 mA
current_scale  = 1.45
sync_gain      = 0.0 mA/(rad/s)
sync_limit     = 500 mA
fault_deg      = 70.0 deg, hard +70.0/-30.0
recover_deg    = 10.0 deg, actual stand recover error 18.0 deg
```

控制器实际使用的四个固定增益为：

```text
K_pitch      = -11.75
K_pitch_rate = -4.40
K_position   = 0.90
K_velocity   = 1.05
```

## 4. 参数意义和调法

| 参数 | 单位 | 当前值 | 意义 | 变大或绝对值变大 |
| --- | --- | --- | --- | --- |
| `theta_eq` | rad | `0.314159` | 自然平衡 pitch 角，18 度前倾 | 目标更前倾；如果平衡点附近仍持续向后给力，尝试减小 |
| `k_pitch` | Nm/rad 等效 | `-11.75` | 姿态 P 主增益，最关键 | 更快救前倾/后仰；太大导致前后摆动或抽振 |
| `k_pitch_rate` | Nm/(rad/s) 等效 | `-4.40` | pitch 角速度阻尼 | 倾倒时更有阻尼；太大可能抖、噪声敏感或反应发闷 |
| `k_position` | Nm/m 等效 | `0.90` | 位置回正，让轮子回到使能点附近 | 抗慢漂更强；太大可能来回跑、和姿态 P 打架 |
| `k_velocity` | Nm/(m/s) 等效 | `1.05` | 速度阻尼，压住慢慢漂移 | 漂移速度更容易被压住；太大可能响应变硬或抖 |
| `k_yaw_rate` | Nm/(rad/s) | `0.0` | 偏航角速度抑制，左右轮差动 | 可抑制自转；方向没确认前保持 0 |
| `current_limit` | mA | `3000` | 最终轮电流硬限幅 | 本项目要求不改；顶到 3000 说明已饱和 |
| `current_scale` | 倍数 | `1.45` | 电流缩放，限幅前放大输出 | 更有力、更早响应；太大容易冲、抖 |
| `stiction_ma` | mA | `1900` | 起步补偿，克服 VESC 小电流死区和静摩擦 | 小角度更容易动；太大使能瞬间踢一下 |
| `stiction_start` | deg | `0.18` | pitch 误差超过多少度开始加 stiction | 越小越早补偿；太小可能零点附近抖 |
| `stiction_full` | deg | `2.40` | pitch 误差到多少度给满 stiction | 越小越快满补偿；太小会突然猛 |
| `sync_gain` | mA/(rad/s) | `0.0` | 左右轮速度同步补偿 | 可减小左右轮速度差；也可能引入原地旋转 |
| `sync_limit` | mA | `500` | 同步补偿的差动电流上限 | 限制同步补偿最大影响 |
| `fault_deg` | deg | `70.0` | 运行时摔倒保护角 | 受硬限制约束，前倾最大 +70，后仰最大 -30 |
| `recover_deg` | deg | `10.0` | 保留参数 | 当前 Ascento 路径实际恢复用 `stand recover error 18 deg` |

## 5. 推荐调参顺序

### 5.1 先确认控制链路

```text
robot status
motor wheel status all
motor dm status all
```

必须先满足：

```text
enable=1
wheels=1
fault=0
```

如果 `fault=1` 或 `current=(0,0)`，先处理保护和反馈，不要急着加增益。

### 5.2 确认方向

方向正确时：

| 姿态 | 轮子应该 |
| --- | --- |
| 前倾，pitch 变大 | 向前追 |
| 后仰，pitch 变小或为负 | 向后追 |

如果追反，优先改方向符号，不要靠增益硬救。

### 5.3 调 `theta_eq`

扶住机器人到你认为的自然平衡姿态，观察 `robot status` 里的 `pitch`。

当前默认：

```text
robot param theta_eq 0.314159
```

如果接近平衡点时轮子仍一直往后给力，可以每次减小：

```text
robot param theta_eq 0.294
robot param theta_eq 0.274
```

如果接近平衡点时轮子一直往前给力，可以每次增大：

```text
robot param theta_eq 0.334
robot param theta_eq 0.354
```

每次建议改 `0.02 rad`，约 `1.1 deg`。

### 5.4 调姿态 P：`k_pitch`

这是“反应不过来”的首要参数。

当前：

```text
robot param k_pitch -11.75
```

如果 P 不够、手扶也追不上：

```text
robot param k_pitch -13.0
robot param k_pitch_rate -5.5
```

还不够再试：

```text
robot param k_pitch -14.0
robot param k_pitch_rate -6.0
```

如果开始前后大幅摆动或高频抽动，回退：

```text
robot param k_pitch -10.0
robot param k_pitch_rate -4.5
```

### 5.5 调阻尼：`k_pitch_rate`

如果 P 加大后能救回来但过冲、来回摆，增加阻尼：

```text
robot param k_pitch_rate -5.5
```

如果轮子动作很硬、噪声敏感、零点附近快速抖，减小一点：

```text
robot param k_pitch_rate -4.0
```

### 5.6 调电流有效输出

`current_limit` 保持 `3000` 不动。

如果轮子有命令但小角度不动：

```text
robot param stiction_ma 2200
robot param stiction_start 0.12
robot param stiction_full 2.00
```

如果使能瞬间猛踢或高频抖：

```text
robot param stiction_ma 1500
robot param stiction_start 0.30
robot param stiction_full 3.00
```

如果整体力还是偏小，但没有顶到 `3000 mA`：

```text
robot param current_scale 1.60
```

如果已经顶到 `3000 mA` 仍救不回来，不要继续加 `current_scale`，检查方向、轮子摩擦、VESC 电流模式、电池电压、机械卡滞。

### 5.7 调慢漂：`k_position` 和 `k_velocity`

如果能短暂扶住，但慢慢向后漂，最后后仰：

```text
robot param k_position 1.10
robot param k_velocity 1.00
```

如果漂移速度还是压不住：

```text
robot param k_velocity 1.20
```

如果增大后后漂反而更快，说明位置或速度项方向可能错了，先回到当前值：

```text
robot param k_position 0.90
robot param k_velocity 1.05
```

然后记录 `robot status` 中 `speed`、`current` 和实际运动方向。

### 5.8 左右轮不一致或原地旋转

当前建议：

```text
robot param k_yaw_rate 0
robot param sync_gain 0
```

如果手动放到平衡位置会原地旋转，先保持这两个为 0，再检查：

```text
motor wheel status all
robot status
```

只有在左右轮速度明显不同、方向完全确认后，再小幅打开同步：

```text
robot param sync_gain 50
robot param sync_limit 300
```

## 6. 现象速查表

| 现象 | 优先判断 | 建议 |
| --- | --- | --- |
| 完全反应不过来，扶着也倒 | 姿态 P 不够或电流没有打出来 | `k_pitch -13~-14`，配合 `k_pitch_rate -5.5~-6.0` |
| 前倾救不回来 | P 不够、stiction 不够、轮方向可能反 | 先看前倾时轮子是否向前追，再加 `k_pitch` |
| 后仰慢慢倒 | 位置/速度回正不足，或 `theta_eq` 偏大 | 试 `k_position 1.10`、`k_velocity 1.0`，必要时降低 `theta_eq` |
| 小角度轮子不动 | VESC 死区或静摩擦 | 加 `stiction_ma`，降低 `stiction_start` |
| 使能瞬间猛冲 | stiction 或 current_scale 太大，theta_eq 不对 | 降 `stiction_ma`，加大 `stiction_full`，检查 `theta_eq` |
| 前后大幅摆 | P 太大或阻尼不足 | 增大 `k_pitch_rate` 绝对值，或回退 `k_pitch` |
| 高频抖动 | stiction/current_scale 太猛，或 D 对噪声敏感 | 降 `stiction_ma`、`current_scale`，必要时减小 D 绝对值 |
| `current=(0,0)` | 没使能、轮反馈丢失、或 fault | 看 `enable/wheels/fault` |
| `current` 长期 `±3000` | 已饱和 | 查方向、机械、电池，别只继续加增益 |
| 左右轮力矩不一样，原地旋转 | yaw/sync 或轮方向问题 | `k_yaw_rate 0`、`sync_gain 0`，检查 CAN ID 100/101 方向 |
| pitch 到 `-30 deg` 后没力 | 后仰硬保护触发 | 正常保护，扶回 `theta_eq` 附近等待恢复 |

## 7. 当前整组参数命令

如果不重烧固件，想把运行参数设成当前推荐值：

```text
robot enable 0
robot param theta_eq 0.314159
robot param k_pitch -11.75
robot param k_pitch_rate -4.40
robot param k_position 0.90
robot param k_velocity 1.05
robot param current_limit 3000
robot param current_scale 1.45
robot param stiction_ma 1900
robot param stiction_start 0.18
robot param stiction_full 2.40
robot param sync_gain 0
robot param k_yaw_rate 0
robot param save
robot enable 1
```

## 8. 保存规则和注意事项

- `robot param <name> <value>` 立即生效，但不保存。
- `robot param save` 会写入 Flash，断电保留。
- `robot param reset` 会清除 Flash 参数，恢复代码默认值。
- 烧录新固件后，如果 `PARAMS_VERSION` 升级，旧保存参数会失效，自动使用新默认值。
- 调参时一次只改 1 到 2 个参数，否则很难判断原因。
- 每次记录：参数、使能时 pitch、现象、`current` 是否饱和、是否 fault。

现场记录模板：

```text
theta_eq=
k_pitch=
k_pitch_rate=
k_position=
k_velocity=
current_scale=
stiction_ma=
stiction_start=
stiction_full=

使能时 pitch=
使能时 roll=
current=(L,R)=
speed=
fault=
现象：
下一步：
```
