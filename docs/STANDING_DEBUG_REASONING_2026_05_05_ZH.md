# 站立调试判断记录 2026-05-05

本文记录当前 Ascento 构型轮腿机器人站立调试中的工程判断、参数修改方向和串口验证步骤。重点是把现场现象转成可执行的调参流程。

## 当前硬件与控制前提

- 主控：DJI F407 C 板。
- 关节：DM4340，CAN1。
- 轮毂/VESC：CAN2，左轮 ID 101，右轮 ID 100。
- 当前启用的是 Ascento 模型控制器路径：`APP_USE_ASCENTO_BALANCE_CONTROLLER=1`。
- 关节电机策略：只要主控运行，就持续发送站立高度锁定目标。
- 轮子策略：只有 enable、轮反馈正常、未进入摔倒保护时才输出电流。

## 已确认的问题和处理

1. 后仰摔倒后轮子还转

   处理：在主循环里加硬摔倒保护，摔倒时停止 debug wheel 输出并向 VESC 发送 0 mA。不使用刹车。

2. 关节上电后无力

   处理：取消“故障后保持当前角”的软逻辑，统一发送站立锁定角。

3. 手动放到平衡点，左右轮原地旋转

   处理：
   - `sync_gain=0`
   - `k_yaw_rate=0`
   - 修正 ID 100 右轮方向：右轮 `APP_WHEEL_RIGHT_FORWARD_CURRENT_SIGN=-1`

4. 轮子力度不够

   处理：
   - 电流上限保持 `3000 mA`
   - 通过 `current_scale` 和 `stiction_ma` 增强有效输出
   - 不突破最终 `current_limit`

5. 当前参数会振动，并且向后漂移后后仰摔倒

   现象摘录：

   ```text
   theta_eq       = 0.3130 rad (17.9 deg)
   k_pitch        = -8.0019*L^2 + 2.9106*L + -8.0000
   k_pitch_rate   = -19.1727*L^2 + 7.0983*L + -2.6000
   k_position     = -0.2500
   k_velocity     = -6.1109*L^2 + 2.3991*L + -0.6500
   stiction_ma    = 1800 mA
   current_limit  = 3000 mA
   current_scale  = 1.50
   sync_gain      = 0.0
   fault_deg      = 70.0 deg (hard +70.0/-30.0)
   pitch -34.16 current 0/0
   ```

   判断：
   - `pitch=-34.16` 已经越过后仰硬保护 `-30 deg`，所以 `current 0/0` 是保护切断，不是轮子命令丢失。
   - 振动主要来自姿态刚度、起步补偿和电流缩放偏猛。
   - 向后漂移说明位置/速度回正项偏弱，且当前符号对漂移回正不够合适。

## 当前建议默认参数组

目标：当前现场反馈是前倾救不回来、轮端力偏小，同时后向会慢慢漂移并最终后仰。新默认值改为“救倾倒优先”的一档：电流上限仍保持 `3000 mA`，但让小角度误差更早进入有效电流，并加强速度/位置回正。

```text
theta_eq       = 0.314159 rad  # 18 deg
k_pitch       = -11.75
k_pitch_rate  = -4.40
k_position    = 0.90
k_velocity    = 1.05
current_limit  = 3000
current_scale  = 1.45
stiction_ma    = 1900
stiction_start = 0.18
stiction_full  = 2.40
sync_gain      = 0
k_yaw_rate     = 0
```

说明：
- `k_pitch` 固定为 `-11.75`，解决姿态 P 不够、倾倒时轮子追不上。
- `k_pitch_rate` 固定为 `-4.40`，P 加大后同步增加角速度阻尼，减少直接抽振。
- `current_scale` 从 `1.25` 加到 `1.45`，不改变最终 `3000 mA` 限幅，只提高有效输出比例。
- `stiction_ma` 从 `1500` 加到 `1900`，起始角从 `0.30 deg` 提前到 `0.18 deg`，让轮子更早突破静摩擦和 VESC 小电流死区。
- `k_position` 从 `0.65` 加到 `0.90`，进一步压住向后慢漂；如果后漂反而更快，优先怀疑位置项方向符号。

## 串口临时设置命令

不重烧固件时，可以直接输入：

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

验证：

```text
robot status
motor wheel status all
motor dm status all
```

## 观察字段

`robot status` 重点看：

```text
pitch=...
err=...
speed=...
lqr=...
current=(L,R)
joint=(L,R)
fault=...
wheels=...
```

判断：
- `fault=1` 且 `current=(0,0)`：保护切断，不是控制器没有输出。
- `pitch < -30`：后仰硬保护触发。
- `pitch > +70`：前倾硬保护触发。
- `current` 长期顶到 `3000`：控制已经饱和，继续加增益意义不大，要检查方向、平衡角或机械问题。
- `current` 很小但车倒：增益或起步补偿不足。
- `current` 一正一负且车原地转：检查 `sync_gain`、`k_yaw_rate`、左右轮方向符号。

## 下一轮调参决策表

| 现象 | 优先调整 |
| --- | --- |
| 高频抖动、左右快速来回抽动 | 先把 `k_pitch` 回退到 `-10.0`；仍抖再降低 `current_scale` 到 `1.25` |
| 一上电就冲、轮子猛转 | 降低 `stiction_ma` 到 `800~1200`；增大 `stiction_full` 到 `4.0` |
| 慢慢向后漂移，最后后仰 | 增大 `k_velocity` 到 `1.2`；或继续试 `k_position 1.10` |
| 还是软，扶着会倒 | `k_pitch` 继续试 `-13.0~-14.0`，但观察是否开始振荡 |
| 平衡点不对，一直单向补偿 | 调 `theta_eq`，每次 `0.02 rad` |
| 后仰到 `-30 deg` 后没电流 | 正常，是硬保护；扶回 `theta_eq` 附近等待恢复 |

## 重要注意事项

- `robot param <name> <value>` 立即生效，但不保存。
- `robot param save` 会写入 flash，后续上电继续使用。
- 代码默认参数改变后，如果参数版本没有升级，旧 flash 参数可能覆盖新默认值。
- 当前固件通过 `PARAMS_VERSION` 控制参数结构版本；本轮建议参数对应版本 `14`。
- 轮子手动 debug 命令会进入 debug 路径，测试完要执行：

```text
motor debug stop
robot enable 0
robot enable 1
```
