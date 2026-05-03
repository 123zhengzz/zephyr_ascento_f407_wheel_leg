# Ascento 轮腿机器人待做任务清单

适用硬件：

- 主控：DJI F407 C 板
- 关节电机：达妙 DM4340，CAN1，左 ID 1，右 ID 2
- 轮电机：DJI M3508 + VESC，CAN2，左 VESC ID 101，右 VESC ID 100

目标：先确认电机驱动和方向，再测物理参数，最后逐步导入运动模型。不要跳过前面的单项测试直接启用整机平衡。

## 0. 已确认事项

- [x] 轮电机电调使用 VESC，不是 DJI C620/C610。
- [x] 当前工程轮电机代码按 VESC CAN 扩展帧协议实现。
- [x] 当前工程已有达妙 DM4340 调试命令。
- [x] 当前工程已有一个暂不启用的 Ascento 模型控制模块。
- [x] VESC 原始正电流方向已确认：左轮正转，右轮反转。
- [x] 控制输出方向已写入 `APP_WHEEL_LEFT_FORWARD_CURRENT_SIGN = 1` 和 `APP_WHEEL_RIGHT_FORWARD_CURRENT_SIGN = -1`。

## 1. 上电前安全检查

- [ ] 轮子架空，或者先拆下轮子。
- [ ] 关节连杆附近不放手，不带大负载测试关节。
- [ ] 电池旁准备实体断电开关。
- [ ] 确认 C 板、VESC、达妙电机全部共地。
- [ ] 确认 CAN1 和 CAN2 都有合适终端电阻。
- [ ] 确认 CAN_H/CAN_L 没有接反。
- [ ] 每次测试前先执行：

```text
robot enable 0
robot stop
motor debug stop
motor wheel stop
motor dm stop left
motor dm stop right
```

## 2. CAN 和反馈确认

- [ ] 烧录当前固件。
- [ ] 打开串口 shell，确认能看到 `uart:~$`。
- [ ] 执行 `motor can status all`，确认 CAN1/CAN2 都是 `error-active`。
- [ ] 执行 `motor dm status all`，确认左右达妙都有反馈。
- [ ] 执行 `motor wheel status all`，确认 VESC ID 101 和 100 都有反馈。
- [ ] 确认 VESC 反馈 `age` 通常小于 20 ms。
- [ ] 如果 VESC 无反馈，检查 VESC Tool：
  - [ ] 左 VESC Controller ID = 101。
  - [ ] 右 VESC Controller ID = 100。
  - [ ] CAN Baud = 1 Mbps。
  - [ ] CAN Mode = VESC。
  - [ ] Status 1 回传频率 = 200 Hz。

## 3. 单电机方向和停止测试

- [ ] 左轮小电流测试：

```text
motor wheel current left 100 3000
motor wheel status left
motor wheel stop
```

- [ ] 右轮小电流测试：

```text
motor wheel current right 100 3000
motor wheel status right
motor wheel stop
```

- [x] 记录左轮正电流时的实际转向：正转 / 前进方向。
- [x] 记录右轮正电流时的实际转向：反转 / 后退方向。
- [ ] 判断左右轮同号电流时，机器人趋势是前进、后退，还是原地打转。
- [x] 已在代码中把控制器右轮输出取反；调试命令仍保持 VESC 原始方向。
- [ ] 达妙左关节小幅测试：

```text
motor dm status left
motor dm wiggle left 0.02 2000 18 0.8 3000
motor dm stop left
```

- [ ] 达妙右关节小幅测试：

```text
motor dm status right
motor dm wiggle right 0.02 2000 18 0.8 3000
motor dm stop right
```

- [ ] 记录左关节正角度对应伸腿还是收腿。
- [ ] 记录右关节正角度对应伸腿还是收腿。
- [ ] 记录左右关节安全机械限位。

## 4. 确认 VESC 电流和轮端力矩系数

目标：确认左右轮各自的 `torque_k`，单位是 `Nm/mA`。当前固件默认按 M3508 理论力矩常数、减速比和效率计算，后续可以分别修正左右轮。

- [ ] 准备弹簧秤、拉力计或电子秤。
- [ ] 测量轮胎接地半径 `r_wheel`，单位 m。
- [ ] 用 VESC Tool 单独验证电流控制正常：
  - [ ] 1 A 命令时 Motor Current 接近 1 A。
  - [ ] 2 A 命令时 Motor Current 接近 2 A。
  - [ ] 正负电流方向符合记录。
- [ ] 在 VESC Tool 中开启 Status 4 和 Status 5 回传。
- [ ] 执行 `motor wheel status all`，确认 `s4_age` 和 `s5_age` 不是 `-1ms`。
- [ ] 确认 `motor_current`、`input`、`vin`、`temp` 会随电机运行更新。
- [ ] 确认 `torque_k` 显示的是左右轮各自使用的系数。
- [ ] 用静态力臂法测左轮：
  - [ ] `300 mA` 对应拉力。
  - [ ] `600 mA` 对应拉力。
  - [ ] `1000 mA` 对应拉力。
  - [ ] `-600 mA` 对应拉力。
- [ ] 用静态力臂法测右轮：
  - [ ] `300 mA` 对应拉力。
  - [ ] `600 mA` 对应拉力。
  - [ ] `1000 mA` 对应拉力。
  - [ ] `-600 mA` 对应拉力。
- [ ] 按公式计算每组数据：

```text
F_N = kg * 9.81
tau_Nm = F_N * r_wheel
k_Nm_per_mA = tau_Nm / I_cmd_mA
```

- [ ] 用多组数据拟合斜率，得到平均 `Nm/mA`。
- [ ] 如果需要覆盖理论值，把结果填入 `src/app_config.h`：

```c
#define APP_ASCENTO_LEFT_CURRENT_MA_TO_WHEEL_TORQUE_NM ...
#define APP_ASCENTO_RIGHT_CURRENT_MA_TO_WHEEL_TORQUE_NM ...
```

## 5. 测四连杆腿部运动学

目标：得到关节角 `q` 和腿长 `L` 的真实关系。

- [ ] 测量所有连杆长度，单位 m。
- [ ] 测量车体固定铰点坐标，建立车体坐标系。
- [ ] 测量轮轴或足端参考点坐标。
- [ ] 记录达妙反馈角 `0 rad` 对应的机械姿态。
- [ ] 记录左腿角度正方向。
- [ ] 记录右腿角度正方向。
- [ ] 实测左腿 `q -> L` 表格，至少 10 个点。
- [ ] 实测右腿 `q -> L` 表格，至少 10 个点。
- [ ] 找出最短安全腿长。
- [ ] 找出最长安全腿长。
- [ ] 检查是否有死点、奇异点或连杆干涉区间。
- [ ] 用几何推导或查表插值替换当前线性腿长映射。

## 6. 测整机物理参数

- [ ] 整机总质量 `m_total`，带电池、外壳、控制板。
- [ ] 单个轮子总质量 `m_wheel`。
- [ ] 左腿质量。
- [ ] 右腿质量。
- [ ] 车体质量。
- [ ] 目标站立腿长下的质心高度 `h_com`。
- [ ] 质心相对轮轴的前后偏移 `x_com`。
- [ ] 轮半径 `r_wheel`。
- [ ] 左右轮中心距 `track_width`。
- [ ] 车体俯仰转动惯量 `I_pitch`。
- [ ] 单个轮子转动惯量 `I_wheel`。
- [ ] M3508 实际减速比。

## 7. 标定 IMU

- [ ] 确认 IMU X/Y/Z 轴分别对应车体哪个方向。
- [ ] 确认机器人前倾时 pitch 是正还是负。
- [ ] 确认机器人左倾时 roll 是正还是负。
- [ ] 确认 yaw 角速度正方向。
- [ ] 机器人机械直立时读取 pitch，作为 `APP_ANGLE_ZERO_DEG` 初值。
- [ ] 手动前倾机器人，确认轮子补偿方向是把机器人推回直立，而不是加速摔倒。

## 8. 仿真任务

- [ ] 四连杆运动学仿真：
  - [ ] 输入关节角，输出腿长。
  - [ ] 输入目标腿长，反解关节角。
  - [ ] 检查限位和奇异点。
- [ ] 固定腿长两轮倒立摆仿真：
  - [ ] 状态包含 pitch、pitch rate、轮位移、轮速度。
  - [ ] 输出为左右轮端力矩。
  - [ ] 得到第一版 PID 或 LQR 增益。
- [ ] 加入电机和控制限制：
  - [ ] 1 kHz 控制周期。
  - [ ] VESC 电流限幅。
  - [ ] CAN 反馈延迟。
  - [ ] 轮端力矩饱和。
- [ ] 多体动力学仿真：
  - [ ] 固定腿长站立。
  - [ ] 小扰动恢复。
  - [ ] 慢速前进和后退。
  - [ ] 左右转向。
  - [ ] 腿长变化时保持平衡。

## 9. 接入模型前的软件任务

- [ ] 把实测物理参数填入 `src/app_config.h`。
- [ ] 把 `q <-> L` 映射做成函数或查表。
- [ ] 把模型输出的轮端力矩 `Nm` 转成 VESC 电流命令 `mA`。
- [ ] 把模型输出的目标腿长 `L` 转成达妙关节目标角 `q`。
- [ ] 在代码里增加模型控制开关，不要默认上电就启用。
- [ ] 模型控制启用前，强制检查参数已经标定。
- [ ] 初次实机测试前，把 `APP_WHEEL_CURRENT_LIMIT` 降到保守值。
- [ ] 初次实机测试前，把 `APP_PITCH_FAULT_DEG` 设置保守。

## 10. 分阶段实机验证

- [ ] 阶段 A：只接 C 板，不接电机动力电，确认串口和 IMU 正常。
- [ ] 阶段 B：只接 CAN1 达妙，确认关节能小幅动作并停止。
- [ ] 阶段 C：只接 CAN2 VESC，确认轮子小电流能动作并停止。
- [ ] 阶段 D：轮子架空，启用很小的平衡输出，观察方向是否正确。
- [ ] 阶段 E：手扶落地，限流很小，只测试能否产生正确扶正趋势。
- [ ] 阶段 F：短时间半自主站立，每次不超过几秒。
- [ ] 阶段 G：调增益，让机器人能稳定站立。
- [ ] 阶段 H：低速前进、后退、转向。
- [ ] 阶段 I：加入腿长变化和左右腿 roll 补偿。

## 11. 每次实机测试要记录

- [ ] 日期和固件版本。
- [ ] 电池电压。
- [ ] 测试命令。
- [ ] 左右轮电流命令。
- [ ] 左右轮反馈速度和电流。
- [ ] 左右达妙位置、速度、温度。
- [ ] IMU pitch、roll、yaw。
- [ ] 是否触发故障保护。
- [ ] 机器人实际动作方向。
- [ ] 是否需要修改方向符号、零点、限幅或增益。
