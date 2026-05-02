# 上电检查清单

1. C 板单独上电，ST-LINK 能连接。
2. `west flash --runner openocd` 正常。
3. 串口 115200 能看到 Zephyr 日志和 `uart:~$` shell。
4. BMI088 日志无持续 SPI 错误。
5. CAN 接线按工程默认分配：DM4340/DM4310 关节在 CAN1，VESC/M3508 轮子在 CAN2。
6. VESC 电调 ID 为左 `101`、右 `100`，CAN Baud 为 `1 Mbps`，Status 1 回传为 `200 Hz`，能收到扩展帧 `0x965`、`0x964` 反馈。
7. DM4340 电机 ID 为 1、2，Master ID 与 `APP_DM_MASTER_ID` 一致。
8. 执行 `motor dm enable left`、`motor dm pos left 0.12 1 800`，确认左关节方向。
9. 执行 `motor wheel current left 400 500`，确认左轮能短时转动。
10. 执行 `robot height 38/60` 时腿部方向正确。
11. 架空执行 `robot enable 1`，轮子输出方向正确。
12. 执行 `robot enable 0` 后，VESC 电流命令归零。
13. 俯仰超过 25 度时，红灯亮且轮子电流归零。
