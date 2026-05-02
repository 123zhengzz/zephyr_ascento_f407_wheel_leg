# DJI F407 C Board Ascento Wheel-Leg Robot

中文初学者调试手册见 [README_ZH.md](README_ZH.md)。

Quick start on this machine:

```bash
cd /home/h/code_leg/zephyr_ascento_f407_wheel_leg
./scripts/build.sh
./scripts/flash.sh
./scripts/serial.sh /dev/ttyUSB0
```

Default bus routing:

- CAN1: Damiao DM4340/DM43xx joint motors, IDs 1 and 2
- CAN2: VESC + DJI M3508 wheel motors, left ID 101 and right ID 100, 1 Mbps extended CAN

Motor wiring and smoke-test commands are documented in:

- `README_ZH.md`
- `docs/VESC_CODE_CHECK_AND_DEBUG_README_ZH.md`
- `docs/MOTOR_DEBUG_README_ZH.md`
- `docs/FLASH_TROUBLESHOOTING_README_ZH.md`
- `docs/ASCENTO_BALANCE_REQUIREMENTS_ZH.md`
