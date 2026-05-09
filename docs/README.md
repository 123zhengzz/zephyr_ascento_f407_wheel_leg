# Ascento Wheel-Leg F407 - Documentation Index

## Quick Start

| Document | Description |
|----------|-------------|
| [wiring_build_flash.md](wiring_build_flash.md) | Hardware wiring, build, and flash quick reference |
| [bringup_checklist.md](bringup_checklist.md) | 13-step power-on bringup checklist |
| [STAND_UP_TUNING_QUICKSTART_ZH.md](STAND_UP_TUNING_QUICKSTART_ZH.md) | On-site command sequence for standing tuning |

## Core Reference

| Document | Description |
|----------|-------------|
| [ASCENTO_README_ZH.md](ASCENTO_README_ZH.md) | Ascento model reference: physical parameters, LQR derivation, simulation, activation checklist |
| [ASCENTO_PARAM_TUNING_MANUAL_ZH.md](ASCENTO_PARAM_TUNING_MANUAL_ZH.md) | Current Ascento balance parameter meaning and tuning manual |
| [SERIAL_COMMAND_REFERENCE.md](SERIAL_COMMAND_REFERENCE.md) | Complete serial shell command reference |

## Motor & VESC

| Document | Description |
|----------|-------------|
| [MOTOR_DEBUG_README_ZH.md](MOTOR_DEBUG_README_ZH.md) | Motor wiring, single-motor debug, VESC protocol, torque coefficient, Nm/mA calibration, serial terminal setup |

## Tuning & Balance

| Document | Description |
|----------|-------------|
| [STANDING_TUNING_GUIDE.md](STANDING_TUNING_GUIDE.md) | LQR standing tuning guide with fault table |
| [STANDING_ADJUSTMENT_STRATEGY_ZH.md](STANDING_ADJUSTMENT_STRATEGY_ZH.md) | Strategy for diagnosing why the robot still cannot stand |
| [PARAM_TUNING_GUIDE.md](PARAM_TUNING_GUIDE.md) | LQR polynomial gain tuning guide |
| [PERSISTENT_PARAMS_GUIDE.md](PERSISTENT_PARAMS_GUIDE.md) | Parameter persistence (Flash/NVS) user guide |

## Flash & Debugging

| Document | Description |
|----------|-------------|
| [FLASH_TROUBLESHOOTING_README_ZH.md](FLASH_TROUBLESHOOTING_README_ZH.md) | Flash troubleshooting guide with error catalog |
| [ASCENTO_WHEEL_NOT_ROTATE_DEBUG_MANUAL_ZH.md](ASCENTO_WHEEL_NOT_ROTATE_DEBUG_MANUAL_ZH.md) | Troubleshooting guide when wheels do not spin |

## Developer Reference

| Document | Description |
|----------|-------------|
| [SERIAL_DEBUG_IMPLEMENTATION_ZH.md](SERIAL_DEBUG_IMPLEMENTATION_ZH.md) | Serial debug 5-layer architecture |
| [GIT_USAGE.md](GIT_USAGE.md) | Git workflow and commit conventions |

## Archive

Debug journals, session reports, and implementation reports have been moved to `/home/h/桌面/新建文件夹/zephyr_ascento_archived_docs/` for reference. This includes:

- EXECUTION_STEP_MANUAL.md (dated step-by-step manual)
- BALANCE_TUNING_REPORT_2026_05_08_ZH.md (balance debug session report)
- STANDING_DEBUG_REASONING_2026_05_05_ZH.md (standing debug reasoning log)
- MATLAB_LQR_IMPORT_2026_05_09.md (LQR gain recomputation report)
- BALANCE_TUNING_GUIDE_ZH.md (older torque budget analysis, stale parameters)
- FLASH_SCRIPT_TEST_REPORT.md (flash script test results)
- REPORT_PERSISTENT_STORAGE_IMPLEMENTATION.md (NVS implementation report)
- SERIAL_README_ZH.md (serial terminal setup, merged into MOTOR_DEBUG_README_ZH)
