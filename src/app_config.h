#pragma once
/*
 * app_config.h — 全局应用配置头文件
 *
 * 本文件集中定义了轮腿机器人（Ascento 风格）的所有硬件参数、控制参数和物理参数。
 * 基于 STM32F407（DJI F407IGH6-C 开发板）运行 Zephyr RTOS。
 *
 * 主要内容包括：
 *   - 系统控制频率与 CAN 总线速率
 *   - 控制器选型（Ascento 模型 LQR / PID 平衡车）
 *   - 轮毂电机（DJI M3508 + VESC）与关节电机（DM4340）的 CAN ID 配置
 *   - 轮毂方向校准、电流限制、力矩常数
 *   - 四连杆腿机构几何参数与关节角度标定数据
 *   - LQR 平衡控制器增益（K_PITCH、K_POSITION 等）
 *   - 防摔保护阈值、静摩擦补偿（Ascento 模型）、轮速同步补偿
 *   - 机器人物理参数（质量、质心高度、转动惯量等）
 *   - IMU 符号校正
 *   - 关节锁定位置（DM4340 MIT 模式）
 *
 * 所有 #define 均可通过此处直接修改，或在运行时通过 NVS（非易失性存储）覆盖。
 */

#include <stdint.h>

/* ======================== 系统基础配置 ======================== */

#define APP_CONTROL_HZ 200          /* 主控制环频率，单位 Hz（赫兹），即每秒执行 200 次控制循环 */
#define APP_CAN_BITRATE 1000000U    /* CAN 总线波特率，单位 bps（比特每秒），1 Mbps */

/*
 * 控制器选型
 * Model-based Ascento LQR controller — ACTIVE.
 *
 * 基于模型的 Ascento LQR 控制器（当前激活）。
 * 采用四连杆运动学 + LQR 全状态反馈平衡控制。
 * DJI M3508 直驱电机（无减速箱），力矩常数 Kt = 0.180 Nm/A。
 * DM4340 关节电机锁定在 115 mm 站立腿长。
 *
 * Four-bar kinematics with LQR full-state-feedback balance control.
 * DJI M3508 direct-drive (no gearbox), Kt = 0.180 Nm/A.
 * DM4340 joints locked at 115 mm standing leg length.
 */
#define APP_USE_ASCENTO_BALANCE_CONTROLLER 1    /* 1 = 启用 Ascento 模型 LQR 控制器；0 = 使用 PID 平衡控制器 */
#define APP_ASCENTO_PARAMS_CALIBRATED 1         /* 1 = 物理参数已标定完成；0 = 使用默认估计值 */

/*
 * CAN 总线路由配置
 * CAN routing is defined by devicetree aliases: joint=CAN1, wheel=CAN2.
 * 关节电机（DM4340）挂载在 CAN1，轮毂电机（M3508 + VESC）挂载在 CAN2。
 * Wheel motors use VESC controllers on CAN2.
 */
#define APP_WHEEL_LEFT_ID  101   /* 左轮 VESC 控制器的 CAN ID，通过 VESC 工具配置 */
#define APP_WHEEL_RIGHT_ID 100   /* 右轮 VESC 控制器的 CAN ID，通过 VESC 工具配置 */

/*
 * 轮毂电机方向校准（基于 VESC 电流测试）
 * Wheel direction calibration from the VESC current test.
 *
 * 语义说明：
 * - 控制器/模型中的轮电流以机器人前进方向为正方向。
 * - VESC 原始调试电流保持不变，不做取反。
 * - raw_vesc_current = logical_forward_current * APP_WHEEL_*_FORWARD_CURRENT_SIGN
 * - logical_forward_speed = raw_vesc_speed * APP_WHEEL_*_FORWARD_CURRENT_SIGN
 *
 * 2026-05-05 现场标定：
 * CAN ID 100（右轮）的安装/接线方向与 CAN ID 101（左轮）相反，
 * 因此其逻辑正向电流需要使用相反的 VESC 原始电流符号。
 *
 * Field-tuned 2026-05-05:
 * CAN ID 100 is mounted/wired opposite to ID 101, so its logical forward
 * current uses the opposite raw VESC-current sign.
 */
#define APP_WHEEL_LEFT_FORWARD_CURRENT_SIGN  -1   /* 左轮前进方向电流符号：-1 表示 VESC 原始正电流对应机器人后退 */
#define APP_WHEEL_RIGHT_FORWARD_CURRENT_SIGN  1   /* 右轮前进方向电流符号：+1 表示 VESC 原始正电流对应机器人前进 */
#define APP_M3508_DIRECTION_TEST_CURRENT_MA  100  /* 方向测试电流幅值，单位 mA（毫安） */
#define APP_M3508_DIRECTION_TEST_DURATION_MS 3000 /* 方向测试持续时间，单位 ms（毫秒） */

/*
 * DM4340 关节电机 CAN 配置
 * DM motors are on CAN1. Command IDs are the motor IDs; feedback IDs are the
 * Master IDs configured in the Damiao tool.
 *
 * DM4340 关节电机挂载在 CAN1 总线上。
 * 发送命令时使用电机 ID（1 和 2）；
 * 接收反馈时使用在达妙工具中配置的 Master ID（0x11 和 0x12）。
 */
#define APP_DM_LEFT_ID              1     /* 左侧 DM4340 电机的 CAN 命令 ID */
#define APP_DM_RIGHT_ID             2     /* 右侧 DM4340 电机的 CAN 命令 ID */
#define APP_DM_LEFT_FEEDBACK_ID     0x11  /* 左侧 DM4340 电机的 CAN 反馈 ID（Master ID） */
#define APP_DM_RIGHT_FEEDBACK_ID    0x12  /* 右侧 DM4340 电机的 CAN 反馈 ID（Master ID） */
#define APP_DM_MASTER_ID            APP_DM_LEFT_FEEDBACK_ID  /* 主电机反馈 ID，用于广播/同步 */

/* ======================== 轮毂电机电流限制 ======================== */

#define APP_WHEEL_CURRENT_LIMIT 6000  /* 轮毂电机最大允许电流，单位 mA（毫安），用于平衡控制时的电流钳位 */
#define APP_WHEEL_CURRENT_SAFE   800  /* 轮毂电机安全调试电流，单位 mA（毫安），用于调试模式下的电流限幅保护 */

/* ======================== 轮毂电机参数（M3508 + VESC） ======================== */

/*
 * 轮毂电流缩放系数
 * Wheel current scale factor.
 *
 * 地面接触模式：1.0f（使用原始 LQR 增益）。
 * 台架测试模式：3.0f~5.0f，用于克服 VESC 最低电流阈值（约 0.5~1 A），
 * 使微小倾斜也能产生可见的轮子转动。
 *
 * GROUND CONTACT: 1.0f (use raw LQR gains).
 * BENCH TEST: 3.0f–5.0f to overcome VESC minimum current threshold (~0.5–1 A)
 * so small tilts produce visible wheel movement.
 */
#define APP_ASCENTO_WHEEL_CURRENT_SCALE  1.0f    /* 轮毂电流缩放系数，LQR 增益按物理力矩单位计算时保持 1.0 */
#define APP_M3508_REDUCTION_RATIO        1.0f    /* M3508 减速比，直驱为 1.0（无减速箱） */
#define APP_M3508_MOTOR_KT_NM_PER_A      0.180f  /* M3508 电机力矩常数 Kt，单位 Nm/A（牛米每安培） */
#define APP_M3508_GEARBOX_EFFICIENCY      1.000f  /* 减速箱效率，直驱为 1.0（无损耗） */
#define APP_VESC_CURRENT_CMD_TO_AMP       0.001f  /* VESC 电流命令到安培的换算系数：mA → A */
#define APP_VESC_MOTOR_POLE_PAIRS         7.0f    /* M3508 电机极对数，用于 ERPM 与 RPM 的换算 */
#define APP_VESC_STATUS_RATE_HZ           200     /* VESC 状态反馈发送频率，单位 Hz */
#define APP_VESC_FEEDBACK_TIMEOUT_MS      50      /* VESC 反馈超时时间，单位 ms，超时后判定通信丢失 */
#define APP_VESC_DEBUG_ERPM_LIMIT         8000    /* 调试模式下 VESC 电气转速限制，单位 ERPM */

/* ======================== 电流到力矩的换算宏 ======================== */

/*
 * 将 VESC 电流命令（mA）转换为轮子力矩（Nm）的换算系数。
 * 公式：力矩 = 电流(mA) * Kt * 减速比 * 效率 * mA→A系数
 * = 电流 * 0.180 * 1.0 * 1.0 * 0.001
 * = 电流 * 0.00018 Nm/mA
 */
#define APP_M3508_CALC_CURRENT_MA_TO_WHEEL_TORQUE_NM \
	(APP_M3508_MOTOR_KT_NM_PER_A * APP_M3508_REDUCTION_RATIO * \
	 APP_M3508_GEARBOX_EFFICIENCY * APP_VESC_CURRENT_CMD_TO_AMP)

/* 左轮电流-力矩换算系数，单位 Nm/mA（与计算值相同，左右对称） */
#define APP_ASCENTO_LEFT_CURRENT_MA_TO_WHEEL_TORQUE_NM \
	APP_M3508_CALC_CURRENT_MA_TO_WHEEL_TORQUE_NM

/* 右轮电流-力矩换算系数，单位 Nm/mA（与计算值相同，左右对称） */
#define APP_ASCENTO_RIGHT_CURRENT_MA_TO_WHEEL_TORQUE_NM \
	APP_M3508_CALC_CURRENT_MA_TO_WHEEL_TORQUE_NM

/* 左右轮平均电流-力矩换算系数，单位 Nm/mA，用于对称控制时的统一换算 */
#define APP_ASCENTO_CURRENT_MA_TO_WHEEL_TORQUE_NM \
	((APP_ASCENTO_LEFT_CURRENT_MA_TO_WHEEL_TORQUE_NM + \
	  APP_ASCENTO_RIGHT_CURRENT_MA_TO_WHEEL_TORQUE_NM) * 0.5f)

/* ======================== 电机调试参数 ======================== */

#define APP_MOTOR_DEBUG_DEFAULT_TIMEOUT_MS 1000              /* 电机调试命令默认超时时间，单位 ms */
#define APP_MOTOR_DEBUG_MAX_TIMEOUT_MS     5000              /* 电机调试命令最大允许超时时间，单位 ms */
#define APP_M3508_DEBUG_CURRENT_LIMIT      APP_WHEEL_CURRENT_SAFE  /* M3508 调试模式电流限制，复用安全电流值（800 mA） */

/* ======================== DM4340 关节电机调试参数 ======================== */

#define APP_DM_FEEDBACK_TIMEOUT_MS           100   /* DM4340 反馈超时时间，单位 ms，超时后判定关节通信丢失 */
#define APP_DM_ERROR_REENABLE_INTERVAL_MS    200   /* DM4340 报错后自动重试间隔，单位 ms */
#define APP_DM_DEBUG_VEL_LIMIT_RAD_S         4.0f  /* DM4340 调试模式速度限制，单位 rad/s（弧度每秒） */
#define APP_DM_DEBUG_KP_LIMIT                80.0f /* DM4340 MIT 模式调试 Kp 增益上限 */
#define APP_DM_DEBUG_KD_LIMIT                2.0f  /* DM4340 MIT 模式调试 Kd 增益上限 */
#define APP_DM_DEBUG_TORQUE_LIMIT_NM         2.0f  /* DM4340 调试模式力矩限制，单位 Nm（牛米） */

/* ======================== 命令超时与故障保护 ======================== */

#define APP_CMD_TIMEOUT_MS        700    /* 遥控命令超时时间，单位 ms，超时未收到新指令则停止电机输出 */
#define APP_BALANCE_RECOVER_TICKS 200    /* 摔倒后恢复等待时间，单位 tick（控制周期数），即 200/200 = 1 秒 */
#define APP_PITCH_FAULT_DEG       70.0f  /* 触发摔倒保护的俯仰角阈值，单位 deg（度），超过此角度判定摔倒 */
#define APP_PITCH_RECOVER_DEG     10.0f  /* 摔倒后恢复的俯仰角阈值，单位 deg，俯仰角回到此范围内才允许重新站起 */

/*
 * Ascento 模型控制器硬保护阈值
 * Hard fall-protection caps for the Ascento model controller.
 *
 * 运行时的 fault_deg 仍可调节，但这些硬保护上限不能通过之前保存的
 * Flash 参数放宽。后倾保护故意更严格：当机身重心远落后于轮轴时，
 * 继续驱动轮子更可能拖拽机器人而非恢复平衡。
 *
 * Runtime fault_deg remains tunable, but these caps cannot be relaxed by
 * previously saved flash parameters. Backward fall is intentionally stricter:
 * once the body is far behind the wheel axle, continuing wheel drive is more
 * likely to drag the robot than recover it.
 */
#define APP_ASCENTO_FORWARD_HARD_FAULT_DEG   70.0f  /* 前倾硬保护阈值，单位 deg，超过此角度强制切断电机 */
#define APP_ASCENTO_BACKWARD_HARD_FAULT_DEG  45.0f  /* 后倾硬保护阈值，单位 deg，比前倾更严格（后倾更难恢复） */
#define APP_ASCENTO_STAND_RECOVER_ERR_DEG    18.0f  /* 站立恢复允许的角度误差，单位 deg，在此范围内视为已恢复 */

/* ======================== 腿部高度参数 ======================== */

#define APP_DEFAULT_HEIGHT 38  /* 默认站立高度，单位为内部高度单位（需结合 APP_LEG_RAD_PER_HEIGHT_UNIT 换算为弧度） */
#define APP_HEIGHT_MIN      32 /* 最小腿部高度，单位为内部高度单位，低于此值可能导致机械干涉 */
#define APP_HEIGHT_MAX      80 /* 最大腿部高度，单位为内部高度单位，高于此值可能超出关节行程 */

/*
 * DM4340 关节位置调试限位（单位：弧度）
 * Debug clamp limits for DM4340 joint positions (rad).
 *
 * 这些限位约束了 motor_debug 命令（pos、mit、wiggle）的范围，
 * 必须覆盖 DM4340 关节的完整物理行程。已从旧的 STS3032 值
 * （0.07~0.70）更新为匹配 DM4340 工作范围。
 *
 * 左腿：最短腿时测量值 2.0251 rad → 钳位范围 1.9 ~ 3.0
 * 右腿：最短腿时测量值 1.8324 rad → 钳位范围 0.8 ~ 2.0
 *
 * These bound motor_debug commands (pos, mit, wiggle) and must cover
 * the full physical range of the DM4340 joints. Updated from old
 * STS3032 values (0.07~0.70) to match DM4340 working range.
 *
 * Left:  measured min-leg 2.0251 rad → clamp 1.9 ~ 3.0
 * Right: measured min-leg 1.8324 rad → clamp 0.8 ~ 2.0
 */
#define APP_LEG_RAD_PER_HEIGHT_UNIT 0.012884f /* 每个高度单位对应的关节弧度，用于高度到关节角度的换算 */
#define APP_LEFT_LEG_ZERO_RAD       0.0f      /* 左腿零位偏移，单位 rad，用于关节角度校准 */
#define APP_RIGHT_LEG_ZERO_RAD      0.0f      /* 右腿零位偏移，单位 rad，用于关节角度校准 */
#define APP_LEFT_LEG_MIN_RAD        1.9f      /* 左腿关节最小角度限位，单位 rad（对应最短腿长） */
#define APP_LEFT_LEG_MAX_RAD        3.0f      /* 左腿关节最大角度限位，单位 rad（对应最长腿长） */
#define APP_RIGHT_LEG_MIN_RAD       0.8f      /* 右腿关节最小角度限位，单位 rad（对应最长腿长，右腿角度方向相反） */
#define APP_RIGHT_LEG_MAX_RAD       2.0f      /* 右腿关节最大角度限位，单位 rad（对应最短腿长） */
#define APP_LEG_VEL_LIMIT_RAD_S     12.0f     /* 关节运动速度限制，单位 rad/s（弧度每秒） */
#define APP_ROLL_RAD_PER_DEG        0.010f    /* 横滚角到腿长补偿的换算系数，单位 rad/deg */
#define APP_ROLL_COMP_LIMIT_RAD     0.22f     /* 横滚角补偿的最大值，单位 rad，防止过度补偿 */

/* ======================== PID 平衡控制器参数 ======================== */

/*
 * 独立 PID 平衡车控制器
 * Independent PID balance-car controller.
 *
 * 此控制器将 DM4340 关节锁定在 115 mm 站立腿长，
 * 仅使用 M3508/VESC 轮毂电机进行平衡和驱动。
 * 类似于传统两轮自平衡小车（平衡车）的控制方式。
 *
 * This keeps the DM joints locked at the 115 mm standing leg length
 * and only uses the M3508/VESC wheels for balancing and driving.
 */
/*
 * DM4340 关节锁定位置（站立腿长约 0.115 m 时的角度）
 * DM4340 joint positions at stand leg length (~0.115 m).
 *
 * 从 3 点标定表中对 0.115 m 腿长进行插值得到，右腿略微缩短以保持机身水平：
 *   左腿：2.28 rad（介于短腿 2.0251 和中腿 2.3806 之间）
 *   右腿：1.58 rad（比名义值 1.55 rad 短约 5 mm）
 *
 * 这些值用作 PID 锁定位置（上电默认值、使能前姿态），
 * 以及 Ascento 模型控制器被禁用时的后备关节位置。
 *
 * Interpolated from the 3-point calibration table for 0.115 m, with the
 * right leg shortened slightly to level the body:
 *   left:  2.28 rad (between short 2.0251 and middle 2.3806)
 *   right: 1.58 rad (about 5 mm shorter than the 1.55 rad nominal point)
 *
 * These are used as PID lock positions (power-on default before enable)
 * and as the fallback when the Ascento model is blocked.
 */
#define APP_PID_BALANCE_LOCK_LEFT_JOINT_RAD   2.28f  /* 左腿关节锁定角度，单位 rad */
#define APP_PID_BALANCE_LOCK_RIGHT_JOINT_RAD  1.58f  /* 右腿关节锁定角度，单位 rad */
#define APP_PID_BALANCE_JOINT_USE_MIT         1      /* 1 = 使用 MIT 模式锁定关节；0 = 使用位置模式 */
#define APP_PID_BALANCE_JOINT_MIT_KP          100.0f /* MIT 模式关节刚度增益 Kp，50→100：更硬的关节锁定 */
#define APP_PID_BALANCE_JOINT_MIT_KD          12.0f  /* MIT 模式关节阻尼增益 Kd，8→12：更强的阻尼 */
/*
 * PID 平衡控制器环路参数
 */

/*
 * PID 模式静摩擦补偿参数
 * 目的是克服 VESC 控制器的最低电流阈值，在微小倾斜时也能驱动轮子。
 */

/*
 * Ascento 模型控制器静摩擦补偿
 * Stiction compensation for the Ascento model controller.
 *
 * VESC 控制器存在最低电流阈值（约 0.5~1 A）。在接近直立时，LQR 力矩命令
 * 经常低于此阈值，导致轮子不转——即使控制器已输出非零电流。
 * 静摩擦补偿根据俯仰角误差线性叠加一个偏置电流，以克服静摩擦力。
 *
 * 缩放系数从 START_DEG 时的 0 线性增长到 FULL_DEG 时的 1：
 *   stiction_ma = STICTION_CURRENT * scale * sign(balance_torque)
 *
 * 将 CURRENT 设为 0 可禁用此功能。
 *
 * VESC controllers have a minimum current threshold (~0.5–1 A).  Near upright
 * the LQR torque command is often below this threshold, so the wheels don't
 * turn even though the controller is commanding non-zero current.  Stiction
 * adds a bias that ramps with pitch error to overcome static friction.
 *
 * Scale ramps linearly from 0 at START_DEG to 1 at FULL_DEG:
 *   stiction_ma = STICTION_CURRENT * scale * sign(balance_torque)
 *
 * Set CURRENT to 0 to disable.
 */
#define APP_ASCENTO_STICTION_CURRENT_MA 800.0f /* 静摩擦补偿电流幅值，单位 mA，设为 0 禁用 */
#define APP_ASCENTO_STICTION_START_DEG   0.5f  /* 静摩擦补偿起始角度，单位 deg，低于此角度不补偿 */
#define APP_ASCENTO_STICTION_FULL_DEG    3.0f  /* 静摩擦补偿满量程角度，单位 deg，高于此角度满量补偿 */

/*
 * 轮速同步补偿（Ascento 模型控制器）
 * Wheel speed synchronization compensation.
 *
 * 当左右轮速度不一致时（由电机参数差异、摩擦差异或轮径偏差引起），
 * 此功能添加差动电流以使两轮速度趋于一致。
 *
 * sync_gain: 比例增益（单位：mA 每 rad/s 速度误差）
 *   值越大同步越强，但可能引起振荡。
 *   建议从 50-100 开始调试，逐步增大直到两轮速度匹配。
 *   设为 0 可禁用同步功能。
 *
 * When left and right wheels have different speeds (due to motor
 * parameter mismatch, friction differences, or wheel diameter
 * variation), this adds a differential current to equalize them.
 *
 * sync_gain: proportional gain (mA per rad/s of speed error)
 *   Higher = stronger sync, but may cause oscillation.
 *   Start with 50-100, increase until wheels match.
 *
 * Set to 0 to disable synchronization.
 */
#define APP_ASCENTO_WHEEL_SYNC_GAIN_MA        100.0f /* 轮速同步比例增益，单位 mA/(rad/s 速度误差) */
#define APP_ASCENTO_WHEEL_SYNC_CURRENT_LIMIT  500.0f /* 轮速同步补偿电流上限，单位 mA */

/* ======================== PID 平衡模式摇杆映射参数 ======================== */

/* ======================== 旧版 PID 控制器增益（Legacy） ======================== */

/* ======================== IMU 零位校准（旧版） ======================== */

/* ======================== 电池电压检测（ADC） ======================== */

#define APP_BATTERY_ADC_CHANNEL       8                   /* 电池电压 ADC 采集通道号，对应 STM32 ADC 输入引脚 */
#define APP_BATTERY_ADC_FULL_SCALE_MV 3300.0f             /* ADC 满量程电压，单位 mV（毫伏），STM32 参考电压 3.3V */
#define APP_BATTERY_DIVIDER_RATIO     ((200.0f + 22.0f) / 22.0f)  /* 电池电压分压比，由 200k + 22k 电阻分压网络计算得出 */
#define APP_BATTERY_LED_THRESHOLD_V   20.5f               /* 电池低电量告警阈值，单位 V（伏特），低于此值 LED 指示低电量 */

/* ======================== 机器人物理参数（Ascento 模型） ======================== */

/* Ascento 模型控制器使用的实测物理数据。 */
/* Measured physical data for the dormant Ascento controller. */
#define APP_ASCENTO_WHEEL_RADIUS_M 0.030f   /* 轮子半径，单位 m（米），实测 30 mm */
#define APP_ASCENTO_WHEEL_BASE_M   0.201f   /* 左右轮间距（轮距），单位 m（米），实测 201 mm */

/*
 * 质量分解（2026-05-04 实测）
 * Mass breakdown (2026-05-04):
 *
 *   机身（不含腿和轮子）：1.7514 kg
 *   单条腿连杆 (m1+m2+m3+m23)：0.1108 kg
 *   两条腿：0.2216 kg
 *   单个轮子：0.171 kg
 *   两个轮子：0.342 kg
 *   总质量：1.7514 + 0.2216 + 0.342 ≈ 2.315 kg
 *
 *   body (without legs/wheels): 1.7514 kg
 *   single leg links (m1+m2+m3+m23): 0.1108 kg
 *   both legs: 0.2216 kg
 *   single wheel: 0.171 kg
 *   both wheels: 0.342 kg
 *   total: 1.7514 + 0.2216 + 0.342 ≈ 2.315 kg
 */
#define APP_ASCENTO_TOTAL_MASS_KG         2.315f   /* 机器人总质量，单位 kg（千克），约 2.315 kg */
#define APP_ASCENTO_BODY_MASS_KG          1.7514f  /* 机身质量（不含腿和轮），单位 kg */
#define APP_ASCENTO_SINGLE_WHEEL_MASS_KG  0.171f   /* 单个轮子质量，单位 kg，含轮毂电机转子 */
#define APP_ASCENTO_SINGLE_LEG_MASS_KG    0.1108f  /* 单条腿连杆总质量，单位 kg */

/*
 * 机身质心与转动惯量参数
 */
#define APP_ASCENTO_BODY_COM_HEIGHT_M          0.098f      /* 机身质心高度（相对于轮轴），单位 m，实测约 98 mm */
#define APP_ASCENTO_BODY_COM_FORWARD_OFFSET_M  (-0.0245f)  /* 机身质心前后偏移，单位 m，负值表示质心在轮轴后方 24.5 mm */
#define APP_ASCENTO_BODY_PITCH_INERTIA_KG_M2   0.060f      /* 机身绕俯仰轴的转动惯量，单位 kg*m^2 */

/*
 * 单个轮子转动惯量
 * Single wheel inertia: I = 0.5 * m_w * r^2 = 0.5 * 0.171 * 0.030^2 = 7.7e-5
 * 按均匀圆盘近似计算，单位 kg*m^2
 */
#define APP_ASCENTO_WHEEL_INERTIA_KG_M2 0.000077f  /* 单轮转动惯量，单位 kg*m^2，按均匀圆盘近似 */

/* ======================== 腿部长度参数 ======================== */

/* 腿部长度（2026-05-04 实测），单位 m（米）。 */
/* Leg lengths (physically measured 2026-05-04). */
#define APP_ASCENTO_LEG_LENGTH_MIN_M     0.064f   /* 最短腿长，单位 m，对应关节极限位置 */
#define APP_ASCENTO_LEG_LENGTH_MAX_M     0.205f   /* 最长腿长，单位 m，对应关节极限位置 */
#define APP_ASCENTO_LEG_LENGTH_DEFAULT_M 0.1345f  /* 默认腿长，单位 m，上电初始值 */
#define APP_ASCENTO_LEG_LENGTH_STAND_M   0.115f   /* 站立腿长，单位 m，平衡控制的标准站姿腿长 */

/* ======================== 四连杆腿机构几何参数 ======================== */

/*
 * 四连杆杆长（单位：米）。默认值来自 Micro-Wheeled_leg-Robot 参考几何，
 * 需要根据实际 DM4340 硬件重新测量。
 *
 * 四连杆机构关键点定义：
 *   C = 电机关节中心 (0, 0)            — 曲柄旋转中心
 *   B = (+-L4/sqrt(2), L4/sqrt(2))     — 机身固定铰接点
 *   D = C - L2*[cos(qC), sin(qC)]       — 膝关节（曲柄末端）
 *   A = circle(B, L3) 交 circle(D, L23) — 连杆交点
 *   E = D + L1*(D - A) / L23            — 轮轴位置
 *   leg_length = -E.y                   — 腿长（轮轴到电机轴的垂直距离）
 *
 * Four-bar link lengths (metres). Defaults from the Micro-Wheeled_leg-Robot
 * reference geometry; remeasure for the actual DM4340 hardware.
 *
 *   C = motor joint centre (0, 0)
 *   B = (±L4/√2, L4/√2)  – body fixed pivot
 *   D = C - L2·[cos(qC), sin(qC)]  – knee
 *   A = circle(B, L3) ∩ circle(D, L23)
 *   E = D + L1·(D - A) / L23  – wheel axle
 *   leg_length = -E.y
 */
#define APP_ASCENTO_FB_L1   0.1121f  /* 杆 L1 长度，单位 m，连接膝关节到轮轴的连杆 */
#define APP_ASCENTO_FB_L2   0.1100f  /* 杆 L2 长度，单位 m，电机曲柄杆（C 到 D） */
#define APP_ASCENTO_FB_L3   0.1130f  /* 杆 L3 长度，单位 m，连接机身固定点 B 到连杆交点 A */
#define APP_ASCENTO_FB_L4   0.0580f  /* 杆 L4 长度，单位 m，机身固定铰接点到电机中心的距离 */
#define APP_ASCENTO_FB_L23  0.0306f  /* 杆 L23 长度，单位 m，膝关节 D 到连杆交点 A 的距离 */

/*
 * DM4340 关节角度到四连杆曲柄角 qC 的偏移量
 * Offset from DM4340 joint-angle to four-bar qC.
 *
 * DM4340 编码器与四连杆模型的 qC 旋转方向相反：
 *   qC = offset - joint_angle
 *
 * 此符号约定于 2026-05-04 验证：若使用 qC = joint_angle - offset
 * （直觉猜测），会产生超过 128 mm 的腿长误差，因为 DM4340 编码器
 * 和模型数学曲柄角的旋转方向相反。
 *
 * 每条腿需要独立的偏移量，因为左右电机零位不同。
 * 标定方法：在已知腿长（如中间标定点）处测量关节角度，
 * 通过四连杆正运动学反解出产生该腿长的 qC，从而求得偏移量。
 *
 * 2026-05-04 重新标定（修正了短腿关节角度测量值）：
 * 短腿 q 重新测量：左 2.0251，右 1.8324。
 * 3 个数据点现在一致性在 2.5 度以内——四连杆模型和杆长已验证。
 *
 *   左腿：offset = 4.892 rad (280°) — 3 个一致偏移量的平均值
 *   右腿：offset = 2.098 rad (120°) — 3 个一致偏移量的平均值
 *
 * The DM4340 and the four-bar model's qC rotate in opposite directions:
 *   qC = offset - joint_angle
 *
 * This sign convention was validated on 2026-05-04: using qC = joint_angle -
 * offset (the intuitive guess) causes >128 mm leg-length errors because the
 * DM4340 encoder and the model's mathematical crank angle spin opposite ways.
 *
 * Each leg needs its own offset because the left/right motor zero positions
 * differ.  Calibrate one offset per leg by measuring at a known leg length
 * (e.g. the middle calibration point) and solving for the qC that produces
 * that length through the four-bar forward kinematics.
 *
 * Recalibrated 2026-05-04 after correcting the short-leg joint-angle
 * measurement (short-leg q was re-measured: left 2.0251, right 1.8324).
 * All 3 data points now agree within 2.5° — the four-bar model and
 * link lengths are validated.
 *
 *   Left:  offset = 4.892 rad (280°)  — average of 3 consistent offsets
 *   Right: offset = 2.098 rad (120°)  — average of 3 consistent offsets
 */
#define APP_ASCENTO_FB_JOINT_ZERO_OFFSET_LEFT_RAD  4.892f  /* 左腿关节零位偏移，单位 rad（约 280 度） */
#define APP_ASCENTO_FB_JOINT_ZERO_OFFSET_RIGHT_RAD 2.098f  /* 右腿关节零位偏移，单位 rad（约 120 度） */

/*
 * 四连杆模型 y=0 平面与实际腿长测量参考平面之间的偏移量
 * Reference-point offset between the four-bar model's y=0 plane and the
 * physical reference point used for leg-length measurements.
 *
 * 模型计算 leg_length = -E.y（从电机轴中心 C 到轮轴 E 的垂直投影距离）。
 * 用户从电机轴中心下方约 20 mm 的参考平面测量腿长。
 * 将此值设为两个参考平面之间的实测垂直距离，使模型输出与物理测量一致。
 *
 * 当前最佳估计：0.0 mm（未验证——需用卡尺测量）。
 *
 * The model computes leg_length = -E.y (distance from motor shaft centre C
 * to wheel axle E, projected onto the vertical axis).  The user measures
 * leg length from a reference plane ~20 mm below the motor shaft centre.
 * Set this to the measured vertical distance between the two reference
 * planes so the model output matches physical measurements.
 *
 * Current best estimate: 0.0 mm (unverified — measure with calipers).
 */
#define APP_ASCENTO_LEG_LENGTH_REF_OFFSET_M 0.0f  /* 腿长参考平面偏移量，单位 m，当前设为 0（待标定） */

/* ======================== 关节角度-腿长三点标定数据 ======================== */

/*
 * 实测三点标定数据：关节角度 ↔ 腿长（2026-05-04）
 * Measured 3-point joint-angle ↔ leg-length calibration (2026-05-04).
 *
 * 这些锚点被分段线性备用代码使用（在 #if 0 中）。
 * 活跃的四连杆模型仅将它们用于偏移量标定和验证。
 *
 * 左腿：角度增大 → 腿伸长（L 随 q 增大）
 * 右腿：角度减小 → 腿伸长（L 随 q 减小——镜像几何）
 *
 *    标定点 | 左腿 q (rad) | 左腿 L (m) | 右腿 q (rad) | 右腿 L (m)
 *   -------+--------------+------------+---------------+------------
 *   短腿   | 2.0251       | 0.064      | 1.8324        | 0.064
 *   中腿   | 2.3806       | 0.135      | 1.4353        | 0.135
 *   长腿   | 2.9028       | 0.205      | 0.9676        | 0.205
 *
 * 2026-05-04 更新：短腿 q 重新测量（原值 2.2097/1.6711，现 2.0251/1.8324）。
 * 旧值与四连杆模型不一致；修正后的值验证了杆长，一致性在 2.5 度以内。
 *
 * These anchor points are used by the piecewise-linear backup code
 * (in #if 0).  The active four-bar model uses them for offset calibration
 * and validation only.
 *
 * Left:  angle ↑ = leg extends  (L increases with q)
 * Right: angle ↓ = leg extends  (L decreases with q — mirror geometry)
 *
 *    Point | Left q (rad) | Left L (m) | Right q (rad) | Right L (m)
 *   -------+--------------+------------+---------------+------------
 *   short  | 2.0251       | 0.064      | 1.8324        | 0.064
 *   middle | 2.3806       | 0.135      | 1.4353        | 0.135
 *   long   | 2.9028       | 0.205      | 0.9676        | 0.205
 *
 * Updated 2026-05-04: short-leg q re-measured (was 2.2097/1.6711, now
 * 2.0251/1.8324).  The old values were inconsistent with the four-bar
 * model; the corrected values validate the link lengths at <2.5° spread.
 */

/* 左腿三点标定：关节角度 q (rad) 与对应腿长 L (m) */
#define APP_LEFT_LEG_CAL_Q1  2.0251f  /* 左腿标定点1：短腿，关节角度 2.0251 rad */
#define APP_LEFT_LEG_CAL_L1  0.064f   /* 左腿标定点1：短腿，腿长 0.064 m (64 mm) */
#define APP_LEFT_LEG_CAL_Q2  2.3806f  /* 左腿标定点2：中腿，关节角度 2.3806 rad */
#define APP_LEFT_LEG_CAL_L2  0.135f   /* 左腿标定点2：中腿，腿长 0.135 m (135 mm) */
#define APP_LEFT_LEG_CAL_Q3  2.9028f  /* 左腿标定点3：长腿，关节角度 2.9028 rad */
#define APP_LEFT_LEG_CAL_L3  0.205f   /* 左腿标定点3：长腿，腿长 0.205 m (205 mm) */

/* 右腿三点标定：关节角度 q (rad) 与对应腿长 L (m)（注意：右腿标定点按角度降序排列） */
#define APP_RIGHT_LEG_CAL_Q1 0.9676f  /* 右腿标定点1：长腿，关节角度 0.9676 rad */
#define APP_RIGHT_LEG_CAL_L1 0.205f   /* 右腿标定点1：长腿，腿长 0.205 m (205 mm) */
#define APP_RIGHT_LEG_CAL_Q2 1.4353f  /* 右腿标定点2：中腿，关节角度 1.4353 rad */
#define APP_RIGHT_LEG_CAL_L2 0.135f   /* 右腿标定点2：中腿，腿长 0.135 m (135 mm) */
#define APP_RIGHT_LEG_CAL_Q3 1.8324f  /* 右腿标定点3：短腿，关节角度 1.8324 rad */
#define APP_RIGHT_LEG_CAL_L3 0.064f   /* 右腿标定点3：短腿，腿长 0.064 m (64 mm) */

/*
 * 关节角度软限位（DM4340，单位 rad）
 * Joint-angle soft limits (DM4340 rad).
 *
 * 这些限位约束了逆运动学搜索范围，必须在安全机械行程之内。
 * 每个软限位包含约 0.03 rad 的安全裕量，远离机械止档。
 * 注意：左右腿的关节角度-腿长方向相反。
 *
 * 2026-05-04 重新测量（手动，机器人未使能）：
 *   左腿：最短腿 = 2.0251  |  最长腿 = 2.9028  （角度增大 → 伸长）
 *   右腿：最短腿 = 1.8324  |  最长腿 = 0.9676  （角度减小 → 伸长）
 *
 * These bound the inverse-kinematics search and must lie inside the safe
 * mechanical range.
 *
 * Re-measured 2026-05-04 (manual, robot enable 0):
 *   left:  min-leg = 2.0251  |  max-leg = 2.9028  (angle ↑ = extend)
 *   right: min-leg = 1.8324  |  max-leg = 0.9676  (angle ↓ = extend)
 *
 * Each soft limit includes ~0.03 rad safety margin from the mechanical stop.
 * Note: left and right have opposite joint-angle-vs-leg-length directions.
 */
#define APP_ASCENTO_LEFT_JOINT_AT_MIN_LEG_RAD  2.055f  /* 左腿最短腿时关节角度，单位 rad（2.0251 + 0.03 安全裕量） */
#define APP_ASCENTO_LEFT_JOINT_AT_MAX_LEG_RAD  2.87f   /* 左腿最长腿时关节角度，单位 rad（2.9028 - 0.03 安全裕量） */
#define APP_ASCENTO_RIGHT_JOINT_AT_MIN_LEG_RAD 1.80f   /* 右腿最短腿时关节角度，单位 rad（1.8324 - 0.03 安全裕量） */
#define APP_ASCENTO_RIGHT_JOINT_AT_MAX_LEG_RAD 1.00f   /* 右腿最长腿时关节角度，单位 rad（0.9676 + 0.03 安全裕量） */

/* ======================== LQR 增益调度多项式系数（旧版兼容） ======================== */

/*
 * 活跃控制器使用的固定平衡增益（见下方 K_* 宏）。
 * 以下旧版多项式增益调度系数仅为兼容旧 Flash 数据而保留。
 * 运行中的控制器现在直接使用固定的 K_* 宏。
 *
 * Fixed balance gains used by the active controller.
 *
 * The legacy polynomial gain-schedule coefficients below are kept only for
 * backward compatibility with old flash data. The running controller now uses
 * the fixed K_* macros directly.
 */

/* 旧版增益调度多项式系数（C0~C3），仅用于旧 Flash 数据的向后兼容 */
#define APP_ASCENTO_GAIN_C0_A  -8.0019f   /* C0 项系数 a，俯仰角增益多项式 */
#define APP_ASCENTO_GAIN_C0_B   2.9106f   /* C0 项系数 b，俯仰角增益多项式 */
#define APP_ASCENTO_GAIN_C0_C  -12.0000f  /* C0 项系数 c，俯仰角增益多项式 */
#define APP_ASCENTO_GAIN_C1_A  -19.1727f  /* C1 项系数 a，俯仰角速度增益多项式 */
#define APP_ASCENTO_GAIN_C1_B   7.0983f   /* C1 项系数 b，俯仰角速度增益多项式 */
#define APP_ASCENTO_GAIN_C1_C  -5.0000f   /* C1 项系数 c，俯仰角速度增益多项式 */
#define APP_ASCENTO_GAIN_C2      0.9000f  /* C2 项系数，位置增益 */
#define APP_ASCENTO_GAIN_C3_A  -6.1109f   /* C3 项系数 a，速度增益多项式 */
#define APP_ASCENTO_GAIN_C3_B   2.3991f   /* C3 项系数 b，速度增益多项式 */
#define APP_ASCENTO_GAIN_C3_C   0.8500f   /* C3 项系数 c，速度增益多项式 */

/* ======================== 平衡俯仰角与 LQR 固定增益 ======================== */

/*
 * 站立腿长时的平衡俯仰角（单位：弧度）
 * Equilibrium pitch angle (rad) at the stand leg length.
 *
 * 默认标定的站立目标：前倾 +18 度 = +0.314159 rad。
 * 在模型坐标系中（经 IMU 符号校正）：正俯仰 = 前倾。
 *
 * 台架测试：0.0f（平衡点 = 直立，倾斜方向随倾斜反转）
 * 地面接触：+0.314159f（前倾 18 度）
 *
 * TODO: 将 theta_eq 设为腿长 L 的函数。
 *
 * Default tuned stand target: +18 deg forward lean = +0.314159 rad.
 *
 * In model coordinates (IMU-sign-corrected): positive pitch = forward tilt.
 *
 * FOR BENCH TEST:  0.0f        (equilibrium = upright, direction reverses with tilt)
 * FOR GROUND CONTACT: +0.314159f  (18 deg forward lean)
 *
 * TODO: make theta_eq a function of leg length L.
 */
#define APP_ASCENTO_THETA_EQ_STAND_RAD 0.315f  /* 平衡俯仰角，单位 rad（18.0 度前倾，地面接触模式） */

/* 活跃控制器使用的 LQR 固定增益默认值。 */
/* Fixed gain defaults used by the active controller. */
#define APP_ASCENTO_K_PITCH                -55.00f   /* LQR 俯仰角增益 K_pitch（Nm/rad）— 恢复力 */
#define APP_ASCENTO_K_PITCH_RATE           -20.00f   /* LQR 俯仰角速度增益 K_pitch_rate（Nm/(rad/s)）— 增加阻尼ζ≈0.75 */
#define APP_ASCENTO_K_POSITION             1.00f     /* LQR 位置增益 K_position（Nm/m）— 防止位置漂移 */
#define APP_ASCENTO_K_VELOCITY             3.00f     /* LQR 速度增益 K_velocity（Nm/(m/s)）— 速度阻尼 */
#define APP_ASCENTO_K_YAW_RATE              0.0f    /* LQR 偏航角速度增益 K_yaw_rate，0 = 不使用偏航控制 */
#define APP_ASCENTO_K_ROLL_TO_LEG_M_PER_RAD 0.0f    /* 横滚角到腿长差动补偿的增益，0 = 不使用横滚补偿（m/rad） */

/* ======================== IMU 符号校正 ======================== */

/*
 * 模型级 IMU 符号校正
 * Model-level IMU sign corrections.
 *
 * 这些符号与 PID 控制器的 APP_PID_BALANCE_AXIS_SIGN 是独立的。
 * Ascento 模型从 bmi088.c 读取校正后的 IMU 字段（俯仰/横滚轴已根据
 * PCB 实际安装方向重映射，2026-05-05），并应用这些符号以匹配模型坐标约定：
 *   前倾  → pitch 为正
 *   左倾  → roll 为正
 *   前倾时的俯仰角速度 → gy 为正
 *   逆时针（从上方看） → gz 为正
 *
 * 2026-05-04 验证，2026-05-05 bmi088 轴修复后重新检查：
 *   pitch：前倾 → 校正后 pitch_deg 为负 → sign = -1
 *   roll：左倾 → 校正后 roll_deg 为正 → sign = +1
 *   gz：偏航角速度逆时针 → 原始 gz 为正 → sign = +1
 *   gy：前点头 → 原始 gx_dps 为正（芯片 X 轴 = 机器人俯仰轴）
 *       → 模型 pitch_rate 必须为正 → sign = +1
 *
 * These are separate from the PID controller's APP_PID_BALANCE_AXIS_SIGN.
 * The Ascento model reads corrected IMU fields from bmi088.c (pitch/roll
 * axes remapped for the physical PCB mounting, 2026-05-05) and applies
 * these signs to match the model's coordinate convention:
 *   forward tilt  → pitch positive
 *   left tilt     → roll positive
 *   pitch rate during forward tilt → gy positive
 *   counter-clockwise (from above) → gz positive
 *
 * Verified 2026-05-04, re-checked after bmi088 axis fix 2026-05-05:
 *   pitch: forward tilt → corrected pitch_deg negative → sign = -1
 *   roll:  left tilt    → corrected roll_deg positive  → sign = +1
 *   gz:    yaw rate CCW → raw gz positive             → sign = +1
 *   gy:    forward nod  → raw gx_dps positive (chip X = robot pitch axis)
 *          → model pitch_rate must be positive → sign = +1
 */
#define APP_ASCENTO_IMU_PITCH_SIGN (1.0f)   /* 俯仰角符号校正：+1 = 原始正 pitch 对应模型前倾（测试：原-1导致前倒） */
#define APP_ASCENTO_IMU_ROLL_SIGN  (1.0f)   /* 横滚角符号校正：+1 表示原始正 roll 对应模型的左倾 */
#define APP_ASCENTO_IMU_GY_SIGN    (1.0f)   /* 俯仰角速度(gy)符号校正：+1 表示原始正值对应前倾方向 */
#define APP_ASCENTO_IMU_GZ_SIGN    (1.0f)   /* 偏航角速度(gz)符号校正：+1 表示原始正值对应逆时针方向 */
#define APP_IMU_ROLL_ZERO_DEG      2.5f     /* 横滚角零位偏移，单位 deg，用于校正 IMU 安装倾斜 */
#define APP_IMU_PITCH_ZERO_DEG     0.0f     /* 俯仰角零位偏移，单位 deg，用于校正 IMU 安装偏差（正值=读数偏大，需减去） */
