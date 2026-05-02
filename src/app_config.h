#pragma once

#include <stdint.h>

#define APP_CONTROL_HZ 1000
#define APP_CAN_BITRATE 1000000U

/*
 * Dormant model-based Ascento controller. Keep disabled until the robot's
 * physical parameters and LQR gains have been measured and validated.
 */
#define APP_USE_ASCENTO_BALANCE_CONTROLLER 0
#define APP_ASCENTO_PARAMS_CALIBRATED 0

/*
 * CAN routing is defined by devicetree aliases: joint=CAN1, wheel=CAN2.
 * Wheel motors use VESC controllers on CAN2.
 */
#define APP_WHEEL_LEFT_ID 101
#define APP_WHEEL_RIGHT_ID 100

/*
 * DM motors are on a separate bus from the M3508 wheels, so IDs may also be
 * 1/2. Change APP_DM_MASTER_ID if the Damiao upper computer uses another
 * feedback/master ID.
 */
#define APP_DM_LEFT_ID 1
#define APP_DM_RIGHT_ID 2
#define APP_DM_MASTER_ID 0x00

#define APP_WHEEL_CURRENT_LIMIT 6500
#define APP_WHEEL_CURRENT_SAFE 2500
#define APP_M3508_REDUCTION_RATIO 19.203208f
#define APP_VESC_CURRENT_CMD_TO_AMP 0.001f
#define APP_VESC_MOTOR_POLE_PAIRS 7.0f
#define APP_VESC_STATUS_RATE_HZ 200
#define APP_VESC_FEEDBACK_TIMEOUT_MS 50
#define APP_VESC_DEBUG_ERPM_LIMIT 8000

#define APP_MOTOR_DEBUG_DEFAULT_TIMEOUT_MS 1000
#define APP_MOTOR_DEBUG_MAX_TIMEOUT_MS 5000
#define APP_M3508_DEBUG_CURRENT_LIMIT APP_WHEEL_CURRENT_SAFE
#define APP_DM_DEBUG_VEL_LIMIT_RAD_S 4.0f
#define APP_DM_DEBUG_KP_LIMIT 80.0f
#define APP_DM_DEBUG_KD_LIMIT 2.0f
#define APP_DM_DEBUG_TORQUE_LIMIT_NM 2.0f

#define APP_CMD_TIMEOUT_MS 700
#define APP_BALANCE_RECOVER_TICKS 200
#define APP_PITCH_FAULT_DEG 25.0f
#define APP_PITCH_RECOVER_DEG 10.0f

#define APP_DEFAULT_HEIGHT 38
#define APP_HEIGHT_MIN 32
#define APP_HEIGHT_MAX 80

/*
 * Original STS3032 mapping was about 8.4 encoder counts per height unit.
 * 8.4 * 2*pi / 4096 = 0.012884 rad per height unit.
 */
#define APP_LEG_RAD_PER_HEIGHT_UNIT 0.012884f
#define APP_LEFT_LEG_ZERO_RAD 0.0f
#define APP_RIGHT_LEG_ZERO_RAD 0.0f
#define APP_LEFT_LEG_MIN_RAD 0.07f
#define APP_LEFT_LEG_MAX_RAD 0.70f
#define APP_RIGHT_LEG_MIN_RAD -0.70f
#define APP_RIGHT_LEG_MAX_RAD -0.07f
#define APP_LEG_VEL_LIMIT_RAD_S 12.0f
#define APP_ROLL_RAD_PER_DEG 0.010f
#define APP_ROLL_COMP_LIMIT_RAD 0.22f

#define APP_JOY_TO_SPEED_RAD_S 0.030f
#define APP_JOY_TO_YAW_RATE_DEG_S 0.20f

/* Conservative starting gains. Tune with wheels suspended first. */
#define APP_PID_ANGLE_P 300.0f
#define APP_PID_ANGLE_I 0.0f
#define APP_PID_ANGLE_D 0.0f
#define APP_PID_GYRO_P 10.0f
#define APP_PID_DISTANCE_P 700.0f
#define APP_PID_SPEED_P 520.0f
#define APP_PID_YAW_ANGLE_P 80.0f
#define APP_PID_YAW_GYRO_P 5.0f

#define APP_ANGLE_ZERO_DEG -2.25f
#define APP_YAW_ZERO_DEG 0.0f

#define APP_BATTERY_ADC_CHANNEL 8
#define APP_BATTERY_ADC_FULL_SCALE_MV 3300.0f
#define APP_BATTERY_DIVIDER_RATIO ((200.0f + 22.0f) / 22.0f)
#define APP_BATTERY_LED_THRESHOLD_V 20.5f

/* Placeholder physical data for the dormant Ascento controller. */
#define APP_ASCENTO_WHEEL_RADIUS_M 0.060f
#define APP_ASCENTO_WHEEL_BASE_M 0.250f
#define APP_ASCENTO_TOTAL_MASS_KG 8.000f
#define APP_ASCENTO_BODY_COM_HEIGHT_M 0.180f
#define APP_ASCENTO_BODY_PITCH_INERTIA_KG_M2 0.060f
#define APP_ASCENTO_WHEEL_INERTIA_KG_M2 0.00035f
#define APP_ASCENTO_CURRENT_MA_TO_WHEEL_TORQUE_NM 0.00045f
#define APP_ASCENTO_LEG_LENGTH_MIN_M 0.120f
#define APP_ASCENTO_LEG_LENGTH_MAX_M 0.360f
#define APP_ASCENTO_LEG_LENGTH_DEFAULT_M 0.220f

/* Linear joint-to-leg-length map until the real four-bar curve is measured. */
#define APP_ASCENTO_LEFT_JOINT_AT_MIN_LEG_RAD APP_LEFT_LEG_MIN_RAD
#define APP_ASCENTO_LEFT_JOINT_AT_MAX_LEG_RAD APP_LEFT_LEG_MAX_RAD
#define APP_ASCENTO_RIGHT_JOINT_AT_MIN_LEG_RAD APP_RIGHT_LEG_MAX_RAD
#define APP_ASCENTO_RIGHT_JOINT_AT_MAX_LEG_RAD APP_RIGHT_LEG_MIN_RAD

/* Safe placeholder gains; replace with identified LQR gains before use. */
#define APP_ASCENTO_K_PITCH 0.0f
#define APP_ASCENTO_K_PITCH_RATE 0.0f
#define APP_ASCENTO_K_POSITION 0.0f
#define APP_ASCENTO_K_VELOCITY 0.0f
#define APP_ASCENTO_K_YAW_RATE 0.0f
#define APP_ASCENTO_K_ROLL_TO_LEG_M_PER_RAD 0.0f
