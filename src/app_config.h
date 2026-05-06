#pragma once

#include <stdint.h>

#define APP_CONTROL_HZ 200
#define APP_CAN_BITRATE 1000000U

/*
 * Model-based Ascento controller — ACTIVE (2026-05-05).
 * Physical parameters measured, four-bar kinematics validated,
 * LQR gain schedule implemented with polynomial fit.
 * Wheel direction corrected — GROUND CONTACT / standing test ready.
 */
#define APP_USE_ASCENTO_BALANCE_CONTROLLER 1
#define APP_ASCENTO_PARAMS_CALIBRATED 1

/*
 * CAN routing is defined by devicetree aliases: joint=CAN1, wheel=CAN2.
 * Wheel motors use VESC controllers on CAN2.
 */
#define APP_WHEEL_LEFT_ID 101
#define APP_WHEEL_RIGHT_ID 100

/*
 * Wheel direction calibration from the VESC current test.
 *
 * Semantics:
 * - Controller/model wheel current is robot-forward positive.
 * - Raw VESC debug current is left untouched.
 * - raw_vesc_current = logical_forward_current * APP_WHEEL_*_FORWARD_CURRENT_SIGN
 * - logical_forward_speed = raw_vesc_speed * APP_WHEEL_*_FORWARD_CURRENT_SIGN
 *
 * Field-tuned 2026-05-05:
 * CAN ID 100 is mounted/wired opposite to ID 101, so its logical forward
 * current uses the opposite raw VESC-current sign.
 */
#define APP_WHEEL_LEFT_FORWARD_CURRENT_SIGN -1
#define APP_WHEEL_RIGHT_FORWARD_CURRENT_SIGN 1
#define APP_M3508_DIRECTION_TEST_CURRENT_MA 100
#define APP_M3508_DIRECTION_TEST_DURATION_MS 3000

/*
 * DM motors are on CAN1. Command IDs are the motor IDs; feedback IDs are the
 * Master IDs configured in the Damiao tool.
 */
#define APP_DM_LEFT_ID 1
#define APP_DM_RIGHT_ID 2
#define APP_DM_LEFT_FEEDBACK_ID 0x11
#define APP_DM_RIGHT_FEEDBACK_ID 0x12
#define APP_DM_MASTER_ID APP_DM_LEFT_FEEDBACK_ID

#define APP_WHEEL_CURRENT_LIMIT 3000
#define APP_WHEEL_CURRENT_SAFE 800

/*
 * Wheel current scale factor.
 *
 * GROUND CONTACT: 1.0f (use raw LQR gains).
 * BENCH TEST: 3.0f–5.0f to overcome VESC minimum current threshold (~0.5–1 A)
 * so small tilts produce visible wheel movement.
 */
#define APP_ASCENTO_WHEEL_CURRENT_SCALE 1.45f  /* stronger fall recovery */
#define APP_M3508_REDUCTION_RATIO 19.203208f
#define APP_M3508_MOTOR_KT_NM_PER_A 0.180f
#define APP_M3508_GEARBOX_EFFICIENCY 1.000f
#define APP_VESC_CURRENT_CMD_TO_AMP 0.001f
#define APP_VESC_MOTOR_POLE_PAIRS 7.0f
#define APP_VESC_STATUS_RATE_HZ 200
#define APP_VESC_FEEDBACK_TIMEOUT_MS 50
#define APP_VESC_DEBUG_ERPM_LIMIT 8000

#define APP_M3508_CALC_CURRENT_MA_TO_WHEEL_TORQUE_NM \
	(APP_M3508_MOTOR_KT_NM_PER_A * APP_M3508_REDUCTION_RATIO * \
	 APP_M3508_GEARBOX_EFFICIENCY * APP_VESC_CURRENT_CMD_TO_AMP)
#define APP_ASCENTO_LEFT_CURRENT_MA_TO_WHEEL_TORQUE_NM \
	APP_M3508_CALC_CURRENT_MA_TO_WHEEL_TORQUE_NM
#define APP_ASCENTO_RIGHT_CURRENT_MA_TO_WHEEL_TORQUE_NM \
	APP_M3508_CALC_CURRENT_MA_TO_WHEEL_TORQUE_NM
#define APP_ASCENTO_CURRENT_MA_TO_WHEEL_TORQUE_NM \
	((APP_ASCENTO_LEFT_CURRENT_MA_TO_WHEEL_TORQUE_NM + \
	  APP_ASCENTO_RIGHT_CURRENT_MA_TO_WHEEL_TORQUE_NM) * 0.5f)

#define APP_MOTOR_DEBUG_DEFAULT_TIMEOUT_MS 1000
#define APP_MOTOR_DEBUG_MAX_TIMEOUT_MS 5000
#define APP_M3508_DEBUG_CURRENT_LIMIT APP_WHEEL_CURRENT_SAFE
#define APP_DM_FEEDBACK_TIMEOUT_MS 100
#define APP_DM_ERROR_REENABLE_INTERVAL_MS 200
#define APP_DM_DEBUG_VEL_LIMIT_RAD_S 4.0f
#define APP_DM_DEBUG_KP_LIMIT 80.0f
#define APP_DM_DEBUG_KD_LIMIT 2.0f
#define APP_DM_DEBUG_TORQUE_LIMIT_NM 2.0f

#define APP_CMD_TIMEOUT_MS 700
#define APP_BALANCE_RECOVER_TICKS 200
#define APP_PITCH_FAULT_DEG 70.0f
#define APP_PITCH_RECOVER_DEG 10.0f

/*
 * Hard fall-protection caps for the Ascento model controller.
 *
 * Runtime fault_deg remains tunable, but these caps cannot be relaxed by
 * previously saved flash parameters. Backward fall is intentionally stricter:
 * once the body is far behind the wheel axle, continuing wheel drive is more
 * likely to drag the robot than recover it.
 */
#define APP_ASCENTO_FORWARD_HARD_FAULT_DEG 70.0f
#define APP_ASCENTO_BACKWARD_HARD_FAULT_DEG 30.0f
#define APP_ASCENTO_STAND_RECOVER_ERR_DEG 18.0f

#define APP_DEFAULT_HEIGHT 38
#define APP_HEIGHT_MIN 32
#define APP_HEIGHT_MAX 80

/*
 * Debug clamp limits for DM4340 joint positions (rad).
 * These bound motor_debug commands (pos, mit, wiggle) and must cover
 * the full physical range of the DM4340 joints. Updated from old
 * STS3032 values (0.07~0.70) to match DM4340 working range.
 *
 * Left:  measured min-leg 2.0251 rad → clamp 1.9 ~ 3.0
 * Right: measured min-leg 1.8324 rad → clamp 0.8 ~ 2.0
 */
#define APP_LEG_RAD_PER_HEIGHT_UNIT 0.012884f
#define APP_LEFT_LEG_ZERO_RAD 0.0f
#define APP_RIGHT_LEG_ZERO_RAD 0.0f
#define APP_LEFT_LEG_MIN_RAD  1.9f
#define APP_LEFT_LEG_MAX_RAD  3.0f
#define APP_RIGHT_LEG_MIN_RAD 0.8f
#define APP_RIGHT_LEG_MAX_RAD 2.0f
#define APP_LEG_VEL_LIMIT_RAD_S 12.0f
#define APP_ROLL_RAD_PER_DEG 0.010f
#define APP_ROLL_COMP_LIMIT_RAD 0.22f

#define APP_JOY_TO_SPEED_RAD_S 0.030f
#define APP_JOY_TO_YAW_RATE_DEG_S 0.20f

/*
 * Independent PID balance-car controller. This keeps the DM joints at one
 * fixed medium height and only uses the M3508/VESC wheels for balancing.
 */
#define APP_PID_BALANCE_LOCK_HEIGHT 45
/*
 * DM4340 joint positions at stand leg length (~0.115 m).
 *
 * Interpolated from the 3-point calibration table for 0.115 m, with the
 * right leg shortened slightly to level the body:
 *   left:  2.28 rad (between short 2.0251 and middle 2.3806)
 *   right: 1.58 rad (about 5 mm shorter than the 1.55 rad nominal point)
 *
 * These are used as PID lock positions (power-on default before enable)
 * and as the fallback when the Ascento model is blocked.
 */
#define APP_PID_BALANCE_LOCK_LEFT_JOINT_RAD 2.28f
#define APP_PID_BALANCE_LOCK_RIGHT_JOINT_RAD 1.58f
#define APP_PID_BALANCE_JOINT_USE_MIT 1
#define APP_PID_BALANCE_JOINT_MIT_KP 100.0f  /* 50→100: stiffer joint lock */
#define APP_PID_BALANCE_JOINT_MIT_KD 12.0f   /* 8→12: more damping */
#define APP_PID_BALANCE_USE_ROLL_AXIS 1
#define APP_PID_BALANCE_AXIS_SIGN 1.0f
#define APP_PID_BALANCE_ZERO_DEG 0.70f
#define APP_PID_BALANCE_ZERO_LIMIT_DEG 45.0f
#define APP_PID_BALANCE_ZERO_NOW_LIMIT_DEG 8.0f
#define APP_PID_BALANCE_CURRENT_LIMIT APP_WHEEL_CURRENT_LIMIT
#define APP_PID_BALANCE_INTEGRAL_LIMIT 0.0f
#define APP_PID_BALANCE_ANGLE_P 900.0f
#define APP_PID_BALANCE_ANGLE_I 0.0f
#define APP_PID_BALANCE_ANGLE_D 0.0f
#define APP_PID_BALANCE_GYRO_P 35.0f
#define APP_PID_BALANCE_DISTANCE_P 50.0f
#define APP_PID_BALANCE_SPEED_P 1000.0f
#define APP_PID_BALANCE_STICTION_CURRENT 2600.0f
#define APP_PID_BALANCE_STICTION_START_DEG 0.50f
#define APP_PID_BALANCE_STICTION_FULL_DEG 2.00f

/*
 * Stiction compensation for the Ascento model controller.
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
#define APP_ASCENTO_STICTION_CURRENT_MA 0.0f
#define APP_ASCENTO_STICTION_START_DEG 0.18f
#define APP_ASCENTO_STICTION_FULL_DEG 2.40f

/*
 * Wheel speed synchronization compensation.
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
#define APP_ASCENTO_WHEEL_SYNC_GAIN_MA 0.0f
#define APP_ASCENTO_WHEEL_SYNC_CURRENT_LIMIT 500.0f

#define APP_PID_BALANCE_JOY_LPF_TIME_S 0.20f
#define APP_PID_BALANCE_JOY_TO_SPEED_RAD_S 0.100f
#define APP_PID_BALANCE_JOY_TO_CURRENT 80.0f
#define APP_PID_BALANCE_WHEEL_SYNC_P 350.0f
#define APP_PID_BALANCE_WHEEL_SYNC_CURRENT_LIMIT 1800.0f
#define APP_PID_BALANCE_JOY_TO_YAW_CURRENT 6.0f
#define APP_PID_BALANCE_YAW_CURRENT_LIMIT 600.0f
#define APP_PID_BALANCE_MOTION_JOY_X 20.0f
#define APP_PID_BALANCE_MOTION_JOY_Y 30.0f
#define APP_PID_BALANCE_DISTANCE_RESET_SPEED_RAD_S 45.0f

/* Legacy controller gains. Tune with wheels suspended first. */
#define APP_PID_ANGLE_P 300.0f
#define APP_PID_ANGLE_I 0.0f
#define APP_PID_ANGLE_D 0.0f
#define APP_PID_GYRO_P 10.0f
#define APP_PID_DISTANCE_P 700.0f
#define APP_PID_SPEED_P 520.0f
#define APP_PID_YAW_ANGLE_P 80.0f
#define APP_PID_YAW_GYRO_P 5.0f

#define APP_ANGLE_ZERO_DEG -1.225f
#define APP_YAW_ZERO_DEG 0.0f

#define APP_BATTERY_ADC_CHANNEL 8
#define APP_BATTERY_ADC_FULL_SCALE_MV 3300.0f
#define APP_BATTERY_DIVIDER_RATIO ((200.0f + 22.0f) / 22.0f)
#define APP_BATTERY_LED_THRESHOLD_V 20.5f

/* Measured physical data for the dormant Ascento controller. */
#define APP_ASCENTO_WHEEL_RADIUS_M 0.030f
#define APP_ASCENTO_WHEEL_BASE_M 0.201f

/*
 * Mass breakdown (2026-05-04):
 *   body (without legs/wheels): 1.7514 kg
 *   single leg links (m1+m2+m3+m23): 0.1108 kg
 *   both legs: 0.2216 kg
 *   single wheel: 0.171 kg
 *   both wheels: 0.342 kg
 *   total: 1.7514 + 0.2216 + 0.342 ≈ 2.315 kg
 */
#define APP_ASCENTO_TOTAL_MASS_KG 2.315f
#define APP_ASCENTO_BODY_MASS_KG 1.7514f
#define APP_ASCENTO_SINGLE_WHEEL_MASS_KG 0.171f
#define APP_ASCENTO_SINGLE_LEG_MASS_KG 0.1108f

#define APP_ASCENTO_BODY_COM_HEIGHT_M 0.098f
#define APP_ASCENTO_BODY_COM_FORWARD_OFFSET_M (-0.0245f)
#define APP_ASCENTO_BODY_PITCH_INERTIA_KG_M2 0.060f

/*
 * Single wheel inertia: I = 0.5 * m_w * r² = 0.5 * 0.171 * 0.030² = 7.7e-5
 */
#define APP_ASCENTO_WHEEL_INERTIA_KG_M2 0.000077f

/* Leg lengths (physically measured 2026-05-04). */
#define APP_ASCENTO_LEG_LENGTH_MIN_M 0.064f
#define APP_ASCENTO_LEG_LENGTH_MAX_M 0.205f
#define APP_ASCENTO_LEG_LENGTH_DEFAULT_M 0.1345f
#define APP_ASCENTO_LEG_LENGTH_STAND_M 0.115f

/*
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
#define APP_ASCENTO_FB_L1   0.1121f
#define APP_ASCENTO_FB_L2   0.1100f
#define APP_ASCENTO_FB_L3   0.1130f
#define APP_ASCENTO_FB_L4   0.0580f
#define APP_ASCENTO_FB_L23  0.0306f

/*
 * Offset from DM4340 joint-angle to four-bar qC.
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
#define APP_ASCENTO_FB_JOINT_ZERO_OFFSET_LEFT_RAD  4.892f
#define APP_ASCENTO_FB_JOINT_ZERO_OFFSET_RIGHT_RAD 2.098f

/*
 * Reference-point offset between the four-bar model's y=0 plane and the
 * physical reference point used for leg-length measurements.
 *
 * The model computes leg_length = -E.y (distance from motor shaft centre C
 * to wheel axle E, projected onto the vertical axis).  The user measures
 * leg length from a reference plane ~20 mm below the motor shaft centre.
 * Set this to the measured vertical distance between the two reference
 * planes so the model output matches physical measurements.
 *
 * Current best estimate: 0.0 mm (unverified — measure with calipers).
 */
#define APP_ASCENTO_LEG_LENGTH_REF_OFFSET_M 0.0f

/*
 * Measured 3-point joint-angle ↔ leg-length calibration (2026-05-04).
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
#define APP_LEFT_LEG_CAL_Q1  2.0251f
#define APP_LEFT_LEG_CAL_L1  0.064f
#define APP_LEFT_LEG_CAL_Q2  2.3806f
#define APP_LEFT_LEG_CAL_L2  0.135f
#define APP_LEFT_LEG_CAL_Q3  2.9028f
#define APP_LEFT_LEG_CAL_L3  0.205f

#define APP_RIGHT_LEG_CAL_Q1 0.9676f
#define APP_RIGHT_LEG_CAL_L1 0.205f
#define APP_RIGHT_LEG_CAL_Q2 1.4353f
#define APP_RIGHT_LEG_CAL_L2 0.135f
#define APP_RIGHT_LEG_CAL_Q3 1.8324f
#define APP_RIGHT_LEG_CAL_L3 0.064f

/*
 * Joint-angle soft limits (DM4340 rad). These bound the inverse-kinematics
 * search and must lie inside the safe mechanical range.
 *
 * Re-measured 2026-05-04 (manual, robot enable 0):
 *   left:  min-leg = 2.0251  |  max-leg = 2.9028  (angle ↑ = extend)
 *   right: min-leg = 1.8324  |  max-leg = 0.9676  (angle ↓ = extend)
 *
 * Each soft limit includes ~0.03 rad safety margin from the mechanical stop.
 * Note: left and right have opposite joint-angle-vs-leg-length directions.
 */
#define APP_ASCENTO_LEFT_JOINT_AT_MIN_LEG_RAD  2.055f  /* 2.0251 + 0.03 */
#define APP_ASCENTO_LEFT_JOINT_AT_MAX_LEG_RAD  2.87f   /* 2.9028 - 0.03, unchanged */
#define APP_ASCENTO_RIGHT_JOINT_AT_MIN_LEG_RAD 1.80f   /* 1.8324 - 0.03 */
#define APP_ASCENTO_RIGHT_JOINT_AT_MAX_LEG_RAD 1.00f   /* 0.9676 + 0.03, unchanged */

/*
 * Fixed balance gains used by the active controller.
 *
 * The legacy polynomial gain-schedule coefficients below are kept only for
 * backward compatibility with old flash data. The running controller now uses
 * the fixed K_* macros directly.
 */
#define APP_ASCENTO_GAIN_C0_A  -8.0019f
#define APP_ASCENTO_GAIN_C0_B   2.9106f
#define APP_ASCENTO_GAIN_C0_C  -12.0000f
#define APP_ASCENTO_GAIN_C1_A  -19.1727f
#define APP_ASCENTO_GAIN_C1_B   7.0983f
#define APP_ASCENTO_GAIN_C1_C  -5.0000f
#define APP_ASCENTO_GAIN_C2      0.9000f
#define APP_ASCENTO_GAIN_C3_A  -6.1109f
#define APP_ASCENTO_GAIN_C3_B   2.3991f
#define APP_ASCENTO_GAIN_C3_C   0.8500f

/*
 * Equilibrium pitch angle (rad) at the stand leg length.
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
#define APP_ASCENTO_THETA_EQ_STAND_RAD 0.314159f  /* 18 deg, ground contact */

/* Fixed gain defaults used by the active controller. */
#define APP_ASCENTO_K_PITCH      -11.75f
#define APP_ASCENTO_K_PITCH_RATE -4.40f
#define APP_ASCENTO_K_POSITION    0.90f
#define APP_ASCENTO_K_VELOCITY    1.05f
#define APP_ASCENTO_K_YAW_RATE 0.0f
#define APP_ASCENTO_K_ROLL_TO_LEG_M_PER_RAD 0.0f

/*
 * Model-level IMU sign corrections.
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
#define APP_ASCENTO_IMU_PITCH_SIGN (-1.0f)
#define APP_ASCENTO_IMU_ROLL_SIGN  (1.0f)
#define APP_ASCENTO_IMU_GY_SIGN    (1.0f)
#define APP_ASCENTO_IMU_GZ_SIGN    (1.0f)
#define APP_IMU_ROLL_ZERO_DEG      2.5f
