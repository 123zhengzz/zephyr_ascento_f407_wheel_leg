#include "control.h"

#include <math.h>
#include <string.h>

#include <zephyr/kernel.h>

#include "app_config.h"
#include "pid.h"

typedef struct {
	struct k_mutex lock;
	bool enable_request;
	int height;
	float joy_x;
	float joy_y;
	robot_motion_t motion;
	int64_t last_cmd_ms;

	app_pid_t angle_pid;
	app_pid_t gyro_pid;
	app_pid_t distance_pid;
	app_pid_t speed_pid;

	float angle_zero_deg;
	float distance_zero_rad;
	float joy_y_lpf;
	bool distance_zero_valid;
	bool faulted;
	int recover_ticks;
	int16_t current_limit_ma;
	control_status_t status;
} pid_balance_ctx_t;

static pid_balance_ctx_t ctx;

static int clamp_height(int height)
{
	if (height < APP_HEIGHT_MIN) {
		return APP_HEIGHT_MIN;
	}
	if (height > APP_HEIGHT_MAX) {
		return APP_HEIGHT_MAX;
	}
	return height;
}

static float wheel_forward_sign(bool left_wheel)
{
	return left_wheel ? (float)APP_WHEEL_LEFT_FORWARD_CURRENT_SIGN :
			    (float)APP_WHEEL_RIGHT_FORWARD_CURRENT_SIGN;
}

static float balance_angle_deg(const bmi088_sample_t *imu)
{
#if APP_PID_BALANCE_USE_ROLL_AXIS
	return APP_PID_BALANCE_AXIS_SIGN * imu->roll_deg;
#else
	return APP_PID_BALANCE_AXIS_SIGN * imu->pitch_deg;
#endif
}

static float balance_gyro_dps(const bmi088_sample_t *imu)
{
#if APP_PID_BALANCE_USE_ROLL_AXIS
	return APP_PID_BALANCE_AXIS_SIGN * imu->gx_dps;
#else
	return APP_PID_BALANCE_AXIS_SIGN * imu->gy_dps;
#endif
}

static void reset_balance_pids(void)
{
	app_pid_reset(&ctx.angle_pid);
	app_pid_reset(&ctx.gyro_pid);
	app_pid_reset(&ctx.distance_pid);
	app_pid_reset(&ctx.speed_pid);
	ctx.distance_zero_valid = false;
	ctx.joy_y_lpf = 0.0f;
}

static int16_t clamp_current_limit(int32_t current_limit_ma)
{
	if (current_limit_ma < 0) {
		current_limit_ma = -current_limit_ma;
	}
	if (current_limit_ma < 100) {
		current_limit_ma = 100;
	}
	if (current_limit_ma > APP_WHEEL_CURRENT_LIMIT) {
		current_limit_ma = APP_WHEEL_CURRENT_LIMIT;
	}
	return (int16_t)current_limit_ma;
}

static void apply_pid_output_limit(float limit)
{
	ctx.angle_pid.output_limit = limit;
	ctx.gyro_pid.output_limit = limit;
	ctx.distance_pid.output_limit = limit;
	ctx.speed_pid.output_limit = limit;
}

static void fixed_joint_targets(int height, float *left_joint,
				float *right_joint)
{
	ARG_UNUSED(height);

	*left_joint = APP_PID_BALANCE_LOCK_LEFT_JOINT_RAD;
	*right_joint = APP_PID_BALANCE_LOCK_RIGHT_JOINT_RAD;
}

void control_init(void)
{
	memset(&ctx, 0, sizeof(ctx));
	k_mutex_init(&ctx.lock);

	ctx.height = clamp_height(APP_PID_BALANCE_LOCK_HEIGHT);
	ctx.motion = ROBOT_STOP;
	ctx.last_cmd_ms = k_uptime_get();
	ctx.angle_zero_deg = APP_PID_BALANCE_ZERO_DEG;
	ctx.current_limit_ma =
		clamp_current_limit(APP_PID_BALANCE_CURRENT_LIMIT);

	app_pid_init(&ctx.angle_pid, APP_PID_BALANCE_ANGLE_P,
		     APP_PID_BALANCE_ANGLE_I, APP_PID_BALANCE_ANGLE_D,
		     APP_PID_BALANCE_INTEGRAL_LIMIT,
		     ctx.current_limit_ma);
	app_pid_init(&ctx.gyro_pid, APP_PID_BALANCE_GYRO_P, 0.0f, 0.0f,
		     0.0f, ctx.current_limit_ma);
	app_pid_init(&ctx.distance_pid, APP_PID_BALANCE_DISTANCE_P, 0.0f,
		     0.0f, 0.0f, ctx.current_limit_ma);
	app_pid_init(&ctx.speed_pid, APP_PID_BALANCE_SPEED_P, 0.0f, 0.0f,
		     0.0f, ctx.current_limit_ma);
}

void control_set_enable(bool enable)
{
	k_mutex_lock(&ctx.lock, K_FOREVER);
	ctx.enable_request = enable;
	ctx.last_cmd_ms = k_uptime_get();
	ctx.status.enable_request = enable;
	reset_balance_pids();
	if (enable) {
		ctx.faulted = false;
		ctx.recover_ticks = 0;
	}
	k_mutex_unlock(&ctx.lock);
}

void control_set_height(int height)
{
	k_mutex_lock(&ctx.lock, K_FOREVER);
	ctx.height = clamp_height(height);
	ctx.last_cmd_ms = k_uptime_get();
	k_mutex_unlock(&ctx.lock);
}

void control_set_joystick(float x, float y)
{
	k_mutex_lock(&ctx.lock, K_FOREVER);
	ctx.joy_x = app_clampf(x, -100.0f, 100.0f);
	ctx.joy_y = app_clampf(y, -100.0f, 100.0f);
	ctx.last_cmd_ms = k_uptime_get();
	k_mutex_unlock(&ctx.lock);
}

void control_set_motion(robot_motion_t motion)
{
	k_mutex_lock(&ctx.lock, K_FOREVER);
	ctx.motion = motion == ROBOT_JUMP ? ROBOT_STOP : motion;
	ctx.last_cmd_ms = k_uptime_get();
	k_mutex_unlock(&ctx.lock);
}

void control_request_jump(void)
{
	control_stop_motion();
}

void control_stop_motion(void)
{
	k_mutex_lock(&ctx.lock, K_FOREVER);
	ctx.joy_x = 0.0f;
	ctx.joy_y = 0.0f;
	ctx.motion = ROBOT_STOP;
	ctx.last_cmd_ms = k_uptime_get();
	k_mutex_unlock(&ctx.lock);
}

void control_set_angle_zero(float zero_deg)
{
	k_mutex_lock(&ctx.lock, K_FOREVER);
	ctx.angle_zero_deg = app_clampf(zero_deg,
					-APP_PID_BALANCE_ZERO_LIMIT_DEG,
					APP_PID_BALANCE_ZERO_LIMIT_DEG);
	reset_balance_pids();
	k_mutex_unlock(&ctx.lock);
}

bool control_get_enable_request(void)
{
	bool enable;

	k_mutex_lock(&ctx.lock, K_FOREVER);
	enable = ctx.enable_request;
	k_mutex_unlock(&ctx.lock);

	return enable;
}

void control_get_status(control_status_t *status)
{
	if (status == NULL) {
		return;
	}

	k_mutex_lock(&ctx.lock, K_FOREVER);
	*status = ctx.status;
	k_mutex_unlock(&ctx.lock);
}

void control_publish_status(const control_status_t *status)
{
	if (status == NULL) {
		return;
	}

	k_mutex_lock(&ctx.lock, K_FOREVER);
	ctx.status = *status;
	k_mutex_unlock(&ctx.lock);
}

void control_get_pid_balance_params(control_pid_balance_params_t *params)
{
	if (params == NULL) {
		return;
	}

	k_mutex_lock(&ctx.lock, K_FOREVER);
	*params = (control_pid_balance_params_t) {
		.angle_p = ctx.angle_pid.kp,
		.angle_i = ctx.angle_pid.ki,
		.angle_d = ctx.angle_pid.kd,
		.gyro_p = ctx.gyro_pid.kp,
		.distance_p = ctx.distance_pid.kp,
		.speed_p = ctx.speed_pid.kp,
		.current_limit_ma = ctx.current_limit_ma,
	};
	k_mutex_unlock(&ctx.lock);
}

void control_set_pid_balance_params(
	const control_pid_balance_params_t *params)
{
	if (params == NULL) {
		return;
	}

	k_mutex_lock(&ctx.lock, K_FOREVER);
	ctx.angle_pid.kp = app_clampf(params->angle_p, 0.0f, 3000.0f);
	ctx.angle_pid.ki = app_clampf(params->angle_i, 0.0f, 300.0f);
	ctx.angle_pid.kd = app_clampf(params->angle_d, 0.0f, 300.0f);
	ctx.gyro_pid.kp = app_clampf(params->gyro_p, 0.0f, 300.0f);
	ctx.distance_pid.kp = app_clampf(params->distance_p, 0.0f, 3000.0f);
	ctx.speed_pid.kp = app_clampf(params->speed_p, 0.0f, 3000.0f);
	ctx.current_limit_ma =
		clamp_current_limit(params->current_limit_ma);
	apply_pid_output_limit((float)ctx.current_limit_ma);
	reset_balance_pids();
	k_mutex_unlock(&ctx.lock);
}

static void copy_command(bool *enable, int *height, float *joy_x,
			 float *joy_y, robot_motion_t *motion,
			 int16_t *current_limit_ma)
{
	const int64_t now_ms = k_uptime_get();

	k_mutex_lock(&ctx.lock, K_FOREVER);
	*enable = ctx.enable_request;
	*height = ctx.height;
	*joy_x = ctx.joy_x;
	*joy_y = ctx.joy_y;
	*motion = ctx.motion;
	*current_limit_ma = ctx.current_limit_ma;

	if (now_ms - ctx.last_cmd_ms > APP_CMD_TIMEOUT_MS) {
		*joy_x = 0.0f;
		*joy_y = 0.0f;
		*motion = ROBOT_STOP;
	}
	k_mutex_unlock(&ctx.lock);
}

static void apply_motion_shortcut(robot_motion_t motion, float *joy_x,
				  float *joy_y)
{
	if (motion == ROBOT_FORWARD) {
		*joy_y = fabsf(*joy_y) > 1.0f ? fabsf(*joy_y) :
						 APP_PID_BALANCE_MOTION_JOY_Y;
	} else if (motion == ROBOT_BACK) {
		*joy_y = fabsf(*joy_y) > 1.0f ? -fabsf(*joy_y) :
						 -APP_PID_BALANCE_MOTION_JOY_Y;
	} else if (motion == ROBOT_LEFT) {
		*joy_x = fabsf(*joy_x) > 1.0f ? -fabsf(*joy_x) :
						 -APP_PID_BALANCE_MOTION_JOY_X;
	} else if (motion == ROBOT_RIGHT) {
		*joy_x = fabsf(*joy_x) > 1.0f ? fabsf(*joy_x) :
						 APP_PID_BALANCE_MOTION_JOY_X;
	}
}

void control_step(const bmi088_sample_t *imu, const dji_m3508_motor_t *left,
		  const dji_m3508_motor_t *right, float dt_s,
		  control_output_t *out)
{
	bool enable;
	int height;
	float joy_x;
	float joy_y;
	robot_motion_t motion;
	int16_t current_limit_ma;

	if (imu == NULL || left == NULL || right == NULL || out == NULL) {
		return;
	}

	memset(out, 0, sizeof(*out));
	if (dt_s <= 0.0f || dt_s > 0.02f) {
		dt_s = 1.0f / APP_CONTROL_HZ;
	}

	copy_command(&enable, &height, &joy_x, &joy_y, &motion,
		     &current_limit_ma);
	apply_motion_shortcut(motion, &joy_x, &joy_y);

	const float left_wheel_angle =
		wheel_forward_sign(true) * left->angle_rad /
		APP_M3508_REDUCTION_RATIO;
	const float right_wheel_angle =
		wheel_forward_sign(false) * right->angle_rad /
		APP_M3508_REDUCTION_RATIO;
	const float left_wheel_speed =
		wheel_forward_sign(true) * left->speed_rad_s /
		APP_M3508_REDUCTION_RATIO;
	const float right_wheel_speed =
		wheel_forward_sign(false) * right->speed_rad_s /
		APP_M3508_REDUCTION_RATIO;
	const float distance = 0.5f * (left_wheel_angle + right_wheel_angle);
	const float speed = 0.5f * (left_wheel_speed + right_wheel_speed);

	if (!ctx.distance_zero_valid) {
		ctx.distance_zero_rad = distance;
		ctx.distance_zero_valid = true;
	}

	const float filtered_joy_y =
		app_lpf_update(joy_y, &ctx.joy_y_lpf,
			       APP_PID_BALANCE_JOY_LPF_TIME_S, dt_s);
	const float speed_target =
		filtered_joy_y * APP_PID_BALANCE_JOY_TO_SPEED_RAD_S;

	if (!enable || fabsf(filtered_joy_y) > 1.0f ||
	    fabsf(speed) > APP_PID_BALANCE_DISTANCE_RESET_SPEED_RAD_S) {
		ctx.distance_zero_rad = distance;
		app_pid_reset(&ctx.distance_pid);
	}

	const float angle_error = balance_angle_deg(imu) - ctx.angle_zero_deg;
	const float angle_output =
		app_pid_update(&ctx.angle_pid, angle_error, dt_s);
	const float gyro_output =
		app_pid_update(&ctx.gyro_pid, balance_gyro_dps(imu), dt_s);
	const float distance_output =
		app_pid_update(&ctx.distance_pid,
			       distance - ctx.distance_zero_rad, dt_s);
	const float speed_output =
		app_pid_update(&ctx.speed_pid, speed - speed_target, dt_s);
	const float drive_output =
		filtered_joy_y * APP_PID_BALANCE_JOY_TO_CURRENT;

	float balance_output =
		angle_output + gyro_output + distance_output + speed_output -
		drive_output;
	if (APP_PID_BALANCE_STICTION_FULL_DEG >
	    APP_PID_BALANCE_STICTION_START_DEG) {
		const float stiction_scale = app_clampf(
			(fabsf(angle_error) -
			 APP_PID_BALANCE_STICTION_START_DEG) /
				(APP_PID_BALANCE_STICTION_FULL_DEG -
				 APP_PID_BALANCE_STICTION_START_DEG),
			0.0f, 1.0f);

		if (stiction_scale > 0.0f && fabsf(balance_output) > 1.0f) {
			balance_output += copysignf(
				APP_PID_BALANCE_STICTION_CURRENT *
					stiction_scale,
				balance_output);
		}
	}
	balance_output = app_clampf(balance_output, -current_limit_ma,
				    current_limit_ma);

	float yaw_output = joy_x * APP_PID_BALANCE_JOY_TO_YAW_CURRENT;
	yaw_output = app_clampf(yaw_output,
				-APP_PID_BALANCE_YAW_CURRENT_LIMIT,
				APP_PID_BALANCE_YAW_CURRENT_LIMIT);
	const float wheel_sync_output = app_clampf(
		(right_wheel_speed - left_wheel_speed) *
			APP_PID_BALANCE_WHEEL_SYNC_P,
		-APP_PID_BALANCE_WHEEL_SYNC_CURRENT_LIMIT,
		APP_PID_BALANCE_WHEEL_SYNC_CURRENT_LIMIT);

	if (fabsf(angle_error) > APP_PITCH_FAULT_DEG) {
		ctx.faulted = true;
		ctx.recover_ticks = 0;
	}

	if (ctx.faulted) {
		if (enable && fabsf(angle_error) < APP_PITCH_RECOVER_DEG) {
			ctx.recover_ticks++;
			if (ctx.recover_ticks > APP_BALANCE_RECOVER_TICKS) {
				ctx.faulted = false;
				ctx.recover_ticks = 0;
				ctx.distance_zero_rad = distance;
				reset_balance_pids();
			}
		} else {
			ctx.recover_ticks = 0;
		}
	}

	const bool motor_feedback_ok = left->initialized && right->initialized;
	const bool wheels_enabled = enable && !ctx.faulted && motor_feedback_ok;
	int32_t left_current = (int32_t)lrintf(-0.5f *
					       (balance_output + yaw_output) +
					       wheel_sync_output);
	int32_t right_current = (int32_t)lrintf(-0.5f *
						(balance_output - yaw_output) -
						wheel_sync_output);

	if (!wheels_enabled) {
		left_current = 0;
		right_current = 0;
		balance_output = 0.0f;
		yaw_output = 0.0f;
	}

	float left_joint;
	float right_joint;
	fixed_joint_targets(height, &left_joint, &right_joint);

	out->wheels_enabled = wheels_enabled;
	out->joints_enabled = true;
	out->left_wheel_current = app_clamp_i16(
		left_current, -current_limit_ma, current_limit_ma);
	out->right_wheel_current = app_clamp_i16(
		right_current, -current_limit_ma, current_limit_ma);
	out->left_joint_position_rad = left_joint;
	out->right_joint_position_rad = right_joint;
	out->joint_velocity_limit_rad_s = APP_LEG_VEL_LIMIT_RAD_S;

	k_mutex_lock(&ctx.lock, K_FOREVER);
	ctx.status = (control_status_t) {
		.enable_request = enable,
		.wheels_enabled = wheels_enabled,
		.faulted = ctx.faulted,
		.height = height,
		.joy_x = joy_x,
		.joy_y = joy_y,
		.motion = motion,
		.pitch_deg = imu->pitch_deg,
		.roll_deg = imu->roll_deg,
		.yaw_deg = imu->yaw_deg,
		.distance_rad = distance,
		.speed_rad_s = speed,
		.lqr_output = balance_output,
		.yaw_output = yaw_output,
		.left_joint_position_rad = left_joint,
		.right_joint_position_rad = right_joint,
		.left_wheel_current = out->left_wheel_current,
		.right_wheel_current = out->right_wheel_current,
		.jump_phase = 0,
	};
	k_mutex_unlock(&ctx.lock);
}
