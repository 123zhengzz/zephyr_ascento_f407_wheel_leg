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
	bool jump_request;
	int64_t last_cmd_ms;

	app_pid_t angle_pid;
	app_pid_t gyro_pid;
	app_pid_t distance_pid;
	app_pid_t speed_pid;
	app_pid_t yaw_angle_pid;
	app_pid_t yaw_gyro_pid;

	float angle_zero_deg;
	float distance_zero_rad;
	float yaw_target_deg;
	float joy_y_lpf;
	bool distance_zero_valid;
	bool faulted;
	int recover_ticks;
	int jump_phase;
	int jump_ticks;
	control_status_t status;
} control_ctx_t;

static control_ctx_t ctx;

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

void control_init(void)
{
	memset(&ctx, 0, sizeof(ctx));
	k_mutex_init(&ctx.lock);

	ctx.height = APP_DEFAULT_HEIGHT;
	ctx.motion = ROBOT_STOP;
	ctx.last_cmd_ms = k_uptime_get();
	ctx.angle_zero_deg = APP_ANGLE_ZERO_DEG;

	app_pid_init(&ctx.angle_pid, APP_PID_ANGLE_P, APP_PID_ANGLE_I,
		     APP_PID_ANGLE_D, 0.0f, APP_WHEEL_CURRENT_LIMIT);
	app_pid_init(&ctx.gyro_pid, APP_PID_GYRO_P, 0.0f, 0.0f, 0.0f,
		     APP_WHEEL_CURRENT_LIMIT);
	app_pid_init(&ctx.distance_pid, APP_PID_DISTANCE_P, 0.0f, 0.0f,
		     0.0f, APP_WHEEL_CURRENT_LIMIT);
	app_pid_init(&ctx.speed_pid, APP_PID_SPEED_P, 0.0f, 0.0f, 0.0f,
		     APP_WHEEL_CURRENT_LIMIT);
	app_pid_init(&ctx.yaw_angle_pid, APP_PID_YAW_ANGLE_P, 0.0f, 0.0f,
		     0.0f, APP_WHEEL_CURRENT_LIMIT);
	app_pid_init(&ctx.yaw_gyro_pid, APP_PID_YAW_GYRO_P, 0.0f, 0.0f,
		     0.0f, APP_WHEEL_CURRENT_LIMIT);
}

void control_set_enable(bool enable)
{
	k_mutex_lock(&ctx.lock, K_FOREVER);
	ctx.enable_request = enable;
	ctx.last_cmd_ms = k_uptime_get();
	if (!enable) {
		app_pid_reset(&ctx.angle_pid);
		app_pid_reset(&ctx.gyro_pid);
		app_pid_reset(&ctx.distance_pid);
		app_pid_reset(&ctx.speed_pid);
		app_pid_reset(&ctx.yaw_angle_pid);
		app_pid_reset(&ctx.yaw_gyro_pid);
		ctx.distance_zero_valid = false;
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
	ctx.motion = motion;
	if (motion == ROBOT_JUMP) {
		ctx.jump_request = true;
		ctx.motion = ROBOT_STOP;
	}
	ctx.last_cmd_ms = k_uptime_get();
	k_mutex_unlock(&ctx.lock);
}

void control_request_jump(void)
{
	control_set_motion(ROBOT_JUMP);
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
	ctx.angle_zero_deg = app_clampf(zero_deg, -15.0f, 15.0f);
	ctx.distance_zero_valid = false;
	k_mutex_unlock(&ctx.lock);
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

static void copy_and_age_command(bool *enable, int *height, float *joy_x,
				 float *joy_y, robot_motion_t *motion,
				 bool *jump_request)
{
	const int64_t now_ms = k_uptime_get();

	k_mutex_lock(&ctx.lock, K_FOREVER);
	*enable = ctx.enable_request;
	*height = ctx.height;
	*joy_x = ctx.joy_x;
	*joy_y = ctx.joy_y;
	*motion = ctx.motion;
	*jump_request = ctx.jump_request;
	ctx.jump_request = false;

	if (now_ms - ctx.last_cmd_ms > APP_CMD_TIMEOUT_MS) {
		*joy_x = 0.0f;
		*joy_y = 0.0f;
		*motion = ROBOT_STOP;
	}
	k_mutex_unlock(&ctx.lock);
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
	bool jump_request;

	if (imu == NULL || left == NULL || right == NULL || out == NULL) {
		return;
	}

	memset(out, 0, sizeof(*out));
	if (dt_s <= 0.0f) {
		dt_s = 1.0f / APP_CONTROL_HZ;
	}

	copy_and_age_command(&enable, &height, &joy_x, &joy_y, &motion,
			     &jump_request);

	if (motion == ROBOT_FORWARD) {
		joy_y = fabsf(joy_y) > 1.0f ? fabsf(joy_y) : 45.0f;
	} else if (motion == ROBOT_BACK) {
		joy_y = fabsf(joy_y) > 1.0f ? -fabsf(joy_y) : -45.0f;
	} else if (motion == ROBOT_LEFT) {
		joy_x = fabsf(joy_x) > 1.0f ? -fabsf(joy_x) : -35.0f;
	} else if (motion == ROBOT_RIGHT) {
		joy_x = fabsf(joy_x) > 1.0f ? fabsf(joy_x) : 35.0f;
	}

	if (jump_request && ctx.jump_phase == 0) {
		ctx.jump_phase = 1;
		ctx.jump_ticks = 0;
	}

	int effective_height = height;
	if (ctx.jump_phase == 1) {
		effective_height = APP_HEIGHT_MAX;
		ctx.jump_ticks++;
		if (ctx.jump_ticks >= 30) {
			ctx.jump_phase = 2;
			ctx.jump_ticks = 0;
		}
	} else if (ctx.jump_phase == 2) {
		effective_height = 40;
		ctx.jump_ticks++;
		if (ctx.jump_ticks >= 170) {
			ctx.jump_phase = 0;
			ctx.jump_ticks = 0;
		}
	}

	const float left_wheel_angle =
		left->angle_rad / APP_M3508_REDUCTION_RATIO;
	const float right_wheel_angle =
		right->angle_rad / APP_M3508_REDUCTION_RATIO;
	const float left_wheel_speed =
		left->speed_rad_s / APP_M3508_REDUCTION_RATIO;
	const float right_wheel_speed =
		right->speed_rad_s / APP_M3508_REDUCTION_RATIO;
	const float distance = -0.5f * (left_wheel_angle + right_wheel_angle);
	const float speed = -0.5f * (left_wheel_speed + right_wheel_speed);

	if (!ctx.distance_zero_valid) {
		ctx.distance_zero_rad = distance;
		ctx.distance_zero_valid = true;
	}

	if (fabsf(joy_y) > 1.0f || fabsf(speed) > 45.0f ||
	    ctx.jump_phase != 0) {
		ctx.distance_zero_rad = distance;
		app_pid_reset(&ctx.distance_pid);
	}

	const float filtered_joy_y =
		app_lpf_update(joy_y, &ctx.joy_y_lpf, 0.20f, dt_s);
	const float speed_target = filtered_joy_y * APP_JOY_TO_SPEED_RAD_S;

	const float angle_error = imu->pitch_deg - ctx.angle_zero_deg;
	const float angle_control = app_pid_update(&ctx.angle_pid,
						   angle_error, dt_s);
	const float gyro_control = app_pid_update(&ctx.gyro_pid,
						  imu->gy_dps, dt_s);
	const float distance_control = app_pid_update(
		&ctx.distance_pid, distance - ctx.distance_zero_rad, dt_s);
	const float speed_control = app_pid_update(&ctx.speed_pid,
						   speed - speed_target, dt_s);

	float lqr_u = angle_control + gyro_control + distance_control +
		      speed_control;
	lqr_u = app_clampf(lqr_u, -APP_WHEEL_CURRENT_LIMIT,
			   APP_WHEEL_CURRENT_LIMIT);

	ctx.yaw_target_deg += joy_x * APP_JOY_TO_YAW_RATE_DEG_S * dt_s;
	ctx.yaw_target_deg = app_wrap_pm180(ctx.yaw_target_deg);
	const float yaw_error = app_wrap_pm180(ctx.yaw_target_deg -
					       imu->yaw_deg);
	float yaw_output = app_pid_update(&ctx.yaw_angle_pid, yaw_error,
					  dt_s) +
			   app_pid_update(&ctx.yaw_gyro_pid, imu->gz_dps,
					  dt_s);
	yaw_output = app_clampf(yaw_output, -APP_WHEEL_CURRENT_LIMIT,
				APP_WHEEL_CURRENT_LIMIT);

	if (fabsf(imu->pitch_deg) > APP_PITCH_FAULT_DEG) {
		ctx.faulted = true;
		ctx.recover_ticks = 0;
	}

	if (ctx.faulted) {
		if (enable && fabsf(imu->pitch_deg) < APP_PITCH_RECOVER_DEG) {
			ctx.recover_ticks++;
			if (ctx.recover_ticks > APP_BALANCE_RECOVER_TICKS) {
				ctx.faulted = false;
				ctx.recover_ticks = 0;
				ctx.distance_zero_rad = distance;
				app_pid_reset(&ctx.angle_pid);
				app_pid_reset(&ctx.gyro_pid);
				app_pid_reset(&ctx.distance_pid);
				app_pid_reset(&ctx.speed_pid);
			}
		} else {
			ctx.recover_ticks = 0;
		}
	}

	const bool motor_feedback_ok = left->initialized && right->initialized;
	const bool wheels_enabled = enable && !ctx.faulted && motor_feedback_ok;
	int32_t left_current = (int32_t)(-0.5f * (lqr_u + yaw_output));
	int32_t right_current = (int32_t)(-0.5f * (lqr_u - yaw_output));

	if (!wheels_enabled) {
		left_current = 0;
		right_current = 0;
		lqr_u = 0.0f;
		yaw_output = 0.0f;
	}

	const float height_term =
		(float)(effective_height - APP_HEIGHT_MIN) *
		APP_LEG_RAD_PER_HEIGHT_UNIT;
	float roll_comp = imu->roll_deg * APP_ROLL_RAD_PER_DEG;
	roll_comp = app_clampf(roll_comp, -APP_ROLL_COMP_LIMIT_RAD,
				APP_ROLL_COMP_LIMIT_RAD);

	const float left_joint = app_clampf(APP_LEFT_LEG_ZERO_RAD +
						    height_term - roll_comp,
					    APP_LEFT_LEG_MIN_RAD,
					    APP_LEFT_LEG_MAX_RAD);
	const float right_joint = app_clampf(APP_RIGHT_LEG_ZERO_RAD -
						     height_term - roll_comp,
					     APP_RIGHT_LEG_MIN_RAD,
					     APP_RIGHT_LEG_MAX_RAD);

	out->wheels_enabled = wheels_enabled;
	out->joints_enabled = true;
	out->left_wheel_current = app_clamp_i16(
		left_current, -APP_WHEEL_CURRENT_LIMIT,
		APP_WHEEL_CURRENT_LIMIT);
	out->right_wheel_current = app_clamp_i16(
		right_current, -APP_WHEEL_CURRENT_LIMIT,
		APP_WHEEL_CURRENT_LIMIT);
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
		.lqr_output = lqr_u,
		.yaw_output = yaw_output,
		.left_joint_position_rad = left_joint,
		.right_joint_position_rad = right_joint,
		.left_wheel_current = out->left_wheel_current,
		.right_wheel_current = out->right_wheel_current,
		.jump_phase = ctx.jump_phase,
	};
	k_mutex_unlock(&ctx.lock);
}
