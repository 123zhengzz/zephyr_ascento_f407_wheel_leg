#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "bmi088.h"
#include "dji_m3508.h"

typedef enum {
	ROBOT_FORWARD = 0,
	ROBOT_BACK = 1,
	ROBOT_RIGHT = 2,
	ROBOT_LEFT = 3,
	ROBOT_STOP = 4,
	ROBOT_JUMP = 5,
} robot_motion_t;

typedef struct {
	bool wheels_enabled;
	bool joints_enabled;
	int16_t left_wheel_current;
	int16_t right_wheel_current;
	float left_joint_position_rad;
	float right_joint_position_rad;
	float joint_velocity_limit_rad_s;
} control_output_t;

typedef struct {
	bool enable_request;
	bool wheels_enabled;
	bool faulted;
	int height;
	float joy_x;
	float joy_y;
	robot_motion_t motion;
	float pitch_deg;
	float roll_deg;
	float yaw_deg;
	float distance_rad;
	float speed_rad_s;
	float lqr_output;
	float yaw_output;
	float left_joint_position_rad;
	float right_joint_position_rad;
	int16_t left_wheel_current;
	int16_t right_wheel_current;
	int jump_phase;
} control_status_t;

typedef struct {
	float angle_p;
	float angle_i;
	float angle_d;
	float gyro_p;
	float distance_p;
	float speed_p;
	int16_t current_limit_ma;
} control_pid_balance_params_t;

void control_init(void);
void control_set_enable(bool enable);
void control_set_height(int height);
void control_set_joystick(float x, float y);
void control_set_motion(robot_motion_t motion);
void control_request_jump(void);
void control_stop_motion(void);
void control_set_angle_zero(float zero_deg);
void control_get_status(control_status_t *status);
void control_get_pid_balance_params(control_pid_balance_params_t *params);
void control_set_pid_balance_params(
	const control_pid_balance_params_t *params);

void control_step(const bmi088_sample_t *imu, const dji_m3508_motor_t *left,
		  const dji_m3508_motor_t *right, float dt_s,
		  control_output_t *out);
