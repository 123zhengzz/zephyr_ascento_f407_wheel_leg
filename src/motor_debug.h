#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "dji_m3508.h"
#include "dm4340.h"

typedef enum {
	MOTOR_DEBUG_DM_NONE = 0,
	MOTOR_DEBUG_DM_POS_VEL,
	MOTOR_DEBUG_DM_VELOCITY,
	MOTOR_DEBUG_DM_MIT,
	MOTOR_DEBUG_DM_WIGGLE,
} motor_debug_dm_mode_t;

typedef enum {
	MOTOR_DEBUG_WHEEL_NONE = 0,
	MOTOR_DEBUG_WHEEL_CURRENT,
	MOTOR_DEBUG_WHEEL_RPM,
} motor_debug_wheel_mode_t;

typedef struct {
	motor_debug_dm_mode_t mode;
	uint8_t id;
	float position_rad;
	float velocity_rad_s;
	float kp;
	float kd;
	float torque_nm;
	float amplitude_rad;
	int32_t period_ms;
	int64_t start_ms;
} motor_debug_dm_command_t;

typedef struct {
	bool active;
	bool wheel_active;
	motor_debug_wheel_mode_t wheel_mode;
	int16_t wheel_current[DJI_M3508_MAX_ID + 1];
	int32_t left_wheel_rpm;
	int32_t right_wheel_rpm;
	motor_debug_dm_command_t dm[DM4340_MAX_ID + 1];
} motor_debug_output_t;

void motor_debug_init(dji_m3508_bus_t *dji_bus, dm4340_bus_t *dm_bus);

int motor_debug_set_wheel_current(uint8_t id, int32_t current,
				  int32_t duration_ms);
int motor_debug_set_wheel_pair(int32_t left_current, int32_t right_current,
			       int32_t duration_ms);
int motor_debug_set_wheel_rpm(uint8_t id, int32_t rpm, int32_t duration_ms);
int motor_debug_stop_wheels(void);

int motor_debug_dm_enable(uint8_t id);
int motor_debug_dm_disable(uint8_t id);
int motor_debug_dm_save_zero(uint8_t id);
int motor_debug_dm_read_reg(uint8_t id, uint8_t rid,
			    dm4340_param_response_t *out);
int motor_debug_set_dm_pos_vel(uint8_t id, float position_rad,
			       float velocity_rad_s, int32_t duration_ms);
int motor_debug_set_dm_velocity(uint8_t id, float velocity_rad_s,
				int32_t duration_ms);
int motor_debug_set_dm_mit(uint8_t id, float position_rad,
			   float velocity_rad_s, float kp, float kd,
			   float torque_nm, int32_t duration_ms);
int motor_debug_set_dm_mit_raw(uint8_t id, float position_rad,
				       float velocity_rad_s, float kp, float kd,
				       float torque_nm, int32_t duration_ms);
int motor_debug_set_dm_wiggle(uint8_t id, float center_rad,
			      float amplitude_rad, int32_t period_ms, float kp,
			      float kd, int32_t duration_ms);
int motor_debug_stop_dm(uint8_t id);
int motor_debug_stop_all(void);

bool motor_debug_get_output(motor_debug_output_t *out);
bool motor_debug_get_m3508(uint8_t id, dji_m3508_motor_t *out);
bool motor_debug_get_dm4340(uint8_t id, dm4340_feedback_t *out);
void motor_debug_dump_dm4340_rx_log(void);
const char *motor_debug_dm_mode_name(motor_debug_dm_mode_t mode);
