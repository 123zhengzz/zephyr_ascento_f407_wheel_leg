#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
	float kp;
	float ki;
	float kd;
	float integral_limit;
	float output_limit;
	float integral;
	float previous_error;
	bool initialized;
} app_pid_t;

void app_pid_init(app_pid_t *pid, float kp, float ki, float kd,
		  float integral_limit, float output_limit);
void app_pid_reset(app_pid_t *pid);
float app_pid_update(app_pid_t *pid, float error, float dt_s);

float app_lpf_update(float input, float *state, float time_constant_s,
		     float dt_s);
float app_clampf(float value, float min_value, float max_value);
int16_t app_clamp_i16(int32_t value, int16_t min_value, int16_t max_value);
float app_wrap_pm180(float angle_deg);
