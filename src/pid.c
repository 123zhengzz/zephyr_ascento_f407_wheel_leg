#include "pid.h"

#include <math.h>

void app_pid_init(app_pid_t *pid, float kp, float ki, float kd,
		  float integral_limit, float output_limit)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->integral_limit = fabsf(integral_limit);
	pid->output_limit = fabsf(output_limit);
	app_pid_reset(pid);
}

void app_pid_reset(app_pid_t *pid)
{
	pid->integral = 0.0f;
	pid->previous_error = 0.0f;
	pid->initialized = false;
}

float app_pid_update(app_pid_t *pid, float error, float dt_s)
{
	if (dt_s <= 0.0f) {
		dt_s = 0.001f;
	}

	if (!pid->initialized) {
		pid->previous_error = error;
		pid->initialized = true;
	}

	pid->integral += error * dt_s;
	pid->integral = app_clampf(pid->integral, -pid->integral_limit,
				   pid->integral_limit);

	const float derivative = (error - pid->previous_error) / dt_s;
	pid->previous_error = error;

	const float output = pid->kp * error + pid->ki * pid->integral +
			     pid->kd * derivative;

	return app_clampf(output, -pid->output_limit, pid->output_limit);
}

float app_lpf_update(float input, float *state, float time_constant_s,
		     float dt_s)
{
	if (time_constant_s <= 0.0f || dt_s <= 0.0f) {
		*state = input;
		return input;
	}

	const float alpha = dt_s / (time_constant_s + dt_s);
	*state += alpha * (input - *state);
	return *state;
}

float app_clampf(float value, float min_value, float max_value)
{
	if (value < min_value) {
		return min_value;
	}
	if (value > max_value) {
		return max_value;
	}
	return value;
}

int16_t app_clamp_i16(int32_t value, int16_t min_value, int16_t max_value)
{
	if (value < min_value) {
		return min_value;
	}
	if (value > max_value) {
		return max_value;
	}
	return (int16_t)value;
}

float app_wrap_pm180(float angle_deg)
{
	while (angle_deg > 180.0f) {
		angle_deg -= 360.0f;
	}
	while (angle_deg < -180.0f) {
		angle_deg += 360.0f;
	}
	return angle_deg;
}
