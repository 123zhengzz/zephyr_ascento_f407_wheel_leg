#include "ascento_balance.h"

#include <math.h>
#include <string.h>

#include "app_config.h"
#include "pid.h"

#define DEG_TO_RAD 0.017453292519943295f

const ascento_balance_params_t ascento_balance_default_params = {
	.calibrated = APP_ASCENTO_PARAMS_CALIBRATED != 0,
	.wheel_radius_m = APP_ASCENTO_WHEEL_RADIUS_M,
	.wheel_base_m = APP_ASCENTO_WHEEL_BASE_M,
	.total_mass_kg = APP_ASCENTO_TOTAL_MASS_KG,
	.body_com_height_m = APP_ASCENTO_BODY_COM_HEIGHT_M,
	.body_pitch_inertia_kg_m2 = APP_ASCENTO_BODY_PITCH_INERTIA_KG_M2,
	.wheel_inertia_kg_m2 = APP_ASCENTO_WHEEL_INERTIA_KG_M2,
	.current_ma_to_wheel_torque_nm =
		APP_ASCENTO_CURRENT_MA_TO_WHEEL_TORQUE_NM,
	.leg_length_min_m = APP_ASCENTO_LEG_LENGTH_MIN_M,
	.leg_length_max_m = APP_ASCENTO_LEG_LENGTH_MAX_M,
	.leg_length_default_m = APP_ASCENTO_LEG_LENGTH_DEFAULT_M,
	.left_joint_at_min_leg_rad = APP_ASCENTO_LEFT_JOINT_AT_MIN_LEG_RAD,
	.left_joint_at_max_leg_rad = APP_ASCENTO_LEFT_JOINT_AT_MAX_LEG_RAD,
	.right_joint_at_min_leg_rad = APP_ASCENTO_RIGHT_JOINT_AT_MIN_LEG_RAD,
	.right_joint_at_max_leg_rad = APP_ASCENTO_RIGHT_JOINT_AT_MAX_LEG_RAD,
	.k_pitch = APP_ASCENTO_K_PITCH,
	.k_pitch_rate = APP_ASCENTO_K_PITCH_RATE,
	.k_position = APP_ASCENTO_K_POSITION,
	.k_velocity = APP_ASCENTO_K_VELOCITY,
	.k_yaw_rate = APP_ASCENTO_K_YAW_RATE,
	.k_roll_to_leg_m_per_rad = APP_ASCENTO_K_ROLL_TO_LEG_M_PER_RAD,
};

static float lerpf(float a, float b, float t)
{
	return a + (b - a) * t;
}

void ascento_balance_init(ascento_balance_state_t *state)
{
	ascento_balance_reset(state);
}

void ascento_balance_reset(ascento_balance_state_t *state)
{
	if (state != NULL) {
		memset(state, 0, sizeof(*state));
	}
}

bool ascento_balance_params_ready(const ascento_balance_params_t *params)
{
	if (params == NULL || !params->calibrated) {
		return false;
	}

	return params->wheel_radius_m > 0.0f &&
	       params->wheel_base_m > 0.0f &&
	       params->total_mass_kg > 0.0f &&
	       params->body_com_height_m > 0.0f &&
	       params->current_ma_to_wheel_torque_nm > 0.0f &&
	       params->leg_length_max_m > params->leg_length_min_m;
}

float ascento_balance_leg_length_from_joint(const ascento_balance_params_t *params,
					    bool left_leg, float joint_rad)
{
	if (params == NULL ||
	    params->leg_length_max_m <= params->leg_length_min_m) {
		return 0.0f;
	}

	const float joint_min = left_leg ? params->left_joint_at_min_leg_rad :
					  params->right_joint_at_min_leg_rad;
	const float joint_max = left_leg ? params->left_joint_at_max_leg_rad :
					  params->right_joint_at_max_leg_rad;
	const float denom = joint_max - joint_min;

	if (fabsf(denom) < 1.0e-6f) {
		return params->leg_length_default_m;
	}

	const float t = app_clampf((joint_rad - joint_min) / denom, 0.0f,
				  1.0f);
	return lerpf(params->leg_length_min_m, params->leg_length_max_m, t);
}

float ascento_balance_joint_from_leg_length(const ascento_balance_params_t *params,
					    bool left_leg, float leg_length_m)
{
	if (params == NULL ||
	    params->leg_length_max_m <= params->leg_length_min_m) {
		return 0.0f;
	}

	leg_length_m = app_clampf(leg_length_m, params->leg_length_min_m,
				  params->leg_length_max_m);
	const float t = (leg_length_m - params->leg_length_min_m) /
			(params->leg_length_max_m - params->leg_length_min_m);
	const float joint_min = left_leg ? params->left_joint_at_min_leg_rad :
					  params->right_joint_at_min_leg_rad;
	const float joint_max = left_leg ? params->left_joint_at_max_leg_rad :
					  params->right_joint_at_max_leg_rad;

	return lerpf(joint_min, joint_max, t);
}

static int16_t torque_to_current_ma(float torque_nm,
				    const ascento_balance_params_t *params)
{
	const float current_ma =
		torque_nm / params->current_ma_to_wheel_torque_nm;
	return app_clamp_i16((int32_t)lrintf(current_ma), -APP_WHEEL_CURRENT_LIMIT,
			     APP_WHEEL_CURRENT_LIMIT);
}

void ascento_balance_update(ascento_balance_state_t *state,
			    const ascento_balance_params_t *params,
			    const ascento_balance_input_t *input,
			    ascento_balance_output_t *output)
{
	if (output == NULL) {
		return;
	}

	memset(output, 0, sizeof(*output));

	if (state == NULL || params == NULL || input == NULL) {
		return;
	}

	output->params_ready = ascento_balance_params_ready(params);
	if (!output->params_ready || !input->enable_request ||
	    !input->wheel_feedback_ok) {
		ascento_balance_reset(state);
		return;
	}

	float dt_s = input->dt_s;
	if (dt_s <= 0.0f || dt_s > 0.02f) {
		dt_s = 1.0f / APP_CONTROL_HZ;
	}

	const float left_wheel_angle =
		input->left_wheel.angle_rad / APP_M3508_REDUCTION_RATIO;
	const float right_wheel_angle =
		input->right_wheel.angle_rad / APP_M3508_REDUCTION_RATIO;
	const float left_wheel_speed =
		input->left_wheel.speed_rad_s / APP_M3508_REDUCTION_RATIO;
	const float right_wheel_speed =
		input->right_wheel.speed_rad_s / APP_M3508_REDUCTION_RATIO;

	const float wheel_distance_m =
		0.5f * (left_wheel_angle + right_wheel_angle) *
		params->wheel_radius_m;
	const float body_velocity_mps =
		0.5f * (left_wheel_speed + right_wheel_speed) *
		params->wheel_radius_m;

	if (!state->initialized) {
		state->body_position_m = wheel_distance_m;
		state->body_velocity_lpf_mps = body_velocity_mps;
		state->initialized = true;
	} else {
		state->body_position_m = wheel_distance_m;
	}

	const float velocity_mps = app_lpf_update(body_velocity_mps,
						  &state->body_velocity_lpf_mps,
						  0.025f, dt_s);
	const float pitch_rad = input->imu.pitch_deg * DEG_TO_RAD;
	const float pitch_rate_rad_s = input->imu.gy_dps * DEG_TO_RAD;
	const float yaw_rate_rad_s = app_lpf_update(input->imu.gz_dps * DEG_TO_RAD,
						    &state->yaw_rate_lpf_rad_s,
						    0.025f, dt_s);
	const float roll_rad = input->imu.roll_deg * DEG_TO_RAD;

	const float x_error = state->body_position_m;
	const float v_error = velocity_mps - input->target_forward_speed_mps;
	const float pitch_error = pitch_rad - input->target_pitch_rad;
	float balance_torque_nm =
		-(params->k_pitch * pitch_error +
		  params->k_pitch_rate * pitch_rate_rad_s +
		  params->k_position * x_error +
		  params->k_velocity * v_error);
	float yaw_torque_nm =
		-params->k_yaw_rate * (yaw_rate_rad_s -
				       input->target_yaw_rate_rad_s);

	const float current_limit_torque =
			params->current_ma_to_wheel_torque_nm *
		(float)APP_WHEEL_CURRENT_LIMIT;
	balance_torque_nm = app_clampf(balance_torque_nm,
				       -current_limit_torque,
				       current_limit_torque);
	yaw_torque_nm = app_clampf(yaw_torque_nm, -current_limit_torque,
				   current_limit_torque);

	const float target_leg_length =
		app_clampf(input->target_leg_length_m,
			   params->leg_length_min_m,
			   params->leg_length_max_m);
	const float roll_leg_delta =
		app_clampf(roll_rad * params->k_roll_to_leg_m_per_rad,
			   -0.5f * (params->leg_length_max_m -
				    params->leg_length_min_m),
			   0.5f * (params->leg_length_max_m -
				   params->leg_length_min_m));
	const float left_leg_length =
		app_clampf(target_leg_length - roll_leg_delta,
			   params->leg_length_min_m,
			   params->leg_length_max_m);
	const float right_leg_length =
		app_clampf(target_leg_length + roll_leg_delta,
			   params->leg_length_min_m,
			   params->leg_length_max_m);

	output->active = true;
	output->left_wheel_current =
		torque_to_current_ma(balance_torque_nm - yaw_torque_nm,
				     params);
	output->right_wheel_current =
		torque_to_current_ma(balance_torque_nm + yaw_torque_nm,
				     params);
	output->left_joint_position_rad =
		ascento_balance_joint_from_leg_length(params, true,
						      left_leg_length);
	output->right_joint_position_rad =
		ascento_balance_joint_from_leg_length(params, false,
						      right_leg_length);
	output->joint_velocity_limit_rad_s = APP_LEG_VEL_LIMIT_RAD_S;
	output->body_position_m = state->body_position_m;
	output->body_velocity_mps = velocity_mps;
	output->pitch_rad = pitch_rad;
	output->pitch_rate_rad_s = pitch_rate_rad_s;
	output->balance_torque_nm = balance_torque_nm;
	output->yaw_torque_nm = yaw_torque_nm;
	output->left_leg_length_m = left_leg_length;
	output->right_leg_length_m = right_leg_length;
}
