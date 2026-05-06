#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "bmi088.h"
#include "dji_m3508.h"

typedef struct {
	bool calibrated;
	float wheel_radius_m;
	float wheel_base_m;
	float total_mass_kg;
	float body_com_height_m;
	float body_com_forward_offset_m;
	float body_pitch_inertia_kg_m2;
	float wheel_inertia_kg_m2;
	float current_ma_to_wheel_torque_nm;
	float left_current_ma_to_wheel_torque_nm;
	float right_current_ma_to_wheel_torque_nm;
	float leg_length_min_m;
	float leg_length_max_m;
	float leg_length_default_m;
	float left_joint_at_min_leg_rad;
	float left_joint_at_max_leg_rad;
	float right_joint_at_min_leg_rad;
	float right_joint_at_max_leg_rad;
	float k_pitch;
	float k_pitch_rate;
	float k_position;
	float k_velocity;
	float k_yaw_rate;
	float k_roll_to_leg_m_per_rad;

	/* Runtime-tunable overrides (initialized from #define defaults). */
	float theta_eq_rad;
	float gain_c0_a, gain_c0_b, gain_c0_c;  /* legacy / flash compatibility */
	float gain_c1_a, gain_c1_b, gain_c1_c;  /* legacy / flash compatibility */
	float gain_c2;                           /* legacy / flash compatibility */
	float gain_c3_a, gain_c3_b, gain_c3_c;  /* legacy / flash compatibility */
	float stiction_current_ma;
	float stiction_start_deg;
	float stiction_full_deg;
	int16_t current_limit_ma;
	float current_scale;
	float fault_deg;
	float recover_deg;
	float wheel_sync_gain_ma;
	float wheel_sync_current_limit_ma;
} ascento_balance_params_t;

typedef struct {
	bool enable_request;
	bool wheel_feedback_ok;
	float dt_s;
	float target_forward_speed_mps;
	float target_yaw_rate_rad_s;
	float target_pitch_rad;
	float left_joint_position_rad;
	float right_joint_position_rad;
	float left_joint_velocity_rad_s;
	float right_joint_velocity_rad_s;
	bmi088_sample_t imu;
	dji_m3508_motor_t left_wheel;
	dji_m3508_motor_t right_wheel;
} ascento_balance_input_t;

typedef struct {
	bool active;
	bool params_ready;
	bool faulted;
	int16_t left_wheel_current;
	int16_t right_wheel_current;
	float left_joint_position_rad;
	float right_joint_position_rad;
	float joint_velocity_limit_rad_s;
	float body_position_m;
	float body_velocity_mps;
	float pitch_rad;
	float pitch_rate_rad_s;
	float balance_torque_nm;
	float yaw_torque_nm;
} ascento_balance_output_t;

typedef struct {
	bool initialized;
	bool faulted;
	int recover_ticks;
	float body_position_m;
	float wheel_position_zero_m;
	float body_velocity_lpf_mps;
	float yaw_rate_lpf_rad_s;
} ascento_balance_state_t;

extern const ascento_balance_params_t *ascento_balance_default_params;

void ascento_balance_get_params(ascento_balance_params_t *params);
void ascento_balance_set_params(const ascento_balance_params_t *params);

int ascento_balance_save_params(void);
int ascento_balance_reset_params(void);
int ascento_balance_settings_init(void);

void ascento_balance_init(ascento_balance_state_t *state);
void ascento_balance_reset(ascento_balance_state_t *state);
bool ascento_balance_params_ready(const ascento_balance_params_t *params);
float ascento_balance_leg_length_from_joint(const ascento_balance_params_t *params,
					    bool left_leg, float joint_rad);
void ascento_balance_update(ascento_balance_state_t *state,
			    const ascento_balance_params_t *params,
			    const ascento_balance_input_t *input,
			    ascento_balance_output_t *output);
