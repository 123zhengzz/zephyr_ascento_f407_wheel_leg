#include "ascento_balance.h"

#include <math.h>
#include <string.h>

#include <zephyr/logging/log.h>

#include "app_config.h"
#include "pid.h"

LOG_MODULE_REGISTER(ascento, LOG_LEVEL_INF);

#define DEG_TO_RAD 0.017453292519943295f

const ascento_balance_params_t ascento_balance_default_params = {
	.calibrated = APP_ASCENTO_PARAMS_CALIBRATED != 0,
	.wheel_radius_m = APP_ASCENTO_WHEEL_RADIUS_M,
	.wheel_base_m = APP_ASCENTO_WHEEL_BASE_M,
	.total_mass_kg = APP_ASCENTO_TOTAL_MASS_KG,
	.body_com_height_m = APP_ASCENTO_BODY_COM_HEIGHT_M,
	.body_com_forward_offset_m = APP_ASCENTO_BODY_COM_FORWARD_OFFSET_M,
	.body_pitch_inertia_kg_m2 = APP_ASCENTO_BODY_PITCH_INERTIA_KG_M2,
	.wheel_inertia_kg_m2 = APP_ASCENTO_WHEEL_INERTIA_KG_M2,
	.current_ma_to_wheel_torque_nm =
		APP_ASCENTO_CURRENT_MA_TO_WHEEL_TORQUE_NM,
	.left_current_ma_to_wheel_torque_nm =
		APP_ASCENTO_LEFT_CURRENT_MA_TO_WHEEL_TORQUE_NM,
	.right_current_ma_to_wheel_torque_nm =
		APP_ASCENTO_RIGHT_CURRENT_MA_TO_WHEEL_TORQUE_NM,
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

/* ------------------------------------------------------------------ */
/* Four-bar linkage geometry (ACTIVE — user-confirmed link lengths)     */
/*                                                                      */
/* SIGN CONVENTION (validated 2026-05-04):                              */
/*   qC = offset - joint_angle                                          */
/*                                                                      */
/* Link lengths confirmed by user measurement 2026-05-04.               */
/* Offsets calibrated from 3-point average (spread <2.5°).             */
/* ------------------------------------------------------------------ */

#define FB_L1       APP_ASCENTO_FB_L1
#define FB_L2       APP_ASCENTO_FB_L2
#define FB_L3       APP_ASCENTO_FB_L3
#define FB_L4       APP_ASCENTO_FB_L4
#define FB_L23      APP_ASCENTO_FB_L23
#define FB_OFF_LEFT APP_ASCENTO_FB_JOINT_ZERO_OFFSET_LEFT_RAD
#define FB_OFF_RIGHT APP_ASCENTO_FB_JOINT_ZERO_OFFSET_RIGHT_RAD
#define FB_DL       APP_ASCENTO_LEG_LENGTH_REF_OFFSET_M

#define FB_SQRT2 1.41421356237f
#define FB_EPS   1.0e-6f

typedef struct {
	float x, y;
} fb_pt_t;

typedef struct {
	fb_pt_t pts[2];
	uint8_t count;
} fb_circle_hit_t;

typedef struct {
	uint8_t valid;
	float qC;
	float leg_length;
	float closure_error;
	fb_pt_t A, B, C, D, E;
} fb_state_t;

static fb_pt_t fb_pt(float x, float y)
{
	fb_pt_t p = { x, y };
	return p;
}

static fb_pt_t fb_add(fb_pt_t a, fb_pt_t b)
{
	return fb_pt(a.x + b.x, a.y + b.y);
}

static fb_pt_t fb_sub(fb_pt_t a, fb_pt_t b)
{
	return fb_pt(a.x - b.x, a.y - b.y);
}

static fb_pt_t fb_scale(fb_pt_t p, float k)
{
	return fb_pt(p.x * k, p.y * k);
}

static float fb_norm(fb_pt_t p)
{
	return sqrtf(p.x * p.x + p.y * p.y);
}

static float fb_dist(fb_pt_t a, fb_pt_t b)
{
	return fb_norm(fb_sub(a, b));
}

static fb_circle_hit_t fb_circle_intersect(fb_pt_t c1, float r1,
					   fb_pt_t c2, float r2)
{
	fb_circle_hit_t hit = { .count = 0 };
	fb_pt_t dvec = fb_sub(c2, c1);
	float d = fb_norm(dvec);

	if (d < FB_EPS) return hit;
	if (d > r1 + r2 + FB_EPS) return hit;
	if (d < fabsf(r1 - r2) - FB_EPS) return hit;

	fb_pt_t ex = fb_scale(dvec, 1.0f / d);
	float a = (r1 * r1 - r2 * r2 + d * d) / (2.0f * d);
	float h2 = r1 * r1 - a * a;
	float h = (h2 > 0.0f) ? sqrtf(h2) : 0.0f;

	fb_pt_t p0 = fb_add(c1, fb_scale(ex, a));
	fb_pt_t ey = fb_pt(-ex.y, ex.x);

	if (h <= FB_EPS) {
		hit.pts[0] = p0;
		hit.count = 1;
	} else {
		hit.pts[0] = fb_add(p0, fb_scale(ey, h));
		hit.pts[1] = fb_sub(p0, fb_scale(ey, h));
		hit.count = 2;
	}
	return hit;
}

/* side = +1 (right leg), -1 (left leg) → B.x polarity */
static fb_pt_t fb_get_B(int8_t side)
{
	int8_t s = (side >= 0) ? 1 : -1;
	return fb_pt(-(float)s * FB_L4 / FB_SQRT2, FB_L4 / FB_SQRT2);
}

/*
 * Forward kinematics:  qC  →  leg_length
 *
 *   D  = C - L2·[cos(qC), sin(qC)]
 *   A  = circle(B, L3) ∩ circle(D, L23)
 *   E  = D + L1·(D - A) / L23
 *   leg_length = -E.y
 *
 * Returns true when the four-bar can be closed.
 */
static bool fb_forward(float qC, int8_t side, fb_state_t *st)
{
	if (st == NULL) return false;

	memset(st, 0, sizeof(*st));

	st->C = fb_pt(0.0f, 0.0f);
	st->B = fb_get_B(side);
	st->qC = qC;

	st->D = fb_pt(-FB_L2 * cosf(qC), -FB_L2 * sinf(qC));

	fb_circle_hit_t cand = fb_circle_intersect(st->B, FB_L3, st->D, FB_L23);
	if (cand.count == 0) return false;

	/* pick the solution on the outboard side */
	float best_score = -1.0e30f;
	uint8_t best_idx = 0;
	int8_t s = (side >= 0) ? 1 : -1;

	for (uint8_t i = 0; i < cand.count; i++) {
		float score = -(float)s * cand.pts[i].x;
		if (score > best_score) {
			best_score = score;
			best_idx = i;
		}
	}

	st->A = cand.pts[best_idx];
	fb_pt_t dir = fb_scale(fb_sub(st->D, st->A), 1.0f / FB_L23);
	st->E = fb_add(st->D, fb_scale(dir, FB_L1));
	st->leg_length = -st->E.y;
	st->closure_error = fabsf(fb_dist(st->A, st->B) - FB_L3);
	st->valid = 1;
	return true;
}

/* joint_angle → leg_length (with measured-reference adjustment) */
static bool fb_leg_length_from_joint(float joint_rad, int8_t side,
				     fb_state_t *st)
{
	float offset = (side >= 0) ? FB_OFF_RIGHT : FB_OFF_LEFT;
	float qC = offset - joint_rad;
	if (!fb_forward(qC, side, st)) return false;
	st->leg_length -= FB_DL;
	return true;
}

/* leg_length → joint_angle (inverse, via coarse scan + bisection)
 *
 * Starting the bisection at qC = 0 is unreliable because the four-bar
 * cannot close near qC = 0 for this geometry (circles don't intersect).
 * Instead we do a coarse scan to find a valid bracket, then bisect. */
static bool fb_joint_from_leg_length(float leg_length_m, int8_t side,
				     float *joint_rad)
{
	float offset = (side >= 0) ? FB_OFF_RIGHT : FB_OFF_LEFT;
	float target = leg_length_m + FB_DL;
	fb_state_t st;
	bool found = false;
	float best_qC = 0.0f, best_err = 1e9f;

	/* Coarse scan over the full qC range to find valid bracket.
	 * fb_forward fails where the linkage can't close; we skip those. */
	float lo = 0.0f, hi = 0.0f;
	bool lo_set = false, hi_set = false;

	for (int i = 0; i <= 32; i++) {
		float qC = -3.1416f + (float)i * (6.2832f / 32.0f);
		if (!fb_forward(qC, side, &st)) continue;
		float e = st.leg_length - target;
		if (fabsf(e) < best_err) {
			best_err = fabsf(e);
			best_qC = qC;
			found = true;
		}
		if (e > 0.0f) {
			if (!hi_set || qC < hi) hi = qC;
			hi_set = true;
		} else {
			if (!lo_set || qC > lo) lo = qC;
			lo_set = true;
		}
	}

	/* Refine with bisection if we have a bracket */
	if (lo_set && hi_set) {
		if (lo > hi) { float t = lo; lo = hi; hi = t; }
		for (int iter = 0; iter < 20; iter++) {
			float mid = 0.5f * (lo + hi);
			if (!fb_forward(mid, side, &st)) break;
			float e = st.leg_length - target;
			if (fabsf(e) < 1e-5f) {
				best_qC = mid; found = true; break;
			}
			if (fabsf(e) < best_err) {
				best_err = fabsf(e); best_qC = mid;
			}
			if (e > 0.0f) hi = mid; else lo = mid;
		}
	}

	if (found) *joint_rad = offset - best_qC;
	return found;
}

/* ------------------------------------------------------------------ */
/* Public mapping functions (four-bar based)                            */
/* ------------------------------------------------------------------ */

static float wheel_forward_sign(bool left_wheel)
{
	return left_wheel ? (float)APP_WHEEL_LEFT_FORWARD_CURRENT_SIGN :
			    (float)APP_WHEEL_RIGHT_FORWARD_CURRENT_SIGN;
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
	       params->left_current_ma_to_wheel_torque_nm > 0.0f &&
	       params->right_current_ma_to_wheel_torque_nm > 0.0f &&
	       params->leg_length_max_m > params->leg_length_min_m;
}

/*
 * Piecewise-linear joint-angle ↔ leg-length lookup (backup).
 * Kept for reference; the four-bar analytical model is the active path.
 */
#if 0

struct leg_cal_pt { float q_rad; float L_m; };

static float leg_lookup_fwd(float q, const struct leg_cal_pt *t, int n)
{
	if (q <= t[0].q_rad) return t[0].L_m;
	if (q >= t[n - 1].q_rad) return t[n - 1].L_m;
	for (int i = 0; i < n - 1; i++) {
		if (q <= t[i + 1].q_rad) {
			float r = (q - t[i].q_rad) / (t[i + 1].q_rad - t[i].q_rad);
			return t[i].L_m + r * (t[i + 1].L_m - t[i].L_m);
		}
	}
	return t[n - 1].L_m;
}

static float leg_lookup_inv(float L, const struct leg_cal_pt *t, int n)
{
	bool L_rises = (t[n - 1].L_m > t[0].L_m);

	if (L_rises) {
		if (L <= t[0].L_m) return t[0].q_rad;
		if (L >= t[n - 1].L_m) return t[n - 1].q_rad;
	} else {
		if (L >= t[0].L_m) return t[0].q_rad;
		if (L <= t[n - 1].L_m) return t[n - 1].q_rad;
	}

	for (int i = 0; i < n - 1; i++) {
		float La = t[i].L_m;
		float Lb = t[i + 1].L_m;
		bool in_seg = L_rises ? (L >= La && L <= Lb)
				      : (L <= La && L >= Lb);
		if (in_seg) {
			float r = fabsf((L - La) / (Lb - La));
			return t[i].q_rad + r * (t[i + 1].q_rad - t[i].q_rad);
		}
	}
	return t[n - 1].q_rad;
}

#endif /* piecewise-linear backup */

float ascento_balance_leg_length_from_joint(const ascento_balance_params_t *params,
					    bool left_leg, float joint_rad)
{
	(void)params;
	int8_t side = left_leg ? -1 : 1;
	fb_state_t st;
	if (fb_leg_length_from_joint(joint_rad, side, &st)) {
		return st.leg_length;
	}
	return APP_ASCENTO_LEG_LENGTH_MIN_M;
}

float ascento_balance_joint_from_leg_length(const ascento_balance_params_t *params,
					    bool left_leg, float leg_length_m)
{
	(void)params;
	int8_t side = left_leg ? -1 : 1;
	float joint_rad;
	if (fb_joint_from_leg_length(leg_length_m, side, &joint_rad)) {
		return joint_rad;
	}
	/* IK failed — return lock position at stand height (safe default). */
	return left_leg ? APP_PID_BALANCE_LOCK_LEFT_JOINT_RAD
			: APP_PID_BALANCE_LOCK_RIGHT_JOINT_RAD;
}

static int16_t torque_to_current_ma(float torque_nm,
				    float current_ma_to_wheel_torque_nm)
{
	const float current_ma = torque_nm / current_ma_to_wheel_torque_nm;
	return app_clamp_i16((int32_t)lrintf(current_ma), -APP_WHEEL_CURRENT_LIMIT,
			     APP_WHEEL_CURRENT_LIMIT);
}

/*
 * Evaluate the polynomial gain schedule at the given leg length.
 * Returns the four gains through pointer arguments.
 */
static void compute_gains(float leg_length_m, float *k_pitch, float *k_pitch_rate,
			  float *k_position, float *k_velocity)
{
	float L = leg_length_m;
	*k_pitch      = APP_ASCENTO_GAIN_C0_A * L * L +
			APP_ASCENTO_GAIN_C0_B * L + APP_ASCENTO_GAIN_C0_C;
	*k_pitch_rate = APP_ASCENTO_GAIN_C1_A * L * L +
			APP_ASCENTO_GAIN_C1_B * L + APP_ASCENTO_GAIN_C1_C;
	*k_position   = APP_ASCENTO_GAIN_C2;
	*k_velocity   = APP_ASCENTO_GAIN_C3_A * L * L +
			APP_ASCENTO_GAIN_C3_B * L + APP_ASCENTO_GAIN_C3_C;
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
		static uint32_t diag_cnt;
		if ((diag_cnt++ & 0x3ff) == 0) {
			LOG_WRN("model blocked: params_ready=%d enable_req=%d "
				"wheel_fb_ok=%d calib=%d",
				output->params_ready, input->enable_request,
				input->wheel_feedback_ok, params->calibrated);
		}
		ascento_balance_reset(state);
		return;
	}

	float dt_s = input->dt_s;
	if (dt_s <= 0.0f || dt_s > 0.02f) {
		dt_s = 1.0f / APP_CONTROL_HZ;
	}

	const float left_wheel_angle =
		wheel_forward_sign(true) * input->left_wheel.angle_rad /
		APP_M3508_REDUCTION_RATIO;
	const float right_wheel_angle =
		wheel_forward_sign(false) * input->right_wheel.angle_rad /
		APP_M3508_REDUCTION_RATIO;
	const float left_wheel_speed =
		wheel_forward_sign(true) * input->left_wheel.speed_rad_s /
		APP_M3508_REDUCTION_RATIO;
	const float right_wheel_speed =
		wheel_forward_sign(false) * input->right_wheel.speed_rad_s /
		APP_M3508_REDUCTION_RATIO;

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
	const float pitch_rad = APP_ASCENTO_IMU_PITCH_SIGN *
				 input->imu.pitch_deg * DEG_TO_RAD;
	const float pitch_rate_rad_s = APP_ASCENTO_IMU_GY_SIGN *
				       input->imu.gx_dps * DEG_TO_RAD;
	const float yaw_rate_rad_s = APP_ASCENTO_IMU_GZ_SIGN *
				     app_lpf_update(input->imu.gz_dps * DEG_TO_RAD,
						    &state->yaw_rate_lpf_rad_s,
						    0.025f, dt_s);
	const float roll_rad = APP_ASCENTO_IMU_ROLL_SIGN *
			       input->imu.roll_deg * DEG_TO_RAD;

	const float x_error = state->body_position_m;
	const float v_error = velocity_mps - input->target_forward_speed_mps;

	/* Gain scheduling: compute gains from current average leg length. */
	const float avg_leg_length_m =
		0.5f * (ascento_balance_leg_length_from_joint(params, true,
				input->left_joint_position_rad) +
			ascento_balance_leg_length_from_joint(params, false,
				input->right_joint_position_rad));
	float k_pitch, k_pitch_rate, k_position, k_velocity;
	compute_gains(avg_leg_length_m, &k_pitch, &k_pitch_rate,
		      &k_position, &k_velocity);

	/*
	 * Equilibrium pitch is non-zero because the COM lies behind the
	 * wheel axle (x_com = −0.040 m).  The robot balances at a forward
	 * lean.  theta_eq ≈ atan(−x_com / h_com) ≈ +0.367 rad (+21°).
	 */
	const float theta_eq = APP_ASCENTO_THETA_EQ_STAND_RAD;
	const float pitch_error = pitch_rad - theta_eq - input->target_pitch_rad;
	const float pitch_error_deg = fabsf(pitch_error) / DEG_TO_RAD;

	if (pitch_error_deg > APP_PITCH_FAULT_DEG) {
		state->faulted = true;
		state->recover_ticks = 0;
	}

	if (state->faulted) {
		if (input->enable_request &&
		    pitch_error_deg < APP_PITCH_RECOVER_DEG) {
			state->recover_ticks++;
			if (state->recover_ticks > APP_BALANCE_RECOVER_TICKS) {
				state->faulted = false;
				state->recover_ticks = 0;
				state->body_position_m = wheel_distance_m;
			}
		} else {
			state->recover_ticks = 0;
		}

		if (state->faulted) {
			static uint32_t fault_log_cnt;
			if ((fault_log_cnt++ & 0x3ff) == 0) {
				LOG_WRN("faulted | pitch=%.2f theta_eq=%.2f "
					"err_deg=%.1f recover=%d/%d",
					(double)pitch_rad, (double)theta_eq,
					(double)pitch_error_deg,
					state->recover_ticks,
					APP_BALANCE_RECOVER_TICKS);
			}
			output->faulted = true;
			output->pitch_rad = pitch_rad;
			output->pitch_rate_rad_s = pitch_rate_rad_s;
			output->body_position_m = state->body_position_m;
			output->body_velocity_mps = velocity_mps;
			return;
		}
	}

	float balance_torque_nm =
		-(k_pitch * pitch_error +
		  k_pitch_rate * pitch_rate_rad_s +
		  k_position * x_error +
		  k_velocity * v_error);
	float yaw_torque_nm =
		-params->k_yaw_rate * (yaw_rate_rad_s -
				       input->target_yaw_rate_rad_s);

	const float weaker_torque_coeff =
		fminf(params->left_current_ma_to_wheel_torque_nm,
		      params->right_current_ma_to_wheel_torque_nm);
	const float current_limit_torque =
		weaker_torque_coeff * (float)APP_WHEEL_CURRENT_LIMIT;
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
		(int16_t)((float)torque_to_current_ma(
				  balance_torque_nm - yaw_torque_nm,
				  params->left_current_ma_to_wheel_torque_nm) *
				  APP_ASCENTO_WHEEL_CURRENT_SCALE);
	output->right_wheel_current =
		(int16_t)((float)torque_to_current_ma(
				  balance_torque_nm + yaw_torque_nm,
				  params->right_current_ma_to_wheel_torque_nm) *
				  APP_ASCENTO_WHEEL_CURRENT_SCALE);

	/* Stiction: add a bias to overcome VESC minimum-current threshold.
	 * Ramps from 0 at START_DEG to full at FULL_DEG. */
	if (APP_ASCENTO_STICTION_CURRENT_MA > 0.0f) {
		const float st_s = app_clampf(
			(pitch_error_deg -
			 APP_ASCENTO_STICTION_START_DEG) /
				(APP_ASCENTO_STICTION_FULL_DEG -
				 APP_ASCENTO_STICTION_START_DEG),
			0.0f, 1.0f);
		if (st_s > 0.0f) {
			const int16_t st_ma = (int16_t)lrintf(
				APP_ASCENTO_STICTION_CURRENT_MA * st_s *
				APP_ASCENTO_WHEEL_CURRENT_SCALE);
			const int16_t sign =
				(balance_torque_nm >= 0.0f) ? 1 : -1;
			output->left_wheel_current += sign * st_ma;
			output->right_wheel_current += sign * st_ma;
		}
	}

	output->left_joint_position_rad =
		ascento_balance_joint_from_leg_length(params, true,
						      left_leg_length);
	output->right_joint_position_rad =
		ascento_balance_joint_from_leg_length(params, false,
						      right_leg_length);

	/* diagnostic: log ~1 Hz. Lt=target, L=actual, jL/jR=IK joint targets */
	{
		static uint32_t diag_active_cnt;
		if ((diag_active_cnt++ & 0x3ff) == 0) {
			LOG_INF("active | pitch=%.2f err=%.2f torque=%.2f "
				"cur=%d/%d Lt=%.3f L=%.3f jL=%.2f jR=%.2f "
				"K=%.2f/%.2f/%.2f/%.2f",
				(double)pitch_rad, (double)pitch_error,
				(double)balance_torque_nm,
				output->left_wheel_current,
				output->right_wheel_current,
				(double)target_leg_length,
				(double)avg_leg_length_m,
				(double)output->left_joint_position_rad,
				(double)output->right_joint_position_rad,
				(double)k_pitch, (double)k_pitch_rate,
				(double)k_position, (double)k_velocity);
		}
	}
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
