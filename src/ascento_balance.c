#include "ascento_balance.h"

#include <math.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "app_config.h"
#include "pid.h"

LOG_MODULE_REGISTER(ascento, LOG_LEVEL_INF);

#define DEG_TO_RAD 0.017453292519943295f

static const ascento_balance_params_t ascento_balance_code_defaults = {
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

	/* Runtime-tunable fields. */
	.theta_eq_rad = APP_ASCENTO_THETA_EQ_STAND_RAD,
	.gain_c0_a = APP_ASCENTO_GAIN_C0_A,
	.gain_c0_b = APP_ASCENTO_GAIN_C0_B,
	.gain_c0_c = APP_ASCENTO_GAIN_C0_C,
	.gain_c1_a = APP_ASCENTO_GAIN_C1_A,
	.gain_c1_b = APP_ASCENTO_GAIN_C1_B,
	.gain_c1_c = APP_ASCENTO_GAIN_C1_C,
	.gain_c2   = APP_ASCENTO_GAIN_C2,
	.gain_c3_a = APP_ASCENTO_GAIN_C3_A,
	.gain_c3_b = APP_ASCENTO_GAIN_C3_B,
	.gain_c3_c = APP_ASCENTO_GAIN_C3_C,
	.stiction_current_ma = APP_ASCENTO_STICTION_CURRENT_MA,
	.stiction_start_deg  = APP_ASCENTO_STICTION_START_DEG,
	.stiction_full_deg   = APP_ASCENTO_STICTION_FULL_DEG,
	.current_limit_ma    = APP_WHEEL_CURRENT_LIMIT,
	.current_scale       = APP_ASCENTO_WHEEL_CURRENT_SCALE,
	.fault_deg           = APP_PITCH_FAULT_DEG,
	.recover_deg         = APP_PITCH_RECOVER_DEG,
	.wheel_sync_gain_ma  = APP_ASCENTO_WHEEL_SYNC_GAIN_MA,
	.wheel_sync_current_limit_ma = APP_ASCENTO_WHEEL_SYNC_CURRENT_LIMIT,
};

K_MUTEX_DEFINE(params_lock);

static ascento_balance_params_t ascento_balance_runtime_params;
static bool ascento_balance_params_initialized;

/* Backward-compat alias for code that still reads the old name. */
const ascento_balance_params_t *ascento_balance_default_params =
	&ascento_balance_code_defaults;

static void ascento_balance_params_init_once(void)
{
	k_mutex_lock(&params_lock, K_FOREVER);
	if (!ascento_balance_params_initialized) {
		ascento_balance_runtime_params = ascento_balance_code_defaults;
		ascento_balance_params_initialized = true;
	}
	k_mutex_unlock(&params_lock);
}

void ascento_balance_get_params(ascento_balance_params_t *params)
{
	ascento_balance_params_init_once();
	k_mutex_lock(&params_lock, K_FOREVER);
	*params = ascento_balance_runtime_params;
	k_mutex_unlock(&params_lock);
}

void ascento_balance_set_params(const ascento_balance_params_t *params)
{
	ascento_balance_params_init_once();
	k_mutex_lock(&params_lock, K_FOREVER);
	ascento_balance_runtime_params = *params;
	ascento_balance_params_initialized = true;
	k_mutex_unlock(&params_lock);
}

/* --- Persistent storage via direct flash read/write --- */
/*
 * NVS cannot be used because STM32F407 128KB flash sectors exceed the
 * uint16_t sector_size field in nvs_fs (max 65535).  Instead we erase
 * the whole partition and write a simple header+payload+CRC blob.
 */
#include <zephyr/storage/flash_map.h>
#include <zephyr/sys/crc.h>

#define PARAMS_FLASH_LABEL  storage_partition
#define PARAMS_MAGIC        0x41534350  /* "ASC P" */
#define PARAMS_VERSION      14

struct params_flash_header {
	uint32_t magic;
	uint32_t version;
	uint32_t data_size;
};

int ascento_balance_save_params(void)
{
	ascento_balance_params_init_once();

	const struct flash_area *fa;
	int rc = flash_area_open(FIXED_PARTITION_ID(PARAMS_FLASH_LABEL), &fa);
	if (rc) {
		printk("ascento: flash_area_open failed: %d\n", rc);
		return rc;
	}

	struct params_flash_header hdr = {
		.magic = PARAMS_MAGIC,
		.version = PARAMS_VERSION,
		.data_size = sizeof(ascento_balance_params_t),
	};

	uint8_t buf[sizeof(hdr) + sizeof(ascento_balance_params_t) + 4];
	memcpy(buf, &hdr, sizeof(hdr));

	k_mutex_lock(&params_lock, K_FOREVER);
	memcpy(buf + sizeof(hdr), &ascento_balance_runtime_params,
	       sizeof(ascento_balance_params_t));
	k_mutex_unlock(&params_lock);

	uint32_t crc = crc32_ieee(buf, sizeof(hdr) + hdr.data_size);
	memcpy(buf + sizeof(hdr) + hdr.data_size, &crc, sizeof(crc));

	const size_t total = sizeof(hdr) + hdr.data_size + sizeof(crc);

	rc = flash_area_erase(fa, 0, fa->fa_size);
	if (rc) {
		printk("ascento: flash erase failed: %d\n", rc);
		flash_area_close(fa);
		return rc;
	}

	rc = flash_area_write(fa, 0, buf, total);
	flash_area_close(fa);

	if (rc) {
		printk("ascento: flash write failed: %d\n", rc);
	} else {
		printk("ascento: params saved to flash (%u bytes)\n",
		       (unsigned)total);
	}
	return rc;
}

int ascento_balance_reset_params(void)
{
	k_mutex_lock(&params_lock, K_FOREVER);
	ascento_balance_runtime_params = ascento_balance_code_defaults;
	ascento_balance_params_initialized = true;
	k_mutex_unlock(&params_lock);

	const struct flash_area *fa;
	int rc = flash_area_open(FIXED_PARTITION_ID(PARAMS_FLASH_LABEL), &fa);
	if (rc) {
		return rc;
	}
	rc = flash_area_erase(fa, 0, fa->fa_size);
	flash_area_close(fa);

	if (rc == 0) {
		printk("ascento: params reset to defaults, flash cleared\n");
	} else {
		printk("ascento: flash erase failed: %d (params reset in RAM)\n",
		       rc);
	}
	return rc;
}

int ascento_balance_settings_init(void)
{
	ascento_balance_params_init_once();

	const struct flash_area *fa;
	int rc = flash_area_open(FIXED_PARTITION_ID(PARAMS_FLASH_LABEL), &fa);
	if (rc) {
		printk("ascento: flash_area_open failed: %d\n", rc);
		return rc;
	}

	struct params_flash_header hdr;
	rc = flash_area_read(fa, 0, &hdr, sizeof(hdr));
	if (rc) {
		printk("ascento: flash read header failed: %d\n", rc);
		flash_area_close(fa);
		return rc;
	}

	if (hdr.magic != PARAMS_MAGIC || hdr.version != PARAMS_VERSION) {
		printk("ascento: no saved params (magic=0x%08x ver=%u)\n",
		       hdr.magic, hdr.version);
		flash_area_close(fa);
		return 0;  /* not an error — just use defaults */
	}

	if (hdr.data_size != sizeof(ascento_balance_params_t)) {
		printk("ascento: params size mismatch: got %u want %u\n",
		       (unsigned)hdr.data_size,
		       (unsigned)sizeof(ascento_balance_params_t));
		flash_area_close(fa);
		return -EINVAL;
	}

	uint8_t buf[sizeof(hdr) + sizeof(ascento_balance_params_t) + 4];
	const size_t total = sizeof(hdr) + hdr.data_size + sizeof(uint32_t);
	rc = flash_area_read(fa, 0, buf, total);
	flash_area_close(fa);
	if (rc) {
		printk("ascento: flash read data failed: %d\n", rc);
		return rc;
	}

	uint32_t stored_crc;
	memcpy(&stored_crc, buf + sizeof(hdr) + hdr.data_size, sizeof(stored_crc));
	uint32_t calc_crc = crc32_ieee(buf, sizeof(hdr) + hdr.data_size);
	if (stored_crc != calc_crc) {
		printk("ascento: CRC mismatch (stored=0x%08x calc=0x%08x)\n",
		       stored_crc, calc_crc);
		return -EIO;
	}

	ascento_balance_params_t tmp;
	memcpy(&tmp, buf + sizeof(hdr), sizeof(tmp));

	k_mutex_lock(&params_lock, K_FOREVER);
	ascento_balance_runtime_params = tmp;
	ascento_balance_params_initialized = true;
	k_mutex_unlock(&params_lock);

	printk("ascento: loaded params from flash OK\n");
	return 0;
}

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

/* Return the four fixed balance gains. */
static void get_fixed_gains(const ascento_balance_params_t *p,
			    float *k_pitch, float *k_pitch_rate,
			    float *k_position, float *k_velocity)
{
	*k_pitch = p->k_pitch;
	*k_pitch_rate = p->k_pitch_rate;
	*k_position = p->k_position;
	*k_velocity = p->k_velocity;
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
	if (!output->params_ready) {
		static uint32_t diag_cnt;
		if ((diag_cnt++ & 0x3ff) == 0) {
			LOG_WRN("model blocked: params_ready=%d calib=%d",
				output->params_ready, params->calibrated);
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
		state->wheel_position_zero_m = wheel_distance_m;
		state->body_position_m = 0.0f;
		state->body_velocity_lpf_mps = body_velocity_mps;
		state->yaw_rate_lpf_rad_s = input->imu.gz_dps * DEG_TO_RAD;
		state->initialized = true;
	} else {
		state->body_position_m =
			wheel_distance_m - state->wheel_position_zero_m;
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
	const float x_error = state->body_position_m;
	const float v_error = velocity_mps - input->target_forward_speed_mps;

	float k_pitch, k_pitch_rate, k_position, k_velocity;
	get_fixed_gains(params, &k_pitch, &k_pitch_rate, &k_position,
			&k_velocity);

	const float theta_eq = params->theta_eq_rad;
	const float pitch_error = pitch_rad - theta_eq - input->target_pitch_rad;
	const float pitch_deg = pitch_rad / DEG_TO_RAD;
	const float abs_pitch_deg = fabsf(pitch_deg);
	const float runtime_fault_deg =
		app_clampf(params->fault_deg, 5.0f,
			   APP_ASCENTO_FORWARD_HARD_FAULT_DEG);
	const float forward_fault_deg =
		fminf(runtime_fault_deg, APP_ASCENTO_FORWARD_HARD_FAULT_DEG);
	const float backward_fault_deg =
		fminf(runtime_fault_deg, APP_ASCENTO_BACKWARD_HARD_FAULT_DEG);
	const bool over_forward_limit = pitch_deg > forward_fault_deg;
	const bool over_backward_limit = pitch_deg < -backward_fault_deg;

	(void)over_forward_limit;
	(void)over_backward_limit;
	(void)abs_pitch_deg;
	(void)forward_fault_deg;
	(void)backward_fault_deg;

	float balance_torque_nm =
		-(k_pitch * pitch_error +
		  k_pitch_rate * pitch_rate_rad_s +
		  k_position * x_error +
		  k_velocity * v_error);
	float yaw_torque_nm =
		-params->k_yaw_rate * (yaw_rate_rad_s -
				       input->target_yaw_rate_rad_s);

	output->active = true;
	const float left_current_ma =
		(balance_torque_nm - yaw_torque_nm) /
		params->left_current_ma_to_wheel_torque_nm *
		params->current_scale;
	const float right_current_ma =
		(balance_torque_nm + yaw_torque_nm) /
		params->right_current_ma_to_wheel_torque_nm *
		params->current_scale;
	output->left_wheel_current = app_clamp_i16(
		(int32_t)lrintf(left_current_ma), INT16_MIN, INT16_MAX);
	output->right_wheel_current = app_clamp_i16(
		(int32_t)lrintf(right_current_ma), INT16_MIN, INT16_MAX);

	/* Wheel speed synchronization: compensate for left/right speed mismatch.
	 *
	 * If left wheel is faster than right, add current to right and reduce
	 * from left. This equalizes wheel speeds without affecting the average
	 * torque used for balancing.
	 */
	if (params->wheel_sync_gain_ma > 0.0f) {
		const float left_speed = wheel_forward_sign(true) *
			input->left_wheel.speed_rad_s /
			APP_M3508_REDUCTION_RATIO;
		const float right_speed = wheel_forward_sign(false) *
			input->right_wheel.speed_rad_s /
			APP_M3508_REDUCTION_RATIO;
		const float speed_error = left_speed - right_speed;
		const float sync_current = app_clampf(
			params->wheel_sync_gain_ma * speed_error,
			-params->wheel_sync_current_limit_ma,
			params->wheel_sync_current_limit_ma);
		output->left_wheel_current -= (int16_t)lrintf(sync_current);
		output->right_wheel_current += (int16_t)lrintf(sync_current);
	}

	output->left_joint_position_rad = APP_PID_BALANCE_LOCK_LEFT_JOINT_RAD;
	output->right_joint_position_rad = APP_PID_BALANCE_LOCK_RIGHT_JOINT_RAD;

	/* diagnostic: log ~1 Hz. */
	{
		static uint32_t diag_active_cnt;
		if ((diag_active_cnt++ & 0x3ff) == 0) {
			LOG_INF("active | pitch=%.2f err=%.2f torque=%.2f "
				"cur=%d/%d joint=%.2f/%.2f K=%.2f/%.2f/%.2f/%.2f",
				(double)pitch_rad, (double)pitch_error,
				(double)balance_torque_nm,
				output->left_wheel_current,
				output->right_wheel_current,
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
}
