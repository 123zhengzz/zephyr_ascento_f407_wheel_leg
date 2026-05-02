#include "motor_debug.h"

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include "app_config.h"
#include "pid.h"

typedef struct {
	bool wheel_active;
	motor_debug_wheel_mode_t wheel_mode;
	int64_t wheel_deadline_ms;
	int16_t wheel_current[DJI_M3508_MAX_ID + 1];
	int32_t left_wheel_rpm;
	int32_t right_wheel_rpm;

	motor_debug_dm_command_t dm[DM4340_MAX_ID + 1];
	int64_t dm_deadline_ms[DM4340_MAX_ID + 1];
} motor_debug_state_t;

static struct {
	dji_m3508_bus_t *dji_bus;
	dm4340_bus_t *dm_bus;
	struct k_spinlock lock;
	motor_debug_state_t state;
} ctx;

static int64_t deadline_from_duration(int32_t duration_ms)
{
	if (duration_ms <= 0) {
		duration_ms = APP_MOTOR_DEBUG_DEFAULT_TIMEOUT_MS;
	}
	if (duration_ms > APP_MOTOR_DEBUG_MAX_TIMEOUT_MS) {
		duration_ms = APP_MOTOR_DEBUG_MAX_TIMEOUT_MS;
	}

	return k_uptime_get() + duration_ms;
}

static bool valid_wheel_id(uint8_t id)
{
	return id > 0;
}

static bool valid_dm_id(uint8_t id)
{
	return id > 0 && id <= DM4340_MAX_ID;
}

static float clamp_joint_position(uint8_t id, float position_rad)
{
	if (id == APP_DM_LEFT_ID) {
		return app_clampf(position_rad, APP_LEFT_LEG_MIN_RAD,
				  APP_LEFT_LEG_MAX_RAD);
	}
	if (id == APP_DM_RIGHT_ID) {
		return app_clampf(position_rad, APP_RIGHT_LEG_MIN_RAD,
				  APP_RIGHT_LEG_MAX_RAD);
	}

	return app_clampf(position_rad, -1.0f, 1.0f);
}

static void clear_expired_locked(int64_t now_ms)
{
	if (ctx.state.wheel_active && now_ms > ctx.state.wheel_deadline_ms) {
		ctx.state.wheel_active = false;
		ctx.state.wheel_mode = MOTOR_DEBUG_WHEEL_NONE;
		memset(ctx.state.wheel_current, 0,
		       sizeof(ctx.state.wheel_current));
		ctx.state.left_wheel_rpm = 0;
		ctx.state.right_wheel_rpm = 0;
	}

	for (uint8_t id = 1; id <= DM4340_MAX_ID; id++) {
		if (ctx.state.dm[id].mode != MOTOR_DEBUG_DM_NONE &&
		    now_ms > ctx.state.dm_deadline_ms[id]) {
			ctx.state.dm[id] = (motor_debug_dm_command_t){ 0 };
			ctx.state.dm_deadline_ms[id] = 0;
		}
	}
}

void motor_debug_init(dji_m3508_bus_t *dji_bus, dm4340_bus_t *dm_bus)
{
	memset(&ctx, 0, sizeof(ctx));
	ctx.dji_bus = dji_bus;
	ctx.dm_bus = dm_bus;
}

int motor_debug_set_wheel_current(uint8_t id, int32_t current,
				  int32_t duration_ms)
{
	if (!valid_wheel_id(id)) {
		return -EINVAL;
	}

	current = app_clamp_i16(current, -APP_M3508_DEBUG_CURRENT_LIMIT,
				APP_M3508_DEBUG_CURRENT_LIMIT);

	k_spinlock_key_t key = k_spin_lock(&ctx.lock);
	memset(ctx.state.wheel_current, 0, sizeof(ctx.state.wheel_current));
	ctx.state.left_wheel_rpm = 0;
	ctx.state.right_wheel_rpm = 0;
	ctx.state.wheel_current[id] = current;
	ctx.state.wheel_active = true;
	ctx.state.wheel_mode = MOTOR_DEBUG_WHEEL_CURRENT;
	ctx.state.wheel_deadline_ms = deadline_from_duration(duration_ms);
	k_spin_unlock(&ctx.lock, key);

	return 0;
}

int motor_debug_set_wheel_pair(int32_t left_current, int32_t right_current,
			       int32_t duration_ms)
{
	left_current = app_clamp_i16(left_current,
				     -APP_M3508_DEBUG_CURRENT_LIMIT,
				     APP_M3508_DEBUG_CURRENT_LIMIT);
	right_current = app_clamp_i16(right_current,
				      -APP_M3508_DEBUG_CURRENT_LIMIT,
				      APP_M3508_DEBUG_CURRENT_LIMIT);

	k_spinlock_key_t key = k_spin_lock(&ctx.lock);
	memset(ctx.state.wheel_current, 0, sizeof(ctx.state.wheel_current));
	ctx.state.left_wheel_rpm = 0;
	ctx.state.right_wheel_rpm = 0;
	ctx.state.wheel_current[APP_WHEEL_LEFT_ID] = left_current;
	ctx.state.wheel_current[APP_WHEEL_RIGHT_ID] = right_current;
	ctx.state.wheel_active = true;
	ctx.state.wheel_mode = MOTOR_DEBUG_WHEEL_CURRENT;
	ctx.state.wheel_deadline_ms = deadline_from_duration(duration_ms);
	k_spin_unlock(&ctx.lock, key);

	return 0;
}

int motor_debug_set_wheel_rpm(uint8_t id, int32_t rpm, int32_t duration_ms)
{
	if (id != APP_WHEEL_LEFT_ID && id != APP_WHEEL_RIGHT_ID) {
		return -EINVAL;
	}

	rpm = CLAMP(rpm, -APP_VESC_DEBUG_ERPM_LIMIT,
		    APP_VESC_DEBUG_ERPM_LIMIT);

	k_spinlock_key_t key = k_spin_lock(&ctx.lock);
	memset(ctx.state.wheel_current, 0, sizeof(ctx.state.wheel_current));
	ctx.state.left_wheel_rpm = id == APP_WHEEL_LEFT_ID ? rpm : 0;
	ctx.state.right_wheel_rpm = id == APP_WHEEL_RIGHT_ID ? rpm : 0;
	ctx.state.wheel_active = true;
	ctx.state.wheel_mode = MOTOR_DEBUG_WHEEL_RPM;
	ctx.state.wheel_deadline_ms = deadline_from_duration(duration_ms);
	k_spin_unlock(&ctx.lock, key);

	return 0;
}

int motor_debug_stop_wheels(void)
{
	k_spinlock_key_t key = k_spin_lock(&ctx.lock);
	ctx.state.wheel_active = false;
	ctx.state.wheel_mode = MOTOR_DEBUG_WHEEL_NONE;
	ctx.state.wheel_deadline_ms = 0;
	memset(ctx.state.wheel_current, 0, sizeof(ctx.state.wheel_current));
	ctx.state.left_wheel_rpm = 0;
	ctx.state.right_wheel_rpm = 0;
	k_spin_unlock(&ctx.lock, key);

	if (ctx.dji_bus != NULL) {
		return dji_m3508_send_group_current(ctx.dji_bus, 0, 0, 0, 0);
	}

	return 0;
}

int motor_debug_dm_enable(uint8_t id)
{
	if (!valid_dm_id(id) || ctx.dm_bus == NULL) {
		return -EINVAL;
	}

	return dm4340_enable(ctx.dm_bus, id);
}

int motor_debug_dm_disable(uint8_t id)
{
	if (!valid_dm_id(id) || ctx.dm_bus == NULL) {
		return -EINVAL;
	}

	(void)motor_debug_stop_dm(id);
	return dm4340_disable(ctx.dm_bus, id);
}

int motor_debug_dm_save_zero(uint8_t id)
{
	if (!valid_dm_id(id) || ctx.dm_bus == NULL) {
		return -EINVAL;
	}

	return dm4340_save_zero(ctx.dm_bus, id);
}

int motor_debug_set_dm_pos_vel(uint8_t id, float position_rad,
			       float velocity_rad_s, int32_t duration_ms)
{
	if (!valid_dm_id(id)) {
		return -EINVAL;
	}

	position_rad = clamp_joint_position(id, position_rad);
	velocity_rad_s = app_clampf(velocity_rad_s,
				    -APP_DM_DEBUG_VEL_LIMIT_RAD_S,
				    APP_DM_DEBUG_VEL_LIMIT_RAD_S);

	k_spinlock_key_t key = k_spin_lock(&ctx.lock);
	ctx.state.dm[id] = (motor_debug_dm_command_t) {
		.mode = MOTOR_DEBUG_DM_POS_VEL,
		.id = id,
		.position_rad = position_rad,
		.velocity_rad_s = velocity_rad_s,
	};
	ctx.state.dm_deadline_ms[id] = deadline_from_duration(duration_ms);
	k_spin_unlock(&ctx.lock, key);

	return 0;
}

int motor_debug_set_dm_velocity(uint8_t id, float velocity_rad_s,
				int32_t duration_ms)
{
	if (!valid_dm_id(id)) {
		return -EINVAL;
	}

	velocity_rad_s = app_clampf(velocity_rad_s,
				    -APP_DM_DEBUG_VEL_LIMIT_RAD_S,
				    APP_DM_DEBUG_VEL_LIMIT_RAD_S);

	k_spinlock_key_t key = k_spin_lock(&ctx.lock);
	ctx.state.dm[id] = (motor_debug_dm_command_t) {
		.mode = MOTOR_DEBUG_DM_VELOCITY,
		.id = id,
		.velocity_rad_s = velocity_rad_s,
	};
	ctx.state.dm_deadline_ms[id] = deadline_from_duration(duration_ms);
	k_spin_unlock(&ctx.lock, key);

	return 0;
}

int motor_debug_set_dm_mit(uint8_t id, float position_rad,
			   float velocity_rad_s, float kp, float kd,
			   float torque_nm, int32_t duration_ms)
{
	if (!valid_dm_id(id)) {
		return -EINVAL;
	}

	position_rad = clamp_joint_position(id, position_rad);
	velocity_rad_s = app_clampf(velocity_rad_s,
				    -APP_DM_DEBUG_VEL_LIMIT_RAD_S,
				    APP_DM_DEBUG_VEL_LIMIT_RAD_S);
	kp = app_clampf(kp, 0.0f, APP_DM_DEBUG_KP_LIMIT);
	kd = app_clampf(kd, 0.0f, APP_DM_DEBUG_KD_LIMIT);
	torque_nm = app_clampf(torque_nm, -APP_DM_DEBUG_TORQUE_LIMIT_NM,
			       APP_DM_DEBUG_TORQUE_LIMIT_NM);

	k_spinlock_key_t key = k_spin_lock(&ctx.lock);
	ctx.state.dm[id] = (motor_debug_dm_command_t) {
		.mode = MOTOR_DEBUG_DM_MIT,
		.id = id,
		.position_rad = position_rad,
		.velocity_rad_s = velocity_rad_s,
		.kp = kp,
		.kd = kd,
		.torque_nm = torque_nm,
	};
	ctx.state.dm_deadline_ms[id] = deadline_from_duration(duration_ms);
	k_spin_unlock(&ctx.lock, key);

	return 0;
}

int motor_debug_stop_dm(uint8_t id)
{
	if (!valid_dm_id(id)) {
		return -EINVAL;
	}

	k_spinlock_key_t key = k_spin_lock(&ctx.lock);
	ctx.state.dm[id] = (motor_debug_dm_command_t){ 0 };
	ctx.state.dm_deadline_ms[id] = 0;
	k_spin_unlock(&ctx.lock, key);

	if (ctx.dm_bus != NULL) {
		return dm4340_send_velocity(ctx.dm_bus, id, 0.0f);
	}

	return 0;
}

int motor_debug_stop_all(void)
{
	k_spinlock_key_t key = k_spin_lock(&ctx.lock);
	memset(&ctx.state, 0, sizeof(ctx.state));
	k_spin_unlock(&ctx.lock, key);

	if (ctx.dji_bus != NULL) {
		(void)dji_m3508_send_group_current(ctx.dji_bus, 0, 0, 0, 0);
	}

	if (ctx.dm_bus != NULL) {
		(void)dm4340_send_velocity(ctx.dm_bus, APP_DM_LEFT_ID, 0.0f);
		(void)dm4340_send_velocity(ctx.dm_bus, APP_DM_RIGHT_ID, 0.0f);
	}

	return 0;
}

bool motor_debug_get_output(motor_debug_output_t *out)
{
	if (out == NULL) {
		return false;
	}

	memset(out, 0, sizeof(*out));

	k_spinlock_key_t key = k_spin_lock(&ctx.lock);
	clear_expired_locked(k_uptime_get());

	out->wheel_active = ctx.state.wheel_active;
	if (out->wheel_active) {
		out->wheel_mode = ctx.state.wheel_mode;
		memcpy(out->wheel_current, ctx.state.wheel_current,
		       sizeof(out->wheel_current));
		out->left_wheel_rpm = ctx.state.left_wheel_rpm;
		out->right_wheel_rpm = ctx.state.right_wheel_rpm;
		out->active = true;
	}

	for (uint8_t id = 1; id <= DM4340_MAX_ID; id++) {
		if (ctx.state.dm[id].mode != MOTOR_DEBUG_DM_NONE) {
			out->dm[id] = ctx.state.dm[id];
			out->active = true;
		}
	}
	k_spin_unlock(&ctx.lock, key);

	return out->active;
}

bool motor_debug_get_m3508(uint8_t id, dji_m3508_motor_t *out)
{
	if (ctx.dji_bus == NULL) {
		return false;
	}

	return dji_m3508_get(ctx.dji_bus, id, out);
}

bool motor_debug_get_dm4340(uint8_t id, dm4340_feedback_t *out)
{
	if (ctx.dm_bus == NULL) {
		return false;
	}

	return dm4340_get(ctx.dm_bus, id, out);
}

const char *motor_debug_dm_mode_name(motor_debug_dm_mode_t mode)
{
	switch (mode) {
	case MOTOR_DEBUG_DM_POS_VEL:
		return "pos";
	case MOTOR_DEBUG_DM_VELOCITY:
		return "vel";
	case MOTOR_DEBUG_DM_MIT:
		return "mit";
	case MOTOR_DEBUG_DM_NONE:
	default:
		return "none";
	}
}
