#include "dm4340.h"

#include <errno.h>
#include <math.h>
#include <string.h>

#include <zephyr/drivers/can.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

LOG_MODULE_REGISTER(dm4340, LOG_LEVEL_INF);

#ifndef CAN_STD_ID_MASK
#define CAN_STD_ID_MASK 0x7ff
#endif

static uint16_t float_to_uint(float value, float min_value, float max_value,
			      uint8_t bits)
{
	const float span = max_value - min_value;
	const uint32_t max_int = (1U << bits) - 1U;

	if (span <= 0.0f) {
		return 0;
	}

	if (value < min_value) {
		value = min_value;
	} else if (value > max_value) {
		value = max_value;
	}

	const float offset = value - min_value;
	return (uint16_t)((offset * (float)max_int / span) + 0.5f);
}

static float uint_to_float(uint16_t value, float min_value, float max_value,
			   uint8_t bits)
{
	const float span = max_value - min_value;
	const float max_int = (float)((1U << bits) - 1U);

	return ((float)value) * span / max_int + min_value;
}

static void put_le_float(uint8_t *data, float value)
{
	uint32_t raw;

	memcpy(&raw, &value, sizeof(raw));
	sys_put_le32(raw, data);
}

static int dm4340_send_special(dm4340_bus_t *bus, uint8_t id, uint8_t cmd)
{
	if (bus == NULL || bus->can == NULL || id == 0 || id > DM4340_MAX_ID) {
		return -EINVAL;
	}

	struct can_frame frame = {
		.flags = 0,
		.id = id,
		.dlc = 8,
		.data = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, cmd },
	};

	return can_send(bus->can, &frame, K_MSEC(2), NULL, NULL);
}

static void dm4340_rx_cb(const struct device *dev, struct can_frame *frame,
			 void *user_data)
{
	ARG_UNUSED(dev);

	dm4340_bus_t *bus = user_data;

	if (frame->dlc < 8) {
		return;
	}

	const uint8_t id = frame->data[0] & 0x0f;
	if (id == 0 || id > DM4340_MAX_ID) {
		return;
	}

	const uint16_t p_int = ((uint16_t)frame->data[1] << 8) |
			       frame->data[2];
	const uint16_t v_int = ((uint16_t)frame->data[3] << 4) |
			       (frame->data[4] >> 4);
	const uint16_t t_int = ((uint16_t)(frame->data[4] & 0x0f) << 8) |
			       frame->data[5];

	dm4340_feedback_t fb = {
		.id = id,
		.error = frame->data[0] >> 4,
		.online = true,
		.last_update_ms = k_uptime_get(),
		.position_rad = uint_to_float(p_int, bus->limit.p_min,
					      bus->limit.p_max, 16),
		.velocity_rad_s = uint_to_float(v_int, bus->limit.v_min,
						bus->limit.v_max, 12),
		.torque_nm = uint_to_float(t_int, bus->limit.t_min,
					   bus->limit.t_max, 12),
		.mos_temperature_c = frame->data[6],
		.rotor_temperature_c = frame->data[7],
	};

	k_spinlock_key_t key = k_spin_lock(&bus->lock);
	bus->motor[id] = fb;
	k_spin_unlock(&bus->lock, key);
}

int dm4340_init(dm4340_bus_t *bus, const struct device *can,
		uint16_t feedback_can_id)
{
	if (bus == NULL || can == NULL) {
		return -EINVAL;
	}

	memset(bus, 0, sizeof(*bus));
	bus->can = can;
	bus->feedback_can_id = feedback_can_id;
	bus->limit = (dm4340_mit_limit_t) {
		.p_min = -12.5f,
		.p_max = 12.5f,
		.v_min = -45.0f,
		.v_max = 45.0f,
		.kp_min = 0.0f,
		.kp_max = 500.0f,
		.kd_min = 0.0f,
		.kd_max = 5.0f,
		.t_min = -18.0f,
		.t_max = 18.0f,
	};

	const struct can_filter filter = {
		.flags = 0,
		.id = feedback_can_id,
		.mask = CAN_STD_ID_MASK,
	};

	const int filter_id = can_add_rx_filter(can, dm4340_rx_cb, bus,
						&filter);
	if (filter_id < 0) {
		LOG_ERR("failed to add DM4340 feedback filter 0x%03x: %d",
			feedback_can_id, filter_id);
		return filter_id;
	}

	LOG_INF("DM4340 feedback filter ready on 0x%03x", feedback_can_id);
	return 0;
}

int dm4340_enable(dm4340_bus_t *bus, uint8_t id)
{
	return dm4340_send_special(bus, id, 0xfc);
}

int dm4340_disable(dm4340_bus_t *bus, uint8_t id)
{
	return dm4340_send_special(bus, id, 0xfd);
}

int dm4340_save_zero(dm4340_bus_t *bus, uint8_t id)
{
	return dm4340_send_special(bus, id, 0xfe);
}

int dm4340_send_pos_vel(dm4340_bus_t *bus, uint8_t id, float position_rad,
			float velocity_limit_rad_s)
{
	if (bus == NULL || bus->can == NULL || id == 0 || id > DM4340_MAX_ID) {
		return -EINVAL;
	}

	struct can_frame frame = {
		.flags = 0,
		.id = 0x100U + id,
		.dlc = 8,
	};

	put_le_float(&frame.data[0], position_rad);
	put_le_float(&frame.data[4], velocity_limit_rad_s);

	return can_send(bus->can, &frame, K_MSEC(2), NULL, NULL);
}

int dm4340_send_velocity(dm4340_bus_t *bus, uint8_t id,
			 float velocity_rad_s)
{
	if (bus == NULL || bus->can == NULL || id == 0 || id > DM4340_MAX_ID) {
		return -EINVAL;
	}

	struct can_frame frame = {
		.flags = 0,
		.id = 0x200U + id,
		.dlc = 4,
	};

	put_le_float(&frame.data[0], velocity_rad_s);
	return can_send(bus->can, &frame, K_MSEC(2), NULL, NULL);
}

int dm4340_send_mit(dm4340_bus_t *bus, uint8_t id, float position_rad,
		    float velocity_rad_s, float kp, float kd, float torque_nm)
{
	if (bus == NULL || bus->can == NULL || id == 0 || id > DM4340_MAX_ID) {
		return -EINVAL;
	}

	const dm4340_mit_limit_t *lim = &bus->limit;
	const uint16_t p = float_to_uint(position_rad, lim->p_min, lim->p_max,
					 16);
	const uint16_t v = float_to_uint(velocity_rad_s, lim->v_min,
					 lim->v_max, 12);
	const uint16_t kp_u = float_to_uint(kp, lim->kp_min, lim->kp_max, 12);
	const uint16_t kd_u = float_to_uint(kd, lim->kd_min, lim->kd_max, 12);
	const uint16_t t = float_to_uint(torque_nm, lim->t_min, lim->t_max,
					 12);

	struct can_frame frame = {
		.flags = 0,
		.id = id,
		.dlc = 8,
	};

	frame.data[0] = p >> 8;
	frame.data[1] = p & 0xff;
	frame.data[2] = v >> 4;
	frame.data[3] = ((v & 0x0f) << 4) | (kp_u >> 8);
	frame.data[4] = kp_u & 0xff;
	frame.data[5] = kd_u >> 4;
	frame.data[6] = ((kd_u & 0x0f) << 4) | (t >> 8);
	frame.data[7] = t & 0xff;

	return can_send(bus->can, &frame, K_MSEC(2), NULL, NULL);
}

bool dm4340_get(const dm4340_bus_t *bus, uint8_t id, dm4340_feedback_t *out)
{
	if (bus == NULL || out == NULL || id == 0 || id > DM4340_MAX_ID) {
		return false;
	}

	k_spinlock_key_t key = k_spin_lock((struct k_spinlock *)&bus->lock);
	const bool ok = bus->motor[id].online;
	if (ok) {
		*out = bus->motor[id];
	}
	k_spin_unlock((struct k_spinlock *)&bus->lock, key);
	return ok;
}
