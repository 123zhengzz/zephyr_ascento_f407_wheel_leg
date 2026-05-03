#include "dm4340.h"

#include <errno.h>
#include <math.h>
#include <string.h>

#include <zephyr/drivers/can.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>

LOG_MODULE_REGISTER(dm4340, LOG_LEVEL_INF);

static bool can_ok(const struct device *can)
{
	enum can_state state;
	struct can_bus_err_cnt err_cnt;

	if (can_get_state(can, &state, &err_cnt) != 0) {
		return false;
	}

	return state != CAN_STATE_BUS_OFF && state != CAN_STATE_STOPPED;
}

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

static uint8_t feedback_motor_id(const dm4340_bus_t *bus,
				 const struct can_frame *frame)
{
	if ((frame->flags & CAN_FRAME_IDE) != 0) {
		return 0;
	}

	if (frame->id != bus->feedback_can_id &&
	    (!bus->has_right_feedback ||
	     frame->id != bus->right_feedback_can_id)) {
		return 0;
	}

	const uint8_t id = frame->id & 0x0f;
	if (id == 0 || id > DM4340_MAX_ID) {
		return 0;
	}

	return id;
}

static bool parse_param_response(dm4340_bus_t *bus,
				 const struct can_frame *frame)
{
	if ((frame->flags & CAN_FRAME_IDE) != 0 || frame->dlc < 4) {
		return false;
	}

	const uint8_t op = frame->data[2];
	if (op != 0x33 && op != 0x55 && op != 0xaa) {
		return false;
	}

	const uint16_t can_id = ((uint16_t)frame->data[1] << 8) |
				frame->data[0];
	const uint8_t id = can_id & 0xff;
	if (id == 0 || id > DM4340_MAX_ID) {
		return true;
	}

	uint32_t raw = 0;
	float value = 0.0f;
	if (frame->dlc >= 8) {
		raw = sys_get_le32(&frame->data[4]);
		memcpy(&value, &raw, sizeof(value));
	}

	const dm4340_param_response_t response = {
		.valid = true,
		.can_id = can_id,
		.op = op,
		.rid = frame->data[3],
		.raw_u32 = raw,
		.value_float = value,
		.last_update_ms = k_uptime_get(),
	};

	k_spinlock_key_t key = k_spin_lock(&bus->lock);
	bus->param[id] = response;
	k_spin_unlock(&bus->lock, key);

	return true;
}

static int dm4340_send_special(dm4340_bus_t *bus, uint8_t id, uint8_t cmd)
{
	if (bus == NULL || bus->can == NULL || id == 0 ||
	    id > DM4340_MAX_ID) {
		return -EINVAL;
	}

	if (!can_ok(bus->can)) {
		return -ENETDOWN;
	}

	struct can_frame frame = {
		.flags = 0,
		.id = id,
		.dlc = 8,
		.data = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, cmd },
	};

	return can_send(bus->can, &frame, K_MSEC(2), NULL, NULL);
}

volatile uint32_t dm4340_rx_cb_count;

static void dm4340_rx_cb(const struct device *dev, struct can_frame *frame,
			 void *user_data)
{
	ARG_UNUSED(dev);

	dm4340_bus_t *bus = user_data;

	dm4340_rx_cb_count++;

	/* Toggle LED on every received CAN frame for debugging */
	extern void dm4340_rx_led_toggle(void);
	dm4340_rx_led_toggle();

	/* Log every CAN frame into ring buffer */
	uint32_t idx = bus->rx_log.head % DM4340_RX_LOG_SIZE;
	dm4340_rx_entry_t *entry = &bus->rx_log.entries[idx];

	if (entry->id == frame->id && entry->dlc == frame->dlc &&
	    memcmp(entry->data, frame->data, frame->dlc) == 0) {
		entry->count++;
	} else {
		bus->rx_log.head++;
		bus->rx_log.total++;
		idx = bus->rx_log.head % DM4340_RX_LOG_SIZE;
		bus->rx_log.entries[idx] = (dm4340_rx_entry_t){
			.id = frame->id,
			.dlc = frame->dlc,
			.flags = frame->flags,
			.count = 1,
		};
		memcpy(bus->rx_log.entries[idx].data, frame->data,
		       MIN(frame->dlc, 8));
	}

	/* Parse configured DM4340 feedback frames. */
	if (frame->dlc < 8) {
		return;
	}

	if (parse_param_response(bus, frame)) {
		return;
	}

	const uint8_t id = feedback_motor_id(bus, frame);
	if (id == 0) {
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

void dm4340_dump_rx_log(const dm4340_bus_t *bus)
{
	if (bus == NULL) {
		return;
	}

	printk("=== DM4340 CAN RX log (total=%u) ===\n", bus->rx_log.total);

	uint32_t start = bus->rx_log.head > DM4340_RX_LOG_SIZE
				 ? bus->rx_log.head - DM4340_RX_LOG_SIZE + 1
				 : 0;
	uint32_t count = MIN(bus->rx_log.head + 1, DM4340_RX_LOG_SIZE);
	if (bus->rx_log.head >= DM4340_RX_LOG_SIZE) {
		start = bus->rx_log.head - DM4340_RX_LOG_SIZE + 1;
		count = DM4340_RX_LOG_SIZE;
	} else {
		start = 0;
		count = bus->rx_log.head + 1;
	}

	for (uint32_t i = 0; i < count; i++) {
		uint32_t idx = (start + i) % DM4340_RX_LOG_SIZE;
		const dm4340_rx_entry_t *e = &bus->rx_log.entries[idx];
		if (e->count == 0) {
			continue;
		}
		printk("  id=0x%03x dlc=%u ext=%d cnt=%u data=%02x%02x%02x%02x%02x%02x%02x%02x\n",
		       e->id, e->dlc,
		       (e->flags & CAN_FRAME_IDE) ? 1 : 0,
		       e->count,
		       e->data[0], e->data[1], e->data[2], e->data[3],
		       e->data[4], e->data[5], e->data[6], e->data[7]);
	}
}

int dm4340_init(dm4340_bus_t *bus, const struct device *can,
		uint16_t feedback_can_id, uint16_t right_feedback_can_id)
{
	if (bus == NULL || can == NULL) {
		return -EINVAL;
	}

	memset(bus, 0, sizeof(*bus));
	bus->can = can;
	bus->feedback_can_id = feedback_can_id;
	bus->right_feedback_can_id = right_feedback_can_id;
	bus->has_right_feedback = (right_feedback_can_id != feedback_can_id);
	bus->limit = (dm4340_mit_limit_t) {
		.p_min = -12.5f,
		.p_max = 12.5f,
		.v_min = -10.0f,
		.v_max = 10.0f,
		.kp_min = 0.0f,
		.kp_max = 500.0f,
		.kd_min = 0.0f,
		.kd_max = 5.0f,
		.t_min = -28.0f,
		.t_max = 28.0f,
	};

	/* Accept all standard frames */
	const struct can_filter filter_std = {
		.flags = 0,
		.id = 0x000,
		.mask = 0x000,
	};

	int filter_id = can_add_rx_filter(can, dm4340_rx_cb, bus, &filter_std);
	if (filter_id < 0) {
		LOG_ERR("failed to add DM4340 std filter: %d", filter_id);
		return filter_id;
	}

	LOG_INF("DM4340 std filter ready (accept-all)");

	/* Also accept all extended frames in case the motor uses 29-bit IDs */
	const struct can_filter filter_ext = {
		.flags = CAN_FILTER_IDE,
		.id = 0x00000000,
		.mask = 0x00000000,
	};

	filter_id = can_add_rx_filter(can, dm4340_rx_cb, bus, &filter_ext);
	if (filter_id < 0) {
		LOG_ERR("failed to add DM4340 ext filter: %d", filter_id);
		/* Not fatal — left motor works on std frames */
	}

	LOG_INF("DM4340 ext filter ready (accept-all)");

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

int dm4340_request_param_read(dm4340_bus_t *bus, uint16_t can_id,
			      uint8_t rid)
{
	if (bus == NULL || bus->can == NULL || can_id == 0 ||
	    can_id > CAN_STD_ID_MASK) {
		return -EINVAL;
	}

	if (!can_ok(bus->can)) {
		return -ENETDOWN;
	}

	struct can_frame frame = {
		.flags = 0,
		.id = 0x7ff,
		.dlc = 4,
		.data = { can_id & 0xff, can_id >> 8, 0x33, rid },
	};

	return can_send(bus->can, &frame, K_MSEC(5), NULL, NULL);
}

int dm4340_send_pos_vel(dm4340_bus_t *bus, uint8_t id, float position_rad,
			float velocity_limit_rad_s)
{
	if (bus == NULL || bus->can == NULL || id == 0 || id > DM4340_MAX_ID) {
		return -EINVAL;
	}

	if (!can_ok(bus->can)) {
		return -ENETDOWN;
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

	if (!can_ok(bus->can)) {
		return -ENETDOWN;
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

	if (!can_ok(bus->can)) {
		return -ENETDOWN;
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

/* Direct CAN FIFO polling — workaround for non-firing CAN RX interrupt.
 * CAN1 base: 0x40006400.  Reads FIFO 0 if a frame is pending and feeds it to
 * the standard rx callback.  Returns the number of frames processed.
 */
#ifndef CAN1_BASE
#define CAN1_BASE 0x40006400UL
#endif

#ifndef CAN_RF0R_FMP0_Msk
#define CAN_RF0R_FMP0_Msk 0x03
#endif

#include <zephyr/sys/util_macro.h>
#include <zephyr/spinlock.h>

int dm4340_poll_rx_fifo(dm4340_bus_t *bus)
{
	if (bus == NULL || bus->can == NULL) {
		return 0;
	}

	volatile uint32_t *rf0r = (volatile uint32_t *)(CAN1_BASE + 0x0C);
	volatile uint32_t *mbox_id = (volatile uint32_t *)(CAN1_BASE + 0x1C);
	volatile uint32_t *mbox_dt = (volatile uint32_t *)(CAN1_BASE + 0x20);
	volatile uint32_t *mbox_dl = (volatile uint32_t *)(CAN1_BASE + 0x24);
	volatile uint32_t *mbox_dh = (volatile uint32_t *)(CAN1_BASE + 0x28);
	int processed = 0;

	while (*rf0r & CAN_RF0R_FMP0_Msk) {
		struct can_frame frame = { .flags = 0 };
		uint32_t rir = *mbox_id;
		uint32_t rdtr = *mbox_dt;

		if (rir & (1U << 2)) {
			frame.flags |= CAN_FRAME_IDE;
			frame.id = (rir >> 3) & CAN_EXT_ID_MASK;
		} else {
			frame.id = (rir >> 21) & CAN_STD_ID_MASK;
		}

		if (rir & (1U << 1)) {
			frame.flags |= CAN_FRAME_RTR;
		}

		frame.dlc = rdtr & 0x0F;
		uint32_t dl = *mbox_dl;
		uint32_t dh = *mbox_dh;

		frame.data[0] = (dl >> 0) & 0xFF;
		frame.data[1] = (dl >> 8) & 0xFF;
		frame.data[2] = (dl >> 16) & 0xFF;
		frame.data[3] = (dl >> 24) & 0xFF;
		frame.data[4] = (dh >> 0) & 0xFF;
		frame.data[5] = (dh >> 8) & 0xFF;
		frame.data[6] = (dh >> 16) & 0xFF;
		frame.data[7] = (dh >> 24) & 0xFF;

		/* Call the rx callback directly */
		dm4340_rx_cb(bus->can, &frame, bus);

		/* Release FIFO */
		*rf0r = *rf0r | (1U << 5);
		processed++;
	}

	return processed;
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

bool dm4340_get_param_response(const dm4340_bus_t *bus, uint8_t id,
			       uint8_t rid, dm4340_param_response_t *out)
{
	if (bus == NULL || out == NULL || id == 0 || id > DM4340_MAX_ID) {
		return false;
	}

	k_spinlock_key_t key = k_spin_lock((struct k_spinlock *)&bus->lock);
	const dm4340_param_response_t response = bus->param[id];
	const bool ok = response.valid && response.rid == rid;
	if (ok) {
		*out = response;
	}
	k_spin_unlock((struct k_spinlock *)&bus->lock, key);
	return ok;
}
