#include "dji_m3508.h"

#include <errno.h>
#include <math.h>
#include <string.h>

#include <zephyr/drivers/can.h>
#include <zephyr/logging/log.h>

#include "app_config.h"

LOG_MODULE_REGISTER(dji_m3508, LOG_LEVEL_INF);

#define VESC_CAN_PACKET_SET_CURRENT 1U
#define VESC_CAN_PACKET_SET_RPM 3U
#define VESC_CAN_PACKET_STATUS 9U
#define VESC_CAN_PACKET_STATUS_4 16U
#define VESC_CAN_PACKET_STATUS_5 27U

static int16_t be_i16(const uint8_t *data)
{
	return (int16_t)((uint16_t)data[0] << 8 | data[1]);
}

static int32_t be_i32(const uint8_t *data)
{
	return (int32_t)((uint32_t)data[0] << 24 |
			 (uint32_t)data[1] << 16 |
			 (uint32_t)data[2] << 8 | data[3]);
}

static void put_be_i32(uint8_t *data, int32_t value)
{
	const uint32_t raw = (uint32_t)value;

	data[0] = (uint8_t)(raw >> 24);
	data[1] = (uint8_t)(raw >> 16);
	data[2] = (uint8_t)(raw >> 8);
	data[3] = (uint8_t)raw;
}

static uint32_t vesc_ext_id(uint8_t controller_id, uint32_t packet_id)
{
	return (packet_id << 8) | controller_id;
}

static float torque_coeff_for_id(uint8_t id)
{
	if (id == APP_WHEEL_LEFT_ID) {
		return APP_ASCENTO_LEFT_CURRENT_MA_TO_WHEEL_TORQUE_NM;
	}
	if (id == APP_WHEEL_RIGHT_ID) {
		return APP_ASCENTO_RIGHT_CURRENT_MA_TO_WHEEL_TORQUE_NM;
	}

	return APP_ASCENTO_CURRENT_MA_TO_WHEEL_TORQUE_NM;
}

static int wheel_forward_sign_for_id(uint8_t id)
{
	if (id == APP_WHEEL_LEFT_ID) {
		return APP_WHEEL_LEFT_FORWARD_CURRENT_SIGN;
	}
	if (id == APP_WHEEL_RIGHT_ID) {
		return APP_WHEEL_RIGHT_FORWARD_CURRENT_SIGN;
	}

	return 1;
}

static bool command_slot_active(const dji_m3508_bus_t *bus, size_t slot)
{
	return slot < bus->command_id_count && bus->command_id[slot] != 0U;
}

int dji_m3508_send_current(dji_m3508_bus_t *bus, uint8_t id,
			   int16_t current_ma)
{
	if (bus == NULL || bus->can == NULL || id == 0U) {
		return -EINVAL;
	}

	const float amps = (float)current_ma * APP_VESC_CURRENT_CMD_TO_AMP;
	const int32_t milliamps = (int32_t)lrintf(amps * 1000.0f);

	struct can_frame frame = {
		.flags = CAN_FRAME_IDE,
		.id = vesc_ext_id(id, VESC_CAN_PACKET_SET_CURRENT),
		.dlc = 4,
	};

	put_be_i32(&frame.data[0], milliamps);
	const int ret = can_send(bus->can, &frame, K_MSEC(2), NULL, NULL);
	if (ret == 0) {
		k_spinlock_key_t key = k_spin_lock(&bus->lock);
		bus->motor[id].command_current_ma = current_ma;
		bus->motor[id].last_command_ms = k_uptime_get();
		k_spin_unlock(&bus->lock, key);
	}

	return ret;
}

int dji_m3508_send_rpm(dji_m3508_bus_t *bus, uint8_t id, int32_t rpm)
{
	if (bus == NULL || bus->can == NULL || id == 0U) {
		return -EINVAL;
	}

	struct can_frame frame = {
		.flags = CAN_FRAME_IDE,
		.id = vesc_ext_id(id, VESC_CAN_PACKET_SET_RPM),
		.dlc = 8,
	};

	put_be_i32(&frame.data[0], rpm);
	return can_send(bus->can, &frame, K_MSEC(2), NULL, NULL);
}

static void parse_status_1(dji_m3508_bus_t *bus, uint8_t id,
			   const struct can_frame *frame)
{
	const int32_t erpm = be_i32(&frame->data[0]);
	const int16_t current_deciamp = be_i16(&frame->data[4]);
	const int16_t duty_permille = be_i16(&frame->data[6]);
	const int64_t now_ms = k_uptime_get();
	const float pole_pairs = APP_VESC_MOTOR_POLE_PAIRS > 0.0f ?
				 APP_VESC_MOTOR_POLE_PAIRS : 1.0f;
	const float motor_rpm = (float)erpm / pole_pairs;
	const float speed_rad_s = motor_rpm * 0.104719755f;

	dji_m3508_motor_t next = { 0 };
	next.id = id;
	next.present = true;
	next.online = true;
	next.erpm = erpm;
	next.speed_rpm = (int32_t)lrintf(motor_rpm);
	next.motor_current_ma = (int32_t)lrintf((float)current_deciamp * 100.0f);
	next.current_ma = next.motor_current_ma;
	next.current_ma_to_wheel_torque_nm = torque_coeff_for_id(id);
	next.estimated_wheel_torque_nm =
		(float)(next.motor_current_ma * wheel_forward_sign_for_id(id)) *
		next.current_ma_to_wheel_torque_nm;
	next.duty = (float)duty_permille / 1000.0f;
	next.last_update_ms = now_ms;
	next.speed_rad_s = speed_rad_s;

	k_spinlock_key_t key = k_spin_lock(&bus->lock);
	dji_m3508_motor_t *motor = &bus->motor[id];
	next.command_current_ma = motor->command_current_ma;
	next.last_command_ms = motor->last_command_ms;
	next.input_current_ma = motor->input_current_ma;
	next.input_voltage_mv = motor->input_voltage_mv;
	next.tachometer = motor->tachometer;
	next.fet_temperature_cdeg = motor->fet_temperature_cdeg;
	next.motor_temperature_cdeg = motor->motor_temperature_cdeg;
	next.last_status4_update_ms = motor->last_status4_update_ms;
	next.last_status5_update_ms = motor->last_status5_update_ms;

	if (motor->initialized && motor->last_update_ms > 0) {
		const float dt_s = (float)(now_ms - motor->last_update_ms) *
				   1.0e-3f;
		if (dt_s > 0.0f && dt_s < 0.5f) {
			next.angle_rad = motor->angle_rad +
					 speed_rad_s * dt_s;
		} else {
			next.angle_rad = motor->angle_rad;
		}
		next.initialized = true;
	} else {
		next.angle_rad = 0.0f;
		next.initialized = true;
	}

	*motor = next;
	k_spin_unlock(&bus->lock, key);
}

static void parse_status_4(dji_m3508_bus_t *bus, uint8_t id,
			   const struct can_frame *frame)
{
	const int16_t fet_temperature_cdeg = be_i16(&frame->data[0]);
	const int16_t motor_temperature_cdeg = be_i16(&frame->data[2]);
	const int16_t input_current_deciamp = be_i16(&frame->data[4]);
	const int64_t now_ms = k_uptime_get();

	k_spinlock_key_t key = k_spin_lock(&bus->lock);
	dji_m3508_motor_t *motor = &bus->motor[id];
	motor->id = id;
	motor->present = true;
	motor->fet_temperature_cdeg = fet_temperature_cdeg;
	motor->motor_temperature_cdeg = motor_temperature_cdeg;
	motor->input_current_ma =
		(int32_t)lrintf((float)input_current_deciamp * 100.0f);
	motor->last_status4_update_ms = now_ms;
	k_spin_unlock(&bus->lock, key);
}

static void parse_status_5(dji_m3508_bus_t *bus, uint8_t id,
			   const struct can_frame *frame)
{
	const int32_t tachometer_scaled = be_i32(&frame->data[0]);
	const int16_t input_voltage_decivolt = be_i16(&frame->data[4]);
	const int64_t now_ms = k_uptime_get();

	k_spinlock_key_t key = k_spin_lock(&bus->lock);
	dji_m3508_motor_t *motor = &bus->motor[id];
	motor->id = id;
	motor->present = true;
	motor->tachometer = tachometer_scaled / 6;
	motor->input_voltage_mv =
		(int32_t)input_voltage_decivolt * 100;
	motor->last_status5_update_ms = now_ms;
	k_spin_unlock(&bus->lock, key);
}

static void dji_m3508_rx_cb(const struct device *dev, struct can_frame *frame,
			    void *user_data)
{
	ARG_UNUSED(dev);

	dji_m3508_bus_t *bus = user_data;
	if ((frame->flags & CAN_FRAME_IDE) == 0 || frame->dlc < 8) {
		return;
	}

	const uint8_t packet_id = (uint8_t)((frame->id >> 8) & 0xffU);
	const uint8_t id = (uint8_t)(frame->id & 0xffU);

	switch (packet_id) {
	case VESC_CAN_PACKET_STATUS:
		parse_status_1(bus, id, frame);
		break;
	case VESC_CAN_PACKET_STATUS_4:
		parse_status_4(bus, id, frame);
		break;
	case VESC_CAN_PACKET_STATUS_5:
		parse_status_5(bus, id, frame);
		break;
	default:
		break;
	}
}

static int add_status_filter(const struct device *can, dji_m3508_bus_t *bus,
			     uint32_t packet_id)
{
	const struct can_filter filter = {
		.flags = CAN_FILTER_IDE,
		.id = packet_id << 8,
		.mask = CAN_EXT_ID_MASK & ~0xffU,
	};

	const int filter_id = can_add_rx_filter(can, dji_m3508_rx_cb, bus,
						&filter);
	if (filter_id < 0) {
		LOG_ERR("failed to add VESC status %u filter: %d",
			(unsigned int)packet_id, filter_id);
		return filter_id;
	}

	return 0;
}

int dji_m3508_init(dji_m3508_bus_t *bus, const struct device *can,
		   const uint8_t *ids, size_t id_count)
{
	if (bus == NULL || can == NULL || ids == NULL) {
		return -EINVAL;
	}

	memset(bus, 0, sizeof(*bus));
	bus->can = can;

	for (size_t i = 0; i < id_count; i++) {
		const uint8_t id = ids[i];

		if (id == 0) {
			LOG_ERR("invalid VESC ID %u", id);
			return -EINVAL;
		}

		bus->motor[id].id = id;
		bus->motor[id].present = true;
		if (bus->command_id_count < DJI_M3508_MAX_COMMAND_MOTORS) {
			bus->command_id[bus->command_id_count++] = id;
		}
	}

	int ret = add_status_filter(can, bus, VESC_CAN_PACKET_STATUS);
	if (ret != 0) {
		return ret;
	}

	ret = add_status_filter(can, bus, VESC_CAN_PACKET_STATUS_4);
	if (ret != 0) {
		return ret;
	}

	ret = add_status_filter(can, bus, VESC_CAN_PACKET_STATUS_5);
	if (ret != 0) {
		return ret;
	}

	LOG_INF("VESC wheel status filters ready, %u motor(s)",
		(unsigned int)bus->command_id_count);
	return 0;
}

int dji_m3508_send_group_current(dji_m3508_bus_t *bus, int16_t id1,
				 int16_t id2, int16_t id3, int16_t id4)
{
	if (bus == NULL || bus->can == NULL) {
		return -EINVAL;
	}

	const int16_t current[DJI_M3508_MAX_COMMAND_MOTORS] = {
		id1,
		id2,
		id3,
		id4,
	};
	int first_error = 0;

	for (size_t i = 0; i < DJI_M3508_MAX_COMMAND_MOTORS; i++) {
		if (command_slot_active(bus, i)) {
			const int ret = dji_m3508_send_current(
				bus, bus->command_id[i], current[i]);
			if (ret != 0 && first_error == 0) {
				first_error = ret;
			}
		}
	}

	return first_error;
}

bool dji_m3508_get(const dji_m3508_bus_t *bus, uint8_t id,
		   dji_m3508_motor_t *out)
{
	if (bus == NULL || out == NULL || id == 0) {
		return false;
	}

	k_spinlock_key_t key = k_spin_lock((struct k_spinlock *)&bus->lock);
	const bool ok = bus->motor[id].present && bus->motor[id].initialized;
	if (ok) {
		*out = bus->motor[id];
	}
	k_spin_unlock((struct k_spinlock *)&bus->lock, key);
	return ok;
}

bool dji_m3508_is_online(const dji_m3508_bus_t *bus, uint8_t id,
			 int64_t now_ms, int64_t timeout_ms)
{
	dji_m3508_motor_t motor;

	if (!dji_m3508_get(bus, id, &motor)) {
		return false;
	}

	return motor.online && (now_ms - motor.last_update_ms) <= timeout_ms;
}
