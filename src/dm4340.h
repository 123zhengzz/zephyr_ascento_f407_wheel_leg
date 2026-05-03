#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>

#define DM4340_MAX_ID 15
#define DM4340_RX_LOG_SIZE 128

typedef struct {
	uint32_t id;
	uint8_t dlc;
	uint8_t flags;
	uint8_t data[8];
	uint32_t count;
} dm4340_rx_entry_t;

typedef struct {
	dm4340_rx_entry_t entries[DM4340_RX_LOG_SIZE];
	uint32_t head;
	uint32_t total;
} dm4340_rx_log_t;

typedef struct {
	bool valid;
	uint16_t can_id;
	uint8_t op;
	uint8_t rid;
	uint32_t raw_u32;
	float value_float;
	int64_t last_update_ms;
} dm4340_param_response_t;

typedef struct {
	uint8_t id;
	uint8_t error;
	bool online;
	int64_t last_update_ms;
	float position_rad;
	float velocity_rad_s;
	float torque_nm;
	uint8_t mos_temperature_c;
	uint8_t rotor_temperature_c;
} dm4340_feedback_t;

typedef struct {
	float p_min;
	float p_max;
	float v_min;
	float v_max;
	float kp_min;
	float kp_max;
	float kd_min;
	float kd_max;
	float t_min;
	float t_max;
} dm4340_mit_limit_t;

typedef struct {
	const struct device *can;
	uint16_t feedback_can_id;
	uint16_t right_feedback_can_id;
	bool has_right_feedback;
	struct k_spinlock lock;
	dm4340_feedback_t motor[DM4340_MAX_ID + 1];
	dm4340_param_response_t param[DM4340_MAX_ID + 1];
	dm4340_mit_limit_t limit;
	dm4340_rx_log_t rx_log;
} dm4340_bus_t;

void dm4340_dump_rx_log(const dm4340_bus_t *bus);
void dm4340_rx_led_toggle(void);
int dm4340_poll_rx_fifo(dm4340_bus_t *bus);

int dm4340_init(dm4340_bus_t *bus, const struct device *can,
		uint16_t feedback_can_id, uint16_t right_feedback_can_id);
int dm4340_enable(dm4340_bus_t *bus, uint8_t id);
int dm4340_disable(dm4340_bus_t *bus, uint8_t id);
int dm4340_save_zero(dm4340_bus_t *bus, uint8_t id);
int dm4340_request_param_read(dm4340_bus_t *bus, uint16_t can_id,
			      uint8_t rid);
int dm4340_send_pos_vel(dm4340_bus_t *bus, uint8_t id, float position_rad,
			float velocity_limit_rad_s);
int dm4340_send_velocity(dm4340_bus_t *bus, uint8_t id,
			 float velocity_rad_s);
int dm4340_send_mit(dm4340_bus_t *bus, uint8_t id, float position_rad,
		    float velocity_rad_s, float kp, float kd, float torque_nm);
bool dm4340_get(const dm4340_bus_t *bus, uint8_t id, dm4340_feedback_t *out);
bool dm4340_get_param_response(const dm4340_bus_t *bus, uint8_t id,
			       uint8_t rid, dm4340_param_response_t *out);
