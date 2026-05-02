#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>

#define DJI_M3508_MAX_ID 255
#define DJI_M3508_MAX_COMMAND_MOTORS 4
#define DJI_M3508_ENCODER_CPR 8192.0f

typedef struct {
	uint8_t id;
	bool present;
	bool online;
	bool initialized;
	uint16_t encoder_raw;
	uint16_t last_encoder_raw;
	int32_t round_count;
	int32_t speed_rpm;
	int32_t erpm;
	int32_t current_ma;
	uint8_t temperature_c;
	int64_t last_update_ms;
	float angle_rad;
	float speed_rad_s;
	float duty;
} dji_m3508_motor_t;

typedef struct {
	const struct device *can;
	struct k_spinlock lock;
	dji_m3508_motor_t motor[DJI_M3508_MAX_ID + 1];
	uint8_t command_id[DJI_M3508_MAX_COMMAND_MOTORS];
	size_t command_id_count;
} dji_m3508_bus_t;

int dji_m3508_init(dji_m3508_bus_t *bus, const struct device *can,
		   const uint8_t *ids, size_t id_count);
int dji_m3508_send_current(dji_m3508_bus_t *bus, uint8_t id,
			   int16_t current_ma);
int dji_m3508_send_rpm(dji_m3508_bus_t *bus, uint8_t id, int32_t rpm);
int dji_m3508_send_group_current(dji_m3508_bus_t *bus, int16_t id1,
				 int16_t id2, int16_t id3, int16_t id4);
bool dji_m3508_get(const dji_m3508_bus_t *bus, uint8_t id,
		   dji_m3508_motor_t *out);
bool dji_m3508_is_online(const dji_m3508_bus_t *bus, uint8_t id,
			 int64_t now_ms, int64_t timeout_ms);
