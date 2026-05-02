#pragma once

#include <stdbool.h>

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>

typedef struct {
	float ax_g;
	float ay_g;
	float az_g;
	float gx_dps;
	float gy_dps;
	float gz_dps;
	float roll_deg;
	float pitch_deg;
	float yaw_deg;
} bmi088_sample_t;

typedef struct {
	struct spi_dt_spec accel_spi;
	struct spi_dt_spec gyro_spi;
	float gyro_offset_dps[3];
	float roll_deg;
	float pitch_deg;
	float yaw_deg;
	int64_t last_update_us;
	bool ready;
} bmi088_t;

int bmi088_init(bmi088_t *imu);
int bmi088_update(bmi088_t *imu, bmi088_sample_t *out);
bool bmi088_is_ready(const bmi088_t *imu);
