#include "bmi088.h"

#include <errno.h>
#include <math.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(bmi088, LOG_LEVEL_INF);

#define BMI088_ACCEL_NODE DT_NODELABEL(bmi088_accel)
#define BMI088_GYRO_NODE DT_NODELABEL(bmi088_gyro)

#define BMI088_ACCEL_CHIP_ID 0x00
#define BMI088_ACCEL_DATA 0x12
#define BMI088_ACCEL_CONF 0x40
#define BMI088_ACCEL_RANGE 0x41
#define BMI088_ACCEL_PWR_CONF 0x7c
#define BMI088_ACCEL_PWR_CTRL 0x7d
#define BMI088_ACCEL_SOFTRESET 0x7e

#define BMI088_GYRO_CHIP_ID 0x00
#define BMI088_GYRO_DATA 0x02
#define BMI088_GYRO_RANGE 0x0f
#define BMI088_GYRO_BANDWIDTH 0x10
#define BMI088_GYRO_LPM1 0x11
#define BMI088_GYRO_SOFTRESET 0x14

#define BMI088_ACCEL_ID_VALUE 0x1e
#define BMI088_GYRO_ID_VALUE 0x0f

#define BMI088_ACCEL_LSB_PER_G 5460.0f
#define BMI088_GYRO_LSB_PER_DPS 16.384f

static int64_t now_us(void)
{
	return k_cyc_to_us_floor64(k_cycle_get_64());
}

static int spi_read_reg(const struct spi_dt_spec *spi, uint8_t reg,
			uint8_t *data, size_t len, bool accel)
{
	uint8_t tx[12] = { 0 };
	uint8_t rx[12] = { 0 };
	const size_t dummy = accel ? 1U : 0U;
	const size_t total = 1U + dummy + len;

	if (total > sizeof(tx)) {
		return -EINVAL;
	}

	tx[0] = reg | 0x80U;

	const struct spi_buf tx_buf = {
		.buf = tx,
		.len = total,
	};
	const struct spi_buf rx_buf = {
		.buf = rx,
		.len = total,
	};
	const struct spi_buf_set tx_set = {
		.buffers = &tx_buf,
		.count = 1,
	};
	const struct spi_buf_set rx_set = {
		.buffers = &rx_buf,
		.count = 1,
	};

	const int ret = spi_transceive_dt(spi, &tx_set, &rx_set);
	if (ret == 0) {
		memcpy(data, &rx[1U + dummy], len);
	}
	return ret;
}

static int spi_write_reg(const struct spi_dt_spec *spi, uint8_t reg,
			 uint8_t value)
{
	uint8_t tx[2] = { reg & 0x7fU, value };
	const struct spi_buf tx_buf = {
		.buf = tx,
		.len = sizeof(tx),
	};
	const struct spi_buf_set tx_set = {
		.buffers = &tx_buf,
		.count = 1,
	};

	return spi_write_dt(spi, &tx_set);
}

static int16_t le_i16(const uint8_t *data)
{
	return (int16_t)((uint16_t)data[1] << 8 | data[0]);
}

static int read_raw(bmi088_t *imu, bmi088_sample_t *sample)
{
	uint8_t raw[6];
	int ret;

	ret = spi_read_reg(&imu->accel_spi, BMI088_ACCEL_DATA, raw,
			   sizeof(raw), true);
	if (ret != 0) {
		return ret;
	}

	sample->ax_g = (float)le_i16(&raw[0]) / BMI088_ACCEL_LSB_PER_G;
	sample->ay_g = (float)le_i16(&raw[2]) / BMI088_ACCEL_LSB_PER_G;
	sample->az_g = (float)le_i16(&raw[4]) / BMI088_ACCEL_LSB_PER_G;

	ret = spi_read_reg(&imu->gyro_spi, BMI088_GYRO_DATA, raw,
			   sizeof(raw), false);
	if (ret != 0) {
		return ret;
	}

	sample->gx_dps = (float)le_i16(&raw[0]) / BMI088_GYRO_LSB_PER_DPS -
			 imu->gyro_offset_dps[0];
	sample->gy_dps = (float)le_i16(&raw[2]) / BMI088_GYRO_LSB_PER_DPS -
			 imu->gyro_offset_dps[1];
	sample->gz_dps = (float)le_i16(&raw[4]) / BMI088_GYRO_LSB_PER_DPS -
			 imu->gyro_offset_dps[2];

	return 0;
}

static void update_attitude(bmi088_t *imu, bmi088_sample_t *sample)
{
	const int64_t now = now_us();
	float dt_s = 0.001f;

	if (imu->last_update_us != 0) {
		dt_s = (float)(now - imu->last_update_us) * 1.0e-6f;
		dt_s = CLAMP(dt_s, 0.0002f, 0.02f);
	}
	imu->last_update_us = now;

	const float roll_acc = atan2f(sample->ay_g, sample->az_g) *
			       57.2957795f;
	const float pitch_acc = atan2f(-sample->ax_g,
				       sqrtf(sample->ay_g * sample->ay_g +
					     sample->az_g * sample->az_g)) *
				57.2957795f;

	const float alpha = 0.985f;
	imu->roll_deg = alpha * (imu->roll_deg + sample->gx_dps * dt_s) +
			(1.0f - alpha) * roll_acc;
	imu->pitch_deg = alpha * (imu->pitch_deg + sample->gy_dps * dt_s) +
			 (1.0f - alpha) * pitch_acc;
	imu->yaw_deg += sample->gz_dps * dt_s;

	while (imu->yaw_deg > 180.0f) {
		imu->yaw_deg -= 360.0f;
	}
	while (imu->yaw_deg < -180.0f) {
		imu->yaw_deg += 360.0f;
	}

	sample->roll_deg = imu->roll_deg;
	sample->pitch_deg = imu->pitch_deg;
	sample->yaw_deg = imu->yaw_deg;
}

int bmi088_init(bmi088_t *imu)
{
	if (imu == NULL) {
		return -EINVAL;
	}

	memset(imu, 0, sizeof(*imu));

	imu->accel_spi = (struct spi_dt_spec)SPI_DT_SPEC_GET(
		BMI088_ACCEL_NODE, SPI_OP_MODE_MASTER | SPI_WORD_SET(8), 0);
	imu->gyro_spi = (struct spi_dt_spec)SPI_DT_SPEC_GET(
		BMI088_GYRO_NODE, SPI_OP_MODE_MASTER | SPI_WORD_SET(8), 0);

	if (!spi_is_ready_dt(&imu->accel_spi) ||
	    !spi_is_ready_dt(&imu->gyro_spi)) {
		LOG_ERR("BMI088 SPI is not ready");
		return -ENODEV;
	}

	(void)spi_write_reg(&imu->accel_spi, BMI088_ACCEL_SOFTRESET, 0xb6);
	(void)spi_write_reg(&imu->gyro_spi, BMI088_GYRO_SOFTRESET, 0xb6);
	k_msleep(50);

	uint8_t id = 0;
	int ret = spi_read_reg(&imu->accel_spi, BMI088_ACCEL_CHIP_ID, &id, 1,
			       true);
	if (ret != 0) {
		return ret;
	}
	if (id != BMI088_ACCEL_ID_VALUE) {
		LOG_WRN("unexpected BMI088 accel chip id 0x%02x", id);
	}

	ret = spi_read_reg(&imu->gyro_spi, BMI088_GYRO_CHIP_ID, &id, 1,
			   false);
	if (ret != 0) {
		return ret;
	}
	if (id != BMI088_GYRO_ID_VALUE) {
		LOG_WRN("unexpected BMI088 gyro chip id 0x%02x", id);
	}

	/* Accel active, 6 g range. Gyro 2000 dps range. */
	(void)spi_write_reg(&imu->accel_spi, BMI088_ACCEL_PWR_CTRL, 0x04);
	k_msleep(5);
	(void)spi_write_reg(&imu->accel_spi, BMI088_ACCEL_PWR_CONF, 0x00);
	k_msleep(5);
	(void)spi_write_reg(&imu->accel_spi, BMI088_ACCEL_CONF, 0xa8);
	(void)spi_write_reg(&imu->accel_spi, BMI088_ACCEL_RANGE, 0x01);
	(void)spi_write_reg(&imu->gyro_spi, BMI088_GYRO_LPM1, 0x00);
	(void)spi_write_reg(&imu->gyro_spi, BMI088_GYRO_RANGE, 0x00);
	(void)spi_write_reg(&imu->gyro_spi, BMI088_GYRO_BANDWIDTH, 0x02);
	k_msleep(20);

	bmi088_sample_t sample;
	float gyro_sum[3] = { 0.0f, 0.0f, 0.0f };
	const int calibration_count = 300;

	for (int i = 0; i < calibration_count; i++) {
		memset(&sample, 0, sizeof(sample));
		ret = read_raw(imu, &sample);
		if (ret != 0) {
			return ret;
		}

		gyro_sum[0] += sample.gx_dps;
		gyro_sum[1] += sample.gy_dps;
		gyro_sum[2] += sample.gz_dps;
		k_msleep(2);
	}

	imu->gyro_offset_dps[0] = gyro_sum[0] / calibration_count;
	imu->gyro_offset_dps[1] = gyro_sum[1] / calibration_count;
	imu->gyro_offset_dps[2] = gyro_sum[2] / calibration_count;

	ret = read_raw(imu, &sample);
	if (ret != 0) {
		return ret;
	}

	imu->roll_deg = atan2f(sample.ay_g, sample.az_g) * 57.2957795f;
	imu->pitch_deg = atan2f(-sample.ax_g,
				sqrtf(sample.ay_g * sample.ay_g +
				      sample.az_g * sample.az_g)) *
			 57.2957795f;
	imu->yaw_deg = 0.0f;
	imu->last_update_us = now_us();
	imu->ready = true;

	LOG_INF("BMI088 ready, gyro offset %.3f %.3f %.3f dps",
		(double)imu->gyro_offset_dps[0],
		(double)imu->gyro_offset_dps[1],
		(double)imu->gyro_offset_dps[2]);
	return 0;
}

int bmi088_update(bmi088_t *imu, bmi088_sample_t *out)
{
	if (imu == NULL || out == NULL || !imu->ready) {
		return -EINVAL;
	}

	int ret = read_raw(imu, out);
	if (ret != 0) {
		return ret;
	}

	update_attitude(imu, out);
	return 0;
}

bool bmi088_is_ready(const bmi088_t *imu)
{
	return imu != NULL && imu->ready;
}
