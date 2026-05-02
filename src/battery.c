#include "battery.h"

#include "app_config.h"

#include <errno.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(battery, LOG_LEVEL_INF);

#define BATTERY_ADC_NODE DT_NODELABEL(adc3)

static const struct device *adc_dev = DEVICE_DT_GET(BATTERY_ADC_NODE);
static int16_t adc_sample;

int battery_init(void)
{
	if (!device_is_ready(adc_dev)) {
		LOG_WRN("ADC3 is not ready; battery reading disabled");
		return -ENODEV;
	}

	const struct adc_channel_cfg channel_cfg = {
		.gain = ADC_GAIN_1,
		.reference = ADC_REF_INTERNAL,
		.acquisition_time = ADC_ACQ_TIME_DEFAULT,
		.channel_id = APP_BATTERY_ADC_CHANNEL,
	};

	const int ret = adc_channel_setup(adc_dev, &channel_cfg);
	if (ret != 0) {
		LOG_WRN("ADC channel setup failed: %d", ret);
		return ret;
	}

	LOG_INF("battery ADC ready");
	return 0;
}

battery_sample_t battery_read(void)
{
	battery_sample_t sample = {
		.valid = false,
		.voltage_v = 0.0f,
	};

	if (!device_is_ready(adc_dev)) {
		return sample;
	}

	struct adc_sequence sequence = {
		.channels = BIT(APP_BATTERY_ADC_CHANNEL),
		.buffer = &adc_sample,
		.buffer_size = sizeof(adc_sample),
		.resolution = 12,
	};

	const int ret = adc_read(adc_dev, &sequence);
	if (ret != 0) {
		return sample;
	}

	const float adc_mv = ((float)adc_sample * APP_BATTERY_ADC_FULL_SCALE_MV) /
			     4095.0f;
	sample.voltage_v = adc_mv * APP_BATTERY_DIVIDER_RATIO / 1000.0f;
	sample.valid = true;
	return sample;
}
