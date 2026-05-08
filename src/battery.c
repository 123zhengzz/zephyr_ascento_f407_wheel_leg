/**
 * @file battery.c
 * @brief 电池电压 ADC 采样模块
 *
 * 本文件通过 STM32F407 的 ADC3 外设读取电池电压。
 * 电池电压经过外部电阻分压后接入 ADC 输入引脚，本模块负责：
 *   1. 初始化 ADC 通道（配置增益、参考电压、采样时间等）
 *   2. 执行单次 ADC 转换，获取原始采样值
 *   3. 将原始 ADC 值转换为实际电压值（考虑分压比）
 *
 * 读取到的电池电压用于主循环中的状态指示（LED 显示）和日志输出，
 * 以便操作人员实时监控电池电量，防止电池过放。
 */

#include "battery.h"

#include "app_config.h"

#include <errno.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

/* 注册电池模块的日志标签为 "battery"，默认日志级别为 INFO */
LOG_MODULE_REGISTER(battery, LOG_LEVEL_INF);

/* 从设备树中获取 ADC3 外设节点，用于电池电压采集 */
#define BATTERY_ADC_NODE DT_NODELABEL(adc3)

/* ADC3 设备句柄，通过设备树宏在编译时绑定 */
static const struct device *adc_dev = DEVICE_DT_GET(BATTERY_ADC_NODE);

/* ADC 采样结果缓冲区，存放 12 位 ADC 原始值（有符号 16 位整数） */
static int16_t adc_sample;

/**
 * @brief 初始化电池 ADC 采集模块
 *
 * 检查 ADC3 设备是否就绪，然后配置 ADC 通道参数：
 *   - 增益：1 倍（无放大）
 *   - 参考电压：内部参考电压
 *   - 采样时间：默认值
 *   - 通道号：由 app_config.h 中 APP_BATTERY_ADC_CHANNEL 指定
 *
 * @return 0 表示初始化成功；-ENODEV 表示 ADC 设备未就绪；其他负值表示通道配置失败
 */
int battery_init(void)
{
	/* 检查 ADC3 设备是否已就绪（设备树中是否正确配置并初始化） */
	if (!device_is_ready(adc_dev)) {
		LOG_WRN("ADC3 is not ready; battery reading disabled");
		return -ENODEV;
	}

	/* 配置 ADC 通道参数 */
	const struct adc_channel_cfg channel_cfg = {
		.gain = ADC_GAIN_1,              /* 增益为 1，不进行信号放大 */
		.reference = ADC_REF_INTERNAL,    /* 使用 MCU 内部参考电压 */
		.acquisition_time = ADC_ACQ_TIME_DEFAULT, /* 采样保持时间使用默认值 */
		.channel_id = APP_BATTERY_ADC_CHANNEL,    /* ADC 通道编号，由配置文件定义 */
	};

	/* 调用 Zephyr ADC API 设置通道，返回 0 表示成功 */
	const int ret = adc_channel_setup(adc_dev, &channel_cfg);
	if (ret != 0) {
		LOG_WRN("ADC channel setup failed: %d", ret);
		return ret;
	}

	LOG_INF("battery ADC ready");
	return 0;
}

/**
 * @brief 执行一次电池电压 ADC 采样并返回电压值
 *
 * 执行流程：
 *   1. 检查 ADC 设备是否就绪
 *   2. 配置 ADC 转换序列（通道、缓冲区、分辨率）
 *   3. 执行 ADC 读取，获取 12 位原始采样值
 *   4. 将原始值转换为毫伏值：adc_mv = raw * full_scale_mv / 4095
 *   5. 再根据外部电阻分压比转换为实际电池电压：voltage = adc_mv * divider_ratio / 1000
 *
 * @return battery_sample_t 结构体，包含：
 *   - valid: true 表示采样成功，false 表示采样失败
 *   - voltage_v: 实际电池电压（单位：伏特），失败时为 0.0f
 */
battery_sample_t battery_read(void)
{
	/* 默认返回无效采样结果（valid=false, voltage=0V） */
	battery_sample_t sample = {
		.valid = false,
		.voltage_v = 0.0f,
	};

	/* 如果 ADC 设备未就绪，直接返回无效结果 */
	if (!device_is_ready(adc_dev)) {
		return sample;
	}

	/* 配置 ADC 转换序列 */
	struct adc_sequence sequence = {
		.channels = BIT(APP_BATTERY_ADC_CHANNEL),  /* 指定要采集的通道（位掩码） */
		.buffer = &adc_sample,            /* 采样结果存放的缓冲区指针 */
		.buffer_size = sizeof(adc_sample), /* 缓冲区大小（2 字节，对应 int16_t） */
		.resolution = 12,                 /* ADC 分辨率为 12 位（0~4095） */
	};

	/* 执行 ADC 单次转换读取 */
	const int ret = adc_read(adc_dev, &sequence);
	if (ret != 0) {
		return sample;  /* 读取失败，返回无效结果 */
	}

	/*
	 * ADC 原始值到实际电压的转换：
	 *   1. 先将 12 位 ADC 值（0~4095）映射到 0~FULL_SCALE_MV 毫伏范围
	 *   2. 再乘以分压比（divider_ratio），还原电池端实际电压
	 *   3. 最后除以 1000 将毫伏转换为伏特
	 *
	 * 例如：若 FULL_SCALE_MV=3300, DIVIDER_RATIO=11.0, adc_sample=2048
	 *   adc_mv = 2048 * 3300 / 4095 = 1651.5 mV
	 *   voltage = 1651.5 * 11.0 / 1000 = 18.17 V
	 */
	const float adc_mv = ((float)adc_sample * APP_BATTERY_ADC_FULL_SCALE_MV) /
			     4095.0f;
	sample.voltage_v = adc_mv * APP_BATTERY_DIVIDER_RATIO / 1000.0f;
	sample.valid = true;  /* 标记采样成功 */
	return sample;
}
