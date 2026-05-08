/**
 * @file battery.h
 * @brief 电池电压读取模块
 *
 * 本文件提供电池电压的 ADC 采集接口。
 * 用于监控电池电量，当电压过低时可触发低电量保护
 * （如降低功率输出或报警）。
 *
 * 硬件说明：
 *   - 通常使用 STM32 内置 ADC 采集电池电压
 *   - 电池电压经过分压电阻后接入 ADC 引脚
 *   - 软件中根据分压比和 ADC 参考电压换算实际电压
 */

#pragma once

#include <stdbool.h>

/**
 * @brief 电池采样数据结构体
 */
typedef struct {
	float voltage_v; /**< 电池电压，单位：伏特（V） */
	bool valid;      /**< 采样是否有效（true = ADC 读取成功，false = 读取失败/未初始化） */
} battery_sample_t;

/**
 * @brief 初始化电池电压采集模块
 * @return 0 成功，非零失败
 *
 * 配置 ADC 外设和相关 GPIO 引脚。
 * 应在系统启动时调用一次。
 */
int battery_init(void);

/**
 * @brief 读取当前电池电压
 * @return battery_sample_t 结构体，包含电压值和有效性标志
 *
 * 每次调用执行一次 ADC 采样并转换为实际电压值。
 * 典型调用频率：1~10 Hz（电池电压变化缓慢，无需高频采样）。
 */
battery_sample_t battery_read(void);
