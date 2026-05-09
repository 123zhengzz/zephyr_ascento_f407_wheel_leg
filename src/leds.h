#pragma once
/**
 * @file leds.h
 * @brief 三色 LED 指示灯模块接口
 *
 * 提供红/绿/蓝三色 LED 的初始化和控制接口。
 * 绿色 LED 由 DM4340 CAN 接收回调自动翻转，用于指示 CAN 活动。
 */

#include <stdbool.h>

/** @brief 初始化红/绿/蓝三色 LED 为输出模式，初始熄灭 */
void leds_init(void);

/** @brief 设置红色 LED 亮灭 */
void led_set_red(bool on);

/** @brief 设置蓝色 LED 亮灭 */
void led_set_blue(bool on);

/**
 * @brief 翻转绿色 LED（由 DM4340 CAN RX 回调调用）
 *
 * 每次 DM4340 收到 CAN 反馈帧时翻转一次，产生闪烁指示 CAN 接收活动。
 * 可从 ISR 上下文安全调用。
 */
void dm4340_rx_led_toggle(void);
