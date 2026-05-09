/**
 * @file leds.c
 * @brief 三色 LED 指示灯模块实现
 *
 * 通过设备树 GPIO 别名绑定红/绿/蓝 LED 引脚。
 * 绿色 LED 翻转函数由 DM4340 CAN 接收 ISR 回调调用，
 * 为调试提供直观的 CAN 总线活动指示。
 */

#include "leds.h"

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>

/* ---- 设备树节点绑定 ---- */
#define LED_RED_NODE   DT_ALIAS(led0)
#define LED_GREEN_NODE DT_ALIAS(led1)
#define LED_BLUE_NODE  DT_ALIAS(led2)

/* ---- 静态 GPIO 描述符 ---- */
static const struct gpio_dt_spec red_led = GPIO_DT_SPEC_GET(LED_RED_NODE, gpios);
static const struct gpio_dt_spec green_led = GPIO_DT_SPEC_GET(LED_GREEN_NODE, gpios);
static const struct gpio_dt_spec blue_led = GPIO_DT_SPEC_GET(LED_BLUE_NODE, gpios);

/* 绿色 LED 翻转状态（volatile —— 可从 ISR 上下文写入） */
static volatile bool rx_led_state;

static void set_led(const struct gpio_dt_spec *led, bool on)
{
	if (gpio_is_ready_dt(led)) {
		(void)gpio_pin_set_dt(led, on ? 1 : 0);
	}
}

void leds_init(void)
{
	if (gpio_is_ready_dt(&red_led)) {
		(void)gpio_pin_configure_dt(&red_led, GPIO_OUTPUT_INACTIVE);
	}
	if (gpio_is_ready_dt(&green_led)) {
		(void)gpio_pin_configure_dt(&green_led, GPIO_OUTPUT_INACTIVE);
	}
	if (gpio_is_ready_dt(&blue_led)) {
		(void)gpio_pin_configure_dt(&blue_led, GPIO_OUTPUT_INACTIVE);
	}
}

void led_set_red(bool on)
{
	set_led(&red_led, on);
}

void led_set_blue(bool on)
{
	set_led(&blue_led, on);
}

void dm4340_rx_led_toggle(void)
{
	rx_led_state = !rx_led_state;
	set_led(&green_led, rx_led_state);
}
