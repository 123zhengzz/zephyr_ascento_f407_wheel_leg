#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "app_config.h"
#include "battery.h"
#include "bmi088.h"
#include "control.h"
#include "dji_m3508.h"
#include "dm4340.h"
#include "motor_debug.h"

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

#define WHEEL_CAN_NODE DT_ALIAS(can_wheel)
#define JOINT_CAN_NODE DT_ALIAS(can_joint)
#define LED_RED_NODE DT_ALIAS(led0)
#define LED_GREEN_NODE DT_ALIAS(led1)
#define LED_BLUE_NODE DT_ALIAS(led2)

static const struct device *wheel_can = DEVICE_DT_GET(WHEEL_CAN_NODE);
static const struct device *joint_can = DEVICE_DT_GET(JOINT_CAN_NODE);
static const struct gpio_dt_spec red_led = GPIO_DT_SPEC_GET(LED_RED_NODE,
							    gpios);
static const struct gpio_dt_spec green_led = GPIO_DT_SPEC_GET(LED_GREEN_NODE,
							      gpios);
static const struct gpio_dt_spec blue_led = GPIO_DT_SPEC_GET(LED_BLUE_NODE,
							     gpios);

static dji_m3508_bus_t dji_bus;
static dm4340_bus_t dm_bus;
static bmi088_t imu;

K_THREAD_STACK_DEFINE(control_stack, 4096);
static struct k_thread control_thread_data;

static int prepare_can(const struct device *can, const char *name)
{
	if (!device_is_ready(can)) {
		LOG_ERR("%s is not ready", name);
		return -ENODEV;
	}

	int ret = can_set_bitrate(can, APP_CAN_BITRATE);
	if (ret != 0 && ret != -ENOTSUP) {
		LOG_WRN("%s bitrate set returned %d, using devicetree bitrate",
			name, ret);
	}

	ret = can_start(can);
	if (ret != 0 && ret != -EALREADY) {
		LOG_ERR("%s start failed: %d", name, ret);
		return ret;
	}

	LOG_INF("%s started at %u bps", name, APP_CAN_BITRATE);
	return 0;
}

static bool dm_any_online(void)
{
	for (uint8_t i = 1; i <= DM4340_MAX_ID; i++) {
		dm4340_feedback_t fb;
		if (dm4340_get(&dm_bus, i, &fb)) {
			return true;
		}
	}
	return false;
}

static void leds_init(void)
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

static volatile bool rx_led_state;

void dm4340_rx_led_toggle(void)
{
	rx_led_state = !rx_led_state;
	if (gpio_is_ready_dt(&green_led)) {
		(void)gpio_pin_set_dt(&green_led, rx_led_state ? 1 : 0);
	}
}

static void set_led(const struct gpio_dt_spec *led, bool on)
{
	if (gpio_is_ready_dt(led)) {
		(void)gpio_pin_set_dt(led, on ? 1 : 0);
	}
}

static int16_t apply_wheel_forward_sign(int16_t current, int sign)
{
	if (sign < 0) {
		return (int16_t)-current;
	}

	return current;
}

static int send_control_wheel_current(const control_output_t *out)
{
	return dji_m3508_send_group_current(
		&dji_bus,
		apply_wheel_forward_sign(out->left_wheel_current,
					 APP_WHEEL_LEFT_FORWARD_CURRENT_SIGN),
		apply_wheel_forward_sign(out->right_wheel_current,
					 APP_WHEEL_RIGHT_FORWARD_CURRENT_SIGN),
		0, 0);
}

static void send_debug_output(const motor_debug_output_t *debug)
{
	if (debug->wheel_mode == MOTOR_DEBUG_WHEEL_RPM) {
		(void)dji_m3508_send_rpm(&dji_bus, APP_WHEEL_LEFT_ID,
					 debug->left_wheel_rpm);
		(void)dji_m3508_send_rpm(&dji_bus, APP_WHEEL_RIGHT_ID,
					 debug->right_wheel_rpm);
	} else {
		(void)dji_m3508_send_group_current(
			&dji_bus, debug->wheel_current[APP_WHEEL_LEFT_ID],
			debug->wheel_current[APP_WHEEL_RIGHT_ID], 0, 0);
	}

	for (uint8_t id = 1; id <= DM4340_MAX_ID; id++) {
		const motor_debug_dm_command_t *cmd = &debug->dm[id];

		switch (cmd->mode) {
		case MOTOR_DEBUG_DM_POS_VEL:
			(void)dm4340_send_pos_vel(&dm_bus, id,
						  cmd->position_rad,
						  cmd->velocity_rad_s);
			break;
		case MOTOR_DEBUG_DM_VELOCITY:
			(void)dm4340_send_velocity(&dm_bus, id,
						   cmd->velocity_rad_s);
			break;
		case MOTOR_DEBUG_DM_MIT:
			(void)dm4340_send_mit(&dm_bus, id,
					      cmd->position_rad,
					      cmd->velocity_rad_s, cmd->kp,
					      cmd->kd, cmd->torque_nm);
			break;
		case MOTOR_DEBUG_DM_WIGGLE: {
			const int32_t period_ms = MAX(cmd->period_ms, 1);
			const int64_t elapsed_ms = k_uptime_get() - cmd->start_ms;
			const int32_t phase_ms = (int32_t)(elapsed_ms % period_ms);
			const float phase = (float)phase_ms / (float)period_ms;
			float wave;

			if (phase < 0.25f) {
				wave = phase * 4.0f;
			} else if (phase < 0.75f) {
				wave = 2.0f - phase * 4.0f;
			} else {
				wave = phase * 4.0f - 4.0f;
			}

			(void)dm4340_send_mit(&dm_bus, id,
					      cmd->position_rad +
						      cmd->amplitude_rad * wave,
					      0.0f, cmd->kp, cmd->kd, 0.0f);
			break;
		}
		case MOTOR_DEBUG_DM_NONE:
		default:
			break;
		}
	}
}

static void control_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	bmi088_sample_t imu_sample = { 0 };
	dji_m3508_motor_t left_motor = { 0 };
	dji_m3508_motor_t right_motor = { 0 };
	control_output_t out;
	int64_t last_us = k_cyc_to_us_floor64(k_cycle_get_64());
	bool debug_was_active = false;

	while (true) {
		const int64_t now_us = k_cyc_to_us_floor64(k_cycle_get_64());
		float dt_s = (float)(now_us - last_us) * 1.0e-6f;
		last_us = now_us;
		if (dt_s <= 0.0f || dt_s > 0.02f) {
			dt_s = 1.0f / APP_CONTROL_HZ;
		}

		const int imu_ret = bmi088_update(&imu, &imu_sample);
		const int64_t now_ms = k_uptime_get();
		const bool left_present = dji_m3508_get(
			&dji_bus, APP_WHEEL_LEFT_ID, &left_motor);
		const bool right_present = dji_m3508_get(
			&dji_bus, APP_WHEEL_RIGHT_ID, &right_motor);
		const bool left_ok =
			left_present &&
			(now_ms - left_motor.last_update_ms) <=
				APP_VESC_FEEDBACK_TIMEOUT_MS;
		const bool right_ok =
			right_present &&
			(now_ms - right_motor.last_update_ms) <=
				APP_VESC_FEEDBACK_TIMEOUT_MS;

		if (imu_ret == 0) {
			if (!left_ok) {
				left_motor = (dji_m3508_motor_t){ 0 };
			}
			if (!right_ok) {
				right_motor = (dji_m3508_motor_t){ 0 };
			}
			control_step(&imu_sample, &left_motor, &right_motor,
				     dt_s, &out);
			if (!left_ok || !right_ok) {
				out.wheels_enabled = false;
				out.left_wheel_current = 0;
				out.right_wheel_current = 0;
			}
		} else {
			out = (control_output_t) {
				.wheels_enabled = false,
				.joints_enabled = true,
				.left_wheel_current = 0,
				.right_wheel_current = 0,
				.left_joint_position_rad = APP_LEFT_LEG_MIN_RAD,
				.right_joint_position_rad = APP_RIGHT_LEG_MAX_RAD,
				.joint_velocity_limit_rad_s =
					APP_LEG_VEL_LIMIT_RAD_S,
			};
		}

			motor_debug_output_t debug;
			if (motor_debug_get_output(&debug)) {
				debug_was_active = true;
				send_debug_output(&debug);
				(void)dm4340_poll_rx_fifo(&dm_bus);
				k_sleep(K_USEC(1000000U / APP_CONTROL_HZ));
				continue;
			}

		if (debug_was_active) {
			(void)dm4340_send_mit(&dm_bus, APP_DM_LEFT_ID, 0.0f,
					      0.0f, 0.0f, 0.0f, 0.0f);
			(void)dm4340_send_mit(&dm_bus, APP_DM_RIGHT_ID, 0.0f,
					      0.0f, 0.0f, 0.0f, 0.0f);
			(void)dji_m3508_send_group_current(&dji_bus, 0, 0, 0,
							   0);
			debug_was_active = false;
		}

		if (out.wheels_enabled && left_ok && right_ok) {
			(void)send_control_wheel_current(&out);
		} else {
			(void)dji_m3508_send_group_current(&dji_bus, 0, 0, 0,
							   0);
		}

		if (out.joints_enabled) {
			(void)dm4340_send_pos_vel(
				&dm_bus, APP_DM_LEFT_ID,
				out.left_joint_position_rad,
				out.joint_velocity_limit_rad_s);
			(void)dm4340_send_pos_vel(
				&dm_bus, APP_DM_RIGHT_ID,
				out.right_joint_position_rad,
				out.joint_velocity_limit_rad_s);
		}

		/* Poll CAN RX FIFO directly (ISR workaround) */
		(void)dm4340_poll_rx_fifo(&dm_bus);

		k_sleep(K_USEC(1000000U / APP_CONTROL_HZ));
	}
}

int main(void)
{
	LOG_INF("DJI F407 Ascento wheel-leg Zephyr app boot");
	leds_init();
	control_init();

	if (prepare_can(joint_can, "CAN joint DM43xx") != 0) {
		LOG_ERR("joint CAN is required");
	}

	if (joint_can != wheel_can) {
		if (prepare_can(wheel_can, "CAN wheel VESC") != 0) {
			LOG_ERR("wheel CAN is required");
		}
	}

	const uint8_t wheel_ids[] = {
		APP_WHEEL_LEFT_ID,
		APP_WHEEL_RIGHT_ID,
	};

	(void)dji_m3508_init(&dji_bus, wheel_can, wheel_ids,
			     ARRAY_SIZE(wheel_ids));
	(void)dm4340_init(&dm_bus, joint_can, APP_DM_LEFT_FEEDBACK_ID,
			  APP_DM_RIGHT_FEEDBACK_ID);
	motor_debug_init(&dji_bus, &dm_bus);
	(void)dm4340_enable(&dm_bus, APP_DM_LEFT_ID);
	(void)dm4340_enable(&dm_bus, APP_DM_RIGHT_ID);

	if (bmi088_init(&imu) != 0) {
		LOG_ERR("BMI088 init failed; balance output will stay disabled");
	}

	(void)battery_init();

	k_thread_create(&control_thread_data, control_stack,
			K_THREAD_STACK_SIZEOF(control_stack), control_thread,
			NULL, NULL, NULL, K_PRIO_PREEMPT(2), 0, K_NO_WAIT);
	k_thread_name_set(&control_thread_data, "control");

	bool heartbeat = false;
	while (true) {
		heartbeat = !heartbeat;
		control_status_t status;
		control_get_status(&status);
		const battery_sample_t battery = battery_read();

		/* green LED used for CAN RX activity debug */
		bool dm_online = dm_any_online();
		set_led(&red_led, !dm_online || status.faulted || !status.wheels_enabled);
		set_led(&blue_led, battery.valid &&
				      battery.voltage_v >
					      APP_BATTERY_LED_THRESHOLD_V);

		if (battery.valid) {
			LOG_INF("pitch %.2f roll %.2f speed %.2f current %d/%d "
				"height %d batt %.2f V",
				(double)status.pitch_deg,
				(double)status.roll_deg,
				(double)status.speed_rad_s,
				status.left_wheel_current,
				status.right_wheel_current, status.height,
				(double)battery.voltage_v);
		} else {
			LOG_INF("pitch %.2f roll %.2f speed %.2f current %d/%d "
				"height %d",
				(double)status.pitch_deg,
				(double)status.roll_deg,
				(double)status.speed_rad_s,
				status.left_wheel_current,
				status.right_wheel_current, status.height);
		}

		k_sleep(K_MSEC(500));
	}
}
