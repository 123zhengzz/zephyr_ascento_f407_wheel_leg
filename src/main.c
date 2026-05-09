/**
 * @file main.c
 * @brief Ascento 式双轮腿机器人 —— 系统入口与初始化
 *
 * 基于 Zephyr RTOS 的 STM32F407 双轮腿自平衡机器人主程序。
 *
 * 【硬件平台】
 *   - MCU: STM32F407 (DJI RoboMaster 开发板 C 型)
 *   - IMU: BMI088（SPI 接口，测量俯仰/横滚/偏航角及角速度）
 *   - 轮毂电机: DJI M3508 ×2（经 VESC 驱动器，CAN2 总线）
 *   - 关节电机: DM4340 ×2（髋关节直驱，CAN1 总线）
 *   - LED: 红/绿/蓝三色指示灯，用于状态反馈
 *
 * 【软件架构】
 *   - 主线程：外设初始化、LED 状态指示、500ms 周期日志输出
 *   - 控制线程：200Hz 实时平衡控制（见 control_loop.c）
 *
 * 【CAN 总线布局】
 *   - CAN1（joint）: DM4340 关节电机
 *   - CAN2（wheel）: DJI M3508 轮毂电机（VESC 协议）
 */

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
#include "control_loop.h"
#include "dji_m3508.h"
#include "dm4340.h"
#include "leds.h"
#include "motor_debug.h"

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

/* ========== 设备树 CAN 节点别名 ========== */
#define WHEEL_CAN_NODE DT_ALIAS(can_wheel)
#define JOINT_CAN_NODE DT_ALIAS(can_joint)

/* ========== CAN 设备句柄 ========== */
static const struct device *wheel_can = DEVICE_DT_GET(WHEEL_CAN_NODE);
static const struct device *joint_can = DEVICE_DT_GET(JOINT_CAN_NODE);

/* ========== 电机和传感器全局对象 ========== */
static dji_m3508_bus_t dji_bus;
static dm4340_bus_t dm_bus;
static bmi088_t imu;

/* ========== 硬件初始化辅助函数 ========== */

/** @brief 初始化并启动指定 CAN 总线 */
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

/** @brief 检查是否有至少一台 DM4340 关节电机在线 */
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

/* ========== 主函数 ========== */

int main(void)
{
	LOG_INF("DJI F407 Ascento wheel-leg Zephyr app boot");

	/* ===== 阶段 1：硬件外设初始化 ===== */
	leds_init();

	/* CAN 总线初始化 */
	if (prepare_can(joint_can, "CAN joint DM43xx") != 0) {
		LOG_ERR("joint CAN is required");
	}
	if (joint_can != wheel_can) {
		if (prepare_can(wheel_can, "CAN wheel VESC") != 0) {
			LOG_ERR("wheel CAN is required");
		}
	}

	/* 电机驱动初始化 */
	const uint8_t wheel_ids[] = { APP_WHEEL_LEFT_ID, APP_WHEEL_RIGHT_ID };
	(void)dji_m3508_init(&dji_bus, wheel_can, wheel_ids,
			     ARRAY_SIZE(wheel_ids));
	(void)dm4340_init(&dm_bus, joint_can, APP_DM_LEFT_FEEDBACK_ID,
			  APP_DM_RIGHT_FEEDBACK_ID);
	motor_debug_init(&dji_bus, &dm_bus);
	(void)dm4340_enable(&dm_bus, APP_DM_LEFT_ID);
	(void)dm4340_enable(&dm_bus, APP_DM_RIGHT_ID);

	/* 传感器初始化 */
	if (bmi088_init(&imu) != 0) {
		LOG_ERR("BMI088 init failed; balance output will stay disabled");
	}
	(void)battery_init();

	/* ===== 阶段 2：启动实时控制线程（200Hz） ===== */
	control_loop_start(&dji_bus, &dm_bus, &imu);

	/* ===== 阶段 3：主线程监控循环（500ms） ===== */
	bool heartbeat = false;
	while (true) {
		heartbeat = !heartbeat;

		control_status_t status;
		control_get_status(&status);

		const battery_sample_t battery = battery_read();

		/* LED 状态指示 */
		led_set_red(!dm_any_online() || status.faulted ||
			    !status.wheels_enabled);
		led_set_blue(battery.valid &&
			     battery.voltage_v > APP_BATTERY_LED_THRESHOLD_V);

		/* 周期日志输出 */
		if (battery.valid) {
			LOG_INF("pitch %.2f roll %.2f yaw %.1f gy %.1f gz %.1f current %d/%d "
				"height %d batt %.2f V",
				(double)status.pitch_deg,
				(double)status.roll_deg,
				(double)status.yaw_deg,
				(double)last_imu_sample.gy_dps,
				(double)last_imu_sample.gz_dps,
				status.left_wheel_current,
				status.right_wheel_current, status.height,
				(double)battery.voltage_v);
		} else {
			LOG_INF("pitch %.2f roll %.2f yaw %.1f gy %.1f gz %.1f current %d/%d "
				"height %d",
				(double)status.pitch_deg,
				(double)status.roll_deg,
				(double)status.yaw_deg,
				(double)last_imu_sample.gy_dps,
				(double)last_imu_sample.gz_dps,
				status.left_wheel_current,
				status.right_wheel_current, status.height);
		}

		k_sleep(K_MSEC(500));
	}
}
