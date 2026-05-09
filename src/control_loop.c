/**
 * @file control_loop.c
 * @brief 200Hz 平衡控制线程实现
 *
 * 本文件包含机器人实时平衡控制的核心逻辑，以 200Hz 固定频率运行：
 *   1. 计算控制周期 dt（微秒级精度）
 *   2. 读取 IMU 姿态数据和轮毂电机反馈
 *   3. 运行平衡控制算法（Ascento LQR 或 PID 备用方案）
 *   4. 发送轮毂电流和关节位置指令
 *   5. 轮询 DM4340 CAN 接收 FIFO（ISR 变通方案）
 *
 * 控制线程优先级为 K_PRIO_PREEMPT(2)，高于主线程，
 * 确保实时控制不受主线程日志输出的干扰。
 */

#include "control_loop.h"

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "app_config.h"
#include "ascento_balance.h"
#include "control.h"
#include "motor_debug.h"

LOG_MODULE_REGISTER(control_loop, LOG_LEVEL_INF);

/* ========== 控制线程栈和 TCB ========== */
K_THREAD_STACK_DEFINE(control_stack, 4096);
static struct k_thread control_thread_data;

/* ========== 文件内全局状态 ========== */

/* 最近一次 IMU 采样数据缓存，供主线程日志输出 */
bmi088_sample_t last_imu_sample;

/* 电机和传感器总线指针（由 control_loop_start 注入） */
static dji_m3508_bus_t *dji_bus;
static dm4340_bus_t *dm_bus;
static bmi088_t *imu;

#if APP_USE_ASCENTO_BALANCE_CONTROLLER
static ascento_balance_state_t ascento_state;
#endif

/* ========== 静态辅助函数 ========== */

/**
 * @brief 根据电机安装方向校正轮毂电流符号
 *
 * 左右轮毂电机物理安装方向可能相反，通过符号位校正确保
 * 相同电流值产生相同方向的力矩。
 */
static int16_t apply_wheel_forward_sign(int16_t current, int sign)
{
	if (sign < 0) {
		return (int16_t)-current;
	}
	return current;
}

/** @brief 向左右轮毂电机发送组电流指令（含方向校正） */
static int send_control_wheel_current(const control_output_t *out)
{
	return dji_m3508_send_group_current(
		dji_bus,
		apply_wheel_forward_sign(out->left_wheel_current,
					 APP_WHEEL_LEFT_FORWARD_CURRENT_SIGN),
		apply_wheel_forward_sign(out->right_wheel_current,
					 APP_WHEEL_RIGHT_FORWARD_CURRENT_SIGN),
		0, 0);
}

/** @brief 向左右关节电机发送位置控制指令（MIT 模式或位置-速度模式） */
static void send_control_joints(const control_output_t *out)
{
#if APP_PID_BALANCE_JOINT_USE_MIT
	(void)dm4340_send_mit(dm_bus, APP_DM_LEFT_ID,
			      out->left_joint_position_rad, 0.0f,
			      APP_PID_BALANCE_JOINT_MIT_KP,
			      APP_PID_BALANCE_JOINT_MIT_KD, 0.0f);
	(void)dm4340_send_mit(dm_bus, APP_DM_RIGHT_ID,
			      out->right_joint_position_rad, 0.0f,
			      APP_PID_BALANCE_JOINT_MIT_KP,
			      APP_PID_BALANCE_JOINT_MIT_KD, 0.0f);
#else
	(void)dm4340_send_pos_vel(dm_bus, APP_DM_LEFT_ID,
				  out->left_joint_position_rad,
				  out->joint_velocity_limit_rad_s);
	(void)dm4340_send_pos_vel(dm_bus, APP_DM_RIGHT_ID,
				  out->right_joint_position_rad,
				  out->joint_velocity_limit_rad_s);
#endif
}

/** @brief 发送电机调试指令（标定/测试模式） */
static void send_debug_output(const motor_debug_output_t *debug)
{
	/* 轮毂电机调试 */
	if (debug->wheel_mode == MOTOR_DEBUG_WHEEL_RPM) {
		(void)dji_m3508_send_rpm(dji_bus, APP_WHEEL_LEFT_ID,
					 debug->left_wheel_rpm);
		(void)dji_m3508_send_rpm(dji_bus, APP_WHEEL_RIGHT_ID,
					 debug->right_wheel_rpm);
	} else {
		(void)dji_m3508_send_group_current(
			dji_bus, debug->wheel_current[APP_WHEEL_LEFT_ID],
			debug->wheel_current[APP_WHEEL_RIGHT_ID], 0, 0);
	}

	/* 关节电机调试 */
	for (uint8_t id = 1; id <= DM4340_MAX_ID; id++) {
		const motor_debug_dm_command_t *cmd = &debug->dm[id];

		switch (cmd->mode) {
		case MOTOR_DEBUG_DM_POS_VEL:
			(void)dm4340_send_pos_vel(dm_bus, id,
						  cmd->position_rad,
						  cmd->velocity_rad_s);
			break;
		case MOTOR_DEBUG_DM_VELOCITY:
			(void)dm4340_send_velocity(dm_bus, id,
						   cmd->velocity_rad_s);
			break;
		case MOTOR_DEBUG_DM_MIT:
			(void)dm4340_send_mit(dm_bus, id,
					      cmd->position_rad,
					      cmd->velocity_rad_s, cmd->kp,
					      cmd->kd, cmd->torque_nm);
			break;
		case MOTOR_DEBUG_DM_WIGGLE: {
			const int32_t period_ms = MAX(cmd->period_ms, 1);
			const int64_t elapsed_ms =
				k_uptime_get() - cmd->start_ms;
			const int32_t phase_ms =
				(int32_t)(elapsed_ms % period_ms);
			const float phase =
				(float)phase_ms / (float)period_ms;
			float wave;

			if (phase < 0.25f) {
				wave = phase * 4.0f;
			} else if (phase < 0.75f) {
				wave = 2.0f - phase * 4.0f;
			} else {
				wave = phase * 4.0f - 4.0f;
			}

			(void)dm4340_send_mit(
				dm_bus, id,
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

/* ========== 控制线程 ========== */

/**
 * @brief 控制线程入口 —— 200Hz 平衡控制循环
 *
 * 每个周期执行：
 *   1. 计算 dt（含合理性限幅，防止阻塞后积分饱和）
 *   2. 读取 IMU + 轮毂电机反馈
 *   3. 运行平衡控制算法（Ascento LQR 或 PID）
 *   4. 发送电机控制指令
 *   5. 轮询 DM4340 CAN RX FIFO
 */
static void control_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	bmi088_sample_t imu_sample = { 0 };
	dji_m3508_motor_t left_motor = { 0 };
	dji_m3508_motor_t right_motor = { 0 };
	control_output_t out;
	int64_t last_us =
		k_cyc_to_us_floor64(k_cycle_get_64());

	while (true) {
		/* ---- 步骤 1：计算 dt ---- */
		const int64_t now_us =
			k_cyc_to_us_floor64(k_cycle_get_64());
		float dt_s = (float)(now_us - last_us) * 1.0e-6f;
		last_us = now_us;

		if (dt_s <= 0.0f || dt_s > 0.02f) {
			dt_s = 1.0f / APP_CONTROL_HZ;
		}

		/* ---- 步骤 2：读取传感器和电机反馈 ---- */
		const int imu_ret = bmi088_update(imu, &imu_sample);
		const bool left_present = dji_m3508_get(
			dji_bus, APP_WHEEL_LEFT_ID, &left_motor);
		const bool right_present = dji_m3508_get(
			dji_bus, APP_WHEEL_RIGHT_ID, &right_motor);
		const bool enable_request = control_get_enable_request();

		/* ---- 步骤 3：运行控制算法 ---- */
		if (imu_ret == 0) {
			last_imu_sample = imu_sample;

			if (!left_present) {
				left_motor = (dji_m3508_motor_t){ 0 };
			}
			if (!right_present) {
				right_motor = (dji_m3508_motor_t){ 0 };
			}

#if APP_USE_ASCENTO_BALANCE_CONTROLLER
			/* ===== Ascento LQR 平衡控制器 ===== */
			ascento_balance_params_t ap_copy;
			ascento_balance_get_params(&ap_copy);
			const ascento_balance_params_t *ap = &ap_copy;

			const ascento_balance_input_t ai = {
				.enable_request = enable_request,
				.wheel_feedback_ok =
					left_present && right_present,
				.dt_s = dt_s,
				.target_forward_speed_mps = 0.0f,
				.target_yaw_rate_rad_s = 0.0f,
				.target_pitch_rad = 0.0f,
				.left_joint_position_rad = 0.0f,
				.right_joint_position_rad = 0.0f,
				.left_joint_velocity_rad_s = 0.0f,
				.right_joint_velocity_rad_s = 0.0f,
				.imu = imu_sample,
				.left_wheel = left_motor,
				.right_wheel = right_motor,
			};

			ascento_balance_output_t ao;
			ascento_balance_update(&ascento_state, ap, &ai,
					       &ao);

			if (ao.active && enable_request && !ao.faulted &&
			    left_present && right_present) {
				out.wheels_enabled = true;
				out.joints_enabled = true;
				out.left_wheel_current =
					ao.left_wheel_current;
				out.right_wheel_current =
					ao.right_wheel_current;
				out.left_joint_position_rad =
					ao.left_joint_position_rad;
				out.right_joint_position_rad =
					ao.right_joint_position_rad;
				out.joint_velocity_limit_rad_s =
					ao.joint_velocity_limit_rad_s;
			} else {
				out.wheels_enabled = false;
				out.joints_enabled = true;
				out.left_wheel_current = 0;
				out.right_wheel_current = 0;
				out.left_joint_position_rad =
					APP_PID_BALANCE_LOCK_LEFT_JOINT_RAD;
				out.right_joint_position_rad =
					APP_PID_BALANCE_LOCK_RIGHT_JOINT_RAD;
				out.joint_velocity_limit_rad_s =
					APP_LEG_VEL_LIMIT_RAD_S;
			}
#else
			/* ===== PID 平衡控制器（备用方案） ===== */
			control_step(&imu_sample, &left_motor,
				     &right_motor, dt_s, &out);
#endif

#if APP_USE_ASCENTO_BALANCE_CONTROLLER
			control_publish_status(&(control_status_t){
				.enable_request = enable_request,
				.wheels_enabled = out.wheels_enabled,
				.faulted = ao.faulted,
				.height = APP_DEFAULT_HEIGHT,
				.joy_x = 0.0f,
				.joy_y = 0.0f,
				.motion = ROBOT_STOP,
				.pitch_deg = APP_ASCENTO_IMU_PITCH_SIGN *
					     imu_sample.pitch_deg,
				.pitch_rate_dps =
					ao.pitch_rate_rad_s *
					57.29577951308232f,
				.roll_deg = APP_ASCENTO_IMU_ROLL_SIGN *
					    imu_sample.roll_deg,
				.yaw_deg = imu_sample.yaw_deg,
				.distance_rad = ao.body_position_m,
				.speed_rad_s = ao.body_velocity_mps,
				.lqr_output = ao.balance_torque_nm,
				.yaw_output = ao.yaw_torque_nm,
				.left_joint_position_rad =
					out.left_joint_position_rad,
				.right_joint_position_rad =
					out.right_joint_position_rad,
				.left_wheel_current =
					out.left_wheel_current,
				.right_wheel_current =
					out.right_wheel_current,
				.jump_phase = 0,
			});
#endif
		} else {
			/* IMU 读取失败：断开轮毂，锁定关节 */
			out = (control_output_t){
				.wheels_enabled = false,
				.joints_enabled = true,
				.left_wheel_current = 0,
				.right_wheel_current = 0,
				.left_joint_position_rad =
					APP_PID_BALANCE_LOCK_LEFT_JOINT_RAD,
				.right_joint_position_rad =
					APP_PID_BALANCE_LOCK_RIGHT_JOINT_RAD,
				.joint_velocity_limit_rad_s =
					APP_LEG_VEL_LIMIT_RAD_S,
			};
		}

		/* ---- 步骤 4：发送电机指令 ---- */
		motor_debug_output_t debug;
		if (motor_debug_get_output(&debug)) {
			send_debug_output(&debug);
			send_control_joints(&out);
			(void)dm4340_poll_rx_fifo(dm_bus);
			k_sleep(K_USEC(1000000U / APP_CONTROL_HZ));
			continue;
		}

		(void)send_control_wheel_current(&out);

		if (out.joints_enabled) {
			send_control_joints(&out);
		}

		(void)dm4340_poll_rx_fifo(dm_bus);

		k_sleep(K_USEC(1000000U / APP_CONTROL_HZ));
	}
}

/* ========== 公共接口 ========== */

void control_loop_start(dji_m3508_bus_t *dji, dm4340_bus_t *dm,
			bmi088_t *imu_dev)
{
	dji_bus = dji;
	dm_bus = dm;
	imu = imu_dev;

	control_init();

#if APP_USE_ASCENTO_BALANCE_CONTROLLER
	ascento_balance_init(&ascento_state);
	ascento_balance_settings_init();
#endif

	k_thread_create(&control_thread_data, control_stack,
			K_THREAD_STACK_SIZEOF(control_stack), control_thread,
			NULL, NULL, NULL, K_PRIO_PREEMPT(2), 0, K_NO_WAIT);
	k_thread_name_set(&control_thread_data, "control");
}
