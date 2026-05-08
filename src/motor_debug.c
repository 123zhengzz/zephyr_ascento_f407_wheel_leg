/**
 * @file motor_debug.c
 * @brief 电机调试与测试模块 (Motor Debug and Testing Module)
 *
 * 本模块为轮腿机器人 (Ascento 风格, STM32F407) 提供电机调试功能，支持：
 *   - DJI M3508 轮毂电机：电流控制、RPM 速度控制（单轮/双轮对）
 *   - DM4340 关节电机：位置+速度限制、纯速度控制、MIT 阻抗控制、正弦抖动
 *
 * 设计要点：
 *   1. 所有调试命令均设有持续时间（deadline），超时后自动清除，防止失控
 *   2. 通过自旋锁 (spinlock) 保证线程安全，可在中断或 shell 线程中调用
 *   3. 输出通过 motor_debug_get_output() 获取，供主控制循环中的混合器使用
 *   4. 提供电机使能/禁用、零位保存、寄存器读取等辅助功能
 *
 * 使用场景：shell 命令行调试、电机标定、关节测试
 */

#include "motor_debug.h"

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include "app_config.h"
#include "pid.h"

/**
 * @brief 电机调试状态结构体
 *
 * 存储所有待发送的调试命令状态，包括：
 *   - 轮毂电机 (DJI M3508) 的电流/RPM 指令
 *   - 关节电机 (DM4340) 的 MIT/位置/速度/抖动指令
 *   - 各指令的截止时间（deadline），超时自动失效
 */
typedef struct {
	bool wheel_active;                /**< 轮毂电机调试命令是否激活 */
	motor_debug_wheel_mode_t wheel_mode; /**< 轮毂电机控制模式：电流或 RPM */
	int64_t wheel_deadline_ms;        /**< 轮毂命令截止时间戳 (ms)，超时自动清除 */
	int16_t wheel_current[DJI_M3508_MAX_ID + 1]; /**< 各轮毂电机的目标电流值 (按 ID 索引) */
	int32_t left_wheel_rpm;           /**< 左轮目标转速 (RPM)，仅在 RPM 模式下有效 */
	int32_t right_wheel_rpm;          /**< 右轮目标转速 (RPM)，仅在 RPM 模式下有效 */

	motor_debug_dm_command_t dm[DM4340_MAX_ID + 1]; /**< 各关节电机的调试命令 (按 ID 索引) */
	int64_t dm_deadline_ms[DM4340_MAX_ID + 1];     /**< 各关节电机命令的截止时间戳 (ms) */
} motor_debug_state_t;

/**
 * @brief 模块全局上下文
 *
 * 包含总线句柄、线程锁和当前调试状态。
 * 所有公共接口通过 lock 互斥访问 state，确保线程安全。
 */
static struct {
	dji_m3508_bus_t *dji_bus;      /**< DJI M3508 CAN 总线句柄 */
	dm4340_bus_t *dm_bus;          /**< DM4340 CAN 总线句柄 */
	struct k_spinlock lock;        /**< 自旋锁，保护 state 的并发访问 */
	motor_debug_state_t state;     /**< 当前调试命令状态 */
} ctx;

/**
 * @brief 根据持续时间计算绝对截止时间戳
 *
 * @param duration_ms 持续时间（毫秒）。若 <= 0 则视为无限期（返回 INT64_MAX）
 * @return int64_t 绝对截止时间戳（毫秒），基于系统启动时间
 */
static int64_t deadline_from_duration(int32_t duration_ms)
{
	return duration_ms > 0 ? k_uptime_get() + duration_ms : INT64_MAX;
}

/**
 * @brief 验证轮毂电机 ID 是否有效
 *
 * @param id 电机 ID（从 1 开始编号）
 * @return true 有效，false 无效（ID 不能为 0）
 */
static bool valid_wheel_id(uint8_t id)
{
	return id > 0;
}

/**
 * @brief 验证 DM4340 关节电机 ID 是否有效
 *
 * @param id 电机 ID
 * @return true 有效（1 ~ DM4340_MAX_ID），false 无效
 */
static bool valid_dm_id(uint8_t id)
{
	return id > 0 && id <= DM4340_MAX_ID;
}

/**
 * @brief 根据关节 ID 将目标位置限制在安全范围内
 *
 * 左腿和右腿有不同的机械限位，需分别限制。
 * 未知 ID 使用保守的 [-1, 1] rad 范围。
 *
 * @param id          关节电机 ID
 * @param position_rad 目标位置（弧度）
 * @return float      限制后的位置（弧度）
 */
static float clamp_joint_position(uint8_t id, float position_rad)
{
	if (id == APP_DM_LEFT_ID) {
		return app_clampf(position_rad, APP_LEFT_LEG_MIN_RAD,
				  APP_LEFT_LEG_MAX_RAD);
	}
	if (id == APP_DM_RIGHT_ID) {
		return app_clampf(position_rad, APP_RIGHT_LEG_MIN_RAD,
				  APP_RIGHT_LEG_MAX_RAD);
	}

	return app_clampf(position_rad, -1.0f, 1.0f);
}

/**
 * @brief 清除所有已过期的 DM4340 关节电机调试命令
 *
 * 遍历所有关节电机，将超过截止时间的命令重置为空。
 * 调用此函数前必须已持有 ctx.lock（后缀 _locked 表示调用约定）。
 *
 * @param now_ms 当前系统时间戳（毫秒）
 */
static void clear_expired_locked(int64_t now_ms)
{
	for (uint8_t id = 1; id <= DM4340_MAX_ID; id++) {
		if (ctx.state.dm[id].mode != MOTOR_DEBUG_DM_NONE &&
		    now_ms > ctx.state.dm_deadline_ms[id]) {
			ctx.state.dm[id] = (motor_debug_dm_command_t){ 0 };
			ctx.state.dm_deadline_ms[id] = 0;
		}
	}
}

/**
 * @brief 初始化电机调试模块
 *
 * 清零全局上下文并绑定 CAN 总线句柄。
 * 应在系统启动时、CAN 总线初始化完成后调用一次。
 *
 * @param dji_bus DJI M3508 CAN 总线句柄（可为 NULL，表示未连接）
 * @param dm_bus  DM4340  CAN 总线句柄（可为 NULL，表示未连接）
 */
void motor_debug_init(dji_m3508_bus_t *dji_bus, dm4340_bus_t *dm_bus)
{
	memset(&ctx, 0, sizeof(ctx));
	ctx.dji_bus = dji_bus;
	ctx.dm_bus = dm_bus;
}

/**
 * @brief 设置单个轮毂电机的目标电流
 *
 * 会先清零所有轮毂电机的电流和 RPM，仅对指定 ID 施加电流。
 * 用于单个电机的标定和测试。
 *
 * @param id          轮毂电机 ID（1 ~ DJI_M3508_MAX_ID）
 * @param current     目标电流值（单位取决于 DJI 协议，通常为 -16384 ~ 16384）
 * @param duration_ms 持续时间（毫秒），<=0 表示无限期
 * @return 0 成功，-EINVAL ID 无效
 */
int motor_debug_set_wheel_current(uint8_t id, int32_t current,
				  int32_t duration_ms)
{
	if (!valid_wheel_id(id)) {
		return -EINVAL;
	}

	k_spinlock_key_t key = k_spin_lock(&ctx.lock);
	memset(ctx.state.wheel_current, 0, sizeof(ctx.state.wheel_current));
	ctx.state.left_wheel_rpm = 0;
	ctx.state.right_wheel_rpm = 0;
	ctx.state.wheel_current[id] = current;
	ctx.state.wheel_active = true;
	ctx.state.wheel_mode = MOTOR_DEBUG_WHEEL_CURRENT;
	ctx.state.wheel_deadline_ms = deadline_from_duration(duration_ms);
	k_spin_unlock(&ctx.lock, key);

	return 0;
}

/**
 * @brief 同时设置左右两个轮毂电机的目标电流
 *
 * 用于双轮同步测试，例如差速转向调试。
 * 左右电流可独立设置，实现原地转向等效果。
 *
 * @param left_current  左轮目标电流
 * @param right_current 右轮目标电流
 * @param duration_ms   持续时间（毫秒），<=0 表示无限期
 * @return 0 始终成功
 */
int motor_debug_set_wheel_pair(int32_t left_current, int32_t right_current,
			       int32_t duration_ms)
{
	k_spinlock_key_t key = k_spin_lock(&ctx.lock);
	memset(ctx.state.wheel_current, 0, sizeof(ctx.state.wheel_current));
	ctx.state.left_wheel_rpm = 0;
	ctx.state.right_wheel_rpm = 0;
	ctx.state.wheel_current[APP_WHEEL_LEFT_ID] = left_current;
	ctx.state.wheel_current[APP_WHEEL_RIGHT_ID] = right_current;
	ctx.state.wheel_active = true;
	ctx.state.wheel_mode = MOTOR_DEBUG_WHEEL_CURRENT;
	ctx.state.wheel_deadline_ms = deadline_from_duration(duration_ms);
	k_spin_unlock(&ctx.lock, key);

	return 0;
}

/**
 * @brief 设置单个轮毂电机的目标转速 (RPM)
 *
 * 使用 RPM 闭环控制模式，清零所有电流指令。
 * 仅接受左轮或右轮的合法 ID。
 *
 * @param id          轮毂电机 ID（APP_WHEEL_LEFT_ID 或 APP_WHEEL_RIGHT_ID）
 * @param rpm         目标转速（转/分钟）
 * @param duration_ms 持续时间（毫秒），<=0 表示无限期
 * @return 0 成功，-EINVAL ID 无效
 */
int motor_debug_set_wheel_rpm(uint8_t id, int32_t rpm, int32_t duration_ms)
{
	if (id != APP_WHEEL_LEFT_ID && id != APP_WHEEL_RIGHT_ID) {
		return -EINVAL;
	}

	k_spinlock_key_t key = k_spin_lock(&ctx.lock);
	memset(ctx.state.wheel_current, 0, sizeof(ctx.state.wheel_current));
	ctx.state.left_wheel_rpm = id == APP_WHEEL_LEFT_ID ? rpm : 0;
	ctx.state.right_wheel_rpm = id == APP_WHEEL_RIGHT_ID ? rpm : 0;
	ctx.state.wheel_active = true;
	ctx.state.wheel_mode = MOTOR_DEBUG_WHEEL_RPM;
	ctx.state.wheel_deadline_ms = deadline_from_duration(duration_ms);
	k_spin_unlock(&ctx.lock, key);

	return 0;
}

/**
 * @brief 立即停止所有轮毂电机
 *
 * 清除调试状态并向所有四个轮毂电机发送零电流指令。
 * 用于紧急停止或测试结束。
 *
 * @return 0 成功，或 CAN 发送错误码
 */
int motor_debug_stop_wheels(void)
{
	k_spinlock_key_t key = k_spin_lock(&ctx.lock);
	ctx.state.wheel_active = false;
	ctx.state.wheel_mode = MOTOR_DEBUG_WHEEL_NONE;
	ctx.state.wheel_deadline_ms = 0;
	memset(ctx.state.wheel_current, 0, sizeof(ctx.state.wheel_current));
	ctx.state.left_wheel_rpm = 0;
	ctx.state.right_wheel_rpm = 0;
	k_spin_unlock(&ctx.lock, key);

	if (ctx.dji_bus != NULL) {
		return dji_m3508_send_group_current(ctx.dji_bus, 0, 0, 0, 0);
	}

	return 0;
}

/**
 * @brief 使能指定 DM4340 关节电机
 *
 * 发送使能指令，电机进入伺服模式，可接收控制命令。
 *
 * @param id 关节电机 ID (1 ~ DM4340_MAX_ID)
 * @return 0 成功，-EINVAL 参数无效
 */
int motor_debug_dm_enable(uint8_t id)
{
	if (!valid_dm_id(id) || ctx.dm_bus == NULL) {
		return -EINVAL;
	}

	return dm4340_enable(ctx.dm_bus, id);
}

/**
 * @brief 禁用指定 DM4340 关节电机
 *
 * 先停止该电机的调试命令，再发送禁用指令，电机退出伺服模式。
 *
 * @param id 关节电机 ID (1 ~ DM4340_MAX_ID)
 * @return 0 成功，-EINVAL 参数无效
 */
int motor_debug_dm_disable(uint8_t id)
{
	if (!valid_dm_id(id) || ctx.dm_bus == NULL) {
		return -EINVAL;
	}

	(void)motor_debug_stop_dm(id);
	return dm4340_disable(ctx.dm_bus, id);
}

/**
 * @brief 保存当前位置为 DM4340 关节电机的零位
 *
 * 将当前位置写入电机 EEPROM 作为机械零点。
 * 标定完成后调用，使后续上电时以当前位置为零位。
 *
 * @param id 关节电机 ID (1 ~ DM4340_MAX_ID)
 * @return 0 成功，-EINVAL 参数无效
 */
int motor_debug_dm_save_zero(uint8_t id)
{
	if (!valid_dm_id(id) || ctx.dm_bus == NULL) {
		return -EINVAL;
	}

	return dm4340_save_zero(ctx.dm_bus, id);
}

/**
 * @brief 读取 DM4340 关节电机的寄存器参数
 *
 * 发送参数读取请求后，轮询等待响应，超时时间为 80ms。
 * 用于调试时读取电机内部参数（如 PID 增益、限位值等）。
 *
 * @param id  关节电机 ID (1 ~ DM4340_MAX_ID)
 * @param rid 寄存器 ID（由 DM4340 协议定义）
 * @param out 输出参数，存放读取到的响应数据
 * @return 0 成功，-EINVAL 参数无效，-ETIMEDOUT 超时无响应
 */
int motor_debug_dm_read_reg(uint8_t id, uint8_t rid,
			    dm4340_param_response_t *out)
{
	if (!valid_dm_id(id) || ctx.dm_bus == NULL || out == NULL) {
		return -EINVAL;
	}

	const int64_t start_ms = k_uptime_get();
	const int ret = dm4340_request_param_read(ctx.dm_bus, id, rid);
	if (ret != 0) {
		return ret;
	}

	/* 轮询接收 FIFO，等待参数响应到达，最多等待 80ms */
	while ((k_uptime_get() - start_ms) < 80) {
		(void)dm4340_poll_rx_fifo(ctx.dm_bus);

		dm4340_param_response_t response;
		if (dm4340_get_param_response(ctx.dm_bus, id, rid,
					      &response) &&
		    response.last_update_ms >= start_ms) {
			*out = response;
			return 0;
		}

		k_sleep(K_MSEC(2));
	}

	return -ETIMEDOUT;
}

/**
 * @brief 设置 DM4340 关节电机的位置+速度限制命令
 *
 * 电机将以给定速度限制运动到目标位置。
 * 位置会被限制在关节安全范围内，速度被限制在调试上限内。
 *
 * @param id             关节电机 ID
 * @param position_rad   目标位置（弧度），将被 clamp 到安全范围
 * @param velocity_rad_s 最大速度限制（弧度/秒）
 * @param duration_ms    持续时间（毫秒），<=0 表示无限期
 * @return 0 成功，-EINVAL ID 无效
 */
int motor_debug_set_dm_pos_vel(uint8_t id, float position_rad,
			       float velocity_rad_s, int32_t duration_ms)
{
	if (!valid_dm_id(id)) {
		return -EINVAL;
	}

	position_rad = clamp_joint_position(id, position_rad);
	velocity_rad_s = app_clampf(velocity_rad_s,
				    -APP_DM_DEBUG_VEL_LIMIT_RAD_S,
				    APP_DM_DEBUG_VEL_LIMIT_RAD_S);

	k_spinlock_key_t key = k_spin_lock(&ctx.lock);
	ctx.state.dm[id] = (motor_debug_dm_command_t) {
		.mode = MOTOR_DEBUG_DM_POS_VEL,
		.id = id,
		.position_rad = position_rad,
		.velocity_rad_s = velocity_rad_s,
	};
	ctx.state.dm_deadline_ms[id] = deadline_from_duration(duration_ms);
	k_spin_unlock(&ctx.lock, key);

	return 0;
}

/**
 * @brief 设置 DM4340 关节电机的纯速度命令
 *
 * 电机以指定速度持续旋转，无位置限制。
 * 速度被限制在调试上限内，防止失控。
 *
 * @param id             关节电机 ID
 * @param velocity_rad_s 目标速度（弧度/秒），正负表示方向
 * @param duration_ms    持续时间（毫秒），<=0 表示无限期
 * @return 0 成功，-EINVAL ID 无效
 */
int motor_debug_set_dm_velocity(uint8_t id, float velocity_rad_s,
				int32_t duration_ms)
{
	if (!valid_dm_id(id)) {
		return -EINVAL;
	}

	velocity_rad_s = app_clampf(velocity_rad_s,
				    -APP_DM_DEBUG_VEL_LIMIT_RAD_S,
				    APP_DM_DEBUG_VEL_LIMIT_RAD_S);

	k_spinlock_key_t key = k_spin_lock(&ctx.lock);
	ctx.state.dm[id] = (motor_debug_dm_command_t) {
		.mode = MOTOR_DEBUG_DM_VELOCITY,
		.id = id,
		.velocity_rad_s = velocity_rad_s,
	};
	ctx.state.dm_deadline_ms[id] = deadline_from_duration(duration_ms);
	k_spin_unlock(&ctx.lock, key);

	return 0;
}

/**
 * @brief 设置 DM4340 关节电机的 MIT 阻抗控制命令（带关节限位保护）
 *
 * MIT 控制模式提供完整的阻抗控制，电机输出力矩为：
 *   tau = kp * (pos_des - pos_cur) + kd * (vel_des - vel_cur) + tau_ff
 *
 * 位置目标会被 clamp 到关节安全范围内，适合正常调试使用。
 *
 * @param id             关节电机 ID
 * @param position_rad   目标位置（弧度），限制在关节安全范围
 * @param velocity_rad_s 目标速度（弧度/秒）
 * @param kp             位置刚度增益
 * @param kd             速度阻尼增益
 * @param torque_nm      前馈力矩（牛米）
 * @param duration_ms    持续时间（毫秒），<=0 表示无限期
 * @return 0 成功，-EINVAL ID 无效
 */
int motor_debug_set_dm_mit(uint8_t id, float position_rad,
			   float velocity_rad_s, float kp, float kd,
			   float torque_nm, int32_t duration_ms)
{
	if (!valid_dm_id(id)) {
		return -EINVAL;
	}

	position_rad = clamp_joint_position(id, position_rad);
	velocity_rad_s = app_clampf(velocity_rad_s,
				    -APP_DM_DEBUG_VEL_LIMIT_RAD_S,
				    APP_DM_DEBUG_VEL_LIMIT_RAD_S);
	kp = app_clampf(kp, 0.0f, APP_DM_DEBUG_KP_LIMIT);
	kd = app_clampf(kd, 0.0f, APP_DM_DEBUG_KD_LIMIT);
	torque_nm = app_clampf(torque_nm, -APP_DM_DEBUG_TORQUE_LIMIT_NM,
			       APP_DM_DEBUG_TORQUE_LIMIT_NM);

	k_spinlock_key_t key = k_spin_lock(&ctx.lock);
	ctx.state.dm[id] = (motor_debug_dm_command_t) {
		.mode = MOTOR_DEBUG_DM_MIT,
		.id = id,
		.position_rad = position_rad,
		.velocity_rad_s = velocity_rad_s,
		.kp = kp,
		.kd = kd,
		.torque_nm = torque_nm,
	};
	ctx.state.dm_deadline_ms[id] = deadline_from_duration(duration_ms);
	k_spin_unlock(&ctx.lock, key);

	return 0;
}

/**
 * @brief 设置 DM4340 关节电机的 MIT 阻抗控制命令（原始范围，无关节限位保护）
 *
 * 与 motor_debug_set_dm_mit() 功能相同，但位置仅做宽范围限制 [-12.5, 12.5] rad，
 * 不做关节安全范围保护。适用于需要跨越关节限位的特殊测试场景（如标定）。
 *
 * 注意：使用此函数时请格外小心，避免机械碰撞。
 *
 * @param id             关节电机 ID
 * @param position_rad   目标位置（弧度），仅做宽范围限制
 * @param velocity_rad_s 目标速度（弧度/秒）
 * @param kp             位置刚度增益
 * @param kd             速度阻尼增益
 * @param torque_nm      前馈力矩（牛米）
 * @param duration_ms    持续时间（毫秒），<=0 表示无限期
 * @return 0 成功，-EINVAL ID 无效
 */
int motor_debug_set_dm_mit_raw(uint8_t id, float position_rad,
				       float velocity_rad_s, float kp, float kd,
				       float torque_nm, int32_t duration_ms)
{
	if (!valid_dm_id(id)) {
		return -EINVAL;
	}

	position_rad = app_clampf(position_rad, -12.5f, 12.5f);
	velocity_rad_s = app_clampf(velocity_rad_s,
				    -APP_DM_DEBUG_VEL_LIMIT_RAD_S,
				    APP_DM_DEBUG_VEL_LIMIT_RAD_S);
	kp = app_clampf(kp, 0.0f, APP_DM_DEBUG_KP_LIMIT);
	kd = app_clampf(kd, 0.0f, APP_DM_DEBUG_KD_LIMIT);
	torque_nm = app_clampf(torque_nm, -APP_DM_DEBUG_TORQUE_LIMIT_NM,
			       APP_DM_DEBUG_TORQUE_LIMIT_NM);

	k_spinlock_key_t key = k_spin_lock(&ctx.lock);
	ctx.state.dm[id] = (motor_debug_dm_command_t) {
		.mode = MOTOR_DEBUG_DM_MIT,
		.id = id,
		.position_rad = position_rad,
		.velocity_rad_s = velocity_rad_s,
		.kp = kp,
		.kd = kd,
		.torque_nm = torque_nm,
	};
	ctx.state.dm_deadline_ms[id] = deadline_from_duration(duration_ms);
	k_spin_unlock(&ctx.lock, key);

	return 0;
}

/**
 * @brief 设置 DM4340 关节电机的正弦抖动（wiggle）命令
 *
 * 电机围绕中心位置做正弦往复运动，用于关节松紧度测试和振动响应分析。
 * 实际目标位置 = center_rad + amplitude_rad * sin(2*pi*t/period_ms)
 *
 * 使用 MIT 控制模式实现，kp/kd 决定刚度和阻尼。
 *
 * @param id             关节电机 ID
 * @param center_rad     正弦运动的中心位置（弧度）
 * @param amplitude_rad  振幅（弧度），限制在 [0.001, 0.08] 范围
 * @param period_ms      振动周期（毫秒），限制在 [500, 5000] 范围
 * @param kp             位置刚度增益
 * @param kd             速度阻尼增益
 * @param duration_ms    总持续时间（毫秒），超时后自动停止
 * @return 0 成功，-EINVAL ID 无效
 */
int motor_debug_set_dm_wiggle(uint8_t id, float center_rad,
			      float amplitude_rad, int32_t period_ms, float kp,
			      float kd, int32_t duration_ms)
{
	if (!valid_dm_id(id)) {
		return -EINVAL;
	}

	center_rad = app_clampf(center_rad, -12.5f, 12.5f);
	amplitude_rad = app_clampf(amplitude_rad, 0.001f, 0.08f);
	period_ms = CLAMP(period_ms, 500, 5000);
	kp = app_clampf(kp, 0.0f, APP_DM_DEBUG_KP_LIMIT);
	kd = app_clampf(kd, 0.0f, APP_DM_DEBUG_KD_LIMIT);

	k_spinlock_key_t key = k_spin_lock(&ctx.lock);
	ctx.state.dm[id] = (motor_debug_dm_command_t) {
		.mode = MOTOR_DEBUG_DM_WIGGLE,
		.id = id,
		.position_rad = center_rad,
		.velocity_rad_s = 0.0f,
		.kp = kp,
		.kd = kd,
		.torque_nm = 0.0f,
		.amplitude_rad = amplitude_rad,
		.period_ms = period_ms,
		.start_ms = k_uptime_get(),
	};
	ctx.state.dm_deadline_ms[id] = deadline_from_duration(duration_ms);
	k_spin_unlock(&ctx.lock, key);

	return 0;
}

/**
 * @brief 停止指定 DM4340 关节电机的调试命令
 *
 * 清除该电机的调试状态，并发送零力矩 MIT 命令和零速度命令。
 * 双重发送（MIT + velocity）确保无论电机当前处于哪种模式都能安全停止。
 *
 * @param id 关节电机 ID
 * @return 0 成功，-EINVAL ID 无效，或 CAN 发送错误码
 */
int motor_debug_stop_dm(uint8_t id)
{
	if (!valid_dm_id(id)) {
		return -EINVAL;
	}

	k_spinlock_key_t key = k_spin_lock(&ctx.lock);
	ctx.state.dm[id] = (motor_debug_dm_command_t){ 0 };
	ctx.state.dm_deadline_ms[id] = 0;
	k_spin_unlock(&ctx.lock, key);

	if (ctx.dm_bus != NULL) {
		int first_error = dm4340_send_mit(ctx.dm_bus, id, 0.0f, 0.0f,
						  0.0f, 0.0f, 0.0f);
		const int ret = dm4340_send_velocity(ctx.dm_bus, id, 0.0f);
		if (first_error == 0) {
			first_error = ret;
		}
		return first_error;
	}

	return 0;
}

/**
 * @brief 停止所有电机（轮毂 + 关节）的调试命令
 *
 * 清零全部调试状态，并向所有电机发送零指令。
 * 用于紧急停止（kill switch）或测试结束后的安全清理。
 *
 * @return 0 始终成功（CAN 发送错误被忽略）
 */
int motor_debug_stop_all(void)
{
	k_spinlock_key_t key = k_spin_lock(&ctx.lock);
	memset(&ctx.state, 0, sizeof(ctx.state));
	k_spin_unlock(&ctx.lock, key);

	/* 向四个轮毂电机发送零电流 */
	if (ctx.dji_bus != NULL) {
		(void)dji_m3508_send_group_current(ctx.dji_bus, 0, 0, 0, 0);
	}

	/* 向左右关节电机发送零 MIT 和零速度命令 */
	if (ctx.dm_bus != NULL) {
		(void)dm4340_send_mit(ctx.dm_bus, APP_DM_LEFT_ID, 0.0f, 0.0f,
				      0.0f, 0.0f, 0.0f);
		(void)dm4340_send_mit(ctx.dm_bus, APP_DM_RIGHT_ID, 0.0f, 0.0f,
				      0.0f, 0.0f, 0.0f);
		(void)dm4340_send_velocity(ctx.dm_bus, APP_DM_LEFT_ID, 0.0f);
		(void)dm4340_send_velocity(ctx.dm_bus, APP_DM_RIGHT_ID, 0.0f);
	}

	return 0;
}

/**
 * @brief 获取当前有效的电机调试输出
 *
 * 主控制循环调用此函数获取调试命令。函数内部会：
 *   1. 清除所有已过期的 DM4340 命令
 *   2. 将当前有效的轮毂和关节命令复制到输出结构体
 *   3. 若有任何有效命令，返回 true
 *
 * 此函数是线程安全的，可在任何上下文中调用。
 *
 * @param out 输出结构体指针，存放当前有效的调试命令
 * @return true 有有效命令，false 无有效命令或 out 为 NULL
 */
bool motor_debug_get_output(motor_debug_output_t *out)
{
	if (out == NULL) {
		return false;
	}

	memset(out, 0, sizeof(*out));

	k_spinlock_key_t key = k_spin_lock(&ctx.lock);
	clear_expired_locked(k_uptime_get());

	out->wheel_active = ctx.state.wheel_active;
	if (out->wheel_active) {
		out->wheel_mode = ctx.state.wheel_mode;
		memcpy(out->wheel_current, ctx.state.wheel_current,
		       sizeof(out->wheel_current));
		out->left_wheel_rpm = ctx.state.left_wheel_rpm;
		out->right_wheel_rpm = ctx.state.right_wheel_rpm;
		out->active = true;
	}

	for (uint8_t id = 1; id <= DM4340_MAX_ID; id++) {
		if (ctx.state.dm[id].mode != MOTOR_DEBUG_DM_NONE) {
			out->dm[id] = ctx.state.dm[id];
			out->active = true;
		}
	}
	k_spin_unlock(&ctx.lock, key);

	return out->active;
}

/**
 * @brief 读取 DJI M3508 轮毂电机的反馈数据
 *
 * 返回最近一次 CAN 接收的电机状态（位置、速度、电流等）。
 *
 * @param id  电机 ID
 * @param out 输出结构体指针，存放电机反馈数据
 * @return true 成功读取，false 总线未初始化或 ID 无效
 */
bool motor_debug_get_m3508(uint8_t id, dji_m3508_motor_t *out)
{
	if (ctx.dji_bus == NULL) {
		return false;
	}

	return dji_m3508_get(ctx.dji_bus, id, out);
}

/**
 * @brief 读取 DM4340 关节电机的反馈数据
 *
 * 返回最近一次 CAN 接收的关节电机状态（位置、速度、力矩等）。
 *
 * @param id  关节电机 ID
 * @param out 输出结构体指针，存放电机反馈数据
 * @return true 成功读取，false 总线未初始化或 ID 无效
 */
bool motor_debug_get_dm4340(uint8_t id, dm4340_feedback_t *out)
{
	if (ctx.dm_bus == NULL) {
		return false;
	}

	return dm4340_get(ctx.dm_bus, id, out);
}

/**
 * @brief 打印 DM4340 CAN 总线的接收日志
 *
 * 将最近收到的 CAN 帧转储到控制台，用于调试通信问题。
 */
void motor_debug_dump_dm4340_rx_log(void)
{
	if (ctx.dm_bus != NULL) {
		dm4340_dump_rx_log(ctx.dm_bus);
	}
}

/**
 * @brief 将 DM4340 调试模式枚举转换为可读字符串
 *
 * 用于 shell 输出和日志显示。
 *
 * @param mode 调试模式枚举值
 * @return const char* 模式名称字符串："pos"、"vel"、"mit"、"wiggle" 或 "none"
 */
const char *motor_debug_dm_mode_name(motor_debug_dm_mode_t mode)
{
	switch (mode) {
	case MOTOR_DEBUG_DM_POS_VEL:
		return "pos";
	case MOTOR_DEBUG_DM_VELOCITY:
		return "vel";
	case MOTOR_DEBUG_DM_MIT:
		return "mit";
	case MOTOR_DEBUG_DM_WIGGLE:
		return "wiggle";
	case MOTOR_DEBUG_DM_NONE:
	default:
		return "none";
	}
}
