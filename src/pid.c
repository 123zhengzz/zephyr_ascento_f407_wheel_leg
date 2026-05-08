/**
 * @file pid.c
 * @brief PID 控制器与工具函数模块 (PID Controller and Utility Functions)
 *
 * 本模块为轮腿机器人 (Ascento 风格, STM32F407) 提供核心控制算法：
 *
 * 1. PID 控制器 (app_pid_update)
 *    - 标准 PID 三项：比例 (P) + 积分 (I) + 微分 (D)
 *    - 抗积分饱和：积分项限幅 (integral clamping)，防止 windup
 *    - 微分项作用于误差 (derivative-on-error)，首次调用初始化避免微分冲击 (kick)
 *    - 输出限幅：防止执行器饱和
 *
 * 2. 一阶低通滤波器 (app_lpf_update)
 *    - 一阶 IIR 滤波器，用于传感器信号平滑
 *    - 离散化公式：y[n] = y[n-1] + alpha * (x[n] - y[n-1])
 *
 * 3. 工具函数
 *    - 浮点数限幅 (app_clampf)
 *    - int16 安全限幅 (app_clamp_i16)
 *    - 角度归一化到 [-180, 180] (app_wrap_pm180)
 */

#include "pid.h"

#include <math.h>

/**
 * @brief 初始化 PID 控制器
 *
 * 设置 PID 增益、积分限幅和输出限幅，并调用 reset 清零内部状态。
 * integral_limit 和 output_limit 取绝对值，确保为非负数。
 *
 * @param pid              PID 控制器结构体指针
 * @param kp               比例增益 (Proportional gain)
 * @param ki               积分增益 (Integral gain)
 * @param kd               微分增益 (Derivative gain)
 * @param integral_limit   积分项绝对值上限，防止积分饱和 (anti-windup)
 * @param output_limit     输出绝对值上限，防止执行器饱和
 */
void app_pid_init(app_pid_t *pid, float kp, float ki, float kd,
		  float integral_limit, float output_limit)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->integral_limit = fabsf(integral_limit);
	pid->output_limit = fabsf(output_limit);
	app_pid_reset(pid);
}

/**
 * @brief 重置 PID 控制器内部状态
 *
 * 清零积分累积量和上一次误差记录，并将 initialized 标志设为 false。
 * 下次调用 app_pid_update() 时会重新初始化微分项，避免因
 * 残留的 previous_error 产生微分冲击 (derivative kick)。
 *
 * 通常在模式切换或控制器启用/禁用时调用。
 *
 * @param pid PID 控制器结构体指针
 */
void app_pid_reset(app_pid_t *pid)
{
	pid->integral = 0.0f;
	pid->previous_error = 0.0f;
	pid->initialized = false;
}

/**
 * @brief 执行一次 PID 控制器更新
 *
 * PID 算法详细说明：
 *
 * 输出公式：u(t) = Kp * e(t) + Ki * integral(e(t)*dt) + Kd * de(t)/dt
 *
 * 各项说明：
 *   - 比例项 (P)：Kp * error
 *     直接对当前误差做出响应，误差越大输出越大。
 *     Kp 过大会导致振荡，过小会导致响应缓慢。
 *
 *   - 积分项 (I)：Ki * sum(error * dt)
 *     累积历史误差，消除稳态误差。
 *     采用积分限幅 (integral clamping) 防止积分饱和 (anti-windup)：
 *     当误差长时间存在时，积分不会无限增长。
 *
 *   - 微分项 (D)：Kd * (error - prev_error) / dt
 *     对误差变化率做出响应，提供阻尼作用，减少超调。
 *     采用"误差的微分"（derivative-on-error）而非"输出的微分"，
 *     首次调用时将 previous_error 初始化为当前 error，
 *     避免设定值突变时产生微分冲击 (derivative kick)。
 *
 * 输出限幅：最终输出被限制在 [-output_limit, output_limit] 范围内。
 *
 * @param pid    PID 控制器结构体指针（包含内部状态）
 * @param error  当前误差值（设定值 - 测量值）
 * @param dt_s   时间步长（秒），若 <= 0 则默认为 0.001s
 * @return float PID 输出值，已限幅
 */
float app_pid_update(app_pid_t *pid, float error, float dt_s)
{
	/* 防御性检查：时间步长不能为零或负数，避免除零 */
	if (dt_s <= 0.0f) {
		dt_s = 0.001f;
	}

	/* 首次调用初始化：将 previous_error 设为当前 error，
	 * 避免微分项因旧值产生突变 (derivative kick) */
	if (!pid->initialized) {
		pid->previous_error = error;
		pid->initialized = true;
	}

	/* 积分项：梯形法近似积分，I += error * dt */
	pid->integral += error * dt_s;

	/* 抗积分饱和 (anti-windup)：积分项限幅
	 * 防止长时间误差导致积分量过大，使控制器输出饱和 */
	pid->integral = app_clampf(pid->integral, -pid->integral_limit,
				   pid->integral_limit);

	/* 微分项：误差的差分除以时间步长
	 * dE/dt = (error - previous_error) / dt */
	const float derivative = (error - pid->previous_error) / dt_s;
	pid->previous_error = error;

	/* PID 输出 = P + I + D */
	const float output = pid->kp * error + pid->ki * pid->integral +
			     pid->kd * derivative;

	/* 输出限幅：防止执行器（电机）饱和 */
	return app_clampf(output, -pid->output_limit, pid->output_limit);
}

/**
 * @brief 一阶低通滤波器 (Low-Pass Filter) 更新
 *
 * 采用一阶 IIR（无限脉冲响应）滤波器实现，用于传感器信号平滑。
 *
 * 数学原理：
 *   连续域传递函数：H(s) = 1 / (tau*s + 1)
 *   其中 tau 为时间常数，tau 越大滤波越强（越平滑），但延迟也越大。
 *
 * 离散化公式（双线性变换的简化形式，即指数移动平均 EMA）：
 *   alpha = dt / (tau + dt)
 *   y[n] = y[n-1] + alpha * (x[n] - y[n-1])
 *        = (1 - alpha) * y[n-1] + alpha * x[n]
 *
 * alpha 的含义：
 *   - alpha 接近 1（dt >> tau）：输出快速跟随输入，几乎无滤波
 *   - alpha 接近 0（dt << tau）：输出变化缓慢，滤波效果强
 *
 * 典型应用：滤除 IMU 陀螺仪噪声、关节力矩抖动等高频干扰。
 *
 * @param input           当前输入信号值（如传感器读数）
 * @param state           指向滤波器状态变量的指针（内部保存上一次输出）
 *                        首次调用时应将 *state 初始化为首次输入值或 0
 * @param time_constant_s 时间常数 tau（秒），越大滤波越强
 * @param dt_s            采样时间步长（秒）
 * @return float          滤波后的输出值
 */
float app_lpf_update(float input, float *state, float time_constant_s,
		     float dt_s)
{
	/* 参数无效时直接直通，不做滤波 */
	if (time_constant_s <= 0.0f || dt_s <= 0.0f) {
		*state = input;
		return input;
	}

	/* 计算滤波系数 alpha = dt / (tau + dt)
	 * alpha 范围 (0, 1)，决定新输入的权重 */
	const float alpha = dt_s / (time_constant_s + dt_s);

	/* 一阶 IIR 滤波：y[n] = y[n-1] + alpha * (x[n] - y[n-1])
	 * 等价于 y[n] = (1-alpha)*y[n-1] + alpha*x[n] */
	*state += alpha * (input - *state);
	return *state;
}

/**
 * @brief 浮点数限幅（clamp）
 *
 * 将浮点值限制在 [min_value, max_value] 范围内。
 * 广泛用于 PID 输出限幅、传感器值范围检查等。
 *
 * @param value     输入值
 * @param min_value 下限
 * @param max_value 上限
 * @return float    限幅后的值
 */
float app_clampf(float value, float min_value, float max_value)
{
	if (value < min_value) {
		return min_value;
	}
	if (value > max_value) {
		return max_value;
	}
	return value;
}

/**
 * @brief int32 到 int16 的安全限幅
 *
 * 将 int32 值限制在 int16 范围 [-32768, 32767] 内，防止溢出截断。
 * 常用于将计算结果转换为 CAN 协议中 16 位电流/速度指令。
 *
 * @param value     输入值（int32）
 * @param min_value 下限（通常为 INT16_MIN 或自定义范围）
 * @param max_value 上限（通常为 INT16_MAX 或自定义范围）
 * @return int16_t  限幅并转换后的值
 */
int16_t app_clamp_i16(int32_t value, int16_t min_value, int16_t max_value)
{
	if (value < min_value) {
		return min_value;
	}
	if (value > max_value) {
		return max_value;
	}
	return (int16_t)value;
}

/**
 * @brief 角度归一化到 [-180, 180] 度范围
 *
 * 将任意角度值归一化到 (-180, 180] 度范围内。
 * 使用 while 循环逐次加减 360 度，适用于任意大小的角度值。
 *
 * 典型应用：
 *   - IMU 航向角归一化
 *   - 关节角度差值计算
 *   - PID 误差计算前的角度预处理
 *
 * @param angle_deg 输入角度（度），可为任意值
 * @return float    归一化后的角度（度），范围 (-180, 180]
 */
float app_wrap_pm180(float angle_deg)
{
	while (angle_deg > 180.0f) {
		angle_deg -= 360.0f;
	}
	while (angle_deg < -180.0f) {
		angle_deg += 360.0f;
	}
	return angle_deg;
}
