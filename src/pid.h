/**
 * @file pid.h
 * @brief PID 控制器与通用数值工具函数
 *
 * 本模块提供了嵌入式控制系统中常用的核心算法：
 *
 * 1. PID 控制器（app_pid_t）：
 *    - 标准位置式 PID 控制器，适用于各种闭环控制场景
 *    - 支持积分限幅（anti-windup），防止积分饱和导致超调
 *    - 支持输出限幅，将控制量限制在安全范围内
 *    - 在 Ascento 轮腿机器人中用于平衡控制、速度控制、角度控制等多个环节
 *
 * 2. 一阶低通滤波器（LPF）：
 *    - 基于指数移动平均的一阶 IIR 低通滤波器
 *    - 用于平滑传感器数据（如陀螺仪、编码器速度）中的噪声
 *    - 通过时间常数参数控制滤波截止频率
 *
 * 3. 通用数值工具函数：
 *    - 浮点数限幅（clamp）
 *    - 16位整数限幅（防止溢出）
 *    - 角度归一化到 ±180° 范围
 *
 * @note PID 控制器传递函数：u(t) = Kp*e(t) + Ki*∫e(t)dt + Kd*de(t)/dt
 * @note 所有时间参数单位为秒（s），角度单位为弧度（rad）或度（°），视具体函数而定
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief PID 控制器结构体
 *
 * 实现标准位置式 PID 控制算法。
 * 使用流程：先调用 app_pid_init() 初始化参数，然后在控制循环中
 * 周期性调用 app_pid_update() 计算控制输出。
 *
 * 控制律：output = Kp * error + Ki * integral + Kd * (error - previous_error) / dt
 */
typedef struct {
	float kp;              /**< 比例增益（Proportional gain）：决定对当前误差的响应强度 */
	float ki;              /**< 积分增益（Integral gain）：消除稳态误差，过大会导致超调和振荡 */
	float kd;              /**< 微分增益（Derivative gain）：抑制误差变化率，提供阻尼效果 */
	float integral_limit;  /**< 积分项限幅值：防止积分累积过大（anti-windup），绝对值上限 */
	float output_limit;    /**< 输出限幅值：将最终控制输出限制在 [-output_limit, +output_limit] 范围内 */
	float integral;        /**< 积分累加值（内部状态）：累积历史误差之和 */
	float previous_error;  /**< 上一次的误差值（内部状态）：用于计算微分项 */
	bool initialized;      /**< 是否已初始化标志：首次调用 update 时用于跳过微分项的突变 */
} app_pid_t;

/**
 * @brief 初始化 PID 控制器
 *
 * 设置 PID 三个增益参数和限幅值，并清零内部状态。
 * 在控制循环开始前调用一次。
 *
 * @param[out] pid             PID 控制器结构体指针
 * @param[in]  kp              比例增益
 * @param[in]  ki              积分增益
 * @param[in]  kd              微分增益
 * @param[in]  integral_limit  积分限幅值（正值，内部会取绝对值使用）
 * @param[in]  output_limit    输出限幅值（正值，内部会取绝对值使用）
 */
void app_pid_init(app_pid_t *pid, float kp, float ki, float kd,
		  float integral_limit, float output_limit);

/**
 * @brief 重置 PID 控制器内部状态
 *
 * 清零积分累加值和历史误差，但保留增益参数不变。
 * 在控制模式切换或需要重新开始积分时调用。
 *
 * @param[in,out] pid  PID 控制器结构体指针
 */
void app_pid_reset(app_pid_t *pid);

/**
 * @brief PID 控制器更新（计算一步控制输出）
 *
 * 根据当前误差和时间步长，计算 PID 控制输出。
 * 内部自动处理积分累加、积分限幅和输出限幅。
 *
 * 首次调用时，微分项为 0（因为没有历史误差）。
 *
 * @param[in,out] pid    PID 控制器结构体指针
 * @param[in]     error  当前误差值（设定值 - 实际值）
 * @param[in]     dt_s   时间步长，单位：秒（s），即控制周期
 * @return 控制输出值（已限幅），可直接用于驱动执行器
 */
float app_pid_update(app_pid_t *pid, float error, float dt_s);

/**
 * @brief 一阶低通滤波器更新（一步滤波计算）
 *
 * 实现一阶 IIR 低通滤波器，基于指数移动平均算法：
 *   output = state + (input - state) * dt / (time_constant + dt)
 *
 * time_constant 越大，滤波越强（越平滑），但响应越慢。
 * 适用于平滑含噪声的传感器信号。
 *
 * @param[in]     input            当前输入采样值
 * @param[in,out] state            滤波器状态（上一次的输出值），会被更新
 * @param[in]     time_constant_s  滤波时间常数，单位：秒（s）。截止频率 f_c = 1/(2π·τ)
 * @param[in]     dt_s             采样时间步长，单位：秒（s）
 * @return 滤波后的输出值
 */
float app_lpf_update(float input, float *state, float time_constant_s,
		     float dt_s);

/**
 * @brief 浮点数限幅（clamp）
 *
 * 将浮点数值限制在 [min_value, max_value] 范围内。
 * 如果 value < min_value 返回 min_value，
 * 如果 value > max_value 返回 max_value，否则返回 value。
 *
 * @param[in] value      输入值
 * @param[in] min_value  下限
 * @param[in] max_value  上限
 * @return 限幅后的值
 */
float app_clampf(float value, float min_value, float max_value);

/**
 * @brief 16 位有符号整数限幅
 *
 * 将 32 位整数值限制到 16 位有符号整数范围内。
 * 用于将浮点运算结果安全转换为 int16_t（如 CAN 总线电流指令）。
 * 溢出时会被钳位到 INT16_MIN 或 INT16_MAX。
 *
 * @param[in] value      输入值（32 位）
 * @param[in] min_value  下限（16 位）
 * @param[in] max_value  上限（16 位）
 * @return 限幅后的 16 位有符号整数
 */
int16_t app_clamp_i16(int32_t value, int16_t min_value, int16_t max_value);

/**
 * @brief 角度归一化到 ±180° 范围
 *
 * 将任意角度值归一化到 (-180°, +180°] 范围内。
 * 例如：190° -> -170°，-200° -> 160°。
 * 常用于处理 IMU 航向角或关节角度的周期性边界问题。
 *
 * @param[in] angle_deg  输入角度，单位：度（°）
 * @return 归一化后的角度，范围 (-180°, +180°]
 */
float app_wrap_pm180(float angle_deg);
