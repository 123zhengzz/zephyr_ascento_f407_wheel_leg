/**
 * @file motor_debug.h
 * @brief 电机调试与测试模块
 *
 * 本模块提供统一的电机调试接口，用于在开发和调试阶段单独或批量测试电机。
 * 支持所有类型的电机控制指令（电流、转速、位置-速度、MIT 阻抗控制等），
 * 并支持定时自动停止功能，防止电机在调试过程中持续运动造成危险。
 *
 * 主要功能：
 *   - 轮毂电机（DJI M3508）调试：电流控制、转速控制、左右轮配对控制
 *   - 关节电机（DM4340）调试：使能/失能、零位保存、寄存器读取、
 *     位置-速度控制、速度控制、MIT 阻抗控制、摆动测试
 *   - 支持每条指令设定持续时间（duration），超时自动停止电机
 *   - 提供电机状态查询接口，可实时读取反馈数据
 *   - 提供接收日志转储功能，用于调试 CAN 通信
 *
 * 所有调试指令通过 shell 命令或上位机软件调用，
 * 适合在机器人组装、标定和故障排查时使用。
 *
 * @warning 调试指令会直接控制电机运动，请确保使用时周围环境安全。
 * @note "wiggle"（摆动）模式：电机围绕中心位置做正弦往复运动，用于验证关节电机响应。
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "dji_m3508.h"
#include "dm4340.h"

/**
 * @brief DM4340 关节电机调试控制模式枚举
 *
 * 定义关节电机可用的各种调试控制方式。
 * 通过 shell 命令选择不同模式来测试电机的不同控制特性。
 */
typedef enum {
	MOTOR_DEBUG_DM_NONE = 0,      /**< 无控制指令（电机不执行任何动作） */
	MOTOR_DEBUG_DM_POS_VEL,       /**< 位置-速度模式：设定目标位置和速度限制 */
	MOTOR_DEBUG_DM_VELOCITY,      /**< 速度模式：以指定速度持续旋转 */
	MOTOR_DEBUG_DM_MIT,           /**< MIT 阻抗控制模式：同时设定位置、速度、Kp、Kd、扭矩 */
	MOTOR_DEBUG_DM_WIGGLE,        /**< 摆动测试模式：围绕中心位置做正弦往复运动 */
} motor_debug_dm_mode_t;

/**
 * @brief M3508 轮毂电机调试控制模式枚举
 *
 * 定义轮毂电机可用的调试控制方式。
 */
typedef enum {
	MOTOR_DEBUG_WHEEL_NONE = 0,   /**< 无控制指令（电机不执行任何动作） */
	MOTOR_DEBUG_WHEEL_CURRENT,    /**< 电流控制模式：直接设定电流值（mA） */
	MOTOR_DEBUG_WHEEL_RPM,        /**< 转速控制模式：设定目标转速（RPM） */
} motor_debug_wheel_mode_t;

/**
 * @brief DM4340 关节电机调试指令结构体
 *
 * 保存一条关节电机调试指令的全部参数。
 * 每个关节电机最多同时存在一条调试指令。
 */
typedef struct {
	motor_debug_dm_mode_t mode;    /**< 当前控制模式 */
	uint8_t id;                    /**< 目标电机 ID（0~15） */
	float position_rad;            /**< 目标位置，单位：弧度（rad），用于位置-速度和 MIT 模式 */
	float velocity_rad_s;          /**< 目标/前馈速度，单位：弧度/秒（rad/s） */
	float kp;                      /**< MIT 模式下的位置比例增益（刚度） */
	float kd;                      /**< MIT 模式下的速度微分增益（阻尼） */
	float torque_nm;               /**< MIT 模式下的前馈扭矩，单位：牛顿米（N·m） */
	float amplitude_rad;           /**< 摆动模式的振幅，单位：弧度（rad） */
	int32_t period_ms;             /**< 摆动模式的周期，单位：毫秒（ms） */
	int64_t start_ms;              /**< 指令开始执行的时间戳，单位：ms（用于计算超时和正弦相位） */
} motor_debug_dm_command_t;

/**
 * @brief 电机调试模块输出状态结构体
 *
 * 保存调试模块的全局状态，包括轮毂电机和关节电机的控制指令。
 * 由 motor_debug 主循环周期性读取并下发到各电机驱动。
 */
typedef struct {
	bool active;                                   /**< 调试模块是否处于活动状态（有指令在执行） */
	bool wheel_active;                             /**< 轮毂电机调试是否处于活动状态 */
	motor_debug_wheel_mode_t wheel_mode;           /**< 轮毂电机当前调试模式 */
	int16_t wheel_current[DJI_M3508_MAX_ID + 1];  /**< 各轮毂电机的设定电流，单位：mA，按 CAN ID 索引 */
	int32_t left_wheel_rpm;                        /**< 左轮目标转速，单位：RPM */
	int32_t right_wheel_rpm;                       /**< 右轮目标转速，单位：RPM */
	motor_debug_dm_command_t dm[DM4340_MAX_ID + 1]; /**< 各关节电机的调试指令，按电机 ID 索引 */
} motor_debug_output_t;

/**
 * @brief 初始化电机调试模块
 *
 * 绑定 M3508 和 DM4340 的总线驱动实例，使调试模块可以向电机发送指令。
 * 在系统启动时调用一次即可。
 *
 * @param[in] dji_bus  DJI M3508 总线驱动指针
 * @param[in] dm_bus   DM4340 总线驱动指针
 */
void motor_debug_init(dji_m3508_bus_t *dji_bus, dm4340_bus_t *dm_bus);

/* ========== 轮毂电机（M3508）调试接口 ========== */

/**
 * @brief 设置单个轮毂电机的电流
 *
 * 以指定电流驱动单个 M3508 电机，持续指定时间后自动停止。
 *
 * @param[in] id           目标电机 CAN ID
 * @param[in] current      设定电流，单位：mA（正值正转，负值反转）
 * @param[in] duration_ms  持续时间，单位：ms（超时后自动停止，-1 表示无限持续）
 * @return 0 表示成功，负值表示失败
 */
int motor_debug_set_wheel_current(uint8_t id, int32_t current,
				  int32_t duration_ms);

/**
 * @brief 设置左右轮毂电机配对电流
 *
 * 同时设定左右两个轮子的电流，适用于快速测试双轮平衡或驱动。
 * 左右轮的 CAN ID 由内部配置决定。
 *
 * @param[in] left_current   左轮设定电流，单位：mA
 * @param[in] right_current  右轮设定电流，单位：mA
 * @param[in] duration_ms    持续时间，单位：ms（-1 表示无限持续）
 * @return 0 表示成功，负值表示失败
 */
int motor_debug_set_wheel_pair(int32_t left_current, int32_t right_current,
			       int32_t duration_ms);

/**
 * @brief 设置单个轮毂电机的转速
 *
 * 以指定转速驱动单个 M3508 电机，持续指定时间后自动停止。
 *
 * @param[in] id           目标电机 CAN ID
 * @param[in] rpm          目标转速，单位：RPM（正值正转，负值反转）
 * @param[in] duration_ms  持续时间，单位：ms（-1 表示无限持续）
 * @return 0 表示成功，负值表示失败
 */
int motor_debug_set_wheel_rpm(uint8_t id, int32_t rpm, int32_t duration_ms);

/**
 * @brief 停止所有轮毂电机
 *
 * 将所有轮毂电机的设定电流置零，立即停止运动。
 *
 * @return 0 表示成功，负值表示失败
 */
int motor_debug_stop_wheels(void);

/* ========== 关节电机（DM4340）调试接口 ========== */

/**
 * @brief 使能指定关节电机
 *
 * @param[in] id  目标电机 ID（0~15）
 * @return 0 表示成功，负值表示失败
 */
int motor_debug_dm_enable(uint8_t id);

/**
 * @brief 失能指定关节电机
 *
 * @param[in] id  目标电机 ID（0~15）
 * @return 0 表示成功，负值表示失败
 */
int motor_debug_dm_disable(uint8_t id);

/**
 * @brief 保存关节电机当前位置为零位
 *
 * @param[in] id  目标电机 ID（0~15）
 * @return 0 表示成功，负值表示失败
 */
int motor_debug_dm_save_zero(uint8_t id);

/**
 * @brief 读取关节电机内部寄存器
 *
 * @param[in]  id   目标电机 ID（0~15）
 * @param[in]  rid  寄存器 ID
 * @param[out] out  输出缓冲区，接收寄存器读取结果
 * @return 0 表示成功，负值表示失败
 */
int motor_debug_dm_read_reg(uint8_t id, uint8_t rid,
			    dm4340_param_response_t *out);

/**
 * @brief 以位置-速度模式控制关节电机
 *
 * @param[in] id              目标电机 ID（0~15）
 * @param[in] position_rad    目标位置，单位：弧度（rad）
 * @param[in] velocity_rad_s  速度限制，单位：弧度/秒（rad/s）
 * @param[in] duration_ms     持续时间，单位：ms（-1 表示无限持续）
 * @return 0 表示成功，负值表示失败
 */
int motor_debug_set_dm_pos_vel(uint8_t id, float position_rad,
			       float velocity_rad_s, int32_t duration_ms);

/**
 * @brief 以速度模式控制关节电机
 *
 * @param[in] id             目标电机 ID（0~15）
 * @param[in] velocity_rad_s 目标速度，单位：弧度/秒（rad/s）
 * @param[in] duration_ms    持续时间，单位：ms（-1 表示无限持续）
 * @return 0 表示成功，负值表示失败
 */
int motor_debug_set_dm_velocity(uint8_t id, float velocity_rad_s,
				int32_t duration_ms);

/**
 * @brief 以 MIT 阻抗模式控制关节电机（带限位归一化）
 *
 * 参数会根据 dm4340_mit_limit_t 中的限位进行归一化编码。
 *
 * @param[in] id             目标电机 ID（0~15）
 * @param[in] position_rad   目标位置，单位：弧度（rad）
 * @param[in] velocity_rad_s 速度前馈，单位：弧度/秒（rad/s）
 * @param[in] kp             位置比例增益（刚度）
 * @param[in] kd             速度微分增益（阻尼）
 * @param[in] torque_nm      前馈扭矩，单位：牛顿米（N·m）
 * @param[in] duration_ms    持续时间，单位：ms（-1 表示无限持续）
 * @return 0 表示成功，负值表示失败
 */
int motor_debug_set_dm_mit(uint8_t id, float position_rad,
			   float velocity_rad_s, float kp, float kd,
			   float torque_nm, int32_t duration_ms);

/**
 * @brief 以 MIT 阻抗模式控制关节电机（原始数据，不经过限位归一化）
 *
 * 与 motor_debug_set_dm_mit() 类似，但参数直接编码为原始字节发送，
 * 不经过 MIT 限位归一化处理。适用于已知电机限位并手动归一化的场景。
 *
 * @param[in] id             目标电机 ID（0~15）
 * @param[in] position_rad   目标位置，单位：弧度（rad）
 * @param[in] velocity_rad_s 速度前馈，单位：弧度/秒（rad/s）
 * @param[in] kp             位置比例增益（刚度）
 * @param[in] kd             速度微分增益（阻尼）
 * @param[in] torque_nm      前馈扭矩，单位：牛顿米（N·m）
 * @param[in] duration_ms    持续时间，单位：ms（-1 表示无限持续）
 * @return 0 表示成功，负值表示失败
 */
int motor_debug_set_dm_mit_raw(uint8_t id, float position_rad,
				       float velocity_rad_s, float kp, float kd,
				       float torque_nm, int32_t duration_ms);

/**
 * @brief 关节电机摆动测试模式
 *
 * 使关节电机围绕指定中心位置做正弦往复运动。
 * 运动规律：θ(t) = center + amplitude * sin(2π * t / period)
 * 用于快速验证关节电机的响应特性和机械结构。
 *
 * @param[in] id            目标电机 ID（0~15）
 * @param[in] center_rad    摆动中心位置，单位：弧度（rad）
 * @param[in] amplitude_rad 摆动振幅，单位：弧度（rad）
 * @param[in] period_ms     摆动周期，单位：毫秒（ms）
 * @param[in] kp            MIT 位置比例增益（刚度）
 * @param[in] kd            MIT 速度微分增益（阻尼）
 * @param[in] duration_ms   总持续时间，单位：ms（-1 表示无限持续）
 * @return 0 表示成功，负值表示失败
 */
int motor_debug_set_dm_wiggle(uint8_t id, float center_rad,
			      float amplitude_rad, int32_t period_ms, float kp,
			      float kd, int32_t duration_ms);

/**
 * @brief 停止指定关节电机的调试指令
 *
 * @param[in] id  目标电机 ID（0~15）
 * @return 0 表示成功，负值表示失败
 */
int motor_debug_stop_dm(uint8_t id);

/**
 * @brief 停止所有电机的调试指令
 *
 * 停止全部轮毂电机和关节电机的调试控制，恢复空闲状态。
 *
 * @return 0 表示成功，负值表示失败
 */
int motor_debug_stop_all(void);

/* ========== 电机状态查询接口 ========== */

/**
 * @brief 获取调试模块的当前输出状态快照
 *
 * @param[out] out  输出缓冲区，接收当前调试输出状态
 * @return true 表示成功获取，false 表示模块未初始化
 */
bool motor_debug_get_output(motor_debug_output_t *out);

/**
 * @brief 获取指定 M3508 轮毂电机的最新状态
 *
 * @param[in]  id   目标电机 CAN ID
 * @param[out] out  输出缓冲区，接收电机状态数据
 * @return true 表示成功获取，false 表示电机 ID 无效
 */
bool motor_debug_get_m3508(uint8_t id, dji_m3508_motor_t *out);

/**
 * @brief 获取指定 DM4340 关节电机的最新反馈数据
 *
 * @param[in]  id   目标电机 ID（0~15）
 * @param[out] out  输出缓冲区，接收电机反馈数据
 * @return true 表示成功获取，false 表示电机 ID 无效
 */
bool motor_debug_get_dm4340(uint8_t id, dm4340_feedback_t *out);

/**
 * @brief 转储 DM4340 CAN 接收日志到控制台
 *
 * 将最近收到的 CAN 帧记录打印到日志，用于调试通信问题。
 */
void motor_debug_dump_dm4340_rx_log(void);

/**
 * @brief 获取 DM4340 调试模式的名称字符串
 *
 * 将 motor_debug_dm_mode_t 枚举值转换为可读的字符串名称，
 * 用于 shell 命令输出和日志显示。
 *
 * @param[in] mode  调试模式枚举值
 * @return 模式名称字符串（如 "NONE"、"POS_VEL"、"MIT" 等）
 */
const char *motor_debug_dm_mode_name(motor_debug_dm_mode_t mode);
