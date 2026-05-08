#pragma once
/**
 * @file control.h
 * @brief 机器人控制系统接口（LQR 控制器路径）
 *
 * 定义控制系统的状态结构体和使能/状态查询 API。
 * 实际平衡算法由 ascento_balance.c 的 LQR 控制器完成，
 * 本文件仅提供状态管理和发布接口。
 */

#include <stdbool.h>
#include <stdint.h>

#include "bmi088.h"
#include "dji_m3508.h"

/**
 * @brief 机器人运动方向枚举
 *
 * 定义机器人整体运动意图，由上层（如遥控器、自主导航）设定。
 */
typedef enum {
	ROBOT_FORWARD = 0, /**< 前进 */
	ROBOT_BACK = 1,    /**< 后退 */
	ROBOT_RIGHT = 2,   /**< 右转 */
	ROBOT_LEFT = 3,    /**< 左转 */
	ROBOT_STOP = 4,    /**< 停止 */
	ROBOT_JUMP = 5,    /**< 跳跃 */
} robot_motion_t;

/**
 * @brief 控制输出结构体
 *
 * 每个控制周期由控制器填充，包含发送给底层电机驱动的指令。
 */
typedef struct {
	bool wheels_enabled;  /**< 轮子是否使能（false 时轮子电流为零，自由转动） */
	bool joints_enabled;  /**< 关节是否使能（false 时关节电机断电） */
	int16_t left_wheel_current;  /**< 左轮目标电流，单位：毫安（mA），正值为前进方向 */
	int16_t right_wheel_current; /**< 右轮目标电流，单位：毫安（mA），正值为前进方向 */
	float left_joint_position_rad;  /**< 左腿关节目标角度，单位：弧度（rad） */
	float right_joint_position_rad; /**< 右腿关节目标角度，单位：弧度（rad） */
	float joint_velocity_limit_rad_s; /**< 关节角速度限幅，单位：弧度/秒（rad/s） */
} control_output_t;

/**
 * @brief 控制状态结构体
 *
 * 包含控制系统的全部运行时状态，用于调试、遥测和状态监控。
 * 可通过 control_get_status() 获取。
 */
typedef struct {
	bool enable_request;  /**< 控制系统使能请求（true = 已使能） */
	bool wheels_enabled;  /**< 轮子当前是否已使能（经故障保护后的实际状态） */
	bool faulted;         /**< 是否处于故障状态（如倾角过大触发保护） */
	int height;           /**< 当前腿部高度档位 */
	float joy_x;          /**< 摇杆 X 轴输入，范围 -1.0 ~ +1.0 */
	float joy_y;          /**< 摇杆 Y 轴输入，范围 -1.0 ~ +1.0 */
	robot_motion_t motion; /**< 当前运动模式 */
	float pitch_deg;      /**< 机体俯仰角，单位：度（deg），正值为前倾 */
	float pitch_rate_dps; /**< 机体俯仰角速度，单位：度/秒（deg/s） */
	float roll_deg;       /**< 机体横滚角，单位：度（deg） */
	float yaw_deg;        /**< 机体偏航角，单位：度（deg） */
	float distance_rad;   /**< 累计行进距离（轮子编码器积分），单位：弧度（rad） */
	float speed_rad_s;    /**< 轮子平均角速度，单位：弧度/秒（rad/s） */
	float lqr_output;     /**< LQR 平衡控制器输出（调试用） */
	float yaw_output;     /**< 偏航控制器输出（调试用） */
	float left_joint_position_rad;  /**< 左腿关节当前角度，单位：弧度（rad） */
	float right_joint_position_rad; /**< 右腿关节当前角度，单位：弧度（rad） */
	int16_t left_wheel_current;  /**< 左轮当前电流，单位：毫安（mA） */
	int16_t right_wheel_current; /**< 右轮当前电流，单位：毫安（mA） */
	int jump_phase;       /**< 跳跃阶段（0=未跳跃，1=蹲下，2=起跳，3=落地等） */
} control_status_t;

/**
 * @brief 初始化控制系统
 *
 * 初始化内部状态、控制器参数、电机通信等。应在系统启动时调用一次。
 */
void control_init(void);

/**
 * @brief 设置控制系统使能/禁用
 * @param enable true = 使能（电机上电），false = 禁用（电机断电）
 */
void control_set_enable(bool enable);

/**
 * @brief 停止当前运动
 *
 * 将运动模式重置为 ROBOT_STOP，停止前进和转向。
 */
void control_stop_motion(void);

/**
 * @brief 获取控制系统使能请求状态
 * @return true = 已请求使能，false = 未使能
 */
bool control_get_enable_request(void);

/**
 * @brief 获取控制系统的当前状态快照
 * @param[out] status 指向状态结构体的指针，函数将填充当前状态数据
 */
void control_get_status(control_status_t *status);

/**
 * @brief 发布控制状态（通过通信接口发送给上位机）
 * @param[in] status 指向要发布的状态结构体
 */
void control_publish_status(const control_status_t *status);
