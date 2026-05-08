/**
 * @file ascento_balance.h
 * @brief Ascento 风格轮腿机器人的模型化平衡控制器
 *
 * 本文件实现了基于物理模型的平衡控制器，核心思路源自 Ascento 机器人。
 * 控制器使用 LQR（线性二次调节器）方法，基于机器人的物理参数
 * （质量、惯量、轮半径、腿长等）计算最优反馈增益。
 *
 * 工作原理：
 *   1. 将机器人建模为倒立摆 + 轮子系统（5 个状态变量）
 *   2. 根据当前腿部长度动态计算 LQR 增益（增益随腿长变化）
 *   3. 增益通过多项式拟合（c0, c1, c2, c3）预计算，运行时查表
 *   4. 输出轮子扭矩指令，实现自平衡 + 前进/转向控制
 *   5. 包含静摩擦补偿、故障保护、偏航同步等功能
 *
 * 参数存储：
 *   - 默认参数通过 #define 宏定义（编译时确定）
 *   - 运行时可通过 NVS（非易失性存储）持久化自定义参数
 *   - ascento_balance_save_params() / reset_params() 管理持久化
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "bmi088.h"   /* IMU 传感器数据类型 */
#include "dji_m3508.h" /* DJI M3508 电机数据类型 */

/**
 * @brief Ascento 平衡控制器参数结构体
 *
 * 包含机器人的物理参数、LQR 增益、运行时可调参数等。
 * 物理参数在初始化时设定，运行时通常不变；
 * LQR 增益和保护阈值可在运行时调整。
 */
typedef struct {
	bool calibrated; /**< 是否已完成校准（true = 参数有效） */

	/* ========== 机器人物理参数（几何与质量） ========== */

	float wheel_radius_m;    /**< 轮子半径，单位：米（m） */
	float wheel_base_m;      /**< 左右轮距（轮距），单位：米（m） */
	float total_mass_kg;     /**< 机器人总质量，单位：千克（kg） */
	float body_com_height_m; /**< 质心高度（从轮轴到质心的距离），单位：米（m） */
	float body_com_forward_offset_m; /**< 质心前后偏移（正=前倾方向），单位：米（m） */

	/* ========== 转动惯量参数 ========== */

	float body_pitch_inertia_kg_m2; /**< 机体绕俯仰轴的转动惯量，单位：kg*m^2 */
	float wheel_inertia_kg_m2;      /**< 轮子绕转轴的转动惯量，单位：kg*m^2 */

	/* ========== 电机电流到扭矩的转换系数 ========== */

	float current_ma_to_wheel_torque_nm;           /**< 默认电流-扭矩转换系数，单位：N*m/mA */
	float left_current_ma_to_wheel_torque_nm;  /**< 左轮专用电流-扭矩系数（补偿个体差异），单位：N*m/mA */
	float right_current_ma_to_wheel_torque_nm; /**< 右轮专用电流-扭矩系数（补偿个体差异），单位：N*m/mA */

	/* ========== 腿部几何参数（关节角度 <-> 腿长映射） ========== */

	float leg_length_min_m;     /**< 腿部最短长度，单位：米（m） */
	float leg_length_max_m;     /**< 腿部最长长度，单位：米（m） */
	float leg_length_default_m; /**< 腿部默认长度（初始站立高度），单位：米（m） */

	/* 关节角度与腿长的对应关系（用于关节角 -> 腿长的线性映射） */
	float left_joint_at_min_leg_rad;  /**< 左腿在最短腿长时的关节角度，单位：弧度（rad） */
	float left_joint_at_max_leg_rad;  /**< 左腿在最长腿长时的关节角度，单位：弧度（rad） */
	float right_joint_at_min_leg_rad; /**< 右腿在最短腿长时的关节角度，单位：弧度（rad） */
	float right_joint_at_max_leg_rad; /**< 右腿在最长腿长时的关节角度，单位：弧度（rad） */

	/* ========== LQR 状态反馈增益（固定部分） ========== */

	float k_pitch;     /**< 俯仰角反馈增益（平衡主回路），单位：N*m/rad */
	float k_pitch_rate; /**< 俯仰角速度反馈增益（阻尼），单位：N*m/(rad/s) */
	float k_position;  /**< 位置反馈增益（防止位置漂移），单位：N*m/m */
	float k_velocity;  /**< 速度反馈增益（速度阻尼），单位：N*m/(m/s) */
	float k_yaw_rate;  /**< 偏航角速度反馈增益（转向阻尼），单位：N*m/(rad/s) */
	float k_roll_to_leg_m_per_rad; /**< 横滚角到腿部差动的映射系数，单位：m/rad */

	/* ========== 运行时可调参数（可从 #define 默认值覆盖） ========== */

	float theta_eq_rad; /**< 平衡点俯仰角偏移（站立时的"零点"），单位：弧度（rad） */

	/* 以下 gain_c*_a/b/c 为腿部长度 -> LQR 增益的多项式拟合系数 */
	/* 用于动态计算：gain = a * leg_len^2 + b * leg_len + c */
	float gain_c0_a, gain_c0_b, gain_c0_c;  /**< 增益 c0 的多项式系数（遗留/Flash 兼容） */
	float gain_c1_a, gain_c1_b, gain_c1_c;  /**< 增益 c1 的多项式系数（遗留/Flash 兼容） */
	float gain_c2;                           /**< 增益 c2 的常数值（遗留/Flash 兼容） */
	float gain_c3_a, gain_c3_b, gain_c3_c;  /**< 增益 c3 的多项式系数（遗留/Flash 兼容） */

	/* ========== 静摩擦补偿参数 ========== */

	float stiction_current_ma;  /**< 最大静摩擦补偿电流，单位：毫安（mA） */
	float stiction_start_deg;   /**< 开始施加静摩擦补偿的角度阈值，单位：度（deg） */
	float stiction_full_deg;    /**< 施加满静摩擦补偿的角度阈值，单位：度（deg） */

	/* ========== 保护与限幅参数 ========== */

	int16_t current_limit_ma;   /**< 轮子最大输出电流限幅，单位：毫安（mA） */
	float current_scale;        /**< 电流输出缩放系数（0.0~1.0，用于降功率运行） */
	float fault_deg;            /**< 触发故障保护的倾角阈值，单位：度（deg） */
	float recover_deg;          /**< 从故障状态恢复的角度阈值（需低于 fault_deg），单位：度（deg） */

	/* ========== 偏航同步参数 ========== */

	float wheel_sync_gain_ma;          /**< 左右轮差速同步增益，单位：mA/(rad/s) */
	float wheel_sync_current_limit_ma; /**< 差速同步最大电流限幅，单位：毫安（mA） */
} ascento_balance_params_t;

/**
 * @brief Ascento 平衡控制器输入结构体
 *
 * 每个控制周期由调用方填充，包含传感器反馈和运动目标。
 */
typedef struct {
	bool enable_request;     /**< 控制器使能请求（false 时输出为零） */
	bool wheel_feedback_ok;  /**< 轮子电机反馈是否有效（通信正常） */
	float dt_s;              /**< 本控制周期的时间步长，单位：秒（s） */

	/* 运动目标 */
	float target_forward_speed_mps; /**< 目标前进速度，单位：米/秒（m/s），正值=前进 */
	float target_yaw_rate_rad_s;    /**< 目标偏航角速度，单位：弧度/秒（rad/s），正值=右转 */
	float target_pitch_rad;         /**< 目标俯仰角（通常为零，用于特殊姿态），单位：弧度（rad） */

	/* 关节反馈（来自腿部关节电机编码器） */
	float left_joint_position_rad;  /**< 左腿关节当前角度，单位：弧度（rad） */
	float right_joint_position_rad; /**< 右腿关节当前角度，单位：弧度（rad） */
	float left_joint_velocity_rad_s;  /**< 左腿关节角速度，单位：弧度/秒（rad/s） */
	float right_joint_velocity_rad_s; /**< 右腿关节角速度，单位：弧度/秒（rad/s） */

	/* 传感器反馈 */
	bmi088_sample_t imu;         /**< IMU 采样数据（加速度、角速度、姿态角） */
	dji_m3508_motor_t left_wheel;  /**< 左轮电机反馈（电流、角度、速度） */
	dji_m3508_motor_t right_wheel; /**< 右轮电机反馈（电流、角度、速度） */
} ascento_balance_input_t;

/**
 * @brief Ascento 平衡控制器输出结构体
 *
 * 每个控制周期由 ascento_balance_update() 填充，包含电机指令和内部状态。
 */
typedef struct {
	bool active;       /**< 控制器是否处于活跃状态（已使能且未故障） */
	bool params_ready; /**< 参数是否已就绪（已加载有效参数） */
	bool faulted;      /**< 是否处于故障保护状态 */

	/* 电机输出指令 */
	int16_t left_wheel_current;  /**< 左轮目标电流，单位：毫安（mA） */
	int16_t right_wheel_current; /**< 右轮目标电流，单位：毫安（mA） */
	float left_joint_position_rad;  /**< 左腿关节目标角度，单位：弧度（rad） */
	float right_joint_position_rad; /**< 右腿关节目标角度，单位：弧度（rad） */
	float joint_velocity_limit_rad_s; /**< 关节角速度限幅，单位：弧度/秒（rad/s） */

	/* 内部计算中间量（用于调试和监控） */
	float body_position_m;   /**< 机体估计位置（轮子编码器积分），单位：米（m） */
	float body_velocity_mps; /**< 机体估计速度（编码器微分 + 滤波），单位：米/秒（m/s） */
	float pitch_rad;         /**< 机体俯仰角（IMU 融合后），单位：弧度（rad） */
	float pitch_rate_rad_s;  /**< 机体俯仰角速度（陀螺仪），单位：弧度/秒（rad/s） */
	float balance_torque_nm; /**< 平衡控制计算的总扭矩，单位：牛顿米（N*m） */
	float yaw_torque_nm;     /**< 偏航控制计算的差动扭矩，单位：牛顿米（N*m） */
} ascento_balance_output_t;

/**
 * @brief Ascento 平衡控制器内部状态结构体
 *
 * 保存控制器的持久化运行状态（跨控制周期保持）。
 * 由 ascento_balance_init() 初始化，ascento_balance_update() 更新。
 */
typedef struct {
	bool initialized;  /**< 控制器是否已初始化 */
	bool faulted;      /**< 是否处于故障保护状态 */
	int recover_ticks; /**< 故障恢复倒计时（tick 数，降到零后尝试恢复平衡） */

	float body_position_m;        /**< 机体累计位移估计，单位：米（m） */
	float wheel_position_zero_m;  /**< 轮子位置零点偏移（用于位置估计归零），单位：米（m） */
	float body_velocity_lpf_mps;  /**< 机体速度的低通滤波值，单位：米/秒（m/s） */
	float yaw_rate_lpf_rad_s;     /**< 偏航角速度的低通滤波值，单位：弧度/秒（rad/s） */
	float pitch_rate_lpf_rad_s;   /**< 俯仰角速度的低通滤波值，单位：弧度/秒（rad/s） */
} ascento_balance_state_t;

/**
 * @brief 默认参数指针（指向编译时定义的默认参数表）
 *
 * 可用于在 NVS 参数损坏时恢复出厂默认值。
 */
extern const ascento_balance_params_t *ascento_balance_default_params;

/**
 * @brief 获取当前平衡控制器参数
 * @param[out] params 指向参数结构体的指针，函数将填充当前参数值
 */
void ascento_balance_get_params(ascento_balance_params_t *params);

/**
 * @brief 设置平衡控制器参数
 * @param[in] params 指向新参数结构体的指针
 *
 * 运行时动态更新参数（不持久化到 Flash）。
 */
void ascento_balance_set_params(const ascento_balance_params_t *params);

/**
 * @brief 将当前参数持久化保存到 NVS（Flash）
 * @return 0 成功，非零失败
 */
int ascento_balance_save_params(void);

/**
 * @brief 将参数重置为出厂默认值
 * @return 0 成功，非零失败
 *
 * 从 NVS 中擦除自定义参数，恢复为 #define 编译时默认值。
 */
int ascento_balance_reset_params(void);

/**
 * @brief 初始化平衡控制器的 NVS 参数存储
 * @return 0 成功，非零失败
 *
 * 从 NVS 加载已保存的参数；若 NVS 为空则使用默认值。
 * 应在系统启动时调用一次。
 */
int ascento_balance_settings_init(void);

/**
 * @brief 初始化平衡控制器状态
 * @param[out] state 指向控制器状态结构体的指针
 *
 * 清零所有内部状态，设置 initialized 标志。
 */
void ascento_balance_init(ascento_balance_state_t *state);

/**
 * @brief 重置平衡控制器状态
 * @param[out] state 指向控制器状态结构体的指针
 *
 * 重置位置估计、滤波器状态等，但保留初始化标志。
 * 用于从故障状态恢复或重新站立时调用。
 */
void ascento_balance_reset(ascento_balance_state_t *state);

/**
 * @brief 检查参数是否已就绪
 * @param[in] params 指向参数结构体的指针
 * @return true = 参数有效（已校准且值合理），false = 参数无效
 */
bool ascento_balance_params_ready(const ascento_balance_params_t *params);

/**
 * @brief 根据关节角度计算腿部长度
 * @param[in] params    指向参数结构体的指针（需要关节-腿长映射参数）
 * @param[in] left_leg  true = 左腿，false = 右腿
 * @param[in] joint_rad 关节角度，单位：弧度（rad）
 * @return 腿部长度，单位：米（m）
 *
 * 使用线性插值将关节角度映射到腿部长度。
 */
float ascento_balance_leg_length_from_joint(const ascento_balance_params_t *params,
					    bool left_leg, float joint_rad);

/**
 * @brief 平衡控制器主更新函数（每个控制周期调用一次）
 * @param[in,out] state  控制器内部状态（跨周期保持）
 * @param[in]     params 控制器参数（物理参数 + 增益）
 * @param[in]     input  本周期的传感器反馈和运动目标
 * @param[out]    output 本周期的电机输出指令和中间量
 *
 * 执行完整的平衡控制算法：
 *   1. 从关节角度计算当前腿长
 *   2. 根据腿长计算 LQR 增益（多项式拟合）
 *   3. 计算 5 维状态向量：[俯仰角, 俯仰角速度, 位置, 速度, 偏航角速度]
 *   4. LQR 状态反馈计算平衡扭矩
 *   5. 叠加静摩擦补偿和偏航控制
 *   6. 电流限幅后输出轮子电流指令
 *   7. 计算关节目标角度（根据目标高度）
 *   8. 故障检测与保护
 */
void ascento_balance_update(ascento_balance_state_t *state,
			    const ascento_balance_params_t *params,
			    const ascento_balance_input_t *input,
			    ascento_balance_output_t *output);
