/**
 * @file dm4340.h
 * @brief 达妙 DM4340 关节电机驱动模块
 *
 * 本模块实现了达妙（DaMiao）DM4340 直驱关节电机的 CAN 总线驱动。
 * DM4340 是一款集成电机、减速器、编码器和驱动器的一体化关节模组，
 * 支持 MIT 控制模式（阻抗控制）、位置-速度模式、纯速度模式等多种控制方式。
 *
 * 主要功能：
 *   - 通过 CAN 总线初始化并管理多个 DM4340 关节电机
 *   - 支持 MIT 控制模式（同时设定位置、速度、Kp、Kd、前馈扭矩）
 *   - 支持位置-速度控制模式（设定目标位置和速度限制）
 *   - 支持纯速度控制模式
 *   - 支持电机使能/失能、零位保存等管理操作
 *   - 支持读取电机内部寄存器参数
 *   - 提供 CAN 接收日志记录，用于调试
 *
 * 在 Ascento 轮腿机器人中，DM4340 用作髋关节电机，控制腿部的展开与收拢。
 * 每条腿配备一个 DM4340 关节电机，共两个（左腿和右腿）。
 *
 * @note MIT 控制模式：一种阻抗控制方式，同时发送位置参考、速度前馈、
 *       刚度（Kp）、阻尼（Kd）和前馈扭矩，电机内部计算控制输出。
 * @note 所有角度单位为弧度（rad），速度单位为弧度/秒（rad/s），扭矩单位为牛顿米（N·m）
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>

/** @brief DM4340 最大电机 ID（DM4340 CAN ID 范围 0~15） */
#define DM4340_MAX_ID 15

/** @brief CAN 接收日志缓冲区大小（环形缓冲区可记录的条目数） */
#define DM4340_RX_LOG_SIZE 128

/**
 * @brief CAN 接收日志条目结构体
 *
 * 记录一条完整的 CAN 帧信息，用于调试和协议分析。
 * 采用环形缓冲区存储，新数据覆盖最旧的数据。
 */
typedef struct {
	uint32_t id;       /**< CAN 帧 ID（标准帧为 11 位，扩展帧为 29 位） */
	uint8_t dlc;       /**< 数据长度码（Data Length Code，0~8，表示数据字节数） */
	uint8_t flags;     /**< CAN 帧标志位（如 RTR、IDE、BRS 等，Zephyr can_frame 定义） */
	uint8_t data[8];   /**< CAN 帧数据域（最多 8 字节） */
	uint32_t count;    /**< 此条目被写入时的累计计数（用于追踪顺序） */
} dm4340_rx_entry_t;

/**
 * @brief CAN 接收日志环形缓冲区结构体
 *
 * 以环形方式存储最近收到的 CAN 帧记录，用于调试时回溯通信历史。
 */
typedef struct {
	dm4340_rx_entry_t entries[DM4340_RX_LOG_SIZE]; /**< 日志条目数组（环形缓冲区） */
	uint32_t head;   /**< 环形缓冲区写指针（下一个写入位置的索引） */
	uint32_t total;  /**< 累计写入的总条目数（用于判断是否发生过回绕） */
} dm4340_rx_log_t;

/**
 * @brief DM4340 寄存器参数读取响应结构体
 *
 * 当向 DM4340 发起参数读取请求后，电机的响应数据存储在此结构体中。
 * DM4340 支持通过 CAN 读写内部寄存器（如 PID 参数、限位值等）。
 */
typedef struct {
	bool valid;          /**< 响应数据是否有效（已收到电机的回复） */
	uint16_t can_id;     /**< 响应对应的 CAN ID */
	uint8_t op;          /**< 操作类型（读/写/读回复/写回复等，见 DM4340 通信协议） */
	uint8_t rid;         /**< 寄存器 ID（Register ID，标识被读取的具体参数） */
	uint32_t raw_u32;    /**< 原始 uint32 型响应值（电机返回的未解析数据） */
	float value_float;   /**< 解析后的浮点型响应值（根据寄存器类型转换） */
	int64_t last_update_ms; /**< 最近一次收到该寄存器响应的时间戳，单位：ms */
} dm4340_param_response_t;

/**
 * @brief DM4340 电机反馈数据结构体
 *
 * 保存单个 DM4340 电机的实时反馈状态，包括位置、速度、扭矩和温度。
 * 电机通过 CAN 反馈帧周期性上报这些数据。
 */
typedef struct {
	uint8_t id;              /**< 电机 ID（CAN 节点 ID，范围 0~15） */
	uint8_t error;           /**< 电机错误状态码（0 表示无错误，非零表示有故障） */
	bool online;             /**< 电机当前是否在线（最近反馈未超时） */
	int64_t last_update_ms;  /**< 最近一次收到反馈的时间戳，单位：ms */
	float position_rad;      /**< 电机当前位置，单位：弧度（rad） */
	float velocity_rad_s;    /**< 电机当前速度，单位：弧度/秒（rad/s） */
	float torque_nm;         /**< 电机当前输出扭矩，单位：牛顿米（N·m） */
	uint8_t mos_temperature_c;   /**< MOSFET 驱动器温度，单位：摄氏度（°C） */
	uint8_t rotor_temperature_c; /**< 电机转子温度，单位：摄氏度（°C） */
} dm4340_feedback_t;

/**
 * @brief DM4340 MIT 控制模式参数限位结构体
 *
 * 定义 MIT 控制模式下各参数的合法范围。
 * 发送 MIT 指令前，需要将用户参数归一化到 [-1, 1] 的范围内，
 * 此结构体提供了归一化/反归一化所需的上下界。
 */
typedef struct {
	float p_min;   /**< 位置最小值，单位：弧度（rad） */
	float p_max;   /**< 位置最大值，单位：弧度（rad） */
	float v_min;   /**< 速度最小值，单位：弧度/秒（rad/s） */
	float v_max;   /**< 速度最大值，单位：弧度/秒（rad/s） */
	float kp_min;  /**< 比例增益 Kp 最小值 */
	float kp_max;  /**< 比例增益 Kp 最大值 */
	float kd_min;  /**< 微分增益 Kd 最小值 */
	float kd_max;  /**< 微分增益 Kd 最大值 */
	float t_min;   /**< 扭矩最小值，单位：牛顿米（N·m） */
	float t_max;   /**< 扭矩最大值，单位：牛顿米（N·m） */
} dm4340_mit_limit_t;

/**
 * @brief DM4340 总线管理结构体
 *
 * 管理一条 CAN 总线上挂载的所有 DM4340 关节电机。
 * 包含 CAN 设备句柄、反馈 CAN ID 配置、自旋锁、电机状态数组、
 * 参数响应缓存、MIT 限位参数和接收日志。
 */
typedef struct {
	const struct device *can;          /**< CAN 设备指针（Zephyr 设备模型句柄） */
	uint16_t feedback_can_id;          /**< 左腿电机反馈帧的 CAN ID 基址 */
	uint16_t right_feedback_can_id;    /**< 右腿电机反馈帧的 CAN ID 基址（如使用双反馈） */
	bool has_right_feedback;           /**< 是否启用了右腿独立反馈 CAN ID */
	struct k_spinlock lock;            /**< 自旋锁，保护多线程并发访问 */
	dm4340_feedback_t motor[DM4340_MAX_ID + 1];       /**< 电机反馈数组，按 ID 索引（0~15） */
	dm4340_param_response_t param[DM4340_MAX_ID + 1]; /**< 参数读取响应缓存，按 ID 索引 */
	dm4340_mit_limit_t limit;          /**< MIT 控制模式的参数限位配置 */
	dm4340_rx_log_t rx_log;            /**< CAN 接收日志（用于调试） */
} dm4340_bus_t;

/**
 * @brief 打印 CAN 接收日志到控制台
 *
 * 将环形缓冲区中记录的所有 CAN 帧信息输出到日志/控制台，
 * 用于调试 CAN 通信问题。
 *
 * @param[in] bus  总线管理结构体指针（const，不会修改）
 */
void dm4340_dump_rx_log(const dm4340_bus_t *bus);

/**
 * @brief 切换 RX 活动指示 LED 状态
 *
 * 每次收到 CAN 数据帧时调用，翻转 LED 以直观指示 CAN 接收活动。
 * 用于硬件调试时确认 CAN 总线是否有数据流入。
 */
void dm4340_rx_led_toggle(void);

/**
 * @brief 手动轮询 CAN 接收 FIFO 并处理数据
 *
 * 从 CAN 外设的接收 FIFO 中读取待处理的 CAN 帧，
 * 解析反馈数据并更新电机状态结构体。
 * 在不使用 CAN 中断回调的场景下，可周期性调用此函数。
 *
 * @param[in,out] bus  总线管理结构体指针
 * @return 0 表示成功，负值表示失败
 */
int dm4340_poll_rx_fifo(dm4340_bus_t *bus);

/**
 * @brief 初始化 DM4340 总线驱动
 *
 * 配置 CAN 设备、注册接收回调、设置反馈 CAN ID 基址。
 * 初始化后，CAN 接收回调会自动解析电机反馈数据。
 *
 * @param[out] bus                    总线管理结构体指针
 * @param[in]  can                    Zephyr CAN 设备句柄
 * @param[in]  feedback_can_id        左腿反馈帧的 CAN ID 基址
 * @param[in]  right_feedback_can_id  右腿反馈帧的 CAN ID 基址
 * @return 0 表示成功，负值表示失败
 */
int dm4340_init(dm4340_bus_t *bus, const struct device *can,
		uint16_t feedback_can_id, uint16_t right_feedback_can_id);

/**
 * @brief 使能指定 DM4340 电机
 *
 * 发送使能指令，电机进入控制就绪状态，可以接收控制指令。
 * 使能前电机处于自由状态（无保持力矩）。
 *
 * @param[in,out] bus  总线管理结构体指针
 * @param[in]     id   目标电机 ID（0~15）
 * @return 0 表示成功，负值表示失败
 */
int dm4340_enable(dm4340_bus_t *bus, uint8_t id);

/**
 * @brief 失能指定 DM4340 电机
 *
 * 发送失能指令，电机退出控制状态，恢复为自由状态。
 *
 * @param[in,out] bus  总线管理结构体指针
 * @param[in]     id   目标电机 ID（0~15）
 * @return 0 表示成功，负值表示失败
 */
int dm4340_disable(dm4340_bus_t *bus, uint8_t id);

/**
 * @brief 保存当前电机位置为零位
 *
 * 将电机当前位置设为编码器零点。掉电后零位信息会持久化保存在电机驱动器中。
 * 通常在机器人组装完成后进行标定时调用。
 *
 * @param[in,out] bus  总线管理结构体指针
 * @param[in]     id   目标电机 ID（0~15）
 * @return 0 表示成功，负值表示失败
 */
int dm4340_save_zero(dm4340_bus_t *bus, uint8_t id);

/**
 * @brief 请求读取电机内部寄存器参数
 *
 * 向指定 CAN ID 的电机发送参数读取请求。电机收到后会回复指定寄存器的当前值。
 * 可用寄存器包括 PID 参数、限位值、电机型号等（具体参见 DM4340 手册）。
 *
 * @param[in,out] bus     总线管理结构体指针
 * @param[in]     can_id  目标电机的 CAN ID
 * @param[in]     rid     要读取的寄存器 ID
 * @return 0 表示成功，负值表示失败
 */
int dm4340_request_param_read(dm4340_bus_t *bus, uint16_t can_id,
			      uint8_t rid);

/**
 * @brief 发送位置-速度控制指令
 *
 * 电机以指定速度限制运动到目标位置。适用于位置伺服场景。
 * 电机内部使用梯形速度曲线（或类似规划器）平滑到达目标位置。
 *
 * @param[in,out] bus                  总线管理结构体指针
 * @param[in]     id                   目标电机 ID（0~15）
 * @param[in]     position_rad         目标位置，单位：弧度（rad）
 * @param[in]     velocity_limit_rad_s 速度限制，单位：弧度/秒（rad/s），正值
 * @return 0 表示成功，负值表示失败
 */
int dm4340_send_pos_vel(dm4340_bus_t *bus, uint8_t id, float position_rad,
			float velocity_limit_rad_s);

/**
 * @brief 发送速度控制指令
 *
 * 控制电机以指定速度持续旋转。适用于连续旋转场景。
 *
 * @param[in,out] bus             总线管理结构体指针
 * @param[in]     id              目标电机 ID（0~15）
 * @param[in]     velocity_rad_s  目标速度，单位：弧度/秒（rad/s），负值表示反转
 * @return 0 表示成功，负值表示失败
 */
int dm4340_send_velocity(dm4340_bus_t *bus, uint8_t id,
			 float velocity_rad_s);

/**
 * @brief 发送 MIT 阻抗控制指令
 *
 * MIT 控制模式（也称为阻抗控制或力位混合控制）同时发送：
 *   - 目标位置（position）：电机趋向的目标角度
 *   - 速度前馈（velocity）：期望的速度
 *   - Kp（刚度）：位置误差的比例增益，越大越"硬"
 *   - Kd（阻尼）：速度的微分增益，抑制振荡
 *   - 前馈扭矩（torque）：直接叠加的力矩
 *
 * 电机内部计算：τ = Kp*(θ_target - θ_actual) + Kd*(ω_target - ω_actual) + τ_ff
 *
 * 此函数会根据 MIT 限位对参数进行归一化编码。
 *
 * @param[in,out] bus             总线管理结构体指针
 * @param[in]     id              目标电机 ID（0~15）
 * @param[in]     position_rad    目标位置，单位：弧度（rad）
 * @param[in]     velocity_rad_s  速度前馈，单位：弧度/秒（rad/s）
 * @param[in]     kp              位置比例增益（刚度）
 * @param[in]     kd              速度微分增益（阻尼）
 * @param[in]     torque_nm       前馈扭矩，单位：牛顿米（N·m）
 * @return 0 表示成功，负值表示失败
 */
int dm4340_send_mit(dm4340_bus_t *bus, uint8_t id, float position_rad,
		    float velocity_rad_s, float kp, float kd, float torque_nm);

/**
 * @brief 获取指定电机的最新反馈数据快照
 *
 * 将内部电机反馈结构体的内容拷贝到调用者提供的缓冲区。
 * 线程安全：函数内部会加锁。
 *
 * @param[in]  bus  总线管理结构体指针（const，不会修改）
 * @param[in]  id   目标电机 ID（0~15）
 * @param[out] out  输出缓冲区，接收电机反馈数据的拷贝
 * @return true 表示成功获取（电机已注册），false 表示电机 ID 无效
 */
bool dm4340_get(const dm4340_bus_t *bus, uint8_t id, dm4340_feedback_t *out);

/**
 * @brief 获取指定电机的寄存器参数读取响应
 *
 * 返回指定电机、指定寄存器的最近一次参数读取响应。
 * 必须先调用 dm4340_request_param_read() 发起读取请求。
 *
 * @param[in]  bus  总线管理结构体指针（const，不会修改）
 * @param[in]  id   目标电机 ID（0~15）
 * @param[in]  rid  寄存器 ID
 * @param[out] out  输出缓冲区，接收参数响应数据的拷贝
 * @return true 表示响应有效且已拷贝，false 表示无有效响应或参数无效
 */
bool dm4340_get_param_response(const dm4340_bus_t *bus, uint8_t id,
			       uint8_t rid, dm4340_param_response_t *out);
