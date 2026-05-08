/**
 * @file dji_m3508.h
 * @brief DJI M3508 无刷电机驱动模块（通过 VESC CAN 协议通信）
 *
 * 本模块实现了 DJI M3508 无刷直流电机的 CAN 总线驱动。
 * M3508 电机内部集成 C620 电调，支持 FOC 矢量控制。
 * 由于本项目中 M3508 搭配 VESC 固件的电调使用，因此通信协议采用 VESC CAN 协议。
 *
 * 主要功能：
 *   - 通过 CAN 总线初始化并管理多个 M3508 电机
 *   - 发送电流（mA）、转速（RPM）控制指令
 *   - 支持单电机和分组电流指令（最多4个电机一组）
 *   - 接收并解析电机反馈数据（编码器、速度、电流、温度等）
 *   - 提供电机在线状态检测与超时判断
 *
 * 硬件连接：M3508 电机通过 CAN 总线连接到 STM32F407 的 CAN 外设。
 * 在 Ascento 轮腿机器人中，M3508 用作轮毂电机，驱动左右两个轮子。
 *
 * @note 编码器分辨率：8192 CPR（counts per revolution）
 * @note CAN 协议：VESC CAN 协议（非 DJI 原厂协议）
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>

/** @brief M3508 电机最大 CAN ID 值（CAN ID 范围 0~255） */
#define DJI_M3508_MAX_ID 255

/** @brief 一条 CAN 指令最多同时控制的电机数量（VESC 分组电流指令最多4台） */
#define DJI_M3508_MAX_COMMAND_MOTORS 4

/** @brief 编码器每转计数值（counts per revolution），M3508 编码器为 13 位，即 8192 */
#define DJI_M3508_ENCODER_CPR 8192.0f

/**
 * @brief DJI M3508 电机状态结构体
 *
 * 保存单个 M3508 电机的全部运行状态，包括编码器读数、速度、电流、温度等。
 * 通过 CAN 总线接收的反馈数据会实时更新到此结构体中。
 */
typedef struct {
	uint8_t id;                /**< 电机的 CAN ID（VESC ID，范围 0~255） */
	bool present;              /**< 电机是否已在总线上被发现（至少收到过一次反馈） */
	bool online;               /**< 电机当前是否在线（最近一次反馈未超时） */
	bool initialized;          /**< 电机驱动是否已初始化完成 */

	uint16_t encoder_raw;      /**< 当前编码器原始值（0~8191，13位） */
	uint16_t last_encoder_raw; /**< 上一次采样的编码器原始值，用于计算圈数变化 */
	int32_t round_count;       /**< 电机累计转过的圈数（由编码器溢出计数得到） */

	int32_t speed_rpm;         /**< 电机转速，单位：RPM（转/分钟），来自 VESC status4 数据包 */
	int32_t erpm;              /**< 电转速（electrical RPM），VESC 原始上报值 */

	int32_t command_current_ma;  /**< 最近一次发送的控制电流指令，单位：mA */
	int32_t motor_current_ma;    /**< 电机实际相电流，单位：mA（来自 VESC status4） */
	int32_t input_current_ma;    /**< 电调输入电流（母线电流），单位：mA（来自 VESC status4） */
	int32_t input_voltage_mv;    /**< 电调输入电压（母线电压），单位：mV（来自 VESC status4） */

	int32_t tachometer;        /**< VESC 转速计累计值（电气转数累计，来自 status4） */

	int16_t fet_temperature_cdeg;   /**< MOSFET 温度，单位：0.1 摄氏度（即 250 表示 25.0°C） */
	int16_t motor_temperature_cdeg; /**< 电机绕组温度，单位：0.1 摄氏度 */

	int32_t current_ma;        /**< 电机电流，单位：mA（用于向 CAN 总线发送控制指令时记录） */
	uint8_t temperature_c;     /**< 温度，单位：摄氏度（整数，部分数据包使用） */

	int64_t last_command_ms;       /**< 最近一次发送控制指令的时间戳，单位：ms（系统时钟） */
	int64_t last_update_ms;        /**< 最近一次收到任意反馈数据的时间戳，单位：ms */
	int64_t last_status4_update_ms; /**< 最近一次收到 VESC status4 数据包的时间戳，单位：ms */
	int64_t last_status5_update_ms; /**< 最近一次收到 VESC status5 数据包的时间戳，单位：ms */

	float angle_rad;           /**< 电机轴累计角度，单位：弧度（由 round_count 和编码器值计算） */
	float speed_rad_s;         /**< 电机轴角速度，单位：弧度/秒（由 RPM 换算） */

	float current_ma_to_wheel_torque_nm; /**< 电流到轮端扭矩的转换系数，单位：N·m/mA */
	float estimated_wheel_torque_nm;     /**< 估算的轮端扭矩，单位：N·m（电流乘以转换系数） */

	float duty;                /**< 占空比，无量纲（0.0~1.0，来自 VESC status5 数据包） */
} dji_m3508_motor_t;

/**
 * @brief DJI M3508 总线管理结构体
 *
 * 管理一条 CAN 总线上挂载的所有 M3508 电机，
 * 包含 CAN 设备句柄、自旋锁（用于线程安全）、电机数组和分组指令配置。
 */
typedef struct {
	const struct device *can;  /**< CAN 设备指针（Zephyr 设备模型句柄） */
	struct k_spinlock lock;    /**< 自旋锁，保护多线程并发访问电机数据 */
	dji_m3508_motor_t motor[DJI_M3508_MAX_ID + 1]; /**< 电机数组，按 CAN ID 索引（0~255） */
	uint8_t command_id[DJI_M3508_MAX_COMMAND_MOTORS]; /**< 分组电流指令中各槽位对应的电机 CAN ID */
	size_t command_id_count;   /**< 当前已注册的分组电机数量（最多4个） */
} dji_m3508_bus_t;

/**
 * @brief 初始化 M3508 总线驱动
 *
 * 注册 CAN 设备并配置要控制的电机 ID 列表。初始化后，
 * CAN 接收回调会自动更新电机的反馈数据。
 *
 * @param[out] bus       总线管理结构体指针
 * @param[in]  can       Zephyr CAN 设备句柄
 * @param[in]  ids       要注册的电机 CAN ID 数组
 * @param[in]  id_count  电机 ID 数组的长度
 * @return 0 表示成功，负值表示失败
 */
int dji_m3508_init(dji_m3508_bus_t *bus, const struct device *can,
		   const uint8_t *ids, size_t id_count);

/**
 * @brief 向单个电机发送电流控制指令
 *
 * 通过 VESC CAN 协议发送设定电流值。电流范围通常为 -16384 ~ +16384 mA。
 * 使用 CAN 扩展帧，根据 VESC 协议编码 CAN ID 和数据。
 *
 * @param[in] bus        总线管理结构体指针
 * @param[in] id         目标电机的 CAN ID
 * @param[in] current_ma 设定电流，单位：mA
 * @return 0 表示成功，负值表示失败
 */
int dji_m3508_send_current(dji_m3508_bus_t *bus, uint8_t id,
			   int16_t current_ma);

/**
 * @brief 向单个电机发送转速控制指令
 *
 * 通过 VESC CAN SET_RPM 命令设定目标转速。
 *
 * @param[in] bus  总线管理结构体指针
 * @param[in] id   目标电机的 CAN ID
 * @param[in] rpm  目标转速，单位：RPM（转/分钟），可为负值表示反转
 * @return 0 表示成功，负值表示失败
 */
int dji_m3508_send_rpm(dji_m3508_bus_t *bus, uint8_t id, int32_t rpm);

/**
 * @brief 同时向4个电机发送分组电流控制指令
 *
 * 一条 CAN 消息控制4个电机，对应 command_id 数组中的4个槽位。
 * 适用于需要同步控制多个电机的场景（如机器人四肢）。
 *
 * @param[in] bus  总线管理结构体指针
 * @param[in] id1  第1个槽位的设定电流，单位：mA
 * @param[in] id2  第2个槽位的设定电流，单位：mA
 * @param[in] id3  第3个槽位的设定电流，单位：mA
 * @param[in] id4  第4个槽位的设定电流，单位：mA
 * @return 0 表示成功，负值表示失败
 */
int dji_m3508_send_group_current(dji_m3508_bus_t *bus, int16_t id1,
				 int16_t id2, int16_t id3, int16_t id4);

/**
 * @brief 获取指定电机的最新状态快照
 *
 * 将内部电机数据结构体的内容拷贝到调用者提供的缓冲区。
 * 调用者无需关心线程安全问题，函数内部会加锁。
 *
 * @param[in]  bus  总线管理结构体指针（const，不会修改）
 * @param[in]  id   目标电机的 CAN ID
 * @param[out] out  输出缓冲区，接收电机状态的拷贝
 * @return true 表示成功获取（电机已注册），false 表示电机 ID 无效
 */
bool dji_m3508_get(const dji_m3508_bus_t *bus, uint8_t id,
		   dji_m3508_motor_t *out);

/**
 * @brief 检查指定电机是否在线（在超时时间内收到过反馈）
 *
 * 比较最近一次收到反馈的时间戳与当前时间，判断电机是否在通信范围内。
 *
 * @param[in] bus         总线管理结构体指针（const，不会修改）
 * @param[in] id          目标电机的 CAN ID
 * @param[in] now_ms      当前系统时间，单位：ms
 * @param[in] timeout_ms  超时阈值，单位：ms（超过此时间未收到反馈视为离线）
 * @return true 表示在线，false 表示离线或电机 ID 无效
 */
bool dji_m3508_is_online(const dji_m3508_bus_t *bus, uint8_t id,
			 int64_t now_ms, int64_t timeout_ms);
