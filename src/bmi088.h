/**
 * @file bmi088.h
 * @brief BMI088 六轴 IMU 传感器驱动接口
 *
 * 本文件定义了 Bosch BMI088 六轴惯性测量单元（IMU）的驱动接口。
 * BMI088 包含一个三轴加速度计和一个三轴陀螺仪，通过 SPI 接口通信。
 *
 * 在本项目中的作用：
 *   - 提供机器人的姿态角（俯仰、横滚、偏航）
 *   - 提供角速度用于平衡控制的阻尼反馈
 *   - 提供加速度数据用于姿态估计
 *
 * 硬件连接：
 *   - 加速度计和陀螺仪各有独立的 SPI 片选信号（CS）
 *   - 共用同一条 SPI 总线，但作为两个独立的 SPI 设备操作
 *
 * 姿态估计：
 *   - 使用互补滤波器融合加速度计和陀螺仪数据
 *   - 加速度计提供静态姿态参考（低频，受振动影响大）
 *   - 陀螺仪提供动态角速度（高频，存在漂移）
 *   - 互补滤波器结合两者优点，输出稳定的姿态角
 */

#pragma once

#include <stdbool.h>

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>

/**
 * @brief BMI088 采样数据结构体
 *
 * 包含一次完整的 IMU 采样结果：原始加速度、角速度和姿态角。
 * 由 bmi088_update() 填充。
 */
typedef struct {
	/* 三轴加速度（加速度计原始数据换算后） */
	float ax_g; /**< X 轴加速度，单位：重力加速度 g（1g ≈ 9.81 m/s^2） */
	float ay_g; /**< Y 轴加速度，单位：g */
	float az_g; /**< Z 轴加速度，单位：g（静止时约为 1g，方向朝上） */

	/* 三轴角速度（陀螺仪原始数据换算后） */
	float gx_dps; /**< X 轴（横滚轴）角速度，单位：度/秒（deg/s） */
	float gy_dps; /**< Y 轴（俯仰轴）角速度，单位：度/秒（deg/s） */
	float gz_dps; /**< Z 轴（偏航轴）角速度，单位：度/秒（deg/s） */

	/* 姿态角（互补滤波器融合后） */
	float roll_deg;  /**< 横滚角，单位：度（deg），右倾为正 */
	float pitch_deg; /**< 俯仰角，单位：度（deg），前倾为正 */
	float yaw_deg;   /**< 偏航角，单位：度（deg），仅陀螺仪积分，会漂移 */
} bmi088_sample_t;

/**
 * @brief BMI088 驱动实例结构体
 *
 * 包含 SPI 设备句柄、校准偏移、姿态估计状态等。
 * 由 bmi088_init() 初始化，bmi088_update() 更新。
 */
typedef struct {
	/* SPI 设备配置 */
	struct spi_dt_spec accel_spi; /**< 加速度计的 SPI 设备规格（含片选引脚、时钟频率等） */
	struct spi_dt_spec gyro_spi;  /**< 陀螺仪的 SPI 设备规格（含片选引脚、时钟频率等） */

	/* 陀螺仪零偏校准 */
	float gyro_offset_dps[3]; /**< 陀螺仪三轴零偏偏移，单位：度/秒（deg/s） */
	/* 陀螺仪在静止时读数不为零，此偏移量在初始化时校准并减去 */

	/* 运行时陀螺仪零偏追踪（对抗温度漂移） */
	float gyro_runtime_bias_dps[3]; /**< 运行时零偏追踪值，单位：度/秒 */
	int stationary_count;            /**< 静止检测计数器 */

	/* 姿态角估计状态（互补滤波器输出） */
	float roll_deg;  /**< 当前横滚角估计值，单位：度（deg） */
	float pitch_deg; /**< 当前俯仰角估计值，单位：度（deg） */
	float yaw_deg;   /**< 当前偏航角估计值，单位：度（deg）（仅陀螺仪积分，会缓慢漂移） */

	int64_t last_update_us; /**< 上次更新的时间戳，单位：微秒（us），用于计算 dt */
	bool ready;             /**< 驱动是否就绪（true = 初始化成功，可读取数据） */
} bmi088_t;

/**
 * @brief 初始化 BMI088 IMU 传感器
 * @param[out] imu 指向驱动实例结构体的指针
 * @return 0 成功，非零失败
 *
 * 初始化过程包括：
 *   1. 配置 SPI 通信参数
 *   2. 复位加速度计和陀螺仪芯片
 *   3. 配置量程和输出数据率（ODR）
 *   4. 执行陀螺仪零偏校准（需保持静止）
 *   5. 初始化姿态角为零
 */
int bmi088_init(bmi088_t *imu);

/**
 * @brief 更新 BMI088 采样数据（读取传感器并更新姿态）
 * @param[in,out] imu 驱动实例（更新内部姿态状态）
 * @param[out]    out 采样数据输出（加速度、角速度、姿态角）
 * @return 0 成功，非零失败
 *
 * 每次调用：
 *   1. 通过 SPI 读取加速度计和陀螺仪的原始数据
 *   2. 减去零偏偏移，换算为物理单位
 *   3. 使用互补滤波器更新姿态角
 *   4. 将结果写入 out 结构体
 *
 * 典型调用频率：100~1000 Hz（取决于 BMI088 的 ODR 配置）。
 */
int bmi088_update(bmi088_t *imu, bmi088_sample_t *out);

/**
 * @brief 检查 IMU 驱动是否就绪
 * @param[in] imu 指向驱动实例的指针
 * @return true = 已初始化且可正常读取，false = 未初始化或故障
 */
bool bmi088_is_ready(const bmi088_t *imu);
