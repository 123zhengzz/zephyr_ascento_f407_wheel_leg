/**
 * @file bmi088.c
 * @brief BMI088 六轴惯性测量单元（IMU）SPI驱动及姿态解算模块
 *
 * 本文件实现了 Bosch BMI088 六轴 IMU 的驱动程序，包含：
 *   1. 通过 SPI 总线读写加速度计和陀螺仪寄存器
 *   2. 原始传感器数据的读取和单位换算
 *   3. 陀螺仪零偏校准（上电静止时采集300个样本取均值）
 *   4. 基于互补滤波器的姿态估计（横滚角roll、俯仰角pitch、偏航角yaw）
 *
 * BMI088 芯片特性：
 *   - 加速度计：三轴加速度，量程 ±6g，灵敏度 5460 LSB/g
 *   - 陀螺仪：三轴角速度，量程 ±2000°/s，灵敏度 16.384 LSB/(°/s)
 *   - 加速度计和陀螺仪各自拥有独立的 SPI 片选线
 *
 * 坐标轴重映射说明：
 *   在本机器人PCB上，BMI088芯片的X轴指向机器人侧向（即机器人的横滚轴roll），
 *   芯片Y轴指向前方（即机器人的俯仰轴pitch）。因此在姿态解算时需要进行轴映射：
 *     - 机器人roll  <- 加速度计X轴 / 陀螺仪Y轴
 *     - 机器人pitch <- 加速度计Y轴 / 陀螺仪X轴
 */

#include "bmi088.h"

#include <errno.h>
#include <math.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "app_config.h"

LOG_MODULE_REGISTER(bmi088, LOG_LEVEL_INF);

/* =========================================================================
 * 设备树节点标签 —— 对应 DTS 中定义的加速度计和陀螺仪 SPI 设备
 * BMI088 的加速度计和陀螺仪是独立的 SPI 从设备，各有各的片选线
 * ========================================================================= */
#define BMI088_ACCEL_NODE DT_NODELABEL(bmi088_accel)
#define BMI088_GYRO_NODE  DT_NODELABEL(bmi088_gyro)

/* =========================================================================
 * BMI088 加速度计寄存器地址（参考 BMI088 数据手册第 7 节 Register Map）
 * 加速度计使用 8 位寄存器地址，SPI 读操作时需将最高位置 1（或 0x80）
 * ========================================================================= */
#define BMI088_ACCEL_CHIP_ID   0x00  /* 芯片 ID 寄存器，读取值应为 0x1E */
#define BMI088_ACCEL_DATA      0x12  /* 加速度数据起始寄存器，6 字节连续读取：
                                       [0x12-0x13] X 轴, [0x14-0x15] Y 轴,
                                       [0x16-0x17] Z 轴，小端格式 int16 */
#define BMI088_ACCEL_CONF      0x40  /* 加速度计配置寄存器：设置 ODR（输出数据速率）
                                       和带宽（BWP），写入 0xA8 表示 ODR=800Hz,
                                       BWP=OSR4（过采样率4倍） */
#define BMI088_ACCEL_RANGE     0x41  /* 加速度计量程寄存器，写入 0x01 = ±6g */
#define BMI088_ACCEL_PWR_CONF  0x7c  /* 电源模式配置寄存器，写入 0x00 = active 模式 */
#define BMI088_ACCEL_PWR_CTRL  0x7d  /* 电源控制寄存器，写入 0x04 = 开启加速度计 */
#define BMI088_ACCEL_SOFTRESET 0x7e  /* 软复位寄存器，写入 0xB6 执行复位 */

/* =========================================================================
 * BMI088 陀螺仪寄存器地址（参考 BMI088 数据手册第 8 节 Register Map）
 * 陀螺仪使用 8 位寄存器地址，SPI 读操作时地址无需置位（直接发送原地址）
 * ========================================================================= */
#define BMI088_GYRO_CHIP_ID    0x00  /* 芯片 ID 寄存器，读取值应为 0x0F */
#define BMI088_GYRO_DATA       0x02  /* 陀螺仪数据起始寄存器，6 字节连续读取：
                                       [0x02-0x03] X 轴, [0x04-0x05] Y 轴,
                                       [0x06-0x07] Z 轴，小端格式 int16 */
#define BMI088_GYRO_RANGE      0x0f  /* 陀螺仪量程寄存器，写入 0x00 = ±2000°/s */
#define BMI088_GYRO_BANDWIDTH  0x10  /* 陀螺仪带宽/ODR 寄存器，写入 0x02 =
                                       ODR=1000Hz, 带宽=116Hz (滤波器截止频率) */
#define BMI088_GYRO_LPM1       0x11  /* 低功耗模式寄存器，写入 0x00 = 正常模式 */
#define BMI088_GYRO_SOFTRESET  0x14  /* 软复位寄存器，写入 0xB6 执行复位 */

/* =========================================================================
 * 芯片 ID 校验值 —— 用于初始化时验证 SPI 通信是否正常
 * 读取 CHIP_ID 寄存器并与以下常量比对
 * ========================================================================= */
#define BMI088_ACCEL_ID_VALUE  0x1e  /* 加速度计芯片 ID 固定值 */
#define BMI088_GYRO_ID_VALUE   0x0f  /* 陀螺仪芯片 ID 固定值 */

/* =========================================================================
 * 换算常量 —— 将原始 ADC 计数值转换为物理单位
 * 当量程为 ±6g 时：灵敏度 = 5460 LSB/g
 * 当量程为 ±2000°/s 时：灵敏度 = 16.384 LSB/(°/s)
 * 原始值除以这些常量即得到 g 或 °/s 为单位的物理量
 * ========================================================================= */
#define BMI088_ACCEL_LSB_PER_G   5460.0f
#define BMI088_GYRO_LSB_PER_DPS  16.384f

/**
 * @brief 获取当前系统时间（微秒精度）
 *
 * 使用 Zephyr 内核的硬件时钟周期计数器，转换为微秒时间戳。
 * 用于互补滤波器中计算两次采样之间的时间间隔 dt。
 *
 * @return 当前时间，单位为微秒（us）
 */
static int64_t now_us(void)
{
	return k_cyc_to_us_floor64(k_cycle_get_64());
}

/**
 * @brief 通过 SPI 读取 BMI088 的一个或多个连续寄存器
 *
 * BMI088 SPI 读操作协议（详见数据手册 Section 4.2.2）：
 *
 * 【加速度计】SPI 读时序：
 *   MOSI: [寄存器地址 | 0x80] [Dummy字节] [Dummy...] ...
 *   MISO: [无效]              [Dummy字节] [数据0] [数据1] ...
 *   说明：加速度计在读操作时，地址字节后需要插入一个 Dummy 字节（无意义），
 *         随后的 MISO 字节才是有效数据。因此 dummy = 1。
 *
 * 【陀螺仪】SPI 读时序：
 *   MOSI: [寄存器地址 | 0x80] [Dummy...] ...
 *   MISO: [无效]              [数据0] [数据1] ...
 *   说明：陀螺仪读操作时，地址字节后直接返回数据，无 Dummy 字节。因此 dummy = 0。
 *
 * 两者共同点：读操作时寄存器地址最高位需置 1（即 reg | 0x80），
 *             数据以小端（Little-Endian）格式传输。
 *
 * @param spi   SPI 设备规格（加速度计或陀螺仪）
 * @param reg   起始寄存器地址
 * @param data  输出缓冲区，存放读取到的数据
 * @param len   要读取的字节数
 * @param accel 是否为加速度计（true = 加速度计，需要 Dummy 字节；false = 陀螺仪）
 * @return 0 成功，负数表示 SPI 传输失败
 */
static int spi_read_reg(const struct spi_dt_spec *spi, uint8_t reg,
			uint8_t *data, size_t len, bool accel)
{
	uint8_t tx[12] = { 0 };
	uint8_t rx[12] = { 0 };
	/* 加速度计读操作需要 1 个 Dummy 字节，陀螺仪不需要 */
	const size_t dummy = accel ? 1U : 0U;
	/* 总传输长度 = 1字节地址 + dummy字节 + 数据字节数 */
	const size_t total = 1U + dummy + len;

	/* 安全检查：确保传输长度不超过缓冲区大小 */
	if (total > sizeof(tx)) {
		return -EINVAL;
	}

	/* 构造地址字节：最高位置 1 表示读操作 */
	tx[0] = reg | 0x80U;

	const struct spi_buf tx_buf = {
		.buf = tx,
		.len = total,
	};
	const struct spi_buf rx_buf = {
		.buf = rx,
		.len = total,
	};
	const struct spi_buf_set tx_set = {
		.buffers = &tx_buf,
		.count = 1,
	};
	const struct spi_buf_set rx_set = {
		.buffers = &rx_buf,
		.count = 1,
	};

	/* 执行 SPI 全双工收发：同时发送地址和接收数据 */
	const int ret = spi_transceive_dt(spi, &tx_set, &rx_set);
	if (ret == 0) {
		/* 从接收缓冲区中提取有效数据：
		 * rx[0] 是地址字节的响应（无效），跳过；
		 * rx[1] 对于加速度计是 Dummy 字节，对于陀螺仪已是第一个数据字节；
		 * 有效数据从 rx[1 + dummy] 开始 */
		memcpy(data, &rx[1U + dummy], len);
	}
	return ret;
}

/**
 * @brief 通过 SPI 向 BMI088 写入单个寄存器
 *
 * BMI088 SPI 写操作协议：
 *   MOSI: [寄存器地址 & 0x7F] [数据字节]
 *   写操作时，寄存器地址最高位必须为 0（即 reg & 0x7F），
 *   第二个字节为要写入的数据值。整个操作仅需 2 字节。
 *
 * @param spi   SPI 设备规格（加速度计或陀螺仪）
 * @param reg   目标寄存器地址
 * @param value 要写入的值
 * @return 0 成功，负数表示 SPI 传输失败
 */
static int spi_write_reg(const struct spi_dt_spec *spi, uint8_t reg,
			 uint8_t value)
{
	/* 地址字节最高位清零表示写操作 */
	uint8_t tx[2] = { reg & 0x7fU, value };
	const struct spi_buf tx_buf = {
		.buf = tx,
		.len = sizeof(tx),
	};
	const struct spi_buf_set tx_set = {
		.buffers = &tx_buf,
		.count = 1,
	};

	return spi_write_dt(spi, &tx_set);
}

/**
 * @brief 将两个字节按小端序（Little-Endian）组合为有符号 16 位整数
 *
 * BMI088 的传感器数据寄存器采用小端格式存储：
 *   data[0] = 低字节（LSB），data[1] = 高字节（MSB）
 * 组合后解释为有符号整数（int16），范围 -32768 ~ +32767。
 *
 * @param data 指向两个字节的指针
 * @return 组合后的 int16 值
 */
static int16_t le_i16(const uint8_t *data)
{
	return (int16_t)((uint16_t)data[1] << 8 | data[0]);
}

/**
 * @brief 从 BMI088 读取一次完整的原始传感器数据并转换为物理单位
 *
 * 执行流程：
 *   1. 从加速度计连续读取 6 字节数据（X/Y/Z 各 2 字节，小端 int16）
 *   2. 将原始 ADC 值除以灵敏度（5460 LSB/g）换算为重力加速度 g
 *   3. 从陀螺仪连续读取 6 字节数据
 *   4. 将原始 ADC 值除以灵敏度（16.384 LSB/°/s）换算为角速度 °/s
 *   5. 减去陀螺仪零偏校准值，得到校准后的角速度
 *
 * @param imu    BMI088 设备实例
 * @param sample 输出结构体，存放换算后的传感器数据
 * @return 0 成功，负数表示 SPI 读取失败
 */
static int read_raw(bmi088_t *imu, bmi088_sample_t *sample)
{
	uint8_t raw[6];
	int ret;

	/* ---------- 读取加速度计数据 ----------
	 * 从寄存器 0x12 开始连续读取 6 字节：
	 *   raw[0-1] = X 轴加速度（小端 int16）
	 *   raw[2-3] = Y 轴加速度（小端 int16）
	 *   raw[4-5] = Z 轴加速度（小端 int16）
	 * 转换公式：加速度(g) = 原始值 / 5460
	 */
	ret = spi_read_reg(&imu->accel_spi, BMI088_ACCEL_DATA, raw,
			   sizeof(raw), true);
	if (ret != 0) {
		return ret;
	}

	sample->ax_g = (float)le_i16(&raw[0]) / BMI088_ACCEL_LSB_PER_G;
	sample->ay_g = (float)le_i16(&raw[2]) / BMI088_ACCEL_LSB_PER_G;
	sample->az_g = (float)le_i16(&raw[4]) / BMI088_ACCEL_LSB_PER_G;

	/* ---------- 读取陀螺仪数据 ----------
	 * 从寄存器 0x02 开始连续读取 6 字节：
	 *   raw[0-1] = X 轴角速度（小端 int16）
	 *   raw[2-3] = Y 轴角速度（小端 int16）
	 *   raw[4-5] = Z 轴角速度（小端 int16）
	 * 转换公式：角速度(°/s) = 原始值 / 16.384 - 零偏
	 * 零偏在 bmi088_init() 中通过静止状态校准获得
	 */
	ret = spi_read_reg(&imu->gyro_spi, BMI088_GYRO_DATA, raw,
			   sizeof(raw), false);
	if (ret != 0) {
		return ret;
	}

	sample->gx_dps = (float)le_i16(&raw[0]) / BMI088_GYRO_LSB_PER_DPS -
			 imu->gyro_offset_dps[0];
	sample->gy_dps = (float)le_i16(&raw[2]) / BMI088_GYRO_LSB_PER_DPS -
			 imu->gyro_offset_dps[1];
	sample->gz_dps = (float)le_i16(&raw[4]) / BMI088_GYRO_LSB_PER_DPS -
			 imu->gyro_offset_dps[2];

	return 0;
}

/**
 * @brief 使用互补滤波器更新姿态角（roll/pitch/yaw）
 *
 * 互补滤波器原理：
 *   加速度计可以测量静态倾角（无漂移但高频噪声大），
 *   陀螺仪可以精确测量角速度（短期精度高但积分会漂移）。
 *   互补滤波器将两者的优势结合：
 *
 *     angle = alpha * (angle_prev + gyro_rate * dt) + (1 - alpha) * accel_angle
 *
 *   其中 alpha = 0.985，即 98.5% 信任陀螺仪积分，1.5% 信任加速度计。
 *   - 陀螺仪积分提供短期高频响应（跟踪快速旋转）
 *   - 加速度计提供长期低频校正（消除陀螺仪积分漂移）
 *   - 57.2957795 = 180/π，弧度转角度的换算因子
 *
 * 偏航角（yaw）特殊处理：
 *   由于没有磁力计，yaw 角无法用加速度计校正（加速度计只能测量 roll 和 pitch），
 *   因此 yaw 仅由陀螺仪 Z 轴角速度积分得到，会随时间缓慢漂移。
 *   yaw 角被归一化到 [-180, +180] 范围内。
 *
 * @param imu    BMI088 设备实例，包含上一次的姿态角状态
 * @param sample 本次读取的传感器数据，函数执行后会填充 roll_deg/pitch_deg/yaw_deg
 */
static void update_attitude(bmi088_t *imu, bmi088_sample_t *sample)
{
	const int64_t now = now_us();
	float dt_s = 0.001f;  /* 默认 dt = 1ms（首次调用时使用） */

	/* 计算自上次更新以来的时间间隔 dt（秒）
	 * 首次调用时 last_update_us 为 0，使用默认值 1ms
	 * CLAMP 限制 dt 在 [0.2ms, 20ms] 范围内，防止异常值 */
	if (imu->last_update_us != 0) {
		dt_s = (float)(now - imu->last_update_us) * 1.0e-6f;
		dt_s = CLAMP(dt_s, 0.0002f, 0.02f);
	}
	imu->last_update_us = now;

	/* =================================================================
	 * 轴重映射 —— 适配 PCB 上芯片的实际安装方向
	 *
	 * BMI088 芯片在 PCB 上的安装方位：
	 *   芯片 X 轴 -> 机器人侧向（左右方向）-> 对应机器人的 roll（横滚）轴
	 *   芯片 Y 轴 -> 机器人前后方向       -> 对应机器人的 pitch（俯仰）轴
	 *   芯片 Z 轴 -> 垂直于 PCB 平面       -> 对应机器人的 yaw（偏航）轴
	 *
	 * 因此：
	 *   机器人 roll  角 = atan2(加速度计X, 加速度计Z)，使用陀螺仪Y轴角速度积分
	 *   机器人 pitch 角 = atan2(-加速度计Y, sqrt(X^2+Z^2))，使用陀螺仪X轴角速度积分
	 *
	 * 加速度计计算倾角的几何原理：
	 *   roll  = atan2(ax, az)：在 Y-Z 平面投影的角度
	 *   pitch = atan2(-ay, sqrt(ax^2 + az^2))：使用重力在前向和水平面的分量
	 *   负号是因为芯片 Y 轴正方向与机器人 pitch 正方向定义相反
	 *
	 * 转换因子 57.2957795 = 180.0 / 3.14159265，将弧度转换为角度
	 * ================================================================= */
	const float roll_acc =
		atan2f(sample->ax_g, sample->az_g) * 57.2957795f;
	const float pitch_acc =
		atan2f(-sample->ay_g,
		       sqrtf(sample->ax_g * sample->ax_g +
			     sample->az_g * sample->az_g)) * 57.2957795f;

	/* =================================================================
	 * 互补滤波器核心算法
	 *
	 * 公式：angle = alpha * (angle + gyro * dt) + (1 - alpha) * accel_angle
	 *
	 * alpha = 0.985 意味着：
	 *   - 陀螺仪积分结果占 98.5% 权重（跟踪快速变化，响应性好）
	 *   - 加速度计测量结果占 1.5% 权重（消除长期漂移）
	 *
	 * 这种高 alpha 值的配置适合机器人平衡控制：
	 *   - 需要快速响应的姿态变化（防跌倒）
	 *   - 同时在长时间运行中不会因陀螺仪漂移而失控
	 * ================================================================= */
	const float alpha = 0.97f;

	/* =================================================================
	 * 运行时陀螺仪零偏追踪
	 *
	 * 当机器人近似静止时（三个轴角速度均低于阈值），缓慢更新零偏估计，
	 * 补偿陀螺仪因温度变化产生的零偏漂移。
	 *
	 * 阈值：2°/s（静止时陀螺仪噪声约 0.1~0.5°/s，2°/s 留有裕量）
	 * 学习率：0.001（每秒约更新 0.2% 的零偏，约 5 秒收敛 63%）
	 * 检测窗口：需要连续 200 个样本（约 1 秒）静止才开始更新
	 * ================================================================= */
	const float gyro_mag = sqrtf(sample->gx_dps * sample->gx_dps +
				     sample->gy_dps * sample->gy_dps +
				     sample->gz_dps * sample->gz_dps);
	if (gyro_mag < 2.0f) {
		imu->stationary_count++;
		if (imu->stationary_count > 200) {
			const float lr = 0.001f;
			imu->gyro_runtime_bias_dps[0] += lr * sample->gx_dps;
			imu->gyro_runtime_bias_dps[1] += lr * sample->gy_dps;
			imu->gyro_runtime_bias_dps[2] += lr * sample->gz_dps;
		}
	} else {
		imu->stationary_count = 0;
	}

	/* 应用运行时零偏修正 */
	const float gx_corr = sample->gx_dps - imu->gyro_runtime_bias_dps[0];
	const float gy_corr = sample->gy_dps - imu->gyro_runtime_bias_dps[1];

	/* roll 更新：陀螺仪 Y 轴（chip Y -> 机器人 roll 方向） */
	imu->roll_deg = alpha * (imu->roll_deg + gy_corr * dt_s) +
			(1.0f - alpha) * roll_acc;

	/* pitch 更新：陀螺仪 X 轴（chip X -> 机器人 pitch 方向） */
	imu->pitch_deg = alpha * (imu->pitch_deg + gx_corr * dt_s) +
			 (1.0f - alpha) * pitch_acc;

	/* yaw 更新：仅陀螺仪 Z 轴积分，无加速度计校正（无磁力计）
	 * yaw 会随时间缓慢漂移，但在短时间操作中可接受 */
	imu->yaw_deg += sample->gz_dps * dt_s;

	/* yaw 角归一化到 [-180°, +180°] 范围 */
	while (imu->yaw_deg > 180.0f) {
		imu->yaw_deg -= 360.0f;
	}
	while (imu->yaw_deg < -180.0f) {
		imu->yaw_deg += 360.0f;
	}

	/* 输出结果：
	 * roll 减去静态零位偏置（APP_IMU_ROLL_ZERO_DEG），用于补偿安装角度误差
	 * pitch 减去静态零位偏置（APP_IMU_PITCH_ZERO_DEG），用于补偿安装偏差
	 * yaw 直接输出 */
	sample->roll_deg = imu->roll_deg - APP_IMU_ROLL_ZERO_DEG;
	sample->pitch_deg = imu->pitch_deg - APP_IMU_PITCH_ZERO_DEG;
	sample->yaw_deg = imu->yaw_deg;
}

/**
 * @brief 初始化 BMI088 IMU 传感器
 *
 * 初始化流程：
 *   1. 从设备树获取加速度计和陀螺仪的 SPI 设备规格
 *   2. 检查 SPI 总线是否就绪
 *   3. 对加速度计和陀螺仪执行软复位
 *   4. 读取并校验芯片 ID（加速度计应为 0x1E，陀螺仪应为 0x0F）
 *   5. 配置加速度计：开启电源、设置量程 ±6g、ODR=800Hz
 *   6. 配置陀螺仪：正常模式、量程 ±2000°/s、ODR=1000Hz
 *   7. 陀螺仪零偏校准：采集 300 个样本取平均值作为零偏
 *   8. 用加速度计读数初始化 roll 和 pitch 角
 *
 * @param imu BMI088 设备实例（调用前无需初始化，函数内会 memset 清零）
 * @return 0 成功，负数表示初始化失败
 */
int bmi088_init(bmi088_t *imu)
{
	if (imu == NULL) {
		return -EINVAL;
	}

	/* 清零整个设备结构体 */
	memset(imu, 0, sizeof(*imu));

	/* 从设备树获取 SPI 设备规格（SPI 模式、频率等在 DTS 中配置）
	 * SPI_OP_MODE_MASTER: 主机模式
	 * SPI_WORD_SET(8):    8 位字长 */
	imu->accel_spi = (struct spi_dt_spec)SPI_DT_SPEC_GET(
		BMI088_ACCEL_NODE, SPI_OP_MODE_MASTER | SPI_WORD_SET(8), 0);
	imu->gyro_spi = (struct spi_dt_spec)SPI_DT_SPEC_GET(
		BMI088_GYRO_NODE, SPI_OP_MODE_MASTER | SPI_WORD_SET(8), 0);

	/* 检查两个 SPI 总线是否都已初始化就绪 */
	if (!spi_is_ready_dt(&imu->accel_spi) ||
	    !spi_is_ready_dt(&imu->gyro_spi)) {
		LOG_ERR("BMI088 SPI is not ready");
		return -ENODEV;
	}

	/* 软复位：向软复位寄存器写入 0xB6，将传感器恢复到默认配置
	 * 复位后需等待 50ms 才能进行后续操作（数据手册规定） */
	(void)spi_write_reg(&imu->accel_spi, BMI088_ACCEL_SOFTRESET, 0xb6);
	(void)spi_write_reg(&imu->gyro_spi, BMI088_GYRO_SOFTRESET, 0xb6);
	k_msleep(50);

	/* 读取芯片 ID 并校验，确认 SPI 通信正常且连接的是 BMI088 */
	uint8_t id = 0;
	int ret = spi_read_reg(&imu->accel_spi, BMI088_ACCEL_CHIP_ID, &id, 1,
			       true);
	if (ret != 0) {
		return ret;
	}
	/* 加速度计 ID 应为 0x1E，不匹配时仅警告（不阻断初始化） */
	if (id != BMI088_ACCEL_ID_VALUE) {
		LOG_WRN("unexpected BMI088 accel chip id 0x%02x", id);
	}

	ret = spi_read_reg(&imu->gyro_spi, BMI088_GYRO_CHIP_ID, &id, 1,
			   false);
	if (ret != 0) {
		return ret;
	}
	/* 陀螺仪 ID 应为 0x0F，不匹配时仅警告 */
	if (id != BMI088_GYRO_ID_VALUE) {
		LOG_WRN("unexpected BMI088 gyro chip id 0x%02x", id);
	}

	/* ========== 加速度计配置 ==========
	 * 配置顺序要求：先开启电源，再配置参数（数据手册规定）
	 */
	/* PWR_CTRL = 0x04: 开启加速度计（ACC_ENABLE = 1） */
	(void)spi_write_reg(&imu->accel_spi, BMI088_ACCEL_PWR_CTRL, 0x04);
	k_msleep(5);  /* 等待加速度计电源稳定 */
	/* PWR_CONF = 0x00: 设为 active 模式（非挂起/低功耗） */
	(void)spi_write_reg(&imu->accel_spi, BMI088_ACCEL_PWR_CONF, 0x00);
	k_msleep(5);  /* 等待模式切换完成 */
	/* CONF = 0xA8: ODR=800Hz, BWP=OSR4（带宽=ODR/4=200Hz，过采样4倍） */
	(void)spi_write_reg(&imu->accel_spi, BMI088_ACCEL_CONF, 0xa8);
	/* RANGE = 0x01: 量程 ±6g（灵敏度 5460 LSB/g） */
	(void)spi_write_reg(&imu->accel_spi, BMI088_ACCEL_RANGE, 0x01);

	/* ========== 陀螺仪配置 ========== */
	/* LPM1 = 0x00: 正常功耗模式（非挂起） */
	(void)spi_write_reg(&imu->gyro_spi, BMI088_GYRO_LPM1, 0x00);
	/* RANGE = 0x00: 量程 ±2000°/s（灵敏度 16.384 LSB/°/s） */
	(void)spi_write_reg(&imu->gyro_spi, BMI088_GYRO_RANGE, 0x00);
	/* BANDWIDTH = 0x02: ODR=1000Hz, 带宽=116Hz（滤波器截止频率） */
	(void)spi_write_reg(&imu->gyro_spi, BMI088_GYRO_BANDWIDTH, 0x02);
	k_msleep(20);  /* 等待陀螺仪配置生效 */

	/* ========== 陀螺仪零偏校准 ==========
	 *
	 * 校准原理：陀螺仪在静止状态下，三个轴的角速度输出理论上应为零，
	 * 但实际上由于制造公差和温度影响，会存在一个非零的固定偏移（零偏/bias）。
	 * 如果不补偿这个零偏，积分得到的角度会随时间快速漂移。
	 *
	 * 校准方法：假设传感器在初始化时保持静止，连续采集 300 个样本取平均值，
	 * 作为零偏补偿量，后续每次读取角速度时减去该零偏。
	 *
	 * 300 个样本，每 2ms 采样一次，总校准时间约 600ms。
	 */
	bmi088_sample_t sample;
	float gyro_sum[3] = { 0.0f, 0.0f, 0.0f };
	const int calibration_count = 1000;

	for (int i = 0; i < calibration_count; i++) {
		memset(&sample, 0, sizeof(sample));
		ret = read_raw(imu, &sample);
		if (ret != 0) {
			return ret;
		}

		/* 累加各轴角速度读数（注意此时 read_raw 尚未减去零偏，
		 * 因为 gyro_offset_dps 还是 0） */
		gyro_sum[0] += sample.gx_dps;
		gyro_sum[1] += sample.gy_dps;
		gyro_sum[2] += sample.gz_dps;
		k_msleep(2);  /* 采样间隔 2ms */
	}

	/* 计算平均值作为零偏补偿量 */
	imu->gyro_offset_dps[0] = gyro_sum[0] / calibration_count;
	imu->gyro_offset_dps[1] = gyro_sum[1] / calibration_count;
	imu->gyro_offset_dps[2] = gyro_sum[2] / calibration_count;

	/* ========== 初始姿态估计 ==========
	 * 用加速度计的当前读数初始化 roll 和 pitch 角
	 * 加速度计在静止时可准确测量重力方向，从而得到初始倾角
	 * yaw 初始设为 0°（无磁力计参考，只能从零开始积分）
	 */
	ret = read_raw(imu, &sample);
	if (ret != 0) {
		return ret;
	}

	/* 初始 roll = atan2(ax, az)，转换为角度 */
	imu->roll_deg = atan2f(sample.ax_g, sample.az_g) * 57.2957795f;
	/* 初始 pitch = atan2(-ay, sqrt(ax^2 + az^2))，转换为角度 */
	imu->pitch_deg = atan2f(-sample.ay_g,
				sqrtf(sample.ax_g * sample.ax_g +
				      sample.az_g * sample.az_g)) *
			 57.2957795f;
	imu->yaw_deg = 0.0f;
	/* 记录初始时间戳，供后续互补滤波器计算 dt */
	imu->last_update_us = now_us();
	/* 标记初始化完成，bmi088_update() 可以开始工作 */
	imu->ready = true;

	LOG_INF("BMI088 ready, gyro offset %.3f %.3f %.3f dps",
		(double)imu->gyro_offset_dps[0],
		(double)imu->gyro_offset_dps[1],
		(double)imu->gyro_offset_dps[2]);
	return 0;
}

/**
 * @brief 读取传感器数据并更新姿态角（主循环调用入口）
 *
 * 本函数应在控制循环中周期性调用（典型频率 500Hz~1kHz）。
 * 执行流程：
 *   1. 参数合法性检查（包括 ready 标志，确保已完成初始化和校准）
 *   2. 读取加速度计和陀螺仪原始数据，换算为物理单位并减去零偏
 *   3. 调用互补滤波器更新 roll/pitch/yaw 姿态角
 *
 * @param imu BMI088 设备实例
 * @param out 输出结构体，包含原始传感器数据和计算后的姿态角
 *            - ax_g/ay_g/az_g: 加速度（g）
 *            - gx_dps/gy_dps/gz_dps: 角速度（°/s，已校准）
 *            - roll_deg/pitch_deg/yaw_deg: 姿态角（°）
 * @return 0 成功，负数表示读取失败或参数无效
 */
int bmi088_update(bmi088_t *imu, bmi088_sample_t *out)
{
	if (imu == NULL || out == NULL || !imu->ready) {
		return -EINVAL;
	}

	/* 读取原始数据并转换为物理单位（含零偏校准） */
	int ret = read_raw(imu, out);
	if (ret != 0) {
		return ret;
	}

	/* 互补滤波器更新姿态角 */
	update_attitude(imu, out);
	return 0;
}

/**
 * @brief 检查 BMI088 是否已完成初始化并可正常使用
 *
 * @param imu BMI088 设备实例
 * @return true 已就绪，false 未初始化或指针为 NULL
 */
bool bmi088_is_ready(const bmi088_t *imu)
{
	return imu != NULL && imu->ready;
}
