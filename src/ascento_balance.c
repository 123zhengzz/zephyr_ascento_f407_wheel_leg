/**
 * @file ascento_balance.c
 * @brief Ascento 风格轮腿机器人核心平衡控制器
 *
 * 本文件实现了一个基于 LQR（线性二次型调节器）的平衡控制器，用于 STM32F407
 * 平台上的轮腿机器人。主要功能包括：
 *
 * 1. 四连杆机构正运动学：将关节角度转换为腿部长度，使用几何圆-圆相交算法
 * 2. LQR 状态反馈平衡控制：基于俯仰角、俯仰角速率、位置、速度四个状态量
 *    计算平衡力矩，转换为左右轮的电流指令
 * 3. 参数持久化存储：由于 STM32F407 的 Flash 扇区大小为 128KB，超过了 Zephyr
 *    NVS 驱动的 uint16_t sector_size 上限（65535 字节），因此采用直接 Flash
 *    读写的方式保存参数，包含魔数、版本号和 CRC32 校验
 * 4. 运行时参数调优：支持通过互斥锁保护的运行时参数结构体进行在线调参
 *
 * 控制算法概述：
 *   平衡力矩 = -(K_pitch * 俯仰误差 + K_pitch_rate * 俯仰角速率
 *                + K_position * 位置误差 + K_velocity * 速度误差)
 *   偏航力矩 = -K_yaw_rate * (偏航角速率 - 目标偏航角速率)
 *   左轮电流 = (平衡力矩 - 偏航力矩) / 力矩常数
 *   右轮电流 = (平衡力矩 + 偏航力矩) / 力矩常数
 */

#include "ascento_balance.h"

#include <math.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "app_config.h"
#include "pid.h"

/* 注册日志模块，日志级别为 INFO */
LOG_MODULE_REGISTER(ascento, LOG_LEVEL_INF);

/* 角度转弧度常量：π / 180 */
#define DEG_TO_RAD 0.017453292519943295f

/**
 * 代码中硬编码的默认参数结构体。
 * 当 Flash 中没有已保存的参数时，将使用这些默认值。
 *
 * 参数分类说明：
 * - 物理参数：轮半径、轮距、总质量、质心高度、质心前偏、转动惯量
 * - 电机映射参数：电流到力矩的转换系数（左轮/右轮分别标定）
 * - 腿部运动学参数：腿部长度范围、关节角度范围
 * - LQR 增益参数：K_pitch（俯仰）、K_pitch_rate（俯仰速率）、
 *   K_position（位置）、K_velocity（速度）、K_yaw_rate（偏航速率）
 * - 运行时可调参数：平衡角、分段增益、静摩擦补偿、电流限制等
 */
static const ascento_balance_params_t ascento_balance_code_defaults = {
	/* 标志位：指示参数是否已完成标定 */
	.calibrated = APP_ASCENTO_PARAMS_CALIBRATED != 0,
	/* 轮子半径 [米] */
	.wheel_radius_m = APP_ASCENTO_WHEEL_RADIUS_M,
	/* 左右轮之间的轮距 [米] */
	.wheel_base_m = APP_ASCENTO_WHEEL_BASE_M,
	/* 机器人总质量 [千克] */
	.total_mass_kg = APP_ASCENTO_TOTAL_MASS_KG,
	/* 质心高度 [米]，用于 LQR 建模 */
	.body_com_height_m = APP_ASCENTO_BODY_COM_HEIGHT_M,
	/* 质心相对于轮轴的前向偏移 [米] */
	.body_com_forward_offset_m = APP_ASCENTO_BODY_COM_FORWARD_OFFSET_M,
	/* 车体绕俯仰轴的转动惯量 [kg*m^2] */
	.body_pitch_inertia_kg_m2 = APP_ASCENTO_BODY_PITCH_INERTIA_KG_M2,
	/* 单个轮子的转动惯量 [kg*m^2] */
	.wheel_inertia_kg_m2 = APP_ASCENTO_WHEEL_INERTIA_KG_M2,
	/* 电流(mA)到轮力矩(N*m)的通用转换系数 */
	.current_ma_to_wheel_torque_nm =
		APP_ASCENTO_CURRENT_MA_TO_WHEEL_TORQUE_NM,
	/* 左轮电流(mA)到轮力矩(N*m)的转换系数（独立标定） */
	.left_current_ma_to_wheel_torque_nm =
		APP_ASCENTO_LEFT_CURRENT_MA_TO_WHEEL_TORQUE_NM,
	/* 右轮电流(mA)到轮力矩(N*m)的转换系数（独立标定） */
	.right_current_ma_to_wheel_torque_nm =
		APP_ASCENTO_RIGHT_CURRENT_MA_TO_WHEEL_TORQUE_NM,
	/* 腿部最短长度 [米] */
	.leg_length_min_m = APP_ASCENTO_LEG_LENGTH_MIN_M,
	/* 腿部最长长度 [米] */
	.leg_length_max_m = APP_ASCENTO_LEG_LENGTH_MAX_M,
	/* 腿部默认长度 [米] */
	.leg_length_default_m = APP_ASCENTO_LEG_LENGTH_DEFAULT_M,
	/* 左腿在最短腿部长度时对应的关节角度 [弧度] */
	.left_joint_at_min_leg_rad = APP_ASCENTO_LEFT_JOINT_AT_MIN_LEG_RAD,
	/* 左腿在最长腿部长度时对应的关节角度 [弧度] */
	.left_joint_at_max_leg_rad = APP_ASCENTO_LEFT_JOINT_AT_MAX_LEG_RAD,
	/* 右腿在最短腿部长度时对应的关节角度 [弧度] */
	.right_joint_at_min_leg_rad = APP_ASCENTO_RIGHT_JOINT_AT_MIN_LEG_RAD,
	/* 右腿在最长腿部长度时对应的关节角度 [弧度] */
	.right_joint_at_max_leg_rad = APP_ASCENTO_RIGHT_JOINT_AT_MAX_LEG_RAD,
	/* LQR 增益：俯仰角误差增益 */
	.k_pitch = APP_ASCENTO_K_PITCH,
	/* LQR 增益：俯仰角速率增益 */
	.k_pitch_rate = APP_ASCENTO_K_PITCH_RATE,
	/* LQR 增益：位置误差增益（用于消除稳态偏移） */
	.k_position = APP_ASCENTO_K_POSITION,
	/* LQR 增益：速度误差增益 */
	.k_velocity = APP_ASCENTO_K_VELOCITY,
	/* 偏航角速率增益（用于差速转向控制） */
	.k_yaw_rate = APP_ASCENTO_K_YAW_RATE,
	/* 横滚角到腿部长度差的映射系数 [米/弧度] */
	.k_roll_to_leg_m_per_rad = APP_ASCENTO_K_ROLL_TO_LEG_M_PER_RAD,

	/* === 以下为运行时可调参数（可通过串口命令在线修改） === */
	/* 平衡角（静态站立时的俯仰角零点偏移）[弧度] */
	.theta_eq_rad = APP_ASCENTO_THETA_EQ_STAND_RAD,
	/* 分段增益 c0 的二次多项式系数：gain = c0_a * x^2 + c0_b * x + c0_c */
	.gain_c0_a = APP_ASCENTO_GAIN_C0_A,
	.gain_c0_b = APP_ASCENTO_GAIN_C0_B,
	.gain_c0_c = APP_ASCENTO_GAIN_C0_C,
	/* 分段增益 c1 的二次多项式系数 */
	.gain_c1_a = APP_ASCENTO_GAIN_C1_A,
	.gain_c1_b = APP_ASCENTO_GAIN_C1_B,
	.gain_c1_c = APP_ASCENTO_GAIN_C1_C,
	/* 分段增益 c2（常数项） */
	.gain_c2   = APP_ASCENTO_GAIN_C2,
	/* 分段增益 c3 的二次多项式系数 */
	.gain_c3_a = APP_ASCENTO_GAIN_C3_A,
	.gain_c3_b = APP_ASCENTO_GAIN_C3_B,
	.gain_c3_c = APP_ASCENTO_GAIN_C3_C,
	/* 静摩擦补偿电流 [mA]，用于克服电机启动时的静摩擦 */
	.stiction_current_ma = APP_ASCENTO_STICTION_CURRENT_MA,
	/* 静摩擦补偿开始生效的角度阈值 [度] */
	.stiction_start_deg  = APP_ASCENTO_STICTION_START_DEG,
	/* 静摩擦补偿完全生效的角度阈值 [度] */
	.stiction_full_deg   = APP_ASCENTO_STICTION_FULL_DEG,
	/* 轮子电流限幅值 [mA]，防止过流损坏电机 */
	.current_limit_ma    = APP_WHEEL_CURRENT_LIMIT,
	/* 电流缩放系数，用于微调实际输出电流 */
	.current_scale       = APP_ASCENTO_WHEEL_CURRENT_SCALE,
	/* 触发故障保护的俯仰角阈值 [度] */
	.fault_deg           = APP_PITCH_FAULT_DEG,
	/* 从故障状态恢复所需的俯仰角误差阈值 [度] */
	.recover_deg         = APP_PITCH_RECOVER_DEG,
	/* 左右轮速度同步增益 [mA/(rad/s)] */
	.wheel_sync_gain_ma  = APP_ASCENTO_WHEEL_SYNC_GAIN_MA,
	/* 左右轮速度同步电流限幅 [mA] */
	.wheel_sync_current_limit_ma = APP_ASCENTO_WHEEL_SYNC_CURRENT_LIMIT,
};

/**
 * 参数访问互斥锁，保护运行时参数结构体的并发读写。
 * 多个线程（主线程、Shell 命令线程）可能同时访问参数，
 * 因此需要互斥保护。
 */
K_MUTEX_DEFINE(params_lock);

/** 运行时参数副本，可在线修改，不影响编译时默认值 */
static ascento_balance_params_t ascento_balance_runtime_params;

/** 标志：运行时参数是否已初始化（首次使用时从默认值拷贝） */
static bool ascento_balance_params_initialized;

/* 向后兼容的别名，供仍然使用旧名称的代码访问默认参数 */
const ascento_balance_params_t *ascento_balance_default_params =
	&ascento_balance_code_defaults;

/**
 * @brief 参数单次初始化函数（懒加载模式）
 *
 * 首次调用时将编译时默认参数拷贝到运行时参数结构体。
 * 使用互斥锁保护，确保多线程环境下只初始化一次。
 */
static void ascento_balance_params_init_once(void)
{
	k_mutex_lock(&params_lock, K_FOREVER);
	if (!ascento_balance_params_initialized) {
		ascento_balance_runtime_params = ascento_balance_code_defaults;
		ascento_balance_params_initialized = true;
	}
	k_mutex_unlock(&params_lock);
}

/**
 * @brief 获取当前运行时参数的副本
 * @param params 输出参数，将运行时参数拷贝到此结构体
 */
void ascento_balance_get_params(ascento_balance_params_t *params)
{
	ascento_balance_params_init_once();
	k_mutex_lock(&params_lock, K_FOREVER);
	*params = ascento_balance_runtime_params;
	k_mutex_unlock(&params_lock);
}

/**
 * @brief 设置运行时参数
 * @param params 输入参数，将此结构体的内容写入运行时参数
 *
 * 用于串口命令在线调参，修改后的参数可通过 save 命令持久化到 Flash。
 */
void ascento_balance_set_params(const ascento_balance_params_t *params)
{
	ascento_balance_params_init_once();
	k_mutex_lock(&params_lock, K_FOREVER);
	ascento_balance_runtime_params = *params;
	ascento_balance_params_initialized = true;
	k_mutex_unlock(&params_lock);
}

/* ================================================================== */
/* 参数持久化存储 —— 直接 Flash 读写（NVS 替代方案）                     */
/* ================================================================== */
/**
 * 为什么不能使用 Zephyr NVS：
 *   STM32F407 的内部 Flash 扇区大小为 128KB（131072 字节），而 Zephyr NVS
 *   驱动的 sector_size 字段为 uint16_t 类型，最大只能表示 65535 字节。
 *   因此无法直接使用 NVS，改为直接操作 Flash 分区。
 *
 * 存储格式：
 *   [Header 12字节] [参数数据 N字节] [CRC32校验 4字节]
 *
 * Header 包含：魔数(4B) + 版本号(4B) + 数据大小(4B)
 *
 * 每次保存时擦除整个分区，然后写入新的 header+payload+CRC 数据块。
 * 这种方式简单可靠，但写入次数受限于 Flash 擦写寿命（约 10000 次）。
 */
#include <zephyr/storage/flash_map.h>
#include <zephyr/sys/crc.h>

/** Flash 分区标签，对应 DTS 中定义的 storage_partition 分区 */
#define PARAMS_FLASH_LABEL  storage_partition
/** 魔数 "ASC P"（ASCII: 0x41='A', 0x53='S', 0x43='C', 0x50='P'），用于校验数据有效性 */
#define PARAMS_MAGIC        0x41534350  /* "ASC P" */
/** 数据格式版本号，结构体字段变化时递增，防止加载不兼容的数据 */
#define PARAMS_VERSION      17

/**
 * Flash 存储头结构体
 * 位于 Flash 分区的起始位置，用于描述后续参数数据的元信息。
 */
struct params_flash_header {
	uint32_t magic;      /**< 魔数，必须等于 PARAMS_MAGIC */
	uint32_t version;    /**< 数据格式版本号，必须等于 PARAMS_VERSION */
	uint32_t data_size;  /**< 参数数据的字节大小，必须等于 sizeof(ascento_balance_params_t) */
};

/**
 * @brief 将运行时参数保存到 Flash
 *
 * 流程：
 *   1. 打开 Flash 分区
 *   2. 构造 header（魔数 + 版本 + 数据大小）
 *   3. 加锁读取运行时参数，拷贝到临时缓冲区
 *   4. 计算 CRC32 校验值，附加到数据末尾
 *   5. 擦除整个分区（Flash 只能按扇区擦除，无法覆盖写入）
 *   6. 写入 header + 参数数据 + CRC
 *
 * @return 0 成功，非零为错误码
 */
int ascento_balance_save_params(void)
{
	ascento_balance_params_init_once();

	const struct flash_area *fa;
	/* 打开 DTS 中定义的 storage_partition 分区 */
	int rc = flash_area_open(FIXED_PARTITION_ID(PARAMS_FLASH_LABEL), &fa);
	if (rc) {
		printk("ascento: flash_area_open failed: %d\n", rc);
		return rc;
	}

	/* 构造存储头 */
	struct params_flash_header hdr = {
		.magic = PARAMS_MAGIC,
		.version = PARAMS_VERSION,
		.data_size = sizeof(ascento_balance_params_t),
	};

	/* 分配缓冲区：header + 参数数据 + 4字节CRC */
	uint8_t buf[sizeof(hdr) + sizeof(ascento_balance_params_t) + 4];
	memcpy(buf, &hdr, sizeof(hdr));

	/* 在互斥锁保护下拷贝运行时参数到缓冲区 */
	k_mutex_lock(&params_lock, K_FOREVER);
	memcpy(buf + sizeof(hdr), &ascento_balance_runtime_params,
	       sizeof(ascento_balance_params_t));
	k_mutex_unlock(&params_lock);

	/* 计算 CRC32 校验（覆盖 header + 参数数据），附加到末尾 */
	uint32_t crc = crc32_ieee(buf, sizeof(hdr) + hdr.data_size);
	memcpy(buf + sizeof(hdr) + hdr.data_size, &crc, sizeof(crc));

	const size_t total = sizeof(hdr) + hdr.data_size + sizeof(crc);

	/* 擦除整个分区（Flash 写入前必须先擦除） */
	rc = flash_area_erase(fa, 0, fa->fa_size);
	if (rc) {
		printk("ascento: flash erase failed: %d\n", rc);
		flash_area_close(fa);
		return rc;
	}

	/* 从分区起始偏移 0 处写入完整数据块 */
	rc = flash_area_write(fa, 0, buf, total);
	flash_area_close(fa);

	if (rc) {
		printk("ascento: flash write failed: %d\n", rc);
	} else {
		printk("ascento: params saved to flash (%u bytes)\n",
		       (unsigned)total);
	}
	return rc;
}

/**
 * @brief 将参数重置为编译时默认值，并清除 Flash 中的保存数据
 *
 * 流程：
 *   1. 将运行时参数恢复为代码中的默认值
 *   2. 擦除 Flash 分区（清除已保存的参数）
 *
 * 即使 Flash 擦除失败，RAM 中的参数也已重置为默认值。
 *
 * @return 0 成功，非零为错误码（Flash 擦除失败）
 */
int ascento_balance_reset_params(void)
{
	/* 先将 RAM 中的运行时参数恢复为编译时默认值 */
	k_mutex_lock(&params_lock, K_FOREVER);
	ascento_balance_runtime_params = ascento_balance_code_defaults;
	ascento_balance_params_initialized = true;
	k_mutex_unlock(&params_lock);

	/* 再擦除 Flash 分区，清除持久化的参数 */
	const struct flash_area *fa;
	int rc = flash_area_open(FIXED_PARTITION_ID(PARAMS_FLASH_LABEL), &fa);
	if (rc) {
		return rc;
	}
	rc = flash_area_erase(fa, 0, fa->fa_size);
	flash_area_close(fa);

	if (rc == 0) {
		printk("ascento: params reset to defaults, flash cleared\n");
	} else {
		printk("ascento: flash erase failed: %d (params reset in RAM)\n",
		       rc);
	}
	return rc;
}

/**
 * @brief 从 Flash 加载已保存的参数（系统启动时调用）
 *
 * 流程：
 *   1. 打开 Flash 分区，读取 header
 *   2. 校验魔数和版本号（不匹配则使用默认值，返回 0）
 *   3. 校验数据大小（不匹配则返回 -EINVAL，说明固件版本不兼容）
 *   4. 读取完整数据块（header + 参数 + CRC）
 *   5. 校验 CRC32（不匹配说明数据损坏，返回 -EIO）
 *   6. 校验通过后，将参数写入运行时参数结构体
 *
 * @return 0 成功（包括无已保存参数的情况），非零为错误码
 */
int ascento_balance_settings_init(void)
{
	ascento_balance_params_init_once();

	const struct flash_area *fa;
	int rc = flash_area_open(FIXED_PARTITION_ID(PARAMS_FLASH_LABEL), &fa);
	if (rc) {
		printk("ascento: flash_area_open failed: %d\n", rc);
		return rc;
	}

	/* 读取 header，检查是否有已保存的参数 */
	struct params_flash_header hdr;
	rc = flash_area_read(fa, 0, &hdr, sizeof(hdr));
	if (rc) {
		printk("ascento: flash read header failed: %d\n", rc);
		flash_area_close(fa);
		return rc;
	}

	/* 魔数或版本号不匹配 → Flash 中无有效参数，使用默认值 */
	if (hdr.magic != PARAMS_MAGIC || hdr.version != PARAMS_VERSION) {
		printk("ascento: no saved params (magic=0x%08x ver=%u)\n",
		       hdr.magic, hdr.version);
		flash_area_close(fa);
		return 0;  /* 不是错误，只是没有已保存的参数 */
	}

	/* 数据大小不匹配 → 固件版本与参数格式不兼容 */
	if (hdr.data_size != sizeof(ascento_balance_params_t)) {
		printk("ascento: params size mismatch: got %u want %u\n",
		       (unsigned)hdr.data_size,
		       (unsigned)sizeof(ascento_balance_params_t));
		flash_area_close(fa);
		return -EINVAL;
	}

	/* 读取完整数据块：header + 参数数据 + CRC32 */
	uint8_t buf[sizeof(hdr) + sizeof(ascento_balance_params_t) + 4];
	const size_t total = sizeof(hdr) + hdr.data_size + sizeof(uint32_t);
	rc = flash_area_read(fa, 0, buf, total);
	flash_area_close(fa);
	if (rc) {
		printk("ascento: flash read data failed: %d\n", rc);
		return rc;
	}

	/* CRC32 校验：对比存储的 CRC 与重新计算的 CRC */
	uint32_t stored_crc;
	memcpy(&stored_crc, buf + sizeof(hdr) + hdr.data_size, sizeof(stored_crc));
	uint32_t calc_crc = crc32_ieee(buf, sizeof(hdr) + hdr.data_size);
	if (stored_crc != calc_crc) {
		printk("ascento: CRC mismatch (stored=0x%08x calc=0x%08x)\n",
		       stored_crc, calc_crc);
		return -EIO;
	}

	/* 校验通过，将参数数据拷贝到运行时参数结构体 */
	ascento_balance_params_t tmp;
	memcpy(&tmp, buf + sizeof(hdr), sizeof(tmp));

	k_mutex_lock(&params_lock, K_FOREVER);
	ascento_balance_runtime_params = tmp;
	ascento_balance_params_initialized = true;
	k_mutex_unlock(&params_lock);

	printk("ascento: loaded params from flash OK\n");
	return 0;
}

/* ================================================================== */
/* 四连杆机构运动学（已启用 —— 用户已确认连杆长度）                      */
/* ================================================================== */
/**
 * 四连杆机构几何说明：
 *
 * Ascento 风格的轮腿机器人采用四连杆（four-bar linkage）机构驱动腿部伸缩。
 * 四连杆由以下构件组成：
 *
 *   B ──── L3 ──── A
 *   |              |
 *   L4             L23（= L2 + L3 的组合连杆）
 *   |              |
 *   C ──── L2 ──── D
 *
 * 关键点：
 *   C - 关节驱动点（原点），由电机通过关节角度控制
 *   B - 固定铰接点（相对于 C 的位置由 L4 和安装角度决定）
 *   D - 连杆 L2 的末端，由 C 和关节角度 qC 决定
 *   A - 圆(B, L3) 与 圆(D, L23) 的交点（四连杆闭合条件）
 *   E - 腿部末端点，沿 D→A 方向延伸 L1 距离
 *
 * 正运动学计算流程：
 *   1. 由关节角度 qC 计算 D 点位置
 *   2. 求解圆-圆相交得到 A 点（四连杆闭合）
 *   3. 沿 D→A 方向延伸得到 E 点（腿部末端）
 *   4. E 点的 y 坐标取负值即为腿部长度
 *
 * 符号约定（2026-05-04 验证）：
 *   qC = offset - joint_angle
 *   其中 offset 是关节零位偏移，由三点标定确定（离散度 < 2.5°）
 *
 * 连杆长度由用户于 2026-05-04 实测确认。
 */

/* 连杆长度 [米]：L1=腿部延伸段, L2=驱动连杆, L3=从动连杆, L4=固定基座 */
#define FB_L1       APP_ASCENTO_FB_L1
#define FB_L2       APP_ASCENTO_FB_L2
#define FB_L3       APP_ASCENTO_FB_L3
#define FB_L4       APP_ASCENTO_FB_L4
/* L23 = L2 + L3 的组合长度，用于圆-圆相交求解 */
#define FB_L23      APP_ASCENTO_FB_L23
/* 左/右腿关节零位偏移 [弧度]，由标定确定 */
#define FB_OFF_LEFT APP_ASCENTO_FB_JOINT_ZERO_OFFSET_LEFT_RAD
#define FB_OFF_RIGHT APP_ASCENTO_FB_JOINT_ZERO_OFFSET_RIGHT_RAD
/* 腿部长度参考偏移 [米]，用于对齐测量基准 */
#define FB_DL       APP_ASCENTO_LEG_LENGTH_REF_OFFSET_M

#define FB_SQRT2 1.41421356237f  /** sqrt(2)，用于 45° 安装角度计算 */
#define FB_EPS   1.0e-6f         /** 浮点数比较容差，避免除零和数值问题 */

/**
 * 二维点结构体，用于四连杆几何计算
 */
typedef struct {
	float x, y;  /**< x/y 坐标 [米] */
} fb_pt_t;

/**
 * 圆-圆相交结果结构体
 * 两个圆最多有两个交点（相切时一个，不相交时零个）
 */
typedef struct {
	fb_pt_t pts[2];  /**< 最多两个交点坐标 */
	uint8_t count;   /**< 实际交点数量：0、1 或 2 */
} fb_circle_hit_t;

/**
 * 四连杆状态结构体，存储正运动学的完整计算结果
 */
typedef struct {
	uint8_t valid;       /**< 计算是否有效（四连杆能否闭合） */
	float qC;            /**< 驱动角度 qC [弧度] */
	float leg_length;    /**< 计算得到的腿部长度 [米] */
	float closure_error; /**< 四连杆闭合误差 [米]（理想情况下为 0） */
	fb_pt_t A, B, C, D, E;  /**< 四连杆各关键点的坐标 */
} fb_state_t;

/* ================================================================== */
/* 二维向量辅助函数                                                     */
/* ================================================================== */

/** 构造二维点 */
static fb_pt_t fb_pt(float x, float y)
{
	fb_pt_t p = { x, y };
	return p;
}

/** 向量加法：a + b */
static fb_pt_t fb_add(fb_pt_t a, fb_pt_t b)
{
	return fb_pt(a.x + b.x, a.y + b.y);
}

/** 向量减法：a - b */
static fb_pt_t fb_sub(fb_pt_t a, fb_pt_t b)
{
	return fb_pt(a.x - b.x, a.y - b.y);
}

/** 向量数乘：p * k */
static fb_pt_t fb_scale(fb_pt_t p, float k)
{
	return fb_pt(p.x * k, p.y * k);
}

/** 向量模长：|p| = sqrt(x^2 + y^2) */
static float fb_norm(fb_pt_t p)
{
	return sqrtf(p.x * p.x + p.y * p.y);
}

/** 两点间距离：|a - b| */
static float fb_dist(fb_pt_t a, fb_pt_t b)
{
	return fb_norm(fb_sub(a, b));
}

/**
 * @brief 求解两个圆的交点（圆-圆相交算法）
 *
 * 几何原理：
 *   给定圆 c1(圆心 c1, 半径 r1) 和圆 c2(圆心 c2, 半径 r2)，
 *   求它们的交点。算法基于以下步骤：
 *
 *   1. 计算两圆心距离 d = |c2 - c1|
 *   2. 排除不相交的情况（d=0、d>r1+r2、d<|r1-r2|）
 *   3. 沿圆心连线方向计算交点到连线中垂线的距离
 *   4. 沿中垂线方向偏移得到两个交点（或一个切点）
 *
 * @param c1   第一个圆的圆心坐标
 * @param r1   第一个圆的半径
 * @param c2   第二个圆的圆心坐标
 * @param r2   第二个圆的半径
 * @return     交点结果（0、1 或 2 个交点）
 */
static fb_circle_hit_t fb_circle_intersect(fb_pt_t c1, float r1,
					   fb_pt_t c2, float r2)
{
	fb_circle_hit_t hit = { .count = 0 };
	/* 计算圆心连线向量及其长度 */
	fb_pt_t dvec = fb_sub(c2, c1);
	float d = fb_norm(dvec);

	/* 以下三种情况两圆不相交 */
	if (d < FB_EPS) return hit;              /* 两圆同心，无交点 */
	if (d > r1 + r2 + FB_EPS) return hit;    /* 两圆分离，无交点 */
	if (d < fabsf(r1 - r2) - FB_EPS) return hit;  /* 一圆包含另一圆，无交点 */

	/* 沿圆心连线方向的单位向量 */
	fb_pt_t ex = fb_scale(dvec, 1.0f / d);
	/* 计算交点连线中点到 c1 的距离 a（余弦定理推导） */
	float a = (r1 * r1 - r2 * r2 + d * d) / (2.0f * d);
	/* 计算交点到圆心连线的垂直距离 h */
	float h2 = r1 * r1 - a * a;
	float h = (h2 > 0.0f) ? sqrtf(h2) : 0.0f;

	/* 连线上的基点 p0 = c1 + a * ex */
	fb_pt_t p0 = fb_add(c1, fb_scale(ex, a));
	/* 垂直于连线方向的单位向量（将 ex 旋转 90°） */
	fb_pt_t ey = fb_pt(-ex.y, ex.x);

	if (h <= FB_EPS) {
		/* 两圆相切，只有一个交点 */
		hit.pts[0] = p0;
		hit.count = 1;
	} else {
		/* 两圆相交，两个交点分别在连线两侧 */
		hit.pts[0] = fb_add(p0, fb_scale(ey, h));
		hit.pts[1] = fb_sub(p0, fb_scale(ey, h));
		hit.count = 2;
	}
	return hit;
}

/**
 * @brief 获取固定铰接点 B 的坐标
 *
 * B 点相对于 C 点（原点）的位置由连杆长度 L4 和 45° 安装角度决定。
 * 使用 sqrt(2) 进行 45° 三角函数计算（cos45° = sin45° = 1/sqrt(2)）。
 *
 * @param side  腿的侧别：+1 = 右腿，-1 = 左腿（影响 B 点 x 坐标的正负号）
 * @return      B 点坐标
 */
static fb_pt_t fb_get_B(int8_t side)
{
	int8_t s = (side >= 0) ? 1 : -1;
	/* B 点位于 C 点的 45° 方向，距离为 L4 */
	/* 右腿 B.x 为负，左腿 B.x 为正（镜像对称） */
	return fb_pt(-(float)s * FB_L4 / FB_SQRT2, FB_L4 / FB_SQRT2);
}

/**
 * @brief 四连杆正运动学：关节角度 qC → 腿部长度
 *
 * 计算步骤：
 *   1. C 点设为原点 (0, 0)
 *   2. B 点由连杆长度 L4 和侧别决定（固定铰接点）
 *   3. D 点由 C 点沿 qC 方向偏移 L2 距离得到：
 *      D = C - L2 * [cos(qC), sin(qC)]
 *   4. A 点是 圆(B, L3) 和 圆(D, L23) 的交点（四连杆闭合条件）
 *   5. 从两个交点中选择外侧的解（物理上正确的构型）
 *   6. E 点沿 D→A 方向延伸 L1 距离：
 *      E = D + L1 * (D - A) / |D - A|
 *   7. 腿部长度 = -E.y（E 点在 C 点下方，y 为负，取负得到正值）
 *
 * @param qC   驱动角度 [弧度]（由 offset - joint_angle 计算）
 * @param side 腿的侧别：+1 = 右腿，-1 = 左腿
 * @param st   输出：四连杆状态（各关键点坐标、腿部长度等）
 * @return     true = 四连杆可以闭合（计算成功），false = 无法闭合
 */
static bool fb_forward(float qC, int8_t side, fb_state_t *st)
{
	if (st == NULL) return false;

	memset(st, 0, sizeof(*st));

	/* 步骤 1：C 点为原点（关节驱动点） */
	st->C = fb_pt(0.0f, 0.0f);
	/* 步骤 2：B 点为固定铰接点 */
	st->B = fb_get_B(side);
	st->qC = qC;

	/* 步骤 3：D 点 = C 点沿 qC 反方向偏移 L2 */
	/* 负号表示 D 点在 C 点的 qC 反方向（向腿部方向延伸） */
	st->D = fb_pt(-FB_L2 * cosf(qC), -FB_L2 * sinf(qC));

	/* 步骤 4：求解 A 点 = 圆(B, L3) ∩ 圆(D, L23) */
	fb_circle_hit_t cand = fb_circle_intersect(st->B, FB_L3, st->D, FB_L23);
	if (cand.count == 0) return false;  /* 无法闭合，返回失败 */

	/* 步骤 5：从候选交点中选择外侧的解（物理上正确的构型）
	 * 对于右腿(side=+1)，选择 x 坐标更负（更靠左）的点
	 * 对于左腿(side=-1)，选择 x 坐标更正（更靠右）的点
	 * score = -s * x，score 越大表示越靠外侧
	 */
	float best_score = -1.0e30f;
	uint8_t best_idx = 0;
	int8_t s = (side >= 0) ? 1 : -1;

	for (uint8_t i = 0; i < cand.count; i++) {
		float score = -(float)s * cand.pts[i].x;
		if (score > best_score) {
			best_score = score;
			best_idx = i;
		}
	}

	/* 步骤 6：计算 E 点（腿部末端） */
	st->A = cand.pts[best_idx];
	/* D→A 方向的单位向量 */
	fb_pt_t dir = fb_scale(fb_sub(st->D, st->A), 1.0f / FB_L23);
	/* E 点 = D 点沿 D→A 方向再延伸 L1 距离 */
	st->E = fb_add(st->D, fb_scale(dir, FB_L1));
	/* 步骤 7：腿部长度 = -E.y（E 在下方，y 为负） */
	st->leg_length = -st->E.y;
	/* 闭合误差：|A-B| 与 L3 的差值（理想情况下为 0） */
	st->closure_error = fabsf(fb_dist(st->A, st->B) - FB_L3);
	st->valid = 1;
	return true;
}

/**
 * @brief 关节角度 → 腿部长度（带参考偏移校正）
 *
 * 将物理关节角度转换为四连杆驱动角度 qC，然后调用正运动学计算腿部长度。
 * 最后减去参考偏移 FB_DL，使腿部长度与实际测量值对齐。
 *
 * 转换公式：qC = offset - joint_rad
 * 其中 offset 是关节零位偏移（由标定确定）
 *
 * @param joint_rad  物理关节角度 [弧度]
 * @param side       腿的侧别：+1 = 右腿，-1 = 左腿
 * @param st         输出：四连杆状态
 * @return           true = 计算成功，false = 四连杆无法闭合
 */
static bool fb_leg_length_from_joint(float joint_rad, int8_t side,
				     fb_state_t *st)
{
	/* 根据侧别选择对应的零位偏移量 */
	float offset = (side >= 0) ? FB_OFF_RIGHT : FB_OFF_LEFT;
	/* 将物理关节角度转换为四连杆驱动角度 */
	float qC = offset - joint_rad;
	/* 调用正运动学计算 */
	if (!fb_forward(qC, side, st)) return false;
	/* 减去参考偏移，使腿部长度与实际测量基准对齐 */
	st->leg_length -= FB_DL;
	return true;
}

/* ================================================================== */
/* 公共映射函数（基于四连杆模型）                                        */
/* ================================================================== */

/**
 * @brief 获取轮子前进方向的电流符号
 *
 * 不同轮子的安装方向可能不同，导致正电流对应的旋转方向也不同。
 * 此函数返回正确的符号，使得正电流 = 正向旋转 = 前进。
 *
 * @param left_wheel  true = 左轮，false = 右轮
 * @return            +1.0f 或 -1.0f，取决于轮子的安装方向
 */
static float wheel_forward_sign(bool left_wheel)
{
	return left_wheel ? (float)APP_WHEEL_LEFT_FORWARD_CURRENT_SIGN :
			    (float)APP_WHEEL_RIGHT_FORWARD_CURRENT_SIGN;
}

/**
 * @brief 初始化平衡控制器状态
 * @param state  控制器状态结构体指针
 */
void ascento_balance_init(ascento_balance_state_t *state)
{
	ascento_balance_reset(state);
}

/**
 * @brief 重置平衡控制器状态（清零所有内部变量）
 *
 * 在故障恢复、禁用控制或系统重启时调用，
 * 清除累积的位置、速度滤波器等状态。
 *
 * @param state  控制器状态结构体指针
 */
void ascento_balance_reset(ascento_balance_state_t *state)
{
	if (state != NULL) {
		memset(state, 0, sizeof(*state));
	}
}

/**
 * @brief 检查参数是否已准备就绪（可用于控制）
 *
 * 必要条件：
 *   - 参数不为 NULL
 *   - calibrated 标志为 true（已完成标定）
 *   - 轮半径、轮距、质量、质心高度均大于 0
 *   - 左右轮的电流-力矩转换系数大于 0
 *   - 腿部最大长度 > 最小长度
 *
 * @param params  参数结构体指针
 * @return        true = 参数有效，可用于控制计算
 */
bool ascento_balance_params_ready(const ascento_balance_params_t *params)
{
	if (params == NULL || !params->calibrated) {
		return false;
	}

	return params->wheel_radius_m > 0.0f &&
	       params->wheel_base_m > 0.0f &&
	       params->total_mass_kg > 0.0f &&
	       params->body_com_height_m > 0.0f &&
	       params->left_current_ma_to_wheel_torque_nm > 0.0f &&
	       params->right_current_ma_to_wheel_torque_nm > 0.0f &&
	       params->leg_length_max_m > params->leg_length_min_m;
}

/**
 * 分段线性关节角度 ↔ 腿部长度查找表（备用方案）
 *
 * 这是四连杆解析模型的备用方案，通过标定点进行线性插值。
 * 当前已被四连杆解析模型取代（#if 0 表示未编译）。
 * 保留此代码作为参考，以防需要回退到查找表方式。
 *
 * 查找表结构：每个标定点包含关节角度(q_rad)和对应的腿部长度(L_m)。
 * 正向查找：关节角度 → 腿部长度（线性插值）
 * 反向查找：腿部长度 → 关节角度（反向线性插值）
 */
#if 0

struct leg_cal_pt { float q_rad; float L_m; };

/**
 * @brief 正向查找：关节角度 → 腿部长度（分段线性插值）
 * @param q  关节角度 [弧度]
 * @param t  标定查找表数组
 * @param n  查找表元素个数
 * @return   插值得到的腿部长度 [米]
 */
static float leg_lookup_fwd(float q, const struct leg_cal_pt *t, int n)
{
	if (q <= t[0].q_rad) return t[0].L_m;
	if (q >= t[n - 1].q_rad) return t[n - 1].L_m;
	for (int i = 0; i < n - 1; i++) {
		if (q <= t[i + 1].q_rad) {
			float r = (q - t[i].q_rad) / (t[i + 1].q_rad - t[i].q_rad);
			return t[i].L_m + r * (t[i + 1].L_m - t[i].L_m);
		}
	}
	return t[n - 1].L_m;
}

/**
 * @brief 反向查找：腿部长度 → 关节角度（分段线性插值）
 * @param L  腿部长度 [米]
 * @param t  标定查找表数组
 * @param n  查找表元素个数
 * @return   插值得到的关节角度 [弧度]
 */
static float leg_lookup_inv(float L, const struct leg_cal_pt *t, int n)
{
	bool L_rises = (t[n - 1].L_m > t[0].L_m);

	if (L_rises) {
		if (L <= t[0].L_m) return t[0].q_rad;
		if (L >= t[n - 1].L_m) return t[n - 1].q_rad;
	} else {
		if (L >= t[0].L_m) return t[0].q_rad;
		if (L <= t[n - 1].L_m) return t[n - 1].q_rad;
	}

	for (int i = 0; i < n - 1; i++) {
		float La = t[i].L_m;
		float Lb = t[i + 1].L_m;
		bool in_seg = L_rises ? (L >= La && L <= Lb)
				      : (L <= La && L >= Lb);
		if (in_seg) {
			float r = fabsf((L - La) / (Lb - La));
			return t[i].q_rad + r * (t[i + 1].q_rad - t[i].q_rad);
		}
	}
	return t[n - 1].q_rad;
}

#endif /* piecewise-linear backup */

/**
 * @brief 公共接口：关节角度 → 腿部长度
 *
 * 使用四连杆正运动学将物理关节角度转换为腿部长度。
 * 如果四连杆无法闭合，返回最小腿部长度作为安全值。
 *
 * @param params     参数结构体（当前未使用，保留接口兼容性）
 * @param left_leg   true = 左腿，false = 右腿
 * @param joint_rad  关节角度 [弧度]
 * @return           腿部长度 [米]
 */
float ascento_balance_leg_length_from_joint(const ascento_balance_params_t *params,
					    bool left_leg, float joint_rad)
{
	(void)params;  /* 参数未使用，保留接口兼容性 */
	int8_t side = left_leg ? -1 : 1;
	fb_state_t st;
	if (fb_leg_length_from_joint(joint_rad, side, &st)) {
		return st.leg_length;
	}
	/* 四连杆无法闭合时返回最小腿部长度（安全值） */
	return APP_ASCENTO_LEG_LENGTH_MIN_M;
}

/**
 * @brief 从参数结构体中提取四个固定的 LQR 平衡增益
 *
 * 这四个增益对应 LQR 状态反馈的四个状态量：
 *   - k_pitch: 俯仰角误差增益 [N*m/rad]
 *   - k_pitch_rate: 俯仰角速率增益 [N*m/(rad/s)]
 *   - k_position: 位置误差增益 [N*m/m]
 *   - k_velocity: 速度误差增益 [N*m/(m/s)]
 *
 * @param p           参数结构体
 * @param k_pitch     输出：俯仰角增益
 * @param k_pitch_rate 输出：俯仰角速率增益
 * @param k_position  输出：位置增益
 * @param k_velocity  输出：速度增益
 */
static void get_fixed_gains(const ascento_balance_params_t *p,
			    float *k_pitch, float *k_pitch_rate,
			    float *k_position, float *k_velocity)
{
	*k_pitch = p->k_pitch;
	*k_pitch_rate = p->k_pitch_rate;
	*k_position = p->k_position;
	*k_velocity = p->k_velocity;
}

/**
 * @brief LQR 平衡控制器主更新函数
 *
 * 这是整个平衡控制器的核心函数，在每个控制周期（约 1000Hz）被调用。
 * 主要流程：
 *   1. 参数有效性检查
 *   2. 从轮子编码器读取轮子角度和速度，计算行驶距离和速度
 *   3. 从 IMU 读取俯仰角、俯仰角速率、偏航角速率
 *   4. 计算状态误差（俯仰误差、位置误差、速度误差）
 *   5. 应用 LQR 状态反馈律计算平衡力矩
 *   6. 应用偏航控制律计算偏航力矩
 *   7. 将力矩转换为左右轮电流指令
 *   8. 应用电流限幅和速度同步补偿
 *   9. 故障检测和恢复逻辑
 *
 * @param state   控制器内部状态（滤波器状态、位置累积等）
 * @param params  控制器参数（增益、物理参数等）
 * @param input   输入数据（IMU、编码器、目标值等）
 * @param output  输出数据（轮子电流、诊断信息等）
 */
void ascento_balance_update(ascento_balance_state_t *state,
			    const ascento_balance_params_t *params,
			    const ascento_balance_input_t *input,
			    ascento_balance_output_t *output)
{
	/* --- 步骤 0：空指针检查 --- */
	if (output == NULL) {
		return;
	}

	/* 清零输出结构体，确保未赋值字段为 0 */
	memset(output, 0, sizeof(*output));

	if (state == NULL || params == NULL || input == NULL) {
		return;
	}

	/* --- 步骤 1：检查参数是否已标定 --- */
	output->params_ready = ascento_balance_params_ready(params);
	if (!output->params_ready) {
		/* 参数未就绪，每 1024 次打印一次警告（避免刷屏） */
		static uint32_t diag_cnt;
		if ((diag_cnt++ & 0x3ff) == 0) {
			LOG_WRN("model blocked: params_ready=%d calib=%d",
				output->params_ready, params->calibrated);
		}
		/* 重置控制器状态，防止累积误差 */
		ascento_balance_reset(state);
		return;
	}

	/* --- 步骤 2：获取控制周期 dt --- */
	float dt_s = input->dt_s;
	/* dt 有效性检查：必须大于 0 且不超过 20ms */
	if (dt_s <= 0.0f || dt_s > 0.02f) {
		dt_s = 1.0f / APP_CONTROL_HZ;
	}

	/* --- 步骤 3：处理轮子编码器数据 --- */
	/* 将电机端角度转换为轮子端角度：除以减速比，乘以方向符号 */
	const float left_wheel_angle =
		wheel_forward_sign(true) * input->left_wheel.angle_rad /
		APP_M3508_REDUCTION_RATIO;
	const float right_wheel_angle =
		wheel_forward_sign(false) * input->right_wheel.angle_rad /
		APP_M3508_REDUCTION_RATIO;
	/* 将电机端角速度转换为轮子端角速度 */
	const float left_wheel_speed =
		wheel_forward_sign(true) * input->left_wheel.speed_rad_s /
		APP_M3508_REDUCTION_RATIO;
	const float right_wheel_speed =
		wheel_forward_sign(false) * input->right_wheel.speed_rad_s /
		APP_M3508_REDUCTION_RATIO;

	/* 左右轮平均角度 × 轮半径 = 行驶距离 [米] */
	const float wheel_distance_m =
		0.5f * (left_wheel_angle + right_wheel_angle) *
		params->wheel_radius_m;
	/* 左右轮平均角速度 × 轮半径 = 行驶速度 [m/s] */
	const float body_velocity_mps =
		0.5f * (left_wheel_speed + right_wheel_speed) *
		params->wheel_radius_m;

	/* --- 步骤 4：位置和速度状态计算 --- */
	if (!state->initialized) {
		/* 首次调用：记录初始轮子位置作为零点 */
		state->wheel_position_zero_m = wheel_distance_m;
		state->body_position_m = 0.0f;
		/* 初始化速度低通滤波器 */
		state->body_velocity_lpf_mps = body_velocity_mps;
		/* 初始化偏航角速率低通滤波器 */
		state->yaw_rate_lpf_rad_s = input->imu.gz_dps * DEG_TO_RAD;
		/* 初始化俯仰角速率低通滤波器 */
		state->pitch_rate_lpf_rad_s = APP_ASCENTO_IMU_GY_SIGN *
					      input->imu.gx_dps * DEG_TO_RAD;
		state->initialized = true;
	} else {
		/* 后续调用：位置 = 当前轮子距离 - 初始零点距离 */
		state->body_position_m =
			wheel_distance_m - state->wheel_position_zero_m;
	}

	/* 速度低通滤波（截止频率约 13Hz，时间常数 0.012s） */
	const float velocity_mps = app_lpf_update(body_velocity_mps,
						  &state->body_velocity_lpf_mps,
						  0.020f, dt_s);

	/* --- 步骤 5：处理 IMU 传感器数据 --- */
	/* 俯仰角 [弧度]：乘以符号因子以统一坐标系方向 */
	const float pitch_rad = APP_ASCENTO_IMU_PITCH_SIGN *
				 input->imu.pitch_deg * DEG_TO_RAD;
	/* 俯仰角速率 [rad/s]：IMU 的 gx 分量对应俯仰轴，经过低通滤波 */
	const float pitch_rate_rad_s = APP_ASCENTO_IMU_GY_SIGN *
				       app_lpf_update(input->imu.gx_dps * DEG_TO_RAD,
						      &state->pitch_rate_lpf_rad_s,
						      0.010f, dt_s);
	/* 偏航角速率 [rad/s]：IMU 的 gz 分量对应偏航轴，经过低通滤波 */
	const float yaw_rate_rad_s = APP_ASCENTO_IMU_GZ_SIGN *
				     app_lpf_update(input->imu.gz_dps * DEG_TO_RAD,
						    &state->yaw_rate_lpf_rad_s,
						    0.012f, dt_s);

	/* --- 步骤 6：计算状态误差 --- */
	/* 位置误差 [米]：当前位置（相对于启动时的位置），限幅防止积分漂移 */
	const float x_error = app_clampf(state->body_position_m, -0.30f, 0.30f);
	/* 速度误差 [m/s]：当前速度 - 目标前进速度 */
	const float v_error = velocity_mps - input->target_forward_speed_mps;

	/* --- 步骤 7：提取 LQR 增益 --- */
	float k_pitch, k_pitch_rate, k_position, k_velocity;
	get_fixed_gains(params, &k_pitch, &k_pitch_rate, &k_position,
			&k_velocity);

	/* --- 步骤 8：计算俯仰角误差 --- */
	/* theta_eq: 平衡角（静态站立时的俯仰角零点偏移）[弧度] */
	const float theta_eq = params->theta_eq_rad;
	/* 俯仰误差 = 当前俯仰角 - 平衡角 - 目标俯仰角
	 * 当机器人直立且无目标俯仰时，pitch_error ≈ 0 */
	const float pitch_error = pitch_rad - theta_eq - input->target_pitch_rad;
	const float pitch_deg = pitch_rad / DEG_TO_RAD;

	/* --- 步骤 9：故障检测与恢复逻辑 --- */
	/* 运行时故障角度阈值（钳位到合理范围） */
	const float runtime_fault_deg =
		app_clampf(params->fault_deg, 5.0f,
			   APP_ASCENTO_FORWARD_HARD_FAULT_DEG);
	/* 前倾故障阈值：取运行时参数和硬件安全限制的较小值 */
	const float forward_fault_deg =
		fminf(runtime_fault_deg, APP_ASCENTO_FORWARD_HARD_FAULT_DEG);
	/* 后仰故障阈值 */
	const float backward_fault_deg =
		fminf(runtime_fault_deg, APP_ASCENTO_BACKWARD_HARD_FAULT_DEG);
	const bool over_forward_limit = pitch_deg > forward_fault_deg;
	const bool over_backward_limit = pitch_deg < -backward_fault_deg;

	/* 如果未使能或轮子反馈异常，重置控制器并返回 */
	if (!input->enable_request || !input->wheel_feedback_ok) {
		ascento_balance_reset(state);
		return;
	}

	/* 触发故障保护：俯仰角超过安全阈值 */
	if (over_forward_limit || over_backward_limit) {
		state->faulted = true;
		state->recover_ticks = 0;
	}

	/* 故障恢复逻辑：需要在恢复角度阈值内持续保持稳定一段时间 */
	if (state->faulted) {
		/* 恢复角度阈值 [度]（钳位到合理范围） */
		const float recover_err_deg =
			app_clampf(params->recover_deg, 1.0f,
				   APP_ASCENTO_STAND_RECOVER_ERR_DEG);

		if (fabsf(pitch_error / DEG_TO_RAD) < recover_err_deg) {
			/* 俯仰角在恢复阈值内，累加恢复计数 */
			state->recover_ticks++;
			if (state->recover_ticks > APP_BALANCE_RECOVER_TICKS) {
				/* 持续稳定足够长时间，解除故障状态 */
				state->faulted = false;
				state->recover_ticks = 0;
				/* 重置位置零点和滤波器，避免恢复后跳变 */
				state->wheel_position_zero_m = wheel_distance_m;
				state->body_position_m = 0.0f;
				state->body_velocity_lpf_mps = body_velocity_mps;
				state->yaw_rate_lpf_rad_s =
					input->imu.gz_dps * DEG_TO_RAD;
			}
		} else {
			/* 俯仰角超出恢复阈值，重置恢复计数 */
			state->recover_ticks = 0;
		}
	}

	/* 如果处于故障状态，不输出控制指令 */
	output->faulted = state->faulted;
	if (state->faulted) {
		return;
	}

	/* ================================================================ */
	/* 步骤 10：LQR 状态反馈控制律 —— 计算平衡力矩                        */
	/* ================================================================ */
	/**
	 * LQR 控制律（线性二次型调节器）：
	 *
	 * 平衡力矩 = -[K_pitch * (θ - θ_eq - θ_target)
	 *            + K_pitch_rate * dθ/dt
	 *            + K_position * x
	 *            + K_velocity * (v - v_target)]
	 *
	 * 其中：
	 *   θ:       当前俯仰角 [rad]
	 *   θ_eq:    平衡角（静态站立零点偏移）[rad]
	 *   θ_target: 目标俯仰角 [rad]
	 *   dθ/dt:   俯仰角速率 [rad/s]
	 *   x:       位置误差（相对于启动位置）[m]
	 *   v:       当前速度 [m/s]
	 *   v_target: 目标前进速度 [m/s]
	 *
	 * 负号表示力矩方向与误差方向相反（负反馈）。
	 * 这是一个简化的 LQR，省略了腿部长度相关的增益调度。
	 */
	float balance_torque_nm =
		-(k_pitch * pitch_error +
		  k_pitch_rate * pitch_rate_rad_s +
		  k_position * x_error +
		  k_velocity * v_error);

	/* ================================================================ */
	/* 步骤 11：偏航控制律 —— 计算偏航力矩                                */
	/* ================================================================ */
	/**
	 * 偏航力矩 = -K_yaw_rate * (当前偏航角速率 - 目标偏航角速率)
	 * 通过差速控制实现转向：偏航力矩使左右轮产生速度差。
	 */
	float yaw_torque_nm =
		-params->k_yaw_rate * (yaw_rate_rad_s -
				       input->target_yaw_rate_rad_s);

	/* ================================================================ */
	/* 步骤 12：力矩 → 电流转换 + 电流限幅                                */
	/* ================================================================ */
	output->active = true;
	/**
	 * 力矩到电流的转换：
	 *   左轮电流 = (平衡力矩 - 偏航力矩) / 左轮力矩常数 * 缩放系数
	 *   右轮电流 = (平衡力矩 + 偏航力矩) / 右轮力矩常数 * 缩放系数
	 *
	 * 偏航力矩的符号：减去偏航力矩使左轮减速/右轮加速，实现右转。
	 * 左右轮的力矩常数分别标定，因为电机特性可能存在差异。
	 */
	float left_current_ma =
		(balance_torque_nm - yaw_torque_nm) /
		params->left_current_ma_to_wheel_torque_nm *
		params->current_scale;
	float right_current_ma =
		(balance_torque_nm + yaw_torque_nm) /
		params->right_current_ma_to_wheel_torque_nm *
		params->current_scale;

	/* 电流限幅处理 */
	int32_t current_limit_ma = params->current_limit_ma;
	/* 确保限幅值为正数 */
	if (current_limit_ma < 0) {
		current_limit_ma = -current_limit_ma;
	}
	/* 如果限幅值为 0 或负数，使用 INT16_MAX 作为安全上限 */
	if (current_limit_ma <= 0) {
		current_limit_ma = INT16_MAX;
	}
	/* 将浮点电流值钳位到 int16 范围 [-current_limit_ma, +current_limit_ma] */
	output->left_wheel_current = app_clamp_i16(
		(int32_t)lrintf(left_current_ma),
		-current_limit_ma, current_limit_ma);
	output->right_wheel_current = app_clamp_i16(
		(int32_t)lrintf(right_current_ma),
		-current_limit_ma, current_limit_ma);

	/* ================================================================ */
	/* 步骤 13：左右轮速度同步补偿                                        */
	/* ================================================================ */
	/**
	 * 左右轮速度同步：补偿左右轮速度不一致的问题。
	 *
	 * 原理：如果左轮比右轮快，给右轮增加电流、左轮减少电流，
	 * 使两者速度趋于一致。这不影响平均力矩（用于平衡），
	 * 只消除左右轮之间的速度差（用于直线行驶稳定性）。
	 *
	 * 同步电流 = K_sync * (左轮速度 - 右轮速度)
	 * 左轮电流 -= 同步电流
	 * 右轮电流 += 同步电流
	 */
	if (params->wheel_sync_gain_ma > 0.0f) {
		const float left_speed = wheel_forward_sign(true) *
			input->left_wheel.speed_rad_s /
			APP_M3508_REDUCTION_RATIO;
		const float right_speed = wheel_forward_sign(false) *
			input->right_wheel.speed_rad_s /
			APP_M3508_REDUCTION_RATIO;
		/* 速度差 = 左轮速度 - 右轮速度 */
		const float speed_error = left_speed - right_speed;
		/* 同步电流，钳位到限幅范围内 */
		const float sync_current = app_clampf(
			params->wheel_sync_gain_ma * speed_error,
			-params->wheel_sync_current_limit_ma,
			params->wheel_sync_current_limit_ma);
		/* 左轮减去同步电流，右轮加上同步电流（消除速度差） */
		output->left_wheel_current -= (int16_t)lrintf(sync_current);
		output->right_wheel_current += (int16_t)lrintf(sync_current);
	}

	/* ================================================================ */
	/* 步骤 14：关节位置指令（平衡模式下锁定关节角度）                      */
	/* ================================================================ */
	/* 在平衡模式下，腿部关节锁定在固定角度，不进行腿部伸缩控制 */
	output->left_joint_position_rad = APP_PID_BALANCE_LOCK_LEFT_JOINT_RAD;
	output->right_joint_position_rad = APP_PID_BALANCE_LOCK_RIGHT_JOINT_RAD;

	/* ================================================================ */
	/* 步骤 15：诊断日志输出（约 1Hz）                                    */
	/* ================================================================ */
	/* 诊断日志：每 1024 次打印一次（~1Hz），pitch 大时每 64 次打印（~3Hz） */
	{
		static uint32_t diag_active_cnt;
		const bool pitch_large = fabsf(pitch_deg) > 8.0f;
		const uint32_t mask = pitch_large ? 0x3f : 0x3ff;
		if ((diag_active_cnt++ & mask) == 0) {
			LOG_INF("pitch=%.2f err=%.2f p_term=%.3f pr_term=%.3f "
				"pos_term=%.4f vel_term=%.4f torque=%.3f "
				"v=%.3f theta_eq=%.3f",
				(double)pitch_deg,
				(double)(pitch_error / DEG_TO_RAD),
				(double)(-k_pitch * pitch_error),
				(double)(-k_pitch_rate * pitch_rate_rad_s),
				(double)(-k_position * x_error),
				(double)(-k_velocity * v_error),
				(double)balance_torque_nm,
				(double)velocity_mps,
				(double)(theta_eq / DEG_TO_RAD));
		}
	}

	/* ================================================================ */
	/* 步骤 16：填充输出结构体的诊断字段                                   */
	/* ================================================================ */
	/* 关节角速度限幅 [rad/s]（用于关节 PID 控制器） */
	output->joint_velocity_limit_rad_s = APP_LEG_VEL_LIMIT_RAD_S;
	/* 车体位置 [米]（相对于启动位置） */
	output->body_position_m = state->body_position_m;
	/* 车体速度 [m/s]（低通滤波后） */
	output->body_velocity_mps = velocity_mps;
	/* 俯仰角 [弧度] */
	output->pitch_rad = pitch_rad;
	/* 俯仰角速率 [rad/s] */
	output->pitch_rate_rad_s = pitch_rate_rad_s;
	/* 平衡力矩 [N*m]（LQR 输出） */
	output->balance_torque_nm = balance_torque_nm;
	/* 偏航力矩 [N*m]（偏航控制输出） */
	output->yaw_torque_nm = yaw_torque_nm;
}
