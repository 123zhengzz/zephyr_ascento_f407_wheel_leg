/**
 * @file dji_m3508.c
 * @brief DJI M3508 电机驱动 -- 基于 VESC CAN 协议通信
 *
 * 本模块通过 VESC 电调的 CAN 扩展帧协议控制 DJI M3508 无刷电机。
 *
 * VESC CAN 协议要点：
 *   - 使用 29 位扩展帧 ID，高字节为包类型 (packet_id)，低字节为控制器 ID (controller_id)
 *   - 发送命令：SET_CURRENT (packet_id=1) 以毫安为单位设定电流
 *             SET_RPM    (packet_id=3) 以 ERPM 为单位设定转速
 *   - 接收状态：STATUS_1  (packet_id=9)  ERPM / 电流 / 占空比
 *             STATUS_4  (packet_id=16) MOS管温度 / 电机温度 / 输入电流
 *             STATUS_5  (packet_id=27) 里程计 / 输入电压
 *
 * 本驱动用于 Ascento 轮腿机器人的轮毂电机控制，支持：
 *   - 电流环控制（dji_m3508_send_current）
 *   - 速度环控制（dji_m3508_send_rpm）
 *   - 批量电流发送（dji_m3508_send_group_current）
 *   - 状态反馈解析与角度估计（通过速度积分）
 *   - 电机在线状态检测
 */

#include "dji_m3508.h"

#include <errno.h>
#include <math.h>
#include <string.h>

#include <zephyr/drivers/can.h>
#include <zephyr/logging/log.h>

#include "app_config.h"

LOG_MODULE_REGISTER(dji_m3508, LOG_LEVEL_INF);

/* ========================================================================== */
/*  VESC CAN 包类型定义                                                       */
/*  VESC 使用 29 位扩展帧，packet_id 占高字节，controller_id 占低字节           */
/*  CAN ID 格式: [packet_id(8bit) | controller_id(8bit)]                       */
/* ========================================================================== */

/** VESC 电流控制命令包 ID：发送后电调以指定电流（毫安）驱动电机 */
#define VESC_CAN_PACKET_SET_CURRENT 1U

/** VESC 转速控制命令包 ID：发送后电调以指定 ERPM 驱动电机 */
#define VESC_CAN_PACKET_SET_RPM 3U

/** VESC 状态反馈包 1 的 ID：包含 ERPM、相电流、占空比 */
#define VESC_CAN_PACKET_STATUS 9U

/** VESC 状态反馈包 4 的 ID：包含 MOS管温度、电机温度、输入电流 */
#define VESC_CAN_PACKET_STATUS_4 16U

/** VESC 状态反馈包 5 的 ID：包含里程计（霍尔脉冲计数）、输入电压 */
#define VESC_CAN_PACKET_STATUS_5 27U

/**
 * @brief 从字节数组中解析大端序有符号 16 位整数
 *
 * VESC 状态帧中的多字节字段采用大端序（Big-Endian）编码，
 * 高字节在前，低字节在后。
 *
 * @param data 指向至少 2 字节的数据指针
 * @return 解析后的有符号 16 位整数
 */
static int16_t be_i16(const uint8_t *data)
{
	return (int16_t)((uint16_t)data[0] << 8 | data[1]);
}

/**
 * @brief 从字节数组中解析大端序有符号 32 位整数
 *
 * 用于解析 VESC STATUS_1 中的 ERPM 字段和 STATUS_5 中的里程计字段。
 *
 * @param data 指向至少 4 字节的数据指针
 * @return 解析后的有符号 32 位整数
 */
static int32_t be_i32(const uint8_t *data)
{
	return (int32_t)((uint32_t)data[0] << 24 |
			 (uint32_t)data[1] << 16 |
			 (uint32_t)data[2] << 8 | data[3]);
}

/**
 * @brief 将有符号 32 位整数以大端序写入字节数组
 *
 * 用于构造 VESC 命令帧（SET_CURRENT / SET_RPM）的负载数据。
 * VESC 协议要求命令帧中的整数字段采用大端序编码。
 *
 * @param data 输出缓冲区（至少 4 字节）
 * @param value 要写入的有符号 32 位整数值
 */
static void put_be_i32(uint8_t *data, int32_t value)
{
	const uint32_t raw = (uint32_t)value;

	data[0] = (uint8_t)(raw >> 24);
	data[1] = (uint8_t)(raw >> 16);
	data[2] = (uint8_t)(raw >> 8);
	data[3] = (uint8_t)raw;
}

/**
 * @brief 构造 VESC 扩展帧 CAN ID
 *
 * VESC CAN 扩展帧的 29 位 ID 格式为：
 *   [packet_id(8bit) << 8] | controller_id(8bit)
 *   即高字节为包类型，低字节为电调的控制器 ID。
 *
 * @param controller_id 电调 ID（1~255）
 * @param packet_id 包类型 ID（如 SET_CURRENT=1, SET_RPM=3）
 * @return 组合后的 29 位扩展帧 ID
 */
static uint32_t vesc_ext_id(uint8_t controller_id, uint32_t packet_id)
{
	return (packet_id << 8) | controller_id;
}

/**
 * @brief 根据电机 ID 返回电流到轮子扭矩的转换系数
 *
 * 不同轮子由于减速比或标定差异，其电流-扭矩系数可能不同。
 * 系数单位：N·m / mA，用于从电机电流估算轮子输出扭矩。
 *
 * @param id 电机 ID（左轮或右轮的电调 ID）
 * @return 电流-扭矩转换系数 (N·m/mA)
 */
static float torque_coeff_for_id(uint8_t id)
{
	if (id == APP_WHEEL_LEFT_ID) {
		return APP_ASCENTO_LEFT_CURRENT_MA_TO_WHEEL_TORQUE_NM;
	}
	if (id == APP_WHEEL_RIGHT_ID) {
		return APP_ASCENTO_RIGHT_CURRENT_MA_TO_WHEEL_TORQUE_NM;
	}

	return APP_ASCENTO_CURRENT_MA_TO_WHEEL_TORQUE_NM;
}

/**
 * @brief 根据电机 ID 返回前进方向对应的电流符号
 *
 * 左轮和右轮的安装方向不同，相同的电流方向可能导致相反的旋转方向。
 * 该函数返回 +1 或 -1，用于在计算轮子扭矩时修正方向。
 * 确保正值电流对应机器人前进方向。
 *
 * @param id 电机 ID
 * @return +1 或 -1，表示前进方向对应的电流正负号
 */
static int wheel_forward_sign_for_id(uint8_t id)
{
	if (id == APP_WHEEL_LEFT_ID) {
		return APP_WHEEL_LEFT_FORWARD_CURRENT_SIGN;
	}
	if (id == APP_WHEEL_RIGHT_ID) {
		return APP_WHEEL_RIGHT_FORWARD_CURRENT_SIGN;
	}

	return 1;
}

/**
 * @brief 检查指定的命令槽位是否有效（已分配且非零 ID）
 *
 * 命令槽位用于 dji_m3508_send_group_current 批量发送电流命令，
 * 每个槽位对应一个电机的 ID。
 *
 * @param bus 总线实例指针
 * @param slot 槽位索引（0~3）
 * @return true 表示该槽位已分配且 ID 非零
 */
static bool command_slot_active(const dji_m3508_bus_t *bus, size_t slot)
{
	return slot < bus->command_id_count && bus->command_id[slot] != 0U;
}

/**
 * @brief 向指定 VESC 电调发送电流控制命令
 *
 * 使用 VESC SET_CURRENT 协议（packet_id=1）控制电机电流。
 * 电流值从 mA 转换为 VESC 协议要求的单位（经过 APP_VESC_CURRENT_CMD_TO_AMP 缩放）。
 *
 * CAN 帧格式：
 *   ID:   29位扩展帧 = [packet_id=1 | controller_id=id]
 *   DLC:  4
 *   Data: 大端序 32 位有符号整数，单位为毫安（经缩放后）
 *
 * @param bus 总线实例指针
 * @param id 电调 ID（1~255）
 * @param current_ma 目标电流值（毫安）
 * @return 0 成功，负值表示错误
 */
int dji_m3508_send_current(dji_m3508_bus_t *bus, uint8_t id,
			   int16_t current_ma)
{
	if (bus == NULL || bus->can == NULL || id == 0U) {
		return -EINVAL;
	}

	/* 将应用层的 mA 值转换为 VESC 协议的安培值，再乘以 1000 转为毫安 */
	const float amps = (float)current_ma * APP_VESC_CURRENT_CMD_TO_AMP;
	const int32_t milliamps = (int32_t)lrintf(amps * 1000.0f);

	/* 构造扩展帧：flags 标记 IDE（使用 29 位 ID），DLC=4 */
	struct can_frame frame = {
		.flags = CAN_FRAME_IDE,
		.id = vesc_ext_id(id, VESC_CAN_PACKET_SET_CURRENT),
		.dlc = 4,
	};

	/* 将电流值以大端序写入帧数据 */
	put_be_i32(&frame.data[0], milliamps);

	const int ret = can_send(bus->can, &frame, K_MSEC(2), NULL, NULL);
	if (ret == 0) {
		/* 发送成功，记录命令电流和时间戳（自旋锁保护共享状态） */
		k_spinlock_key_t key = k_spin_lock(&bus->lock);
		bus->motor[id].command_current_ma = current_ma;
		bus->motor[id].last_command_ms = k_uptime_get();
		k_spin_unlock(&bus->lock, key);
	}

	return ret;
}

/**
 * @brief 向指定 VESC 电调发送转速控制命令
 *
 * 使用 VESC SET_RPM 协议（packet_id=3）控制电机转速。
 * VESC 的 RPM 命令使用电气转速 ERPM = 机械转速 × 极对数。
 *
 * CAN 帧格式：
 *   ID:   29位扩展帧 = [packet_id=3 | controller_id=id]
 *   DLC:  8
 *   Data: 大端序 32 位有符号整数，单位 ERPM
 *
 * @param bus 总线实例指针
 * @param id 电调 ID（1~255）
 * @param rpm 目标转速（ERPM，电气转速）
 * @return 0 成功，负值表示错误
 */
int dji_m3508_send_rpm(dji_m3508_bus_t *bus, uint8_t id, int32_t rpm)
{
	if (bus == NULL || bus->can == NULL || id == 0U) {
		return -EINVAL;
	}

	/* 构造扩展帧：DLC=8，使用 SET_RPM 包类型 */
	struct can_frame frame = {
		.flags = CAN_FRAME_IDE,
		.id = vesc_ext_id(id, VESC_CAN_PACKET_SET_RPM),
		.dlc = 8,
	};

	/* ERPM 以大端序写入帧数据 */
	put_be_i32(&frame.data[0], rpm);
	return can_send(bus->can, &frame, K_MSEC(2), NULL, NULL);
}

/**
 * @brief 解析 VESC STATUS_1 状态包（packet_id=9）
 *
 * STATUS_1 是 VESC 的主状态反馈包，包含电机运行的核心数据。
 *
 * CAN 帧数据布局（8 字节，大端序）：
 *   data[0..3] : ERPM (int32) -- 电气转速，= 机械转速 × 极对数
 *   data[4..5] : 相电流 (int16) -- 单位：0.1A（deci-amp）
 *   data[6..7] : 占空比 (int16) -- 单位：0.1%（permille）
 *
 * 本函数还负责：
 *   - 将 ERPM 转换为机械 RPM 和 rad/s
 *   - 将 deci-amp 电流转换为 mA
 *   - 估算轮子扭矩 = 电流 × 方向符号 × 扭矩系数
 *   - 通过速度积分估算转角（因为 M3508 使用 VESC 电调，无独立编码器）
 *
 * @param bus 总线实例指针
 * @param id 电调 ID
 * @param frame 接收到的 CAN 帧
 */
static void parse_status_1(dji_m3508_bus_t *bus, uint8_t id,
			   const struct can_frame *frame)
{
	/* ---- 解析帧数据 ---- */
	const int32_t erpm = be_i32(&frame->data[0]);          /* 电气转速 ERPM */
	const int16_t current_deciamp = be_i16(&frame->data[4]); /* 相电流，0.1A 单位 */
	const int16_t duty_permille = be_i16(&frame->data[6]);   /* 占空比，0.1% 单位 */
	const int64_t now_ms = k_uptime_get();

	/* ERPM 转机械 RPM：机械 RPM = ERPM / 极对数 */
	const float pole_pairs = APP_VESC_MOTOR_POLE_PAIRS > 0.0f ?
				 APP_VESC_MOTOR_POLE_PAIRS : 1.0f;
	const float motor_rpm = (float)erpm / pole_pairs;

	/* RPM 转弧度/秒：1 RPM = 2π/60 ≈ 0.104719755 rad/s */
	const float speed_rad_s = motor_rpm * 0.104719755f;

	/* ---- 构建新的电机状态 ---- */
	dji_m3508_motor_t next = { 0 };
	next.id = id;
	next.present = true;
	next.online = true;
	next.erpm = erpm;
	next.speed_rpm = (int32_t)lrintf(motor_rpm);

	/* deci-amp 转 mA：乘以 100 */
	next.motor_current_ma = (int32_t)lrintf((float)current_deciamp * 100.0f);
	next.current_ma = next.motor_current_ma;

	/* 估算轮子输出扭矩：电流(mA) × 方向符号 × 扭矩系数(N·m/mA) */
	next.current_ma_to_wheel_torque_nm = torque_coeff_for_id(id);
	next.estimated_wheel_torque_nm =
		(float)(next.motor_current_ma * wheel_forward_sign_for_id(id)) *
		next.current_ma_to_wheel_torque_nm;

	/* permille 转小数：除以 1000 */
	next.duty = (float)duty_permille / 1000.0f;
	next.last_update_ms = now_ms;
	next.speed_rad_s = speed_rad_s;

	/* ---- 在自旋锁保护下合并其他状态包的数据（来自 STATUS_4/STATUS_5） ---- */
	k_spinlock_key_t key = k_spin_lock(&bus->lock);
	dji_m3508_motor_t *motor = &bus->motor[id];

	/* 保留来自其他状态包和命令函数的字段 */
	next.command_current_ma = motor->command_current_ma;
	next.last_command_ms = motor->last_command_ms;
	next.input_current_ma = motor->input_current_ma;
	next.input_voltage_mv = motor->input_voltage_mv;
	next.tachometer = motor->tachometer;
	next.fet_temperature_cdeg = motor->fet_temperature_cdeg;
	next.motor_temperature_cdeg = motor->motor_temperature_cdeg;
	next.last_status4_update_ms = motor->last_status4_update_ms;
	next.last_status5_update_ms = motor->last_status5_update_ms;

	/* ---- 转角估计：通过速度积分 ---- */
	/* M3508 使用 VESC 电调控制，没有独立的角度编码器反馈，
	 * 因此通过对角速度积分来估算转角。积分步长 dt 受限于
	 * 状态包发送周期（通常 50~100ms），精度有限。 */
	if (motor->initialized && motor->last_update_ms > 0) {
		const float dt_s = (float)(now_ms - motor->last_update_ms) *
				   1.0e-3f;
		/* dt 在合理范围内才积分，防止长时间断线后跳变 */
		if (dt_s > 0.0f && dt_s < 0.5f) {
			next.angle_rad = motor->angle_rad +
					 speed_rad_s * dt_s;
		} else {
			next.angle_rad = motor->angle_rad;
		}
		next.initialized = true;
	} else {
		/* 首次收到状态包，以当前时刻作为零点 */
		next.angle_rad = 0.0f;
		next.initialized = true;
	}

	/* 原子更新电机状态 */
	*motor = next;
	k_spin_unlock(&bus->lock, key);
}

/**
 * @brief 解析 VESC STATUS_4 状态包（packet_id=16）
 *
 * STATUS_4 提供温度和输入电流信息。
 *
 * CAN 帧数据布局（8 字节，大端序）：
 *   data[0..1] : MOS管温度 (int16) -- 单位：0.1°C
 *   data[2..3] : 电机绕组温度 (int16) -- 单位：0.1°C
 *   data[4..5] : 输入电流 (int16) -- 单位：0.1A（deci-amp）
 *   data[6..7] : 保留
 *
 * @param bus 总线实例指针
 * @param id 电调 ID
 * @param frame 接收到的 CAN 帧
 */
static void parse_status_4(dji_m3508_bus_t *bus, uint8_t id,
			   const struct can_frame *frame)
{
	const int16_t fet_temperature_cdeg = be_i16(&frame->data[0]);   /* MOS管温度，0.1°C */
	const int16_t motor_temperature_cdeg = be_i16(&frame->data[2]); /* 电机温度，0.1°C */
	const int16_t input_current_deciamp = be_i16(&frame->data[4]);  /* 输入电流，0.1A */
	const int64_t now_ms = k_uptime_get();

	/* 自旋锁保护下更新温度和输入电流字段 */
	k_spinlock_key_t key = k_spin_lock(&bus->lock);
	dji_m3508_motor_t *motor = &bus->motor[id];
	motor->id = id;
	motor->present = true;
	motor->fet_temperature_cdeg = fet_temperature_cdeg;
	motor->motor_temperature_cdeg = motor_temperature_cdeg;
	/* deci-amp 转 mA：乘以 100 */
	motor->input_current_ma =
		(int32_t)lrintf((float)input_current_deciamp * 100.0f);
	motor->last_status4_update_ms = now_ms;
	k_spin_unlock(&bus->lock, key);
}

/**
 * @brief 解析 VESC STATUS_5 状态包（packet_id=27）
 *
 * STATUS_5 提供里程计和电池电压信息。
 *
 * CAN 帧数据布局（8 字节，大端序）：
 *   data[0..3] : 里程计值 (int32) -- 霍尔传感器脉冲计数，需除以 6 得到电周期数
 *   data[4..5] : 输入电压 (int16) -- 单位：0.1V（deci-volt）
 *   data[6..7] : 保留
 *
 * @param bus 总线实例指针
 * @param id 电调 ID
 * @param frame 接收到的 CAN 帧
 */
static void parse_status_5(dji_m3508_bus_t *bus, uint8_t id,
			   const struct can_frame *frame)
{
	const int32_t tachometer_scaled = be_i32(&frame->data[0]);   /* 原始里程计脉冲数 */
	const int16_t input_voltage_decivolt = be_i16(&frame->data[4]); /* 输入电压，0.1V */
	const int64_t now_ms = k_uptime_get();

	/* 自旋锁保护下更新里程计和电压字段 */
	k_spinlock_key_t key = k_spin_lock(&bus->lock);
	dji_m3508_motor_t *motor = &bus->motor[id];
	motor->id = id;
	motor->present = true;
	/* VESC 里程计除以 6 得到电气旋转周期数 */
	motor->tachometer = tachometer_scaled / 6;
	/* deci-volt 转 milli-volt：乘以 100 */
	motor->input_voltage_mv =
		(int32_t)input_voltage_decivolt * 100;
	motor->last_status5_update_ms = now_ms;
	k_spin_unlock(&bus->lock, key);
}

/**
 * @brief VESC CAN 接收回调函数
 *
 * 当 Zephyr CAN 驱动收到匹配过滤器的帧时调用此函数。
 * 首先验证帧为 29 位扩展帧且 DLC=8（VESC 状态帧固定 8 字节），
 * 然后从 CAN ID 中提取 packet_id 和 controller_id，分发到对应的解析函数。
 *
 * VESC 扩展帧 ID 解码：
 *   packet_id     = (frame->id >> 8) & 0xFF  -- 包类型（STATUS_1/4/5）
 *   controller_id = frame->id & 0xFF          -- 电调 ID
 *
 * @param dev CAN 设备指针（未使用）
 * @param frame 接收到的 CAN 帧
 * @param user_data 用户数据，指向 dji_m3508_bus_t 实例
 */
static void dji_m3508_rx_cb(const struct device *dev, struct can_frame *frame,
			    void *user_data)
{
	ARG_UNUSED(dev);

	dji_m3508_bus_t *bus = user_data;

	/* VESC 状态帧必须是 29 位扩展帧且 DLC=8，否则忽略 */
	if ((frame->flags & CAN_FRAME_IDE) == 0 || frame->dlc < 8) {
		return;
	}

	/* 从 29 位扩展帧 ID 中解码包类型和电调 ID */
	const uint8_t packet_id = (uint8_t)((frame->id >> 8) & 0xffU);
	const uint8_t id = (uint8_t)(frame->id & 0xffU);

	/* 根据包类型分发到对应的解析函数 */
	switch (packet_id) {
	case VESC_CAN_PACKET_STATUS:
		parse_status_1(bus, id, frame);   /* ERPM / 电流 / 占空比 */
		break;
	case VESC_CAN_PACKET_STATUS_4:
		parse_status_4(bus, id, frame);   /* 温度 / 输入电流 */
		break;
	case VESC_CAN_PACKET_STATUS_5:
		parse_status_5(bus, id, frame);   /* 里程计 / 输入电压 */
		break;
	default:
		break;
	}
}

/**
 * @brief 为指定的 VESC 状态包类型添加 CAN 接收过滤器
 *
 * 过滤器匹配规则：
 *   - 使用 29 位扩展帧过滤（CAN_FILTER_IDE 标志）
 *   - 匹配 packet_id 部分（ID 的高字节），忽略 controller_id（低字节）
 *   - mask = CAN_EXT_ID_MASK & ~0xFF 表示只比较高字节，低 8 位为通配
 *
 * 这样一个过滤器可以接收所有电调（不同 controller_id）的同类型状态包。
 *
 * @param can CAN 设备指针
 * @param bus 总线实例指针（传递给回调函数）
 * @param packet_id 要过滤的 VESC 包类型 ID
 * @return 0 成功，负值表示错误
 */
static int add_status_filter(const struct device *can, dji_m3508_bus_t *bus,
			     uint32_t packet_id)
{
	const struct can_filter filter = {
		.flags = CAN_FILTER_IDE,       /* 匹配扩展帧 */
		.id = packet_id << 8,           /* 过滤条件：packet_id 在高字节 */
		.mask = CAN_EXT_ID_MASK & ~0xffU, /* 掩码：只匹配高字节，低字节通配 */
	};

	const int filter_id = can_add_rx_filter(can, dji_m3508_rx_cb, bus,
						&filter);
	if (filter_id < 0) {
		LOG_ERR("failed to add VESC status %u filter: %d",
			(unsigned int)packet_id, filter_id);
		return filter_id;
	}

	return 0;
}

/**
 * @brief 初始化 DJI M3508/VESC 电机总线
 *
 * 初始化流程：
 *   1. 清零总线结构体
 *   2. 注册电机 ID（标记为 present），分配命令槽位
 *   3. 为三种 VESC 状态包（STATUS_1/4/5）分别添加 CAN 接收过滤器
 *
 * 初始化完成后，CAN 驱动会自动接收所有电调的状态帧并触发回调。
 *
 * @param bus 总线实例指针（输出参数，会被初始化为零）
 * @param can CAN 设备指针
 * @param ids 要注册的电调 ID 数组
 * @param id_count ID 数组长度
 * @return 0 成功，负值表示错误
 */
int dji_m3508_init(dji_m3508_bus_t *bus, const struct device *can,
		   const uint8_t *ids, size_t id_count)
{
	if (bus == NULL || can == NULL || ids == NULL) {
		return -EINVAL;
	}

	/* 清零总线结构体，所有电机状态初始化为空 */
	memset(bus, 0, sizeof(*bus));
	bus->can = can;

	/* 注册每个电调 ID，标记为 present，并分配命令槽位 */
	for (size_t i = 0; i < id_count; i++) {
		const uint8_t id = ids[i];

		if (id == 0) {
			LOG_ERR("invalid VESC ID %u", id);
			return -EINVAL;
		}

		bus->motor[id].id = id;
		bus->motor[id].present = true;
		/* 最多支持 4 个命令槽位（用于批量电流发送） */
		if (bus->command_id_count < DJI_M3508_MAX_COMMAND_MOTORS) {
			bus->command_id[bus->command_id_count++] = id;
		}
	}

	/* 添加三种 VESC 状态包的 CAN 接收过滤器 */
	int ret = add_status_filter(can, bus, VESC_CAN_PACKET_STATUS);
	if (ret != 0) {
		return ret;
	}

	ret = add_status_filter(can, bus, VESC_CAN_PACKET_STATUS_4);
	if (ret != 0) {
		return ret;
	}

	ret = add_status_filter(can, bus, VESC_CAN_PACKET_STATUS_5);
	if (ret != 0) {
		return ret;
	}

	LOG_INF("VESC wheel status filters ready, %u motor(s)",
		(unsigned int)bus->command_id_count);
	return 0;
}

/**
 * @brief 批量向最多 4 个 VESC 电调发送电流命令
 *
 * 按命令槽位顺序发送电流值。每个槽位对应初始化时注册的一个电调 ID。
 * 常用于同时控制左轮和右轮电机，以及可能的关节电机。
 *
 * 注意：这是 4 个独立的 CAN 帧发送，不是 VESC 的多电机合并帧。
 * 如果某个发送失败，会记录第一个错误并继续发送剩余的。
 *
 * @param bus 总线实例指针
 * @param id1 第 0 号槽位的电流值（mA）
 * @param id2 第 1 号槽位的电流值（mA）
 * @param id3 第 2 号槽位的电流值（mA）
 * @param id4 第 3 号槽位的电流值（mA）
 * @return 0 全部成功，否则返回第一个错误码
 */
int dji_m3508_send_group_current(dji_m3508_bus_t *bus, int16_t id1,
				 int16_t id2, int16_t id3, int16_t id4)
{
	if (bus == NULL || bus->can == NULL) {
		return -EINVAL;
	}

	/* 将参数放入数组，按槽位索引对应 */
	const int16_t current[DJI_M3508_MAX_COMMAND_MOTORS] = {
		id1,
		id2,
		id3,
		id4,
	};
	int first_error = 0;

	/* 遍历所有命令槽位，对有效槽位发送电流命令 */
	for (size_t i = 0; i < DJI_M3508_MAX_COMMAND_MOTORS; i++) {
		if (command_slot_active(bus, i)) {
			const int ret = dji_m3508_send_current(
				bus, bus->command_id[i], current[i]);
			/* 记录第一个错误，但不中断后续发送 */
			if (ret != 0 && first_error == 0) {
				first_error = ret;
			}
		}
	}

	return first_error;
}

/**
 * @brief 获取指定电机的当前状态（线程安全）
 *
 * 在自旋锁保护下拷贝电机状态到输出参数。
 * 只有当电机标记为 present 且已收到至少一个 STATUS_1 包（initialized=true）时才返回 true。
 *
 * @param bus 总线实例指针
 * @param id 电调 ID
 * @param out 输出参数，用于存储电机状态的拷贝
 * @return true 表示电机在线且状态有效
 */
bool dji_m3508_get(const dji_m3508_bus_t *bus, uint8_t id,
		   dji_m3508_motor_t *out)
{
	if (bus == NULL || out == NULL || id == 0) {
		return false;
	}

	/* 自旋锁保护下读取电机状态 */
	k_spinlock_key_t key = k_spin_lock((struct k_spinlock *)&bus->lock);
	const bool ok = bus->motor[id].present && bus->motor[id].initialized;
	if (ok) {
		*out = bus->motor[id];
	}
	k_spin_unlock((struct k_spinlock *)&bus->lock, key);
	return ok;
}

/**
 * @brief 检查指定电机是否在线（状态更新未超时）
 *
 * 通过比较当前时间与最后一次状态更新时间来判断电机是否仍在通信。
 * 超时时间由调用者指定，通常设置为状态包发送周期的 2~3 倍。
 *
 * @param bus 总线实例指针
 * @param id 电调 ID
 * @param now_ms 当前时间戳（毫秒）
 * @param timeout_ms 超时阈值（毫秒）
 * @return true 表示电机在线（已初始化且未超时）
 */
bool dji_m3508_is_online(const dji_m3508_bus_t *bus, uint8_t id,
			 int64_t now_ms, int64_t timeout_ms)
{
	dji_m3508_motor_t motor;

	if (!dji_m3508_get(bus, id, &motor)) {
		return false;
	}

	return motor.online && (now_ms - motor.last_update_ms) <= timeout_ms;
}
