/**
 * @file dm4340.c
 * @brief DM4340 达妙关节电机驱动 -- MIT 控制协议
 *
 * 本模块通过 CAN 总线驱动 DM4340 达妙 (Damiao) 关节执行器，
 * 使用 MIT 控制协议进行位置/速度/力矩混合控制。
 *
 * DM4340 MIT 协议要点：
 *   - 使用标准 CAN 帧（11 位 ID），CAN ID = 电机 ID（1~15）
 *   - 发送帧（8 字节）打包 5 个控制量：
 *     位置 position(16bit) + 速度 velocity(12bit) + kp(12bit) + kd(12bit) + 力矩 torque(12bit)
 *   - 反馈帧（8 字节）包含：
 *     error(4bit) + 位置(16bit) + 速度(12bit) + 力矩(12bit) + MOS温度(8bit) + 转子温度(8bit)
 *   - 特殊命令帧：data[7] = 0xFC(使能) / 0xFD(失能) / 0xFE(保存零点)
 *
 * 额外支持的控制模式：
 *   - 位置+速度控制：CAN ID = 0x100 + 电机ID，数据为两个小端浮点数
 *   - 纯速度控制：CAN ID = 0x200 + 电机ID，数据为一个小端浮点数
 *
 * 已知问题与变通方案：
 *   本驱动包含一个直接轮询 CAN FIFO 的变通函数 dm4340_poll_rx_fifo()，
 *   因为在某些配置下 Zephyr 的 CAN RX 中断回调不会正常触发，
 *   需要通过寄存器轮询手动读取 FIFO 中的帧。
 *
 * 本驱动用于 Ascento 轮腿机器人的髋关节/膝关节电机控制。
 */

#include "dm4340.h"

#include <errno.h>
#include <math.h>
#include <string.h>

#include <zephyr/drivers/can.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>

LOG_MODULE_REGISTER(dm4340, LOG_LEVEL_INF);

/**
 * @brief 检查 CAN 总线是否处于可用状态
 *
 * 在发送命令前调用此函数，避免在总线关闭或总线关闭（BUS_OFF）状态下发送帧。
 * BUS_OFF 通常由过多错误帧触发，表示物理层存在问题。
 *
 * @param can CAN 设备指针
 * @return true 表示总线可用（非 BUS_OFF 且非 STOPPED）
 */
static bool can_ok(const struct device *can)
{
	enum can_state state;
	struct can_bus_err_cnt err_cnt;

	if (can_get_state(can, &state, &err_cnt) != 0) {
		return false;
	}

	return state != CAN_STATE_BUS_OFF && state != CAN_STATE_STOPPED;
}

/* 标准 CAN ID 掩码（11 位），若系统头文件未定义则手动定义 */
#ifndef CAN_STD_ID_MASK
#define CAN_STD_ID_MASK 0x7ff
#endif

/**
 * @brief 将浮点数线性映射为无符号整数（MIT 协议编码）
 *
 * MIT 协议使用定点数编码：将 [min_value, max_value] 范围内的浮点值
 * 线性映射到 [0, 2^bits - 1] 的无符号整数。
 * 例如：16 位位置编码范围为 0~65535，12 位速度/增益/力矩编码范围为 0~4095。
 *
 * @param value 待编码的浮点值
 * @param min_value 范围下限
 * @param max_value 范围上限
 * @param bits 编码位数（12 或 16）
 * @return 编码后的无符号整数
 */
static uint16_t float_to_uint(float value, float min_value, float max_value,
			      uint8_t bits)
{
	const float span = max_value - min_value;
	const uint32_t max_int = (1U << bits) - 1U;

	if (span <= 0.0f) {
		return 0;
	}

	/* 钳位到有效范围 */
	if (value < min_value) {
		value = min_value;
	} else if (value > max_value) {
		value = max_value;
	}

	/* 线性映射：offset/span * max_int，+0.5f 用于四舍五入 */
	const float offset = value - min_value;
	return (uint16_t)((offset * (float)max_int / span) + 0.5f);
}

/**
 * @brief 将无符号整数反解码为浮点数（MIT 协议解码）
 *
 * float_to_uint 的逆运算，用于解析反馈帧中的位置、速度、力矩字段。
 *
 * @param value 编码后的无符号整数
 * @param min_value 范围下限
 * @param max_value 范围上限
 * @param bits 编码位数（12 或 16）
 * @return 解码后的浮点值
 */
static float uint_to_float(uint16_t value, float min_value, float max_value,
			   uint8_t bits)
{
	const float span = max_value - min_value;
	const float max_int = (float)((1U << bits) - 1U);

	return ((float)value) * span / max_int + min_value;
}

/**
 * @brief 将浮点数以小端序写入字节数组
 *
 * 用于构造位置+速度控制帧和纯速度控制帧的数据负载。
 * DM4340 的位置/速度命令模式使用小端序浮点数（与 MIT 协议的定点数不同）。
 *
 * @param data 输出缓冲区（至少 4 字节）
 * @param value 要写入的浮点值
 */
static void put_le_float(uint8_t *data, float value)
{
	uint32_t raw;

	/* 将浮点数的位模式原样拷贝到整数，再以小端序写入 */
	memcpy(&raw, &value, sizeof(raw));
	sys_put_le32(raw, data);
}

/**
 * @brief 从反馈帧的 CAN ID 中提取电机 ID
 *
 * DM4340 MIT 协议的反馈帧使用标准 CAN 帧（11 位 ID），
 * CAN ID 的低 4 位即为电机 ID（1~15）。
 * 同时检查帧的 CAN ID 是否匹配已配置的反馈 ID（左/右关节）。
 *
 * @param bus 总线实例指针（包含已配置的反馈 CAN ID）
 * @param frame 接收到的 CAN 帧
 * @return 电机 ID（1~15），0 表示不匹配
 */
static uint8_t feedback_motor_id(const dm4340_bus_t *bus,
				 const struct can_frame *frame)
{
	/* MIT 协议反馈帧必须是标准帧（11 位 ID），扩展帧不处理 */
	if ((frame->flags & CAN_FRAME_IDE) != 0) {
		return 0;
	}

	/* 检查 CAN ID 是否匹配已配置的左/右关节反馈 ID */
	if (frame->id != bus->feedback_can_id &&
	    (!bus->has_right_feedback ||
	     frame->id != bus->right_feedback_can_id)) {
		return 0;
	}

	/* 低 4 位为电机 ID */
	const uint8_t id = frame->id & 0x0f;
	if (id == 0 || id > DM4340_MAX_ID) {
		return 0;
	}

	return id;
}

/**
 * @brief 解析 DM4340 参数读取响应帧
 *
 * DM4340 支持通过 CAN 读写内部参数（如 PID 增益、限位等）。
 * 参数响应帧格式：
 *   data[0..1] : CAN ID（小端序）
 *   data[2]    : 操作码 (op)，0x33=读响应，0x55=写响应，0xAA=错误响应
 *   data[3]    : 寄存器 ID (rid)
 *   data[4..7] : 参数值（小端序浮点数，仅 DLC>=8 时有效）
 *
 * @param bus 总线实例指针
 * @param frame 接收到的 CAN 帧
 * @return true 表示帧是参数响应并已处理，false 表示不是参数响应帧
 */
static bool parse_param_response(dm4340_bus_t *bus,
				 const struct can_frame *frame)
{
	/* 参数响应帧必须是标准帧且至少 4 字节 */
	if ((frame->flags & CAN_FRAME_IDE) != 0 || frame->dlc < 4) {
		return false;
	}

	/* 检查操作码：0x33=读响应, 0x55=写响应, 0xAA=错误响应 */
	const uint8_t op = frame->data[2];
	if (op != 0x33 && op != 0x55 && op != 0xaa) {
		return false;
	}

	/* 提取 CAN ID 和电机 ID */
	const uint16_t can_id = ((uint16_t)frame->data[1] << 8) |
				frame->data[0];
	const uint8_t id = can_id & 0xff;
	if (id == 0 || id > DM4340_MAX_ID) {
		return true; /* 已识别为参数响应，但 ID 无效，跳过存储 */
	}

	/* 如果 DLC>=8，解析 data[4..7] 中的浮点参数值 */
	uint32_t raw = 0;
	float value = 0.0f;
	if (frame->dlc >= 8) {
		raw = sys_get_le32(&frame->data[4]); /* 小端序读取 32 位原始值 */
		memcpy(&value, &raw, sizeof(value));  /* 将位模式解释为浮点数 */
	}

	/* 构建参数响应结构并存储 */
	const dm4340_param_response_t response = {
		.valid = true,
		.can_id = can_id,
		.op = op,
		.rid = frame->data[3],     /* 寄存器 ID */
		.raw_u32 = raw,            /* 原始 u32 值（位模式） */
		.value_float = value,      /* 浮点解释值 */
		.last_update_ms = k_uptime_get(),
	};

	/* 自旋锁保护下更新参数响应缓存 */
	k_spinlock_key_t key = k_spin_lock(&bus->lock);
	bus->param[id] = response;
	k_spin_unlock(&bus->lock, key);

	return true;
}

/**
 * @brief 向 DM4340 电机发送特殊控制命令
 *
 * DM4340 MIT 协议的特殊命令通过特定的帧格式识别：
 *   - CAN ID = 电机 ID（1~15）
 *   - DLC = 8
 *   - data[0..6] = 0xFF（全 1 用于标识特殊命令帧）
 *   - data[7] = 命令字节：
 *     0xFC = 使能电机（enable）-- 电机进入控制模式，可以接受指令
 *     0xFD = 失能电机（disable）-- 电机回到未激活状态
 *     0xFE = 保存当前位置为零点（save_zero）-- 将当前位置设为机械零位
 *
 * @param bus 总线实例指针
 * @param id 电机 ID（1~15）
 * @param cmd 命令字节（0xFC/0xFD/0xFE）
 * @return 0 成功，负值表示错误
 */
static int dm4340_send_special(dm4340_bus_t *bus, uint8_t id, uint8_t cmd)
{
	if (bus == NULL || bus->can == NULL || id == 0 ||
	    id > DM4340_MAX_ID) {
		return -EINVAL;
	}

	/* 发送前检查 CAN 总线状态 */
	if (!can_ok(bus->can)) {
		return -ENETDOWN;
	}

	/* 构造特殊命令帧：data[0..6]=0xFF 标识为特殊命令 */
	struct can_frame frame = {
		.flags = 0,
		.id = id,       /* CAN ID = 电机 ID */
		.dlc = 8,
		.data = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, cmd },
	};

	return can_send(bus->can, &frame, K_MSEC(2), NULL, NULL);
}

/** 全局计数器：记录 DM4340 RX 回调被调用的总次数（用于调试） */
volatile uint32_t dm4340_rx_cb_count;

/**
 * @brief DM4340 CAN 接收回调函数
 *
 * 当 Zephyr CAN 驱动收到匹配过滤器的帧时调用。
 * 处理流程：
 *   1. 递增全局回调计数器，切换 LED（调试用）
 *   2. 将帧记录到环形日志缓冲区（用于事后分析）
 *   3. 尝试解析为参数响应帧（op=0x33/0x55/0xAA）
 *   4. 否则尝试解析为 MIT 协议反馈帧
 *
 * MIT 反馈帧数据布局（8 字节，大端序 / 混合位域）：
 *   data[0]    : [error(4bit) | position_hi(4bit)] -- 高 4 位为错误码
 *   data[1]    : position_lo(8bit)                  -- 位置低 8 位，共 16 位
 *   data[2..3] : velocity(12bit) | kp_hi(4bit)      -- 速度 12 位
 *   data[4..5] : kp_lo(4bit) | kd(12bit)            -- kd 12 位
 *   data[6]    : MOS管温度（°C）
 *   data[7]    : 转子温度（°C）
 *
 * 本驱动中只提取位置、速度、力矩三个字段（与 MIT 发送帧的 5 参数对应）。
 * 实际反馈帧中 data 字段的位分配为：
 *   位置: data[1..2] 共 16 位
 *   速度: data[3] 高 4 位 + data[4] 低 4 位，共 12 位
 *   力矩: data[4] 高 4 位 + data[5] 低 8 位，共 12 位
 *
 * @param dev CAN 设备指针（未使用）
 * @param frame 接收到的 CAN 帧
 * @param user_data 用户数据，指向 dm4340_bus_t 实例
 */
static void dm4340_rx_cb(const struct device *dev, struct can_frame *frame,
			 void *user_data)
{
	ARG_UNUSED(dev);

	dm4340_bus_t *bus = user_data;

	/* 递增全局回调计数器（volatile，用于调试监视） */
	dm4340_rx_cb_count++;

	/* 每收到一帧切换一次 LED，用于物理层调试 */
	extern void dm4340_rx_led_toggle(void);
	dm4340_rx_led_toggle();

	/* ---- 环形日志记录 ---- */
	/* 记录每一帧到环形缓冲区，相同帧（ID+DLC+数据完全一致）则只递增计数 */
	uint32_t idx = bus->rx_log.head % DM4340_RX_LOG_SIZE;
	dm4340_rx_entry_t *entry = &bus->rx_log.entries[idx];

	if (entry->id == frame->id && entry->dlc == frame->dlc &&
	    memcmp(entry->data, frame->data, frame->dlc) == 0) {
		/* 与上一帧完全相同，只递增重复计数 */
		entry->count++;
	} else {
		/* 新帧，推进头指针并写入新条目 */
		bus->rx_log.head++;
		bus->rx_log.total++;
		idx = bus->rx_log.head % DM4340_RX_LOG_SIZE;
		bus->rx_log.entries[idx] = (dm4340_rx_entry_t){
			.id = frame->id,
			.dlc = frame->dlc,
			.flags = frame->flags,
			.count = 1,
		};
		memcpy(bus->rx_log.entries[idx].data, frame->data,
		       MIN(frame->dlc, 8));
	}

	/* ---- MIT 反馈帧解析 ---- */
	/* 反馈帧固定 8 字节，不足则忽略 */
	if (frame->dlc < 8) {
		return;
	}

	/* 先尝试解析为参数读取/写入响应帧 */
	if (parse_param_response(bus, frame)) {
		return; /* 是参数响应，已处理 */
	}

	/* 尝试提取电机 ID（检查是否为已配置的反馈 CAN ID） */
	const uint8_t id = feedback_motor_id(bus, frame);
	if (id == 0) {
		return; /* 不匹配任何已知的反馈 ID */
	}

	/* ---- 解析 MIT 反馈帧的位域 ---- */
	/* 位置 (16 bit): data[1] 为高 8 位，data[2] 为低 8 位 */
	const uint16_t p_int = ((uint16_t)frame->data[1] << 8) |
			       frame->data[2];
	/* 速度 (12 bit): data[3] 全 8 位为高 8 位，data[4] 高 4 位为低 4 位 */
	const uint16_t v_int = ((uint16_t)frame->data[3] << 4) |
			       (frame->data[4] >> 4);
	/* 力矩 (12 bit): data[4] 低 4 位为高 4 位，data[5] 全 8 位为低 8 位 */
	const uint16_t t_int = ((uint16_t)(frame->data[4] & 0x0f) << 8) |
			       frame->data[5];

	/* 将定点数解码为浮点值，使用配置的 MIT 参数范围 */
	dm4340_feedback_t fb = {
		.id = id,
		.error = frame->data[0] >> 4,  /* 错误码在 data[0] 高 4 位 */
		.online = true,
		.last_update_ms = k_uptime_get(),
		/* 位置：16 位定点数解码为弧度 */
		.position_rad = uint_to_float(p_int, bus->limit.p_min,
					      bus->limit.p_max, 16),
		/* 速度：12 位定点数解码为 rad/s */
		.velocity_rad_s = uint_to_float(v_int, bus->limit.v_min,
						bus->limit.v_max, 12),
		/* 力矩：12 位定点数解码为 N·m */
		.torque_nm = uint_to_float(t_int, bus->limit.t_min,
					   bus->limit.t_max, 12),
		.mos_temperature_c = frame->data[6],   /* MOS管温度（°C） */
		.rotor_temperature_c = frame->data[7], /* 转子温度（°C） */
	};

	/* 自旋锁保护下更新电机反馈状态 */
	k_spinlock_key_t key = k_spin_lock(&bus->lock);
	bus->motor[id] = fb;
	k_spin_unlock(&bus->lock, key);
}

/**
 * @brief 打印 DM4340 CAN 接收日志（调试用）
 *
 * 通过 printk 输出环形缓冲区中记录的所有 CAN 帧。
 * 每行输出格式：CAN ID、DLC、是否扩展帧、重复计数、8 字节数据。
 * 可用于排查 CAN 通信问题（如电机不响应、帧格式错误等）。
 *
 * @param bus 总线实例指针
 */
void dm4340_dump_rx_log(const dm4340_bus_t *bus)
{
	if (bus == NULL) {
		return;
	}

	printk("=== DM4340 CAN RX log (total=%u) ===\n", bus->rx_log.total);

	/* 计算环形缓冲区中有效条目的起始位置和数量 */
	uint32_t start = bus->rx_log.head > DM4340_RX_LOG_SIZE
				 ? bus->rx_log.head - DM4340_RX_LOG_SIZE + 1
				 : 0;
	uint32_t count = MIN(bus->rx_log.head + 1, DM4340_RX_LOG_SIZE);
	if (bus->rx_log.head >= DM4340_RX_LOG_SIZE) {
		start = bus->rx_log.head - DM4340_RX_LOG_SIZE + 1;
		count = DM4340_RX_LOG_SIZE;
	} else {
		start = 0;
		count = bus->rx_log.head + 1;
	}

	/* 遍历有效条目并打印 */
	for (uint32_t i = 0; i < count; i++) {
		uint32_t idx = (start + i) % DM4340_RX_LOG_SIZE;
		const dm4340_rx_entry_t *e = &bus->rx_log.entries[idx];
		if (e->count == 0) {
			continue;
		}
		printk("  id=0x%03x dlc=%u ext=%d cnt=%u data=%02x%02x%02x%02x%02x%02x%02x%02x\n",
		       e->id, e->dlc,
		       (e->flags & CAN_FRAME_IDE) ? 1 : 0,
		       e->count,
		       e->data[0], e->data[1], e->data[2], e->data[3],
		       e->data[4], e->data[5], e->data[6], e->data[7]);
	}
}

/**
 * @brief 初始化 DM4340 电机总线
 *
 * 初始化流程：
 *   1. 清零总线结构体
 *   2. 配置反馈 CAN ID（左/右关节可能使用不同的 ID）
 *   3. 设置 MIT 协议的参数范围限制（位置/速度/增益/力矩）
 *   4. 添加标准帧全接收过滤器（接收所有 11 位 ID 的帧）
 *   5. 添加扩展帧全接收过滤器（以防电机使用 29 位 ID）
 *
 * MIT 协议默认参数范围（DM4340 规格）：
 *   位置: ±12.5 rad
 *   速度: ±10 rad/s
 *   Kp:   0~500 N·m/rad
 *   Kd:   0~5 N·m·s/rad
 *   力矩: ±28 N·m
 *
 * @param bus 总线实例指针（输出参数）
 * @param can CAN 设备指针
 * @param feedback_can_id 左关节反馈帧的 CAN ID
 * @param right_feedback_can_id 右关节反馈帧的 CAN ID（与左关节相同时表示只有单关节）
 * @return 0 成功，负值表示错误
 */
int dm4340_init(dm4340_bus_t *bus, const struct device *can,
		uint16_t feedback_can_id, uint16_t right_feedback_can_id)
{
	if (bus == NULL || can == NULL) {
		return -EINVAL;
	}

	/* 清零所有电机状态和参数缓存 */
	memset(bus, 0, sizeof(*bus));
	bus->can = can;
	bus->feedback_can_id = feedback_can_id;
	bus->right_feedback_can_id = right_feedback_can_id;
	/* 如果左右反馈 ID 不同，则启用双关节模式 */
	bus->has_right_feedback = (right_feedback_can_id != feedback_can_id);

	/* 设置 MIT 协议的定点数编码/解码范围 */
	bus->limit = (dm4340_mit_limit_t) {
		.p_min = -12.5f,     /* 位置下限 (rad) */
		.p_max = 12.5f,      /* 位置上限 (rad) */
		.v_min = -10.0f,     /* 速度下限 (rad/s) */
		.v_max = 10.0f,      /* 速度上限 (rad/s) */
		.kp_min = 0.0f,      /* Kp 下限 (N·m/rad) */
		.kp_max = 500.0f,    /* Kp 上限 (N·m/rad) */
		.kd_min = 0.0f,      /* Kd 下限 (N·m·s/rad) */
		.kd_max = 5.0f,      /* Kd 上限 (N·m·s/rad) */
		.t_min = -28.0f,     /* 力矩下限 (N·m) */
		.t_max = 28.0f,      /* 力矩上限 (N·m) */
	};

	/* ---- 添加标准帧全接收过滤器 ---- */
	/* 接收所有标准帧（id=0, mask=0 表示匹配所有 ID），
	 * 用于接收 MIT 反馈帧和参数响应帧 */
	const struct can_filter filter_std = {
		.flags = 0,        /* 标准帧（11 位 ID） */
		.id = 0x000,
		.mask = 0x000,     /* 掩码为 0 = 全部匹配 */
	};

	int filter_id = can_add_rx_filter(can, dm4340_rx_cb, bus, &filter_std);
	if (filter_id < 0) {
		LOG_ERR("failed to add DM4340 std filter: %d", filter_id);
		return filter_id;
	}

	LOG_INF("DM4340 std filter ready (accept-all)");

	/* ---- 添加扩展帧全接收过滤器 ---- */
	/* 额外接收扩展帧，以防某些电机配置使用 29 位 ID。
	 * 失败不致命，左关节电机使用标准帧即可正常工作。 */
	const struct can_filter filter_ext = {
		.flags = CAN_FILTER_IDE,   /* 扩展帧（29 位 ID） */
		.id = 0x00000000,
		.mask = 0x00000000,
	};

	filter_id = can_add_rx_filter(can, dm4340_rx_cb, bus, &filter_ext);
	if (filter_id < 0) {
		LOG_ERR("failed to add DM4340 ext filter: %d", filter_id);
		/* Not fatal — left motor works on std frames */
	}

	LOG_INF("DM4340 ext filter ready (accept-all)");

	return 0;
}

/**
 * @brief 使能 DM4340 电机
 *
 * 发送特殊命令 0xFC，电机进入控制模式，可以接受 MIT 控制指令。
 * 电机上电后默认处于失能状态，必须先使能才能控制。
 *
 * @param bus 总线实例指针
 * @param id 电机 ID（1~15）
 * @return 0 成功，负值表示错误
 */
int dm4340_enable(dm4340_bus_t *bus, uint8_t id)
{
	return dm4340_send_special(bus, id, 0xfc);
}

/**
 * @brief 失能 DM4340 电机
 *
 * 发送特殊命令 0xFD，电机退出控制模式，回到未激活状态。
 * 电机失能后会释放力矩输出（自由转动或制动，取决于电机硬件配置）。
 *
 * @param bus 总线实例指针
 * @param id 电机 ID（1~15）
 * @return 0 成功，负值表示错误
 */
int dm4340_disable(dm4340_bus_t *bus, uint8_t id)
{
	return dm4340_send_special(bus, id, 0xfd);
}

/**
 * @brief 保存当前位置为零点
 *
 * 发送特殊命令 0xFE，将电机当前位置设为机械零位。
 * 后续的位置反馈和位置控制命令都以此零点为参考。
 * 注意：此操作会写入电机内部 Flash，有写入寿命限制。
 *
 * @param bus 总线实例指针
 * @param id 电机 ID（1~15）
 * @return 0 成功，负值表示错误
 */
int dm4340_save_zero(dm4340_bus_t *bus, uint8_t id)
{
	return dm4340_send_special(bus, id, 0xfe);
}

/**
 * @brief 请求读取 DM4340 电机的内部参数
 *
 * DM4340 支持通过 CAN 读写内部寄存器（如 PID 参数、限位值等）。
 * 参数读取请求帧格式：
 *   CAN ID = 0x7FF（广播地址）
 *   DLC = 4
 *   data[0..1] : 目标电机的 CAN ID（小端序）
 *   data[2]    : 操作码 0x33 表示读取请求
 *   data[3]    : 寄存器 ID (rid)
 *
 * 电机收到后会以相同格式回传响应（op=0x33），响应中包含参数值。
 * 可通过 dm4340_get_param_response() 获取解析后的响应。
 *
 * @param bus 总线实例指针
 * @param can_id 目标电机的 CAN ID
 * @param rid 寄存器 ID
 * @return 0 成功，负值表示错误
 */
int dm4340_request_param_read(dm4340_bus_t *bus, uint16_t can_id,
			      uint8_t rid)
{
	if (bus == NULL || bus->can == NULL || can_id == 0 ||
	    can_id > CAN_STD_ID_MASK) {
		return -EINVAL;
	}

	if (!can_ok(bus->can)) {
		return -ENETDOWN;
	}

	/* 构造参数读取请求帧：CAN ID=0x7FF，data 包含目标 CAN ID、操作码、寄存器 ID */
	struct can_frame frame = {
		.flags = 0,
		.id = 0x7ff,       /* 广播地址 */
		.dlc = 4,
		.data = { can_id & 0xff, can_id >> 8, 0x33, rid },
	};

	return can_send(bus->can, &frame, K_MSEC(5), NULL, NULL);
}

/**
 * @brief 向 DM4340 发送位置+速度限制控制命令
 *
 * 使用 DM4340 的位置控制模式（非 MIT 协议）：
 *   CAN ID = 0x100 + 电机 ID
 *   DLC = 8
 *   data[0..3] : 目标位置（小端序浮点数，单位 rad）
 *   data[4..7] : 速度限制（小端序浮点数，单位 rad/s）
 *
 * 与 MIT 模式不同，此模式直接指定位置和速度限制两个参数，
 * 电机内部控制器会自动规划轨迹。
 *
 * @param bus 总线实例指针
 * @param id 电机 ID（1~15）
 * @param position_rad 目标位置（弧度）
 * @param velocity_limit_rad_s 运动过程中的速度限制（rad/s）
 * @return 0 成功，负值表示错误
 */
int dm4340_send_pos_vel(dm4340_bus_t *bus, uint8_t id, float position_rad,
			float velocity_limit_rad_s)
{
	if (bus == NULL || bus->can == NULL || id == 0 || id > DM4340_MAX_ID) {
		return -EINVAL;
	}

	if (!can_ok(bus->can)) {
		return -ENETDOWN;
	}

	/* 构造位置+速度控制帧：CAN ID = 0x100 + id */
	struct can_frame frame = {
		.flags = 0,
		.id = 0x100U + id,
		.dlc = 8,
	};

	/* 两个小端序浮点数：位置 + 速度限制 */
	put_le_float(&frame.data[0], position_rad);
	put_le_float(&frame.data[4], velocity_limit_rad_s);

	return can_send(bus->can, &frame, K_MSEC(2), NULL, NULL);
}

/**
 * @brief 向 DM4340 发送纯速度控制命令
 *
 * 使用 DM4340 的速度控制模式（非 MIT 协议）：
 *   CAN ID = 0x200 + 电机 ID
 *   DLC = 4
 *   data[0..3] : 目标速度（小端序浮点数，单位 rad/s）
 *
 * 电机以指定速度持续运行，直到收到新的命令或失能。
 *
 * @param bus 总线实例指针
 * @param id 电机 ID（1~15）
 * @param velocity_rad_s 目标速度（rad/s）
 * @return 0 成功，负值表示错误
 */
int dm4340_send_velocity(dm4340_bus_t *bus, uint8_t id,
			 float velocity_rad_s)
{
	if (bus == NULL || bus->can == NULL || id == 0 || id > DM4340_MAX_ID) {
		return -EINVAL;
	}

	if (!can_ok(bus->can)) {
		return -ENETDOWN;
	}

	/* 构造纯速度控制帧：CAN ID = 0x200 + id */
	struct can_frame frame = {
		.flags = 0,
		.id = 0x200U + id,
		.dlc = 4,
	};

	/* 一个小端序浮点数：目标速度 */
	put_le_float(&frame.data[0], velocity_rad_s);
	return can_send(bus->can, &frame, K_MSEC(2), NULL, NULL);
}

/**
 * @brief 向 DM4340 发送 MIT 协议控制帧（位置+速度+Kp+Kd+力矩）
 *
 * MIT 控制协议将 5 个控制参数打包到 8 字节的标准 CAN 帧中：
 *
 * CAN 帧位域布局（总计 64 bit = 8 字节）：
 *   位置 position   : 16 bit  (data[0..1])
 *   速度 velocity   : 12 bit  (data[2] + data[3] 高 4 位)
 *   Kp              : 12 bit  (data[3] 低 4 位 + data[4])
 *   Kd              : 12 bit  (data[5] + data[6] 高 4 位)
 *   力矩 torque     : 12 bit  (data[6] 低 4 位 + data[7])
 *
 * 控制律：torque_out = Kp * (pos_desired - pos_actual) + Kd * (vel_desired - vel_actual) + torque_ff
 *
 * CAN ID = 电机 ID（1~15）
 *
 * @param bus 总线实例指针
 * @param id 电机 ID（1~15）
 * @param position_rad 目标位置（弧度）
 * @param velocity_rad_s 目标速度（rad/s）
 * @param kp 位置增益（N·m/rad）
 * @param kd 速度增益（N·m·s/rad）
 * @param torque_nm 前馈力矩（N·m）
 * @return 0 成功，负值表示错误
 */
int dm4340_send_mit(dm4340_bus_t *bus, uint8_t id, float position_rad,
		    float velocity_rad_s, float kp, float kd, float torque_nm)
{
	if (bus == NULL || bus->can == NULL || id == 0 || id > DM4340_MAX_ID) {
		return -EINVAL;
	}

	if (!can_ok(bus->can)) {
		return -ENETDOWN;
	}

	/* ---- 将浮点值编码为定点无符号整数 ---- */
	const dm4340_mit_limit_t *lim = &bus->limit;
	const uint16_t p = float_to_uint(position_rad, lim->p_min, lim->p_max,
					 16);      /* 位置：16 bit */
	const uint16_t v = float_to_uint(velocity_rad_s, lim->v_min,
					 lim->v_max, 12);  /* 速度：12 bit */
	const uint16_t kp_u = float_to_uint(kp, lim->kp_min, lim->kp_max, 12); /* Kp：12 bit */
	const uint16_t kd_u = float_to_uint(kd, lim->kd_min, lim->kd_max, 12); /* Kd：12 bit */
	const uint16_t t = float_to_uint(torque_nm, lim->t_min, lim->t_max,
					 12);      /* 力矩：12 bit */

	/* 构造 MIT 控制帧：CAN ID = 电机 ID */
	struct can_frame frame = {
		.flags = 0,
		.id = id,
		.dlc = 8,
	};

	/* ---- 按 MIT 协议位域格式打包到 8 字节 ---- */
	/* data[0..1] : 位置 (16 bit)，高位在前 */
	frame.data[0] = p >> 8;
	frame.data[1] = p & 0xff;
	/* data[2] : 速度高 8 位 */
	frame.data[2] = v >> 4;
	/* data[3] : 速度低 4 位 | Kp 高 4 位 */
	frame.data[3] = ((v & 0x0f) << 4) | (kp_u >> 8);
	/* data[4] : Kp 低 8 位 */
	frame.data[4] = kp_u & 0xff;
	/* data[5] : Kd 高 8 位 */
	frame.data[5] = kd_u >> 4;
	/* data[6] : Kd 低 4 位 | 力矩高 4 位 */
	frame.data[6] = ((kd_u & 0x0f) << 4) | (t >> 8);
	/* data[7] : 力矩低 8 位 */
	frame.data[7] = t & 0xff;

	return can_send(bus->can, &frame, K_MSEC(2), NULL, NULL);
}

/**
 * @section CAN FIFO 直接轮询变通方案
 *
 * 背景：
 *   在某些 STM32F407 + Zephyr 的配置下，CAN RX 中断回调（ISR）不会正常触发，
 *   导致 dm4340_rx_cb 永远不会被调用，电机反馈帧无法被解析。
 *
 * 解决方案：
 *   直接通过内存映射寄存器轮询 CAN1 外设的 FIFO 0，
 *   手动读取帧数据并调用标准的 rx 回调函数进行处理。
 *
 * STM32F407 bxCAN 外设寄存器布局（CAN1 基地址 0x40006400）：
 *   +0x0C : RF0R  (接收 FIFO 0 寄存器) -- FMP0 位表示 FIFO 中待读帧数
 *   +0x1C : RI0R  (接收 FIFO 0 邮箱标识符寄存器) -- 帧 ID 和标志
 *   +0x20 : RDT0R (接收 FIFO 0 邮箱数据长度和时间戳寄存器) -- DLC
 *   +0x24 : RDL0R (接收 FIFO 0 邮箱低字节数据寄存器) -- data[0..3]
 *   +0x28 : RDH0R (接收 FIFO 0 邮箱高字节数据寄存器) -- data[4..7]
 *
 * RI0R 寄存器位域：
 *   bit 2  : IDE 标志（1=扩展帧 29 位，0=标准帧 11 位）
 *   bit 1  : RTR 标志（1=远程帧，0=数据帧）
 *   [31:3] : 扩展帧 ID（29 位）
 *   [31:21]: 标准帧 ID（11 位）
 *
 * RF0R 寄存器位域：
 *   [1:0]  : FMP0 -- FIFO 0 消息挂起计数（0~3）
 *   bit 5  : RFOM0 -- 释放 FIFO 0 输出邮箱（写 1 释放）
 */

/* CAN1 外设基地址（STM32F407） */
#ifndef CAN1_BASE
#define CAN1_BASE 0x40006400UL
#endif

/* RF0R 寄存器中 FMP0 字段的掩码（bit[1:0]，表示 FIFO 中待读帧数） */
#ifndef CAN_RF0R_FMP0_Msk
#define CAN_RF0R_FMP0_Msk 0x03
#endif

#include <zephyr/sys/util_macro.h>
#include <zephyr/spinlock.h>

/**
 * @brief 直接轮询 CAN1 FIFO 0，手动读取并处理挂起的帧
 *
 * 此函数是 Zephyr CAN RX 中断不触发时的变通方案。
 * 它直接访问 STM32 bxCAN 外设的 FIFO 0 寄存器，
 * 读取帧数据后调用 dm4340_rx_cb 进行解析处理。
 *
 * 使用方式：在主循环或定时任务中周期性调用此函数。
 *
 * @param bus 总线实例指针
 * @return 本次调用处理的帧数（0 表示 FIFO 为空）
 */
int dm4340_poll_rx_fifo(dm4340_bus_t *bus)
{
	if (bus == NULL || bus->can == NULL) {
		return 0;
	}

	/* 映射 CAN1 外设寄存器地址 */
	volatile uint32_t *rf0r = (volatile uint32_t *)(CAN1_BASE + 0x0C);   /* FIFO 0 状态寄存器 */
	volatile uint32_t *mbox_id = (volatile uint32_t *)(CAN1_BASE + 0x1C); /* 邮箱 ID 寄存器 */
	volatile uint32_t *mbox_dt = (volatile uint32_t *)(CAN1_BASE + 0x20); /* 邮箱 DLC 寄存器 */
	volatile uint32_t *mbox_dl = (volatile uint32_t *)(CAN1_BASE + 0x24); /* 邮箱数据低 4 字节 */
	volatile uint32_t *mbox_dh = (volatile uint32_t *)(CAN1_BASE + 0x28); /* 邮箱数据高 4 字节 */
	int processed = 0;

	/* 循环读取直到 FIFO 中没有挂起的帧 */
	/* FMP0 字段表示 FIFO 中待读取的帧数，非零时继续读取 */
	while (*rf0r & CAN_RF0R_FMP0_Msk) {
		struct can_frame frame = { .flags = 0 };
		uint32_t rir = *mbox_id;   /* 读取 ID 和标志寄存器 */
		uint32_t rdtr = *mbox_dt;  /* 读取 DLC 寄存器 */

		/* 解析 IDE 标志（bit 2）判断帧类型 */
		if (rir & (1U << 2)) {
			/* 扩展帧（29 位 ID）：ID 在 bit[31:3] */
			frame.flags |= CAN_FRAME_IDE;
			frame.id = (rir >> 3) & CAN_EXT_ID_MASK;
		} else {
			/* 标准帧（11 位 ID）：ID 在 bit[31:21] */
			frame.id = (rir >> 21) & CAN_STD_ID_MASK;
		}

		/* 解析 RTR 标志（bit 1）：远程传输请求帧 */
		if (rir & (1U << 1)) {
			frame.flags |= CAN_FRAME_RTR;
		}

		/* DLC 在 RDT0R 寄存器的低 4 位 */
		frame.dlc = rdtr & 0x0F;

		/* 读取 8 字节数据（STM32 bxCAN 以两个 32 位字存储） */
		uint32_t dl = *mbox_dl;  /* data[0..3]，小端序存储 */
		uint32_t dh = *mbox_dh;  /* data[4..7]，小端序存储 */

		frame.data[0] = (dl >> 0) & 0xFF;
		frame.data[1] = (dl >> 8) & 0xFF;
		frame.data[2] = (dl >> 16) & 0xFF;
		frame.data[3] = (dl >> 24) & 0xFF;
		frame.data[4] = (dh >> 0) & 0xFF;
		frame.data[5] = (dh >> 8) & 0xFF;
		frame.data[6] = (dh >> 16) & 0xFF;
		frame.data[7] = (dh >> 24) & 0xFF;

		/* 直接调用标准 rx 回调，复用全部解析逻辑 */
		dm4340_rx_cb(bus->can, &frame, bus);

		/* 释放 FIFO 输出邮箱：向 RF0R 的 bit 5 写 1 */
		*rf0r = *rf0r | (1U << 5);
		processed++;
	}

	return processed;
}

/**
 * @brief 获取指定 DM4340 电机的最新反馈状态（线程安全）
 *
 * 在自旋锁保护下拷贝电机反馈数据到输出参数。
 * 只有当电机至少收到过一次有效的 MIT 反馈帧（online=true）时才返回 true。
 *
 * @param bus 总线实例指针
 * @param id 电机 ID（1~15）
 * @param out 输出参数，用于存储反馈数据的拷贝
 * @return true 表示电机在线且数据有效
 */
bool dm4340_get(const dm4340_bus_t *bus, uint8_t id, dm4340_feedback_t *out)
{
	if (bus == NULL || out == NULL || id == 0 || id > DM4340_MAX_ID) {
		return false;
	}

	/* 自旋锁保护下读取电机反馈状态 */
	k_spinlock_key_t key = k_spin_lock((struct k_spinlock *)&bus->lock);
	const bool ok = bus->motor[id].online;
	if (ok) {
		*out = bus->motor[id];
	}
	k_spin_unlock((struct k_spinlock *)&bus->lock, key);
	return ok;
}

/**
 * @brief 获取 DM4340 参数读取响应（线程安全）
 *
 * 获取指定电机指定寄存器的最新参数读取响应。
 * 使用 dm4340_request_param_read() 发送读取请求后，
 * 通过此函数获取解析后的响应数据。
 *
 * @param bus 总线实例指针
 * @param id 电机 ID（1~15）
 * @param rid 期望的寄存器 ID（用于校验响应是否匹配）
 * @param out 输出参数，用于存储参数响应的拷贝
 * @return true 表示有匹配的有效响应
 */
bool dm4340_get_param_response(const dm4340_bus_t *bus, uint8_t id,
			       uint8_t rid, dm4340_param_response_t *out)
{
	if (bus == NULL || out == NULL || id == 0 || id > DM4340_MAX_ID) {
		return false;
	}

	/* 自旋锁保护下读取参数响应缓存 */
	k_spinlock_key_t key = k_spin_lock((struct k_spinlock *)&bus->lock);
	const dm4340_param_response_t response = bus->param[id];
	/* 校验响应有效且寄存器 ID 匹配 */
	const bool ok = response.valid && response.rid == rid;
	if (ok) {
		*out = response;
	}
	k_spin_unlock((struct k_spinlock *)&bus->lock, key);
	return ok;
}
