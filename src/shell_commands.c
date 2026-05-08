/**
 * @file shell_commands.c
 * @brief Zephyr Shell 命令接口 — 轮腿机器人控制与电机调试
 *
 * 本文件为 Ascento 风格轮腿机器人（STM32F407）注册两组顶层 Shell 命令：
 *
 *   robot  — 机器人级控制命令
 *     enable   启用/禁用平衡控制器
 *     stop     立即停止运动
 *     status   显示机器人完整状态（姿态、轮速、电流、关节角等）
 *     param    运行时参数调优（支持 NVS Flash 持久化）
 *
 *   motor  — 单电机调试命令（用于装机调试和故障排查）
 *     wheel    VESC/M3508 轮毂电机调试
 *       status   显示轮毂电机反馈状态（转速/电流/温度/里程计等）
 *       current  设置轮毂电机电流（mA）
 *       rpm      设置轮毂电机目标转速（ERPM）
 *       pair     同时设置左右轮电流
 *       stop     停止轮毂电机电流输出
 *     dm       DM4340 关节电机调试
 *       status   显示关节电机反馈状态（位置/速度/扭矩/温度）
 *       enable   使能关节电机
 *       disable  失能关节电机
 *       zero     将当前位置保存为关节电机零点
 *       reg      读取 DM4340 寄存器
 *       diag     一键读取所有关键诊断寄存器
 *       pos      设置关节电机目标位置（位置-速度模式）
 *       vel      设置关节电机目标速度
 *       mit      MIT 模式直接控制（位置+速度+Kp+Kd+前馈扭矩）
 *       nudge    微调关节位置（从当前位置偏移 delta 弧度）
 *       wiggle   关节电机正弦摆动测试（用于校准和振动检测）
 *       stop     停止指定关节电机的调试输出
 *       rxlog    打印 DM4340 接收日志（用于 CAN 通信诊断）
 *     debug    手动电机调试状态管理
 *       status   显示当前所有手动调试指令状态
 *       stop     停止所有手动调试指令
 *     can      CAN 总线调试
 *       status   显示 CAN 总线状态（错误计数器等）
 *       raw      发送标准帧（11位 ID）原始 CAN 报文
 *       rawx     发送扩展帧（29位 ID）原始 CAN 报文
 *       recover  重置 CAN 控制器（从 bus-off 状态恢复）
 *
 * 注意：所有 motor 子命令在执行前会自动调用 enter_motor_debug_mode()，
 *       该函数会禁用平衡控制器并停止运动，以防止调试指令与自动控制冲突。
 *
 * 命令缩写说明：
 *   left/l  — 左侧电机（wheel: M3508, dm: DM4340）
 *   right/r — 右侧电机
 *   joint/dm/can1 — 关节 CAN 总线（CAN1，连接 DM4340）
 *   wheel/m3508/can2 — 轮毂 CAN 总线（CAN2，连接 VESC/M3508）
 */

#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/util.h>

/* ========== 项目头文件 ========== */
#include "app_config.h"          /* 应用配置宏（电机ID、CAN别名、调试超时等） */
#include "ascento_balance.h"     /* Ascento 平衡控制器接口（参数读写、状态查询） */
#include "control.h"             /* 机器人级控制接口（运动指令、使能/禁用） */
#include "motor_debug.h"         /* 电机调试接口（电流/转速/位置指令、反馈查询） */

/* ========== CAN 总线设备节点 ========== */
/* 从设备树别名获取两条 CAN 总线：
 *   can_wheel (CAN2) — 连接左右 VESC/M3508 轮毂电机
 *   can_joint  (CAN1) — 连接左右 DM4340 关节电机
 */
#define SHELL_WHEEL_CAN_NODE DT_ALIAS(can_wheel)
#define SHELL_JOINT_CAN_NODE DT_ALIAS(can_joint)

static const struct device *const shell_wheel_can =
	DEVICE_DT_GET(SHELL_WHEEL_CAN_NODE);  /* 轮毂电机 CAN 设备句柄 */
static const struct device *const shell_joint_can =
	DEVICE_DT_GET(SHELL_JOINT_CAN_NODE);  /* 关节电机 CAN 设备句柄 */

/**
 * @brief 解析布尔值字符串
 *
 * 接受 "1"/"on"/"true"/"enable" 为真，其余为假。
 * 用于 robot enable 等命令的参数解析。
 *
 * @param text 输入字符串
 * @return true 表示启用，false 表示禁用
 */
static bool parse_bool(const char *text)
{
	return strcmp(text, "1") == 0 || strcmp(text, "on") == 0 ||
	       strcmp(text, "true") == 0 || strcmp(text, "enable") == 0;
}

/**
 * @brief 解析 32 位有符号整数
 *
 * 支持十进制、十六进制（0x前缀）、八进制（0前缀）。
 * 用于解析电机ID、电流值、CAN帧ID等整数参数。
 *
 * @param text 输入字符串
 * @param out  输出整数值
 * @return true 解析成功，false 格式错误或溢出
 */
static bool parse_i32(const char *text, int32_t *out)
{
	char *end = NULL;

	errno = 0;
	const long value = strtol(text, &end, 0);
	if (errno != 0 || end == text || *end != '\0') {
		return false;
	}

	*out = (int32_t)value;
	return true;
}

/**
 * @brief 解析浮点数参数
 *
 * 用于解析 PID 增益、角度、速度等浮点命令参数。
 *
 * @param text 输入字符串
 * @param out  输出浮点值
 * @return true 解析成功，false 格式错误
 */
static bool parse_float_arg(const char *text, float *out)
{
	char *end = NULL;

	errno = 0;
	const float value = strtof(text, &end);
	if (errno != 0 || end == text || *end != '\0') {
		return false;
	}

	*out = value;
	return true;
}

/**
 * @brief 将 CAN 总线状态枚举转换为可读字符串
 *
 * 用于 motor can status 命令的输出显示。
 * 状态含义：
 *   error-active  — 正常工作状态，可正常收发
 *   error-warning — 发送/接收错误计数超过 96，发出警告
 *   error-passive — 错误计数超过 128，进入被动错误状态
 *   bus-off       — 发送错误计数超过 255，总线脱离
 *   stopped       — CAN 控制器已停止
 *
 * @param state CAN 状态枚举值
 * @return 对应的状态名称字符串
 */
static const char *can_state_name(enum can_state state)
{
	switch (state) {
	case CAN_STATE_ERROR_ACTIVE:
		return "error-active";
	case CAN_STATE_ERROR_WARNING:
		return "error-warning";
	case CAN_STATE_ERROR_PASSIVE:
		return "error-passive";
	case CAN_STATE_BUS_OFF:
		return "bus-off";
	case CAN_STATE_STOPPED:
		return "stopped";
	default:
		return "unknown";
	}
}

/**
 * @brief 解析 CAN 总线名称参数
 *
 * 将命令行中的总线名称映射为对应的 CAN 设备句柄和人类可读名称。
 * 支持的别名：
 *   "joint"/"dm"/"can1"  → 关节 CAN 总线（CAN1，DM4340 关节电机）
 *   "wheel"/"m3508"/"can2" → 轮毂 CAN 总线（CAN2，VESC/M3508 轮毂电机）
 *
 * @param text 输入的总线名称字符串
 * @param dev  输出：CAN 设备句柄
 * @param name 输出：人类可读的总线名称
 * @return true 解析成功，false 未知总线名称
 */
static bool parse_can_bus(const char *text, const struct device **dev,
			  const char **name)
{
	if (strcmp(text, "joint") == 0 || strcmp(text, "dm") == 0 ||
	    strcmp(text, "can1") == 0) {
		*dev = shell_joint_can;
		*name = "joint/CAN1";
		return true;
	}

	if (strcmp(text, "wheel") == 0 || strcmp(text, "m3508") == 0 ||
	    strcmp(text, "can2") == 0) {
		*dev = shell_wheel_can;
		*name = "wheel/CAN2";
		return true;
	}

	return false;
}

/**
 * @brief 解析轮毂电机（VESC/M3508）ID 参数
 *
 * 支持 "left"/"l" 和 "right"/"r" 别名，或直接输入数字 ID（1..255）。
 * left/right 映射到 app_config.h 中定义的 APP_WHEEL_LEFT_ID / APP_WHEEL_RIGHT_ID。
 *
 * @param text 输入的电机标识字符串
 * @param id   输出：电机 CAN ID
 * @return true 解析成功，false 无效 ID
 */
static bool parse_wheel_id(const char *text, uint8_t *id)
{
	if (strcmp(text, "left") == 0 || strcmp(text, "l") == 0) {
		*id = APP_WHEEL_LEFT_ID;
		return true;
	}
	if (strcmp(text, "right") == 0 || strcmp(text, "r") == 0) {
		*id = APP_WHEEL_RIGHT_ID;
		return true;
	}

	int32_t value;
	if (!parse_i32(text, &value) || value <= 0 ||
	    value > DJI_M3508_MAX_ID) {
		return false;
	}

	*id = (uint8_t)value;
	return true;
}

/**
 * @brief 解析关节电机（DM4340）ID 参数
 *
 * 支持 "left"/"l" 和 "right"/"r" 别名，或直接输入数字 ID（1..15）。
 * left/right 映射到 app_config.h 中定义的 APP_DM_LEFT_ID / APP_DM_RIGHT_ID。
 *
 * @param text 输入的电机标识字符串
 * @param id   输出：电机 CAN ID
 * @return true 解析成功，false 无效 ID
 */
static bool parse_dm_id(const char *text, uint8_t *id)
{
	if (strcmp(text, "left") == 0 || strcmp(text, "l") == 0) {
		*id = APP_DM_LEFT_ID;
		return true;
	}
	if (strcmp(text, "right") == 0 || strcmp(text, "r") == 0) {
		*id = APP_DM_RIGHT_ID;
		return true;
	}

	int32_t value;
	if (!parse_i32(text, &value) || value <= 0 || value > DM4340_MAX_ID) {
		return false;
	}

	*id = (uint8_t)value;
	return true;
}

/**
 * @brief 解析可选的持续时间参数（毫秒）
 *
 * 如果用户未提供该参数，使用 APP_MOTOR_DEBUG_DEFAULT_TIMEOUT_MS 默认值。
 * 持续时间到期后，电机调试模块会自动停止对应电机的输出，防止失控。
 *
 * @param argc        命令参数总数
 * @param argv        命令参数数组
 * @param index       持续时间参数在 argv 中的位置
 * @param duration_ms 输出：持续时间（毫秒）
 * @return true 解析成功（包括未提供参数的情况），false 参数格式错误
 */
static bool parse_optional_duration(size_t argc, char **argv, size_t index,
				    int32_t *duration_ms)
{
	*duration_ms = APP_MOTOR_DEBUG_DEFAULT_TIMEOUT_MS;
	if (argc <= index) {
		return true;
	}

	return parse_i32(argv[index], duration_ms);
}

/**
 * @brief 进入电机调试模式
 *
 * 在执行任何 motor 子命令前调用此函数，确保：
 *   1. 禁用平衡控制器（control_set_enable(false)），防止自动控制干扰手动调试
 *   2. 停止运动指令（control_stop_motion()），防止残留运动指令
 *
 * 这是所有电机调试命令的安全前提，避免自动控制与手动指令冲突导致失控。
 */
static void enter_motor_debug_mode(void)
{
	control_set_enable(false);
	control_stop_motion();
}

/**
 * @brief 将 DM4340 寄存器 ID 转换为人类可读名称
 *
 * DM4340 关节电机的关键寄存器：
 *   0x01 KT_Value   — 扭矩常数
 *   0x02 OT_Value   — 过温保护阈值
 *   0x03 OC_Value   — 过流保护阈值
 *   0x04 ACC        — 加速度限制
 *   0x05 DEC        — 减速度限制
 *   0x06 MAX_SPD    — 最大速度限制
 *   0x07 MST_ID     — 主站 CAN ID
 *   0x08 ESC_ID     — 电调自身 CAN ID
 *   0x09 TIMEOUT    — CAN 通信超时时间
 *   0x0a CTRL_MODE  — 控制模式（1=MIT, 2=pos_vel, 3=velocity, 4=pvt）
 *   0x15 PMAX       — 位置范围上限（弧度）
 *   0x16 VMAX       — 速度范围上限（弧度/秒）
 *   0x17 TMAX       — 扭矩范围上限（牛米）
 *   0x23 can_br     — CAN 波特率
 *   0x3b Imax       — 最大电流
 *   0x3c VBus       — 母线电压
 *
 * @param rid 寄存器 ID
 * @return 寄存器名称字符串（未知寄存器返回 "reg"）
 */
static const char *dm_reg_name(uint8_t rid)
{
	switch (rid) {
	case 0x01:
		return "KT_Value";   /* 扭矩常数 */
	case 0x02:
		return "OT_Value";   /* 过温阈值 */
	case 0x03:
		return "OC_Value";   /* 过流阈值 */
	case 0x04:
		return "ACC";        /* 加速度 */
	case 0x05:
		return "DEC";        /* 减速度 */
	case 0x06:
		return "MAX_SPD";    /* 最大速度 */
	case 0x07:
		return "MST_ID";     /* 主站ID */
	case 0x08:
		return "ESC_ID";     /* 电调ID */
	case 0x09:
		return "TIMEOUT";    /* 超时时间 */
	case 0x0a:
		return "CTRL_MODE";  /* 控制模式 */
	case 0x15:
		return "PMAX";       /* 位置范围 */
	case 0x16:
		return "VMAX";       /* 速度范围 */
	case 0x17:
		return "TMAX";       /* 扭矩范围 */
	case 0x23:
		return "can_br";     /* CAN波特率 */
	case 0x3b:
		return "Imax";       /* 最大电流 */
	case 0x3c:
		return "VBus";       /* 母线电压 */
	default:
		return "reg";
	}
}

/**
 * @brief 判断 DM4340 寄存器值是否为无符号 32 位整数类型
 *
 * 部分寄存器（如 MST_ID、ESC_ID、TIMEOUT、CTRL_MODE、can_br 等）
 * 存储的是无符号整数值，而大部分寄存器（如 KT、PMAX、VMAX 等）
 * 存储的是浮点值。此函数用于决定寄存器读取后的显示格式。
 *
 * @param rid 寄存器 ID
 * @return true 表示 u32 类型，false 表示浮点类型
 */
static bool dm_reg_is_u32(uint8_t rid)
{
	switch (rid) {
	case 0x07:
	case 0x08:
	case 0x09:
	case 0x0a:
	case 0x23:
	case 0x24:
	case 0x25:
		return true;
	default:
		return false;
	}
}

/**
 * @brief 将 DM4340 控制模式编号转换为可读名称
 *
 * DM4340 支持四种控制模式：
 *   1 MIT      — MIT 模式，直接指定位置/速度/Kp/Kd/前馈扭矩
 *   2 pos_vel  — 位置-速度模式，指定目标位置和最大速度
 *   3 velocity — 速度模式，直接指定目标速度
 *   4 pvt      — 位置-速度-扭矩（PVT）模式
 *
 * @param mode 控制模式编号
 * @return 模式名称字符串
 */
static const char *dm_ctrl_mode_name(uint32_t mode)
{
	switch (mode) {
	case 1:
		return "MIT";
	case 2:
		return "pos_vel";
	case 3:
		return "velocity";
	case 4:
		return "pvt";
	default:
		return "unknown";
	}
}

/**
 * @brief robot enable — 启用/禁用平衡控制器
 *
 * 用法：robot enable <0|1|on|off|true|false|enable|disable>
 *
 * 启用后，平衡控制器开始工作，机器人尝试自主保持平衡；
 * 禁用后，所有轮子电流归零，机器人停止平衡。
 *
 * @param sh   Shell 实例
 * @param argc 参数总数（含命令名本身）
 * @param argv 参数数组：argv[1] 为启用/禁用标志
 */
static int cmd_robot_enable(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);

	const bool enable = parse_bool(argv[1]);
	control_set_enable(enable);
	shell_print(sh, "balance %s", enable ? "enabled" : "disabled");
	return 0;
}

/**
 * @brief robot stop — 立即停止运动
 *
 * 清除所有运动指令，机器人回到原地站立状态（保持平衡但不移动）。
 * 这是 robot motion stop 的快捷方式。
 *
 * @param sh   Shell 实例
 * @param argc 参数总数（仅 1，无额外参数）
 * @param argv 参数数组
 */
static int cmd_robot_stop(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	control_stop_motion();
	shell_print(sh, "motion stopped");
	return 0;
}

/**
 * @brief robot status — 显示机器人完整状态
 *
 * 输出所有关键状态信息，包括：
 *   enable_request  — 平衡控制器是否请求启用
 *   wheels_enabled  — 轮毂电机是否实际使能
 *   faulted         — 是否触发故障保护
 *   height          — 当前腿部高度
 *   joy_x, joy_y   — 当前摇杆输入
 *   pitch_deg       — 俯仰角（度）
 *   err             — 俯仰角与平衡零点的偏差（度）
 *   pitch_rate      — 俯仰角速率（度/秒）
 *   roll_deg        — 横滚角（度）
 *   yaw_deg         — 偏航角（度）
 *   speed           — 轮速（弧度/秒）
 *   lqr_output      — LQR 控制器输出（平衡力矩）
 *   yaw_output      — 偏航控制输出（转向力矩）
 *   current         — 左右轮电流（mA）
 *   joint           — 左右关节位置（弧度）
 *   jump_phase      — 跳跃状态机阶段
 *
 * @param sh   Shell 实例
 * @param argc 参数总数（仅 1）
 * @param argv 参数数组
 */
static int cmd_robot_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	control_status_t st;
	ascento_balance_params_t params;
	float pitch_error_deg = 0.0f;

	control_get_status(&st);
	ascento_balance_get_params(&params);
	pitch_error_deg =
		st.pitch_deg - params.theta_eq_rad / 0.017453292519943295f;

	shell_print(sh,
		    "enable=%d wheels=%d fault=%d height=%d joy=(%.1f,%.1f) "
		    "pitch=%.2f err=%.2f pitch_rate=%.1f roll=%.2f yaw=%.2f speed=%.2f "
		    "lqr=%.1f yaw_out=%.1f current=(%d,%d) "
		    "joint=(%.3f,%.3f) jump=%d",
		    st.enable_request, st.wheels_enabled, st.faulted, st.height,
		    (double)st.joy_x, (double)st.joy_y,
		    (double)st.pitch_deg, (double)pitch_error_deg,
		    (double)st.pitch_rate_dps,
		    (double)st.roll_deg,
		    (double)st.yaw_deg, (double)st.speed_rad_s,
		    (double)st.lqr_output, (double)st.yaw_output,
		    st.left_wheel_current, st.right_wheel_current,
		    (double)st.left_joint_position_rad,
		    (double)st.right_joint_position_rad, st.jump_phase);
	return 0;
}

/**
 * @brief 打印指定 CAN 总线的状态信息
 *
 * 查询并显示 CAN 控制器的状态和错误计数器：
 *   state   — 当前状态（error-active/error-warning/error-passive/bus-off/stopped）
 *   tx_err  — 发送错误计数器（超过 255 进入 bus-off）
 *   rx_err  — 接收错误计数器
 *
 * @param sh   Shell 实例
 * @param name 总线人类可读名称（如 "joint/CAN1"）
 * @param dev  CAN 设备句柄
 * @return 0 成功，负值表示错误
 */
static int print_can_status(const struct shell *sh, const char *name,
			    const struct device *dev)
{
	if (!device_is_ready(dev)) {
		shell_error(sh, "%s device is not ready", name);
		return -ENODEV;
	}

	enum can_state state;
	struct can_bus_err_cnt err_cnt;
	const int ret = can_get_state(dev, &state, &err_cnt);

	if (ret != 0) {
		shell_error(sh, "%s get state failed: %d", name, ret);
		return ret;
	}

	shell_print(sh, "%s state=%s tx_err=%u rx_err=%u", name,
		    can_state_name(state), err_cnt.tx_err_cnt,
		    err_cnt.rx_err_cnt);
	return 0;
}

/**
 * @brief motor can status — 显示 CAN 总线状态
 *
 * 用法：
 *   motor can status              — 显示所有 CAN 总线状态
 *   motor can status all          — 同上
 *   motor can status <joint|dm|can1>  — 仅显示关节 CAN 总线
 *   motor can status <wheel|m3508|can2> — 仅显示轮毂 CAN 总线
 *
 * @param sh   Shell 实例
 * @param argc 参数总数
 * @param argv 参数数组：可选的总线名称
 */
static int cmd_motor_can_status(const struct shell *sh, size_t argc,
				char **argv)
{
	if (argc == 1 || strcmp(argv[1], "all") == 0) {
		(void)print_can_status(sh, "joint/CAN1", shell_joint_can);
		(void)print_can_status(sh, "wheel/CAN2", shell_wheel_can);
		return 0;
	}

	const struct device *dev;
	const char *name;
	if (!parse_can_bus(argv[1], &dev, &name)) {
		shell_error(sh, "bus must be joint|dm|can1|wheel|m3508|can2");
		return -EINVAL;
	}

	return print_can_status(sh, name, dev);
}

/**
 * @brief 发送原始 CAN 帧（标准帧或扩展帧的通用实现）
 *
 * 用法：
 *   motor can raw  <joint|wheel|can1|can2> <std_id> [byte0..byte7]
 *   motor can rawx <joint|wheel|can1|can2> <ext_id> [byte0..byte7]
 *
 * 直接在指定 CAN 总线上发送一帧原始报文，用于底层调试和电机初始化。
 * 标准帧 ID 为 11 位（0x000..0x7FF），扩展帧 ID 为 29 位（0x00000000..0x1FFFFFFF）。
 * 数据字节数由参数个数决定（0..8 字节）。
 *
 * 注意：此命令会自动进入电机调试模式（禁用平衡控制器）。
 *
 * @param sh       Shell 实例
 * @param argc     参数总数
 * @param argv     参数数组
 * @param extended true 发送扩展帧（29位ID），false 发送标准帧（11位ID）
 * @return 0 成功，负值表示错误
 */
static int cmd_motor_can_raw_common(const struct shell *sh, size_t argc,
				    char **argv, bool extended)
{
	const struct device *dev;
	const char *name;
	int32_t id_value;
	const int32_t max_id = extended ? CAN_EXT_ID_MASK : CAN_STD_ID_MASK;

	if (!parse_can_bus(argv[1], &dev, &name) ||
	    !parse_i32(argv[2], &id_value) || id_value < 0 ||
	    id_value > max_id) {
		shell_error(sh,
			    "usage: motor can %s <joint|wheel|can1|can2> "
			    "<%s_id> [byte0..byte7]",
			    extended ? "rawx" : "raw",
			    extended ? "ext" : "std");
		return -EINVAL;
	}

	struct can_frame frame = {
		.flags = extended ? CAN_FRAME_IDE : 0,
		.id = (uint32_t)id_value,
		.dlc = (uint8_t)(argc - 3U),
	};

	for (size_t i = 3; i < argc; i++) {
		int32_t byte_value;

		if (!parse_i32(argv[i], &byte_value) || byte_value < 0 ||
		    byte_value > 0xff) {
			shell_error(sh, "byte%u must be 0..255",
				    (unsigned int)(i - 3U));
			return -EINVAL;
		}
		frame.data[i - 3U] = (uint8_t)byte_value;
	}

	enter_motor_debug_mode();
	const int ret = can_send(dev, &frame, K_MSEC(5), NULL, NULL);
	shell_print(sh, "%s raw%s id=0x%0*x dlc=%u ret=%d", name,
		    extended ? "x" : "", extended ? 8 : 3, frame.id,
		    frame.dlc, ret);
	return ret;
}

/**
 * @brief motor can raw — 发送标准帧（11位ID）原始 CAN 报文
 *
 * @param sh   Shell 实例
 * @param argc 参数总数
 * @param argv 参数数组
 */
static int cmd_motor_can_raw(const struct shell *sh, size_t argc, char **argv)
{
	return cmd_motor_can_raw_common(sh, argc, argv, false);
}

/**
 * @brief motor can rawx — 发送扩展帧（29位ID）原始 CAN 报文
 *
 * @param sh   Shell 实例
 * @param argc 参数总数
 * @param argv 参数数组
 */
static int cmd_motor_can_rawx(const struct shell *sh, size_t argc, char **argv)
{
	return cmd_motor_can_raw_common(sh, argc, argv, true);
}

/**
 * @brief motor can recover — 重置 CAN 控制器
 *
 * 用法：
 *   motor can recover                     — 重置所有 CAN 总线
 *   motor can recover all                 — 同上
 *   motor can recover <joint|dm|can1>     — 仅重置关节 CAN 总线
 *   motor can recover <wheel|m3508|can2>  — 仅重置轮毂 CAN 总线
 *
 * 通过先 stop 再 start 的方式重置 CAN 控制器，用于从 bus-off 等异常状态恢复。
 * 重置后会自动打印总线状态供确认。
 *
 * 注意：此命令会自动进入电机调试模式（禁用平衡控制器）。
 *
 * @param sh   Shell 实例
 * @param argc 参数总数
 * @param argv 参数数组：可选的总线名称
 */
static int cmd_motor_can_recover(const struct shell *sh, size_t argc, char **argv)
{
	if (argc == 1 || strcmp(argv[1], "all") == 0) {
		can_stop(shell_joint_can);
		can_start(shell_joint_can);
		can_stop(shell_wheel_can);
		can_start(shell_wheel_can);
		(void)print_can_status(sh, "joint/CAN1", shell_joint_can);
		(void)print_can_status(sh, "wheel/CAN2", shell_wheel_can);
		return 0;
	}

	const struct device *dev;
	const char *name;
	if (!parse_can_bus(argv[1], &dev, &name)) {
		shell_error(sh, "bus must be joint|dm|can1|wheel|m3508|can2|all");
		return -EINVAL;
	}

	enter_motor_debug_mode();
	can_stop(dev);
	can_start(dev);
	return print_can_status(sh, name, dev);
}

/**
 * @brief 打印指定轮毂电机（VESC/M3508）的完整状态
 *
 * 显示信息包括：
 *   age           — 自上次收到反馈以来的时间（毫秒）
 *   erpm          — 电气转速（ERPM，电调上报的原始值）
 *   motor_rpm     — 机械转速（RPM）
 *   angle         — 转子角度（弧度，由电调反馈累积）
 *   speed         — 角速度（弧度/秒）
 *   cmd           — 当前下发的指令电流（mA）
 *   motor_current — 电机实际电流（mA）
 *   input         — 输入侧电流（mA）
 *   vin           — 输入电压（V）
 *   temp          — FET/电机温度（摄氏度）
 *   tach          — 里程计（转数计数器）
 *   torque_k      — 电流-扭矩转换系数（Nm/mA）
 *   torque_est    — 估算扭矩（Nm）
 *   duty          — 占空比
 *   s4_age/s5_age — 状态包4/5的更新年龄（用于诊断 VESC 反馈频率）
 *
 * @param sh Shell 实例
 * @param id 电机 CAN ID
 * @return 0 成功，-ENODATA 表示无反馈数据
 */
static int print_wheel_status(const struct shell *sh, uint8_t id)
{
	dji_m3508_motor_t motor;

	if (!motor_debug_get_m3508(id, &motor)) {
		shell_print(sh, "VESC/M3508 id=%u no feedback", id);
		return -ENODATA;
	}

	const int64_t age_ms = k_uptime_get() - motor.last_update_ms;
	const int64_t s4_age_ms = motor.last_status4_update_ms > 0 ?
				  k_uptime_get() -
					  motor.last_status4_update_ms :
				  -1;
	const int64_t s5_age_ms = motor.last_status5_update_ms > 0 ?
				  k_uptime_get() -
					  motor.last_status5_update_ms :
				  -1;
	shell_print(sh,
		    "VESC/M3508 id=%u age=%lldms erpm=%d motor_rpm=%d "
		    "angle=%.3f rad speed=%.3f rad/s cmd=%d mA "
		    "motor_current=%d mA input=%d mA vin=%.2f V "
		    "temp=%.1f/%.1fC tach=%d torque_k=%.6f "
		    "torque_est=%.3f Nm duty=%.3f s4_age=%lldms "
		    "s5_age=%lldms",
		    id, (long long)age_ms, motor.erpm, motor.speed_rpm,
		    (double)motor.angle_rad, (double)motor.speed_rad_s,
		    motor.command_current_ma, motor.motor_current_ma,
		    motor.input_current_ma,
		    (double)motor.input_voltage_mv * 0.001,
		    (double)motor.fet_temperature_cdeg * 0.1,
		    (double)motor.motor_temperature_cdeg * 0.1,
		    motor.tachometer,
		    (double)motor.current_ma_to_wheel_torque_nm,
		    (double)motor.estimated_wheel_torque_nm,
		    (double)motor.duty, (long long)s4_age_ms,
		    (long long)s5_age_ms);
	return 0;
}

/**
 * @brief motor wheel status — 显示轮毂电机状态
 *
 * 用法：
 *   motor wheel status                  — 显示左右两个轮毂电机的状态
 *   motor wheel status all              — 同上
 *   motor wheel status <left|right|id>  — 仅显示指定电机
 *
 * @param sh   Shell 实例
 * @param argc 参数总数
 * @param argv 参数数组：可选的电机标识
 */
static int cmd_motor_wheel_status(const struct shell *sh, size_t argc,
				  char **argv)
{
	if (argc == 1 || strcmp(argv[1], "all") == 0) {
		(void)print_wheel_status(sh, APP_WHEEL_LEFT_ID);
		(void)print_wheel_status(sh, APP_WHEEL_RIGHT_ID);
		return 0;
	}

	uint8_t id;
	if (!parse_wheel_id(argv[1], &id)) {
		shell_error(sh, "wheel id must be left/right/1..255");
		return -EINVAL;
	}

	return print_wheel_status(sh, id);
}

/**
 * @brief motor wheel current — 设置轮毂电机电流指令
 *
 * 用法：motor wheel current <left|right|id> <current_mA> [ms]
 *
 * 通过 VESC 协议直接向指定轮毂电机发送电流指令（毫安）。
 * 正值前进，负值后退。可选参数 ms 指定指令持续时间，到期后自动停止。
 *
 * 注意：此命令会自动进入电机调试模式（禁用平衡控制器）。
 *
 * @param sh   Shell 实例
 * @param argc 参数总数
 * @param argv 参数数组：argv[1]=电机ID, argv[2]=电流mA, argv[3]=持续时间ms(可选)
 */
static int cmd_motor_wheel_current(const struct shell *sh, size_t argc,
				   char **argv)
{
	uint8_t id;
	int32_t current;
	int32_t duration_ms;

	if (!parse_wheel_id(argv[1], &id) || !parse_i32(argv[2], &current) ||
	    !parse_optional_duration(argc, argv, 3, &duration_ms)) {
		shell_error(sh,
			    "usage: motor wheel current <left|right|id> "
			    "<current_mA> [ms]");
		return -EINVAL;
	}

	enter_motor_debug_mode();
	const int ret = motor_debug_set_wheel_current(id, current, duration_ms);
	if (ret != 0) {
		shell_error(sh, "set wheel current failed: %d", ret);
		return ret;
	}

	shell_print(sh, "VESC/M3508 id=%u current=%d mA for %d ms", id,
		    (int)current, duration_ms);
	return 0;
}

/**
 * @brief motor wheel rpm — 设置轮毂电机目标转速
 *
 * 用法：motor wheel rpm <left|right|100|101> <target_erpm> [ms]
 *
 * 通过 VESC 协议设置轮毂电机的目标电气转速（ERPM）。
 * ERPM = RPM * 电机极对数。实际转速受 APP_VESC_DEBUG_ERPM_LIMIT 限制。
 * 此命令仅支持已配置的左右轮电机（APP_WHEEL_LEFT_ID / APP_WHEEL_RIGHT_ID）。
 *
 * 注意：此命令会自动进入电机调试模式（禁用平衡控制器）。
 *
 * @param sh   Shell 实例
 * @param argc 参数总数
 * @param argv 参数数组：argv[1]=电机ID, argv[2]=目标ERPM, argv[3]=持续时间ms(可选)
 */
static int cmd_motor_wheel_rpm(const struct shell *sh, size_t argc,
			       char **argv)
{
	uint8_t id;
	int32_t rpm;
	int32_t duration_ms;

	if (!parse_wheel_id(argv[1], &id) || !parse_i32(argv[2], &rpm) ||
	    !parse_optional_duration(argc, argv, 3, &duration_ms)) {
		shell_error(sh,
			    "usage: motor wheel rpm <left|right|100|101> "
			    "<target_erpm> [ms]");
		return -EINVAL;
	}

	enter_motor_debug_mode();
	const int ret = motor_debug_set_wheel_rpm(id, rpm, duration_ms);
	if (ret != 0) {
		shell_error(sh,
			    "rpm command only supports configured wheels "
			    "left=%u right=%u",
			    APP_WHEEL_LEFT_ID, APP_WHEEL_RIGHT_ID);
		return ret;
	}

	shell_print(sh, "VESC/M3508 id=%u target_erpm=%d for %d ms", id,
		    (int)CLAMP(rpm, -APP_VESC_DEBUG_ERPM_LIMIT,
			       APP_VESC_DEBUG_ERPM_LIMIT),
		    duration_ms);
	return 0;
}

/**
 * @brief motor wheel pair — 同时设置左右轮毂电机电流
 *
 * 用法：motor wheel pair <left_current_mA> <right_current_mA> [ms]
 *
 * 一条命令同时控制两个轮毂电机的电流，方便测试差速转向。
 * 左右电流可以不同以产生转向力矩。可选参数 ms 指定持续时间。
 *
 * 注意：此命令会自动进入电机调试模式（禁用平衡控制器）。
 *
 * @param sh   Shell 实例
 * @param argc 参数总数
 * @param argv 参数数组：argv[1]=左轮电流, argv[2]=右轮电流, argv[3]=持续时间ms(可选)
 */
static int cmd_motor_wheel_pair(const struct shell *sh, size_t argc,
				char **argv)
{
	int32_t left_current;
	int32_t right_current;
	int32_t duration_ms;

	if (!parse_i32(argv[1], &left_current) ||
	    !parse_i32(argv[2], &right_current) ||
	    !parse_optional_duration(argc, argv, 3, &duration_ms)) {
		shell_error(sh,
			    "usage: motor wheel pair <left_current_mA> "
			    "<right_current_mA> [ms]");
		return -EINVAL;
	}

	enter_motor_debug_mode();
	const int ret = motor_debug_set_wheel_pair(left_current, right_current,
						   duration_ms);
	if (ret != 0) {
		shell_error(sh, "set wheel pair failed: %d", ret);
		return ret;
	}

	shell_print(sh, "VESC/M3508 pair current=(%d,%d) mA for %d ms",
		    (int)left_current, (int)right_current, duration_ms);
	return 0;
}

/**
 * @brief motor wheel stop — 停止所有轮毂电机电流输出
 *
 * 立即将左右轮毂电机的电流指令归零。
 * 是 motor wheel current/rpm/pair 的安全停止命令。
 *
 * 注意：此命令会自动进入电机调试模式（禁用平衡控制器）。
 *
 * @param sh   Shell 实例
 * @param argc 参数总数（仅 1）
 * @param argv 参数数组
 */
static int cmd_motor_wheel_stop(const struct shell *sh, size_t argc,
				char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	enter_motor_debug_mode();
	const int ret = motor_debug_stop_wheels();
	shell_print(sh, "VESC/M3508 current stopped");
	return ret;
}

/**
 * @brief 打印指定关节电机（DM4340）的反馈状态
 *
 * 显示信息包括：
 *   err      — 错误码（0 表示正常）
 *   age      — 自上次收到反馈以来的时间（毫秒）
 *   pos      — 当前关节位置（弧度）
 *   vel      — 当前关节速度（弧度/秒）
 *   torque   — 当前关节扭矩（牛米）
 *   temp     — MOS管温度/转子温度（摄氏度）
 *
 * @param sh Shell 实例
 * @param id 电机 CAN ID
 * @return 0 成功，-ENODATA 表示无反馈数据
 */
static int print_dm_status(const struct shell *sh, uint8_t id)
{
	dm4340_feedback_t fb;

	if (!motor_debug_get_dm4340(id, &fb)) {
		shell_print(sh, "DM4340 id=%u no feedback", id);
		return -ENODATA;
	}

	const int64_t age_ms = k_uptime_get() - fb.last_update_ms;
	shell_print(sh,
		    "DM4340 id=%u err=%u age=%lldms pos=%.4f rad "
		    "vel=%.4f rad/s torque=%.3f Nm temp=%u/%uC",
		    id, fb.error, (long long)age_ms,
		    (double)fb.position_rad, (double)fb.velocity_rad_s,
		    (double)fb.torque_nm, fb.mos_temperature_c,
		    fb.rotor_temperature_c);
	return 0;
}

/**
 * @brief motor dm status — 显示关节电机状态
 *
 * 用法：
 *   motor dm status                  — 显示左右两个关节电机的状态
 *   motor dm status all              — 同上
 *   motor dm status <left|right|id>  — 仅显示指定关节电机
 *
 * @param sh   Shell 实例
 * @param argc 参数总数
 * @param argv 参数数组：可选的电机标识
 */
static int cmd_motor_dm_status(const struct shell *sh, size_t argc,
			       char **argv)
{
	if (argc == 1 || strcmp(argv[1], "all") == 0) {
		(void)print_dm_status(sh, APP_DM_LEFT_ID);
		(void)print_dm_status(sh, APP_DM_RIGHT_ID);
		return 0;
	}

	uint8_t id;
	if (!parse_dm_id(argv[1], &id)) {
		shell_error(sh, "DM id must be left/right/1..15");
		return -EINVAL;
	}

	return print_dm_status(sh, id);
}

/**
 * @brief motor dm enable — 使能关节电机
 *
 * 用法：motor dm enable <left|right|id>
 *
 * 向 DM4340 发送使能指令，电机进入伺服模式，开始响应位置/速度/扭矩指令。
 * 使能前建议先确认零点已正确设置。
 *
 * 注意：此命令会自动进入电机调试模式（禁用平衡控制器）。
 *
 * @param sh   Shell 实例
 * @param argc 参数总数
 * @param argv 参数数组：argv[1] 为电机标识
 */
static int cmd_motor_dm_enable(const struct shell *sh, size_t argc,
			       char **argv)
{
	ARG_UNUSED(argc);

	uint8_t id;
	if (!parse_dm_id(argv[1], &id)) {
		shell_error(sh, "DM id must be left/right/1..15");
		return -EINVAL;
	}

	enter_motor_debug_mode();
	const int ret = motor_debug_dm_enable(id);
	shell_print(sh, "DM4340 id=%u enable ret=%d", id, ret);
	return ret;
}

/**
 * @brief motor dm disable — 失能关节电机
 *
 * 用法：motor dm disable <left|right|id>
 *
 * 向 DM4340 发送失能指令，电机退出伺服模式，进入自由转动状态。
 * 失能后关节可以自由摆动，适合手动调整关节位置。
 *
 * 注意：此命令会自动进入电机调试模式（禁用平衡控制器）。
 *
 * @param sh   Shell 实例
 * @param argc 参数总数
 * @param argv 参数数组：argv[1] 为电机标识
 */
static int cmd_motor_dm_disable(const struct shell *sh, size_t argc,
				char **argv)
{
	ARG_UNUSED(argc);

	uint8_t id;
	if (!parse_dm_id(argv[1], &id)) {
		shell_error(sh, "DM id must be left/right/1..15");
		return -EINVAL;
	}

	enter_motor_debug_mode();
	const int ret = motor_debug_dm_disable(id);
	shell_print(sh, "DM4340 id=%u disable ret=%d", id, ret);
	return ret;
}

/**
 * @brief motor dm zero — 保存关节电机当前位置为零点
 *
 * 用法：motor dm zero <left|right|id>
 *
 * 将 DM4340 当前的编码器位置保存为零点（写入电机内部 Flash）。
 * 之后的位置指令都相对于此零点。通常在机器人组装完成后、
 * 关节处于期望的"零位"姿态时执行一次。
 *
 * 注意：此命令会自动进入电机调试模式（禁用平衡控制器）。
 *
 * @param sh   Shell 实例
 * @param argc 参数总数
 * @param argv 参数数组：argv[1] 为电机标识
 */
static int cmd_motor_dm_zero(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);

	uint8_t id;
	if (!parse_dm_id(argv[1], &id)) {
		shell_error(sh, "DM id must be left/right/1..15");
		return -EINVAL;
	}

	enter_motor_debug_mode();
	const int ret = motor_debug_dm_save_zero(id);
	shell_print(sh, "DM4340 id=%u save zero ret=%d", id, ret);
	return ret;
}

/**
 * @brief motor dm reg — 读取 DM4340 寄存器
 *
 * 用法：motor dm reg <left|right|id> <rid>
 *
 * 通过 CAN 读取 DM4340 指定寄存器的值。rid 为十六进制寄存器地址。
 * 寄存器值的显示格式取决于寄存器类型：
 *   - u32 类型（如 MST_ID、CTRL_MODE）显示为无符号整数
 *   - 浮点类型（如 KT、PMAX、VMAX）显示为浮点数和原始十六进制值
 *   - CTRL_MODE 寄存器会额外显示模式名称（MIT/pos_vel/velocity/pvt）
 *
 * 常用寄存器参见 dm_reg_name() 中的注释。
 *
 * 注意：此命令会自动进入电机调试模式（禁用平衡控制器）。
 *
 * @param sh   Shell 实例
 * @param argc 参数总数
 * @param argv 参数数组：argv[1]=电机标识, argv[2]=寄存器ID
 */
static int cmd_motor_dm_reg(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);

	uint8_t id;
	int32_t rid_value;
	if (!parse_dm_id(argv[1], &id) || !parse_i32(argv[2], &rid_value) ||
	    rid_value < 0 || rid_value > 0xff) {
		shell_error(sh, "usage: motor dm reg <left|right|id> <rid>");
		return -EINVAL;
	}

	enter_motor_debug_mode();
	dm4340_param_response_t response;
	const int ret = motor_debug_dm_read_reg(id, (uint8_t)rid_value,
						&response);
	if (ret != 0) {
		shell_error(sh, "DM4340 id=%u reg 0x%02x read failed: %d",
			    id, (unsigned int)rid_value, ret);
		return ret;
	}

	const int64_t age_ms = k_uptime_get() - response.last_update_ms;
	const uint32_t u32 = response.raw_u32;
	const char *name = dm_reg_name((uint8_t)rid_value);

	if (dm_reg_is_u32((uint8_t)rid_value)) {
		if ((uint8_t)rid_value == 0x0a) {
			shell_print(sh,
				    "DM4340 id=%u %s(0x%02x) u32=%u mode=%s raw=0x%08x age=%lldms",
				    id, name, (unsigned int)rid_value, u32,
				    dm_ctrl_mode_name(u32), u32,
				    (long long)age_ms);
		} else {
			shell_print(sh,
				    "DM4340 id=%u %s(0x%02x) u32=%u raw=0x%08x age=%lldms",
				    id, name, (unsigned int)rid_value, u32,
				    u32, (long long)age_ms);
		}
	} else {
		shell_print(sh,
			    "DM4340 id=%u %s(0x%02x) f=%.6f raw=0x%08x u32=%u age=%lldms",
			    id, name, (unsigned int)rid_value,
			    (double)response.value_float, u32, u32,
			    (long long)age_ms);
	}

	return 0;
}

/**
 * @brief motor dm diag — 一键读取 DM4340 所有关键诊断寄存器
 *
 * 用法：motor dm diag <left|right|id>
 *
 * 批量读取以下关键寄存器，用于快速诊断电机健康状态：
 *   0x01 FAULT      — 故障码
 *   0x02 WARNING    — 警告码
 *   0x04 STATUS     — 状态码
 *   0x05 CAN_ERR    — CAN 通信错误计数
 *   0x06 MOTOR_ERR  — 电机错误码
 *   0x09 TIMEOUT    — 通信超时设置
 *   0x0a CTRL_MODE  — 当前控制模式
 *   0x15 PMAX       — 位置范围上限
 *   0x16 VMAX       — 速度范围上限
 *   0x17 TMAX       — 扭矩范围上限
 *   0x3b Imax       — 最大电流
 *   0x3c VBus       — 母线电压
 *
 * 同时也会显示电机的实时反馈（位置/速度/扭矩/温度）。
 *
 * 注意：此命令会自动进入电机调试模式（禁用平衡控制器）。
 *
 * @param sh   Shell 实例
 * @param argc 参数总数
 * @param argv 参数数组：argv[1] 为电机标识
 */
static int cmd_motor_dm_diag(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);

	uint8_t id;
	if (!parse_dm_id(argv[1], &id)) {
		shell_error(sh, "DM id must be left/right/1..15");
		return -EINVAL;
	}

	enter_motor_debug_mode();

	/* 需要读取的关键诊断寄存器列表 */
	static const uint8_t diag_rids[] = {
		0x01, /* FAULT     — 故障码 */
		0x02, /* WARNING   — 警告码 */
		0x04, /* STATUS    — 状态码 */
		0x05, /* CAN_ERR   — CAN 通信错误计数 */
		0x06, /* MOTOR_ERR — 电机错误码 */
		0x09, /* TIMEOUT   — 通信超时设置 */
		0x0a, /* CTRL_MODE — 当前控制模式 */
		0x15, /* PMAX      — 位置范围上限 */
		0x16, /* VMAX      — 速度范围上限 */
		0x17, /* TMAX      — 扭矩范围上限 */
		0x3b, /* Imax      — 最大电流 */
		0x3c, /* VBus      — 母线电压 */
	};

	dm4340_feedback_t fb;
	if (!motor_debug_get_dm4340(id, &fb)) {
		shell_print(sh, "DM4340 id=%u no feedback", id);
	} else {
		const int64_t age_ms = k_uptime_get() - fb.last_update_ms;
		shell_print(sh,
			    "DM4340 id=%u err=0x%x age=%lldms pos=%.4f "
			    "vel=%.4f torque=%.4f temp=%u/%uC",
			    id, fb.error, (long long)age_ms,
			    (double)fb.position_rad, (double)fb.velocity_rad_s,
			    (double)fb.torque_nm, fb.mos_temperature_c,
			    fb.rotor_temperature_c);
	}

	shell_print(sh, "--- DM4340 id=%u diagnostic registers ---", id);

	for (size_t i = 0; i < ARRAY_SIZE(diag_rids); i++) {
		const uint8_t rid = diag_rids[i];
		dm4340_param_response_t response;
		const int ret = motor_debug_dm_read_reg(id, rid, &response);
		if (ret != 0) {
			shell_print(sh, "  %s(0x%02x) read failed: %d",
				    dm_reg_name(rid), (unsigned int)rid, ret);
			continue;
		}

		const char *name = dm_reg_name(rid);
		const uint32_t u32 = response.raw_u32;
		const int64_t age_ms = k_uptime_get() - response.last_update_ms;

		if (dm_reg_is_u32(rid)) {
			if (rid == 0x0a) {
				shell_print(sh,
					    "  %s(0x%02x) u32=%u mode=%s age=%lldms",
					    name, (unsigned int)rid, u32,
					    dm_ctrl_mode_name(u32),
					    (long long)age_ms);
			} else {
				shell_print(sh,
					    "  %s(0x%02x) u32=%u raw=0x%08x age=%lldms",
					    name, (unsigned int)rid, u32, u32,
					    (long long)age_ms);
			}
		} else {
			shell_print(sh,
				    "  %s(0x%02x) f=%.6f raw=0x%08x age=%lldms",
				    name, (unsigned int)rid,
				    (double)response.value_float, u32,
				    (long long)age_ms);
		}
	}

	shell_print(sh, "--- end diag ---");
	return 0;
}

/**
 * @brief motor dm pos — 设置关节电机目标位置（位置-速度模式）
 *
 * 用法：motor dm pos <left|right|id> <rad> [vel_rad_s] [ms]
 *
 * 使用 DM4340 的位置-速度模式（pos_vel），指定目标位置和最大运动速度。
 *   rad       — 目标位置（弧度，相对于零点）
 *   vel_rad_s — 最大运动速度（弧度/秒，默认 1.0）
 *   ms        — 指令持续时间（毫秒），到期后停止
 *
 * 注意：此命令会自动进入电机调试模式（禁用平衡控制器）。
 *
 * @param sh   Shell 实例
 * @param argc 参数总数
 * @param argv 参数数组
 */
static int cmd_motor_dm_pos(const struct shell *sh, size_t argc, char **argv)
{
	uint8_t id;
	float position_rad;
	float velocity_rad_s = 1.0f;
	int32_t duration_ms = APP_MOTOR_DEBUG_DEFAULT_TIMEOUT_MS;

	if (!parse_dm_id(argv[1], &id) ||
	    !parse_float_arg(argv[2], &position_rad)) {
		shell_error(sh,
			    "usage: motor dm pos <left|right|id> <rad> "
			    "[vel_rad_s] [ms]");
		return -EINVAL;
	}
	if (argc > 3 && !parse_float_arg(argv[3], &velocity_rad_s)) {
		shell_error(sh, "invalid velocity");
		return -EINVAL;
	}
	if (argc > 4 && !parse_i32(argv[4], &duration_ms)) {
		shell_error(sh, "invalid duration");
		return -EINVAL;
	}

	enter_motor_debug_mode();
	const int ret = motor_debug_set_dm_pos_vel(
		id, position_rad, velocity_rad_s, duration_ms);
	if (ret != 0) {
		shell_error(sh, "set DM position failed: %d", ret);
		return ret;
	}

	shell_print(sh, "DM4340 id=%u pos=%.4f vel=%.3f for %d ms", id,
		    (double)position_rad, (double)velocity_rad_s,
		    duration_ms);
	return 0;
}

/**
 * @brief motor dm vel — 设置关节电机目标速度
 *
 * 用法：motor dm vel <left|right|id> <rad_s> [ms]
 *
 * 使用 DM4340 的速度模式（velocity），关节以指定速度持续转动。
 *   rad_s — 目标角速度（弧度/秒），正值/负值对应不同方向
 *   ms    — 指令持续时间（毫秒），默认 APP_MOTOR_DEBUG_DEFAULT_TIMEOUT_MS
 *
 * 注意：此命令会自动进入电机调试模式（禁用平衡控制器）。
 *
 * @param sh   Shell 实例
 * @param argc 参数总数
 * @param argv 参数数组
 */
static int cmd_motor_dm_vel(const struct shell *sh, size_t argc, char **argv)
{
	uint8_t id;
	float velocity_rad_s;
	int32_t duration_ms;

	if (!parse_dm_id(argv[1], &id) ||
	    !parse_float_arg(argv[2], &velocity_rad_s) ||
	    !parse_optional_duration(argc, argv, 3, &duration_ms)) {
		shell_error(sh,
			    "usage: motor dm vel <left|right|id> "
			    "<rad_s> [ms]");
		return -EINVAL;
	}

	enter_motor_debug_mode();
	const int ret = motor_debug_set_dm_velocity(id, velocity_rad_s,
						    duration_ms);
	if (ret != 0) {
		shell_error(sh, "set DM velocity failed: %d", ret);
		return ret;
	}

	shell_print(sh, "DM4340 id=%u vel=%.3f for %d ms", id,
		    (double)velocity_rad_s, duration_ms);
	return 0;
}

/**
 * @brief motor dm mit — MIT 模式直接控制关节电机
 *
 * 用法：motor dm mit <left|right|id> <pos_rad> <vel_rad_s> <kp> <kd> <torque_nm> [ms]
 *
 * 使用 DM4340 的 MIT 模式，直接指定五个控制参数：
 *   pos_rad    — 目标位置（弧度），用于 PD 位置控制
 *   vel_rad_s  — 目标速度（弧度/秒），用于 PD 速度控制
 *   kp         — 位置增益（N.m/rad），对位置偏差的刚度
 *   kd         — 速度增益（N.m.s/rad），对速度偏差的阻尼
 *   torque_nm  — 前馈扭矩（N.m），直接叠加的力矩
 *   ms         — 指令持续时间（毫秒）
 *
 * MIT 模式是 DM4340 最灵活的控制模式，扭矩计算公式：
 *   τ = kp * (pos_target - pos_actual) + kd * (vel_target - vel_actual) + torque_ff
 *
 * 注意：此命令会自动进入电机调试模式（禁用平衡控制器）。
 *
 * @param sh   Shell 实例
 * @param argc 参数总数
 * @param argv 参数数组
 */
static int cmd_motor_dm_mit(const struct shell *sh, size_t argc, char **argv)
{
	uint8_t id;
	float position_rad;
	float velocity_rad_s;
	float kp;
	float kd;
	float torque_nm;
	int32_t duration_ms;

	if (!parse_dm_id(argv[1], &id) ||
	    !parse_float_arg(argv[2], &position_rad) ||
	    !parse_float_arg(argv[3], &velocity_rad_s) ||
	    !parse_float_arg(argv[4], &kp) ||
	    !parse_float_arg(argv[5], &kd) ||
	    !parse_float_arg(argv[6], &torque_nm) ||
	    !parse_optional_duration(argc, argv, 7, &duration_ms)) {
		shell_error(sh,
			    "usage: motor dm mit <left|right|id> <pos_rad> "
			    "<vel_rad_s> <kp> <kd> <torque_nm> [ms]");
		return -EINVAL;
	}

	enter_motor_debug_mode();
	const int ret = motor_debug_set_dm_mit(id, position_rad,
					       velocity_rad_s, kp, kd,
					       torque_nm, duration_ms);
	if (ret != 0) {
		shell_error(sh, "set DM MIT failed: %d", ret);
		return ret;
	}

	shell_print(sh,
		    "DM4340 id=%u mit pos=%.4f vel=%.3f kp=%.2f kd=%.2f "
		    "t=%.3f for %d ms",
		    id, (double)position_rad, (double)velocity_rad_s,
		    (double)kp, (double)kd, (double)torque_nm, duration_ms);
	return 0;
}

/**
 * @brief motor dm nudge — 微调关节位置
 *
 * 用法：motor dm nudge <left|right|id> <delta_rad> [kp] [kd] [ms]
 *
 * 从当前位置偏移 delta_rad 弧度（正值/负值），使用 MIT 模式的 PD 控制实现。
 * 适用于小幅调整关节位置（如校准测试），偏移量限制在 +/-0.10 弧度（约 5.7 度）。
 *
 * 参数默认值：
 *   kp = 3.0   — 位置刚度
 *   kd = 0.20  — 阻尼
 *   ms = 800   — 持续时间
 *
 * 会先读取当前关节位置，然后以当前位置 + delta 作为目标。
 *
 * 注意：此命令会自动进入电机调试模式（禁用平衡控制器）。
 *
 * @param sh   Shell 实例
 * @param argc 参数总数
 * @param argv 参数数组
 */
static int cmd_motor_dm_nudge(const struct shell *sh, size_t argc, char **argv)
{
	uint8_t id;
	float delta_rad;
	float kp = 3.0f;
	float kd = 0.20f;
	int32_t duration_ms = 800;

	if (!parse_dm_id(argv[1], &id) ||
	    !parse_float_arg(argv[2], &delta_rad)) {
		shell_error(sh,
			    "usage: motor dm nudge <left|right|id> <delta_rad> [kp] [kd] [ms]");
		return -EINVAL;
	}
	if (argc > 3 && !parse_float_arg(argv[3], &kp)) {
		shell_error(sh, "invalid kp");
		return -EINVAL;
	}
	if (argc > 4 && !parse_float_arg(argv[4], &kd)) {
		shell_error(sh, "invalid kd");
		return -EINVAL;
	}
	if (argc > 5 && !parse_i32(argv[5], &duration_ms)) {
		shell_error(sh, "invalid duration");
		return -EINVAL;
	}

	dm4340_feedback_t fb;
	if (!motor_debug_get_dm4340(id, &fb)) {
		shell_error(sh, "DM4340 id=%u has no feedback", id);
		return -ENODATA;
	}

	delta_rad = CLAMP(delta_rad, -0.10f, 0.10f);
	enter_motor_debug_mode();
	const float target_rad = fb.position_rad + delta_rad;
	const int ret = motor_debug_set_dm_mit_raw(id, target_rad, 0.0f,
						   kp, kd, 0.0f,
						   duration_ms);
	if (ret != 0) {
		shell_error(sh, "set DM nudge failed: %d", ret);
		return ret;
	}

	shell_print(sh,
		    "DM4340 id=%u nudge from %.4f to %.4f rad delta=%.4f kp=%.2f kd=%.2f for %d ms",
		    id, (double)fb.position_rad, (double)target_rad,
		    (double)delta_rad, (double)kp, (double)kd, duration_ms);
	return 0;
}

/**
 * @brief motor dm wiggle — 关节电机正弦摆动测试
 *
 * 用法：motor dm wiggle <left|right|id> <amp_rad> [period_ms] [kp] [kd] [ms]
 *
 * 以当前位置为中心，按正弦波摆动关节。用于：
 *   - 验证关节电机是否正常工作
 *   - 检测机械间隙和松动
 *   - 校准电流-扭矩系数
 *
 * 参数默认值和范围：
 *   amp_rad   — 振幅（弧度），限制在 0.001..0.08（约 0.06..4.6 度）
 *   period_ms — 摆动周期（毫秒），限制在 500..5000
 *   kp        — 位置刚度，默认 12.0
 *   kd        — 阻尼，默认 0.50
 *   ms        — 总持续时间（毫秒），默认 APP_MOTOR_DEBUG_MAX_TIMEOUT_MS
 *
 * 注意：此命令会自动进入电机调试模式（禁用平衡控制器）。
 *
 * @param sh   Shell 实例
 * @param argc 参数总数
 * @param argv 参数数组
 */
static int cmd_motor_dm_wiggle(const struct shell *sh, size_t argc, char **argv)
{
	uint8_t id;
	float amplitude_rad;
	int32_t period_ms = 2000;
	float kp = 12.0f;
	float kd = 0.50f;
	int32_t duration_ms = APP_MOTOR_DEBUG_MAX_TIMEOUT_MS;

	if (!parse_dm_id(argv[1], &id) ||
	    !parse_float_arg(argv[2], &amplitude_rad)) {
		shell_error(sh,
			    "usage: motor dm wiggle <left|right|id> <amp_rad> [period_ms] [kp] [kd] [ms]");
		return -EINVAL;
	}
	if (argc > 3 && !parse_i32(argv[3], &period_ms)) {
		shell_error(sh, "invalid period");
		return -EINVAL;
	}
	if (argc > 4 && !parse_float_arg(argv[4], &kp)) {
		shell_error(sh, "invalid kp");
		return -EINVAL;
	}
	if (argc > 5 && !parse_float_arg(argv[5], &kd)) {
		shell_error(sh, "invalid kd");
		return -EINVAL;
	}
	if (argc > 6 && !parse_i32(argv[6], &duration_ms)) {
		shell_error(sh, "invalid duration");
		return -EINVAL;
	}

	dm4340_feedback_t fb;
	if (!motor_debug_get_dm4340(id, &fb)) {
		shell_error(sh, "DM4340 id=%u has no feedback", id);
		return -ENODATA;
	}

	amplitude_rad = app_clampf(amplitude_rad, 0.001f, 0.08f);
	period_ms = CLAMP(period_ms, 500, 5000);
	if (duration_ms <= 0) {
		duration_ms = APP_MOTOR_DEBUG_DEFAULT_TIMEOUT_MS;
	}
	duration_ms = MIN(duration_ms, APP_MOTOR_DEBUG_MAX_TIMEOUT_MS);

	enter_motor_debug_mode();
	const int ret = motor_debug_set_dm_wiggle(id, fb.position_rad,
						 amplitude_rad, period_ms, kp,
						 kd, duration_ms);
	if (ret != 0) {
		shell_error(sh, "set DM wiggle failed: %d", ret);
		return ret;
	}

	shell_print(sh,
		    "DM4340 id=%u wiggle center=%.4f amp=%.4f period=%dms kp=%.2f kd=%.2f for %d ms",
		    id, (double)fb.position_rad, (double)amplitude_rad,
		    period_ms, (double)kp, (double)kd, duration_ms);
	return 0;
}

/**
 * @brief motor dm stop — 停止指定关节电机的调试输出
 *
 * 用法：motor dm stop <left|right|id>
 *
 * 停止向指定 DM4340 发送调试指令（位置/速度/MIT 等）。
 * 是 motor dm pos/vel/mit/nudge/wiggle 的安全停止命令。
 *
 * 注意：此命令会自动进入电机调试模式（禁用平衡控制器）。
 *
 * @param sh   Shell 实例
 * @param argc 参数总数
 * @param argv 参数数组：argv[1] 为电机标识
 */
static int cmd_motor_dm_stop(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);

	uint8_t id;
	if (!parse_dm_id(argv[1], &id)) {
		shell_error(sh, "DM id must be left/right/1..15");
		return -EINVAL;
	}

	enter_motor_debug_mode();
	const int ret = motor_debug_stop_dm(id);
	shell_print(sh, "DM4340 id=%u debug stopped", id);
	return ret;
}

/**
 * @brief motor dm rxlog — 打印 DM4340 CAN 接收日志
 *
 * 调用 motor_debug_dump_dm4340_rx_log() 输出最近收到的 DM4340 CAN 帧。
 * 用于诊断 CAN 通信问题（如丢帧、ID 冲突、数据错误等）。
 *
 * @param sh   Shell 实例
 * @param argc 参数总数（仅 1）
 * @param argv 参数数组
 */
static int cmd_motor_dm_rxlog(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(sh);
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	motor_debug_dump_dm4340_rx_log();
	return 0;
}

/**
 * @brief motor debug status — 显示当前所有手动调试指令状态
 *
 * 显示电机调试模块的当前输出状态：
 *   - 轮毂电机是否活跃，以及左右轮的电流指令值
 *   - 每个活跃的 DM4340 的控制模式、目标位置/速度、Kp/Kd/扭矩
 *
 * 如果没有活跃的手动调试指令，输出 "motor debug inactive"。
 *
 * @param sh   Shell 实例
 * @param argc 参数总数（仅 1）
 * @param argv 参数数组
 */
static int cmd_motor_debug_status(const struct shell *sh, size_t argc,
				  char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	motor_debug_output_t out;

	if (!motor_debug_get_output(&out)) {
		shell_print(sh, "motor debug inactive");
		return 0;
	}

	shell_print(sh, "motor debug active wheel=%d current=(%d,%d)",
		    out.wheel_active,
		    out.wheel_current[APP_WHEEL_LEFT_ID],
		    out.wheel_current[APP_WHEEL_RIGHT_ID]);
	for (uint8_t id = 1; id <= DM4340_MAX_ID; id++) {
		const motor_debug_dm_command_t *cmd = &out.dm[id];
		if (cmd->mode == MOTOR_DEBUG_DM_NONE) {
			continue;
		}
		shell_print(sh,
			    "dm id=%u mode=%s pos=%.4f vel=%.3f "
			    "kp=%.2f kd=%.2f t=%.3f",
			    id, motor_debug_dm_mode_name(cmd->mode),
			    (double)cmd->position_rad,
			    (double)cmd->velocity_rad_s, (double)cmd->kp,
			    (double)cmd->kd, (double)cmd->torque_nm);
	}

	return 0;
}

/**
 * @brief motor debug stop — 停止所有手动电机调试指令
 *
 * 一次性停止所有轮毂电机电流和所有 DM4340 关节电机的调试指令。
 * 是最全面的电机停止命令，用于紧急情况或调试结束时恢复安全状态。
 *
 * 注意：此命令会自动进入电机调试模式（禁用平衡控制器）。
 *
 * @param sh   Shell 实例
 * @param argc 参数总数（仅 1）
 * @param argv 参数数组
 */
static int cmd_motor_debug_stop(const struct shell *sh, size_t argc,
				char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	enter_motor_debug_mode();
	const int ret = motor_debug_stop_all();
	shell_print(sh, "all manual motor debug stopped");
	return ret;
}

/* ------------------------------------------------------------------ */
/* robot param — 运行时平衡参数调优（支持 NVS Flash 持久化）           */
/* ------------------------------------------------------------------ */

/**
 * @brief 可调参数查找表
 *
 * 此表定义了所有可通过 "robot param <name> <value>" 命令运行时调整的
 * 平衡控制器参数。每个条目包含：
 *   name   — 命令行中使用的参数名称
 *   offset — 参数在 ascento_balance_params_t 结构体中的字节偏移
 *   type   — 数据类型（PF_FLOAT 浮点 / PF_I16 16位整数）
 *   desc   — 参数的中文说明
 *
 * 通过 robot param save 可将当前参数持久化到 NVS Flash，
 * 下次启动时自动加载。robot param reset 恢复出厂默认值。
 */
static const struct {
	const char *name;   /* 参数名称（命令行中使用） */
	size_t offset;      /* 在 ascento_balance_params_t 中的字节偏移 */
	enum { PF_FLOAT, PF_I16 } type;  /* 数据类型：浮点 或 16位整数 */
	const char *desc;   /* 参数说明 */
	} param_table[] = {
	{ "theta_eq",        offsetof(ascento_balance_params_t, theta_eq_rad),          PF_FLOAT,
	  "平衡零点俯仰角（弧度），即机器人站立时的目标 pitch 角" },
	{ "k_pitch",        offsetof(ascento_balance_params_t, k_pitch),              PF_FLOAT,
	  "俯仰角增益：对俯仰角偏差的比例控制强度" },
	{ "k_pitch_rate",   offsetof(ascento_balance_params_t, k_pitch_rate),         PF_FLOAT,
	  "俯仰角速率增益：对俯仰角变化率的阻尼强度（类似微分项）" },
	{ "k_position",     offsetof(ascento_balance_params_t, k_position),           PF_FLOAT,
	  "位置增益：防止机器人水平漂移的位置保持强度" },
	{ "k_velocity",     offsetof(ascento_balance_params_t, k_velocity),           PF_FLOAT,
	  "速度增益：跟踪目标速度的速度控制强度" },
	{ "k_yaw_rate",      offsetof(ascento_balance_params_t, k_yaw_rate),           PF_FLOAT,
	  "偏航角速率增益：对转向角速率的控制强度" },
	{ "stiction_ma",     offsetof(ascento_balance_params_t, stiction_current_ma),  PF_FLOAT,
	  "静摩擦补偿电流（mA）：克服轮毂电机静摩擦所需的偏置电流" },
	{ "stiction_start",  offsetof(ascento_balance_params_t, stiction_start_deg),   PF_FLOAT,
	  "静摩擦补偿起始角度（度）：偏差超过此值时开始施加补偿" },
	{ "stiction_full",   offsetof(ascento_balance_params_t, stiction_full_deg),    PF_FLOAT,
	  "静摩擦补偿满量程角度（度）：偏差超过此值时施加满量程补偿" },
	{ "current_limit",   offsetof(ascento_balance_params_t, current_limit_ma),     PF_I16,
	  "轮毂电机电流限制（mA）：保护电机和电池的电流上限" },
	{ "current_scale",   offsetof(ascento_balance_params_t, current_scale),        PF_FLOAT,
	  "轮毂电机电流缩放系数：对计算出的电流指令的全局缩放" },
	{ "sync_gain",       offsetof(ascento_balance_params_t, wheel_sync_gain_ma),   PF_FLOAT,
	  "轮毂同步增益（mA/(rad/s)）：补偿左右轮速度差异的电流" },
	{ "sync_limit",      offsetof(ascento_balance_params_t, wheel_sync_current_limit_ma), PF_FLOAT,
	  "轮毂同步电流限制（mA）：同步补偿电流的最大值" },
	{ "fault_deg",       offsetof(ascento_balance_params_t, fault_deg),            PF_FLOAT,
	  "俯仰角故障阈值（度）：超过此角度触发故障保护（软限位）" },
	{ "recover_deg",     offsetof(ascento_balance_params_t, recover_deg),          PF_FLOAT,
	  "俯仰角恢复阈值（度）：故障后角度回到此范围内可自动恢复" },
};
#define PARAM_COUNT ARRAY_SIZE(param_table)

/**
 * @brief 打印所有平衡控制器参数的当前值
 *
 * 以人类可读格式显示 ascento_balance_params_t 中的所有参数，
 * 包含单位转换（弧度→度、mA 等）。
 *
 * @param sh Shell 实例
 * @param p  参数结构体指针
 */
static void print_all_params(const struct shell *sh,
			     const ascento_balance_params_t *p)
{
	shell_print(sh, "theta_eq       = %.4f rad (%.1f deg)",
		    (double)p->theta_eq_rad,
		    (double)(p->theta_eq_rad / 0.017453292519943295f));
	shell_print(sh, "k_pitch        = %.4f", (double)p->k_pitch);
	shell_print(sh, "k_pitch_rate   = %.4f", (double)p->k_pitch_rate);
	shell_print(sh, "k_position     = %.4f", (double)p->k_position);
	shell_print(sh, "k_velocity     = %.4f", (double)p->k_velocity);
	shell_print(sh, "k_yaw_rate     = %.4f", (double)p->k_yaw_rate);
	shell_print(sh, "stiction_ma    = %.0f mA", (double)p->stiction_current_ma);
	shell_print(sh, "stiction_start = %.2f deg", (double)p->stiction_start_deg);
	shell_print(sh, "stiction_full  = %.2f deg", (double)p->stiction_full_deg);
	shell_print(sh, "current_limit  = %d mA", p->current_limit_ma);
	shell_print(sh, "current_scale  = %.2f", (double)p->current_scale);
	shell_print(sh, "sync_gain      = %.1f mA/(rad/s)", (double)p->wheel_sync_gain_ma);
	shell_print(sh, "sync_limit     = %.0f mA", (double)p->wheel_sync_current_limit_ma);
	shell_print(sh, "torque_k       = %.6f / %.6f Nm/mA",
		    (double)p->left_current_ma_to_wheel_torque_nm,
		    (double)p->right_current_ma_to_wheel_torque_nm);
	shell_print(sh, "fault_deg      = %.1f deg (hard +%.1f/-%.1f)",
		    (double)p->fault_deg,
		    (double)APP_ASCENTO_FORWARD_HARD_FAULT_DEG,
		    (double)APP_ASCENTO_BACKWARD_HARD_FAULT_DEG);
	shell_print(sh, "recover_deg    = %.1f deg (stand err %.1f)",
		    (double)p->recover_deg,
		    (double)APP_ASCENTO_STAND_RECOVER_ERR_DEG);
}

/**
 * @brief robot param — 运行时参数调优命令
 *
 * 用法：
 *   robot param              — 显示所有参数的当前值
 *   robot param list         — 列出所有可调参数的名称和说明
 *   robot param save         — 将当前参数保存到 NVS Flash（掉电不丢失）
 *   robot param reset        — 恢复出厂默认值并清除 Flash 中的保存值
 *   robot param <name> <value> — 设置指定参数的值（立即生效）
 *
 * 参数修改立即生效（通过 ascento_balance_set_params 写入运行时参数结构体），
 * 但不会自动持久化。需要显式执行 "robot param save" 才会写入 Flash。
 *
 * 可调参数列表参见 param_table 定义，或通过 "robot param list" 查看。
 *
 * @param sh   Shell 实例
 * @param argc 参数总数
 * @param argv 参数数组
 * @return 0 成功，负值表示错误
 */
static int cmd_robot_param(const struct shell *sh, size_t argc, char **argv)
{
	ascento_balance_params_t params;
	ascento_balance_get_params(&params);

	/* robot param — 无参数时显示所有参数的当前值 */
	if (argc == 1) {
		print_all_params(sh, &params);
		return 0;
	}

	/* robot param list — 列出所有可调参数的名称和说明 */
	if (strcmp(argv[1], "list") == 0) {
		for (size_t i = 0; i < PARAM_COUNT; i++) {
			shell_print(sh, "  %-16s  %s", param_table[i].name,
				    param_table[i].desc);
		}
		return 0;
	}

	/* robot param save — 将当前参数持久化到 NVS Flash */
	if (strcmp(argv[1], "save") == 0) {
		int rc = ascento_balance_save_params();
		if (rc == 0) {
			shell_print(sh, "params saved to flash");
		} else {
			shell_error(sh, "save failed: %d", rc);
		}
		return rc;
	}

	/* robot param reset — 恢复出厂默认值并清除 Flash 中的保存值 */
	if (strcmp(argv[1], "reset") == 0) {
		ascento_balance_reset_params();
		ascento_balance_params_t p;
		ascento_balance_get_params(&p);
		shell_print(sh, "params reset to defaults");
		print_all_params(sh, &p);
		return 0;
	}

	/* robot param <name> <value> — 设置指定参数的值 */
	if (argc < 3) {
		shell_error(sh, "usage: robot param <name> <value>");
		return -EINVAL;
	}

	for (size_t i = 0; i < PARAM_COUNT; i++) {
		if (strcmp(argv[1], param_table[i].name) != 0) {
			continue;
		}

		float fval;
		if (!parse_float_arg(argv[2], &fval)) {
			shell_error(sh, "invalid value: %s", argv[2]);
			return -EINVAL;
		}

		uint8_t *base = (uint8_t *)&params;
		if (param_table[i].type == PF_FLOAT) {
			*(float *)(base + param_table[i].offset) = fval;
		} else {
			*(int16_t *)(base + param_table[i].offset) =
				(int16_t)fval;
		}

		ascento_balance_set_params(&params);
		shell_print(sh, "%s = %s", param_table[i].name, argv[2]);
		return 0;
	}

	shell_error(sh, "unknown param: %s (use 'robot param list')",
		    argv[1]);
	return -EINVAL;
}

/**
 * @brief "robot" 命令子集定义 — 机器人级控制命令
 *
 * 注册所有 robot 子命令到 Zephyr Shell 框架。
 * SHELL_CMD_ARG 参数说明：(名称, 子子命令集, 帮助文本, 处理函数, 必选参数数, 可选参数数)
 *
 * 命令树：
 *   robot
 *   ├── enable   <0|1|on|off>        — 启用/禁用平衡控制器
 *   ├── stop                         — 停止运动
 *   ├── status                       — 显示完整状态
 *   └── param    [list|save|reset|name value] — 运行时参数调优
 */
SHELL_STATIC_SUBCMD_SET_CREATE(
	robot_cmds,
	SHELL_CMD_ARG(enable, NULL, "robot enable <0|1|on|off>",
		      cmd_robot_enable, 2, 0),
	SHELL_CMD_ARG(stop, NULL, "robot stop", cmd_robot_stop, 1, 0),
	SHELL_CMD_ARG(status, NULL, "robot status", cmd_robot_status, 1, 0),
	SHELL_CMD_ARG(param, NULL,
		      "robot param [list | save | reset | <name> <value>]",
		      cmd_robot_param, 1, 2),
	SHELL_SUBCMD_SET_END);

/* 注册 "robot" 为顶层 Shell 命令 */
SHELL_CMD_REGISTER(robot, &robot_cmds, "wheel-leg robot control", NULL);

/**
 * @brief "motor wheel" 子命令集 — VESC/M3508 轮毂电机调试命令
 *
 * 命令树：
 *   motor wheel
 *   ├── status  [left|right|id|all]  — 显示轮毂电机反馈状态
 *   ├── current <id> <mA> [ms]      — 设置电流指令
 *   ├── rpm     <id> <erpm> [ms]    — 设置目标转速
 *   ├── pair    <L_mA> <R_mA> [ms]  — 同时设置左右轮电流
 *   └── stop                        — 停止电流输出
 */
SHELL_STATIC_SUBCMD_SET_CREATE(
	motor_wheel_cmds,
	SHELL_CMD_ARG(status, NULL, "motor wheel status [left|right|1..255|all]",
		      cmd_motor_wheel_status, 1, 1),
	SHELL_CMD_ARG(current, NULL,
		      "motor wheel current <left|right|id> <current_mA> [ms]",
		      cmd_motor_wheel_current, 3, 1),
	SHELL_CMD_ARG(rpm, NULL,
		      "motor wheel rpm <left|right|100|101> <target_erpm> [ms]",
		      cmd_motor_wheel_rpm, 3, 1),
	SHELL_CMD_ARG(pair, NULL,
		      "motor wheel pair <left_current_mA> <right_current_mA> [ms]",
		      cmd_motor_wheel_pair, 3, 1),
	SHELL_CMD_ARG(stop, NULL, "motor wheel stop", cmd_motor_wheel_stop,
		      1, 0),
	SHELL_SUBCMD_SET_END);

/**
 * @brief "motor dm" 子命令集 — DM4340 关节电机调试命令
 *
 * 命令树：
 *   motor dm
 *   ├── status  [left|right|id|all]           — 显示关节电机反馈状态
 *   ├── enable  <id>                          — 使能关节电机（进入伺服模式）
 *   ├── disable <id>                          — 失能关节电机（自由转动）
 *   ├── zero    <id>                          — 保存当前位置为零点
 *   ├── reg     <id> <rid>                    — 读取 DM4340 寄存器
 *   ├── diag    <id>                          — 一键读取所有诊断寄存器
 *   ├── pos     <id> <rad> [vel] [ms]        — 位置-速度模式控制
 *   ├── vel     <id> <rad_s> [ms]             — 速度模式控制
 *   ├── mit     <id> <pos> <vel> <kp> <kd> <t> [ms] — MIT 模式直接控制
 *   ├── nudge   <id> <delta> [kp] [kd] [ms]  — 微调位置
 *   ├── wiggle  <id> <amp> [period] [kp] [kd] [ms] — 正弦摆动测试
 *   ├── stop    <id>                          — 停止指定电机调试输出
 *   └── rxlog                                 — 打印 CAN 接收日志
 */
SHELL_STATIC_SUBCMD_SET_CREATE(
	motor_dm_cmds,
	SHELL_CMD_ARG(status, NULL, "motor dm status [left|right|id|all]",
		      cmd_motor_dm_status, 1, 1),
	SHELL_CMD_ARG(enable, NULL, "motor dm enable <left|right|id>",
		      cmd_motor_dm_enable, 2, 0),
	SHELL_CMD_ARG(disable, NULL, "motor dm disable <left|right|id>",
		      cmd_motor_dm_disable, 2, 0),
	SHELL_CMD_ARG(zero, NULL, "motor dm zero <left|right|id>",
		      cmd_motor_dm_zero, 2, 0),
	SHELL_CMD_ARG(reg, NULL, "motor dm reg <left|right|id> <rid>",
		      cmd_motor_dm_reg, 3, 0),
		SHELL_CMD_ARG(diag, NULL, "motor dm diag <left|right|id>", cmd_motor_dm_diag, 2, 0),
	SHELL_CMD_ARG(pos, NULL,
		      "motor dm pos <left|right|id> <rad> [vel_rad_s] [ms]",
		      cmd_motor_dm_pos, 3, 2),
	SHELL_CMD_ARG(vel, NULL, "motor dm vel <left|right|id> <rad_s> [ms]",
		      cmd_motor_dm_vel, 3, 1),
	SHELL_CMD_ARG(mit, NULL,
		      "motor dm mit <left|right|id> <pos_rad> <vel_rad_s> <kp> <kd> <torque_nm> [ms]",
		      cmd_motor_dm_mit, 7, 1),
	SHELL_CMD_ARG(nudge, NULL,
		      "motor dm nudge <left|right|id> <delta_rad> [kp] [kd] [ms]",
		      cmd_motor_dm_nudge, 3, 3),
	SHELL_CMD_ARG(wiggle, NULL,
		      "motor dm wiggle <left|right|id> <amp_rad> [period_ms] [kp] [kd] [ms]",
		      cmd_motor_dm_wiggle, 3, 4),
	SHELL_CMD_ARG(stop, NULL, "motor dm stop <left|right|id>",
		      cmd_motor_dm_stop, 2, 0),
	SHELL_CMD(rxlog, NULL, "motor dm rxlog", cmd_motor_dm_rxlog),
	SHELL_SUBCMD_SET_END);

/**
 * @brief "motor debug" 子命令集 — 手动电机调试状态管理
 *
 * 命令树：
 *   motor debug
 *   ├── status  — 显示当前所有手动调试指令状态
 *   └── stop    — 停止所有手动调试指令（最全面的停止命令）
 */
SHELL_STATIC_SUBCMD_SET_CREATE(
	motor_debug_cmds,
	SHELL_CMD_ARG(status, NULL, "motor debug status",
		      cmd_motor_debug_status, 1, 0),
	SHELL_CMD_ARG(stop, NULL, "motor debug stop", cmd_motor_debug_stop,
		      1, 0),
	SHELL_SUBCMD_SET_END);

/**
 * @brief "motor can" 子命令集 — CAN 总线调试命令
 *
 * 命令树：
 *   motor can
 *   ├── status  [bus|all]                     — 显示 CAN 总线状态和错误计数
 *   ├── raw     <bus> <std_id> [bytes...]     — 发送标准帧原始 CAN 报文
 *   ├── rawx    <bus> <ext_id> [bytes...]     — 发送扩展帧原始 CAN 报文
 *   └── recover [bus|all]                     — 重置 CAN 控制器（从 bus-off 恢复）
 */
SHELL_STATIC_SUBCMD_SET_CREATE(
	motor_can_cmds,
	SHELL_CMD_ARG(status, NULL,
		      "motor can status [joint|dm|can1|wheel|m3508|can2|all]",
		      cmd_motor_can_status, 1, 1),
	SHELL_CMD_ARG(raw, NULL,
		      "motor can raw <joint|wheel|can1|can2> <std_id> [byte0..byte7]",
		      cmd_motor_can_raw, 3, 8),
	SHELL_CMD_ARG(rawx, NULL,
		      "motor can rawx <joint|wheel|can1|can2> <ext_id> [byte0..byte7]",
		      cmd_motor_can_rawx, 3, 8),
	SHELL_CMD_ARG(recover, NULL,
		      "motor can recover [joint|dm|can1|wheel|m3508|can2|all]",
		      cmd_motor_can_recover, 1, 1),
	SHELL_SUBCMD_SET_END);

/**
 * @brief "motor" 顶层命令子集定义 — 电机调试命令总入口
 *
 * 命令树：
 *   motor
 *   ├── wheel  — VESC/M3508 轮毂电机调试（电流/转速/状态）
 *   ├── dm     — DM4340 关节电机调试（位置/速度/MIT/寄存器）
 *   ├── debug  — 手动调试状态管理（查看/停止所有调试指令）
 *   └── can    — CAN 总线调试（状态/原始帧/恢复）
 */
SHELL_STATIC_SUBCMD_SET_CREATE(
	motor_cmds,
	SHELL_CMD(wheel, &motor_wheel_cmds, "VESC/M3508 wheel motor debug", NULL),
	SHELL_CMD(dm, &motor_dm_cmds, "DM4340 joint motor debug", NULL),
	SHELL_CMD(debug, &motor_debug_cmds, "manual motor debug state", NULL),
	SHELL_CMD(can, &motor_can_cmds, "CAN bus debug", NULL),
	SHELL_SUBCMD_SET_END);

/* 注册 "motor" 为顶层 Shell 命令 */
SHELL_CMD_REGISTER(motor, &motor_cmds, "single motor debug", NULL);
