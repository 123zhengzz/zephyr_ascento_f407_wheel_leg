/**
 * @file main.c
 * @brief Ascento 式双轮腿机器人主程序入口
 *
 * 本文件是基于 Zephyr RTOS 的 STM32F407 双轮腿自平衡机器人的主程序。
 * 整体架构如下：
 *
 * 【硬件平台】
 *   - MCU: STM32F407
 *   - IMU: BMI088（用于测量机体俯仰角、横滚角、偏航角及角速度）
 *   - 轮毂电机: DJI M3508（通过 VESC 驱动器连接在 CAN2 总线上）
 *   - 关节电机: DM4340（连接在 CAN1 总线上，负责腿部伸缩）
 *   - LED 指示灯: 红/绿/蓝三色 LED，用于状态指示
 *
 * 【软件架构】
 *   - 主线程（main thread）：负责 LED 状态指示和周期性日志输出（500ms 间隔）
 *   - 控制线程（control thread）：以 200Hz 频率运行，执行以下任务：
 *     1. 读取 IMU 数据（俯仰角/角速度）
 *     2. 读取轮毂电机反馈（转速/电流）
 *     3. 运行平衡控制算法（Ascento 模型 LQR 或 PID 备用方案）
 *     4. 向电机发送控制指令（轮毂电流 + 关节位置）
 *
 * 【控制模式】
 *   - Ascento 平衡控制器（LQR）：基于物理模型的线性二次调节器，为主要控制方案
 *   - PID 平衡控制器：作为备用/回退方案，通过编译宏 APP_USE_ASCENTO_BALANCE_CONTROLLER 切换
 *
 * 【CAN 总线布局】
 *   - CAN1（joint）: 连接 DM4340 关节电机，控制腿部关节角度
 *   - CAN2（wheel）: 连接 DJI M3508 轮毂电机（经 VESC），驱动车轮转动
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "app_config.h"           /* 应用配置参数（CAN 速率、电机 ID、控制频率等） */
#include "ascento_balance.h"      /* Ascento 模型 LQR 平衡控制器 */
#include "battery.h"              /* 电池电压 ADC 采集模块 */
#include "bmi088.h"               /* BMI088 IMU 驱动（加速度计 + 陀螺仪） */
#include "control.h"              /* PID 平衡控制器及控制状态管理 */
#include "dji_m3508.h"            /* DJI M3508 轮毂电机 CAN 驱动 */
#include "dm4340.h"               /* DM4340 关节电机 CAN 驱动 */
#include "motor_debug.h"          /* 电机调试接口（用于标定和测试） */

/* 注册主程序日志模块，标签为 "app"，默认日志级别为 INFO */
LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

/* ========== 设备树节点别名定义 ========== */
/* 从设备树别名获取 CAN 总线和 LED 引脚节点 */
#define WHEEL_CAN_NODE DT_ALIAS(can_wheel)   /* 轮毂电机 CAN 总线节点（CAN2，连接 VESC） */
#define JOINT_CAN_NODE DT_ALIAS(can_joint)   /* 关节电机 CAN 总线节点（CAN1，连接 DM4340） */
#define LED_RED_NODE DT_ALIAS(led0)          /* 红色 LED 引脚节点 */
#define LED_GREEN_NODE DT_ALIAS(led1)        /* 绿色 LED 引脚节点 */
#define LED_BLUE_NODE DT_ALIAS(led2)         /* 蓝色 LED 引脚节点 */

/* ========== 外设设备句柄 ========== */
/* 通过设备树宏在编译时绑定各外设的设备指针 */
static const struct device *wheel_can = DEVICE_DT_GET(WHEEL_CAN_NODE);  /* 轮毂 CAN 设备句柄 */
static const struct device *joint_can = DEVICE_DT_GET(JOINT_CAN_NODE);  /* 关节 CAN 设备句柄 */

/* LED 引脚的 GPIO 规格描述符，包含端口、引脚号和标志位 */
static const struct gpio_dt_spec red_led = GPIO_DT_SPEC_GET(LED_RED_NODE,
							    gpios);       /* 红色 LED */
static const struct gpio_dt_spec green_led = GPIO_DT_SPEC_GET(LED_GREEN_NODE,
							      gpios);     /* 绿色 LED */
static const struct gpio_dt_spec blue_led = GPIO_DT_SPEC_GET(LED_BLUE_NODE,
							     gpios);      /* 蓝色 LED */

/* ========== 电机和传感器全局对象 ========== */
static dji_m3508_bus_t dji_bus;   /* DJI M3508 轮毂电机总线管理结构体，管理左右轮电机 */
static dm4340_bus_t dm_bus;       /* DM4340 关节电机总线管理结构体，管理左右关节电机 */
static bmi088_t imu;              /* BMI088 IMU 实例，包含加速度计和陀螺仪数据 */

/* 最近一次 IMU 采样数据的缓存副本，供主循环日志输出使用 */
static bmi088_sample_t last_imu_sample;

/* 条件编译：仅在使用 Ascento LQR 控制器时编译以下代码 */
#if APP_USE_ASCENTO_BALANCE_CONTROLLER
static ascento_balance_state_t ascento_state;  /* Ascento LQR 平衡控制器的内部状态 */
#endif

/* 控制线程栈空间：预分配 4096 字节栈空间（Zephyr 内核宏） */
K_THREAD_STACK_DEFINE(control_stack, 4096);
/* 控制线程的线程控制块（TCB），用于线程管理 */
static struct k_thread control_thread_data;

/**
 * @brief 初始化并启动指定的 CAN 总线
 *
 * 执行流程：
 *   1. 检查 CAN 控制器设备是否就绪
 *   2. 设置 CAN 总线波特率（若驱动不支持则忽略错误，使用设备树默认值）
 *   3. 启动 CAN 控制器（若已在运行则忽略）
 *
 * @param can   CAN 控制器设备指针
 * @param name  CAN 总线名称，用于日志输出标识（如 "CAN joint"、"CAN wheel"）
 * @return 0 表示成功；-ENODEV 表示设备未就绪；其他负值表示启动失败
 */
static int prepare_can(const struct device *can, const char *name)
{
	/* 检查 CAN 控制器硬件是否已初始化并就绪 */
	if (!device_is_ready(can)) {
		LOG_ERR("%s is not ready", name);
		return -ENODEV;
	}

	/* 设置 CAN 总线波特率；若驱动不支持运行时设置（返回 -ENOTSUP），则忽略 */
	int ret = can_set_bitrate(can, APP_CAN_BITRATE);
	if (ret != 0 && ret != -ENOTSUP) {
		LOG_WRN("%s bitrate set returned %d, using devicetree bitrate",
			name, ret);
	}

	/* 启动 CAN 控制器；若已在运行中（返回 -EALREADY），则忽略 */
	ret = can_start(can);
	if (ret != 0 && ret != -EALREADY) {
		LOG_ERR("%s start failed: %d", name, ret);
		return ret;
	}

	LOG_INF("%s started at %u bps", name, APP_CAN_BITRATE);
	return 0;
}

/**
 * @brief 检查是否有任何 DM4340 关节电机在线
 *
 * 遍历所有可能的 DM4340 电机 ID（1 到 DM4340_MAX_ID），
 * 只要有一个电机有有效的反馈数据就返回 true。
 * 用于 LED 状态指示：判断关节电机是否已连接并通信正常。
 *
 * @return true 表示至少有一台 DM4340 电机在线；false 表示全部离线
 */
static bool dm_any_online(void)
{
	/* 遍历所有可能的 DM4340 电机 ID */
	for (uint8_t i = 1; i <= DM4340_MAX_ID; i++) {
		dm4340_feedback_t fb;
		/* 尝试获取该 ID 电机的最新反馈数据 */
		if (dm4340_get(&dm_bus, i, &fb)) {
			return true;  /* 成功获取到反馈，说明该电机在线 */
		}
	}
	return false;  /* 所有 ID 都没有反馈数据，电机全部离线 */
}

/**
 * @brief 初始化三色 LED 指示灯
 *
 * 将红、绿、蓝三个 LED 引脚配置为输出模式，初始状态为熄灭（低电平）。
 * 如果某个 LED 引脚未在设备树中配置或设备未就绪，则跳过该 LED。
 */
static void leds_init(void)
{
	/* 配置红色 LED 为输出模式，初始状态为熄灭（INACTIVE = 低电平） */
	if (gpio_is_ready_dt(&red_led)) {
		(void)gpio_pin_configure_dt(&red_led, GPIO_OUTPUT_INACTIVE);
	}
	/* 配置绿色 LED 为输出模式，初始状态为熄灭 */
	if (gpio_is_ready_dt(&green_led)) {
		(void)gpio_pin_configure_dt(&green_led, GPIO_OUTPUT_INACTIVE);
	}
	/* 配置蓝色 LED 为输出模式，初始状态为熄灭 */
	if (gpio_is_ready_dt(&blue_led)) {
		(void)gpio_pin_configure_dt(&blue_led, GPIO_OUTPUT_INACTIVE);
	}
}

/* 绿色 LED 翻转状态标志（volatile 保证在中断上下文中可见） */
static volatile bool rx_led_state;

/**
 * @brief DM4340 CAN 接收回调中调用的 LED 翻转函数
 *
 * 每次 DM4340 电机通过 CAN 总线接收到反馈数据时调用此函数，
 * 将绿色 LED 状态取反，产生闪烁效果，用于直观指示 CAN 接收活动。
 * 该函数可从 ISR（中断服务程序）上下文调用。
 */
void dm4340_rx_led_toggle(void)
{
	rx_led_state = !rx_led_state;  /* 翻转状态标志 */
	if (gpio_is_ready_dt(&green_led)) {
		(void)gpio_pin_set_dt(&green_led, rx_led_state ? 1 : 0);
	}
}

/**
 * @brief 设置指定 LED 的亮灭状态
 *
 * @param led  LED 的 GPIO 规格描述符指针
 * @param on   true 表示点亮 LED，false 表示熄灭 LED
 */
static void set_led(const struct gpio_dt_spec *led, bool on)
{
	if (gpio_is_ready_dt(led)) {
		(void)gpio_pin_set_dt(led, on ? 1 : 0);
	}
}

/**
 * @brief 根据电机安装方向校正轮毂电流符号
 *
 * 由于左右轮毂电机的物理安装方向可能相反（例如一个朝前一个朝后），
 * 需要根据配置文件中的正方向标志来翻转电流符号，确保两侧轮子
 * 在相同的电流值下产生相同方向的力矩。
 *
 * @param current  控制器输出的原始电流值（有符号 16 位整数）
 * @param sign     正方向标志，来自 APP_WHEEL_xxx_FORWARD_CURRENT_SIGN：
 *                 - 正值（>=0）：电流方向与期望一致，无需翻转
 *                 - 负值（<0）：需要翻转电流符号
 * @return 校正后的电流值
 */
static int16_t apply_wheel_forward_sign(int16_t current, int sign)
{
	/* 如果正方向标志为负，说明该电机安装方向相反，需要取反电流 */
	if (sign < 0) {
		return (int16_t)-current;
	}

	return current;  /* 方向一致，直接返回原始电流值 */
}

/**
 * @brief 向左右轮毂电机发送电流控制指令
 *
 * 将控制输出中的左右轮电流值经过方向校正后，通过 DJI M3508 协议
 * 以组电流模式发送到 CAN 总线。第 3、4 个电机槽位未使用，填 0。
 *
 * @param out  控制输出结构体指针，包含左右轮目标电流值
 * @return 0 表示发送成功；负值表示发送失败
 */
static int send_control_wheel_current(const control_output_t *out)
{
	return dji_m3508_send_group_current(
		&dji_bus,
		apply_wheel_forward_sign(out->left_wheel_current,
					 APP_WHEEL_LEFT_FORWARD_CURRENT_SIGN),   /* 校正左轮电流方向 */
		apply_wheel_forward_sign(out->right_wheel_current,
					 APP_WHEEL_RIGHT_FORWARD_CURRENT_SIGN),  /* 校正右轮电流方向 */
		0, 0);  /* 第 3、4 个电机未使用 */
}

/**
 * @brief 向左右关节电机发送位置控制指令
 *
 * 根据编译配置选择两种控制模式之一：
 *   - MIT 模式（APP_PID_BALANCE_JOINT_USE_MIT=1）：
 *     使用 MIT 控制协议，同时指定位置、速度、前馈力矩和 PD 增益（Kp/Kd）。
 *     MIT 模式下电机驱动器内部会执行 PD 位置控制。
 *   - 位置-速度模式（默认）：
 *     使用位置-速度控制协议，指定目标位置和最大运动速度限制。
 *     电机驱动器内部会以指定速度平滑地运动到目标位置。
 *
 * @param out  控制输出结构体指针，包含左右关节目标位置和速度限制
 */
static void send_control_joints(const control_output_t *out)
{
#if APP_PID_BALANCE_JOINT_USE_MIT
	/* MIT 模式：通过 PD 增益控制关节位置 */
	/* 左关节：目标位置（rad）、速度=0、Kp、Kd、前馈力矩=0 */
	(void)dm4340_send_mit(&dm_bus, APP_DM_LEFT_ID,
			      out->left_joint_position_rad, 0.0f,
			      APP_PID_BALANCE_JOINT_MIT_KP,
			      APP_PID_BALANCE_JOINT_MIT_KD, 0.0f);
	/* 右关节：同样参数 */
	(void)dm4340_send_mit(&dm_bus, APP_DM_RIGHT_ID,
			      out->right_joint_position_rad, 0.0f,
			      APP_PID_BALANCE_JOINT_MIT_KP,
			      APP_PID_BALANCE_JOINT_MIT_KD, 0.0f);
#else
	/* 位置-速度模式：指定目标位置和最大运动速度 */
	(void)dm4340_send_pos_vel(&dm_bus, APP_DM_LEFT_ID,
				  out->left_joint_position_rad,         /* 左关节目标位置（rad） */
				  out->joint_velocity_limit_rad_s);     /* 运动速度限制（rad/s） */
	(void)dm4340_send_pos_vel(&dm_bus, APP_DM_RIGHT_ID,
				  out->right_joint_position_rad,        /* 右关节目标位置（rad） */
				  out->joint_velocity_limit_rad_s);     /* 运动速度限制（rad/s） */
#endif
}

/**
 * @brief 发送电机调试指令
 *
 * 在电机调试模式下（motor_debug 激活时），此函数替代正常控制流程，
 * 直接将调试命令发送到电机。支持以下调试功能：
 *
 * 轮毂电机（DJI M3508）调试模式：
 *   - RPM 模式：直接发送目标转速
 *   - 电流模式：直接发送目标电流值
 *
 * 关节电机（DM4340）调试模式：
 *   - POS_VEL：位置-速度控制
 *   - VELOCITY：纯速度控制
 *   - MIT：MIT 力控模式（位置+速度+Kp+Kd+力矩）
 *   - WIGGLE：三角波摆动模式，用于关节电机标定测试
 *   - NONE：不发送指令
 *
 * @param debug  调试输出结构体指针，包含所有电机的调试指令
 */
static void send_debug_output(const motor_debug_output_t *debug)
{
	/* ---- 轮毂电机调试指令 ---- */
	if (debug->wheel_mode == MOTOR_DEBUG_WHEEL_RPM) {
		/* RPM 模式：直接发送目标转速给左右轮 */
		(void)dji_m3508_send_rpm(&dji_bus, APP_WHEEL_LEFT_ID,
					 debug->left_wheel_rpm);
		(void)dji_m3508_send_rpm(&dji_bus, APP_WHEEL_RIGHT_ID,
					 debug->right_wheel_rpm);
	} else {
		/* 电流模式：发送组电流指令 */
		(void)dji_m3508_send_group_current(
			&dji_bus, debug->wheel_current[APP_WHEEL_LEFT_ID],
			debug->wheel_current[APP_WHEEL_RIGHT_ID], 0, 0);
	}

	/* ---- 关节电机调试指令 ---- */
	/* 遍历所有可能的 DM4340 电机 ID，根据各自的调试模式发送指令 */
	for (uint8_t id = 1; id <= DM4340_MAX_ID; id++) {
		const motor_debug_dm_command_t *cmd = &debug->dm[id];

		switch (cmd->mode) {
		case MOTOR_DEBUG_DM_POS_VEL:
			/* 位置-速度模式：运动到指定位置，限制最大速度 */
			(void)dm4340_send_pos_vel(&dm_bus, id,
						  cmd->position_rad,
						  cmd->velocity_rad_s);
			break;
		case MOTOR_DEBUG_DM_VELOCITY:
			/* 纯速度模式：以指定角速度持续旋转 */
			(void)dm4340_send_velocity(&dm_bus, id,
						   cmd->velocity_rad_s);
			break;
		case MOTOR_DEBUG_DM_MIT:
			/* MIT 力控模式：完整的力矩控制参数（位置+速度+增益+前馈） */
			(void)dm4340_send_mit(&dm_bus, id,
					      cmd->position_rad,
					      cmd->velocity_rad_s, cmd->kp,
					      cmd->kd, cmd->torque_nm);
			break;
		case MOTOR_DEBUG_DM_WIGGLE: {
			/*
			 * 三角波摆动模式：用于关节电机标定和功能验证
			 * 生成一个周期性的三角波位置偏移，使关节在基准位置附近来回摆动
			 *
			 * 波形计算：
			 *   - phase 在 [0, 1) 范围内周期变化
			 *   - [0, 0.25)：wave 从 0 线性上升到 1（正向运动）
			 *   - [0.25, 0.75)：wave 从 1 线性下降到 -1（反向运动）
			 *   - [0.75, 1.0)：wave 从 -1 线性上升到 0（回到起点）
			 *   - 最终位置 = 基准位置 + 振幅 * wave
			 */
			const int32_t period_ms = MAX(cmd->period_ms, 1);     /* 摆动周期（ms），最小为 1 */
			const int64_t elapsed_ms = k_uptime_get() - cmd->start_ms;  /* 自启动以来的经过时间 */
			const int32_t phase_ms = (int32_t)(elapsed_ms % period_ms);  /* 当前周期内的相位时间 */
			const float phase = (float)phase_ms / (float)period_ms;     /* 归一化相位 [0, 1) */
			float wave;  /* 三角波值 [-1, 1] */

			/* 生成三角波形的三个阶段 */
			if (phase < 0.25f) {
				wave = phase * 4.0f;          /* 第一阶段：0 -> 1 */
			} else if (phase < 0.75f) {
				wave = 2.0f - phase * 4.0f;   /* 第二阶段：1 -> -1 */
			} else {
				wave = phase * 4.0f - 4.0f;   /* 第三阶段：-1 -> 0 */
			}

			/* 使用 MIT 模式发送摆动位置指令 */
			(void)dm4340_send_mit(&dm_bus, id,
					      cmd->position_rad +
						      cmd->amplitude_rad * wave,  /* 基准位置 + 振幅偏移 */
					      0.0f, cmd->kp, cmd->kd, 0.0f);     /* 速度=0，前馈力矩=0 */
			break;
		}
		case MOTOR_DEBUG_DM_NONE:
		default:
			/* 无指令或未知模式：跳过，不发送任何命令 */
			break;
		}
	}
}

/**
 * @brief 控制线程主函数 —— 以 200Hz 频率执行平衡控制循环
 *
 * 这是整个机器人控制系统的核心线程，运行优先级高于主线程（抢占式调度）。
 * 每个控制周期执行以下步骤：
 *   1. 计算自上次循环以来的时间间隔 dt（用于积分和微分计算）
 *   2. 读取 IMU 最新数据（俯仰角、角速度等）
 *   3. 读取左右轮毂电机的反馈数据（转速、电流）
 *   4. 根据控制模式运行平衡控制算法
 *   5. 将控制输出发送到电机（轮毂电流 + 关节位置）
 *   6. 轮询 DM4340 CAN 接收 FIFO（工作区：避免 ISR 相关问题）
 *   7. 精确休眠到下一个控制周期
 *
 * @param p1, p2, p3  线程参数（未使用）
 */
static void control_thread(void *p1, void *p2, void *p3)
{
	/* 标记线程参数为未使用，避免编译器警告 */
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	/* ---- 局部变量声明 ---- */
	bmi088_sample_t imu_sample = { 0 };     /* 当前周期的 IMU 采样数据 */
	dji_m3508_motor_t left_motor = { 0 };   /* 左轮毂电机反馈数据 */
	dji_m3508_motor_t right_motor = { 0 };  /* 右轮毂电机反馈数据 */
	control_output_t out;                    /* 控制输出（电流+关节位置） */
	int64_t last_us = k_cyc_to_us_floor64(k_cycle_get_64());  /* 上次循环的时间戳（微秒） */

	/* ===== 主控制循环：以固定频率持续运行 ===== */
	while (true) {
		/* ---- 步骤 1：计算时间间隔 dt ---- */
		const int64_t now_us = k_cyc_to_us_floor64(k_cycle_get_64());
		float dt_s = (float)(now_us - last_us) * 1.0e-6f;  /* 微秒转秒 */
		last_us = now_us;
		/*
		 * dt 合理性检查：
		 *   - dt <= 0：时钟回绕或异常，使用理论周期
		 *   - dt > 20ms（0.02s）：说明控制循环被长时间阻塞（如高优先级中断），
		 *     使用理论周期避免积分器累积过大误差
		 */
		if (dt_s <= 0.0f || dt_s > 0.02f) {
			dt_s = 1.0f / APP_CONTROL_HZ;
		}

		/* ---- 步骤 2：读取传感器和电机反馈 ---- */
		/* 更新 IMU 数据（加速度计 + 陀螺仪融合结果） */
		const int imu_ret = bmi088_update(&imu, &imu_sample);
		/* 获取左右轮毂电机的最新反馈（转速、电流等） */
		const bool left_present = dji_m3508_get(
			&dji_bus, APP_WHEEL_LEFT_ID, &left_motor);
		const bool right_present = dji_m3508_get(
			&dji_bus, APP_WHEEL_RIGHT_ID, &right_motor);
		/* 检查是否有外部使能请求（如遥控器或上位机指令） */
		const bool enable_request = control_get_enable_request();

		/* ---- 步骤 3：运行控制算法 ---- */
		if (imu_ret == 0) {
			/* IMU 数据有效，更新缓存供主线程日志输出使用 */
			last_imu_sample = imu_sample;
			/* 如果轮毂电机不在线，将其反馈清零（避免使用过期数据） */
			if (!left_present) {
				left_motor = (dji_m3508_motor_t){ 0 };
			}
			if (!right_present) {
				right_motor = (dji_m3508_motor_t){ 0 };
			}

#if APP_USE_ASCENTO_BALANCE_CONTROLLER
			/* ===== Ascento LQR 平衡控制器模式 ===== */

			/* 获取当前控制器参数（支持运行时动态调整） */
				ascento_balance_params_t ap_copy;
				ascento_balance_get_params(&ap_copy);
				const ascento_balance_params_t *ap = &ap_copy;

			/* 构建 Ascento 控制器输入结构体 */
			const ascento_balance_input_t ai = {
					.enable_request = enable_request,      /* 使能请求 */
					.wheel_feedback_ok =
						left_present && right_present,  /* 轮毂电机反馈是否可用 */
					.dt_s = dt_s,                        /* 控制周期（秒） */
					.target_forward_speed_mps = 0.0f,    /* 目标前进速度（目前为 0，待添加遥控） */
					.target_yaw_rate_rad_s = 0.0f,       /* 目标偏航角速度（目前为 0） */
					.target_pitch_rad = 0.0f,            /* 目标俯仰角（0 = 直立） */
					.left_joint_position_rad = 0.0f,     /* 左关节目标位置 */
					.right_joint_position_rad = 0.0f,    /* 右关节目标位置 */
					.left_joint_velocity_rad_s = 0.0f,   /* 左关节角速度 */
					.right_joint_velocity_rad_s = 0.0f,  /* 右关节角速度 */
					.imu = imu_sample,                   /* IMU 采样数据 */
					.left_wheel = left_motor,            /* 左轮反馈 */
					.right_wheel = right_motor,          /* 右轮反馈 */
				};

			/* 执行 Ascento LQR 平衡控制更新 */
				ascento_balance_output_t ao;
				ascento_balance_update(&ascento_state, ap, &ai, &ao);

			/*
			 * 判断是否启用电机输出：
			 *   - 控制器已激活（active）
			 *   - 有使能请求（enable_request）
			 *   - 无故障（!faulted）
			 *   - 两侧轮毂电机都有有效反馈
			 * 以上条件全部满足时，使用控制器输出驱动电机
			 */
				if (ao.active && enable_request && !ao.faulted &&
				    left_present && right_present) {
				/* 控制器输出有效：驱动轮毂和关节电机 */
				out.wheels_enabled = true;
				out.joints_enabled = true;
				out.left_wheel_current =
					ao.left_wheel_current;       /* 左轮目标电流 */
				out.right_wheel_current =
					ao.right_wheel_current;      /* 右轮目标电流 */
				out.left_joint_position_rad =
					ao.left_joint_position_rad;  /* 左关节目标位置 */
				out.right_joint_position_rad =
					ao.right_joint_position_rad; /* 右关节目标位置 */
				out.joint_velocity_limit_rad_s =
					ao.joint_velocity_limit_rad_s;  /* 关节运动速度限制 */
			} else {
				/*
				 * 安全模式：控制器未激活或存在故障
				 *   - 轮毂电机断电（电流为 0）
				 *   - 关节电机锁定到预设位置（保持腿部姿态）
				 */
				out.wheels_enabled = false;
				out.joints_enabled = true;
				out.left_wheel_current = 0;
				out.right_wheel_current = 0;
				out.left_joint_position_rad =
					APP_PID_BALANCE_LOCK_LEFT_JOINT_RAD;   /* 左关节锁定位置 */
				out.right_joint_position_rad =
					APP_PID_BALANCE_LOCK_RIGHT_JOINT_RAD;  /* 右关节锁定位置 */
				out.joint_velocity_limit_rad_s =
					APP_LEG_VEL_LIMIT_RAD_S;               /* 关节速度限制 */
			}
#else
			/* ===== PID 平衡控制器模式（备用方案） ===== */
			control_step(&imu_sample, &left_motor, &right_motor,
				     dt_s, &out);
#endif

#if APP_USE_ASCENTO_BALANCE_CONTROLLER
			/*
			 * 发布控制状态信息到状态结构体，供主线程读取和日志输出。
			 * 包含：IMU 姿态角、角速度、控制器输出、电机电流、电池电压等。
			 * 57.29577951308232f = 180/PI，将弧度转换为角度。
			 */
			control_publish_status(&(control_status_t) {
				.enable_request = enable_request,
				.wheels_enabled = out.wheels_enabled,
				.faulted = ao.faulted,
				.height = APP_DEFAULT_HEIGHT,
				.joy_x = 0.0f,         /* 遥控器 X 轴（未使用） */
				.joy_y = 0.0f,         /* 遥控器 Y 轴（未使用） */
				.motion = ROBOT_STOP,  /* 运动状态：停止 */
				.pitch_deg = APP_ASCENTO_IMU_PITCH_SIGN *
					     imu_sample.pitch_deg,    /* 俯仰角（度），考虑 IMU 安装方向 */
				.pitch_rate_dps = ao.pitch_rate_rad_s *
						  57.29577951308232f,  /* 俯仰角速度（度/秒） */
				.roll_deg = APP_ASCENTO_IMU_ROLL_SIGN *
					    imu_sample.roll_deg,      /* 横滚角（度） */
				.yaw_deg = imu_sample.yaw_deg,              /* 偏航角（度） */
				.distance_rad = ao.body_position_m,         /* 机体位置（米） */
				.speed_rad_s = ao.body_velocity_mps,        /* 机体速度（米/秒） */
				.lqr_output = ao.balance_torque_nm,         /* LQR 平衡力矩输出（Nm） */
				.yaw_output = ao.yaw_torque_nm,             /* 偏航控制力矩输出（Nm） */
				.left_joint_position_rad =
					out.left_joint_position_rad,
				.right_joint_position_rad =
					out.right_joint_position_rad,
				.left_wheel_current = out.left_wheel_current,
				.right_wheel_current = out.right_wheel_current,
				.jump_phase = 0,        /* 跳跃阶段（未使用） */
			});
#endif
		} else {
			/*
			 * IMU 数据读取失败的安全处理：
			 *   - 断开轮毂电机输出（避免在无姿态信息时驱动车轮）
			 *   - 关节电机锁定到预设位置，保持稳定姿态
			 */
			out = (control_output_t) {
				.wheels_enabled = false,
				.joints_enabled = true,
				.left_wheel_current = 0,
				.right_wheel_current = 0,
				.left_joint_position_rad =
					APP_PID_BALANCE_LOCK_LEFT_JOINT_RAD,
				.right_joint_position_rad =
					APP_PID_BALANCE_LOCK_RIGHT_JOINT_RAD,
				.joint_velocity_limit_rad_s =
					APP_LEG_VEL_LIMIT_RAD_S,
			};
		}

		/* ---- 步骤 4：发送控制指令到电机 ---- */

		/*
		 * 检查是否有电机调试指令待执行。
		 * 调试模式优先级高于正常控制：如果调试器激活，
		 * 跳过正常控制输出，直接发送调试指令，然后继续下一个周期。
		 */
		motor_debug_output_t debug;
		if (motor_debug_get_output(&debug)) {
				send_debug_output(&debug);           /* 发送调试电机指令 */
				send_control_joints(&out);           /* 同时发送关节控制 */
				(void)dm4340_poll_rx_fifo(&dm_bus);  /* 轮询关节电机 CAN 接收 */
				k_sleep(K_USEC(1000000U / APP_CONTROL_HZ));  /* 休眠到下一周期 */
				continue;  /* 跳过后续正常控制流程 */
			}

		/* 正常控制流程：发送轮毂电机电流指令 */
		(void)send_control_wheel_current(&out);

		/* 如果关节电机已使能，发送关节位置指令 */
		if (out.joints_enabled) {
			send_control_joints(&out);
		}

		/*
		 * 手动轮询 DM4340 CAN 接收 FIFO。
		 * 工作区（workaround）：由于 ISR 回调存在某些问题，
		 * 在主循环中直接轮询 CAN RX FIFO 来获取关节电机反馈数据。
		 */
		(void)dm4340_poll_rx_fifo(&dm_bus);

		/* 精确休眠到下一个控制周期（1000000/APP_CONTROL_HZ 微秒） */
		k_sleep(K_USEC(1000000U / APP_CONTROL_HZ));
	}
}

/**
 * @brief 主函数 —— 系统入口点
 *
 * 主函数负责整个系统的初始化和主线程的运行。
 * 初始化完成后，主线程以 500ms 间隔循环执行以下任务：
 *   1. 读取控制状态和电池电压
 *   2. 根据系统状态更新 LED 指示灯
 *   3. 输出调试日志（姿态角、电机电流、电池电压等）
 *
 * 初始化顺序：
 *   1. LED 指示灯初始化
 *   2. PID 控制器初始化
 *   3. Ascento LQR 控制器初始化（条件编译）
 *   4. CAN 总线初始化（关节 CAN + 轮毂 CAN）
 *   5. 电机驱动初始化（DJI M3508 + DM4340）
 *   6. DM4340 关节电机使能
 *   7. BMI088 IMU 初始化
 *   8. 电池 ADC 初始化
 *   9. 创建控制线程
 *
 * @return 0（实际上由于主循环永不退出，不会返回）
 */
int main(void)
{
	LOG_INF("DJI F407 Ascento wheel-leg Zephyr app boot");

	/* ---- 硬件外设初始化 ---- */
	leds_init();    /* 初始化三色 LED 指示灯 */
	control_init(); /* 初始化 PID 控制器和状态管理 */

#if APP_USE_ASCENTO_BALANCE_CONTROLLER
	/* 初始化 Ascento LQR 平衡控制器状态和运行时参数 */
	ascento_balance_init(&ascento_state);
	ascento_balance_settings_init();
#endif

	/* ---- CAN 总线初始化 ---- */
	/* 初始化关节电机 CAN 总线（CAN1，连接 DM4340） */
	if (prepare_can(joint_can, "CAN joint DM43xx") != 0) {
		LOG_ERR("joint CAN is required");
	}

	/*
	 * 如果轮毂 CAN 和关节 CAN 是不同的物理总线（双 CAN 架构），
	 * 则需要单独初始化轮毂 CAN 总线（CAN2，连接 VESC）。
	 * 如果两者共用同一 CAN 总线，则只需初始化一次。
	 */
	if (joint_can != wheel_can) {
		if (prepare_can(wheel_can, "CAN wheel VESC") != 0) {
			LOG_ERR("wheel CAN is required");
		}
	}

	/* ---- 电机驱动初始化 ---- */
	/* 轮毂电机 ID 列表：左轮和右轮 */
	const uint8_t wheel_ids[] = {
		APP_WHEEL_LEFT_ID,
		APP_WHEEL_RIGHT_ID,
	};

	/* 初始化 DJI M3508 轮毂电机驱动，注册左右轮 ID */
	(void)dji_m3508_init(&dji_bus, wheel_can, wheel_ids,
			     ARRAY_SIZE(wheel_ids));

	/* 初始化 DM4340 关节电机驱动，注册反馈 ID 映射 */
	(void)dm4340_init(&dm_bus, joint_can, APP_DM_LEFT_FEEDBACK_ID,
			  APP_DM_RIGHT_FEEDBACK_ID);

	/* 初始化电机调试模块（传入电机总线引用，用于标定和测试） */
	motor_debug_init(&dji_bus, &dm_bus);

	/* 使能左右 DM4340 关节电机（发送使能命令） */
	(void)dm4340_enable(&dm_bus, APP_DM_LEFT_ID);
	(void)dm4340_enable(&dm_bus, APP_DM_RIGHT_ID);

	/* ---- 传感器初始化 ---- */
	/* 初始化 BMI088 IMU（加速度计 + 陀螺仪） */
	if (bmi088_init(&imu) != 0) {
		LOG_ERR("BMI088 init failed; balance output will stay disabled");
	}

	/* 初始化电池电压 ADC 采集 */
	(void)battery_init();

	/* ---- 创建控制线程 ---- */
	/*
	 * 控制线程配置：
	 *   - 优先级：K_PRIO_PREEMPT(2) —— 高于主线程的默认优先级，确保实时性
	 *   - 栈大小：4096 字节
	 *   - 启动时机：K_NO_WAIT —— 创建后立即开始运行
	 */
	k_thread_create(&control_thread_data, control_stack,
			K_THREAD_STACK_SIZEOF(control_stack), control_thread,
			NULL, NULL, NULL, K_PRIO_PREEMPT(2), 0, K_NO_WAIT);
	k_thread_name_set(&control_thread_data, "control");  /* 命名线程，便于调试 */

	/* ===== 主线程循环 ===== */
	/*
	 * 主线程以 500ms 间隔运行，负责：
	 *   1. LED 状态指示（反映系统健康状态）
	 *   2. 周期性日志输出（便于调试和监控）
	 * 主线程优先级低于控制线程，不会干扰实时控制
	 */
	bool heartbeat = false;  /* 心搏标志（当前未用于 LED 控制，保留扩展用） */
	while (true) {
		heartbeat = !heartbeat;  /* 翻转心搏标志 */

		/* 读取当前控制状态（由控制线程更新） */
		control_status_t status;
		control_get_status(&status);

		/* 读取电池电压 */
		const battery_sample_t battery = battery_read();

		/*
		 * LED 状态指示逻辑：
		 *   - 红色 LED：DM4340 关节电机全部离线 OR 控制器故障 OR 轮毂未使能 -> 亮
		 *   - 绿色 LED：用于 CAN 接收活动指示（在 dm4340_rx_led_toggle 中控制）
		 *   - 蓝色 LED：电池电压高于阈值 -> 亮（表示电量充足）
		 */
		bool dm_online = dm_any_online();
		set_led(&red_led, !dm_online || status.faulted || !status.wheels_enabled);
		set_led(&blue_led, battery.valid &&
				      battery.voltage_v >
					      APP_BATTERY_LED_THRESHOLD_V);

		/*
		 * 输出调试日志，包含以下信息：
		 *   - 姿态角：pitch（俯仰）、roll（横滚）、yaw（偏航）
		 *   - 角速度：gy（俯仰角速度）、gz（偏航角速度）
		 *   - 电机电流：left/right wheel current
		 *   - 腿部高度：height
		 *   - 电池电压：batt（仅在电池读数有效时输出）
		 */
		if (battery.valid) {
			LOG_INF("pitch %.2f roll %.2f yaw %.1f gy %.1f gz %.1f current %d/%d "
				"height %d batt %.2f V",
				(double)status.pitch_deg,
				(double)status.roll_deg,
				(double)status.yaw_deg,
				(double)last_imu_sample.gy_dps,
				(double)last_imu_sample.gz_dps,
				status.left_wheel_current,
				status.right_wheel_current, status.height,
				(double)battery.voltage_v);
		} else {
			/* 电池读数无效时，不显示电压字段 */
			LOG_INF("pitch %.2f roll %.2f yaw %.1f gy %.1f gz %.1f current %d/%d "
				"height %d",
				(double)status.pitch_deg,
				(double)status.roll_deg,
				(double)status.yaw_deg,
				(double)last_imu_sample.gy_dps,
				(double)last_imu_sample.gz_dps,
				status.left_wheel_current,
				status.right_wheel_current, status.height);
		}

		/* 主线程休眠 500ms，然后进入下一个状态更新周期 */
		k_sleep(K_MSEC(500));
	}
}
