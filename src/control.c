/*
 * ==========================================================================
 *  control.c — 旧版 PID 平衡控制器（已弃用，仅供参考）
 * ==========================================================================
 *
 *  注意：此文件 不参与编译（参见 CMakeLists.txt）。
 *  所有控制接口（control_init、control_set_enable、control_step 等）
 *  已由 src/pid_balance_control.c 提供，后者是当前使用的活动模块。
 *  本文件仅作参考保留，或作为纯 PID 备用方案。
 *
 *  ---- 与新版 pid_balance_control.c / ascento_balance.c 的主要区别 ----
 *  1. 腿部高度控制：本文件支持可变腿长（通过 jump_phase 状态机
 *     实现跳跃功能），而新版将腿部锁定在固定高度。
 *  2. 跳跃支持：本文件包含完整的跳跃状态机（phase 1: 伸展,
 *     phase 2: 收缩），新版不支持跳跃。
 *  3. 偏航控制：本文件使用级联 PID（yaw_angle_pid + yaw_gyro_pid），
 *     新版仅用比例控制。
 *  4. 车轮同步：本文件无车轮同步补偿，新版增加了左右轮速度
 *     差异的 P 控制补偿。
 *  5. 静摩擦补偿：本文件无静摩擦补偿逻辑，新版在接近直立时
 *     增加偏置电流以克服 VESC 最小电流阈值。
 *  6. PID 级联结构：两者相同——角度 PID + 角速度 PID + 距离 PID
 *     + 速度 PID，四项求和作为平衡输出。
 *
 * ==========================================================================
 */
#include "control.h"

#include <math.h>
#include <string.h>

#include <zephyr/kernel.h>

#include "app_config.h"
#include "pid.h"

/*
 * 控制器上下文结构体
 * 保存所有控制器运行时状态，包括命令输入、PID 实例、内部状态等。
 * 通过静态全局变量 ctx 持有唯一实例，所有公开 API 通过互斥锁保护访问。
 */
typedef struct {
	struct k_mutex lock;           /* 互斥锁，保护多线程并发访问 */
	bool enable_request;           /* 使能请求标志：true = 允许电机输出 */
	int height;                    /* 当前目标腿部高度（单位：配置中的高度单位） */
	float joy_x;                   /* 摇杆 X 轴输入 [-100, 100]，用于偏航控制 */
	float joy_y;                   /* 摇杆 Y 轴输入 [-100, 100]，用于前后运动 */
	robot_motion_t motion;         /* 当前运动模式（前进/后退/左转/右转/停止） */
	bool jump_request;             /* 跳跃请求标志（由 control_request_jump 设置） */
	int64_t last_cmd_ms;           /* 上次收到命令的时间戳（毫秒），用于超时检测 */

	app_pid_t angle_pid;           /* 角度 PID 控制器：跟踪俯仰角误差 */
	app_pid_t gyro_pid;            /* 角速度 PID 控制器：抑制俯仰角速率（阻尼） */
	app_pid_t distance_pid;        /* 距离 PID 控制器：消除轮子位移偏差 */
	app_pid_t speed_pid;           /* 速度 PID 控制器：跟踪目标速度 */
	app_pid_t yaw_angle_pid;       /* 偏航角度 PID 控制器：跟踪目标偏航角 */
	app_pid_t yaw_gyro_pid;        /* 偏航角速度 PID 控制器：抑制偏航角速率 */

	float angle_zero_deg;          /* 角度零点偏移（度），用于校准直立角度 */
	float distance_zero_rad;       /* 距离零点（弧度），首次使能时记录当前位置 */
	float yaw_target_deg;          /* 目标偏航角（度），由摇杆输入累积更新 */
	float joy_y_lpf;               /* 摇杆 Y 轴低通滤波器状态变量 */
	bool distance_zero_valid;      /* 距离零点是否已初始化 */
	bool faulted;                  /* 故障标志：俯仰角过大时置 true */
	int recover_ticks;             /* 故障恢复计数器：在安全范围内持续的周期数 */
	int jump_phase;                /* 跳跃状态机阶段：0=空闲, 1=伸展, 2=收缩 */
	int jump_ticks;                /* 跳跃阶段内的周期计数器 */
	control_status_t status;       /* 控制状态快照，用于外部查询（如 shell 命令） */
} control_ctx_t;

static control_ctx_t ctx;

/*
 * clamp_height — 将腿部高度限制在允许范围内
 * 输入超出 [APP_HEIGHT_MIN, APP_HEIGHT_MAX] 时做饱和截断。
 */
static int clamp_height(int height)
{
	if (height < APP_HEIGHT_MIN) {
		return APP_HEIGHT_MIN;
	}
	if (height > APP_HEIGHT_MAX) {
		return APP_HEIGHT_MAX;
	}
	return height;
}

/*
 * wheel_forward_sign — 返回指定车轮"前进"方向的电流符号
 * 左右轮因安装方向不同，正电流可能对应不同转向。
 * 此函数返回 +1 或 -1，用于将电机反馈归一化到统一的前进方向。
 */
static float wheel_forward_sign(bool left_wheel)
{
	return left_wheel ? (float)APP_WHEEL_LEFT_FORWARD_CURRENT_SIGN :
			    (float)APP_WHEEL_RIGHT_FORWARD_CURRENT_SIGN;
}

/*
 * control_init — 初始化控制器
 * 清零上下文，初始化互斥锁，并使用 app_config.h 中的默认参数
 * 初始化所有 PID 控制器实例。
 * PID 级联结构（从内到外）：
 *   角度 PID (P+I+D) -> 角速度 PID (仅 P) -> 距离 PID (仅 P) -> 速度 PID (仅 P)
 *   偏航：yaw_angle_pid (仅 P) -> yaw_gyro_pid (仅 P)
 */
void control_init(void)
{
	memset(&ctx, 0, sizeof(ctx));
	k_mutex_init(&ctx.lock);

	ctx.height = APP_DEFAULT_HEIGHT;
	ctx.motion = ROBOT_STOP;
	ctx.last_cmd_ms = k_uptime_get();
	ctx.angle_zero_deg = APP_ANGLE_ZERO_DEG;

	/* 角度 PID：完整的 P+I+D，用于跟踪俯仰角误差 */
	app_pid_init(&ctx.angle_pid, APP_PID_ANGLE_P, APP_PID_ANGLE_I,
		     APP_PID_ANGLE_D, 0.0f, APP_WHEEL_CURRENT_LIMIT);
	/* 角速度 PID：仅 P 项，提供阻尼抑制角速率 */
	app_pid_init(&ctx.gyro_pid, APP_PID_GYRO_P, 0.0f, 0.0f, 0.0f,
		     APP_WHEEL_CURRENT_LIMIT);
	/* 距离 PID：仅 P 项，消除轮子累计位移偏差（位置环） */
	app_pid_init(&ctx.distance_pid, APP_PID_DISTANCE_P, 0.0f, 0.0f,
		     0.0f, APP_WHEEL_CURRENT_LIMIT);
	/* 速度 PID：仅 P 项，跟踪摇杆设定的目标速度 */
	app_pid_init(&ctx.speed_pid, APP_PID_SPEED_P, 0.0f, 0.0f, 0.0f,
		     APP_WHEEL_CURRENT_LIMIT);
	/* 偏航角度 PID：仅 P 项，跟踪目标偏航角 */
	app_pid_init(&ctx.yaw_angle_pid, APP_PID_YAW_ANGLE_P, 0.0f, 0.0f,
		     0.0f, APP_WHEEL_CURRENT_LIMIT);
	/* 偏航角速度 PID：仅 P 项，抑制偏航旋转速率 */
	app_pid_init(&ctx.yaw_gyro_pid, APP_PID_YAW_GYRO_P, 0.0f, 0.0f,
		     0.0f, APP_WHEEL_CURRENT_LIMIT);
}

/*
 * control_set_enable — 设置电机使能状态
 * enable=true 时允许电机输出；enable=false 时禁止输出并重置所有
 * PID 积分项和内部状态，防止下次使能时出现积分饱和冲击。
 */
void control_set_enable(bool enable)
{
	k_mutex_lock(&ctx.lock, K_FOREVER);
	ctx.enable_request = enable;
	ctx.last_cmd_ms = k_uptime_get();
	if (!enable) {
		/* 禁用时重置所有 PID 控制器，清零积分累积 */
		app_pid_reset(&ctx.angle_pid);
		app_pid_reset(&ctx.gyro_pid);
		app_pid_reset(&ctx.distance_pid);
		app_pid_reset(&ctx.speed_pid);
		app_pid_reset(&ctx.yaw_angle_pid);
		app_pid_reset(&ctx.yaw_gyro_pid);
		ctx.distance_zero_valid = false;
	}
	k_mutex_unlock(&ctx.lock);
}

/*
 * control_set_height — 设置腿部目标高度
 * 高度值会被 clamp_height 限制在安全范围内。
 * 此函数仅更新命令，实际控制计算在 control_step 中完成。
 */
void control_set_height(int height)
{
	k_mutex_lock(&ctx.lock, K_FOREVER);
	ctx.height = clamp_height(height);
	ctx.last_cmd_ms = k_uptime_get();
	k_mutex_unlock(&ctx.lock);
}

/*
 * control_set_joystick — 设置摇杆输入值
 * x: 左右转向 [-100, 100]，用于偏航控制
 * y: 前后运动 [-100, 100]，用于速度跟踪
 * 值被限制在 [-100, 100] 范围内。
 */
void control_set_joystick(float x, float y)
{
	k_mutex_lock(&ctx.lock, K_FOREVER);
	ctx.joy_x = app_clampf(x, -100.0f, 100.0f);
	ctx.joy_y = app_clampf(y, -100.0f, 100.0f);
	ctx.last_cmd_ms = k_uptime_get();
	k_mutex_unlock(&ctx.lock);
}

/*
 * control_set_motion — 设置运动模式
 * 支持的模式：ROBOT_FORWARD, ROBOT_BACK, ROBOT_LEFT, ROBOT_RIGHT, ROBOT_STOP
 * 若设置 ROBOT_JUMP，则触发跳跃请求并将运动模式重置为 ROBOT_STOP。
 */
void control_set_motion(robot_motion_t motion)
{
	k_mutex_lock(&ctx.lock, K_FOREVER);
	ctx.motion = motion;
	if (motion == ROBOT_JUMP) {
		ctx.jump_request = true;
		ctx.motion = ROBOT_STOP;
	}
	ctx.last_cmd_ms = k_uptime_get();
	k_mutex_unlock(&ctx.lock);
}

/*
 * control_request_jump — 请求跳跃
 * 内部调用 control_set_motion(ROBOT_JUMP)，触发跳跃状态机。
 */
void control_request_jump(void)
{
	control_set_motion(ROBOT_JUMP);
}

/*
 * control_stop_motion — 立即停止所有运动
 * 清零摇杆输入，将运动模式设为 ROBOT_STOP。
 */
void control_stop_motion(void)
{
	k_mutex_lock(&ctx.lock, K_FOREVER);
	ctx.joy_x = 0.0f;
	ctx.joy_y = 0.0f;
	ctx.motion = ROBOT_STOP;
	ctx.last_cmd_ms = k_uptime_get();
	k_mutex_unlock(&ctx.lock);
}

/*
 * control_set_angle_zero — 设置俯仰角零点偏移
 * 用于校准机器人的直立角度，范围限制在 [-15, 15] 度。
 * 修改零点会使距离零点失效，下次 control_step 会重新记录。
 */
void control_set_angle_zero(float zero_deg)
{
	k_mutex_lock(&ctx.lock, K_FOREVER);
	ctx.angle_zero_deg = app_clampf(zero_deg, -15.0f, 15.0f);
	ctx.distance_zero_valid = false;
	k_mutex_unlock(&ctx.lock);
}

/*
 * control_get_status — 获取当前控制状态快照
 * 将内部状态结构体拷贝到调用者提供的缓冲区。
 * 状态信息包括：使能状态、故障标志、IMU 数据、轮子速度、
 * 平衡输出、关节位置等，用于 shell 命令的 "status" 显示。
 */
void control_get_status(control_status_t *status)
{
	if (status == NULL) {
		return;
	}

	k_mutex_lock(&ctx.lock, K_FOREVER);
	*status = ctx.status;
	k_mutex_unlock(&ctx.lock);
}

/*
 * copy_and_age_command — 复制命令并执行超时老化
 *
 * 在 control_step 的开头调用，将共享的命令数据拷贝到局部变量，
 * 同时检查命令超时：如果距离上次命令超过 APP_CMD_TIMEOUT_MS 毫秒，
 * 则将摇杆输入清零、运动模式设为停止，防止失控。
 *
 * 跳跃请求（jump_request）在复制后立即清除，实现"请求-消费"语义。
 */
static void copy_and_age_command(bool *enable, int *height, float *joy_x,
				 float *joy_y, robot_motion_t *motion,
				 bool *jump_request)
{
	const int64_t now_ms = k_uptime_get();

	k_mutex_lock(&ctx.lock, K_FOREVER);
	*enable = ctx.enable_request;
	*height = ctx.height;
	*joy_x = ctx.joy_x;
	*joy_y = ctx.joy_y;
	*motion = ctx.motion;
	*jump_request = ctx.jump_request;
	ctx.jump_request = false;

	/* 命令超时检测：超过 APP_CMD_TIMEOUT_MS 未收到新命令则停止 */
	if (now_ms - ctx.last_cmd_ms > APP_CMD_TIMEOUT_MS) {
		*joy_x = 0.0f;
		*joy_y = 0.0f;
		*motion = ROBOT_STOP;
	}
	k_mutex_unlock(&ctx.lock);
}

/*
 * control_step — 主控制循环（每个控制周期调用一次）
 *
 * 参数：
 *   imu   - BMI088 IMU 采样数据（俯仰角、横滚角、偏航角、角速率）
 *   left  - 左轮 DJI M3508 电机反馈（角度、速度）
 *   right - 右轮 DJI M3508 电机反馈（角度、速度）
 *   dt_s  - 控制周期（秒），若无效则使用默认值 1/APP_CONTROL_HZ
 *   out   - 输出结构体（轮子电流、关节位置等）
 *
 * 算法流程：
 *   1. 复制命令并检查超时
 *   2. 处理运动模式快捷键（前进/后退/左转/右转）
 *   3. 跳跃状态机（phase 1: 伸展, phase 2: 收缩）
 *   4. 计算轮子角度和速度（归一化到输出轴）
 *   5. 距离零点管理（首次使能或摇杆有输入时重置）
 *   6. PID 级联控制：角度 + 角速度 + 距离 + 速度 -> 平衡输出
 *   7. 偏航 PID 级联控制：偏航角度 + 偏航角速度 -> 偏航输出
 *   8. 故障检测与恢复
 *   9. 计算左右轮电流和关节位置
 *  10. 更新状态快照
 */
void control_step(const bmi088_sample_t *imu, const dji_m3508_motor_t *left,
		  const dji_m3508_motor_t *right, float dt_s,
		  control_output_t *out)
{
	bool enable;
	int height;
	float joy_x;
	float joy_y;
	robot_motion_t motion;
	bool jump_request;

	/* 空指针检查 */
	if (imu == NULL || left == NULL || right == NULL || out == NULL) {
		return;
	}

	memset(out, 0, sizeof(*out));
	/* dt_s 无效时使用默认控制周期 */
	if (dt_s <= 0.0f) {
		dt_s = 1.0f / APP_CONTROL_HZ;
	}

	/* 步骤 1：复制命令数据并检查超时 */
	copy_and_age_command(&enable, &height, &joy_x, &joy_y, &motion,
			     &jump_request);

	/*
	 * 步骤 2：运动模式快捷键处理
	 * 若设置了运动模式（前进/后退/左转/右转），则用摇杆值或默认值
	 * 覆盖 joy_x/joy_y。摇杆值 > 1 时使用摇杆值，否则使用默认速度。
	 * 这允许通过 shell 命令（如 "ctrl forward"）直接控制运动方向。
	 */
	if (motion == ROBOT_FORWARD) {
		joy_y = fabsf(joy_y) > 1.0f ? fabsf(joy_y) : 45.0f;
	} else if (motion == ROBOT_BACK) {
		joy_y = fabsf(joy_y) > 1.0f ? -fabsf(joy_y) : -45.0f;
	} else if (motion == ROBOT_LEFT) {
		joy_x = fabsf(joy_x) > 1.0f ? -fabsf(joy_x) : -35.0f;
	} else if (motion == ROBOT_RIGHT) {
		joy_x = fabsf(joy_x) > 1.0f ? fabsf(joy_x) : 35.0f;
	}

	/*
	 * 步骤 3：跳跃状态机
	 *
	 * 跳跃分为两个阶段：
	 *   Phase 1（伸展）：腿部伸展到最大高度（APP_HEIGHT_MAX），
	 *     持续 30 个控制周期，为跳跃蓄力。
	 *   Phase 2（收缩）：腿部快速收缩到最小高度（40），
	 *     持续 170 个控制周期，产生向上的推力。
	 *
	 * 状态转换：空闲(0) -> 伸展(1) -> 收缩(2) -> 空闲(0)
	 */
	if (jump_request && ctx.jump_phase == 0) {
		ctx.jump_phase = 1;
		ctx.jump_ticks = 0;
	}

	int effective_height = height;
	if (ctx.jump_phase == 1) {
		/* Phase 1：伸展到最大高度 */
		effective_height = APP_HEIGHT_MAX;
		ctx.jump_ticks++;
		if (ctx.jump_ticks >= 30) {
			ctx.jump_phase = 2;
			ctx.jump_ticks = 0;
		}
	} else if (ctx.jump_phase == 2) {
		/* Phase 2：收缩到最小高度，产生跳跃推力 */
		effective_height = 40;
		ctx.jump_ticks++;
		if (ctx.jump_ticks >= 170) {
			ctx.jump_phase = 0;
			ctx.jump_ticks = 0;
		}
	}

	/*
	 * 步骤 4：计算轮子角度和速度（归一化到输出轴）
	 *
	 * M3508 电机经过减速比 APP_M3508_REDUCTION_RATIO 减速后驱动轮子。
	 * wheel_forward_sign 将左右轮反馈归一化到统一的前进方向。
	 * distance = 左右轮角度的平均值（用于位置环）
	 * speed = 左右轮速度的平均值（用于速度环）
	 */
	const float left_wheel_angle =
		wheel_forward_sign(true) * left->angle_rad /
		APP_M3508_REDUCTION_RATIO;
	const float right_wheel_angle =
		wheel_forward_sign(false) * right->angle_rad /
		APP_M3508_REDUCTION_RATIO;
	const float left_wheel_speed =
		wheel_forward_sign(true) * left->speed_rad_s /
		APP_M3508_REDUCTION_RATIO;
	const float right_wheel_speed =
		wheel_forward_sign(false) * right->speed_rad_s /
		APP_M3508_REDUCTION_RATIO;
	/* 距离：左右轮角度平均值（弧度），用于距离 PID */
	const float distance = 0.5f * (left_wheel_angle + right_wheel_angle);
	/* 速度：左右轮速度平均值（弧度/秒），用于速度 PID */
	const float speed = 0.5f * (left_wheel_speed + right_wheel_speed);

	/*
	 * 步骤 5：距离零点管理
	 *
	 * 距离零点是平衡控制器的参考原点，用于计算位置误差。
	 * - 首次使能时记录当前位置作为零点。
	 * - 当摇杆有输入（joy_y > 1）、速度过大（> 45 rad/s）或
	 *   正在跳跃时，重置零点到当前位置，防止积分饱和。
	 */
	if (!ctx.distance_zero_valid) {
		ctx.distance_zero_rad = distance;
		ctx.distance_zero_valid = true;
	}

	/* 运动中或跳跃时重置距离零点和距离 PID */
	if (fabsf(joy_y) > 1.0f || fabsf(speed) > 45.0f ||
	    ctx.jump_phase != 0) {
		ctx.distance_zero_rad = distance;
		app_pid_reset(&ctx.distance_pid);
	}

	/*
	 * 步骤 6：PID 级联控制 —— 平衡输出
	 *
	 * 级联结构（从内环到外环）：
	 *
	 *   角度误差 (pitch_deg - zero) ──> [角度 PID] ──> angle_control
	 *                                                          │
	 *   角速度 (gy_dps) ──────────────> [角速度 PID] ──> gyro_control
	 *                                                          │
	 *   位置误差 (distance - zero) ───> [距离 PID] ───> distance_control
	 *                                                          │
	 *   速度误差 (speed - target) ────> [速度 PID] ────> speed_control
	 *                                                          │
	 *   ┌──────────────────────────────────────────────────────┘
	 *   └──> lqr_u = angle + gyro + distance + speed  （求和）
	 *
	 * 角度 PID 是完整的 P+I+D，其余三个仅使用 P 项。
	 * 最终输出 lqr_u 是轮子的目标电流（mA）。
	 */

	/* 摇杆 Y 轴低通滤波，平滑输入突变 */
	const float filtered_joy_y =
		app_lpf_update(joy_y, &ctx.joy_y_lpf, 0.20f, dt_s);
	/* 目标速度 = 滤波后的摇杆值 * 速度系数 */
	const float speed_target = filtered_joy_y * APP_JOY_TO_SPEED_RAD_S;

	/* 角度 PID：输入为俯仰角误差（当前角度 - 零点偏移） */
	const float angle_error = imu->pitch_deg - ctx.angle_zero_deg;
	const float angle_control = app_pid_update(&ctx.angle_pid,
						   angle_error, dt_s);
	/* 角速度 PID：输入为俯仰角速率，提供阻尼 */
	const float gyro_control = app_pid_update(&ctx.gyro_pid,
						  imu->gy_dps, dt_s);
	/* 距离 PID：输入为轮子位移误差，消除稳态偏移 */
	const float distance_control = app_pid_update(
		&ctx.distance_pid, distance - ctx.distance_zero_rad, dt_s);
	/* 速度 PID：输入为速度误差，跟踪目标速度 */
	const float speed_control = app_pid_update(&ctx.speed_pid,
						   speed - speed_target, dt_s);

	/*
	 * 平衡输出 = 四项 PID 输出之和
	 * 负号在电流分配时应用（见下方 left_current / right_current 计算）
	 */
	float lqr_u = angle_control + gyro_control + distance_control +
		      speed_control;
	/* 限幅到最大电流限制 */
	lqr_u = app_clampf(lqr_u, -APP_WHEEL_CURRENT_LIMIT,
			   APP_WHEEL_CURRENT_LIMIT);

	/*
	 * 步骤 7：偏航 PID 级联控制
	 *
	 * 偏航控制采用级联 PID：
	 *   目标偏航角 = 摇杆 X 输入 * 偏航速率系数 * dt（累积积分）
	 *   偏航角度误差 = 目标偏航角 - 当前偏航角（包裹到 [-180, 180]）
	 *   偏航输出 = yaw_angle_pid(偏航角度误差) + yaw_gyro_pid(偏航角速率)
	 *
	 * 与平衡控制不同，偏航使用两个 PID 控制器级联，
	 * 而非直接的比例前馈。
	 */
	/* 积分摇杆 X 输入，累积为目标偏航角 */
	ctx.yaw_target_deg += joy_x * APP_JOY_TO_YAW_RATE_DEG_S * dt_s;
	ctx.yaw_target_deg = app_wrap_pm180(ctx.yaw_target_deg);
	/* 偏航角度误差，包裹到 [-180, 180] 度 */
	const float yaw_error = app_wrap_pm180(ctx.yaw_target_deg -
					       imu->yaw_deg);
	/* 偏航级联输出 = 角度 PID + 角速度 PID */
	float yaw_output = app_pid_update(&ctx.yaw_angle_pid, yaw_error,
					  dt_s) +
			   app_pid_update(&ctx.yaw_gyro_pid, imu->gz_dps,
					  dt_s);
	yaw_output = app_clampf(yaw_output, -APP_WHEEL_CURRENT_LIMIT,
				APP_WHEEL_CURRENT_LIMIT);

	/*
	 * 步骤 8：故障检测与恢复
	 *
	 * 故障检测：当俯仰角绝对值超过 APP_PITCH_FAULT_DEG 时，
	 *   判定为倾倒故障，立即置 faulted=true 并清零恢复计数器。
	 *   故障状态下所有电机输出为零（见下方 wheels_enabled 判断）。
	 *
	 * 故障恢复：需要同时满足以下条件：
	 *   1. 控制器已使能（enable=true）
	 *   2. 俯仰角误差回落到 APP_PITCH_RECOVER_DEG 以内
	 *   3. 在安全范围内持续超过 APP_BALANCE_RECOVER_TICKS 个周期
	 *
	 * 恢复时重置距离零点和所有 PID 控制器，避免积分饱和导致
	 * 再次倾倒。
	 */

	/* 故障检测：俯仰角过大 -> 进入故障状态 */
	if (fabsf(imu->pitch_deg) > APP_PITCH_FAULT_DEG) {
		ctx.faulted = true;
		ctx.recover_ticks = 0;
	}

	/* 故障恢复逻辑 */
	if (ctx.faulted) {
		if (enable && fabsf(imu->pitch_deg) < APP_PITCH_RECOVER_DEG) {
			/* 在安全范围内，累加恢复计数 */
			ctx.recover_ticks++;
			if (ctx.recover_ticks > APP_BALANCE_RECOVER_TICKS) {
				/* 恢复成功：清除故障，重置控制器 */
				ctx.faulted = false;
				ctx.recover_ticks = 0;
				ctx.distance_zero_rad = distance;
				app_pid_reset(&ctx.angle_pid);
				app_pid_reset(&ctx.gyro_pid);
				app_pid_reset(&ctx.distance_pid);
				app_pid_reset(&ctx.speed_pid);
			}
		} else {
			/* 超出安全范围，重置恢复计数 */
			ctx.recover_ticks = 0;
		}
	}

	/*
	 * 步骤 9：计算左右轮电流
	 *
	 * 使能条件：enable=true 且 无故障 且 电机反馈正常
	 *
	 * 电流分配公式：
	 *   左轮 = -0.5 * (balance_output + yaw_output)
	 *   右轮 = -0.5 * (balance_output - yaw_output)
	 *
	 * 负号是因为正的平衡输出需要产生反向力矩来纠正前倾。
	 * yaw_output 通过差速实现转向：左加右减产生偏航力矩。
	 */
	const bool motor_feedback_ok = left->initialized && right->initialized;
	const bool wheels_enabled = enable && !ctx.faulted && motor_feedback_ok;
	int32_t left_current = (int32_t)(-0.5f * (lqr_u + yaw_output));
	int32_t right_current = (int32_t)(-0.5f * (lqr_u - yaw_output));

	/* 不满足使能条件时，输出零电流 */
	if (!wheels_enabled) {
		left_current = 0;
		right_current = 0;
		lqr_u = 0.0f;
		yaw_output = 0.0f;
	}

	/*
	 * 步骤 10：计算关节位置（腿部高度和横滚补偿）
	 *
	 * 关节位置由三部分组成：
	 *   1. 基准零点（APP_LEFT_LEG_ZERO_RAD / APP_RIGHT_LEG_ZERO_RAD）
	 *   2. 高度项：(effective_height - APP_HEIGHT_MIN) * APP_LEG_RAD_PER_HEIGHT_UNIT
	 *      左腿加高度项，右腿减高度项，实现对称伸展
	 *   3. 横滚补偿：roll_deg * APP_ROLL_RAD_PER_DEG
	 *      用于在横滚方向上保持平衡，左右腿同向补偿
	 *
	 * effective_height 在跳跃状态下由状态机控制，否则使用用户设定值。
	 */
	const float height_term =
		(float)(effective_height - APP_HEIGHT_MIN) *
		APP_LEG_RAD_PER_HEIGHT_UNIT;
	/* 横滚补偿：将横滚角转换为关节角度偏移 */
	float roll_comp = imu->roll_deg * APP_ROLL_RAD_PER_DEG;
	roll_comp = app_clampf(roll_comp, -APP_ROLL_COMP_LIMIT_RAD,
				APP_ROLL_COMP_LIMIT_RAD);

	/* 左腿关节 = 零点 + 高度 - 横滚补偿（限幅到安全范围） */
	const float left_joint = app_clampf(APP_LEFT_LEG_ZERO_RAD +
						    height_term - roll_comp,
					    APP_LEFT_LEG_MIN_RAD,
					    APP_LEFT_LEG_MAX_RAD);
	/* 右腿关节 = 零点 - 高度 - 横滚补偿（对称伸展） */
	const float right_joint = app_clampf(APP_RIGHT_LEG_ZERO_RAD -
						     height_term - roll_comp,
					     APP_RIGHT_LEG_MIN_RAD,
					     APP_RIGHT_LEG_MAX_RAD);

	/*
	 * 步骤 11：填充输出结构体
	 * 将计算结果写入输出结构体，供电机驱动和关节控制器使用。
	 * 电流值被限制在 [-APP_WHEEL_CURRENT_LIMIT, APP_WHEEL_CURRENT_LIMIT] 范围内。
	 */
	out->wheels_enabled = wheels_enabled;
	out->joints_enabled = true;
	out->left_wheel_current = app_clamp_i16(
		left_current, -APP_WHEEL_CURRENT_LIMIT,
		APP_WHEEL_CURRENT_LIMIT);
	out->right_wheel_current = app_clamp_i16(
		right_current, -APP_WHEEL_CURRENT_LIMIT,
		APP_WHEEL_CURRENT_LIMIT);
	out->left_joint_position_rad = left_joint;
	out->right_joint_position_rad = right_joint;
	out->joint_velocity_limit_rad_s = APP_LEG_VEL_LIMIT_RAD_S;

	/*
	 * 步骤 12：更新状态快照
	 * 将当前控制周期的所有关键数据写入状态结构体，
	 * 供 shell 命令（如 "ctrl status"）查询显示。
	 */
	k_mutex_lock(&ctx.lock, K_FOREVER);
	ctx.status = (control_status_t) {
		.enable_request = enable,
		.wheels_enabled = wheels_enabled,
		.faulted = ctx.faulted,
		.height = height,
		.joy_x = joy_x,
		.joy_y = joy_y,
		.motion = motion,
		.pitch_deg = imu->pitch_deg,
		.roll_deg = imu->roll_deg,
		.yaw_deg = imu->yaw_deg,
		.distance_rad = distance,
		.speed_rad_s = speed,
		.lqr_output = lqr_u,
		.yaw_output = yaw_output,
		.left_joint_position_rad = left_joint,
		.right_joint_position_rad = right_joint,
		.left_wheel_current = out->left_wheel_current,
		.right_wheel_current = out->right_wheel_current,
		.jump_phase = ctx.jump_phase,
	};
	k_mutex_unlock(&ctx.lock);
}
