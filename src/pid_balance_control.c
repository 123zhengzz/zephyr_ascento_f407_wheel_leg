/*
 * ==========================================================================
 *  pid_balance_control.c — control.h 接口的精简实现（LQR 控制器路径）
 * ==========================================================================
 *
 *  本文件提供 control.h 定义的全部控制接口的精简实现。
 *
 *  背景：当 APP_USE_ASCENTO_BALANCE_CONTROLLER = 1 时，实际的平衡控制
 *  算法由 src/ascento_balance.c 中的 LQR 控制器完成。本文件仅保留：
 *    - 使能/禁用状态管理
 *    - 故障标志管理
 *    - 状态发布接口（供 ascento_balance 主循环写入控制状态）
 *
 *  旧版完整的 PID 级联控制循环（角度->角速度->距离->速度）已被移除，
 *  因为 LQR 控制器直接输出轮子电流指令，不再需要 PID 级联。
 *
 *  ---- 历史说明 ----
 *  此文件最初是从 control.c（旧版 PID 控制器）拆分而来（commit cf6f3dc），
 *  旧版 control.c 包含完整的 PID 级联算法、跳跃状态机、偏航级联 PID 等功能，
 *  但已被移除且不再存在于仓库中。
 *
 * ==========================================================================
 */
#include "control.h"

#include <string.h>

#include <zephyr/kernel.h>

#include "app_config.h"

/*
 * 控制器上下文结构体（精简版）
 *
 * 在 LQR 控制器路径下，仅保留以下状态：
 *   - enable_request：使能请求标志，由 control_set_enable 设置
 *   - faulted / recover_ticks：故障状态及恢复计数（当前由 ascento_balance 管理，
 *     此处保留字段以兼容接口）
 *   - status：控制状态快照，由 ascento_balance 主循环通过 control_publish_status 写入
 *   - last_cmd_ms：最后命令时间戳，用于超时检测
 *
 * 注意：旧版中的 PID 实例（angle_pid、gyro_pid 等）、摇杆输入、
 * 运动模式、跳跃状态机等字段均已移除，因为 LQR 控制器在
 * ascento_balance.c 中独立管理这些状态。
 */
typedef struct {
	struct k_mutex lock;           /* 互斥锁，保护多线程并发访问 */
	bool enable_request;           /* 使能请求标志：true = 允许电机输出 */
	int64_t last_cmd_ms;           /* 上次收到命令的时间戳（毫秒） */
	bool faulted;                  /* 故障标志（当前未使用，由 ascento_balance 管理） */
	int recover_ticks;             /* 故障恢复计数器（当前未使用） */
	control_status_t status;       /* 控制状态快照，供 shell 命令等外部查询 */
} control_ctx_t;

/** 控制器上下文的唯一静态实例 */
static control_ctx_t ctx;

/*
 * control_init — 初始化控制器上下文
 * 清零所有字段，初始化互斥锁，记录启动时间戳。
 * 在 main 函数启动时调用一次。
 */
void control_init(void)
{
	memset(&ctx, 0, sizeof(ctx));
	k_mutex_init(&ctx.lock);
	ctx.last_cmd_ms = k_uptime_get();
}

/*
 * control_set_enable — 设置电机使能状态
 *
 * enable=true 时：允许电机输出，清除故障标志和恢复计数器。
 * enable=false 时：禁止电机输出。实际的电机禁用逻辑由
 * ascento_balance 主循环根据 enable_request 和 faulted 标志决定。
 *
 * 此函数由 shell 命令（如 "ctrl enable"）或遥控命令调用。
 */
void control_set_enable(bool enable)
{
	k_mutex_lock(&ctx.lock, K_FOREVER);
	ctx.enable_request = enable;
	ctx.last_cmd_ms = k_uptime_get();
	ctx.status.enable_request = enable;
	if (enable) {
		ctx.faulted = false;
		ctx.recover_ticks = 0;
	}
	k_mutex_unlock(&ctx.lock);
}

/*
 * control_stop_motion — 停止运动（更新时间戳）
 *
 * 在精简版中仅更新最后命令时间戳，防止命令超时。
 * 旧版中还会清零摇杆输入和运动模式，但这些字段已移除。
 * 实际的运动停止逻辑由 ascento_balance 主循环处理。
 */
void control_stop_motion(void)
{
	k_mutex_lock(&ctx.lock, K_FOREVER);
	ctx.last_cmd_ms = k_uptime_get();
	k_mutex_unlock(&ctx.lock);
}

/*
 * control_get_enable_request — 查询当前使能请求状态
 * 返回 true 表示控制器已使能（允许电机输出）。
 * 线程安全：通过互斥锁保护读取。
 */
bool control_get_enable_request(void)
{
	bool enable;

	k_mutex_lock(&ctx.lock, K_FOREVER);
	enable = ctx.enable_request;
	k_mutex_unlock(&ctx.lock);

	return enable;
}

/*
 * control_get_status — 获取当前控制状态快照
 * 将内部状态结构体拷贝到调用者提供的缓冲区。
 * 状态信息包括：使能状态、故障标志、IMU 数据、轮子速度、
 * 平衡输出、关节位置等，用于 shell 命令的 "status" 显示。
 * 线程安全：通过互斥锁保护读取。
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
 * control_publish_status — 发布控制状态（由主循环调用）
 *
 * ascento_balance 主循环在每个控制周期结束时调用此函数，
 * 将最新的控制状态（包括 LQR 输出、轮子电流、关节位置等）
 * 写入共享状态结构体，供 shell 命令和其他查询接口读取。
 * 线程安全：通过互斥锁保护写入。
 */
void control_publish_status(const control_status_t *status)
{
	if (status == NULL) {
		return;
	}

	k_mutex_lock(&ctx.lock, K_FOREVER);
	ctx.status = *status;
	k_mutex_unlock(&ctx.lock);
}
