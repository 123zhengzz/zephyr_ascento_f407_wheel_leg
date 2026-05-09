#pragma once
/**
 * @file control_loop.h
 * @brief 200Hz 平衡控制线程接口
 *
 * 封装实时平衡控制线程的创建和启动。
 * 控制线程以 200Hz 固定频率运行，执行 IMU 读取、状态估计、
 * 平衡控制算法（Ascento LQR 或 PID）以及电机指令下发。
 */

#include "bmi088.h"
#include "dji_m3508.h"
#include "dm4340.h"

/**
 * @brief 最近一次 IMU 采样数据的缓存副本
 *
 * 由控制线程写入，主线程读取（用于周期性日志输出）。
 * 由于只有单个写入者和读取者，且数据可容忍一个采样周期的延迟，
 * 因此无需加锁。
 */
extern bmi088_sample_t last_imu_sample;

/**
 * @brief 启动 200Hz 平衡控制线程
 *
 * 内部完成以下操作：
 *   1. 初始化 PID 控制器状态
 *   2. 初始化 Ascento LQR 控制器（条件编译）
 *   3. 加载运行时参数（NVS）
 *   4. 创建高优先级控制线程（K_PRIO_PREEMPT(2)）
 *
 * @param dji_bus  DJI M3508 轮毂电机总线指针
 * @param dm_bus   DM4340 关节电机总线指针
 * @param imu      BMI088 IMU 实例指针
 */
void control_loop_start(dji_m3508_bus_t *dji_bus, dm4340_bus_t *dm_bus,
			bmi088_t *imu);
