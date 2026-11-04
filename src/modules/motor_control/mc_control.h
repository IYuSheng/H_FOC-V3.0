/**
 * @file mc_control.h
 * @brief FOC 顶层控制编排接口。
 */

#ifndef MC_CONTROL_H_
#define MC_CONTROL_H_

#include <stdint.h>

#include "adc.h"
#include "board_config.h"
#include "identify_mechanical.h"
#include "mc_calibration.h"
#include "mc_config.h"
#include "mc_encoder.h"
#include "mc_frequency_sweep.h"
#include "mc_math.h"
#include "mc_runtime.h"
#include "mc_s_curve_planner.h"
#include "tim.h"
#include "usart.h"

/** @brief S 曲线默认最大速度（3600 deg/s，约 600 rpm） */
#define MC_S_CURVE_MAX_VELOCITY     62.83185307f
/** @brief S 曲线默认最大加速度（10000 deg/s^2） */
#define MC_S_CURVE_MAX_ACCELERATION 174.5329252f
/** @brief S 曲线默认最大加加速度（25000 deg/s^3） */
#define MC_S_CURVE_MAX_JERK         436.3323129f

/**
 * @brief 初始化 FOC 控制器
 */
void mc_start_init(void);

/**
 * @brief 启动指定闭环控制类型
 * @param profile 电流、速度或位置控制类型
 * @return 1 表示成功，0 表示状态不允许
 */
uint8_t mc_control_start(mc_run_profile_t profile);

/**
 * @brief 停止闭环控制并关闭功率级输出
 */
void mc_control_stop(void);

/**
 * @brief 使用默认电流幅值启动机械参数辨识
 * @return 1 表示成功，0 表示状态不允许
 */
uint8_t mc_control_start_mechanical_identification(void);

/**
 * @brief 使用指定低/高电流幅值启动机械参数辨识
 * @param low_amplitude_a 低频段电流幅值（A）
 * @param high_amplitude_a 高频段电流幅值（A）
 * @return 1 表示成功，0 表示参数或状态不允许
 */
uint8_t mc_control_start_mechanical_identification_with_amplitudes(
    float low_amplitude_a, float high_amplitude_a);

/**
 * @brief 启动频率响应扫频
 * @return 1 表示成功，0 表示状态不允许
 */
uint8_t mc_control_start_frequency_sweep(void);

/**
 * @brief 执行控制后台任务
 */
void mc_control_background_process(void);

/**
 * @brief 在电流环中断中执行电流控制
 */
void mc_current_in_control(void);

/**
 * @brief 在外环中断中执行速度、位置或维护任务
 */
void mc_control(void);

#endif /* MC_CONTROL_H_ */
