/**
 * @file mc_position.h
 * @brief FOC 位置电流单环、三环级联及 S 曲线规划。
 */

#ifndef MC_POSITION_H_
#define MC_POSITION_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 使用默认限幅初始化位置环
 * @param initial_position_rad 初始机械位置（rad）
 */
void mc_position_control_init(float initial_position_rad);

/**
 * @brief 使用指定轨迹限制复位位置环
 * @param initial_position_rad 初始机械位置（rad）
 * @param maximum_velocity_rad_s 最大规划速度（rad/s）
 * @param maximum_acceleration_rad_s2 最大规划加速度（rad/s^2）
 * @param maximum_jerk_rad_s3 最大规划加加速度（rad/s^3）
 */
void mc_position_control_reset(float initial_position_rad,
                               float maximum_velocity_rad_s,
                               float maximum_acceleration_rad_s2,
                               float maximum_jerk_rad_s3);

/**
 * @brief 按 FLASH 中的参数重新配置位置环输入整形器
 * @return 1 表示配置成功或已关闭整形，0 表示当前采样率无法实现
 */
uint8_t mc_position_control_apply_shaper_config(void);

/**
 * @brief 设置位置环是否旁路输入整形
 * @param bypass 非零表示旁路，用于辨识/扫频返回位置等内部运动
 */
void mc_position_control_set_shaper_bypass(uint8_t bypass);

/**
 * @brief 设置位置→电流单环是否始终启用积分
 * @param enable 非零表示不再按运动进度门控积分
 */
void mc_position_control_set_integral_always(uint8_t enable);

/**
 * @brief 执行一次位置到速度级联控制并更新规划轨迹
 * @param target_position_rad 目标机械位置（rad）
 */
void mc_position_control_run(float target_position_rad);

/**
 * @brief 执行位置环直出电流控制并更新 S 曲线轨迹
 * @param target_position_rad 目标机械位置（rad）
 * @return q 轴电流给定（A）
 * @note 比例和积分项使用位置误差，微分项使用规划速度与实际速度之差。
 */
float mc_position_control_run_current(float target_position_rad);

/**
 * @brief 跟踪规划轨迹并生成速度给定
 * @param planned_position_rad 规划位置（rad）
 * @param planned_velocity_rad_s 规划速度（rad/s）
 * @param planned_acceleration_rad_s2 规划加速度（rad/s^2）
 * @param actual_position_rad 实际机械位置（rad）
 * @return 速度环给定（rad/s）
 */
float mc_position_control_track(float planned_position_rad,
                                float planned_velocity_rad_s,
                                float planned_acceleration_rad_s2,
                                float actual_position_rad);

#ifdef __cplusplus
}
#endif

#endif /* MC_POSITION_H_ */
