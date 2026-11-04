/**
 * @file mc_speed.h
 * @brief FOC 速度环 PID 控制。
 */

#ifndef MC_SPEED_H_
#define MC_SPEED_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化速度环 PID
 * @param initial_speed_rad_s 初始机械速度（rad/s）
 */
void mc_speed_control_init(float initial_speed_rad_s);

/**
 * @brief 从 FLASH 参数加载并复位速度环 PID
 * @param initial_speed_rad_s 初始机械速度（rad/s）
 */
void mc_speed_control_reset(float initial_speed_rad_s);

/**
 * @brief 执行一次速度环控制
 * @param target_speed_rad_s 目标机械速度（rad/s）
 * @param target_acceleration_rad_s2 机械加速度前馈（rad/s^2）
 * @param actual_speed_rad_s 机械速度反馈（rad/s）
 * @return q 轴电流给定（A）
 */
float mc_speed_control_run(float target_speed_rad_s,
                           float target_acceleration_rad_s2,
                           float actual_speed_rad_s);

#ifdef __cplusplus
}
#endif

#endif /* MC_SPEED_H_ */
