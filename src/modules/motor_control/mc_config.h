/**
 * @file mc_config.h
 * @brief 电流环 PI 参数初始化。
 */

#ifndef MC_CONFIG_H_
#define MC_CONFIG_H_

/**
 * @brief 初始化电流环 PI 控制器
 * @param kp_d d 轴比例系数
 * @param kp_q q 轴比例系数
 * @param ki 每个电流环周期的离散积分系数
 * @param ki_limit 积分限幅值
 */
void mc_current_pi_init(float kp_d, float kp_q, float ki, float ki_limit);

#endif /* MC_CONFIG_H_ */
