#ifndef __FOC_SETTING_H
#define __FOC_SETTING_H

#include "foc_control.h"

/**
 * @brief 初始化速度环PI控制器参数
 * @param kp 比例增益
 * @param ki 积分增益
 * @param ki_limit 积分限幅值
 */
void foc_speed_pi_init(float kp, float ki, float ki_limit);

/**
 * @brief 电流环PID控制器初始化
 * @param kp_d D轴比例系数
 * @param kp_q Q轴比例系数
 * @param ki 积分系数
 * @param ki_limit 积分限幅值
 */
void foc_current_pi_init(float kp_d, float kp_q, float ki, float ki_limit);

/**
 * @brief 初始化位置环PI控制器
 * @param kp 比例增益
 * @param ki 积分增益
 * @param kd 微分增益
 * @param integral_limit 积分限幅值
 */
void foc_position_pi_init(float kp, float ki, float kd, float integral_limit);

#endif /* __FOC_SETTING_H */
