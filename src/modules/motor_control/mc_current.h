/**
 * @file mc_current.h
 * @brief FOC 电流环、解耦前馈及弱磁集成。
 */

#ifndef MC_CURRENT_H_
#define MC_CURRENT_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 根据当前电机参数初始化 d/q 轴电流 PI
 */
void mc_current_control_init_from_motor(void);

/**
 * @brief 复位 d/q 轴电流 PI 状态
 */
void mc_current_control_reset(void);

/**
 * @brief 设置开环电压输出目标
 * @param angle_rad 电角度（rad）
 * @param voltage_d d 轴电压（V）
 * @param voltage_q q 轴电压（V）
 */
void mc_current_control_set_open_loop(float angle_rad, float voltage_d,
                                      float voltage_q);

/**
 * @brief 在电流环中断中输出已设置的开环电压
 */
void mc_current_control_open_loop_tick(void);

/**
 * @brief 更新电流环速度前馈系数
 * @param electrical_speed_rad_s 电角速度，单位：rad/s
 * @note 该函数应在 5 kHz 外环中调用。
 */
void mc_current_control_update_feedforward(float electrical_speed_rad_s);

/**
 * @brief 执行一次 d/q 轴电流闭环控制
 * @param target_d d 轴目标电流（A）
 * @param target_q q 轴目标电流（A）
 * @param control_running 1 表示正常闭环运行，0 表示维护模式
 */
void mc_current_control_run(float target_d, float target_q,
                            uint8_t control_running);

/**
 * @brief 仅执行电流采样和 Park 变换，不改变 PWM 输出
 */
void mc_current_control_measure(void);

/**
 * @brief 输出四相 PWM 中点，使电机进入零电压状态
 */
void mc_output_idle(void);

#ifdef __cplusplus
}
#endif

#endif /* MC_CURRENT_H_ */
