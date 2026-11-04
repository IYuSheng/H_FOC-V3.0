/**
 * @file mc_hfi_pll.h
 * @brief HFI 电流包络锁相环。
 */

#ifndef MC_HFI_PLL_H_
#define MC_HFI_PLL_H_

#include "board_config.h"

#ifdef __cplusplus
extern "C" {
#endif

struct HFI_Observer;

/**
 * @brief HFI 电流包络锁相环状态
 */
typedef struct {
    float theta_est;      /**< 估计电角度（rad） */
    float omega_est;      /**< 估计电角速度（rad/s） */
    float omega_est_filt; /**< 滤波后的电角速度（rad/s） */
    float kp;             /**< 比例增益 */
    float ki;             /**< 积分增益 */
    float error;          /**< 角度误差 */
    float p_term;         /**< 比例项 */
    float i_term;         /**< 积分项 */
} HFI_PLL_t;

#define HFI_PLL_KP 20000.0f
#define HFI_PLL_KI (2048000.0f * PWM_PERIOD_S)

/**
 * @brief HFI-PLL 全局实例
 */
extern HFI_PLL_t g_hfi_pll;

/**
 * @brief 初始化 HFI-PLL
 * @param pll PLL 状态指针
 * @param kp 比例增益
 * @param ki 积分增益
 */
void hfi_pll_init(HFI_PLL_t *pll, float kp, float ki);

/**
 * @brief 使用解调电流包络更新 HFI-PLL
 * @param pll PLL 状态指针
 * @param obs HFI 观测器状态指针
 * @param ts 更新周期（s）
 */
void hfi_pll_update(HFI_PLL_t *pll, struct HFI_Observer *obs, float ts);

#ifdef __cplusplus
}
#endif

#endif /* MC_HFI_PLL_H_ */
