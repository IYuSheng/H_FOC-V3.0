/**
 * @file mc_flux_observer.h
 * @brief 两相步进电机非线性磁链观测器。
 */

#ifndef MC_FLUX_OBSERVER_H_
#define MC_FLUX_OBSERVER_H_

#include "board_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 非线性磁链观测器状态
 */
typedef struct {
    float x1;          /**< α 轴定子磁链积分状态（Wb） */
    float x2;          /**< β 轴定子磁链积分状态（Wb） */
    float cos_theta;   /**< 估计电角度的余弦值 */
    float sin_theta;   /**< 估计电角度的正弦值 */
    float omega_filt;  /**< 滤波后的电角速度（rad/s） */
    float theta_hat;   /**< 估计电角度（rad，-π~π） */
    float omega_integ; /**< PLL 积分器输出（rad/s） */
} FluxObserver_t;

#define FLUX_PHI_M            MOTOR_FLUX_LINKAGE
#define _1_FLUX_PHI_M         (1.0f / MOTOR_FLUX_LINKAGE)
#define FLUX_PHI_M_2          (MOTOR_FLUX_LINKAGE * MOTOR_FLUX_LINKAGE)
#define FLUX_GAMMA            10000000.0f
#define FLUX_GAMMA_K          0.5f * FLUX_GAMMA
#define FLUX_L_S              MOTOR_INDUCTANCE
#define FLUX_R_S              MOTOR_RESISTANCE
#define FLUX_OBSERVER_USE_PLL 1
#define PLL_BANDWIDTH_HZ      1000.0f
#define PLL_KP                (2.0f * 0.707f * 6.2831853f * PLL_BANDWIDTH_HZ)
#define PLL_KI                                                                 \
    ((6.2831853f * PLL_BANDWIDTH_HZ) * (6.2831853f * PLL_BANDWIDTH_HZ))

/**
 * @brief 磁链观测器全局实例
 */
extern FluxObserver_t g_flux_obs;

/**
 * @brief 初始化非线性磁链观测器
 * @param obs 观测器状态指针
 */
void flux_observer_init(FluxObserver_t *obs);

/**
 * @brief 更新非线性磁链观测器
 * @param obs 观测器状态指针
 * @param u_alpha α 轴电压（V）
 * @param u_beta β 轴电压（V）
 * @param i_alpha α 轴电流（A）
 * @param i_beta β 轴电流（A）
 * @param ts 采样周期（s）
 */
void step_flux_observer_update(FluxObserver_t *obs, float u_alpha, float u_beta,
                               float i_alpha, float i_beta, float ts);

#ifdef __cplusplus
}
#endif

#endif /* MC_FLUX_OBSERVER_H_ */
