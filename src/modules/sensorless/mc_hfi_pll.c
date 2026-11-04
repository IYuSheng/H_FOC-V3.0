#include "mc_hfi_pll.h"

#include "cordic.h"
#include "mc_filter.h"
#include "mc_hfi_observer.h"
#include "mc_math.h"

#define HFI_PLL_INTEGRAL_LIMIT 1000.0f

HFI_PLL_t g_hfi_pll;

/**
 * @brief 初始化 HFI-PLL
 * @param pll PLL 状态指针
 * @param kp 比例增益
 * @param ki 积分增益
 */
void hfi_pll_init(HFI_PLL_t *pll, float kp, float ki)
{
    if (pll == NULL) {
        return;
    }

    pll->kp = kp;
    pll->ki = ki;
    pll->theta_est = 0.0f;
    pll->omega_est = 0.0f;
    pll->omega_est_filt = 0.0f;
    pll->p_term = 0.0f;
    pll->i_term = 0.0f;
    pll->error = 0.0f;
}

/**
 * @brief 使用解调电流包络更新 HFI-PLL
 * @param pll PLL 状态指针
 * @param obs HFI 观测器状态指针
 * @param ts 更新周期（s）
 */
void hfi_pll_update(HFI_PLL_t *pll, HFI_Observer_t *obs, float ts)
{
    static ButterworthLPF_t omega_filter;
    static uint8_t filter_initialized = 0U;
    float sin_theta;
    float cos_theta;

    if ((pll == NULL) || (obs == NULL) || (ts <= 0.0f)) {
        return;
    }
    if (filter_initialized == 0U) {
        butterworth_init(&omega_filter, 10.0f, PWM_FREQ);
        filter_initialized = 1U;
    }

    pll->theta_est = angle_normalize_pi(pll->theta_est);
    CORDIC_SinCos_Rad(pll->theta_est, &sin_theta, &cos_theta);
    pll->error =
        (-obs->i_env_beta * cos_theta) + (obs->i_env_alpha * sin_theta);
    pll->p_term = pll->kp * pll->error;
    pll->i_term += pll->ki * pll->error;

    if (pll->i_term > HFI_PLL_INTEGRAL_LIMIT) {
        pll->i_term = HFI_PLL_INTEGRAL_LIMIT;
    }
    else if (pll->i_term < -HFI_PLL_INTEGRAL_LIMIT) {
        pll->i_term = -HFI_PLL_INTEGRAL_LIMIT;
    }

    pll->omega_est = pll->p_term + pll->i_term;
    pll->omega_est_filt = butterworth_filter(&omega_filter, pll->omega_est);
    pll->theta_est += pll->omega_est * ts;
}
