#include "mc_flux_observer.h"

#include "cordic.h"
#include "mc_filter.h"
#include "mc_math.h"

FluxObserver_t g_flux_obs;

/**
 * @brief 初始化非线性磁链观测器
 * @param obs 观测器状态指针
 */
void flux_observer_init(FluxObserver_t *obs)
{
    if (obs == NULL) {
        return;
    }

    obs->x1 = 0.0f;
    obs->x2 = 0.0f;
    obs->theta_hat = 0.0f;
    obs->omega_integ = 0.0f;
    obs->omega_filt = 0.0f;
    obs->cos_theta = 1.0f;
    obs->sin_theta = 0.0f;
}

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
                               float i_alpha, float i_beta, float ts)
{
    static ButterworthLPF_t omega_flux_filter;
    static uint8_t filter_initialized = 0U;
    float y1;
    float y2;
    float eta1;
    float eta2;
    float eta_norm_squared;
    float error_term;

    if ((obs == NULL) || (ts <= 0.0f)) {
        return;
    }

    if (filter_initialized == 0U) {
        butterworth_init(&omega_flux_filter, 10.0f, PWM_FREQ);
        filter_initialized = 1U;
    }

    y1 = u_alpha - (FLUX_R_S * i_alpha);
    y2 = u_beta - (FLUX_R_S * i_beta);
    eta1 = obs->x1 - (FLUX_L_S * i_alpha);
    eta2 = obs->x2 - (FLUX_L_S * i_beta);
    eta_norm_squared = (eta1 * eta1) + (eta2 * eta2);
    error_term = FLUX_GAMMA_K * (FLUX_PHI_M_2 - eta_norm_squared);

    obs->x1 += ts * (y1 + (error_term * eta1));
    obs->x2 += ts * (y2 + (error_term * eta2));

    eta1 = obs->x1 - (FLUX_L_S * i_alpha);
    eta2 = obs->x2 - (FLUX_L_S * i_beta);

#if FLUX_OBSERVER_USE_PLL
    {
        const float sin_theta_real = eta2 * _1_FLUX_PHI_M;
        const float cos_theta_real = eta1 * _1_FLUX_PHI_M;
        const float angle_error = (sin_theta_real * obs->cos_theta) -
            (cos_theta_real * obs->sin_theta);
        const float omega_pll_before_filter =
            obs->omega_integ + (PLL_KP * angle_error);

        obs->omega_integ += PLL_KI * angle_error * ts;
        obs->omega_filt =
            butterworth_filter(&omega_flux_filter, omega_pll_before_filter);
        obs->theta_hat += omega_pll_before_filter * ts;
        obs->theta_hat = angle_normalize_pi(obs->theta_hat);
        CORDIC_SinCos_Rad(obs->theta_hat, &obs->sin_theta, &obs->cos_theta);
    }
#else
    {
        const float theta_atan = atan2f(eta2, eta1);
        const float delta_theta =
            angle_normalize_pi(theta_atan - obs->theta_hat);
        const float omega_atan = delta_theta / ts;

        obs->theta_hat = angle_normalize_pi(theta_atan);
        obs->omega_filt = butterworth_filter(&omega_flux_filter, omega_atan);
        obs->omega_integ = 0.0f;
        CORDIC_SinCos_Rad(obs->theta_hat, &obs->sin_theta, &obs->cos_theta);
    }
#endif
}
