#include "mc_hfi_observer.h"

#include <math.h>

#include "mc_hfi_pll.h"
#include "mc_math.h"
#include "usart.h"

HFI_Observer_t g_hfi_obs;

/**
 * @brief 初始化 HFI 观测器
 * @param obs 观测器状态指针
 * @param u_inject 注入电压幅值（V）
 */
void hfi_observer_init(HFI_Observer_t *obs, float u_inject)
{
    if (obs == NULL) {
        return;
    }

    obs->u_inject = u_inject;
    obs->u_inject_d = 0.0f;
    obs->inject_flag = 0U;
    obs->init_done = 0U;
    obs->i_alpha_last[0] = 0.0f;
    obs->i_alpha_last[1] = 0.0f;
    obs->i_beta_last[0] = 0.0f;
    obs->i_beta_last[1] = 0.0f;
    obs->i_alpha_h = 0.0f;
    obs->i_beta_h = 0.0f;
    obs->i_alpha_h_last = 0.0f;
    obs->i_beta_h_last = 0.0f;
    obs->i_env_alpha = 0.0f;
    obs->i_env_beta = 0.0f;
    obs->i_d_fund = 0.0f;
    obs->i_q_fund = 0.0f;
    obs->nsd_count = 0U;
    obs->nsd_target_d = 0.0f;
    obs->sum_pos = 0.0f;
    obs->sum_neg = 0.0f;
    obs->polarity_determined = 0U;
    obs->state = (uint8_t)HFI_STATE_CONVERGE;
    obs->converge_cnt = 0U;
}

/**
 * @brief 提取高频电流并完成同步解调
 * @param obs 观测器状态指针
 * @param i_alpha α 轴电流（A）
 * @param i_beta β 轴电流（A）
 */
void hfi_observer_update(HFI_Observer_t *obs, float i_alpha, float i_beta)
{
    float sign;

    if (obs == NULL) {
        return;
    }

    obs->i_alpha_h =
        (i_alpha - (2.0f * obs->i_alpha_last[0]) + obs->i_alpha_last[1]) *
        0.25f;
    obs->i_beta_h =
        (i_beta - (2.0f * obs->i_beta_last[0]) + obs->i_beta_last[1]) * 0.25f;
    obs->i_alpha_last[1] = obs->i_alpha_last[0];
    obs->i_alpha_last[0] = i_alpha;
    obs->i_beta_last[1] = obs->i_beta_last[0];
    obs->i_beta_last[0] = i_beta;

    sign = (obs->inject_flag == 0U) ? 1.0f : -1.0f;
    obs->i_env_alpha = (obs->i_alpha_h - obs->i_alpha_h_last) * sign;
    obs->i_env_beta = (obs->i_beta_h - obs->i_beta_h_last) * sign;
    obs->i_alpha_h_last = obs->i_alpha_h;
    obs->i_beta_h_last = obs->i_beta_h;
    obs->inject_flag ^= 1U;
}

/**
 * @brief 从含高频分量的 d/q 电流中提取基频电流
 * @param obs 观测器状态指针
 * @param i_d d 轴电流（A）
 * @param i_q q 轴电流（A）
 * @param i_d_fund 输出 d 轴基频电流（A）
 * @param i_q_fund 输出 q 轴基频电流（A）
 */
void hfi_extract_fundamental(HFI_Observer_t *obs, float i_d, float i_q,
                             float *i_d_fund, float *i_q_fund)
{
    static float i_d_last[2] = {0.0f, 0.0f};
    static float i_q_last[2] = {0.0f, 0.0f};

    if ((obs == NULL) || (i_d_fund == NULL) || (i_q_fund == NULL)) {
        return;
    }

    *i_d_fund = (i_d + (2.0f * i_d_last[0]) + i_d_last[1]) * 0.25f;
    *i_q_fund = (i_q + (2.0f * i_q_last[0]) + i_q_last[1]) * 0.25f;
    i_d_last[1] = i_d_last[0];
    i_d_last[0] = i_d;
    i_q_last[1] = i_q_last[0];
    i_q_last[0] = i_q;
    obs->i_d_fund = *i_d_fund;
    obs->i_q_fund = *i_q_fund;
}

/**
 * @brief 获取当前 d 轴方波注入电压
 * @param obs 观测器状态指针
 * @return 当前注入电压（V）
 */
float hfi_get_inject_voltage(HFI_Observer_t *obs)
{
    if (obs == NULL) {
        return 0.0f;
    }

    obs->u_inject_d = (obs->inject_flag == 0U) ? obs->u_inject : -obs->u_inject;
    return obs->u_inject_d;
}

/**
 * @brief 执行正负脉冲极性辨识
 * @param obs HFI 观测器状态指针
 * @param pll HFI-PLL 状态指针
 * @param i_d d 轴电流（A）
 * @param i_q q 轴电流（A）
 */
void hfi_nsd_check(HFI_Observer_t *obs, HFI_PLL_t *pll, float i_d, float i_q)
{
    (void)i_d;
    (void)i_q;
    if ((obs == NULL) || (pll == NULL) || (obs->polarity_determined != 0U)) {
        return;
    }
    obs->nsd_count++;

#if HFI_NSD_ENABLE
    {
        static float i_d_last[2] = {0.0f, 0.0f};
        const float i_d_h = (i_d - (2.0f * i_d_last[0]) + i_d_last[1]) * 0.25f;

        i_d_last[1] = i_d_last[0];
        i_d_last[0] = i_d;
        if (obs->nsd_count < 200U) {
            obs->nsd_target_d = 0.2f;
        }
        else if (obs->nsd_count < 250U) {
            obs->sum_pos += fabsf(i_d_h);
            obs->nsd_target_d = 0.2f;
        }
        else if (obs->nsd_count < 350U) {
            obs->nsd_target_d = -0.2f;
        }
        else if (obs->nsd_count < 400U) {
            obs->sum_neg += fabsf(i_d_h);
            obs->nsd_target_d = -0.2f;
        }
        else {
            if (obs->sum_neg > obs->sum_pos) {
                pll->theta_est = angle_normalize_pi(pll->theta_est + PI);
            }
            obs->polarity_determined = 1U;
            obs->nsd_target_d = 0.0f;
        }
    }
#else
    if (obs->nsd_count < 2000U) {
        obs->nsd_target_d = 1.0f;
    }
    else {
        obs->nsd_target_d = 0.0f;
        pll->theta_est = angle_normalize_pi(pll->theta_est);
        obs->polarity_determined = 1U;
    }
#endif
}
