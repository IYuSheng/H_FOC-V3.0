/**
 * @file mc_hfi_observer.h
 * @brief 高频方波注入电流解调观测器。
 */

#ifndef MC_HFI_OBSERVER_H_
#define MC_HFI_OBSERVER_H_

#include <stdint.h>

#include "board_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief HFI 高频注入观测器状态
 */
typedef struct HFI_Observer {
    float u_inject;              /**< 注入电压幅值（V） */
    float u_inject_d;            /**< 当前 d 轴注入电压（V） */
    uint8_t inject_flag;         /**< 注入极性标志 */
    uint8_t init_done;           /**< 初始化完成标志 */
    float i_alpha_last[2];       /**< α 轴最近两次电流（A） */
    float i_beta_last[2];        /**< β 轴最近两次电流（A） */
    float i_alpha_h;             /**< α 轴高频电流（A） */
    float i_beta_h;              /**< β 轴高频电流（A） */
    float i_alpha_h_last;        /**< 上一周期 α 轴高频电流（A） */
    float i_beta_h_last;         /**< 上一周期 β 轴高频电流（A） */
    float i_env_alpha;           /**< α 轴解调电流包络（A） */
    float i_env_beta;            /**< β 轴解调电流包络（A） */
    float i_d_fund;              /**< d 轴基频电流（A） */
    float i_q_fund;              /**< q 轴基频电流（A） */
    uint16_t nsd_count;          /**< 极性辨识计数 */
    float nsd_target_d;          /**< 极性辨识 d 轴目标电流（A） */
    float sum_pos;               /**< 正脉冲响应累积 */
    float sum_neg;               /**< 负脉冲响应累积 */
    uint8_t polarity_determined; /**< 极性确定标志 */
    uint8_t state;               /**< HFI 状态 */
    uint32_t converge_cnt;       /**< 收敛等待计数 */
} HFI_Observer_t;

typedef enum {
    HFI_STATE_CONVERGE = 0,
    HFI_STATE_NSD,
    HFI_STATE_RUN
} HFI_State_t;

#define HFI_INJECT_VOLTAGE 0.5f
#define HFI_NSD_ENABLE     0

/**
 * @brief HFI 观测器全局实例
 */
extern HFI_Observer_t g_hfi_obs;

/**
 * @brief 初始化 HFI 观测器
 * @param obs 观测器状态指针
 * @param u_inject 注入电压幅值（V）
 */
void hfi_observer_init(HFI_Observer_t *obs, float u_inject);

/**
 * @brief 提取高频电流并完成同步解调
 * @param obs 观测器状态指针
 * @param i_alpha α 轴电流（A）
 * @param i_beta β 轴电流（A）
 */
void hfi_observer_update(HFI_Observer_t *obs, float i_alpha, float i_beta);

/**
 * @brief 从含高频分量的 d/q 电流中提取基频电流
 * @param obs 观测器状态指针
 * @param i_d d 轴电流（A）
 * @param i_q q 轴电流（A）
 * @param i_d_fund 输出 d 轴基频电流（A）
 * @param i_q_fund 输出 q 轴基频电流（A）
 */
void hfi_extract_fundamental(HFI_Observer_t *obs, float i_d, float i_q,
                             float *i_d_fund, float *i_q_fund);

/**
 * @brief 获取当前 d 轴方波注入电压
 * @param obs 观测器状态指针
 * @return 当前注入电压（V）
 */
float hfi_get_inject_voltage(HFI_Observer_t *obs);

#ifdef __cplusplus
}
#endif

#endif /* MC_HFI_OBSERVER_H_ */
