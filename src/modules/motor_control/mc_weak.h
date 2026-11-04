/**
 * @file mc_weak.h
 * @brief 母线电压反馈弱磁控制。
 */

#ifndef MC_WEAK_H_
#define MC_WEAK_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MC_WEAK_MAX_D_CURRENT_A           (-0.5f)
#define MC_WEAK_START_UTILIZATION         0.90f
#define MC_WEAK_KP_A_PER_V                0.15f
#define MC_WEAK_KI_A_PER_V_S              25.0f
#define MC_WEAK_MIN_SPEED_RAD_S           60.0f
#define MC_WEAK_LOOP_DIVIDER              3U
#define MC_WEAK_TRACKING_RELEASE_A_PER_S  35.0f
#define MC_WEAK_VOLTAGE_FILTER_TAU_S      0.0002f
#define MC_WEAK_D_TRACKING_MARGIN_A       0.1f
#define MC_WEAK_D_TRACKING_RECOVERY_A     0.05f
#define MC_WEAK_D_TRACKING_DEBOUNCE_COUNT 10U

/**
 * @brief 初始化弱磁控制器
 * @param current_loop_dt 电流环控制周期（s）
 * @param current_limit 合成电流矢量限幅（A）
 */
void mc_weak_init(float current_loop_dt, float current_limit);

/**
 * @brief 清除弱磁积分器和跟踪恢复状态
 */
void mc_weak_reset(void);

/**
 * @brief 根据母线电压利用率更新弱磁 d 轴电流
 * @param enabled 弱磁使能标志
 * @param voltage_d d 轴输出电压（V）
 * @param voltage_q q 轴输出电压（V）
 * @param v_bus 母线电压（V）
 * @param min_v_bus 弱磁允许的最低母线电压（V）
 * @param mech_speed_rad_s 机械角速度（rad/s）
 * @param d_current_target 弱磁前 d 轴电流给定（A）
 * @param d_current_measured d 轴电流反馈（A）
 */
void mc_weak_update(bool enabled, float voltage_d, float voltage_q, float v_bus,
                    float min_v_bus, float mech_speed_rad_s,
                    float d_current_target, float d_current_measured);

/**
 * @brief 将弱磁电流叠加到给定并执行圆形电流限幅
 * @param d_current d 轴电流给定（A），函数内更新
 * @param q_current q 轴电流给定（A），函数内更新
 */
void mc_weak_apply_current_limit(float *d_current, float *q_current);

/**
 * @brief 获取当前弱磁附加 d 轴电流
 * @return 弱磁附加电流（A，范围负值到 0）
 */
float mc_weak_get_d_current(void);

#ifdef __cplusplus
}
#endif

#endif /* MC_WEAK_H_ */
