#include "mc_weak.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>

#if defined(__GNUC__)
#define MC_WEAK_CCM_BSS __attribute__((section(".ccmram_bss"), aligned(4)))
#else
#define MC_WEAK_CCM_BSS
#endif

static float weak_integral MC_WEAK_CCM_BSS;
static float weak_d_current_target MC_WEAK_CCM_BSS;
static float weak_integral_gain MC_WEAK_CCM_BSS;
static float weak_tracking_release_step MC_WEAK_CCM_BSS;
static float weak_voltage_filter_alpha MC_WEAK_CCM_BSS;
static float weak_voltage_utilization_filtered MC_WEAK_CCM_BSS;
static float weak_current_limit MC_WEAK_CCM_BSS;
static float weak_current_limit_sq MC_WEAK_CCM_BSS;
static uint32_t weak_loop_counter MC_WEAK_CCM_BSS;
static uint32_t weak_tracking_error_count MC_WEAK_CCM_BSS;
static bool weak_voltage_filter_initialized MC_WEAK_CCM_BSS;
static bool weak_tracking_recovery MC_WEAK_CCM_BSS;

/**
 * @brief 将浮点数限制到闭区间
 * @param value 输入值
 * @param minimum 最小值
 * @param maximum 最大值
 * @return 限幅后的值
 */
static inline float mc_weak_constrain(float value, float minimum, float maximum)
{
    if (value > maximum) {
        return maximum;
    }
    if (value < minimum) {
        return minimum;
    }
    return value;
}

/**
 * @brief 初始化弱磁控制器
 * @param current_loop_dt 电流环控制周期（s）
 * @param current_limit 合成电流矢量限幅（A）
 */
void mc_weak_init(float current_loop_dt, float current_limit)
{
    const float weak_loop_dt = current_loop_dt * (float)MC_WEAK_LOOP_DIVIDER;

    weak_integral_gain = MC_WEAK_KI_A_PER_V_S * weak_loop_dt;
    weak_tracking_release_step =
        MC_WEAK_TRACKING_RELEASE_A_PER_S * weak_loop_dt;
    weak_voltage_filter_alpha =
        weak_loop_dt / (MC_WEAK_VOLTAGE_FILTER_TAU_S + weak_loop_dt);
    weak_current_limit = current_limit;
    weak_current_limit_sq = current_limit * current_limit;
    mc_weak_reset();
}

/**
 * @brief 清除弱磁积分器和跟踪恢复状态
 */
void mc_weak_reset(void)
{
    weak_integral = 0.0f;
    weak_d_current_target = 0.0f;
    weak_voltage_utilization_filtered = 0.0f;
    weak_loop_counter = 0U;
    weak_tracking_error_count = 0U;
    weak_voltage_filter_initialized = false;
    weak_tracking_recovery = false;
}

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
                    float d_current_target, float d_current_measured)
{
    float voltage_utilization;
    float d_tracking_error;
    float voltage_error;
    float output;

    if (!enabled || (v_bus < min_v_bus) ||
        (fabsf(mech_speed_rad_s) < MC_WEAK_MIN_SPEED_RAD_S)) {
        mc_weak_reset();
        return;
    }

    weak_loop_counter++;
    if (weak_loop_counter < MC_WEAK_LOOP_DIVIDER) {
        return;
    }
    weak_loop_counter = 0U;

    voltage_utilization =
        sqrtf((voltage_d * voltage_d) + (voltage_q * voltage_q)) / v_bus;
    if (!weak_voltage_filter_initialized) {
        weak_voltage_utilization_filtered = voltage_utilization;
        weak_voltage_filter_initialized = true;
    }
    else {
        weak_voltage_utilization_filtered += weak_voltage_filter_alpha *
            (voltage_utilization - weak_voltage_utilization_filtered);
    }

    d_tracking_error = d_current_target - d_current_measured;
    if (!weak_tracking_recovery) {
        if ((weak_d_current_target < 0.0f) &&
            (d_tracking_error < -MC_WEAK_D_TRACKING_MARGIN_A)) {
            weak_tracking_error_count++;
            if (weak_tracking_error_count >=
                MC_WEAK_D_TRACKING_DEBOUNCE_COUNT) {
                weak_tracking_recovery = true;
            }
        }
        else {
            weak_tracking_error_count = 0U;
        }
    }

    if (weak_tracking_recovery) {
        if (d_tracking_error >= -MC_WEAK_D_TRACKING_RECOVERY_A) {
            weak_tracking_recovery = false;
            weak_tracking_error_count = 0U;
        }
        else {
            weak_d_current_target = mc_weak_constrain(
                weak_d_current_target + weak_tracking_release_step,
                MC_WEAK_MAX_D_CURRENT_A, 0.0f);
            weak_integral = weak_d_current_target;
            return;
        }
    }

    voltage_error =
        (MC_WEAK_START_UTILIZATION - weak_voltage_utilization_filtered) * v_bus;
    weak_integral += weak_integral_gain * voltage_error;
    weak_integral =
        mc_weak_constrain(weak_integral, MC_WEAK_MAX_D_CURRENT_A, 0.0f);

    output = (MC_WEAK_KP_A_PER_V * voltage_error) + weak_integral;
    weak_d_current_target =
        mc_weak_constrain(output, MC_WEAK_MAX_D_CURRENT_A, 0.0f);
}

/**
 * @brief 将弱磁电流叠加到给定并执行圆形电流限幅
 * @param d_current d 轴电流给定（A），函数内更新
 * @param q_current q 轴电流给定（A），函数内更新
 */
void mc_weak_apply_current_limit(float *d_current, float *q_current)
{
    float q_limit_sq;
    float q_limit;

    if ((d_current == NULL) || (q_current == NULL)) {
        return;
    }

    *d_current += weak_d_current_target;
    if (weak_d_current_target >= 0.0f) {
        return;
    }

    *d_current =
        mc_weak_constrain(*d_current, -weak_current_limit, weak_current_limit);
    q_limit_sq = weak_current_limit_sq - (*d_current * *d_current);
    if (q_limit_sq < 0.0f) {
        q_limit_sq = 0.0f;
    }

    q_limit = sqrtf(q_limit_sq);
    *q_current = mc_weak_constrain(*q_current, -q_limit, q_limit);
}

/**
 * @brief 获取当前弱磁附加 d 轴电流
 * @return 弱磁附加电流（A，范围负值到 0）
 */
float mc_weak_get_d_current(void)
{
    return weak_d_current_target;
}
