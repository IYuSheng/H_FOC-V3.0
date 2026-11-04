#include "mc_current.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>

#include "board_config.h"
#include "cordic.h"
#include "mc_config.h"
#include "mc_encoder.h"
#include "mc_math.h"
#include "mc_runtime.h"
#include "mc_weak.h"
#include "tim.h"

#define MC_VOLTAGE_LIMIT_UTILIZATION 0.95f
#define MC_CURRENT_LIMIT_D           0x01U
#define MC_CURRENT_LIMIT_Q           0x02U

static volatile float open_loop_angle_rad MC_CCM_BSS;
static volatile float open_loop_voltage_d MC_CCM_BSS;
static volatile float open_loop_voltage_q MC_CCM_BSS;
static const mc_motor_config_t *current_motor_config MC_CCM_BSS;
static const mc_runtime_config_t *current_runtime_config MC_CCM_BSS;
static float decoupling_d_gain MC_CCM_BSS;
static float decoupling_q_gain MC_CCM_BSS;
static float bemf_gain MC_CCM_BSS;
static bool weakening_active MC_CCM_BSS;

extern pi_t id_pid;
extern pi_t iq_pid;

/**
 * @brief 将电压矢量限制到母线电压允许的圆形范围内
 * @param voltage_d d 轴电压（V），函数内更新
 * @param voltage_q q 轴电压（V），函数内更新
 * @param maximum 最大电压矢量幅值（V）
 * @return 限幅状态位，bit0 为 d 轴限幅，bit1 为 q 轴限幅
 */
static uint8_t mc_current_limit_voltage(float *voltage_d, float *voltage_q,
                                        float maximum)
{
    float d_squared;
    float q_squared;
    float maximum_d_squared;
    float q_limit_squared;
    float q_limit;
    uint8_t limited = 0U;

    if ((voltage_d == NULL) || (voltage_q == NULL)) {
        return 0U;
    }
    if (maximum <= 0.0f) {
        *voltage_d = 0.0f;
        *voltage_q = 0.0f;
        return MC_CURRENT_LIMIT_D | MC_CURRENT_LIMIT_Q;
    }

    if (*voltage_d > maximum) {
        *voltage_d = maximum;
        limited |= MC_CURRENT_LIMIT_D;
    }
    else if (*voltage_d < -maximum) {
        *voltage_d = -maximum;
        limited |= MC_CURRENT_LIMIT_D;
    }

    d_squared = *voltage_d * *voltage_d;
    q_squared = *voltage_q * *voltage_q;
    maximum_d_squared = maximum * maximum;
    if ((d_squared + q_squared) <= maximum_d_squared) {
        return limited;
    }

    q_limit_squared = maximum_d_squared - d_squared;
    if (q_limit_squared < 0.0f) {
        q_limit_squared = 0.0f;
    }
    q_limit = sqrtf(q_limit_squared);
    if (*voltage_q > q_limit) {
        *voltage_q = q_limit;
        limited |= MC_CURRENT_LIMIT_Q;
    }
    else if (*voltage_q < -q_limit) {
        *voltage_q = -q_limit;
        limited |= MC_CURRENT_LIMIT_Q;
    }

    return limited;
}

/**
 * @brief 根据输出电压限幅量修正 PI 积分项
 * @param pid PI 控制器
 * @param voltage_limited 限幅后的输出电压
 * @param voltage_unlimited 限幅前的输出电压
 */
static void mc_current_apply_antiwindup(pi_t *pid, float voltage_limited,
                                        float voltage_unlimited)
{
    float integral;

    if ((pid == NULL) || (voltage_limited == voltage_unlimited)) {
        return;
    }

    integral = pid->integral + (voltage_limited - voltage_unlimited);
    if (integral > pid->integral_limit) {
        integral = pid->integral_limit;
    }
    else if (integral < -pid->integral_limit) {
        integral = -pid->integral_limit;
    }
    pid->integral = integral;
}

/**
 * @brief 根据当前电机参数初始化 d/q 轴电流 PI
 */
void mc_current_control_init_from_motor(void)
{
    const mc_motor_config_t *motor = mc_runtime_get_motor_config();
    const float tuning_delay = 3.0f * CURRENT_LOOP_DT;
    const float kp_d = motor->inductance_ld / tuning_delay;
    const float kp_q = motor->inductance_lq / tuning_delay;
    const float ki_continuous = motor->resistance / tuning_delay;
    const float ki_discrete = ki_continuous * CURRENT_LOOP_DT;

    current_motor_config = motor;
    current_runtime_config = mc_runtime_get_config();
    weakening_active = false;
    mc_current_pi_init(kp_d, kp_q, ki_discrete, I_I_LIMIT);
    mc_weak_init(CURRENT_LOOP_DT, CURRENT_LIMIT);
    mc_current_control_update_feedforward(encoder_data.electrical_speed);
}

/**
 * @brief 更新电流环速度前馈系数
 * @param electrical_speed_rad_s 电角速度，单位：rad/s
 */
void mc_current_control_update_feedforward(float electrical_speed_rad_s)
{
    if (current_motor_config == NULL) {
        decoupling_d_gain = 0.0f;
        decoupling_q_gain = 0.0f;
        bemf_gain = 0.0f;
        return;
    }

    decoupling_d_gain =
        electrical_speed_rad_s * current_motor_config->inductance_lq;
    decoupling_q_gain =
        electrical_speed_rad_s * current_motor_config->inductance_ld;
    bemf_gain = electrical_speed_rad_s * current_motor_config->flux_linkage;
}

/**
 * @brief 复位 d/q 轴电流 PI 状态
 */
void mc_current_control_reset(void)
{
    id_pid.integral = 0.0f;
    iq_pid.integral = 0.0f;
    id_pid.output = 0.0f;
    iq_pid.output = 0.0f;
    weakening_active = false;
    mc_weak_reset();
}

/**
 * @brief 设置开环电压输出目标
 * @param angle_rad 电角度（rad）
 * @param voltage_d d 轴电压（V）
 * @param voltage_q q 轴电压（V）
 */
void mc_current_control_set_open_loop(float angle_rad, float voltage_d,
                                      float voltage_q)
{
    open_loop_angle_rad = angle_rad;
    open_loop_voltage_d = voltage_d;
    open_loop_voltage_q = voltage_q;
}

/**
 * @brief 在电流环中断中输出已设置的开环电压
 */
void mc_current_control_open_loop_tick(void)
{
    const float angle_rad = open_loop_angle_rad;
    float sin_theta;
    float cos_theta;

    CORDIC_SinCos_Rad(angle_rad, &sin_theta, &cos_theta);
    mc_ctrl.angle = angle_rad;
    mc_ctrl.out_d = open_loop_voltage_d;
    mc_ctrl.out_q = open_loop_voltage_q;
    inv_park_transform_f32(&mc_ctrl, &alpha_beta, sin_theta, cos_theta);
    spwm_duty_calc(&spwm, &alpha_beta);
    bsp_pwm_set_duty_four_phase(spwm.duty_Ap, spwm.duty_An, spwm.duty_Bp,
                                spwm.duty_Bn);
}

/**
 * @brief 执行一次 d/q 轴电流闭环控制
 * @param target_d d 轴目标电流（A）
 * @param target_q q 轴目标电流（A）
 * @param control_running 1 表示正常闭环运行，0 表示维护模式
 */
void mc_current_control_run(float target_d, float target_q,
                            uint8_t control_running)
{
    const bool weakening_enabled = (control_running != 0U) &&
        (current_runtime_config != NULL) &&
        (current_runtime_config->weakening_enabled != 0U);
    const float requested_d = target_d;
    float effective_d = target_d;
    float effective_q = target_q;
    float voltage_d;
    float voltage_q;
    float maximum_voltage;
    float voltage_d_unlimited;
    float voltage_q_unlimited;
    uint8_t limit_flags;
    const float sin_theta = encoder_data.electrical_sin;
    const float cos_theta = encoder_data.electrical_cos;

    alpha_beta.alpha_i = mc_current_data.IA;
    alpha_beta.beta_i = mc_current_data.IB;
    mc_ctrl.angle = encoder_data.electrical_angle;
    park_transform(&alpha_beta, &mc_ctrl, sin_theta, cos_theta);

    if (weakening_enabled) {
        if (!weakening_active) {
            mc_weak_reset();
            weakening_active = true;
        }
        mc_weak_apply_current_limit(&effective_d, &effective_q);
    }
    else if (weakening_active) {
        mc_weak_reset();
        weakening_active = false;
    }

    voltage_d = mc_id_pid_calculate(effective_d, mc_ctrl.abc_dq.current_d);
    voltage_q = mc_iq_pid_calculate(effective_q, mc_ctrl.abc_dq.current_q);

    voltage_d -= decoupling_d_gain * mc_ctrl.abc_dq.current_q;
    voltage_q += (decoupling_q_gain * mc_ctrl.abc_dq.current_d) + bemf_gain;

    voltage_d_unlimited = voltage_d;
    voltage_q_unlimited = voltage_q;
    maximum_voltage = mc_voltage_data.vbus * MC_VOLTAGE_LIMIT_UTILIZATION;
    limit_flags =
        mc_current_limit_voltage(&voltage_d, &voltage_q, maximum_voltage);
    if ((limit_flags & MC_CURRENT_LIMIT_D) != 0U) {
        mc_current_apply_antiwindup(&id_pid, voltage_d, voltage_d_unlimited);
    }
    if ((limit_flags & MC_CURRENT_LIMIT_Q) != 0U) {
        mc_current_apply_antiwindup(&iq_pid, voltage_q, voltage_q_unlimited);
    }

    mc_ctrl.target_d_effective = effective_d;
    mc_ctrl.target_q_effective = effective_q;
    mc_ctrl.out_d = voltage_d;
    mc_ctrl.out_q = voltage_q;

    if (weakening_enabled) {
        mc_weak_update(true, voltage_d, voltage_q, mc_voltage_data.vbus,
                       UNDER_VOLTAGE_THRESH, encoder_data.mechanical_speed,
                       requested_d, mc_ctrl.abc_dq.current_d);
    }

    inv_park_transform_f32(&mc_ctrl, &alpha_beta, sin_theta, cos_theta);
    spwm_duty_calc(&spwm, &alpha_beta);
    bsp_pwm_set_duty_four_phase(spwm.duty_Ap, spwm.duty_An, spwm.duty_Bp,
                                spwm.duty_Bn);
}

/**
 * @brief 输出四相 PWM 中点，使电机进入零电压状态
 */
void mc_output_idle(void)
{
    const uint32_t half_pwm = (uint32_t)(PWM_PERIOD * 0.5f);

    bsp_pwm_disable_outputs();
    mc_ctrl.target_d = 0.0f;
    mc_ctrl.target_q = 0.0f;
    mc_ctrl.target_d_effective = 0.0f;
    mc_ctrl.target_q_effective = 0.0f;
    mc_ctrl.out_d = 0.0f;
    mc_ctrl.out_q = 0.0f;
    alpha_beta.alpha = 0.0f;
    alpha_beta.beta = 0.0f;
    weakening_active = false;
    mc_weak_reset();
    bsp_pwm_set_duty_four_phase(half_pwm, half_pwm, half_pwm, half_pwm);
}
