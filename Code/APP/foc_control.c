#include "foc_control.h"

#include <math.h>

static SCurve_Planner_t position_planner;
static PositionLadrc_t position_ladrc;
static uint32_t step_tick;
static uint8_t step_target_b;
static uint8_t position_plan_valid;
static float position_last_target;

/**
 * @brief 使用当前电机参数初始化电流环 PI
 */
static void foc_current_pi_init_from_motor(void);

/**
 * @brief 在 IDLE 模式输出四相中点占空比
 */
static inline void foc_output_idle(void);

/**
 * @brief FOC电流闭环控制函数
 * @param target_d d 轴目标电流，单位 A
 * @param target_q q 轴目标电流，单位 A
 */
static inline void foc_current_control(float target_d, float target_q);

/**
 * @brief FOC位置闭环控制函数
 * @param target_position 目标机械位置，单位 rad
 */
static inline void foc_position_control(float target_position);

/**
 * @brief 按串口开关输出连续五通道波形
 */
void foc_debug(void)
{
    if ((foc_runtime_debug_is_enabled() == 0U) ||
        (foc_encoder_calibration_is_running() != 0U)) {
        return;
    }

    // debug_log("%.4f, %.4f, %.4f, %.4f, %.4f", foc_ctrl.target_q,
    // foc_ctrl.abc_dq.current_q, foc_ctrl.shaped_t_p,
    // encoder_data.mechanical_speed, encoder_data.mechanical_position);
    // debug_log("%.4f, %.4f, %.4f, %.4f", foc_ctrl.target_q,
    //           foc_ctrl.abc_dq.current_q, foc_ctrl.target_speed,
    //           encoder_data.mechanical_speed);
    // debug_log("%u, %u, %u, %u", spwm.duty_An, spwm.duty_Ap, spwm.duty_Bn,
    // spwm.duty_Bp); debug_log("%.4f, %.4f, %.4f", alpha_beta.alpha,
    // alpha_beta.beta, foc_voltage_data.vbus); debug_log("%.4f, %.4f, %.4f,
    // %.4f", foc_current_data.IA, foc_current_data.IB,
    // foc_ctrl.abc_dq.current_q, foc_ctrl.abc_dq.current_d); debug_log("%.4f,
    // %.4f, %.4f, %.4f", foc_ctrl.abc_dq.current_q, g_flux_obs.omega_filt,
    // g_flux_obs.theta_hat, foc_ctrl.angle);
    // debug_log("%.4f", foc_voltage_data.vbus);

    // 电磁转矩计算
    // foc_ctrl.Te = 1.5f * MOTOR_POLE_PAIRS * (MOTOR_FLUX_LINKAGE *
    // foc_ctrl.abc_dq.current_q);
    // debug_log("%.4f", foc_ctrl.Te);

    debug_log("%.4f, %.4f, %.4f, %.4f, %.4f", foc_ctrl.target_q,
              foc_ctrl.abc_dq.current_q, encoder_data.mechanical_position,
              encoder_data.mechanical_speed, foc_ctrl.shaped_t_p);
}

/**
 * @brief 初始化 FOC 控制器并校准电流采样零偏
 */
void foc_start_init(void)
{
    foc_ctrl.target_speed = 0.0f;
    foc_ctrl.out_q = 0.0f;
    foc_ctrl.out_d = 0.0f;
    foc_ctrl.target_q = 0.0f;
    foc_ctrl.target_d = 0.0f;
    foc_ctrl.target_position = encoder_data.mechanical_position;
    foc_ctrl.shaped_t_p = encoder_data.mechanical_position;
    foc_ctrl.shaped_t_v = 0.0f;
    foc_ctrl.shaped_t_a = 0.0f;

    foc_current_pi_init_from_motor();
    position_ladrc_reset(&position_ladrc, encoder_data.mechanical_position);
    s_curve_planner_init(&position_planner, FOC_S_CURVE_MAX_VELOCITY,
                         FOC_S_CURVE_MAX_ACCELERATION, FOC_S_CURVE_MAX_JERK,
                         POSITION_LOOP_DT);

    get_foc_bus_voltage();
    debug_log("Vbus: %.2f V", foc_voltage_data.vbus);
    adc1_current_offset_calibrate();
}

/**
 * @brief 按指定控制类型进入 RUNNING 模式
 * @param profile 电流运行或位置阶跃
 * @return 1 表示启动成功，0 表示状态不允许
 */
uint8_t foc_control_start(foc_run_profile_t profile)
{
    const foc_runtime_config_t *config = foc_runtime_get_config();

    if ((foc_runtime_is_ready() == 0U) ||
        (foc_runtime_get_mode() != FOC_MODE_IDLE) ||
        (foc_encoder_calibration_is_running() != 0U) ||
        foc_mechanical_param_is_active()) {
        return 0U;
    }

    foc_current_pi_init_from_motor();
    position_ladrc_reset(&position_ladrc, encoder_data.mechanical_position);
    s_curve_planner_init(&position_planner, FOC_S_CURVE_MAX_VELOCITY,
                         FOC_S_CURVE_MAX_ACCELERATION, FOC_S_CURVE_MAX_JERK,
                         POSITION_LOOP_DT);

    foc_ctrl.target_d = config->target_id;
    foc_ctrl.target_q = (profile == FOC_RUN_CURRENT) ? config->target_iq : 0.0f;
    foc_ctrl.target_position = (profile == FOC_RUN_POSITION_STEP)
        ? config->step_position_a
        : encoder_data.mechanical_position;
    foc_ctrl.shaped_t_p = encoder_data.mechanical_position;
    foc_ctrl.shaped_t_v = 0.0f;
    foc_ctrl.shaped_t_a = 0.0f;
    foc_ctrl.out_d = 0.0f;
    foc_ctrl.out_q = 0.0f;

    step_tick = 0U;
    step_target_b = 0U;
    position_plan_valid = 0U;

    return foc_runtime_start(profile);
}

/**
 * @brief 启动机械参数辨识维护任务
 * @return 1 表示已启动，0 表示状态或硬件条件不允许
 */
uint8_t foc_control_start_mechanical_identification(void)
{
    if ((foc_runtime_is_ready() == 0U) ||
        (foc_runtime_get_mode() != FOC_MODE_IDLE) ||
        (foc_encoder_calibration_is_running() != 0U) ||
        foc_mechanical_param_is_active()) {
        return 0U;
    }

    foc_current_pi_init_from_motor();
    foc_ctrl.target_d = 0.0f;
    foc_ctrl.target_q = 0.0f;
    foc_ctrl.out_d = 0.0f;
    foc_ctrl.out_q = 0.0f;
    foc_mechanical_param_start();
    return 1U;
}

/**
 * @brief 在主循环处理机械辨识结果并保存 Ka/Kv/Tc 到 FLASH
 */
void foc_control_background_process(void)
{
    const mech_identify_result_t *result;
    float ka;
    float kv;
    float tc;
    uint8_t result_valid;

    if (g_mech_identify.state == MECH_IDENTIFY_FAILED) {
        foc_mechanical_param_stop();
        foc_output_idle();
        debug_log("<ERR:identify mech failed>");
        return;
    }
    if (g_mech_identify.state != MECH_IDENTIFY_COMPLETE) {
        return;
    }

    result = foc_mechanical_param_get_result();
    ka = result->Ka;
    kv = result->Kv;
    tc = fabsf(result->Tc);
    result_valid =
        ((result->valid != false) && isfinite(ka) && isfinite(kv) &&
         isfinite(tc) && (ka > 0.0f) && (ka <= 100.0f) && (kv >= 0.0f) &&
         (kv <= 100.0f) && (tc >= 0.0f) && (tc <= 100.0f))
        ? 1U
        : 0U;
    foc_mechanical_param_stop();
    foc_output_idle();

    if (result_valid == 0U) {
        debug_log("<ERR:identify mech result invalid>");
    }
    else if (foc_runtime_set_mechanical(ka, kv, tc) != 0U) {
        debug_log("<OK:identify mech saved Ka=%.9f Kv=%.9f Tc=%.9f>", ka, kv,
                  tc);
    }
    else {
        debug_log("<ERR:identify mech FLASH write failed>");
    }
}

static void foc_current_pi_init_from_motor(void)
{
    const foc_motor_config_t *motor = foc_runtime_get_motor_config();
    const float bandwidth_rad = 2.0f * PI * CURRENT_LOOP_BANDWIDTH;

    foc_current_pi_init(bandwidth_rad * motor->inductance_ld,
                        bandwidth_rad * motor->inductance_lq,
                        bandwidth_rad * motor->resistance * CURRENT_LOOP_DT,
                        I_I_LIMIT);
}

/**
 * @brief FOC外环控制主函数
 */
void foc_control(void)
{
    const foc_runtime_config_t *config;
    uint32_t ticks_per_ms;
    uint32_t toggle_ticks;

    if (foc_mechanical_param_is_active()) {
        foc_mechanical_param_process();
        return;
    }

    if ((foc_runtime_get_mode() != FOC_MODE_RUNNING) ||
        (foc_runtime_get_profile() != FOC_RUN_POSITION_STEP)) {
        return;
    }

    config = foc_runtime_get_config();
    ticks_per_ms = (uint32_t)POSITION_LOOP_TIME / 1000U;
    if (ticks_per_ms == 0U) {
        ticks_per_ms = 1U;
    }
    toggle_ticks = config->step_period_ms * ticks_per_ms;

    if (++step_tick >= toggle_ticks) {
        step_tick = 0U;
        step_target_b ^= 1U;
        foc_ctrl.target_position = (step_target_b != 0U)
            ? config->step_position_b
            : config->step_position_a;
    }

    foc_position_control(foc_ctrl.target_position);
}

/**
 * @brief FOC内环电流环外部调用接口
 */
void foc_current_in_control(void)
{
    if (foc_encoder_calibration_is_running() != 0U) {
        foc_encoder_calibration_open_loop_calibrate();
        return;
    }

    if (foc_mechanical_param_is_active()) {
        foc_current_control(foc_ctrl.target_d, foc_ctrl.target_q);
        return;
    }

    if (foc_runtime_get_mode() != FOC_MODE_RUNNING) {
        foc_output_idle();
        return;
    }

    foc_current_control(foc_ctrl.target_d, foc_ctrl.target_q);
}

/**
 * @brief 在 IDLE 模式输出四相中点占空比
 */
static inline void foc_output_idle(void)
{
    const uint32_t half_pwm = (uint32_t)(PWM_PERIOD * 0.5f);

    foc_ctrl.target_d = 0.0f;
    foc_ctrl.target_q = 0.0f;
    foc_ctrl.out_d = 0.0f;
    foc_ctrl.out_q = 0.0f;
    alpha_beta.alpha = 0.0f;
    alpha_beta.beta = 0.0f;
    bsp_pwm_set_duty_four_phase(half_pwm, half_pwm, half_pwm, half_pwm);
}

/**
 * @brief FOC电流闭环控制函数
 * @param target_d d 轴目标电流，单位 A
 * @param target_q q 轴目标电流，单位 A
 */
static inline void foc_current_control(float target_d, float target_q)
{
    const foc_motor_config_t *motor = foc_runtime_get_motor_config();
    float sin_theta;
    float cos_theta;

    alpha_beta.alpha_i = foc_current_data.IA;
    alpha_beta.beta_i = foc_current_data.IB;
    foc_ctrl.angle = encoder_data.electrical_angle;
    CORDIC_SinCos_Rad(foc_ctrl.angle, &sin_theta, &cos_theta);
    park_transform(&alpha_beta, &foc_ctrl, sin_theta, cos_theta);

    float voltage_d = foc_id_pid_calculate(target_d, foc_ctrl.abc_dq.current_d);
    float voltage_q = foc_iq_pid_calculate(target_q, foc_ctrl.abc_dq.current_q);
    float electrical_speed = encoder_data.electrical_speed;

    voltage_d +=
        -electrical_speed * motor->inductance_lq * foc_ctrl.abc_dq.current_q;
    voltage_q += electrical_speed *
        (motor->inductance_ld * foc_ctrl.abc_dq.current_d +
         motor->flux_linkage);

    foc_ctrl.out_d = voltage_d;
    foc_ctrl.out_q = voltage_q;
    inv_park_transform_f32(&foc_ctrl, &alpha_beta, sin_theta, cos_theta);
    spwm_duty_calc(&spwm, &alpha_beta, foc_voltage_data.vbus);
    bsp_pwm_set_duty_four_phase(spwm.duty_Ap, spwm.duty_An, spwm.duty_Bp,
                                spwm.duty_Bn);
}

/**
 * @brief FOC位置闭环控制函数
 * @param target_position 目标机械位置，单位 rad
 */
static inline void foc_position_control(float target_position)
{
    float planned_position;

    if ((position_plan_valid == 0U) ||
        (fabsf(target_position - position_last_target) > 0.00174532925f)) {
        s_curve_set_target(&position_planner, foc_ctrl.shaped_t_p,
                           target_position);
        position_last_target = target_position;
        position_plan_valid = 1U;
    }

    if (s_curve_is_done(&position_planner) != 0U) {
        planned_position = target_position;
        foc_ctrl.shaped_t_v = 0.0f;
        foc_ctrl.shaped_t_a = 0.0f;
    }
    else {
        planned_position = s_curve_update(&position_planner);
        foc_ctrl.shaped_t_v = position_planner.current_vel;
        foc_ctrl.shaped_t_a = position_planner.current_acc;
    }

    foc_ctrl.shaped_t_p = planned_position;
    foc_ctrl.target_q = position_ladrc_update(
        &position_ladrc, foc_ctrl.shaped_t_p, foc_ctrl.shaped_t_v,
        foc_ctrl.shaped_t_a, encoder_data.mechanical_position);
}
