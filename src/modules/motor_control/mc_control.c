#define LOG_TAG "mc_ctrl"
#include "mc_control.h"

#include <math.h>

#include "identify_parameter.h"
#include "logger.h"
#include "mc_current.h"
#include "mc_position.h"
#include "mc_speed.h"
#include "vofa_plot.h"

#define MECH_RETURN_MAX_ACCELERATION_RAD_S2 10.0f
#define MECH_RETURN_MAX_JERK_RAD_S3         50.0f

static uint32_t step_tick;
static uint8_t step_target_b;
static uint8_t position_loop_tick;
static uint8_t mechanical_return_control_active;

/**
 * @brief 判断机械位置是否超出已启用的角度限制
 * @return 1 表示超限，0 表示未超限或对应方向未启用限制
 * @note 角度值为 0 时表示该方向不限制。
 */
static uint8_t mc_angle_limit_exceeded(void)
{
    const float position = encoder_data.mechanical_position;

    if ((motor_ctrl.angle_min != 0.0f) && (position < motor_ctrl.angle_min)) {
        return 1U;
    }
    if ((motor_ctrl.angle_max != 0.0f) && (position > motor_ctrl.angle_max)) {
        return 1U;
    }
    return 0U;
}

/**
 * @brief 初始化 FOC 控制器并校准电流采样零偏
 */
void mc_start_init(void)
{
    mc_ctrl.target_speed = 0.0f;
    mc_ctrl.out_q = 0.0f;
    mc_ctrl.out_d = 0.0f;
    mc_ctrl.target_q = 0.0f;
    mc_ctrl.target_d = 0.0f;
    mc_ctrl.target_d_effective = 0.0f;
    mc_ctrl.target_q_effective = 0.0f;
    mc_ctrl.target_position = encoder_data.mechanical_position;
    mc_ctrl.shaped_t_p = encoder_data.mechanical_position;
    mc_ctrl.shaped_t_v = 0.0f;
    mc_ctrl.shaped_t_a = 0.0f;

    mc_current_control_init_from_motor();
    mc_speed_control_init(encoder_data.mechanical_speed);
    mc_position_control_init(encoder_data.mechanical_position);

    get_mc_bus_voltage();
    log_i("Vbus: %.2f V", mc_voltage_data.vbus);
    adc1_current_offset_calibrate();
    mc_start();
}

/**
 * @brief 按指定控制类型进入 RUNNING 模式
 * @param profile 电流、速度、单目标位置或位置阶跃运行模式
 * @return 1 表示启动成功，0 表示状态不允许
 */
uint8_t mc_control_start(mc_run_profile_t profile)
{
    const mc_runtime_config_t *config = mc_runtime_get_config();
    uint32_t primask;
    uint8_t started;

    if ((mc_runtime_is_ready() == 0U) ||
        (motor_ctrl.fault_type != MOTOR_FAULT_NONE) ||
        (mc_encoder_calibration_is_running() != 0U) ||
        identify_mechanical_is_active() ||
        identify_parameter_is_electrical_active()) {
        return 0U;
    }

    if ((mc_runtime_get_mode() == MC_MODE_RUNNING) &&
        (mc_runtime_get_profile() == profile) &&
        (profile != MC_RUN_POSITION_STEP) &&
        (profile != MC_RUN_POSITION) &&
        (profile != MC_RUN_POSITION_CASCADE)) {
        mc_ctrl.target_d = config->target_id;
        if (profile == MC_RUN_CURRENT) {
            mc_ctrl.target_q = config->target_iq;
        }
        else if (profile == MC_RUN_SPEED) {
            mc_ctrl.target_speed = config->target_speed;
        }
        return 1U;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    mc_start();
    mc_current_control_init_from_motor();
    mc_speed_control_reset(encoder_data.mechanical_speed);
    mc_position_control_set_shaper_bypass(0U);
    mc_position_control_set_integral_always(0U);
    mc_position_control_init(encoder_data.mechanical_position);

    mc_ctrl.target_d = config->target_id;
    mc_ctrl.target_q = (profile == MC_RUN_CURRENT) ? config->target_iq : 0.0f;
    mc_ctrl.target_speed =
        (profile == MC_RUN_SPEED) ? config->target_speed : 0.0f;
    if (profile == MC_RUN_POSITION_STEP) {
        mc_ctrl.target_position = config->step_position_a;
    }
    else if ((profile == MC_RUN_POSITION) ||
             (profile == MC_RUN_POSITION_CASCADE)) {
        mc_ctrl.target_position = config->target_position;
    }
    else {
        mc_ctrl.target_position = encoder_data.mechanical_position;
    }
    mc_ctrl.shaped_t_p = encoder_data.mechanical_position;
    mc_ctrl.shaped_t_v = 0.0f;
    mc_ctrl.shaped_t_a = 0.0f;
    mc_ctrl.out_d = 0.0f;
    mc_ctrl.out_q = 0.0f;
    mc_ctrl.target_d_effective = mc_ctrl.target_d;
    mc_ctrl.target_q_effective = mc_ctrl.target_q;

    step_tick = 0U;
    step_target_b = 0U;
    position_loop_tick = MC_POSITION_LOOP_DECIMATION - 1U;

    started = mc_runtime_start(profile);
    if (started != 0U) {
        bsp_pwm_enable_outputs();
    }
    if (primask == 0U) {
        __enable_irq();
    }
    return started;
}

/**
 * @brief 停止闭环控制并关闭功率级输出
 */
void mc_control_stop(void)
{
    const uint32_t primask = __get_PRIMASK();

    __disable_irq();
    mc_runtime_stop();
    mc_output_idle();
    if (primask == 0U) {
        __enable_irq();
    }
}

/**
 * @brief 启动机械参数辨识维护任务
 * @return 1 表示已启动，0 表示状态或硬件条件不允许
 */
uint8_t mc_control_start_mechanical_identification(void)
{
    return mc_control_start_mechanical_identification_with_amplitudes(0.100f,
                                                                      0.200f);
}

/**
 * @brief 使用指定电流幅值启动机械参数辨识
 * @param low_amplitude_a 低频段电流幅值（A）
 * @param high_amplitude_a 高频段电流幅值（A）
 * @return 1 表示已启动，0 表示状态或参数不允许
 */
uint8_t mc_control_start_mechanical_identification_with_amplitudes(
    float low_amplitude_a, float high_amplitude_a)
{
    uint32_t primask;

    if ((mc_runtime_is_ready() == 0U) ||
        (motor_ctrl.fault_type != MOTOR_FAULT_NONE) ||
        (mc_runtime_get_mode() != MC_MODE_IDLE) ||
        (mc_encoder_calibration_is_running() != 0U) ||
        identify_mechanical_is_active() ||
        identify_parameter_is_electrical_active()) {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    mc_start();
    mc_current_control_init_from_motor();
    mc_speed_control_reset(encoder_data.mechanical_speed);
    mc_position_control_set_shaper_bypass(1U);
    mc_position_control_set_integral_always(1U);
    mc_position_control_init(encoder_data.mechanical_position);
    mc_ctrl.target_d = 0.0f;
    mc_ctrl.target_q = 0.0f;
    mc_ctrl.target_d_effective = 0.0f;
    mc_ctrl.target_q_effective = 0.0f;
    mc_ctrl.out_d = 0.0f;
    mc_ctrl.out_q = 0.0f;
    mc_ctrl.target_speed = 0.0f;
    mc_ctrl.shaped_t_p = encoder_data.mechanical_position;
    mc_ctrl.shaped_t_v = 0.0f;
    mc_ctrl.shaped_t_a = 0.0f;
    position_loop_tick = MC_POSITION_LOOP_DECIMATION - 1U;
    mechanical_return_control_active = 0U;
    if (identify_mechanical_start_with_amplitudes(low_amplitude_a,
                                                  high_amplitude_a) != 0) {
        if (primask == 0U) {
            __enable_irq();
        }
        return 0U;
    }
    bsp_pwm_enable_outputs();
    if (primask == 0U) {
        __enable_irq();
    }
    return 1U;
}

/**
 * @brief 在主循环处理机械辨识结果
 */
void mc_control_background_process(void)
{
    identify_mechanical_result_t result;
    ident_mech_state_t state;

    identify_parameter_background_process();

    if (identify_mechanical_is_active()) {
        identify_mechanical_background_process();
    }

    state = identify_mechanical_get_state();
    if ((state != IDENT_MECH_DONE) && (state != IDENT_MECH_ERROR)) {
        return;
    }

    /* 先把结果复制出来，再结束任务；结果保留供 identify status 查询。 */
    result = *identify_mechanical_get_result();
    identify_mechanical_acknowledge();
    mc_output_idle();

    if (state == IDENT_MECH_ERROR) {
        log_e("identify mech failed low=%u res=%u",
              (unsigned int)result.low_frequency_error_code,
              (unsigned int)result.resonance_error_code);
        return;
    }

    if (result.low_frequency_valid) {
        log_i("IDENTIFY_MECH Ka=%.9f Kv=%.9f Tc=%.9f rms=%.2f%%",
              (double)result.ka, (double)result.kv, (double)result.tc,
              (double)result.weighted_rms_percent);
    }
    else {
        log_e("IDENTIFY_MECH low invalid code=%u",
              (unsigned int)result.low_frequency_error_code);
    }
    if (result.resonance_valid) {
        const mc_shaper_config_t *shaper = mc_runtime_get_shaper_config();

        log_i("IDENTIFY_MECH resonance f=%.2fHz zeta=%.3f Q=%.2f peak=%.1fdB",
              (double)result.resonance_frequency_hz,
              (double)result.resonance_damping_ratio,
              (double)result.resonance_quality_factor,
              (double)result.resonance_prominence_db);
        log_i("IDENTIFY_MECH shaper type=%u f=%.2fHz zeta=%.3f",
              (unsigned int)shaper->type, (double)shaper->frequency_hz,
              (double)shaper->damping_ratio);
    }
    else {
        log_e("IDENTIFY_MECH resonance invalid code=%u",
              (unsigned int)result.resonance_error_code);
    }
}

/**
 * @brief 外环节拍输出 VOFA JustFloat 波形
 */
static void mc_control_vofa_tick(void)
{
    float values[VOFA_PLOT_CHANNELS];

    if (!vofa_plot_is_enabled()) {
        return;
    }

    values[0] = mc_ctrl.shaped_t_p;
    values[1] = encoder_data.mechanical_position;
    values[2] = mc_ctrl.target_speed;
    values[3] = encoder_data.mechanical_speed;
    values[4] = mc_ctrl.target_q;
    values[5] = mc_ctrl.abc_dq.current_q;
    values[6] = mc_ctrl.target_d;
    values[7] = mc_ctrl.abc_dq.current_d;
    vofa_plot_send(values, (uint8_t)VOFA_PLOT_CHANNELS);
}

/**
 * @brief FOC外环控制主函数
 */
void mc_control(void)
{
    mc_control_vofa_tick();

    const uint8_t motion_active =
        (mc_runtime_get_mode() == MC_MODE_RUNNING) ||
        identify_parameter_is_electrical_active() ||
        identify_mechanical_is_active();

    if ((mc_encoder_calibration_is_running() == 0U) &&
        (motor_ctrl.fault_type == MOTOR_FAULT_NONE) && (motion_active != 0U) &&
        (mc_angle_limit_exceeded() != 0U)) {
        motor_ctrl.fault_type = FAULT_ENCODER_ANGLE;
        mc_runtime_stop();
        mc_output_idle();
        return;
    }

    if (identify_parameter_is_electrical_active()) {
        identify_parameter_control_tick();
        return;
    }

    if (identify_mechanical_is_active()) {
        const uint8_t returning = identify_mechanical_is_returning() ? 1U : 0U;

        identify_mechanical_process();
        if (returning != 0U) {
            if (mechanical_return_control_active == 0U) {
                mc_speed_control_reset(encoder_data.mechanical_speed);
                mc_position_control_reset(encoder_data.mechanical_position,
                                          IDENT_MECH_RETURN_VELOCITY_RAD_S,
                                          MECH_RETURN_MAX_ACCELERATION_RAD_S2,
                                          MECH_RETURN_MAX_JERK_RAD_S3);
                mc_ctrl.target_position =
                    identify_mechanical_get_start_position();
                mc_ctrl.shaped_t_p = encoder_data.mechanical_position;
                mc_ctrl.shaped_t_v = 0.0f;
                mc_ctrl.shaped_t_a = 0.0f;
                position_loop_tick = MC_POSITION_LOOP_DECIMATION - 1U;
                mechanical_return_control_active = 1U;
            }

            if (++position_loop_tick >= MC_POSITION_LOOP_DECIMATION) {
                position_loop_tick = 0U;
                /* 位置→电流单环带积分，保证能克服静摩擦回到辨识起点，
                 * 而不是停在 0.02 rad 门限之外。 */
                mc_ctrl.target_q = mc_position_control_run_current(
                    identify_mechanical_get_start_position());
            }
            identify_mechanical_return_feedback_tick(
                encoder_data.mechanical_position,
                encoder_data.mechanical_speed);
        }
        else {
            mechanical_return_control_active = 0U;
        }
        return;
    }
    mechanical_return_control_active = 0U;

    if (mc_runtime_get_mode() != MC_MODE_RUNNING) {
        return;
    }

    const mc_run_profile_t profile = mc_runtime_get_profile();

    if (profile == MC_RUN_SPEED) {
        mc_ctrl.target_q = mc_speed_control_run(mc_ctrl.target_speed, 0.0f,
                                                encoder_data.mechanical_speed);
        return;
    }
    if ((profile != MC_RUN_POSITION_STEP) && (profile != MC_RUN_POSITION) &&
        (profile != MC_RUN_POSITION_CASCADE)) {
        return;
    }

    if (++position_loop_tick >= MC_POSITION_LOOP_DECIMATION) {
        const mc_runtime_config_t *config = mc_runtime_get_config();
        position_loop_tick = 0U;
        if (profile == MC_RUN_POSITION_STEP) {
            uint32_t ticks_per_ms = (uint32_t)POSITION_LOOP_TIME / 1000U;
            uint32_t toggle_ticks;

            if (ticks_per_ms == 0U) {
                ticks_per_ms = 1U;
            }
            toggle_ticks = config->step_period_ms * ticks_per_ms;
            if (++step_tick >= toggle_ticks) {
                step_tick = 0U;
                step_target_b ^= 1U;
                mc_ctrl.target_position = (step_target_b != 0U)
                    ? config->step_position_b
                    : config->step_position_a;
            }
        }
        else {
            mc_ctrl.target_position = config->target_position;
        }

        if (profile == MC_RUN_POSITION_CASCADE) {
            mc_position_control_run(mc_ctrl.target_position);
        }
        else {
            mc_ctrl.target_q =
                mc_position_control_run_current(mc_ctrl.target_position);
        }
    }

    if (profile == MC_RUN_POSITION_CASCADE) {
        mc_ctrl.target_q =
            mc_speed_control_run(mc_ctrl.target_speed, mc_ctrl.shaped_t_a,
                                 encoder_data.mechanical_speed);
    }
}

/**
 * @brief FOC内环电流环外部调用接口
 */
void mc_current_in_control(void)
{
    if (mc_encoder_calibration_is_running() != 0U) {
        mc_encoder_calibration_open_loop_calibrate();
        return;
    }

    if (motor_ctrl.fault_type != MOTOR_FAULT_NONE) {
        mc_output_idle();
        return;
    }

    if (identify_parameter_is_electrical_active()) {
        mc_current_control_open_loop_tick();
        return;
    }

    if (identify_mechanical_is_active()) {
        mc_current_control_run(mc_ctrl.target_d, mc_ctrl.target_q, 0U);
        return;
    }

    if (mc_runtime_get_mode() != MC_MODE_RUNNING) {
        /* 上电/空闲时持续采样并计算 d/q 电流，同时保持功率级关闭。 */
        mc_output_idle();
        mc_current_control_measure();
        return;
    }
    mc_current_control_run(mc_ctrl.target_d, mc_ctrl.target_q, 1U);
}
