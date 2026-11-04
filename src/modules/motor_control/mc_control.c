#define LOG_TAG "mc_ctrl"
#include "mc_control.h"

#include <math.h>

#include "identify_parameter.h"
#include "logger.h"
#include "mc_current.h"
#include "mc_position.h"
#include "mc_speed.h"

#define SWEEP_RETURN_MAX_VELOCITY_RAD_S      2.0f
#define SWEEP_RETURN_MAX_ACCELERATION_RAD_S2 10.0f
#define SWEEP_RETURN_MAX_JERK_RAD_S3         50.0f

static uint32_t step_tick;
static uint8_t step_target_b;
static uint8_t position_loop_tick;
static uint8_t sweep_return_control_active;
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
        identify_parameter_is_electrical_active() ||
        mc_frequency_sweep_is_active()) {
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
    sweep_return_control_active = 0U;

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
    return mc_control_start_mechanical_identification_with_amplitudes(0.140f,
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
        identify_parameter_is_electrical_active() ||
        mc_frequency_sweep_is_active()) {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    mc_start();
    mc_current_control_init_from_motor();
    mc_speed_control_reset(encoder_data.mechanical_speed);
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
 * @brief 从当前机械位置启动逐频正弦扫频
 * @return 1 表示启动成功，0 表示当前状态不允许
 */
uint8_t mc_control_start_frequency_sweep(void)
{
    uint32_t primask;
    uint8_t started;

    if ((mc_runtime_is_ready() == 0U) ||
        (motor_ctrl.fault_type != MOTOR_FAULT_NONE) ||
        (mc_runtime_get_mode() != MC_MODE_IDLE) ||
        (mc_encoder_calibration_is_running() != 0U) ||
        identify_mechanical_is_active() ||
        identify_parameter_is_electrical_active() ||
        mc_frequency_sweep_is_active()) {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    mc_start();
    mc_current_control_init_from_motor();
    mc_speed_control_reset(encoder_data.mechanical_speed);
    mc_position_control_init(encoder_data.mechanical_position);
    mc_ctrl.target_d = 0.0f;
    mc_ctrl.target_q = 0.0f;
    mc_ctrl.target_d_effective = 0.0f;
    mc_ctrl.target_q_effective = 0.0f;
    mc_ctrl.target_speed = 0.0f;
    mc_ctrl.shaped_t_p = encoder_data.mechanical_position;
    mc_ctrl.shaped_t_v = 0.0f;
    mc_ctrl.shaped_t_a = 0.0f;
    position_loop_tick = MC_POSITION_LOOP_DECIMATION - 1U;
    sweep_return_control_active = 0U;
    started = mc_frequency_sweep_start();
    if (started != 0U) {
        bsp_pwm_enable_outputs();
    }
    if (primask == 0U) {
        __enable_irq();
    }
    return started;
}

/**
 * @brief 在主循环处理机械辨识结果并保存 Ka/Kv/Tc 到 FLASH
 */
void mc_control_background_process(void)
{
    const mech_identify_result_t *result;
    float ka;
    float kv;
    float tc;
    float weighted_rms;
    float low_rms;
    float high_rms;
    float median_gain_std;
    float median_phase_std;
    float ka_high_band;
    uint16_t point_count;
    uint8_t error_code;
    uint8_t result_valid;

    identify_parameter_background_process();

    if (mc_frequency_sweep_is_active()) {
        mc_frequency_sweep_background_process();
        return;
    }

    if (identify_mechanical_is_active()) {
        identify_mechanical_background_process();
    }

    if (identify_mechanical_get_state() == MECH_IDENTIFY_FAILED) {
        result = identify_mechanical_get_result();
        error_code = result->error_code;
        identify_mechanical_stop();
        mc_output_idle();
        log_e("identify mech failed code=%u", (unsigned int)error_code);
        return;
    }
    if (identify_mechanical_get_state() != MECH_IDENTIFY_COMPLETE) {
        return;
    }

    result = identify_mechanical_get_result();
    ka = result->Ka;
    kv = result->Kv;
    tc = result->Tc;
    ka_high_band = result->Ka_high_band;
    weighted_rms = result->weighted_rms_percent;
    low_rms = result->low_band_rms_percent;
    high_rms = result->high_band_rms_percent;
    median_gain_std = result->median_gain_std_percent;
    median_phase_std = result->median_phase_std_deg;
    point_count = result->point_count;
    error_code = result->error_code;
    result_valid = ((result->valid != false) && isfinite(ka) && isfinite(kv) &&
                    isfinite(tc) && isfinite(weighted_rms) && (ka > 0.0f) &&
                    (ka <= 0.1f) && (kv >= 0.0f) && (kv <= 1.0f) &&
                    (tc >= 0.0f) && (tc <= 1.0f) && (weighted_rms <= 25.0f) &&
                    (point_count == 24U) && (error_code == 0U))
        ? 1U
        : 0U;
    identify_mechanical_stop();
    mc_output_idle();

    if (result_valid == 0U) {
        log_e("identify mech result invalid");
    }
    else if (mc_runtime_set_mechanical(ka, kv, tc) != 0U) {
        log_i("IDENTIFY_MECH points=%u Ka_high=%.9f rms=%.2f low=%.2f "
              "high=%.2f gain_std=%.3f phase_std=%.3f",
              (unsigned int)point_count, ka_high_band, weighted_rms, low_rms,
              high_rms, median_gain_std, median_phase_std);
        log_i("identify mech saved Ka=%.9f Kv=%.9f Tc=%.9f", ka, kv, tc);
    }
    else {
        log_e("identify mech FLASH write failed");
    }
}

/**
 * @brief FOC外环控制主函数
 */
void mc_control(void)
{
    const uint8_t motion_active =
        (mc_runtime_get_mode() == MC_MODE_RUNNING) ||
        identify_parameter_is_electrical_active() ||
        mc_frequency_sweep_is_active() || identify_mechanical_is_active();

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

    if (mc_frequency_sweep_is_active()) {
        const uint8_t returning = mc_frequency_sweep_is_returning() ? 1U : 0U;

        mc_frequency_sweep_control_tick();
        if (returning != 0U) {
            if (sweep_return_control_active == 0U) {
                mc_speed_control_reset(encoder_data.mechanical_speed);
                mc_position_control_reset(encoder_data.mechanical_position,
                                          SWEEP_RETURN_MAX_VELOCITY_RAD_S,
                                          SWEEP_RETURN_MAX_ACCELERATION_RAD_S2,
                                          SWEEP_RETURN_MAX_JERK_RAD_S3);
                mc_ctrl.target_position =
                    mc_frequency_sweep_get_start_position();
                mc_ctrl.shaped_t_p = encoder_data.mechanical_position;
                mc_ctrl.shaped_t_v = 0.0f;
                mc_ctrl.shaped_t_a = 0.0f;
                position_loop_tick = MC_POSITION_LOOP_DECIMATION - 1U;
                sweep_return_control_active = 1U;
            }

            if (++position_loop_tick >= MC_POSITION_LOOP_DECIMATION) {
                position_loop_tick = 0U;
                mc_position_control_run(
                    mc_frequency_sweep_get_start_position());
            }
            mc_ctrl.target_q =
                mc_speed_control_run(mc_ctrl.target_speed, mc_ctrl.shaped_t_a,
                                     encoder_data.mechanical_speed);
            mc_frequency_sweep_return_feedback_tick(
                encoder_data.mechanical_position,
                encoder_data.mechanical_speed);
        }
        else {
            sweep_return_control_active = 0U;
        }
        return;
    }

    if (identify_mechanical_is_active()) {
        const uint8_t returning = identify_mechanical_is_returning() ? 1U : 0U;

        identify_mechanical_process();
        if (returning != 0U) {
            if (mechanical_return_control_active == 0U) {
                mc_speed_control_reset(encoder_data.mechanical_speed);
                mc_position_control_reset(encoder_data.mechanical_position,
                                          SWEEP_RETURN_MAX_VELOCITY_RAD_S,
                                          SWEEP_RETURN_MAX_ACCELERATION_RAD_S2,
                                          SWEEP_RETURN_MAX_JERK_RAD_S3);
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
                mc_position_control_run(
                    identify_mechanical_get_start_position());
            }
            mc_ctrl.target_q =
                mc_speed_control_run(mc_ctrl.target_speed, mc_ctrl.shaped_t_a,
                                     encoder_data.mechanical_speed);
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

    if (mc_frequency_sweep_is_active() || identify_mechanical_is_active()) {
        mc_current_control_run(mc_ctrl.target_d, mc_ctrl.target_q, 0U);
        return;
    }

    if (mc_runtime_get_mode() != MC_MODE_RUNNING) {
        mc_output_idle();
        return;
    }
    mc_current_control_run(mc_ctrl.target_d, mc_ctrl.target_q, 1U);
}
