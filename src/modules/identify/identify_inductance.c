#define LOG_TAG "ident_l"
#include "identify_inductance.h"

#include <math.h>
#include <string.h>

#include "board_config.h"
#include "identify_mechanical.h"
#include "identify_resistance.h"
#include "logger.h"
#include "main.h"
#include "mc_current.h"
#include "mc_runtime.h"
#include "tim.h"
#include "usart.h"

#define IDENT_L_CONTROL_RATE_HZ 5000.0f
#define IDENT_L_FREQ_HZ         200.0f
#define IDENT_L_INJECT_MS       400U
#define IDENT_L_ALIGN_CURRENT   0.6f
#define IDENT_L_ALIGN_MS        150U
#define IDENT_L_HOLD_CURRENT    0.5f
#define IDENT_L_V_PROBE         0.5f
#define IDENT_L_PROBE_MS        200U
#define IDENT_L_I_TARGET        1.0f
#define IDENT_L_HOLD_MS         100U
#define IDENT_L_PHASE_SWITCH_MS 300U
#define IDENT_L_PHASE_ZERO_MS   1U
#define IDENT_L_PHASE_SELECT_MS 2U
#define IDENT_L_V_INJECT        2.5f
#define IDENT_L_L_MIN           0.00002f
#define IDENT_L_L_MAX           0.200f
#define IDENT_L_SYMMETRY_WARN   0.20f
#define IDENT_L_SYMMETRY_REJECT 0.40f
#define IDENT_L_SAFETY_CURRENT  2.0f

static identify_inductance_t s_inductance;

/**
 * @brief 判断状态计时是否达到指定毫秒数
 * @param ticks 当前状态计数值
 * @param milliseconds 目标时间（ms）
 * @return 1 表示已到达，0 表示未到达
 */
static uint8_t ident_l_elapsed_ms(uint32_t ticks, uint32_t milliseconds)
{
    const float elapsed_ms = (float)ticks * (1000.0f / IDENT_L_CONTROL_RATE_HZ);

    return (elapsed_ms >= (float)milliseconds) ? 1U : 0U;
}

/**
 * @brief 获取指定相的固定开环电角度
 * @param phase_idx 相索引，0=A，1=B
 * @return 开环电角度（rad）
 */
static float ident_l_phase_angle(uint8_t phase_idx)
{
    return (phase_idx == 0U) ? 0.0f : _PI_2;
}

/**
 * @brief 获取指定相电流
 * @param phase_idx 相索引，0=A，1=B
 * @return 相电流（A）
 */
static float ident_l_phase_current(uint8_t phase_idx)
{
    return (phase_idx == 0U) ? mc_current_data.IA : mc_current_data.IB;
}

/**
 * @brief 在一个控制节拍内累加锁相检测结果
 * @param phase_omega_t 当前注入相位（rad）
 */
static void ident_l_lockin_accumulate(float phase_omega_t)
{
    const float compensated_phase = phase_omega_t - s_inductance.zoh_half;
    const float sin_reference = sinf(compensated_phase);
    const float cos_reference = cosf(compensated_phase);
    const float current = ident_l_phase_current(s_inductance.phase_idx);

    s_inductance.accum_i_sin += current * sin_reference;
    s_inductance.accum_i_cos += current * cos_reference;
    s_inductance.accum_cnt++;
}

/**
 * @brief 将电感辨识切换到错误状态
 * @param reason 错误原因
 */
static void ident_l_enter_error(const char *reason)
{
    log_e("identify ind %s", reason);
    mc_current_control_set_open_loop(0.0f, 0.0f, 0.0f);
    s_inductance.state = IDENT_L_ERROR;
    s_inductance.timer_ticks = 0U;
}

/**
 * @brief 检查硬件故障
 * @return 1 表示存在故障，0 表示正常
 */
static uint8_t ident_l_faulted(void)
{
    if (motor_ctrl.fault_type != MOTOR_FAULT_NONE) {
        return 1U;
    }
    if ((fabsf(mc_current_data.IA) > CURRENT_LIMIT) ||
        (fabsf(mc_current_data.IB) > CURRENT_LIMIT)) {
        return 1U;
    }
    return 0U;
}

/**
 * @brief 初始化电感辨识模块
 * @return 0 表示成功
 */
int8_t identify_inductance_init(void)
{
    memset(&s_inductance, 0, sizeof(s_inductance));
    s_inductance.state = IDENT_L_IDLE;
    return 0;
}

/**
 * @brief 启动电感辨识
 * @return 0 表示成功，负值表示状态不允许
 */
int8_t identify_inductance_start(void)
{
    const ident_r_state_t resistance_state = identify_resistance_get()->state;

    if (identify_mechanical_is_active() ||
        ((resistance_state != IDENT_R_IDLE) &&
         (resistance_state != IDENT_R_DONE) &&
         (resistance_state != IDENT_R_ERROR))) {
        log_w("identify already running");
        return -1;
    }
    if ((mc_runtime_is_ready() == 0U) ||
        (mc_runtime_get_mode() != MC_MODE_IDLE) || (ident_l_faulted() != 0U)) {
        log_w("identify ind not ready");
        return -1;
    }

    mc_start();
    memset(&s_inductance, 0, sizeof(s_inductance));
    s_inductance.r_known = mc_runtime_get_motor_config()->resistance;
    s_inductance.freq_hz = IDENT_L_FREQ_HZ;
    s_inductance.omega = _2PI * IDENT_L_FREQ_HZ;
    s_inductance.state = IDENT_L_INIT;
    mc_current_control_set_open_loop(0.0f, 0.0f, 0.0f);
    bsp_pwm_enable_outputs();
    log_i("identify ind started f=%.0fHz", s_inductance.freq_hz);
    return 0;
}

/**
 * @brief 在 5 kHz 控制节拍推进电感辨识
 */
void identify_inductance_control_tick(void)
{
    float half_bus_voltage;
    float phase_angle;
    float time_s;
    float phase_omega_t;
    float voltage_d;
    float sample_count;
    float current_real;
    float current_imag;
    float current_magnitude_squared;
    float current_magnitude;
    float impedance_magnitude;
    float voltage_safety;
    float voltage_command;
    float impedance_real;
    float impedance_imag;
    float inductance_measured;
    float zoh_sinc;
    float inductance_average;
    float inductance_difference;
    float inductance_difference_ratio;

    if ((s_inductance.state == IDENT_L_IDLE) ||
        (s_inductance.state == IDENT_L_DONE) ||
        (s_inductance.state == IDENT_L_ERROR)) {
        return;
    }

    s_inductance.timer_ticks++;
    if ((s_inductance.state != IDENT_L_INIT) && (ident_l_faulted() != 0U)) {
        ident_l_enter_error("fault or overcurrent");
        return;
    }

    phase_angle = ident_l_phase_angle(s_inductance.phase_idx);
    switch (s_inductance.state) {
        case IDENT_L_INIT:
            half_bus_voltage = mc_voltage_data.vbus * 0.45f;
            s_inductance.zoh_half =
                s_inductance.omega * (0.5f / IDENT_L_CONTROL_RATE_HZ);
            s_inductance.v_dc_bias =
                IDENT_L_HOLD_CURRENT * s_inductance.r_known;
            if (s_inductance.v_dc_bias < 0.3f) {
                s_inductance.v_dc_bias = 0.3f;
            }
            else if (s_inductance.v_dc_bias > 1.5f) {
                s_inductance.v_dc_bias = 1.5f;
            }
            if ((s_inductance.v_dc_bias + IDENT_L_V_INJECT) >
                half_bus_voltage) {
                s_inductance.v_dc_bias = half_bus_voltage - IDENT_L_V_INJECT;
                if (s_inductance.v_dc_bias < 0.0f) {
                    s_inductance.v_dc_bias = 0.0f;
                }
                else if (s_inductance.v_dc_bias > 1.5f) {
                    s_inductance.v_dc_bias = 1.5f;
                }
            }
            s_inductance.phase_idx = 0U;
            s_inductance.state = IDENT_L_ALIGN;
            s_inductance.timer_ticks = 0U;
            break;

        case IDENT_L_ALIGN:
            mc_current_control_set_open_loop(phase_angle, IDENT_L_ALIGN_CURRENT,
                                             0.0f);
            if (ident_l_elapsed_ms(s_inductance.timer_ticks,
                                   IDENT_L_ALIGN_MS) != 0U) {
                s_inductance.state = IDENT_L_DISCHARGE;
                s_inductance.timer_ticks = 0U;
            }
            break;

        case IDENT_L_DISCHARGE:
            mc_current_control_set_open_loop(phase_angle,
                                             s_inductance.v_dc_bias, 0.0f);
            if (ident_l_elapsed_ms(s_inductance.timer_ticks, IDENT_L_HOLD_MS) !=
                0U) {
                s_inductance.accum_i_sin = 0.0f;
                s_inductance.accum_i_cos = 0.0f;
                s_inductance.accum_cnt = 0U;
                s_inductance.state = IDENT_L_PROBE;
                s_inductance.timer_ticks = 0U;
            }
            break;

        case IDENT_L_PROBE:
            time_s = (float)s_inductance.timer_ticks / IDENT_L_CONTROL_RATE_HZ;
            phase_omega_t = s_inductance.omega * time_s;
            voltage_d = s_inductance.v_dc_bias +
                (IDENT_L_V_PROBE * sinf(phase_omega_t));
            mc_current_control_set_open_loop(phase_angle, voltage_d, 0.0f);
            ident_l_lockin_accumulate(phase_omega_t);
            if (ident_l_elapsed_ms(s_inductance.timer_ticks,
                                   IDENT_L_PROBE_MS) == 0U) {
                break;
            }
            mc_current_control_set_open_loop(phase_angle, 0.0f, 0.0f);
            if (s_inductance.accum_cnt < 20U) {
                ident_l_enter_error("too few probe samples");
                return;
            }
            sample_count = (float)s_inductance.accum_cnt;
            current_real = (2.0f * s_inductance.accum_i_sin) / sample_count;
            current_imag = (2.0f * s_inductance.accum_i_cos) / sample_count;
            current_magnitude_squared =
                (current_real * current_real) + (current_imag * current_imag);
            if (current_magnitude_squared < 1.0e-12f) {
                ident_l_enter_error("zero current");
                return;
            }
            current_magnitude = sqrtf(current_magnitude_squared);
            impedance_magnitude = IDENT_L_V_PROBE / current_magnitude;
            if (impedance_magnitude < s_inductance.r_known) {
                impedance_magnitude = s_inductance.r_known;
            }
            voltage_safety =
                IDENT_L_SAFETY_CURRENT * 0.9f * s_inductance.r_known;
            voltage_command = IDENT_L_I_TARGET * impedance_magnitude;
            if (voltage_command < 0.5f) {
                voltage_command = 0.5f;
            }
            if (voltage_command > voltage_safety) {
                voltage_command = voltage_safety;
            }
            if (voltage_command > (mc_voltage_data.vbus * 0.45f)) {
                voltage_command = mc_voltage_data.vbus * 0.45f;
            }
            s_inductance.v_m = voltage_command;
            s_inductance.accum_i_sin = 0.0f;
            s_inductance.accum_i_cos = 0.0f;
            s_inductance.accum_cnt = 0U;
            s_inductance.state = IDENT_L_INJECT;
            s_inductance.timer_ticks = 0U;
            break;

        case IDENT_L_INJECT:
            time_s = (float)s_inductance.timer_ticks / IDENT_L_CONTROL_RATE_HZ;
            phase_omega_t = s_inductance.omega * time_s;
            voltage_d = s_inductance.v_dc_bias +
                (s_inductance.v_m * sinf(phase_omega_t));
            mc_current_control_set_open_loop(phase_angle, voltage_d, 0.0f);
            ident_l_lockin_accumulate(phase_omega_t);
            if (ident_l_elapsed_ms(s_inductance.timer_ticks,
                                   IDENT_L_INJECT_MS) != 0U) {
                mc_current_control_set_open_loop(phase_angle, 0.0f, 0.0f);
                s_inductance.state = IDENT_L_FIT;
                s_inductance.timer_ticks = 0U;
            }
            break;

        case IDENT_L_FIT:
            if (s_inductance.accum_cnt < 100U) {
                ident_l_enter_error("too few injection samples");
                return;
            }
            sample_count = (float)s_inductance.accum_cnt;
            current_real = (2.0f * s_inductance.accum_i_sin) / sample_count;
            current_imag = (2.0f * s_inductance.accum_i_cos) / sample_count;
            current_magnitude_squared =
                (current_real * current_real) + (current_imag * current_imag);
            if (current_magnitude_squared < 1.0e-12f) {
                ident_l_enter_error("zero current");
                return;
            }
            current_magnitude = sqrtf(current_magnitude_squared);
            impedance_magnitude = s_inductance.v_m / current_magnitude;
            impedance_real =
                (s_inductance.v_m * current_real) / current_magnitude_squared;
            impedance_imag =
                (-s_inductance.v_m * current_imag) / current_magnitude_squared;
            inductance_measured = impedance_imag / s_inductance.omega;
            zoh_sinc = (s_inductance.zoh_half < 1.0e-6f)
                ? 1.0f
                : sinf(s_inductance.zoh_half) / s_inductance.zoh_half;
            inductance_measured *= zoh_sinc;
            if (inductance_measured < IDENT_L_L_MIN) {
                inductance_measured = IDENT_L_L_MIN;
            }
            else if (inductance_measured > IDENT_L_L_MAX) {
                inductance_measured = IDENT_L_L_MAX;
            }
            s_inductance.l_phase[s_inductance.phase_idx] = inductance_measured;
            log_i("IDENTIFY_L phase=%u L=%.6fH R=%.4fohm Z=%.4fohm",
                  s_inductance.phase_idx,
                  s_inductance.l_phase[s_inductance.phase_idx],
                  impedance_real, impedance_magnitude);
            if (s_inductance.phase_idx == 0U) {
                s_inductance.state = IDENT_L_PHASE_SWITCH;
            }
            else {
                s_inductance.state = IDENT_L_COMPUTE;
            }
            s_inductance.timer_ticks = 0U;
            break;

        case IDENT_L_PHASE_SWITCH:
            if (s_inductance.phase_idx == 0U) {
                if (ident_l_elapsed_ms(s_inductance.timer_ticks,
                                       IDENT_L_PHASE_SELECT_MS) != 0U) {
                    s_inductance.phase_idx = 1U;
                    mc_current_control_set_open_loop(_PI_2, 0.0f, 0.0f);
                }
                else if (ident_l_elapsed_ms(s_inductance.timer_ticks,
                                            IDENT_L_PHASE_ZERO_MS) != 0U) {
                    mc_current_control_set_open_loop(0.0f, 0.0f, 0.0f);
                }
            }
            if (ident_l_elapsed_ms(s_inductance.timer_ticks,
                                   IDENT_L_PHASE_SWITCH_MS) != 0U) {
                s_inductance.state = IDENT_L_ALIGN;
                s_inductance.timer_ticks = 0U;
            }
            break;

        case IDENT_L_COMPUTE:
            inductance_average =
                0.5f * (s_inductance.l_phase[0] + s_inductance.l_phase[1]);
            inductance_difference =
                fabsf(s_inductance.l_phase[0] - s_inductance.l_phase[1]);
            inductance_difference_ratio = (inductance_average > 1.0e-9f)
                ? inductance_difference / inductance_average
                : 1.0f;
            if (inductance_difference_ratio > IDENT_L_SYMMETRY_REJECT) {
                ident_l_enter_error("phase asymmetry > 40%");
                return;
            }
            if (inductance_difference_ratio > IDENT_L_SYMMETRY_WARN) {
                log_w("identify ind asymmetry %.1f%%",
                      inductance_difference_ratio * 100.0f);
            }
            s_inductance.l_final = inductance_average;
            s_inductance.state = IDENT_L_SAVE;
            s_inductance.timer_ticks = 0U;
            break;

        case IDENT_L_SAVE:
            mc_current_control_set_open_loop(0.0f, 0.0f, 0.0f);
            break;

        default:
            ident_l_enter_error("invalid state");
            break;
    }
}

/**
 * @brief 在主循环保存电感辨识结果
 */
void identify_inductance_background_process(void)
{
    if (s_inductance.state == IDENT_L_SAVE) {
        mc_current_control_set_open_loop(0.0f, 0.0f, 0.0f);
        if (mc_runtime_set_motor_inductance(s_inductance.l_final) != 0U) {
            log_i("identify ind %.6fH saved", s_inductance.l_final);
            s_inductance.state = IDENT_L_DONE;
        }
        else {
            ident_l_enter_error("FLASH save failed");
        }
    }
}

/**
 * @brief 停止电感辨识
 */
void identify_inductance_stop(void)
{
    if ((s_inductance.state == IDENT_L_IDLE) ||
        (s_inductance.state == IDENT_L_DONE) ||
        (s_inductance.state == IDENT_L_ERROR)) {
        return;
    }
    s_inductance.state = IDENT_L_IDLE;
    s_inductance.timer_ticks = 0U;
    mc_current_control_set_open_loop(0.0f, 0.0f, 0.0f);
}

/**
 * @brief 获取电感辨识上下文
 * @return 只读上下文指针
 */
identify_inductance_t *identify_inductance_get(void)
{
    return &s_inductance;
}
