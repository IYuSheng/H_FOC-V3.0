#define LOG_TAG "ident_r"
#include "identify_resistance.h"

#include <math.h>
#include <string.h>

#include "board_config.h"
#include "identify_inductance.h"
#include "identify_mechanical.h"
#include "logger.h"
#include "main.h"
#include "mc_current.h"
#include "mc_runtime.h"
#include "tim.h"
#include "usart.h"

#define IDENT_R_CONTROL_RATE_HZ 5000.0f
#define IDENT_R_PROBE_VOLTAGE   0.5f
#define IDENT_R_PROBE_RAMP_MS   50U
#define IDENT_R_PROBE_SETTLE_MS 100U
#define IDENT_R_PROBE_SAMPLE_MS 50U
#define IDENT_R_POINT_SETTLE_MS 50U
#define IDENT_R_POINT_SAMPLE_MS 100U
#define IDENT_R_PHASE_SWITCH_MS 400U
#define IDENT_R_DISCARD_MS      10U
#define IDENT_R_PHASE_ZERO_MS   1U
#define IDENT_R_PHASE_SELECT_MS 2U
#define IDENT_R_VCMD_MAX_ABS    8.0f
#define IDENT_R_R_MIN           0.1f
#define IDENT_R_R_MAX           50.0f
#define IDENT_R_SYMMETRY_WARN   0.15f
#define IDENT_R_SYMMETRY_REJECT 0.30f

static const float ident_r_current_targets[IDENT_R_NUM_POINTS] = {
    0.4f, 0.6f, 0.8f, 1.0f, 1.2f, 1.4f};

static identify_resistance_t s_resistance;

/**
 * @brief 判断状态计时是否达到指定毫秒数
 * @param ticks 当前状态计数值
 * @param milliseconds 目标时间（ms）
 * @return 1 表示已到达，0 表示未到达
 */
static uint8_t ident_r_elapsed_ms(uint32_t ticks, uint32_t milliseconds)
{
    const float elapsed_ms = (float)ticks * (1000.0f / IDENT_R_CONTROL_RATE_HZ);

    return (elapsed_ms >= (float)milliseconds) ? 1U : 0U;
}

/**
 * @brief 获取指定相的固定开环电角度
 * @param phase_idx 相索引，0=A，1=B
 * @return 开环电角度（rad）
 */
static float ident_r_phase_angle(uint8_t phase_idx)
{
    return (phase_idx == 0U) ? 0.0f : _PI_2;
}

/**
 * @brief 获取指定相的有符号采样电流
 * @param phase_idx 相索引，0=A，1=B
 * @return 相电流（A）
 */
static float ident_r_phase_current(uint8_t phase_idx)
{
    return (phase_idx == 0U) ? mc_current_data.IA : mc_current_data.IB;
}

/**
 * @brief 对电压和电流样本执行一次线性最小二乘拟合
 * @param current 电流数组（A）
 * @param voltage 电压数组（V）
 * @param count 样本数
 * @param resistance 输出电阻斜率（ohm）
 * @param offset 输出电压截距（V）
 * @return 0 表示成功，负值表示矩阵奇异
 */
static int8_t ident_r_lsq_fit(const float *current, const float *voltage,
                              uint32_t count, float *resistance, float *offset)
{
    float sum_i = 0.0f;
    float sum_v = 0.0f;
    float sum_ii = 0.0f;
    float sum_iv = 0.0f;
    float denominator;

    for (uint32_t index = 0U; index < count; index++) {
        sum_i += current[index];
        sum_v += voltage[index];
        sum_ii += current[index] * current[index];
        sum_iv += current[index] * voltage[index];
    }

    denominator = ((float)count * sum_ii) - (sum_i * sum_i);
    if (fabsf(denominator) < 1.0e-3f) {
        return -1;
    }

    *resistance = (((float)count * sum_iv) - (sum_i * sum_v)) / denominator;
    *offset = (sum_v - (*resistance * sum_i)) / (float)count;
    return 0;
}

/**
 * @brief 将电阻辨识切换到错误状态
 * @param reason 错误原因
 */
static void ident_r_enter_error(const char *reason)
{
    log_e("identify res %s", reason);
    mc_current_control_set_open_loop(0.0f, 0.0f, 0.0f);
    s_resistance.state = IDENT_R_ERROR;
    s_resistance.timer_ticks = 0U;
}

/**
 * @brief 检查硬件故障
 * @return 1 表示存在故障，0 表示正常
 */
static uint8_t ident_r_faulted(void)
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
 * @brief 初始化电阻辨识模块
 * @return 0 表示成功
 */
int8_t identify_resistance_init(void)
{
    memset(&s_resistance, 0, sizeof(s_resistance));
    s_resistance.state = IDENT_R_IDLE;
    return 0;
}

/**
 * @brief 启动电阻辨识
 * @return 0 表示成功，负值表示状态不允许
 */
int8_t identify_resistance_start(void)
{
    const ident_l_state_t inductance_state = identify_inductance_get()->state;

    if (identify_mechanical_is_active() ||
        ((inductance_state != IDENT_L_IDLE) &&
         (inductance_state != IDENT_L_DONE) &&
         (inductance_state != IDENT_L_ERROR))) {
        log_w("identify already running");
        return -1;
    }
    if ((mc_runtime_is_ready() == 0U) ||
        (mc_runtime_get_mode() != MC_MODE_IDLE) || (ident_r_faulted() != 0U)) {
        log_w("identify res not ready");
        return -1;
    }

    mc_start();
    memset(&s_resistance, 0, sizeof(s_resistance));
    s_resistance.state = IDENT_R_INIT;
    mc_current_control_set_open_loop(0.0f, 0.0f, 0.0f);
    bsp_pwm_enable_outputs();
    log_i("identify res started");
    return 0;
}

/**
 * @brief 在 5 kHz 控制节拍推进电阻辨识
 */
void identify_resistance_control_tick(void)
{
    float elapsed_ms;
    float fraction;
    float current_average;
    float voltage_average;
    float resistance_average;
    float resistance_difference;
    float resistance_difference_ratio;
    float voltage_cap;
    float phase_angle;
    float current_target;

    if ((s_resistance.state == IDENT_R_IDLE) ||
        (s_resistance.state == IDENT_R_DONE) ||
        (s_resistance.state == IDENT_R_ERROR)) {
        return;
    }

    s_resistance.timer_ticks++;
    if ((s_resistance.state != IDENT_R_INIT) && (ident_r_faulted() != 0U)) {
        ident_r_enter_error("fault or overcurrent");
        return;
    }

    phase_angle = ident_r_phase_angle(s_resistance.phase_idx);
    switch (s_resistance.state) {
        case IDENT_R_INIT:
            s_resistance.phase_idx = 0U;
            s_resistance.point_idx = 0U;
            s_resistance.voltage_now = 0.0f;
            mc_current_control_set_open_loop(phase_angle, 0.0f, 0.0f);
            s_resistance.state = IDENT_R_PROBE_RAMP;
            s_resistance.timer_ticks = 0U;
            break;

        case IDENT_R_PROBE_RAMP:
            elapsed_ms = (float)s_resistance.timer_ticks *
                (1000.0f / IDENT_R_CONTROL_RATE_HZ);
            fraction = elapsed_ms / (float)IDENT_R_PROBE_RAMP_MS;
            if (fraction > 1.0f) {
                fraction = 1.0f;
            }
            s_resistance.voltage_now = IDENT_R_PROBE_VOLTAGE * fraction;
            mc_current_control_set_open_loop(phase_angle,
                                             s_resistance.voltage_now, 0.0f);
            if (ident_r_elapsed_ms(s_resistance.timer_ticks,
                                   IDENT_R_PROBE_RAMP_MS) != 0U) {
                s_resistance.state = IDENT_R_PROBE_SETTLE;
                s_resistance.timer_ticks = 0U;
            }
            break;

        case IDENT_R_PROBE_SETTLE:
            mc_current_control_set_open_loop(phase_angle,
                                             s_resistance.voltage_now, 0.0f);
            if (ident_r_elapsed_ms(s_resistance.timer_ticks,
                                   IDENT_R_PROBE_SETTLE_MS) != 0U) {
                s_resistance.current_accum = 0.0f;
                s_resistance.sample_count = 0U;
                s_resistance.state = IDENT_R_PROBE_SAMPLE;
                s_resistance.timer_ticks = 0U;
            }
            break;

        case IDENT_R_PROBE_SAMPLE:
            mc_current_control_set_open_loop(phase_angle,
                                             s_resistance.voltage_now, 0.0f);
            s_resistance.current_accum +=
                fabsf(ident_r_phase_current(s_resistance.phase_idx));
            s_resistance.sample_count++;
            if (ident_r_elapsed_ms(s_resistance.timer_ticks,
                                   IDENT_R_PROBE_SAMPLE_MS) == 0U) {
                break;
            }
            if (s_resistance.sample_count == 0U) {
                ident_r_enter_error("no probe samples");
                return;
            }
            current_average =
                s_resistance.current_accum / (float)s_resistance.sample_count;
            if (current_average < 1.0e-4f) {
                ident_r_enter_error("zero current");
                return;
            }
            s_resistance.r_estimate = IDENT_R_PROBE_VOLTAGE / current_average;
            if ((s_resistance.r_estimate < IDENT_R_R_MIN) ||
                (s_resistance.r_estimate > IDENT_R_R_MAX)) {
                ident_r_enter_error("resistance out of range");
                return;
            }

            voltage_cap = IDENT_R_VCMD_MAX_ABS;
            if ((mc_voltage_data.vbus * 0.5f) < voltage_cap) {
                voltage_cap = mc_voltage_data.vbus * 0.5f;
            }
            s_resistance.voltage_cap = voltage_cap;
            s_resistance.point_idx = 0U;
            s_resistance.state = IDENT_R_POINT_SETTLE;
            s_resistance.timer_ticks = 0U;
            break;

        case IDENT_R_POINT_SETTLE:
            current_target = ident_r_current_targets[s_resistance.point_idx];
            s_resistance.voltage_now = s_resistance.r_estimate * current_target;
            if (s_resistance.voltage_now > s_resistance.voltage_cap) {
                s_resistance.voltage_now = s_resistance.voltage_cap;
            }
            mc_current_control_set_open_loop(phase_angle,
                                             s_resistance.voltage_now, 0.0f);
            if (ident_r_elapsed_ms(s_resistance.timer_ticks,
                                   IDENT_R_POINT_SETTLE_MS) != 0U) {
                s_resistance.current_accum = 0.0f;
                s_resistance.voltage_accum = 0.0f;
                s_resistance.sample_count = 0U;
                s_resistance.state = IDENT_R_POINT_SAMPLE;
                s_resistance.timer_ticks = 0U;
            }
            break;

        case IDENT_R_POINT_SAMPLE:
            mc_current_control_set_open_loop(phase_angle,
                                             s_resistance.voltage_now, 0.0f);
            if (ident_r_elapsed_ms(s_resistance.timer_ticks,
                                   IDENT_R_DISCARD_MS) == 0U) {
                s_resistance.current_accum +=
                    ident_r_phase_current(s_resistance.phase_idx);
                s_resistance.voltage_accum += s_resistance.voltage_now;
                s_resistance.sample_count++;
            }
            if (ident_r_elapsed_ms(s_resistance.timer_ticks,
                                   IDENT_R_POINT_SAMPLE_MS) == 0U) {
                break;
            }
            if (s_resistance.sample_count == 0U) {
                ident_r_enter_error("no point samples");
                return;
            }
            current_average =
                s_resistance.current_accum / (float)s_resistance.sample_count;
            voltage_average =
                s_resistance.voltage_accum / (float)s_resistance.sample_count;
            s_resistance
                .current_meas[s_resistance.phase_idx][s_resistance.point_idx] =
                current_average;
            s_resistance
                .voltage_meas[s_resistance.phase_idx][s_resistance.point_idx] =
                voltage_average;
            s_resistance.point_idx++;
            if (s_resistance.point_idx < IDENT_R_NUM_POINTS) {
                s_resistance.state = IDENT_R_POINT_SETTLE;
            }
            else if (s_resistance.phase_idx == 0U) {
                s_resistance.state = IDENT_R_PHASE_SWITCH;
            }
            else {
                s_resistance.state = IDENT_R_COMPUTE;
            }
            s_resistance.timer_ticks = 0U;
            break;

        case IDENT_R_PHASE_SWITCH:
            if (s_resistance.phase_idx == 0U) {
                if (ident_r_elapsed_ms(s_resistance.timer_ticks,
                                       IDENT_R_PHASE_SELECT_MS) != 0U) {
                    s_resistance.phase_idx = 1U;
                    mc_current_control_set_open_loop(_PI_2, 0.0f, 0.0f);
                }
                else if (ident_r_elapsed_ms(s_resistance.timer_ticks,
                                            IDENT_R_PHASE_ZERO_MS) != 0U) {
                    s_resistance.voltage_now = 0.0f;
                    mc_current_control_set_open_loop(0.0f, 0.0f, 0.0f);
                }
            }
            if (ident_r_elapsed_ms(s_resistance.timer_ticks,
                                   IDENT_R_PHASE_SWITCH_MS) != 0U) {
                s_resistance.point_idx = 0U;
                s_resistance.state = IDENT_R_POINT_SETTLE;
                s_resistance.timer_ticks = 0U;
            }
            break;

        case IDENT_R_COMPUTE:
            mc_current_control_set_open_loop(0.0f, 0.0f, 0.0f);
            for (uint8_t phase = 0U; phase < 2U; phase++) {
                if (ident_r_lsq_fit(s_resistance.current_meas[phase],
                                    s_resistance.voltage_meas[phase],
                                    IDENT_R_NUM_POINTS,
                                    &s_resistance.r_phase[phase],
                                    &s_resistance.v_offset[phase]) != 0) {
                    ident_r_enter_error("LSQ singular");
                    return;
                }
                if ((s_resistance.r_phase[phase] < IDENT_R_R_MIN) ||
                    (s_resistance.r_phase[phase] > IDENT_R_R_MAX)) {
                    ident_r_enter_error("phase resistance out of range");
                    return;
                }
            }

            resistance_average =
                0.5f * (s_resistance.r_phase[0] + s_resistance.r_phase[1]);
            resistance_difference =
                fabsf(s_resistance.r_phase[0] - s_resistance.r_phase[1]);
            resistance_difference_ratio = (resistance_average > 1.0e-6f)
                ? resistance_difference / resistance_average
                : 1.0f;
            if (resistance_difference_ratio > IDENT_R_SYMMETRY_REJECT) {
                ident_r_enter_error("phase asymmetry > 30%");
                return;
            }
            if (resistance_difference_ratio > IDENT_R_SYMMETRY_WARN) {
                log_w("identify res asymmetry %.1f%%",
                      resistance_difference_ratio * 100.0f);
            }

            s_resistance.r_final = resistance_average;
            s_resistance.state = IDENT_R_SAVE;
            s_resistance.timer_ticks = 0U;
            break;

        case IDENT_R_SAVE:
            mc_current_control_set_open_loop(0.0f, 0.0f, 0.0f);
            break;

        default:
            ident_r_enter_error("invalid state");
            break;
    }
}

/**
 * @brief 在主循环保存电阻辨识结果
 */
void identify_resistance_background_process(void)
{
    if (s_resistance.state == IDENT_R_SAVE) {
        mc_current_control_set_open_loop(0.0f, 0.0f, 0.0f);
        if (mc_runtime_set_motor_resistance(s_resistance.r_final) != 0U) {
            log_i("identify res %.4fohm saved", s_resistance.r_final);
            s_resistance.state = IDENT_R_DONE;
        }
        else {
            ident_r_enter_error("FLASH save failed");
        }
    }
}

/**
 * @brief 停止电阻辨识
 */
void identify_resistance_stop(void)
{
    if ((s_resistance.state == IDENT_R_IDLE) ||
        (s_resistance.state == IDENT_R_DONE) ||
        (s_resistance.state == IDENT_R_ERROR)) {
        return;
    }
    s_resistance.state = IDENT_R_IDLE;
    s_resistance.timer_ticks = 0U;
    mc_current_control_set_open_loop(0.0f, 0.0f, 0.0f);
}

/**
 * @brief 获取电阻辨识上下文
 * @return 只读上下文指针
 */
identify_resistance_t *identify_resistance_get(void)
{
    return &s_resistance;
}
