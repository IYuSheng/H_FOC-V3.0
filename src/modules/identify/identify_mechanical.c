#include "identify_mechanical.h"

#include <math.h>
#include <string.h>

#include "board_config.h"
#include "data_types.h"
#include "identify_inductance.h"
#include "identify_resistance.h"
#include "mc_control.h"
#include "mc_encoder.h"
#include "mc_input_shaper.h"
#include "mc_position.h"
#include "mc_runtime.h"
#include "stm32g4xx.h"

#define LOG_TAG "ident_M"
#include "logger.h"

#define IDENT_MECH_SETTLE_TIME_S             5.0f
#define IDENT_MECH_LS_BASIS_COUNT            4U
#define IDENT_MECH_LS_PIVOT_MIN              1.0e-6f
#define IDENT_MECH_MAX_WINDOW_COUNT          3U
#define IDENT_MECH_FRICTION_SEED_MAX_HZ      2.0f
#define IDENT_MECH_RETURN_POSITION_ERROR_RAD 0.02f
#define IDENT_MECH_RETURN_SPEED_ABORT_RAD_S  1.0f
#define IDENT_MECH_RETURN_DELAY_S            0.5f
#define IDENT_MECH_RETURN_STABLE_TIME_S      1.0f
#define IDENT_MECH_RETURN_TIMEOUT_MIN_S      15.0f
#define IDENT_MECH_RETURN_TIMEOUT_MAX_S      120.0f
#define IDENT_MECH_INPUT_AMPLITUDE_MIN_A     1.0e-5f
#define IDENT_MECH_TC_MAX_A                  1.0f

#define IDENT_MECH_QUALITY_GAIN_STD_MAX_PERCENT 7.0f
#define IDENT_MECH_QUALITY_PHASE_STD_MAX_DEG    2.0f
#define IDENT_MECH_QUALITY_MEDIAN_GAIN_MAX      3.0f
#define IDENT_MECH_QUALITY_MEDIAN_PHASE_MAX     1.0f
#define IDENT_MECH_QUALITY_SIGMA_FLOOR          0.01f
#define IDENT_MECH_FIT_RMS_MAX_PERCENT          25.0f
#define IDENT_MECH_REPEATABILITY_MAX_RETRIES    2U

#define IDENT_MECH_RES_START_HZ          30.0f
#define IDENT_MECH_RES_STOP_HZ           150.0f
#define IDENT_MECH_RES_POINT_COUNT       24U
#define IDENT_MECH_RES_CURRENT_A         0.7f
#define IDENT_MECH_RES_SETTLE_CYCLES     10U
#define IDENT_MECH_RES_MEASURE_CYCLES    60U
#define IDENT_MECH_RES_DISCARD_CYCLES    10U
#define IDENT_MECH_RES_GAP_TIME_S        0.25f
#define IDENT_MECH_RES_MIN_PROMINENCE_DB 1.0f
#define IDENT_MECH_RES_MIN_CURRENT_A     0.05f
#define IDENT_MECH_RES_MIN_Q             1.5f
#define IDENT_MECH_RES_MAX_Q             50.0f
#define IDENT_MECH_RES_MIN_DAMPING       0.02f
#define IDENT_MECH_RES_MAX_DAMPING       0.12f
#define IDENT_MECH_RES_DEFAULT_DAMPING   0.05f

#define IDENT_MECH_NM_MAX_DIMENSION       3U
#define IDENT_MECH_NM_MAX_VERTICES        (IDENT_MECH_NM_MAX_DIMENSION + 1U)
#define IDENT_MECH_NM_MAX_ITERATIONS      800U
#define IDENT_MECH_NM_INITIAL_STEP        0.15f
#define IDENT_MECH_NM_PARAMETER_TOLERANCE 1.0e-5f
#define IDENT_MECH_NM_COST_TOLERANCE      1.0e-7f
#define IDENT_MECH_INVALID_COST           1.0e6f

#define IDENT_MECH_ERROR_NONE           0U
#define IDENT_MECH_ERROR_CAPTURE        1U
#define IDENT_MECH_ERROR_RETURN_TIMEOUT 2U
#define IDENT_MECH_ERROR_REPEATABILITY  3U
#define IDENT_MECH_ERROR_FIT            4U
#define IDENT_MECH_ERROR_FIT_QUALITY    5U
#define IDENT_MECH_ERROR_CONTROL_MODE   6U

#define IDENT_MECH_RES_ERROR_NONE     0U
#define IDENT_MECH_RES_ERROR_ABORTED  1U
#define IDENT_MECH_RES_ERROR_POINTS   2U
#define IDENT_MECH_RES_ERROR_BASELINE 3U
#define IDENT_MECH_RES_ERROR_PEAK     4U
#define IDENT_MECH_RES_ERROR_SHAPER   5U
#define IDENT_MECH_RES_ERROR_START    6U

#define IDENT_MECH_LN10 2.302585093f

// 采样表按 5 kHz 辨识节拍设计。
static const uint32_t ident_mech_target_rate_hz = 5000U;

typedef struct {
    uint8_t amplitude_index;
    uint16_t sample_divider;
    uint16_t frequency_bin;
    uint8_t window_count;
} ident_mech_frequency_t;

static const float ident_mech_default_amplitudes_a[2U] = {0.100f, 0.200f};

// 低频段扫频：每个点使用 3 个 1024 点窗口，250 Hz 采样（每个窗口 4.096 s）。
// 0.488 Hz 和 0.977 Hz 组因起动和行程漂移过大而省略；4.88 Hz 以上开始
// 混入柔性响应，因此拟合只使用 1.953 Hz 和 3.418 Hz 两组。
static const ident_mech_frequency_t ident_mech_frequency_table[] = {
    {0U, 20U, 8U, 3U},
    {1U, 20U, 8U, 3U},
    {0U, 20U, 14U, 3U},
    {1U, 20U, 14U, 3U},
};

#define IDENT_MECH_FREQUENCY_COUNT                                             \
    ((uint16_t)(sizeof(ident_mech_frequency_table) /                           \
                sizeof(ident_mech_frequency_table[0])))

_Static_assert(IDENT_MECH_FREQUENCY_COUNT == 4U,
               "mechanical identification table must contain 4 records");

typedef struct {
    float gram[IDENT_MECH_LS_BASIS_COUNT][IDENT_MECH_LS_BASIS_COUNT];
    float iq_rhs[IDENT_MECH_LS_BASIS_COUNT];
    float position_rhs[IDENT_MECH_LS_BASIS_COUNT];
    float iq_reference;
    float position_reference;
    uint8_t reference_valid;
} ident_mech_ls_accumulator_t;

typedef struct {
    float frequency_hz;
    float iq_amplitude_a;
    float response_real;
    float response_imag;
    float gain_std_percent;
    float phase_std_deg;
    float weight;
    uint8_t window_count;
} ident_mech_record_t;

typedef enum {
    IDENT_MECH_FIT_LINEAR_ALL = 0U,
    IDENT_MECH_FIT_FRICTION_LOW,
    IDENT_MECH_FIT_FRICTION_ALL
} ident_mech_fit_mode_t;

typedef enum {
    IDENT_MECH_RES_IDLE = 0U,
    IDENT_MECH_RES_SETTLE,
    IDENT_MECH_RES_RUN,
    IDENT_MECH_RES_GAP,
    IDENT_MECH_RES_DONE
} ident_mech_resonance_state_t;

typedef struct {
    uint32_t count;
    float sum_t;
    float sum_tt;
    float sum_s;
    float sum_c;
    float sum_ss;
    float sum_cc;
    float sum_ts;
    float sum_tc;
    float sum_sc;
    float pos_b0;
    float pos_b1;
    float pos_bs;
    float pos_bc;
    float iq_b0;
    float iq_b1;
    float iq_bs;
    float iq_bc;
} ident_mech_res_regression_t;

typedef struct {
    float frequency_hz;
    float gain;
    float phase_rad;
    bool valid;
} ident_mech_res_point_t;

typedef struct {
    volatile ident_mech_resonance_state_t state;
    volatile bool done_pending;
    volatile bool aborted;
    uint8_t error_code;
    uint8_t abort_fault;
    uint16_t valid_point_count;
    float best_prominence_db;
    float tune_frequency_hz;
    float tune_damping_ratio;
    float sample_dt;
    float frequency_hz;
    float phase;
    float phase_increment;
    uint16_t point_index;
    uint32_t settle_ticks;
    uint32_t run_ticks;
    uint32_t discard_ticks;
    uint32_t accumulation_ticks;
    uint32_t gap_ticks;
    uint32_t point_tick;
    ident_mech_res_regression_t regression;
    ident_mech_res_point_t points[IDENT_MECH_RES_POINT_COUNT];
} ident_mech_resonance_t;

typedef struct {
    volatile ident_mech_state_t state;
    uint32_t control_rate_hz;
    uint32_t return_stable_tick_limit;
    uint32_t return_tick_limit;
    uint16_t frequency_index;
    uint16_t sample_index;
    uint16_t sample_divider_counter;
    uint8_t window_index;
    uint8_t valid_window_count;
    uint8_t frequency_retry_count;
    uint8_t frequency_retry_pending;
    uint8_t fail_after_return;
    ident_mech_resonance_t resonance;
    uint32_t settle_tick;
    uint32_t settle_tick_limit;
    uint32_t return_delay_tick;
    uint32_t return_delay_tick_limit;
    uint32_t return_stable_tick;
    uint32_t return_tick;
    float start_position;
    float return_position_error;
    float return_speed;
    float phase;
    float phase_increment;
    float gain_sum;
    float gain_square_sum;
    float phase_unit_real_sum;
    float phase_unit_imag_sum;
    float window_response_real[IDENT_MECH_MAX_WINDOW_COUNT];
    float window_response_imag[IDENT_MECH_MAX_WINDOW_COUNT];
    float window_gain[IDENT_MECH_MAX_WINDOW_COUNT];
    float window_iq_amplitude[IDENT_MECH_MAX_WINDOW_COUNT];
    float fit_fixed_ka;
    float excitation_amplitude_a[2U];
    ident_mech_ls_accumulator_t ls;
    ident_mech_record_t records[IDENT_MECH_FREQUENCY_COUNT];
    identify_mechanical_result_t result;
} ident_mech_context_t;

static ident_mech_context_t _ident_mech;
static ident_mech_context_t *ident_mech = &_ident_mech;

static float ident_mech_median(const float source[], uint16_t count);

static bool ident_mech_is_finite(float value)
{
    // 直接检查 IEEE 754 指数位，避免依赖库函数 isfinite。
    uint32_t bits;

    _Static_assert(sizeof(value) == sizeof(bits), "32-bit float required");
    memcpy(&bits, &value, sizeof(bits));
    return (bits & UINT32_C(0x7f800000)) != UINT32_C(0x7f800000);
}

static float ident_mech_clamp(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static float ident_mech_current_position(void)
{
    return encoder_data.mechanical_position;
}

static float
ident_mech_actual_frequency_hz(const ident_mech_frequency_t *frequency)
{
    return ((float)ident_mech->control_rate_hz *
            (float)frequency->frequency_bin) /
        ((float)frequency->sample_divider * (float)IDENT_MECH_WINDOW_SIZE);
}

static const char *ident_mech_low_error_name(uint8_t error_code)
{
    switch (error_code) {
        case IDENT_MECH_ERROR_NONE:
            return "none";
        case IDENT_MECH_ERROR_CAPTURE:
            return "capture";
        case IDENT_MECH_ERROR_RETURN_TIMEOUT:
            return "return timeout";
        case IDENT_MECH_ERROR_REPEATABILITY:
            return "repeatability";
        case IDENT_MECH_ERROR_FIT:
            return "fit";
        case IDENT_MECH_ERROR_FIT_QUALITY:
            return "fit quality";
        case IDENT_MECH_ERROR_CONTROL_MODE:
            return "control mode";
        default:
            return "unknown";
    }
}

static void ident_mech_set_failed(uint8_t error_code)
{
    mc_ctrl.target_d = 0.0f;
    mc_ctrl.target_q = 0.0f;
    ident_mech->result.low_frequency_valid = false;
    ident_mech->result.low_frequency_error_code = error_code;
    log_e("low failed: %s code=%u point=%u fault=%u",
          ident_mech_low_error_name(error_code), (unsigned int)error_code,
          (unsigned int)(ident_mech->frequency_index + 1U),
          (unsigned int)motor_ctrl.fault_type);
    __DMB();
    ident_mech->state = IDENT_MECH_RESONANCE_PENDING;
}

static void ident_mech_ls_reset(void)
{
    memset(&ident_mech->ls, 0, sizeof(ident_mech->ls));
}

static void ident_mech_prepare_frequency(void)
{
    const ident_mech_frequency_t *frequency =
        &ident_mech_frequency_table[ident_mech->frequency_index];
    const float actual_frequency_hz = ident_mech_actual_frequency_hz(frequency);
    const uint32_t settle_ticks =
        (uint32_t)((IDENT_MECH_SETTLE_TIME_S *
                    (float)ident_mech->control_rate_hz) +
                   0.5f);

    ident_mech->sample_index = 0U;
    ident_mech->sample_divider_counter = 0U;
    ident_mech->window_index = 0U;
    ident_mech->valid_window_count = 0U;
    ident_mech->settle_tick = 0U;
    ident_mech->settle_tick_limit = settle_ticks;
    ident_mech->gain_sum = 0.0f;
    ident_mech->gain_square_sum = 0.0f;
    ident_mech->phase_unit_real_sum = 0.0f;
    ident_mech->phase_unit_imag_sum = 0.0f;
    ident_mech->phase = 0.0f;
    ident_mech->phase_increment =
        _2PI * actual_frequency_hz / (float)ident_mech->control_rate_hz;
    ident_mech_ls_reset();
    __DMB();
    ident_mech->state = IDENT_MECH_SETTLE;
}

static void ident_mech_ls_add_sample(const ident_mech_frequency_t *frequency,
                                     float iq, float position)
{
    // 基函数包含直流和线性漂移，避免转子慢速移动影响频响估计。
    float basis[IDENT_MECH_LS_BASIS_COUNT];
    const float angle = _2PI * (float)frequency->frequency_bin *
        (float)ident_mech->sample_index / (float)IDENT_MECH_WINDOW_SIZE;

    if (ident_mech->ls.reference_valid == 0U) {
        ident_mech->ls.iq_reference = iq;
        ident_mech->ls.position_reference = position;
        ident_mech->ls.reference_valid = 1U;
    }

    basis[0] = cosf(angle);
    basis[1] = sinf(angle);
    basis[2] = 1.0f;
    basis[3] = (2.0f * (float)ident_mech->sample_index /
                (float)(IDENT_MECH_WINDOW_SIZE - 1U)) -
        1.0f;
    iq -= ident_mech->ls.iq_reference;
    position -= ident_mech->ls.position_reference;

    for (uint8_t row = 0U; row < IDENT_MECH_LS_BASIS_COUNT; row++) {
        ident_mech->ls.iq_rhs[row] += basis[row] * iq;
        ident_mech->ls.position_rhs[row] += basis[row] * position;
        for (uint8_t column = 0U; column < IDENT_MECH_LS_BASIS_COUNT;
             column++) {
            ident_mech->ls.gram[row][column] += basis[row] * basis[column];
        }
    }
}

static uint8_t
ident_mech_ls_solve(float iq_coefficients[IDENT_MECH_LS_BASIS_COUNT],
                    float position_coefficients[IDENT_MECH_LS_BASIS_COUNT])
{
    float augmented[IDENT_MECH_LS_BASIS_COUNT][IDENT_MECH_LS_BASIS_COUNT + 2U];

    for (uint8_t row = 0U; row < IDENT_MECH_LS_BASIS_COUNT; row++) {
        for (uint8_t column = 0U; column < IDENT_MECH_LS_BASIS_COUNT;
             column++) {
            augmented[row][column] = ident_mech->ls.gram[row][column];
        }
        augmented[row][IDENT_MECH_LS_BASIS_COUNT] = ident_mech->ls.iq_rhs[row];
        augmented[row][IDENT_MECH_LS_BASIS_COUNT + 1U] =
            ident_mech->ls.position_rhs[row];
    }

    for (uint8_t pivot_column = 0U; pivot_column < IDENT_MECH_LS_BASIS_COUNT;
         pivot_column++) {
        uint8_t pivot_row = pivot_column;
        float pivot_magnitude = fabsf(augmented[pivot_row][pivot_column]);

        for (uint8_t row = (uint8_t)(pivot_column + 1U);
             row < IDENT_MECH_LS_BASIS_COUNT; row++) {
            const float magnitude = fabsf(augmented[row][pivot_column]);
            if (magnitude > pivot_magnitude) {
                pivot_magnitude = magnitude;
                pivot_row = row;
            }
        }
        if (pivot_magnitude < IDENT_MECH_LS_PIVOT_MIN) {
            return 0U;
        }

        if (pivot_row != pivot_column) {
            for (uint8_t column = 0U; column < (IDENT_MECH_LS_BASIS_COUNT + 2U);
                 column++) {
                const float temporary = augmented[pivot_column][column];
                augmented[pivot_column][column] = augmented[pivot_row][column];
                augmented[pivot_row][column] = temporary;
            }
        }

        const float pivot = augmented[pivot_column][pivot_column];
        for (uint8_t column = pivot_column;
             column < (IDENT_MECH_LS_BASIS_COUNT + 2U); column++) {
            augmented[pivot_column][column] /= pivot;
        }

        for (uint8_t row = 0U; row < IDENT_MECH_LS_BASIS_COUNT; row++) {
            if (row == pivot_column) {
                continue;
            }
            const float scale = augmented[row][pivot_column];
            for (uint8_t column = pivot_column;
                 column < (IDENT_MECH_LS_BASIS_COUNT + 2U); column++) {
                augmented[row][column] -=
                    scale * augmented[pivot_column][column];
            }
        }
    }

    for (uint8_t row = 0U; row < IDENT_MECH_LS_BASIS_COUNT; row++) {
        iq_coefficients[row] = augmented[row][IDENT_MECH_LS_BASIS_COUNT];
        position_coefficients[row] =
            augmented[row][IDENT_MECH_LS_BASIS_COUNT + 1U];
    }
    return 1U;
}

static uint8_t ident_mech_finish_window(void)
{
    float iq_coefficients[IDENT_MECH_LS_BASIS_COUNT];
    float position_coefficients[IDENT_MECH_LS_BASIS_COUNT];
    float input_real;
    float input_imag;
    float output_real;
    float output_imag;
    float input_power;
    float response_real;
    float response_imag;
    float response_gain;

    if (ident_mech_ls_solve(iq_coefficients, position_coefficients) == 0U) {
        return 0U;
    }

    input_real = iq_coefficients[0];
    input_imag = -iq_coefficients[1];
    output_real = position_coefficients[0];
    output_imag = -position_coefficients[1];
    input_power = (input_real * input_real) + (input_imag * input_imag);
    if (input_power <
        (IDENT_MECH_INPUT_AMPLITUDE_MIN_A * IDENT_MECH_INPUT_AMPLITUDE_MIN_A)) {
        return 0U;
    }

    response_real =
        ((output_real * input_real) + (output_imag * input_imag)) / input_power;
    response_imag =
        ((output_imag * input_real) - (output_real * input_imag)) / input_power;
    response_gain = sqrtf((response_real * response_real) +
                          (response_imag * response_imag));
    if ((!ident_mech_is_finite(response_gain)) || (response_gain <= 0.0f)) {
        return 0U;
    }

    if (ident_mech->valid_window_count >= IDENT_MECH_MAX_WINDOW_COUNT) {
        return 0U;
    }

    ident_mech->gain_sum += response_gain;
    ident_mech->gain_square_sum += response_gain * response_gain;
    ident_mech->phase_unit_real_sum += response_real / response_gain;
    ident_mech->phase_unit_imag_sum += response_imag / response_gain;
    ident_mech->window_response_real[ident_mech->valid_window_count] =
        response_real;
    ident_mech->window_response_imag[ident_mech->valid_window_count] =
        response_imag;
    ident_mech->window_gain[ident_mech->valid_window_count] = response_gain;
    ident_mech->window_iq_amplitude[ident_mech->valid_window_count] =
        sqrtf(input_power);
    ident_mech->valid_window_count++;
    return 1U;
}

static void ident_mech_sort_values(float values[], uint16_t count)
{
    for (uint16_t index = 1U; index < count; index++) {
        const float value = values[index];
        uint16_t position = index;

        while ((position > 0U) && (values[position - 1U] > value)) {
            values[position] = values[position - 1U];
            position--;
        }
        values[position] = value;
    }
}

static float ident_mech_median(const float source[], uint16_t count)
{
    float values[IDENT_MECH_FREQUENCY_COUNT];

    if ((count == 0U) || (count > IDENT_MECH_FREQUENCY_COUNT)) {
        return NAN;
    }
    memcpy(values, source, count * sizeof(float));
    ident_mech_sort_values(values, count);
    if ((count & 1U) != 0U) {
        return values[count / 2U];
    }
    return 0.5f * (values[(count / 2U) - 1U] + values[count / 2U]);
}

static uint8_t ident_mech_prepare_weights(void)
{
    // 把窗口间重复性折算为归一化拟合权重。
    float raw_quality[IDENT_MECH_FREQUENCY_COUNT];
    float gain_std[IDENT_MECH_FREQUENCY_COUNT];
    float phase_std[IDENT_MECH_FREQUENCY_COUNT];

    for (uint16_t index = 0U; index < IDENT_MECH_FREQUENCY_COUNT; index++) {
        const ident_mech_record_t *record = &ident_mech->records[index];
        float relative_standard_error;

        gain_std[index] = record->gain_std_percent;
        phase_std[index] = record->phase_std_deg;
        if ((!ident_mech_is_finite(record->gain_std_percent)) ||
            (!ident_mech_is_finite(record->phase_std_deg)) ||
            (record->gain_std_percent >
             IDENT_MECH_QUALITY_GAIN_STD_MAX_PERCENT) ||
            (record->phase_std_deg > IDENT_MECH_QUALITY_PHASE_STD_MAX_DEG)) {
            return 0U;
        }

        relative_standard_error =
            sqrtf(((record->gain_std_percent / 100.0f) *
                   (record->gain_std_percent / 100.0f)) +
                  ((record->phase_std_deg * PI / 180.0f) *
                   (record->phase_std_deg * PI / 180.0f))) /
            sqrtf((float)record->window_count);
        if (relative_standard_error < IDENT_MECH_QUALITY_SIGMA_FLOOR) {
            relative_standard_error = IDENT_MECH_QUALITY_SIGMA_FLOOR;
        }
        raw_quality[index] =
            1.0f / (relative_standard_error * relative_standard_error);
    }

    const float median_gain_std_percent =
        ident_mech_median(gain_std, IDENT_MECH_FREQUENCY_COUNT);
    const float median_phase_std_deg =
        ident_mech_median(phase_std, IDENT_MECH_FREQUENCY_COUNT);
    if ((median_gain_std_percent > IDENT_MECH_QUALITY_MEDIAN_GAIN_MAX) ||
        (median_phase_std_deg > IDENT_MECH_QUALITY_MEDIAN_PHASE_MAX)) {
        return 0U;
    }

    const float reference_quality =
        ident_mech_median(raw_quality, IDENT_MECH_FREQUENCY_COUNT);
    if ((!ident_mech_is_finite(reference_quality)) ||
        (reference_quality <= 0.0f)) {
        return 0U;
    }

    for (uint16_t index = 0U; index < IDENT_MECH_FREQUENCY_COUNT; index++) {
        float quality = raw_quality[index] / reference_quality;
        uint16_t frequency_record_count = 0U;

        if (quality < 0.28f) {
            quality = 0.28f;
        }
        if (quality > 4.0f) {
            quality = 4.0f;
        }
        for (uint16_t other = 0U; other < IDENT_MECH_FREQUENCY_COUNT; other++) {
            if (fabsf(ident_mech->records[other].frequency_hz -
                      ident_mech->records[index].frequency_hz) < 1.0e-5f) {
                frequency_record_count++;
            }
        }
        if (frequency_record_count == 0U) {
            return 0U;
        }
        ident_mech->records[index].weight =
            quality / (float)frequency_record_count;
    }
    return 1U;
}

static uint8_t ident_mech_linear_seed(float *ka, float *kv)
{
    float ka_numerator = 0.0f;
    float ka_denominator = 0.0f;
    float kv_numerator = 0.0f;
    float kv_denominator = 0.0f;

    for (uint16_t index = 0U; index < IDENT_MECH_FREQUENCY_COUNT; index++) {
        const ident_mech_record_t *record = &ident_mech->records[index];
        float response_power;
        float inverse_real;
        float inverse_imag;
        float angular_frequency;
        float angular_frequency_squared;

        response_power = (record->response_real * record->response_real) +
            (record->response_imag * record->response_imag);
        if (response_power <= 0.0f) {
            return 0U;
        }
        inverse_real = record->response_real / response_power;
        inverse_imag = -record->response_imag / response_power;
        angular_frequency = _2PI * record->frequency_hz;
        angular_frequency_squared = angular_frequency * angular_frequency;
        ka_numerator +=
            record->weight * (-angular_frequency_squared) * inverse_real;
        ka_denominator += record->weight * angular_frequency_squared *
            angular_frequency_squared;
        kv_numerator += record->weight * angular_frequency * inverse_imag;
        kv_denominator += record->weight * angular_frequency_squared;
    }

    if ((!ident_mech_is_finite(ka_numerator)) ||
        (!ident_mech_is_finite(ka_denominator)) ||
        (!ident_mech_is_finite(kv_numerator)) ||
        (!ident_mech_is_finite(kv_denominator)) || (ka_denominator <= 0.0f) ||
        (kv_denominator <= 0.0f)) {
        return 0U;
    }
    *ka = ka_numerator / ka_denominator;
    *kv = kv_numerator / kv_denominator;
    return (ident_mech_is_finite(*ka) && ident_mech_is_finite(*kv) &&
            (*ka > 0.0f) && (*kv > 0.0f))
        ? 1U
        : 0U;
}

static uint8_t ident_mech_model_response(float ka, float kv, float tc,
                                         const ident_mech_record_t *record,
                                         uint8_t friction_enabled,
                                         float *response_real,
                                         float *response_imag)
{
    // 位置/电流频响模型：J、粘滞阻尼和库仑摩擦分别对应 Ka、Kv、Tc。
    const float angular_frequency = _2PI * record->frequency_hz;
    const float inertia_term = ka * angular_frequency * angular_frequency;
    float denominator_real = -inertia_term;
    float denominator_imag = kv * angular_frequency;

    if (friction_enabled != 0U) {
        const float friction_fundamental = 4.0f * tc / PI;
        const float quadratic = (inertia_term * inertia_term) +
            (denominator_imag * denominator_imag);
        const float radicand = ((denominator_imag * friction_fundamental) *
                                (denominator_imag * friction_fundamental)) +
            quadratic *
                ((record->iq_amplitude_a * record->iq_amplitude_a) -
                 (friction_fundamental * friction_fundamental));
        float position_amplitude;

        if ((record->iq_amplitude_a <= friction_fundamental) ||
            (radicand <= 0.0f) || (quadratic <= 0.0f)) {
            return 0U;
        }
        position_amplitude =
            (-denominator_imag * friction_fundamental + sqrtf(radicand)) /
            quadratic;
        if ((!ident_mech_is_finite(position_amplitude)) ||
            (position_amplitude <= 0.0f)) {
            return 0U;
        }
        denominator_imag += friction_fundamental / position_amplitude;
    }

    const float denominator_power = (denominator_real * denominator_real) +
        (denominator_imag * denominator_imag);

    if ((!ident_mech_is_finite(denominator_power)) ||
        (denominator_power <= 0.0f)) {
        return 0U;
    }
    *response_real = denominator_real / denominator_power;
    *response_imag = -denominator_imag / denominator_power;
    return (ident_mech_is_finite(*response_real) &&
            ident_mech_is_finite(*response_imag))
        ? 1U
        : 0U;
}

static float
ident_mech_fit_cost(const float parameters[IDENT_MECH_NM_MAX_DIMENSION],
                    uint8_t dimension, ident_mech_fit_mode_t mode)
{
    float ka;
    float kv;
    float tc = 0.0f;
    float weighted_error_sum = 0.0f;
    float weight_sum = 0.0f;
    uint8_t friction_enabled = 0U;

    if ((mode == IDENT_MECH_FIT_LINEAR_ALL) && (dimension == 2U)) {
        ka = expf(parameters[0]);
        kv = expf(parameters[1]);
    }
    else if ((mode == IDENT_MECH_FIT_FRICTION_LOW) && (dimension == 2U)) {
        ka = ident_mech->fit_fixed_ka;
        kv = expf(parameters[0]);
        tc = expf(parameters[1]);
        friction_enabled = 1U;
    }
    else if ((mode == IDENT_MECH_FIT_FRICTION_ALL) && (dimension == 3U)) {
        ka = expf(parameters[0]);
        kv = expf(parameters[1]);
        tc = expf(parameters[2]);
        friction_enabled = 1U;
    }
    else {
        return IDENT_MECH_INVALID_COST;
    }

    if ((!ident_mech_is_finite(ka)) || (!ident_mech_is_finite(kv)) ||
        (!ident_mech_is_finite(tc)) || (ka < 1.0e-8f) || (ka > 0.1f) ||
        (kv < 1.0e-8f) || (kv > 1.0f) ||
        ((friction_enabled != 0U) &&
         ((tc < 1.0e-7f) || (tc > IDENT_MECH_TC_MAX_A)))) {
        return IDENT_MECH_INVALID_COST;
    }

    for (uint16_t index = 0U; index < IDENT_MECH_FREQUENCY_COUNT; index++) {
        const ident_mech_record_t *record = &ident_mech->records[index];
        float model_real;
        float model_imag;
        float measured_power;
        float error_real;
        float error_imag;

        if ((mode == IDENT_MECH_FIT_FRICTION_LOW) &&
            (record->frequency_hz > IDENT_MECH_FRICTION_SEED_MAX_HZ)) {
            continue;
        }
        if (ident_mech_model_response(ka, kv, tc, record, friction_enabled,
                                      &model_real, &model_imag) == 0U) {
            return IDENT_MECH_INVALID_COST;
        }
        measured_power = (record->response_real * record->response_real) +
            (record->response_imag * record->response_imag);
        if (measured_power <= 0.0f) {
            return IDENT_MECH_INVALID_COST;
        }
        error_real = model_real - record->response_real;
        error_imag = model_imag - record->response_imag;
        weighted_error_sum += record->weight *
            ((error_real * error_real) + (error_imag * error_imag)) /
            measured_power;
        weight_sum += record->weight;
    }

    if ((!ident_mech_is_finite(weighted_error_sum)) || (weight_sum <= 0.0f)) {
        return IDENT_MECH_INVALID_COST;
    }
    return weighted_error_sum / weight_sum;
}

static void ident_mech_nm_sort(
    float simplex[IDENT_MECH_NM_MAX_VERTICES][IDENT_MECH_NM_MAX_DIMENSION],
    float costs[IDENT_MECH_NM_MAX_VERTICES], uint8_t dimension)
{
    const uint8_t vertex_count = (uint8_t)(dimension + 1U);

    for (uint8_t index = 0U; index < vertex_count; index++) {
        for (uint8_t other = (uint8_t)(index + 1U); other < vertex_count;
             other++) {
            if (costs[other] < costs[index]) {
                const float temporary_cost = costs[index];
                costs[index] = costs[other];
                costs[other] = temporary_cost;
                for (uint8_t axis = 0U; axis < dimension; axis++) {
                    const float temporary = simplex[index][axis];
                    simplex[index][axis] = simplex[other][axis];
                    simplex[other][axis] = temporary;
                }
            }
        }
    }
}

static uint8_t
ident_mech_nelder_mead(float parameters[IDENT_MECH_NM_MAX_DIMENSION],
                       uint8_t dimension, ident_mech_fit_mode_t mode,
                       float *final_cost)
{
    float simplex[IDENT_MECH_NM_MAX_VERTICES][IDENT_MECH_NM_MAX_DIMENSION] = {
        {0.0f}};
    float costs[IDENT_MECH_NM_MAX_VERTICES] = {0.0f};
    float centroid[IDENT_MECH_NM_MAX_DIMENSION];
    float reflected[IDENT_MECH_NM_MAX_DIMENSION];
    float candidate[IDENT_MECH_NM_MAX_DIMENSION];
    const uint8_t vertex_count = (uint8_t)(dimension + 1U);

    if ((dimension == 0U) || (dimension > IDENT_MECH_NM_MAX_DIMENSION)) {
        return 0U;
    }
    for (uint8_t vertex = 0U; vertex < vertex_count; vertex++) {
        for (uint8_t axis = 0U; axis < dimension; axis++) {
            simplex[vertex][axis] = parameters[axis];
        }
        if (vertex > 0U) {
            simplex[vertex][vertex - 1U] += IDENT_MECH_NM_INITIAL_STEP;
        }
        costs[vertex] = ident_mech_fit_cost(simplex[vertex], dimension, mode);
    }

    for (uint16_t iteration = 0U; iteration < IDENT_MECH_NM_MAX_ITERATIONS;
         iteration++) {
        float parameter_spread = 0.0f;
        float reflected_cost;

        ident_mech_nm_sort(simplex, costs, dimension);
        for (uint8_t vertex = 1U; vertex < vertex_count; vertex++) {
            for (uint8_t axis = 0U; axis < dimension; axis++) {
                const float difference =
                    fabsf(simplex[vertex][axis] - simplex[0][axis]);
                if (difference > parameter_spread) {
                    parameter_spread = difference;
                }
            }
        }
        if ((parameter_spread < IDENT_MECH_NM_PARAMETER_TOLERANCE) &&
            (fabsf(costs[dimension] - costs[0]) <
             IDENT_MECH_NM_COST_TOLERANCE)) {
            break;
        }

        for (uint8_t axis = 0U; axis < dimension; axis++) {
            centroid[axis] = 0.0f;
            for (uint8_t vertex = 0U; vertex < dimension; vertex++) {
                centroid[axis] += simplex[vertex][axis];
            }
            centroid[axis] /= (float)dimension;
            reflected[axis] =
                centroid[axis] + (centroid[axis] - simplex[dimension][axis]);
        }
        reflected_cost = ident_mech_fit_cost(reflected, dimension, mode);

        if (reflected_cost < costs[0]) {
            for (uint8_t axis = 0U; axis < dimension; axis++) {
                candidate[axis] =
                    centroid[axis] + 2.0f * (reflected[axis] - centroid[axis]);
            }
            const float candidate_cost =
                ident_mech_fit_cost(candidate, dimension, mode);
            if (candidate_cost < reflected_cost) {
                memcpy(simplex[dimension], candidate,
                       dimension * sizeof(float));
                costs[dimension] = candidate_cost;
            }
            else {
                memcpy(simplex[dimension], reflected,
                       dimension * sizeof(float));
                costs[dimension] = reflected_cost;
            }
            continue;
        }

        if (reflected_cost < costs[dimension - 1U]) {
            memcpy(simplex[dimension], reflected, dimension * sizeof(float));
            costs[dimension] = reflected_cost;
            continue;
        }

        if (reflected_cost < costs[dimension]) {
            for (uint8_t axis = 0U; axis < dimension; axis++) {
                candidate[axis] =
                    centroid[axis] + 0.5f * (reflected[axis] - centroid[axis]);
            }
            const float candidate_cost =
                ident_mech_fit_cost(candidate, dimension, mode);
            if (candidate_cost <= reflected_cost) {
                memcpy(simplex[dimension], candidate,
                       dimension * sizeof(float));
                costs[dimension] = candidate_cost;
                continue;
            }
        }
        else {
            for (uint8_t axis = 0U; axis < dimension; axis++) {
                candidate[axis] = centroid[axis] +
                    0.5f * (simplex[dimension][axis] - centroid[axis]);
            }
            const float candidate_cost =
                ident_mech_fit_cost(candidate, dimension, mode);
            if (candidate_cost < costs[dimension]) {
                memcpy(simplex[dimension], candidate,
                       dimension * sizeof(float));
                costs[dimension] = candidate_cost;
                continue;
            }
        }

        for (uint8_t vertex = 1U; vertex < vertex_count; vertex++) {
            for (uint8_t axis = 0U; axis < dimension; axis++) {
                simplex[vertex][axis] = simplex[0][axis] +
                    0.5f * (simplex[vertex][axis] - simplex[0][axis]);
            }
            costs[vertex] =
                ident_mech_fit_cost(simplex[vertex], dimension, mode);
        }
    }

    ident_mech_nm_sort(simplex, costs, dimension);
    if ((!ident_mech_is_finite(costs[0])) ||
        (costs[0] >= IDENT_MECH_INVALID_COST)) {
        return 0U;
    }
    memcpy(parameters, simplex[0], dimension * sizeof(float));
    *final_cost = costs[0];
    return 1U;
}

static uint8_t ident_mech_fit_parameters(void)
{
    float ka_initial;
    float kv_initial;
    float linear_parameters[IDENT_MECH_NM_MAX_DIMENSION] = {0.0f};
    float low_parameters[IDENT_MECH_NM_MAX_DIMENSION] = {0.0f};
    float final_parameters[IDENT_MECH_NM_MAX_DIMENSION] = {0.0f};
    float unused_cost;
    float final_cost;
    float ka;
    float kv;
    float tc;
    float minimum_iq = INFINITY;

    if (ident_mech_prepare_weights() == 0U) {
        ident_mech->result.low_frequency_error_code =
            IDENT_MECH_ERROR_REPEATABILITY;
        return 0U;
    }

    // 先用 1/G = -Ka*w^2 + j*Kv*w 求线性初值，再用低频段估计摩擦，
    // 最后在完整低频段上联合细化三个系数。
    if (ident_mech_linear_seed(&ka_initial, &kv_initial) == 0U) {
        ident_mech->result.low_frequency_error_code = IDENT_MECH_ERROR_FIT;
        return 0U;
    }

    linear_parameters[0] = logf(ka_initial);
    linear_parameters[1] = logf(kv_initial);
    if (ident_mech_nelder_mead(linear_parameters, 2U, IDENT_MECH_FIT_LINEAR_ALL,
                               &unused_cost) == 0U) {
        ident_mech->result.low_frequency_error_code = IDENT_MECH_ERROR_FIT;
        return 0U;
    }

    ident_mech->fit_fixed_ka = expf(linear_parameters[0]);
    low_parameters[0] = logf(fmaxf(0.5f * expf(linear_parameters[1]), 1.0e-4f));
    low_parameters[1] = logf(0.10f);
    if (ident_mech_nelder_mead(low_parameters, 2U, IDENT_MECH_FIT_FRICTION_LOW,
                               &unused_cost) == 0U) {
        ident_mech->result.low_frequency_error_code = IDENT_MECH_ERROR_FIT;
        return 0U;
    }

    final_parameters[0] = linear_parameters[0];
    final_parameters[1] = low_parameters[0];
    final_parameters[2] = low_parameters[1];
    if (ident_mech_nelder_mead(final_parameters, 3U,
                               IDENT_MECH_FIT_FRICTION_ALL,
                               &final_cost) == 0U) {
        ident_mech->result.low_frequency_error_code = IDENT_MECH_ERROR_FIT;
        return 0U;
    }

    ka = expf(final_parameters[0]);
    kv = expf(final_parameters[1]);
    tc = expf(final_parameters[2]);
    for (uint16_t index = 0U; index < IDENT_MECH_FREQUENCY_COUNT; index++) {
        if (ident_mech->records[index].iq_amplitude_a < minimum_iq) {
            minimum_iq = ident_mech->records[index].iq_amplitude_a;
        }
    }

    ident_mech->result.ka = ka;
    ident_mech->result.kv = kv;
    ident_mech->result.tc = tc;
    ident_mech->result.weighted_rms_percent = 100.0f * sqrtf(final_cost);

    if ((!ident_mech_is_finite(ka)) || (!ident_mech_is_finite(kv)) ||
        (!ident_mech_is_finite(tc)) ||
        (!ident_mech_is_finite(ident_mech->result.weighted_rms_percent)) ||
        (ka <= 0.0f) || (ka > 0.1f) || (kv < 0.0f) || (kv > 1.0f) ||
        (tc < 0.0f) || (tc > IDENT_MECH_TC_MAX_A) ||
        (tc >= (0.95f * minimum_iq * PI / 4.0f)) ||
        (ident_mech->result.weighted_rms_percent >
         IDENT_MECH_FIT_RMS_MAX_PERCENT)) {
        ident_mech->result.low_frequency_error_code =
            IDENT_MECH_ERROR_FIT_QUALITY;
        return 0U;
    }

    ident_mech->result.low_frequency_error_code = IDENT_MECH_ERROR_NONE;
    ident_mech->result.low_frequency_valid = true;
    return 1U;
}

static uint8_t ident_mech_store_pending_record(void)
{
    const ident_mech_frequency_t *frequency =
        &ident_mech_frequency_table[ident_mech->frequency_index];
    ident_mech_record_t *record =
        &ident_mech->records[ident_mech->frequency_index];
    const uint16_t valid_window_count = ident_mech->valid_window_count;
    float response_gain;
    float gain_variance;
    float phase_resultant;

    if (ident_mech->valid_window_count != frequency->window_count) {
        return 0U;
    }

    record->frequency_hz = ident_mech_actual_frequency_hz(frequency);
    float gains[IDENT_MECH_MAX_WINDOW_COUNT];
    float phases[IDENT_MECH_MAX_WINDOW_COUNT];
    float amplitudes[IDENT_MECH_MAX_WINDOW_COUNT];
    const float mean_phase = atan2f(ident_mech->phase_unit_imag_sum,
                                    ident_mech->phase_unit_real_sum);

    for (uint16_t index = 0U; index < valid_window_count; index++) {
        const float phase = atan2f(ident_mech->window_response_imag[index],
                                   ident_mech->window_response_real[index]);
        float phase_delta = phase - mean_phase;

        if (phase_delta > PI) {
            phase_delta -= _2PI;
        }
        else if (phase_delta < -PI) {
            phase_delta += _2PI;
        }
        gains[index] = ident_mech->window_gain[index];
        phases[index] = phase_delta;
        amplitudes[index] = ident_mech->window_iq_amplitude[index];
    }
    const float median_gain = ident_mech_median(gains, valid_window_count);
    const float median_phase =
        mean_phase + ident_mech_median(phases, valid_window_count);
    record->iq_amplitude_a = ident_mech_median(amplitudes, valid_window_count);
    record->response_real = median_gain * cosf(median_phase);
    record->response_imag = median_gain * sinf(median_phase);
    response_gain = sqrtf((record->response_real * record->response_real) +
                          (record->response_imag * record->response_imag));
    record->window_count = ident_mech->valid_window_count;

    gain_variance = ident_mech->gain_square_sum -
        ((ident_mech->gain_sum * ident_mech->gain_sum) /
         (float)valid_window_count);
    if (gain_variance < 0.0f) {
        gain_variance = 0.0f;
    }
    record->gain_std_percent = 100.0f *
        sqrtf(gain_variance / ((float)valid_window_count - 1.0f)) /
        (ident_mech->gain_sum / (float)valid_window_count);

    phase_resultant = sqrtf((ident_mech->phase_unit_real_sum *
                             ident_mech->phase_unit_real_sum) +
                            (ident_mech->phase_unit_imag_sum *
                             ident_mech->phase_unit_imag_sum)) /
        (float)valid_window_count;
    if (phase_resultant > 1.0f) {
        phase_resultant = 1.0f;
    }
    if (phase_resultant < 1.0e-6f) {
        phase_resultant = 1.0e-6f;
    }
    record->phase_std_deg =
        sqrtf(-2.0f * logf(phase_resultant)) * (180.0f / PI);

    return (ident_mech_is_finite(record->iq_amplitude_a) &&
            ident_mech_is_finite(response_gain) &&
            ident_mech_is_finite(record->response_real) &&
            ident_mech_is_finite(record->response_imag) &&
            (record->iq_amplitude_a > 0.0f) && (response_gain > 0.0f))
        ? 1U
        : 0U;
}

static uint32_t ident_mech_res_ticks_for_cycles(float frequency_hz,
                                                uint16_t cycles,
                                                float sample_dt)
{
    const float ticks = (float)cycles / (frequency_hz * sample_dt);

    if (ticks <= 1.0f) {
        return 1U;
    }
    if (ticks >= (float)UINT32_MAX) {
        return UINT32_MAX;
    }
    return (uint32_t)(ticks + 0.5f);
}

static uint32_t ident_mech_res_ticks_for_seconds(float seconds, float sample_dt)
{
    const float ticks = seconds / sample_dt;

    if (ticks <= 1.0f) {
        return 1U;
    }
    if (ticks >= (float)UINT32_MAX) {
        return UINT32_MAX;
    }
    return (uint32_t)(ticks + 0.5f);
}

static float ident_mech_res_wrap_phase(float phase)
{
    while (phase > PI) {
        phase -= _2PI;
    }
    while (phase < -PI) {
        phase += _2PI;
    }
    return phase;
}

static float ident_mech_res_frequency_for_point(uint16_t point_index)
{
    const float position =
        (float)point_index / (float)(IDENT_MECH_RES_POINT_COUNT - 1U);

    return IDENT_MECH_RES_START_HZ *
        powf(IDENT_MECH_RES_STOP_HZ / IDENT_MECH_RES_START_HZ, position);
}

static void ident_mech_res_prepare_point(void)
{
    ident_mech_resonance_t *res = &ident_mech->resonance;

    res->frequency_hz = ident_mech_res_frequency_for_point(res->point_index);
    res->phase = 0.0f;
    res->phase_increment = _2PI * res->frequency_hz * res->sample_dt;
    res->settle_ticks = ident_mech_res_ticks_for_cycles(
        res->frequency_hz, IDENT_MECH_RES_SETTLE_CYCLES, res->sample_dt);
    res->run_ticks = ident_mech_res_ticks_for_cycles(
        res->frequency_hz, IDENT_MECH_RES_MEASURE_CYCLES, res->sample_dt);
    res->discard_ticks = ident_mech_res_ticks_for_cycles(
        res->frequency_hz, IDENT_MECH_RES_DISCARD_CYCLES, res->sample_dt);
    if (res->discard_ticks >= res->run_ticks) {
        res->discard_ticks = res->run_ticks / 2U;
    }
    res->accumulation_ticks = res->run_ticks - res->discard_ticks;
    memset(&res->regression, 0, sizeof(res->regression));
    res->point_tick = 0U;
    res->state = IDENT_MECH_RES_SETTLE;
}

static void ident_mech_res_set_q(float q_current_a)
{
    mc_ctrl.target_d = 0.0f;
    mc_ctrl.target_q = q_current_a;
}

static void ident_mech_res_accumulate(float position_rad, float current_a,
                                      float sin_phase, float cos_phase)
{
    ident_mech_resonance_t *res = &ident_mech->resonance;
    ident_mech_res_regression_t *reg = &res->regression;
    const uint32_t count = reg->count;
    const uint32_t expected = res->accumulation_ticks;
    float t;

    if ((count >= expected) || (expected < 4U)) {
        return;
    }

    t = ((2.0f * (float)count + 1.0f) / (float)expected) - 1.0f;
    reg->sum_t += t;
    reg->sum_tt += t * t;
    reg->sum_s += sin_phase;
    reg->sum_c += cos_phase;
    reg->sum_ss += sin_phase * sin_phase;
    reg->sum_cc += cos_phase * cos_phase;
    reg->sum_ts += t * sin_phase;
    reg->sum_tc += t * cos_phase;
    reg->sum_sc += sin_phase * cos_phase;
    reg->pos_b0 += position_rad;
    reg->pos_b1 += t * position_rad;
    reg->pos_bs += sin_phase * position_rad;
    reg->pos_bc += cos_phase * position_rad;
    reg->iq_b0 += current_a;
    reg->iq_b1 += t * current_a;
    reg->iq_bs += sin_phase * current_a;
    reg->iq_bc += cos_phase * current_a;
    reg->count++;
}

static bool ident_mech_res_solve_4x2(float matrix[4][6], float x[4], float y[4])
{
    for (uint8_t column = 0U; column < 4U; column++) {
        uint8_t pivot = column;
        float pivot_abs = fabsf(matrix[column][column]);

        for (uint8_t row = (uint8_t)(column + 1U); row < 4U; row++) {
            const float candidate_abs = fabsf(matrix[row][column]);
            if (candidate_abs > pivot_abs) {
                pivot = row;
                pivot_abs = candidate_abs;
            }
        }
        if (pivot_abs < 1.0e-9f) {
            return false;
        }
        if (pivot != column) {
            for (uint8_t index = column; index < 6U; index++) {
                const float temporary = matrix[column][index];
                matrix[column][index] = matrix[pivot][index];
                matrix[pivot][index] = temporary;
            }
        }

        const float divisor = matrix[column][column];
        for (uint8_t index = column; index < 6U; index++) {
            matrix[column][index] /= divisor;
        }
        for (uint8_t row = 0U; row < 4U; row++) {
            if (row == column) {
                continue;
            }
            const float factor = matrix[row][column];
            if (factor == 0.0f) {
                continue;
            }
            for (uint8_t index = column; index < 6U; index++) {
                matrix[row][index] -= factor * matrix[column][index];
            }
        }
    }

    for (uint8_t row = 0U; row < 4U; row++) {
        x[row] = matrix[row][4];
        y[row] = matrix[row][5];
    }
    return true;
}

static void ident_mech_res_finish_point(void)
{
    ident_mech_resonance_t *res = &ident_mech->resonance;
    ident_mech_res_regression_t *reg = &res->regression;
    ident_mech_res_point_t *point = &res->points[res->point_index];
    float matrix[4][6] = {{0.0f}};
    float pos_coeff[4];
    float iq_coeff[4];
    const float count = (float)reg->count;

    memset(point, 0, sizeof(*point));
    point->frequency_hz = res->frequency_hz;
    if (reg->count < 4U) {
        return;
    }

    matrix[0][0] = count;
    matrix[0][1] = reg->sum_t;
    matrix[0][2] = reg->sum_s;
    matrix[0][3] = reg->sum_c;
    matrix[0][4] = reg->pos_b0;
    matrix[0][5] = reg->iq_b0;
    matrix[1][0] = reg->sum_t;
    matrix[1][1] = reg->sum_tt;
    matrix[1][2] = reg->sum_ts;
    matrix[1][3] = reg->sum_tc;
    matrix[1][4] = reg->pos_b1;
    matrix[1][5] = reg->iq_b1;
    matrix[2][0] = reg->sum_s;
    matrix[2][1] = reg->sum_ts;
    matrix[2][2] = reg->sum_ss;
    matrix[2][3] = reg->sum_sc;
    matrix[2][4] = reg->pos_bs;
    matrix[2][5] = reg->iq_bs;
    matrix[3][0] = reg->sum_c;
    matrix[3][1] = reg->sum_tc;
    matrix[3][2] = reg->sum_sc;
    matrix[3][3] = reg->sum_cc;
    matrix[3][4] = reg->pos_bc;
    matrix[3][5] = reg->iq_bc;

    if (!ident_mech_res_solve_4x2(matrix, pos_coeff, iq_coeff)) {
        return;
    }

    const float position_amplitude_rad =
        sqrtf((pos_coeff[2] * pos_coeff[2]) +
              (pos_coeff[3] * pos_coeff[3]));
    const float current_amplitude_a =
        sqrtf((iq_coeff[2] * iq_coeff[2]) + (iq_coeff[3] * iq_coeff[3]));

    if ((current_amplitude_a < IDENT_MECH_RES_MIN_CURRENT_A) ||
        (position_amplitude_rad <= 0.0f)) {
        return;
    }

    point->gain = position_amplitude_rad / current_amplitude_a;
    point->phase_rad =
        ident_mech_res_wrap_phase(atan2f(-pos_coeff[2], pos_coeff[3]) -
                                  atan2f(-iq_coeff[2], iq_coeff[3]));
    point->valid = true;
}

static void ident_mech_res_finish_sweep(bool aborted)
{
    ident_mech->resonance.aborted = aborted;
    __DMB();
    ident_mech->resonance.done_pending = true;
    ident_mech->resonance.state = IDENT_MECH_RES_DONE;
}

static int8_t ident_mech_res_start(void)
{
    const uint32_t primask = __get_PRIMASK();

    if ((ident_mech->control_rate_hz == 0U) ||
        (motor_ctrl.fault_type != MOTOR_FAULT_NONE)) {
        return -1;
    }

    __disable_irq();
    const float sample_dt = 1.0f / (float)ident_mech->control_rate_hz;

    memset(&ident_mech->resonance, 0, sizeof(ident_mech->resonance));
    ident_mech->resonance.sample_dt = sample_dt;
    ident_mech->resonance.gap_ticks =
        ident_mech_res_ticks_for_seconds(IDENT_MECH_RES_GAP_TIME_S, sample_dt);
    ident_mech_res_prepare_point();
    ident_mech_res_set_q(0.0f);
    if (primask == 0U) {
        __enable_irq();
    }
    return 0;
}

static void ident_mech_resonance_tick(void)
{
    ident_mech_resonance_t *res = &ident_mech->resonance;

    if ((ident_mech->state != IDENT_MECH_RESONANCE) ||
        ((res->state != IDENT_MECH_RES_SETTLE) &&
         (res->state != IDENT_MECH_RES_RUN) &&
         (res->state != IDENT_MECH_RES_GAP))) {
        return;
    }
    if (motor_ctrl.fault_type != MOTOR_FAULT_NONE) {
        res->error_code = IDENT_MECH_RES_ERROR_ABORTED;
        res->abort_fault = (uint8_t)motor_ctrl.fault_type;
        ident_mech_res_set_q(0.0f);
        ident_mech_res_finish_sweep(true);
        return;
    }

    switch (res->state) {
        case IDENT_MECH_RES_SETTLE:
            ident_mech_res_set_q(0.0f);
            if (++res->point_tick >= res->settle_ticks) {
                res->point_tick = 0U;
                res->phase = 0.0f;
                res->state = IDENT_MECH_RES_RUN;
            }
            break;

        case IDENT_MECH_RES_RUN: {
            if (res->point_tick >= res->run_ticks) {
                ident_mech_res_finish_point();
                ident_mech_res_set_q(0.0f);
                res->point_tick = 0U;
                res->state = IDENT_MECH_RES_GAP;
                break;
            }

            const float sin_phase = sinf(res->phase);
            const float cos_phase = cosf(res->phase);

            ident_mech_res_set_q(IDENT_MECH_RES_CURRENT_A * sin_phase);
            if (res->point_tick >= res->discard_ticks) {
                ident_mech_res_accumulate(ident_mech_current_position(),
                                          mc_ctrl.abc_dq.current_q, sin_phase,
                                          cos_phase);
            }
            res->phase += res->phase_increment;
            if (res->phase >= _2PI) {
                res->phase -= _2PI;
            }
            res->point_tick++;
            break;
        }

        case IDENT_MECH_RES_GAP:
            ident_mech_res_set_q(0.0f);
            if (++res->point_tick >= res->gap_ticks) {
                res->point_tick = 0U;
                if (++res->point_index >= IDENT_MECH_RES_POINT_COUNT) {
                    ident_mech_res_set_q(0.0f);
                    ident_mech_res_finish_sweep(false);
                    return;
                }
                ident_mech_res_prepare_point();
            }
            break;

        default:
            break;
    }
}

static bool ident_mech_res_tune(void)
{
    ident_mech_resonance_t *res = &ident_mech->resonance;
    float mean_x = 0.0f;
    float mean_y = 0.0f;
    float sxx = 0.0f;
    float sxy = 0.0f;
    float best_prominence = -INFINITY;
    int16_t peak_index = -1;
    uint16_t valid_count = 0U;

    for (uint16_t i = 0U; i < IDENT_MECH_RES_POINT_COUNT; i++) {
        const ident_mech_res_point_t *point = &res->points[i];

        if ((!point->valid) || (point->gain <= 0.0f)) {
            continue;
        }
        mean_x += logf(point->frequency_hz);
        mean_y += logf(point->gain);
        valid_count++;
    }
    res->valid_point_count = valid_count;
    if (valid_count < 5U) {
        res->error_code = IDENT_MECH_RES_ERROR_POINTS;
        return false;
    }
    mean_x /= (float)valid_count;
    mean_y /= (float)valid_count;

    for (uint16_t i = 0U; i < IDENT_MECH_RES_POINT_COUNT; i++) {
        const ident_mech_res_point_t *point = &res->points[i];

        if ((!point->valid) || (point->gain <= 0.0f)) {
            continue;
        }
        const float x = logf(point->frequency_hz);
        const float y = logf(point->gain);

        sxx += (x - mean_x) * (x - mean_x);
        sxy += (x - mean_x) * (y - mean_y);
    }
    if (sxx <= 0.0f) {
        res->error_code = IDENT_MECH_RES_ERROR_BASELINE;
        return false;
    }
    const float slope = sxy / sxx;
    const float intercept = mean_y - (slope * mean_x);

    for (uint16_t i = 1U; i + 1U < IDENT_MECH_RES_POINT_COUNT; i++) {
        const ident_mech_res_point_t *previous = &res->points[i - 1U];
        const ident_mech_res_point_t *point = &res->points[i];
        const ident_mech_res_point_t *next = &res->points[i + 1U];

        if ((!previous->valid) || (!point->valid) || (!next->valid)) {
            continue;
        }
        if ((point->gain < previous->gain) || (point->gain < next->gain)) {
            continue;
        }
        const float baseline = (slope * logf(point->frequency_hz)) + intercept;
        const float prominence_db =
            (20.0f / IDENT_MECH_LN10) * (logf(point->gain) - baseline);

        if (prominence_db > best_prominence) {
            best_prominence = prominence_db;
            peak_index = (int16_t)i;
        }
    }
    res->best_prominence_db = best_prominence;
    if ((peak_index < 1) ||
        (best_prominence < IDENT_MECH_RES_MIN_PROMINENCE_DB)) {
        res->error_code = IDENT_MECH_RES_ERROR_PEAK;
        return false;
    }

    const uint16_t center = (uint16_t)peak_index;
    const ident_mech_res_point_t *lower = &res->points[center - 1U];
    const ident_mech_res_point_t *middle = &res->points[center];
    const ident_mech_res_point_t *upper = &res->points[center + 1U];
    const float x1 = logf(middle->frequency_hz);
    const float x2 = logf(upper->frequency_hz);
    const float y0 = logf(lower->gain);
    const float y1 = logf(middle->gain);
    const float y2 = logf(upper->gain);
    const float denominator = y0 - (2.0f * y1) + y2;
    float peak_frequency_hz = middle->frequency_hz;

    if (fabsf(denominator) > 1.0e-9f) {
        const float offset = 0.5f * (y0 - y2) / denominator;

        if (fabsf(offset) <= 0.5f) {
            peak_frequency_hz = expf(x1 + (offset * (x2 - x1)));
        }
    }

    const float delta_phase =
        ident_mech_res_wrap_phase(upper->phase_rad - lower->phase_rad);
    const float delta_frequency = upper->frequency_hz - lower->frequency_hz;
    float damping_ratio = IDENT_MECH_RES_DEFAULT_DAMPING;
    float quality_factor = 0.0f;

    if (delta_frequency > 0.0f) {
        quality_factor =
            -0.5f * peak_frequency_hz * delta_phase / delta_frequency;
        if ((quality_factor >= IDENT_MECH_RES_MIN_Q) &&
            (quality_factor <= IDENT_MECH_RES_MAX_Q)) {
            damping_ratio = ident_mech_clamp(1.0f / (2.0f * quality_factor),
                                             IDENT_MECH_RES_MIN_DAMPING,
                                             IDENT_MECH_RES_MAX_DAMPING);
        }
    }

    res->tune_frequency_hz = peak_frequency_hz;
    res->tune_damping_ratio = damping_ratio;

    /* 辨识结果直接落到 FLASH，并立即刷新位置环整形器。 */
    if (mc_runtime_set_shaper((uint8_t)INPUT_SHAPER_TYPE_EI, peak_frequency_hz,
                              damping_ratio) == 0U) {
        res->error_code = IDENT_MECH_RES_ERROR_SHAPER;
        return false;
    }
    if (mc_position_control_apply_shaper_config() == 0U) {
        (void)mc_runtime_set_shaper((uint8_t)INPUT_SHAPER_TYPE_NONE, 0.0f,
                                    0.0f);
        (void)mc_position_control_apply_shaper_config();
        res->error_code = IDENT_MECH_RES_ERROR_SHAPER;
        return false;
    }

    ident_mech->result.resonance_frequency_hz = peak_frequency_hz;
    ident_mech->result.resonance_damping_ratio = damping_ratio;
    ident_mech->result.resonance_quality_factor = quality_factor;
    ident_mech->result.resonance_prominence_db = best_prominence;
    ident_mech->result.resonance_error_code = IDENT_MECH_RES_ERROR_NONE;
    ident_mech->result.resonance_valid = true;
    res->error_code = IDENT_MECH_RES_ERROR_NONE;
    return true;
}

static uint8_t
ident_mech_record_is_repeatable(const ident_mech_record_t *record)
{
    return (ident_mech_is_finite(record->gain_std_percent) &&
            ident_mech_is_finite(record->phase_std_deg) &&
            (record->gain_std_percent <=
             IDENT_MECH_QUALITY_GAIN_STD_MAX_PERCENT) &&
            (record->phase_std_deg <= IDENT_MECH_QUALITY_PHASE_STD_MAX_DEG))
        ? 1U
        : 0U;
}

static void ident_mech_reset(void)
{
    const uint32_t control_rate_hz = ident_mech->control_rate_hz;

    memset(ident_mech, 0, sizeof(*ident_mech));
    ident_mech->control_rate_hz = control_rate_hz;
    ident_mech->return_delay_tick_limit =
        (uint32_t)(IDENT_MECH_RETURN_DELAY_S * (float)control_rate_hz);
    ident_mech->return_stable_tick_limit =
        (uint32_t)(IDENT_MECH_RETURN_STABLE_TIME_S * (float)control_rate_hz);
    ident_mech->return_tick_limit =
        (uint32_t)(IDENT_MECH_RETURN_TIMEOUT_MIN_S * (float)control_rate_hz);
    ident_mech->state = IDENT_MECH_IDLE;
}

int8_t identify_mechanical_init(void)
{
    memset(ident_mech, 0, sizeof(*ident_mech));
    ident_mech->state = IDENT_MECH_IDLE;
    ident_mech->control_rate_hz = ident_mech_target_rate_hz;
    ident_mech_reset();
    return 0;
}

int8_t identify_mechanical_start(void)
{
    return identify_mechanical_start_with_amplitudes(
        ident_mech_default_amplitudes_a[0U],
        ident_mech_default_amplitudes_a[1U]);
}

int8_t identify_mechanical_start_with_amplitudes(float low_amplitude_a,
                                                 float high_amplitude_a)
{
    if (identify_mechanical_is_active() ||
        (mc_runtime_get_mode() != MC_MODE_IDLE) ||
        (motor_ctrl.fault_type != MOTOR_FAULT_NONE)) {
        log_w("ident_M not ready");
        return -1;
    }
    if ((ident_mech->control_rate_hz == 0U)) {
        log_e("ident_M not initialized");
        return -1;
    }
    if (((identify_resistance_get()->state != IDENT_R_IDLE) &&
         (identify_resistance_get()->state != IDENT_R_DONE) &&
         (identify_resistance_get()->state != IDENT_R_ERROR)) ||
        ((identify_inductance_get()->state != IDENT_L_IDLE) &&
         (identify_inductance_get()->state != IDENT_L_DONE) &&
         (identify_inductance_get()->state != IDENT_L_ERROR))) {
        log_w("electrical identification already running");
        return -1;
    }
    if ((!ident_mech_is_finite(low_amplitude_a)) ||
        (!ident_mech_is_finite(high_amplitude_a)) ||
        (low_amplitude_a <= 0.0f) || (low_amplitude_a >= high_amplitude_a) ||
        (high_amplitude_a > CURRENT_LIMIT)) {
        log_e("invalid amplitudes:%.3f %.3fA, expected 0<low<high<=%.1fA",
              (double)low_amplitude_a, (double)high_amplitude_a,
              (double)CURRENT_LIMIT);
        return -1;
    }

    const uint32_t primask = __get_PRIMASK();

    __disable_irq();
    ident_mech_reset();
    ident_mech->excitation_amplitude_a[0U] = low_amplitude_a;
    ident_mech->excitation_amplitude_a[1U] = high_amplitude_a;
    ident_mech->start_position = ident_mech_current_position();
    ident_mech->frequency_index = 0U;
    ident_mech_res_set_q(0.0f);
    ident_mech_prepare_frequency();
    if (primask == 0U) {
        __enable_irq();
    }
    return 0;
}

void identify_mechanical_stop(void)
{
    const uint32_t primask = __get_PRIMASK();

    __disable_irq();
    ident_mech_res_set_q(0.0f);
    ident_mech_reset();
    if (primask == 0U) {
        __enable_irq();
    }
}

void identify_mechanical_acknowledge(void)
{
    const uint32_t primask = __get_PRIMASK();

    __disable_irq();
    ident_mech_res_set_q(0.0f);
    if ((ident_mech->state == IDENT_MECH_DONE) ||
        (ident_mech->state == IDENT_MECH_ERROR)) {
        __DMB();
        ident_mech->state = IDENT_MECH_IDLE;
    }
    if (primask == 0U) {
        __enable_irq();
    }
}

bool identify_mechanical_is_active(void)
{
    return (ident_mech->state != IDENT_MECH_IDLE) &&
        (ident_mech->state != IDENT_MECH_DONE) &&
        (ident_mech->state != IDENT_MECH_ERROR);
}

bool identify_mechanical_is_returning(void)
{
    return ident_mech->state == IDENT_MECH_RETURN_POSITION;
}

float identify_mechanical_get_start_position(void)
{
    return ident_mech->start_position;
}

ident_mech_state_t identify_mechanical_get_state(void)
{
    return ident_mech->state;
}

const identify_mechanical_result_t *identify_mechanical_get_result(void)
{
    return &ident_mech->result;
}

static void ident_mech_capture_tick(void)
{
    const ident_mech_frequency_t *frequency =
        &ident_mech_frequency_table[ident_mech->frequency_index];
    const float iq_amplitude_a =
        ident_mech->excitation_amplitude_a[frequency->amplitude_index];

    mc_ctrl.target_d = 0.0f;
    mc_ctrl.target_q = iq_amplitude_a * sinf(ident_mech->phase);
    ident_mech->phase += ident_mech->phase_increment;
    if (ident_mech->phase >= _2PI) {
        ident_mech->phase -= _2PI;
    }

    if (ident_mech->state == IDENT_MECH_SETTLE) {
        if (++ident_mech->settle_tick >= ident_mech->settle_tick_limit) {
            ident_mech->sample_index = 0U;
            ident_mech->sample_divider_counter = 0U;
            ident_mech->state = IDENT_MECH_CAPTURE;
        }
        return;
    }

    if (++ident_mech->sample_divider_counter < frequency->sample_divider) {
        return;
    }
    ident_mech->sample_divider_counter = 0U;
    const float iq = mc_ctrl.abc_dq.current_q;
    const float position = ident_mech_current_position();

    ident_mech_ls_add_sample(frequency, iq, position);

    if (++ident_mech->sample_index < IDENT_MECH_WINDOW_SIZE) {
        return;
    }
    ident_mech->window_index++;
    if (ident_mech_finish_window() == 0U) {
        ident_mech_set_failed(IDENT_MECH_ERROR_CAPTURE);
        return;
    }
    if (ident_mech->valid_window_count >= frequency->window_count) {
        ident_mech_res_set_q(0.0f);
        ident_mech->return_delay_tick = 0U;
        __DMB();
        ident_mech->state = IDENT_MECH_RETURN_DELAY;
        return;
    }

    ident_mech->sample_index = 0U;
    ident_mech->sample_divider_counter = 0U;
    ident_mech_ls_reset();
}

static void ident_mech_return_tick(float position, float speed)
{
    // 位置斜坡目标始终是本次辨识开始前记录的位置。
    if (motor_ctrl.fault_type != MOTOR_FAULT_NONE) {
        ident_mech_set_failed(IDENT_MECH_ERROR_CONTROL_MODE);
        return;
    }
    ident_mech->return_position_error = position - ident_mech->start_position;
    ident_mech->return_speed = speed;
    ident_mech->return_tick++;

    if ((fabsf(ident_mech->return_position_error) <
         IDENT_MECH_RETURN_POSITION_ERROR_RAD) &&
        (fabsf(ident_mech->return_speed) <
         IDENT_MECH_RETURN_SPEED_ABORT_RAD_S)) {
        ident_mech->return_stable_tick++;
    }
    else {
        ident_mech->return_stable_tick = 0U;
    }

    if (ident_mech->return_tick >= ident_mech->return_tick_limit) {
        log_e("return timeout point=%u err=%.4f speed=%.4f",
              (unsigned int)(ident_mech->frequency_index + 1U),
              (double)ident_mech->return_position_error,
              (double)ident_mech->return_speed);
        ident_mech_set_failed(IDENT_MECH_ERROR_RETURN_TIMEOUT);
        return;
    }
    if (ident_mech->return_stable_tick < ident_mech->return_stable_tick_limit) {
        return;
    }

    ident_mech_res_set_q(0.0f);
    ident_mech->return_stable_tick = 0U;
    ident_mech->return_tick = 0U;
    if (ident_mech->fail_after_return != 0U) {
        ident_mech_set_failed(IDENT_MECH_ERROR_REPEATABILITY);
        return;
    }
    if (ident_mech->frequency_retry_pending != 0U) {
        ident_mech->frequency_retry_pending = 0U;
        ident_mech_prepare_frequency();
    }
    else if (++ident_mech->frequency_index >= IDENT_MECH_FREQUENCY_COUNT) {
        __DMB();
        ident_mech->state = IDENT_MECH_FIT_PENDING;
    }
    else {
        ident_mech->frequency_retry_count = 0U;
        ident_mech_prepare_frequency();
    }
}

void identify_mechanical_process(void)
{
    if (!identify_mechanical_is_active()) {
        ident_mech_res_set_q(0.0f);
        return;
    }
    if (ident_mech->state == IDENT_MECH_RESONANCE_PENDING) {
        ident_mech_res_set_q(0.0f);
        return;
    }
    if (ident_mech->state == IDENT_MECH_RESONANCE) {
        ident_mech_resonance_tick();
        return;
    }
    if ((ident_mech->state != IDENT_MECH_SETTLE) &&
        (ident_mech->state != IDENT_MECH_CAPTURE) &&
        (ident_mech->state != IDENT_MECH_RETURN_POSITION) &&
        (ident_mech->state != IDENT_MECH_RETURN_DELAY) &&
        (ident_mech->state != IDENT_MECH_RESULT_PENDING) &&
        (ident_mech->state != IDENT_MECH_FIT_PENDING)) {
        ident_mech_res_set_q(0.0f);
        return;
    }
    if (motor_ctrl.fault_type != MOTOR_FAULT_NONE) {
        ident_mech_set_failed(IDENT_MECH_ERROR_CAPTURE);
        return;
    }

    switch (ident_mech->state) {
        case IDENT_MECH_SETTLE:
        case IDENT_MECH_CAPTURE:
            ident_mech_capture_tick();
            break;

        case IDENT_MECH_RETURN_POSITION:
            ident_mech_res_set_q(0.0f);
            break;

        case IDENT_MECH_RETURN_DELAY:
            ident_mech_res_set_q(0.0f);
            if (++ident_mech->return_delay_tick >=
                ident_mech->return_delay_tick_limit) {
                __DMB();
                ident_mech->state = IDENT_MECH_RESULT_PENDING;
            }
            break;

        case IDENT_MECH_RESULT_PENDING:
        case IDENT_MECH_FIT_PENDING:
            ident_mech_res_set_q(0.0f);
            break;

        default:
            ident_mech_res_set_q(0.0f);
            break;
    }
}

void identify_mechanical_return_feedback_tick(float position, float speed)
{
    if (ident_mech->state != IDENT_MECH_RETURN_POSITION) {
        return;
    }
    ident_mech_return_tick(position, speed);
}

static void ident_mech_finish_identification(void)
{
    const uint32_t primask = __get_PRIMASK();
    const bool any_valid = ident_mech->result.low_frequency_valid ||
        ident_mech->result.resonance_valid;

    __disable_irq();
    ident_mech->state = any_valid ? IDENT_MECH_DONE : IDENT_MECH_ERROR;
    if (primask == 0U) {
        __enable_irq();
    }
}

static void ident_mech_log_resonance_curve(const ident_mech_resonance_t *res)
{
    const ident_mech_res_point_t *first = NULL;
    const ident_mech_res_point_t *last = NULL;
    const ident_mech_res_point_t *maximum = NULL;
    uint16_t maximum_index = 0U;

    for (uint16_t i = 0U; i < IDENT_MECH_RES_POINT_COUNT; i++) {
        const ident_mech_res_point_t *point = &res->points[i];

        if ((!point->valid) || (point->gain <= 0.0f)) {
            continue;
        }
        if (first == NULL) {
            first = point;
        }
        last = point;
        if ((maximum == NULL) || (point->gain > maximum->gain)) {
            maximum = point;
            maximum_index = i;
        }
    }
    if (maximum == NULL) {
        return;
    }

    log_e("res curve max=%.6f@%.2fHz start=%.6f end=%.6f at=%s",
          (double)maximum->gain, (double)maximum->frequency_hz,
          (double)first->gain, (double)last->gain,
          (maximum_index == 0U)
              ? "low-edge"
              : ((maximum_index == (IDENT_MECH_RES_POINT_COUNT - 1U))
                     ? "high-edge"
                     : "interior"));
    for (uint16_t i = 0U; i < IDENT_MECH_RES_POINT_COUNT; i++) {
        const ident_mech_res_point_t *point = &res->points[i];

        if ((!point->valid) || (point->gain <= 0.0f)) {
            log_e("res pt %02u invalid", (unsigned int)i);
            continue;
        }
        log_e("res pt %02u f=%.2fHz g=%.6f ph=%.1fdeg", (unsigned int)i,
              (double)point->frequency_hz, (double)point->gain,
              (double)(point->phase_rad * (180.0f / PI)));
    }
}

static void ident_mech_log_resonance_failure(const ident_mech_resonance_t *res)
{
    switch (res->error_code) {
        case IDENT_MECH_RES_ERROR_ABORTED:
            log_e("resonance failed: aborted fault=%u",
                  (unsigned int)res->abort_fault);
            break;

        case IDENT_MECH_RES_ERROR_POINTS:
            log_e("resonance failed: valid points=%u/%u",
                  (unsigned int)res->valid_point_count,
                  (unsigned int)IDENT_MECH_RES_POINT_COUNT);
            break;

        case IDENT_MECH_RES_ERROR_BASELINE:
            log_e("resonance failed: invalid gain baseline valid=%u/%u",
                  (unsigned int)res->valid_point_count,
                  (unsigned int)IDENT_MECH_RES_POINT_COUNT);
            break;

        case IDENT_MECH_RES_ERROR_PEAK:
            if (ident_mech_is_finite(res->best_prominence_db)) {
                log_e("resonance failed: no peak %.1f-%.1fHz valid=%u/%u "
                      "prom=%.2fdB min=%.1fdB",
                      (double)IDENT_MECH_RES_START_HZ,
                      (double)IDENT_MECH_RES_STOP_HZ,
                      (unsigned int)res->valid_point_count,
                      (unsigned int)IDENT_MECH_RES_POINT_COUNT,
                      (double)res->best_prominence_db,
                      (double)IDENT_MECH_RES_MIN_PROMINENCE_DB);
            }
            else {
                log_e("resonance failed: no peak %.1f-%.1fHz valid=%u/%u "
                      "prom=none",
                      (double)IDENT_MECH_RES_START_HZ,
                      (double)IDENT_MECH_RES_STOP_HZ,
                      (unsigned int)res->valid_point_count,
                      (unsigned int)IDENT_MECH_RES_POINT_COUNT);
            }
            ident_mech_log_resonance_curve(res);
            break;

        case IDENT_MECH_RES_ERROR_SHAPER:
            log_e("resonance failed: shaper config f=%.2fHz zeta=%.3f",
                  (double)res->tune_frequency_hz,
                  (double)res->tune_damping_ratio);
            break;

        default:
            log_e("resonance failed: code=%u valid=%u/%u",
                  (unsigned int)res->error_code,
                  (unsigned int)res->valid_point_count,
                  (unsigned int)IDENT_MECH_RES_POINT_COUNT);
            break;
    }
}

static void ident_mech_start_resonance(void)
{
    const uint32_t primask = __get_PRIMASK();

    __disable_irq();
    if (ident_mech->state != IDENT_MECH_RESONANCE_PENDING) {
        if (primask == 0U) {
            __enable_irq();
        }
        return;
    }
    __DMB();
    ident_mech->state = IDENT_MECH_RESONANCE;
    if (primask == 0U) {
        __enable_irq();
    }

    if (ident_mech_res_start() == 0) {
        return;
    }

    ident_mech->result.resonance_valid = false;
    ident_mech->result.resonance_error_code = IDENT_MECH_RES_ERROR_START;
    log_e("resonance failed: start fault=%u rate=%luHz",
          (unsigned int)motor_ctrl.fault_type,
          (unsigned long)ident_mech->control_rate_hz);
    ident_mech_finish_identification();
}

void identify_mechanical_background_process(void)
{
    if (ident_mech->state == IDENT_MECH_RESULT_PENDING) {
        // 先把频点结果保存到 RAM，再回到公共起始位置；被拒绝的频点
        // 在返回完成后重试。
        const float current_position = ident_mech_current_position();
        const ident_mech_record_t *record =
            &ident_mech->records[ident_mech->frequency_index];
        const float return_distance =
            fabsf(current_position - ident_mech->start_position);
        float timeout_s =
            ((2.0f * return_distance) / IDENT_MECH_RETURN_VELOCITY_RAD_S) +
            5.0f;

        if (ident_mech_store_pending_record() == 0U) {
            ident_mech_set_failed(IDENT_MECH_ERROR_CAPTURE);
            return;
        }
        if ((ident_mech_record_is_repeatable(record) == 0U) &&
            (ident_mech->frequency_retry_count <
             IDENT_MECH_REPEATABILITY_MAX_RETRIES)) {
            ident_mech->frequency_retry_count++;
            ident_mech->frequency_retry_pending = 1U;
        }
        else {
            ident_mech->frequency_retry_pending = 0U;
            if (ident_mech_record_is_repeatable(record) == 0U) {
                ident_mech->fail_after_return = 1U;
            }
        }
        if (timeout_s < IDENT_MECH_RETURN_TIMEOUT_MIN_S) {
            timeout_s = IDENT_MECH_RETURN_TIMEOUT_MIN_S;
        }
        if (timeout_s > IDENT_MECH_RETURN_TIMEOUT_MAX_S) {
            timeout_s = IDENT_MECH_RETURN_TIMEOUT_MAX_S;
        }

        const uint32_t primask = __get_PRIMASK();

        __disable_irq();
        if ((ident_mech->state != IDENT_MECH_RESULT_PENDING) ||
            (motor_ctrl.fault_type != MOTOR_FAULT_NONE)) {
            if (primask == 0U) {
                __enable_irq();
            }
            return;
        }
        ident_mech->return_stable_tick = 0U;
        ident_mech->return_tick = 0U;
        ident_mech->return_tick_limit =
            (uint32_t)(timeout_s * (float)ident_mech->control_rate_hz);
        ident_mech->return_position_error =
            current_position - ident_mech->start_position;
        ident_mech->return_speed = encoder_data.mechanical_speed;
        __DMB();
        ident_mech->state = IDENT_MECH_RETURN_POSITION;
        if (primask == 0U) {
            __enable_irq();
        }
        return;
    }

    if (ident_mech->state == IDENT_MECH_RESONANCE_PENDING) {
        ident_mech_start_resonance();
        return;
    }

    if (ident_mech->state == IDENT_MECH_RESONANCE) {
        ident_mech_resonance_t *res = &ident_mech->resonance;

        if (!res->done_pending) {
            return;
        }
        if (ident_mech->state != IDENT_MECH_RESONANCE) {
            return;
        }
        const bool resonance_ok = (!res->aborted) && ident_mech_res_tune();
        const uint32_t primask = __get_PRIMASK();
        bool processed = false;

        __disable_irq();
        if (ident_mech->state == IDENT_MECH_RESONANCE) {
            res->done_pending = false;
            if (resonance_ok) {
                ident_mech->result.resonance_valid = true;
                ident_mech->result.resonance_error_code =
                    IDENT_MECH_RES_ERROR_NONE;
            }
            else {
                ident_mech->result.resonance_valid = false;
                ident_mech->result.resonance_error_code = res->aborted
                    ? IDENT_MECH_RES_ERROR_ABORTED
                    : res->error_code;
            }
            processed = true;
        }
        if (primask == 0U) {
            __enable_irq();
        }
        if (!processed) {
            return;
        }
        if (resonance_ok) {
            log_i("resonance f=%.2fHz zeta=%.3f Q=%.2f peak=%.1fdB",
                  (double)ident_mech->result.resonance_frequency_hz,
                  (double)ident_mech->result.resonance_damping_ratio,
                  (double)ident_mech->result.resonance_quality_factor,
                  (double)ident_mech->result.resonance_prominence_db);
        }
        else {
            ident_mech_log_resonance_failure(res);
        }
        ident_mech_finish_identification();
        return;
    }

    if (ident_mech->state != IDENT_MECH_FIT_PENDING) {
        return;
    }

    ident_mech_res_set_q(0.0f);
    const uint8_t fit_ok = ident_mech_fit_parameters();
    const uint32_t primask = __get_PRIMASK();

    __disable_irq();
    if (ident_mech->state != IDENT_MECH_FIT_PENDING) {
        if (primask == 0U) {
            __enable_irq();
        }
        return;
    }
    const bool control_ok = motor_ctrl.fault_type == MOTOR_FAULT_NONE;
    const bool low_ok = (fit_ok != 0U) && control_ok;

    if (!low_ok) {
        ident_mech->result.low_frequency_valid = false;
        if (!control_ok) {
            ident_mech->result.low_frequency_error_code =
                IDENT_MECH_ERROR_CONTROL_MODE;
        }
    }
    __DMB();
    ident_mech->state = IDENT_MECH_RESONANCE_PENDING;
    if (primask == 0U) {
        __enable_irq();
    }

    if (low_ok) {
        if (mc_runtime_set_mechanical(ident_mech->result.ka,
                                      ident_mech->result.kv,
                                      ident_mech->result.tc) == 0U) {
            log_e("low FLASH write failed");
        }
        log_i("low Ka=%.6f Kv=%.6f Tc=%.4fA RMS=%.1f%%",
              (double)ident_mech->result.ka, (double)ident_mech->result.kv,
              (double)ident_mech->result.tc,
              (double)ident_mech->result.weighted_rms_percent);
    }
    else {
        log_e("low failed: %s code=%u Ka=%.6f Kv=%.6f Tc=%.4fA RMS=%.1f%% "
              "fault=%u",
              ident_mech_low_error_name(
                  ident_mech->result.low_frequency_error_code),
              (unsigned int)ident_mech->result.low_frequency_error_code,
              (double)ident_mech->result.ka, (double)ident_mech->result.kv,
              (double)ident_mech->result.tc,
              (double)ident_mech->result.weighted_rms_percent,
              (unsigned int)motor_ctrl.fault_type);
    }
}
