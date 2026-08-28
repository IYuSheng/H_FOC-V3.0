#include "foc_mechanical_param.h"

#include <math.h>
#include <string.h>

#include "Config.h"
#include "foc_control.h"
#include "stm32g4xx.h"

#define MECH_CONTROL_RATE_HZ             5000U
#define MECH_SETTLE_CYCLES               1U
#define MECH_LS_BASIS_COUNT              4U
#define MECH_LS_PIVOT_MIN                1.0e-6f
#define MECH_MIN_SETTLE_TICKS            (MECH_CONTROL_RATE_HZ / 2U)
#define MECH_RETURN_POSITION_ERROR_RAD   0.02f
#define MECH_RETURN_SPEED_RAD_S          0.25f
#define MECH_RETURN_STABLE_TICKS         (MECH_CONTROL_RATE_HZ / 20U)
#define MECH_RETURN_TIMEOUT_TICKS        (MECH_CONTROL_RATE_HZ * 15U)
#define MECH_INPUT_AMPLITUDE_MIN_A       1.0e-5f

#define MECH_QUALITY_GAIN_STD_MAX_PERCENT 5.0f
#define MECH_QUALITY_PHASE_STD_MAX_DEG    1.0f
#define MECH_QUALITY_MEDIAN_GAIN_MAX      1.0f
#define MECH_QUALITY_MEDIAN_PHASE_MAX     0.5f
#define MECH_QUALITY_SIGMA_FLOOR          0.01f
#define MECH_FIT_RMS_MAX_PERCENT          30.0f

#define MECH_NM_MAX_DIMENSION             3U
#define MECH_NM_MAX_VERTICES              (MECH_NM_MAX_DIMENSION + 1U)
#define MECH_NM_MAX_ITERATIONS            800U
#define MECH_NM_INITIAL_STEP              0.15f
#define MECH_NM_PARAMETER_TOLERANCE        1.0e-5f
#define MECH_NM_COST_TOLERANCE             1.0e-7f
#define MECH_INVALID_COST                  1.0e6f

#define MECH_ERROR_NONE                   0U
#define MECH_ERROR_CAPTURE                1U
#define MECH_ERROR_RETURN_TIMEOUT         2U
#define MECH_ERROR_REPEATABILITY          3U
#define MECH_ERROR_FIT                    4U
#define MECH_ERROR_FIT_QUALITY            5U

typedef struct
{
    float iq_amplitude_a;
    uint16_t sample_divider;
    uint16_t frequency_bin;
    uint8_t window_count;
} mech_frequency_t;

typedef struct
{
    float gram[MECH_LS_BASIS_COUNT][MECH_LS_BASIS_COUNT];
    float iq_rhs[MECH_LS_BASIS_COUNT];
    float position_rhs[MECH_LS_BASIS_COUNT];
    float iq_reference;
    float position_reference;
    uint8_t reference_valid;
} mech_ls_accumulator_t;

typedef struct
{
    float frequency_hz;
    float iq_amplitude_a;
    float position_amplitude_rad;
    float response_real;
    float response_imag;
    float gain_std_percent;
    float phase_std_deg;
    float weight;
    uint8_t window_count;
} mech_record_t;

typedef enum
{
    MECH_FIT_LINEAR_ALL = 0U,
    MECH_FIT_FRICTION_LOW,
    MECH_FIT_FRICTION_ALL
} mech_fit_mode_t;

typedef struct
{
    volatile mech_identify_state_t state;
    uint16_t frequency_index;
    uint16_t sample_index;
    uint16_t sample_divider_counter;
    uint8_t window_index;
    uint8_t valid_window_count;
    uint32_t settle_tick;
    uint32_t settle_tick_limit;
    uint32_t return_stable_tick;
    uint32_t return_tick;
    float start_position;
    float return_position_error;
    float return_speed;
    float phase;
    float phase_increment;
    float response_real_sum;
    float response_imag_sum;
    float input_amplitude_sum;
    float gain_sum;
    float gain_square_sum;
    float phase_unit_real_sum;
    float phase_unit_imag_sum;
    float fit_fixed_ka;
    mech_ls_accumulator_t ls;
    mech_record_t records[27U];
    mech_identify_result_t result;
} mech_context_t;

static const mech_frequency_t s_frequency_table[] = {
    { 0.140f, 100U, 10U, 2U },
    { 0.140f, 100U, 14U, 2U },
    { 0.140f, 100U, 20U, 2U },
    { 0.140f,  20U,  6U, 2U },
    { 0.140f,  20U,  8U, 2U },
    { 0.140f,  20U, 12U, 2U },

    { 0.140f,  20U, 20U, 2U },
    { 0.170f,  20U, 20U, 2U },
    { 0.200f,  20U, 20U, 2U },
    { 0.140f,  20U, 29U, 2U },
    { 0.170f,  20U, 29U, 2U },
    { 0.200f,  20U, 29U, 2U },
    { 0.140f,  20U, 41U, 2U },
    { 0.170f,  20U, 41U, 2U },
    { 0.200f,  20U, 41U, 2U },

    { 0.140f,   5U, 15U, 4U },
    { 0.170f,   5U, 15U, 4U },
    { 0.200f,   5U, 15U, 4U },
    { 0.140f,   5U, 20U, 4U },
    { 0.170f,   5U, 20U, 4U },
    { 0.200f,   5U, 20U, 4U },

    { 0.140f,   5U, 31U, 8U },
    { 0.170f,   5U, 31U, 8U },
    { 0.200f,   5U, 31U, 8U },
    { 0.140f,   5U, 41U, 8U },
    { 0.170f,   5U, 41U, 8U },
    { 0.200f,   5U, 41U, 8U },
};

#define MECH_FREQUENCY_COUNT \
    ((uint16_t)(sizeof(s_frequency_table) / sizeof(s_frequency_table[0])))

_Static_assert(MECH_FREQUENCY_COUNT == 27U,
               "mechanical identification table must contain 27 records");

static mech_context_t s_mech;

static float mech_actual_frequency_hz(const mech_frequency_t *frequency)
{
    return ((float)MECH_CONTROL_RATE_HZ *
            (float)frequency->frequency_bin) /
        ((float)frequency->sample_divider *
         (float)MECH_IDENTIFY_WINDOW_SIZE);
}

static void mech_set_failed(uint8_t error_code)
{
    foc_ctrl.target_q = 0.0f;
    foc_ctrl.target_d = 0.0f;
    s_mech.result.valid = false;
    s_mech.result.error_code = error_code;
    __DMB();
    s_mech.state = MECH_IDENTIFY_FAILED;
}

static void mech_ls_reset(void)
{
    memset(&s_mech.ls, 0, sizeof(s_mech.ls));
}

static void mech_prepare_frequency(void)
{
    const mech_frequency_t *frequency =
        &s_frequency_table[s_mech.frequency_index];
    const float actual_frequency_hz = mech_actual_frequency_hz(frequency);
    uint32_t settle_ticks = (uint32_t)(
        (((float)MECH_SETTLE_CYCLES * (float)MECH_CONTROL_RATE_HZ /
          actual_frequency_hz) + 0.5f));

    if (settle_ticks < MECH_MIN_SETTLE_TICKS) {
        settle_ticks = MECH_MIN_SETTLE_TICKS;
    }

    s_mech.sample_index = 0U;
    s_mech.sample_divider_counter = 0U;
    s_mech.window_index = 0U;
    s_mech.valid_window_count = 0U;
    s_mech.settle_tick = 0U;
    s_mech.settle_tick_limit = settle_ticks;
    s_mech.response_real_sum = 0.0f;
    s_mech.response_imag_sum = 0.0f;
    s_mech.input_amplitude_sum = 0.0f;
    s_mech.gain_sum = 0.0f;
    s_mech.gain_square_sum = 0.0f;
    s_mech.phase_unit_real_sum = 0.0f;
    s_mech.phase_unit_imag_sum = 0.0f;
    s_mech.phase = 0.0f;
    s_mech.phase_increment =
        _2PI * actual_frequency_hz / (float)MECH_CONTROL_RATE_HZ;
    mech_ls_reset();
    __DMB();
    s_mech.state = MECH_IDENTIFY_SETTLE;
}

static void mech_ls_add_sample(const mech_frequency_t *frequency,
                               float iq, float position)
{
    float basis[MECH_LS_BASIS_COUNT];
    const float angle = _2PI * (float)frequency->frequency_bin *
        (float)s_mech.sample_index / (float)MECH_IDENTIFY_WINDOW_SIZE;

    if (s_mech.ls.reference_valid == 0U) {
        s_mech.ls.iq_reference = iq;
        s_mech.ls.position_reference = position;
        s_mech.ls.reference_valid = 1U;
    }

    basis[0] = cosf(angle);
    basis[1] = sinf(angle);
    basis[2] = 1.0f;
    basis[3] = (2.0f * (float)s_mech.sample_index /
                (float)(MECH_IDENTIFY_WINDOW_SIZE - 1U)) - 1.0f;
    iq -= s_mech.ls.iq_reference;
    position -= s_mech.ls.position_reference;

    for (uint8_t row = 0U; row < MECH_LS_BASIS_COUNT; row++) {
        s_mech.ls.iq_rhs[row] += basis[row] * iq;
        s_mech.ls.position_rhs[row] += basis[row] * position;
        for (uint8_t column = 0U; column < MECH_LS_BASIS_COUNT; column++) {
            s_mech.ls.gram[row][column] += basis[row] * basis[column];
        }
    }
}

static uint8_t mech_ls_solve(float iq_coefficients[MECH_LS_BASIS_COUNT],
                             float position_coefficients[MECH_LS_BASIS_COUNT])
{
    float augmented[MECH_LS_BASIS_COUNT][MECH_LS_BASIS_COUNT + 2U];

    for (uint8_t row = 0U; row < MECH_LS_BASIS_COUNT; row++) {
        for (uint8_t column = 0U; column < MECH_LS_BASIS_COUNT; column++) {
            augmented[row][column] = s_mech.ls.gram[row][column];
        }
        augmented[row][MECH_LS_BASIS_COUNT] = s_mech.ls.iq_rhs[row];
        augmented[row][MECH_LS_BASIS_COUNT + 1U] =
            s_mech.ls.position_rhs[row];
    }

    for (uint8_t pivot_column = 0U;
         pivot_column < MECH_LS_BASIS_COUNT; pivot_column++) {
        uint8_t pivot_row = pivot_column;
        float pivot_magnitude = fabsf(augmented[pivot_row][pivot_column]);

        for (uint8_t row = (uint8_t)(pivot_column + 1U);
             row < MECH_LS_BASIS_COUNT; row++) {
            const float magnitude = fabsf(augmented[row][pivot_column]);
            if (magnitude > pivot_magnitude) {
                pivot_magnitude = magnitude;
                pivot_row = row;
            }
        }
        if (pivot_magnitude < MECH_LS_PIVOT_MIN) {
            return 0U;
        }

        if (pivot_row != pivot_column) {
            for (uint8_t column = 0U;
                 column < (MECH_LS_BASIS_COUNT + 2U); column++) {
                const float temporary = augmented[pivot_column][column];
                augmented[pivot_column][column] = augmented[pivot_row][column];
                augmented[pivot_row][column] = temporary;
            }
        }

        const float pivot = augmented[pivot_column][pivot_column];
        for (uint8_t column = pivot_column;
             column < (MECH_LS_BASIS_COUNT + 2U); column++) {
            augmented[pivot_column][column] /= pivot;
        }

        for (uint8_t row = 0U; row < MECH_LS_BASIS_COUNT; row++) {
            if (row == pivot_column) {
                continue;
            }
            const float scale = augmented[row][pivot_column];
            for (uint8_t column = pivot_column;
                 column < (MECH_LS_BASIS_COUNT + 2U); column++) {
                augmented[row][column] -=
                    scale * augmented[pivot_column][column];
            }
        }
    }

    for (uint8_t row = 0U; row < MECH_LS_BASIS_COUNT; row++) {
        iq_coefficients[row] = augmented[row][MECH_LS_BASIS_COUNT];
        position_coefficients[row] =
            augmented[row][MECH_LS_BASIS_COUNT + 1U];
    }
    return 1U;
}

static uint8_t mech_finish_window(void)
{
    float iq_coefficients[MECH_LS_BASIS_COUNT];
    float position_coefficients[MECH_LS_BASIS_COUNT];
    float input_real;
    float input_imag;
    float output_real;
    float output_imag;
    float input_power;
    float response_real;
    float response_imag;
    float response_gain;

    if (mech_ls_solve(iq_coefficients, position_coefficients) == 0U) {
        return 0U;
    }

    input_real = iq_coefficients[0];
    input_imag = -iq_coefficients[1];
    output_real = position_coefficients[0];
    output_imag = -position_coefficients[1];
    input_power = (input_real * input_real) + (input_imag * input_imag);
    if (input_power < (MECH_INPUT_AMPLITUDE_MIN_A *
                       MECH_INPUT_AMPLITUDE_MIN_A)) {
        return 0U;
    }

    response_real =
        ((output_real * input_real) + (output_imag * input_imag)) /
        input_power;
    response_imag =
        ((output_imag * input_real) - (output_real * input_imag)) /
        input_power;
    response_gain = sqrtf((response_real * response_real) +
                          (response_imag * response_imag));
    if ((!isfinite(response_gain)) || (response_gain <= 0.0f)) {
        return 0U;
    }

    s_mech.response_real_sum += response_real;
    s_mech.response_imag_sum += response_imag;
    s_mech.input_amplitude_sum += sqrtf(input_power);
    s_mech.gain_sum += response_gain;
    s_mech.gain_square_sum += response_gain * response_gain;
    s_mech.phase_unit_real_sum += response_real / response_gain;
    s_mech.phase_unit_imag_sum += response_imag / response_gain;
    s_mech.valid_window_count++;
    return 1U;
}

static void mech_sort_values(float values[], uint16_t count)
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

static float mech_median(const float source[], uint16_t count)
{
    float values[MECH_FREQUENCY_COUNT];

    if ((count == 0U) || (count > MECH_FREQUENCY_COUNT)) {
        return NAN;
    }
    memcpy(values, source, count * sizeof(float));
    mech_sort_values(values, count);
    if ((count & 1U) != 0U) {
        return values[count / 2U];
    }
    return 0.5f * (values[(count / 2U) - 1U] + values[count / 2U]);
}

static uint8_t mech_prepare_weights(void)
{
    float raw_quality[MECH_FREQUENCY_COUNT];
    float gain_std[MECH_FREQUENCY_COUNT];
    float phase_std[MECH_FREQUENCY_COUNT];

    for (uint16_t index = 0U; index < MECH_FREQUENCY_COUNT; index++) {
        const mech_record_t *record = &s_mech.records[index];
        float relative_standard_error;

        gain_std[index] = record->gain_std_percent;
        phase_std[index] = record->phase_std_deg;
        if ((!isfinite(record->gain_std_percent)) ||
            (!isfinite(record->phase_std_deg)) ||
            (record->gain_std_percent >
             MECH_QUALITY_GAIN_STD_MAX_PERCENT) ||
            (record->phase_std_deg > MECH_QUALITY_PHASE_STD_MAX_DEG)) {
            return 0U;
        }

        relative_standard_error = sqrtf(
            ((record->gain_std_percent / 100.0f) *
             (record->gain_std_percent / 100.0f)) +
            ((record->phase_std_deg * PI / 180.0f) *
             (record->phase_std_deg * PI / 180.0f))) /
            sqrtf((float)record->window_count);
        if (relative_standard_error < MECH_QUALITY_SIGMA_FLOOR) {
            relative_standard_error = MECH_QUALITY_SIGMA_FLOOR;
        }
        raw_quality[index] =
            1.0f / (relative_standard_error * relative_standard_error);
    }

    s_mech.result.median_gain_std_percent =
        mech_median(gain_std, MECH_FREQUENCY_COUNT);
    s_mech.result.median_phase_std_deg =
        mech_median(phase_std, MECH_FREQUENCY_COUNT);
    if ((s_mech.result.median_gain_std_percent >
         MECH_QUALITY_MEDIAN_GAIN_MAX) ||
        (s_mech.result.median_phase_std_deg >
         MECH_QUALITY_MEDIAN_PHASE_MAX)) {
        return 0U;
    }

    const float reference_quality =
        mech_median(raw_quality, MECH_FREQUENCY_COUNT);
    if ((!isfinite(reference_quality)) || (reference_quality <= 0.0f)) {
        return 0U;
    }

    for (uint16_t index = 0U; index < MECH_FREQUENCY_COUNT; index++) {
        float quality = raw_quality[index] / reference_quality;
        uint16_t frequency_record_count = 0U;

        if (quality < 0.25f) {
            quality = 0.25f;
        }
        if (quality > 4.0f) {
            quality = 4.0f;
        }
        for (uint16_t other = 0U; other < MECH_FREQUENCY_COUNT; other++) {
            if (fabsf(s_mech.records[other].frequency_hz -
                      s_mech.records[index].frequency_hz) < 1.0e-5f) {
                frequency_record_count++;
            }
        }
        if (frequency_record_count == 0U) {
            return 0U;
        }
        s_mech.records[index].weight =
            quality / (float)frequency_record_count;
    }
    return 1U;
}

static uint8_t mech_high_band_ka(float *ka)
{
    float numerator = 0.0f;
    float denominator = 0.0f;

    for (uint16_t index = 0U; index < MECH_FREQUENCY_COUNT; index++) {
        const mech_record_t *record = &s_mech.records[index];
        float response_power;
        float inverse_real;
        float inverse_magnitude;
        float angular_frequency;
        float coefficient;
        float target;

        if (record->frequency_hz < 10.0f) {
            continue;
        }
        response_power =
            (record->response_real * record->response_real) +
            (record->response_imag * record->response_imag);
        if (response_power <= 0.0f) {
            return 0U;
        }
        inverse_real = record->response_real / response_power;
        inverse_magnitude = 1.0f / sqrtf(response_power);
        angular_frequency = _2PI * record->frequency_hz;
        coefficient = -(angular_frequency * angular_frequency) /
            inverse_magnitude;
        target = inverse_real / inverse_magnitude;
        numerator += record->weight * coefficient * target;
        denominator += record->weight * coefficient * coefficient;
    }

    if ((!isfinite(numerator)) || (!isfinite(denominator)) ||
        (denominator <= 0.0f)) {
        return 0U;
    }
    *ka = numerator / denominator;
    return (isfinite(*ka) && (*ka > 0.0f)) ? 1U : 0U;
}

static uint8_t mech_model_response(float ka, float kv, float tc,
                                   const mech_record_t *record,
                                   uint8_t friction_enabled,
                                   float *response_real,
                                   float *response_imag)
{
    const float angular_frequency = _2PI * record->frequency_hz;
    const float inertia_term = ka * angular_frequency * angular_frequency;
    float denominator_real = -inertia_term;
    float denominator_imag = kv * angular_frequency;

    if (friction_enabled != 0U) {
        const float friction_fundamental = 4.0f * tc / PI;
        const float quadratic = (inertia_term * inertia_term) +
            (denominator_imag * denominator_imag);
        const float radicand =
            ((denominator_imag * friction_fundamental) *
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
        if ((!isfinite(position_amplitude)) ||
            (position_amplitude <= 0.0f)) {
            return 0U;
        }
        denominator_imag += friction_fundamental / position_amplitude;
    }

    const float denominator_power =
        (denominator_real * denominator_real) +
        (denominator_imag * denominator_imag);
    if ((!isfinite(denominator_power)) || (denominator_power <= 0.0f)) {
        return 0U;
    }
    *response_real = denominator_real / denominator_power;
    *response_imag = -denominator_imag / denominator_power;
    return (isfinite(*response_real) && isfinite(*response_imag)) ? 1U : 0U;
}

static float mech_fit_cost(const float parameters[MECH_NM_MAX_DIMENSION],
                           uint8_t dimension, mech_fit_mode_t mode)
{
    float ka;
    float kv;
    float tc = 0.0f;
    float weighted_error_sum = 0.0f;
    float weight_sum = 0.0f;
    uint8_t friction_enabled = 0U;

    if ((mode == MECH_FIT_LINEAR_ALL) && (dimension == 2U)) {
        ka = expf(parameters[0]);
        kv = expf(parameters[1]);
    }
    else if ((mode == MECH_FIT_FRICTION_LOW) && (dimension == 2U)) {
        ka = s_mech.fit_fixed_ka;
        kv = expf(parameters[0]);
        tc = expf(parameters[1]);
        friction_enabled = 1U;
    }
    else if ((mode == MECH_FIT_FRICTION_ALL) && (dimension == 3U)) {
        ka = expf(parameters[0]);
        kv = expf(parameters[1]);
        tc = expf(parameters[2]);
        friction_enabled = 1U;
    }
    else {
        return MECH_INVALID_COST;
    }

    if ((!isfinite(ka)) || (!isfinite(kv)) || (!isfinite(tc)) ||
        (ka < 1.0e-8f) || (ka > 0.1f) ||
        (kv < 1.0e-8f) || (kv > 1.0f) ||
        ((friction_enabled != 0U) &&
         ((tc < 1.0e-7f) || (tc > 0.2f)))) {
        return MECH_INVALID_COST;
    }

    for (uint16_t index = 0U; index < MECH_FREQUENCY_COUNT; index++) {
        const mech_record_t *record = &s_mech.records[index];
        float model_real;
        float model_imag;
        float measured_power;
        float error_real;
        float error_imag;

        if ((mode == MECH_FIT_FRICTION_LOW) &&
            (record->frequency_hz > 5.0f)) {
            continue;
        }
        if (mech_model_response(ka, kv, tc, record, friction_enabled,
                                &model_real, &model_imag) == 0U) {
            return MECH_INVALID_COST;
        }
        measured_power =
            (record->response_real * record->response_real) +
            (record->response_imag * record->response_imag);
        if (measured_power <= 0.0f) {
            return MECH_INVALID_COST;
        }
        error_real = model_real - record->response_real;
        error_imag = model_imag - record->response_imag;
        weighted_error_sum += record->weight *
            ((error_real * error_real) + (error_imag * error_imag)) /
            measured_power;
        weight_sum += record->weight;
    }

    if ((!isfinite(weighted_error_sum)) || (weight_sum <= 0.0f)) {
        return MECH_INVALID_COST;
    }
    return weighted_error_sum / weight_sum;
}

static void mech_nm_sort(float simplex[MECH_NM_MAX_VERTICES]
                                      [MECH_NM_MAX_DIMENSION],
                         float costs[MECH_NM_MAX_VERTICES],
                         uint8_t dimension)
{
    const uint8_t vertex_count = (uint8_t)(dimension + 1U);

    for (uint8_t index = 0U; index < vertex_count; index++) {
        for (uint8_t other = (uint8_t)(index + 1U);
             other < vertex_count; other++) {
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

static uint8_t mech_nelder_mead(float parameters[MECH_NM_MAX_DIMENSION],
                                uint8_t dimension, mech_fit_mode_t mode,
                                float *final_cost)
{
    float simplex[MECH_NM_MAX_VERTICES][MECH_NM_MAX_DIMENSION] = {{0.0f}};
    float costs[MECH_NM_MAX_VERTICES] = {0.0f};
    float centroid[MECH_NM_MAX_DIMENSION];
    float reflected[MECH_NM_MAX_DIMENSION];
    float candidate[MECH_NM_MAX_DIMENSION];
    const uint8_t vertex_count = (uint8_t)(dimension + 1U);

    if ((dimension == 0U) || (dimension > MECH_NM_MAX_DIMENSION)) {
        return 0U;
    }
    for (uint8_t vertex = 0U; vertex < vertex_count; vertex++) {
        for (uint8_t axis = 0U; axis < dimension; axis++) {
            simplex[vertex][axis] = parameters[axis];
        }
        if (vertex > 0U) {
            simplex[vertex][vertex - 1U] += MECH_NM_INITIAL_STEP;
        }
        costs[vertex] = mech_fit_cost(simplex[vertex], dimension, mode);
    }

    for (uint16_t iteration = 0U;
         iteration < MECH_NM_MAX_ITERATIONS; iteration++) {
        float parameter_spread = 0.0f;
        float reflected_cost;

        mech_nm_sort(simplex, costs, dimension);
        for (uint8_t vertex = 1U; vertex < vertex_count; vertex++) {
            for (uint8_t axis = 0U; axis < dimension; axis++) {
                const float difference =
                    fabsf(simplex[vertex][axis] - simplex[0][axis]);
                if (difference > parameter_spread) {
                    parameter_spread = difference;
                }
            }
        }
        if ((parameter_spread < MECH_NM_PARAMETER_TOLERANCE) &&
            (fabsf(costs[dimension] - costs[0]) <
             MECH_NM_COST_TOLERANCE)) {
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
        reflected_cost = mech_fit_cost(reflected, dimension, mode);

        if (reflected_cost < costs[0]) {
            for (uint8_t axis = 0U; axis < dimension; axis++) {
                candidate[axis] = centroid[axis] +
                    2.0f * (reflected[axis] - centroid[axis]);
            }
            const float candidate_cost =
                mech_fit_cost(candidate, dimension, mode);
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
            memcpy(simplex[dimension], reflected,
                   dimension * sizeof(float));
            costs[dimension] = reflected_cost;
            continue;
        }

        if (reflected_cost < costs[dimension]) {
            for (uint8_t axis = 0U; axis < dimension; axis++) {
                candidate[axis] = centroid[axis] +
                    0.5f * (reflected[axis] - centroid[axis]);
            }
            const float candidate_cost =
                mech_fit_cost(candidate, dimension, mode);
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
                mech_fit_cost(candidate, dimension, mode);
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
            costs[vertex] = mech_fit_cost(simplex[vertex], dimension, mode);
        }
    }

    mech_nm_sort(simplex, costs, dimension);
    if ((!isfinite(costs[0])) || (costs[0] >= MECH_INVALID_COST)) {
        return 0U;
    }
    memcpy(parameters, simplex[0], dimension * sizeof(float));
    *final_cost = costs[0];
    return 1U;
}

static float mech_unweighted_rms(float ka, float kv, float tc,
                                 float minimum_frequency,
                                 float maximum_frequency)
{
    float error_sum = 0.0f;
    uint16_t count = 0U;

    for (uint16_t index = 0U; index < MECH_FREQUENCY_COUNT; index++) {
        const mech_record_t *record = &s_mech.records[index];
        float model_real;
        float model_imag;
        float measured_power;

        if ((record->frequency_hz < minimum_frequency) ||
            (record->frequency_hz > maximum_frequency)) {
            continue;
        }
        if (mech_model_response(ka, kv, tc, record, 1U,
                                &model_real, &model_imag) == 0U) {
            return INFINITY;
        }
        measured_power =
            (record->response_real * record->response_real) +
            (record->response_imag * record->response_imag);
        error_sum +=
            (((model_real - record->response_real) *
              (model_real - record->response_real)) +
             ((model_imag - record->response_imag) *
              (model_imag - record->response_imag))) /
            measured_power;
        count++;
    }
    if (count == 0U) {
        return INFINITY;
    }
    return 100.0f * sqrtf(error_sum / (float)count);
}

static uint8_t mech_fit_parameters(void)
{
    float ka_stage;
    float linear_parameters[MECH_NM_MAX_DIMENSION] = {0.0f};
    float low_parameters[MECH_NM_MAX_DIMENSION] = {0.0f};
    float final_parameters[MECH_NM_MAX_DIMENSION] = {0.0f};
    float unused_cost;
    float final_cost;
    float ka;
    float kv;
    float tc;
    float minimum_iq = INFINITY;

    if (mech_prepare_weights() == 0U) {
        s_mech.result.error_code = MECH_ERROR_REPEATABILITY;
        return 0U;
    }
    if (mech_high_band_ka(&ka_stage) == 0U) {
        s_mech.result.error_code = MECH_ERROR_FIT;
        return 0U;
    }
    s_mech.result.Ka_high_band = ka_stage;

    linear_parameters[0] = logf(ka_stage);
    linear_parameters[1] = logf(1.0e-2f);
    if (mech_nelder_mead(linear_parameters, 2U, MECH_FIT_LINEAR_ALL,
                         &unused_cost) == 0U) {
        s_mech.result.error_code = MECH_ERROR_FIT;
        return 0U;
    }

    s_mech.fit_fixed_ka = ka_stage;
    low_parameters[0] = logf(fmaxf(0.5f * expf(linear_parameters[1]),
                                   1.0e-4f));
    low_parameters[1] = logf(0.03f);
    if (mech_nelder_mead(low_parameters, 2U, MECH_FIT_FRICTION_LOW,
                         &unused_cost) == 0U) {
        s_mech.result.error_code = MECH_ERROR_FIT;
        return 0U;
    }

    final_parameters[0] = logf(ka_stage);
    final_parameters[1] = low_parameters[0];
    final_parameters[2] = low_parameters[1];
    if (mech_nelder_mead(final_parameters, 3U, MECH_FIT_FRICTION_ALL,
                         &final_cost) == 0U) {
        s_mech.result.error_code = MECH_ERROR_FIT;
        return 0U;
    }

    ka = expf(final_parameters[0]);
    kv = expf(final_parameters[1]);
    tc = expf(final_parameters[2]);
    for (uint16_t index = 0U; index < MECH_FREQUENCY_COUNT; index++) {
        if (s_mech.records[index].iq_amplitude_a < minimum_iq) {
            minimum_iq = s_mech.records[index].iq_amplitude_a;
        }
    }

    s_mech.result.Ka = ka;
    s_mech.result.Kv = kv;
    s_mech.result.Tc = tc;
    s_mech.result.weighted_rms_percent = 100.0f * sqrtf(final_cost);
    s_mech.result.low_band_rms_percent =
        mech_unweighted_rms(ka, kv, tc, 0.0f, 5.0f);
    s_mech.result.high_band_rms_percent =
        mech_unweighted_rms(ka, kv, tc, 10.0f, INFINITY);
    s_mech.result.point_count = MECH_FREQUENCY_COUNT;

    if ((!isfinite(ka)) || (!isfinite(kv)) || (!isfinite(tc)) ||
        (!isfinite(s_mech.result.weighted_rms_percent)) ||
        (ka <= 0.0f) || (ka > 0.1f) ||
        (kv < 0.0f) || (kv > 1.0f) ||
        (tc < 0.0f) || (tc >= (0.95f * minimum_iq * PI / 4.0f)) ||
        (s_mech.result.weighted_rms_percent >
         MECH_FIT_RMS_MAX_PERCENT)) {
        s_mech.result.error_code = MECH_ERROR_FIT_QUALITY;
        return 0U;
    }

    s_mech.result.error_code = MECH_ERROR_NONE;
    s_mech.result.valid = true;
    return 1U;
}

static uint8_t mech_store_pending_record(void)
{
    const mech_frequency_t *frequency =
        &s_frequency_table[s_mech.frequency_index];
    mech_record_t *record = &s_mech.records[s_mech.frequency_index];
    const float window_count = (float)s_mech.valid_window_count;
    float response_gain;
    float gain_variance;
    float phase_resultant;

    if (s_mech.valid_window_count != frequency->window_count) {
        return 0U;
    }

    record->frequency_hz = mech_actual_frequency_hz(frequency);
    record->iq_amplitude_a = s_mech.input_amplitude_sum / window_count;
    record->response_real = s_mech.response_real_sum / window_count;
    record->response_imag = s_mech.response_imag_sum / window_count;
    response_gain = sqrtf(
        (record->response_real * record->response_real) +
        (record->response_imag * record->response_imag));
    record->position_amplitude_rad =
        record->iq_amplitude_a * response_gain;
    record->window_count = s_mech.valid_window_count;

    gain_variance = s_mech.gain_square_sum -
        ((s_mech.gain_sum * s_mech.gain_sum) / window_count);
    if (gain_variance < 0.0f) {
        gain_variance = 0.0f;
    }
    record->gain_std_percent =
        100.0f * sqrtf(gain_variance / (window_count - 1.0f)) /
        (s_mech.gain_sum / window_count);

    phase_resultant = sqrtf(
        (s_mech.phase_unit_real_sum * s_mech.phase_unit_real_sum) +
        (s_mech.phase_unit_imag_sum * s_mech.phase_unit_imag_sum)) /
        window_count;
    if (phase_resultant > 1.0f) {
        phase_resultant = 1.0f;
    }
    if (phase_resultant < 1.0e-6f) {
        phase_resultant = 1.0e-6f;
    }
    record->phase_std_deg =
        sqrtf(-2.0f * logf(phase_resultant)) * (180.0f / PI);

    return (isfinite(record->iq_amplitude_a) &&
            isfinite(record->position_amplitude_rad) &&
            isfinite(record->response_real) &&
            isfinite(record->response_imag) &&
            (record->iq_amplitude_a > 0.0f) &&
            (record->position_amplitude_rad > 0.0f))
        ? 1U
        : 0U;
}

void foc_mechanical_param_init(void)
{
    memset(&s_mech, 0, sizeof(s_mech));
    s_mech.state = MECH_IDENTIFY_IDLE;
}

void foc_mechanical_param_start(void)
{
    memset(&s_mech, 0, sizeof(s_mech));
    s_mech.start_position = encoder_data.mechanical_position;
    s_mech.frequency_index = 0U;
    foc_ctrl.target_d = 0.0f;
    foc_ctrl.target_q = 0.0f;
    mech_prepare_frequency();
}

void foc_mechanical_param_stop(void)
{
    const uint32_t primask = __get_PRIMASK();

    __disable_irq();
    foc_ctrl.target_q = 0.0f;
    foc_ctrl.target_d = 0.0f;
    memset(&s_mech, 0, sizeof(s_mech));
    s_mech.state = MECH_IDENTIFY_IDLE;
    if (primask == 0U) {
        __enable_irq();
    }
}

bool foc_mechanical_param_is_active(void)
{
    return s_mech.state != MECH_IDENTIFY_IDLE;
}

bool foc_mechanical_param_is_returning(void)
{
    return s_mech.state == MECH_IDENTIFY_RETURN_POSITION;
}

float foc_mechanical_param_get_start_position(void)
{
    return s_mech.start_position;
}

mech_identify_state_t foc_mechanical_param_get_state(void)
{
    return s_mech.state;
}

const mech_identify_result_t *foc_mechanical_param_get_result(void)
{
    return &s_mech.result;
}

void foc_mechanical_param_process(void)
{
    const mech_frequency_t *frequency;

    if ((s_mech.state != MECH_IDENTIFY_SETTLE) &&
        (s_mech.state != MECH_IDENTIFY_CAPTURE)) {
        foc_ctrl.target_q = 0.0f;
        return;
    }

    frequency = &s_frequency_table[s_mech.frequency_index];
    foc_ctrl.target_d = 0.0f;
    foc_ctrl.target_q = frequency->iq_amplitude_a * sinf(s_mech.phase);
    s_mech.phase += s_mech.phase_increment;
    if (s_mech.phase >= _2PI) {
        s_mech.phase -= _2PI;
    }

    if (s_mech.state == MECH_IDENTIFY_SETTLE) {
        if (++s_mech.settle_tick >= s_mech.settle_tick_limit) {
            s_mech.sample_index = 0U;
            s_mech.sample_divider_counter = 0U;
            s_mech.state = MECH_IDENTIFY_CAPTURE;
        }
        return;
    }

    if (++s_mech.sample_divider_counter < frequency->sample_divider) {
        return;
    }
    s_mech.sample_divider_counter = 0U;
    mech_ls_add_sample(frequency, foc_ctrl.abc_dq.current_q,
                       encoder_data.mechanical_position);

    if (++s_mech.sample_index < MECH_IDENTIFY_WINDOW_SIZE) {
        return;
    }
    if (mech_finish_window() == 0U) {
        mech_set_failed(MECH_ERROR_CAPTURE);
        return;
    }
    if (++s_mech.window_index >= frequency->window_count) {
        foc_ctrl.target_q = 0.0f;
        __DMB();
        s_mech.state = MECH_IDENTIFY_RESULT_PENDING;
        return;
    }

    s_mech.sample_index = 0U;
    s_mech.sample_divider_counter = 0U;
    mech_ls_reset();
}

void foc_mechanical_param_return_feedback_tick(float position, float speed)
{
    if (s_mech.state != MECH_IDENTIFY_RETURN_POSITION) {
        return;
    }

    s_mech.return_position_error = position - s_mech.start_position;
    s_mech.return_speed = speed;
    s_mech.return_tick++;
    if ((fabsf(s_mech.return_position_error) <
         MECH_RETURN_POSITION_ERROR_RAD) &&
        (fabsf(speed) < MECH_RETURN_SPEED_RAD_S)) {
        s_mech.return_stable_tick++;
    }
    else {
        s_mech.return_stable_tick = 0U;
    }

    if (s_mech.return_tick >= MECH_RETURN_TIMEOUT_TICKS) {
        mech_set_failed(MECH_ERROR_RETURN_TIMEOUT);
        return;
    }
    if (s_mech.return_stable_tick < MECH_RETURN_STABLE_TICKS) {
        return;
    }

    foc_ctrl.target_q = 0.0f;
    s_mech.return_stable_tick = 0U;
    s_mech.return_tick = 0U;
    if (++s_mech.frequency_index >= MECH_FREQUENCY_COUNT) {
        __DMB();
        s_mech.state = MECH_IDENTIFY_FIT_PENDING;
    }
    else {
        mech_prepare_frequency();
    }
}

void foc_mechanical_param_background_process(void)
{
    if (s_mech.state == MECH_IDENTIFY_RESULT_PENDING) {
        if (mech_store_pending_record() == 0U) {
            mech_set_failed(MECH_ERROR_CAPTURE);
            return;
        }
        s_mech.return_stable_tick = 0U;
        s_mech.return_tick = 0U;
        s_mech.return_position_error =
            encoder_data.mechanical_position - s_mech.start_position;
        s_mech.return_speed = encoder_data.mechanical_speed;
        __DMB();
        s_mech.state = MECH_IDENTIFY_RETURN_POSITION;
        return;
    }

    if (s_mech.state != MECH_IDENTIFY_FIT_PENDING) {
        return;
    }
    foc_ctrl.target_q = 0.0f;
    if (mech_fit_parameters() == 0U) {
        __DMB();
        s_mech.state = MECH_IDENTIFY_FAILED;
    }
    else {
        __DMB();
        s_mech.state = MECH_IDENTIFY_COMPLETE;
    }
}
