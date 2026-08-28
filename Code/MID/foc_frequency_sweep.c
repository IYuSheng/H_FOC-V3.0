#include "foc_frequency_sweep.h"

#include <math.h>
#include <string.h>

#include "Config.h"
#include "foc_control.h"
#include "usart.h"

#define FOC_SWEEP_CONTROL_RATE_HZ          5000U
#define FOC_SWEEP_SETTLE_CYCLES            1U
#define FOC_SWEEP_WAVEFORM_PRINT_ENABLE    0U
#define FOC_SWEEP_LS_BASIS_COUNT           4U
#define FOC_SWEEP_LS_PIVOT_MIN             1.0e-6f
#define FOC_SWEEP_MIN_SETTLE_TICKS         (FOC_SWEEP_CONTROL_RATE_HZ / 2U)
#define FOC_SWEEP_RETURN_POSITION_ERROR_RAD 0.02f
#define FOC_SWEEP_RETURN_SPEED_RAD_S        0.25f
#define FOC_SWEEP_RETURN_STABLE_TICKS       (FOC_SWEEP_CONTROL_RATE_HZ / 20U)
#define FOC_SWEEP_RETURN_TIMEOUT_TICKS       (FOC_SWEEP_CONTROL_RATE_HZ * 15U)
#define FOC_SWEEP_INPUT_AMPLITUDE_MIN_A     1.0e-5f

typedef struct
{
    float requested_frequency_hz;
    float iq_amplitude_a;
    uint16_t sample_divider;
    uint16_t fft_bin;
    uint8_t window_count;
} foc_sweep_frequency_t;

typedef struct
{
    float gram[FOC_SWEEP_LS_BASIS_COUNT][FOC_SWEEP_LS_BASIS_COUNT];
    float iq_rhs[FOC_SWEEP_LS_BASIS_COUNT];
    float position_rhs[FOC_SWEEP_LS_BASIS_COUNT];
    float iq_reference;
    float position_reference;
    uint8_t reference_valid;
} foc_sweep_ls_accumulator_t;

typedef struct
{
    volatile foc_sweep_state_t state;
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
    foc_sweep_ls_accumulator_t ls;
} foc_sweep_context_t;

static const foc_sweep_frequency_t s_frequency_table[] = {
    {  0.5f, 0.140f, 100U, 10U, 2U },
    {  0.7f, 0.140f, 100U, 14U, 2U },
    {  1.0f, 0.140f, 100U, 20U, 2U },
    {  1.5f, 0.140f,  20U,  6U, 2U },
    {  2.0f, 0.140f,  20U,  8U, 2U },
    {  3.0f, 0.140f,  20U, 12U, 2U },

    {  5.0f, 0.140f,  20U, 20U, 2U },
    {  5.0f, 0.170f,  20U, 20U, 2U },
    {  5.0f, 0.200f,  20U, 20U, 2U },
    {  7.0f, 0.140f,  20U, 29U, 2U },
    {  7.0f, 0.170f,  20U, 29U, 2U },
    {  7.0f, 0.200f,  20U, 29U, 2U },
    { 10.0f, 0.140f,  20U, 41U, 2U },
    { 10.0f, 0.170f,  20U, 41U, 2U },
    { 10.0f, 0.200f,  20U, 41U, 2U },

    { 15.0f, 0.140f,   5U, 15U, 4U },
    { 15.0f, 0.170f,   5U, 15U, 4U },
    { 15.0f, 0.200f,   5U, 15U, 4U },
    { 20.0f, 0.140f,   5U, 20U, 4U },
    { 20.0f, 0.170f,   5U, 20U, 4U },
    { 20.0f, 0.200f,   5U, 20U, 4U },

    { 30.0f, 0.140f,   5U, 31U, 8U },
    { 30.0f, 0.170f,   5U, 31U, 8U },
    { 30.0f, 0.200f,   5U, 31U, 8U },
    { 40.0f, 0.140f,   5U, 41U, 8U },
    { 40.0f, 0.170f,   5U, 41U, 8U },
    { 40.0f, 0.200f,   5U, 41U, 8U },
};

#define FOC_SWEEP_FREQUENCY_COUNT \
    ((uint16_t)(sizeof(s_frequency_table) / sizeof(s_frequency_table[0])))

static foc_sweep_context_t s_sweep;

static float sweep_actual_frequency_hz(const foc_sweep_frequency_t *frequency)
{
    return ((float)FOC_SWEEP_CONTROL_RATE_HZ * (float)frequency->fft_bin) /
        ((float)frequency->sample_divider * (float)FOC_SWEEP_WINDOW_SIZE);
}

static void sweep_ls_reset(void)
{
    memset(&s_sweep.ls, 0, sizeof(s_sweep.ls));
}

static void sweep_prepare_frequency(void)
{
    const foc_sweep_frequency_t *frequency =
        &s_frequency_table[s_sweep.frequency_index];
    const float actual_frequency_hz = sweep_actual_frequency_hz(frequency);
    uint32_t settle_ticks = (uint32_t)
        ((((float)FOC_SWEEP_SETTLE_CYCLES *
           (float)FOC_SWEEP_CONTROL_RATE_HZ) / actual_frequency_hz) + 0.5f);

    if (settle_ticks < FOC_SWEEP_MIN_SETTLE_TICKS) {
        settle_ticks = FOC_SWEEP_MIN_SETTLE_TICKS;
    }

    s_sweep.sample_index = 0U;
    s_sweep.sample_divider_counter = 0U;
    s_sweep.window_index = 0U;
    s_sweep.valid_window_count = 0U;
    s_sweep.settle_tick = 0U;
    s_sweep.settle_tick_limit = settle_ticks;
    s_sweep.response_real_sum = 0.0f;
    s_sweep.response_imag_sum = 0.0f;
    s_sweep.input_amplitude_sum = 0.0f;
    s_sweep.gain_sum = 0.0f;
    s_sweep.gain_square_sum = 0.0f;
    s_sweep.phase_unit_real_sum = 0.0f;
    s_sweep.phase_unit_imag_sum = 0.0f;
    s_sweep.phase = 0.0f;
    s_sweep.phase_increment =
        _2PI * actual_frequency_hz / (float)FOC_SWEEP_CONTROL_RATE_HZ;
    sweep_ls_reset();
    s_sweep.state = FOC_SWEEP_SETTLE;
}

static void sweep_ls_add_sample(const foc_sweep_frequency_t *frequency,
                                float iq, float position)
{
    float basis[FOC_SWEEP_LS_BASIS_COUNT];
    const float angle = _2PI * (float)frequency->fft_bin *
        (float)s_sweep.sample_index / (float)FOC_SWEEP_WINDOW_SIZE;

    if (s_sweep.ls.reference_valid == 0U) {
        s_sweep.ls.iq_reference = iq;
        s_sweep.ls.position_reference = position;
        s_sweep.ls.reference_valid = 1U;
    }

    basis[0] = cosf(angle);
    basis[1] = sinf(angle);
    basis[2] = 1.0f;
    basis[3] = (2.0f * (float)s_sweep.sample_index /
                (float)(FOC_SWEEP_WINDOW_SIZE - 1U)) - 1.0f;
    iq -= s_sweep.ls.iq_reference;
    position -= s_sweep.ls.position_reference;

    for (uint8_t row = 0U; row < FOC_SWEEP_LS_BASIS_COUNT; row++) {
        s_sweep.ls.iq_rhs[row] += basis[row] * iq;
        s_sweep.ls.position_rhs[row] += basis[row] * position;
        for (uint8_t column = 0U;
             column < FOC_SWEEP_LS_BASIS_COUNT; column++) {
            s_sweep.ls.gram[row][column] += basis[row] * basis[column];
        }
    }
}

static uint8_t sweep_ls_solve(float iq_coefficients[FOC_SWEEP_LS_BASIS_COUNT],
                              float position_coefficients[
                                  FOC_SWEEP_LS_BASIS_COUNT])
{
    float augmented[FOC_SWEEP_LS_BASIS_COUNT]
                   [FOC_SWEEP_LS_BASIS_COUNT + 2U];

    for (uint8_t row = 0U; row < FOC_SWEEP_LS_BASIS_COUNT; row++) {
        for (uint8_t column = 0U;
             column < FOC_SWEEP_LS_BASIS_COUNT; column++) {
            augmented[row][column] = s_sweep.ls.gram[row][column];
        }
        augmented[row][FOC_SWEEP_LS_BASIS_COUNT] =
            s_sweep.ls.iq_rhs[row];
        augmented[row][FOC_SWEEP_LS_BASIS_COUNT + 1U] =
            s_sweep.ls.position_rhs[row];
    }

    for (uint8_t pivot_column = 0U;
         pivot_column < FOC_SWEEP_LS_BASIS_COUNT; pivot_column++) {
        uint8_t pivot_row = pivot_column;
        float pivot_magnitude = fabsf(augmented[pivot_row][pivot_column]);

        for (uint8_t row = (uint8_t)(pivot_column + 1U);
             row < FOC_SWEEP_LS_BASIS_COUNT; row++) {
            const float magnitude = fabsf(augmented[row][pivot_column]);
            if (magnitude > pivot_magnitude) {
                pivot_magnitude = magnitude;
                pivot_row = row;
            }
        }
        if (pivot_magnitude < FOC_SWEEP_LS_PIVOT_MIN) {
            return 0U;
        }

        if (pivot_row != pivot_column) {
            for (uint8_t column = 0U;
                 column < (FOC_SWEEP_LS_BASIS_COUNT + 2U); column++) {
                const float temporary = augmented[pivot_column][column];
                augmented[pivot_column][column] = augmented[pivot_row][column];
                augmented[pivot_row][column] = temporary;
            }
        }

        const float pivot = augmented[pivot_column][pivot_column];
        for (uint8_t column = pivot_column;
             column < (FOC_SWEEP_LS_BASIS_COUNT + 2U); column++) {
            augmented[pivot_column][column] /= pivot;
        }

        for (uint8_t row = 0U; row < FOC_SWEEP_LS_BASIS_COUNT; row++) {
            if (row == pivot_column) {
                continue;
            }
            const float scale = augmented[row][pivot_column];
            for (uint8_t column = pivot_column;
                 column < (FOC_SWEEP_LS_BASIS_COUNT + 2U); column++) {
                augmented[row][column] -=
                    scale * augmented[pivot_column][column];
            }
        }
    }

    for (uint8_t row = 0U; row < FOC_SWEEP_LS_BASIS_COUNT; row++) {
        iq_coefficients[row] =
            augmented[row][FOC_SWEEP_LS_BASIS_COUNT];
        position_coefficients[row] =
            augmented[row][FOC_SWEEP_LS_BASIS_COUNT + 1U];
    }
    return 1U;
}

static uint8_t sweep_finish_window(void)
{
    float iq_coefficients[FOC_SWEEP_LS_BASIS_COUNT];
    float position_coefficients[FOC_SWEEP_LS_BASIS_COUNT];
    float input_real;
    float input_imag;
    float output_real;
    float output_imag;
    float input_power;
    float response_real;
    float response_imag;
    float response_gain;

    if (sweep_ls_solve(iq_coefficients, position_coefficients) == 0U) {
        return 0U;
    }

    input_real = iq_coefficients[0];
    input_imag = -iq_coefficients[1];
    output_real = position_coefficients[0];
    output_imag = -position_coefficients[1];
    input_power = (input_real * input_real) + (input_imag * input_imag);
    if (input_power < (FOC_SWEEP_INPUT_AMPLITUDE_MIN_A *
                       FOC_SWEEP_INPUT_AMPLITUDE_MIN_A)) {
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

    s_sweep.response_real_sum += response_real;
    s_sweep.response_imag_sum += response_imag;
    s_sweep.input_amplitude_sum += sqrtf(input_power);
    s_sweep.gain_sum += response_gain;
    s_sweep.gain_square_sum += response_gain * response_gain;
    if (response_gain > 0.0f) {
        s_sweep.phase_unit_real_sum += response_real / response_gain;
        s_sweep.phase_unit_imag_sum += response_imag / response_gain;
    }
    s_sweep.valid_window_count++;
    return 1U;
}

void foc_frequency_sweep_init(void)
{
    memset(&s_sweep, 0, sizeof(s_sweep));
    s_sweep.state = FOC_SWEEP_IDLE;
}

uint8_t foc_frequency_sweep_start(void)
{
    uint32_t primask;

    if (s_sweep.state != FOC_SWEEP_IDLE) {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    memset(&s_sweep, 0, sizeof(s_sweep));
    s_sweep.start_position = encoder_data.mechanical_position;
    s_sweep.frequency_index = 0U;
    foc_ctrl.target_d = 0.0f;
    foc_ctrl.target_q = 0.0f;
    sweep_prepare_frequency();
    if (primask == 0U) {
        __enable_irq();
    }
    return 1U;
}

void foc_frequency_sweep_stop(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    foc_ctrl.target_q = 0.0f;
    foc_ctrl.target_d = 0.0f;
    s_sweep.state = FOC_SWEEP_IDLE;
    if (primask == 0U) {
        __enable_irq();
    }
}

bool foc_frequency_sweep_is_active(void)
{
    return s_sweep.state != FOC_SWEEP_IDLE;
}

bool foc_frequency_sweep_is_returning(void)
{
    return s_sweep.state == FOC_SWEEP_RETURN_POSITION;
}

float foc_frequency_sweep_get_start_position(void)
{
    return s_sweep.start_position;
}

float foc_frequency_sweep_get_iq_amplitude_min(void)
{
    return s_frequency_table[0].iq_amplitude_a;
}

float foc_frequency_sweep_get_iq_amplitude_max(void)
{
    return s_frequency_table[FOC_SWEEP_FREQUENCY_COUNT - 1U].iq_amplitude_a;
}

const char *foc_frequency_sweep_get_state_name(void)
{
    switch (s_sweep.state) {
        case FOC_SWEEP_SETTLE:
            return "settle";
        case FOC_SWEEP_CAPTURE:
            return "capture";
        case FOC_SWEEP_RESULT_PENDING:
            return "analyze";
        case FOC_SWEEP_RETURN_POSITION:
            return "return";
        case FOC_SWEEP_COMPLETE:
            return "complete";
        case FOC_SWEEP_ABORTED:
            return "abort";
        case FOC_SWEEP_IDLE:
        default:
            return "idle";
    }
}

void foc_frequency_sweep_get_status(foc_sweep_status_t *status)
{
    uint32_t primask;

    if (status == NULL) {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    status->state = s_sweep.state;
    status->frequency_index = s_sweep.frequency_index;
    status->position_error =
        encoder_data.mechanical_position - s_sweep.start_position;
    status->speed = encoder_data.mechanical_speed;
    status->return_elapsed_ms = s_sweep.return_tick /
        (FOC_SWEEP_CONTROL_RATE_HZ / 1000U);
    if (primask == 0U) {
        __enable_irq();
    }
}

void foc_frequency_sweep_control_tick(void)
{
    const foc_sweep_frequency_t *frequency;

    if ((s_sweep.state != FOC_SWEEP_SETTLE) &&
        (s_sweep.state != FOC_SWEEP_CAPTURE)) {
        foc_ctrl.target_q = 0.0f;
        return;
    }

    frequency = &s_frequency_table[s_sweep.frequency_index];
    foc_ctrl.target_d = 0.0f;
    foc_ctrl.target_q = frequency->iq_amplitude_a * sinf(s_sweep.phase);
    s_sweep.phase += s_sweep.phase_increment;
    if (s_sweep.phase >= _2PI) {
        s_sweep.phase -= _2PI;
    }

    if (s_sweep.state == FOC_SWEEP_SETTLE) {
        if (++s_sweep.settle_tick >= s_sweep.settle_tick_limit) {
            s_sweep.sample_index = 0U;
            s_sweep.sample_divider_counter = 0U;
            s_sweep.state = FOC_SWEEP_CAPTURE;
        }
        return;
    }

    if (++s_sweep.sample_divider_counter < frequency->sample_divider) {
        return;
    }
    s_sweep.sample_divider_counter = 0U;
    sweep_ls_add_sample(frequency, foc_ctrl.abc_dq.current_q,
                        encoder_data.mechanical_position);
#if FOC_SWEEP_WAVEFORM_PRINT_ENABLE
    debug_log("%.6f,%.6f", foc_ctrl.abc_dq.current_q,
              encoder_data.mechanical_position);
#endif

    if (++s_sweep.sample_index < FOC_SWEEP_WINDOW_SIZE) {
        return;
    }

    if (sweep_finish_window() == 0U) {
        foc_ctrl.target_q = 0.0f;
        __DMB();
        s_sweep.state = FOC_SWEEP_RESULT_PENDING;
        return;
    }

    if (++s_sweep.window_index >= frequency->window_count) {
        foc_ctrl.target_q = 0.0f;
        __DMB();
        s_sweep.state = FOC_SWEEP_RESULT_PENDING;
        return;
    }

    s_sweep.sample_index = 0U;
    s_sweep.sample_divider_counter = 0U;
    sweep_ls_reset();
}

void foc_frequency_sweep_return_feedback_tick(float position, float speed)
{
    if (s_sweep.state != FOC_SWEEP_RETURN_POSITION) {
        return;
    }

    s_sweep.return_position_error = position - s_sweep.start_position;
    s_sweep.return_speed = speed;
    s_sweep.return_tick++;

    if ((fabsf(s_sweep.return_position_error) <
         FOC_SWEEP_RETURN_POSITION_ERROR_RAD) &&
        (fabsf(speed) < FOC_SWEEP_RETURN_SPEED_RAD_S)) {
        s_sweep.return_stable_tick++;
    }
    else {
        s_sweep.return_stable_tick = 0U;
    }

    if (s_sweep.return_tick >= FOC_SWEEP_RETURN_TIMEOUT_TICKS) {
        foc_ctrl.target_q = 0.0f;
        s_sweep.state = FOC_SWEEP_ABORTED;
        return;
    }

    if (s_sweep.return_stable_tick < FOC_SWEEP_RETURN_STABLE_TICKS) {
        return;
    }

    foc_ctrl.target_q = 0.0f;
    s_sweep.return_stable_tick = 0U;
    s_sweep.return_tick = 0U;
    if (++s_sweep.frequency_index >= FOC_SWEEP_FREQUENCY_COUNT) {
        s_sweep.state = FOC_SWEEP_COMPLETE;
    }
    else {
        sweep_prepare_frequency();
    }
}

void foc_frequency_sweep_background_process(void)
{
    const foc_sweep_frequency_t *frequency;
    float response_real;
    float response_imag;
    float input_amplitude;
    float output_amplitude;
    float gain;
    float phase_deg;
    float gain_std_percent;
    float phase_std_deg;

    if (s_sweep.state == FOC_SWEEP_COMPLETE) {
        s_sweep.state = FOC_SWEEP_IDLE;
        return;
    }
    if (s_sweep.state == FOC_SWEEP_ABORTED) {
        debug_log("<ERR:sweep return_timeout,index=%u,error=%.6f,speed=%.6f>",
                  (unsigned int)s_sweep.frequency_index,
                  s_sweep.return_position_error, s_sweep.return_speed);
        s_sweep.state = FOC_SWEEP_IDLE;
        return;
    }
    if (s_sweep.state != FOC_SWEEP_RESULT_PENDING) {
        return;
    }

    frequency = &s_frequency_table[s_sweep.frequency_index];
    if (s_sweep.valid_window_count != frequency->window_count) {
        debug_log("<ERR:sweep least_squares_failed,index=%u>",
                  (unsigned int)s_sweep.frequency_index);
    }
    else {
        response_real = s_sweep.response_real_sum /
            (float)s_sweep.valid_window_count;
        response_imag = s_sweep.response_imag_sum /
            (float)s_sweep.valid_window_count;
        input_amplitude = s_sweep.input_amplitude_sum /
            (float)s_sweep.valid_window_count;
        gain = sqrtf((response_real * response_real) +
                     (response_imag * response_imag));
        output_amplitude = input_amplitude * gain;
        phase_deg = atan2f(response_imag, response_real) * (180.0f / PI);
        while (phase_deg > 180.0f) {
            phase_deg -= 360.0f;
        }
        while (phase_deg < -180.0f) {
            phase_deg += 360.0f;
        }

        gain_std_percent = 0.0f;
        phase_std_deg = 0.0f;
        if (s_sweep.valid_window_count > 1U) {
            const float window_count = (float)s_sweep.valid_window_count;
            float gain_variance =
                s_sweep.gain_square_sum -
                ((s_sweep.gain_sum * s_sweep.gain_sum) / window_count);
            float phase_resultant = sqrtf(
                (s_sweep.phase_unit_real_sum *
                 s_sweep.phase_unit_real_sum) +
                (s_sweep.phase_unit_imag_sum *
                 s_sweep.phase_unit_imag_sum)) / window_count;

            if (gain_variance < 0.0f) {
                gain_variance = 0.0f;
            }
            if (s_sweep.gain_sum > 0.0f) {
                const float gain_std = sqrtf(
                    gain_variance / (window_count - 1.0f));
                gain_std_percent =
                    100.0f * gain_std / (s_sweep.gain_sum / window_count);
            }
            if (phase_resultant > 1.0f) {
                phase_resultant = 1.0f;
            }
            if (phase_resultant < 1.0e-6f) {
                phase_resultant = 1.0e-6f;
            }
            phase_std_deg = sqrtf(-2.0f * logf(phase_resultant)) *
                (180.0f / PI);
        }

        debug_log("%u,%.6f,%.6f,%.6f,%.9f,%.3f,%u,%.3f,%.3f",
                  (unsigned int)s_sweep.frequency_index,
                  sweep_actual_frequency_hz(frequency),
                  input_amplitude, output_amplitude, gain, phase_deg,
                  (unsigned int)s_sweep.valid_window_count,
                  gain_std_percent, phase_std_deg);
    }

    s_sweep.return_stable_tick = 0U;
    s_sweep.return_tick = 0U;
    s_sweep.return_position_error =
        encoder_data.mechanical_position - s_sweep.start_position;
    s_sweep.return_speed = encoder_data.mechanical_speed;
    __DMB();
    s_sweep.state = FOC_SWEEP_RETURN_POSITION;
}
