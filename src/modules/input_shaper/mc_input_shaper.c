#include "mc_input_shaper.h"

#include <math.h>
#include <stddef.h>

#include "board_config.h"

// 固定速率离散整形器。连续域脉冲时刻量化到调用周期后必须保持单调，
// 否则整形器无法用一条延迟线实现。
#define INPUT_SHAPER_MIN_SAMPLE_RATE_RATIO       20.0f
#define INPUT_SHAPER_DEFAULT_VIBRATION_TOLERANCE (1.0f / 20.0f)
#define INPUT_SHAPER_PI                          3.14159265358979323846f

static bool has_sufficient_sample_rate(float resonance_freq_hz,
                                       float sample_time_s)
{
    float sample_rate_hz = 1.0f / sample_time_s;

    return sample_rate_hz >=
        resonance_freq_hz * INPUT_SHAPER_MIN_SAMPLE_RATE_RATIO;
}

static int8_t validate_quantized_delays(const uint16_t *delay, uint8_t n)
{
    if ((delay == NULL) || (n == 0U) || (delay[0] != 0U)) {
        return -1;
    }

    for (uint8_t i = 1U; i < n; i++) {
        if (delay[i] <= delay[i - 1U]) {
            return -1;
        }
    }

    return 0;
}

// 归一化脉冲幅值之和为 1，保证整形器直流增益为 1。
static int8_t normalize_amplitudes(float *amp, uint8_t n)
{
    float sum = 0.0f;

    for (uint8_t i = 0U; i < n; i++) {
        sum += amp[i];
    }
    if (sum <= 0.0f) {
        return -1;
    }

    float inv_sum = 1.0f / sum;

    for (uint8_t i = 0U; i < n; i++) {
        amp[i] *= inv_sum;
    }

    return 0;
}

static float get_max_damping_ratio(input_shaper_type_t type)
{
    switch (type) {
        case INPUT_SHAPER_TYPE_ZV:
        case INPUT_SHAPER_TYPE_ZVD:
        case INPUT_SHAPER_TYPE_MZV: {
            return 0.99f;
        }

        case INPUT_SHAPER_TYPE_EI: {
            return 0.4f;
        }

        case INPUT_SHAPER_TYPE_2HUMP_EI: {
            return 0.3f;
        }

        case INPUT_SHAPER_TYPE_3HUMP_EI: {
            return 0.2f;
        }

        default: {
            return -1.0f;
        }
    }
}

void input_shaper_disable(input_shaper_t *obj)
{
    if (obj == NULL) {
        return;
    }

    obj->resonance_freq_hz = 0.0f;
    obj->damping_ratio = 0.0f;
    obj->vibration_tolerance = INPUT_SHAPER_DEFAULT_VIBRATION_TOLERANCE;
    obj->num_taps = 0U;
    obj->buf_idx = 0U;
    obj->buf_size = 0U;
    obj->pre_active_samples = 0U;
    obj->post_active_samples = 0U;
    obj->last_input = 0.0f;
    obj->last_input_valid = false;

    for (uint8_t i = 0U; i < INPUT_SHAPER_MAX_TAPS; i++) {
        obj->amp[i] = 0.0f;
        obj->delay[i] = 0U;
    }
}

int8_t input_shaper_init(input_shaper_t *obj, input_shaper_type_t type,
                         float resonance_freq_hz, float damping_ratio,
                         float sample_time_s)
{
    if (obj == NULL) {
        return -1;
    }

    obj->type = INPUT_SHAPER_TYPE_NONE;
    input_shaper_disable(obj);
    for (uint16_t i = 0U; i < INPUT_SHAPER_BUFFER_MAX; i++) {
        obj->buffer[i] = 0.0f;
    }

    return input_shaper_config(obj, type, resonance_freq_hz, damping_ratio,
                               sample_time_s);
}

static void calc_zv(float freq, float dr, float *amp, float *time_delay,
                    uint8_t *n)
{
    float df = sqrtf(1.0f - (dr * dr));
    float k = expf(-dr * INPUT_SHAPER_PI / df);
    float t_d = 1.0f / (freq * df);

    *n = 2U;
    amp[0] = 1.0f;
    time_delay[0] = 0.0f;
    amp[1] = k;
    time_delay[1] = 0.5f * t_d;
}

static void calc_zvd(float freq, float dr, float *amp, float *time_delay,
                     uint8_t *n)
{
    float df = sqrtf(1.0f - (dr * dr));
    float k = expf(-dr * INPUT_SHAPER_PI / df);
    float t_d = 1.0f / (freq * df);

    *n = 3U;
    amp[0] = 1.0f;
    time_delay[0] = 0.0f;
    amp[1] = 2.0f * k;
    time_delay[1] = 0.5f * t_d;
    amp[2] = k * k;
    time_delay[2] = t_d;
}

static void calc_mzv(float freq, float dr, float *amp, float *time_delay,
                     uint8_t *n)
{
    float df = sqrtf(1.0f - (dr * dr));
    float k = expf(-0.75f * dr * INPUT_SHAPER_PI / df);
    float t_d = 1.0f / (freq * df);

    *n = 3U;
    amp[0] = 0.2928932188f;
    amp[1] = 0.4142135624f * k;
    amp[2] = 0.2928932188f * k * k;
    time_delay[0] = 0.0f;
    time_delay[1] = 0.375f * t_d;
    time_delay[2] = 0.75f * t_d;
}

static void calc_ei(float freq, float dr, float v_tol, float *amp,
                    float *time_delay, uint8_t *n)
{
    float df = sqrtf(1.0f - (dr * dr));
    float t_d = 1.0f / (freq * df);

    float a1 = (0.24968f + (0.24961f * v_tol)) +
        (((0.80008f + (1.23328f * v_tol)) +
          ((0.49599f + (3.17316f * v_tol)) * dr)) *
         dr);
    float a3 = (0.25149f + (0.21474f * v_tol)) +
        (((-0.83249f + (1.41498f * v_tol)) +
          ((0.85181f - (4.90094f * v_tol)) * dr)) *
         dr);
    float a2 = 1.0f - a1 - a3;

    float t2 = 0.4999f +
        ((((0.46159f + (8.57843f * v_tol)) * v_tol) +
          ((((4.26169f - (108.644f * v_tol)) * v_tol) +
            (((1.75601f + (336.989f * v_tol)) * v_tol) * dr)) *
           dr)) *
         dr);

    *n = 3U;
    amp[0] = a1;
    time_delay[0] = 0.0f;
    amp[1] = a2;
    time_delay[1] = t2 * t_d;
    amp[2] = a3;
    time_delay[2] = t_d;
}

static void calc_expansion_coeffs(float freq, float dr,
                                  const float t_matrix[][4],
                                  const float a_matrix[][4], uint8_t num_taps,
                                  float *amp, float *time_delay, uint8_t *n)
{
    float tau = 1.0f / freq;

    *n = num_taps;
    for (uint8_t i = 0U; i < num_taps; i++) {
        float u = t_matrix[i][3];
        float v = a_matrix[i][3];

        for (int8_t j = 0; j < 3; j++) {
            u = (u * dr) + t_matrix[i][2 - j];
            v = (v * dr) + a_matrix[i][2 - j];
        }
        time_delay[i] = u * tau;
        amp[i] = v;
    }
}

static void calc_2hump_ei(float freq, float dr, float *amp, float *time_delay,
                          uint8_t *n)
{
    const float t_mat[4][4] = {{0.0f, 0.0f, 0.0f, 0.0f},
                               {0.49890f, 0.16270f, -0.54262f, 6.16180f},
                               {0.99748f, 0.18382f, -1.58270f, 8.17120f},
                               {1.49920f, -0.09297f, -0.28338f, 1.85710f}};
    const float a_mat[4][4] = {{0.16054f, 0.76699f, 2.26560f, -1.22750f},
                               {0.33911f, 0.45081f, -2.58080f, 1.73650f},
                               {0.34089f, -0.61533f, -0.68765f, 0.42261f},
                               {0.15997f, -0.60246f, 1.00280f, -0.93145f}};

    calc_expansion_coeffs(freq, dr, t_mat, a_mat, 4U, amp, time_delay, n);
}

static void calc_3hump_ei(float freq, float dr, float *amp, float *time_delay,
                          uint8_t *n)
{
    const float t_mat[5][4] = {{0.0f, 0.0f, 0.0f, 0.0f},
                               {0.49974f, 0.23834f, 0.44559f, 12.4720f},
                               {0.99849f, 0.29808f, -2.36460f, 23.3990f},
                               {1.49870f, 0.10306f, -2.01390f, 17.0320f},
                               {1.99960f, -0.28231f, 0.61536f, 5.40450f}};
    const float a_mat[5][4] = {{0.11275f, 0.76632f, 3.29160f, -1.44380f},
                               {0.23698f, 0.61164f, -2.57850f, 4.85220f},
                               {0.30008f, -0.19062f, -2.14560f, 0.13744f},
                               {0.23775f, -0.73297f, 0.46885f, -2.08650f},
                               {0.11244f, -0.45439f, 0.96382f, -1.46000f}};

    calc_expansion_coeffs(freq, dr, t_mat, a_mat, 5U, amp, time_delay, n);
}

static int8_t calc_shaper(input_shaper_type_t type, float freq, float dr,
                          float vibration_tolerance, float *amp,
                          float *time_delay, uint8_t *n)
{
    switch (type) {
        case INPUT_SHAPER_TYPE_ZV: {
            calc_zv(freq, dr, amp, time_delay, n);
            return 0;
        }

        case INPUT_SHAPER_TYPE_ZVD: {
            calc_zvd(freq, dr, amp, time_delay, n);
            return 0;
        }

        case INPUT_SHAPER_TYPE_MZV: {
            calc_mzv(freq, dr, amp, time_delay, n);
            return 0;
        }

        case INPUT_SHAPER_TYPE_EI: {
            calc_ei(freq, dr, vibration_tolerance, amp, time_delay, n);
            return 0;
        }

        case INPUT_SHAPER_TYPE_2HUMP_EI: {
            calc_2hump_ei(freq, dr, amp, time_delay, n);
            return 0;
        }

        case INPUT_SHAPER_TYPE_3HUMP_EI: {
            calc_3hump_ei(freq, dr, amp, time_delay, n);
            return 0;
        }

        default: {
            return -1;
        }
    }
}

static int8_t
input_shaper_config_internal(input_shaper_t *obj, input_shaper_type_t type,
                             float resonance_freq_hz, float damping_ratio,
                             float sample_time_s, float vibration_tolerance)
{
    if ((obj == NULL) || (sample_time_s <= 0.0f) || (damping_ratio < 0.0f)) {
        return -1;
    }

    if ((type == INPUT_SHAPER_TYPE_NONE) || (resonance_freq_hz == 0.0f)) {
        input_shaper_disable(obj);
        obj->type = type;
        obj->vibration_tolerance = vibration_tolerance;
        return 0;
    }

    float max_damping_ratio = get_max_damping_ratio(type);

    if ((resonance_freq_hz < 0.0f) || (damping_ratio > max_damping_ratio)) {
        return -1;
    }
    if (!has_sufficient_sample_rate(resonance_freq_hz, sample_time_s)) {
        return -1;
    }

    float amp_arr[INPUT_SHAPER_MAX_TAPS] = {0.0f};
    float time_delay[INPUT_SHAPER_MAX_TAPS] = {0.0f};
    float amp_local[INPUT_SHAPER_MAX_TAPS] = {0.0f};
    uint16_t delay_local[INPUT_SHAPER_MAX_TAPS] = {0U};
    uint8_t n = 0U;

    if ((calc_shaper(type, resonance_freq_hz, damping_ratio,
                     vibration_tolerance, amp_arr, time_delay, &n) != 0) ||
        (n == 0U)) {
        return -1;
    }

    if (normalize_amplitudes(amp_arr, n) != 0) {
        return -1;
    }

    uint32_t max_delay = 0U;

    for (uint8_t i = 0U; i < n; i++) {
        uint32_t quantized_delay =
            (uint32_t)((time_delay[i] / sample_time_s) + 0.5f);

        amp_local[i] = amp_arr[i];
        delay_local[i] = (uint16_t)quantized_delay;
        if (quantized_delay > max_delay) {
            max_delay = quantized_delay;
        }
    }

    if (max_delay >= INPUT_SHAPER_BUFFER_MAX) {
        return -1;
    }
    if (validate_quantized_delays(delay_local, n) != 0) {
        return -1;
    }

    // 运行时滤波器使用量化后的脉冲，扫描窗口按量化质心计算。
    float quantized_centroid = 0.0f;

    for (uint8_t i = 0U; i < n; i++) {
        quantized_centroid += amp_local[i] * (float)delay_local[i];
    }
    uint16_t pre_active_samples = (uint16_t)(quantized_centroid + 0.5f);

    if (pre_active_samples > max_delay) {
        pre_active_samples = (uint16_t)max_delay;
    }
    uint16_t post_active_samples = (uint16_t)(max_delay - pre_active_samples);

    input_shaper_disable(obj);
    obj->type = type;
    obj->resonance_freq_hz = resonance_freq_hz;
    obj->damping_ratio = damping_ratio;
    obj->vibration_tolerance = vibration_tolerance;
    obj->num_taps = n;
    obj->pre_active_samples = pre_active_samples;
    obj->post_active_samples = post_active_samples;
    obj->buf_size = (uint16_t)(max_delay + 1U);

    for (uint8_t i = 0U; i < n; i++) {
        obj->amp[i] = amp_local[i];
        obj->delay[i] = delay_local[i];
    }

    return 0;
}

int8_t input_shaper_config(input_shaper_t *obj, input_shaper_type_t type,
                           float resonance_freq_hz, float damping_ratio,
                           float sample_time_s)
{
    return input_shaper_config_internal(
        obj, type, resonance_freq_hz, damping_ratio, sample_time_s,
        INPUT_SHAPER_DEFAULT_VIBRATION_TOLERANCE);
}

void input_shaper_reset(input_shaper_t *obj, float initial_value)
{
    if (obj == NULL) {
        return;
    }

    if (obj->buf_size > 0U) {
        for (uint16_t i = 0U; i < obj->buf_size; i++) {
            obj->buffer[i] = initial_value;
        }
    }
    obj->buf_idx = 0U;
    obj->last_input = initial_value;
    obj->last_input_valid = true;
}

float input_shaper_process(input_shaper_t *obj, float input)
{
    if ((obj == NULL) || (obj->buf_size == 0U) || (obj->num_taps == 0U)) {
        return input;
    }

    obj->buffer[obj->buf_idx] = input;

    float output = 0.0f;

    for (uint8_t i = 0U; i < obj->num_taps; i++) {
        int32_t read_idx = (int32_t)obj->buf_idx - (int32_t)obj->delay[i];

        if (read_idx < 0) {
            read_idx += obj->buf_size;
        }
        output += obj->amp[i] * obj->buffer[read_idx];
    }

    obj->buf_idx++;
    if (obj->buf_idx >= obj->buf_size) {
        obj->buf_idx = 0U;
    }

    obj->last_input = input;
    obj->last_input_valid = true;
    return output;
}

bool input_shaper_is_enabled(const input_shaper_t *obj)
{
    return (obj != NULL) && (obj->num_taps > 0U) && (obj->buf_size > 0U);
}

uint16_t input_shaper_get_pre_active_samples(const input_shaper_t *obj)
{
    if (obj == NULL) {
        return 0U;
    }
    return obj->pre_active_samples;
}

uint16_t input_shaper_get_post_active_samples(const input_shaper_t *obj)
{
    if (obj == NULL) {
        return 0U;
    }
    return obj->post_active_samples;
}

uint16_t input_shaper_get_max_delay(const input_shaper_t *obj)
{
    if ((obj == NULL) || (obj->num_taps == 0U)) {
        return 0U;
    }
    return obj->delay[obj->num_taps - 1U];
}

bool input_shaper_get_last_input(const input_shaper_t *obj, float *last_input)
{
    if ((obj == NULL) || (!obj->last_input_valid) || (last_input == NULL)) {
        return false;
    }
    *last_input = obj->last_input;
    return true;
}

int8_t
input_shaper_get_step_sequence(const input_shaper_t *obj, float start_value,
                               float end_value,
                               float targets[INPUT_SHAPER_MAX_TAPS],
                               uint16_t wait_samples[INPUT_SHAPER_MAX_TAPS])
{
    if ((!input_shaper_is_enabled(obj)) || (targets == NULL) ||
        (wait_samples == NULL)) {
        return -1;
    }

    float delta = end_value - start_value;
    float cumulative_amp = 0.0f;
    uint16_t prev_delay = 0U;

    for (uint8_t i = 0U; i < obj->num_taps; i++) {
        cumulative_amp += obj->amp[i];
        targets[i] = start_value + (cumulative_amp * delta);
        wait_samples[i] = obj->delay[i] - prev_delay;
        prev_delay = obj->delay[i];
    }
    for (uint8_t i = obj->num_taps; i < INPUT_SHAPER_MAX_TAPS; i++) {
        targets[i] = end_value;
        wait_samples[i] = 0U;
    }
    return (int8_t)obj->num_taps;
}
