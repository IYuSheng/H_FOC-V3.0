#include "foc_input_shaper.h"
#include "Config.h"
#include <math.h>

void foc_zv_input_shaper_init(foc_zv_input_shaper_t *shaper,
                              float resonance_hz,
                              float damping_ratio,
                              float sample_time_s)
{
    float sqrt_term;
    float wd;
    float k;
    float delay_time_s;
    uint16_t delay_ticks;

    if (shaper == 0) {
        return;
    }

    shaper->gain_now = 1.0f;
    shaper->gain_delay = 0.0f;
    shaper->delay_ticks = 0U;
    shaper->buffer_size = 1U;
    shaper->write_index = 0U;
    shaper->initialized = 0U;

    if ((resonance_hz <= 0.0f) || (sample_time_s <= 0.0f)) {
        return;
    }

    if (damping_ratio < 0.0f) {
        damping_ratio = 0.0f;
    } else if (damping_ratio > 0.95f) {
        damping_ratio = 0.95f;
    }

    sqrt_term = sqrtf(1.0f - damping_ratio * damping_ratio);
    wd = 2.0f * PI * resonance_hz * sqrt_term;
    if (wd <= 0.0f) {
        return;
    }

    k = expf(-damping_ratio * PI / sqrt_term);
    delay_time_s = PI / wd;
    delay_ticks = (uint16_t)((delay_time_s / sample_time_s) + 0.5f);

    if (delay_ticks > FOC_INPUT_SHAPER_MAX_DELAY_TICKS) {
        delay_ticks = FOC_INPUT_SHAPER_MAX_DELAY_TICKS;
    }

    shaper->gain_now = 1.0f / (1.0f + k);
    shaper->gain_delay = k / (1.0f + k);
    shaper->delay_ticks = delay_ticks;
    shaper->buffer_size = delay_ticks + 1U;
    if (shaper->buffer_size == 0U) {
        shaper->buffer_size = 1U;
    }
}

void foc_zv_input_shaper_reset(foc_zv_input_shaper_t *shaper, float value)
{
    uint16_t i;

    if (shaper == 0) {
        return;
    }

    for (i = 0U; i < shaper->buffer_size; i++) {
        shaper->buffer[i] = value;
    }

    shaper->write_index = 0U;
    shaper->initialized = 1U;
}

float foc_zv_input_shaper_update(foc_zv_input_shaper_t *shaper, float input)
{
    uint16_t read_index;
    float delayed_input;
    float output;

    if (shaper == 0) {
        return input;
    }

    if (shaper->initialized == 0U) {
        foc_zv_input_shaper_reset(shaper, input);
    }

    shaper->buffer[shaper->write_index] = input;

    if (shaper->write_index >= shaper->delay_ticks) {
        read_index = shaper->write_index - shaper->delay_ticks;
    } else {
        read_index = (uint16_t)(shaper->buffer_size + shaper->write_index - shaper->delay_ticks);
    }

    delayed_input = shaper->buffer[read_index];
    output = shaper->gain_now * input + shaper->gain_delay * delayed_input;

    shaper->write_index++;
    if (shaper->write_index >= shaper->buffer_size) {
        shaper->write_index = 0U;
    }

    return output;
}
