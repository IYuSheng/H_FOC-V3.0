#ifndef FOC_INPUT_SHAPER_H
#define FOC_INPUT_SHAPER_H

#include <stdint.h>

#ifndef FOC_INPUT_SHAPER_MAX_DELAY_TICKS
#define FOC_INPUT_SHAPER_MAX_DELAY_TICKS 200U
#endif

typedef struct {
    float gain_now;
    float gain_delay;
    uint16_t delay_ticks;
    uint16_t buffer_size;
    uint16_t write_index;
    uint8_t initialized;
    float buffer[FOC_INPUT_SHAPER_MAX_DELAY_TICKS + 1U];
} foc_zv_input_shaper_t;

void foc_zv_input_shaper_init(foc_zv_input_shaper_t *shaper,
                              float resonance_hz,
                              float damping_ratio,
                              float sample_time_s);

void foc_zv_input_shaper_reset(foc_zv_input_shaper_t *shaper, float value);

float foc_zv_input_shaper_update(foc_zv_input_shaper_t *shaper, float input);

#endif /* FOC_INPUT_SHAPER_H */
