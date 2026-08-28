#ifndef FOC_FREQUENCY_SWEEP_H
#define FOC_FREQUENCY_SWEEP_H

#include <stdbool.h>
#include <stdint.h>

#define FOC_SWEEP_WINDOW_SIZE 1024U

typedef enum
{
    FOC_SWEEP_IDLE = 0U,
    FOC_SWEEP_SETTLE,
    FOC_SWEEP_CAPTURE,
    FOC_SWEEP_RESULT_PENDING,
    FOC_SWEEP_RETURN_POSITION,
    FOC_SWEEP_COMPLETE,
    FOC_SWEEP_ABORTED
} foc_sweep_state_t;

typedef struct
{
    foc_sweep_state_t state;
    uint16_t frequency_index;
    float position_error;
    float speed;
    uint32_t return_elapsed_ms;
} foc_sweep_status_t;

void foc_frequency_sweep_init(void);
uint8_t foc_frequency_sweep_start(void);
void foc_frequency_sweep_stop(void);
bool foc_frequency_sweep_is_active(void);
bool foc_frequency_sweep_is_returning(void);
float foc_frequency_sweep_get_start_position(void);
float foc_frequency_sweep_get_iq_amplitude_min(void);
float foc_frequency_sweep_get_iq_amplitude_max(void);
const char *foc_frequency_sweep_get_state_name(void);
void foc_frequency_sweep_get_status(foc_sweep_status_t *status);

void foc_frequency_sweep_control_tick(void);
void foc_frequency_sweep_return_feedback_tick(float position, float speed);
void foc_frequency_sweep_background_process(void);

#endif /* FOC_FREQUENCY_SWEEP_H */
