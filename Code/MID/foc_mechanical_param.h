#ifndef FOC_MECHANICAL_PARAM_H
#define FOC_MECHANICAL_PARAM_H

#include <stdbool.h>
#include <stdint.h>

#define MECH_IDENTIFY_WINDOW_SIZE 1024U

typedef enum
{
    MECH_IDENTIFY_IDLE = 0U,
    MECH_IDENTIFY_SETTLE,
    MECH_IDENTIFY_CAPTURE,
    MECH_IDENTIFY_RESULT_PENDING,
    MECH_IDENTIFY_RETURN_POSITION,
    MECH_IDENTIFY_FIT_PENDING,
    MECH_IDENTIFY_COMPLETE,
    MECH_IDENTIFY_FAILED
} mech_identify_state_t;

typedef struct
{
    float Ka;
    float Kv;
    float Tc;
    float Ka_high_band;
    float weighted_rms_percent;
    float low_band_rms_percent;
    float high_band_rms_percent;
    float median_gain_std_percent;
    float median_phase_std_deg;
    uint16_t point_count;
    uint8_t error_code;
    bool valid;
} mech_identify_result_t;

void foc_mechanical_param_init(void);
void foc_mechanical_param_start(void);
void foc_mechanical_param_stop(void);
bool foc_mechanical_param_is_active(void);
bool foc_mechanical_param_is_returning(void);
float foc_mechanical_param_get_start_position(void);
mech_identify_state_t foc_mechanical_param_get_state(void);
const mech_identify_result_t *foc_mechanical_param_get_result(void);

void foc_mechanical_param_process(void);
void foc_mechanical_param_return_feedback_tick(float position, float speed);
void foc_mechanical_param_background_process(void);

#endif /* FOC_MECHANICAL_PARAM_H */
