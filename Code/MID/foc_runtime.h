#ifndef FOC_RUNTIME_H
#define FOC_RUNTIME_H

#include <stdint.h>

typedef enum
{
    FOC_MODE_IDLE = 0,
    FOC_MODE_RUNNING
} foc_mode_t;

typedef enum
{
    FOC_RUN_CURRENT = 0,
    FOC_RUN_POSITION_STEP
} foc_run_profile_t;

typedef struct
{
    float target_id;
    float target_iq;
    float step_position_a;
    float step_position_b;
    uint32_t step_period_ms;
} foc_runtime_config_t;

typedef struct
{
    float resistance;
    float inductance_ld;
    float inductance_lq;
    float flux_linkage;
    float ka;
    float kv;
    float tc;
} foc_motor_config_t;

typedef struct
{
    foc_motor_config_t motor;
    uint16_t version;
    uint8_t mechanical_stored;
} foc_runtime_flash_info_t;

void foc_runtime_init(void);
void foc_runtime_set_ready(void);
uint8_t foc_runtime_is_ready(void);
foc_mode_t foc_runtime_get_mode(void);
foc_run_profile_t foc_runtime_get_profile(void);
const foc_runtime_config_t *foc_runtime_get_config(void);
const foc_motor_config_t *foc_runtime_get_motor_config(void);
uint8_t foc_runtime_read_flash(foc_runtime_flash_info_t *info);
uint8_t foc_runtime_debug_is_enabled(void);
void foc_runtime_set_debug(uint8_t enabled);

uint8_t foc_runtime_set_id(float target_id);
uint8_t foc_runtime_set_iq(float target_iq);
uint8_t foc_runtime_set_step(float position_a, float position_b,
                             uint32_t period_ms);
uint8_t foc_runtime_set_motor(float resistance, float inductance_ld,
                              float inductance_lq, float flux_linkage);
uint8_t foc_runtime_set_mechanical(float ka, float kv, float tc);
uint8_t foc_runtime_start(foc_run_profile_t profile);

#endif /* FOC_RUNTIME_H */
