#ifndef FOC_CONTROL_H
#define FOC_CONTROL_H

#include <stdint.h>

#include "Config.h"
#include "adc.h"
#include "foc_conversion.h"
#include "foc_encoder.h"
#include "foc_encoder_calibration.h"
#include "foc_ladrc.h"
#include "foc_mechanical_param.h"
#include "foc_runtime.h"
#include "foc_setting.h"
#include "foc_trajectory_planning.h"
#include "tim.h"
#include "usart.h"

#define FOC_S_CURVE_MAX_VELOCITY     0.0f /* <= 0: no velocity ceiling */
#define FOC_S_CURVE_MAX_ACCELERATION 349.0658504f
#define FOC_S_CURVE_MAX_JERK         1745.329252f

void foc_start_init(void);
uint8_t foc_control_start(foc_run_profile_t profile);
uint8_t foc_control_start_mechanical_identification(void);
void foc_control_background_process(void);
void foc_current_in_control(void);
void foc_control(void);
void foc_debug(void);

#endif /* FOC_CONTROL_H */
