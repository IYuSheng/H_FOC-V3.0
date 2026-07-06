#ifndef __FOC_CONTROL_H
#define __FOC_CONTROL_H

#include "foc_setting.h"
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "Config.h"
#include "foc_conversion.h"
#include "foc_encoder.h"
#include "foc_prase.h"
#include "foc_vibration_sweep.h"
#include "foc_Parameteridentifikation.h"
#include "foc_cogging_compensation.h"
#include "foc_trajectory_planning.h"
#include "foc_sensorless.h"
#include "foc_filter.h"
#include "foc_ladrc.h"
#include "foc_encoder_calibration.h"
#include "foc_trapezoid_planner.h"
#include <math.h>

#define FOC_SPEED_CONTROL_ENABLE            1
#define FOC_SPEED_ESO_PI_ENABLE             1
#define FOC_SPEED_LADRC_ENABLE              0
#define FOC_POSITION_CONTROL_ENABLE         0
#define FOC_POSITION_LADRC_ENABLE           0
#define FOC_PARAMETER_IDENTIFICATION_ENABLE 0
#define FOC_COGGING_COMPENSATION_ENABLE     0
#define FLUX_OBSERVER_ENABLE                0
#define FOC_TEST_ENABLE                     0
#ifndef FOC_ENCODER_CALIBRATION_ENABLE
#define FOC_ENCODER_CALIBRATION_ENABLE      1
#endif
#define FOC_RESONANCE_ENABLE                0
#define FOC_SHAPER_ENABLE                   0

#define FOC_TRAJECTORY_PLANNER_NONE         0
#define FOC_TRAJECTORY_PLANNER_S_CURVE      1
#define FOC_TRAJECTORY_PLANNER_TRAPEZOID    2
#define FOC_TRAJECTORY_PLANNER_TYPE         FOC_TRAJECTORY_PLANNER_S_CURVE

#define FOC_S_CURVE_MAX_VELOCITY            30000.0f
#define FOC_S_CURVE_MAX_ACCELERATION        500000.0f
#define FOC_S_CURVE_MAX_JERK                5000000.0f

#define FOC_TRAPEZOID_MAX_VELOCITY          30000.0f
#define FOC_TRAPEZOID_COMMAND_MAX_VELOCITY  30000.0f
#define FOC_TRAPEZOID_ACCELERATION          500000.0f
#define FOC_TRAPEZOID_TOLERANCE             0.001f

#define FOC_EI_SHAPER_ENABLE                0
#define FOC_EI_SHAPER_FREQ_HZ               11.5f
#define FOC_EI_SHAPER_V                     0.05f

#define FOC_DEBUG_POSITION_TOGGLE_ENABLE    0
#define FOC_DEBUG_POSITION_TOGGLE_TICKS     1000U
#define FOC_DEBUG_POSITION_TARGET_A         50.0f
#define FOC_DEBUG_POSITION_TARGET_B         (-50.0f)

/**
 * @brief FOC初始化，锁定电机至零位并校零编码器
 */
void foc_start_init(void);

/**
 * @brief FOC内环电流环外部调用接口
 */
void foc_current_in_control(void);

/**
 * @brief FOC控制主函数
 */
void foc_control(void);

/**
 * @brief FOC打印调试信息
 */
void foc_debug(void);

#endif /* __FOC_CONTROL_H */
