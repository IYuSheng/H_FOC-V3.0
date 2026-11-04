/**
 * @file identify_resistance.h
 * @brief 基于多工作点最小二乘的定子电阻辨识。
 */

#ifndef IDENTIFY_RESISTANCE_H_
#define IDENTIFY_RESISTANCE_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IDENT_R_NUM_POINTS 6U

/**
 * @brief 电阻辨识状态
 */
typedef enum {
    IDENT_R_IDLE = 0,
    IDENT_R_INIT,
    IDENT_R_PROBE_RAMP,
    IDENT_R_PROBE_SETTLE,
    IDENT_R_PROBE_SAMPLE,
    IDENT_R_POINT_SETTLE,
    IDENT_R_POINT_SAMPLE,
    IDENT_R_PHASE_SWITCH,
    IDENT_R_COMPUTE,
    IDENT_R_SAVE,
    IDENT_R_DONE,
    IDENT_R_ERROR
} ident_r_state_t;

/**
 * @brief 电阻辨识上下文
 */
typedef struct {
    ident_r_state_t state;                     /**< 当前状态 */
    uint32_t timer_ticks;                      /**< 当前状态计时 */
    uint8_t phase_idx;                         /**< 相索引，0=A，1=B */
    uint8_t point_idx;                         /**< 当前采样点 */
    float voltage_meas[2][IDENT_R_NUM_POINTS]; /**< 各相电压样本（V） */
    float current_meas[2][IDENT_R_NUM_POINTS]; /**< 各相电流样本（A） */
    float voltage_now;                         /**< 当前开环电压（V） */
    float voltage_cap;                         /**< 当前电压上限（V） */
    float current_accum;                       /**< 电流累加值 */
    float voltage_accum;                       /**< 电压累加值 */
    uint32_t sample_count;                     /**< 累加样本数 */
    float r_estimate;                          /**< 探针估算电阻（ohm） */
    float r_phase[2];                          /**< 各相拟合电阻（ohm） */
    float v_offset[2];                         /**< 各相拟合电压偏置（V） */
    float r_final;                             /**< 最终平均电阻（ohm） */
} identify_resistance_t;

/**
 * @brief 初始化电阻辨识模块
 * @return 0 表示成功，负值表示失败
 */
int8_t identify_resistance_init(void);

/**
 * @brief 启动电阻辨识
 * @return 0 表示成功，负值表示状态不允许
 */
int8_t identify_resistance_start(void);

/**
 * @brief 在 5 kHz 控制节拍推进电阻辨识
 */
void identify_resistance_control_tick(void);

/**
 * @brief 在主循环保存电阻辨识结果
 */
void identify_resistance_background_process(void);

/**
 * @brief 停止电阻辨识
 */
void identify_resistance_stop(void);

/**
 * @brief 获取电阻辨识上下文
 * @return 电阻辨识上下文指针
 */
identify_resistance_t *identify_resistance_get(void);

#ifdef __cplusplus
}
#endif

#endif /* IDENTIFY_RESISTANCE_H_ */
