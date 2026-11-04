/**
 * @file identify_inductance.h
 * @brief 基于正弦电压注入和锁相检测的相电感辨识。
 */

#ifndef IDENTIFY_INDUCTANCE_H_
#define IDENTIFY_INDUCTANCE_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 电感辨识状态
 */
typedef enum {
    IDENT_L_IDLE = 0,
    IDENT_L_INIT,
    IDENT_L_ALIGN,
    IDENT_L_DISCHARGE,
    IDENT_L_PROBE,
    IDENT_L_INJECT,
    IDENT_L_FIT,
    IDENT_L_PHASE_SWITCH,
    IDENT_L_COMPUTE,
    IDENT_L_SAVE,
    IDENT_L_DONE,
    IDENT_L_ERROR
} ident_l_state_t;

/**
 * @brief 电感辨识上下文
 */
typedef struct {
    ident_l_state_t state; /**< 当前状态 */
    uint32_t timer_ticks;  /**< 当前状态计时 */
    uint8_t phase_idx;     /**< 相索引，0=A，1=B */
    float r_known;         /**< 已知相电阻（ohm） */
    float v_m;             /**< 注入电压幅值（V） */
    float v_dc_bias;       /**< 转子锁定直流偏置（V） */
    float freq_hz;         /**< 注入频率（Hz） */
    float omega;           /**< 注入角频率（rad/s） */
    float zoh_half;        /**< 零阶保持半采样相位（rad） */
    float accum_i_sin;     /**< 与正弦参考的锁相累加值 */
    float accum_i_cos;     /**< 与余弦参考的锁相累加值 */
    uint32_t accum_cnt;    /**< 锁相累加样本数 */
    float l_phase[2];      /**< 各相电感（H） */
    float l_final;         /**< 最终平均电感（H） */
} identify_inductance_t;

/**
 * @brief 初始化电感辨识模块
 * @return 0 表示成功，负值表示失败
 */
int8_t identify_inductance_init(void);

/**
 * @brief 启动电感辨识
 * @return 0 表示成功，负值表示状态不允许
 */
int8_t identify_inductance_start(void);

/**
 * @brief 在 5 kHz 控制节拍推进电感辨识
 */
void identify_inductance_control_tick(void);

/**
 * @brief 在主循环保存电感辨识结果
 */
void identify_inductance_background_process(void);

/**
 * @brief 停止电感辨识
 */
void identify_inductance_stop(void);

/**
 * @brief 获取电感辨识上下文
 * @return 电感辨识上下文指针
 */
identify_inductance_t *identify_inductance_get(void);

#ifdef __cplusplus
}
#endif

#endif /* IDENTIFY_INDUCTANCE_H_ */
