/**
 * @file mc_filter.h
 * @brief 二阶巴特沃斯低通滤波器和陷波滤波器。
 */

#ifndef MC_FILTER_H_
#define MC_FILTER_H_

#include "mc_control.h"

/**
 * @brief 二阶巴特沃斯低通滤波器状态
 */
typedef struct {
    float b0;          /**< 分子系数 b0 */
    float b1;          /**< 分子系数 b1 */
    float b2;          /**< 分子系数 b2 */
    float a1;          /**< 分母系数 a1 */
    float a2;          /**< 分母系数 a2 */
    float x[2];        /**< 输入历史 x[n-1]、x[n-2] */
    float y[2];        /**< 输出历史 y[n-1]、y[n-2] */
    float cutoff_freq; /**< 截止频率（Hz） */
    float sample_freq; /**< 采样频率（Hz） */
} ButterworthLPF_t;

/**
 * @brief 陷波滤波器状态
 */
typedef struct {
    float fc;     /**< 中心频率（Hz） */
    float bw;     /**< 带宽（Hz） */
    float b[3];   /**< 分子系数 */
    float a[3];   /**< 分母系数，a[0] 归一化为 1 */
    float x[3];   /**< 输入历史 */
    float y[3];   /**< 输出历史 */
    uint8_t init; /**< 初始化标志 */
} NotchFilter_t;

/**
 * @brief 初始化二阶巴特沃斯低通滤波器
 * @param filt 滤波器状态
 * @param cutoff_freq 截止频率（Hz）
 * @param sample_freq 采样频率（Hz）
 */
void butterworth_init(ButterworthLPF_t *filt, float cutoff_freq,
                      float sample_freq);

/**
 * @brief 更新二阶巴特沃斯低通滤波器
 * @param filt 滤波器状态
 * @param input 当前输入
 * @return 滤波后的输出
 */
float butterworth_filter(ButterworthLPF_t *filt, float input);

/**
 * @brief 初始化陷波滤波器
 * @param nf 滤波器状态
 * @param fc 中心频率（Hz）
 * @param bw 带宽（Hz）
 * @param sample_time_ms 采样周期（ms）
 */
void notch_filter_init(NotchFilter_t *nf, float fc, float bw,
                       float sample_time_ms);

/**
 * @brief 处理一个陷波滤波采样点
 * @param nf 滤波器状态
 * @param input 当前输入
 * @return 滤波后的输出
 */
float notch_filter_process(NotchFilter_t *nf, float input);

/**
 * @brief 复位陷波滤波器历史状态
 * @param nf 滤波器状态
 */
void notch_filter_reset(NotchFilter_t *nf);

#endif /* MC_FILTER_H_ */
