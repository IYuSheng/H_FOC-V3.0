/**
 * @file mc_input_shaper.h
 * @brief 振动测试和 EI 输入整形器。
 */

#ifndef MC_INPUT_SHAPER_H_
#define MC_INPUT_SHAPER_H_

#include <math.h>
#include <stdint.h>

/**
 * @brief 振动测试状态
 */
typedef struct {
    uint8_t state;       /**< 0 空闲，1 触发，2 记录，3 完成，4 打印 */
    float start_pos;     /**< 起始位置（rad） */
    float target_step;   /**< 目标阶跃（rad） */
    uint32_t sample_cnt; /**< 采样计数 */
    float pos_data[500]; /**< 位置记录 */
    float iq_data[500];  /**< q 轴电流记录 */
    uint32_t print_idx;  /**< 打印索引 */
} resonance_test_t;

/**
 * @brief EI 输入整形器状态
 */
typedef struct {
    float A[3];         /**< 三个脉冲幅值 */
    uint32_t delay[3];  /**< 三个脉冲延迟（采样点数） */
    float buffer[1000]; /**< 延迟线 */
    uint32_t buf_idx;   /**< 延迟线写索引 */
    uint32_t buf_size;  /**< 延迟线长度 */
    uint8_t init;       /**< 初始化标志 */
} EI_Shaper_t;

/**
 * @brief 全局 EI 整形器
 */
extern EI_Shaper_t g_ei_shaper;

/**
 * @brief 全局振动测试状态
 */
extern resonance_test_t g_res_test;

/**
 * @brief 初始化 EI 输入整形器
 * @param freq_hz 目标振动频率（Hz）
 * @param V 残余振动允许比例
 * @param sample_time_s 采样周期（s）
 */
void ei_shaper_init(float freq_hz, float V, float sample_time_s);

/**
 * @brief 复位 EI 整形器
 * @param value 延迟线填充值
 */
void ei_shaper_reset(float value);

/**
 * @brief 处理一个 EI 整形输入
 * @param input 输入指令
 * @return 整形后的输出
 */
float ei_shaper_process(float input);

#endif /* MC_INPUT_SHAPER_H_ */
