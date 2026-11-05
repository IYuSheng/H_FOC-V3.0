/**
 * @file vofa_plot.h
 * @brief VOFA+ JustFloat 波形输出接口。
 */

#ifndef VOFA_PLOT_H_
#define VOFA_PLOT_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief JustFloat 一帧最多通道数 */
#define VOFA_PLOT_CHANNELS 8U

/**
 * @brief 使能/关闭 VOFA 输出
 * @param enabled true 表示输出，false 表示停止
 */
void vofa_plot_set_enabled(bool enabled);

/**
 * @brief 查询 VOFA 输出是否使能
 * @return true 表示使能
 */
bool vofa_plot_is_enabled(void);

/**
 * @brief 发送一帧 JustFloat 数据
 * @param values 浮点数组
 * @param count 通道数，范围 1~VOFA_PLOT_CHANNELS
 * @note 通过 UART DMA 非阻塞发送；DMA 忙时丢弃本帧。
 *       只能在静态/全局帧缓冲区上操作，调用者数组会立即拷贝。
 */
void vofa_plot_send(const float *values, uint8_t count);

#ifdef __cplusplus
}
#endif

#endif /* VOFA_PLOT_H_ */
