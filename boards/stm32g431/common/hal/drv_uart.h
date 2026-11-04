/**
 * @file drv_uart.h
 * @brief 与参考库 develop 分支一致的 UART 收发驱动接口。
 */

#ifndef DRIVERS_DRV_UART_H_
#define DRIVERS_DRV_UART_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化指定 UART 实例
 * @param instance UART 实例编号
 * @return 0 表示成功，负值表示失败
 */
int8_t drv_uart_init(uint8_t instance);

/**
 * @brief 查询 UART 发送 DMA 是否空闲
 * @param instance UART 实例编号
 * @return true 表示可以启动新一帧 DMA 发送
 */
bool drv_uart_is_tx_dma_ready(uint8_t instance);

/**
 * @brief 使用 DMA 发送一帧数据
 * @param instance UART 实例编号
 * @param data 数据地址，发送完成前必须保持有效
 * @param size 数据长度
 * @return 已加入 DMA 发送的数据长度
 * @warning data 必须指向静态、全局或生命周期覆盖整个 DMA 发送过程的内存。
 */
size_t drv_uart_write_dma(uint8_t instance, const void *data, size_t size);

/**
 * @brief 阻塞发送 UART 数据
 * @param instance UART 实例编号
 * @param data 数据地址
 * @param size 数据长度
 * @return 实际发送的数据长度
 */
size_t drv_uart_write(uint8_t instance, const void *data, size_t size);

/**
 * @brief 从 UART 接收环形缓冲读取数据
 * @param instance UART 实例编号
 * @param dst 接收缓冲区
 * @param size 最大读取长度
 * @return 实际读取的数据长度
 */
size_t drv_uart_read(uint8_t instance, void *dst, size_t size);

/**
 * @brief UART IDLE 中断处理入口
 */
void drv_uart_irq_handler(void);

#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_DRV_UART_H_ */
