/**
 * @file usart.h
 * @brief 调试串口驱动声明。
 */

#ifndef MC_USART_H_
#define MC_USART_H_

#include "drv_uart.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化调试串口
 */
void MX_USART3_UART_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* MC_USART_H_ */
