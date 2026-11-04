#include "usart.h"

#include "board_config.h"

/**
 * @brief 初始化 USART3 及 DMA 收发
 */
void MX_USART3_UART_Init(void)
{
    (void)drv_uart_init(CONFIG_UART_INSTANCE);
}

/**
 * @brief USART3 中断入口
 */
void USART3_IRQHandler(void)
{
    drv_uart_irq_handler();
}
