/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "usart.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "foc_prase.h"

#define UART_TX_BUFFER_SIZE      512U
#define UART_RX_LINE_SIZE        96U
#define UART_RX_DMA_BUFFER_SIZE  128U
#define UART_RX_RING_SIZE        256U
#define UART_RX_RING_MASK        (UART_RX_RING_SIZE - 1U)
#define UART_RX_DMA_CHANNEL      LL_DMA_CHANNEL_1

static char d_buffer[UART_TX_BUFFER_SIZE];
static uint8_t rx_dma_buffer[UART_RX_DMA_BUFFER_SIZE];
static uint8_t rx_ring_buffer[UART_RX_RING_SIZE];
static char rx_line_buffer[UART_RX_LINE_SIZE];
static volatile uint16_t rx_ring_head = 0U;
static volatile uint16_t rx_ring_tail = 0U;
static volatile uint8_t rx_error_pending = 0U;
static uint8_t rx_line_index = 0U;
static uint8_t rx_line_discarding = 0U;

_Static_assert((UART_RX_RING_SIZE & (UART_RX_RING_SIZE - 1U)) == 0U,
               "UART RX ring size must be a power of two");

static void bsp_uart_rx_dma_restart(void)
{
  LL_DMA_DisableChannel(DMA1, UART_RX_DMA_CHANNEL);
  while (LL_DMA_IsEnabledChannel(DMA1, UART_RX_DMA_CHANNEL))
    {
    }

  LL_DMA_ClearFlag_GI1(DMA1);
  LL_DMA_SetMemoryAddress(DMA1, UART_RX_DMA_CHANNEL,
                          (uint32_t)rx_dma_buffer);
  LL_DMA_SetDataLength(DMA1, UART_RX_DMA_CHANNEL,
                       UART_RX_DMA_BUFFER_SIZE);
  LL_DMA_EnableChannel(DMA1, UART_RX_DMA_CHANNEL);
}

static void bsp_uart_rx_ring_write(const uint8_t *data, uint16_t length)
{
  for (uint16_t i = 0U; i < length; i++)
    {
      uint16_t next = (uint16_t)((rx_ring_head + 1U) & UART_RX_RING_MASK);

      if (next == rx_ring_tail)
        {
          rx_error_pending = 1U;
          return;
        }

      rx_ring_buffer[rx_ring_head] = data[i];
      __DMB();
      rx_ring_head = next;
    }
}

static uint8_t bsp_uart_rx_ring_read(uint8_t *data)
{
  if ((data == NULL) || (rx_ring_tail == rx_ring_head))
    {
      return 0U;
    }

  *data = rx_ring_buffer[rx_ring_tail];
  rx_ring_tail = (uint16_t)((rx_ring_tail + 1U) & UART_RX_RING_MASK);
  return 1U;
}

void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_DMA_InitTypeDef DMA_InitStruct = {0};

  LL_RCC_SetUSARTClockSource(LL_RCC_USART3_CLKSOURCE_PCLK1);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMAMUX1);
  /**USART3 GPIO Configuration
  PB10   ------> USART3_TX
  PB11   ------> USART3_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  DMA_InitStruct.PeriphRequest = LL_DMAMUX_REQ_USART3_RX;
  DMA_InitStruct.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
  DMA_InitStruct.Mode = LL_DMA_MODE_NORMAL;
  DMA_InitStruct.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
  DMA_InitStruct.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
  DMA_InitStruct.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
  DMA_InitStruct.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;
  DMA_InitStruct.Priority = LL_DMA_PRIORITY_HIGH;
  DMA_InitStruct.NbData = UART_RX_DMA_BUFFER_SIZE;
  DMA_InitStruct.PeriphOrM2MSrcAddress =
      LL_USART_DMA_GetRegAddr(USART3, LL_USART_DMA_REG_DATA_RECEIVE);
  DMA_InitStruct.MemoryOrM2MDstAddress = (uint32_t)rx_dma_buffer;
  LL_DMA_Init(DMA1, UART_RX_DMA_CHANNEL, &DMA_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
  USART_InitStruct.BaudRate = 921600;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART3, &USART_InitStruct);
  LL_USART_SetTXFIFOThreshold(USART3, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_SetRXFIFOThreshold(USART3, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_DisableFIFO(USART3);
  LL_USART_ConfigAsyncMode(USART3);

  /* USER CODE BEGIN WKUPType USART3 */

  /* USER CODE END WKUPType USART3 */

  LL_USART_Enable(USART3);

  /* Polling USART3 initialisation */
  while(!(LL_USART_IsActiveFlag_TEACK(USART3)))
  {
  }
  while(!(LL_USART_IsActiveFlag_REACK(USART3)))
  {
  }

  // 设置中断优先级
  NVIC_SetPriority(USART3_IRQn, 4); // 使用较低的优先级值
  NVIC_EnableIRQ(USART3_IRQn);

  // DMA接收一批字符，串口空闲中断负责转存到软件环形缓冲。
  LL_USART_ClearFlag_IDLE(USART3);
  bsp_uart_rx_dma_restart();
  LL_USART_EnableDMAReq_RX(USART3);
  LL_USART_EnableIT_IDLE(USART3);
  LL_USART_EnableIT_ERROR(USART3);
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
 * @brief UART3发送一个字节（阻塞方式）
 * @param data 要发送的数据
 */
static void bsp_uart_send_byte(uint8_t data)
{
  // 带超时的等待（避免硬件异常导致永久阻塞）
  uint32_t timeout = 0xFFFF;
  while ((!(LL_USART_IsActiveFlag_TXE(USART3))) && (timeout-- > 0));
  if (timeout > 0)
    {
      LL_USART_TransmitData8(USART3, data);
    }
}

/**
 * @brief UART3发送字符串（阻塞方式）
 * @param str 要发送的字符串
 */
void bsp_uart_send_string(char *str)
{
  while (*str)
    {
      bsp_uart_send_byte(*str++);
    }
}

/**
 * @brief UART3发送缓冲区数据（阻塞方式）
 * @param buffer 数据缓冲区
 * @param len 数据长度
 */
static inline void bsp_uart_send_buffer(uint8_t *buffer, uint16_t len)
{
  uint16_t i;
  for (i = 0; i < len; i++)
    {
      bsp_uart_send_byte(buffer[i]);
    }
}

/**
 * @brief 格式化打印调试信息
 * @param format 格式化字符串
 * @param ... 可变参数
 */
void debug_log(const char *format, ...)
{
  va_list args;
  int len;

  // 格式化字符串
  va_start(args, format);
  len = vsnprintf(d_buffer, sizeof(d_buffer), format, args);
  va_end(args);

  // 确保不会超出缓冲区范围
  if (len > sizeof(d_buffer) - 3)
    {
      len = sizeof(d_buffer) - 3;
    }

  // 添加\r\n确保换行
  if (len > 0)
    {
      d_buffer[len++] = '\r';
      d_buffer[len++] = '\n';
      d_buffer[len] = '\0';
    }

  if (len > 0)
    {
      bsp_uart_send_buffer((uint8_t*)d_buffer, (uint16_t)len);
    }
}

/**
 * @brief 等待串口最后一个字节发送完成
 */
void bsp_uart_wait_tx_complete(void)
{
  uint32_t timeout = 0xFFFFFU;

  while ((!LL_USART_IsActiveFlag_TC(USART3)) && (timeout-- > 0U))
    {
    }
}

/**
 * @brief 在主循环解析中断收集到的完整命令行
 */
void bsp_uart_process_rx(void)
{
  uint8_t received_char;

  if (rx_error_pending != 0U)
    {
      uint32_t primask = __get_PRIMASK();

      __disable_irq();
      rx_ring_tail = rx_ring_head;
      rx_error_pending = 0U;
      if (primask == 0U)
        {
          __enable_irq();
        }

      rx_line_index = 0U;
      rx_line_discarding = 0U;
      debug_log("<ERR:UART RX overflow>");
    }

  while (bsp_uart_rx_ring_read(&received_char) != 0U)
    {
      if ((received_char == '\n') || (received_char == '\r'))
        {
          if (rx_line_discarding != 0U)
            {
              rx_line_discarding = 0U;
              rx_line_index = 0U;
              debug_log("<ERR:UART command too long>");
            }
          else if (rx_line_index > 0U)
            {
              rx_line_buffer[rx_line_index] = '\0';
              parse_received_data(rx_line_buffer, rx_line_index);
              rx_line_index = 0U;
            }
          continue;
        }

      if (rx_line_discarding != 0U)
        {
          continue;
        }
      if (rx_line_index < (UART_RX_LINE_SIZE - 1U))
        {
          rx_line_buffer[rx_line_index++] = (char)received_char;
        }
      else
        {
          rx_line_index = 0U;
          rx_line_discarding = 1U;
        }
    }
}

/**
 * @brief 串口3中断处理函数
 */
void USART3_IRQHandler(void)
{
    if (LL_USART_IsActiveFlag_PE(USART3) ||
        LL_USART_IsActiveFlag_FE(USART3) ||
        LL_USART_IsActiveFlag_NE(USART3) ||
        LL_USART_IsActiveFlag_ORE(USART3))
    {
        LL_USART_ClearFlag_PE(USART3);
        LL_USART_ClearFlag_FE(USART3);
        LL_USART_ClearFlag_NE(USART3);
        LL_USART_ClearFlag_ORE(USART3);
        rx_error_pending = 1U;
        bsp_uart_rx_dma_restart();
        return;
    }

    if (LL_USART_IsActiveFlag_IDLE(USART3))
    {
        uint16_t received;

        LL_USART_ClearFlag_IDLE(USART3);
        LL_DMA_DisableChannel(DMA1, UART_RX_DMA_CHANNEL);
        while (LL_DMA_IsEnabledChannel(DMA1, UART_RX_DMA_CHANNEL))
        {
        }

        received = (uint16_t)(UART_RX_DMA_BUFFER_SIZE -
            LL_DMA_GetDataLength(DMA1, UART_RX_DMA_CHANNEL));
        if (received > 0U)
        {
            bsp_uart_rx_ring_write(rx_dma_buffer, received);
        }
        bsp_uart_rx_dma_restart();
    }
}
