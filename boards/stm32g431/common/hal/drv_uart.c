#include "drv_uart.h"

#include <limits.h>
#include <stddef.h>

#include "board_config.h"
#include "lwrb.h"
#include "stm32g4xx.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_usart.h"

typedef struct drv_uart {
    uint8_t instance;
    USART_TypeDef *uart;
    DMA_TypeDef *dma;
    uint32_t dma_rx_channel;
    uint32_t dma_tx_channel;

    uint32_t baud_rate;
    uint32_t word_length;
    uint32_t stop_bits;
    uint32_t parity;

    lwrb_t rx_rb;
    uint8_t rx_buf[CONFIG_UART_RX_BUFFSIZE];
    uint8_t rx_dma_buf[CONFIG_UART_RX_DMA_BUFFSIZE];

    volatile bool tx_dma_busy;
    bool is_initialized;
} uart_dev_t;

static uart_dev_t _uart_dev[] = {
    {
        .instance = CONFIG_UART_INSTANCE,
        .uart = USART3,
        .dma = DMA1,
        .dma_rx_channel = LL_DMA_CHANNEL_1,
        .dma_tx_channel = LL_DMA_CHANNEL_3,
        .baud_rate = CONFIG_UART_BAUDRATE,
        .word_length = LL_USART_DATAWIDTH_8B,
        .stop_bits = LL_USART_STOPBITS_1,
        .parity = LL_USART_PARITY_NONE,
        .tx_dma_busy = false,
        .is_initialized = false,
    },
};

#define UART_DEV_SIZE (sizeof(_uart_dev) / sizeof(_uart_dev[0]))

/**
 * @brief 按实例编号查找 UART 设备
 * @param instance UART 实例编号
 * @return UART 设备指针，未找到时返回 NULL
 */
static inline uart_dev_t *uart_dev_get_by_instance(uint8_t instance)
{
    for (uint8_t i = 0U; i < UART_DEV_SIZE; i++) {
        if (_uart_dev[i].instance == instance) {
            return &_uart_dev[i];
        }
    }
    return NULL;
}

/**
 * @brief 按外设地址查找 UART 设备
 * @param uart UART 外设指针
 * @return UART 设备指针，未找到时返回 NULL
 */
static inline uart_dev_t *uart_dev_get_by_uart(USART_TypeDef *uart)
{
    for (uint8_t i = 0U; i < UART_DEV_SIZE; i++) {
        if (_uart_dev[i].uart == uart) {
            return &_uart_dev[i];
        }
    }
    return NULL;
}

/**
 * @brief 获取 UART 外设时钟频率
 * @param uart UART 外设指针
 * @return UART 外设时钟频率（Hz）
 */
static inline uint32_t uart_dev_get_clk_freq(USART_TypeDef *uart)
{
    switch ((uint32_t)uart) {
        case (uint32_t)USART1:
            return LL_RCC_GetUSARTClockFreq(LL_RCC_USART1_CLKSOURCE);
        case (uint32_t)USART2:
            return LL_RCC_GetUSARTClockFreq(LL_RCC_USART2_CLKSOURCE);
        case (uint32_t)USART3:
            return LL_RCC_GetUSARTClockFreq(LL_RCC_USART3_CLKSOURCE);
        default:
            return 0U;
    }
}

/**
 * @brief 重新启动接收 DMA
 * @param pdev UART 设备指针
 */
static void drv_uart_rx_restart(uart_dev_t *pdev)
{
    LL_DMA_DisableChannel(pdev->dma, pdev->dma_rx_channel);
    while (LL_DMA_IsEnabledChannel(pdev->dma, pdev->dma_rx_channel)) {
    }
    LL_DMA_ClearFlag_GI1(pdev->dma);
    LL_DMA_ConfigAddresses(
        pdev->dma, pdev->dma_rx_channel, (uint32_t)&pdev->uart->RDR,
        (uint32_t)pdev->rx_dma_buf, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetDataLength(pdev->dma, pdev->dma_rx_channel,
                         sizeof(pdev->rx_dma_buf));
    LL_DMA_EnableChannel(pdev->dma, pdev->dma_rx_channel);
}

/**
 * @brief 初始化 USART3、GPIO 和 DMA 硬件
 * @return 0 表示成功，负值表示失败
 */
static int8_t uart_dev_hardware_init(void)
{
    LL_USART_InitTypeDef usart_init = {0};
    LL_GPIO_InitTypeDef gpio_init = {0};
    LL_DMA_InitTypeDef dma_init = {0};

    LL_RCC_SetUSARTClockSource(LL_RCC_USART3_CLKSOURCE_PCLK1);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMAMUX1);

    gpio_init.Pin = LL_GPIO_PIN_10 | LL_GPIO_PIN_11;
    gpio_init.Mode = LL_GPIO_MODE_ALTERNATE;
    gpio_init.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    gpio_init.Pull = LL_GPIO_PULL_UP;
    gpio_init.Alternate = LL_GPIO_AF_7;
    LL_GPIO_Init(GPIOB, &gpio_init);

    dma_init.PeriphRequest = LL_DMAMUX_REQ_USART3_RX;
    dma_init.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
    dma_init.Mode = LL_DMA_MODE_NORMAL;
    dma_init.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
    dma_init.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
    dma_init.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
    dma_init.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;
    dma_init.Priority = LL_DMA_PRIORITY_LOW;
    dma_init.NbData = CONFIG_UART_RX_DMA_BUFFSIZE;
    dma_init.PeriphOrM2MSrcAddress =
        LL_USART_DMA_GetRegAddr(USART3, LL_USART_DMA_REG_DATA_RECEIVE);
    dma_init.MemoryOrM2MDstAddress = (uint32_t)_uart_dev[0].rx_dma_buf;
    LL_DMA_Init(DMA1, LL_DMA_CHANNEL_1, &dma_init);

    dma_init.PeriphRequest = LL_DMAMUX_REQ_USART3_TX;
    dma_init.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
    dma_init.PeriphOrM2MSrcAddress =
        LL_USART_DMA_GetRegAddr(USART3, LL_USART_DMA_REG_DATA_TRANSMIT);
    dma_init.MemoryOrM2MDstAddress = 0U;
    LL_DMA_Init(DMA1, LL_DMA_CHANNEL_3, &dma_init);

    usart_init.PrescalerValue = LL_USART_PRESCALER_DIV1;
    usart_init.BaudRate = CONFIG_UART_BAUDRATE;
    usart_init.DataWidth = LL_USART_DATAWIDTH_8B;
    usart_init.StopBits = LL_USART_STOPBITS_1;
    usart_init.Parity = LL_USART_PARITY_NONE;
    usart_init.TransferDirection = LL_USART_DIRECTION_TX_RX;
    usart_init.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    usart_init.OverSampling = LL_USART_OVERSAMPLING_16;
    LL_USART_Init(USART3, &usart_init);
    LL_USART_SetTXFIFOThreshold(USART3, LL_USART_FIFOTHRESHOLD_1_8);
    LL_USART_SetRXFIFOThreshold(USART3, LL_USART_FIFOTHRESHOLD_1_8);
    LL_USART_DisableFIFO(USART3);
    LL_USART_ConfigAsyncMode(USART3);

    NVIC_SetPriority(USART3_IRQn, 4U);
    NVIC_EnableIRQ(USART3_IRQn);
    return 0;
}

int8_t drv_uart_init(uint8_t instance)
{
    uart_dev_t *pdev = uart_dev_get_by_instance(instance);

    if (pdev == NULL) {
        return -1;
    }
    if (pdev->is_initialized) {
        return 0;
    }

    LL_USART_Disable(pdev->uart);
    if (uart_dev_hardware_init() != 0) {
        return -1;
    }

    LL_USART_SetBaudRate(pdev->uart,
                         uart_dev_get_clk_freq(pdev->uart),
                         LL_USART_PRESCALER_DIV1,
                         LL_USART_OVERSAMPLING_16,
                         pdev->baud_rate);
    LL_USART_SetDataWidth(pdev->uart, pdev->word_length);
    LL_USART_SetStopBitsLength(pdev->uart, pdev->stop_bits);
    LL_USART_SetParity(pdev->uart, pdev->parity);
    LL_USART_Enable(pdev->uart);
    while (!LL_USART_IsActiveFlag_TEACK(pdev->uart)) {
    }
    while (!LL_USART_IsActiveFlag_REACK(pdev->uart)) {
    }

    LL_USART_SetTXFIFOThreshold(pdev->uart, LL_USART_FIFOTHRESHOLD_1_8);
    LL_USART_SetRXFIFOThreshold(pdev->uart, LL_USART_FIFOTHRESHOLD_1_8);
    LL_USART_DisableFIFO(pdev->uart);

    (void)lwrb_init(&pdev->rx_rb, pdev->rx_buf, sizeof(pdev->rx_buf));

    drv_uart_rx_restart(pdev);
    LL_USART_ClearFlag_IDLE(pdev->uart);
    LL_USART_EnableDMAReq_RX(pdev->uart);
    LL_USART_EnableIT_IDLE(pdev->uart);
    LL_USART_EnableIT_ERROR(pdev->uart);

    LL_DMA_DisableChannel(pdev->dma, pdev->dma_tx_channel);
    LL_DMA_ClearFlag_GI3(pdev->dma);
    LL_USART_DisableDMAReq_TX(pdev->uart);
    pdev->tx_dma_busy = false;
    pdev->is_initialized = true;
    return 0;
}

bool drv_uart_is_tx_dma_ready(uint8_t instance)
{
    uart_dev_t *pdev = uart_dev_get_by_instance(instance);

    if ((pdev == NULL) || !pdev->is_initialized) {
        return false;
    }

    if (LL_DMA_IsActiveFlag_TE3(pdev->dma)) {
        LL_DMA_DisableChannel(pdev->dma, pdev->dma_tx_channel);
        LL_DMA_ClearFlag_GI3(pdev->dma);
        LL_USART_DisableDMAReq_TX(pdev->uart);
        pdev->tx_dma_busy = false;
    }
    else if (LL_DMA_IsActiveFlag_TC3(pdev->dma)) {
        LL_DMA_ClearFlag_TC3(pdev->dma);
        LL_DMA_DisableChannel(pdev->dma, pdev->dma_tx_channel);
        LL_USART_DisableDMAReq_TX(pdev->uart);
        pdev->tx_dma_busy = false;
    }

    return !pdev->tx_dma_busy;
}

size_t drv_uart_write_dma(uint8_t instance, const void *data, size_t size)
{
    uart_dev_t *pdev = uart_dev_get_by_instance(instance);

    if ((pdev == NULL) || !pdev->is_initialized || (data == NULL) ||
        (size == 0U) || (size > UINT16_MAX) ||
        !drv_uart_is_tx_dma_ready(instance)) {
        return 0U;
    }

    LL_DMA_ConfigAddresses(pdev->dma, pdev->dma_tx_channel, (uint32_t)data,
                           (uint32_t)&pdev->uart->TDR,
                           LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    LL_DMA_SetDataLength(pdev->dma, pdev->dma_tx_channel, size);
    LL_DMA_ClearFlag_GI3(pdev->dma);
    LL_DMA_EnableChannel(pdev->dma, pdev->dma_tx_channel);
    LL_USART_EnableDMAReq_TX(pdev->uart);

    pdev->tx_dma_busy = true;
    return size;
}

size_t drv_uart_write(uint8_t instance, const void *data, size_t size)
{
    uart_dev_t *pdev = uart_dev_get_by_instance(instance);
    const uint8_t *src = (const uint8_t *)data;

    if ((pdev == NULL) || !pdev->is_initialized || (data == NULL) ||
        (size == 0U)) {
        return 0U;
    }

    while (!drv_uart_is_tx_dma_ready(instance)) {
    }

    for (size_t i = 0U; i < size; i++) {
        while (!LL_USART_IsActiveFlag_TXE(pdev->uart)) {
        }
        LL_USART_TransmitData8(pdev->uart, src[i]);
    }
    while (!LL_USART_IsActiveFlag_TC(pdev->uart)) {
    }

    return size;
}

size_t drv_uart_read(uint8_t instance, void *dst, size_t size)
{
    uart_dev_t *pdev = uart_dev_get_by_instance(instance);

    if ((pdev == NULL) || !pdev->is_initialized || (dst == NULL) ||
        (size == 0U) || (lwrb_get_full(&pdev->rx_rb) == 0U)) {
        return 0U;
    }

    return (size_t)lwrb_read(&pdev->rx_rb, dst, size);
}

void drv_uart_irq_handler(void)
{
    uart_dev_t *pdev = uart_dev_get_by_uart(USART3);

    if (pdev == NULL) {
        return;
    }

    if (LL_USART_IsActiveFlag_PE(pdev->uart) ||
        LL_USART_IsActiveFlag_FE(pdev->uart) ||
        LL_USART_IsActiveFlag_NE(pdev->uart) ||
        LL_USART_IsActiveFlag_ORE(pdev->uart)) {
        LL_USART_ClearFlag_PE(pdev->uart);
        LL_USART_ClearFlag_FE(pdev->uart);
        LL_USART_ClearFlag_NE(pdev->uart);
        LL_USART_ClearFlag_ORE(pdev->uart);
        lwrb_reset(&pdev->rx_rb);
        drv_uart_rx_restart(pdev);
        return;
    }

    if (LL_USART_IsActiveFlag_IDLE(pdev->uart)) {
        uint16_t count;

        LL_USART_ClearFlag_IDLE(pdev->uart);
        count = (uint16_t)(sizeof(pdev->rx_dma_buf) -
                           LL_DMA_GetDataLength(pdev->dma,
                                                pdev->dma_rx_channel));
        if (count > 0U) {
            (void)lwrb_write(&pdev->rx_rb, pdev->rx_dma_buf, count);
        }

        drv_uart_rx_restart(pdev);
    }
}
