#include "mc_init.h"

#include "adc.h"
#include "cordic.h"
#include "gpio.h"
#include "identify_parameter.h"
#include "mc_calibration.h"
#include "mc_frequency_sweep.h"
#include "mc_math.h"
#include "mc_runtime.h"
#include "mc_sys.h"
#include "spi.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_utils.h"
#include "tim.h"
#include "usart.h"

void SystemClock_Config(void);

static uint8_t mc_control_started;

void mc_init(void)
{
    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();
    MX_TIM1_Init();
    MX_TIM15_Init();
    MX_ADC1_Init();
    MX_SPI1_Init();
    MX_USART3_UART_Init();
    MX_CORDIC_Init();
    mc_sine_table_init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    mc_runtime_init();
    mc_encoder_calibration_init();
    identify_parameter_init();
    mc_frequency_sweep_init();
}

void mc_start(void)
{
    if (mc_control_started != 0U) {
        return;
    }

    adc1_enable_current_sampling();
    NVIC_EnableIRQ(ADC1_2_IRQn);

    LL_TIM_ClearFlag_CC1(TIM15);
    LL_TIM_EnableIT_CC1(TIM15);
    LL_TIM_SetCounter(TIM15, 0U);
    LL_TIM_EnableCounter(TIM15);

    bsp_pwm_start();
    mc_control_started = 1U;
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
    while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_4) {
    }
    LL_PWR_EnableRange1BoostMode();
    LL_RCC_HSE_Enable();
    /* Wait till HSE is ready */
    while (LL_RCC_HSE_IsReady() != 1) {
    }

    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_3, 85,
                                LL_RCC_PLLR_DIV_2);
    LL_RCC_PLL_EnableDomain_SYS();
    LL_RCC_PLL_Enable();
    /* Wait till PLL is ready */
    while (LL_RCC_PLL_IsReady() != 1) {
    }

    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_2);
    /* Wait till System clock is ready */
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {
    }

    /* Insure 1us transition state at intermediate medium speed clock*/
    for (__IO uint32_t i = (170 >> 1); i != 0; i--)
        ;

    /* Set AHB prescaler*/
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
    LL_SetSystemCoreClock(170000000);

    /* Update the time base */
    if (HAL_InitTick(TICK_INT_PRIORITY) != HAL_OK) {
        Error_Handler();
    }
}
