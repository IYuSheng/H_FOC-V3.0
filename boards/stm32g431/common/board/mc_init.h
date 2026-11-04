/**
 * @file mc_init.h
 * @brief STM32G431 外设、时钟和 FOC 子系统初始化接口。
 */

#ifndef MC_INIT_H_
#define MC_INIT_H_

#include <stdint.h>

#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_adc.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_cordic.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_crs.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_exti.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_spi.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_usart.h"
#include "stm32g4xx_ll_utils.h"

/**
 * @brief 初始化系统时钟、外设和 FOC 软件模块
 */
void mc_init(void);

/**
 * @brief 启动 ADC 电流采样、编码器采样和 PWM 定时器，功率级保持关闭
 */
void mc_start(void);

#endif /* MC_INIT_H_ */
