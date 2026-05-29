/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.h
  * @brief   This file contains all the function prototypes for
  *          the tim.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

void MX_TIM1_Init(void);

void bsp_pwm_start(void);
void bsp_pwm_stop(void);

/**
 * @brief 设置四相PWM占空比
 * @param duty_A
 * @param duty_A_
 * @param duty_B
 * @param duty_B_
 */
void bsp_pwm_set_duty_four_phase(uint32_t duty_A, uint32_t duty_A_, uint32_t duty_B, uint32_t duty_B_);

void MX_TIM15_Init(void);
void MX_TIM3_Init(void);
void MX_TIM4_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */

