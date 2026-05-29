/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
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
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

// FOC电流采样结构体(原始数据)
typedef struct
{
  int16_t IA;
  int16_t IB;
  int16_t ia;    // A相电流
  int16_t ib;    // B相电流
  int16_t ic;    // C相电流
  int16_t va;    // A相电压
  int16_t vb;    // B相电压
  int16_t vc;    // C相电压
  int16_t vbus;  // 母线电压
} foc_data_t;

// FOC电流转换结构体
typedef struct
{
  float IA;
  float IB;
  float ia;    // A相电流
  float ib;    // B相电流
  float ic;    // C相电流
} foc_data_i;

// FOC转换结构体
typedef struct
{
  float va;    // A相电压
  float vb;    // B相电压
  float vc;    // C相电压
  float vbus;  // 母线电压
  float temp;  // 温度值
} foc_data_v;

extern foc_data_t foc_raw_data;
extern foc_data_v foc_voltage_data;
extern foc_data_i foc_current_data;

void MX_ADC1_Init(void);

/**
 * @brief ADC校准电流偏置
 */
void adc1_current_offset_calibrate(void);

void adc1_enable_current_sampling(void);
void adc1_process_injected_current_sample(void);

/**
 * @brief 获取母线电压
 */
float get_foc_bus_voltage(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

