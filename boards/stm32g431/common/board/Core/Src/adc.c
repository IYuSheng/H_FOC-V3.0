/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.c
  * @brief   This file provides code for the configuration
  *          of the ADC instances.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#define LOG_TAG "adc"
#include "adc.h"

/* USER CODE BEGIN 0 */
#include "board_config.h"
#include <math.h>
#include "logger.h"
#include "mc_control.h"
#include "tim.h"
#include "usart.h"

static float voltage_conv_factor = ADC_REF_VOLTAGE / ADC_MAX_VALUE * (R_Voaltage_1 + R_Voaltage_2) / R_Voaltage_2;
static float current_conv_factor = ADC_REF_VOLTAGE / ADC_MAX_VALUE / R_Current / INA240_GAIN;

#define ADC1_REG_CHANNELS      1U
#define ADC1_DMA_BUFFER_SIZE   ADC1_REG_CHANNELS
#define ADC1_DMA_CHANNEL       LL_DMA_CHANNEL_2

static uint16_t adc1_dma_buffer[ADC1_DMA_BUFFER_SIZE] = {0};
static uint16_t adc1_offset[2] = {2048U, 2048U};

mc_data_t mc_raw_data MC_CCM_BSS = {0};
mc_data_i mc_current_data MC_CCM_BSS = {0};
mc_data_v mc_voltage_data MC_CCM_BSS = {0};
/* USER CODE END 0 */

void MX_ADC1_Init(void)
{
  LL_DMA_InitTypeDef DMA_InitStruct = {0};
  LL_ADC_INJ_InitTypeDef ADC_INJ_InitStruct = {0};
  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
  LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  uint32_t wait_loop_index = 0;

  LL_RCC_SetADCClockSource(LL_RCC_ADC12_CLKSOURCE_SYSCLK);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC12);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMAMUX1);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_12;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);

  /* Regular组只保留母线电压后台采样，避免与高实时性的电流环抢时序。 */
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_CONTINUOUS;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_UNLIMITED;
  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_OVERWRITTEN;
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);

  LL_ADC_SetGainCompensation(ADC1, 0);
  LL_ADC_SetOverSamplingScope(ADC1, LL_ADC_OVS_DISABLE);

  ADC_CommonInitStruct.CommonClock = LL_ADC_CLOCK_SYNC_PCLK_DIV4;
  ADC_CommonInitStruct.Multimode = LL_ADC_MULTI_INDEPENDENT;
  LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);

  LL_ADC_DisableDeepPowerDown(ADC1);
  LL_ADC_EnableInternalRegulator(ADC1);

  wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000U * 2U))) / 10U);
  while (wait_loop_index != 0U)
  {
    wait_loop_index--;
  }

  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_11);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_11, LL_ADC_SAMPLINGTIME_92CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_11, LL_ADC_SINGLE_ENDED);

  /* 注入组专门服务电流环，两个通道分别采PA1/IN2和PA2/IN3。 */
  LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_2, LL_ADC_SINGLE_ENDED);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_2, LL_ADC_SAMPLINGTIME_24CYCLES_5);

  LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_3, LL_ADC_SINGLE_ENDED);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_3, LL_ADC_SAMPLINGTIME_24CYCLES_5);

  /* 初始化阶段先保持软件触发，等mc_start()后再切到TIM1_TRGO2硬件触发。 */
  ADC_INJ_InitStruct.TriggerSource = LL_ADC_INJ_TRIG_SOFTWARE;
  ADC_INJ_InitStruct.SequencerLength = LL_ADC_INJ_SEQ_SCAN_ENABLE_2RANKS;
  ADC_INJ_InitStruct.SequencerDiscont = LL_ADC_INJ_SEQ_DISCONT_DISABLE;
  ADC_INJ_InitStruct.TrigAuto = LL_ADC_INJ_TRIG_INDEPENDENT;
  LL_ADC_INJ_Init(ADC1, &ADC_INJ_InitStruct);
  LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_1, LL_ADC_CHANNEL_2);
  LL_ADC_INJ_SetSequencerRanks(ADC1, LL_ADC_INJ_RANK_2, LL_ADC_CHANNEL_3);

  DMA_InitStruct.PeriphRequest = LL_DMAMUX_REQ_ADC1;
  DMA_InitStruct.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
  DMA_InitStruct.Mode = LL_DMA_MODE_CIRCULAR;
  DMA_InitStruct.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
  DMA_InitStruct.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
  DMA_InitStruct.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_HALFWORD;
  DMA_InitStruct.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_HALFWORD;
  DMA_InitStruct.Priority = LL_DMA_PRIORITY_LOW;
  DMA_InitStruct.NbData = ADC1_DMA_BUFFER_SIZE;
  DMA_InitStruct.PeriphOrM2MSrcAddress = LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA);
  DMA_InitStruct.MemoryOrM2MDstAddress = (uint32_t)adc1_dma_buffer;
  LL_DMA_Init(DMA1, ADC1_DMA_CHANNEL, &DMA_InitStruct);
  /* 母线电压只做后台刷新，不需要DMA中断介入控制环。 */
  LL_DMA_DisableIT_HT(DMA1, ADC1_DMA_CHANNEL);
  LL_DMA_DisableIT_TC(DMA1, ADC1_DMA_CHANNEL);
  LL_DMA_DisableIT_TE(DMA1, ADC1_DMA_CHANNEL);

  LL_ADC_StartCalibration(ADC1, LL_ADC_SINGLE_ENDED);
  while (LL_ADC_IsCalibrationOnGoing(ADC1))
  {
  }

  LL_DMA_EnableChannel(DMA1, ADC1_DMA_CHANNEL);
  LL_ADC_Enable(ADC1);
  while (!LL_ADC_IsActiveFlag_ADRDY(ADC1))
  {
  }
  LL_ADC_ClearFlag_ADRDY(ADC1);

  NVIC_SetPriority(ADC1_2_IRQn, 1);

  LL_ADC_REG_StartConversion(ADC1);
}

void adc1_enable_current_sampling(void)
{
  /* 运行阶段把注入组切到TIM1_CH5生成的TRGO2脉冲，
   * 这样采样点固定落在PWM峰值前20个计数。 */
  LL_ADC_ClearFlag_JEOC(ADC1);
  LL_ADC_ClearFlag_JEOS(ADC1);
  LL_ADC_INJ_SetTriggerSource(ADC1, LL_ADC_INJ_TRIG_EXT_TIM1_TRGO2);
  LL_ADC_INJ_SetTriggerEdge(ADC1, LL_ADC_INJ_TRIG_EXT_RISING);
  LL_ADC_EnableIT_JEOS(ADC1);
  LL_ADC_INJ_StartConversion(ADC1);
}

void adc1_process_injected_current_sample(void)
{
  uint16_t raw_ia = LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1);
  uint16_t raw_ib = LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_2);

  /* 注入组中断到来后立即完成零偏扣除、量纲转换并进入电流环。 */
  mc_raw_data.IA = (int16_t)adc1_offset[0] - (int16_t)raw_ia;
  mc_raw_data.IB = (int16_t)adc1_offset[1] - (int16_t)raw_ib;

  mc_current_data.IA = (float)mc_raw_data.IA * current_conv_factor;
  mc_current_data.IB = (float)mc_raw_data.IB * current_conv_factor;

  mc_current_in_control();
}

void adc1_current_offset_calibrate(void)
{
  uint32_t sum0 = 0;
  uint32_t sum1 = 0;
  uint32_t valid_samples = 0;
  const uint32_t samples = 1000;
  const uint32_t timeout_ms = 2;
  uint8_t calibration_ok = 1;

  bsp_pwm_stop();
  HAL_Delay(100);

  LL_ADC_DisableIT_JEOS(ADC1);
  LL_ADC_REG_StopConversion(ADC1);
  LL_DMA_DisableChannel(DMA1, ADC1_DMA_CHANNEL);
  LL_DMA_ClearFlag_GI2(DMA1);

  /* 校准阶段强制改回软件触发，保证静态零电流采样不受PWM时序影响。 */
  LL_ADC_INJ_SetTriggerSource(ADC1, LL_ADC_INJ_TRIG_SOFTWARE);

  for (uint32_t i = 0; i < samples; i++)
  {
    uint32_t start_tick;

    LL_ADC_ClearFlag_JEOC(ADC1);
    LL_ADC_ClearFlag_JEOS(ADC1);
    LL_ADC_INJ_StartConversion(ADC1);
    start_tick = HAL_GetTick();
    while (!LL_ADC_IsActiveFlag_JEOS(ADC1) &&
           ((HAL_GetTick() - start_tick) < timeout_ms))
    {
    }
    if (!LL_ADC_IsActiveFlag_JEOS(ADC1))
    {
      calibration_ok = 0;
      break;
    }

    sum0 += LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_1);
    sum1 += LL_ADC_INJ_ReadConversionData12(ADC1, LL_ADC_INJ_RANK_2);
    valid_samples++;
  }

  if (valid_samples > 0U)
  {
    adc1_offset[0] = (uint16_t)(sum0 / valid_samples);
    adc1_offset[1] = (uint16_t)(sum1 / valid_samples);
  }
  else
  {
    calibration_ok = 0;
  }

  if (calibration_ok != 0U)
  {
    log_i("ADC1 Offset Calibrated: IA=%d, IB=%d",
          adc1_offset[0], adc1_offset[1]);
  }
  else
  {
    log_e("ADC offset calibration timeout, samples=%lu/%lu",
          (unsigned long)valid_samples, (unsigned long)samples);
  }

  LL_DMA_EnableChannel(DMA1, ADC1_DMA_CHANNEL);
  LL_ADC_REG_StartConversion(ADC1);
  adc1_enable_current_sampling();
}

float get_mc_bus_voltage(void)
{
  /* Regular DMA缓冲区只存母线电压，索引0即当前Vbus采样值。 */
  mc_voltage_data.vbus = (float)adc1_dma_buffer[0] * voltage_conv_factor;
  if (mc_voltage_data.vbus > EPS_F)
  {
    mc_voltage_data.vbus_inv = 1.0f / mc_voltage_data.vbus;
    mc_voltage_data.spwm_scale =
        (0.5f * PWM_PERIOD) * mc_voltage_data.vbus_inv;
  }
  else
  {
    mc_voltage_data.vbus_inv = 0.0f;
    mc_voltage_data.spwm_scale = 0.0f;
  }
  return mc_voltage_data.vbus;
}
