/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    foc_encoder.h
  * @brief   MT6816 编码器驱动头文件（SPI1 4线接口）
  *          适用于 FOC 电机控制，支持角度读取、零点校准、错误处理
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

#ifndef FOC_ENCODER_H
#define FOC_ENCODER_H

#include "spi.h"
#include <math.h>
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_bus.h"
#include "foc_conversion.h"

/* ================================= 硬件配置宏 ================================= */
// MT6816 CS 引脚配置
#define ENCODER_CS_PORT        GPIOA
#define ENCODER_CS_PIN         LL_GPIO_PIN_4
#define ENCODER_CS_LOW()       LL_GPIO_ResetOutputPin(ENCODER_CS_PORT, ENCODER_CS_PIN)
#define ENCODER_CS_HIGH()      LL_GPIO_SetOutputPin(ENCODER_CS_PORT, ENCODER_CS_PIN)

// MT6816 4线SPI 命令格式
// 帧结构: Bit15=R/W(1读0写), Bit[14:8]=地址A[6:0], Bit[7:0]=数据D[7:0]
// 写命令: ((addr & 0x7F) << 8) | data
// 读命令: 0x8000 | ((addr & 0x7F) << 8)
#define ENCODER_READ_BIT         0x8000          // Bit15=1 读标志
#define ENCODER_ADDR_MASK        0x7F            // 7位地址掩码
#define ENCODER_ADDR_SHIFT       8               // 地址左移量

// MT6816 寄存器地址（4线SPI模式，7位地址）
#define ENCODER_REG_ID           0x00            // ID / 芯片识别
#define ENCODER_REG_STATUS       0x01            // 状态/诊断寄存器
#define ENCODER_REG_ANGLE_H      0x03            // 角度高8位 [13:6]
#define ENCODER_REG_ANGLE_L      0x04            // 角度低字节 [5:0] + Bit1弱磁告警 + Bit0奇偶校验
#define ENCODER_REG_CLEAR_ERR    0x05            // 清除错误标志 / 超速状态

// 编码器参数
#define ENCODER_RESOLUTION      14              // 14位分辨率
#define ENCODER_MAX_VALUE       ((1 << ENCODER_RESOLUTION) - 1)  // 最大原始值：16383
#define ENCODER_ANGLE_SCALE     (_2PI / (float)(1 << ENCODER_RESOLUTION))
#define ENCODER_ELECTRICAL_SCALE (ENCODER_ANGLE_SCALE * MOTOR_POLE_PAIRS)
// Current magnet produces two encoder angle periods per mechanical revolution.
#define ENCODER_SENSOR_PERIODS_PER_REV 2U
#define ENCODER_PERIOD_COUNTS          ((1U << ENCODER_RESOLUTION) / ENCODER_SENSOR_PERIODS_PER_REV)
#define ENCODER_PERIOD_ANGLE_SCALE     (_2PI / (float)ENCODER_PERIOD_COUNTS)
#define ENCODER_EFFECTIVE_POLE_PAIRS   (MOTOR_POLE_PAIRS / ENCODER_SENSOR_PERIODS_PER_REV)
#define SPEED_CALC_INTERVAL     0.001f
#define SPEED_FILTER_K          0.2f

/* Encoder status. */
typedef enum {
    ENCODER_STATUS_OK = 0,        // 正常
    ENCODER_STATUS_ERR_COMM,      // 通信错误
    ENCODER_STATUS_ERR_CORDIC,    // CORDIC 溢出错误
    ENCODER_STATUS_ERR_MAGNET     // 磁场强度异常（过强/过弱）
} encoder_status_t;

// 编码器数据结构体
typedef struct {
    uint16_t raw_angle;                 // 原始角度值（0~16383）
    float    angle_raw_scaled;          // 单圈机械角度 [0, 2*pi), rad
    float    mechanical_angle;          // 单圈机械角度 [0, 2*pi), rad
    float    electrical_angle;          // 电角度 [0, 2*pi), rad
    float    electrical_speed;          // 电角速度 (rad/s)
    int32_t  mechanical_turns;          // accumulated mechanical turns
    float    mechanical_position;       // multi-turn position, rad
    float    mechanical_speed;          // 机械角速度 (rad/s)
    float    mechanical_accsd;          // 机械角加速度 (rad/s^2)
    uint8_t  error_flag;                // 错误标志（0=无错误，1=有错误）
    uint8_t  agc_value;                 // AGC 值（反映磁场强度：0=强，255=弱）
} encoder_data_t;

#if 0
typedef struct {
    float theta_hat;       // estimated position, rad
    float omega_hat;       // estimated velocity, rad/s
    uint8_t inited;
} pos_obs_t;
#endif

typedef struct {
    float theta_est;       // PLL estimated single-turn angle, rad
    float omega_integral;  // PLL integral frequency state, rad/s
    float omega_est;       // PLL loop-filter speed output, rad/s
    float bandwidth_hz;    // Current adaptive bandwidth, Hz
    uint8_t inited;
} encoder_pll_t;

extern encoder_data_t encoder_data;
#if 0
extern pos_obs_t g_pos_obs;
#endif
extern encoder_pll_t g_encoder_pll;

/* ================================= 函数声明 ================================= */
/**
  * @brief  编码器初始化
  * @retval encoder_status_t 初始化状态（ENCODER_STATUS_OK 为成功）
  */
encoder_status_t encoder_init(void);

/**
  * @brief  读取编码器原始角度值（14位，未缩放）
  * @note   约 2.5us
  * @retval uint16_t 原始角度值（0~16383），读取失败返回 0xFFFF
  */
uint16_t encoder_read_raw_angle(void);

/**
  * @brief  读取编码器单圈机械角度 [0, 2*pi)，已含零点校准
  * @note   约 2.8us
  * @retval float 机械角度值 (rad)，读取失败返回 -1.0f
  */
float encoder_read_mechanical_angle(void);

/**
  * @brief  读取编码器电角度
  * @note   将机械角度乘以极对数得到电角度，用于FOC控制中的磁场定向
  * @retval float 电角度值 (rad)，失败返回 -1.0f
  */
float encoder_read_electrical_angle(void);

/**
  * @brief  设置编码器零点
  */
encoder_status_t encoder_set_zero_temp(void);

/**
  * @brief  清除编码器错误标志
  */
void encoder_clear_error(void);

#endif  // FOC_ENCODER_H
