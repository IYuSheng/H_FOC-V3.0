/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    foc_encoder.c
  * @brief   MT6816 编码器驱动源文件（SPI1 4线接口）
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

#include "foc_encoder.h"
#include "foc_encoder_calibration.h"

encoder_data_t encoder_data;

static inline void delay_500ns(void) {
    // 基于170MHz CPU：1个__NOP() ≈ 5.88ns，500ns需约85个__NOP()
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();
}

/* ================================= 静态函数实现 ================================= */

/**
  * @brief  SPI1发送并接收16位数据（阻塞模式，全双工）
  * @note   MT6816 4线SPI模式：MOSI发命令的同时MISO接收角度数据
  * @param  tx_data: 待发送的16位数据
  * @retval uint16_t: 同时接收到的16位数据
  */
static inline uint16_t spi_transfer16(uint16_t tx_data) {
    uint16_t rx_data = 0;

    while (!LL_SPI_IsActiveFlag_TXE(SPI1));
    LL_SPI_TransmitData16(SPI1, tx_data);

    while (!LL_SPI_IsActiveFlag_RXNE(SPI1));
    rx_data = LL_SPI_ReceiveData16(SPI1);

    while (LL_SPI_IsActiveFlag_BSY(SPI1));

    return rx_data;
}

/**
  * @brief  读取编码器指定寄存器值（MT6816 4线SPI协议）
  * @note   帧格式: 1bit R/W + 7bit地址 + 8bit数据
  *         发送 [0x80|addr][0x00]，接收 [dummy][data]
  *         一次16bit SPI传输完成一个寄存器读，单CS周期
  * @param  addr: 7位寄存器地址
  * @retval uint8_t: 寄存器数据（D[7:0]）
  */
static inline uint8_t encoder_read_reg(uint8_t addr) {
    uint16_t cmd = ENCODER_READ_BIT | ((uint16_t)(addr & ENCODER_ADDR_MASK) << ENCODER_ADDR_SHIFT);
    ENCODER_CS_LOW();
    uint16_t rx = spi_transfer16(cmd);
    ENCODER_CS_HIGH();
    delay_500ns();
    return (uint8_t)(rx & 0xFF);
}

static inline uint8_t encoder_angle_frame_parity_ok(uint16_t frame)
{
    uint8_t ones = 0;
    for (uint8_t i = 0; i < 16; i++) {
        if (frame & (1U << i)) {
            ones++;
        }
    }
    return ((ones & 0x01U) == 0U);
}

/**
  * @brief  向编码器指定寄存器写入值（MT6816 4线SPI协议）
  * @note   帧格式: Bit15=0(写) + 7bit地址 + 8bit数据
  *         一次16bit SPI传输完成，单CS周期
  * @param  addr: 7位寄存器地址
  * @param  data: 8位数据
  * @retval encoder_status_t: 写入状态
  */
static inline encoder_status_t encoder_write_reg(uint8_t addr, uint8_t data) {
    uint16_t cmd = ((uint16_t)(addr & ENCODER_ADDR_MASK) << ENCODER_ADDR_SHIFT) | data;
    ENCODER_CS_LOW();
    spi_transfer16(cmd);
    ENCODER_CS_HIGH();
    delay_500ns();

    uint8_t readback = encoder_read_reg(addr);
    if (readback == data) {
        return ENCODER_STATUS_OK;
    } else {
        debug_log("[ERROR] expect:0x%02X, Actual:0x%02X\r\n", data, readback);
        return ENCODER_STATUS_ERR_COMM;
    }
}

/**
  * @brief  编码器初始化
  * @note   通过读取角度寄存器检查MT6816通信是否正常
  * @param  无
  * @retval encoder_status_t: 初始化状态
  */
encoder_status_t encoder_init(void)
{
    // 连续读取多次角度值验证通信稳定性
    uint16_t ang1 = encoder_read_raw_angle();
    if (ang1 == 0xFFFF)
    {
        debug_log("MT6816 init Error");
        return ENCODER_STATUS_ERR_COMM;
    }

    uint16_t ang2 = encoder_read_raw_angle();
    if (ang2 == 0xFFFF)
    {
        return ENCODER_STATUS_ERR_COMM;
    }

    debug_log("MT6816 init OK, angle: %d\r\n", ang2);

    encoder_status_t enc_status = ENCODER_STATUS_OK;
    return enc_status;
}

/**
  * @brief  读取编码器原始角度值（14位）
  * @note   读取14位原始角度值，范围0~16383（对应一机械角周期）
  * @param  无
  * @retval uint16_t: 原始角度值（0~16383），失败返回0xFFFF
  */
uint16_t encoder_read_raw_angle(void)
{
    uint8_t reg03 = encoder_read_reg(ENCODER_REG_ANGLE_H);  // 角度[13:6]
    uint8_t reg04 = encoder_read_reg(ENCODER_REG_ANGLE_L);  // 角度[5:0] + Bit1弱磁告警 + Bit0奇偶校验
    uint16_t angle_frame = ((uint16_t)reg03 << 8) | reg04;

    if (angle_frame == 0xFFFFU) {
        encoder_data.error_flag = 1U;
        return 0xFFFFU;
    }

    if (!encoder_angle_frame_parity_ok(angle_frame)) {
        encoder_data.error_flag = 1U;
        return 0xFFFFU;
    }

    // // 弱磁判断
    // if ((reg04 & 0x02U) != 0U) {
    //     encoder_data.error_flag = 1U;
    //     return 0xFFFFU;
    // }

    encoder_data.error_flag = 0U;
    uint16_t raw_angle = ((uint16_t)reg03 << 6) | (reg04 >> 2);

    return raw_angle;
}

pos_obs_t g_pos_obs = {0};

// 观测器带宽
#define POS_OBS_BW_HZ  (60.0f)
#define POS_OBS_BW     (2.0f * _PI * POS_OBS_BW_HZ)

/**
 * @brief 二阶龙伯格观测器
 * @note  双重极点配置，仅用位置测量估计平滑位置和转速
 * @param theta_meas 实测位置
 * @param dt 采样周期(s)
 */
static inline void pos_obs_update(float theta_meas, float dt)
{
    if (dt <= 0.0f) return;

    if (!g_pos_obs.inited) {
        g_pos_obs.theta_hat = theta_meas;
        g_pos_obs.omega_hat = 0.0f;
        g_pos_obs.inited = 1;
        return;
    }

    const float w = POS_OBS_BW;
    const float Kp = 2.0f * w;
    const float Kv = w * w;

    /* 状态预测（恒速模型） */
    const float theta_pred = g_pos_obs.theta_hat + g_pos_obs.omega_hat * dt;
    const float omega_pred = g_pos_obs.omega_hat;

    /* position residual */
    float e = theta_meas - theta_pred;
    while (e > _PI) {
        e -= _2PI;
    }
    while (e < -_PI) {
        e += _2PI;
    }

    /* 状态校正 */
    g_pos_obs.theta_hat = angle_normalize(theta_pred + Kp * dt * e);
    g_pos_obs.omega_hat = omega_pred + Kv * dt * e;
}

/**
  * @brief  更新编码器单圈机械角度和多圈机械位置
  * @note   多圈位置格式为 turns * 2*pi + current_angle，单位 rad
  * @retval float: 单圈机械角度 [0, 2*pi)，单位 rad
  */
float encoder_read_mechanical_angle(void)
{
    static float last_raw = 0.0f;
    static uint8_t first_run = 1U;
    float current_angle = encoder_data.angle_raw_scaled;

    if (first_run != 0U) {
        encoder_data.mechanical_angle = current_angle;
        encoder_data.mechanical_position =
            ((float)encoder_data.mechanical_turns * _2PI) + current_angle;
        last_raw = current_angle;
        first_run = 0U;

        pos_obs_update(encoder_data.mechanical_angle, PWM_PERIOD_S);
        encoder_data.mechanical_speed = g_pos_obs.omega_hat;
        encoder_data.electrical_speed =
            encoder_data.mechanical_speed * MOTOR_POLE_PAIRS;
        return encoder_data.mechanical_angle;
    }

    float delta = current_angle - last_raw;

    if (delta > _PI) {
        delta -= _2PI;
        encoder_data.mechanical_turns--;
    } else if (delta < -_PI) {
        delta += _2PI;
        encoder_data.mechanical_turns++;
    }

    encoder_data.mechanical_angle = current_angle;
    encoder_data.mechanical_position =
        ((float)encoder_data.mechanical_turns * _2PI) + current_angle;
    last_raw = current_angle;

    pos_obs_update(encoder_data.mechanical_angle, PWM_PERIOD_S);

    encoder_data.mechanical_speed = g_pos_obs.omega_hat;
    encoder_data.electrical_speed =
        encoder_data.mechanical_speed * MOTOR_POLE_PAIRS;
    return encoder_data.mechanical_angle;
}

/**
  * @brief  读取编码器电角度
  * @note   将机械角度乘以极对数得到电角度，用于FOC控制中的磁场定向
  * @param  无
  * @retval float: 电角度值 (rad)，失败返回-1.0f
  */
float encoder_read_electrical_angle(void)
{
#if FOC_ENCODER_CALIBRATION_USE_CONST_TABLE
    float calibrated_electrical_angle;
    float calibrated_mechanical_angle;
#endif

    encoder_data.raw_angle = encoder_read_raw_angle();
    if (encoder_data.raw_angle == 0xFFFF) {
        return -1.0f;
    }

#if FOC_ENCODER_CALIBRATION_USE_CONST_TABLE
    if (foc_encoder_calibration_linearize(encoder_data.raw_angle,
                                          &calibrated_electrical_angle,
                                          &calibrated_mechanical_angle) != 0U)
    {
        encoder_data.angle_raw_scaled = calibrated_mechanical_angle;
        encoder_data.electrical_angle =
            angle_normalize(calibrated_electrical_angle + _PI_2);
        return encoder_data.electrical_angle;
    }
#endif

    encoder_data.angle_raw_scaled = (encoder_data.raw_angle * ENCODER_ANGLE_SCALE);
    encoder_data.electrical_angle =
        angle_normalize(encoder_data.angle_raw_scaled * MOTOR_POLE_PAIRS);
    return encoder_data.electrical_angle;
}

/**
  * @brief  清除编码器错误标志
  * @note   读取清除错误标志寄存器即可清除错误状态
  * @param  无
  * @retval 无
  */
void encoder_clear_error(void) {
    encoder_read_reg(ENCODER_REG_CLEAR_ERR);
}
