/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    mc_encoder.c
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

#define LOG_TAG "encoder"
#include "mc_encoder.h"

#include "data_types.h"
#include "mc_calibration.h"
#include "logger.h"

#define ENCODER_READ_RETRY_COUNT  3U
#define ENCODER_FAULT_ERROR_LIMIT 10U

encoder_data_t encoder_data MC_CCM_BSS;

static inline void encoder_cs_setup_delay(void)
{
    // MT6816 TL >= 100 ns，保留约 118 ns 余量
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
}

static inline void encoder_cs_hold_delay(void)
{
    // MT6816 TH >= 0.5 TSCK，当前 SPI 周期下至少 32 ns
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
}

/* ================================= 静态函数实现
 * ================================= */

/**
 * @brief  SPI1发送并接收16位数据（阻塞模式，全双工）
 * @note   MT6816 4线SPI模式：MOSI发命令的同时MISO接收角度数据
 * @param  tx_data: 待发送的16位数据
 * @retval uint16_t: 同时接收到的16位数据
 */
static inline uint16_t spi_transfer16(uint16_t tx_data)
{
    uint16_t rx_data = 0;

    while (!LL_SPI_IsActiveFlag_TXE(SPI1))
        ;
    LL_SPI_TransmitData16(SPI1, tx_data);

    while (!LL_SPI_IsActiveFlag_RXNE(SPI1))
        ;
    rx_data = LL_SPI_ReceiveData16(SPI1);

    while (LL_SPI_IsActiveFlag_BSY(SPI1))
        ;

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
static inline uint8_t encoder_read_reg(uint8_t addr)
{
    uint16_t cmd = ENCODER_READ_BIT |
        ((uint16_t)(addr & ENCODER_ADDR_MASK) << ENCODER_ADDR_SHIFT);
    uint16_t rx;

    ENCODER_CS_LOW();
    encoder_cs_setup_delay();
    rx = spi_transfer16(cmd);
    encoder_cs_hold_delay();
    ENCODER_CS_HIGH();
    return (uint8_t)(rx & 0xFF);
}

static inline uint8_t encoder_angle_frame_parity_ok(uint16_t frame)
{
    uint16_t parity = (uint16_t)(frame ^ (frame >> 8U));

    parity ^= parity >> 4U;
    parity ^= parity >> 2U;
    parity ^= parity >> 1U;
    return ((parity & 0x01U) == 0U);
}

/**
 * @brief 记录编码器读取错误并更新故障状态
 */
static void encoder_record_error(void)
{
    encoder_data.raw_valid = 0U;
    encoder_data.error_flag = 1U;

    if (encoder_data.error_count < UINT32_MAX) {
        encoder_data.error_count++;
    }
    if (encoder_data.consecutive_error_count < UINT32_MAX) {
        encoder_data.consecutive_error_count++;
    }

    if ((encoder_data.fault_latched == 0U) &&
        (encoder_data.consecutive_error_count >= ENCODER_FAULT_ERROR_LIMIT)) {
        encoder_data.fault_latched = 1U;
        motor_ctrl.fault_type = FAULT_ENCODER;
    }
}

/**
 * @brief 记录编码器读取成功并清除连续错误状态
 */
static void encoder_record_success(void)
{
    encoder_data.error_flag = 0U;
    encoder_data.consecutive_error_count = 0U;
}

/**
 * @brief  向编码器指定寄存器写入值（MT6816 4线SPI协议）
 * @note   帧格式: Bit15=0(写) + 7bit地址 + 8bit数据
 *         一次16bit SPI传输完成，单CS周期
 * @param  addr: 7位寄存器地址
 * @param  data: 8位数据
 * @retval encoder_status_t: 写入状态
 */
static inline encoder_status_t encoder_write_reg(uint8_t addr, uint8_t data)
{
    uint16_t cmd =
        ((uint16_t)(addr & ENCODER_ADDR_MASK) << ENCODER_ADDR_SHIFT) | data;

    ENCODER_CS_LOW();
    encoder_cs_setup_delay();
    spi_transfer16(cmd);
    encoder_cs_hold_delay();
    ENCODER_CS_HIGH();

    uint8_t readback = encoder_read_reg(addr);
    if (readback == data) {
        return ENCODER_STATUS_OK;
    }
    else {
        log_e("expect:0x%02X, actual:0x%02X", data, readback);
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
    if (ang1 == 0xFFFF) {
        log_e("MT6816 init error");
        return ENCODER_STATUS_ERR_COMM;
    }

    uint16_t ang2 = encoder_read_raw_angle();
    if (ang2 == 0xFFFF) {
        return ENCODER_STATUS_ERR_COMM;
    }

    log_i("MT6816 init OK, angle: %d", ang2);

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
    uint8_t reg03;
    uint8_t reg04;
    uint16_t angle_frame;

    for (uint8_t retry = 0U; retry < ENCODER_READ_RETRY_COUNT; retry++) {
        reg03 = encoder_read_reg(ENCODER_REG_ANGLE_H); // 角度[13:6]
        reg04 = encoder_read_reg(
            ENCODER_REG_ANGLE_L); // 角度[5:0] + 告警 + 奇偶校验
        angle_frame = ((uint16_t)reg03 << 8U) | reg04;

        if ((angle_frame != 0xFFFFU) &&
            (encoder_angle_frame_parity_ok(angle_frame) != 0U)) {
            encoder_record_success();
            return (uint16_t)(((uint16_t)reg03 << 6U) | (reg04 >> 2U));
        }
    }

    encoder_record_error();
    return 0xFFFFU;
}

#if 0
pos_obs_t g_pos_obs = {0};

// 观测器带宽
#define POS_OBS_BW_HZ (60.0f)
#define POS_OBS_BW    (2.0f * _PI * POS_OBS_BW_HZ)

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
#endif

#define ENCODER_PLL_BW_MIN_HZ        30.0f
#define ENCODER_PLL_BW_MAX_HZ        180.0f
#define ENCODER_PLL_FULL_SPEED_RAD_S 100.0f
#define ENCODER_PLL_DAMPING          1.0f
#define ENCODER_PLL_BW_ADAPT_TAU_S   0.005f

encoder_pll_t g_encoder_pll MC_CCM_BSS = {0};

/**
 * @brief 自适应带宽编码器PLL，输出机械速度
 * @param theta_meas 编码器单圈机械角度，rad
 * @param dt 更新周期，s
 */
static inline void encoder_pll_update(float theta_meas, float dt)
{
    float error;
    float activity;
    float schedule;
    float target_bandwidth_hz;
    float adapt_alpha;
    float wn;
    float kp;
    float ki;

    if (dt <= 0.0f) {
        return;
    }

    if (g_encoder_pll.inited == 0U) {
        g_encoder_pll.theta_est = theta_meas;
        g_encoder_pll.omega_integral = 0.0f;
        g_encoder_pll.omega_est = 0.0f;
        g_encoder_pll.bandwidth_hz = ENCODER_PLL_BW_MIN_HZ;
        g_encoder_pll.inited = 1U;
        return;
    }

    error = theta_meas - g_encoder_pll.theta_est;
    while (error > _PI) {
        error -= _2PI;
    }
    while (error < -_PI) {
        error += _2PI;
    }

    /* 积分速度负责稳态调度，相位误差负责静止起动时快速提升带宽。 */
    activity = fabsf(g_encoder_pll.omega_integral) +
        (2.0f * _PI * ENCODER_PLL_BW_MIN_HZ) * fabsf(error);
    schedule = activity / ENCODER_PLL_FULL_SPEED_RAD_S;
    if (schedule > 1.0f) {
        schedule = 1.0f;
    }
    schedule = schedule * schedule * (3.0f - 2.0f * schedule);

    target_bandwidth_hz = ENCODER_PLL_BW_MIN_HZ +
        (ENCODER_PLL_BW_MAX_HZ - ENCODER_PLL_BW_MIN_HZ) * schedule;
    adapt_alpha = dt / (ENCODER_PLL_BW_ADAPT_TAU_S + dt);
    g_encoder_pll.bandwidth_hz +=
        adapt_alpha * (target_bandwidth_hz - g_encoder_pll.bandwidth_hz);

    wn = _2PI * g_encoder_pll.bandwidth_hz;
    kp = 2.0f * ENCODER_PLL_DAMPING * wn;
    ki = wn * wn;

    g_encoder_pll.omega_integral += ki * error * dt;
    g_encoder_pll.omega_est = g_encoder_pll.omega_integral + kp * error;
    g_encoder_pll.theta_est =
        angle_normalize(g_encoder_pll.theta_est + g_encoder_pll.omega_est * dt);
}

/**
 * @brief  更新编码器单圈机械角度和多圈机械位置
 * @note   多圈位置格式为 turns * 2*pi + current_angle，单位 rad
 * @note   该函数在 5 kHz 外环中断中调用，并使用最近一次有效的原始角度。
 * @retval float: 单圈机械角度 [0, 2*pi)，单位 rad
 */
float encoder_read_mechanical_angle(void)
{
    static float last_raw = 0.0f;
    static uint8_t first_run = 1U;
    static uint32_t last_update_count;
    float current_angle;

    if ((encoder_data.raw_valid == 0U) ||
        (encoder_data.raw_angle == MC_ENCODER_CALIBRATION_INVALID_RAW)) {
        return -1.0f;
    }

    if ((first_run == 0U) &&
        (last_update_count == encoder_data.raw_update_count)) {
        return encoder_data.mechanical_angle;
    }
    last_update_count = encoder_data.raw_update_count;

    if (mc_encoder_calibration_get_mechanical_angle(
            encoder_data.raw_angle, &current_angle) == 0U) {
        current_angle = encoder_data.raw_angle * ENCODER_ANGLE_SCALE;
    }
    encoder_data.angle_raw_scaled = current_angle;

    if (first_run != 0U) {
        encoder_data.mechanical_angle = current_angle;
        encoder_data.mechanical_position =
            ((float)encoder_data.mechanical_turns * _2PI) + current_angle;
        last_raw = current_angle;
        first_run = 0U;

        encoder_pll_update(encoder_data.mechanical_angle, SPEED_LOOP_DT);
        encoder_data.mechanical_speed = g_encoder_pll.omega_est;
        encoder_data.electrical_speed =
            encoder_data.mechanical_speed * MOTOR_POLE_PAIRS;
        return encoder_data.mechanical_angle;
    }

    float delta = current_angle - last_raw;

    if (delta > _PI) {
        delta -= _2PI;
        encoder_data.mechanical_turns--;
    }
    else if (delta < -_PI) {
        delta += _2PI;
        encoder_data.mechanical_turns++;
    }

    encoder_data.mechanical_angle = current_angle;
    encoder_data.mechanical_position =
        ((float)encoder_data.mechanical_turns * _2PI) + current_angle;
    last_raw = current_angle;

    encoder_pll_update(encoder_data.mechanical_angle, SPEED_LOOP_DT);

    encoder_data.mechanical_speed = g_encoder_pll.omega_est;
    encoder_data.electrical_speed =
        encoder_data.mechanical_speed * MOTOR_POLE_PAIRS;
    return encoder_data.mechanical_angle;
}

/**
 * @brief  读取编码器电角度、Q10 索引和对应的 sin/cos
 * @note   该函数在 30 kHz 采样中断中调用，不在热路径执行 CORDIC。
 * @param  无
 * @retval float: 电角度值 (rad)，失败返回-1.0f
 */
float encoder_read_electrical_angle(void)
{
    uint16_t electrical_angle_index;

    encoder_data.raw_angle = encoder_read_raw_angle();
    if (encoder_data.raw_angle == MC_ENCODER_CALIBRATION_INVALID_RAW) {
        encoder_data.raw_valid = 0U;
        return -1.0f;
    }

    encoder_data.raw_valid = 1U;
    encoder_data.raw_update_count++;

    if ((mc_encoder_calibration_get_angle_index_fast(
             encoder_data.raw_angle, &electrical_angle_index) == 0U) &&
        (mc_encoder_calibration_get_angle_index(
             encoder_data.raw_angle, &electrical_angle_index) == 0U)) {
        electrical_angle_index = (uint16_t)(
            ((uint32_t)encoder_data.raw_angle * MOTOR_POLE_PAIRS) >>
            (ENCODER_RESOLUTION - MC_ANGLE_INDEX_BITS)) &
            MC_ANGLE_INDEX_MASK;
    }

    encoder_data.electrical_angle_index =
        (electrical_angle_index + MC_ANGLE_INDEX_QUARTER) &
        MC_ANGLE_INDEX_MASK;
    encoder_data.electrical_sin =
        mc_sine_table[encoder_data.electrical_angle_index];
    encoder_data.electrical_cos = mc_sine_table[
        (encoder_data.electrical_angle_index + MC_ANGLE_INDEX_QUARTER) &
        MC_ANGLE_INDEX_MASK];
    encoder_data.electrical_angle =
        (float)encoder_data.electrical_angle_index * MC_ANGLE_INDEX_TO_RAD;
    return encoder_data.electrical_angle;
}

/**
 * @brief  清除编码器错误标志
 * @note   读取清除错误标志寄存器即可清除错误状态
 * @param  无
 * @retval 无
 */
void encoder_clear_error(void)
{
    encoder_read_reg(ENCODER_REG_CLEAR_ERR);
}
