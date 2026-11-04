/**
 * @file mc_encoder.h
 * @brief MT6816 14 位磁编码器采样、角度换算和自适应 PLL。
 */

#ifndef MC_ENCODER_H_
#define MC_ENCODER_H_

#include <math.h>
#include <stdint.h>

#include "mc_math.h"
#include "spi.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_gpio.h"

#define ENCODER_CS_PORT GPIOA         /**< MT6816 片选端口 */
#define ENCODER_CS_PIN  LL_GPIO_PIN_4 /**< MT6816 片选引脚 */
#define ENCODER_CS_LOW()                                                       \
    LL_GPIO_ResetOutputPin(ENCODER_CS_PORT, ENCODER_CS_PIN) /**< 拉低片选 */
#define ENCODER_CS_HIGH()                                                      \
    LL_GPIO_SetOutputPin(ENCODER_CS_PORT, ENCODER_CS_PIN) /**< 拉高片选 */

#define ENCODER_READ_BIT   0x8000U /**< 读命令标志位 */
#define ENCODER_ADDR_MASK  0x7FU   /**< 7 位寄存器地址掩码 */
#define ENCODER_ADDR_SHIFT 8U      /**< 寄存器地址移位量 */

#define ENCODER_REG_ID        0x00U /**< 芯片识别寄存器 */
#define ENCODER_REG_STATUS    0x01U /**< 状态和诊断寄存器 */
#define ENCODER_REG_ANGLE_H   0x03U /**< 角度高 8 位 */
#define ENCODER_REG_ANGLE_L   0x04U /**< 角度低 6 位、告警和校验位 */
#define ENCODER_REG_CLEAR_ERR 0x05U /**< 清除错误标志寄存器 */

#define ENCODER_RESOLUTION 14U /**< 编码器分辨率（bit） */
#define ENCODER_MAX_VALUE  ((1U << ENCODER_RESOLUTION) - 1U)
#define ENCODER_ANGLE_SCALE                                                    \
    (_2PI / (float)(1U << ENCODER_RESOLUTION)) /**< 原始值到角度比例 */
#define ENCODER_ELECTRICAL_SCALE                                               \
    (ENCODER_ANGLE_SCALE * MOTOR_POLE_PAIRS) /**< 机械角到电角度比例 */
#define ENCODER_SENSOR_PERIODS_PER_REV 2U    /**< 每机械圈磁周期数 */
#define ENCODER_PERIOD_COUNTS                                                  \
    ((1U << ENCODER_RESOLUTION) / ENCODER_SENSOR_PERIODS_PER_REV)
#define ENCODER_PERIOD_ANGLE_SCALE (_2PI / (float)ENCODER_PERIOD_COUNTS)
#define ENCODER_EFFECTIVE_POLE_PAIRS                                           \
    (MOTOR_POLE_PAIRS / ENCODER_SENSOR_PERIODS_PER_REV)
#define SPEED_CALC_INTERVAL 0.001f /**< 速度计算周期（s） */
#define SPEED_FILTER_K      0.2f   /**< 速度低通系数 */

/**
 * @brief 编码器初始化状态
 */
typedef enum {
    ENCODER_STATUS_OK = 0,     /**< 正常 */
    ENCODER_STATUS_ERR_COMM,   /**< 通信错误 */
    ENCODER_STATUS_ERR_CORDIC, /**< CORDIC 错误 */
    ENCODER_STATUS_ERR_MAGNET  /**< 磁场异常 */
} encoder_status_t;

/**
 * @brief 编码器采样数据
 */
typedef struct {
    uint16_t raw_angle;              /**< 原始角度（0~16383） */
    uint16_t electrical_angle_index; /**< 用于 FOC 的 Q10 电角度索引 */
    float angle_raw_scaled;          /**< 单磁周期机械角（rad） */
    float mechanical_angle;          /**< 单圈机械角（rad） */
    float electrical_angle;          /**< 电角度（rad） */
    float electrical_sin;            /**< 电角度正弦值 */
    float electrical_cos;            /**< 电角度余弦值 */
    float electrical_speed;          /**< 电角速度（rad/s） */
    int32_t mechanical_turns;        /**< 累计机械圈数 */
    float mechanical_position;       /**< 多圈机械位置（rad） */
    float mechanical_speed;          /**< 机械速度（rad/s） */
    float mechanical_accsd;          /**< 机械加速度（rad/s^2） */
    volatile uint32_t raw_update_count; /**< 有效原始角度更新计数 */
    uint32_t error_count;               /**< 编码器累计读取错误次数 */
    uint32_t consecutive_error_count;   /**< 编码器连续读取错误次数 */
    uint8_t raw_valid;                  /**< 最近一次原始角度是否有效 */
    uint8_t error_flag;                 /**< 当前读取周期错误标志 */
    uint8_t fault_latched;              /**< 编码器故障是否已经锁存 */
    uint8_t agc_value;                  /**< 自动增益值 */
} encoder_data_t;

/**
 * @brief 编码器自适应 PLL 状态
 */
typedef struct {
    float theta_est;      /**< 估计单圈机械角（rad） */
    float omega_integral; /**< PLL 积分速度（rad/s） */
    float omega_est;      /**< PLL 输出机械速度（rad/s） */
    float bandwidth_hz;   /**< 当前自适应带宽（Hz） */
    uint8_t inited;       /**< 初始化标志 */
} encoder_pll_t;

/**
 * @brief 全局编码器数据
 */
extern encoder_data_t encoder_data;

/**
 * @brief 全局编码器 PLL 状态
 */
extern encoder_pll_t g_encoder_pll;

/**
 * @brief 初始化编码器
 * @return 编码器初始化状态
 */
encoder_status_t encoder_init(void);

/**
 * @brief 读取编码器原始角度
 * @return 原始角度（0~16383），失败时返回 0xFFFF
 */
uint16_t encoder_read_raw_angle(void);

/**
 * @brief 读取单圈机械角度并更新速度
 * @note 该函数应在外环 5 kHz 中断中调用。
 * @return 单圈机械角度（rad），失败时返回 -1.0f
 */
float encoder_read_mechanical_angle(void);

/**
 * @brief 读取电角度、Q10 索引和对应的 sin/cos
 * @return 电角度（rad），失败时返回 -1.0f
 */
float encoder_read_electrical_angle(void);

/**
 * @brief 临时设置当前编码器角度为零点
 * @return 编码器状态
 */
encoder_status_t encoder_set_zero_temp(void);

/**
 * @brief 清除编码器错误标志
 */
void encoder_clear_error(void);

#endif /* MC_ENCODER_H_ */
