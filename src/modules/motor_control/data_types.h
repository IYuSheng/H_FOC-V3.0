/**
 * @file data_types.h
 * @brief 电流、电压采样和电机控制公共数据类型。
 */

#ifndef MC_DATA_TYPES_H_
#define MC_DATA_TYPES_H_

#include <stdint.h>

#include "board_config.h"

/**
 * @brief ADC 原始采样数据
 */
typedef struct {
    int16_t IA;   /**< A 相原始电流 */
    int16_t IB;   /**< B 相原始电流 */
    int16_t ia;   /**< A 相电流 */
    int16_t ib;   /**< B 相电流 */
    int16_t ic;   /**< C 相电流 */
    int16_t va;   /**< A 相电压 */
    int16_t vb;   /**< B 相电压 */
    int16_t vc;   /**< C 相电压 */
    int16_t vbus; /**< 母线电压 */
} mc_data_t;

/**
 * @brief 电流采样换算数据
 */
typedef struct {
    float IA; /**< A 相电流（A） */
    float IB; /**< B 相电流（A） */
    float ia; /**< A 相电流（A） */
    float ib; /**< B 相电流（A） */
    float ic; /**< C 相电流（A） */
} mc_data_i;

/**
 * @brief 电压和温度采样换算数据
 */
typedef struct {
    float va;         /**< A 相电压（V） */
    float vb;         /**< B 相电压（V） */
    float vc;         /**< C 相电压（V） */
    float vbus;       /**< 母线电压（V） */
    float vbus_inv;   /**< 母线电压倒数（1/V） */
    float spwm_scale; /**< SPWM 电压到比较值的预计算比例 */
    float temp;       /**< 温度（摄氏度） */
} mc_data_v;

/**
 * @brief ADC 原始采样全局数据
 */
extern mc_data_t mc_raw_data;

/**
 * @brief 电流采样全局数据
 */
extern mc_data_i mc_current_data;

/**
 * @brief 电压和温度采样全局数据
 */
extern mc_data_v mc_voltage_data;

/**
 * @brief 电机保护状态
 */
typedef struct {
    MC_FAULT fault_type; /**< 当前故障类型 */
    float temp_max;      /**< 最高温度阈值（摄氏度） */
    float vbus_max;      /**< 母线最高电压阈值（V） */
    float vbus_min;      /**< 母线最低电压阈值（V） */
    float current_max;   /**< 最大电流阈值（A） */
    float angle_max;     /**< 机械角度上限（rad），0 表示该方向不限制 */
    float angle_min;     /**< 机械角度下限（rad），0 表示该方向不限制 */
} Motor_Control_t;

/**
 * @brief 全局电机保护状态
 */
extern Motor_Control_t motor_ctrl;

#endif /* MC_DATA_TYPES_H_ */
