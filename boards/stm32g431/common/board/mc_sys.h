/**
 * @file mc_sys.h
 * @brief FOC 定时任务和中断入口。
 */

#ifndef MC_SYS_H_
#define MC_SYS_H_

#include <config.h>
#include <stdbool.h>

#include "adc.h"
#include "main.h"
#include "mc_encoder.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_tim.h"

/**
 * @brief FOC定时任务结构体
 */
typedef struct {
    volatile bool task_update_vbus; /**< 请求更新母线电压 */
} MC_TASK;

/**
 * @brief 全局 FOC 定时任务标志
 */
extern MC_TASK mc_task;

#define SYSTICK_FREQUENCY_HZ          1000U /**< 系统滴答频率（Hz） */
#define TASK_VBUS_UPDATE_FREQUENCY_HZ 100U  /**< 母线电压更新频率（Hz） */
/**
 * @brief 致命错误处理入口
 */
void Error_Handler(void);

#endif /* MC_SYS_H_ */
