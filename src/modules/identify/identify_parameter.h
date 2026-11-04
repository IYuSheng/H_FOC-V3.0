/**
 * @file identify_parameter.h
 * @brief 电机参数辨识模块统一入口。
 */

#ifndef IDENTIFY_PARAMETER_H_
#define IDENTIFY_PARAMETER_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化电阻、电感和机械参数辨识模块
 * @return 0 表示初始化成功，负值表示失败
 */
int8_t identify_parameter_init(void);

/**
 * @brief 查询电阻或电感辨识是否正在运行
 * @return true 表示电气参数辨识正在运行
 */
bool identify_parameter_is_electrical_active(void);

/**
 * @brief 查询任一参数辨识任务是否正在运行
 * @return true 表示存在活动辨识任务
 */
bool identify_parameter_is_active(void);

/**
 * @brief 在 5 kHz 控制节拍推进电气参数辨识状态机
 */
void identify_parameter_control_tick(void);

/**
 * @brief 在主循环完成需要 FLASH 写入的参数保存
 */
void identify_parameter_background_process(void);

/**
 * @brief 停止电气参数辨识并将功率级置为零电压
 */
void identify_parameter_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* IDENTIFY_PARAMETER_H_ */
