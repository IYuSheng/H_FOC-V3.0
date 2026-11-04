/**
 * @file mc_shell_cfg.h
 * @brief letter-shell 的板级配置。
 */

#ifndef MC_SHELL_CFG_H_
#define MC_SHELL_CFG_H_

#include "stm32g4xx_hal.h"

#define SHELL_TASK_WHILE       0
#define SHELL_USING_CMD_EXPORT 1
#define SHELL_USING_COMPANION  0
#define SHELL_GET_TICK()       HAL_GetTick()
#define SHELL_SHOW_INFO        0

#endif /* MC_SHELL_CFG_H_ */
