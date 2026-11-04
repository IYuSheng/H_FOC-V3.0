/**
 * @file logger.h
 * @brief EasyLogger 初始化和日志宏入口。
 */

#ifndef LOGGER_H_
#define LOGGER_H_

#include "elog.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 EasyLogger
 */
void syslog_init(void);

/**
 * @brief 停止并反初始化 EasyLogger
 */
void syslog_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* LOGGER_H_ */
