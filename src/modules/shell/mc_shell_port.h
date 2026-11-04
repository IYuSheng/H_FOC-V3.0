/**
 * @file mc_shell_port.h
 * @brief letter-shell 串口接入接口。
 */

#ifndef MC_SHELL_PORT_H_
#define MC_SHELL_PORT_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 letter-shell
 */
void shell_init(void);

/**
 * @brief 执行 letter-shell 后台任务
 */
void shell_task(void);

#ifdef __cplusplus
}
#endif

#endif /* MC_SHELL_PORT_H_ */
