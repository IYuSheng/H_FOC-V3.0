#include <stdint.h>

#include "shell.h"
#include "stm32g4xx.h"

/**
 * @brief 重启系统
 * @param argc 参数数量
 * @param argv 参数数组
 */
static void cmd_reboot(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    NVIC_SystemReset();
}

#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-function-type"
#endif

SHELL_EXPORT_CMD(
    SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN) |
        SHELL_CMD_DISABLE_RETURN,
    reboot, cmd_reboot, reboot system);

#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
