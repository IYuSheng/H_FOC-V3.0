#include <string.h>

#include "printf.h"
#include "shell.h"
#include "vofa_plot.h"

/**
 * @brief 打印 vofa 命令帮助
 */
static void vofa_usage(void)
{
    printf("vofa <on|off|status>\r\n");
}

/**
 * @brief vofa 命令入口
 * @param argc 参数数量
 * @param argv 参数数组
 */
void test_vofa(int argc, char *argv[])
{
    if (argc != 2) {
        vofa_usage();
        return;
    }

    if (strcmp(argv[1], "on") == 0) {
        /* 先输出确认文本，再开始二进制流，避免文本混入 JustFloat 帧间。 */
        printf("vofa on: %u channels\r\n", (unsigned int)VOFA_PLOT_CHANNELS);
        vofa_plot_set_enabled(true);
        return;
    }

    if (strcmp(argv[1], "off") == 0) {
        vofa_plot_set_enabled(false);
        printf("vofa off\r\n");
        return;
    }

    if (strcmp(argv[1], "status") == 0) {
        if (vofa_plot_is_enabled()) {
            /* 使能时串口被二进制流占用，文本会被丢弃，这里只是保持状态机一致。 */
            printf("vofa on\r\n");
        }
        else {
            printf("vofa off\r\n");
        }
        return;
    }

    vofa_usage();
}

#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-function-type"
#endif

SHELL_EXPORT_CMD(
    SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN) |
        SHELL_CMD_DISABLE_RETURN,
    vofa, test_vofa, VOFA just float output);

#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
