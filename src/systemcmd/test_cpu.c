#include <string.h>
#include <stdlib.h>

#include "cpu.h"
#include "printf.h"
#include "shell.h"

/**
 * @brief 打印 cpu 命令帮助
 */
static void usage(void)
{
    printf("cpu [<set|stop>] [ms]\r\n");
    printf(" set <ms> - set auto print interval\r\n");
    printf(" stop     - stop auto print\r\n");
}

/**
 * @brief 严格解析无符号整数
 * @param token 参数字符串
 * @param value 解析结果
 * @return 1 表示成功，0 表示失败
 */
static uint8_t parse_u32(const char *token, uint32_t *value)
{
    char *end;
    unsigned long parsed;

    if ((token == NULL) || (value == NULL) || (*token == '-')) {
        return 0U;
    }
    parsed = strtoul(token, &end, 10);
    if ((end == token) || (*end != '\0') || (parsed > UINT32_MAX)) {
        return 0U;
    }
    *value = (uint32_t)parsed;
    return 1U;
}

/**
 * @brief CPU 占用率统计命令入口
 * @param argc 参数数量
 * @param argv 参数数组
 */
void test_cpu(int argc, char *argv[])
{
    uint32_t interval_ms;

    if (argc < 2) {
        cpu_usage_printf();
        return;
    }

    if (strcmp(argv[1], "set") == 0) {
        if ((argc < 3) || (parse_u32(argv[2], &interval_ms) == 0U) ||
            (interval_ms == 0U)) {
            printf("err:ms>0\r\n");
            return;
        }
        cpu_set_print_interval(interval_ms);
        printf("interval:%lums\r\n", (unsigned long)interval_ms);
        return;
    }

    if (strcmp(argv[1], "stop") == 0) {
        cpu_set_print_interval(0U);
        printf("auto print stopped\r\n");
        return;
    }

    usage();
}

#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-function-type"
#endif

SHELL_EXPORT_CMD(
    SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN) |
        SHELL_CMD_DISABLE_RETURN,
    cpu, test_cpu, CPU usage statistics);

#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
