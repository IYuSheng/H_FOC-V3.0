#include "mc_shell_port.h"

#include <stddef.h>

#include "board_config.h"
#include "drv_uart.h"
#include "shell.h"

static Shell shell;
static char shell_buffer[512];

/**
 * @brief 判断输入字节是否为有效 Shell 命令字符
 * @param data 输入字节
 * @return 1 表示有效，0 表示应丢弃
 */
static uint8_t mc_shell_byte_valid(uint8_t data)
{
    return ((data >= 0x20U) && (data <= 0x7EU)) ||
        (data == '\r') || (data == '\n') || (data == '\t') ||
        (data == '\b') || (data == 0x7FU) || (data == 0x1BU);
}

/**
 * @brief letter-shell 串口读取回调
 * @param data 接收缓冲区
 * @param length 最大读取长度
 * @return 实际读取字节数
 */
static signed short mc_shell_read(char *data, unsigned short length)
{
    uint8_t value;

    if ((data == NULL) || (length == 0U)) {
        return 0;
    }

    for (uint8_t attempt = 0U; attempt < 32U; attempt++) {
        if (drv_uart_read(CONFIG_UART_INSTANCE, &value, 1U) == 0U) {
            return 0;
        }
        if (mc_shell_byte_valid(value) != 0U) {
            *data = (char)value;
            return 1;
        }
    }
    return 0;
}

/**
 * @brief letter-shell 串口写入回调
 * @param data 发送缓冲区
 * @param length 发送长度
 * @return 实际写入字节数
 */
static signed short mc_shell_write(char *data, unsigned short length)
{
    return (signed short)drv_uart_write(CONFIG_UART_INSTANCE, data, length);
}

/**
 * @brief 初始化 letter-shell
 */
void shell_init(void)
{
    shell.write = mc_shell_write;
    shell.read = mc_shell_read;
    shellInit(&shell, shell_buffer, sizeof(shell_buffer));
}

/**
 * @brief 执行 letter-shell 后台任务
 */
void shell_task(void)
{
    shellTask(&shell);
}
