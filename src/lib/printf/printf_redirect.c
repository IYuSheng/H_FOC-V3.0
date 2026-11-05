// Copyright (c) 2026 Shenzhen Langzhua Intelligence Technology Co., Ltd.
// (CLAWLAB). All rights reserved.

#include "board_config.h"
#include "drv_uart.h"
#include "vofa_plot.h"

#if defined(CONFIG_UART)
void _putchar(char character)
{
    /* VOFA 二进制流占用串口时丢弃文本，避免破坏 JustFloat 帧。 */
    if (vofa_plot_is_enabled()) {
        return;
    }
    drv_uart_write(CONFIG_UART_INSTANCE, &character, 1);
}

#if 0
int _write(int file, char *ptr, int len)
{
    for (int idx = 0; idx < len; idx++) {
        drv_uart_write(CONFIG_UART_INSTANCE, &ptr[idx], 1);
    }
    return len;
}
#endif

#endif
