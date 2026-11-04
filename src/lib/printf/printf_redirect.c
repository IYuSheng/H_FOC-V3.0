// Copyright (c) 2026 Shenzhen Langzhua Intelligence Technology Co., Ltd.
// (CLAWLAB). All rights reserved.

#include "board_config.h"
#include "drv_uart.h"

#if defined(CONFIG_UART)
void _putchar(char character)
{
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
