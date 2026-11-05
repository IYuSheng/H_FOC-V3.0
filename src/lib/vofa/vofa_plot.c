#include "vofa_plot.h"

#include <stddef.h>

#include "board_config.h"
#include "drv_uart.h"

/** @brief JustFloat 帧：N 个 little-endian float + 4 字节帧尾 */
typedef struct {
    float data[VOFA_PLOT_CHANNELS];
    uint8_t tail[4];
} vofa_just_float_t;

_Static_assert(sizeof(vofa_just_float_t) ==
                   ((VOFA_PLOT_CHANNELS * sizeof(float)) + 4U),
               "VOFA JustFloat frame layout changed");

static vofa_just_float_t vofa_frame;
static volatile bool vofa_enabled;

void vofa_plot_set_enabled(bool enabled)
{
    vofa_enabled = enabled;
}

bool vofa_plot_is_enabled(void)
{
    return vofa_enabled;
}

void vofa_plot_send(const float *values, uint8_t count)
{
    if ((!vofa_enabled) || (values == NULL) || (count == 0U) ||
        (count > VOFA_PLOT_CHANNELS)) {
        return;
    }
    if (!drv_uart_is_tx_dma_ready(CONFIG_UART_INSTANCE)) {
        return;
    }

    for (uint8_t i = 0U; i < count; i++) {
        vofa_frame.data[i] = values[i];
    }
    for (uint8_t i = count; i < VOFA_PLOT_CHANNELS; i++) {
        vofa_frame.data[i] = 0.0f;
    }

    /* VOFA+ JustFloat 帧尾：0x00 0x00 0x80 0x7f */
    vofa_frame.tail[0] = 0x00U;
    vofa_frame.tail[1] = 0x00U;
    vofa_frame.tail[2] = 0x80U;
    vofa_frame.tail[3] = 0x7FU;

    (void)drv_uart_write_dma(CONFIG_UART_INSTANCE, &vofa_frame,
                             sizeof(vofa_frame));
}
