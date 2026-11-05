#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "drv_uart.h"
#include "vofa_plot.h"

static uint8_t captured[64];
static size_t captured_len;
static int dma_ready = 1;
static unsigned int write_calls;

bool drv_uart_is_tx_dma_ready(uint8_t instance)
{
    (void)instance;
    return dma_ready != 0;
}

size_t drv_uart_write_dma(uint8_t instance, const void *data, size_t size)
{
    (void)instance;
    write_calls++;
    if ((dma_ready == 0) || (data == NULL) || (size > sizeof(captured))) {
        return 0U;
    }
    memcpy(captured, data, size);
    captured_len = size;
    return size;
}

static int failures;

#define CHECK(condition, message)                                              \
    do {                                                                       \
        if (!(condition)) {                                                    \
            printf("FAIL: %s\n", message);                                     \
            failures++;                                                        \
        }                                                                      \
    } while (0)

static void test_disabled(void)
{
    const float values[8] = {1.0f};

    captured_len = 0U;
    write_calls = 0U;
    vofa_plot_set_enabled(false);
    vofa_plot_send(values, 8U);
    CHECK(captured_len == 0U, "disabled sends nothing");
    CHECK(write_calls == 0U, "disabled does not touch DMA");
}

static void test_just_float_frame(void)
{
    const float values[8] = {1.5f,   -2.25f, 3.0f,   4.0f,
                             -5.0f,  6.5f,   7.0f,   -8.75f};
    const uint8_t expected_tail[4] = {0x00U, 0x00U, 0x80U, 0x7FU};

    captured_len = 0U;
    write_calls = 0U;
    dma_ready = 1;
    vofa_plot_set_enabled(true);
    vofa_plot_send(values, 8U);

    CHECK(captured_len == 36U, "frame is 8 floats plus 4 tail bytes");
    CHECK(write_calls == 1U, "frame sent with one DMA write");

    for (size_t i = 0U; i < 8U; i++) {
        float received;

        memcpy(&received, &captured[i * sizeof(float)], sizeof(received));
        CHECK(fabsf(received - values[i]) < 1.0e-6f,
              "channel value round-trips");
    }
    CHECK(memcmp(&captured[32], expected_tail, 4U) == 0,
          "JustFloat tail is 00 00 80 7F");
}

static void test_dma_busy(void)
{
    const float values[3] = {1.0f, 2.0f, 3.0f};

    captured_len = 0U;
    write_calls = 0U;
    dma_ready = 0;
    vofa_plot_send(values, 3U);
    CHECK(captured_len == 0U, "busy DMA drops frame");
    CHECK(write_calls == 0U, "busy DMA is not written");
}

int main(void)
{
    test_disabled();
    test_just_float_frame();
    test_dma_busy();

    if (failures != 0) {
        printf("vofa_plot: %d failure(s)\n", failures);
        return EXIT_FAILURE;
    }
    printf("vofa_plot: PASS\n");
    return EXIT_SUCCESS;
}
