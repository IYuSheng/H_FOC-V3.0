#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "mc_input_shaper.h"

#define TEST_PI 3.14159265358979323846f

static int failures;

#define CHECK(condition, message)                                              \
    do {                                                                       \
        if (!(condition)) {                                                    \
            printf("FAIL: %s\n", message);                                     \
            failures++;                                                        \
        }                                                                      \
    } while (0)

static float amp_sum(const input_shaper_t *shaper)
{
    float sum = 0.0f;

    for (uint8_t i = 0U; i < shaper->num_taps; i++) {
        sum += shaper->amp[i];
    }
    return sum;
}

static void test_disabled_passthrough(void)
{
    input_shaper_t shaper;

    CHECK(input_shaper_init(&shaper, INPUT_SHAPER_TYPE_NONE, 0.0f, 0.0f,
                            0.001f) == 0,
          "NONE init");
    CHECK(!input_shaper_is_enabled(&shaper), "NONE disabled");
    CHECK(input_shaper_process(&shaper, 1.25f) == 1.25f, "NONE passthrough");
}

static void test_ei_config(void)
{
    input_shaper_t shaper;

    CHECK(input_shaper_init(&shaper, INPUT_SHAPER_TYPE_EI, 50.0f, 0.05f,
                            0.0002f) == 0,
          "EI 50Hz init");
    CHECK(input_shaper_is_enabled(&shaper), "EI 50Hz enabled");
    CHECK(shaper.num_taps == 3U, "EI has three taps");
    CHECK(shaper.delay[0] == 0U, "first tap has no delay");
    CHECK(shaper.delay[1] > shaper.delay[0], "delays increasing 0->1");
    CHECK(shaper.delay[2] > shaper.delay[1], "delays increasing 1->2");
    CHECK(fabsf(amp_sum(&shaper) - 1.0f) < 1.0e-5f, "EI amplitudes sum to 1");
    CHECK(input_shaper_get_max_delay(&shaper) == shaper.delay[2],
          "max delay is last tap");
}

static void test_zv_config(void)
{
    input_shaper_t shaper;

    CHECK(input_shaper_init(&shaper, INPUT_SHAPER_TYPE_ZV, 50.0f, 0.05f,
                            0.0002f) == 0,
          "ZV 50Hz init");
    CHECK(shaper.num_taps == 2U, "ZV has two taps");
    CHECK(fabsf(amp_sum(&shaper) - 1.0f) < 1.0e-5f, "ZV amplitudes sum to 1");
}

static void test_sample_rate_guard(void)
{
    input_shaper_t shaper;

    CHECK(input_shaper_init(&shaper, INPUT_SHAPER_TYPE_EI, 300.0f, 0.05f,
                            0.0002f) != 0,
          "reject 300Hz at 5kHz position loop");
    CHECK(!input_shaper_is_enabled(&shaper),
          "rejected shaper stays passthrough");
    CHECK(input_shaper_init(&shaper, INPUT_SHAPER_TYPE_EI, 150.0f, 0.05f,
                            0.0002f) == 0,
          "accept 150Hz at 5kHz position loop");
}

static void test_step_response(void)
{
    input_shaper_t shaper;
    float output = 0.0f;

    CHECK(input_shaper_init(&shaper, INPUT_SHAPER_TYPE_EI, 40.0f, 0.05f,
                            0.0002f) == 0,
          "EI 40Hz init");
    input_shaper_reset(&shaper, 0.0f);

    for (int i = 0; i < 1000; i++) {
        output = input_shaper_process(&shaper, 1.0f);
        if (i == 0) {
            CHECK(output < 1.0f, "shaped step is delayed");
        }
    }
    CHECK(fabsf(output - 1.0f) < 1.0e-5f, "shaped step settles to one");
}

static void test_oscillator_residual(void)
{
    const float frequency_hz = 50.0f;
    const float omega = 2.0f * TEST_PI * frequency_hz;
    const float zeta = 0.01f;
    const float dt = 0.0002f;
    input_shaper_t shaper;
    float x_raw = 0.0f;
    float v_raw = 0.0f;
    float x_shaped = 0.0f;
    float v_shaped = 0.0f;
    float raw_peak = 0.0f;
    float shaped_peak = 0.0f;

    CHECK(input_shaper_init(&shaper, INPUT_SHAPER_TYPE_EI, frequency_hz, zeta,
                            dt) == 0,
          "EI oscillator init");
    input_shaper_reset(&shaper, 0.0f);

    for (int i = 0; i < 10000; i++) {
        const float command = 1.0f;
        const float shaped = input_shaper_process(&shaper, command);
        float accel_raw;
        float accel_shaped;

        /* 二阶谐振模型：x'' = w^2 (u - x) - 2 zeta w x' */
        accel_raw = (omega * omega * (command - x_raw)) - (2.0f * zeta * omega * v_raw);
        v_raw += accel_raw * dt;
        x_raw += v_raw * dt;
        accel_shaped = (omega * omega * (shaped - x_shaped)) -
            (2.0f * zeta * omega * v_shaped);
        v_shaped += accel_shaped * dt;
        x_shaped += v_shaped * dt;

        if (i > 5000) {
            raw_peak = fmaxf(raw_peak, fabsf(x_raw - 1.0f));
            shaped_peak = fmaxf(shaped_peak, fabsf(x_shaped - 1.0f));
        }
    }

    printf("oscillator residual raw=%.6f shaped=%.6f ratio=%.3f\n",
           (double)raw_peak, (double)shaped_peak,
           (double)(shaped_peak / raw_peak));
    CHECK(shaped_peak < raw_peak, "EI reduces residual vibration");
    CHECK(shaped_peak < (0.2f * raw_peak), "EI residual below 20%");
}

int main(void)
{
    test_disabled_passthrough();
    test_ei_config();
    test_zv_config();
    test_sample_rate_guard();
    test_step_response();
    test_oscillator_residual();

    if (failures != 0) {
        printf("mc_input_shaper: %d failure(s)\n", failures);
        return EXIT_FAILURE;
    }
    printf("mc_input_shaper: PASS\n");
    return EXIT_SUCCESS;
}
