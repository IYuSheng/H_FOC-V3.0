#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "board_config.h"
#include "identify_inductance.h"
#include "identify_mechanical.h"
#include "identify_parameter.h"
#include "identify_resistance.h"
#include "mc_calibration.h"
#include "mc_control.h"
#include "mc_parse.h"
#include "mc_runtime.h"
#include "printf.h"
#include "shell.h"

/**
 * @brief 检查是否允许开始辨识
 * @return 1 表示允许，0 表示拒绝
 */
static uint8_t identify_require_idle(void)
{
    if (mc_runtime_get_mode() != MC_MODE_IDLE) {
        printf("err:mc not idle\r\n");
        return 0U;
    }
    if (motor_ctrl.fault_type != MOTOR_FAULT_NONE) {
        printf("err:identify fault=%d\r\n", motor_ctrl.fault_type);
        return 0U;
    }
    if ((mc_encoder_calibration_is_running() != 0U) ||
        identify_parameter_is_active()) {
        printf("err:maintenance busy\r\n");
        return 0U;
    }
    return 1U;
}

/**
 * @brief 打印 identify 帮助
 */
static void usage(void)
{
    printf("identify <res|ind|mech|stop|status>\r\n");
    printf(" res                         - identify resistance\r\n");
    printf(" ind                         - identify inductance\r\n");
    printf(" mech                        - identify mechanical parameters\r\n");
    printf(" mech <low_a> <high_a>       - identify with current amplitudes\r\n");
    printf(" stop                        - stop identification\r\n");
    printf(" status                      - show identification state\r\n");
}

/**
 * @brief identify 命令入口
 * @param argc 参数数量
 * @param argv 参数数组
 */
void test_identify(int argc, char *argv[])
{
    if (argc < 2) {
        goto exit;
    }

    if ((strcmp(argv[1], "res") == 0) ||
        (strcmp(argv[1], "resistance") == 0)) {
        if (identify_require_idle() == 0U) {
            return;
        }
        if (identify_resistance_start() != 0) {
            printf("err:res start fail\r\n");
        }
        else {
            printf("identifying resistance...\r\n");
        }
        return;
    }

    if ((strcmp(argv[1], "ind") == 0) ||
        (strcmp(argv[1], "inductance") == 0)) {
        if (identify_require_idle() == 0U) {
            return;
        }
        if (identify_inductance_start() != 0) {
            printf("err:ind start fail\r\n");
        }
        else {
            printf("identifying inductance...\r\n");
        }
        return;
    }

    if ((strcmp(argv[1], "mech") == 0) ||
        (strcmp(argv[1], "mechanical") == 0)) {
        if (identify_require_idle() == 0U) {
            return;
        }
        if (argc == 2) {
            if (mc_control_start_mechanical_identification() != 0U) {
                printf("identifying mechanical...\r\n");
            }
            else {
                printf("err:mech start fail\r\n");
            }
            return;
        }
        if (argc == 4) {
            float low_amplitude;
            float high_amplitude;

            if ((mc_parse_float(argv[2], &low_amplitude) == 0U) ||
                (mc_parse_float(argv[3], &high_amplitude) == 0U) ||
                (low_amplitude <= 0.0f) ||
                (low_amplitude >= high_amplitude) ||
                (high_amplitude > CURRENT_LIMIT)) {
                printf("err:expected 0 < low < high <= %.2fA\r\n",
                       CURRENT_LIMIT);
                return;
            }
            if (mc_control_start_mechanical_identification_with_amplitudes(
                    low_amplitude, high_amplitude) != 0U) {
                printf("identifying mechanical low:%.3fA high:%.3fA\r\n",
                       low_amplitude, high_amplitude);
            }
            else {
                printf("err:mech start fail\r\n");
            }
            return;
        }
    }

    if (strcmp(argv[1], "stop") == 0) {
        identify_parameter_stop();
        identify_mechanical_stop();
        printf("identify stopped\r\n");
        return;
    }

    if (strcmp(argv[1], "status") == 0) {
        const identify_mechanical_result_t *result =
            identify_mechanical_get_result();

        printf("identify res:%u ind:%u mech:%u\r\n",
               (unsigned int)identify_resistance_get()->state,
               (unsigned int)identify_inductance_get()->state,
               (unsigned int)identify_mechanical_get_state());
        printf("mech low:%u code:%u Ka:%.9f Kv:%.9f Tc:%.9f rms:%.2f%%\r\n",
               (unsigned int)result->low_frequency_valid,
               (unsigned int)result->low_frequency_error_code,
               (double)result->ka, (double)result->kv, (double)result->tc,
               (double)result->weighted_rms_percent);
        printf("mech res:%u code:%u f:%.2fHz zeta:%.3f Q:%.2f peak:%.1fdB\r\n",
               (unsigned int)result->resonance_valid,
               (unsigned int)result->resonance_error_code,
               (double)result->resonance_frequency_hz,
               (double)result->resonance_damping_ratio,
               (double)result->resonance_quality_factor,
               (double)result->resonance_prominence_db);
        return;
    }

exit:
    usage();
}

#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-function-type"
#endif

SHELL_EXPORT_CMD(
    SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN) |
        SHELL_CMD_DISABLE_RETURN,
    identify, test_identify, Motor parameter identification);

#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
