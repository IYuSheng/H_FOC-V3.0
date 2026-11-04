#include <string.h>

#include "identify_parameter.h"
#include "mc_calibration.h"
#include "mc_control.h"
#include "mc_frequency_sweep.h"
#include "mc_runtime.h"
#include "printf.h"
#include "shell.h"

/**
 * @brief 打印 sweep 帮助
 */
static void usage(void)
{
    printf("sweep <start|stop|status>\r\n");
}

/**
 * @brief 频率响应扫频命令入口
 * @param argc 参数数量
 * @param argv 参数数组
 */
void test_sweep(int argc, char *argv[])
{
    if (argc < 2) {
        goto exit;
    }

    if (strcmp(argv[1], "start") == 0) {
        if ((mc_runtime_get_mode() != MC_MODE_IDLE) ||
            (mc_encoder_calibration_is_running() != 0U) ||
            identify_parameter_is_active()) {
            printf("err:sweep busy\r\n");
            return;
        }
        if (motor_ctrl.fault_type != MOTOR_FAULT_NONE) {
            printf("err:sweep fault=%d\r\n", motor_ctrl.fault_type);
            return;
        }
        if (mc_control_start_frequency_sweep() != 0U) {
            printf("index,frequency_hz,iq_amplitude_a,position_amplitude_rad,"
                   "gain_rad_per_a,phase_deg,window_count,gain_std_percent,"
                   "phase_std_deg\r\n");
        }
        else {
            printf("err:sweep start failed\r\n");
        }
        return;
    }

    if (strcmp(argv[1], "stop") == 0) {
        if (mc_frequency_sweep_is_active()) {
            mc_frequency_sweep_stop();
            printf("sweep stopped\r\n");
        }
        else {
            printf("err:sweep not active\r\n");
        }
        return;
    }

    if (strcmp(argv[1], "status") == 0) {
        mc_sweep_status_t status;

        mc_frequency_sweep_get_status(&status);
        printf("sweep state:%s index:%u error:%.6f speed:%.6f return_ms:%lu\r\n",
               mc_frequency_sweep_get_state_name(),
               (unsigned int)status.frequency_index, status.position_error,
               status.speed, (unsigned long)status.return_elapsed_ms);
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
    sweep, test_sweep, Frequency response sweep);

#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
