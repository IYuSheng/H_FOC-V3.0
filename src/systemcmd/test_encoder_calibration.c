#include <string.h>

#include "data_types.h"
#include "identify_parameter.h"
#include "mc_calibration.h"
#include "mc_runtime.h"
#include "printf.h"
#include "shell.h"

/**
 * @brief 打印 calib_enc 帮助
 */
static void usage(void)
{
    printf("calib_enc <start>\r\n");
    printf(" start - start encoder calibration\r\n");
}

/**
 * @brief 编码器校准命令入口
 * @param argc 参数数量
 * @param argv 参数数组
 */
void test_encoder_calibration(int argc, char *argv[])
{
    if (argc < 2) {
        goto exit;
    }
    if (strcmp(argv[1], "start") != 0) {
        goto exit;
    }
    if ((mc_runtime_get_mode() != MC_MODE_IDLE) ||
        identify_parameter_is_active()) {
        printf("err:calib busy\r\n");
        return;
    }
    if (mc_runtime_is_ready() == 0U) {
        printf("err:calib not ready\r\n");
        return;
    }
    if (motor_ctrl.fault_type != MOTOR_FAULT_NONE) {
        printf("err:calib fault=%d\r\n", motor_ctrl.fault_type);
        return;
    }
    if (mc_encoder_calibration_start() != 0U) {
        printf("start encoder calib...\r\n");
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
    calib_enc, test_encoder_calibration, Encoder calibration test);

#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
