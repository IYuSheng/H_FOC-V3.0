#define LOG_TAG "user_main"
#include "user_main.h"

#include "board_config.h"
#include "cpu.h"
#include "mc_init.h"
#include "mc_runtime.h"
#include "mc_shell_port.h"
#include "mc_sys.h"
#include "version.h"
#include "logger.h"
#include "usart.h"

/**
 * @brief 全局电机保护状态
 */
Motor_Control_t motor_ctrl = {
    .fault_type = MOTOR_FAULT_NONE,
    .temp_max = OVER_TEMPERATURE_THRESH,
    .vbus_max = VOLTAGE_LIMIT,
    .vbus_min = UNDER_VOLTAGE_THRESH,
    .current_max = CURRENT_LIMIT,
    .angle_max = MOTOR_ANGLE_MAX,
    .angle_min = MOTOR_ANGLE_MIN,
};

/**
 * @brief 初始化电机应用并进入主循环
 */
void user_main(void)
{
    encoder_status_t encoder_status;

    syslog_init();
    print_version_info();
    shell_init();
    encoder_status = encoder_init();
    if (encoder_status == ENCODER_STATUS_OK) {
        (void)encoder_read_electrical_angle();
        (void)encoder_read_mechanical_angle();
    }
    mc_start_init();
    cpu_usage_init();

    if (encoder_status == ENCODER_STATUS_OK) {
        mc_runtime_set_ready();
    }
    else {
        motor_ctrl.fault_type = FAULT_ENCODER;
        log_e("encoder error; motor not ready");
    }
    for (;;) {
        shell_task();
        mc_encoder_calibration_process();
        mc_control_background_process();
        cpu_usage_calculate();

        if (mc_task.task_update_vbus) {
            mc_task.task_update_vbus = 0;
            get_mc_bus_voltage();
        }
    }
}
