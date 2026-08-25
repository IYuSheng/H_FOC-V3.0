#include "main.h"

#include "foc_runtime.h"
#include "usart.h"

Motor_Control_t motor_ctrl = {
    .fault_type = MOTOR_FAULT_NONE,
    .temp_max = OVER_TEMPERATURE_THRESH,
    .vbus_max = VOLTAGE_LIMIT,
    .vbus_min = UNDER_VOLTAGE_THRESH,
    .current_max = CURRENT_LIMIT,
    .angle_max = 1.221730476f,
    .angle_min = -2.967059728f
};

encoder_status_t enc_status;

/**
 * @brief 应用程序入口，初始化后保持 IDLE 并处理串口命令
 * @return 不返回
 */
int main(void)
{
    FOC_Init();

    enc_status = encoder_init();
    foc_start_init();
    foc_start();

    if (enc_status == ENCODER_STATUS_OK) {
        foc_runtime_set_ready();
        debug_log("<READY:IDLE>");
    }
    else {
        motor_ctrl.fault_type = FAULT_ENCODER;
        debug_log("<ERR:encoder; motor not ready>");
    }

    for (;;) {
        bsp_uart_process_rx();
        foc_encoder_calibration_process();
        foc_control_background_process();

        if (foc_task.task_update_vbus) {
            foc_task.task_update_vbus = 0;
            get_foc_bus_voltage();
        }
        if (foc_task.task_sys_common) {
            foc_task.task_sys_common = 0;
            foc_debug();
        }
    }
}
