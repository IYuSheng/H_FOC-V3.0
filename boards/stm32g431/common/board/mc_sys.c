#include "mc_sys.h"

#include "mc_calibration.h"
#include "mc_control.h"
#include "mc_current.h"

MC_TASK mc_task;

static volatile uint32_t task_vbus_counter = 0;

static const uint32_t task_vbus_period =
    SYSTICK_FREQUENCY_HZ / TASK_VBUS_UPDATE_FREQUENCY_HZ;

// 定时状态机变量更新
void TIM3_IRQHandler(void)
{
    if (LL_TIM_IsActiveFlag_UPDATE(TIM3)) {
        LL_TIM_ClearFlag_UPDATE(TIM3);

        if (++task_vbus_counter >= task_vbus_period) {
            task_vbus_counter = 0;
            mc_task.task_update_vbus = 1;
        }
    }
}

// 编码器采样中断
void TIM1_BRK_TIM15_IRQHandler(void)
{
    if (LL_TIM_IsActiveFlag_CC1(TIM15)) {
        LL_TIM_ClearFlag_CC1(TIM15);
        if (mc_encoder_calibration_is_running() != 0U) {
            mc_encoder_calibration_set_raw(encoder_read_raw_angle());
        }
        else {
            (void)encoder_read_electrical_angle();
        }
    }
}

// 电流读取及电流环
void ADC1_2_IRQHandler(void)
{
    if (LL_ADC_IsActiveFlag_JEOS(ADC1)) {
        LL_ADC_ClearFlag_JEOS(ADC1);
        LL_ADC_ClearFlag_JEOC(ADC1);
        adc1_process_injected_current_sample();
    }
}

// 外环中断处理
void TIM4_IRQHandler(void)
{
    if (LL_TIM_IsActiveFlag_UPDATE(TIM4)) {
        LL_TIM_ClearFlag_UPDATE(TIM4);
        if (mc_encoder_calibration_is_running() == 0U) {
            (void)encoder_read_mechanical_angle();
        }
        mc_current_control_update_feedforward(encoder_data.electrical_speed);
        mc_control();
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {
    }
}
