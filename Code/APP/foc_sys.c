#include "foc_sys.h"

#include "foc_control.h"
#include "foc_encoder_calibration.h"

FOC_TASK foc_task;

static volatile uint32_t system_tick = 0;
static volatile uint32_t task_vbus_counter = 0;
static volatile uint32_t task_three_voltage_counter = 0;

static const uint32_t task_vbus_period =
    SYSTICK_FREQUENCY_HZ / TASK_VBUS_UPDATE_FREQUENCY_HZ;
static const uint32_t task_three_voltage_period =
    SYSTICK_FREQUENCY_HZ / TASK_THREE_VOLTAGE_UPDATE_FREQUENCY_HZ;

// 定时状态机变量更新
void TIM3_IRQHandler(void)
{
    if (LL_TIM_IsActiveFlag_UPDATE(TIM3)) {
        LL_TIM_ClearFlag_UPDATE(TIM3);

        system_tick++;
        foc_task.task_sys_common = 1;

        if (++task_vbus_counter >= task_vbus_period) {
            task_vbus_counter = 0;
            foc_task.task_update_vbus = 1;
        }

        if (++task_three_voltage_counter >= task_three_voltage_period) {
            task_three_voltage_counter = 0;
            foc_task.task_update_three_phase_voltage = 1;
        }
    }
}

// 编码器采样中断
void TIM1_BRK_TIM15_IRQHandler(void)
{
    if (LL_TIM_IsActiveFlag_CC1(TIM15)) {
        LL_TIM_ClearFlag_CC1(TIM15);
        // encoder_spi_debug();
#if FOC_ENCODER_CALIBRATION_ENABLE && !FOC_ENCODER_CALIBRATION_USE_CONST_TABLE
        foc_encoder_calibration_set_raw(encoder_read_raw_angle());
#else
        encoder_read_electrical_angle();
        encoder_read_mechanical_angle();
#endif
#if 0
    encoder_get_mechanical_speed();
#endif
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
        foc_control();
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {
    }
}
