#include "identify_parameter.h"

#include "identify_inductance.h"
#include "identify_mechanical.h"
#include "identify_resistance.h"

/**
 * @brief 初始化电阻、电感和机械参数辨识模块
 * @return 0 表示初始化成功，负值表示失败
 */
int8_t identify_parameter_init(void)
{
    if (identify_resistance_init() != 0) {
        return -1;
    }
    if (identify_inductance_init() != 0) {
        return -1;
    }
    return identify_mechanical_init();
}

/**
 * @brief 查询电阻或电感辨识是否正在运行
 * @return true 表示电气参数辨识正在运行
 */
bool identify_parameter_is_electrical_active(void)
{
    const ident_r_state_t resistance_state = identify_resistance_get()->state;
    const ident_l_state_t inductance_state = identify_inductance_get()->state;

    return ((resistance_state != IDENT_R_IDLE) &&
            (resistance_state != IDENT_R_DONE) &&
            (resistance_state != IDENT_R_ERROR)) ||
        ((inductance_state != IDENT_L_IDLE) &&
         (inductance_state != IDENT_L_DONE) &&
         (inductance_state != IDENT_L_ERROR));
}

/**
 * @brief 查询任一参数辨识任务是否正在运行
 * @return true 表示存在活动辨识任务
 */
bool identify_parameter_is_active(void)
{
    return identify_parameter_is_electrical_active() ||
        identify_mechanical_is_active();
}

/**
 * @brief 在 5 kHz 控制节拍推进电气参数辨识状态机
 */
void identify_parameter_control_tick(void)
{
    const ident_r_state_t resistance_state = identify_resistance_get()->state;
    const ident_l_state_t inductance_state = identify_inductance_get()->state;

    if ((resistance_state != IDENT_R_IDLE) &&
        (resistance_state != IDENT_R_DONE) &&
        (resistance_state != IDENT_R_ERROR)) {
        identify_resistance_control_tick();
        return;
    }
    if ((inductance_state != IDENT_L_IDLE) &&
        (inductance_state != IDENT_L_DONE) &&
        (inductance_state != IDENT_L_ERROR)) {
        identify_inductance_control_tick();
    }
}

/**
 * @brief 在主循环完成需要 FLASH 写入的参数保存
 */
void identify_parameter_background_process(void)
{
    identify_resistance_background_process();
    identify_inductance_background_process();
}

/**
 * @brief 停止电气参数辨识并将功率级置为零电压
 */
void identify_parameter_stop(void)
{
    identify_resistance_stop();
    identify_inductance_stop();
}
