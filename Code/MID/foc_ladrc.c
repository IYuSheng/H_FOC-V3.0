#include "foc_ladrc.h"

#include <math.h>

#include "Config.h"
#include "foc_runtime.h"

/**
 * @brief 获取由 Ka=J/Kt 确定的机械环输入增益
 * @return 电流到机械角加速度的增益，单位 rad/s^2/A
 */
static float ladrc_get_b0(float fallback)
{
    const foc_motor_config_t *motor = foc_runtime_get_motor_config();
    float b0 = 1.0f / motor->ka;

    if ((!isfinite(b0)) || (b0 <= 0.0f)) {
        return fallback;
    }
    return b0;
}

static inline float ladrc_limit(float value, float min, float max)
{
    if (value > max)
        return max;
    if (value < min)
        return min;
    return value;
}

void position_ladrc_reset(PositionLadrc_t *controller, float position_feedback)
{
    if (controller == 0)
        return;

    controller->z1 = position_feedback;
    controller->z2 = 0.0f;
    controller->z3 = 0.0f;
    controller->output = 0.0f;
    controller->initialized = 1U;
}

float position_ladrc_update(PositionLadrc_t *controller, float target_position,
                            float target_velocity, float target_acceleration,
                            float position_feedback)
{
    if (controller == 0)
        return 0.0f;

    if (controller->initialized == 0U) {
        position_ladrc_reset(controller, position_feedback);
    }

    const float wo = POSITION_LESO_BANDWIDTH;
    const float wo2 = wo * wo;
    const float b0 = ladrc_get_b0(POSITION_LADRC_B0);
    const float observer_error = controller->z1 - position_feedback;
    const float z1_dot = controller->z2 - 3.0f * wo * observer_error;
    const float z2_dot =
        controller->z3 - 3.0f * wo2 * observer_error + b0 * controller->output;
    const float z3_dot = -(wo2 * wo) * observer_error;

    controller->z1 += POSITION_LOOP_DT * z1_dot;
    controller->z2 += POSITION_LOOP_DT * z2_dot;
    controller->z3 += POSITION_LOOP_DT * z3_dot;

    const float wc = POSITION_LADRC_CONTROL_BANDWIDTH;
    const float position_error = target_position - controller->z1;
    const float desired_acceleration = target_acceleration +
        (wc * wc) * position_error +
        2.0f * wc * (target_velocity - controller->z2);
    const float output_unsaturated =
        (desired_acceleration - controller->z3) / b0;

    controller->output = ladrc_limit(output_unsaturated, POSITION_LADRC_OUT_MIN,
                                     POSITION_LADRC_OUT_MAX);
    return controller->output;
}

void speed_ladrc_reset(SpeedLadrc_t *controller, float speed_feedback)
{
    if (controller == 0)
        return;

    controller->z1 = speed_feedback;
    controller->z2 = 0.0f;
    controller->output = 0.0f;
    controller->initialized = 1U;
}

float speed_ladrc_update(SpeedLadrc_t *controller, float target_speed,
                         float target_acceleration, float speed_feedback)
{
    if (controller == 0)
        return 0.0f;

    if (controller->initialized == 0U) {
        speed_ladrc_reset(controller, speed_feedback);
    }

    const float wo = SPEED_LESO_BANDWIDTH;
    const float b0 = ladrc_get_b0(SPEED_LADRC_B0);
    const float observer_error = controller->z1 - speed_feedback;
    const float z1_dot =
        controller->z2 - 2.0f * wo * observer_error + b0 * controller->output;
    const float z2_dot = -(wo * wo) * observer_error;

    controller->z1 += SPEED_LOOP_DT * z1_dot;
    controller->z2 += SPEED_LOOP_DT * z2_dot;

    const float desired_acceleration = target_acceleration +
        SPEED_LADRC_CONTROL_BANDWIDTH * (target_speed - controller->z1);
    const float output_unsaturated =
        (desired_acceleration - controller->z2) / b0;

    controller->output = ladrc_limit(output_unsaturated, SPEED_LADRC_OUT_MIN,
                                     SPEED_LADRC_OUT_MAX);
    return controller->output;
}
