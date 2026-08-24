#include "foc_ladrc.h"

#include "Config.h"

static inline float position_ladrc_limit(float value, float min, float max)
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
    const float observer_error = controller->z1 - position_feedback;
    const float z1_dot = controller->z2 - 3.0f * wo * observer_error;
    const float z2_dot = controller->z3 - 3.0f * wo2 * observer_error +
        POSITION_LADRC_B0 * controller->output;
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
        (desired_acceleration - controller->z3) / POSITION_LADRC_B0;

    controller->output = position_ladrc_limit(
        output_unsaturated, POSITION_LADRC_OUT_MIN, POSITION_LADRC_OUT_MAX);
    return controller->output;
}
