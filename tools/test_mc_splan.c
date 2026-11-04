#include <math.h>
#include <stdio.h>

#include "mc_s_curve_planner.h"

#define TEST_PI                       3.14159265358979f
#define TEST_DT_S                     0.001f
#define TEST_MAX_VELOCITY_RAD_S       62.83185307f
#define TEST_MAX_ACCELERATION_RAD_S2  174.5329252f
#define TEST_MAX_JERK_RAD_S3          436.3323129f
#define TEST_MAX_TICKS                100000U
#define TEST_POSITION_TOLERANCE_RAD   1.0e-5f
#define TEST_MONOTONIC_TOLERANCE_RAD  1.0e-4f

/**
 * @brief 验证指定目标位置的 S 曲线约束和到位精度
 * @param target_deg 目标机械角度（deg）
 * @return 0 表示通过，1 表示失败
 */
static int test_splan_move(float target_deg)
{
    SCurve_Planner_t planner;
    const float target_rad = target_deg * (TEST_PI / 180.0f);
    float previous_position = 0.0f;
    float previous_acceleration = 0.0f;
    float maximum_velocity = 0.0f;
    float maximum_acceleration = 0.0f;
    float maximum_jerk = 0.0f;
    float maximum_reverse_step = 0.0f;
    uint32_t ticks = 0U;
    int failed = 0;

    s_curve_planner_init(&planner, TEST_MAX_VELOCITY_RAD_S,
                         TEST_MAX_ACCELERATION_RAD_S2,
                         TEST_MAX_JERK_RAD_S3, TEST_DT_S);
    if (s_curve_set_target(&planner, 0.0f, target_rad) != 0U) {
        return 1;
    }

    while ((s_curve_is_done(&planner) == 0U) && (ticks < TEST_MAX_TICKS)) {
        const float position = s_curve_update(&planner);
        const float acceleration_delta =
            fabsf(planner.current_acc - previous_acceleration);
        const float reverse_step = (target_rad > 0.0f)
            ? (previous_position - position)
            : (position - previous_position);

        maximum_velocity = fmaxf(maximum_velocity, fabsf(planner.current_vel));
        maximum_acceleration =
            fmaxf(maximum_acceleration, fabsf(planner.current_acc));
        maximum_jerk =
            fmaxf(maximum_jerk, acceleration_delta / TEST_DT_S);
        maximum_reverse_step =
            fmaxf(maximum_reverse_step, reverse_step);
        previous_position = position;
        previous_acceleration = planner.current_acc;
        ticks++;
    }

    if ((s_curve_is_done(&planner) == 0U) ||
        (fabsf(planner.current_pos - target_rad) >
         TEST_POSITION_TOLERANCE_RAD)) {
        failed = 1;
    }
    if (maximum_velocity > (TEST_MAX_VELOCITY_RAD_S * 1.001f)) {
        failed = 1;
    }
    if (maximum_acceleration > (TEST_MAX_ACCELERATION_RAD_S2 * 1.001f)) {
        failed = 1;
    }
    if (maximum_jerk > (TEST_MAX_JERK_RAD_S3 * 1.01f)) {
        failed = 1;
    }
    if (maximum_reverse_step > TEST_MONOTONIC_TOLERANCE_RAD) {
        failed = 1;
    }

    printf("splan %+.1fdeg: time=%.3fs v=%.3f a=%.3f j=%.3f rev=%.3g %s\n",
           target_deg, (float)ticks * TEST_DT_S, maximum_velocity,
           maximum_acceleration, maximum_jerk, maximum_reverse_step,
           (failed == 0) ? "PASS" : "FAIL");
    return failed;
}

int main(void)
{
    int failed = 0;

    failed |= test_splan_move(100.0f);
    failed |= test_splan_move(-100.0f);
    failed |= test_splan_move(4300.0f);
    failed |= test_splan_move(-4300.0f);
    failed |= test_splan_move(10000.0f);
    return failed;
}
