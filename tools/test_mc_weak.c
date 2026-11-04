/**
 * @file test_mc_weak.c
 * @brief 弱磁控制主机侧单元测试。
 */

#include <assert.h>
#include <math.h>
#include <stdint.h>

#include "mc_weak.h"

/**
 * @brief 在当前弱磁输出下执行一次 d/q 电流限幅
 * @return 限幅后的 d 轴电流（A）
 */
static float weak_test_d_target(void)
{
    float d_current = 0.0f;
    float q_current = 4.0f;

    mc_weak_apply_current_limit(&d_current, &q_current);
    assert((d_current <= 0.0f) && (d_current >= MC_WEAK_MAX_D_CURRENT_A));
    assert((q_current <= 4.0f) && (q_current >= 0.0f));
    return d_current;
}

/**
 * @brief 运行一个完整弱磁降采样周期
 * @param speed 机械速度（rad/s）
 * @param voltage_q q 轴电压（V）
 * @param d_measured_offset d 轴电流跟踪误差（A）
 */
static void weak_test_update_cycle(float speed, float voltage_q,
                                   float d_measured_offset)
{
    for (uint32_t index = 0U; index < MC_WEAK_LOOP_DIVIDER; index++) {
        const float d_target = weak_test_d_target();

        mc_weak_update(true, 0.0f, voltage_q, 24.0f, 8.0f, speed, d_target,
                       d_target + d_measured_offset);
    }
}

/**
 * @brief 执行弱磁控制全部测试用例
 * @return 0 表示全部通过
 */
int main(void)
{
    float before_voltage_drop;
    float previous;

    mc_weak_init(1.0f / 30000.0f, 4.0f);
    assert(weak_test_d_target() == 0.0f);
    assert(MC_WEAK_LOOP_DIVIDER == 3U);

    for (uint32_t index = 1U; index < MC_WEAK_LOOP_DIVIDER; index++) {
        mc_weak_update(true, 0.0f, 24.0f * 0.97f, 24.0f, 8.0f, 170.0f, 0.0f,
                       0.0f);
        assert(weak_test_d_target() == 0.0f);
    }
    mc_weak_update(true, 0.0f, 24.0f * 0.97f, 24.0f, 8.0f, 170.0f, 0.0f, 0.0f);
    assert(weak_test_d_target() < 0.0f);

    before_voltage_drop = weak_test_d_target();
    weak_test_update_cycle(170.0f, 24.0f * 0.89f, 0.0f);
    assert(weak_test_d_target() > before_voltage_drop);
    assert(weak_test_d_target() < 0.0f);
    mc_weak_reset();

    for (uint32_t index = 0U; index < 1000U; index++) {
        weak_test_update_cycle(170.0f, 24.0f * 0.97f, 0.0f);
    }
    assert(fabsf(weak_test_d_target() - MC_WEAK_MAX_D_CURRENT_A) < 0.0001f);

    weak_test_update_cycle(170.0f, 24.0f * 0.97f, 0.0f);
    assert(fabsf(weak_test_d_target() - MC_WEAK_MAX_D_CURRENT_A) < 0.0001f);

    weak_test_update_cycle(170.0f, 24.0f * 0.97f,
                           MC_WEAK_D_TRACKING_MARGIN_A - 0.001f);
    assert(fabsf(weak_test_d_target() - MC_WEAK_MAX_D_CURRENT_A) < 0.0001f);

    for (uint32_t index = 1U; index < MC_WEAK_D_TRACKING_DEBOUNCE_COUNT;
         index++) {
        weak_test_update_cycle(170.0f, 24.0f * 0.97f,
                               MC_WEAK_D_TRACKING_MARGIN_A + 0.01f);
        assert(fabsf(weak_test_d_target() - MC_WEAK_MAX_D_CURRENT_A) < 0.0001f);
    }

    previous = weak_test_d_target();
    for (uint32_t index = 0U; index < 400U; index++) {
        float current;

        weak_test_update_cycle(170.0f, 24.0f * 0.97f,
                               MC_WEAK_D_TRACKING_MARGIN_A + 0.01f);
        current = weak_test_d_target();
        assert(current >= previous);
        previous = current;
    }
    assert(weak_test_d_target() == 0.0f);

    weak_test_update_cycle(170.0f, 24.0f * 0.97f, 0.0f);
    mc_weak_update(true, 0.0f, 24.0f, 24.0f, 8.0f, 59.0f, weak_test_d_target(),
                   weak_test_d_target());
    assert(weak_test_d_target() == 0.0f);

    weak_test_update_cycle(-170.0f, -24.0f * 0.97f, 0.0f);
    assert(weak_test_d_target() < 0.0f);
    for (uint32_t index = 1U; index < MC_WEAK_D_TRACKING_DEBOUNCE_COUNT;
         index++) {
        weak_test_update_cycle(-170.0f, -24.0f * 0.97f,
                               MC_WEAK_D_TRACKING_MARGIN_A + 0.01f);
    }
    previous = weak_test_d_target();
    weak_test_update_cycle(-170.0f, -24.0f * 0.97f,
                           MC_WEAK_D_TRACKING_MARGIN_A + 0.01f);
    assert(weak_test_d_target() > previous);

    return 0;
}
