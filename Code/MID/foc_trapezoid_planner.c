#include "foc_trapezoid_planner.h"

#include <math.h>

/** 极小值保护，避免周期/速度/加速度为 0 导致除零或无意义规划 */
#define TRAPEZOID_PLANNER_MIN_VALUE  (1.0e-6f)

/**
 * @brief 计算绝对值
 * @param value 输入值
 * @return float 绝对值
 */
static float trapezoid_abs(float value)
{
    return (value >= 0.0f) ? value : -value;
}

/**
 * @brief 计算符号
 * @param value 输入值
 * @return float value >= 0 返回 +1，否则返回 -1
 */
static float trapezoid_sign(float value)
{
    return (value >= 0.0f) ? 1.0f : -1.0f;
}

/**
 * @brief 限幅函数
 * @param value     输入值
 * @param min_value 下限
 * @param max_value 上限
 * @return float 限幅后的值
 */
static float trapezoid_clamp(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

/**
 * @brief 初始化梯形加减速规划器
 * @param planner      规划器实例指针
 * @param max_velocity 最大速度限制，单位与位置单位一致/s
 * @param period       固定控制周期，单位 s
 *
 * @note 初始化后规划器为空闲状态。第一次使用前可以再调用
 *       trapezoid_planner_reset() 将输出位置对齐到当前位置。
 */
void trapezoid_planner_init(TrapezoidPlanner_t *planner,
                            float max_velocity,
                            float period)
{
    if (planner == 0)
    {
        return;
    }

    planner->max_velocity = trapezoid_abs(max_velocity);
    planner->command_max_velocity = planner->max_velocity;
    planner->period = period;
    planner->position_setpoint = 0.0f;
    planner->velocity_setpoint = 0.0f;
    planner->acceleration_setpoint = 0.0f;
    planner->target_position = 0.0f;
    planner->tolerance = 0.0000174532925f;
    planner->active = 0U;
}

/**
 * @brief 复位规划器运行状态
 * @param planner          规划器实例指针
 * @param current_position 当前实际位置
 *
 * @note 复位后位置给定等于当前位置，速度和加速度给定清零。
 */
void trapezoid_planner_reset(TrapezoidPlanner_t *planner,
                             float current_position)
{
    if (planner == 0)
    {
        return;
    }

    planner->position_setpoint = current_position;
    planner->velocity_setpoint = 0.0f;
    planner->acceleration_setpoint = 0.0f;
    planner->target_position = current_position;
    planner->command_max_velocity = planner->max_velocity;
    planner->active = 0U;
}

/**
 * @brief 设置最大速度限制
 * @param planner      规划器实例指针
 * @param max_velocity 最大速度限制，单位与位置单位一致/s
 */
void trapezoid_planner_set_max_velocity(TrapezoidPlanner_t *planner,
                                        float max_velocity)
{
    if (planner == 0)
    {
        return;
    }

    planner->max_velocity = trapezoid_abs(max_velocity);
    if (planner->command_max_velocity > planner->max_velocity)
    {
        planner->command_max_velocity = planner->max_velocity;
    }
}

/**
 * @brief 设置到位容差
 * @param planner   规划器实例指针
 * @param tolerance 到位容差
 */
void trapezoid_planner_set_tolerance(TrapezoidPlanner_t *planner,
                                     float tolerance)
{
    if (planner == 0)
    {
        return;
    }

    planner->tolerance = trapezoid_abs(tolerance);
}

/**
 * @brief 周期执行在线梯形加减速规划
 * @param planner          规划器实例指针
 * @param target_position  目标位置
 * @param current_position 当前实际位置反馈
 * @param command_max_velocity 当前指令最大速度，内部会被初始化设定的 max_velocity 限幅
 * @param acceleration     加速度限制，单位与位置单位一致/s^2
 * @return float           本周期规划位置给定
 *
 * @details 算法流程：
 * 1. 计算目标位置与当前规划位置的误差，确定运动方向。
 * 2. 根据当前规划速度计算刹车距离 stop_distance = v^2 / (2a)。
 * 3. 若距离目标还远，则按目标方向匀加速到本指令最大速度。
 * 4. 到达最大速度后保持匀速；若进入刹车距离，则按反向加速度匀减速。
 * 5. 用梯形积分 step = 0.5 * (v_old + v_new) * dt 得到本周期位置增量。
 * 6. 若本周期会越过目标点，则直接钳位到目标点并清零速度。
 *
 * @note 该函数是“在线型”规划器，target_position 可以每个周期更新。
 *       对 3D 打印机来说，位置单位可以是 mm；本电机角度控制使用 rad。
 *       current_position 只用于空指针保护返回和外部 reset 对齐，规划推进使用内部
 *       position_setpoint，避免实际位置滞后时把轨迹给定拖慢。
 */
float trapezoid_planner_update(TrapezoidPlanner_t *planner,
                               float target_position,
                               float current_position,
                               float command_max_velocity,
                               float acceleration)
{
    float error;
    float abs_error;
    float direction;
    float max_velocity;
    float accel;
    float dt;
    float velocity;
    float stop_distance;
    float command_accel;
    float next_velocity;
    float step;

    if (planner == 0)
    {
        return current_position;
    }

    /* 参数保护：速度、加速度或周期无效时保持当前位置，避免输出异常 */
    accel = trapezoid_abs(acceleration);
    dt = planner->period;
    max_velocity = trapezoid_abs(command_max_velocity);
    if (max_velocity > planner->max_velocity)
    {
        max_velocity = planner->max_velocity;
    }
    planner->command_max_velocity = max_velocity;
    planner->target_position = target_position;

    if ((accel <= TRAPEZOID_PLANNER_MIN_VALUE) ||
        (dt <= TRAPEZOID_PLANNER_MIN_VALUE) ||
        (max_velocity <= TRAPEZOID_PLANNER_MIN_VALUE))
    {
        planner->velocity_setpoint = 0.0f;
        planner->acceleration_setpoint = 0.0f;
        planner->active = 0U;
        return planner->position_setpoint;
    }

    /* 到位判定：位置误差足够小，并且下一周期可以把速度降到 0 */
    error = target_position - planner->position_setpoint;
    abs_error = trapezoid_abs(error);
    velocity = planner->velocity_setpoint;

    if ((abs_error <= planner->tolerance) &&
        (trapezoid_abs(velocity) <= (accel * dt)))
    {
        planner->position_setpoint = target_position;
        planner->velocity_setpoint = 0.0f;
        planner->acceleration_setpoint = 0.0f;
        planner->active = 0U;
        return planner->position_setpoint;
    }

    direction = trapezoid_sign(error);
    stop_distance = (velocity * velocity) / (2.0f * accel);

    /*
     * 加速度决策：
     * - 如果当前速度方向与目标方向相反，先朝目标方向加速以消除反向速度。
     * - 如果剩余距离已经小于刹车距离，开始减速。
     * - 否则继续朝目标方向加速，直到速度被 max_velocity 限幅。
     */
    if ((velocity * direction) < 0.0f)
    {
        command_accel = direction * accel;
    }
    else if (abs_error <= (stop_distance + 0.5f * accel * dt * dt))
    {
        command_accel = -direction * accel;
    }
    else
    {
        command_accel = direction * accel;
    }

    next_velocity = velocity + command_accel * dt;
    next_velocity = trapezoid_clamp(next_velocity, -max_velocity, max_velocity);

    /* 梯形积分：用本周期初末速度平均值计算位置增量 */
    step = 0.5f * (velocity + next_velocity) * dt;
    if ((step * error > 0.0f) && (trapezoid_abs(step) > abs_error))
    {
        /* 防止最后一个周期越过目标点 */
        step = error;
        next_velocity = 0.0f;
        command_accel = 0.0f;
    }

    planner->position_setpoint += step;
    planner->velocity_setpoint = next_velocity;
    planner->acceleration_setpoint = command_accel;
    planner->active = 1U;

    return planner->position_setpoint;
}
