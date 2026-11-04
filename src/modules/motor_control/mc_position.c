#include "mc_position.h"

#include <math.h>

#include "board_config.h"
#include "mc_math.h"
#include "mc_runtime.h"
#include "mc_s_curve_planner.h"

#define MC_POSITION_TARGET_CHANGE_RAD 0.00174532925f
#define MC_POSITION_INTEGRAL_ENABLE_PROGRESS 0.90f
#define MC_POSITION_INTEGRAL_DECAY           0.99f

static SCurve_Planner_t position_planner;
static pi_t position_pid;
static pi_t cascade_position_pid;
static float position_last_target;
static uint8_t position_plan_valid;

/**
 * @brief 获取当前 S 曲线位置运动进度
 * @return 运动进度，范围 0~1
 */
static float mc_position_control_get_progress(void)
{
    const float total_distance =
        position_planner.pos_end - position_planner.pos_init;
    float progress;

    if ((position_plan_valid == 0U) ||
        (s_curve_is_done(&position_planner) != 0U) ||
        (fabsf(total_distance) < EPS_F)) {
        return 1.0f;
    }

    progress = (encoder_data.mechanical_position -
                position_planner.pos_init) /
        total_distance;
    if (progress < 0.0f) {
        progress = 0.0f;
    }
    else if (progress > 1.0f) {
        progress = 1.0f;
    }
    return progress;
}

/**
 * @brief 更新 S 曲线位置规划
 * @param target_position_rad 目标机械位置（rad）
 * @return 当前规划位置（rad）
 */
static float mc_position_control_update_plan(float target_position_rad)
{
    float planned_position;

    if ((position_plan_valid == 0U) ||
        (fabsf(target_position_rad - position_last_target) >
         MC_POSITION_TARGET_CHANGE_RAD)) {
        s_curve_set_target(&position_planner, mc_ctrl.shaped_t_p,
                           target_position_rad);
        position_last_target = target_position_rad;
        position_plan_valid = 1U;
    }

    if (s_curve_is_done(&position_planner) != 0U) {
        planned_position = target_position_rad;
        mc_ctrl.shaped_t_v = 0.0f;
        mc_ctrl.shaped_t_a = 0.0f;
    }
    else {
        planned_position = s_curve_update(&position_planner);
        mc_ctrl.shaped_t_v = position_planner.current_vel;
        mc_ctrl.shaped_t_a = position_planner.current_acc;
    }

    return planned_position;
}

/**
 * @brief 使用默认限幅初始化位置环
 * @param initial_position_rad 初始机械位置（rad）
 */
void mc_position_control_init(float initial_position_rad)
{
    mc_position_control_reset(initial_position_rad, MC_S_CURVE_MAX_VELOCITY,
                              MC_S_CURVE_MAX_ACCELERATION, MC_S_CURVE_MAX_JERK);
}

/**
 * @brief 使用指定轨迹限制复位位置环
 * @param initial_position_rad 初始机械位置（rad）
 * @param maximum_velocity_rad_s 最大规划速度（rad/s）
 * @param maximum_acceleration_rad_s2 最大规划加速度（rad/s^2）
 * @param maximum_jerk_rad_s3 最大规划加加速度（rad/s^3）
 */
void mc_position_control_reset(float initial_position_rad,
                               float maximum_velocity_rad_s,
                               float maximum_acceleration_rad_s2,
                               float maximum_jerk_rad_s3)
{
    const mc_tuning_config_t *tuning = mc_runtime_get_tuning_config();

    s_curve_planner_init(&position_planner, maximum_velocity_rad_s,
                         maximum_acceleration_rad_s2, maximum_jerk_rad_s3,
                         POSITION_LOOP_DT);
    position_last_target = initial_position_rad;
    position_plan_valid = 0U;
    mc_ctrl.shaped_t_p = initial_position_rad;
    mc_ctrl.shaped_t_v = 0.0f;
    mc_ctrl.shaped_t_a = 0.0f;
    position_pid.kp = tuning->position_kp;
    position_pid.ki = tuning->position_ki;
    position_pid.kd = tuning->position_kd;
    position_pid.integral_limit = POSITION_SPEED_LIMIT_RAD_S;
    mc_pid_reset(&position_pid);
    cascade_position_pid.kp = tuning->cascade_kp;
    cascade_position_pid.ki = tuning->cascade_ki;
    cascade_position_pid.kd = tuning->cascade_kd;
    cascade_position_pid.integral_limit = POSITION_SPEED_LIMIT_RAD_S;
    mc_pid_reset(&cascade_position_pid);
}

/**
 * @brief 执行一次位置到速度级联控制并更新规划轨迹
 * @param target_position_rad 目标机械位置（rad）
 */
void mc_position_control_run(float target_position_rad)
{
    const float planned_position =
        mc_position_control_update_plan(target_position_rad);
    mc_ctrl.target_speed = mc_position_control_track(
        planned_position, mc_ctrl.shaped_t_v, mc_ctrl.shaped_t_a,
        encoder_data.mechanical_position);
}

/**
 * @brief 执行位置环直出电流控制并更新 S 曲线轨迹
 * @param target_position_rad 目标机械位置（rad）
 * @return q 轴电流给定（A）
 */
float mc_position_control_run_current(float target_position_rad)
{
    const float planned_position =
        mc_position_control_update_plan(target_position_rad);
    const float position_error =
        planned_position - encoder_data.mechanical_position;
    const float speed_error =
        mc_ctrl.shaped_t_v - encoder_data.mechanical_speed;
    const float proportional = position_pid.kp * position_error;
    const float derivative = position_pid.kd * speed_error;
    const float progress = mc_position_control_get_progress();
    const float previous_integral = position_pid.integral;
    float integral = previous_integral;
    float output_unlimited;
    float current_output;

    mc_ctrl.target_speed = mc_ctrl.shaped_t_v;
    position_pid.integral_limit = POSITION_CURRENT_LIMIT_A;

    /* 接近目标后才允许积分累积，避免长行程阶段积分饱和。 */
    if (progress >= MC_POSITION_INTEGRAL_ENABLE_PROGRESS) {
        integral += position_pid.ki * POSITION_LOOP_DT * position_error;
    }
    else {
        integral *= MC_POSITION_INTEGRAL_DECAY;
    }
    if (integral > POSITION_CURRENT_LIMIT_A) {
        integral = POSITION_CURRENT_LIMIT_A;
    }
    else if (integral < -POSITION_CURRENT_LIMIT_A) {
        integral = -POSITION_CURRENT_LIMIT_A;
    }

    output_unlimited = proportional + integral + derivative;
    if (output_unlimited > POSITION_CURRENT_LIMIT_A) {
        current_output = POSITION_CURRENT_LIMIT_A;
    }
    else if (output_unlimited < -POSITION_CURRENT_LIMIT_A) {
        current_output = -POSITION_CURRENT_LIMIT_A;
    }
    else {
        current_output = output_unlimited;
    }

    /* 输出饱和且误差继续推向饱和方向时，不继续累积积分。 */
    if (((output_unlimited > POSITION_CURRENT_LIMIT_A) &&
         (position_error > 0.0f)) ||
        ((output_unlimited < -POSITION_CURRENT_LIMIT_A) &&
         (position_error < 0.0f))) {
        integral = previous_integral;
    }

    position_pid.integral = integral;
    position_pid.error = position_error;
    position_pid.last_error = position_error;
    position_pid.output = current_output;
    return current_output;
}

/**
 * @brief 跟踪规划轨迹并生成速度给定
 * @param planned_position_rad 规划位置（rad）
 * @param planned_velocity_rad_s 规划速度（rad/s）
 * @param planned_acceleration_rad_s2 规划加速度（rad/s^2）
 * @param actual_position_rad 实际机械位置（rad）
 * @return 速度环给定（rad/s）
 */
float mc_position_control_track(float planned_position_rad,
                                float planned_velocity_rad_s,
                                float planned_acceleration_rad_s2,
                                float actual_position_rad)
{
    const float position_error = planned_position_rad - actual_position_rad;
    float target_speed;

    mc_ctrl.shaped_t_p = planned_position_rad;
    mc_ctrl.shaped_t_v = planned_velocity_rad_s;
    mc_ctrl.shaped_t_a = planned_acceleration_rad_s2;
    cascade_position_pid.integral_limit = POSITION_SPEED_LIMIT_RAD_S;
    target_speed = planned_velocity_rad_s +
        mc_pid_update(&cascade_position_pid, position_error, POSITION_LOOP_DT,
                      POSITION_SPEED_LIMIT_RAD_S);
    if (target_speed > POSITION_SPEED_LIMIT_RAD_S) {
        target_speed = POSITION_SPEED_LIMIT_RAD_S;
    }
    else if (target_speed < -POSITION_SPEED_LIMIT_RAD_S) {
        target_speed = -POSITION_SPEED_LIMIT_RAD_S;
    }
    return target_speed;
}
