#include "mc_position.h"

#include <math.h>

#include "board_config.h"
#include "mc_input_shaper.h"
#include "mc_math.h"
#include "mc_runtime.h"
#include "mc_s_curve_planner.h"

#define MC_POSITION_TARGET_CHANGE_RAD 0.00174532925f
#define MC_POSITION_INTEGRAL_ENABLE_PROGRESS 0.90f
#define MC_POSITION_INTEGRAL_DECAY           0.99f

static SCurve_Planner_t position_planner;
static pi_t position_pid;
static pi_t cascade_position_pid;
static input_shaper_t position_shaper;
static input_shaper_t velocity_shaper;
static input_shaper_t acceleration_shaper;
static float position_last_target;
static uint16_t shaper_flush_ticks;
static uint8_t position_plan_valid;
static uint8_t shaper_bypass;
static uint8_t position_integral_always;

/**
 * @brief 设置位置环电流单环是否始终启用积分
 * @param enable 非零表示不再按运动进度门控积分
 * @note 辨识/扫频返回位置时使用；正常位置运动仍按 90% 进度门控，
 *       避免长行程阶段积分饱和。
 */
void mc_position_control_set_integral_always(uint8_t enable)
{
    position_integral_always = (enable != 0U) ? 1U : 0U;
}

/**
 * @brief 把位置、速度、加速度整形器复位到同一初始状态
 * @param position_rad 初始位置（rad）
 */
static void mc_position_shaper_reset(float position_rad)
{
    input_shaper_reset(&position_shaper, position_rad);
    input_shaper_reset(&velocity_shaper, 0.0f);
    input_shaper_reset(&acceleration_shaper, 0.0f);
    shaper_flush_ticks = 0U;
}

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
 * @brief 更新 S 曲线位置规划并执行输入整形
 * @param target_position_rad 目标机械位置（rad）
 * @return 当前规划位置（rad）
 */
static float mc_position_control_update_plan(float target_position_rad)
{
    float planned_position;
    float planned_velocity;
    float planned_acceleration;
    uint8_t plan_done;

    if ((position_plan_valid == 0U) ||
        (fabsf(target_position_rad - position_last_target) >
         MC_POSITION_TARGET_CHANGE_RAD)) {
        s_curve_set_target(&position_planner, mc_ctrl.shaped_t_p,
                           target_position_rad);
        position_last_target = target_position_rad;
        position_plan_valid = 1U;
        mc_position_shaper_reset(mc_ctrl.shaped_t_p);
    }

    plan_done = s_curve_is_done(&position_planner);
    if (plan_done != 0U) {
        planned_position = target_position_rad;
        planned_velocity = 0.0f;
        planned_acceleration = 0.0f;
    }
    else {
        planned_position = s_curve_update(&position_planner);
        planned_velocity = position_planner.current_vel;
        planned_acceleration = position_planner.current_acc;
    }

    /* 默认直通；辨识出谐振频率后按 EI 整形器滞后输出规划轨迹。 */
    if ((shaper_bypass == 0U) && input_shaper_is_enabled(&position_shaper)) {
        if (plan_done == 0U) {
            shaper_flush_ticks = input_shaper_get_max_delay(&position_shaper);
        }

        if ((plan_done == 0U) || (shaper_flush_ticks > 0U)) {
            planned_position =
                input_shaper_process(&position_shaper, planned_position);
            planned_velocity =
                input_shaper_process(&velocity_shaper, planned_velocity);
            planned_acceleration = input_shaper_process(&acceleration_shaper,
                                                        planned_acceleration);
            if ((plan_done != 0U) && (shaper_flush_ticks > 0U)) {
                shaper_flush_ticks--;
            }
        }
        else {
            /* 延迟线已经排空，直接给定目标值，避免浮点残差。 */
            planned_velocity = 0.0f;
            planned_acceleration = 0.0f;
            mc_ctrl.shaped_t_p = planned_position;
            mc_ctrl.shaped_t_v = 0.0f;
            mc_ctrl.shaped_t_a = 0.0f;
            return planned_position;
        }
    }

    mc_ctrl.shaped_t_p = planned_position;
    mc_ctrl.shaped_t_v = planned_velocity;
    mc_ctrl.shaped_t_a = planned_acceleration;
    return planned_position;
}

/**
 * @brief 按 FLASH 中的参数重新配置三个位置的输入整形器
 * @return 1 表示配置成功或已关闭整形，0 表示参数无法在当前采样率实现
 * @note 位置环周期为 POSITION_LOOP_DT，改变位置环频率后必须重新配置。
 */
uint8_t mc_position_control_apply_shaper_config(void)
{
    const mc_shaper_config_t *config = mc_runtime_get_shaper_config();
    const input_shaper_type_t type = (input_shaper_type_t)config->type;
    uint8_t result = 1U;

    if (input_shaper_config(&position_shaper, type, config->frequency_hz,
                            config->damping_ratio, POSITION_LOOP_DT) != 0) {
        result = 0U;
    }
    if (input_shaper_config(&velocity_shaper, type, config->frequency_hz,
                            config->damping_ratio, POSITION_LOOP_DT) != 0) {
        result = 0U;
    }
    if (input_shaper_config(&acceleration_shaper, type, config->frequency_hz,
                            config->damping_ratio, POSITION_LOOP_DT) != 0) {
        result = 0U;
    }

    if (result == 0U) {
        input_shaper_disable(&position_shaper);
        input_shaper_disable(&velocity_shaper);
        input_shaper_disable(&acceleration_shaper);
    }
    shaper_flush_ticks = 0U;
    return result;
}

/**
 * @brief 设置位置环是否旁路输入整形
 * @param bypass 非零表示旁路，用于辨识/扫频返回位置等内部运动
 */
void mc_position_control_set_shaper_bypass(uint8_t bypass)
{
    shaper_bypass = (bypass != 0U) ? 1U : 0U;
    shaper_flush_ticks = 0U;
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
    (void)mc_position_control_apply_shaper_config();
    mc_position_shaper_reset(initial_position_rad);
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

    /* 接近目标后才允许积分累积，避免长行程阶段积分饱和；
     * 辨识/扫频返回位置时强制启用积分以消除摩擦死区。 */
    if ((position_integral_always != 0U) ||
        (progress >= MC_POSITION_INTEGRAL_ENABLE_PROGRESS)) {
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
