#include "mc_speed.h"

#include "board_config.h"
#include "mc_math.h"
#include "mc_runtime.h"

static pi_t speed_pid;

/**
 * @brief 初始化速度环 PID
 * @param initial_speed_rad_s 初始机械速度（rad/s）
 */
void mc_speed_control_init(float initial_speed_rad_s)
{
    mc_speed_control_reset(initial_speed_rad_s);
}

/**
 * @brief 从 FLASH 参数加载并复位速度环 PID
 * @param initial_speed_rad_s 初始机械速度（rad/s）
 */
void mc_speed_control_reset(float initial_speed_rad_s)
{
    const mc_tuning_config_t *tuning = mc_runtime_get_tuning_config();

    (void)initial_speed_rad_s;
    speed_pid.kp = tuning->speed_kp;
    speed_pid.ki = tuning->speed_ki;
    speed_pid.kd = tuning->speed_kd;
    speed_pid.integral_limit = SPEED_OUT_LIMIT;
    mc_pid_reset(&speed_pid);
}

/**
 * @brief 执行一次速度环控制
 * @param target_speed_rad_s 目标机械速度（rad/s）
 * @param target_acceleration_rad_s2 机械加速度前馈（rad/s^2）
 * @param actual_speed_rad_s 机械速度反馈（rad/s）
 * @return q 轴电流给定（A）
 */
float mc_speed_control_run(float target_speed_rad_s,
                           float target_acceleration_rad_s2,
                           float actual_speed_rad_s)
{
    const mc_motor_config_t *motor = mc_runtime_get_motor_config();
    const float feedforward = motor->ka * target_acceleration_rad_s2;
    float output = mc_pid_update(&speed_pid,
                                 target_speed_rad_s - actual_speed_rad_s,
                                 SPEED_LOOP_DT, SPEED_OUT_LIMIT);

    output += feedforward;
    if (output > SPEED_OUT_LIMIT) {
        output = SPEED_OUT_LIMIT;
    }
    else if (output < -SPEED_OUT_LIMIT) {
        output = -SPEED_OUT_LIMIT;
    }
    return output;
}
