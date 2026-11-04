#include "mc_config.h"

#include "mc_math.h"

extern pi_t id_pid;
extern pi_t iq_pid;

/**
 * @brief 电流环PID控制器初始化
 * @param kp_d D轴比例系数
 * @param kp_q Q轴比例系数
 * @param ki 每个电流环周期使用的离散积分系数，等于连续域 Ki 乘 Ts
 *
 * @param ki_limit 积分限幅值
 */
void mc_current_pi_init(float kp_d, float kp_q, float ki, float ki_limit)
{
    // 初始化D轴电流环PI参数
    id_pid.kp = kp_d;
    id_pid.ki = ki;
    id_pid.integral_limit = ki_limit;
    id_pid.integral = 0.0f;

    // 初始化Q轴电流环PI参数
    iq_pid.kp = kp_q;
    iq_pid.ki = ki;
    iq_pid.integral_limit = ki_limit;
    iq_pid.integral = 0.0f;
}

