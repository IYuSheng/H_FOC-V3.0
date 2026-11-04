#include "mc_math.h"
#include <math.h>
#include <stddef.h>

#include "cordic.h"

pi_t iq_pid MC_CCM_BSS;
pi_t id_pid MC_CCM_BSS;
SVPWM_t svpwm MC_CCM_BSS;
SPWM_t spwm MC_CCM_BSS;
mc_control_t mc_ctrl MC_CCM_BSS;
AlphaBetaTypeDef alpha_beta MC_CCM_BSS;
float mc_sine_table[MC_ANGLE_INDEX_COUNT] MC_CCM_BSS;

/**
 * @brief 初始化 Q10 正弦查表
 * @note 启动阶段使用硬件 CORDIC 生成，运行阶段可避免电流环等待 CORDIC。
 */
void mc_sine_table_init(void)
{
    float sin_value;
    float cos_value;

    for (uint16_t i = 0U; i < MC_ANGLE_INDEX_COUNT; i++) {
        CORDIC_SinCos_Rad((float)i * MC_ANGLE_INDEX_TO_RAD, &sin_value,
                          &cos_value);
        mc_sine_table[i] = sin_value;
    }
}

/**
 * @brief 电角度归一化（映射到0~2π范围）
 * @param angle 输入电角度（rad，范围无限制）
 * @return 归一化后电角度（rad，0~2π）
 */
float angle_normalize(float angle)
{
    while (angle < 0.0f)
        angle += _2PI;
    while (angle >= _2PI)
        angle -= _2PI;
    return angle;
}

/**
 * @brief 电角度归一化（映射到-π~π范围）
 * @param angle 输入电角度（rad，范围无限制）
 * @return 归一化后电角度（rad，-π~π）
 */
float angle_normalize_pi(float angle)
{
    while (angle < -_PI)
        angle += _2PI;
    while (angle >= _PI)
        angle -= _2PI;
    return angle;
}

static inline float clampf(float x, float min_v, float max_v)
{
    if (x > max_v)
        return max_v;
    if (x < min_v)
        return min_v;
    return x;
}

/**
 * @brief SVPWM通用扇区判断函数
 * @param u_alpha α轴目标电压（V）
 * @param u_beta  β轴目标电压（V）
 * @return 扇区编号（1~6，对应一电角周期的六个扇区）
 */
uint8_t svpwm_sector_calc(AlphaBetaTypeDef *alpha_beta)
{
    uint8_t sector, pos1, pos2, pos3 = 0;

    float X = alpha_beta->beta;
    float Y = (_SQRT3 * alpha_beta->alpha - alpha_beta->beta);
    float Z = (-_SQRT3 * alpha_beta->alpha - alpha_beta->beta);

    pos1 = (X > 1e-6f) ? 1 : 0;
    pos2 = (Y > 1e-6f) ? 1 : 0;
    pos3 = (Z > 1e-6f) ? 1 : 0;

    sector = pos1 * 1 + pos2 * 2 + pos3 * 4;

    switch (sector) {
        case 3:
            return 1;
        case 1:
            return 2;
        case 5:
            return 3;
        case 4:
            return 4;
        case 6:
            return 5;
        case 2:
            return 6;
        default:
            return -1;
    }
}

/**
 * @brief SVPWM基本矢量作用时间计算
 * @param sector 当前扇区（1~6）
 * @param u_alpha α轴目标电压（V）
 * @param u_beta  β轴目标电压（V）
 * @param vdc 母线电压（V）
 * @param T1 第一个基本矢量作用时间（s）
 * @param T2 第二个基本矢量作用时间（s）
 * @param T0 零矢量总作用时间（s）
 */
void svpwm_calc_times(AlphaBetaTypeDef *alpha_beta, SVPWM_t *svpwm, float vdc)
{
    // 1) 计算幅值平方
    float alpha = alpha_beta->alpha;
    float beta = alpha_beta->beta;
    float u_mag_sq = alpha * alpha + beta * beta;
    const float u_max = _1_SQRT3 * vdc;
    const float u_max_sq = u_max * u_max;

    // 2) 过调制处理
    if (u_mag_sq > u_max_sq && u_mag_sq > 1e-12f) {
        float u_mag = sqrtf(u_mag_sq);
        float scale = u_max / u_mag;
        alpha *= scale;
        beta *= scale;
        alpha_beta->alpha = alpha;
        alpha_beta->beta = beta;
    }

    // 3) 中间变量计算
    float factor = FACTOR / vdc;
    float half_factor = factor * 0.5f;
    float X = beta * factor;
    float Y = (_SQRT3 * alpha + beta) * half_factor;
    float Z = (beta - _SQRT3 * alpha) * half_factor;

    // 4. 按扇区计算T1和T2（基于X/Y/Z）
    switch (svpwm->sector) {
        case 1: // 扇区1：V4(100) + V6(110) → T1=-Z, T2=X 3
            svpwm->T1 = -Z;
            svpwm->T2 = X;
            break;
        case 2: // 扇区2：V6(110) + V2(010) → T1=Z, T2=Y 1
            svpwm->T1 = Z;
            svpwm->T2 = Y;
            break;
        case 3: // 扇区3：V2(010) + V3(011) → T1=X, T2=-Y 5
            svpwm->T1 = X;
            svpwm->T2 = -Y;
            break;
        case 4: // 扇区4：V3(011) + V1(001) → T1=-X, T2=Z 4
            svpwm->T1 = -X;
            svpwm->T2 = Z;
            break;
        case 5: // 扇区5：V1(001) + V5(101) → T1=-Y, T2=-Z 6
            svpwm->T1 = -Y;
            svpwm->T2 = -Z;
            break;
        case 6: // 扇区6：V5(101) + V4(100) → T1=Y, T2=-X 2
            svpwm->T1 = Y;
            svpwm->T2 = -X;
            break;
        default:
            svpwm->T1 = 0.0f;
            svpwm->T2 = 0.0f;
            break;
    }

    // 6. 计算零矢量时间（确保T0≥0，避免负数）
    svpwm->T0 = PWM_PERIOD_S - svpwm->T1 - svpwm->T2;
    if (svpwm->T0 < 0.0f)
        svpwm->T0 = 0.0f;
}

/**
 * @brief SVPWM计算三相导通时间并转换为比较值
 * @param sector 当前扇区 (1-6)
 * @param T1 基本矢量1作用时间
 * @param T2 基本矢量2作用时间
 * @param T0 零矢量作用时间
 * @param pwm_a A相比较值指针
 * @param pwm_b B相比较值指针
 * @param pwm_c C相比较值指针
 */
void svpwm_duty_calc(SVPWM_t *svpwm)
{
    float Ta, Tb, Tc; // 三相桥臂导通时间（s）
    float T0_half = svpwm->T0 / 2.0f;

    // 按扇区计算三相导通时间
    switch (svpwm->sector) {
        case 1: // 扇区1
            Ta = T0_half + svpwm->T1 + svpwm->T2;
            Tb = T0_half + svpwm->T2;
            Tc = T0_half;
            break;
        case 2: // 扇区2
            Ta = T0_half + svpwm->T2;
            Tb = T0_half + svpwm->T1 + svpwm->T2;
            Tc = T0_half;
            break;
        case 3: // 扇区3
            Ta = T0_half;
            Tb = T0_half + svpwm->T1 + svpwm->T2;
            Tc = T0_half + svpwm->T2;
            break;
        case 4: // 扇区4
            Ta = T0_half;
            Tb = T0_half + svpwm->T2;
            Tc = T0_half + svpwm->T1 + svpwm->T2;
            break;
        case 5: // 扇区5
            Ta = T0_half + svpwm->T2;
            Tb = T0_half;
            Tc = T0_half + svpwm->T1 + svpwm->T2;
            break;
        case 6: // 扇区6
            Ta = T0_half + svpwm->T1 + svpwm->T2;
            Tb = T0_half;
            Tc = T0_half + svpwm->T2;
            break;
        default: // 异常扇区
            Ta = T0_half;
            Tb = T0_half;
            Tc = T0_half;
            break;
    }
    // 导通时间→比较值
    const float k = PWM_FREQ_PERIOD;
    svpwm->pwm_a = Ta * k;
    svpwm->pwm_b = Tb * k;
    svpwm->pwm_c = Tc * k;
}

/**
 * @brief 生成单极性倍频SPWM波
 * @param spwm 四相spwm比较值指针
 * @param alpha_beta 输出的αβ轴值
 * @param vbus 母线电压
 */
void spwm_duty_calc(SPWM_t *spwm, AlphaBetaTypeDef *alpha_beta)
{
    const float half_pwm = 0.5f * PWM_PERIOD;
    const float vbus = mc_voltage_data.vbus;
    const float scale = mc_voltage_data.spwm_scale;
    float UA = clampf(alpha_beta->alpha, -vbus, vbus);
    float UB = clampf(alpha_beta->beta, -vbus, vbus);

    // SPWM比较值计算
    spwm->duty_Ap = (uint32_t)(half_pwm + UA * scale);
    spwm->duty_An = (uint32_t)(half_pwm - UA * scale);
    spwm->duty_Bp = (uint32_t)(half_pwm + UB * scale);
    spwm->duty_Bn = (uint32_t)(half_pwm - UB * scale);
}

/**
 * @brief 将三相电流转换为αβ坐标系下的值
 * @param abc_i 三相电流指针
 * @param alpha_beta 输出的αβ轴值
 */
void clark_transform(void *abc_i, AlphaBetaTypeDef *alpha_beta)
{
    mc_data_i *current_abc = (mc_data_i *)abc_i;

    // 标准幅值不变性Clarke变换
    // α = (2/3)*ia + (-1/3)*ib + (-1/3)*ic
    // β = 0*ia + (1/√3)*ib + (-1/√3)*ic
    const float TWO_THIRD = 2.0f / 3.0f;
    const float ONE_THIRD = 1.0f / 3.0f;
    const float ONE_SQRT3 = _1_SQRT3;

    // 电流Clarke变换（a/b/c → α/β）
    alpha_beta->alpha_i = TWO_THIRD * current_abc->ia -
        ONE_THIRD * current_abc->ib - ONE_THIRD * current_abc->ic;
    alpha_beta->beta_i =
        ONE_SQRT3 * current_abc->ib - ONE_SQRT3 * current_abc->ic;
}

/**
 * @brief 将αβ转换为dq坐标系下的电流值
 * @param current_abc αβ电流值
 * @param current_αβ 输出的dq轴电流值
 */
void park_transform(AlphaBetaTypeDef *alpha_beta, mc_control_t *mc_ctrl,
                    float sin_theta, float cos_theta)
{
    mc_ctrl->abc_dq.current_d =
        cos_theta * alpha_beta->alpha_i + sin_theta * alpha_beta->beta_i;
    mc_ctrl->abc_dq.current_q =
        -sin_theta * alpha_beta->alpha_i + cos_theta * alpha_beta->beta_i;
}

/**
 * @brief 反Park变换
 * @param d_ptr D轴输入指针
 * @param q_ptr Q轴输入指针
 * @param alpha_ptr Alpha轴输出指针
 * @param beta_ptr Beta轴输出指针
 * @param angle 电角度 (rad)
 */
void inv_park_transform_f32(mc_control_t *mc_ctrl, AlphaBetaTypeDef *alpha_beta,
                            float sin_theta, float cos_theta)
{
    // 反Park变换：将旋转DQ坐标系电压转换为静止αβ坐标系电压
    alpha_beta->alpha = mc_ctrl->out_d * cos_theta - mc_ctrl->out_q * sin_theta;
    alpha_beta->beta = mc_ctrl->out_d * sin_theta + mc_ctrl->out_q * cos_theta;
}

/**
 * @brief D轴电流环PID计算
 * @param target_id 目标D轴电流
 * @param actual_id 实际D轴电流
 * @return D轴电压输出
 */
float mc_id_pid_calculate(float target_id, float actual_id)
{
    float error = target_id - actual_id;
    float p_term = id_pid.kp * error;

    // 积分项计算与限幅
    id_pid.integral += id_pid.ki * error;

    if (id_pid.integral > id_pid.integral_limit) {
        id_pid.integral = id_pid.integral_limit;
    }
    else if (id_pid.integral < -id_pid.integral_limit) {
        id_pid.integral = -id_pid.integral_limit;
    }

    float id_out = p_term + id_pid.integral;
    return id_out;
}

/**
 * @brief Q轴电流环PID计算
 * @param target_iq 目标Q轴电流
 * @param actual_iq 实际Q轴电流
 * @return Q轴电压输出
 */
float mc_iq_pid_calculate(float target_iq, float actual_iq)
{
    float error = target_iq - actual_iq;
    float p_term = iq_pid.kp * error;

    // 积分项计算与限幅
    iq_pid.integral += iq_pid.ki * error;

    if (iq_pid.integral > iq_pid.integral_limit) {
        iq_pid.integral = iq_pid.integral_limit;
    }
    else if (iq_pid.integral < -iq_pid.integral_limit) {
        iq_pid.integral = -iq_pid.integral_limit;
    }

    float iq_out = p_term + iq_pid.integral;
    return iq_out;
}

/**
 * @brief 复位通用 PID 控制器状态
 * @param pid PID 控制器
 */
void mc_pid_reset(pi_t *pid)
{
    if (pid == NULL) {
        return;
    }
    pid->integral = 0.0f;
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->output = 0.0f;
}

/**
 * @brief 执行一次通用 PID 计算
 * @param pid PID 控制器
 * @param error 控制误差
 * @param dt 控制周期（s）
 * @param output_limit 输出和积分限幅
 * @return PID 输出
 */
float mc_pid_update(pi_t *pid, float error, float dt, float output_limit)
{
    float proportional;
    float integral;
    float derivative;
    float output;

    if ((pid == NULL) || (!isfinite(error)) || (!isfinite(dt)) || (dt <= 0.0f) ||
        (!isfinite(output_limit)) || (output_limit <= 0.0f)) {
        return 0.0f;
    }

    proportional = pid->kp * error;
    integral = pid->integral + (pid->ki * dt * error);
    integral = clampf(integral, -output_limit, output_limit);
    derivative = (pid->kd * (error - pid->last_error)) / dt;
    output = proportional + integral + derivative;

    if (output > output_limit) {
        output = output_limit;
        if ((pid->ki > 0.0f) && (error > 0.0f)) {
            integral = pid->integral;
        }
    }
    else if (output < -output_limit) {
        output = -output_limit;
        if ((pid->ki > 0.0f) && (error < 0.0f)) {
            integral = pid->integral;
        }
    }

    pid->integral = integral;
    pid->error = error;
    pid->last_error = error;
    pid->output = output;
    return output;
}

