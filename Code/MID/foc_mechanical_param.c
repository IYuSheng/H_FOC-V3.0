#include "foc_mechanical_param.h"
#include "foc_control.h"

#include <math.h>
#include <string.h>

mech_identify_t g_mech_identify = {0};
static const float s_current_levels[MECH_IDENTIFY_CURRENT_LEVELS] = {
    0.14f, 0.13f, 0.12f
};

/* 静态函数声明 */
static void mech_identify_reset(void);
static void mech_identify_calculate(void);
static void mech_identify_accumulate_weighted(float iq, float speed, float accel, float weight);
static bool mech_solve_linear4(float A[4][4], float b[4], float x[4]);
static float mech_calc_huber_weight(float residual);

/**
 * @brief 初始化机械参数辨识状态机
 */
void foc_mechanical_param_init(void)
{
    mech_identify_reset();
    g_mech_identify.identify_request = false;
}

/**
 * @brief 复位机械参数辨识的全部运行数据
 */
static void mech_identify_reset(void)
{
    g_mech_identify.state = MECH_IDENTIFY_IDLE;
    g_mech_identify.state_timer = 0U;
    g_mech_identify.current_iq_level = 0.0f;
    g_mech_identify.speed_start = 0.0f;
    g_mech_identify.speed_max = 0.0f;
    g_mech_identify.speed_steady = 0.0f;
    g_mech_identify.iq_steady_sum = 0.0f;
    g_mech_identify.iq_steady_cnt = 0U;
    g_mech_identify.sample_count = 0U;
    g_mech_identify.iteration_count = 0U;
    g_mech_identify.irls_iter = 0U;
    g_mech_identify.identify_request = false;

    g_mech_identify.speed_filt = 0.0f;
    g_mech_identify.prev_speed_filt = 0.0f;
    g_mech_identify.accel_filt = 0.0f;

    memset(g_mech_identify.acc_A, 0, sizeof(g_mech_identify.acc_A));
    memset(g_mech_identify.acc_b, 0, sizeof(g_mech_identify.acc_b));
    g_mech_identify.sum_iq_sq = 0.0f;
    g_mech_identify.acc_sample_count = 0U;

    memset(g_mech_identify.theta, 0, sizeof(g_mech_identify.theta));

    g_mech_identify.result.Ka = 0.0f;
    g_mech_identify.result.Kv = 0.0f;
    g_mech_identify.result.Tc = 0.0f;
    g_mech_identify.result.J_est = 0.0f;
    g_mech_identify.result.B_est = 0.0f;
    g_mech_identify.result.speed_steady = 0.0f;
    g_mech_identify.result.accel_avg = 0.0f;
    g_mech_identify.result.iq_step = 0.0f;
    g_mech_identify.result.iteration = 0U;
    g_mech_identify.result.valid = false;
}

/**
 * @brief 请求启动一次机械参数辨识
 */
void foc_mechanical_param_start(void)
{
    mech_identify_reset();
    g_mech_identify.identify_request = true;
}

/**
 * @brief 停止机械参数辨识并清零目标电流
 */
void foc_mechanical_param_stop(void)
{
    foc_ctrl.target_q = 0.0f;
    mech_identify_reset();
}

/**
 * @brief 查询机械参数辨识是否处于活动状态
 * @return true 表示辨识请求、采样或结果处理尚未结束
 */
bool foc_mechanical_param_is_active(void)
{
    return g_mech_identify.identify_request ||
        (g_mech_identify.state != MECH_IDENTIFY_IDLE);
}

/**
 * @brief 获取最近一次机械参数辨识结果
 * @return 辨识结果只读指针
 */
const mech_identify_result_t* foc_mechanical_param_get_result(void)
{
    return &g_mech_identify.result;
}

/**
 * @brief 根据残差计算 Huber 鲁棒回归权重
 * @param residual 当前样本残差
 * @return 样本权重
 */
static float mech_calc_huber_weight(float residual)
{
    float abs_r = fabsf(residual);
    if (abs_r <= MECH_IDENTIFY_HUBER_DELTA) {
        return 1.0f;
    }
    return MECH_IDENTIFY_HUBER_DELTA / abs_r;
}

/**
 * @brief 累积一个样本到加权法方程
 */
static void mech_identify_accumulate_weighted(float iq, float speed, float accel, float weight)
{
    float w = speed;
    float s = tanhf(MECH_IDENTIFY_SIGN_GAMMA * w);
    float a = accel;
    float y = iq;

    g_mech_identify.acc_A[0][0] += weight * a * a;
    g_mech_identify.acc_A[0][1] += weight * a * w;
    g_mech_identify.acc_A[0][2] += weight * a * s;
    g_mech_identify.acc_A[0][3] += weight * a;

    g_mech_identify.acc_A[1][1] += weight * w * w;
    g_mech_identify.acc_A[1][2] += weight * w * s;
    g_mech_identify.acc_A[1][3] += weight * w;

    g_mech_identify.acc_A[2][2] += weight * s * s;
    g_mech_identify.acc_A[2][3] += weight * s;

    g_mech_identify.acc_A[3][3] += weight;

    g_mech_identify.acc_b[0] += weight * y * a;
    g_mech_identify.acc_b[1] += weight * y * w;
    g_mech_identify.acc_b[2] += weight * y * s;
    g_mech_identify.acc_b[3] += weight * y;

    g_mech_identify.sum_iq_sq += weight * y * y;
    g_mech_identify.acc_sample_count++;
}

/**
 * @brief 求解 4x4 线性方程组（高斯消元）
 */
static bool mech_solve_linear4(float A[4][4], float b[4], float x[4])
{
    float aug[4][5];
    int i, j, k;

    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            aug[i][j] = A[i][j];
        }
        aug[i][4] = b[i];
    }

    for (i = 0; i < 4; i++) {
        int max_row = i;
        float max_val = fabsf(aug[i][i]);
        for (k = i + 1; k < 4; k++) {
            if (fabsf(aug[k][i]) > max_val) {
                max_val = fabsf(aug[k][i]);
                max_row = k;
            }
        }
        if (max_val < 1e-9f) {
            return false;
        }
        if (max_row != i) {
            for (j = i; j < 5; j++) {
                float tmp = aug[i][j];
                aug[i][j] = aug[max_row][j];
                aug[max_row][j] = tmp;
            }
        }
        for (k = i + 1; k < 4; k++) {
            float factor = aug[k][i] / aug[i][i];
            for (j = i; j < 5; j++) {
                aug[k][j] -= factor * aug[i][j];
            }
        }
    }

    for (i = 3; i >= 0; i--) {
        float sum = aug[i][4];
        for (j = i + 1; j < 4; j++) {
            sum -= aug[i][j] * x[j];
        }
        x[i] = sum / aug[i][i];
    }
    return true;
}

/**
 * @brief 计算阶段：一次完整往复结束，判断是否继续/求解
 */
static void mech_identify_calculate(void)
{
    if (g_mech_identify.acc_sample_count < 20U) {
        g_mech_identify.result.valid = false;
        g_mech_identify.state = MECH_IDENTIFY_FAILED;
        return;
    }

    if (g_mech_identify.iteration_count < MECH_IDENTIFY_REPEAT_TIMES) {
        /* 继续下一次往复 */
        g_mech_identify.iteration_count++;
        g_mech_identify.state = MECH_IDENTIFY_START;
        return;
    }

    /* 求解当前累积的法方程 */
    float A[4][4];
    int i, j;
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            if (i <= j) A[i][j] = g_mech_identify.acc_A[i][j];
            else A[i][j] = g_mech_identify.acc_A[j][i];
        }
    }
    float b[4] = {
        g_mech_identify.acc_b[0],
        g_mech_identify.acc_b[1],
        g_mech_identify.acc_b[2],
        g_mech_identify.acc_b[3]
    };
    float theta_new[4];

    if (!mech_solve_linear4(A, b, theta_new)) {
        g_mech_identify.result.valid = false;
        g_mech_identify.state = MECH_IDENTIFY_FAILED;
        return;
    }

    if (g_mech_identify.irls_iter < MECH_IDENTIFY_MAX_IRLS_ITERATIONS - 1U) {
        memcpy(g_mech_identify.theta, theta_new, sizeof(theta_new));
        g_mech_identify.irls_iter++;

        /* 清空累积，重新采集 */
        memset(g_mech_identify.acc_A, 0, sizeof(g_mech_identify.acc_A));
        memset(g_mech_identify.acc_b, 0, sizeof(g_mech_identify.acc_b));
        g_mech_identify.sum_iq_sq = 0.0f;
        g_mech_identify.acc_sample_count = 0U;
        g_mech_identify.iteration_count = 0U;
        g_mech_identify.state = MECH_IDENTIFY_START;
        return;
    }

    /* 最终参数检查 */
    if ((!isfinite(theta_new[0])) || (!isfinite(theta_new[1])) ||
        (!isfinite(theta_new[2])) || (theta_new[0] <= 0.0f) ||
        (theta_new[1] <= 0.0f)) {
        g_mech_identify.result.valid = false;
        g_mech_identify.state = MECH_IDENTIFY_FAILED;
        return;
    }

    /* 计算 R² */
    float N = (float)g_mech_identify.acc_sample_count;
    float mean_iq = b[3] / N;
    float ss_tot = g_mech_identify.sum_iq_sq - N * mean_iq * mean_iq;
    if ((!isfinite(ss_tot)) || (ss_tot <= 1e-9f)) {
        g_mech_identify.result.valid = false;
        g_mech_identify.state = MECH_IDENTIFY_FAILED;
        return;
    }
    float tAtheta = 0.0f;
    for (i = 0; i < 4; i++)
        for (j = 0; j < 4; j++)
            tAtheta += theta_new[i] * theta_new[j] * A[i][j];
    float ss_res = g_mech_identify.sum_iq_sq
                 - 2.0f * (theta_new[0]*b[0] + theta_new[1]*b[1] + theta_new[2]*b[2] + theta_new[3]*b[3])
                 + tAtheta;
    if (ss_res < 0) ss_res = 0;
    float r2 = 1.0f - ss_res / ss_tot;

    if ((!isfinite(r2)) || (r2 < MECH_IDENTIFY_FIT_R2_THRESHOLD)) {
        g_mech_identify.result.valid = false;
        g_mech_identify.state = MECH_IDENTIFY_FAILED;
        return;
    }

    g_mech_identify.result.Ka = theta_new[0];
    g_mech_identify.result.Kv = theta_new[1];
    g_mech_identify.result.Tc = theta_new[2];
    g_mech_identify.result.J_est = 0.0f;
    g_mech_identify.result.B_est = 0.0f;
    g_mech_identify.result.speed_steady = g_mech_identify.speed_steady;
    g_mech_identify.result.accel_avg = 0.0f;
    g_mech_identify.result.iq_step = g_mech_identify.current_iq_level;
    g_mech_identify.result.iteration = g_mech_identify.iteration_count;
    g_mech_identify.result.valid = true;

    g_mech_identify.state = MECH_IDENTIFY_COMPLETE;
}
/**
 * @brief 主处理函数
 */
void foc_mechanical_param_process(void)
{
    if (g_mech_identify.identify_request) {
        g_mech_identify.state = MECH_IDENTIFY_START;
        g_mech_identify.identify_request = false;
    }
    if (g_mech_identify.state == MECH_IDENTIFY_IDLE) {
        return;
    }

    float current_speed = encoder_data.mechanical_speed;  // 已滤波的速度
    float current_iq = foc_ctrl.abc_dq.current_q;

    /* 加速度计算：对滤波后速度进行差分并低通 */
    float dt = 1.0f / (float)MECH_IDENTIFY_LOOP_FREQ_HZ;
    float raw_accel = (current_speed - g_mech_identify.prev_speed_filt) / dt;
    g_mech_identify.prev_speed_filt = current_speed;
    g_mech_identify.accel_filt = MECH_IDENTIFY_ACCEL_FILT_ALPHA * g_mech_identify.accel_filt
                               + (1.0f - MECH_IDENTIFY_ACCEL_FILT_ALPHA) * raw_accel;

    switch (g_mech_identify.state) {
        case MECH_IDENTIFY_START: {
            if (fabsf(current_speed) > 0.3f) {
                foc_ctrl.target_q = 0.0f;
                return;
            }

            uint32_t level_index = g_mech_identify.iteration_count % MECH_IDENTIFY_CURRENT_LEVELS;
            g_mech_identify.current_iq_level = s_current_levels[level_index];

            g_mech_identify.speed_start = current_speed;
            g_mech_identify.speed_max = 0.0f;
            g_mech_identify.speed_steady = 0.0f;
            g_mech_identify.iq_steady_sum = 0.0f;
            g_mech_identify.iq_steady_cnt = 0U;
            g_mech_identify.state_timer = 0U;
            g_mech_identify.sample_count = 0U;

            g_mech_identify.prev_speed_filt = current_speed;
            g_mech_identify.accel_filt = 0.0f;

            foc_ctrl.target_q = 0.0f;
            g_mech_identify.state = MECH_IDENTIFY_RAMP_POS;
            break;
        }

        case MECH_IDENTIFY_RAMP_POS: {
            g_mech_identify.state_timer++;
            float ramp = (float)g_mech_identify.state_timer / (float)MECH_IDENTIFY_RAMP_TICKS;
            if (ramp > 1.0f) ramp = 1.0f;
            foc_ctrl.target_q = g_mech_identify.current_iq_level * ramp;
            if (g_mech_identify.state_timer >= MECH_IDENTIFY_RAMP_TICKS) {
                g_mech_identify.state_timer = 0U;
                g_mech_identify.state = MECH_IDENTIFY_POS_ACCEL;
            }
            break;
        }

        case MECH_IDENTIFY_POS_ACCEL: {
            foc_ctrl.target_q = g_mech_identify.current_iq_level;
            g_mech_identify.state_timer++;

            if (fabsf(current_speed) > MECH_IDENTIFY_SPEED_DEADZONE) {
                float res = 0.0f;
                if (g_mech_identify.irls_iter > 0U) {
                    float s = tanhf(MECH_IDENTIFY_SIGN_GAMMA * current_speed);
                    res = current_iq - (g_mech_identify.theta[0] * g_mech_identify.accel_filt
                                      + g_mech_identify.theta[1] * current_speed
                                      + g_mech_identify.theta[2] * s
                                      + g_mech_identify.theta[3]);
                }
                float weight = (g_mech_identify.irls_iter == 0U) ? 1.0f : mech_calc_huber_weight(res);
                mech_identify_accumulate_weighted(current_iq, current_speed,
                                                 g_mech_identify.accel_filt, weight);
            }

            if (g_mech_identify.state_timer >= MECH_IDENTIFY_ACCEL_TICKS) {
                g_mech_identify.state_timer = 0U;
                g_mech_identify.state = MECH_IDENTIFY_POS_CONST;
            }
            break;
        }

        case MECH_IDENTIFY_POS_CONST: {
            foc_ctrl.target_q = g_mech_identify.current_iq_level * 0.7f;
            g_mech_identify.state_timer++;

            if (fabsf(current_speed) > MECH_IDENTIFY_SPEED_DEADZONE) {
                float res = 0.0f;
                if (g_mech_identify.irls_iter > 0U) {
                    float s = tanhf(MECH_IDENTIFY_SIGN_GAMMA * current_speed);
                    res = current_iq - (g_mech_identify.theta[0] * g_mech_identify.accel_filt
                                      + g_mech_identify.theta[1] * current_speed
                                      + g_mech_identify.theta[2] * s
                                      + g_mech_identify.theta[3]);
                }
                float weight = (g_mech_identify.irls_iter == 0U) ? 1.0f : mech_calc_huber_weight(res);
                mech_identify_accumulate_weighted(current_iq, current_speed,
                                                 g_mech_identify.accel_filt, weight);
            }

            if (g_mech_identify.state_timer >= MECH_IDENTIFY_CONST_TICKS) {
                g_mech_identify.state_timer = 0U;
                g_mech_identify.state = MECH_IDENTIFY_POS_DECEL;
            }
            break;
        }

        case MECH_IDENTIFY_POS_DECEL: {
            foc_ctrl.target_q = -g_mech_identify.current_iq_level;
            g_mech_identify.state_timer++;

            if (fabsf(current_speed) > MECH_IDENTIFY_SPEED_DEADZONE) {
                float res = 0.0f;
                if (g_mech_identify.irls_iter > 0U) {
                    float s = tanhf(MECH_IDENTIFY_SIGN_GAMMA * current_speed);
                    res = current_iq - (g_mech_identify.theta[0] * g_mech_identify.accel_filt
                                      + g_mech_identify.theta[1] * current_speed
                                      + g_mech_identify.theta[2] * s
                                      + g_mech_identify.theta[3]);
                }
                float weight = (g_mech_identify.irls_iter == 0U) ? 1.0f : mech_calc_huber_weight(res);
                mech_identify_accumulate_weighted(current_iq, current_speed,
                                                 g_mech_identify.accel_filt, weight);
            }

            if (g_mech_identify.state_timer >= MECH_IDENTIFY_DECEL_TICKS) {
                g_mech_identify.state_timer = 0U;
                g_mech_identify.state = MECH_IDENTIFY_DWELL1;
            }
            break;
        }

        case MECH_IDENTIFY_DWELL1: {
            foc_ctrl.target_q = 0.0f;
            g_mech_identify.state_timer++;
            if (g_mech_identify.state_timer >= MECH_IDENTIFY_DWELL_TICKS) {
                g_mech_identify.state_timer = 0U;
                g_mech_identify.state = MECH_IDENTIFY_RAMP_NEG;
            }
            break;
        }

        case MECH_IDENTIFY_RAMP_NEG: {
            g_mech_identify.state_timer++;
            float ramp = (float)g_mech_identify.state_timer / (float)MECH_IDENTIFY_RAMP_TICKS;
            if (ramp > 1.0f) ramp = 1.0f;
            foc_ctrl.target_q = -g_mech_identify.current_iq_level * ramp;
            if (g_mech_identify.state_timer >= MECH_IDENTIFY_RAMP_TICKS) {
                g_mech_identify.state_timer = 0U;
                g_mech_identify.state = MECH_IDENTIFY_NEG_ACCEL;
            }
            break;
        }

        case MECH_IDENTIFY_NEG_ACCEL: {
            foc_ctrl.target_q = -g_mech_identify.current_iq_level;
            g_mech_identify.state_timer++;

            if (fabsf(current_speed) > MECH_IDENTIFY_SPEED_DEADZONE) {
                float res = 0.0f;
                if (g_mech_identify.irls_iter > 0U) {
                    float s = tanhf(MECH_IDENTIFY_SIGN_GAMMA * current_speed);
                    res = current_iq - (g_mech_identify.theta[0] * g_mech_identify.accel_filt
                                      + g_mech_identify.theta[1] * current_speed
                                      + g_mech_identify.theta[2] * s
                                      + g_mech_identify.theta[3]);
                }
                float weight = (g_mech_identify.irls_iter == 0U) ? 1.0f : mech_calc_huber_weight(res);
                mech_identify_accumulate_weighted(current_iq, current_speed,
                                                 g_mech_identify.accel_filt, weight);
            }

            if (g_mech_identify.state_timer >= MECH_IDENTIFY_ACCEL_TICKS) {
                g_mech_identify.state_timer = 0U;
                g_mech_identify.state = MECH_IDENTIFY_NEG_CONST;
            }
            break;
        }

        case MECH_IDENTIFY_NEG_CONST: {
            foc_ctrl.target_q = -g_mech_identify.current_iq_level * 0.7f;
            g_mech_identify.state_timer++;

            if (fabsf(current_speed) > MECH_IDENTIFY_SPEED_DEADZONE) {
                float res = 0.0f;
                if (g_mech_identify.irls_iter > 0U) {
                    float s = tanhf(MECH_IDENTIFY_SIGN_GAMMA * current_speed);
                    res = current_iq - (g_mech_identify.theta[0] * g_mech_identify.accel_filt
                                      + g_mech_identify.theta[1] * current_speed
                                      + g_mech_identify.theta[2] * s
                                      + g_mech_identify.theta[3]);
                }
                float weight = (g_mech_identify.irls_iter == 0U) ? 1.0f : mech_calc_huber_weight(res);
                mech_identify_accumulate_weighted(current_iq, current_speed,
                                                 g_mech_identify.accel_filt, weight);
            }

            if (g_mech_identify.state_timer >= MECH_IDENTIFY_CONST_TICKS) {
                g_mech_identify.state_timer = 0U;
                g_mech_identify.state = MECH_IDENTIFY_NEG_DECEL;
            }
            break;
        }

        case MECH_IDENTIFY_NEG_DECEL: {
            foc_ctrl.target_q = g_mech_identify.current_iq_level;
            g_mech_identify.state_timer++;

            if (fabsf(current_speed) > MECH_IDENTIFY_SPEED_DEADZONE) {
                float res = 0.0f;
                if (g_mech_identify.irls_iter > 0U) {
                    float s = tanhf(MECH_IDENTIFY_SIGN_GAMMA * current_speed);
                    res = current_iq - (g_mech_identify.theta[0] * g_mech_identify.accel_filt
                                      + g_mech_identify.theta[1] * current_speed
                                      + g_mech_identify.theta[2] * s
                                      + g_mech_identify.theta[3]);
                }
                float weight = (g_mech_identify.irls_iter == 0U) ? 1.0f : mech_calc_huber_weight(res);
                mech_identify_accumulate_weighted(current_iq, current_speed,
                                                 g_mech_identify.accel_filt, weight);
            }

            if (g_mech_identify.state_timer >= MECH_IDENTIFY_DECEL_TICKS) {
                g_mech_identify.state_timer = 0U;
                g_mech_identify.state = MECH_IDENTIFY_DWELL2;
            }
            break;
        }

        case MECH_IDENTIFY_DWELL2: {
            foc_ctrl.target_q = 0.0f;
            g_mech_identify.state_timer++;
            if (g_mech_identify.state_timer >= MECH_IDENTIFY_DWELL_TICKS) {
                g_mech_identify.state_timer = 0U;
                g_mech_identify.speed_steady = current_speed;  // 记录最终速度
                g_mech_identify.state = MECH_IDENTIFY_CALCULATE;
            }
            break;
        }

        case MECH_IDENTIFY_CALCULATE:
            mech_identify_calculate();
            if (g_mech_identify.state == MECH_IDENTIFY_START) {
                foc_ctrl.target_q = 0.0f;
            }
            break;

        case MECH_IDENTIFY_COMPLETE:
        case MECH_IDENTIFY_FAILED:
            foc_ctrl.target_q = 0.0f;
            break;

        default:
            break;
    }
}
