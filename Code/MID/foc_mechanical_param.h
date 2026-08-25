#ifndef __FOC_MECHANICAL_PARAM_H
#define __FOC_MECHANICAL_PARAM_H

#include <stdbool.h>
#include <stdint.h>

#define MECH_IDENTIFY_LOOP_FREQ_HZ              (2000U)

/* ===================== 时域辨识配置 ===================== */
/* 电流等级（A），峰值 ≤ 0.15A，且必须大于库仑摩擦电流 */
#define MECH_IDENTIFY_CURRENT_LEVELS            (3U)

/* 各阶段持续时间（毫秒），根据行程调整 */
#define MECH_IDENTIFY_ACCEL_MS                  (250U)  /* 加速时间 */
#define MECH_IDENTIFY_CONST_MS                  (400U)  /* 匀速时间 */
#define MECH_IDENTIFY_DECEL_MS                  (250U)  /* 减速时间 */
#define MECH_IDENTIFY_DWELL_MS                  (150U)  /* 换向停顿 */
#define MECH_IDENTIFY_RAMP_MS                   (2U)    /* 电流软启动斜坡 */

/* 换算成控制周期数 */
#define MECH_IDENTIFY_ACCEL_TICKS               ((MECH_IDENTIFY_ACCEL_MS) * (MECH_IDENTIFY_LOOP_FREQ_HZ) / 1000U)
#define MECH_IDENTIFY_CONST_TICKS               ((MECH_IDENTIFY_CONST_MS) * (MECH_IDENTIFY_LOOP_FREQ_HZ) / 1000U)
#define MECH_IDENTIFY_DECEL_TICKS               ((MECH_IDENTIFY_DECEL_MS) * (MECH_IDENTIFY_LOOP_FREQ_HZ) / 1000U)
#define MECH_IDENTIFY_DWELL_TICKS               ((MECH_IDENTIFY_DWELL_MS) * (MECH_IDENTIFY_LOOP_FREQ_HZ) / 1000U)
#define MECH_IDENTIFY_RAMP_TICKS                ((MECH_IDENTIFY_RAMP_MS) * (MECH_IDENTIFY_LOOP_FREQ_HZ) / 1000U)

/* 正反往复总次数（正反各算一次完整序列） */
#define MECH_IDENTIFY_REPEAT_TIMES              (2U)
/* IRLS 最大迭代次数（1=仅普通最小二乘） */
#define MECH_IDENTIFY_MAX_IRLS_ITERATIONS       (4U)

/* 拟合优度阈值 */
#define MECH_IDENTIFY_FIT_R2_THRESHOLD          (0.90f)
/* Huber 鲁棒回归阈值（A） */
#define MECH_IDENTIFY_HUBER_DELTA               (0.03f)

/* 速度死区（rad/s），低于此值不参与回归 */
#define MECH_IDENTIFY_SPEED_DEADZONE            (0.05f)
/* tanh 平滑符号函数系数 */
#define MECH_IDENTIFY_SIGN_GAMMA                (200.0f)
/* 加速度低通滤波系数（0~1），越大滤波越强 */
#define MECH_IDENTIFY_ACCEL_FILT_ALPHA          (0.8f)

/* 最小稳态速度阈值，若最终速度低于此值认为摩擦太大 */
#define MECH_IDENTIFY_MIN_SPEED_THRESHOLD       (0.5f)

typedef enum
{
    MECH_IDENTIFY_IDLE          = 0U,
    MECH_IDENTIFY_START         = 1U,   /* 等待静止，选择电流等级 */
    MECH_IDENTIFY_RAMP_POS      = 2U,   /* 正向软启动斜坡 */
    MECH_IDENTIFY_POS_ACCEL     = 3U,   /* 正向加速 */
    MECH_IDENTIFY_POS_CONST     = 4U,   /* 正向匀速 */
    MECH_IDENTIFY_POS_DECEL     = 5U,   /* 正向减速 */
    MECH_IDENTIFY_DWELL1        = 6U,   /* 停顿 */
    MECH_IDENTIFY_RAMP_NEG      = 7U,   /* 反向软启动斜坡 */
    MECH_IDENTIFY_NEG_ACCEL     = 8U,   /* 反向加速 */
    MECH_IDENTIFY_NEG_CONST     = 9U,   /* 反向匀速 */
    MECH_IDENTIFY_NEG_DECEL     = 10U,  /* 反向减速 */
    MECH_IDENTIFY_DWELL2        = 11U,  /* 停顿 */
    MECH_IDENTIFY_CALCULATE     = 12U,  /* 完成一次往复，判断继续/求解 */
    MECH_IDENTIFY_COMPLETE      = 13U,
    MECH_IDENTIFY_FAILED        = 14U
} mech_identify_state_t;

typedef struct
{
    float Ka;          /* J/Kt 估计值 (A·s²/rad) */
    float Kv;          /* B/Kt 估计值 (A·s/rad) */
    float Tc;          /* 库仑摩擦电流 (A) */
    float J_est;       /* 未使用 */
    float B_est;       /* 未使用 */
    float speed_steady;/* 最后一次稳态速度 */
    float accel_avg;   /* 最后一次平均加速度 */
    float iq_step;     /* 当前阶跃电流幅值 */
    uint32_t iteration;/* 实际完成往复次数 */
    bool valid;        /* 结果是否有效 */
} mech_identify_result_t;

typedef struct
{
    mech_identify_state_t state;
    mech_identify_result_t result;

    uint32_t state_timer;
    float current_iq_level;       /* 当前电流等级 (A) */
    float speed_start;
    float speed_max;
    float speed_steady;
    float iq_steady_sum;
    uint32_t iq_steady_cnt;
    uint32_t sample_count;
    uint32_t iteration_count;     /* 已完成完整往复次数 */
    uint32_t irls_iter;           /* 当前 IRLS 迭代次数 */

    /* 滤波变量 */
    float speed_filt;             /* 速度滤波（直接用外部已滤波值，不再内部滤波） */
    float prev_speed_filt;
    float accel_filt;             /* 加速度滤波值 */

    /* 加权最小二乘累积量（上三角） */
    float acc_A[4][4];
    float acc_b[4];
    float sum_iq_sq;
    uint32_t acc_sample_count;

    /* 当前 θ 估计（用于 IRLS 权重计算） */
    float theta[4];

    bool identify_request;
} mech_identify_t;

extern mech_identify_t g_mech_identify;

void foc_mechanical_param_init(void);
void foc_mechanical_param_process(void);
void foc_mechanical_param_start(void);
void foc_mechanical_param_stop(void);
bool foc_mechanical_param_is_active(void);
const mech_identify_result_t* foc_mechanical_param_get_result(void);

#endif /* __FOC_MECHANICAL_PARAM_H */
