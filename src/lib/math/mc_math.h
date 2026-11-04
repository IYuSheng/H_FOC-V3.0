/**
 * @file mc_math.h
 * @brief FOC 坐标变换、SVPWM/SPWM 与基础控制器接口。
 */

#ifndef MC_MATH_H_
#define MC_MATH_H_

#include "board_config.h"
#include "data_types.h"
#include "mc_encoder.h"

/** 电角度索引位数 */
#define MC_ANGLE_INDEX_BITS 10U
/** 一个电周期的角度索引数量 */
#define MC_ANGLE_INDEX_COUNT (1U << MC_ANGLE_INDEX_BITS)
/** 电角度索引掩码 */
#define MC_ANGLE_INDEX_MASK  (MC_ANGLE_INDEX_COUNT - 1U)
/** 1/4 电周期对应的索引偏移 */
#define MC_ANGLE_INDEX_QUARTER (MC_ANGLE_INDEX_COUNT / 4U)
/** 单个角度索引对应的电角度，单位：rad */
#define MC_ANGLE_INDEX_TO_RAD (_2PI / (float)MC_ANGLE_INDEX_COUNT)

/**
 * @brief 两相静止坐标系数据
 */
typedef struct {
    float alpha;   /**< α 轴电压（V） */
    float beta;    /**< β 轴电压（V） */
    float alpha_i; /**< α 轴电流（A） */
    float beta_i;  /**< β 轴电流（A） */
    float alpha_v; /**< α 轴电压备份（V） */
    float beta_v;  /**< β 轴电压备份（V） */
} AlphaBetaTypeDef;

/**
 * @brief SVPWM 计算状态
 */
typedef struct {
    float T0;       /**< 零矢量作用时间（s） */
    float T1;       /**< 第一基本矢量作用时间（s） */
    float T2;       /**< 第二基本矢量作用时间（s） */
    uint8_t sector; /**< 扇区编号，范围 1~6 */
    uint16_t pwm_a; /**< A 相 PWM 比较值 */
    uint16_t pwm_b; /**< B 相 PWM 比较值 */
    uint16_t pwm_c; /**< C 相 PWM 比较值 */
} SVPWM_t;

/**
 * @brief 四相单极性倍频 SPWM 比较值
 */
typedef struct {
    uint16_t duty_Ap; /**< A+ 比较值 */
    uint16_t duty_An; /**< A- 比较值 */
    uint16_t duty_Bp; /**< B+ 比较值 */
    uint16_t duty_Bn; /**< B- 比较值 */
} SPWM_t;

/**
 * @brief FOC 控制共享状态
 */
typedef struct {
    float out_d;              /**< d 轴输出电压（V） */
    float out_q;              /**< q 轴输出电压（V） */
    float target_d;           /**< 弱磁前的 d 轴电流给定（A） */
    float target_q;           /**< 弱磁前的 q 轴电流给定（A） */
    float target_d_effective; /**< 弱磁和限幅后的 d 轴电流给定（A） */
    float target_q_effective; /**< 限幅后的 q 轴电流给定（A） */
    float angle;              /**< 当前电角度（rad） */
    float speed;              /**< 当前机械速度（rad/s） */
    float target_speed;       /**< 机械速度给定（rad/s） */
    float target_position;    /**< 目标机械位置（rad） */
    float shaped_t_p;         /**< 规划后位置给定（rad） */
    float shaped_t_v;         /**< 规划后速度给定（rad/s） */
    float shaped_t_a;         /**< 规划后加速度给定（rad/s^2） */
    float Te;                 /**< 电磁转矩（N*m） */
    uint8_t sensor_mode;      /**< 传感器模式 */
    struct {
        float current_d;      /**< d 轴实际电流（A） */
        float current_q;      /**< q 轴实际电流（A） */
        float current_d_filt; /**< 滤波后的 d 轴电流（A） */
        float current_q_filt; /**< 滤波后的 q 轴电流（A） */
    } abc_dq;
} mc_control_t;

/**
 * @brief 通用 PID 控制器状态
 */
typedef struct {
    float kp;             /**< 比例增益 */
    float ki;             /**< 积分增益 */
    float kd;             /**< 微分增益 */
    float integral;       /**< 积分项 */
    float integral_limit; /**< 积分限幅 */
    float error;          /**< 当前误差 */
    float last_error;     /**< 上次误差 */
    float output;         /**< 控制器输出 */
    float target;         /**< 目标位置（rad） */
    float current;        /**< 当前位置（rad） */
    float w_ref;          /**< 期望速度（rad/s） */
} pi_t;

/**
 * @brief 全局 FOC 控制状态
 */
extern mc_control_t mc_ctrl;

/**
 * @brief 全局 αβ 坐标状态
 */
extern AlphaBetaTypeDef alpha_beta;

/**
 * @brief 全局 SVPWM 状态
 */
extern SVPWM_t svpwm;

/**
 * @brief 全局 SPWM 状态
 */
extern SPWM_t spwm;

/**
 * @brief Q10 正弦查表数据，共 1024 项
 */
extern float mc_sine_table[MC_ANGLE_INDEX_COUNT];

/**
 * @brief 初始化 Q10 正弦查表
 * @note 必须在 CORDIC 初始化完成后、启动电流环前调用。
 */
void mc_sine_table_init(void);

/**
 * @brief 电角度归一化（映射到 0~2π 范围）
 * @param angle 输入电角度（rad，范围无限制）
 * @return 归一化后电角度（rad，0~2π）
 */
float angle_normalize(float angle);

/**
 * @brief 电角度归一化（映射到 -π~π 范围）
 * @param angle 输入电角度（rad，范围无限制）
 * @return 归一化后电角度（rad，-π~π）
 */
float angle_normalize_pi(float angle);

/**
 * @brief 计算 SVPWM 扇区
 * @param alpha_beta αβ 坐标系电压
 * @return 扇区编号（1~6）
 */
uint8_t svpwm_sector_calc(AlphaBetaTypeDef *alpha_beta);

/**
 * @brief 计算 SVPWM 基本矢量作用时间
 * @param alpha_beta αβ 坐标系电压
 * @param svpwm SVPWM 状态
 * @param vdc 母线电压（V）
 */
void svpwm_calc_times(AlphaBetaTypeDef *alpha_beta, SVPWM_t *svpwm, float vdc);

/**
 * @brief 计算 SVPWM 三相比较值
 * @param svpwm SVPWM 状态
 */
void svpwm_duty_calc(SVPWM_t *svpwm);

/**
 * @brief 计算四相单极性倍频 SPWM 比较值
 * @param spwm 四相 SPWM 状态
 * @param alpha_beta αβ 坐标系电压
 */
void spwm_duty_calc(SPWM_t *spwm, AlphaBetaTypeDef *alpha_beta);

/**
 * @brief 执行 Clarke 变换
 * @param abc_i 三相电流输入
 * @param alpha_beta αβ 坐标输出
 */
void clark_transform(void *abc_i, AlphaBetaTypeDef *alpha_beta);

/**
 * @brief 执行 Park 变换
 * @param alpha_beta αβ 电流输入
 * @param mc_ctrl d/q 电流输出
 * @param sin_theta 电角度正弦值
 * @param cos_theta 电角度余弦值
 */
void park_transform(AlphaBetaTypeDef *alpha_beta, mc_control_t *mc_ctrl,
                    float sin_theta, float cos_theta);

/**
 * @brief 执行反 Park 变换
 * @param mc_ctrl d/q 电压输入
 * @param alpha_beta αβ 电压输出
 * @param sin_theta 电角度正弦值
 * @param cos_theta 电角度余弦值
 */
void inv_park_transform_f32(mc_control_t *mc_ctrl, AlphaBetaTypeDef *alpha_beta,
                            float sin_theta, float cos_theta);

/**
 * @brief 计算 d 轴电流 PI 输出
 * @param target_id 目标 d 轴电流（A）
 * @param actual_id 实际 d 轴电流（A）
 * @return d 轴电压输出（V）
 */
float mc_id_pid_calculate(float target_id, float actual_id);

/**
 * @brief 计算 q 轴电流 PI 输出
 * @param target_iq 目标 q 轴电流（A）
 * @param actual_iq 实际 q 轴电流（A）
 * @return q 轴电压输出（V）
 */
float mc_iq_pid_calculate(float target_iq, float actual_iq);

/**
 * @brief 复位通用 PID 控制器状态
 * @param pid PID 控制器
 */
void mc_pid_reset(pi_t *pid);

/**
 * @brief 执行一次通用 PID 计算
 * @param pid PID 控制器
 * @param error 控制误差
 * @param dt 控制周期（s）
 * @param output_limit 输出和积分限幅
 * @return PID 输出
 */
float mc_pid_update(pi_t *pid, float error, float dt, float output_limit);

#endif /* MC_MATH_H_ */
