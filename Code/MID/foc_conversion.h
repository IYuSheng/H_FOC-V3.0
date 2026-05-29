#ifndef __FOC_CONVERSION_H
#define __FOC_CONVERSION_H

#include "Config.h"
#include "adc.h"
#include "foc_encoder.h"

// 两相静止坐标系结构体 (Alpha-Beta)
typedef struct
{
 // 输出dq转换
 float alpha;
 float beta;
 // 输入abc转换
 float alpha_i;
 float beta_i;
 // 输入abc转换
 float alpha_v;
 float beta_v;
} AlphaBetaTypeDef;

// SVPWM结构体参数定义
typedef struct
{
 float T0;
 float T1;
 float T2;
 uint8_t sector;
 uint16_t pwm_a;
 uint16_t pwm_b;
 uint16_t pwm_c;
} SVPWM_t;

// 单极性倍频spwm调制结构体参数定义
typedef struct
{
 uint16_t duty_Ap;
 uint16_t duty_An;
 uint16_t duty_Bp;
 uint16_t duty_Bn;
} SPWM_t;

// FOC控制参数结构体
typedef struct
{
 float out_d;            // 输出D轴电压
 float out_q;            // 输出Q轴电压
 float target_d;         // 目标D轴电流
 float target_q;         // 目标Q轴电流
 float angle;            // 电角度(弧度)
 float speed;            // 速度（rad/s）
 float speed_rpm;        // 速度（rpm）
 float target_speed;     // 目标速度(rpm)
 float target_position;  // 目标位置（°）
 float shaped_t_p;       // 整形后目标位置
 float Te;               // 电磁转矩（N·m）
 uint8_t sensor_mode;        // 传感器模式
 struct
 {
   float current_d;       // 实际D轴电流
   float current_q;       // 实际Q轴电流
   float current_d_filt;  // 滤波后的实际D轴电流
   float current_q_filt;  // 滤波后的实际Q轴电流
 }abc_dq;  // 经采集变换后的实际dq轴电流
} foc_control_t;

// PID控制器结构体
typedef struct {
   float kp;           // 比例增益
   float ki;           // 积分增益
   float kd;           // 微分增益
   float integral;     // 积分项
   float integral_limit; // 积分限幅
   float error;        // 当前误差
   float last_error;   // 上次误差
   float output;       // 输出值
   float target;       // 目标位置
   float current;      // 当前位置
   float w_ref;        // 期望速度
} pi_t;

extern foc_control_t foc_ctrl;
extern AlphaBetaTypeDef alpha_beta;
extern SVPWM_t svpwm;
extern SPWM_t spwm;

// FOC变换相关函数声明
/**
* @brief 电角度归一化（映射到0~2π范围）
* @param angle 输入电角度（rad，范围无限制）
* @return 归一化后电角度（rad，0~2π）
*/
extern inline float angle_normalize(float angle);

/**
 * @brief 电角度归一化（映射到-π~π范围）
 * @param angle 输入电角度（rad，范围无限制）
 * @return 归一化后电角度（rad，-π~π）
 */
extern inline float angle_normalize_pi(float angle);

/**
 * @brief 电角度归一化（映射到0~360范围）
 * @param angle 输入电角度（°，范围无限制）
 * @return 归一化后电角度（°，0~360）
 */
extern inline float angle_normalize_360(float angle);

/**
 * @brief 角度制转弧度制
 * @param deg 输入角度（°）
 * @return 转换后弧度值（rad）
 */
extern inline float deg2rad(float deg);

/**
 * @brief 弧度制转角度制
 * @param rad 输入弧度（rad）
 * @return 转换后角度值（°）
 */
extern inline float rad2deg(float rad);

/**
* @brief SVPWM通用扇区判断函数
* @param alpha_beta αβ坐标系下的电压指针
* @return 扇区编号（1~6，对应0~60°~360°）
*/
extern inline uint8_t svpwm_sector_calc(AlphaBetaTypeDef *alpha_beta);

/**
* @brief SVPWM基本矢量作用时间计算
* @param alpha_beta αβ坐标系下的电压指针
* @param svpwm SVPWM结构体指针
* @param vdc 母线电压（V）
*/
extern inline void svpwm_calc_times(AlphaBetaTypeDef *alpha_beta, SVPWM_t *svpwm, float vdc);

/**
* @brief SVPWM计算三相导通时间并转换为比较值
* @param svpwm SVPWM结构体指针
*/
extern inline void svpwm_duty_calc(SVPWM_t *svpwm);

/**
 * @brief 生成单极性倍频SPWM波
 * @param spwm 四相spwm比较值指针
 * @param alpha_beta 输出的αβ轴值
 * @param vbus 母线电压
 */
extern inline void spwm_duty_calc(SPWM_t *spwm, AlphaBetaTypeDef *alpha_beta, float vbus);

/**
* @brief 将三相电流转换为αβ坐标系下的值
* @param abc_i 三相电流指针
* @param alpha_beta 输出的αβ轴值
*/
extern inline void clark_transform(void *abc_i, AlphaBetaTypeDef *alpha_beta);

/**
 * @brief 将αβ转换为dq坐标系下的电流值
 * @param current_abc αβ电流值
 * @param current_αβ 输出的dq轴电流值
 */
extern inline void park_transform(AlphaBetaTypeDef *alpha_beta, foc_control_t *foc_ctrl, float sin_theta, float cos_theta);

/**
 * @brief 反Park变换
 * @param d_ptr D轴输入指针
 * @param q_ptr Q轴输入指针
 * @param alpha_ptr Alpha轴输出指针
 * @param beta_ptr Beta轴输出指针
 * @param angle 电角度(角度, 0-360度)
 */
extern inline void inv_park_transform_f32(foc_control_t *foc_ctrl, AlphaBetaTypeDef *alpha_beta, float sin_theta, float cos_theta);

/**
* @brief D轴电流环PID计算
* @param target_id 目标D轴电流
* @param actual_id 实际D轴电流
* @return D轴电压输出
*/
extern inline float foc_id_pid_calculate(float target_id, float actual_id);

/**
* @brief Q轴电流环PID计算
* @param target_iq 目标Q轴电流
* @param actual_iq 实际Q轴电流
* @return Q轴电压输出
*/
extern inline float foc_iq_pid_calculate(float target_iq, float actual_iq);

/**
* @brief 速度环PID计算
* @param target_speed 目标速度(°/s)
* @param actual_speed 实际速度(°/s)
* @return 输出值(Q轴电流)
*/
extern inline float foc_speed_pid_calculate(float target_speed, float actual_speed);

/**
* @brief 位置环PI控制器计算
* @param target_position 目标位置（弧度）
* @param current_position 当前位置（弧度）
* @return PI控制器输出（速度指令，°/s）
*/
extern inline float foc_position_pid_calculate(float target_position, float current_position);

#endif /* __FOC_CONVERSION_H */
