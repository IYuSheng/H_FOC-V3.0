#ifndef __CONFIG_H
#define __CONFIG_H

// --------------------------- 模式选项 --------------------------
#define DEBUG_MODE 1 // 打开仿真波形打印

// -------------------------- 电源与硬件限制 --------------------------
#define VOLTAGE_LIMIT           20.0f // 直流母线电压限制 (V)
#define CURRENT_LIMIT           2.5f  // 最大相电流限制 (A)
#define UNDER_VOLTAGE_THRESH    9.0f  // 欠压保护阈值 (V)
#define OVER_TEMPERATURE_THRESH 60.0f // 过温保护阈值 (摄氏度)

// -------------------------- 电机参数 --------------------------
#define MOTOR_POLE_PAIRS    50      // 电机极对数
#define MOTOR_RESISTANCE    1.0f    // 相电阻 (欧姆)
#define MOTOR_INDUCTANCE    0.0024f // 相电感 (H)
#define MOTOR_INDUCTANCE_Ld MOTOR_INDUCTANCE
#define MOTOR_INDUCTANCE_Lq MOTOR_INDUCTANCE
#define MOTOR_FLUX_LINKAGE  0.003f // 磁链 (Wb)

// -------------------------- FOC控制参数 --------------------------
#define PWM_FREQ      30000.0f        // PWM频率 (Hz)，需与定时器配置匹配
#define HALF_PWM_FREQ PWM_FREQ * 0.5f // PWM频率 (Hz)，需与定时器配置匹配
#define PWM_PERIOD_S                                                           \
    1.0f / PWM_FREQ // PWM周期（单位：s），与PWM_FREQ对应：T = 1/F
#define SVPWM_VOLT_COEF                                                        \
    (2.0f / sqrtf(3.0f))      // SVPWM基本矢量幅值系数（母线电压相关，固定值）
#define DEAD_TIME_S (170e-9f) // 死区时间 170ns

// -------------------------- PI调节器参数 --------------------------
// 电流环PI
#define CURRENT_LOOP_BANDWIDTH 1000.0f // 电流环带宽 (Hz)
#define CURRENT_LOOP_COUNT     PWM_FREQ
#define CURRENT_LOOP_DT        (1.0f / CURRENT_LOOP_COUNT) // 电流环调用周期 (单位：s)
#define I_D_P_GAIN                                                             \
    (2.0f * 3.14159265358979f * MOTOR_INDUCTANCE_Ld * CURRENT_LOOP_BANDWIDTH)
#define I_Q_P_GAIN                                                             \
    (2.0f * 3.14159265358979f * MOTOR_INDUCTANCE_Lq * CURRENT_LOOP_BANDWIDTH)
#define I_I_GAIN                                                               \
    (2.0f * 3.14159265358979f * MOTOR_RESISTANCE * CURRENT_LOOP_BANDWIDTH *    \
     CURRENT_LOOP_DT)
#define I_I_LIMIT 5.0f

// 速度环PI
#define SPEED_LOOP_TIME 2000.0f
#define SPEED_LOOP_DT   (1.0f / SPEED_LOOP_TIME) // 速度环调用周期 (单位：s)
#define SPEED_P_GAIN    0.00572957795f           // A/(rad/s)
#define SPEED_I         0.000f
#define SPEED_I_GAIN    (SPEED_I * SPEED_LOOP_DT) // KI = Ki * Ts(减小一个乘法)
#define SPEED_I_LIMIT   0.2f
#define SPEED_OUT_LIMIT 0.4f

// 位置环PID
#define POSITION_LOOP_TIME 2000.0f // 位置环调用频率 (2000Hz)
#define POSITION_LOOP_DT   (1.0f / POSITION_LOOP_TIME) // 位置环调用周期 (单位：s)
#define POSITION_P_GAIN    1.2f                        // A/rad
#define POSITION_I         0.0f
#define POSITION_I_GAIN                                                        \
    (POSITION_I * POSITION_LOOP_DT) // KI = Ki * Ts(减小一个乘法)
#define POSITION_D_GAIN                  0.000f
#define POSITION_I_LIMIT                 0.08f
#define POSITION_OUT_LIMIT               2.0f
#define POSITION_KV                      8.0f
#define POSITION_W_MAX                   10.4719755f // rad/s
#define POSITION_LADRC_CONTROL_BANDWIDTH 60.0f       // wc, rad/s
#define POSITION_LESO_BANDWIDTH          180.0f      // wo, rad/s
#define POSITION_LADRC_B0                1264.0f      // rad/s^2/A
#define POSITION_LADRC_OUT_MIN           (-POSITION_OUT_LIMIT)
#define POSITION_LADRC_OUT_MAX           POSITION_OUT_LIMIT

// -------------------------- 电流采样配置 --------------------------
#define ADC_REF_VOLTAGE 3.3f    // ADC参考电压 (V)
#define ADC_MAX_VALUE   4095.0f // ADC分辨率 (例如：12位ADC为4095，0-4095)
#define INA240_GAIN     50.0f   // 电流采样运放增益
#define V_REF           1.65f   // REF1 引脚电压3.3V，单位：V
#define R_Current       0.006f  // 采样电阻阻值

// -------------------------- 电压采样配置 --------------------------
#define R_Voaltage_1 20.0f // 20KΩ
#define R_Voaltage_2 2.2f  // 2.2KΩ

// -------------------------- 常用常量及中间变量定义 --------------------------
#define PI       3.14159265358979f
#define _PI      PI
#define _2PI     (2.0f * PI)
#define _PI_2    (PI / 2.0f)
#define _2_PI    (1.0f / (2.0f * PI))
#define _SQRT3   1.732050807568877f
#define _SQRT3_2 0.866025403784439f  // sqrt(3)/2
#define _1_SQRT3 0.577350269189626f  // 1/sqrt(3)
#define _2_SQRT3 1.154700538379252f  // 2/sqrt(3)
#define FACTOR   (_SQRT3 / PWM_FREQ) // SVPWM时间中间计算系数
#define PWM_PERIOD                                                             \
    ((170000000.0f / (2.0f * PWM_FREQ)) -                                      \
     1) // PWM周期计数值 (TIM时钟频率需根据实际配置调整)
#define PWM_FREQ_PERIOD (PWM_FREQ * PWM_PERIOD) // SVPWM时间转比较值中间系数
#define EPS_F           1e-6f                   // 浮点数精度

// -------------------------- 故障状态定义 --------------------------
typedef enum {
    MOTOR_FAULT_NONE,       // 无故障
    FAULT_OVER_CURRENT,     // 过流
    FAULT_OVER_VOLTAGE,     // 过压
    FAULT_UNDER_VOLTAGE,    // 欠压
    FAULT_OVER_TEMPERATURE, // 过温
    FAULT_ENCODER,          // 编码器故障
    FAULT_ENCODER_ANGLE,    // 编码器角度超限制
    FAULT_I_SAMPLING_ERROR  // 电流采样错误
} FOC_FAULT;

// -------------------------- FOC状态定义 --------------------------
typedef enum {
    FOC_STATE_INIT,    // 初始化
    FOC_STATE_RUNNING, // 运行中
    FOC_STATE_FAULT,   // 故障
    FOC_STATE_STOP     // 停止
} FOC_STATUS;

#endif /* __CONFIG_H */
