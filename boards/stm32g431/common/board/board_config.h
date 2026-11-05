/**
 * @file board_config.h
 * @brief 硬件限制、电机参数和控制环默认参数。
 */

#ifndef BOARD_CONFIG_H_
#define BOARD_CONFIG_H_

/** @brief 放入初始化数据 CCMRAM 段 */
#define MC_CCM_DATA __attribute__((section(".ccmram_data"), aligned(4)))
/** @brief 放入零初始化 CCMRAM 段 */
#define MC_CCM_BSS __attribute__((section(".ccmram_bss"), aligned(4)))

/** @name 电源与硬件限制 */
/** @{ */
#define VOLTAGE_LIMIT           20.0f /**< 直流母线电压限制（V） */
#define CURRENT_LIMIT           2.5f  /**< 最大相电流限制（A） */
#define UNDER_VOLTAGE_THRESH    9.0f  /**< 欠压保护阈值（V） */
#define OVER_TEMPERATURE_THRESH 60.0f /**< 过温保护阈值（摄氏度） */
/** @} */

/** @name 电机默认参数 */
/** @{ */
#define MOTOR_POLE_PAIRS    50               /**< 电机极对数 */
#define MOTOR_RESISTANCE    1.0f             /**< 相电阻（ohm） */
#define MOTOR_INDUCTANCE    0.0024f          /**< 相电感（H） */
#define MOTOR_INDUCTANCE_Ld MOTOR_INDUCTANCE /**< d 轴电感（H） */
#define MOTOR_INDUCTANCE_Lq MOTOR_INDUCTANCE /**< q 轴电感（H） */
#define MOTOR_FLUX_LINKAGE  0.003f           /**< 永磁磁链（Wb） */
#define MOTOR_KA            0.0f             /**< J/Kt（A*s^2/rad） */
#define MOTOR_KV            0.0f             /**< B/Kt（A*s/rad） */
#define MOTOR_TC            0.0f             /**< 库仑摩擦电流（A） */
#define MOTOR_ANGLE_MIN     0.0f             /**< 默认角度下限（rad），0 表示不限制 */
#define MOTOR_ANGLE_MAX     0.0f             /**< 默认角度上限（rad），0 表示不限制 */
#define MOTOR_SHAPER_TYPE          0U   /**< 输入整形类型，0 表示默认不整形 */
#define MOTOR_SHAPER_FREQUENCY_HZ  0.0f /**< 输入整形谐振频率（Hz） */
#define MOTOR_SHAPER_DAMPING_RATIO 0.0f /**< 输入整形阻尼比 */
/** @} */

/** @name FOC 控制参数 */
/** @{ */
#define PWM_FREQ        30000.0f             /**< PWM 频率（Hz） */
#define HALF_PWM_FREQ   (PWM_FREQ * 0.5f)    /**< 半 PWM 频率（Hz） */
#define PWM_PERIOD_S    (1.0f / PWM_FREQ)    /**< PWM 周期（s） */
#define SVPWM_VOLT_COEF (2.0f / sqrtf(3.0f)) /**< SVPWM 电压系数 */
#define DEAD_TIME_S     (0)                  /**< 死区时间（s） */
/** @} */

/** @name 电流环参数 */
/** @{ */
#define CURRENT_LOOP_COUNT PWM_FREQ
#define CURRENT_LOOP_DT    (1.0f / CURRENT_LOOP_COUNT) /**< 电流环周期（s） */
#define I_I_LIMIT          3.0f                        /**< 电流积分限幅（V） */
/** @} */

/** @name 速度环参数 */
/** @{ */
#define SPEED_LOOP_TIME      5000.0f
#define SPEED_LOOP_DT        (1.0f / SPEED_LOOP_TIME) /**< 速度环周期（s） */
#define SPEED_OUT_LIMIT      3.0f    /**< 速度环输出限幅（A） */
#define SPEED_PID_KP_DEFAULT 0.0125f /**< 速度环默认 Kp */
#define SPEED_PID_KI_DEFAULT 0.125f  /**< 速度环默认 Ki */
#define SPEED_PID_KD_DEFAULT 0.0f    /**< 速度环默认 Kd */
/** @} */

/** @name 位置环参数 */
/** @{ */
#define POSITION_LOOP_TIME 5000.0f                     /**< 位置环频率（Hz） */
#define POSITION_LOOP_DT   (1.0f / POSITION_LOOP_TIME) /**< 位置环周期（s） */
#define MC_POSITION_LOOP_DECIMATION 1U     /**< 外环到位置环的降采样系数 */
#define POSITION_PID_KP_DEFAULT     6.0f   /**< 位置环默认 Kp */
#define POSITION_PID_KI_DEFAULT     100.0f /**< 位置环默认 Ki */
#define POSITION_PID_KD_DEFAULT     0.01f  /**< 位置环默认 Kd */
#define CASCADE_PID_KP_DEFAULT      10.0f  /**< 三环位置环默认 Kp */
#define CASCADE_PID_KI_DEFAULT      0.0f   /**< 三环位置环默认 Ki */
#define CASCADE_PID_KD_DEFAULT      0.0f   /**< 三环位置环默认 Kd */
#define POSITION_CURRENT_LIMIT_A    CURRENT_LIMIT /**< 位置环输出电流限幅（A） */
#define POSITION_SPEED_LIMIT_RAD_S  100.0f /**< 位置环输出速度限幅（rad/s） */
/** @} */

/** @name 电流采样配置 */
/** @{ */
#define ADC_REF_VOLTAGE 3.3f    /**< ADC 参考电压（V） */
#define ADC_MAX_VALUE   4095.0f /**< 12 位 ADC 最大码值 */
#define INA240_GAIN     50.0f   /**< 电流放大器增益 */
#define V_REF           1.65f   /**< 电流采样偏置电压（V） */
#define R_Current       0.006f  /**< 采样电阻（ohm） */
/** @} */

/** @name 调试串口配置 */
/** @{ */
#define CONFIG_UART                          /**< 启用调试串口 */
#define CONFIG_UART_INSTANCE        3U       /**< 调试串口实例 */
#define CONFIG_UART_BAUDRATE        4000000U /**< 调试串口波特率 */
#define CONFIG_UART_RX_BUFFSIZE     128U     /**< 接收环形缓冲大小 */
#define CONFIG_UART_RX_DMA_BUFFSIZE 64U      /**< 接收 DMA 缓冲大小 */
/** @} */

/** @name 母线电压采样配置 */
/** @{ */
#define R_Voaltage_1 20.0f /**< 分压上电阻（kohm） */
#define R_Voaltage_2 2.2f  /**< 分压下电阻（kohm） */
/** @} */

/** @name 数学常量 */
/** @{ */
#define PI              3.14159265358979f
#define _PI             PI
#define _2PI            (2.0f * PI)
#define _PI_2           (PI / 2.0f)
#define _2_PI           (1.0f / (2.0f * PI))
#define _SQRT3          1.732050807568877f
#define _SQRT3_2        0.866025403784439f
#define _1_SQRT3        0.577350269189626f
#define _2_SQRT3        1.154700538379252f
#define FACTOR          (_SQRT3 / PWM_FREQ)
#define PWM_PERIOD      ((170000000.0f / (2.0f * PWM_FREQ)) - 1.0f)
#define PWM_FREQ_PERIOD (PWM_FREQ * PWM_PERIOD)
#define EPS_F           1e-6f
/** @} */

/**
 * @brief 电机故障类型
 */
typedef enum {
    MOTOR_FAULT_NONE,       /**< 无故障 */
    FAULT_OVER_CURRENT,     /**< 过流 */
    FAULT_OVER_VOLTAGE,     /**< 过压 */
    FAULT_UNDER_VOLTAGE,    /**< 欠压 */
    FAULT_OVER_TEMPERATURE, /**< 过温 */
    FAULT_ENCODER,          /**< 编码器故障 */
    FAULT_ENCODER_ANGLE,    /**< 编码器角度超限 */
    FAULT_I_SAMPLING_ERROR  /**< 电流采样故障 */
} MC_FAULT;

#endif /* BOARD_CONFIG_H_ */
