/**
 * @file identify_mechanical.h
 * @brief 机械参数辨识：低频段 Ka/Kv/Tc 拟合 + 高频段谐振扫频。
 */

#ifndef IDENTIFY_MECHANICAL_H_
#define IDENTIFY_MECHANICAL_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 单个频点 FFT/最小二乘窗口长度 */
#define IDENT_MECH_WINDOW_SIZE 1024U
/** @brief 辨识结束返回起始位置时使用的最大速度（rad/s） */
#define IDENT_MECH_RETURN_VELOCITY_RAD_S 10.0f

/**
 * @brief 机械参数辨识状态
 */
typedef enum {
    IDENT_MECH_IDLE = 0U,          /**< 空闲 */
    IDENT_MECH_SETTLE,             /**< 等待频率点稳定 */
    IDENT_MECH_CAPTURE,            /**< 采集频率响应 */
    IDENT_MECH_RETURN_DELAY,       /**< 返回位置前的零电流延时 */
    IDENT_MECH_RESULT_PENDING,     /**< 等待主循环保存频点结果 */
    IDENT_MECH_RETURN_POSITION,    /**< 返回起始位置 */
    IDENT_MECH_FIT_PENDING,        /**< 等待低频参数拟合 */
    IDENT_MECH_RESONANCE_PENDING,  /**< 等待启动谐振扫频 */
    IDENT_MECH_RESONANCE,          /**< 谐振扫频 */
    IDENT_MECH_DONE,               /**< 辨识完成 */
    IDENT_MECH_ERROR               /**< 辨识失败 */
} ident_mech_state_t;

/**
 * @brief 机械参数辨识结果
 *
 * 前馈系数描述克服惯量、粘滞阻尼和库仑摩擦所需的电流；
 * 谐振参数用于配置位置环 EI 输入整形器。
 */
typedef struct {
    float ka;                      /**< 惯量系数 J/Kt（A*s^2/rad） */
    float kv;                      /**< 粘滞阻尼系数 B/Kt（A*s/rad） */
    float tc;                      /**< 库仑摩擦电流前馈（A） */
    float resonance_frequency_hz;  /**< 高频段谐振频率（Hz） */
    float resonance_damping_ratio; /**< EI 整形器阻尼比 */
    float resonance_quality_factor;/**< 谐振品质因数 */
    float resonance_prominence_db; /**< 峰值相对增益基线的突出度（dB） */

    float weighted_rms_percent;    /**< 低频拟合加权相对误差（%） */
    uint8_t low_frequency_error_code; /**< 低频段错误码 */
    uint8_t resonance_error_code;     /**< 谐振段错误码 */
    bool low_frequency_valid;         /**< 低频结果有效 */
    bool resonance_valid;             /**< 谐振结果有效 */
} identify_mechanical_result_t;

/**
 * @brief 初始化机械参数辨识模块
 * @return 0 表示成功，负值表示失败
 */
int8_t identify_mechanical_init(void);

/**
 * @brief 使用默认电流幅值启动机械辨识
 * @return 0 表示成功，负值表示状态不允许
 */
int8_t identify_mechanical_start(void);

/**
 * @brief 使用指定低/高电流幅值启动机械辨识
 * @param low_amplitude_a 低频段电流幅值（A）
 * @param high_amplitude_a 高频段电流幅值（A）
 * @return 0 表示成功，负值表示参数或状态不允许
 */
int8_t identify_mechanical_start_with_amplitudes(float low_amplitude_a,
                                                 float high_amplitude_a);

/**
 * @brief 停止机械参数辨识
 */
void identify_mechanical_stop(void);

/**
 * @brief 结束辨识任务并回到空闲状态，保留最近一次结果供查询
 */
void identify_mechanical_acknowledge(void);

/**
 * @brief 查询机械参数辨识是否正在运行
 * @return true 表示正在运行
 */
bool identify_mechanical_is_active(void);

/**
 * @brief 查询机械辨识是否正在返回起始位置
 * @return true 表示正在返回
 */
bool identify_mechanical_is_returning(void);

/**
 * @brief 获取机械辨识起始位置
 * @return 起始机械位置（rad）
 */
float identify_mechanical_get_start_position(void);

/**
 * @brief 获取机械参数辨识状态
 * @return 当前状态
 */
ident_mech_state_t identify_mechanical_get_state(void);

/**
 * @brief 获取机械参数辨识结果
 * @return 只读结果指针
 */
const identify_mechanical_result_t *identify_mechanical_get_result(void);

/**
 * @brief 执行机械辨识控制节拍（5 kHz）
 */
void identify_mechanical_process(void);

/**
 * @brief 返回位置闭环反馈节拍
 * @param position 机械位置（rad）
 * @param speed 机械速度（rad/s）
 */
void identify_mechanical_return_feedback_tick(float position, float speed);

/**
 * @brief 在主循环处理机械辨识结果
 */
void identify_mechanical_background_process(void);

#ifdef __cplusplus
}
#endif

#endif /* IDENTIFY_MECHANICAL_H_ */
