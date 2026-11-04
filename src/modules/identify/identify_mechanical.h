/**
 * @file identify_mechanical.h
 * @brief 机械参数 Ka、Kv、Tc 扫频辨识。
 */

#ifndef IDENTIFY_MECHANICAL_H_
#define IDENTIFY_MECHANICAL_H_

#include <stdbool.h>
#include <stdint.h>

#define MECH_IDENTIFY_WINDOW_SIZE 1024U

/**
 * @brief 机械参数辨识状态
 */
typedef enum {
    MECH_IDENTIFY_IDLE = 0U,       /**< 空闲 */
    MECH_IDENTIFY_SETTLE,          /**< 等待频率点稳定 */
    MECH_IDENTIFY_CAPTURE,         /**< 采集频率响应 */
    MECH_IDENTIFY_RESULT_PENDING,  /**< 等待主循环处理结果 */
    MECH_IDENTIFY_RETURN_POSITION, /**< 返回起始位置 */
    MECH_IDENTIFY_FIT_PENDING,     /**< 等待参数拟合 */
    MECH_IDENTIFY_COMPLETE,        /**< 辨识完成 */
    MECH_IDENTIFY_FAILED           /**< 辨识失败 */
} mech_identify_state_t;

/**
 * @brief 机械参数辨识结果
 */
typedef struct {
    float Ka;                      /**< 惯量等效系数（A*s^2/rad） */
    float Kv;                      /**< 粘滞阻尼等效系数（A*s/rad） */
    float Tc;                      /**< 库仑摩擦等效电流（A） */
    float Ka_high_band;            /**< 高频段惯量估计（A*s^2/rad） */
    float weighted_rms_percent;    /**< 加权拟合相对误差（%） */
    float low_band_rms_percent;    /**< 低频段拟合相对误差（%） */
    float high_band_rms_percent;   /**< 高频段拟合相对误差（%） */
    float median_gain_std_percent; /**< 增益中位标准差（%） */
    float median_phase_std_deg;    /**< 相位中位标准差（deg） */
    uint16_t point_count;          /**< 有效频点数量 */
    uint8_t error_code;            /**< 错误码 */
    bool valid;                    /**< 结果有效标志 */
} mech_identify_result_t;

typedef mech_identify_state_t ident_mech_state_t;
typedef mech_identify_result_t identify_mechanical_result_t;

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
 * @brief 执行机械辨识控制节拍
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

#endif /* IDENTIFY_MECHANICAL_H_ */
