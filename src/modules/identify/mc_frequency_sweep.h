/**
 * @file mc_frequency_sweep.h
 * @brief 机械频率响应扫频与结果输出。
 */

#ifndef MC_FREQUENCY_SWEEP_H_
#define MC_FREQUENCY_SWEEP_H_

#include <stdbool.h>
#include <stdint.h>

#define MC_SWEEP_WINDOW_SIZE 1024U

/**
 * @brief 扫频状态
 */
typedef enum {
    MC_SWEEP_IDLE = 0U,       /**< 空闲 */
    MC_SWEEP_SETTLE,          /**< 等待激励稳定 */
    MC_SWEEP_CAPTURE,         /**< 采集频率响应 */
    MC_SWEEP_RESULT_PENDING,  /**< 等待后台分析 */
    MC_SWEEP_RETURN_POSITION, /**< 返回起始位置 */
    MC_SWEEP_COMPLETE,        /**< 扫频完成 */
    MC_SWEEP_ABORTED          /**< 扫频中止 */
} mc_sweep_state_t;

/**
 * @brief 扫频状态快照
 */
typedef struct {
    mc_sweep_state_t state;     /**< 当前状态 */
    uint16_t frequency_index;   /**< 当前频点索引 */
    float position_error;       /**< 相对起始位置误差（rad） */
    float speed;                /**< 当前机械速度（rad/s） */
    uint32_t return_elapsed_ms; /**< 返回阶段耗时（ms） */
} mc_sweep_status_t;

/**
 * @brief 初始化频率响应扫频
 */
void mc_frequency_sweep_init(void);

/**
 * @brief 启动频率响应扫频
 * @return 1 表示成功，0 表示状态不允许
 */
uint8_t mc_frequency_sweep_start(void);

/**
 * @brief 停止频率响应扫频
 */
void mc_frequency_sweep_stop(void);

/**
 * @brief 查询扫频是否正在运行
 * @return true 表示正在运行
 */
bool mc_frequency_sweep_is_active(void);

/**
 * @brief 查询扫频是否正在返回起始位置
 * @return true 表示正在返回
 */
bool mc_frequency_sweep_is_returning(void);

/**
 * @brief 获取扫频起始机械位置
 * @return 起始位置（rad）
 */
float mc_frequency_sweep_get_start_position(void);

/**
 * @brief 获取扫频电流幅值下限
 * @return 电流幅值（A）
 */
float mc_frequency_sweep_get_iq_amplitude_min(void);

/**
 * @brief 获取扫频电流幅值上限
 * @return 电流幅值（A）
 */
float mc_frequency_sweep_get_iq_amplitude_max(void);

/**
 * @brief 获取当前扫频状态名称
 * @return 状态名称字符串
 */
const char *mc_frequency_sweep_get_state_name(void);

/**
 * @brief 获取扫频状态快照
 * @param status 状态输出
 */
void mc_frequency_sweep_get_status(mc_sweep_status_t *status);

/**
 * @brief 在 5 kHz 外环节拍推进扫频
 */
void mc_frequency_sweep_control_tick(void);

/**
 * @brief 返回位置闭环反馈节拍
 * @param position 机械位置（rad）
 * @param speed 机械速度（rad/s）
 */
void mc_frequency_sweep_return_feedback_tick(float position, float speed);

/**
 * @brief 在主循环处理扫频结果
 */
void mc_frequency_sweep_background_process(void);

#endif /* MC_FREQUENCY_SWEEP_H_ */
