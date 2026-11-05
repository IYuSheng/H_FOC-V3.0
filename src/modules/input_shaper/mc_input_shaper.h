/**
 * @file mc_input_shaper.h
 * @brief 固定采样率离散输入整形器（ZV/ZVD/MZV/EI/2HUMP_EI/3HUMP_EI）。
 *
 * 该实现与参考仓库的 input_shaper 模块保持一致：按控制周期对指令序列做
 * 多脉冲卷积，脉冲时延按采样周期量化。位置环按 POSITION_LOOP_DT 调用。
 */

#ifndef MC_INPUT_SHAPER_H_
#define MC_INPUT_SHAPER_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 支持的输入整形器类型
 */
typedef enum {
    INPUT_SHAPER_TYPE_NONE = 0, /**< 不整形，直通 */
    INPUT_SHAPER_TYPE_ZV,       /**< 零振动 */
    INPUT_SHAPER_TYPE_ZVD,      /**< 零振动零导数 */
    INPUT_SHAPER_TYPE_MZV,      /**< 改进 ZV */
    INPUT_SHAPER_TYPE_EI,       /**< 极不敏感（机械辨识默认） */
    INPUT_SHAPER_TYPE_2HUMP_EI, /**< 双峰 EI */
    INPUT_SHAPER_TYPE_3HUMP_EI  /**< 三峰 EI */
} input_shaper_type_t;

/** @brief 最大脉冲数 */
#define INPUT_SHAPER_MAX_TAPS   5
/** @brief 延迟线最大长度（采样点数） */
#define INPUT_SHAPER_BUFFER_MAX 512

/**
 * @brief 输入整形器实例
 */
typedef struct {
    input_shaper_type_t type;  /**< 整形器类型 */
    float resonance_freq_hz;   /**< 谐振频率（Hz） */
    float damping_ratio;       /**< 阻尼比 */
    float vibration_tolerance; /**< EI 残余振动容限 */

    uint8_t num_taps;                      /**< 有效脉冲数 */
    float amp[INPUT_SHAPER_MAX_TAPS];      /**< 归一化脉冲幅值 */
    uint16_t delay[INPUT_SHAPER_MAX_TAPS]; /**< 量化后的脉冲延迟 */
    uint16_t pre_active_samples;           /**< 延迟质心 */
    uint16_t post_active_samples;          /**< 质心之后的活动采样数 */

    float buffer[INPUT_SHAPER_BUFFER_MAX]; /**< 延迟线 */
    uint16_t buf_idx;                      /**< 当前写索引 */
    uint16_t buf_size;                     /**< 延迟线长度 */

    float last_input; /**< 最近一次输入 */
    bool last_input_valid;
} input_shaper_t;

/**
 * @brief 初始化整形器
 * @param obj 整形器实例
 * @param type 整形器类型
 * @param resonance_freq_hz 谐振频率（Hz）
 * @param damping_ratio 阻尼比
 * @param sample_time_s 调用 input_shaper_process 的固定周期（s）
 * @return 0 成功，-1 参数无效
 */
int8_t input_shaper_init(input_shaper_t *obj, input_shaper_type_t type,
                         float resonance_freq_hz, float damping_ratio,
                         float sample_time_s);

/**
 * @brief 关闭整形，保留直通状态
 * @param obj 整形器实例
 */
void input_shaper_disable(input_shaper_t *obj);

/**
 * @brief 重新配置固定速率离散整形器
 * @param obj 整形器实例
 * @param type 整形器类型
 * @param resonance_freq_hz 谐振频率（Hz）
 * @param damping_ratio 阻尼比
 * @param sample_time_s 固定调用周期（s）
 * @return 0 成功；-1 参数无效、采样率不足或延迟线越界
 */
int8_t input_shaper_config(input_shaper_t *obj, input_shaper_type_t type,
                           float resonance_freq_hz, float damping_ratio,
                           float sample_time_s);

/**
 * @brief 用初始值填充延迟线并复位索引
 * @param obj 整形器实例
 * @param initial_value 初始值
 */
void input_shaper_reset(input_shaper_t *obj, float initial_value);

/**
 * @brief 处理一个采样点
 * @param obj 整形器实例
 * @param input 当前指令采样
 * @return 整形后的指令
 */
float input_shaper_process(input_shaper_t *obj, float input);

/**
 * @brief 查询整形器是否启用
 * @param obj 整形器实例
 * @return true 表示启用
 */
bool input_shaper_is_enabled(const input_shaper_t *obj);

/**
 * @brief 获取延迟质心对应的采样数
 * @param obj 整形器实例
 * @return 采样数
 */
uint16_t input_shaper_get_pre_active_samples(const input_shaper_t *obj);

/**
 * @brief 获取质心之后的活动采样数
 * @param obj 整形器实例
 * @return 采样数
 */
uint16_t input_shaper_get_post_active_samples(const input_shaper_t *obj);

/**
 * @brief 获取最大脉冲延迟
 * @param obj 整形器实例
 * @return 采样数
 */
uint16_t input_shaper_get_max_delay(const input_shaper_t *obj);

/**
 * @brief 获取最近一次输入
 * @param obj 整形器实例
 * @param last_input 输出
 * @return true 表示有效
 */
bool input_shaper_get_last_input(const input_shaper_t *obj, float *last_input);

/**
 * @brief 把阶跃变化拆成离散整形序列
 * @param obj 整形器实例
 * @param start_value 起始值
 * @param end_value 终止值
 * @param targets 各脉冲目标值
 * @param wait_samples 各脉冲之间的等待采样数
 * @return 脉冲数，失败返回 -1
 */
int8_t input_shaper_get_step_sequence(
    const input_shaper_t *obj, float start_value, float end_value,
    float targets[INPUT_SHAPER_MAX_TAPS],
    uint16_t wait_samples[INPUT_SHAPER_MAX_TAPS]);

#ifdef __cplusplus
}
#endif

#endif /* MC_INPUT_SHAPER_H_ */
