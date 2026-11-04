/**
 * @file mc_calibration.h
 * @brief 编码器开环旋转校准与角度线性化接口。
 */

#ifndef MC_CALIBRATION_H_
#define MC_CALIBRATION_H_

#include <stdint.h>

/** 编码器校准扇区数量，200 个整步对应一个机械角周期 */
#define MC_ENCODER_CALIBRATION_SECTOR_COUNT 200U
/** Encoder magnetic periods per mechanical revolution. */
#ifndef MC_ENCODER_CALIBRATION_SENSOR_PERIODS_PER_REV
#define MC_ENCODER_CALIBRATION_SENSOR_PERIODS_PER_REV 1U
#endif
/** Sectors covered by one unique encoder magnetic period. */
#define MC_ENCODER_CALIBRATION_ACTIVE_SECTOR_COUNT                             \
    (MC_ENCODER_CALIBRATION_SECTOR_COUNT /                                     \
     MC_ENCODER_CALIBRATION_SENSOR_PERIODS_PER_REV)
/** Table entries used by one unique encoder magnetic period. */
#define MC_ENCODER_CALIBRATION_ACTIVE_TABLE_SIZE                               \
    (MC_ENCODER_CALIBRATION_ACTIVE_SECTOR_COUNT + 1U)
/** 校准线性表大小，多出的 1 项为跨零点哨兵项 */
#define MC_ENCODER_CALIBRATION_TABLE_SIZE                                      \
    (MC_ENCODER_CALIBRATION_SECTOR_COUNT + 1U)
/** MT6816 14 位编码器一圈计数 */
#define MC_ENCODER_CALIBRATION_ENCODER_CPR 16384U
/** Raw count span of one repeated encoder magnetic period. */
#define MC_ENCODER_CALIBRATION_ENCODER_PERIOD_COUNTS                           \
    (MC_ENCODER_CALIBRATION_ENCODER_CPR /                                      \
     MC_ENCODER_CALIBRATION_SENSOR_PERIODS_PER_REV)
/** 编码器原始角度无效值 */
#define MC_ENCODER_CALIBRATION_INVALID_RAW 0xFFFFU

/**
 * @brief 编码器开环校准状态机状态
 */
typedef enum {
    MC_ENCODER_CALIBRATION_STATE_IDLE = 0, /**< 空闲，首次调用后进入校准 */
    MC_ENCODER_CALIBRATION_STATE_DIR_CHECK_ALIGN, /**< 方向检测前锁定 */
    MC_ENCODER_CALIBRATION_STATE_DIR_CHECK_MOVE,  /**< 小幅移动判断编码器方向 */
    MC_ENCODER_CALIBRATION_STATE_FIND_ZERO_CROSSING, /**< 寻找编码器原始零点跨越
                                                      */
    MC_ENCODER_CALIBRATION_STATE_ALIGN_TO_ZERO, /**< 对齐到最近电角度零点 */
    MC_ENCODER_CALIBRATION_STATE_FORWARD_MOVE,  /**< 正向推进到采样点 */
    MC_ENCODER_CALIBRATION_STATE_FORWARD_SETTLE_SAMPLE, /**< 正向稳定并多次采样
                                                         */
    MC_ENCODER_CALIBRATION_STATE_OVERSHOOT_MOVE, /**< 正向超调以消除换向回差 */
    MC_ENCODER_CALIBRATION_STATE_BACKTRACK_MOVE, /**< 反向回到最后采样点 */
    MC_ENCODER_CALIBRATION_STATE_BACKWARD_MOVE,  /**< 反向推进到采样点 */
    MC_ENCODER_CALIBRATION_STATE_BACKWARD_SETTLE_SAMPLE, /**< 反向稳定并多次采样
                                                          */
    MC_ENCODER_CALIBRATION_STATE_DONE, /**< 校准完成，等待写入 FLASH */
    MC_ENCODER_CALIBRATION_STATE_ERROR /**< 校准失败 */
} mc_encoder_calibration_state_t;

/**
 * @brief 编码器校准错误码
 */
typedef enum {
    MC_ENCODER_CALIBRATION_ERROR_NONE = 0,    /**< 无错误 */
    MC_ENCODER_CALIBRATION_ERROR_RAW_INVALID, /**< 编码器原始值读取失败 */
    MC_ENCODER_CALIBRATION_ERROR_DIRECTION,   /**< 无法判断编码器计数方向 */
    MC_ENCODER_CALIBRATION_ERROR_INTERVAL,    /**< 相邻校准点间隔超出允许范围 */
    MC_ENCODER_CALIBRATION_ERROR_FLASH        /**< FLASH 擦写或校验失败 */
} mc_encoder_calibration_error_t;

/**
 * @brief 编码器校准运行数据
 */
typedef struct {
    volatile uint16_t latest_raw; /**< TIM15 中断最近一次读取到的编码器原始值 */
    volatile uint8_t latest_raw_valid; /**< 最近一次原始值是否有效 */
    volatile uint32_t
        raw_update_count; /**< 原始值更新计数，用于调试采样是否正常 */

    uint16_t
        raw_table[MC_ENCODER_CALIBRATION_SECTOR_COUNT]; /**< 200
                                                           个整步边界原始角度 */
    int32_t linear_table
        [MC_ENCODER_CALIBRATION_TABLE_SIZE]; /**< 单调化后的查找表，含哨兵项 */

    volatile mc_encoder_calibration_state_t state; /**< 当前校准状态 */
    mc_encoder_calibration_error_t error;          /**< 最近一次校准错误码 */
    int8_t raw_direction; /**< 编码器原始值随开环正转变化方向，取值 +1/-1 */
    uint8_t raw_reverse;  /**< 校准内部是否反向处理原始角度，使正转时计数递增 */
    uint8_t calibrated;   /**< 查找表是否已经生成 */
    uint16_t bad_interval_count;     /**< 相邻校准点间隔异常数量 */
    uint16_t first_bad_sector;       /**< 第一个间隔异常的扇区编号 */
    int32_t max_interval_delta;      /**< 最大异常间隔对应的编码器差值 */
    int32_t measured_span_counts;    /**< 采样表实际覆盖的编码器计数范围 */
    int32_t max_nonlinearity_counts; /**< 最大非线性偏差，单位：编码器 count */
    float
        max_nonlinearity_percent; /**< 最大非线性偏差百分比，用于评估磁铁偏心 */

    uint16_t active_sector;      /**< 当前正在校准的扇区编号 */
    uint16_t sample_count;       /**< 已采集的校准点数量或当前点采样次数 */
    uint32_t phase_tick;         /**< 当前状态持续的电流环 tick 数 */
    float command_angle_rad;     /**< 当前开环指令电角度，单位：rad */
    float target_angle_rad;      /**< 当前开环目标电角度，单位：rad */
    float start_angle_rad;       /**< 找零/对齐后的电角度基准，单位：rad */
    uint16_t dir_check_raw;      /**< 方向检测起始原始值 */
    uint16_t prev_processed_raw; /**< 找零时上一次处理后的原始值 */
    int32_t sample_reference;    /**< 当前点环形平均参考值 */
    int32_t sample_accum;        /**< 当前点展开后的采样累加值 */
} mc_encoder_calibration_t;

extern mc_encoder_calibration_t g_encoder_calibration;

/**
 * @brief 初始化校准模块并从 FLASH 加载有效校准表
 */
void mc_encoder_calibration_init(void);

/**
 * @brief 请求启动一次编码器校准
 * @return 1 表示已接受，0 表示校准或 FLASH 保存正在进行
 */
uint8_t mc_encoder_calibration_start(void);

/**
 * @brief 查询编码器校准状态机是否正在运行
 */
uint8_t mc_encoder_calibration_is_running(void);

/**
 * @brief 主循环后台任务，负责将完成的校准结果写入 FLASH
 */
void mc_encoder_calibration_process(void);

/**
 * @brief 复位编码器校准状态和 RAM 校准表
 */
void mc_encoder_calibration_reset(void);

/**
 * @brief 写入最新编码器原始角度
 * @param raw_angle MT6816 原始角度，范围 0~16383，读取失败时传入 0xFFFF
 * @note 该函数设计用于 TIM15 编码器采样中断，校准状态机会在 ADC
 * 电流环中读取该值。
 */
void mc_encoder_calibration_set_raw(uint16_t raw_angle);

/**
 * @brief 执行编码器开环旋转校准状态机
 * @note 该函数应在电流环中周期调用，内部使用开环电角度和 d/q 轴电流闭环完成 200
 * 点采样。
 */
void mc_encoder_calibration_open_loop_calibrate(void);

/**
 * @brief 将编码器原始角度线性化为 FOC 使用角度
 * @param raw_angle MT6816 原始角度，范围 0~16383
 * @param electrical_angle 输出电角度，单位：rad，范围 [0, 2*pi)
 * @param mechanical_angle
 * 输出机械角度，单位：rad；多磁周期编码器下为单个磁周期内角度
 * @return 1 表示转换成功，0 表示未校准或输入无效
 */
uint8_t mc_encoder_calibration_linearize(uint16_t raw_angle,
                                         float *electrical_angle,
                                         float *mechanical_angle);

/**
 * @brief 将编码器原始角度转换为 Q10 电角度索引
 * @param raw_angle MT6816 原始角度，范围 0~16383
 * @param electrical_angle_index 输出电角度索引，范围 0~1023
 * @return 1 表示转换成功，0 表示未校准或输入无效
 */
uint8_t mc_encoder_calibration_get_angle_index(
    uint16_t raw_angle, uint16_t *electrical_angle_index);

/**
 * @brief 使用预计算表快速获取 Q10 电角度索引
 * @param raw_angle MT6816 原始角度，范围 0~16383
 * @param electrical_angle_index 输出电角度索引，范围 0~1023
 * @return 1 表示成功，0 表示快速表不可用或输入无效
 */
uint8_t mc_encoder_calibration_get_angle_index_fast(
    uint16_t raw_angle, uint16_t *electrical_angle_index);

/**
 * @brief 将编码器原始角度转换为机械角度
 * @param raw_angle MT6816 原始角度，范围 0~16383
 * @param mechanical_angle 输出单个编码器磁周期内的机械角度，单位：rad
 * @return 1 表示转换成功，0 表示未校准或输入无效
 */
uint8_t mc_encoder_calibration_get_mechanical_angle(uint16_t raw_angle,
                                                    float *mechanical_angle);

#endif
