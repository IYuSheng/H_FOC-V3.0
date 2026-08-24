#ifndef FOC_ENCODER_CALIBRATION_H
#define FOC_ENCODER_CALIBRATION_H

#include <stdint.h>

#ifndef FOC_ENCODER_CALIBRATION_ENABLE
#define FOC_ENCODER_CALIBRATION_ENABLE 0
#endif

/** 编码器校准扇区数量，200 个整步对应一个机械角周期 */
#define FOC_ENCODER_CALIBRATION_SECTOR_COUNT 200U
/** Encoder magnetic periods per mechanical revolution. */
#ifndef FOC_ENCODER_CALIBRATION_SENSOR_PERIODS_PER_REV
#define FOC_ENCODER_CALIBRATION_SENSOR_PERIODS_PER_REV 1U
#endif
/** Sectors covered by one unique encoder magnetic period. */
#define FOC_ENCODER_CALIBRATION_ACTIVE_SECTOR_COUNT \
    (FOC_ENCODER_CALIBRATION_SECTOR_COUNT / FOC_ENCODER_CALIBRATION_SENSOR_PERIODS_PER_REV)
/** Table entries used by one unique encoder magnetic period. */
#define FOC_ENCODER_CALIBRATION_ACTIVE_TABLE_SIZE \
    (FOC_ENCODER_CALIBRATION_ACTIVE_SECTOR_COUNT + 1U)
/** 校准线性表大小，多出的 1 项为跨零点哨兵项 */
#define FOC_ENCODER_CALIBRATION_TABLE_SIZE   (FOC_ENCODER_CALIBRATION_SECTOR_COUNT + 1U)
/** MT6816 14 位编码器一圈计数 */
#define FOC_ENCODER_CALIBRATION_ENCODER_CPR  16384U
/** Raw count span of one repeated encoder magnetic period. */
#define FOC_ENCODER_CALIBRATION_ENCODER_PERIOD_COUNTS \
    (FOC_ENCODER_CALIBRATION_ENCODER_CPR / FOC_ENCODER_CALIBRATION_SENSOR_PERIODS_PER_REV)
/** 编码器原始角度无效值 */
#define FOC_ENCODER_CALIBRATION_INVALID_RAW  0xFFFFU

/**
 * @brief 是否使用固化的编码器校准表
 * @note 置 1 后，foc_encoder_calibration_linearize() 将使用 foc_encoder_calibration_const_table。
 */
#ifndef FOC_ENCODER_CALIBRATION_USE_CONST_TABLE
#define FOC_ENCODER_CALIBRATION_USE_CONST_TABLE 1
#endif

extern const int32_t foc_encoder_calibration_const_table[FOC_ENCODER_CALIBRATION_TABLE_SIZE];
extern const int8_t foc_encoder_calibration_const_direction;
extern const uint8_t foc_encoder_calibration_const_reverse;

/**
 * @brief 编码器开环校准状态机状态
 */
typedef enum
{
    FOC_ENCODER_CALIBRATION_STATE_IDLE = 0,  /**< 空闲，首次调用后进入校准 */
    FOC_ENCODER_CALIBRATION_STATE_DIR_CHECK_ALIGN, /**< 方向检测前锁定 */
    FOC_ENCODER_CALIBRATION_STATE_DIR_CHECK_MOVE,  /**< 小幅移动判断编码器方向 */
    FOC_ENCODER_CALIBRATION_STATE_FIND_ZERO_CROSSING, /**< 寻找编码器原始零点跨越 */
    FOC_ENCODER_CALIBRATION_STATE_ALIGN_TO_ZERO, /**< 对齐到最近电角度零点 */
    FOC_ENCODER_CALIBRATION_STATE_FORWARD_MOVE,  /**< 正向推进到采样点 */
    FOC_ENCODER_CALIBRATION_STATE_FORWARD_SETTLE_SAMPLE, /**< 正向稳定并多次采样 */
    FOC_ENCODER_CALIBRATION_STATE_OVERSHOOT_MOVE, /**< 正向超调以消除换向回差 */
    FOC_ENCODER_CALIBRATION_STATE_BACKTRACK_MOVE, /**< 反向回到最后采样点 */
    FOC_ENCODER_CALIBRATION_STATE_BACKWARD_MOVE, /**< 反向推进到采样点 */
    FOC_ENCODER_CALIBRATION_STATE_BACKWARD_SETTLE_SAMPLE, /**< 反向稳定并多次采样 */
    FOC_ENCODER_CALIBRATION_STATE_DONE,      /**< 校准完成，等待打印结果 */
    FOC_ENCODER_CALIBRATION_STATE_ERROR      /**< 校准失败 */
} foc_encoder_calibration_state_t;

/**
 * @brief 编码器校准错误码
 */
typedef enum
{
    FOC_ENCODER_CALIBRATION_ERROR_NONE = 0,  /**< 无错误 */
    FOC_ENCODER_CALIBRATION_ERROR_RAW_INVALID, /**< 编码器原始值读取失败 */
    FOC_ENCODER_CALIBRATION_ERROR_DIRECTION, /**< 无法判断编码器计数方向 */
    FOC_ENCODER_CALIBRATION_ERROR_INTERVAL   /**< 相邻校准点间隔超出允许范围 */
} foc_encoder_calibration_error_t;

/**
 * @brief 编码器校准运行数据
 */
typedef struct
{
    volatile uint16_t latest_raw;        /**< TIM15 中断最近一次读取到的编码器原始值 */
    volatile uint8_t latest_raw_valid;   /**< 最近一次原始值是否有效 */
    volatile uint32_t raw_update_count;  /**< 原始值更新计数，用于调试采样是否正常 */

    uint16_t raw_table[FOC_ENCODER_CALIBRATION_SECTOR_COUNT]; /**< 200 个整步边界原始角度 */
    int32_t linear_table[FOC_ENCODER_CALIBRATION_TABLE_SIZE];  /**< 单调化后的查找表，含哨兵项 */

    volatile foc_encoder_calibration_state_t state; /**< 当前校准状态 */
    foc_encoder_calibration_error_t error;          /**< 最近一次校准错误码 */
    int8_t raw_direction;                           /**< 编码器原始值随开环正转变化方向，取值 +1/-1 */
    uint8_t raw_reverse;                            /**< 校准内部是否反向处理原始角度，使正转时计数递增 */
    uint8_t calibrated;                             /**< 查找表是否已经生成 */
    uint16_t bad_interval_count;                    /**< 相邻校准点间隔异常数量 */
    uint16_t first_bad_sector;                      /**< 第一个间隔异常的扇区编号 */
    int32_t max_interval_delta;                     /**< 最大异常间隔对应的编码器差值 */
    int32_t measured_span_counts;                   /**< 采样表实际覆盖的编码器计数范围 */
    int32_t max_nonlinearity_counts;                /**< 最大非线性偏差，单位：编码器 count */
    float max_nonlinearity_percent;                 /**< 最大非线性偏差百分比，用于评估磁铁偏心 */

    uint16_t active_sector;   /**< 当前正在校准的扇区编号 */
    uint16_t sample_count;    /**< 已采集的校准点数量或当前点采样次数 */
    uint32_t phase_tick;      /**< 当前状态持续的电流环 tick 数 */
    float command_angle_rad;  /**< 当前开环指令电角度，单位：rad */
    float target_angle_rad;   /**< 当前开环目标电角度，单位：rad */
    float start_angle_rad;    /**< 找零/对齐后的电角度基准，单位：rad */
    uint16_t dir_check_raw;   /**< 方向检测起始原始值 */
    uint16_t prev_processed_raw; /**< 找零时上一次处理后的原始值 */
    int32_t sample_reference; /**< 当前点环形平均参考值 */
    int32_t sample_accum;     /**< 当前点展开后的采样累加值 */
    uint16_t progress_sector; /**< 当前已经打印到的校准进度扇区 */

    uint8_t start_report_pending; /**< 等待打印开始信息 */
    uint8_t progress_report_pending; /**< 等待打印运行中校准进度 */
    uint8_t done_report_pending;  /**< 等待打印完成信息 */
    uint8_t error_report_pending; /**< 等待打印错误信息 */
    uint8_t quality_report_pending; /**< 等待打印校准质量统计 */
    uint8_t print_pending;        /**< 等待逐点打印校准表 */
    uint16_t print_index;         /**< 当前打印的校准表索引 */
} foc_encoder_calibration_t;

extern foc_encoder_calibration_t g_encoder_calibration;

/**
 * @brief 复位编码器校准状态和 RAM 校准表
 */
void foc_encoder_calibration_reset(void);

/**
 * @brief 写入最新编码器原始角度
 * @param raw_angle MT6816 原始角度，范围 0~16383，读取失败时传入 0xFFFF
 * @note 该函数设计用于 TIM15 编码器采样中断，校准状态机会在 ADC 电流环中读取该值。
 */
void foc_encoder_calibration_set_raw(uint16_t raw_angle);

/**
 * @brief 执行编码器开环旋转校准状态机
 * @note 该函数应在电流环中周期调用，内部使用开环电角度和 d/q 轴电流闭环完成 200 点采样。
 */
void foc_encoder_calibration_open_loop_calibrate(void);

/**
 * @brief 将编码器原始角度线性化为 FOC 使用角度
 * @param raw_angle MT6816 原始角度，范围 0~16383
 * @param electrical_angle 输出电角度，单位：rad，范围 [0, 2*pi)
 * @param mechanical_angle 输出机械角度，单位：rad；多磁周期编码器下为单个磁周期内角度
 * @return 1 表示转换成功，0 表示未校准或输入无效
 */
uint8_t foc_encoder_calibration_linearize(uint16_t raw_angle,
                                          float *electrical_angle,
                                          float *mechanical_angle);

/**
 * @brief 打印编码器校准调试信息
 * @note 校准完成后会逐次打印 200 个校准点，避免在中断中一次性阻塞输出。
 */
void foc_encoder_calibration_debug_log(void);

#endif
