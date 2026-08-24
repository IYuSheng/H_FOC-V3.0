#include "foc_encoder_calibration.h"

#include "Config.h"
#include "adc.h"
#include "cordic.h"
#include "foc_conversion.h"
#include "tim.h"
#include "usart.h"

#ifndef FOC_ENCODER_CALIBRATION_ENABLE
#define FOC_ENCODER_CALIBRATION_ENABLE 0
#endif

#if FOC_ENCODER_CALIBRATION_ENABLE || FOC_ENCODER_CALIBRATION_USE_CONST_TABLE

#if (FOC_ENCODER_CALIBRATION_SENSOR_PERIODS_PER_REV == 0U)
#error "FOC_ENCODER_CALIBRATION_SENSOR_PERIODS_PER_REV must be greater than 0"
#endif

#if ((FOC_ENCODER_CALIBRATION_SECTOR_COUNT %                                   \
      FOC_ENCODER_CALIBRATION_SENSOR_PERIODS_PER_REV) != 0U)
#error                                                                         \
    "FOC_ENCODER_CALIBRATION_SECTOR_COUNT must be divisible by SENSOR_PERIODS_PER_REV"
#endif

#if ((FOC_ENCODER_CALIBRATION_ENCODER_CPR %                                    \
      FOC_ENCODER_CALIBRATION_SENSOR_PERIODS_PER_REV) != 0U)
#error                                                                         \
    "FOC_ENCODER_CALIBRATION_ENCODER_CPR must be divisible by SENSOR_PERIODS_PER_REV"
#endif

#if ((MOTOR_POLE_PAIRS % FOC_ENCODER_CALIBRATION_SENSOR_PERIODS_PER_REV) != 0U)
#error                                                                         \
    "MOTOR_POLE_PAIRS must be divisible by SENSOR_PERIODS_PER_REV for unambiguous electrical angle"
#endif

/** d 轴校准开环电压，单位：V */
#define ENC_CAL_D_AXIS_VOLTAGE 2.0f
/** q 轴校准开环电压，校准时保持为 0，避免主动产生转矩 */
#define ENC_CAL_Q_AXIS_VOLTAGE 0.0f
/** 初始电角度锁定时间，单位：电流环 tick */
#define ENC_CAL_ALIGN_TICKS ((uint32_t)(0.50f / PWM_PERIOD_S))
/** Per-point settling time before capturing raw angle. */
#define ENC_CAL_SETTLE_TICKS ((uint32_t)(0.02f / PWM_PERIOD_S))
/** 每个校准点平均采样次数 */
#define ENC_CAL_SAMPLES_PER_POINT 8U
/** 方向检测和找零过程中的单步电角度增量，单位：rad */
#define ENC_CAL_ELEC_INC_RAD 0.000872664626f
/** 方向检测移动角度，单位：电角度 */
#define ENC_CAL_DIR_CHECK_RAD ENC_CAL_ELEC_STEP_RAD
/** 寻找零点前的最小预旋转整步数，避免刚启动阶段机械未稳定 */
#define ENC_CAL_ZERO_MIN_STEPS 20U
/** 正向采集结束后的超调整步数，用于消除反向采样回差 */
#define ENC_CAL_OVERSHOOT_STEPS 20U
/** 校准完成后继续保持最后电角度的时间，便于观察电机最终位置 */
#define ENC_CAL_HOLD_TICKS ((uint32_t)(2.00f / PWM_PERIOD_S))
/** 运行中进度打印间隔，单位：扇区 */
#define ENC_CAL_PROGRESS_STEP 20U
/** Unique sectors in one repeated encoder magnetic period. */
#define ENC_CAL_ACTIVE_SECTOR_COUNT FOC_ENCODER_CALIBRATION_ACTIVE_SECTOR_COUNT
/** Unique table entries in one repeated encoder magnetic period. */
#define ENC_CAL_ACTIVE_TABLE_SIZE FOC_ENCODER_CALIBRATION_ACTIVE_TABLE_SIZE
/** Raw count span of one repeated encoder magnetic period. */
#define ENC_CAL_RAW_PERIOD_COUNTS FOC_ENCODER_CALIBRATION_ENCODER_PERIOD_COUNTS
/** Total electrical angle covered by one unique encoder magnetic period. */
#define ENC_CAL_TOTAL_ELEC_RAD                                                 \
    (_2PI * (float)MOTOR_POLE_PAIRS /                                          \
     (float)FOC_ENCODER_CALIBRATION_SENSOR_PERIODS_PER_REV)
/** 每个校准扇区对应的电角度步进，单位：rad */
#define ENC_CAL_ELEC_STEP_RAD                                                  \
    (ENC_CAL_TOTAL_ELEC_RAD / (float)ENC_CAL_ACTIVE_SECTOR_COUNT)
/** 每个校准扇区对应的机械角度步进，单位：rad */
#define ENC_CAL_MECH_STEP_RAD                                                  \
    (_2PI / (float)FOC_ENCODER_CALIBRATION_SECTOR_COUNT)
/** 相邻校准点最小允许间隔，理论值 period_counts/active_sectors，此处允许 -25%
 */
#define ENC_CAL_STEP_MIN_COUNTS                                                \
    ((ENC_CAL_RAW_PERIOD_COUNTS * 3U) / (ENC_CAL_ACTIVE_SECTOR_COUNT * 4U))
/** 相邻校准点最大允许间隔，理论值 period_counts/active_sectors，此处允许 +25%
 */
#define ENC_CAL_STEP_MAX_COUNTS                                                \
    ((ENC_CAL_RAW_PERIOD_COUNTS * 5U +                                         \
      (ENC_CAL_ACTIVE_SECTOR_COUNT * 4U - 1U)) /                               \
     (ENC_CAL_ACTIVE_SECTOR_COUNT * 4U))

foc_encoder_calibration_t g_encoder_calibration = {
    .latest_raw = FOC_ENCODER_CALIBRATION_INVALID_RAW,
    .latest_raw_valid = 0U,
    .state = FOC_ENCODER_CALIBRATION_STATE_IDLE,
    .error = FOC_ENCODER_CALIBRATION_ERROR_NONE,
    .raw_direction = 1,
    .raw_reverse = 0U,
    .first_bad_sector = ENC_CAL_ACTIVE_SECTOR_COUNT,
};

const int32_t
    foc_encoder_calibration_const_table[FOC_ENCODER_CALIBRATION_TABLE_SIZE] = {
        8,     90,    171,   252,   335,   416,   497,   579,   662,   744,
        825,   907,   990,   1072,  1153,  1234,  1317,  1399,  1480,  1562,
        1644,  1726,  1807,  1888,  1971,  2053,  2134,  2215,  2298,  2379,
        2460,  2542,  2625,  2707,  2788,  2870,  2953,  3035,  3118,  3199,
        3283,  3365,  3446,  3528,  3611,  3693,  3774,  3856,  3938,  4020,
        4101,  4183,  4265,  4347,  4427,  4509,  4592,  4674,  4755,  4837,
        4920,  5002,  5084,  5166,  5248,  5331,  5412,  5494,  5577,  5659,
        5740,  5822,  5904,  5986,  6067,  6150,  6232,  6314,  6395,  6477,
        6560,  6641,  6723,  6805,  6888,  6969,  7052,  7134,  7217,  7299,
        7381,  7464,  7546,  7628,  7710,  7792,  7875,  7956,  8038,  8120,
        8202,  8283,  8365,  8447,  8529,  8610,  8692,  8774,  8857,  8938,
        9020,  9103,  9185,  9267,  9349,  9431,  9513,  9594,  9676,  9759,
        9840,  9922,  10003, 10085, 10165, 10247, 10329, 10411, 10493, 10574,
        10656, 10738, 10821, 10902, 10985, 11067, 11150, 11231, 11314, 11396,
        11479, 11560, 11642, 11724, 11807, 11888, 11970, 12051, 12134, 12214,
        12296, 12378, 12460, 12540, 12622, 12704, 12786, 12867, 12949, 13031,
        13114, 13195, 13277, 13360, 13442, 13524, 13606, 13688, 13771, 13853,
        13935, 14017, 14099, 14180, 14262, 14345, 14427, 14509, 14590, 14671,
        14754, 14835, 14918, 15000, 15083, 15164, 15246, 15328, 15411, 15492,
        15574, 15656, 15739, 15820, 15902, 15984, 16066, 16147, 16228, 16310,
        16392};

const int8_t foc_encoder_calibration_const_direction = 1;
const uint8_t foc_encoder_calibration_const_reverse = 1U;

static void enc_cal_openloop_voltage(float electrical_angle_rad,
                                     float voltage_d, float voltage_q);

/**
 * @brief 停止校准输出并将四相 PWM 置于中点占空比
 */
static void enc_cal_output_zero(void)
{
    const uint32_t half_pwm = (uint32_t)(PWM_PERIOD * 0.5f);

    foc_ctrl.target_d = 0.0f;
    foc_ctrl.target_q = 0.0f;
    foc_ctrl.out_d = 0.0f;
    foc_ctrl.out_q = 0.0f;
    alpha_beta.alpha = 0.0f;
    alpha_beta.beta = 0.0f;

    bsp_pwm_set_duty_four_phase(half_pwm, half_pwm, half_pwm, half_pwm);
}

/**
 * @brief 设置校准错误状态
 * @param error 错误码
 */
static void enc_cal_set_error(foc_encoder_calibration_error_t error)
{
    g_encoder_calibration.error = error;
    g_encoder_calibration.state = FOC_ENCODER_CALIBRATION_STATE_ERROR;
    g_encoder_calibration.error_report_pending = 1U;
    g_encoder_calibration.print_pending = 0U;
}

/**
 * @brief 计算编码器环形差值
 * @param now 当前原始角度
 * @param prev 上一次原始角度
 * @return 处理跨零点后的差值，范围约 -8192~8191
 */
static int32_t enc_cal_wrap_delta(uint16_t now, uint16_t prev)
{
    int32_t delta = (int32_t)now - (int32_t)prev;
    const int32_t half_cpr = (int32_t)(ENC_CAL_RAW_PERIOD_COUNTS / 2U);

    if (delta > half_cpr) {
        delta -= (int32_t)ENC_CAL_RAW_PERIOD_COUNTS;
    }
    else if (delta < -half_cpr) {
        delta += (int32_t)ENC_CAL_RAW_PERIOD_COUNTS;
    }

    return delta;
}

static uint16_t enc_cal_normalize_raw_int(int32_t value)
{
    while (value < 0) {
        value += (int32_t)ENC_CAL_RAW_PERIOD_COUNTS;
    }
    while (value >= (int32_t)ENC_CAL_RAW_PERIOD_COUNTS) {
        value -= (int32_t)ENC_CAL_RAW_PERIOD_COUNTS;
    }

    return (uint16_t)value;
}

static uint16_t enc_cal_process_raw_with_reverse(uint16_t raw, uint8_t reverse)
{
    if (reverse != 0U) {
        return (uint16_t)((ENC_CAL_RAW_PERIOD_COUNTS - 1U) - raw);
    }

    return raw;
}

static uint16_t enc_cal_get_processed_raw(uint16_t raw)
{
    return enc_cal_process_raw_with_reverse(raw,
                                            g_encoder_calibration.raw_reverse);
}

static void enc_cal_sample_reset(void)
{
    g_encoder_calibration.sample_count = 0U;
    g_encoder_calibration.sample_reference = 0;
    g_encoder_calibration.sample_accum = 0;
}

static uint8_t enc_cal_sample_processed_average(uint16_t *avg_raw)
{
    uint16_t processed;
    int32_t unwrapped;

    if ((g_encoder_calibration.latest_raw_valid == 0U) ||
        (g_encoder_calibration.latest_raw ==
         FOC_ENCODER_CALIBRATION_INVALID_RAW)) {
        enc_cal_set_error(FOC_ENCODER_CALIBRATION_ERROR_RAW_INVALID);
        return 0U;
    }

    processed = enc_cal_get_processed_raw(g_encoder_calibration.latest_raw);
    if (g_encoder_calibration.sample_count == 0U) {
        g_encoder_calibration.sample_reference = (int32_t)processed;
    }

    unwrapped = g_encoder_calibration.sample_reference +
        enc_cal_wrap_delta(processed,
                           (uint16_t)g_encoder_calibration.sample_reference);
    g_encoder_calibration.sample_accum += unwrapped;
    g_encoder_calibration.sample_count++;

    if (g_encoder_calibration.sample_count >= ENC_CAL_SAMPLES_PER_POINT) {
        *avg_raw = enc_cal_normalize_raw_int(
            g_encoder_calibration.sample_accum /
            (int32_t)g_encoder_calibration.sample_count);
        return 1U;
    }

    return 0U;
}

static uint8_t enc_cal_step_towards_target(void)
{
    if (g_encoder_calibration.command_angle_rad <
        g_encoder_calibration.target_angle_rad) {
        g_encoder_calibration.command_angle_rad += ENC_CAL_ELEC_INC_RAD;
        if (g_encoder_calibration.command_angle_rad >
            g_encoder_calibration.target_angle_rad) {
            g_encoder_calibration.command_angle_rad =
                g_encoder_calibration.target_angle_rad;
        }
    }
    else if (g_encoder_calibration.command_angle_rad >
             g_encoder_calibration.target_angle_rad) {
        g_encoder_calibration.command_angle_rad -= ENC_CAL_ELEC_INC_RAD;
        if (g_encoder_calibration.command_angle_rad <
            g_encoder_calibration.target_angle_rad) {
            g_encoder_calibration.command_angle_rad =
                g_encoder_calibration.target_angle_rad;
        }
    }

    enc_cal_openloop_voltage(g_encoder_calibration.command_angle_rad,
                             ENC_CAL_D_AXIS_VOLTAGE, ENC_CAL_Q_AXIS_VOLTAGE);

    return (g_encoder_calibration.command_angle_rad ==
            g_encoder_calibration.target_angle_rad)
        ? 1U
        : 0U;
}

/**
 * @brief 捕获当前扇区的编码器原始角度
 * @param sector 需要写入的校准扇区编号
 * @return 1 表示捕获成功，0 表示编码器原始值无效
 */
static uint8_t enc_cal_capture_sector(uint16_t sector)
{
    uint16_t avg;

    if (enc_cal_sample_processed_average(&avg) == 0U) {
        return 0U;
    }

    g_encoder_calibration.raw_table[sector] = avg;
    return 1U;
}

/**
 * @brief 检查相邻校准点间隔是否合理
 * @param delta 相邻两点的环形差值
 * @return 1 表示间隔合理，0 表示偏差过大
 */
static uint8_t enc_cal_validate_interval(int32_t delta)
{
    int32_t abs_delta = delta;
    if (abs_delta < 0) {
        abs_delta = -abs_delta;
    }

    return ((abs_delta >= (int32_t)ENC_CAL_STEP_MIN_COUNTS) &&
            (abs_delta <= (int32_t)ENC_CAL_STEP_MAX_COUNTS));
}

/**
 * @brief 根据采集到的一个传感器磁周期原始点生成单调查找表
 * @return 1 表示生成成功，0 表示方向或间隔检查失败
 */
static uint8_t enc_cal_finalize_table(void)
{
    int32_t delta_sum = 0;
    float max_nonlinearity = 0.0f;
    // 判断编码器方向
    for (uint16_t i = 1U; i < ENC_CAL_ACTIVE_SECTOR_COUNT; i++) {
        delta_sum +=
            enc_cal_wrap_delta(g_encoder_calibration.raw_table[i],
                               g_encoder_calibration.raw_table[i - 1U]);
    }

    if (delta_sum > 0) {
        g_encoder_calibration.raw_direction = 1;
    }
    else if (delta_sum < 0) {
        g_encoder_calibration.raw_direction = -1;
    }
    else {
        g_encoder_calibration.raw_direction = 1;
    }
    g_encoder_calibration.bad_interval_count = 0U;
    g_encoder_calibration.first_bad_sector = ENC_CAL_ACTIVE_SECTOR_COUNT;
    g_encoder_calibration.max_interval_delta = 0;
    g_encoder_calibration.linear_table[0] =
        (int32_t)g_encoder_calibration.raw_table[0];
    // 逐段生成linear表
    for (uint16_t i = 1U; i < ENC_CAL_ACTIVE_SECTOR_COUNT; i++) {
        int32_t delta =
            enc_cal_wrap_delta(g_encoder_calibration.raw_table[i],
                               g_encoder_calibration.raw_table[i - 1U]);
        int32_t abs_delta = delta;

        if (abs_delta < 0) {
            abs_delta = -abs_delta;
        }
        if (abs_delta > g_encoder_calibration.max_interval_delta) {
            g_encoder_calibration.max_interval_delta = abs_delta;
        }
        if ((delta * (int32_t)g_encoder_calibration.raw_direction <= 0) ||
            (enc_cal_validate_interval(delta) == 0U)) {
            if (g_encoder_calibration.bad_interval_count == 0U) {
                g_encoder_calibration.first_bad_sector = i;
            }
            g_encoder_calibration.bad_interval_count++;
        }

        g_encoder_calibration.linear_table[i] =
            g_encoder_calibration.linear_table[i - 1U] + delta;
    }

    int32_t end_delta = enc_cal_wrap_delta(
        g_encoder_calibration.raw_table[0],
        g_encoder_calibration.raw_table[ENC_CAL_ACTIVE_SECTOR_COUNT - 1U]);
    int32_t abs_end_delta = end_delta;
    if (abs_end_delta < 0) {
        abs_end_delta = -abs_end_delta;
    }
    if (abs_end_delta > g_encoder_calibration.max_interval_delta) {
        g_encoder_calibration.max_interval_delta = abs_end_delta;
    }
    if ((end_delta * (int32_t)g_encoder_calibration.raw_direction <= 0) ||
        (enc_cal_validate_interval(end_delta) == 0U)) {
        if (g_encoder_calibration.bad_interval_count == 0U) {
            g_encoder_calibration.first_bad_sector =
                ENC_CAL_ACTIVE_SECTOR_COUNT;
        }
        g_encoder_calibration.bad_interval_count++;
    }

    g_encoder_calibration.measured_span_counts = delta_sum + end_delta;
    if (g_encoder_calibration.measured_span_counts < 0) {
        g_encoder_calibration.measured_span_counts =
            -g_encoder_calibration.measured_span_counts;
    }

    if ((g_encoder_calibration.bad_interval_count != 0U) ||
        (g_encoder_calibration.measured_span_counts <
         (int32_t)((ENC_CAL_RAW_PERIOD_COUNTS * 9U) / 10U)) ||
        (g_encoder_calibration.measured_span_counts >
         (int32_t)((ENC_CAL_RAW_PERIOD_COUNTS * 11U) / 10U))) {
        enc_cal_set_error(FOC_ENCODER_CALIBRATION_ERROR_INTERVAL);
        g_encoder_calibration.quality_report_pending = 1U;
        return 0U;
    }

    g_encoder_calibration.linear_table[ENC_CAL_ACTIVE_SECTOR_COUNT] =
        g_encoder_calibration.linear_table[0] +
        (int32_t)g_encoder_calibration.raw_direction *
            (int32_t)ENC_CAL_RAW_PERIOD_COUNTS;

    for (uint16_t i = 0U; i < ENC_CAL_ACTIVE_SECTOR_COUNT; i++) {
        float ideal = (float)g_encoder_calibration.linear_table[0] +
            (float)g_encoder_calibration.raw_direction *
                (float)ENC_CAL_RAW_PERIOD_COUNTS * (float)i /
                (float)ENC_CAL_ACTIVE_SECTOR_COUNT;
        float error = (float)g_encoder_calibration.linear_table[i] - ideal;

        if (error < 0.0f) {
            error = -error;
        }
        if (error > max_nonlinearity) {
            max_nonlinearity = error;
        }
    }

    g_encoder_calibration.max_nonlinearity_counts =
        (int32_t)(max_nonlinearity + 0.5f);
    g_encoder_calibration.max_nonlinearity_percent =
        max_nonlinearity * 100.0f / (float)ENC_CAL_RAW_PERIOD_COUNTS;

    g_encoder_calibration.error = FOC_ENCODER_CALIBRATION_ERROR_NONE;
    g_encoder_calibration.calibrated = 1U;
    g_encoder_calibration.done_report_pending = 1U;
    g_encoder_calibration.print_pending = 1U;
    g_encoder_calibration.quality_report_pending = 1U;
    g_encoder_calibration.print_index = 0U;
    return 1U;
}

/**
 * @brief 使用开环电角度直接输出 d/q 轴电压
 * @param electrical_angle_rad 开环电角度指令，单位：rad
 * @param voltage_d d 轴电压指令，单位：V
 * @param voltage_q q 轴电压指令，单位：V
 */
static void enc_cal_openloop_voltage(float electrical_angle_rad,
                                     float voltage_d, float voltage_q)
{
    float sin_theta;
    float cos_theta;

    if (foc_voltage_data.vbus <= EPS_F) {
        enc_cal_output_zero();
        return;
    }

    foc_ctrl.angle = angle_normalize(electrical_angle_rad);
    foc_ctrl.target_d = 0.0f;
    foc_ctrl.target_q = 0.0f;
    foc_ctrl.out_d = voltage_d;
    foc_ctrl.out_q = voltage_q;

    CORDIC_SinCos_Rad(foc_ctrl.angle, &sin_theta, &cos_theta);
    inv_park_transform_f32(&foc_ctrl, &alpha_beta, sin_theta, cos_theta);
    spwm_duty_calc(&spwm, &alpha_beta, foc_voltage_data.vbus);
    bsp_pwm_set_duty_four_phase(spwm.duty_Ap, spwm.duty_An, spwm.duty_Bp,
                                spwm.duty_Bn);
}

/**
 * @brief 复位编码器校准状态和 RAM 校准表
 */
void foc_encoder_calibration_reset(void)
{
    g_encoder_calibration.latest_raw = FOC_ENCODER_CALIBRATION_INVALID_RAW;
    g_encoder_calibration.latest_raw_valid = 0U;
    g_encoder_calibration.raw_update_count = 0U;
    g_encoder_calibration.state = FOC_ENCODER_CALIBRATION_STATE_IDLE;
    g_encoder_calibration.error = FOC_ENCODER_CALIBRATION_ERROR_NONE;
    g_encoder_calibration.raw_direction = 1;
    g_encoder_calibration.raw_reverse = 0U;
    g_encoder_calibration.calibrated = 0U;
    g_encoder_calibration.bad_interval_count = 0U;
    g_encoder_calibration.first_bad_sector = ENC_CAL_ACTIVE_SECTOR_COUNT;
    g_encoder_calibration.max_interval_delta = 0;
    g_encoder_calibration.measured_span_counts = 0;
    g_encoder_calibration.max_nonlinearity_counts = 0;
    g_encoder_calibration.max_nonlinearity_percent = 0.0f;
    g_encoder_calibration.active_sector = 0U;
    g_encoder_calibration.sample_count = 0U;
    g_encoder_calibration.phase_tick = 0U;
    g_encoder_calibration.command_angle_rad = 0.0f;
    g_encoder_calibration.target_angle_rad = 0.0f;
    g_encoder_calibration.start_angle_rad = 0.0f;
    g_encoder_calibration.dir_check_raw = 0U;
    g_encoder_calibration.prev_processed_raw = 0U;
    g_encoder_calibration.sample_reference = 0;
    g_encoder_calibration.sample_accum = 0;
    g_encoder_calibration.progress_sector = 0U;
    g_encoder_calibration.start_report_pending = 0U;
    g_encoder_calibration.progress_report_pending = 0U;
    g_encoder_calibration.done_report_pending = 0U;
    g_encoder_calibration.error_report_pending = 0U;
    g_encoder_calibration.quality_report_pending = 0U;
    g_encoder_calibration.print_pending = 0U;
    g_encoder_calibration.print_index = 0U;
}

/**
 * @brief 写入最新编码器原始角度
 * @param raw_angle MT6816 原始角度，范围 0~16383，读取失败时传入 0xFFFF
 */
void foc_encoder_calibration_set_raw(uint16_t raw_angle)
{
    if (raw_angle == FOC_ENCODER_CALIBRATION_INVALID_RAW) {
        g_encoder_calibration.latest_raw_valid = 0U;
        g_encoder_calibration.latest_raw = raw_angle;
        return;
    }

    g_encoder_calibration.latest_raw = raw_angle;
    g_encoder_calibration.latest_raw_valid = 1U;
    g_encoder_calibration.raw_update_count++;
}

/**
 * @brief 执行编码器开环旋转校准状态机
 * @note 该函数在 ADC 电流环中调用，依次完成锁定、移动、稳定、采样和查找表生成。
 */
void foc_encoder_calibration_open_loop_calibrate(void)
{
    switch (g_encoder_calibration.state) {
        case FOC_ENCODER_CALIBRATION_STATE_IDLE:
            g_encoder_calibration.error = FOC_ENCODER_CALIBRATION_ERROR_NONE;
            g_encoder_calibration.raw_direction = 1;
            g_encoder_calibration.raw_reverse = 0U;
            g_encoder_calibration.calibrated = 0U;
            g_encoder_calibration.bad_interval_count = 0U;
            g_encoder_calibration.first_bad_sector =
                ENC_CAL_ACTIVE_SECTOR_COUNT;
            g_encoder_calibration.max_interval_delta = 0;
            g_encoder_calibration.measured_span_counts = 0;
            g_encoder_calibration.max_nonlinearity_counts = 0;
            g_encoder_calibration.max_nonlinearity_percent = 0.0f;
            g_encoder_calibration.active_sector = 0U;
            g_encoder_calibration.sample_count = 0U;
            g_encoder_calibration.phase_tick = 0U;
            g_encoder_calibration.command_angle_rad = -ENC_CAL_ELEC_STEP_RAD;
            g_encoder_calibration.target_angle_rad = -ENC_CAL_ELEC_STEP_RAD;
            g_encoder_calibration.start_angle_rad = 0.0f;
            g_encoder_calibration.dir_check_raw = 0U;
            g_encoder_calibration.prev_processed_raw = 0U;
            g_encoder_calibration.sample_reference = 0;
            g_encoder_calibration.sample_accum = 0;
            g_encoder_calibration.progress_sector = 0U;
            g_encoder_calibration.start_report_pending = 1U;
            g_encoder_calibration.progress_report_pending = 0U;
            g_encoder_calibration.done_report_pending = 0U;
            g_encoder_calibration.error_report_pending = 0U;
            g_encoder_calibration.quality_report_pending = 0U;
            g_encoder_calibration.print_pending = 0U;
            g_encoder_calibration.print_index = 0U;
            g_encoder_calibration.state =
                FOC_ENCODER_CALIBRATION_STATE_DIR_CHECK_ALIGN;
            break;

        case FOC_ENCODER_CALIBRATION_STATE_DIR_CHECK_ALIGN:
            enc_cal_openloop_voltage(g_encoder_calibration.command_angle_rad,
                                     ENC_CAL_D_AXIS_VOLTAGE,
                                     ENC_CAL_Q_AXIS_VOLTAGE);
            if (++g_encoder_calibration.phase_tick >= ENC_CAL_ALIGN_TICKS) {
                if ((g_encoder_calibration.latest_raw_valid == 0U) ||
                    (g_encoder_calibration.latest_raw ==
                     FOC_ENCODER_CALIBRATION_INVALID_RAW)) {
                    enc_cal_set_error(
                        FOC_ENCODER_CALIBRATION_ERROR_RAW_INVALID);
                    break;
                }

                g_encoder_calibration.dir_check_raw =
                    g_encoder_calibration.latest_raw;
                g_encoder_calibration.target_angle_rad =
                    g_encoder_calibration.command_angle_rad +
                    ENC_CAL_DIR_CHECK_RAD;
                g_encoder_calibration.phase_tick = 0U;
                g_encoder_calibration.state =
                    FOC_ENCODER_CALIBRATION_STATE_DIR_CHECK_MOVE;
            }
            break;

        case FOC_ENCODER_CALIBRATION_STATE_DIR_CHECK_MOVE:
            if (enc_cal_step_towards_target() == 0U) {
                break;
            }

            if (++g_encoder_calibration.phase_tick >= ENC_CAL_SETTLE_TICKS) {
                int32_t diff;

                if ((g_encoder_calibration.latest_raw_valid == 0U) ||
                    (g_encoder_calibration.latest_raw ==
                     FOC_ENCODER_CALIBRATION_INVALID_RAW)) {
                    enc_cal_set_error(
                        FOC_ENCODER_CALIBRATION_ERROR_RAW_INVALID);
                    break;
                }

                diff = enc_cal_wrap_delta(g_encoder_calibration.latest_raw,
                                          g_encoder_calibration.dir_check_raw);
                if (diff > 0) {
                    g_encoder_calibration.raw_direction = 1;
                    g_encoder_calibration.raw_reverse = 0U;
                }
                else if (diff < 0) {
                    g_encoder_calibration.raw_direction = -1;
                    g_encoder_calibration.raw_reverse = 1U;
                }
                else {
                    enc_cal_set_error(FOC_ENCODER_CALIBRATION_ERROR_DIRECTION);
                    break;
                }

                g_encoder_calibration.start_angle_rad =
                    g_encoder_calibration.command_angle_rad;
                g_encoder_calibration.prev_processed_raw =
                    enc_cal_get_processed_raw(g_encoder_calibration.latest_raw);
                g_encoder_calibration.phase_tick = 0U;
                g_encoder_calibration.state =
                    FOC_ENCODER_CALIBRATION_STATE_FIND_ZERO_CROSSING;
            }
            break;

        case FOC_ENCODER_CALIBRATION_STATE_FIND_ZERO_CROSSING: {
            uint16_t current_raw;
            uint16_t prev_raw;
            float moved_rad;

            g_encoder_calibration.command_angle_rad += ENC_CAL_ELEC_INC_RAD;
            enc_cal_openloop_voltage(g_encoder_calibration.command_angle_rad,
                                     ENC_CAL_D_AXIS_VOLTAGE,
                                     ENC_CAL_Q_AXIS_VOLTAGE);

            if ((g_encoder_calibration.latest_raw_valid == 0U) ||
                (g_encoder_calibration.latest_raw ==
                 FOC_ENCODER_CALIBRATION_INVALID_RAW)) {
                enc_cal_set_error(FOC_ENCODER_CALIBRATION_ERROR_RAW_INVALID);
                break;
            }

            current_raw =
                enc_cal_get_processed_raw(g_encoder_calibration.latest_raw);
            prev_raw = g_encoder_calibration.prev_processed_raw;
            moved_rad = g_encoder_calibration.command_angle_rad -
                g_encoder_calibration.start_angle_rad;
            g_encoder_calibration.prev_processed_raw = current_raw;

            if ((moved_rad >
                 ((float)ENC_CAL_ZERO_MIN_STEPS * ENC_CAL_ELEC_STEP_RAD)) &&
                (prev_raw >
                 (uint16_t)((ENC_CAL_RAW_PERIOD_COUNTS * 3U) / 4U)) &&
                (current_raw < (uint16_t)(ENC_CAL_RAW_PERIOD_COUNTS / 4U))) {
                float nearest_step =
                    (float)((int32_t)((g_encoder_calibration.command_angle_rad /
                                       ENC_CAL_ELEC_STEP_RAD) +
                                      0.5f));
                g_encoder_calibration.target_angle_rad =
                    nearest_step * ENC_CAL_ELEC_STEP_RAD;
                g_encoder_calibration.phase_tick = 0U;
                g_encoder_calibration.state =
                    FOC_ENCODER_CALIBRATION_STATE_ALIGN_TO_ZERO;
            }
            break;
        }

        case FOC_ENCODER_CALIBRATION_STATE_ALIGN_TO_ZERO:
            if (enc_cal_step_towards_target() == 0U) {
                g_encoder_calibration.phase_tick = 0U;
                break;
            }

            if (++g_encoder_calibration.phase_tick >= ENC_CAL_SETTLE_TICKS) {
                g_encoder_calibration.start_angle_rad =
                    g_encoder_calibration.target_angle_rad;
                g_encoder_calibration.active_sector = 0U;
                g_encoder_calibration.phase_tick = 0U;
                g_encoder_calibration.target_angle_rad =
                    g_encoder_calibration.start_angle_rad;
                g_encoder_calibration.state =
                    FOC_ENCODER_CALIBRATION_STATE_FORWARD_MOVE;
            }
            break;

        case FOC_ENCODER_CALIBRATION_STATE_FORWARD_MOVE:
            if (g_encoder_calibration.active_sector >=
                ENC_CAL_ACTIVE_SECTOR_COUNT) {
                g_encoder_calibration.target_angle_rad =
                    g_encoder_calibration.start_angle_rad +
                    (float)(ENC_CAL_ACTIVE_SECTOR_COUNT +
                            ENC_CAL_OVERSHOOT_STEPS) *
                        ENC_CAL_ELEC_STEP_RAD;
                g_encoder_calibration.phase_tick = 0U;
                g_encoder_calibration.state =
                    FOC_ENCODER_CALIBRATION_STATE_OVERSHOOT_MOVE;
                break;
            }

            g_encoder_calibration.target_angle_rad =
                g_encoder_calibration.start_angle_rad +
                (float)g_encoder_calibration.active_sector *
                    ENC_CAL_ELEC_STEP_RAD;
            if (enc_cal_step_towards_target() != 0U) {
                enc_cal_sample_reset();
                g_encoder_calibration.phase_tick = 0U;
                g_encoder_calibration.state =
                    FOC_ENCODER_CALIBRATION_STATE_FORWARD_SETTLE_SAMPLE;
            }
            break;

        case FOC_ENCODER_CALIBRATION_STATE_FORWARD_SETTLE_SAMPLE:
            enc_cal_openloop_voltage(g_encoder_calibration.command_angle_rad,
                                     ENC_CAL_D_AXIS_VOLTAGE,
                                     ENC_CAL_Q_AXIS_VOLTAGE);
            if (++g_encoder_calibration.phase_tick >= ENC_CAL_SETTLE_TICKS) {
                if (enc_cal_capture_sector(
                        g_encoder_calibration.active_sector) == 0U) {
                    break;
                }

                if (g_encoder_calibration.sample_count >=
                    ENC_CAL_SAMPLES_PER_POINT) {
                    g_encoder_calibration.progress_sector =
                        g_encoder_calibration.active_sector;
                    g_encoder_calibration.progress_report_pending = 1U;
                    g_encoder_calibration.active_sector++;
                    g_encoder_calibration.phase_tick = 0U;
                    g_encoder_calibration.state =
                        FOC_ENCODER_CALIBRATION_STATE_FORWARD_MOVE;
                }
            }
            break;

        case FOC_ENCODER_CALIBRATION_STATE_OVERSHOOT_MOVE:
            if (enc_cal_step_towards_target() == 0U) {
                g_encoder_calibration.phase_tick = 0U;
                break;
            }

            if (++g_encoder_calibration.phase_tick >= ENC_CAL_SETTLE_TICKS) {
                g_encoder_calibration.active_sector =
                    ENC_CAL_ACTIVE_SECTOR_COUNT - 1U;
                g_encoder_calibration.target_angle_rad =
                    g_encoder_calibration.start_angle_rad +
                    (float)ENC_CAL_ACTIVE_SECTOR_COUNT * ENC_CAL_ELEC_STEP_RAD;
                g_encoder_calibration.phase_tick = 0U;
                g_encoder_calibration.state =
                    FOC_ENCODER_CALIBRATION_STATE_BACKTRACK_MOVE;
            }
            break;

        case FOC_ENCODER_CALIBRATION_STATE_BACKTRACK_MOVE:
            if (enc_cal_step_towards_target() == 0U) {
                g_encoder_calibration.phase_tick = 0U;
                break;
            }

            if (++g_encoder_calibration.phase_tick >= ENC_CAL_SETTLE_TICKS) {
                g_encoder_calibration.phase_tick = 0U;
                g_encoder_calibration.state =
                    FOC_ENCODER_CALIBRATION_STATE_BACKWARD_MOVE;
            }
            break;

        case FOC_ENCODER_CALIBRATION_STATE_BACKWARD_MOVE:
            g_encoder_calibration.target_angle_rad =
                g_encoder_calibration.start_angle_rad +
                (float)g_encoder_calibration.active_sector *
                    ENC_CAL_ELEC_STEP_RAD;
            if (enc_cal_step_towards_target() != 0U) {
                enc_cal_sample_reset();
                g_encoder_calibration.phase_tick = 0U;
                g_encoder_calibration.state =
                    FOC_ENCODER_CALIBRATION_STATE_BACKWARD_SETTLE_SAMPLE;
            }
            break;

        case FOC_ENCODER_CALIBRATION_STATE_BACKWARD_SETTLE_SAMPLE:
            enc_cal_openloop_voltage(g_encoder_calibration.command_angle_rad,
                                     ENC_CAL_D_AXIS_VOLTAGE,
                                     ENC_CAL_Q_AXIS_VOLTAGE);
            if (++g_encoder_calibration.phase_tick >= ENC_CAL_SETTLE_TICKS) {
                uint16_t avg;

                if (enc_cal_sample_processed_average(&avg) == 0U) {
                    break;
                }

                if (g_encoder_calibration.sample_count >=
                    ENC_CAL_SAMPLES_PER_POINT) {
                    uint16_t fwd =
                        g_encoder_calibration
                            .raw_table[g_encoder_calibration.active_sector];
                    int32_t diff = enc_cal_wrap_delta(avg, fwd);
                    int32_t bwd_unwrapped = (int32_t)fwd + diff;
                    int32_t result = ((int32_t)fwd + bwd_unwrapped) / 2;

                    g_encoder_calibration
                        .raw_table[g_encoder_calibration.active_sector] =
                        enc_cal_normalize_raw_int(result);
                    g_encoder_calibration.progress_sector =
                        g_encoder_calibration.active_sector;
                    g_encoder_calibration.progress_report_pending = 1U;
                    g_encoder_calibration.phase_tick = 0U;

                    if (g_encoder_calibration.active_sector == 0U) {
                        if (enc_cal_finalize_table() != 0U) {
                            g_encoder_calibration.state =
                                FOC_ENCODER_CALIBRATION_STATE_DONE;
                        }
                    }
                    else {
                        g_encoder_calibration.active_sector--;
                        g_encoder_calibration.state =
                            FOC_ENCODER_CALIBRATION_STATE_BACKWARD_MOVE;
                    }
                }
            }
            break;

        case FOC_ENCODER_CALIBRATION_STATE_DONE:
            enc_cal_openloop_voltage(g_encoder_calibration.command_angle_rad,
                                     ENC_CAL_D_AXIS_VOLTAGE,
                                     ENC_CAL_Q_AXIS_VOLTAGE);
            if (++g_encoder_calibration.phase_tick >= ENC_CAL_HOLD_TICKS) {
                enc_cal_output_zero();
            }
            break;

        case FOC_ENCODER_CALIBRATION_STATE_ERROR:
        default:
            enc_cal_output_zero();
            break;
    }
}

/**
 * @brief 将编码器原始角度线性化为 FOC 使用角度
 * @param raw_angle MT6816 原始角度，范围 0~16383
 * @param electrical_angle 输出电角度，单位：rad，范围 [0, 2*pi)
 * @param mechanical_angle
 * 输出机械角度，单位：rad；多磁周期编码器下为单个磁周期内角度
 * @return 1 表示转换成功，0 表示未校准或输入无效
 */
uint8_t foc_encoder_calibration_linearize(uint16_t raw_angle,
                                          float *electrical_angle,
                                          float *mechanical_angle)
{
    const int32_t *table;
    int8_t direction;
    uint8_t table_ready;
    int32_t input;
    uint16_t low = 0U;
    uint16_t high = ENC_CAL_ACTIVE_SECTOR_COUNT;
    int32_t a;
    int32_t b;
    float fraction;
    float sector_position;

#if FOC_ENCODER_CALIBRATION_USE_CONST_TABLE
    table = foc_encoder_calibration_const_table;
    direction = foc_encoder_calibration_const_direction;
    table_ready = 1U;
    input = (int32_t)enc_cal_process_raw_with_reverse(
        raw_angle, foc_encoder_calibration_const_reverse);
#else
    table = g_encoder_calibration.linear_table;
    direction = g_encoder_calibration.raw_direction;
    table_ready = g_encoder_calibration.calibrated;
    input = (int32_t)enc_cal_get_processed_raw(raw_angle);
#endif

    if ((table_ready == 0U) ||
        (raw_angle == FOC_ENCODER_CALIBRATION_INVALID_RAW) ||
        (electrical_angle == 0) || (mechanical_angle == 0)) {
        return 0U;
    }

    if (direction >= 0) {
        while (input < table[0]) {
            input += (int32_t)ENC_CAL_RAW_PERIOD_COUNTS;
        }
        while (input >= table[0] + (int32_t)ENC_CAL_RAW_PERIOD_COUNTS) {
            input -= (int32_t)ENC_CAL_RAW_PERIOD_COUNTS;
        }

        while ((high - low) > 1U) {
            uint16_t mid = (uint16_t)((low + high) >> 1U);
            if (input >= table[mid]) {
                low = mid;
            }
            else {
                high = mid;
            }
        }
    }
    else {
        while (input > table[0]) {
            input -= (int32_t)ENC_CAL_RAW_PERIOD_COUNTS;
        }
        while (input <= table[0] - (int32_t)ENC_CAL_RAW_PERIOD_COUNTS) {
            input += (int32_t)ENC_CAL_RAW_PERIOD_COUNTS;
        }

        while ((high - low) > 1U) {
            uint16_t mid = (uint16_t)((low + high) >> 1U);
            if (input <= table[mid]) {
                low = mid;
            }
            else {
                high = mid;
            }
        }
    }

    a = table[low];
    b = table[low + 1U];
    if (a == b) {
        return 0U;
    }

    fraction = (float)(input - a) / (float)(b - a);
    if (fraction < 0.0f) {
        fraction = 0.0f;
    }
    else if (fraction > 1.0f) {
        fraction = 1.0f;
    }

    sector_position = (float)low + fraction;
    *mechanical_angle = sector_position * ENC_CAL_MECH_STEP_RAD;
    *electrical_angle =
        angle_normalize(*mechanical_angle * MOTOR_POLE_PAIRS);

    return 1U;
}
/**
 * @brief 打印编码器校准调试信息
 * @note 每次调用最多打印一行，适合放在 1kHz 调试任务中分批输出。
 */
void foc_encoder_calibration_debug_log(void)
{
    if (g_encoder_calibration.start_report_pending != 0U) {
        g_encoder_calibration.start_report_pending = 0U;
        debug_log("[ENC_CAL] start: sectors=%u sensor_periods=%u pole_pairs=%u "
                  "capture_step_rad=%.3f inc_rad=%.3f settle_ticks=%lu "
                  "total_elec_rad=%.1f "
                  "d_voltage=%.3f",
                  ENC_CAL_ACTIVE_SECTOR_COUNT,
                  FOC_ENCODER_CALIBRATION_SENSOR_PERIODS_PER_REV,
                  MOTOR_POLE_PAIRS, ENC_CAL_ELEC_STEP_RAD, ENC_CAL_ELEC_INC_RAD,
                  (unsigned long)ENC_CAL_SETTLE_TICKS,
                  ENC_CAL_ELEC_STEP_RAD * (float)ENC_CAL_ACTIVE_SECTOR_COUNT,
                  ENC_CAL_D_AXIS_VOLTAGE);
        return;
    }

    if (g_encoder_calibration.progress_report_pending != 0U) {
        g_encoder_calibration.progress_report_pending = 0U;
        debug_log("%03u, %u", g_encoder_calibration.progress_sector,
                  g_encoder_calibration
                      .raw_table[g_encoder_calibration.progress_sector]);
        return;
    }

    if (g_encoder_calibration.error_report_pending != 0U) {
        g_encoder_calibration.error_report_pending = 0U;
        debug_log("[ENC_CAL] ERROR: code=%u sector=%u raw=%u",
                  (uint8_t)g_encoder_calibration.error,
                  g_encoder_calibration.active_sector,
                  g_encoder_calibration.latest_raw);
        return;
    }

    if (g_encoder_calibration.done_report_pending != 0U) {
        g_encoder_calibration.done_report_pending = 0U;
        debug_log("[ENC_CAL] done: dir=%d reverse=%u",
                  g_encoder_calibration.raw_direction,
                  g_encoder_calibration.raw_reverse);
        return;
    }

    if (g_encoder_calibration.print_pending != 0U) {
        if (g_encoder_calibration.print_index < ENC_CAL_ACTIVE_TABLE_SIZE) {
            uint16_t index = g_encoder_calibration.print_index;
            debug_log("%d,", (int)g_encoder_calibration.linear_table[index]);
            g_encoder_calibration.print_index++;
            return;
        }

        g_encoder_calibration.print_pending = 0U;
        return;
    }

    if (g_encoder_calibration.quality_report_pending != 0U) {
        g_encoder_calibration.quality_report_pending = 0U;
        debug_log("[ENC_CAL] quality: bad=%u first=%u span=%d max_delta=%d "
                  "ecc=%.3f%% max_err=%d",
                  g_encoder_calibration.bad_interval_count,
                  g_encoder_calibration.first_bad_sector,
                  (int)g_encoder_calibration.measured_span_counts,
                  (int)g_encoder_calibration.max_interval_delta,
                  g_encoder_calibration.max_nonlinearity_percent,
                  (int)g_encoder_calibration.max_nonlinearity_counts);
        return;
    }
}

#else

/**
 * @brief 校准功能关闭时的空复位接口
 */
void foc_encoder_calibration_reset(void)
{
}

/**
 * @brief 校准功能关闭时的空原始角度写入接口
 * @param raw_angle MT6816 原始角度
 */
void foc_encoder_calibration_set_raw(uint16_t raw_angle)
{
    (void)raw_angle;
}

/**
 * @brief 校准功能关闭时的空校准接口
 */
void foc_encoder_calibration_open_loop_calibrate(void)
{
}

/**
 * @brief 校准功能关闭时的线性化接口
 * @param raw_angle MT6816 原始角度
 * @param electrical_angle 输出电角度指针
 * @param mechanical_angle 输出机械角度指针
 * @return 固定返回 0，表示当前没有可用校准表
 */
uint8_t foc_encoder_calibration_linearize(uint16_t raw_angle,
                                          float *electrical_angle,
                                          float *mechanical_angle)
{
    (void)raw_angle;
    (void)electrical_angle;
    (void)mechanical_angle;
    return 0U;
}
/**
 * @brief 校准功能关闭时的空调试打印接口
 */
void foc_encoder_calibration_debug_log(void)
{
}

#endif
