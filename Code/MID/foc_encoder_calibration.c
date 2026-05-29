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

#if ((FOC_ENCODER_CALIBRATION_SECTOR_COUNT % FOC_ENCODER_CALIBRATION_SENSOR_PERIODS_PER_REV) != 0U)
#error "FOC_ENCODER_CALIBRATION_SECTOR_COUNT must be divisible by SENSOR_PERIODS_PER_REV"
#endif

#if ((FOC_ENCODER_CALIBRATION_ENCODER_CPR % FOC_ENCODER_CALIBRATION_SENSOR_PERIODS_PER_REV) != 0U)
#error "FOC_ENCODER_CALIBRATION_ENCODER_CPR must be divisible by SENSOR_PERIODS_PER_REV"
#endif

#if ((MOTOR_POLE_PAIRS % FOC_ENCODER_CALIBRATION_SENSOR_PERIODS_PER_REV) != 0U)
#error "MOTOR_POLE_PAIRS must be divisible by SENSOR_PERIODS_PER_REV for unambiguous electrical angle"
#endif

/** d 轴校准电流，单位：A */
#define ENC_CAL_D_AXIS_CURRENT     0.3f
/** q 轴校准电流，校准时保持为 0，避免主动产生转矩 */
#define ENC_CAL_Q_AXIS_CURRENT     0.0f
/** 初始电角度锁定时间，单位：电流环 tick */
#define ENC_CAL_ALIGN_TICKS        ((uint32_t)(0.50f / PWM_PERIOD_S))
/** Per-point settling time before capturing raw angle. */
#define ENC_CAL_SETTLE_TICKS       ((uint32_t)(0.02f / PWM_PERIOD_S))
/** 从一个整步边界移动到下一个边界的时间，单位：电流环 tick */
#define ENC_CAL_ELEC_INC_DEG       0.05f
/** 每个整步边界采样前的稳定时间，单位：电流环 tick */
/** 校准完成后继续保持最后电角度的时间，便于观察电机最终位置 */
#define ENC_CAL_HOLD_TICKS         ((uint32_t)(2.00f / PWM_PERIOD_S))
/** 运行中进度打印间隔，单位：扇区 */
#define ENC_CAL_PROGRESS_STEP      20U
/** Unique sectors in one repeated encoder magnetic period. */
#define ENC_CAL_ACTIVE_SECTOR_COUNT FOC_ENCODER_CALIBRATION_ACTIVE_SECTOR_COUNT
/** Unique table entries in one repeated encoder magnetic period. */
#define ENC_CAL_ACTIVE_TABLE_SIZE   FOC_ENCODER_CALIBRATION_ACTIVE_TABLE_SIZE
/** Raw count span of one repeated encoder magnetic period. */
#define ENC_CAL_RAW_PERIOD_COUNTS   FOC_ENCODER_CALIBRATION_ENCODER_PERIOD_COUNTS
/** Total electrical angle covered by one unique encoder magnetic period. */
#define ENC_CAL_TOTAL_ELEC_DEG      (360.0f * (float)MOTOR_POLE_PAIRS / \
                                     (float)FOC_ENCODER_CALIBRATION_SENSOR_PERIODS_PER_REV)
/** 每个校准扇区对应的电角度步进，单位：度 */
#define ENC_CAL_ELEC_STEP_DEG      (ENC_CAL_TOTAL_ELEC_DEG / (float)ENC_CAL_ACTIVE_SECTOR_COUNT)
/** 每个校准扇区对应的机械角度步进，单位：度 */
#define ENC_CAL_MECH_STEP_DEG      (360.0f / (float)FOC_ENCODER_CALIBRATION_SECTOR_COUNT)
/** 相邻校准点最小允许间隔，理论值 period_counts/active_sectors，此处允许 -25% */
#define ENC_CAL_STEP_MIN_COUNTS    ((ENC_CAL_RAW_PERIOD_COUNTS * 3U) / (ENC_CAL_ACTIVE_SECTOR_COUNT * 4U))
/** 相邻校准点最大允许间隔，理论值 period_counts/active_sectors，此处允许 +25% */
#define ENC_CAL_STEP_MAX_COUNTS    ((ENC_CAL_RAW_PERIOD_COUNTS * 5U + (ENC_CAL_ACTIVE_SECTOR_COUNT * 4U - 1U)) / (ENC_CAL_ACTIVE_SECTOR_COUNT * 4U))

foc_encoder_calibration_t g_encoder_calibration = {
    .latest_raw = FOC_ENCODER_CALIBRATION_INVALID_RAW,
    .latest_raw_valid = 0U,
    .state = FOC_ENCODER_CALIBRATION_STATE_IDLE,
    .error = FOC_ENCODER_CALIBRATION_ERROR_NONE,
    .raw_direction = 1,
    .first_bad_sector = ENC_CAL_ACTIVE_SECTOR_COUNT,
};

const int32_t foc_encoder_calibration_const_table[FOC_ENCODER_CALIBRATION_TABLE_SIZE] = {
5998,
5917,
5837,
5758,
5673,
5592,
5511,
5432,
5346,
5265,
5184,
5105,
5019,
4938,
4857,
4778,
4692,
4610,
4530,
4451,
4366,
4285,
4204,
4125,
4040,
3958,
3879,
3800,
3714,
3632,
3552,
3473,
3386,
3304,
3223,
3143,
3056,
2974,
2894,
2814,
2728,
2646,
2564,
2485,
2399,
2318,
2237,
2158,
2071,
1989,
1909,
1831,
1745,
1663,
1582,
1503,
1417,
1334,
1253,
1173,
1087,
1005,
923,
844,
757,
675,
594,
515,
429,
347,
266,
187,
101,
19,
-61,
-140,
-225,
-308,
-388,
-467,
-552,
-634,
-715,
-795,
-880,
-963,
-1043,
-1123,
-1208,
-1290,
-1370,
-1449,
-1535,
-1617,
-1698,
-1775,
-1861,
-1942,
-2024,
-2103,
-2189,
-2269,
-2351,
-2430,
-2516,
-2597,
-2679,
-2759,
-2845,
-2927,
-3008,
-3088,
-3174,
-3257,
-3339,
-3418,
-3504,
-3586,
-3668,
-3747,
-3832,
-3914,
-3995,
-4075,
-4160,
-4241,
-4322,
-4401,
-4487,
-4568,
-4649,
-4729,
-4815,
-4897,
-4978,
-5058,
-5145,
-5226,
-5307,
-5387,
-5472,
-5554,
-5636,
-5716,
-5802,
-5882,
-5964,
-6044,
-6130,
-6211,
-6294,
-6373,
-6459,
-6541,
-6622,
-6701,
-6787,
-6869,
-6950,
-7030,
-7116,
-7198,
-7280,
-7359,
-7446,
-7527,
-7608,
-7688,
-7774,
-7855,
-7935,
-8015,
-8100,
-8182,
-8262,
-8342,
-8427,
-8508,
-8588,
-8668,
-8753,
-8835,
-8916,
-8996,
-9081,
-9163,
-9244,
-9324,
-9409,
-9490,
-9571,
-9650,
-9736,
-9816,
-9897,
-9975,
-10061,
-10142,
-10222,
-10302,
-10386
};

const int8_t foc_encoder_calibration_const_direction = -1;

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

    if (delta > half_cpr)
    {
        delta -= (int32_t)ENC_CAL_RAW_PERIOD_COUNTS;
    }
    else if (delta < -half_cpr)
    {
        delta += (int32_t)ENC_CAL_RAW_PERIOD_COUNTS;
    }

    return delta;
}

/**
 * @brief 捕获当前扇区的编码器原始角度
 * @param sector 需要写入的校准扇区编号
 * @return 1 表示捕获成功，0 表示编码器原始值无效
 */
static uint8_t enc_cal_capture_sector(uint16_t sector)
{
    if ((g_encoder_calibration.latest_raw_valid == 0U) ||
        (g_encoder_calibration.latest_raw == FOC_ENCODER_CALIBRATION_INVALID_RAW))
    {
        enc_cal_set_error(FOC_ENCODER_CALIBRATION_ERROR_RAW_INVALID);
        return 0U;
    }

    // 对当前扇区写入当前编码器实际读取角度
    g_encoder_calibration.raw_table[sector] = g_encoder_calibration.latest_raw;
    g_encoder_calibration.sample_count = sector + 1U;
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
    if (abs_delta < 0)
    {
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
    for (uint16_t i = 1U; i < ENC_CAL_ACTIVE_SECTOR_COUNT; i++)
    {
        delta_sum += enc_cal_wrap_delta(g_encoder_calibration.raw_table[i],
                                        g_encoder_calibration.raw_table[i - 1U]);
    }

    if (delta_sum > 0)
    {
        g_encoder_calibration.raw_direction = 1;
    }
    else if (delta_sum < 0)
    {
        g_encoder_calibration.raw_direction = -1;
    }
    else
    {
        g_encoder_calibration.raw_direction = 1;
    }
    g_encoder_calibration.bad_interval_count = 0U;
    g_encoder_calibration.first_bad_sector = ENC_CAL_ACTIVE_SECTOR_COUNT;
    g_encoder_calibration.max_interval_delta = 0;
    g_encoder_calibration.linear_table[0] = (int32_t)g_encoder_calibration.raw_table[0];
    // 逐段生成linear表
    for (uint16_t i = 1U; i < ENC_CAL_ACTIVE_SECTOR_COUNT; i++)
    {
        int32_t delta = enc_cal_wrap_delta(g_encoder_calibration.raw_table[i],
                                           g_encoder_calibration.raw_table[i - 1U]);
        int32_t abs_delta = delta;

        if (abs_delta < 0)
        {
            abs_delta = -abs_delta;
        }
        if (abs_delta > g_encoder_calibration.max_interval_delta)
        {
            g_encoder_calibration.max_interval_delta = abs_delta;
        }
        if ((delta * (int32_t)g_encoder_calibration.raw_direction <= 0) ||
            (enc_cal_validate_interval(delta) == 0U))
        {
            if (g_encoder_calibration.bad_interval_count == 0U)
            {
                g_encoder_calibration.first_bad_sector = i;
            }
            g_encoder_calibration.bad_interval_count++;
        }

        g_encoder_calibration.linear_table[i] = g_encoder_calibration.linear_table[i - 1U] + delta;
    }

    int32_t end_delta = enc_cal_wrap_delta(g_encoder_calibration.raw_table[0],
                                           g_encoder_calibration.raw_table[ENC_CAL_ACTIVE_SECTOR_COUNT - 1U]);
    int32_t abs_end_delta = end_delta;
    if (abs_end_delta < 0)
    {
        abs_end_delta = -abs_end_delta;
    }
    if (abs_end_delta > g_encoder_calibration.max_interval_delta)
    {
        g_encoder_calibration.max_interval_delta = abs_end_delta;
    }
    if ((end_delta * (int32_t)g_encoder_calibration.raw_direction <= 0) ||
        (enc_cal_validate_interval(end_delta) == 0U))
    {
        if (g_encoder_calibration.bad_interval_count == 0U)
        {
            g_encoder_calibration.first_bad_sector = ENC_CAL_ACTIVE_SECTOR_COUNT;
        }
        g_encoder_calibration.bad_interval_count++;
    }

    g_encoder_calibration.measured_span_counts = delta_sum + end_delta;
    if (g_encoder_calibration.measured_span_counts < 0)
    {
        g_encoder_calibration.measured_span_counts = -g_encoder_calibration.measured_span_counts;
    }

    g_encoder_calibration.linear_table[ENC_CAL_ACTIVE_SECTOR_COUNT] =
        g_encoder_calibration.linear_table[0] +
        (int32_t)g_encoder_calibration.raw_direction * (int32_t)ENC_CAL_RAW_PERIOD_COUNTS;

    for (uint16_t i = 0U; i < ENC_CAL_ACTIVE_SECTOR_COUNT; i++)
    {
        float ideal = (float)g_encoder_calibration.linear_table[0] +
                      (float)g_encoder_calibration.raw_direction *
                      (float)ENC_CAL_RAW_PERIOD_COUNTS *
                      (float)i /
                      (float)ENC_CAL_ACTIVE_SECTOR_COUNT;
        float error = (float)g_encoder_calibration.linear_table[i] - ideal;

        if (error < 0.0f)
        {
            error = -error;
        }
        if (error > max_nonlinearity)
        {
            max_nonlinearity = error;
        }
    }

    g_encoder_calibration.max_nonlinearity_counts = (int32_t)(max_nonlinearity + 0.5f);
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
 * @brief 使用开环电角度执行 d/q 轴电流闭环
 * @param electrical_angle_deg 开环电角度指令，单位：度
 * @param target_d d 轴目标电流，单位：A
 * @param target_q q 轴目标电流，单位：A
 */
static void enc_cal_current_loop(float electrical_angle_deg, float target_d, float target_q)
{
    float sin_theta;
    float cos_theta;

    if (foc_voltage_data.vbus <= EPS_F)
    {
        enc_cal_output_zero();
        return;
    }

    foc_ctrl.angle = angle_normalize_360(electrical_angle_deg);
    foc_ctrl.target_d = target_d;
    foc_ctrl.target_q = target_q;

    alpha_beta.alpha_i = foc_current_data.IA;
    alpha_beta.beta_i = foc_current_data.IB;

    CORDIC_SinCos_Deg(foc_ctrl.angle, &sin_theta, &cos_theta);
    park_transform(&alpha_beta, &foc_ctrl, sin_theta, cos_theta);

    foc_ctrl.out_d = foc_id_pid_calculate(target_d, foc_ctrl.abc_dq.current_d);
    foc_ctrl.out_q = foc_iq_pid_calculate(target_q, foc_ctrl.abc_dq.current_q);

    inv_park_transform_f32(&foc_ctrl, &alpha_beta, sin_theta, cos_theta);
    spwm_duty_calc(&spwm, &alpha_beta, foc_voltage_data.vbus);
    bsp_pwm_set_duty_four_phase(spwm.duty_Ap, spwm.duty_An, spwm.duty_Bp, spwm.duty_Bn);
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
    g_encoder_calibration.command_angle_deg = 0.0f;
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
    if (raw_angle == FOC_ENCODER_CALIBRATION_INVALID_RAW)
    {
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
    switch (g_encoder_calibration.state)
    {
        case FOC_ENCODER_CALIBRATION_STATE_IDLE:
            g_encoder_calibration.error = FOC_ENCODER_CALIBRATION_ERROR_NONE;
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
            g_encoder_calibration.command_angle_deg = 0.0f;
            g_encoder_calibration.progress_sector = 0U;
            g_encoder_calibration.start_report_pending = 1U;
            g_encoder_calibration.progress_report_pending = 0U;
            g_encoder_calibration.done_report_pending = 0U;
            g_encoder_calibration.error_report_pending = 0U;
            g_encoder_calibration.quality_report_pending = 0U;
            g_encoder_calibration.print_pending = 0U;
            g_encoder_calibration.print_index = 0U;
            g_encoder_calibration.state = FOC_ENCODER_CALIBRATION_STATE_ALIGN;
            break;

        // 等待转子稳定
        case FOC_ENCODER_CALIBRATION_STATE_ALIGN:
            g_encoder_calibration.command_angle_deg = -ENC_CAL_ELEC_STEP_DEG;
            enc_cal_current_loop(g_encoder_calibration.command_angle_deg,
                                 ENC_CAL_D_AXIS_CURRENT,
                                 ENC_CAL_Q_AXIS_CURRENT);
            if (++g_encoder_calibration.phase_tick >= ENC_CAL_ALIGN_TICKS)
            {
                g_encoder_calibration.active_sector = 0U;
                g_encoder_calibration.phase_tick = 0U;
                g_encoder_calibration.state = FOC_ENCODER_CALIBRATION_STATE_MOVE;
            }
            break;

        case FOC_ENCODER_CALIBRATION_STATE_MOVE:
        {
            float target_angle_deg;

            if (g_encoder_calibration.active_sector >= ENC_CAL_ACTIVE_SECTOR_COUNT)
            {
                if (enc_cal_finalize_table() != 0U)
                {
                    g_encoder_calibration.phase_tick = 0U;
                    g_encoder_calibration.state = FOC_ENCODER_CALIBRATION_STATE_DONE;
                }
                break;
            }

            target_angle_deg =
                (float)g_encoder_calibration.active_sector * ENC_CAL_ELEC_STEP_DEG;

            // 不断增加电角度
            g_encoder_calibration.command_angle_deg += ENC_CAL_ELEC_INC_DEG;
            if (g_encoder_calibration.command_angle_deg > target_angle_deg)
            {
                g_encoder_calibration.command_angle_deg = target_angle_deg;
            }

            enc_cal_current_loop(g_encoder_calibration.command_angle_deg,
                                 ENC_CAL_D_AXIS_CURRENT,
                                 ENC_CAL_Q_AXIS_CURRENT);

            if (g_encoder_calibration.command_angle_deg >= target_angle_deg)
            {
                g_encoder_calibration.phase_tick = 0U;
                g_encoder_calibration.state = FOC_ENCODER_CALIBRATION_STATE_SETTLE;
            }
            break;
        }

        case FOC_ENCODER_CALIBRATION_STATE_SETTLE:
            enc_cal_current_loop(g_encoder_calibration.command_angle_deg,
                                 ENC_CAL_D_AXIS_CURRENT,
                                 ENC_CAL_Q_AXIS_CURRENT);

            if (++g_encoder_calibration.phase_tick >= ENC_CAL_SETTLE_TICKS)
            {
                if (enc_cal_capture_sector(g_encoder_calibration.active_sector) == 0U)
                {
                    break;
                }

                g_encoder_calibration.progress_sector = g_encoder_calibration.active_sector;
                g_encoder_calibration.progress_report_pending = 1U;
                g_encoder_calibration.active_sector++;
                g_encoder_calibration.phase_tick = 0U;

                if (g_encoder_calibration.active_sector >= ENC_CAL_ACTIVE_SECTOR_COUNT)
                {
                    if (enc_cal_finalize_table() != 0U)
                    {
                        g_encoder_calibration.state = FOC_ENCODER_CALIBRATION_STATE_DONE;
                    }
                }
                else
                {
                    g_encoder_calibration.state = FOC_ENCODER_CALIBRATION_STATE_MOVE;
                }
            }
            break;

        case FOC_ENCODER_CALIBRATION_STATE_DONE:
            enc_cal_current_loop(g_encoder_calibration.command_angle_deg,
                                 ENC_CAL_D_AXIS_CURRENT,
                                 ENC_CAL_Q_AXIS_CURRENT);
            if (++g_encoder_calibration.phase_tick >= ENC_CAL_HOLD_TICKS)
            {
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
 * @param electrical_angle 输出电角度，单位：度，范围 0~360
 * @param mechanical_angle 输出机械角度，单位：度；多磁周期编码器下为单个磁周期内角度
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
    #else
    table = g_encoder_calibration.linear_table;
    direction = g_encoder_calibration.raw_direction;
    table_ready = g_encoder_calibration.calibrated;
    #endif

    if ((table_ready == 0U) ||
        (raw_angle == FOC_ENCODER_CALIBRATION_INVALID_RAW) ||
        (electrical_angle == 0) ||
        (mechanical_angle == 0))
    {
        return 0U;
    }

    input = (int32_t)raw_angle;

    if (direction >= 0)
    {
        while (input < table[0])
        {
            input += (int32_t)ENC_CAL_RAW_PERIOD_COUNTS;
        }
        while (input >= table[0] + (int32_t)ENC_CAL_RAW_PERIOD_COUNTS)
        {
            input -= (int32_t)ENC_CAL_RAW_PERIOD_COUNTS;
        }

        while ((high - low) > 1U)
        {
            uint16_t mid = (uint16_t)((low + high) >> 1U);
            if (input >= table[mid])
            {
                low = mid;
            }
            else
            {
                high = mid;
            }
        }
    }
    else
    {
        while (input > table[0])
        {
            input -= (int32_t)ENC_CAL_RAW_PERIOD_COUNTS;
        }
        while (input <= table[0] - (int32_t)ENC_CAL_RAW_PERIOD_COUNTS)
        {
            input += (int32_t)ENC_CAL_RAW_PERIOD_COUNTS;
        }

        while ((high - low) > 1U)
        {
            uint16_t mid = (uint16_t)((low + high) >> 1U);
            if (input <= table[mid])
            {
                low = mid;
            }
            else
            {
                high = mid;
            }
        }
    }

    a = table[low];
    b = table[low + 1U];
    if (a == b)
    {
        return 0U;
    }

    fraction = (float)(input - a) / (float)(b - a);
    if (fraction < 0.0f)
    {
        fraction = 0.0f;
    }
    else if (fraction > 1.0f)
    {
        fraction = 1.0f;
    }

    sector_position = (float)low + fraction;
    *mechanical_angle = sector_position * ENC_CAL_MECH_STEP_DEG;
    *electrical_angle = angle_normalize_360(*mechanical_angle * MOTOR_POLE_PAIRS);

    return 1U;
}
/**
 * @brief 打印编码器校准调试信息
 * @note 每次调用最多打印一行，适合放在 1kHz 调试任务中分批输出。
 */
void foc_encoder_calibration_debug_log(void)
{
    if (g_encoder_calibration.start_report_pending != 0U)
    {
        g_encoder_calibration.start_report_pending = 0U;
        debug_log("[ENC_CAL] start: sectors=%u sensor_periods=%u pole_pairs=%u capture_step=%.3f inc=%.3f settle_ticks=%lu total_elec=%.1f d_current=%.3f",
                  ENC_CAL_ACTIVE_SECTOR_COUNT,
                  FOC_ENCODER_CALIBRATION_SENSOR_PERIODS_PER_REV,
                  MOTOR_POLE_PAIRS,
                  ENC_CAL_ELEC_STEP_DEG,
                  ENC_CAL_ELEC_INC_DEG,
                  (unsigned long)ENC_CAL_SETTLE_TICKS,
                  ENC_CAL_ELEC_STEP_DEG * (float)ENC_CAL_ACTIVE_SECTOR_COUNT,
                  ENC_CAL_D_AXIS_CURRENT);
        return;
    }

    if (g_encoder_calibration.progress_report_pending != 0U)
    {
        g_encoder_calibration.progress_report_pending = 0U;
        debug_log("%03u, %u",
                  g_encoder_calibration.progress_sector,
                  g_encoder_calibration.raw_table[g_encoder_calibration.progress_sector]);
        return;
    }

    if (g_encoder_calibration.error_report_pending != 0U)
    {
        g_encoder_calibration.error_report_pending = 0U;
        debug_log("[ENC_CAL] ERROR: code=%u sector=%u raw=%u",
                  (uint8_t)g_encoder_calibration.error,
                  g_encoder_calibration.active_sector,
                  g_encoder_calibration.latest_raw);
        return;
    }

    if (g_encoder_calibration.done_report_pending != 0U)
    {
        g_encoder_calibration.done_report_pending = 0U;
        debug_log("[ENC_CAL] done: dir=%d", g_encoder_calibration.raw_direction);
        return;
    }

    if (g_encoder_calibration.print_pending != 0U)
    {
        if (g_encoder_calibration.print_index < ENC_CAL_ACTIVE_TABLE_SIZE)
        {
            uint16_t index = g_encoder_calibration.print_index;
            debug_log("%d,", (int)g_encoder_calibration.linear_table[index]);
            g_encoder_calibration.print_index++;
            return;
        }

        g_encoder_calibration.print_pending = 0U;
        return;
    }

    if (g_encoder_calibration.quality_report_pending != 0U)
    {
        g_encoder_calibration.quality_report_pending = 0U;
        debug_log("[ENC_CAL] quality: bad=%u first=%u span=%d max_delta=%d ecc=%.3f%% max_err=%d",
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
