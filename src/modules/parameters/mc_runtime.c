#include "mc_runtime.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "board_config.h"
#include "data_types.h"
#include "stm32g4xx_hal.h"

#define MC_RUNTIME_FLASH_ADDRESS            0x0801F000UL
#define MC_RUNTIME_FLASH_PAGE               62U
#define MC_RUNTIME_FLASH_MAGIC              0x464F4346UL
#define MC_RUNTIME_FLASH_VERSION            8U
#define MC_RUNTIME_FLASH_VERSION_V7         7U

#define MC_RUNTIME_DEFAULT_ID          0.0f
#define MC_RUNTIME_DEFAULT_IQ          0.5f
#define MC_RUNTIME_DEFAULT_SPEED       0.0f
#define MC_RUNTIME_DEFAULT_POSITION    0.0f
#define MC_RUNTIME_DEFAULT_STEP_A      4.537856055f
#define MC_RUNTIME_DEFAULT_STEP_B      (-4.537856055f)
#define MC_RUNTIME_DEFAULT_STEP_PERIOD 2000U
#define MC_RUNTIME_STEP_PERIOD_MIN     10U
#define MC_RUNTIME_STEP_PERIOD_MAX     600000U

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t record_size;
    float resistance;
    float inductance_ld;
    float inductance_lq;
    float flux_linkage;
    float ka;
    float kv;
    float tc;
    float angle_min;
    float angle_max;
    float speed_pid_kp;
    float speed_pid_ki;
    float speed_pid_kd;
    float position_pid_kp;
    float position_pid_ki;
    float position_pid_kd;
    float cascade_pid_kp;
    float cascade_pid_ki;
    float cascade_pid_kd;
    uint32_t weakening_enabled;
    uint32_t crc32;
} mc_runtime_flash_record_t;

/** @brief v7 FLASH 参数记录，仅用于升级时保留已辨识参数 */
typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t record_size;
    float resistance;
    float inductance_ld;
    float inductance_lq;
    float flux_linkage;
    float ka;
    float kv;
    float tc;
    float angle_min;
    float angle_max;
    float speed_pid_kp;
    float speed_pid_ki;
    float speed_pid_kd;
    float position_pid_kp;
    float position_pid_ki;
    float position_pid_kd;
    uint32_t weakening_enabled;
    uint32_t crc32;
} mc_runtime_flash_record_v7_t;


_Static_assert(sizeof(mc_runtime_flash_record_t) <= FLASH_PAGE_SIZE,
               "motor config record does not fit in one FLASH page");
_Static_assert(sizeof(mc_runtime_flash_record_t) == 88U,
               "v8 FLASH record layout changed");
_Static_assert(sizeof(mc_runtime_flash_record_v7_t) == 76U,
               "v7 FLASH compatibility layout changed");
static mc_runtime_config_t runtime_config;
static mc_motor_config_t motor_config;
static mc_tuning_config_t tuning_config;
static volatile mc_mode_t runtime_mode = MC_MODE_IDLE;
static volatile mc_run_profile_t runtime_profile = MC_RUN_CURRENT;
static volatile uint8_t runtime_ready = 0U;
static mc_runtime_flash_record_t flash_write_buffer;

/**
 * @brief 计算持久化记录的 CRC32
 * @param data 数据起始地址
 * @param length 数据长度
 * @return CRC32 校验值
 */
static uint32_t runtime_crc32(const void *data, uint32_t length)
{
    const uint8_t *bytes = (const uint8_t *)data;
    uint32_t crc = 0xFFFFFFFFUL;

    for (uint32_t i = 0U; i < length; i++) {
        crc ^= bytes[i];
        for (uint8_t bit = 0U; bit < 8U; bit++) {
            crc = (crc >> 1U) ^ (((crc & 1U) != 0U) ? 0xEDB88320UL : 0UL);
        }
    }

    return crc ^ 0xFFFFFFFFUL;
}

/**
 * @brief 校验电机参数是否处于允许范围
 * @param config 电机参数
 * @return 1 表示有效，0 表示无效
 */
static uint8_t motor_config_valid(const mc_motor_config_t *config)
{
    if ((!isfinite(config->resistance)) || (!isfinite(config->inductance_ld)) ||
        (!isfinite(config->inductance_lq)) ||
        (!isfinite(config->flux_linkage)) || (config->resistance <= 0.0f) ||
        (config->resistance > 100.0f) || (config->inductance_ld <= 0.0f) ||
        (config->inductance_ld > 1.0f) || (config->inductance_lq <= 0.0f) ||
        (config->inductance_lq > 1.0f) || (config->flux_linkage <= 0.0f) ||
        (config->flux_linkage > 1.0f) || (!isfinite(config->ka)) ||
        (!isfinite(config->kv)) || (!isfinite(config->tc)) ||
        (config->ka < 0.0f) || (config->ka > 100.0f) || (config->kv < 0.0f) ||
        (config->kv > 100.0f) || (config->tc < 0.0f) ||
        (config->tc > 100.0f) || (!isfinite(config->angle_min)) ||
        (!isfinite(config->angle_max)) || (fabsf(config->angle_min) > 1.0e6f) ||
        (fabsf(config->angle_max) > 1.0e6f) ||
        ((config->angle_min != 0.0f) && (config->angle_max != 0.0f) &&
         (config->angle_max <= config->angle_min))) {
        return 0U;
    }

    return 1U;
}

/**
 * @brief 校验速度和位置环 PID 参数
 * @param config PID 参数
 * @return 1 表示有效，0 表示无效
 */
static uint8_t tuning_config_valid(const mc_tuning_config_t *config)
{
    if ((!isfinite(config->speed_kp)) || (!isfinite(config->speed_ki)) ||
        (!isfinite(config->speed_kd)) || (!isfinite(config->position_kp)) ||
        (!isfinite(config->position_ki)) ||
        (!isfinite(config->position_kd)) || (config->speed_kp < 0.0f) ||
        (config->speed_kp > 10000.0f) || (config->speed_ki < 0.0f) ||
        (config->speed_ki > 10000.0f) || (config->speed_kd < 0.0f) ||
        (config->speed_kd > 1000.0f) || (config->position_kp < 0.0f) ||
        (config->position_kp > 10000.0f) || (config->position_ki < 0.0f) ||
        (config->position_ki > 10000.0f) || (config->position_kd < 0.0f) ||
        (config->position_kd > 1000.0f) || (!isfinite(config->cascade_kp)) ||
        (!isfinite(config->cascade_ki)) || (!isfinite(config->cascade_kd)) ||
        (config->cascade_kp < 0.0f) || (config->cascade_kp > 10000.0f) ||
        (config->cascade_ki < 0.0f) || (config->cascade_ki > 10000.0f) ||
        (config->cascade_kd < 0.0f) || (config->cascade_kd > 1000.0f)) {
        return 0U;
    }
    return 1U;
}

/**
 * @brief 校验 FLASH 中电机参数记录的格式和 CRC
 * @param record FLASH 记录
 * @return 1 表示有效，0 表示无效
 */
static uint8_t
runtime_flash_record_valid(const mc_runtime_flash_record_t *record)
{
    mc_motor_config_t config;
    mc_tuning_config_t tuning;

    if ((record->magic != MC_RUNTIME_FLASH_MAGIC) ||
        (record->version != MC_RUNTIME_FLASH_VERSION) ||
        (record->record_size != sizeof(mc_runtime_flash_record_t)) ||
        (record->crc32 !=
         runtime_crc32(record, offsetof(mc_runtime_flash_record_t, crc32)))) {
        return 0U;
    }

    config.resistance = record->resistance;
    config.inductance_ld = record->inductance_ld;
    config.inductance_lq = record->inductance_lq;
    config.flux_linkage = record->flux_linkage;
    config.ka = record->ka;
    config.kv = record->kv;
    config.tc = record->tc;
    config.angle_min = record->angle_min;
    config.angle_max = record->angle_max;
    tuning.speed_kp = record->speed_pid_kp;
    tuning.speed_ki = record->speed_pid_ki;
    tuning.speed_kd = record->speed_pid_kd;
    tuning.position_kp = record->position_pid_kp;
    tuning.position_ki = record->position_pid_ki;
    tuning.position_kd = record->position_pid_kd;
    tuning.cascade_kp = record->cascade_pid_kp;
    tuning.cascade_ki = record->cascade_pid_ki;
    tuning.cascade_kd = record->cascade_pid_kd;
    return (motor_config_valid(&config) != 0U &&
            tuning_config_valid(&tuning) != 0U)
        ? 1U
        : 0U;
}

/**
 * @brief 校验 v7 FLASH 参数记录并补齐三环 PID 默认值
 * @param record v7 FLASH 记录
 * @param config 电机参数输出
 * @param tuning PID 参数输出
 * @return 1 表示有效，0 表示无效
 */
static uint8_t runtime_flash_record_v7_read(
    const mc_runtime_flash_record_v7_t *record, mc_motor_config_t *config,
    mc_tuning_config_t *tuning)
{
    if ((record->magic != MC_RUNTIME_FLASH_MAGIC) ||
        (record->version != MC_RUNTIME_FLASH_VERSION_V7) ||
        (record->record_size != sizeof(mc_runtime_flash_record_v7_t)) ||
        (record->crc32 != runtime_crc32(
             record, offsetof(mc_runtime_flash_record_v7_t, crc32)))) {
        return 0U;
    }

    config->resistance = record->resistance;
    config->inductance_ld = record->inductance_ld;
    config->inductance_lq = record->inductance_lq;
    config->flux_linkage = record->flux_linkage;
    config->ka = record->ka;
    config->kv = record->kv;
    config->tc = record->tc;
    config->angle_min = record->angle_min;
    config->angle_max = record->angle_max;
    tuning->speed_kp = record->speed_pid_kp;
    tuning->speed_ki = record->speed_pid_ki;
    tuning->speed_kd = record->speed_pid_kd;
    tuning->position_kp = record->position_pid_kp;
    tuning->position_ki = record->position_pid_ki;
    tuning->position_kd = record->position_pid_kd;
    tuning->cascade_kp = CASCADE_PID_KP_DEFAULT;
    tuning->cascade_ki = CASCADE_PID_KI_DEFAULT;
    tuning->cascade_kd = CASCADE_PID_KD_DEFAULT;
    return (motor_config_valid(config) != 0U &&
            tuning_config_valid(tuning) != 0U)
        ? 1U
        : 0U;
}


/**
 * @brief 将当前电机参数写入专用 FLASH 页
 * @return 1 表示写入并回读成功，0 表示失败
 */
static uint8_t runtime_save_to_flash(void)
{
    FLASH_EraseInitTypeDef erase = {0};
    HAL_StatusTypeDef status;
    uint32_t page_error = 0U;

    memset(&flash_write_buffer, 0, sizeof(flash_write_buffer));
    flash_write_buffer.magic = MC_RUNTIME_FLASH_MAGIC;
    flash_write_buffer.version = MC_RUNTIME_FLASH_VERSION;
    flash_write_buffer.record_size = sizeof(mc_runtime_flash_record_t);
    flash_write_buffer.resistance = motor_config.resistance;
    flash_write_buffer.inductance_ld = motor_config.inductance_ld;
    flash_write_buffer.inductance_lq = motor_config.inductance_lq;
    flash_write_buffer.flux_linkage = motor_config.flux_linkage;
    flash_write_buffer.ka = motor_config.ka;
    flash_write_buffer.kv = motor_config.kv;
    flash_write_buffer.tc = motor_config.tc;
    flash_write_buffer.angle_min = motor_config.angle_min;
    flash_write_buffer.angle_max = motor_config.angle_max;
    flash_write_buffer.speed_pid_kp = tuning_config.speed_kp;
    flash_write_buffer.speed_pid_ki = tuning_config.speed_ki;
    flash_write_buffer.speed_pid_kd = tuning_config.speed_kd;
    flash_write_buffer.position_pid_kp = tuning_config.position_kp;
    flash_write_buffer.position_pid_ki = tuning_config.position_ki;
    flash_write_buffer.position_pid_kd = tuning_config.position_kd;
    flash_write_buffer.cascade_pid_kp = tuning_config.cascade_kp;
    flash_write_buffer.cascade_pid_ki = tuning_config.cascade_ki;
    flash_write_buffer.cascade_pid_kd = tuning_config.cascade_kd;
    flash_write_buffer.weakening_enabled = runtime_config.weakening_enabled;
    flash_write_buffer.crc32 = runtime_crc32(
        &flash_write_buffer, offsetof(mc_runtime_flash_record_t, crc32));

    if (HAL_FLASH_Unlock() != HAL_OK) {
        return 0U;
    }

    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks = FLASH_BANK_1;
    erase.Page = MC_RUNTIME_FLASH_PAGE;
    erase.NbPages = 1U;
    status = HAL_FLASHEx_Erase(&erase, &page_error);

    for (uint32_t offset = 0U;
         (status == HAL_OK) && (offset < sizeof(flash_write_buffer));
         offset += 8U) {
        uint64_t double_word = 0xFFFFFFFFFFFFFFFFULL;
        uint32_t remaining = (uint32_t)sizeof(flash_write_buffer) - offset;
        uint32_t copy_size = (remaining < 8U) ? remaining : 8U;

        memcpy(&double_word, ((const uint8_t *)&flash_write_buffer) + offset,
               copy_size);
        status =
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                              MC_RUNTIME_FLASH_ADDRESS + offset, double_word);
    }

    (void)HAL_FLASH_Lock();

    if ((status != HAL_OK) ||
        (memcmp((const void *)MC_RUNTIME_FLASH_ADDRESS, &flash_write_buffer,
                sizeof(flash_write_buffer)) != 0)) {
        return 0U;
    }

    return 1U;
}

/**
 * @brief 初始化运行状态并加载持久化电机参数
 */
void mc_runtime_init(void)
{
    mc_runtime_flash_info_t flash_info;

    runtime_config.target_id = MC_RUNTIME_DEFAULT_ID;
    runtime_config.target_iq = MC_RUNTIME_DEFAULT_IQ;
    runtime_config.target_speed = MC_RUNTIME_DEFAULT_SPEED;
    runtime_config.target_position = MC_RUNTIME_DEFAULT_POSITION;
    runtime_config.step_position_a = MC_RUNTIME_DEFAULT_STEP_A;
    runtime_config.step_position_b = MC_RUNTIME_DEFAULT_STEP_B;
    runtime_config.step_period_ms = MC_RUNTIME_DEFAULT_STEP_PERIOD;
    runtime_config.weakening_enabled = 0U;

    motor_config.resistance = MOTOR_RESISTANCE;
    motor_config.inductance_ld = MOTOR_INDUCTANCE_Ld;
    motor_config.inductance_lq = MOTOR_INDUCTANCE_Lq;
    motor_config.flux_linkage = MOTOR_FLUX_LINKAGE;
    motor_config.ka = MOTOR_KA;
    motor_config.kv = MOTOR_KV;
    motor_config.tc = MOTOR_TC;
    motor_config.angle_min = MOTOR_ANGLE_MIN;
    motor_config.angle_max = MOTOR_ANGLE_MAX;
    tuning_config.speed_kp = SPEED_PID_KP_DEFAULT;
    tuning_config.speed_ki = SPEED_PID_KI_DEFAULT;
    tuning_config.speed_kd = SPEED_PID_KD_DEFAULT;
    tuning_config.position_kp = POSITION_PID_KP_DEFAULT;
    tuning_config.position_ki = POSITION_PID_KI_DEFAULT;
    tuning_config.position_kd = POSITION_PID_KD_DEFAULT;
    tuning_config.cascade_kp = CASCADE_PID_KP_DEFAULT;
    tuning_config.cascade_ki = CASCADE_PID_KI_DEFAULT;
    tuning_config.cascade_kd = CASCADE_PID_KD_DEFAULT;

    if (mc_runtime_read_flash(&flash_info) != 0U) {
        motor_config.resistance = flash_info.motor.resistance;
        motor_config.inductance_ld = flash_info.motor.inductance_ld;
        motor_config.inductance_lq = flash_info.motor.inductance_lq;
        motor_config.flux_linkage = flash_info.motor.flux_linkage;
        motor_config.ka = flash_info.motor.ka;
        motor_config.kv = flash_info.motor.kv;
        motor_config.tc = flash_info.motor.tc;
        motor_config.angle_min = flash_info.motor.angle_min;
        motor_config.angle_max = flash_info.motor.angle_max;
        runtime_config.weakening_enabled = flash_info.weakening_enabled;
        tuning_config = flash_info.tuning;
    }

    motor_ctrl.angle_min = motor_config.angle_min;
    motor_ctrl.angle_max = motor_config.angle_max;
    runtime_profile = MC_RUN_CURRENT;
    runtime_mode = MC_MODE_IDLE;
    runtime_ready = 0U;
}

/**
 * @brief 标记电机硬件初始化完成
 */
void mc_runtime_set_ready(void)
{
    runtime_ready = 1U;
}

/**
 * @brief 查询电机硬件是否可运行
 * @return 1 表示已就绪，0 表示未就绪
 */
uint8_t mc_runtime_is_ready(void)
{
    return runtime_ready;
}

/**
 * @brief 获取当前 IDLE/RUNNING 模式
 * @return 当前运行模式
 */
mc_mode_t mc_runtime_get_mode(void)
{
    return runtime_mode;
}

/**
 * @brief 获取 RUNNING 模式的控制类型
 * @return 电流、速度、单目标位置或位置阶跃运行模式
 */
mc_run_profile_t mc_runtime_get_profile(void)
{
    return runtime_profile;
}

/**
 * @brief 获取仅在 RAM 中保存的运行配置
 * @return 运行配置只读指针
 */
const mc_runtime_config_t *mc_runtime_get_config(void)
{
    return &runtime_config;
}

/**
 * @brief 获取当前生效的持久化电机参数
 * @return 电机参数只读指针
 */
const mc_motor_config_t *mc_runtime_get_motor_config(void)
{
    return &motor_config;
}

/**
 * @brief 获取速度环、位置环和三环位置环 PID 参数
 * @return PID 参数只读指针
 */
const mc_tuning_config_t *mc_runtime_get_tuning_config(void)
{
    return &tuning_config;
}

/**
 * @brief 校验并读取 FLASH 中实际保存的电机参数
 * @param info FLASH 记录信息输出
 * @return 1 表示记录有效，0 表示无有效记录或参数为空
 */
uint8_t mc_runtime_read_flash(mc_runtime_flash_info_t *info)
{
    const mc_runtime_flash_record_t *record =
        (const mc_runtime_flash_record_t *)MC_RUNTIME_FLASH_ADDRESS;
    const mc_runtime_flash_record_v7_t *record_v7 =
        (const mc_runtime_flash_record_v7_t *)MC_RUNTIME_FLASH_ADDRESS;

    if (info == NULL) {
        return 0U;
    }
    memset(info, 0, sizeof(*info));

    if (runtime_flash_record_valid(record) != 0U) {
        info->version = record->version;
        info->motor.resistance = record->resistance;
        info->motor.inductance_ld = record->inductance_ld;
        info->motor.inductance_lq = record->inductance_lq;
        info->motor.flux_linkage = record->flux_linkage;
        info->motor.ka = record->ka;
        info->motor.kv = record->kv;
        info->motor.tc = record->tc;
        info->motor.angle_min = record->angle_min;
        info->motor.angle_max = record->angle_max;
        info->tuning.speed_kp = record->speed_pid_kp;
        info->tuning.speed_ki = record->speed_pid_ki;
        info->tuning.speed_kd = record->speed_pid_kd;
        info->tuning.position_kp = record->position_pid_kp;
        info->tuning.position_ki = record->position_pid_ki;
        info->tuning.position_kd = record->position_pid_kd;
        info->tuning.cascade_kp = record->cascade_pid_kp;
        info->tuning.cascade_ki = record->cascade_pid_ki;
        info->tuning.cascade_kd = record->cascade_pid_kd;
        info->weakening_enabled = (record->weakening_enabled != 0U) ? 1U : 0U;
        return 1U;
    }

    if (runtime_flash_record_v7_read(record_v7, &info->motor,
                                     &info->tuning) != 0U) {
        info->version = MC_RUNTIME_FLASH_VERSION;
        info->weakening_enabled =
            (record_v7->weakening_enabled != 0U) ? 1U : 0U;
        return 1U;
    }

    return 0U;
}

/**
 * @brief 查询弱磁是否启用
 * @return 1 表示启用，0 表示关闭
 */
uint8_t mc_runtime_weakening_is_enabled(void)
{
    return runtime_config.weakening_enabled;
}

/**
 * @brief 设置弱磁使能状态
 * @param enabled 非零启用，零关闭
 * @return 1 表示设置成功
 */
uint8_t mc_runtime_set_weakening(uint8_t enabled)
{
    const uint8_t old_value = runtime_config.weakening_enabled;
    const uint8_t new_value = (enabled != 0U) ? 1U : 0U;

    if (new_value == old_value) {
        return 1U;
    }

    runtime_config.weakening_enabled = new_value;
    if ((runtime_mode == MC_MODE_IDLE) && (runtime_save_to_flash() == 0U)) {
        runtime_config.weakening_enabled = old_value;
        return 0U;
    }
    return 1U;
}

/**
 * @brief 设置 RAM 中的 d 轴目标电流
 * @param target_id d 轴目标电流，单位 A
 * @return 1 表示成功，0 表示状态或数值无效
 */
uint8_t mc_runtime_set_id(float target_id)
{
    if ((!isfinite(target_id)) || (target_id < -CURRENT_LIMIT) ||
        (target_id > CURRENT_LIMIT)) {
        return 0U;
    }
    if (target_id == runtime_config.target_id) {
        return 1U;
    }

    runtime_config.target_id = target_id;
    return 1U;
}

/**
 * @brief 设置 RAM 中的 q 轴目标电流
 * @param target_iq q 轴目标电流，单位 A
 * @return 1 表示成功，0 表示状态或数值无效
 */
uint8_t mc_runtime_set_iq(float target_iq)
{
    if ((!isfinite(target_iq)) || (target_iq < -CURRENT_LIMIT) ||
        (target_iq > CURRENT_LIMIT)) {
        return 0U;
    }
    if (target_iq == runtime_config.target_iq) {
        return 1U;
    }

    runtime_config.target_iq = target_iq;
    return 1U;
}

/**
 * @brief 设置 RAM 中的目标机械速度
 * @param target_speed 目标机械速度，单位 rad/s
 * @return 1 表示成功，0 表示状态或数值无效
 */
uint8_t mc_runtime_set_speed(float target_speed)
{
    if (!isfinite(target_speed)) {
        return 0U;
    }
    if (target_speed == runtime_config.target_speed) {
        return 1U;
    }

    runtime_config.target_speed = target_speed;
    return 1U;
}

/**
 * @brief 设置 RAM 中的单目标机械位置
 * @param target_position 目标机械位置，单位 rad
 * @return 1 表示成功，0 表示数值无效
 */
uint8_t mc_runtime_set_position(float target_position)
{
    if (!isfinite(target_position)) {
        return 0U;
    }
    if (target_position == runtime_config.target_position) {
        return 1U;
    }

    runtime_config.target_position = target_position;
    return 1U;
}

/**
 * @brief 设置 RAM 中的位置阶跃参数
 * @param position_a 阶跃端点 A，单位 rad
 * @param position_b 阶跃端点 B，单位 rad
 * @param period_ms 端点切换周期，单位 ms
 * @return 1 表示成功，0 表示状态或数值无效
 */
uint8_t mc_runtime_set_step(float position_a, float position_b,
                            uint32_t period_ms)
{
    if ((!isfinite(position_a)) || (!isfinite(position_b)) ||
        (period_ms < MC_RUNTIME_STEP_PERIOD_MIN) ||
        (period_ms > MC_RUNTIME_STEP_PERIOD_MAX)) {
        return 0U;
    }
    if ((position_a == runtime_config.step_position_a) &&
        (position_b == runtime_config.step_position_b) &&
        (period_ms == runtime_config.step_period_ms)) {
        return 1U;
    }

    runtime_config.step_position_a = position_a;
    runtime_config.step_position_b = position_b;
    runtime_config.step_period_ms = period_ms;
    return 1U;
}

/**
 * @brief 在 IDLE 模式保存电机电气参数
 * @param resistance 相电阻，单位 ohm
 * @param inductance_ld d 轴电感，单位 H
 * @param inductance_lq q 轴电感，单位 H
 * @param flux_linkage 磁链，单位 Wb
 * @return 1 表示 FLASH 保存成功，0 表示失败
 */
uint8_t mc_runtime_set_motor(float resistance, float inductance_ld,
                             float inductance_lq, float flux_linkage)
{
    mc_motor_config_t old_config;
    mc_motor_config_t new_config = {
        .resistance = resistance,
        .inductance_ld = inductance_ld,
        .inductance_lq = inductance_lq,
        .flux_linkage = flux_linkage,
        .ka = motor_config.ka,
        .kv = motor_config.kv,
        .tc = motor_config.tc,
        .angle_min = motor_config.angle_min,
        .angle_max = motor_config.angle_max,
    };

    if ((runtime_mode != MC_MODE_IDLE) ||
        (motor_config_valid(&new_config) == 0U)) {
        return 0U;
    }
    if (memcmp(&new_config, &motor_config, sizeof(motor_config)) == 0) {
        return 1U;
    }

    old_config = motor_config;
    motor_config = new_config;
    if (runtime_save_to_flash() == 0U) {
        motor_config = old_config;
        return 0U;
    }

    return 1U;
}

/**
 * @brief 仅更新相电阻并保存到 FLASH
 * @param resistance 相电阻（ohm）
 * @return 1 表示保存成功，0 表示失败
 */
uint8_t mc_runtime_set_motor_resistance(float resistance)
{
    return mc_runtime_set_motor(resistance, motor_config.inductance_ld,
                                motor_config.inductance_lq,
                                motor_config.flux_linkage);
}

/**
 * @brief 将平均相电感同时写入 d/q 轴并保存到 FLASH
 * @param inductance 相电感（H）
 * @return 1 表示保存成功，0 表示失败
 */
uint8_t mc_runtime_set_motor_inductance(float inductance)
{
    return mc_runtime_set_motor(motor_config.resistance, inductance, inductance,
                                motor_config.flux_linkage);
}

/**
 * @brief 在 IDLE 模式保存机械辨识参数到 FLASH
 * @param ka J/Kt，单位 A*s^2/rad
 * @param kv B/Kt，单位 A*s/rad
 * @param tc 库仑摩擦等效电流，单位 A
 * @return 1 表示 FLASH 保存成功，0 表示失败
 */
uint8_t mc_runtime_set_mechanical(float ka, float kv, float tc)
{
    mc_motor_config_t old_config;
    mc_motor_config_t new_config = motor_config;

    new_config.ka = ka;
    new_config.kv = kv;
    new_config.tc = tc;
    if ((runtime_mode != MC_MODE_IDLE) ||
        (motor_config_valid(&new_config) == 0U)) {
        return 0U;
    }

    old_config = motor_config;
    motor_config = new_config;
    if (runtime_save_to_flash() == 0U) {
        motor_config = old_config;
        return 0U;
    }

    return 1U;
}

/**
 * @brief 保存机械角度上下限
 * @param angle_min 机械角度下限（rad），0 表示该方向不限制
 * @param angle_max 机械角度上限（rad），0 表示该方向不限制
 * @return 1 表示保存成功，0 表示运行状态或参数不允许
 */
uint8_t mc_runtime_set_angle_limits(float angle_min, float angle_max)
{
    mc_motor_config_t old_config;
    mc_motor_config_t new_config = motor_config;

    new_config.angle_min = angle_min;
    new_config.angle_max = angle_max;
    if ((runtime_mode != MC_MODE_IDLE) ||
        (motor_config_valid(&new_config) == 0U)) {
        return 0U;
    }
    if ((angle_min == motor_config.angle_min) &&
        (angle_max == motor_config.angle_max)) {
        motor_ctrl.angle_min = angle_min;
        motor_ctrl.angle_max = angle_max;
        return 1U;
    }

    old_config = motor_config;
    motor_config = new_config;
    if (runtime_save_to_flash() == 0U) {
        motor_config = old_config;
        return 0U;
    }

    motor_ctrl.angle_min = angle_min;
    motor_ctrl.angle_max = angle_max;
    return 1U;
}

/**
 * @brief 更新并保存三组控制环 PID 参数
 * @param new_config 新 PID 参数
 * @return 1 表示保存成功，0 表示失败
 */
static uint8_t runtime_set_tuning(const mc_tuning_config_t *new_config)
{
    mc_tuning_config_t old_config;

    if ((runtime_mode != MC_MODE_IDLE) || (new_config == NULL) ||
        (tuning_config_valid(new_config) == 0U)) {
        return 0U;
    }
    if (memcmp(new_config, &tuning_config, sizeof(tuning_config)) == 0) {
        return 1U;
    }

    old_config = tuning_config;
    tuning_config = *new_config;
    if (runtime_save_to_flash() == 0U) {
        tuning_config = old_config;
        return 0U;
    }
    return 1U;
}

/**
 * @brief 保存速度环 PID 参数到 FLASH
 * @param kp 比例增益
 * @param ki 积分增益
 * @param kd 微分增益
 * @return 1 表示成功，0 表示运行状态或参数不允许
 */
uint8_t mc_runtime_set_speed_pid(float kp, float ki, float kd)
{
    mc_tuning_config_t new_config = tuning_config;

    new_config.speed_kp = kp;
    new_config.speed_ki = ki;
    new_config.speed_kd = kd;
    return runtime_set_tuning(&new_config);
}

/**
 * @brief 保存位置环 PID 参数到 FLASH
 * @param kp 比例增益
 * @param ki 积分增益
 * @param kd 微分增益
 * @return 1 表示成功，0 表示运行状态或参数不允许
 */
uint8_t mc_runtime_set_position_pid(float kp, float ki, float kd)
{
    mc_tuning_config_t new_config = tuning_config;

    new_config.position_kp = kp;
    new_config.position_ki = ki;
    new_config.position_kd = kd;
    return runtime_set_tuning(&new_config);
}

/**
 * @brief 保存三环位置环 PID 参数到 FLASH
 * @param kp 比例增益
 * @param ki 积分增益
 * @param kd 微分增益
 * @return 1 表示成功，0 表示运行状态或参数不允许
 */
uint8_t mc_runtime_set_cascade_pid(float kp, float ki, float kd)
{
    mc_tuning_config_t new_config = tuning_config;

    new_config.cascade_kp = kp;
    new_config.cascade_ki = ki;
    new_config.cascade_kd = kd;
    return runtime_set_tuning(&new_config);
}

/**
 * @brief 进入 RUNNING 模式或在线切换运行控制类型
 * @param profile RUNNING 模式使用的控制类型
 * @return 1 表示切换成功，0 表示状态不允许
 */
uint8_t mc_runtime_start(mc_run_profile_t profile)
{
    if ((runtime_ready == 0U) ||
        ((profile != MC_RUN_CURRENT) && (profile != MC_RUN_POSITION_STEP) &&
         (profile != MC_RUN_SPEED) && (profile != MC_RUN_POSITION) &&
         (profile != MC_RUN_POSITION_CASCADE))) {
        return 0U;
    }

    runtime_profile = profile;
    __DMB();
    runtime_mode = MC_MODE_RUNNING;
    return 1U;
}

/**
 * @brief 从 RUNNING 模式返回 IDLE
 */
void mc_runtime_stop(void)
{
    const uint32_t primask = __get_PRIMASK();

    __disable_irq();
    runtime_profile = MC_RUN_CURRENT;
    __DMB();
    runtime_mode = MC_MODE_IDLE;
    if (primask == 0U) {
        __enable_irq();
    }
}

/**
 * @brief 将 RAM 运行目标恢复为默认值
 * @return 1 表示成功，0 表示当前不处于 IDLE
 */
uint8_t mc_runtime_reset_config(void)
{
    if (runtime_mode != MC_MODE_IDLE) {
        return 0U;
    }

    runtime_config.target_id = MC_RUNTIME_DEFAULT_ID;
    runtime_config.target_iq = MC_RUNTIME_DEFAULT_IQ;
    runtime_config.target_speed = MC_RUNTIME_DEFAULT_SPEED;
    runtime_config.target_position = MC_RUNTIME_DEFAULT_POSITION;
    runtime_config.step_position_a = MC_RUNTIME_DEFAULT_STEP_A;
    runtime_config.step_position_b = MC_RUNTIME_DEFAULT_STEP_B;
    runtime_config.step_period_ms = MC_RUNTIME_DEFAULT_STEP_PERIOD;
    return 1U;
}

/**
 * @brief 将电机、机械和弱磁参数恢复为默认值并保存到 FLASH
 * @return 1 表示成功，0 表示当前不处于 IDLE 或保存失败
 */
uint8_t mc_runtime_reset_persistent(void)
{
    const mc_motor_config_t old_motor = motor_config;
    const mc_tuning_config_t old_tuning = tuning_config;
    const uint8_t old_weakening = runtime_config.weakening_enabled;

    if (runtime_mode != MC_MODE_IDLE) {
        return 0U;
    }

    motor_config.resistance = MOTOR_RESISTANCE;
    motor_config.inductance_ld = MOTOR_INDUCTANCE_Ld;
    motor_config.inductance_lq = MOTOR_INDUCTANCE_Lq;
    motor_config.flux_linkage = MOTOR_FLUX_LINKAGE;
    motor_config.ka = MOTOR_KA;
    motor_config.kv = MOTOR_KV;
    motor_config.tc = MOTOR_TC;
    motor_config.angle_min = MOTOR_ANGLE_MIN;
    motor_config.angle_max = MOTOR_ANGLE_MAX;
    tuning_config.speed_kp = SPEED_PID_KP_DEFAULT;
    tuning_config.speed_ki = SPEED_PID_KI_DEFAULT;
    tuning_config.speed_kd = SPEED_PID_KD_DEFAULT;
    tuning_config.position_kp = POSITION_PID_KP_DEFAULT;
    tuning_config.position_ki = POSITION_PID_KI_DEFAULT;
    tuning_config.position_kd = POSITION_PID_KD_DEFAULT;
    tuning_config.cascade_kp = CASCADE_PID_KP_DEFAULT;
    tuning_config.cascade_ki = CASCADE_PID_KI_DEFAULT;
    tuning_config.cascade_kd = CASCADE_PID_KD_DEFAULT;
    runtime_config.weakening_enabled = 0U;

    if (runtime_save_to_flash() == 0U) {
        motor_config = old_motor;
        tuning_config = old_tuning;
        runtime_config.weakening_enabled = old_weakening;
        return 0U;
    }

    motor_ctrl.angle_min = motor_config.angle_min;
    motor_ctrl.angle_max = motor_config.angle_max;
    return 1U;
}
