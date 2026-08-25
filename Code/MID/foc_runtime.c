#include "foc_runtime.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "Config.h"
#include "stm32g4xx_hal.h"

#define FOC_RUNTIME_FLASH_ADDRESS 0x0801F000UL
#define FOC_RUNTIME_FLASH_PAGE    62U
#define FOC_RUNTIME_FLASH_MAGIC   0x464F4346UL
#define FOC_RUNTIME_FLASH_VERSION            4U
#define FOC_RUNTIME_FLASH_VERSION_PHYSICAL   3U
#define FOC_RUNTIME_FLASH_VERSION_ELECTRICAL 2U

#define FOC_RUNTIME_DEFAULT_ID          0.0f
#define FOC_RUNTIME_DEFAULT_IQ          0.5f
#define FOC_RUNTIME_DEFAULT_STEP_A      4.537856055f
#define FOC_RUNTIME_DEFAULT_STEP_B      (-4.537856055f)
#define FOC_RUNTIME_DEFAULT_STEP_PERIOD 2000U
#define FOC_RUNTIME_STEP_PERIOD_MIN     10U
#define FOC_RUNTIME_STEP_PERIOD_MAX     600000U

typedef struct
{
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
    uint32_t crc32;
} foc_runtime_flash_record_t;

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t record_size;
    float resistance;
    float inductance_ld;
    float inductance_lq;
    float flux_linkage;
    float inertia;
    float viscous_damping;
    float coulomb_friction;
    uint32_t crc32;
} foc_runtime_flash_record_physical_t;

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t record_size;
    float resistance;
    float inductance_ld;
    float inductance_lq;
    float flux_linkage;
    uint32_t crc32;
} foc_runtime_flash_record_electrical_t;

_Static_assert(sizeof(foc_runtime_flash_record_t) <= FLASH_PAGE_SIZE,
               "motor config record does not fit in one FLASH page");
_Static_assert(sizeof(foc_runtime_flash_record_t) ==
                   sizeof(foc_runtime_flash_record_physical_t),
               "v3 and v4 motor records must keep the same layout size");

static foc_runtime_config_t runtime_config;
static foc_motor_config_t motor_config;
static volatile foc_mode_t runtime_mode = FOC_MODE_IDLE;
static volatile foc_run_profile_t runtime_profile = FOC_RUN_CURRENT;
static volatile uint8_t runtime_ready = 0U;
static volatile uint8_t runtime_debug_enabled = 0U;
static foc_runtime_flash_record_t flash_write_buffer;

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
            crc = (crc >> 1U) ^
                (((crc & 1U) != 0U) ? 0xEDB88320UL : 0UL);
        }
    }

    return crc ^ 0xFFFFFFFFUL;
}

/**
 * @brief 校验电机参数是否处于允许范围
 * @param config 电机参数
 * @return 1 表示有效，0 表示无效
 */
static uint8_t motor_config_valid(const foc_motor_config_t *config)
{
    if ((!isfinite(config->resistance)) ||
        (!isfinite(config->inductance_ld)) ||
        (!isfinite(config->inductance_lq)) ||
        (!isfinite(config->flux_linkage)) ||
        (config->resistance <= 0.0f) || (config->resistance > 100.0f) ||
        (config->inductance_ld <= 0.0f) ||
        (config->inductance_ld > 1.0f) ||
        (config->inductance_lq <= 0.0f) ||
        (config->inductance_lq > 1.0f) ||
        (config->flux_linkage <= 0.0f) ||
        (config->flux_linkage > 1.0f) ||
        (!isfinite(config->ka)) || (!isfinite(config->kv)) ||
        (!isfinite(config->tc)) ||
        (config->ka <= 0.0f) || (config->ka > 100.0f) ||
        (config->kv < 0.0f) || (config->kv > 100.0f) ||
        (config->tc < 0.0f) || (config->tc > 100.0f)) {
        return 0U;
    }

    return 1U;
}

/**
 * @brief 校验 FLASH 中电机参数记录的格式和 CRC
 * @param record FLASH 记录
 * @return 1 表示有效，0 表示无效
 */
static uint8_t runtime_flash_record_valid(
    const foc_runtime_flash_record_t *record)
{
    foc_motor_config_t config;

    if ((record->magic != FOC_RUNTIME_FLASH_MAGIC) ||
        (record->version != FOC_RUNTIME_FLASH_VERSION) ||
        (record->record_size != sizeof(foc_runtime_flash_record_t)) ||
        (record->crc32 !=
         runtime_crc32(record,
                       offsetof(foc_runtime_flash_record_t, crc32)))) {
        return 0U;
    }

    config.resistance = record->resistance;
    config.inductance_ld = record->inductance_ld;
    config.inductance_lq = record->inductance_lq;
    config.flux_linkage = record->flux_linkage;
    config.ka = record->ka;
    config.kv = record->kv;
    config.tc = record->tc;
    return motor_config_valid(&config);
}

/**
 * @brief 校验保存物理 J/B/Tc 的 v3 FLASH 记录
 */
static uint8_t runtime_flash_physical_record_valid(
    const foc_runtime_flash_record_physical_t *record)
{
    float torque_constant;
    foc_motor_config_t config;

    if ((record->magic != FOC_RUNTIME_FLASH_MAGIC) ||
        (record->version != FOC_RUNTIME_FLASH_VERSION_PHYSICAL) ||
        (record->record_size != sizeof(foc_runtime_flash_record_physical_t)) ||
        (record->crc32 != runtime_crc32(
            record, offsetof(foc_runtime_flash_record_physical_t, crc32)))) {
        return 0U;
    }

    torque_constant = 1.5f * (float)MOTOR_POLE_PAIRS * record->flux_linkage;
    config.resistance = record->resistance;
    config.inductance_ld = record->inductance_ld;
    config.inductance_lq = record->inductance_lq;
    config.flux_linkage = record->flux_linkage;
    config.ka = record->inertia / torque_constant;
    config.kv = record->viscous_damping / torque_constant;
    config.tc = record->coulomb_friction / torque_constant;
    return motor_config_valid(&config);
}

/**
 * @brief 校验仅包含电气参数的 v2 FLASH 记录
 */
static uint8_t runtime_flash_electrical_record_valid(
    const foc_runtime_flash_record_electrical_t *record)
{
    if ((record->magic != FOC_RUNTIME_FLASH_MAGIC) ||
        (record->version != FOC_RUNTIME_FLASH_VERSION_ELECTRICAL) ||
        (record->record_size != sizeof(foc_runtime_flash_record_electrical_t)) ||
        (record->crc32 != runtime_crc32(
            record, offsetof(foc_runtime_flash_record_electrical_t, crc32))) ||
        (!isfinite(record->resistance)) ||
        (!isfinite(record->inductance_ld)) ||
        (!isfinite(record->inductance_lq)) ||
        (!isfinite(record->flux_linkage)) ||
        (record->resistance <= 0.0f) || (record->resistance > 100.0f) ||
        (record->inductance_ld <= 0.0f) ||
        (record->inductance_ld > 1.0f) ||
        (record->inductance_lq <= 0.0f) ||
        (record->inductance_lq > 1.0f) ||
        (record->flux_linkage <= 0.0f) ||
        (record->flux_linkage > 1.0f)) {
        return 0U;
    }

    return 1U;
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
    flash_write_buffer.magic = FOC_RUNTIME_FLASH_MAGIC;
    flash_write_buffer.version = FOC_RUNTIME_FLASH_VERSION;
    flash_write_buffer.record_size = sizeof(foc_runtime_flash_record_t);
    flash_write_buffer.resistance = motor_config.resistance;
    flash_write_buffer.inductance_ld = motor_config.inductance_ld;
    flash_write_buffer.inductance_lq = motor_config.inductance_lq;
    flash_write_buffer.flux_linkage = motor_config.flux_linkage;
    flash_write_buffer.ka = motor_config.ka;
    flash_write_buffer.kv = motor_config.kv;
    flash_write_buffer.tc = motor_config.tc;
    flash_write_buffer.crc32 = runtime_crc32(
        &flash_write_buffer, offsetof(foc_runtime_flash_record_t, crc32));

    if (HAL_FLASH_Unlock() != HAL_OK) {
        return 0U;
    }

    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks = FLASH_BANK_1;
    erase.Page = FOC_RUNTIME_FLASH_PAGE;
    erase.NbPages = 1U;
    status = HAL_FLASHEx_Erase(&erase, &page_error);

    for (uint32_t offset = 0U;
         (status == HAL_OK) && (offset < sizeof(flash_write_buffer));
         offset += 8U) {
        uint64_t double_word = 0xFFFFFFFFFFFFFFFFULL;
        uint32_t remaining = (uint32_t)sizeof(flash_write_buffer) - offset;
        uint32_t copy_size = (remaining < 8U) ? remaining : 8U;

        memcpy(&double_word,
               ((const uint8_t *)&flash_write_buffer) + offset,
               copy_size);
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                   FOC_RUNTIME_FLASH_ADDRESS + offset,
                                   double_word);
    }

    (void)HAL_FLASH_Lock();

    if ((status != HAL_OK) ||
        (memcmp((const void *)FOC_RUNTIME_FLASH_ADDRESS,
                &flash_write_buffer, sizeof(flash_write_buffer)) != 0)) {
        return 0U;
    }

    return 1U;
}

/**
 * @brief 初始化运行状态并加载持久化电机参数
 */
void foc_runtime_init(void)
{
    foc_runtime_flash_info_t flash_info;

    runtime_config.target_id = FOC_RUNTIME_DEFAULT_ID;
    runtime_config.target_iq = FOC_RUNTIME_DEFAULT_IQ;
    runtime_config.step_position_a = FOC_RUNTIME_DEFAULT_STEP_A;
    runtime_config.step_position_b = FOC_RUNTIME_DEFAULT_STEP_B;
    runtime_config.step_period_ms = FOC_RUNTIME_DEFAULT_STEP_PERIOD;

    motor_config.resistance = MOTOR_RESISTANCE;
    motor_config.inductance_ld = MOTOR_INDUCTANCE_Ld;
    motor_config.inductance_lq = MOTOR_INDUCTANCE_Lq;
    motor_config.flux_linkage = MOTOR_FLUX_LINKAGE;
    motor_config.ka = MOTOR_KA;
    motor_config.kv = MOTOR_KV;
    motor_config.tc = MOTOR_TC;

    if (foc_runtime_read_flash(&flash_info) != 0U) {
        motor_config.resistance = flash_info.motor.resistance;
        motor_config.inductance_ld = flash_info.motor.inductance_ld;
        motor_config.inductance_lq = flash_info.motor.inductance_lq;
        motor_config.flux_linkage = flash_info.motor.flux_linkage;
        if (flash_info.mechanical_stored != 0U) {
            motor_config.ka = flash_info.motor.ka;
            motor_config.kv = flash_info.motor.kv;
            motor_config.tc = flash_info.motor.tc;
        }
    }

    runtime_profile = FOC_RUN_CURRENT;
    runtime_mode = FOC_MODE_IDLE;
    runtime_ready = 0U;
    runtime_debug_enabled = 0U;
}

/**
 * @brief 标记电机硬件初始化完成
 */
void foc_runtime_set_ready(void)
{
    runtime_ready = 1U;
}

/**
 * @brief 查询电机硬件是否可运行
 * @return 1 表示已就绪，0 表示未就绪
 */
uint8_t foc_runtime_is_ready(void)
{
    return runtime_ready;
}

/**
 * @brief 获取当前 IDLE/RUNNING 模式
 * @return 当前运行模式
 */
foc_mode_t foc_runtime_get_mode(void)
{
    return runtime_mode;
}

/**
 * @brief 获取 RUNNING 模式的控制类型
 * @return 电流运行或位置阶跃
 */
foc_run_profile_t foc_runtime_get_profile(void)
{
    return runtime_profile;
}

/**
 * @brief 获取仅在 RAM 中保存的运行配置
 * @return 运行配置只读指针
 */
const foc_runtime_config_t *foc_runtime_get_config(void)
{
    return &runtime_config;
}

/**
 * @brief 获取当前生效的持久化电机参数
 * @return 电机参数只读指针
 */
const foc_motor_config_t *foc_runtime_get_motor_config(void)
{
    return &motor_config;
}

/**
 * @brief 校验并读取 FLASH 中实际保存的电机参数
 * @param info FLASH 记录信息输出；旧版记录不包含机械参数
 * @return 1 表示记录有效，0 表示无有效记录或参数为空
 */
uint8_t foc_runtime_read_flash(foc_runtime_flash_info_t *info)
{
    const foc_runtime_flash_record_t *record =
        (const foc_runtime_flash_record_t *)FOC_RUNTIME_FLASH_ADDRESS;
    const foc_runtime_flash_record_physical_t *physical_record =
        (const foc_runtime_flash_record_physical_t *)
            FOC_RUNTIME_FLASH_ADDRESS;
    const foc_runtime_flash_record_electrical_t *electrical_record =
        (const foc_runtime_flash_record_electrical_t *)
            FOC_RUNTIME_FLASH_ADDRESS;

    if (info == NULL) {
        return 0U;
    }
    memset(info, 0, sizeof(*info));

    if (runtime_flash_record_valid(record) != 0U) {
        info->version = record->version;
        info->mechanical_stored = 1U;
        info->motor.resistance = record->resistance;
        info->motor.inductance_ld = record->inductance_ld;
        info->motor.inductance_lq = record->inductance_lq;
        info->motor.flux_linkage = record->flux_linkage;
        info->motor.ka = record->ka;
        info->motor.kv = record->kv;
        info->motor.tc = record->tc;
        return 1U;
    }

    if (runtime_flash_physical_record_valid(physical_record) != 0U) {
        float torque_constant = 1.5f * (float)MOTOR_POLE_PAIRS *
            physical_record->flux_linkage;

        info->version = physical_record->version;
        info->mechanical_stored = 1U;
        info->motor.resistance = physical_record->resistance;
        info->motor.inductance_ld = physical_record->inductance_ld;
        info->motor.inductance_lq = physical_record->inductance_lq;
        info->motor.flux_linkage = physical_record->flux_linkage;
        info->motor.ka = physical_record->inertia / torque_constant;
        info->motor.kv = physical_record->viscous_damping / torque_constant;
        info->motor.tc = physical_record->coulomb_friction / torque_constant;
        return 1U;
    }

    if (runtime_flash_electrical_record_valid(electrical_record) != 0U) {
        info->version = electrical_record->version;
        info->motor.resistance = electrical_record->resistance;
        info->motor.inductance_ld = electrical_record->inductance_ld;
        info->motor.inductance_lq = electrical_record->inductance_lq;
        info->motor.flux_linkage = electrical_record->flux_linkage;
        return 1U;
    }

    return 0U;
}

/**
 * @brief 查询连续波形打印是否开启
 * @return 1 表示开启，0 表示关闭
 */
uint8_t foc_runtime_debug_is_enabled(void)
{
    return runtime_debug_enabled;
}

/**
 * @brief 设置连续波形打印开关
 * @param enabled 非零开启，零关闭
 */
void foc_runtime_set_debug(uint8_t enabled)
{
    runtime_debug_enabled = (enabled != 0U) ? 1U : 0U;
}

/**
 * @brief 在 IDLE 模式设置 RAM 中的 d 轴目标电流
 * @param target_id d 轴目标电流，单位 A
 * @return 1 表示成功，0 表示状态或数值无效
 */
uint8_t foc_runtime_set_id(float target_id)
{
    if ((runtime_mode != FOC_MODE_IDLE) || (!isfinite(target_id)) ||
        (target_id < -CURRENT_LIMIT) || (target_id > CURRENT_LIMIT)) {
        return 0U;
    }
    if (target_id == runtime_config.target_id) {
        return 1U;
    }

    runtime_config.target_id = target_id;
    return 1U;
}

/**
 * @brief 在 IDLE 模式设置 RAM 中的 q 轴目标电流
 * @param target_iq q 轴目标电流，单位 A
 * @return 1 表示成功，0 表示状态或数值无效
 */
uint8_t foc_runtime_set_iq(float target_iq)
{
    if ((runtime_mode != FOC_MODE_IDLE) || (!isfinite(target_iq)) ||
        (target_iq < -CURRENT_LIMIT) || (target_iq > CURRENT_LIMIT)) {
        return 0U;
    }
    if (target_iq == runtime_config.target_iq) {
        return 1U;
    }

    runtime_config.target_iq = target_iq;
    return 1U;
}

/**
 * @brief 在 IDLE 模式设置 RAM 中的位置阶跃参数
 * @param position_a 阶跃端点 A，单位 rad
 * @param position_b 阶跃端点 B，单位 rad
 * @param period_ms 端点切换周期，单位 ms
 * @return 1 表示成功，0 表示状态或数值无效
 */
uint8_t foc_runtime_set_step(float position_a, float position_b,
                             uint32_t period_ms)
{
    if ((runtime_mode != FOC_MODE_IDLE) || (!isfinite(position_a)) ||
        (!isfinite(position_b)) ||
        (period_ms < FOC_RUNTIME_STEP_PERIOD_MIN) ||
        (period_ms > FOC_RUNTIME_STEP_PERIOD_MAX)) {
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
uint8_t foc_runtime_set_motor(float resistance, float inductance_ld,
                              float inductance_lq, float flux_linkage)
{
    foc_motor_config_t old_config;
    foc_motor_config_t new_config = {
        .resistance = resistance,
        .inductance_ld = inductance_ld,
        .inductance_lq = inductance_lq,
        .flux_linkage = flux_linkage,
        .ka = motor_config.ka,
        .kv = motor_config.kv,
        .tc = motor_config.tc,
    };

    if ((runtime_mode != FOC_MODE_IDLE) ||
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
 * @brief 在 IDLE 模式保存机械辨识参数到 FLASH
 * @param ka J/Kt，单位 A*s^2/rad
 * @param kv B/Kt，单位 A*s/rad
 * @param tc 库仑摩擦等效电流，单位 A
 * @return 1 表示 FLASH 保存成功，0 表示失败
 */
uint8_t foc_runtime_set_mechanical(float ka, float kv, float tc)
{
    foc_motor_config_t old_config;
    foc_motor_config_t new_config = motor_config;

    new_config.ka = ka;
    new_config.kv = kv;
    new_config.tc = tc;
    if ((runtime_mode != FOC_MODE_IDLE) ||
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
 * @brief 执行不可逆的 IDLE 到 RUNNING 模式切换
 * @param profile RUNNING 模式使用的控制类型
 * @return 1 表示切换成功，0 表示状态不允许
 */
uint8_t foc_runtime_start(foc_run_profile_t profile)
{
    if ((runtime_ready == 0U) || (runtime_mode != FOC_MODE_IDLE) ||
        ((profile != FOC_RUN_CURRENT) &&
         (profile != FOC_RUN_POSITION_STEP))) {
        return 0U;
    }

    runtime_profile = profile;
    __DMB();
    runtime_mode = FOC_MODE_RUNNING;
    return 1U;
}
