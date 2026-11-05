#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "board_config.h"
#include "mc_parse.h"
#include "mc_runtime.h"
#include "printf.h"
#include "shell.h"

typedef struct {
    const char *name;
    float value;
    float default_value;
} float_param_t;

/**
 * @brief 严格解析无符号整数
 * @param token 参数字符串
 * @param value 解析结果
 * @return 1 表示成功，0 表示失败
 */
static uint8_t parse_u32(const char *token, uint32_t *value)
{
    char *end;
    unsigned long parsed;

    if ((token == NULL) || (value == NULL) || (*token == '-')) {
        return 0U;
    }
    parsed = strtoul(token, &end, 10);
    if ((end == token) || (*end != '\0') || (parsed > UINT32_MAX)) {
        return 0U;
    }
    *value = (uint32_t)parsed;
    return 1U;
}

/**
 * @brief 判断浮点参数是否已偏离默认值
 * @param value 当前值
 * @param default_value 默认值
 * @return 1 表示已修改，0 表示未修改
 */
static uint8_t float_param_changed(float value, float default_value)
{
    return fabsf(value - default_value) > 1.0e-9f;
}

/**
 * @brief 打印参数列表
 * @param changed_only 是否只显示相对默认值变化的参数
 */
static void param_list(uint8_t changed_only)
{
    const mc_motor_config_t *motor = mc_runtime_get_motor_config();
    const mc_tuning_config_t *tuning = mc_runtime_get_tuning_config();
    const mc_shaper_config_t *shaper = mc_runtime_get_shaper_config();
    const float_param_t parameters[] = {
        {"RESISTANCE", motor->resistance, MOTOR_RESISTANCE},
        {"INDUCTANCE_LD", motor->inductance_ld, MOTOR_INDUCTANCE_Ld},
        {"INDUCTANCE_LQ", motor->inductance_lq, MOTOR_INDUCTANCE_Lq},
        {"FLUX_LINKAGE", motor->flux_linkage, MOTOR_FLUX_LINKAGE},
        {"KA", motor->ka, MOTOR_KA},
        {"KV", motor->kv, MOTOR_KV},
        {"TC", motor->tc, MOTOR_TC},
        {"ANGLE_MIN", motor->angle_min, MOTOR_ANGLE_MIN},
        {"ANGLE_MAX", motor->angle_max, MOTOR_ANGLE_MAX},
        {"SPEED_KP", tuning->speed_kp, SPEED_PID_KP_DEFAULT},
        {"SPEED_KI", tuning->speed_ki, SPEED_PID_KI_DEFAULT},
        {"SPEED_KD", tuning->speed_kd, SPEED_PID_KD_DEFAULT},
        {"POS_KP", tuning->position_kp, POSITION_PID_KP_DEFAULT},
        {"POS_KI", tuning->position_ki, POSITION_PID_KI_DEFAULT},
        {"POS_KD", tuning->position_kd, POSITION_PID_KD_DEFAULT},
        {"CASCADE_KP", tuning->cascade_kp, CASCADE_PID_KP_DEFAULT},
        {"CASCADE_KI", tuning->cascade_ki, CASCADE_PID_KI_DEFAULT},
        {"CASCADE_KD", tuning->cascade_kd, CASCADE_PID_KD_DEFAULT},
        {"SHAPER_FREQ_HZ", shaper->frequency_hz, MOTOR_SHAPER_FREQUENCY_HZ},
        {"SHAPER_DAMPING", shaper->damping_ratio, MOTOR_SHAPER_DAMPING_RATIO},
    };
    int count = 0;

    printf("S %-18s %-4s %s\r\n", "NAME", "TYPE", "VALUE");
    printf("- ------------------ ---- ----------\r\n");
    for (size_t i = 0U; i < sizeof(parameters) / sizeof(parameters[0]); i++) {
        const uint8_t changed =
            float_param_changed(parameters[i].value, parameters[i].default_value);

        if ((changed_only != 0U) && (changed == 0U)) {
            continue;
        }
        printf("%c %-18s %-4s %.9f\r\n", (changed != 0U) ? 'x' : ' ',
               parameters[i].name, "F32", parameters[i].value);
        count++;
    }

    if ((shaper->type != (uint8_t)MOTOR_SHAPER_TYPE) ||
        (changed_only == 0U)) {
        printf("%c %-18s %-4s %u\r\n",
               (shaper->type != (uint8_t)MOTOR_SHAPER_TYPE) ? 'x' : ' ',
               "SHAPER_TYPE", "U8", (unsigned int)shaper->type);
        count++;
    }

    if (mc_runtime_weakening_is_enabled() != 0U) {
        printf("%c %-18s %-4s %u\r\n", 'x', "WEAKENING_ENABLED", "U8",
               mc_runtime_weakening_is_enabled());
        count++;
    }
    else if (changed_only == 0U) {
        printf("  %-18s %-4s %u\r\n", "WEAKENING_ENABLED", "U8", 0U);
        count++;
    }

    if (count == 0) {
        printf("(no changed parameters)\r\n");
    }
}

/**
 * @brief 设置单个 FLASH 参数
 * @param name 参数名
 * @param value 参数值字符串
 * @return 0 表示成功，-1 表示失败
 */
static int param_set(const char *name, const char *value)
{
    const mc_motor_config_t *motor = mc_runtime_get_motor_config();
    const mc_tuning_config_t *tuning = mc_runtime_get_tuning_config();
    float number = 0.0f;
    uint32_t integer = 0U;

    if ((name == NULL) || (value == NULL)) {
        return -1;
    }

    if ((strcmp(name, "RESISTANCE") == 0) ||
        (strcmp(name, "INDUCTANCE_LD") == 0) ||
        (strcmp(name, "INDUCTANCE_LQ") == 0) ||
        (strcmp(name, "FLUX_LINKAGE") == 0) ||
        (strcmp(name, "KA") == 0) || (strcmp(name, "KV") == 0) ||
        (strcmp(name, "TC") == 0) || (strcmp(name, "SPEED_KP") == 0) ||
        (strcmp(name, "ANGLE_MIN") == 0) ||
        (strcmp(name, "ANGLE_MAX") == 0) ||
        (strcmp(name, "SPEED_KI") == 0) ||
        (strcmp(name, "SPEED_KD") == 0) ||
        (strcmp(name, "POS_KP") == 0) || (strcmp(name, "POS_KI") == 0) ||
        (strcmp(name, "POS_KD") == 0) ||
        (strcmp(name, "CASCADE_KP") == 0) ||
        (strcmp(name, "CASCADE_KI") == 0) ||
        (strcmp(name, "CASCADE_KD") == 0) ||
        (strcmp(name, "SHAPER_FREQ_HZ") == 0) ||
        (strcmp(name, "SHAPER_DAMPING") == 0)) {
        if (mc_parse_float(value, &number) == 0U) {
            return -1;
        }
    }
    else if (strcmp(name, "WEAKENING_ENABLED") == 0) {
        if ((parse_u32(value, &integer) == 0U) || (integer > 1U)) {
            return -1;
        }
    }
    else if (strcmp(name, "SHAPER_TYPE") == 0) {
        if ((parse_u32(value, &integer) == 0U) || (integer > 6U)) {
            return -1;
        }
    }
    else {
        return -1;
    }

    if (strcmp(name, "RESISTANCE") == 0) {
        return mc_runtime_set_motor_resistance(number) != 0U ? 0 : -1;
    }
    if (strcmp(name, "INDUCTANCE_LD") == 0) {
        return mc_runtime_set_motor(motor->resistance, number,
                                    motor->inductance_lq,
                                    motor->flux_linkage) != 0U
            ? 0
            : -1;
    }
    if (strcmp(name, "INDUCTANCE_LQ") == 0) {
        return mc_runtime_set_motor(motor->resistance,
                                    motor->inductance_ld, number,
                                    motor->flux_linkage) != 0U
            ? 0
            : -1;
    }
    if (strcmp(name, "FLUX_LINKAGE") == 0) {
        return mc_runtime_set_motor(motor->resistance, motor->inductance_ld,
                                    motor->inductance_lq, number) != 0U
            ? 0
            : -1;
    }
    if ((strcmp(name, "KA") == 0) || (strcmp(name, "KV") == 0) ||
        (strcmp(name, "TC") == 0)) {
        const float ka = (strcmp(name, "KA") == 0) ? number : motor->ka;
        const float kv = (strcmp(name, "KV") == 0) ? number : motor->kv;
        const float tc = (strcmp(name, "TC") == 0) ? number : motor->tc;
        return mc_runtime_set_mechanical(ka, kv, tc) != 0U ? 0 : -1;
    }
    if ((strcmp(name, "ANGLE_MIN") == 0) ||
        (strcmp(name, "ANGLE_MAX") == 0)) {
        const float angle_min =
            (strcmp(name, "ANGLE_MIN") == 0) ? number : motor->angle_min;
        const float angle_max =
            (strcmp(name, "ANGLE_MAX") == 0) ? number : motor->angle_max;
        return mc_runtime_set_angle_limits(angle_min, angle_max) != 0U ? 0
                                                                      : -1;
    }
    if ((strcmp(name, "SPEED_KP") == 0) ||
        (strcmp(name, "SPEED_KI") == 0) ||
        (strcmp(name, "SPEED_KD") == 0)) {
        const float kp =
            (strcmp(name, "SPEED_KP") == 0) ? number : tuning->speed_kp;
        const float ki =
            (strcmp(name, "SPEED_KI") == 0) ? number : tuning->speed_ki;
        const float kd =
            (strcmp(name, "SPEED_KD") == 0) ? number : tuning->speed_kd;
        return mc_runtime_set_speed_pid(kp, ki, kd) != 0U ? 0 : -1;
    }
    if ((strcmp(name, "POS_KP") == 0) || (strcmp(name, "POS_KI") == 0) ||
        (strcmp(name, "POS_KD") == 0)) {
        const float kp =
            (strcmp(name, "POS_KP") == 0) ? number : tuning->position_kp;
        const float ki =
            (strcmp(name, "POS_KI") == 0) ? number : tuning->position_ki;
        const float kd =
            (strcmp(name, "POS_KD") == 0) ? number : tuning->position_kd;
        return mc_runtime_set_position_pid(kp, ki, kd) != 0U ? 0 : -1;
    }
    if ((strcmp(name, "CASCADE_KP") == 0) ||
        (strcmp(name, "CASCADE_KI") == 0) ||
        (strcmp(name, "CASCADE_KD") == 0)) {
        const float kp =
            (strcmp(name, "CASCADE_KP") == 0) ? number : tuning->cascade_kp;
        const float ki =
            (strcmp(name, "CASCADE_KI") == 0) ? number : tuning->cascade_ki;
        const float kd =
            (strcmp(name, "CASCADE_KD") == 0) ? number : tuning->cascade_kd;
        return mc_runtime_set_cascade_pid(kp, ki, kd) != 0U ? 0 : -1;
    }
    if ((strcmp(name, "SHAPER_TYPE") == 0) ||
        (strcmp(name, "SHAPER_FREQ_HZ") == 0) ||
        (strcmp(name, "SHAPER_DAMPING") == 0)) {
        const mc_shaper_config_t *shaper = mc_runtime_get_shaper_config();
        const uint8_t type = (strcmp(name, "SHAPER_TYPE") == 0)
            ? (uint8_t)integer
            : shaper->type;
        const float frequency_hz = (strcmp(name, "SHAPER_FREQ_HZ") == 0)
            ? number
            : shaper->frequency_hz;
        const float damping_ratio = (strcmp(name, "SHAPER_DAMPING") == 0)
            ? number
            : shaper->damping_ratio;

        return mc_runtime_set_shaper(type, frequency_hz, damping_ratio) != 0U
            ? 0
            : -1;
    }
    if (strcmp(name, "WEAKENING_ENABLED") == 0) {
        return mc_runtime_set_weakening(integer) != 0U ? 0 : -1;
    }
    return -1;
}

/**
 * @brief 打印 param 命令帮助
 */
static void usage(void)
{
    printf("param <list|set|reset>\r\n");
    printf(" list [-c]\r\n");
    printf(" set <param> <val>\r\n");
    printf(" reset\r\n");
    printf("<ERR:param>\r\n");
}

/**
 * @brief param 命令入口
 * @param argc 参数数量
 * @param argv 参数数组
 */
void param(int argc, char *argv[])
{
    int result = -1;

    if (argc < 2) {
        goto exit;
    }
    if (strcmp(argv[1], "list") == 0) {
        if ((argc == 2) ||
            ((argc == 3) && (strcmp(argv[2], "-c") == 0))) {
            param_list((argc == 3) ? 1U : 0U);
            result = 0;
        }
    }
    else if ((strcmp(argv[1], "set") == 0) && (argc == 4)) {
        result = param_set(argv[2], argv[3]);
    }
    else if ((strcmp(argv[1], "reset") == 0) && (argc == 2)) {
        result = mc_runtime_reset_persistent() != 0U ? 0 : -1;
    }

    if (result == 0) {
        printf("<OK:param>\r\n");
        return;
    }

exit:
    usage();
}

#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-function-type"
#endif

SHELL_EXPORT_CMD(
    SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN) |
        SHELL_CMD_DISABLE_RETURN,
    param, param, param command);

#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
