#include "foc_prase.h"

#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "Config.h"
#include "foc_control.h"
#include "foc_encoder_calibration.h"
#include "foc_mechanical_param.h"
#include "foc_runtime.h"
#include "stm32g4xx.h"
#include "usart.h"

#define SHELL_MAX_TOKENS 8U

/**
 * @brief 严格解析一个有限浮点数参数
 * @param token 参数字符串
 * @param value 解析结果
 * @return 1 表示成功，0 表示格式无效
 */
static uint8_t parse_float_token(const char *token, float *value)
{
    char *end;

    if ((token == NULL) || (value == NULL)) {
        return 0U;
    }

    errno = 0;
    *value = strtof(token, &end);
    return ((end != token) && (*end == '\0') && (errno != ERANGE) &&
            isfinite(*value))
        ? 1U
        : 0U;
}

/**
 * @brief 严格解析一个无符号整数参数
 * @param token 参数字符串
 * @param value 解析结果
 * @return 1 表示成功，0 表示格式无效
 */
static uint8_t parse_u32_token(const char *token, uint32_t *value)
{
    char *end;
    unsigned long parsed;

    if ((token == NULL) || (value == NULL) || (*token == '-')) {
        return 0U;
    }

    errno = 0;
    parsed = strtoul(token, &end, 10);
    if ((end == token) || (*end != '\0') || (errno == ERANGE) ||
        (parsed > UINT32_MAX)) {
        return 0U;
    }

    *value = (uint32_t)parsed;
    return 1U;
}

/**
 * @brief 检查命令是否可在当前维护状态执行
 * @return 1 表示处于空闲且无维护任务，0 表示拒绝
 */
static uint8_t shell_require_idle(void)
{
    if (foc_runtime_get_mode() != FOC_MODE_IDLE) {
        debug_log("<ERR:state RUNNING; reboot required>");
        return 0U;
    }
    if ((foc_encoder_calibration_is_running() != 0U) ||
        foc_mechanical_param_is_active()) {
        debug_log("<ERR:maintenance busy>");
        return 0U;
    }
    return 1U;
}

/**
 * @brief 输出当前模式、运行配置和电机参数
 */
static void shell_print_status(void)
{
    const foc_runtime_config_t *config = foc_runtime_get_config();
    const foc_motor_config_t *motor = foc_runtime_get_motor_config();
    const char *mode = (foc_runtime_get_mode() == FOC_MODE_IDLE)
        ? "IDLE"
        : "RUNNING";
    const char *profile = (foc_runtime_get_profile() == FOC_RUN_CURRENT)
        ? "current"
        : "step";
    const char *activity = (foc_encoder_calibration_is_running() != 0U)
        ? "encoder_calibration"
        : (foc_mechanical_param_is_active() ? "mech_identification" : "none");

    debug_log("<STATUS:mode=%s ready=%u profile=%s activity=%s debug=%u>", mode,
              foc_runtime_is_ready(), profile,
              activity, foc_runtime_debug_is_enabled());
    debug_log("<CURRENT:id=%.4f iq=%.4f>", config->target_id,
              config->target_iq);
    debug_log("<STEP:a=%.4f b=%.4f period_ms=%lu>",
              config->step_position_a, config->step_position_b,
              (unsigned long)config->step_period_ms);
    debug_log("<MOTOR:r=%.7f ld=%.9f lq=%.9f flux=%.9f>",
              motor->resistance, motor->inductance_ld,
              motor->inductance_lq, motor->flux_linkage);
    debug_log("<MECH:Ka=%.9f Kv=%.9f Tc=%.9f>", motor->ka,
              motor->kv, motor->tc);
}

/**
 * @brief 校验并输出 FLASH 中实际保存的电机参数
 */
static void shell_print_flash(void)
{
    foc_runtime_flash_info_t info;

    if (foc_runtime_read_flash(&info) == 0U) {
        debug_log("<FLASH:valid=0>");
        return;
    }

    debug_log("<FLASH:valid=1 version=%u>", (unsigned int)info.version);
    debug_log("<FLASH_MOTOR:R=%.7f Ld=%.9f Lq=%.9f flux=%.9f>",
              info.motor.resistance, info.motor.inductance_ld,
              info.motor.inductance_lq, info.motor.flux_linkage);
    if (info.mechanical_stored != 0U) {
        debug_log("<FLASH_MECH:Ka=%.9f Kv=%.9f Tc=%.9f>",
                  info.motor.ka, info.motor.kv, info.motor.tc);
    }
    else {
        debug_log("<FLASH_MECH:not stored in version %u>",
                  (unsigned int)info.version);
    }
}

/**
 * @brief 输出串口 shell 命令列表
 */
static void shell_print_help(void)
{
    debug_log("set id <A> | set iq <A>");
    debug_log("set step <pos_a_rad> <pos_b_rad> <period_ms>");
    debug_log("set motor <R_ohm> <Ld_H> <Lq_H> <flux_Wb>");
    debug_log("run | step start | debug on|off");
    debug_log("calib encoder | identify mech | flash show");
    debug_log("status | reboot | help");
}

/**
 * @brief 解析并执行一行串口 shell 命令
 * @param data 可写的命令缓冲区
 * @param len 命令长度
 */
void parse_received_data(char *data, uint8_t len)
{
    char *tokens[SHELL_MAX_TOKENS];
    uint8_t token_count = 0U;
    char *token;

    if ((data == NULL) || (len == 0U)) {
        return;
    }
    data[len] = '\0';

    token = strtok(data, " \t");
    while ((token != NULL) && (token_count < SHELL_MAX_TOKENS)) {
        tokens[token_count++] = token;
        token = strtok(NULL, " \t");
    }
    if ((token_count == 0U) || (token != NULL)) {
        debug_log("<ERR:args>");
        return;
    }

    if ((token_count == 1U) && (strcmp(tokens[0], "help") == 0)) {
        shell_print_help();
        return;
    }
    if ((token_count == 1U) && (strcmp(tokens[0], "status") == 0)) {
        shell_print_status();
        return;
    }
    if ((token_count == 2U) && (strcmp(tokens[0], "flash") == 0) &&
        (strcmp(tokens[1], "show") == 0)) {
        shell_print_flash();
        return;
    }
    if ((token_count == 1U) && (strcmp(tokens[0], "reboot") == 0)) {
        debug_log("<OK:reboot>");
        bsp_uart_wait_tx_complete();
        NVIC_SystemReset();
        return;
    }

    if ((token_count == 2U) && (strcmp(tokens[0], "debug") == 0)) {
        if (strcmp(tokens[1], "on") == 0) {
            foc_runtime_set_debug(1U);
            debug_log("<OK:debug on>");
        }
        else if (strcmp(tokens[1], "off") == 0) {
            foc_runtime_set_debug(0U);
            debug_log("<OK:debug off>");
        }
        else {
            debug_log("<ERR:args>");
        }
        return;
    }

    if ((token_count == 2U) && (strcmp(tokens[0], "calib") == 0) &&
        (strcmp(tokens[1], "encoder") == 0)) {
        if (shell_require_idle() == 0U) {
            return;
        }
        if (foc_runtime_is_ready() == 0U) {
            debug_log("<ERR:not ready>");
        }
        else if (foc_encoder_calibration_start() != 0U) {
            debug_log("<OK:calib encoder>");
        }
        else {
            debug_log("<ERR:calibration busy>");
        }
        return;
    }

    if ((token_count == 2U) && (strcmp(tokens[0], "identify") == 0) &&
        (strcmp(tokens[1], "mech") == 0)) {
        if (shell_require_idle() == 0U) {
            return;
        }
        if (foc_control_start_mechanical_identification() != 0U) {
            debug_log("<OK:identify mech started>");
        }
        else {
            debug_log("<ERR:not ready>");
        }
        return;
    }

    if ((token_count == 1U) && (strcmp(tokens[0], "run") == 0)) {
        if (shell_require_idle() == 0U) {
            return;
        }
        if (foc_control_start(FOC_RUN_CURRENT) != 0U) {
            debug_log("<OK:RUNNING current>");
        }
        else {
            debug_log("<ERR:not ready or calibration busy>");
        }
        return;
    }

    if ((token_count == 2U) && (strcmp(tokens[0], "step") == 0) &&
        (strcmp(tokens[1], "start") == 0)) {
        if (shell_require_idle() == 0U) {
            return;
        }
        if (foc_control_start(FOC_RUN_POSITION_STEP) != 0U) {
            debug_log("<OK:RUNNING step>");
        }
        else {
            debug_log("<ERR:not ready or calibration busy>");
        }
        return;
    }

    if ((token_count == 3U) && (strcmp(tokens[0], "set") == 0) &&
        ((strcmp(tokens[1], "id") == 0) ||
         (strcmp(tokens[1], "iq") == 0))) {
        float value;

        if (shell_require_idle() == 0U) {
            return;
        }
        if ((parse_float_token(tokens[2], &value) == 0U) ||
            (value < -CURRENT_LIMIT) || (value > CURRENT_LIMIT)) {
            debug_log("<ERR:value range %.2f..%.2f A>", -CURRENT_LIMIT,
                      CURRENT_LIMIT);
            return;
        }

        if (strcmp(tokens[1], "id") == 0) {
            (void)foc_runtime_set_id(value);
            debug_log("<OK:set id %.4f>", value);
        }
        else {
            (void)foc_runtime_set_iq(value);
            debug_log("<OK:set iq %.4f>", value);
        }
        return;
    }

    if ((token_count == 5U) && (strcmp(tokens[0], "set") == 0) &&
        (strcmp(tokens[1], "step") == 0)) {
        float position_a;
        float position_b;
        uint32_t period_ms;

        if (shell_require_idle() == 0U) {
            return;
        }
        if ((parse_float_token(tokens[2], &position_a) == 0U) ||
            (parse_float_token(tokens[3], &position_b) == 0U) ||
            (parse_u32_token(tokens[4], &period_ms) == 0U) ||
            (period_ms < 10U) || (period_ms > 600000U)) {
            debug_log("<ERR:value; period_ms range 10..600000>");
            return;
        }

        (void)foc_runtime_set_step(position_a, position_b, period_ms);
        debug_log("<OK:set step %.4f %.4f %lu>", position_a, position_b,
                  (unsigned long)period_ms);
        return;
    }

    if ((token_count == 6U) && (strcmp(tokens[0], "set") == 0) &&
        (strcmp(tokens[1], "motor") == 0)) {
        float resistance;
        float inductance_ld;
        float inductance_lq;
        float flux_linkage;

        if (shell_require_idle() == 0U) {
            return;
        }
        if ((parse_float_token(tokens[2], &resistance) == 0U) ||
            (parse_float_token(tokens[3], &inductance_ld) == 0U) ||
            (parse_float_token(tokens[4], &inductance_lq) == 0U) ||
            (parse_float_token(tokens[5], &flux_linkage) == 0U) ||
            (resistance <= 0.0f) || (resistance > 100.0f) ||
            (inductance_ld <= 0.0f) || (inductance_ld > 1.0f) ||
            (inductance_lq <= 0.0f) || (inductance_lq > 1.0f) ||
            (flux_linkage <= 0.0f) || (flux_linkage > 1.0f)) {
            debug_log("<ERR:motor values out of range>");
            return;
        }

        if (foc_runtime_set_motor(resistance, inductance_ld, inductance_lq,
                                  flux_linkage) != 0U) {
            debug_log("<OK:set motor>");
        }
        else {
            debug_log("<ERR:FLASH write failed>");
        }
        return;
    }

    debug_log("<ERR:unknown command; use help>");
}
