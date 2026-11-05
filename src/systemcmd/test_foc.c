#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "board_config.h"
#include "identify_parameter.h"
#include "mc_calibration.h"
#include "mc_control.h"
#include "mc_encoder.h"
#include "mc_parse.h"
#include "mc_runtime.h"
#include "printf.h"
#include "shell.h"

typedef struct {
    const char *name;
    int (*func)(int argc, char *argv[]);
} foc_cmd_t;

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
 * @brief 检查是否允许开始维护任务
 * @return 1 表示允许，0 表示拒绝
 */
static uint8_t foc_require_idle(void)
{
    if (mc_runtime_get_mode() != MC_MODE_IDLE) {
        printf("err:foc busy\r\n");
        return 0U;
    }
    if ((mc_encoder_calibration_is_running() != 0U) ||
        identify_parameter_is_active()) {
        printf("err:maintenance busy\r\n");
        return 0U;
    }
    return 1U;
}

/**
 * @brief 检查是否允许闭环运行
 * @return 1 表示允许，0 表示拒绝
 */
static uint8_t foc_require_ready(void)
{
    if (mc_runtime_is_ready() == 0U) {
        printf("err:foc not ready\r\n");
        return 0U;
    }
    if (motor_ctrl.fault_type != MOTOR_FAULT_NONE) {
        printf("err:foc fault=%d\r\n", motor_ctrl.fault_type);
        return 0U;
    }
    if ((mc_encoder_calibration_is_running() != 0U) ||
        identify_parameter_is_active()) {
        printf("err:maintenance busy\r\n");
        return 0U;
    }
    return 1U;
}

/**
 * @brief 打印 foc 命令帮助
 */
static void usage(void)
{
    printf("Usage: foc <cmd> [args]\r\n");
    printf(" curr <id> <iq>              - current control\r\n");
    printf(" speed <rpm>                 - speed control\r\n");
    printf(" pos <deg>                   - position-current control\r\n");
    printf(" splan <deg>                 - s-curve position-current control\r\n");
    printf(" cascade <deg>               - position-speed-current control\r\n");
    printf(" step <a_deg> <b_deg> <ms>   - position step\r\n");
    printf(" motor <R> <Ld> <Lq> <flux>  - motor parameters\r\n");
    printf(" weak <on|off>               - field weakening\r\n");
    printf(" stop                        - stop control\r\n");
    printf(" status                      - show status\r\n");
}

static int do_curr(int argc, char *argv[])
{
    float id;
    float iq;

    if ((argc < 3) || (mc_parse_float(argv[1], &id) == 0U) ||
        (mc_parse_float(argv[2], &iq) == 0U) ||
        (foc_require_ready() == 0U)) {
        return -1;
    }
    if ((mc_runtime_set_id(id) == 0U) || (mc_runtime_set_iq(iq) == 0U) ||
        (mc_control_start(MC_RUN_CURRENT) == 0U)) {
        printf("err:current\r\n");
        return -1;
    }
    printf("id:%.6f iq:%.6f\r\n", id, iq);
    return 0;
}

static int do_speed(int argc, char *argv[])
{
    float rpm;
    float speed_rad_s;

    if ((argc < 2) || (mc_parse_float(argv[1], &rpm) == 0U) ||
        (foc_require_ready() == 0U)) {
        return -1;
    }
    speed_rad_s = rpm * (_2PI / 60.0f);
    if ((mc_runtime_set_speed(speed_rad_s) == 0U) ||
        (mc_control_start(MC_RUN_SPEED) == 0U)) {
        printf("err:speed\r\n");
        return -1;
    }
    printf("speed:%.4frpm\r\n", rpm);
    return 0;
}

static int do_pos(int argc, char *argv[])
{
    float deg;
    float position_rad;

    if ((argc < 2) || (mc_parse_float(argv[1], &deg) == 0U) ||
        (foc_require_ready() == 0U)) {
        return -1;
    }
    position_rad = deg * (PI / 180.0f);
    if ((mc_runtime_set_position(position_rad) == 0U) ||
        (mc_control_start(MC_RUN_POSITION) == 0U)) {
        printf("err:position\r\n");
        return -1;
    }
    mc_ctrl.target_position = position_rad;
    printf("pos:%.4fdeg\r\n", deg);
    return 0;
}

static int do_splan(int argc, char *argv[])
{
    float deg;
    float position_rad;

    if ((argc < 2) || (mc_parse_float(argv[1], &deg) == 0U) ||
        (foc_require_ready() == 0U)) {
        return -1;
    }
    position_rad = deg * (PI / 180.0f);
    if ((mc_runtime_set_position(position_rad) == 0U) ||
        (mc_control_start(MC_RUN_POSITION) == 0U)) {
        printf("err:splan\r\n");
        return -1;
    }
    mc_ctrl.target_position = position_rad;
    printf("splan:%.4fdeg\r\n", deg);
    return 0;
}

static int do_cascade(int argc, char *argv[])
{
    float deg;
    float position_rad;

    if ((argc < 2) || (mc_parse_float(argv[1], &deg) == 0U) ||
        (foc_require_ready() == 0U)) {
        return -1;
    }
    position_rad = deg * (PI / 180.0f);
    if ((mc_runtime_set_position(position_rad) == 0U) ||
        (mc_control_start(MC_RUN_POSITION_CASCADE) == 0U)) {
        printf("err:cascade\r\n");
        return -1;
    }
    mc_ctrl.target_position = position_rad;
    printf("cascade:%.4fdeg\r\n", deg);
    return 0;
}

static int do_step(int argc, char *argv[])
{
    float position_a_deg;
    float position_b_deg;
    uint32_t period_ms;

    if ((argc < 4) || (mc_parse_float(argv[1], &position_a_deg) == 0U) ||
        (mc_parse_float(argv[2], &position_b_deg) == 0U) ||
        (parse_u32(argv[3], &period_ms) == 0U) ||
        (foc_require_ready() == 0U)) {
        return -1;
    }
    if ((mc_runtime_set_step(position_a_deg * (PI / 180.0f),
                             position_b_deg * (PI / 180.0f),
                             period_ms) == 0U) ||
        (mc_control_start(MC_RUN_POSITION_STEP) == 0U)) {
        printf("err:step\r\n");
        return -1;
    }
    printf("step:%.4f %.4f %lums\r\n", position_a_deg, position_b_deg,
           (unsigned long)period_ms);
    return 0;
}

static int do_motor(int argc, char *argv[])
{
    float resistance;
    float inductance_ld;
    float inductance_lq;
    float flux_linkage;

    if ((argc < 5) || (mc_parse_float(argv[1], &resistance) == 0U) ||
        (mc_parse_float(argv[2], &inductance_ld) == 0U) ||
        (mc_parse_float(argv[3], &inductance_lq) == 0U) ||
        (mc_parse_float(argv[4], &flux_linkage) == 0U) ||
        (foc_require_idle() == 0U)) {
        return -1;
    }
    if (mc_runtime_set_motor(resistance, inductance_ld, inductance_lq,
                             flux_linkage) == 0U) {
        printf("err:motor\r\n");
        return -1;
    }
    printf("motor:r=%.7f ld=%.9f lq=%.9f flux=%.9f\r\n", resistance,
           inductance_ld, inductance_lq, flux_linkage);
    return 0;
}

static int do_weak(int argc, char *argv[])
{
    uint8_t enabled;

    if (argc < 2) {
        return -1;
    }
    if ((strcmp(argv[1], "on") == 0) || (strcmp(argv[1], "1") == 0)) {
        enabled = 1U;
    }
    else if ((strcmp(argv[1], "off") == 0) || (strcmp(argv[1], "0") == 0)) {
        enabled = 0U;
    }
    else {
        return -1;
    }
    if (mc_runtime_set_weakening(enabled) == 0U) {
        printf("err:weak\r\n");
        return -1;
    }
    printf("weak:%s\r\n", (enabled != 0U) ? "on" : "off");
    return 0;
}

static int do_stop(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    mc_control_stop();
    identify_parameter_stop();
    identify_mechanical_stop();
    printf("foc stopped\r\n");
    return 0;
}

static int do_status(int argc, char *argv[])
{
    const mc_runtime_config_t *config = mc_runtime_get_config();
    const float speed_rpm = encoder_data.mechanical_speed * (60.0f / _2PI);
    const float position_deg = encoder_data.mechanical_position * (180.0f / PI);

    (void)argc;
    (void)argv;
    printf("mode:%d fault:%d profile:%d\r\n", mc_runtime_get_mode(),
           motor_ctrl.fault_type, mc_runtime_get_profile());
    printf("vbus:%.4fV angle:%.4f(raw:%u) speed:%.4frpm pos:%.4fdeg\r\n",
           mc_voltage_data.vbus, encoder_data.electrical_angle,
           encoder_data.raw_angle, speed_rpm, position_deg);
    printf("id_ref:%.6f iq_ref:%.6f id:%.6f iq:%.6f\r\n", config->target_id,
           config->target_iq, mc_ctrl.abc_dq.current_d,
           mc_ctrl.abc_dq.current_q);
    printf("ud:%.6f uq:%.6f\r\n", mc_ctrl.out_d, mc_ctrl.out_q);
    return 0;
}

static const foc_cmd_t foc_commands[] = {
    {"curr", do_curr},       {"speed", do_speed}, {"pos", do_pos},
    {"splan", do_splan},     {"cascade", do_cascade},
    {"step", do_step},       {"motor", do_motor},
    {"weak", do_weak},       {"stop", do_stop},
    {"status", do_status},
};

/**
 * @brief foc 命令入口
 * @param argc 参数数量
 * @param argv 参数数组
 */
void test_foc(int argc, char *argv[])
{
    int result = -1;

    if (argc < 2) {
        goto exit;
    }

    for (size_t i = 0U; i < sizeof(foc_commands) / sizeof(foc_commands[0]);
         i++) {
        if (strcmp(argv[1], foc_commands[i].name) == 0) {
            result = foc_commands[i].func(argc - 1, &argv[1]);
            if (result == 0) {
                printf("<OK:foc>\r\n");
                return;
            }
            break;
        }
    }
    printf("err:unknown cmd %s\r\n", argv[1]);

exit:
    usage();
    printf("<ERR:foc>\r\n");
}

#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-function-type"
#endif

SHELL_EXPORT_CMD(
    SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN) |
        SHELL_CMD_DISABLE_RETURN,
    foc, test_foc, FOC test commands);

#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
