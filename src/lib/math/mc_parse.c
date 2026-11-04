/**
 * @file mc_parse.c
 * @brief Shell 参数轻量解析实现。
 */

#include "mc_parse.h"

#include <math.h>
#include <stddef.h>

#define MC_PARSE_MAX_DECIMAL_EXPONENT 38

/**
 * @brief 判断字符是否为十进制数字
 * @param character 输入字符
 * @return 1 表示是数字，0 表示不是
 */
static uint8_t mc_parse_is_digit(char character)
{
    return ((character >= '0') && (character <= '9')) ? 1U : 0U;
}

/**
 * @brief 严格解析十进制浮点数
 * @param text 输入字符串
 * @param value 解析结果
 * @return 1 表示成功，0 表示格式错误或结果超出 float 范围
 */
uint8_t mc_parse_float(const char *text, float *value)
{
    const char *cursor;
    float magnitude = 0.0f;
    uint32_t fraction_digits = 0U;
    int32_t exponent = 0;
    int32_t effective_exponent;
    uint8_t negative = 0U;
    uint8_t has_digit = 0U;

    if ((text == NULL) || (value == NULL)) {
        return 0U;
    }

    cursor = text;
    if ((*cursor == '+') || (*cursor == '-')) {
        negative = (*cursor == '-') ? 1U : 0U;
        cursor++;
    }

    while (mc_parse_is_digit(*cursor) != 0U) {
        has_digit = 1U;
        magnitude = (magnitude * 10.0f) + (float)(*cursor - '0');
        cursor++;
    }

    if (*cursor == '.') {
        cursor++;
        while (mc_parse_is_digit(*cursor) != 0U) {
            has_digit = 1U;
            magnitude = (magnitude * 10.0f) + (float)(*cursor - '0');
            fraction_digits++;
            cursor++;
        }
    }

    if (has_digit == 0U) {
        return 0U;
    }

    if ((*cursor == 'e') || (*cursor == 'E')) {
        uint8_t exponent_negative = 0U;
        uint8_t exponent_has_digit = 0U;

        cursor++;
        if ((*cursor == '+') || (*cursor == '-')) {
            exponent_negative = (*cursor == '-') ? 1U : 0U;
            cursor++;
        }
        while (mc_parse_is_digit(*cursor) != 0U) {
            exponent_has_digit = 1U;
            if (exponent <= MC_PARSE_MAX_DECIMAL_EXPONENT) {
                exponent = (exponent * 10) + (*cursor - '0');
            }
            cursor++;
        }
        if (exponent_has_digit == 0U) {
            return 0U;
        }
        if (exponent_negative != 0U) {
            exponent = -exponent;
        }
    }

    if (*cursor != '\0') {
        return 0U;
    }

    effective_exponent = exponent - (int32_t)fraction_digits;
    if ((effective_exponent > MC_PARSE_MAX_DECIMAL_EXPONENT) ||
        (effective_exponent < -MC_PARSE_MAX_DECIMAL_EXPONENT)) {
        return (magnitude == 0.0f) ? 1U : 0U;
    }

    while (effective_exponent > 0) {
        magnitude *= 10.0f;
        effective_exponent--;
    }
    while (effective_exponent < 0) {
        magnitude *= 0.1f;
        effective_exponent++;
    }

    if (!isfinite(magnitude)) {
        return 0U;
    }

    *value = (negative != 0U) ? -magnitude : magnitude;
    return 1U;
}
