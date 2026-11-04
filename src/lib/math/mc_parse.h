/**
 * @file mc_parse.h
 * @brief Shell 参数轻量解析接口。
 */

#ifndef MC_PARSE_H_
#define MC_PARSE_H_

#include <stdint.h>

/**
 * @brief 严格解析十进制浮点数
 * @param text 输入字符串
 * @param value 解析结果
 * @return 1 表示成功，0 表示格式错误或结果超出 float 范围
 * @note 支持符号、小数点和 e/E 指数，不支持十六进制和 inf/nan。
 */
uint8_t mc_parse_float(const char *text, float *value);

#endif /* MC_PARSE_H_ */
