#include "foc_vibration_sweep.h"
#include "foc_control.h"
#include <string.h>

resonance_test_t g_res_test = {0};
EI_Shaper_t g_ei_shaper;

/**
 * @brief EI (Extra Insensitive) 输入整形器初始化
 * 脉冲序列:
 *     A1          A2          A3
 *     |           |           |
 *     v           v           v
 *  ----|-----------|-----------|----> 时间
 *     t1=0        t2=Td       t3=2*Td
 * 
 * @param freq_hz      系统共振频率(Hz)
 * @param V            允许的残留振荡百分比(0~1)，建议0.05~0.1
 *                     V=0时退化为ZV整形器（理论零残留）
 * @param sample_time_ms 控制周期(ms)，如1kHz控制频率则为1.0ms
 */
void ei_shaper_init(float freq_hz, float V, float sample_time_s)
{
    float Td;
    uint32_t i;

    g_ei_shaper.init = 0;
    g_ei_shaper.buf_idx = 0;
    g_ei_shaper.buf_size = 1;
    memset(g_ei_shaper.buffer, 0, sizeof(g_ei_shaper.buffer));

    if ((freq_hz <= 0.0f) || (sample_time_s <= 0.0f)) {
        g_ei_shaper.A[0] = 1.0f;
        g_ei_shaper.A[1] = 0.0f;
        g_ei_shaper.A[2] = 0.0f;
        g_ei_shaper.delay[0] = 0U;
        g_ei_shaper.delay[1] = 0U;
        g_ei_shaper.delay[2] = 0U;
        g_ei_shaper.init = 1;
        return;
    }

    if (V < 0.0f) {
        V = 0.0f;
    } else if (V > 1.0f) {
        V = 1.0f;
    }

    /* EI整形器三脉冲间隔，Td为半个振动周期 */
    Td = 0.5f / freq_hz;

    g_ei_shaper.A[0] = (1.0f + V) / 4.0f;
    g_ei_shaper.A[1] = (1.0f - V) / 2.0f;
    g_ei_shaper.A[2] = (1.0f + V) / 4.0f;

    g_ei_shaper.delay[0] = 0U;
    g_ei_shaper.delay[1] = (uint32_t)((Td / sample_time_s) + 0.5f);
    g_ei_shaper.delay[2] = 2U * g_ei_shaper.delay[1];

    if (g_ei_shaper.delay[2] >= (uint32_t)(sizeof(g_ei_shaper.buffer) / sizeof(g_ei_shaper.buffer[0]))) {
        g_ei_shaper.delay[2] = (uint32_t)(sizeof(g_ei_shaper.buffer) / sizeof(g_ei_shaper.buffer[0])) - 1U;
        g_ei_shaper.delay[1] = g_ei_shaper.delay[2] / 2U;
    }

    g_ei_shaper.buf_size = g_ei_shaper.delay[2] + 1U;
    if (g_ei_shaper.buf_size == 0U) {
        g_ei_shaper.buf_size = 1U;
    }

    for (i = 0U; i < g_ei_shaper.buf_size; i++) {
        g_ei_shaper.buffer[i] = 0.0f;
    }

    g_ei_shaper.init = 1;
}

void ei_shaper_reset(float value)
{
    uint32_t i;

    if (!g_ei_shaper.init) {
        return;
    }

    for (i = 0U; i < g_ei_shaper.buf_size; i++) {
        g_ei_shaper.buffer[i] = value;
    }

    g_ei_shaper.buf_idx = 0U;
}

// EI整形处理
float ei_shaper_process(float input)
{
    if (!g_ei_shaper.init) return input;
    
    // 存入当前输入
    g_ei_shaper.buffer[g_ei_shaper.buf_idx] = input;
    
    // 计算三个脉冲的贡献
    float output = 0;
    for (int i = 0; i < 3; i++) {
        int32_t read_idx = (int32_t)g_ei_shaper.buf_idx - (int32_t)g_ei_shaper.delay[i];
        if (read_idx < 0) read_idx += g_ei_shaper.buf_size;
        output += g_ei_shaper.A[i] * g_ei_shaper.buffer[read_idx];
    }
    
    // 更新索引
    g_ei_shaper.buf_idx++;
    if (g_ei_shaper.buf_idx >= g_ei_shaper.buf_size) {
        g_ei_shaper.buf_idx = 0;
    }
    
    return output;
}
