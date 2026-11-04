/**
 * @file cpu.h
 * @brief CPU 占用率统计接口。
 */

#ifndef CPU_H_
#define CPU_H_

#include <stdint.h>

/**
 * @brief 初始化 CPU 占用率统计并校准满速循环计数
 * @note 初始化过程中会短暂关闭中断并执行约 100 ms 基准测试。
 */
void cpu_usage_init(void);

/**
 * @brief 获取最近一次计算的 CPU 占用率
 * @return CPU 占用率，范围 0~1000，单位 0.1%
 */
uint32_t cpu_usage_ratio_get(void);

/**
 * @brief 计算 CPU 占用率
 * @note 该函数需要在主循环中持续调用。
 */
void cpu_usage_calculate(void);

/**
 * @brief 打印当前 CPU 占用率
 */
void cpu_usage_printf(void);

/**
 * @brief 设置 CPU 占用率自动打印周期
 * @param ms 自动打印周期，单位 ms；设置为 0 时关闭自动打印
 */
void cpu_set_print_interval(uint32_t ms);

#endif /* CPU_H_ */
