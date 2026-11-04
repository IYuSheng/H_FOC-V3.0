/**
 * @file cpu.c
 * @brief 基于空闲循环计数和 SysTick 的 CPU 占用率统计。
 */

#include "cpu.h"

#include "printf.h"
#include "stm32g4xx_hal.h"

/** 满速测试循环次数，保证单次执行时间约为 0.1~0.5 ms */
#define CPU_USAGE_LOOP_COUNT 3000U
/** CPU 占用率统计窗口，单位 ms */
#define CPU_USAGE_WINDOW_MS  1000U

typedef struct {
    uint32_t tick_start;
    uint32_t full_speed_count;
    uint32_t last_load_run_count;
    uint32_t load_run_count;
    uint32_t usage_ratio;
    uint32_t print_interval;
    uint32_t last_print_time;
} cpu_usage_t;

static cpu_usage_t s_cpu_usage;

/**
 * @brief 关闭全局中断并保存原始中断状态
 * @return 关闭中断前的 PRIMASK 值
 */
static uint32_t cpu_irq_lock(void)
{
    const uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

/**
 * @brief 恢复原始全局中断状态
 * @param primask 关闭中断前保存的 PRIMASK 值
 */
static void cpu_irq_unlock(uint32_t primask)
{
    if (primask == 0U) {
        __enable_irq();
    }
}

#pragma GCC push_options
#pragma GCC optimize("O0")
/**
 * @brief 执行固定次数的空循环，用于测量 CPU 剩余执行能力
 */
static void cpu_usage_ticking(void)
{
    volatile uint32_t index;

    for (index = 0U; index < CPU_USAGE_LOOP_COUNT; index++) {
        __NOP();
    }
}
#pragma GCC pop_options

/**
 * @brief 初始化 CPU 占用率统计并校准满速循环计数
 */
void cpu_usage_init(void)
{
    uint32_t time_now;
    uint32_t time_old;
    uint32_t time_count = 0U;
    uint32_t run_count = 0U;
    uint32_t primask;
    uint32_t reload;
    uint32_t calibration_ticks;

    s_cpu_usage.print_interval = 0U;
    s_cpu_usage.last_print_time = 0U;
    s_cpu_usage.last_load_run_count = 0U;
    s_cpu_usage.load_run_count = 0U;
    s_cpu_usage.usage_ratio = 0U;
    s_cpu_usage.full_speed_count = 0U;

    primask = cpu_irq_lock();
    time_old = SysTick->VAL;
    reload = SysTick->LOAD;
    calibration_ticks = reload * (CPU_USAGE_WINDOW_MS / 10U);

    while (time_count < calibration_ticks) {
        cpu_usage_ticking();
        run_count++;

        time_now = SysTick->VAL;
        if (time_now <= time_old) {
            time_count += time_old - time_now;
        }
        else {
            time_count += reload - time_now + time_old;
        }
        time_old = time_now;
    }
    cpu_irq_unlock(primask);

    s_cpu_usage.full_speed_count = run_count * 10U;
    s_cpu_usage.tick_start = HAL_GetTick();
}

/**
 * @brief 计算 CPU 占用率
 * @note CPU 占用率 = 1 - 负载循环计数 / 满速循环计数。
 */
void cpu_usage_calculate(void)
{
    const uint32_t now = HAL_GetTick();

    if ((uint32_t)(now - s_cpu_usage.tick_start) < CPU_USAGE_WINDOW_MS) {
        cpu_usage_ticking();
        s_cpu_usage.load_run_count++;
    }
    else {
        s_cpu_usage.last_load_run_count = s_cpu_usage.load_run_count;
        if ((s_cpu_usage.full_speed_count == 0U) ||
            (s_cpu_usage.last_load_run_count >=
             s_cpu_usage.full_speed_count)) {
            s_cpu_usage.usage_ratio =
                (s_cpu_usage.full_speed_count == 0U) ? 0U : 1000U;
        }
        else {
            s_cpu_usage.usage_ratio =
                ((s_cpu_usage.full_speed_count -
                  s_cpu_usage.last_load_run_count) *
                 1000U) /
                s_cpu_usage.full_speed_count;
        }

        s_cpu_usage.tick_start = now;
        s_cpu_usage.load_run_count = 0U;
    }

    if ((s_cpu_usage.print_interval != 0U) &&
        ((uint32_t)(now - s_cpu_usage.last_print_time) >=
         s_cpu_usage.print_interval)) {
        s_cpu_usage.last_print_time = now;
        cpu_usage_printf();
    }
}

/**
 * @brief 获取最近一次计算的 CPU 占用率
 * @return CPU 占用率，范围 0~1000，单位 0.1%
 */
uint32_t cpu_usage_ratio_get(void)
{
    return s_cpu_usage.usage_ratio;
}

/**
 * @brief 打印当前 CPU 占用率
 */
void cpu_usage_printf(void)
{
    printf("[%lu] 1-%lu/%lu= %lu.%lu %%\r\n",
           (unsigned long)HAL_GetTick(),
           (unsigned long)s_cpu_usage.last_load_run_count,
           (unsigned long)s_cpu_usage.full_speed_count,
           (unsigned long)(s_cpu_usage.usage_ratio / 10U),
           (unsigned long)(s_cpu_usage.usage_ratio % 10U));
}

/**
 * @brief 设置 CPU 占用率自动打印周期
 * @param ms 自动打印周期，单位 ms；设置为 0 时关闭自动打印
 */
void cpu_set_print_interval(uint32_t ms)
{
    s_cpu_usage.print_interval = ms;
    s_cpu_usage.last_print_time = HAL_GetTick();
}
