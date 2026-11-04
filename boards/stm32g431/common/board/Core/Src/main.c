#include "main.h"

#include "user_main.h"

/**
 * @brief STM32 板级启动入口
 * @return 不返回
 */
int main(void)
{
    mc_init();
    user_main();

    for (;;) {
    }
}
