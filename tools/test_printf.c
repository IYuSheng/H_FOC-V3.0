#include <stdio.h>
#include <string.h>

#include "printf.h"

void _putchar(char character)
{
    (void)character;
}

int main(void)
{
    static const char expected[] = "I 1.25 0.002400 42 1234567890 A5";
    char buffer[96];
    const int length = __wrap_snprintf(buffer, sizeof(buffer),
                                       "%-1s %.2f %.6f %u %lu %02X", "I",
                                       1.25f, 0.0024f, 42U, 1234567890UL, 0xA5U);

    if ((length != (int)(sizeof(expected) - 1U)) ||
        (strcmp(buffer, expected) != 0)) {
        puts("mc_printf: FAIL");
        return 1;
    }

    puts("mc_printf: PASS");
    return 0;
}
