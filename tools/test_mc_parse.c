#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "mc_parse.h"

typedef struct {
    const char *text;
    int valid;
    float expected;
} parse_case_t;

int main(void)
{
    static const parse_case_t cases[] = {
        {"0", 1, 0.0f},          {"-0.5", 1, -0.5f},
        {"+1.25", 1, 1.25f},     {".25", 1, 0.25f},
        {"1.", 1, 1.0f},         {"1e-3", 1, 0.001f},
        {"-2.5E2", 1, -250.0f},  {"0.0024", 1, 0.0024f},
        {"0.000791111", 1, 0.000791111f},
        {"3.4e38", 1, 3.4e38f},  {"1e-38", 1, 1.0e-38f},
        {"", 0, 0.0f},           {"-", 0, 0.0f},
        {".", 0, 0.0f},          {"1e", 0, 0.0f},
        {"1e+", 0, 0.0f},        {"--1", 0, 0.0f},
        {"1.0x", 0, 0.0f},       {"nan", 0, 0.0f},
        {"inf", 0, 0.0f},        {"0x10", 0, 0.0f},
        {"1e39", 0, 0.0f},
    };

    for (size_t i = 0U; i < sizeof(cases) / sizeof(cases[0]); i++) {
        float value = 123.0f;
        const uint8_t result = mc_parse_float(cases[i].text, &value);

        if (result != (uint8_t)cases[i].valid) {
            fprintf(stderr, "validity mismatch: '%s', got %u\n",
                    cases[i].text, (unsigned int)result);
            return EXIT_FAILURE;
        }
        if ((result != 0U) && (fabsf(value - cases[i].expected) >
                               (5.0e-6f * fabsf(cases[i].expected)))) {
            fprintf(stderr, "value mismatch: '%s', got %.9g expected %.9g\n",
                    cases[i].text, (double)value, (double)cases[i].expected);
            return EXIT_FAILURE;
        }
    }

    puts("mc_parse: PASS");
    return EXIT_SUCCESS;
}
