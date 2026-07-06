#ifndef __FOC_LADRC_H
#define __FOC_LADRC_H

#include <stdint.h>
#include "Config.h"

typedef struct {
    float td_x1;
    float td_x2;
    float eso_z1;
    float eso_z2;
    float eso_z3;
    float u;
} LADRC_t;

typedef struct {
    float integral;
    float eso_z1;
    float eso_z2;
    float error;
    float u_pi;
    float u_comp;
    float u;
    uint8_t inited;
} SpeedEsoPi_t;

void ladrc_init(LADRC_t *ladrc);
float ladrc_calc(LADRC_t *ladrc, float ref, float feedback);
float ladrc_calc_ex(LADRC_t *ladrc,
                    float ref,
                    float feedback,
                    float bw,
                    float ts,
                    float b0,
                    float out_min,
                    float out_max);

void speed_eso_pi_init(SpeedEsoPi_t *ctrl);
void speed_eso_pi_reset(SpeedEsoPi_t *ctrl, float feedback);
float speed_eso_pi_calc(SpeedEsoPi_t *ctrl, float ref, float feedback);
float speed_eso_pi_calc_ex(SpeedEsoPi_t *ctrl,
                           float ref,
                           float feedback,
                           float kp,
                           float ki,
                           float integral_limit,
                           float wo,
                           float ts,
                           float b0,
                           float comp_gain,
                           float out_min,
                           float out_max);

#endif /* __FOC_LADRC_H */
