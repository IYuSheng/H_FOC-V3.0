#include "foc_ladrc.h"

static inline float ladrc_limit(float value, float min, float max)
{
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

void ladrc_init(LADRC_t *ladrc)
{
    if (!ladrc) return;

    ladrc->td_x1 = 0.0f;
    ladrc->td_x2 = 0.0f;
    ladrc->eso_z1 = 0.0f;
    ladrc->eso_z2 = 0.0f;
    ladrc->eso_z3 = 0.0f;
    ladrc->u = 0.0f;
}

float ladrc_calc(LADRC_t *ladrc, float ref, float feedback)
{
    return ladrc_calc_ex(ladrc,
                         ref,
                         feedback,
                         LADRC_BW,
                         LADRC_TS,
                         LADRC_B0,
                         LADRC_OUT_MIN,
                         LADRC_OUT_MAX);
}

float ladrc_calc_ex(LADRC_t *ladrc,
                    float ref,
                    float feedback,
                    float bw,
                    float ts,
                    float b0,
                    float out_min,
                    float out_max)
{
    if (!ladrc) return 0.0f;
    if (b0 > -EPS_F && b0 < EPS_F) return 0.0f;

    float td_r = bw * 2.0f;
    float td_fh = (td_r * td_r) * (ref - ladrc->td_x1) -
                  (2.0f * td_r * ladrc->td_x2);

    ladrc->td_x1 += ts * ladrc->td_x2;
    ladrc->td_x2 += ts * td_fh;

    float beta1 = 3.0f * bw;
    float beta2 = 3.0f * bw * bw;
    float beta3 = bw * bw * bw;
    float e = ladrc->eso_z1 - feedback;

    ladrc->eso_z1 += ts * (ladrc->eso_z2 - beta1 * e);
    ladrc->eso_z2 += ts * (ladrc->eso_z3 - beta2 * e + b0 * ladrc->u);
    ladrc->eso_z3 += ts * (-beta3 * e);

    float kp = bw;
    float kd = 0.5f * bw;
    float e1 = ladrc->td_x1 - ladrc->eso_z1;
    float e2 = ladrc->td_x2 - ladrc->eso_z2;
    float u0 = kp * e1 + kd * e2;

    ladrc->u = u0 - ladrc->eso_z3 / b0;
    ladrc->u = ladrc_limit(ladrc->u, out_min, out_max);

    return ladrc->u;
}

void speed_eso_pi_init(SpeedEsoPi_t *ctrl)
{
    speed_eso_pi_reset(ctrl, 0.0f);
}

void speed_eso_pi_reset(SpeedEsoPi_t *ctrl, float feedback)
{
    if (!ctrl) return;

    ctrl->integral = 0.0f;
    ctrl->eso_z1 = feedback;
    ctrl->eso_z2 = 0.0f;
    ctrl->error = 0.0f;
    ctrl->u_pi = 0.0f;
    ctrl->u_comp = 0.0f;
    ctrl->u = 0.0f;
    ctrl->inited = 1U;
}

float speed_eso_pi_calc(SpeedEsoPi_t *ctrl, float ref, float feedback)
{
    return speed_eso_pi_calc_ex(ctrl,
                                ref,
                                feedback,
                                SPEED_P_GAIN,
                                SPEED_I_GAIN,
                                SPEED_I_LIMIT,
                                SPEED_ESO_WO,
                                SPEED_LOOP_DT,
                                SPEED_ESO_B0,
                                SPEED_ESO_COMP_GAIN,
                                SPEED_ESO_OUT_MIN,
                                SPEED_ESO_OUT_MAX);
}

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
                           float out_max)
{
    if (!ctrl) return 0.0f;
    if (ts <= 0.0f) return 0.0f;

    if (!ctrl->inited) {
        speed_eso_pi_reset(ctrl, feedback);
    }

    if ((b0 > -EPS_F) && (b0 < EPS_F)) {
        b0 = 0.0f;
    }

    if (wo > 0.0f) {
        const float beta1 = 2.0f * wo;
        const float beta2 = wo * wo;
        const float e_eso = ctrl->eso_z1 - feedback;

        ctrl->eso_z1 += ts * (ctrl->eso_z2 - beta1 * e_eso + b0 * ctrl->u);
        ctrl->eso_z2 += ts * (-beta2 * e_eso);
    } else {
        ctrl->eso_z1 = feedback;
        ctrl->eso_z2 = 0.0f;
    }

    ctrl->error = ref - feedback;

    float integral_next = ctrl->integral + ki * ctrl->error;
    integral_next = ladrc_limit(integral_next, -integral_limit, integral_limit);

    const float u_pi_next = kp * ctrl->error + integral_next;
    float u_comp = 0.0f;
    if (b0 != 0.0f) {
        u_comp = comp_gain * ctrl->eso_z2 / b0;
    }

    const float u_unsat_next = u_pi_next - u_comp;
    const uint8_t saturated_high = (u_unsat_next > out_max);
    const uint8_t saturated_low = (u_unsat_next < out_min);

    if ((!saturated_high && !saturated_low) ||
        (saturated_high && (ctrl->error < 0.0f)) ||
        (saturated_low && (ctrl->error > 0.0f))) {
        ctrl->integral = integral_next;
    }

    ctrl->u_pi = kp * ctrl->error + ctrl->integral;
    ctrl->u_comp = u_comp;
    ctrl->u = ladrc_limit(ctrl->u_pi - ctrl->u_comp, out_min, out_max);

    return ctrl->u;
}
