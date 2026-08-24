#ifndef __FOC_LADRC_H
#define __FOC_LADRC_H

#include <stdint.h>

typedef struct {
    float z1;      /**< LESO estimated position, rad. */
    float z2;      /**< LESO estimated velocity, rad/s. */
    float z3;      /**< LESO estimated total disturbance, rad/s^2. */
    float output;  /**< q-axis current command, A. */
    uint8_t initialized;
} PositionLadrc_t;

void position_ladrc_reset(PositionLadrc_t *controller, float position_feedback);
float position_ladrc_update(PositionLadrc_t *controller,
                            float target_position,
                            float target_velocity,
                            float target_acceleration,
                            float position_feedback);

#endif /* __FOC_LADRC_H */
