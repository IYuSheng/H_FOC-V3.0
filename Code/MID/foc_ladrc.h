#ifndef __FOC_LADRC_H
#define __FOC_LADRC_H

#include <stdint.h>

typedef struct {
    float z1;     /**< LESO estimated position, rad. */
    float z2;     /**< LESO estimated velocity, rad/s. */
    float z3;     /**< LESO estimated total disturbance, rad/s^2. */
    float output; /**< q-axis current command, A. */
    uint8_t initialized;
} PositionLadrc_t;

typedef struct {
    float z1;     /**< LESO estimated mechanical speed, rad/s. */
    float z2;     /**< LESO estimated total disturbance, rad/s^2. */
    float output; /**< q-axis current command, A. */
    uint8_t initialized;
} SpeedLadrc_t;

void position_ladrc_reset(PositionLadrc_t *controller, float position_feedback);
float position_ladrc_update(PositionLadrc_t *controller, float target_position,
                            float target_velocity, float target_acceleration,
                            float position_feedback);

void speed_ladrc_reset(SpeedLadrc_t *controller, float speed_feedback);
float speed_ladrc_update(SpeedLadrc_t *controller, float target_speed,
                         float target_acceleration,
                         float speed_feedback);

#endif /* __FOC_LADRC_H */
