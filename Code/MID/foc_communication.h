#ifndef FOC_COMMUNICATION_H
#define FOC_COMMUNICATION_H

#include "foc_prase.h"
#include "fdcan.h"
#include "foc_conversion.h"

extern uint8_t g_local_node_id;  // 本关节 ID

typedef struct
{
    float target_current;
    float target_angle;        // rad
    float target_velocity;     // rad/s
    float target_acceleration; // rad/s^2
    float target_jerk;         // rad/s^3
    float stiffness;
    float damping;
    uint8_t control_mode;
    uint8_t enabled;
} canfd_joint_cmd_t;

extern canfd_joint_cmd_t g_canfd_joint_cmd;

void foc_control_set(void);
void CAN_ReportStatus(void);
void CAN_Process(void);

#endif // FOC_COMMUNICATION_H
