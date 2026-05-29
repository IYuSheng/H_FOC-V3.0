#ifndef FOC_TRAPEZOID_PLANNER_H
#define FOC_TRAPEZOID_PLANNER_H

#include <stdint.h>

/**
 * @brief 在线梯形加减速规划器
 * @note  该规划器不预先生成完整轨迹，而是在每个控制周期根据目标位置、
 *        当前反馈位置和加速度限制实时计算下一步位置给定。适合 3D 打印机
 *        这类目标位置可能连续刷新的场景。
 */
typedef struct
{
    float max_velocity;        /**< 初始化设定的全局最大速度限制，单位与位置单位一致/s */
    float command_max_velocity; /**< 当前指令实际使用的最大速度，受 max_velocity 限幅 */
    float period;              /**< 控制周期，单位 s，例如 1kHz 位置环对应 0.001f */
    float position_setpoint;   /**< 当前规划输出位置，可作为位置环目标 */
    float velocity_setpoint;   /**< 当前规划输出速度，可用于速度前馈 */
    float acceleration_setpoint; /**< 当前规划输出加速度，可用于加速度前馈 */
    float target_position;     /**< 最近一次输入的目标位置 */
    float tolerance;           /**< 到位容差，误差小于该值且速度足够低时判定完成 */
    uint8_t active;            /**< 规划运行标志，1 表示正在运动，0 表示已到位/空闲 */
} TrapezoidPlanner_t;

/**
 * @brief 初始化梯形规划器
 * @param planner      规划器实例指针
 * @param max_velocity 最大速度限制，单位与位置单位一致/s
 * @param period       控制周期，单位 s
 */
void trapezoid_planner_init(TrapezoidPlanner_t *planner,
                            float max_velocity,
                            float period);

/**
 * @brief 复位规划器状态
 * @param planner          规划器实例指针
 * @param current_position 当前实际位置，复位后规划输出会对齐到该位置
 */
void trapezoid_planner_reset(TrapezoidPlanner_t *planner,
                             float current_position);

/**
 * @brief 修改最大速度限制
 * @param planner      规划器实例指针
 * @param max_velocity 最大速度限制，传入负数时会自动取绝对值
 */
void trapezoid_planner_set_max_velocity(TrapezoidPlanner_t *planner,
                                        float max_velocity);

/**
 * @brief 设置到位容差
 * @param planner   规划器实例指针
 * @param tolerance 到位容差，传入负数时会自动取绝对值
 */
void trapezoid_planner_set_tolerance(TrapezoidPlanner_t *planner,
                                     float tolerance);

/**
 * @brief 周期更新梯形加减速规划
 * @param planner          规划器实例指针
 * @param target_position  目标位置
 * @param current_position 当前实际位置反馈
 * @param command_max_velocity 当前指令最大速度，不能超过初始化设定的 max_velocity
 * @param acceleration     加速度限制，单位与位置单位一致/s^2
 * @return float           本周期输出的位置给定
 *
 * @note 该函数需要在固定周期中调用。内部保存上一周期速度，通过刹车距离
 *       v^2/(2a) 判断当前应加速、匀速还是减速。
 */
float trapezoid_planner_update(TrapezoidPlanner_t *planner,
                               float target_position,
                               float current_position,
                               float command_max_velocity,
                               float acceleration);

/**
 * @brief 判断规划是否完成
 * @param planner 规划器实例指针
 * @return uint8_t 1 表示已完成，0 表示仍在运动
 */
static inline uint8_t trapezoid_planner_is_done(const TrapezoidPlanner_t *planner)
{
    return (planner->active == 0U);
}

#endif /* FOC_TRAPEZOID_PLANNER_H */
