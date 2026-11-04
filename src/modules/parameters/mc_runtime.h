/**
 * @file mc_runtime.h
 * @brief 电机运行状态、RAM 配置及 FLASH 参数持久化接口。
 */

#ifndef MC_RUNTIME_H
#define MC_RUNTIME_H

#include <stdint.h>

/**
 * @brief 电机运行模式
 */
typedef enum {
    MC_MODE_IDLE = 0, /**< 空闲或维护模式 */
    MC_MODE_RUNNING   /**< 闭环运行模式 */
} mc_mode_t;

/**
 * @brief 闭环运行控制类型
 */
typedef enum {
    MC_RUN_CURRENT = 0,   /**< 电流闭环 */
    MC_RUN_POSITION_STEP, /**< 位置阶跃 */
    MC_RUN_SPEED,         /**< 速度闭环 */
    MC_RUN_POSITION,      /**< 单目标位置闭环 */
    MC_RUN_POSITION_CASCADE /**< 位置、速度、电流三环闭环 */
} mc_run_profile_t;

/**
 * @brief RAM 中的运行配置
 */
typedef struct {
    float target_id;           /**< d 轴电流给定（A） */
    float target_iq;           /**< q 轴电流给定（A） */
    float target_speed;        /**< 机械速度给定（rad/s） */
    float target_position;     /**< S 曲线目标机械位置（rad） */
    float step_position_a;     /**< 位置阶跃端点 A（rad） */
    float step_position_b;     /**< 位置阶跃端点 B（rad） */
    uint32_t step_period_ms;   /**< 阶跃端点切换周期（ms） */
    uint8_t weakening_enabled; /**< 弱磁使能标志 */
} mc_runtime_config_t;

/**
 * @brief 当前生效的电机参数
 */
typedef struct {
    float resistance;    /**< 相电阻（ohm） */
    float inductance_ld; /**< d 轴电感（H） */
    float inductance_lq; /**< q 轴电感（H） */
    float flux_linkage;  /**< 永磁磁链（Wb） */
    float ka;            /**< 惯量等效系数（A*s^2/rad） */
    float kv;            /**< 粘滞阻尼等效系数（A*s/rad） */
    float tc;            /**< 库仑摩擦等效电流（A） */
    float angle_min;     /**< 机械角度下限（rad），0 表示不限制 */
    float angle_max;     /**< 机械角度上限（rad），0 表示不限制 */
} mc_motor_config_t;

/**
 * @brief 持久化的速度环、位置环和三环位置环 PID 参数
 */
typedef struct {
    float speed_kp;    /**< 速度环比例增益 */
    float speed_ki;    /**< 速度环积分增益 */
    float speed_kd;    /**< 速度环微分增益 */
    float position_kp; /**< 位置环比例增益 */
    float position_ki; /**< 位置环积分增益 */
    float position_kd; /**< 位置环微分增益 */
    float cascade_kp;  /**< 三环位置环比例增益 */
    float cascade_ki;  /**< 三环位置环积分增益 */
    float cascade_kd;  /**< 三环位置环微分增益 */
} mc_tuning_config_t;

/**
 * @brief FLASH 参数记录摘要
 */
typedef struct {
    mc_motor_config_t motor;   /**< 电机参数 */
    mc_tuning_config_t tuning; /**< 三组控制环 PID 参数 */
    uint16_t version;          /**< 记录版本 */
    uint8_t weakening_enabled; /**< 弱磁使能 */
} mc_runtime_flash_info_t;

/**
 * @brief 初始化运行状态并加载 FLASH 参数
 */
void mc_runtime_init(void);

/**
 * @brief 标记硬件初始化完成
 */
void mc_runtime_set_ready(void);

/**
 * @brief 查询硬件是否已就绪
 * @return 1 表示就绪，0 表示未就绪
 */
uint8_t mc_runtime_is_ready(void);

/**
 * @brief 获取当前运行模式
 * @return 当前运行模式
 */
mc_mode_t mc_runtime_get_mode(void);

/**
 * @brief 获取当前运行控制类型
 * @return 当前控制类型
 */
mc_run_profile_t mc_runtime_get_profile(void);

/**
 * @brief 获取 RAM 运行配置
 * @return 运行配置只读指针
 */
const mc_runtime_config_t *mc_runtime_get_config(void);

/**
 * @brief 获取当前电机参数
 * @return 电机参数只读指针
 */
const mc_motor_config_t *mc_runtime_get_motor_config(void);

/**
 * @brief 获取速度环、位置环和三环位置环 PID 参数
 * @return PID 参数只读指针
 */
const mc_tuning_config_t *mc_runtime_get_tuning_config(void);

/**
 * @brief 读取并校验 FLASH 参数记录
 * @param info 输出的记录摘要
 * @return 1 表示记录有效，0 表示无有效记录
 */
uint8_t mc_runtime_read_flash(mc_runtime_flash_info_t *info);

/**
 * @brief 查询弱磁是否启用
 * @return 1 表示启用，0 表示关闭
 */
uint8_t mc_runtime_weakening_is_enabled(void);

/**
 * @brief 设置并持久化弱磁使能状态
 * @param enabled 非零启用，零关闭
 * @return 1 表示成功，0 表示运行状态不允许或保存失败
 */
uint8_t mc_runtime_set_weakening(uint8_t enabled);

/**
 * @brief 设置 d 轴电流给定
 * @param target_id d 轴电流给定（A）
 * @return 1 表示成功，0 表示数值无效
 */
uint8_t mc_runtime_set_id(float target_id);

/**
 * @brief 设置 q 轴电流给定
 * @param target_iq q 轴电流给定（A）
 * @return 1 表示成功，0 表示数值无效
 */
uint8_t mc_runtime_set_iq(float target_iq);

/**
 * @brief 设置机械速度给定
 * @param target_speed 机械速度给定（rad/s）
 * @return 1 表示成功，0 表示数值无效
 */
uint8_t mc_runtime_set_speed(float target_speed);

/**
 * @brief 设置单目标位置
 * @param target_position 目标机械位置（rad）
 * @return 1 表示成功，0 表示数值无效
 */
uint8_t mc_runtime_set_position(float target_position);

/**
 * @brief 设置位置阶跃参数
 * @param position_a 阶跃端点 A（rad）
 * @param position_b 阶跃端点 B（rad）
 * @param period_ms 切换周期（ms）
 * @return 1 表示成功，0 表示参数无效
 */
uint8_t mc_runtime_set_step(float position_a, float position_b,
                            uint32_t period_ms);

/**
 * @brief 保存完整电机电气参数
 * @param resistance 相电阻（ohm）
 * @param inductance_ld d 轴电感（H）
 * @param inductance_lq q 轴电感（H）
 * @param flux_linkage 永磁磁链（Wb）
 * @return 1 表示保存成功，0 表示失败
 */
uint8_t mc_runtime_set_motor(float resistance, float inductance_ld,
                             float inductance_lq, float flux_linkage);

/**
 * @brief 仅更新并保存相电阻
 * @param resistance 相电阻（ohm）
 * @return 1 表示保存成功，0 表示失败
 */
uint8_t mc_runtime_set_motor_resistance(float resistance);

/**
 * @brief 将平均相电感同时写入 d/q 轴并保存
 * @param inductance 相电感（H）
 * @return 1 表示保存成功，0 表示失败
 */
uint8_t mc_runtime_set_motor_inductance(float inductance);

/**
 * @brief 保存机械辨识参数
 * @param ka 惯量等效系数（A*s^2/rad）
 * @param kv 粘滞阻尼等效系数（A*s/rad）
 * @param tc 库仑摩擦等效电流（A）
 * @return 1 表示保存成功，0 表示失败
 */
uint8_t mc_runtime_set_mechanical(float ka, float kv, float tc);

/**
 * @brief 保存机械角度上下限
 * @param angle_min 机械角度下限（rad），0 表示该方向不限制
 * @param angle_max 机械角度上限（rad），0 表示该方向不限制
 * @return 1 表示保存成功，0 表示运行状态或参数不允许
 */
uint8_t mc_runtime_set_angle_limits(float angle_min, float angle_max);

/**
 * @brief 保存速度环 PID 参数到 FLASH
 * @param kp 比例增益
 * @param ki 积分增益
 * @param kd 微分增益
 * @return 1 表示成功，0 表示运行状态或参数不允许
 */
uint8_t mc_runtime_set_speed_pid(float kp, float ki, float kd);

/**
 * @brief 保存位置环 PID 参数到 FLASH
 * @param kp 比例增益
 * @param ki 积分增益
 * @param kd 微分增益
 * @return 1 表示成功，0 表示运行状态或参数不允许
 */
uint8_t mc_runtime_set_position_pid(float kp, float ki, float kd);

/**
 * @brief 保存三环位置环 PID 参数到 FLASH
 * @param kp 比例增益
 * @param ki 积分增益
 * @param kd 微分增益
 * @return 1 表示成功，0 表示运行状态或参数不允许
 */
uint8_t mc_runtime_set_cascade_pid(float kp, float ki, float kd);

/**
 * @brief 进入 RUNNING 模式
 * @param profile 控制类型
 * @return 1 表示成功，0 表示状态不允许
 */
uint8_t mc_runtime_start(mc_run_profile_t profile);

/**
 * @brief 从 RUNNING 模式返回 IDLE
 */
void mc_runtime_stop(void);

/**
 * @brief 将 RAM 运行目标恢复为默认值
 * @return 1 表示成功，0 表示当前不处于 IDLE
 */
uint8_t mc_runtime_reset_config(void);

/**
 * @brief 将电机、机械和弱磁参数恢复为默认值并保存到 FLASH
 * @return 1 表示成功，0 表示当前不处于 IDLE 或保存失败
 */
uint8_t mc_runtime_reset_persistent(void);

#endif /* MC_RUNTIME_H */
