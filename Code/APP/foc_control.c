#include "foc_control.h"

extern pi_t id_pid;
extern pi_t iq_pid;

#if FOC_SHAPER_ENABLE &&                                                       \
    (FOC_TRAJECTORY_PLANNER_TYPE == FOC_TRAJECTORY_PLANNER_S_CURVE)
SCurve_Planner_t g_pos_planner;
#elif FOC_SHAPER_ENABLE &&                                                     \
    (FOC_TRAJECTORY_PLANNER_TYPE == FOC_TRAJECTORY_PLANNER_TRAPEZOID)
TrapezoidPlanner_t g_pos_planner;
#endif

#if FOC_SPEED_ESO_PI_ENABLE
static SpeedEsoPi_t speed_eso_pi;
#elif FOC_SPEED_LADRC_ENABLE
static LADRC_t speed_ladrc;
#endif
#if FOC_POSITION_LADRC_ENABLE
static LADRC_t position_ladrc;
#endif

static inline void foc_open_loop_control(float target_speed, float target_outq);
static inline void foc_current_control(float target_d, float target_q);
static inline void foc_speed_control(float target_speed);
static inline void foc_position_control(float target_position);

/**
 * @brief FOC打印调试信息
 */
void foc_debug(void)
{
#if FOC_ENCODER_CALIBRATION_ENABLE && !FOC_ENCODER_CALIBRATION_USE_CONST_TABLE
    foc_encoder_calibration_debug_log();
#else
#if FOC_DEBUG_POSITION_TOGGLE_ENABLE && FOC_POSITION_CONTROL_ENABLE
    static uint16_t position_toggle_tick = 0U;
    static uint8_t position_toggle_state = 0U;

    if (++position_toggle_tick >= FOC_DEBUG_POSITION_TOGGLE_TICKS) {
        position_toggle_tick = 0U;
        position_toggle_state ^= 1U;
        foc_ctrl.target_position = (position_toggle_state != 0U)
            ? FOC_DEBUG_POSITION_TARGET_B
            : FOC_DEBUG_POSITION_TARGET_A;
    }
#endif

    // debug_log("%.4f, %.4f, %.4f, %.4f, %.4f", foc_ctrl.target_q,
    // foc_ctrl.abc_dq.current_q, foc_ctrl.shaped_t_p,
    // encoder_data.mechanical_speed, encoder_data.mechanical_position);
    debug_log("%.4f, %.4f, %.4f, %.4f", foc_ctrl.target_q,
              foc_ctrl.abc_dq.current_q, foc_ctrl.target_speed,
              encoder_data.mechanical_speed);
    // debug_log("%u, %u, %u, %u", spwm.duty_An, spwm.duty_Ap, spwm.duty_Bn,
    // spwm.duty_Bp); debug_log("%.4f, %.4f, %.4f", alpha_beta.alpha,
    // alpha_beta.beta, foc_voltage_data.vbus); debug_log("%.4f, %.4f, %.4f,
    // %.4f", foc_current_data.IA, foc_current_data.IB,
    // foc_ctrl.abc_dq.current_q, foc_ctrl.abc_dq.current_d); debug_log("%.4f,
    // %.4f, %.4f, %.4f", foc_ctrl.abc_dq.current_q, g_flux_obs.omega_filt,
    // angle_normalize_360(rad2deg(g_flux_obs.theta_hat)), foc_ctrl.angle);
    // debug_log("%.4f", foc_voltage_data.vbus);

    // debug_log("%.4f, %.4f, %.4f, %.4f, %.4f",
    //       foc_ctrl.target_q,
    //       encoder_data.mechanical_speed,
    //       speed_eso_pi.eso_z1,
    //       speed_eso_pi.eso_z2,
    //       -speed_eso_pi.eso_z2 / SPEED_ESO_B0);

    // 电磁转矩计算
    // foc_ctrl.Te = 1.5f * MOTOR_POLE_PAIRS * (MOTOR_FLUX_LINKAGE *
    // foc_ctrl.abc_dq.current_q); debug_log("%.4f", foc_ctrl.Te)

    // debug_log("%.4f, %.4f, %.4f", encoder_data.mechanical_position,
    // encoder_data.mechanical_speed, encoder_data.electrical_angle);
    // debug_log("%d", encoder_read_raw_angle());

#if FOC_TEST_ENABLE // 扫频测试
    // q轴正弦波扫频输出
    static float sine_angle = 0.0f;
    const float amplitude = 0.1f;

    // 计算当前正弦值并设置到q轴输出
    foc_ctrl.target_q = amplitude * sinf(sine_angle);

    // 角度每次增加1度（弧度制）
    sine_angle += 0.0174533f; // 1° = π/180 弧度

    // 限制角度在0-2π范围内
    if (sine_angle >= 6.283185f) {
        sine_angle -= 6.283185f;
    }
#endif
#endif
}

/**
 * @brief FOC初始化，锁定电机至零位
 */
void foc_start_init(void)
{
    foc_ctrl.target_speed = 200.0f;     // 设置目标速度(°/s)
    foc_ctrl.out_q = 0.0f;              // 设置q轴输出电压(开环用)
    foc_ctrl.out_d = 0.3f;              // 设置d轴输出电压(开环用)
    foc_ctrl.target_q = 0.1f;           // 设置目标Q轴电流
    foc_ctrl.target_d = 0.0f;           // 设置目标D轴电流
    foc_ctrl.target_position = 1000.0f; // 设置目标位置
    // 初始化电流环PI参数
    foc_current_pi_init(I_D_P_GAIN, I_Q_P_GAIN, I_I_GAIN, I_I_LIMIT);
    // 初始化速度环PI参数
    foc_speed_pi_init(SPEED_P_GAIN, SPEED_I_GAIN, SPEED_I_LIMIT);
#if FOC_SPEED_ESO_PI_ENABLE
    speed_eso_pi_reset(&speed_eso_pi, encoder_data.mechanical_speed);
#elif FOC_SPEED_LADRC_ENABLE
    ladrc_init(&speed_ladrc);
#endif
    // 初始化位置环PI参数
    foc_position_pi_init(POSITION_P_GAIN, POSITION_I_GAIN, POSITION_D_GAIN,
                         POSITION_I_LIMIT);
#if FOC_POSITION_LADRC_ENABLE
    ladrc_init(&position_ladrc);
#endif

    // 锁定电机至零位
    get_foc_bus_voltage(); // 获取电机母线电压
    debug_log("Vbus: %.2f V", foc_voltage_data.vbus);
    // 电流偏置校准
    adc1_current_offset_calibrate();

#if FOC_SHAPER_ENABLE
// 轨迹规划器，具体类型由 FOC_TRAJECTORY_PLANNER_TYPE 选择
#if (FOC_TRAJECTORY_PLANNER_TYPE == FOC_TRAJECTORY_PLANNER_S_CURVE)
    // S曲线轨迹规划 (最大速度 最大加速度 最大加加速度 Ts)
    s_curve_planner_init(&g_pos_planner, FOC_S_CURVE_MAX_VELOCITY,
                         FOC_S_CURVE_MAX_ACCELERATION, FOC_S_CURVE_MAX_JERK,
                         POSITION_LOOP_DT);
#if FOC_EI_SHAPER_ENABLE
    ei_shaper_init(FOC_EI_SHAPER_FREQ_HZ, FOC_EI_SHAPER_V, POSITION_LOOP_DT);
#endif
#elif (FOC_TRAJECTORY_PLANNER_TYPE == FOC_TRAJECTORY_PLANNER_TRAPEZOID)
    // 梯形轨迹规划 (全局最大速度 Ts)，每条指令最大速度在 update 中传入
    trapezoid_planner_init(&g_pos_planner, FOC_TRAPEZOID_MAX_VELOCITY,
                           POSITION_LOOP_DT);
    trapezoid_planner_set_tolerance(&g_pos_planner, FOC_TRAPEZOID_TOLERANCE);
#endif
#endif

#if FLUX_OBSERVER_ENABLE
    // 初始化磁链观测器
    flux_observer_init(&g_flux_obs);
#endif

#if 0 // 电机零位校准时启用
    LL_TIM_EnableIT_CC4(TIM8);
    foc_ctrl.out_q = 1.5f; // 初始q轴电压
    //  将电角度设为90度(直接当作驱动电机角度)锁定电机至零位
    float sin_theta, cos_theta;
    CORDIC_SinCos_Deg(90.0f, &sin_theta, &cos_theta);
    inv_park_transform_f32(&foc_ctrl, &alpha_beta, sin_theta, cos_theta);
    svpwm.sector = svpwm_sector_calc(&alpha_beta);
    svpwm_calc_times(&alpha_beta, &svpwm, foc_voltage_data.vbus);
    svpwm_duty_calc(&svpwm);
    bsp_pwm_set_duty_three_phase(svpwm.pwm_a, svpwm.pwm_b, svpwm.pwm_c);
    HAL_Delay(2000); // 等待电机稳定
    debug_log("%f", encoder_data.mechanical_angle);
#endif
}

/**
 * @brief FOC外环控制主函数 1kHZ运行
 */
void foc_control(void)
{
#if FOC_SPEED_CONTROL_ENABLE
    // 速度环
    foc_speed_control(foc_ctrl.target_speed);
#endif
#if FOC_POSITION_CONTROL_ENABLE
    // 位置环
    foc_position_control(foc_ctrl.target_position);
#endif
}

/**
 * @brief FOC内环电流环外部调用接口
 */
void foc_current_in_control(void)
{
#if FOC_ENCODER_CALIBRATION_ENABLE && !FOC_ENCODER_CALIBRATION_USE_CONST_TABLE
    foc_encoder_calibration_open_loop_calibrate();
#elif FOC_PARAMETER_IDENTIFICATION_ENABLE
    // 调用电机参数自辨识步骤
    if (foc_motor_param_ident_is_running()) {
        foc_motor_parameter_ident_step(); // 执行辨识步骤
    }
#else

    // foc_open_loop_control(foc_ctrl.target_speed, foc_ctrl.out_q);
    foc_current_control(foc_ctrl.target_d, foc_ctrl.target_q);
#endif
}

/**
 * @brief FOC速度开环控制
 */
static inline void foc_open_loop_control(float target_speed, float target_outq)
{
    static float angle_accum = 0.0f; // 电角度累加器
    // foc_ctrl.out_q = target_outq;

    // 赋值A/B相电流
    alpha_beta.alpha_i = foc_current_data.IA;
    alpha_beta.beta_i = foc_current_data.IB;

    /************************** 1. 开环电角度计算 **************************/
    // 机械转速 → 电角度速度（°/s）
    foc_ctrl.speed = target_speed * MOTOR_POLE_PAIRS;

    // 积分更新电角度：θ = θ + ω_e * 控制周期（控制周期 = 1/CONTROL_LOOP_FREQ）
    float ctrl_period = PWM_PERIOD_S;
    angle_accum += foc_ctrl.speed * ctrl_period;
    angle_accum = angle_normalize_360(angle_accum);
    foc_ctrl.angle = angle_accum;

    step_flux_observer_update(&g_flux_obs, alpha_beta.alpha,
                              alpha_beta.beta, // 电压（上次输出）
                              alpha_beta.alpha_i, alpha_beta.beta_i, // 电流
                              PWM_PERIOD_S);

    /************************** 2. Park变换（αβ→DQ） **************************/
    float sin_theta, cos_theta;
    CORDIC_SinCos_Deg(foc_ctrl.angle, &sin_theta, &cos_theta);

    // 将alpha-beta坐标系电流转换为DQ坐标系电流
    park_transform(&alpha_beta, &foc_ctrl, sin_theta, cos_theta);

    foc_ctrl.out_d =
        foc_id_pid_calculate(foc_ctrl.target_d, foc_ctrl.abc_dq.current_d);
    foc_ctrl.out_q =
        foc_iq_pid_calculate(foc_ctrl.target_q, foc_ctrl.abc_dq.current_q);

    /************************** 3. 反Park变换（DQ→αβ）
     * **************************/
    inv_park_transform_f32(&foc_ctrl, &alpha_beta, sin_theta, cos_theta);

    /************************** 4. SPWM生成 **************************/
    spwm_duty_calc(&spwm, &alpha_beta, foc_voltage_data.vbus);

    // 输出PWM到定时器
    bsp_pwm_set_duty_four_phase(spwm.duty_Ap, spwm.duty_An, spwm.duty_Bp,
                                spwm.duty_Bn);
}

/**
 * @brief FOC电流闭环控制函数
 */
static inline void foc_current_control(float target_d, float target_q)
{
    float we, id, iq;
    // 赋值A/B相电流
    alpha_beta.alpha_i = foc_current_data.IA;
    alpha_beta.beta_i = foc_current_data.IB;

#if FLUX_OBSERVER_ENABLE
    // 观测器更新
    step_flux_observer_update(&g_flux_obs, alpha_beta.alpha,
                              alpha_beta.beta, // 电压（上次输出）
                              alpha_beta.alpha_i, alpha_beta.beta_i, // 电流
                              PWM_PERIOD_S);
    foc_ctrl.angle = g_flux_obs.theta_hat;
    we = g_flux_obs.omega_filt;
    float sin_theta, cos_theta;
    CORDIC_SinCos_Rad(foc_ctrl.angle, &sin_theta, &cos_theta);
#else
    foc_ctrl.angle = encoder_data.electrical_angle;
    we = encoder_data.electrical_speed;
    float sin_theta, cos_theta;
    CORDIC_SinCos_Deg(foc_ctrl.angle, &sin_theta, &cos_theta);
#endif

    // 将alpha-beta坐标系电流转换为DQ坐标系电流
    park_transform(&alpha_beta, &foc_ctrl, sin_theta, cos_theta);

    // 电流环PID计算
    float vd_pi = foc_id_pid_calculate(target_d, foc_ctrl.abc_dq.current_d);
    float vq_pi = foc_iq_pid_calculate(target_q, foc_ctrl.abc_dq.current_q);

    // 交叉/反电动势前馈解耦
    id = foc_ctrl.abc_dq.current_d;
    iq = foc_ctrl.abc_dq.current_q;

    vd_pi += -we * MOTOR_INDUCTANCE_Lq * iq;
    vq_pi += we * (MOTOR_INDUCTANCE_Ld * id + MOTOR_FLUX_LINKAGE);

    foc_ctrl.out_d = vd_pi;
    foc_ctrl.out_q = vq_pi;

    /************************** 1. 反Park变换（DQ→αβ）
     * **************************/
    inv_park_transform_f32(&foc_ctrl, &alpha_beta, sin_theta, cos_theta);

    /************************** 2. SPWM生成 **************************/
    spwm_duty_calc(&spwm, &alpha_beta, foc_voltage_data.vbus);

    // 输出PWM到定时器
    bsp_pwm_set_duty_four_phase(spwm.duty_Ap, spwm.duty_An, spwm.duty_Bp,
                                spwm.duty_Bn);
}

/**
 * @brief FOC速度闭环控制 2Khz 运行
 */
static inline void foc_speed_control(float target_speed)
{
#if FOC_SPEED_ESO_PI_ENABLE
    float iq_fb = speed_eso_pi_calc(&speed_eso_pi, target_speed,
                                    encoder_data.mechanical_speed);
#elif FOC_SPEED_LADRC_ENABLE
    float iq_fb =
        ladrc_calc(&speed_ladrc, target_speed, encoder_data.mechanical_speed);
#else
    float iq_fb =
        foc_speed_pid_calculate(target_speed, encoder_data.mechanical_speed);
#endif
    foc_ctrl.target_q = iq_fb;
}

/**
 * @brief FOC位置闭环（外环位置环） 2Khz 运行
 * @param target_position 目标位置（机械弧度，范围：0~360）
 */
static inline void foc_position_control(float target_position)
{
    /************************** 位置环PI控制 **************************/
#if FOC_SHAPER_ENABLE
#if (FOC_TRAJECTORY_PLANNER_TYPE == FOC_TRAJECTORY_PLANNER_S_CURVE)
    static float last_target = 0.0f;
    static uint8_t first_run = 1U;
    float scurve_position;

    if ((first_run != 0U) || (fabsf(target_position - last_target) > 0.1f)) {
        if (first_run != 0U) {
            foc_ctrl.shaped_t_p = encoder_data.mechanical_position;
#if FOC_EI_SHAPER_ENABLE
            ei_shaper_reset(foc_ctrl.shaped_t_p);
#endif
        }

        s_curve_set_target(&g_pos_planner, foc_ctrl.shaped_t_p,
                           target_position);
        last_target = target_position;
        first_run = 0U;
    }

    if (s_curve_is_done(&g_pos_planner)) {
        scurve_position = target_position;
    }
    else {
        scurve_position = s_curve_update(&g_pos_planner);
    }

#if FOC_EI_SHAPER_ENABLE
    foc_ctrl.shaped_t_p = ei_shaper_process(scurve_position);
#else
    foc_ctrl.shaped_t_p = scurve_position;
#endif
#elif (FOC_TRAJECTORY_PLANNER_TYPE == FOC_TRAJECTORY_PLANNER_TRAPEZOID)
    static uint8_t first_run = 1U;

    if (first_run != 0U) {
        trapezoid_planner_reset(&g_pos_planner,
                                encoder_data.mechanical_position);
        first_run = 0U;
    }

    foc_ctrl.shaped_t_p = trapezoid_planner_update(
        &g_pos_planner, target_position, encoder_data.mechanical_position,
        FOC_TRAPEZOID_COMMAND_MAX_VELOCITY, FOC_TRAPEZOID_ACCELERATION);
#else
    foc_ctrl.shaped_t_p = target_position;
#endif
#else
    foc_ctrl.shaped_t_p = target_position;
#endif

#if FOC_POSITION_LADRC_ENABLE
    float iq_fb = ladrc_calc_ex(
        &position_ladrc, foc_ctrl.shaped_t_p, encoder_data.mechanical_position,
        POSITION_LADRC_BW, POSITION_LOOP_DT, POSITION_LADRC_B0,
        POSITION_LADRC_OUT_MIN, POSITION_LADRC_OUT_MAX);
#else
    float iq_fb = foc_position_pid_calculate(foc_ctrl.shaped_t_p,
                                             encoder_data.mechanical_position);
#endif

    // 计算最终目标Q轴电流
    foc_ctrl.target_q = iq_fb;
}
