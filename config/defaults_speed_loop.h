/**
 * @file defaults_speed_loop.h
 * @date 2026-10-28
 * @brief 速度/位置环默认宏（仅 M1_SPEED_LOOP_ENABLE=1 时由 motor_params 纳入）。
 * @note 本头在 profile 之前纳入；覆盖靠 profile 的 #undef+#define。
 */
#ifndef CONFIG_DEFAULTS_SPEED_LOOP_H
#define CONFIG_DEFAULTS_SPEED_LOOP_H

#define M1_SPEED_DECIM                  10u
#define M1_SPEED_TS_S                   (M1_CTRL_TS_S * (float)M1_SPEED_DECIM)
#define M1_SPEED_PI_KP                  0.015f
#if (M1_BRINGUP_MODE == M1_BRINGUP_MODE_SPEED_IDENT)
#define M1_SPEED_PI_KI                  0.002f
#else
#define M1_SPEED_PI_KI                  0.002f
#endif
/** 2-DOF 速度 PI：Kp 作用。(β·ω_ref 。ω_fb)。.0=标准 PI。.3~0.5 抑阶跃超。*/
#if (M1_BRINGUP_MODE == M1_BRINGUP_MODE_SPEED_IDENT)
#define M1_SPEED_PI_BETA                0.4f
#else
#define M1_SPEED_PI_BETA                1.0f
#endif
#if (M1_BRINGUP_MODE == M1_BRINGUP_MODE_SPEED_IDENT)
#define M1_SPEED_OMEGA_RAMP_ENABLE      1   /* 抑指令硬切；速率见 TODO(factory) */
#else
#define M1_SPEED_OMEGA_RAMP_ENABLE      0
#endif
/* TODO(factory)：rpm/s 非物理默认值；出厂按 J、Kt、I_avail 写入。勿随 VBUS 改。 */
#define M1_SPEED_OMEGA_RAMP_RPM_S       500.0f
#define M1_SPEED_IQ_SLEW_ENABLE         0
/* TODO(factory)：Iq slew 出厂收；3 A/s 仅为未开 slew 时的占位默认。 */
#define M1_SPEED_IQ_SLEW_A_PER_S        3.0f
/*
 * 降速制动下限（VESC CURRENT_BRAKE 简化）：|ω_ref|+margin < |ω_fb| 时
 * 至少施加 ±I_BRAKE（与转速反向）。默认关；profile 按台架打开。
 * TODO(factory)：I_BRAKE / margin 出厂整定，勿随 VBUS 臆改。
 */
#define M1_VOFA_SPEED_CH8_11            1
/** 1=SPEED 布局 ch10 发 isr_delta（DWT cycle）；0=发 iq_ref。 */
#define M1_VOFA_SPEED_ISR_CH            0

/** 1=位置 P 外环（θ_ref 。ω_ref 。速度 PI）；上电相对零位，不 NVM */
#define M1_POS_LOOP_ENABLE              1
#if M1_POS_LOOP_ENABLE
/** ω_ref = Kp * (θ_ref 。θ_mech) [rpm/rad]，多。unwrap 全误。*/
#define M1_POS_KP_RPM_PER_RAD           80.0f
#define M1_POS_OMEGA_MAX_RPM            400.0f
/** 1=θ_err 滞环停驱区：区内 ω_ref=0，可选冻速度 PI；抑制目标附近抖。*/
#define M1_POS_ERR_HYST_ENABLE          1
#if M1_POS_ERR_HYST_ENABLE
#define M1_POS_ERR_HYST_ENTER_RAD       0.0087266f  /* 0.5° 进入停驱。*/
#define M1_POS_ERR_HYST_EXIT_RAD        0.0174533f  /* 1.0° 退出（> ENTER。*/
#define M1_POS_ERR_HYST_FREEZE_SPEED_PI 1
#endif /* M1_POS_ERR_HYST_ENABLE */
/** 位置 P 分频：外。2kHz / DECIM 。P 更新率。=500Hz（相对速度 Fn。0Hz 足够）；仍抖可试 10=200Hz */
#define M1_POS_DECIM                      4u
#if M1_POS_DECIM < 1u
#error "M1_POS_DECIM must be >= 1"
#endif
/** 1=上电 POSITION 并保。θ_ref=θ_mech(0)。=沿用 SPEED / ident 启动 */
/** 1=位置 26 。+ MIT 同轨迹联测（NORMAL+SPEED_OFF 默认开。*/
#if (M1_BRINGUP_MODE == M1_BRINGUP_MODE_NORMAL) && \
    (M1_DEADBAND_BRINGUP_PHASE == M1_DB_BRINGUP_SPEED_OFF)
#define M1_POS_MIT_COMBO_ENABLE         1
#else
#define M1_POS_MIT_COMBO_ENABLE         0
#endif

/** 1=combo 含位。26 档；0=。MIT 同轨迹验收（暂停位置环） */
#define M1_POS_MIT_COMBO_POS_ENABLE         0

/* M1_VOFA_MIT_CH8_11：无 .c 引用，已删默认；profile 若仍 #define 不影响主线。 */

/** 1=上电 POSITION 阶跃序列（见 M1_POS_STEP_*）；与 M1_SPEED_REVERSAL_TEST 互斥 */
#if M1_POS_MIT_COMBO_ENABLE
#define M1_POS_STEP_TEST_ENABLE         1
#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_NORMAL) && \
    (M1_DEADBAND_BRINGUP_PHASE == M1_DB_BRINGUP_SPEED_OFF)
#define M1_POS_STEP_TEST_ENABLE         1
#else
#define M1_POS_STEP_TEST_ENABLE         0
#endif
#if M1_POS_STEP_TEST_ENABLE
/** 阶跃联调：Kp 30。5 。P+滞环抖；ω_max 暂保。250 rpm */
#define M1_POS_STEP_KP_RPM_PER_RAD      15.0f
#define M1_POS_STEP_OMEGA_MAX_RPM       250.0f
#undef M1_POS_KP_RPM_PER_RAD
#define M1_POS_KP_RPM_PER_RAD           M1_POS_STEP_KP_RPM_PER_RAD
#undef M1_POS_OMEGA_MAX_RPM
#define M1_POS_OMEGA_MAX_RPM            M1_POS_STEP_OMEGA_MAX_RPM
#endif /* M1_POS_STEP_TEST_ENABLE */

#if M1_POS_MIT_COMBO_ENABLE
/** τ = Kp·Δθ + Kd·Δω + τ_ff；ω_des=τ_ff=0。Kt 见 plant 表（原 M1_MIT_KT_NM_A）。 */
#if M1_POS_MIT_COMBO_ENABLE && M1_SPEED_IDENT_ENABLE
#error "M1_POS_MIT_COMBO_ENABLE and M1_SPEED_IDENT_ENABLE are mutually exclusive"
#endif
#endif /* M1_POS_MIT_COMBO_ENABLE */

#if M1_POS_STEP_TEST_ENABLE && (M1_BRINGUP_MODE == M1_BRINGUP_MODE_NORMAL)
#undef M1_SPEED_OMEGA_RAMP_RPM_S
#define M1_SPEED_OMEGA_RAMP_RPM_S       200.0f
#endif

#endif /* M1_POS_LOOP_ENABLE */

/**
 * deadband 效果测试（由 M1_DEADBAND_BRINGUP_PHASE 自动选择；勿手改除非单测 LUT。
 */
#define M1_SPEED_DEADBAND_TEST_OFF        0
#define M1_SPEED_DEADBAND_TEST_LUT        1
#if M1_DEADBAND_BRINGUP_PHASE == M1_DB_BRINGUP_SPEED_LUT
#define M1_SPEED_DEADBAND_TEST            M1_SPEED_DEADBAND_TEST_LUT
#else
#define M1_SPEED_DEADBAND_TEST            M1_SPEED_DEADBAND_TEST_OFF
#endif

/** 1=上电自动阶梯 rpm，每档 M1_SPEED_PROFILE_HOLD_S */
#if M1_DEADBAND_BRINGUP_PHASE == M1_DB_BRINGUP_PASS0
#define M1_SPEED_PROFILE_ENABLE           0
#elif M1_DEADBAND_BRINGUP_PHASE == M1_DB_BRINGUP_SPEED_IDENT
#define M1_SPEED_PROFILE_ENABLE           0
#elif M1_DEADBAND_BRINGUP_PHASE == M1_DB_BRINGUP_SPEED_OFF
#define M1_SPEED_PROFILE_ENABLE           0   /* 300 rpm，不跑阶梯 profile */
#else
#define M1_SPEED_PROFILE_ENABLE           1
#endif
/* M1_SPEED_PROFILE_REPEAT：无 .c 引用，已删默认。 */

/** 1=上电 ±RPM 交替（正反转观测）；SPEED_OFF 默认开，每。M1_SPEED_REVERSAL_HOLD_S */
#if M1_POS_STEP_TEST_ENABLE
#define M1_SPEED_REVERSAL_TEST_ENABLE     0
#elif M1_DEADBAND_BRINGUP_PHASE == M1_DB_BRINGUP_SPEED_OFF
#define M1_SPEED_REVERSAL_TEST_ENABLE     0
#else
#define M1_SPEED_REVERSAL_TEST_ENABLE     0
#endif

#endif /* CONFIG_DEFAULTS_SPEED_LOOP_H */
