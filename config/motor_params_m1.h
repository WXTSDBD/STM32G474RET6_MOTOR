/**
 * @file motor_params_m1.h
 * @brief M1 电机硬件常数：电流采样链路的标度（初始化预计算，热路径只乘不除）。
 *
 * ADC2 三相：外部放大 ×10，采样电阻 10 mΩ，12bit 单端 @ VDDA。
 */

#ifndef MOTOR_PARAMS_M1_H
#define MOTOR_PARAMS_M1_H

/** N5065 极对数 */
#define M1_POLE_PAIRS       7u

/** VDDA / ADC 参考（V） */
#define M1_ADC_VREF_V       3.3f

/** 相电流采样电阻（Ω），10 mΩ */
#define M1_ADC_SHUNT_OHM    0.01f

/** 外部电流放大倍数 */
#define M1_ADC_AMP_GAIN     10.0f

/** 安培/LSB：Vref / (4096 × R_shunt × Gain)，热路径仅做 (raw-offset)*scale */
#define M1_ADC_SCALE_A_LSB  (M1_ADC_VREF_V / (4096.0f * M1_ADC_SHUNT_OHM * M1_ADC_AMP_GAIN))

/** 物理 JDR rank0/1/2 per-channel gain；统一 1.0（扇区采样/KCL 修后再标定，勿用 run RMS 凑） */
#ifndef M1_ADC_GAIN_CH0
#define M1_ADC_GAIN_CH0     1.00f
#endif
#ifndef M1_ADC_GAIN_CH1
#define M1_ADC_GAIN_CH1     1.00f
#endif
#ifndef M1_ADC_GAIN_CH2
#define M1_ADC_GAIN_CH2     1.00f
#endif

/* --- 电气参数（LCR @ 1 kHz，AB 线；Rs 多轮 VASI 辨识 ~0.122 Ω） --- */
#define M1_RS_OHM           0.122f
#define M1_LD_H             59e-6f
#define M1_LQ_H             87e-6f

/** JEOC 电流环节拍（s） */
#define M1_CTRL_TS_S        50e-6f

/**
 * Type-II PLL @ 20 kHz（bringup 试用；架构文档默认 2 kHz，后续可迁 TIM6）。
 * M1_PLL_THETA_PARK_ENABLE=0 时 Park 仍用编码器 raw 电角，仅遥测 ω。
 *
 * 带宽整定（ζ=0.707）：Ki=ωn²，Kp=2ζωn，ωn=2π·fn；fn [Hz] 为机械角域自然频率。
 *   fn≈22  匀速最平滑，加减速/低速切换慢（现网 1242 CSV 表现）
 *   fn≈50~80  速度切换 / 手拨加减速（推荐 bringup）
 *   fn≈100+ 更跟手，20 kHz 下 ω 噪声会变大
 */
#ifndef M1_PLL_ENABLE
#define M1_PLL_ENABLE                   1
#endif
#ifndef M1_PLL_FN_HZ
#define M1_PLL_FN_HZ                    80.0f
#endif
#ifndef M1_PLL_ZETA
#define M1_PLL_ZETA                     0.707106781f
#endif
#ifndef M1_PLL_WN_RAD_S
#define M1_PLL_WN_RAD_S                 (6.28318530718f * M1_PLL_FN_HZ)
#endif
#ifndef M1_PLL_KP
#define M1_PLL_KP                       (2.0f * M1_PLL_ZETA * M1_PLL_WN_RAD_S)
#endif
#ifndef M1_PLL_KI
#define M1_PLL_KI                       (M1_PLL_WN_RAD_S * M1_PLL_WN_RAD_S)
#endif
#ifndef M1_PLL_OMEGA_LIMIT_RAD_S
#define M1_PLL_OMEGA_LIMIT_RAD_S        650.0f   /* ~6200 rpm mech */
#endif
#ifndef M1_PLL_INTEGRATOR_LIMIT_RAD_S
#define M1_PLL_INTEGRATOR_LIMIT_RAD_S   M1_PLL_OMEGA_LIMIT_RAD_S
#endif
#ifndef M1_PLL_THETA_PARK_ENABLE
#define M1_PLL_THETA_PARK_ENABLE        0
#endif
/** 1=VOFA ch8–11 输出 PLL（ch8 ω_pll ch9 ω_diff ch10 θ_err ch11 Δω）；0=Id_ref/Iq_ref/duty_dev/open_seq */
#ifndef M1_VOFA_PLL_CH8_11
#define M1_VOFA_PLL_CH8_11              M1_PLL_ENABLE
#endif

/* ==========================================================================
 * 死区 LUT 三步联调 — 只改 M1_DEADBAND_BRINGUP_PHASE → Rebuild
 *
 *   M1_DB_BRINGUP_SPEED_OFF   ① 速度阶梯 profile，deadband OFF（基线）
 *   M1_DB_BRINGUP_PASS0         ② Pass0 30° 建表 → commit → VOFA LUT 突发
 *   M1_DB_BRINGUP_ONE_SHOT      ★ Pass0→OFF 阶梯→LUT 阶梯
 *   M1_DB_BRINGUP_SPEED_IDENT   ★ 上电 deadband OFF → 速度阶跃（无 Bode，带载测）
 *
 * SPEED_IDENT：open_seq 220=HOLD 221..226=STEP 239=DONE
 * ONE_SHOT：open_seq 201=Pass0 / 200=OFF / 210=LUT
 * 分步 0/1/2 才需 parse_lut_vofa.py --emit-baked
 * ========================================================================== */
#define M1_DB_BRINGUP_SPEED_OFF         0
#define M1_DB_BRINGUP_PASS0             1
#define M1_DB_BRINGUP_SPEED_LUT         2
/** 一次上电：Pass0 锁轴建表 → 速度 OFF 阶梯 → 速度 LUT 阶梯（不导出 baked） */
#define M1_DB_BRINGUP_ONE_SHOT          3
/** 上电 deadband OFF → 速度环阶跃（无 Bode，带载测） */
#define M1_DB_BRINGUP_SPEED_IDENT       4
#ifndef M1_DEADBAND_BRINGUP_PHASE
#define M1_DEADBAND_BRINGUP_PHASE       M1_DB_BRINGUP_SPEED_IDENT
#endif
#if M1_DEADBAND_BRINGUP_PHASE == M1_DB_BRINGUP_ONE_SHOT
#define M1_DEADBAND_FLOW_ONE_SHOT         1
#else
#define M1_DEADBAND_FLOW_ONE_SHOT         0
#endif

/* Bringup mode（须在 M1_SPEED_LOOP_ENABLE 之前选定） */
#define M1_BRINGUP_MODE_NORMAL                0
#define M1_BRINGUP_MODE_IDENT_IQ_STEP         1
#define M1_BRINGUP_MODE_ID_CAL_DUAL_FULL      2
#define M1_BRINGUP_MODE_ID_CAL_PASS0_ONLY     3
#define M1_BRINGUP_MODE_MULTI_ANGLE_PASS0     4
#define M1_BRINGUP_MODE_ID_CAL_PASS0_RS       5
#define M1_BRINGUP_MODE_IQ_PROBE_OFF_ONLY     6
#define M1_BRINGUP_MODE_ID_CAL_PASS0_BUILD    7
#define M1_BRINGUP_MODE_SPEED_IDENT           8
#define M1_BRINGUP_MODE_BODE_OFF_ONLY         9
#define M1_BRINGUP_MODE_RS_LD_LQ_ONLY         10
#define M1_BRINGUP_MODE_BODE_ID_OFF_ONLY      11
/** ② Pass0 后：开环 Ud 阶梯验 LUT（NVM LUT ON；无 Pass0/ident） */
#define M1_BRINGUP_MODE_OPEN_UD_LUT_VERIFY    12

/*
 * 速度环收尾轻载试：SPEED_IDENT + ω_ref 斜坡（无 Pass0/VASI/建表）
 * 恢复辨识：改回 M1_BRINGUP_MODE_ID_CAL_PASS0_BUILD + M1_DB_BRINGUP_PASS0
 */
#ifndef M1_BRINGUP_MODE
#define M1_BRINGUP_MODE  M1_BRINGUP_MODE_SPEED_IDENT
#endif
/** 实验1 签收配置：500 rpm/s 双向斜坡，抑减速硬阶跃 */
#define M1_SPEED_OMEGA_RAMP_ENABLE      1
#define M1_SPEED_OMEGA_RAMP_RPM_S       500.0f

/* --- E1 速度环 @ 2 kHz（20 kHz 分频；限幅宏见 M1_I_REF_ABS_MAX 之后）--- */
#ifndef M1_SPEED_LOOP_ENABLE
#if (M1_BRINGUP_MODE == M1_BRINGUP_MODE_BODE_OFF_ONLY) || \
    (M1_BRINGUP_MODE == M1_BRINGUP_MODE_BODE_ID_OFF_ONLY)
#define M1_SPEED_LOOP_ENABLE            0
#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_RS_LD_LQ_ONLY)
#define M1_SPEED_LOOP_ENABLE            0
#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_OPEN_UD_LUT_VERIFY)
#define M1_SPEED_LOOP_ENABLE            0
#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_SPEED_IDENT)
#define M1_SPEED_LOOP_ENABLE            1
#elif (M1_BRINGUP_MODE != M1_BRINGUP_MODE_NORMAL)
/** 所有辨识/标定 bringup（除 SPEED_IDENT）均关外环，避免与 Id 锁轴/VASI 抢 iq_ref */
#define M1_SPEED_LOOP_ENABLE            0
#elif M1_DEADBAND_BRINGUP_PHASE == M1_DB_BRINGUP_PASS0
#define M1_SPEED_LOOP_ENABLE            0
#else
#define M1_SPEED_LOOP_ENABLE            1
#endif
#endif
#if M1_SPEED_LOOP_ENABLE
#ifndef M1_SPEED_DECIM
#define M1_SPEED_DECIM                  10u
#endif
#define M1_SPEED_TS_S                   (M1_CTRL_TS_S * (float)M1_SPEED_DECIM)
#ifndef M1_SPEED_PI_FN_HZ
#define M1_SPEED_PI_FN_HZ               20.0f
#endif
#ifndef M1_SPEED_PI_ZETA
#define M1_SPEED_PI_ZETA                0.707106781f
#endif
#define M1_SPEED_PI_WN_RAD_S            (6.28318530718f * M1_SPEED_PI_FN_HZ)
#ifndef M1_SPEED_PI_KP
#define M1_SPEED_PI_KP                  0.015f
#endif
#ifndef M1_SPEED_PI_KI
#if (M1_BRINGUP_MODE == M1_BRINGUP_MODE_SPEED_IDENT)
#define M1_SPEED_PI_KI                  0.002f
#else
#define M1_SPEED_PI_KI                  0.002f
#endif
#endif
/** 2-DOF 速度 PI：Kp 作用在 (β·ω_ref − ω_fb)；1.0=标准 PI，0.3~0.5 抑阶跃超调 */
#ifndef M1_SPEED_PI_BETA
#if (M1_BRINGUP_MODE == M1_BRINGUP_MODE_SPEED_IDENT)
#define M1_SPEED_PI_BETA                0.4f
#else
#define M1_SPEED_PI_BETA                1.0f
#endif
#endif
#ifndef M1_SPEED_OMEGA_RAMP_ENABLE
#if (M1_BRINGUP_MODE == M1_BRINGUP_MODE_SPEED_IDENT)
#define M1_SPEED_OMEGA_RAMP_ENABLE      1   /* 实验1：抑减速硬阶跃 */
#else
#define M1_SPEED_OMEGA_RAMP_ENABLE      0
#endif
#endif
#ifndef M1_SPEED_OMEGA_RAMP_RPM_S
#define M1_SPEED_OMEGA_RAMP_RPM_S       500.0f
#endif
#ifndef M1_SPEED_IQ_SLEW_ENABLE
#define M1_SPEED_IQ_SLEW_ENABLE         0
#endif
#ifndef M1_SPEED_IQ_SLEW_A_PER_S
#define M1_SPEED_IQ_SLEW_A_PER_S        3.0f
#endif
#ifndef M1_FOC_ROTATION_FF_ENABLE
#define M1_FOC_ROTATION_FF_ENABLE       0
#endif
#ifndef M1_SPEED_REF_RPM_DEFAULT
#define M1_SPEED_REF_RPM_DEFAULT        300.0f
#endif
#ifndef M1_SPEED_LOOP_BOOT
#define M1_SPEED_LOOP_BOOT              1
#endif
#ifndef M1_VOFA_SPEED_CH8_11
#define M1_VOFA_SPEED_CH8_11            1
#endif

/** 1=位置 P 外环（θ_ref → ω_ref → 速度 PI）；上电相对零位，不 NVM */
#ifndef M1_POS_LOOP_ENABLE
#define M1_POS_LOOP_ENABLE              1
#endif
#if M1_POS_LOOP_ENABLE
/** ω_ref = Kp * (θ_ref − θ_mech) [rpm/rad]，多圈 unwrap 全误差 */
#ifndef M1_POS_KP_RPM_PER_RAD
#define M1_POS_KP_RPM_PER_RAD           80.0f
#endif
#ifndef M1_POS_OMEGA_MAX_RPM
#define M1_POS_OMEGA_MAX_RPM            400.0f
#endif
/** 1=θ_err 滞环停驱区：区内 ω_ref=0，可选冻速度 PI；抑制目标附近抖动 */
#ifndef M1_POS_ERR_HYST_ENABLE
#define M1_POS_ERR_HYST_ENABLE          1
#endif
#if M1_POS_ERR_HYST_ENABLE
#ifndef M1_POS_ERR_HYST_ENTER_RAD
#define M1_POS_ERR_HYST_ENTER_RAD       0.0052360f  /* 0.3° 进入 */
#endif
#ifndef M1_POS_ERR_HYST_EXIT_RAD
#define M1_POS_ERR_HYST_EXIT_RAD        0.0087266f  /* 0.5° 退出（> ENTER） */
#endif
#ifndef M1_POS_ERR_HYST_FREEZE_SPEED_PI
#define M1_POS_ERR_HYST_FREEZE_SPEED_PI 1
#endif
#endif /* M1_POS_ERR_HYST_ENABLE */
/** 位置 P 相对 2 kHz 外环再分频；1=2 kHz，4=500 Hz；速度 PI 内环仍 2 kHz */
#ifndef M1_POS_DECIM
#define M1_POS_DECIM                      4u
#endif
#if M1_POS_DECIM < 1u
#error "M1_POS_DECIM must be >= 1"
#endif
#define M1_POS_TS_S                       (M1_SPEED_TS_S * (float)M1_POS_DECIM)
/** 1=上电 POSITION 并保持 θ_ref=θ_mech(0)；0=沿用 SPEED / ident 启动 */
#ifndef M1_POS_LOOP_BOOT
#define M1_POS_LOOP_BOOT                0
#endif
/** 1=上电 POSITION 阶跃序列（见 M1_POS_STEP_*）；与 M1_SPEED_REVERSAL_TEST 互斥 */
#ifndef M1_POS_STEP_TEST_ENABLE
#if (M1_BRINGUP_MODE == M1_BRINGUP_MODE_NORMAL) && \
    (M1_DEADBAND_BRINGUP_PHASE == M1_DB_BRINGUP_SPEED_OFF)
#define M1_POS_STEP_TEST_ENABLE         1
#else
#define M1_POS_STEP_TEST_ENABLE         0
#endif
#endif
#if M1_POS_STEP_TEST_ENABLE
#ifndef M1_POS_STEP_HOLD_S
#define M1_POS_STEP_HOLD_S              1.0f    /* 初始 hold @ θ(0) */
#endif
#ifndef M1_POS_STEP_DWELL_S
#define M1_POS_STEP_DWELL_S             1.5f    /* |Δθ| < 180° */
#endif
#ifndef M1_POS_STEP_DWELL_WIDE_S
#define M1_POS_STEP_DWELL_WIDE_S        2.0f    /* 180° ≤ |Δθ| < 360° */
#endif
#ifndef M1_POS_STEP_DWELL_MULT_S
#define M1_POS_STEP_DWELL_MULT_S        3.0f    /* 360° ≤ |Δθ| < 720° */
#endif
#ifndef M1_POS_STEP_DWELL_LONG_S
#define M1_POS_STEP_DWELL_LONG_S        4.0f    /* |Δθ| ≥ 720° */
#endif
#ifndef M1_POS_STEP_SEQ_BASE
#define M1_POS_STEP_SEQ_BASE            229u    /* uint8：229..254=各档（26 档），255=done */
#endif
/** 首版整定：Kp=60 rpm/rad，ω 限 250 rpm（比默认 80/400 更柔） */
#ifndef M1_POS_STEP_KP_RPM_PER_RAD
#define M1_POS_STEP_KP_RPM_PER_RAD      30.0f
#endif
#ifndef M1_POS_STEP_OMEGA_MAX_RPM
#define M1_POS_STEP_OMEGA_MAX_RPM       250.0f
#endif
#undef M1_POS_KP_RPM_PER_RAD
#define M1_POS_KP_RPM_PER_RAD           M1_POS_STEP_KP_RPM_PER_RAD
#undef M1_POS_OMEGA_MAX_RPM
#define M1_POS_OMEGA_MAX_RPM            M1_POS_STEP_OMEGA_MAX_RPM
#endif /* M1_POS_STEP_TEST_ENABLE */
#endif /* M1_POS_LOOP_ENABLE */

/**
 * deadband 效果测试（由 M1_DEADBAND_BRINGUP_PHASE 自动选择；勿手改除非单测 LUT）
 */
#define M1_SPEED_DEADBAND_TEST_OFF        0
#define M1_SPEED_DEADBAND_TEST_LUT        1
#ifndef M1_SPEED_DEADBAND_TEST
#if M1_DEADBAND_BRINGUP_PHASE == M1_DB_BRINGUP_SPEED_LUT
#define M1_SPEED_DEADBAND_TEST            M1_SPEED_DEADBAND_TEST_LUT
#else
#define M1_SPEED_DEADBAND_TEST            M1_SPEED_DEADBAND_TEST_OFF
#endif
#endif

/** @deprecated 由 M1_SPEED_DEADBAND_TEST 推导，勿手改 */
#if M1_SPEED_DEADBAND_TEST == M1_SPEED_DEADBAND_TEST_OFF
#define M1_SPEED_LOOP_DEADBAND_OFF        1
#else
#define M1_SPEED_LOOP_DEADBAND_OFF        0
#endif

/** 1=上电自动阶梯：100→300→500→700→900 rpm，每档 M1_SPEED_PROFILE_HOLD_S */
#ifndef M1_SPEED_PROFILE_ENABLE
#if M1_DEADBAND_BRINGUP_PHASE == M1_DB_BRINGUP_PASS0
#define M1_SPEED_PROFILE_ENABLE           0
#elif M1_DEADBAND_BRINGUP_PHASE == M1_DB_BRINGUP_SPEED_IDENT
#define M1_SPEED_PROFILE_ENABLE           0
#elif M1_DEADBAND_BRINGUP_PHASE == M1_DB_BRINGUP_SPEED_OFF
#define M1_SPEED_PROFILE_ENABLE           0   /* 恒 300 rpm，不跑阶梯 profile */
#else
#define M1_SPEED_PROFILE_ENABLE           1
#endif
#endif
#if M1_SPEED_PROFILE_ENABLE
#ifndef M1_SPEED_PROFILE_HOLD_S
#define M1_SPEED_PROFILE_HOLD_S           10.0f
#endif
#ifndef M1_SPEED_PROFILE_RPM_START
#define M1_SPEED_PROFILE_RPM_START        100.0f
#endif
#ifndef M1_SPEED_PROFILE_RPM_STEP
#define M1_SPEED_PROFILE_RPM_STEP         200.0f
#endif
#ifndef M1_SPEED_PROFILE_RPM_END
#define M1_SPEED_PROFILE_RPM_END          900.0f
#endif
#ifndef M1_SPEED_PROFILE_REPEAT
#define M1_SPEED_PROFILE_REPEAT           1   /* 1=900 后回到 100 循环 */
#endif
#endif /* M1_SPEED_PROFILE_ENABLE */

/** 1=上电 ±RPM 交替（正反转观测）；SPEED_OFF 默认开，每档 M1_SPEED_REVERSAL_HOLD_S */
#ifndef M1_SPEED_REVERSAL_TEST_ENABLE
#if M1_POS_STEP_TEST_ENABLE
#define M1_SPEED_REVERSAL_TEST_ENABLE     0
#elif M1_DEADBAND_BRINGUP_PHASE == M1_DB_BRINGUP_SPEED_OFF
#define M1_SPEED_REVERSAL_TEST_ENABLE     0
#else
#define M1_SPEED_REVERSAL_TEST_ENABLE     0
#endif
#endif
#if M1_SPEED_REVERSAL_TEST_ENABLE
#ifndef M1_SPEED_REVERSAL_RPM
#define M1_SPEED_REVERSAL_RPM             300.0f
#endif
#ifndef M1_SPEED_REVERSAL_HOLD_S
#define M1_SPEED_REVERSAL_HOLD_S          10.0f
#endif
#ifndef M1_SPEED_REVERSAL_REPEAT
#define M1_SPEED_REVERSAL_REPEAT          1   /* 1=+/- 循环；0=各跑一档后保持末档 */
#endif
#endif /* M1_SPEED_REVERSAL_TEST_ENABLE */
#endif /* M1_SPEED_LOOP_ENABLE */

#ifndef M1_FOC_ROTATION_FF_ENABLE
#define M1_FOC_ROTATION_FF_ENABLE       0
#endif

/** VOFA JustFloat 通道数与分频（TIM1 20kHz 基准，D=2 → 10kHz 帧率） */
#ifndef M1_TELEM_BRINGUP_K
#define M1_TELEM_BRINGUP_K              12u
#endif
#ifndef M1_TELEM_BRINGUP_DECIMATION
#define M1_TELEM_BRINGUP_DECIMATION     2u
#endif

/**
 * 1=bringup 全阶段固定 VOFA×12（Id cal / ident / 开环阶梯同布局，不再按 open_seq 换 ch3–11）。
 * ch0=Ia ch1=Ib ch2=Ic ch3=Id ch4=Iq ch5=θ_el
 * M1_VOFA_IDENT_DUTY_12CH=1（辨识/标定，默认）：ch6=Vd_est ch7=Vq_est ch8=Ta ch9=Tb ch10=Tc ch11=open_seq；
 *   VASI(open_seq=57) 时 ch8=grid ch9=proc_code ch10=L_est_uH（ch6/7 仍为端电压估计）。
 * M1_VOFA_IDENT_DUTY_12CH=0：ch6=Ud_out ch7=Uq_out；
 *   M1_VOFA_PLL_CH8_11=1 → ch8=ω_pll ch9=ω_diff ch10=θ_err ch11=Δω；
 *   M1_VOFA_PLL_CH8_11=0 → ch8=Id_ref ch9=Iq_ref ch10=duty_dev ch11=open_seq。
 */
#ifndef M1_VOFA_UNIFIED_12CH
#define M1_VOFA_UNIFIED_12CH            1
#endif

/** 1=Park/VOFA 用 -θ_enc；0=与 SVPWM 同 +θ（与 Core/Inc/main.h 同名宏兼容） */
#ifndef M1_THETA_NEGATE
#define M1_THETA_NEGATE     0
#endif

/** 母线电压（V），限幅用 Vbus/√3 */
#define M1_VBUS_V           24.0f
#define M1_PI_V_MAX         (M1_VBUS_V * 0.577350269f)
#define M1_PI_V_MIN         (-M1_PI_V_MAX)

/** PI 带宽（Hz）；M1_PI_USE_FIXED_GAIN=0 时用于 L×ωc 整定（阶跃验收参考 1000 Hz） */
#define M1_PI_FC_HZ         1000.0f
#define M1_PI_WC_RADS       (2.0f * 3.14159265359f * M1_PI_FC_HZ)

/** 联调：1=固定 Kp/Ki（试凑）；0=按 fc 整定 */
#ifndef M1_PI_USE_FIXED_GAIN
#define M1_PI_USE_FIXED_GAIN  0
#endif

#if M1_PI_USE_FIXED_GAIN
#define M1_PI_KP_ID         2.0f
#define M1_PI_KP_IQ         2.0f
#define M1_PI_KI            0.0002f
#else
#define M1_PI_KP_ID         (M1_LD_H * M1_PI_WC_RADS)
#define M1_PI_KP_IQ         (M1_LQ_H * M1_PI_WC_RADS)
#define M1_PI_KI_NOMINAL    (M1_RS_OHM * M1_PI_WC_RADS * M1_CTRL_TS_S)
#ifndef M1_PI_KI_SCALE
#define M1_PI_KI_SCALE      1.0f
#endif
#define M1_PI_KI            (M1_PI_KI_NOMINAL * M1_PI_KI_SCALE)
#endif

/**
 * 首次闭环联调（24V/0.8A 电源）：降低 dq 电压限幅，ref 硬钳位。
 * 稳定后设 M1_CLOSURE_BRINGUP=0 恢复满幅 SVPWM 线性区。
 */
#ifndef M1_CLOSURE_BRINGUP
#if M1_SPEED_LOOP_ENABLE
#define M1_CLOSURE_BRINGUP  0   /* 速度环联调：满幅 Iq/Uq */
#else
#define M1_CLOSURE_BRINGUP  1
#endif
#endif

#if M1_CLOSURE_BRINGUP
#define M1_PI_V_LIMIT_V     6.0f
#define M1_I_REF_ABS_MAX    0.5f
#else
#define M1_PI_V_LIMIT_V     M1_PI_V_MAX
#if (M1_BRINGUP_MODE == M1_BRINGUP_MODE_SPEED_IDENT)
#define M1_I_REF_ABS_MAX    11.0f
#else
#define M1_I_REF_ABS_MAX    5.0f
#endif
#endif

#define M1_PI_V_LIMIT_MIN   (-M1_PI_V_LIMIT_V)

/** 积分器限幅（V），与输出限幅同量级，防 windup */
#define M1_PI_INT_LIMIT_V   M1_PI_V_LIMIT_V
#define M1_PI_INT_LIMIT_MIN (-M1_PI_INT_LIMIT_V)

#if M1_SPEED_LOOP_ENABLE
/** 速度环 Iq_ref 限幅 [A]（与辨识分轨；SPEED_IDENT 实验 11 A） */
#ifndef M1_SPEED_IQ_REF_ABS_MAX
#if (M1_BRINGUP_MODE == M1_BRINGUP_MODE_SPEED_IDENT)
#define M1_SPEED_IQ_REF_ABS_MAX         11.0f
#else
#define M1_SPEED_IQ_REF_ABS_MAX         M1_I_REF_ABS_MAX
#endif
#endif
#define M1_SPEED_PI_OUT_MAX             M1_SPEED_IQ_REF_ABS_MAX
#define M1_SPEED_PI_OUT_MIN             (-M1_SPEED_IQ_REF_ABS_MAX)
#define M1_SPEED_PI_INT_MAX             M1_SPEED_IQ_REF_ABS_MAX
#define M1_SPEED_PI_INT_MIN             (-M1_SPEED_IQ_REF_ABS_MAX)
#endif

/** 电流环 Iq 目标（A）；无启动策略时上电即用，手拨启动（Iq 探路/日常：0.5 A） */
#ifndef M1_IQ_REF_A
#define M1_IQ_REF_A         0.5f
#endif

/** 0=无 ALIGN/DRAG，编码器 θ 直接闭环；1=启动状态机 */
#ifndef M1_STARTUP_ENABLE
#define M1_STARTUP_ENABLE   0
#endif

/* ==========================================================================
 * 联调模式（只改这一处，其余 M1_ID_* / M1_IDENT_* 由下面自动推导）
 *
 * 电感 Ld/Lq 三步联调（无端电压表）：
 *   ① M1_BRINGUP_MODE_ID_CAL_PASS0_BUILD  → Pass0 闭环建表 → commit → NVM
 *   ② M1_BRINGUP_MODE_OPEN_UD_LUT_VERIFY  → 开环 Ud 阶梯 + LUT runtime 验表
 *   ③ M1_BRINGUP_MODE_RS_LD_LQ_ONLY       → VASI（INJECT 段 LUT ON；Rs 固定）
 *
 *   M1_BRINGUP_MODE_NORMAL              日常 Iq 环，无自动序列
 *   M1_BRINGUP_MODE_IDENT_IQ_STEP       Pass0 30° 30档(Id→1.5A) → Iq稳态 OFF/LUT（0.3A OFF 20s → LUT ON）
 *   M1_BRINGUP_MODE_ID_CAL_DUAL_FULL    Pass0 三角 30°+150°+270° → commit → Pass1 验表
 *   M1_BRINGUP_MODE_ID_CAL_PASS0_ONLY   Pass0 双角 only（无 Pass1 / 无 Iq 探路）
 *   M1_BRINGUP_MODE_ID_CAL_PASS0_RS      Pass0 30° 建表 → commit → 单轮 OFF Rs+VASI Ld/Lq
 *   M1_BRINGUP_MODE_RS_LD_LQ_ONLY        ③ VASI（Rs 固定；INJECT 段 LUT ON；SETTLE OFF）
 *   M1_BRINGUP_MODE_OPEN_UD_LUT_VERIFY  ② 开环 Ud 阶梯验 LUT（Pass0 后单独烧录）
 *   M1_BRINGUP_MODE_ID_CAL_PASS0_BUILD   ① Pass0 30° 建表 → commit → NVM
 *   M1_BRINGUP_MODE_MULTI_ANGLE_PASS0   Pass0 五角 0/30/60/90/120° only（多角度 raw 录波）
 *   M1_BRINGUP_MODE_IQ_PROBE_OFF_ONLY   跳过建表：上电即 Iq+deadband OFF（PLL 联调）
 *   M1_BRINGUP_MODE_ID_CAL_PASS0_BUILD  Pass0 30° 建表 → commit → VOFA LUT 突发 → DONE
 *   M1_BRINGUP_MODE_SPEED_IDENT         上电 deadband OFF → 速度阶跃 → Bode
 *   M1_BRINGUP_MODE_BODE_OFF_ONLY       无建表/阶跃：HOLD → Iq Bode OFF-lo/hi @6ch 20kHz
 *   M1_BRINGUP_MODE_BODE_ID_OFF_ONLY    同上，Id 轴 sin（Iq=0，θ=30°）ch3=Id ch4=Id_ref
 * （枚举与 M1_BRINGUP_MODE 默认见文件前部，M1_SPEED_LOOP_ENABLE 之前）
 * ========================================================================== */

#if (M1_BRINGUP_MODE == M1_BRINGUP_MODE_NORMAL)
#define M1_ID_LOCK_CAL_SWEEP            0
#define M1_IDENT_ENABLE                 0
#define M1_ID_CAL_MULTI_ANGLE_ENABLE    0
#define M1_ID_CAL_PASS0_ONLY_ENABLE     0
#define M1_ID_CAL_LUT_VERIFY_SWEEP     0
#define M1_ID_CAL_IQ_PROBE_ENABLE       0
#if M1_SPEED_LOOP_ENABLE && (M1_SPEED_DEADBAND_TEST == M1_SPEED_DEADBAND_TEST_OFF)
/** Run A：速度环 deadband 效果测试 — LUT OFF */
#define M1_DEADBAND_LUT_BAKED_ENABLE    0
#define M1_DEADBAND_ENABLE              0
#elif M1_SPEED_LOOP_ENABLE && (M1_SPEED_DEADBAND_TEST == M1_SPEED_DEADBAND_TEST_LUT)
/** Run B：速度环 deadband 效果测试 — baked LUT ON（abc duty，runtime_apply_ud=0） */
#undef M1_DEADBAND_GEO_TWO_CLUSTER_ENABLE
#define M1_DEADBAND_GEO_TWO_CLUSTER_ENABLE  0
#define M1_DEADBAND_NVM_ON_BOOT         0
#define M1_DEADBAND_LUT_BAKED_ENABLE    1
#define M1_DEADBAND_LUT_RUNTIME_SCALE   1.0f
#define M1_DEADBAND_ENABLE              1
#else
/** 产品：两簇 plut + NVM 上电 LUT ON（标定一次后切 NORMAL） */
#ifndef M1_DEADBAND_GEO_TWO_CLUSTER_ENABLE
#define M1_DEADBAND_GEO_TWO_CLUSTER_ENABLE  1
#endif
#ifndef M1_DEADBAND_NVM_ON_BOOT
#define M1_DEADBAND_NVM_ON_BOOT         0
#endif
#define M1_DEADBAND_LUT_BAKED_ENABLE    1
#if M1_DEADBAND_LUT_BAKED_ENABLE
#ifndef M1_DEADBAND_GEO_TWO_CLUSTER_ENABLE
#define M1_DEADBAND_GEO_TWO_CLUSTER_ENABLE  0
#endif
#ifndef M1_DEADBAND_LUT_RUNTIME_SCALE
#define M1_DEADBAND_LUT_RUNTIME_SCALE   1.0f
#endif
#elif !defined(M1_DEADBAND_LUT_RUNTIME_SCALE)
#define M1_DEADBAND_LUT_RUNTIME_SCALE   0.25f
#endif
#ifndef M1_DEADBAND_ENABLE
#define M1_DEADBAND_ENABLE              1
#endif
#endif /* speed loop deadband off */

#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_ID_CAL_PASS0_BUILD)
/** ① Pass0 → ② Ud 阶梯 OFF→ON → ③ VASI Ld/Lq(INJECT LUT) → LUT/ident 突发；同次上电 RAM only */
#define M1_ID_LOCK_CAL_SWEEP            1
#define M1_IDENT_ENABLE                 0
#define M1_ID_CAL_MULTI_ANGLE_ENABLE    0
#define M1_ID_CAL_PASS0_ONLY_ENABLE     0
#define M1_ID_CAL_DUAL_ANGLE_ENABLE     0
#define M1_ID_CAL_FIX_THETA_ENABLE      1
#define M1_ID_CAL_ALIGN_ENABLE          1
#define M1_ID_CAL_COMMIT_LUT            1
#define M1_ID_CAL_LUT_VERIFY_SWEEP      0
#define M1_ID_CAL_IQ_PROBE_ENABLE       0
#define M1_RS_IDENT_ENABLE              0
#undef M1_RS_IDENT_USE_FIXED_NOMINAL
#define M1_RS_IDENT_USE_FIXED_NOMINAL   1
#define M1_LD_LQ_IDENT_ENABLE           1
#undef M1_LD_LQ_IDENT_ABORT_ON_THETA_DRIFT
#define M1_LD_LQ_IDENT_ABORT_ON_THETA_DRIFT  0
#undef M1_LD_LQ_MULTI_ANGLE_ENABLE
#define M1_LD_LQ_MULTI_ANGLE_ENABLE     0
#undef M1_LD_LQ_PRE_DECAY_S
#define M1_LD_LQ_PRE_DECAY_S            1.0f
#undef M1_LD_LQ_IDENT_OPEN_LOOP_ENABLE
#define M1_LD_LQ_IDENT_OPEN_LOOP_ENABLE   1
#undef M1_LD_LQ_IDENT_BODE_BIAS_GRID_ENABLE
#define M1_LD_LQ_IDENT_BODE_BIAS_GRID_ENABLE  0
#undef M1_LD_LQ_IDENT_FINE_GRID_ENABLE
#define M1_LD_LQ_IDENT_FINE_GRID_ENABLE  1
#undef M1_LD_LQ_IDENT_FINE_ONLY
#define M1_LD_LQ_IDENT_FINE_ONLY         0
/** 15 格 + 500 Hz coarse → 1 kHz fine（关 F2 / FINE_ONLY） */
#undef M1_LD_LQ_IDENT_F_COARSE_HZ
#define M1_LD_LQ_IDENT_F_COARSE_HZ        500.0f
#undef M1_LD_LQ_IDENT_F_FINE_HZ
#define M1_LD_LQ_IDENT_F_FINE_HZ        1000.0f
#undef M1_LD_LQ_IDENT_F2_ENABLE
#define M1_LD_LQ_IDENT_F2_ENABLE        0
#undef M1_LD_LQ_IDENT_INJECT_LUT_ENABLE
#define M1_LD_LQ_IDENT_INJECT_LUT_ENABLE  1
#undef M1_LD_LQ_IDENT_L_NOM_H
#define M1_LD_LQ_IDENT_L_NOM_H            85e-6f
#undef M1_LD_LQ_IDENT_SETTLE_S
#define M1_LD_LQ_IDENT_SETTLE_S           1.0f
#undef M1_LD_LQ_IDENT_CYCLES_PER_AMP
#define M1_LD_LQ_IDENT_CYCLES_PER_AMP     15u
#undef M1_VOFA_IDENT_DUMP_ENABLE
#define M1_VOFA_IDENT_DUMP_ENABLE         1
#undef M1_DEADBAND_FLOW_LD_LQ_AFTER_OPEN_UD
#define M1_DEADBAND_FLOW_LD_LQ_AFTER_OPEN_UD  1
#undef M1_ID_CAL_AMP_TABLE_LEN
#define M1_ID_CAL_AMP_TABLE_LEN         30u
#undef M1_ID_CAL_I_MAX_A
#define M1_ID_CAL_I_MAX_A               1.5f
#undef M1_ID_CAL_ID_DWELL_S
#define M1_ID_CAL_ID_DWELL_S            0.5f
#undef M1_ID_CAL_ID_DWELL_LOW_S
#define M1_ID_CAL_ID_DWELL_LOW_S        0.5f
#undef M1_ID_CAL_I_REF_ABS_MAX
#define M1_ID_CAL_I_REF_ABS_MAX         3.5f
#undef M1_TELEM_BRINGUP_K
#define M1_TELEM_BRINGUP_K              12u
#undef M1_VOFA_LUT_DUMP_ENABLE
#if M1_DEADBAND_FLOW_ONE_SHOT
#define M1_VOFA_LUT_DUMP_ENABLE         0
#else
#define M1_VOFA_LUT_DUMP_ENABLE         1
#endif
#undef M1_DEADBAND_I_ZERO_DISABLE
#define M1_DEADBAND_I_ZERO_DISABLE      1
#define M1_DEADBAND_RUNTIME_GEO_ENABLE    0
#undef M1_DEADBAND_GEO_MERGE_VAL30_ONLY
#define M1_DEADBAND_GEO_MERGE_VAL30_ONLY    1
#undef M1_DEADBAND_GEO_TWO_CLUSTER_ENABLE
#define M1_DEADBAND_GEO_TWO_CLUSTER_ENABLE  0
#define M1_DEADBAND_LUT_BAKED_ENABLE    0
#define M1_DEADBAND_LUT_RUNTIME_SCALE   1.0f
#undef M1_DEADBAND_LUT_COMMIT_NORMALIZE
#define M1_DEADBAND_LUT_COMMIT_NORMALIZE  0
#undef M1_DEADBAND_LUT_APPLY_UD
#define M1_DEADBAND_LUT_APPLY_UD        0
#define M1_DEADBAND_ENABLE              1
#undef M1_DEADBAND_NVM_COMMIT_ENABLE
#define M1_DEADBAND_NVM_COMMIT_ENABLE   0
#undef M1_OPEN_UQ_PRE_ID_CAL_ENABLE
#define M1_OPEN_UQ_PRE_ID_CAL_ENABLE    0
#undef M1_OPEN_UD_PRE_ID_CAL_ENABLE
#define M1_OPEN_UD_PRE_ID_CAL_ENABLE    1
#undef M1_OPEN_UD_AFTER_ID_CAL
#define M1_OPEN_UD_AFTER_ID_CAL         1
#undef M1_OPEN_PRE_ID_LADDER_LUT_RUNTIME
#define M1_OPEN_PRE_ID_LADDER_LUT_RUNTIME  1
#undef M1_OPEN_PRE_ID_LADDER_AB_ENABLE
#define M1_OPEN_PRE_ID_LADDER_AB_ENABLE    1
#undef M1_OPEN_PRE_ID_LADDER_DWELL_S
#define M1_OPEN_PRE_ID_LADDER_DWELL_S       0.3f
#undef M1_VOFA_UNIFIED_12CH
#define M1_VOFA_UNIFIED_12CH            1
#undef M1_VOFA_IDENT_DUTY_12CH
#define M1_VOFA_IDENT_DUTY_12CH         1
#undef M1_PLL_ENABLE
#define M1_PLL_ENABLE                   0

#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_IDENT_IQ_STEP)
/** Id Pass0 30° 30档 → commit → Iq=0.3A OFF 20s → LUT ON 稳态（无阶跃/Bode） */
#define M1_IDENT_ID_CAL_BEFORE_STEP     1
#define M1_ID_LOCK_CAL_SWEEP            1
#define M1_IDENT_ENABLE                 0
#define M1_ID_CAL_MULTI_ANGLE_ENABLE    0
#define M1_ID_CAL_PASS0_ONLY_ENABLE     0
#define M1_ID_CAL_LUT_VERIFY_SWEEP      0
#define M1_ID_CAL_IQ_PROBE_ENABLE       1
#define M1_ID_CAL_IQ_PROBE_A            0.3f
#define M1_ID_CAL_IQ_PROBE_OFF_S        20.0f
#define M1_ID_CAL_IQ_PROBE_FIXED_S      0.0f
#define M1_ID_CAL_IQ_PROBE_ID_PI_ENABLE 0
#define M1_ID_CAL_DUAL_ANGLE_ENABLE     0
#define M1_ID_CAL_FIX_THETA_ENABLE      1
#define M1_ID_CAL_ALIGN_ENABLE          0
#define M1_ID_CAL_COMMIT_LUT            1
#undef M1_ID_CAL_AMP_TABLE_LEN
#define M1_ID_CAL_AMP_TABLE_LEN         30u
#undef M1_ID_CAL_I_MAX_A
#define M1_ID_CAL_I_MAX_A               1.5f
/** Pass0 单角 30°：0.05~1.5 A @50 mA/档，0.5 s/档（≈15 s） */
#undef M1_ID_CAL_ID_DWELL_S
#define M1_ID_CAL_ID_DWELL_S            0.5f
#undef M1_ID_CAL_ID_DWELL_LOW_S
#define M1_ID_CAL_ID_DWELL_LOW_S        0.5f
#undef M1_DEADBAND_GEO_SAMPLE_MAX
#define M1_DEADBAND_GEO_SAMPLE_MAX      (M1_ID_CAL_AMP_TABLE_LEN * 3u)
#define M1_IDENT_IQ_STEP_ENABLE         1
#define M1_IDENT_IQ_BODE_ENABLE         1
/** 阶跃：低 4 轮 0 起点 + 高 4 轮 1 A 基线（OFF/LUT 各 2）；每轮 6 相 × 1 s */
#undef M1_IDENT_STEP_BANDS
#define M1_IDENT_STEP_BANDS             2u
#undef M1_IDENT_STEP_ROUNDS_PER_PROFILE
#define M1_IDENT_STEP_ROUNDS_PER_PROFILE 2u
#undef M1_IDENT_STEP_ROUNDS
#define M1_IDENT_STEP_ROUNDS            (M1_IDENT_STEP_BANDS * 2u * M1_IDENT_STEP_ROUNDS_PER_PROFILE)
#undef M1_IDENT_STEP_OFF_ROUNDS
#define M1_IDENT_STEP_OFF_ROUNDS        M1_IDENT_STEP_ROUNDS_PER_PROFILE
#undef M1_IDENT_STEP_FIXED_ROUNDS
#define M1_IDENT_STEP_FIXED_ROUNDS      0u
#undef M1_IDENT_STEP_I0_A
#define M1_IDENT_STEP_I0_A              0.0f   /* 低段回零 */
#undef M1_IDENT_STEP_I1_A
#define M1_IDENT_STEP_I1_A              0.3f   /* 低段：小 */
#undef M1_IDENT_STEP_I2_A
#define M1_IDENT_STEP_I2_A              0.5f   /* 低段：中 */
#undef M1_IDENT_STEP_I3_A
#define M1_IDENT_STEP_I3_A              1.0f   /* 低段：大 */
#undef M1_IDENT_STEP_I_BASE_HI_A
#define M1_IDENT_STEP_I_BASE_HI_A       1.0f   /* 高段基线 */
#undef M1_IDENT_STEP_I5_A
#define M1_IDENT_STEP_I5_A              1.3f   /* 高段：1.0→1.3 */
#undef M1_IDENT_STEP_I6_A
#define M1_IDENT_STEP_I6_A              1.5f   /* 高段：1.0→1.5 */
#undef M1_IDENT_STEP_I7_A
#define M1_IDENT_STEP_I7_A              2.0f   /* 高段：1.0→2.0 */
/** 阶跃各档 dwell 1 s（含回基线/回零），便于 Ts/稳态验收 */
#undef M1_IDENT_STEP_DWELL_S
#define M1_IDENT_STEP_DWELL_S           1.0f
#undef M1_IDENT_STEP_ZERO_DWELL_S
#define M1_IDENT_STEP_ZERO_DWELL_S      1.0f
/** Bode 方案 D：0.25/1.25 A 各 OFF/LUT = 4 轮；57 点 10→1570 Hz，20 cycles/f */
#undef M1_IDENT_BODE_BANDS
#define M1_IDENT_BODE_BANDS             2u
#undef M1_IDENT_BODE_ROUNDS
#define M1_IDENT_BODE_ROUNDS            (M1_IDENT_BODE_BANDS * 2u)
#undef M1_IDENT_BODE_OFF_ROUNDS
#define M1_IDENT_BODE_OFF_ROUNDS        1u
#undef M1_IDENT_BODE_FIXED_ROUNDS
#define M1_IDENT_BODE_FIXED_ROUNDS      0u
#undef M1_IDENT_BODE_I_BIAS_A
#define M1_IDENT_BODE_I_BIAS_A          0.25f
#undef M1_IDENT_BODE_I_AMP_A
#define M1_IDENT_BODE_I_AMP_A           0.05f
#undef M1_IDENT_BODE_I_BIAS_HI_A
#define M1_IDENT_BODE_I_BIAS_HI_A       1.25f  /* 高 I 区 Bode（不用 1.0 A） */
#undef M1_IDENT_BODE_I_AMP_HI_A
#define M1_IDENT_BODE_I_AMP_HI_A        0.10f
#undef M1_IDENT_BODE_CYCLES_PER_FREQ
#define M1_IDENT_BODE_CYCLES_PER_FREQ   20.0f
#undef M1_IDENT_BODE_F0_HZ
#define M1_IDENT_BODE_F0_HZ             10.0f
#undef M1_IDENT_BODE_F1_HZ
#define M1_IDENT_BODE_F1_HZ             1500.0f
#undef M1_IDENT_BODE_F_RATIO
#define M1_IDENT_BODE_F_RATIO           1.15f   /* 10 ~ F_SPLIT 以下 */
#undef M1_IDENT_BODE_F_SPLIT_HZ
#define M1_IDENT_BODE_F_SPLIT_HZ        200.0f
#undef M1_IDENT_BODE_F_RATIO_HI
#define M1_IDENT_BODE_F_RATIO_HI        1.06f   /* F_SPLIT ~ F1，方案 D 密扫 crossover 区 */
/** 阶跃/Bode 与 Pass0 同 30° 锁轴（Iq@30°，非编码器旋转） */
#undef M1_IDENT_FIX_THETA_ENABLE
#define M1_IDENT_FIX_THETA_ENABLE       1
#undef M1_IDENT_THETA_EL_RAD
#define M1_IDENT_THETA_EL_RAD           M1_ID_CAL_THETA_EL_RAD
#define M1_DEADBAND_LUT_BAKED_ENABLE    0
#ifndef M1_DEADBAND_GEO_TWO_CLUSTER_ENABLE
#define M1_DEADBAND_GEO_TWO_CLUSTER_ENABLE  0
#endif
#define M1_DEADBAND_LUT_RUNTIME_SCALE   1.0f
#ifndef M1_DEADBAND_LUT_COMMIT_NORMALIZE
#define M1_DEADBAND_LUT_COMMIT_NORMALIZE  1
#endif
#ifndef M1_DEADBAND_NVM_COMMIT_ENABLE
#define M1_DEADBAND_NVM_COMMIT_ENABLE   0
#endif
#undef M1_DEADBAND_LUT_APPLY_UD
#define M1_DEADBAND_LUT_APPLY_UD        0
#undef M1_OPEN_UQ_PRE_ID_CAL_ENABLE
#define M1_OPEN_UQ_PRE_ID_CAL_ENABLE       0
#undef M1_OPEN_UD_PRE_ID_CAL_ENABLE
#define M1_OPEN_UD_PRE_ID_CAL_ENABLE       0
#define M1_IDENT_POST_BODE_OPEN_UQ_ENABLE  0

#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_ID_CAL_DUAL_FULL)
/** Pass0 三角 30/150/270° capture(138档) → commit → Pass1 abc duty 验表（Pass1 可暂不录波） */
#define M1_ID_LOCK_CAL_SWEEP            1
#define M1_IDENT_ENABLE                 0
#define M1_ID_CAL_MULTI_ANGLE_ENABLE    1
#undef M1_ID_CAL_PASS0_ANGLE_COUNT
#define M1_ID_CAL_PASS0_ANGLE_COUNT     3
#define M1_ID_CAL_PASS0_ONLY_ENABLE     0
#define M1_ID_CAL_DUAL_ANGLE_ENABLE     1
#define M1_ID_CAL_FIX_THETA_ENABLE      1
#define M1_ID_CAL_ALIGN_ENABLE          1
#define M1_ID_CAL_COMMIT_LUT            1
#define M1_ID_CAL_LUT_VERIFY_SWEEP      1
#undef M1_ID_CAL_PASS1_USE_APPLY_DUTY
#define M1_ID_CAL_PASS1_USE_APPLY_DUTY  1
#define M1_ID_CAL_IQ_PROBE_ENABLE       0
#define M1_ID_CAL_IQ_PROBE_OFF_S        5.0f
#define M1_ID_CAL_IQ_PROBE_FIXED_S      0.0f
#define M1_ID_CAL_IQ_PROBE_ID_PI_ENABLE 0
#undef M1_ID_CAL_ID_DWELL_S
#define M1_ID_CAL_ID_DWELL_S            0.5f
#undef M1_ID_CAL_ID_DWELL_LOW_S
#define M1_ID_CAL_ID_DWELL_LOW_S        0.5f
#undef M1_ID_CAL_VERIFY_ID_DWELL_S
#define M1_ID_CAL_VERIFY_ID_DWELL_S     0.5f
/** Pass0-B @0° 专用 dwell；0=与 Pass0-A 相同。058 高 Id 欠流验证：1.0 s */
#undef M1_ID_CAL_PASS0_B_DWELL_S
#define M1_ID_CAL_PASS0_B_DWELL_S       1.0f
#undef M1_OPEN_UQ_PRE_ID_CAL_ENABLE
#define M1_OPEN_UQ_PRE_ID_CAL_ENABLE    0
#undef M1_OPEN_UD_PRE_ID_CAL_ENABLE
#define M1_OPEN_UD_PRE_ID_CAL_ENABLE    0
#undef M1_TELEM_BRINGUP_K
#define M1_TELEM_BRINGUP_K              12u
#undef M1_VOFA_LUT_DUMP_ENABLE
#define M1_VOFA_LUT_DUMP_ENABLE         1
/** 标定联调：关闭 |i|<I_ZERO 硬切，小电流连续查表 */
#undef M1_DEADBAND_I_ZERO_DISABLE
#define M1_DEADBAND_I_ZERO_DISABLE      1
/** 第一档 A：d 表 + 当前 θ 反 Park→abc duty；1603 过补，先关回 plut+scale */
#define M1_DEADBAND_RUNTIME_GEO_ENABLE    0
/** 论文式单表 merge plut；0010 双角 median 小 I 差 → val 仅用 30° */
#undef M1_DEADBAND_GEO_MERGE_VAL30_ONLY
#define M1_DEADBAND_GEO_MERGE_VAL30_ONLY    1
/** Pass0-B 坏 capture（|u'|<阈值）不进 geo 池 */
#undef M1_ID_CAL_GEO_U_MIN_V
#define M1_ID_CAL_GEO_U_MIN_V               0.10f
#undef M1_DEADBAND_GEO_TWO_CLUSTER_ENABLE
#define M1_DEADBAND_GEO_TWO_CLUSTER_ENABLE  0
#define M1_DEADBAND_LUT_RUNTIME_SCALE   1.0f
#undef M1_DEADBAND_LUT_COMMIT_NORMALIZE
#define M1_DEADBAND_LUT_COMMIT_NORMALIZE  0
/** commit 后写 Flash（与 FOC 互斥，后续加状态机再开）；完整标定测试阶段关 */
#ifndef M1_DEADBAND_NVM_COMMIT_ENABLE
#define M1_DEADBAND_NVM_COMMIT_ENABLE   0
#endif
/** 0=归一化在 commit；1=旧路径 runtime AUTO scale */
#ifndef M1_DEADBAND_LUT_RUNTIME_SCALE_AUTO
#define M1_DEADBAND_LUT_RUNTIME_SCALE_AUTO  0
#endif

#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_IQ_PROBE_OFF_ONLY)
/** PLL/联调：无 Pass0；上电即 Id=0 Iq=探路电流 deadband OFF，编码器 θ Park，不切 LUT */
#define M1_ID_LOCK_CAL_SWEEP                1
#define M1_IDENT_ENABLE                     0
#define M1_ID_CAL_IQ_PROBE_ONLY_ENABLE      1
#define M1_ID_CAL_IQ_PROBE_ENABLE           1
#define M1_ID_CAL_IQ_PROBE_LUT_AFTER_OFF    0
#define M1_ID_CAL_IQ_PROBE_A                0.4f
#define M1_ID_CAL_IQ_PROBE_OFF_S            0.0f   /* 0=一直 OFF，手动停录 */
#define M1_ID_CAL_IQ_PROBE_FIXED_S          0.0f
#define M1_ID_CAL_IQ_PROBE_ID_PI_ENABLE     1
#define M1_ID_CAL_COMMIT_LUT                0
#define M1_ID_CAL_FIX_THETA_ENABLE          0
#define M1_ID_CAL_ALIGN_ENABLE              0
#define M1_ID_CAL_DUAL_ANGLE_ENABLE         0
#define M1_ID_CAL_MULTI_ANGLE_ENABLE        0
#define M1_ID_CAL_PASS0_ONLY_ENABLE         0
#define M1_ID_CAL_LUT_VERIFY_SWEEP          0
#define M1_RS_IDENT_ENABLE                  0
#define M1_LD_LQ_IDENT_ENABLE               0
#define M1_DEADBAND_LUT_BAKED_ENABLE        0
#ifndef M1_DEADBAND_ENABLE
#define M1_DEADBAND_ENABLE                  1
#endif

#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_ID_CAL_PASS0_RS)
/** Pass0 30° 30档 → commit → 单轮 OFF Rs+VASI（死区 OFF 辨识；DONE 后切 LUT runtime） */
#define M1_ID_LOCK_CAL_SWEEP            1
#define M1_IDENT_ENABLE                 0
#define M1_ID_CAL_MULTI_ANGLE_ENABLE    0
#define M1_ID_CAL_PASS0_ONLY_ENABLE     0
#define M1_ID_CAL_LUT_VERIFY_SWEEP      0
#define M1_ID_CAL_IQ_PROBE_ENABLE       0
#define M1_RS_IDENT_ENABLE              1
#define M1_LD_LQ_IDENT_ENABLE           1
#undef M1_LD_LQ_IDENT_ABORT_ON_THETA_DRIFT
/** 0=θ 漂移只记 dbg，9 格 VASI 强制跑完（离线再筛）；1=超限 ld_lq_abort */
#define M1_LD_LQ_IDENT_ABORT_ON_THETA_DRIFT  0
#undef M1_LD_LQ_MULTI_ANGLE_ENABLE
#define M1_LD_LQ_MULTI_ANGLE_ENABLE     0
#undef M1_LD_LQ_IDENT_ANGLE_COUNT
#define M1_LD_LQ_IDENT_ANGLE_COUNT      3u
#undef M1_LD_LQ_ALIGN_S
#define M1_LD_LQ_ALIGN_S                1.0f
#undef M1_LD_LQ_PRE_DECAY_S
#define M1_LD_LQ_PRE_DECAY_S            1.0f
#define M1_ID_CAL_DUAL_ANGLE_ENABLE     0
#define M1_ID_CAL_FIX_THETA_ENABLE      1
#define M1_ID_CAL_ALIGN_ENABLE          0
#define M1_ID_CAL_COMMIT_LUT            1
#undef M1_ID_CAL_AMP_TABLE_LEN
#define M1_ID_CAL_AMP_TABLE_LEN         30u
#undef M1_ID_CAL_I_MAX_A
#define M1_ID_CAL_I_MAX_A               1.5f
#undef M1_ID_CAL_ID_DWELL_S
#define M1_ID_CAL_ID_DWELL_S            0.5f
#undef M1_ID_CAL_ID_DWELL_LOW_S
#define M1_ID_CAL_ID_DWELL_LOW_S        0.5f
#undef M1_ID_CAL_I_REF_ABS_MAX
#define M1_ID_CAL_I_REF_ABS_MAX         3.5f
#undef M1_DEADBAND_GEO_SAMPLE_MAX
#define M1_DEADBAND_GEO_SAMPLE_MAX      (M1_ID_CAL_AMP_TABLE_LEN * 3u)
#undef M1_RS_IDENT_REPEAT_N
#define M1_RS_IDENT_REPEAT_N            2u
#undef M1_ID_CAL_PASS1_USE_APPLY_DUTY
#define M1_ID_CAL_PASS1_USE_APPLY_DUTY  0
/** 0=仅 OFF 轮 Rs+VASI；1=commit 后再跑 LUT 轮对照（open_seq +100） */
#undef M1_RS_L_IDENT_DUAL_LUT_ROUND_ENABLE
#define M1_RS_L_IDENT_DUAL_LUT_ROUND_ENABLE  0

#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_RS_LD_LQ_ONLY)
/** Pass0 已完成：ALIGN@30° → HOLD → VASI Ld/Lq → DONE 切 LUT runtime（Rs 固定 M1_RS_OHM） */
#define M1_ID_LOCK_CAL_SWEEP            1
#define M1_IDENT_ENABLE                 0
#define M1_ID_CAL_RS_LD_LQ_ONLY_ENABLE  1
#define M1_ID_CAL_MULTI_ANGLE_ENABLE    0
#define M1_ID_CAL_PASS0_ONLY_ENABLE     0
#define M1_ID_CAL_LUT_VERIFY_SWEEP      0
#define M1_ID_CAL_IQ_PROBE_ENABLE       0
#undef M1_RS_IDENT_USE_FIXED_NOMINAL
#define M1_RS_IDENT_USE_FIXED_NOMINAL   1
#define M1_RS_IDENT_ENABLE              0
#define M1_LD_LQ_IDENT_ENABLE           1
#undef M1_LD_LQ_IDENT_ABORT_ON_THETA_DRIFT
#define M1_LD_LQ_IDENT_ABORT_ON_THETA_DRIFT  0
#undef M1_LD_LQ_MULTI_ANGLE_ENABLE
#define M1_LD_LQ_MULTI_ANGLE_ENABLE     0
#undef M1_LD_LQ_PRE_DECAY_S
#define M1_LD_LQ_PRE_DECAY_S            1.0f
#define M1_ID_CAL_DUAL_ANGLE_ENABLE     0
#define M1_ID_CAL_FIX_THETA_ENABLE      1
#define M1_ID_CAL_ALIGN_ENABLE          1
#define M1_ID_CAL_COMMIT_LUT            0
#undef M1_RS_IDENT_REPEAT_N
#define M1_RS_IDENT_REPEAT_N            2u
#undef M1_RS_L_IDENT_DUAL_LUT_ROUND_ENABLE
#define M1_RS_L_IDENT_DUAL_LUT_ROUND_ENABLE  0
#undef M1_VOFA_IDENT_DUMP_ENABLE
#define M1_VOFA_IDENT_DUMP_ENABLE         1
#undef M1_PLL_ENABLE
#define M1_PLL_ENABLE                   0
#undef M1_VOFA_PLL_CH8_11
#define M1_VOFA_PLL_CH8_11              0
#undef M1_DEADBAND_NVM_ON_BOOT
#define M1_DEADBAND_NVM_ON_BOOT         0
#undef M1_DEADBAND_ENABLE
#define M1_DEADBAND_ENABLE              1
/** 论文 §2.3：SETTLE 用 PI 到偏置 → 注入段关 PI、冻结 Ud/Uq + 对称 HF 方波（合力矩≈0） */
#undef M1_LD_LQ_IDENT_OPEN_LOOP_ENABLE
#define M1_LD_LQ_IDENT_OPEN_LOOP_ENABLE   1
/** 0=论文 9 格；1=方案 A 15 格 0–1 A 加密 */
#undef M1_LD_LQ_IDENT_BODE_BIAS_GRID_ENABLE
#define M1_LD_LQ_IDENT_BODE_BIAS_GRID_ENABLE  0
#undef M1_LD_LQ_IDENT_FINE_GRID_ENABLE
#define M1_LD_LQ_IDENT_FINE_GRID_ENABLE  1
#undef M1_LD_LQ_IDENT_FINE_ONLY
#define M1_LD_LQ_IDENT_FINE_ONLY         0
/** 15 格 + 500 Hz coarse → 1 kHz fine（关 F2 / FINE_ONLY） */
#undef M1_LD_LQ_IDENT_F_COARSE_HZ
#define M1_LD_LQ_IDENT_F_COARSE_HZ        500.0f
#undef M1_LD_LQ_IDENT_F_FINE_HZ
#define M1_LD_LQ_IDENT_F_FINE_HZ        1000.0f
#undef M1_LD_LQ_IDENT_F2_ENABLE
#define M1_LD_LQ_IDENT_F2_ENABLE        0
/** INJECT 开环段 abc duty LUT；SETTLE/PRE_DECAY 仍 OFF（PI 自补偿） */
#undef M1_LD_LQ_IDENT_INJECT_LUT_ENABLE
#define M1_LD_LQ_IDENT_INJECT_LUT_ENABLE  1

#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_OPEN_UD_LUT_VERIFY)
/** ② 开环 Ud 0/0.2/0.5/1/2/4 V 阶梯，deadband LUT runtime（验 Pass0 表） */
#define M1_ID_LOCK_CAL_SWEEP            0
#define M1_IDENT_ENABLE                 0
#define M1_SPEED_LOOP_ENABLE            0
#define M1_OPEN_UD_PRE_ID_CAL_ENABLE    1
#define M1_OPEN_UQ_PRE_ID_CAL_ENABLE    0
#undef M1_OPEN_PRE_ID_LADDER_LUT_RUNTIME
#define M1_OPEN_PRE_ID_LADDER_LUT_RUNTIME  1
#define M1_OPEN_PRE_ID_LADDER_FIX_THETA_ENABLE  1
#undef M1_DEADBAND_NVM_ON_BOOT
#define M1_DEADBAND_NVM_ON_BOOT         0
#define M1_DEADBAND_ENABLE              1
#undef M1_VOFA_UNIFIED_12CH
#define M1_VOFA_UNIFIED_12CH            1
#undef M1_PLL_ENABLE
#define M1_PLL_ENABLE                   0

#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_ID_CAL_PASS0_ONLY)
#define M1_ID_LOCK_CAL_SWEEP            1
#define M1_IDENT_ENABLE                 0
#define M1_ID_CAL_MULTI_ANGLE_ENABLE    0
#define M1_ID_CAL_PASS0_ONLY_ENABLE     1
#define M1_ID_CAL_LUT_VERIFY_SWEEP     0
#define M1_ID_CAL_IQ_PROBE_ENABLE       0

#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_MULTI_ANGLE_PASS0)
#define M1_ID_LOCK_CAL_SWEEP            1
#define M1_IDENT_ENABLE                 0
#define M1_ID_CAL_MULTI_ANGLE_ENABLE    1
#define M1_ID_CAL_PASS0_ONLY_ENABLE     1
#define M1_ID_CAL_LUT_VERIFY_SWEEP     0
#define M1_ID_CAL_IQ_PROBE_ENABLE       0

#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_BODE_OFF_ONLY)
/** 堵转 Iq Bode：无 Pass0/阶跃/LUT；HOLD 2s → OFF-lo 0.25A → OFF-hi 1.25A；VOFA×6 @20kHz
 *  频表 57 点 F1=2500 Hz（单轮 ~18 s @T_obs_hi=100 ms，两轮 ~38 s）；勿用 legacy F1=800（仅 38 点/846 Hz） */
#define M1_ID_LOCK_CAL_SWEEP            0
#define M1_IDENT_ID_CAL_BEFORE_STEP     0
#define M1_IDENT_ENABLE                 1
#define M1_IDENT_IQ_STEP_ENABLE         0
#define M1_IDENT_IQ_BODE_ENABLE         1
#undef M1_IDENT_STEP_ROUNDS
#define M1_IDENT_STEP_ROUNDS            0u
#undef M1_IDENT_STEP_OFF_ROUNDS
#define M1_IDENT_STEP_OFF_ROUNDS        0u
#undef M1_IDENT_STEP_FIXED_ROUNDS
#define M1_IDENT_STEP_FIXED_ROUNDS      0u
#define M1_RS_IDENT_ENABLE              0
#define M1_LD_LQ_IDENT_ENABLE           0
#define M1_SPEED_IDENT_ENABLE           0
#undef M1_IDENT_BODE_LUT_ENABLE
#define M1_IDENT_BODE_LUT_ENABLE        0
#undef M1_IDENT_BODE_BANDS
#define M1_IDENT_BODE_BANDS             2u
#undef M1_IDENT_BODE_ROUNDS
#define M1_IDENT_BODE_ROUNDS            2u
#undef M1_IDENT_BODE_OFF_ROUNDS
#define M1_IDENT_BODE_OFF_ROUNDS        2u
#undef M1_IDENT_BODE_FIXED_ROUNDS
#define M1_IDENT_BODE_FIXED_ROUNDS      0u
#undef M1_IDENT_BODE_I_BIAS_A
#define M1_IDENT_BODE_I_BIAS_A          0.25f
#undef M1_IDENT_BODE_I_AMP_A
#define M1_IDENT_BODE_I_AMP_A           0.05f
#undef M1_IDENT_BODE_I_BIAS_HI_A
#define M1_IDENT_BODE_I_BIAS_HI_A       1.25f
#undef M1_IDENT_BODE_I_AMP_HI_A
#define M1_IDENT_BODE_I_AMP_HI_A        0.10f
#undef M1_IDENT_BODE_CYCLES_PER_FREQ
#define M1_IDENT_BODE_CYCLES_PER_FREQ   20.0f
#undef M1_IDENT_BODE_CYCLES_HI
#define M1_IDENT_BODE_CYCLES_HI         50.0f  /* T_OBS_HI 优先；此为 fallback */
#undef M1_IDENT_BODE_F0_HZ
#define M1_IDENT_BODE_F0_HZ             10.0f
#undef M1_IDENT_BODE_F1_HZ
#define M1_IDENT_BODE_F1_HZ             2500.0f
#undef M1_IDENT_BODE_F_RATIO
#define M1_IDENT_BODE_F_RATIO           1.15f
#undef M1_IDENT_BODE_F_SPLIT_HZ
#define M1_IDENT_BODE_F_SPLIT_HZ        500.0f
#undef M1_IDENT_BODE_USE_T_OBS_HI
#define M1_IDENT_BODE_USE_T_OBS_HI      1
#undef M1_IDENT_BODE_T_OBS_HI_S
#define M1_IDENT_BODE_T_OBS_HI_S        0.1f
#undef M1_IDENT_BODE_F_RATIO_HI
#define M1_IDENT_BODE_F_RATIO_HI        1.06f
#undef M1_IDENT_BODE_AXIS_ID
#define M1_IDENT_BODE_AXIS_ID           0
#undef M1_IDENT_FIX_THETA_ENABLE
#define M1_IDENT_FIX_THETA_ENABLE       1
#undef M1_IDENT_THETA_EL_RAD
#define M1_IDENT_THETA_EL_RAD           M1_ID_CAL_THETA_EL_RAD
#undef M1_IDENT_HOLD_S
#define M1_IDENT_HOLD_S                 2.0f
#undef M1_IDENT_OVERRIDE_LIMITS
#define M1_IDENT_OVERRIDE_LIMITS        1
#define M1_DEADBAND_LUT_BAKED_ENABLE    0
#define M1_DEADBAND_ENABLE              1
#undef M1_VOFA_UNIFIED_12CH
#define M1_VOFA_UNIFIED_12CH            0
#undef M1_TELEM_BRINGUP_K
#define M1_TELEM_BRINGUP_K              6u
#undef M1_TELEM_BRINGUP_DECIMATION
#define M1_TELEM_BRINGUP_DECIMATION     1u
#undef M1_VOFA_LUT_DUMP_ENABLE
#define M1_VOFA_LUT_DUMP_ENABLE         0
#undef M1_CLOSURE_BRINGUP
#define M1_CLOSURE_BRINGUP              0
#define M1_IDENT_POST_BODE_OPEN_UQ_ENABLE  0

#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_BODE_ID_OFF_ONLY)
/** 堵转 Id Bode：Iq=0；HOLD 2s → OFF-lo 0.25A → OFF-hi 1.25A；VOFA×6 ch3=Id ch4=Id_ref ch5=f_hz */
#define M1_ID_LOCK_CAL_SWEEP            0
#define M1_IDENT_ID_CAL_BEFORE_STEP     0
#define M1_IDENT_ENABLE                 1
#define M1_IDENT_IQ_STEP_ENABLE         0
#define M1_IDENT_IQ_BODE_ENABLE         1
#undef M1_IDENT_STEP_ROUNDS
#define M1_IDENT_STEP_ROUNDS            0u
#undef M1_IDENT_STEP_OFF_ROUNDS
#define M1_IDENT_STEP_OFF_ROUNDS        0u
#undef M1_IDENT_STEP_FIXED_ROUNDS
#define M1_IDENT_STEP_FIXED_ROUNDS      0u
#define M1_RS_IDENT_ENABLE              0
#define M1_LD_LQ_IDENT_ENABLE           0
#define M1_SPEED_IDENT_ENABLE           0
#undef M1_IDENT_BODE_LUT_ENABLE
#define M1_IDENT_BODE_LUT_ENABLE        0
#undef M1_IDENT_BODE_BANDS
#define M1_IDENT_BODE_BANDS             2u
#undef M1_IDENT_BODE_ROUNDS
#define M1_IDENT_BODE_ROUNDS            2u
#undef M1_IDENT_BODE_OFF_ROUNDS
#define M1_IDENT_BODE_OFF_ROUNDS        2u
#undef M1_IDENT_BODE_FIXED_ROUNDS
#define M1_IDENT_BODE_FIXED_ROUNDS      0u
#undef M1_IDENT_BODE_I_BIAS_A
#define M1_IDENT_BODE_I_BIAS_A          0.25f
#undef M1_IDENT_BODE_I_AMP_A
#define M1_IDENT_BODE_I_AMP_A           0.05f
#undef M1_IDENT_BODE_I_BIAS_HI_A
#define M1_IDENT_BODE_I_BIAS_HI_A       1.25f
#undef M1_IDENT_BODE_I_AMP_HI_A
#define M1_IDENT_BODE_I_AMP_HI_A        0.10f
#undef M1_IDENT_BODE_CYCLES_PER_FREQ
#define M1_IDENT_BODE_CYCLES_PER_FREQ   20.0f
#undef M1_IDENT_BODE_CYCLES_HI
#define M1_IDENT_BODE_CYCLES_HI         50.0f
#undef M1_IDENT_BODE_F0_HZ
#define M1_IDENT_BODE_F0_HZ             10.0f
#undef M1_IDENT_BODE_F1_HZ
#define M1_IDENT_BODE_F1_HZ             2500.0f
#undef M1_IDENT_BODE_F_RATIO
#define M1_IDENT_BODE_F_RATIO           1.15f
#undef M1_IDENT_BODE_F_SPLIT_HZ
#define M1_IDENT_BODE_F_SPLIT_HZ        500.0f
#undef M1_IDENT_BODE_USE_T_OBS_HI
#define M1_IDENT_BODE_USE_T_OBS_HI      1
#undef M1_IDENT_BODE_T_OBS_HI_S
#define M1_IDENT_BODE_T_OBS_HI_S        0.1f
#undef M1_IDENT_BODE_F_RATIO_HI
#define M1_IDENT_BODE_F_RATIO_HI        1.06f
#undef M1_IDENT_BODE_AXIS_ID
#define M1_IDENT_BODE_AXIS_ID           1
#undef M1_IDENT_FIX_THETA_ENABLE
#define M1_IDENT_FIX_THETA_ENABLE       1
#undef M1_IDENT_THETA_EL_RAD
#define M1_IDENT_THETA_EL_RAD           M1_ID_CAL_THETA_EL_RAD
#undef M1_IDENT_HOLD_S
#define M1_IDENT_HOLD_S                 2.0f
#undef M1_IDENT_OVERRIDE_LIMITS
#define M1_IDENT_OVERRIDE_LIMITS        1
#define M1_DEADBAND_LUT_BAKED_ENABLE    0
#define M1_DEADBAND_ENABLE              1
#undef M1_VOFA_UNIFIED_12CH
#define M1_VOFA_UNIFIED_12CH            0
#undef M1_TELEM_BRINGUP_K
#define M1_TELEM_BRINGUP_K              6u
#undef M1_TELEM_BRINGUP_DECIMATION
#define M1_TELEM_BRINGUP_DECIMATION     1u
#undef M1_VOFA_LUT_DUMP_ENABLE
#define M1_VOFA_LUT_DUMP_ENABLE         0
#undef M1_CLOSURE_BRINGUP
#define M1_CLOSURE_BRINGUP              0
#define M1_IDENT_POST_BODE_OPEN_UQ_ENABLE  0

#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_SPEED_IDENT)
/**
 * 7/5 1746 签收序列（联调报告 §4.3）：
 *   HOLD 0.5s settle + 100→300 rpm（5s）→ 500→300→700→300→200→300 → DONE @~29s
 * open_seq：220=HOLD  221..226=STEP 各相  230=BODE(关)  239=DONE
 * ω_ref 斜坡 500 rpm/s 为 7/8 0840 实验1（抑重载减速 Mp）；轻载看硬阶跃可关 M1_SPEED_OMEGA_RAMP_ENABLE
 */
#define M1_ID_LOCK_CAL_SWEEP            0
#define M1_IDENT_ENABLE                 0
#define M1_ID_CAL_ALIGN_ENABLE          0
#define M1_ID_CAL_COMMIT_LUT            0
#define M1_LD_LQ_IDENT_ENABLE           0
#define M1_RS_IDENT_ENABLE              0
#define M1_VOFA_IDENT_DUMP_ENABLE       0
#define M1_SPEED_IDENT_ENABLE           1
#undef M1_OPEN_UD_PRE_ID_CAL_ENABLE
#define M1_OPEN_UD_PRE_ID_CAL_ENABLE    0
#undef M1_OPEN_UQ_PRE_ID_CAL_ENABLE
#define M1_OPEN_UQ_PRE_ID_CAL_ENABLE    0
#define M1_DEADBAND_ENABLE              0
#define M1_DEADBAND_LUT_BAKED_ENABLE    0
#define M1_DEADBAND_NVM_ON_BOOT         0
#define M1_SPEED_PROFILE_ENABLE         0
#define M1_SPEED_IDENT_STEP_ENABLE      1
#define M1_SPEED_IDENT_BODE_ENABLE      0
#define M1_SPEED_IDENT_HOLD_S           5.0f
#ifndef M1_SPEED_IDENT_RPM_START
#define M1_SPEED_IDENT_RPM_START        100.0f
#endif
#define M1_SPEED_IDENT_STEP_ROUNDS      1u
#define M1_SPEED_IDENT_STEP_RPM0        300.0f   /* 基准；带载时在此档加/load */
#define M1_SPEED_IDENT_STEP_RPM1        500.0f
#define M1_SPEED_IDENT_STEP_RPM2        700.0f
#define M1_SPEED_IDENT_STEP_RPM3        200.0f
#define M1_SPEED_IDENT_STEP_DWELL_S     4.0f     /* 非 300 档 dwell（7/5：3～5 s） */
#define M1_SPEED_IDENT_STEP_ZERO_DWELL_S 3.0f   /* 回 300 rpm dwell（7/5 1746 签收） */
/** HOLD 前 PLL 重锁 + Iq=0 时长 [s]，抑制上电 ch8 毛刺 / iq_ref 打满 */
#ifndef M1_SPEED_IDENT_PLL_SETTLE_S
#define M1_SPEED_IDENT_PLL_SETTLE_S     0.5f
#endif
#undef M1_VOFA_PLL_CH8_11
#define M1_VOFA_PLL_CH8_11              0
#undef M1_VOFA_SPEED_CH8_11
#define M1_VOFA_SPEED_CH8_11            1

#else
#error "Unknown M1_BRINGUP_MODE — use M1_BRINGUP_MODE_* in motor_params_m1.h"
#endif

/** 辨识/标定 bringup：关 PLL，避免 ch8–11 被 ω/θ 占用导致 open_seq 不可读 */
#if (M1_BRINGUP_MODE != M1_BRINGUP_MODE_NORMAL) && \
    (M1_BRINGUP_MODE != M1_BRINGUP_MODE_SPEED_IDENT)
#undef M1_PLL_ENABLE
#define M1_PLL_ENABLE                   0
#undef M1_VOFA_PLL_CH8_11
#define M1_VOFA_PLL_CH8_11              0
#endif

/** 1=辨识 VOFA ch6–7 端电压估计 + ch8–10 abc duty（可离线核对 ψ）；0=旧 Ud/Uq + Id_ref 布局 */
#ifndef M1_VOFA_IDENT_DUTY_12CH
#if (M1_BRINGUP_MODE != M1_BRINGUP_MODE_NORMAL) && \
    (M1_BRINGUP_MODE != M1_BRINGUP_MODE_SPEED_IDENT) && \
    (M1_BRINGUP_MODE != M1_BRINGUP_MODE_BODE_OFF_ONLY) && \
    (M1_BRINGUP_MODE != M1_BRINGUP_MODE_BODE_ID_OFF_ONLY)
#define M1_VOFA_IDENT_DUTY_12CH         1
#else
#define M1_VOFA_IDENT_DUTY_12CH         0
#endif
#endif

#ifndef M1_DEADBAND_LUT_BAKED_ENABLE
#define M1_DEADBAND_LUT_BAKED_ENABLE  0
#endif

/* 当前模式名（调试/VOFA 备注用；勿用 #pragma message，Keil 每 TU 重复告警） */
#if (M1_BRINGUP_MODE == M1_BRINGUP_MODE_MULTI_ANGLE_PASS0)
#define M1_BRINGUP_MODE_NAME  "MULTI_ANGLE_PASS0"
#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_IDENT_IQ_STEP)
#define M1_BRINGUP_MODE_NAME  "IDENT_IQ_STEP"
#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_ID_CAL_DUAL_FULL)
#define M1_BRINGUP_MODE_NAME  "ID_CAL_DUAL_FULL"
#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_ID_CAL_PASS0_RS)
#define M1_BRINGUP_MODE_NAME  "ID_CAL_PASS0_RS"
#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_RS_LD_LQ_ONLY)
#define M1_BRINGUP_MODE_NAME  "RS_LD_LQ_ONLY"
#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_OPEN_UD_LUT_VERIFY)
#define M1_BRINGUP_MODE_NAME  "OPEN_UD_LUT_VERIFY"
#elif M1_DEADBAND_FLOW_ONE_SHOT
#define M1_BRINGUP_MODE_NAME  "ONE_SHOT"
#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_ID_CAL_PASS0_BUILD)
#define M1_BRINGUP_MODE_NAME  "ID_CAL_PASS0_BUILD"
#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_ID_CAL_PASS0_ONLY)
#define M1_BRINGUP_MODE_NAME  "ID_CAL_PASS0_ONLY"
#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_IQ_PROBE_OFF_ONLY)
#define M1_BRINGUP_MODE_NAME  "IQ_PROBE_OFF_ONLY"
#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_BODE_OFF_ONLY)
#define M1_BRINGUP_MODE_NAME  "BODE_OFF_ONLY"
#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_BODE_ID_OFF_ONLY)
#define M1_BRINGUP_MODE_NAME  "BODE_ID_OFF_ONLY"
#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_SPEED_IDENT)
#define M1_BRINGUP_MODE_NAME  "SPEED_IDENT"
#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_NORMAL) && M1_SPEED_LOOP_ENABLE && \
      (M1_DEADBAND_BRINGUP_PHASE == M1_DB_BRINGUP_SPEED_LUT)
#define M1_BRINGUP_MODE_NAME  "NORMAL_SPEED_DB_LUT"
#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_NORMAL) && M1_SPEED_LOOP_ENABLE && \
      (M1_DEADBAND_BRINGUP_PHASE == M1_DB_BRINGUP_SPEED_OFF) && \
      M1_SPEED_REVERSAL_TEST_ENABLE
#define M1_BRINGUP_MODE_NAME  "NORMAL_SPEED_REV_TEST"
#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_NORMAL) && M1_SPEED_LOOP_ENABLE && \
      (M1_DEADBAND_BRINGUP_PHASE == M1_DB_BRINGUP_SPEED_OFF)
#define M1_BRINGUP_MODE_NAME  "NORMAL_SPEED_DB_OFF"
#else
#define M1_BRINGUP_MODE_NAME  "NORMAL"
#endif

/**
 * Id 锁轴标定状态机（1D Id 扫表 → 段 1 测 Ud_pi；段 2+ 在线建 LUT）。
 *
 * =1：强制 CURRENT_LOOP、Iq_ref=0、deadband OFF；自动扫 Id。
 * 与 M1_OPEN_UQ_DEADBAND_AB_SWEEP 互斥。
 * VOFA×12（M1_VOFA_UNIFIED_12CH=1）：ch0=Ia ch1=Ib ch2=Ic ch3=Id ch4=Iq ch5=θ
 *       M1_VOFA_IDENT_DUTY_12CH=1：ch6=Vd_est ch7=Vq_est ch8=Ta ch9=Tb ch10=Tc ch11=open_seq；
 *       M1_VOFA_IDENT_DUTY_12CH=0：ch6=Ud_out ch7=Uq_out；
 *       M1_VOFA_PLL_CH8_11=1 → ch8=ω_pll ch9=ω_diff ch10=θ_err ch11=Δω；
 *       M1_VOFA_PLL_CH8_11=0 → ch8=Id_ref ch9=Iq_ref ch10=duty_dev ch11=open_seq；
 *       VASI(open_seq=57) 时 ch8=grid ch9=proc_code ch10=L_est_uH ch11=open_seq（ch6/7 不变）。
 * M1_LD_LQ_IDENT_INJECT_LUT_ENABLE=1：仅 INJ_LD/LQ 段 LUT runtime；SETTLE/PRE_DECAY OFF。
 * M1_OPEN_PRE_ID_LADDER_LUT_RUNTIME=1：开环 Ud/Uq ladder 用 LUT runtime（AB=0 时整轮 ON）。
 * M1_OPEN_PRE_ID_LADDER_AB_ENABLE=1：Ud/Uq 阶梯先 OFF(70..75) 再 LUT ON(80..85)，77/87=轮末。
 * dbg.open_seq_phase：68=ALIGN(30° Ud)；54=Rs ramp；55=Rs OK；56=Rs FAIL；
 *   57=L ident 中；58=L OK；59=L FAIL；163=Rs/L decay；
 *   M1_RS_L_IDENT_DUAL_LUT_ROUND_ENABLE=1 时 open_seq +100（157/158…）；ch10=0/1 表 OFF/LUT 轮；
 *   9=DONE；
 *   0=init，1..N=标定档，39=Pass0 衰减，40/41..=Pass1 LUT 验收，42=JustFloat LUT 突发，
 *   50=Iq 探路 LUT OFF，51=Iq 探路 LUT ON，9=DONE；
 *   DONE 后 M1_VOFA_IDENT_DUMP_ENABLE=1：I0≈−777777 突发 Rs+9 点 Ld/Lq（proto 1.0）；
 *   120+leg×50+step=多角 Pass0（三角 leg0..2 → 120..269，五角 → 120..369）。
 *
 * 由 M1_BRINGUP_MODE 推导；勿与 M1_IDENT_ENABLE 同开。
 */
#ifndef M1_ID_LOCK_CAL_SWEEP
#define M1_ID_LOCK_CAL_SWEEP  0
#endif

#if M1_ID_LOCK_CAL_SWEEP
#if M1_OPEN_UQ_DEADBAND_AB_SWEEP
#error "M1_ID_LOCK_CAL_SWEEP and M1_OPEN_UQ_DEADBAND_AB_SWEEP are mutually exclusive"
#endif

/** 标定态临时放宽 I_ref / PI 限幅（与 M1_CLOSURE_BRINGUP 日常 0.5 A 解耦） */
#ifndef M1_ID_CAL_OVERRIDE_LIMITS
#define M1_ID_CAL_OVERRIDE_LIMITS  1
#endif

#if M1_ID_CAL_OVERRIDE_LIMITS
#ifndef M1_ID_CAL_I_REF_ABS_MAX
#define M1_ID_CAL_I_REF_ABS_MAX    3.0f
#endif
/** 与 2347 电流环一致（M1_CLOSURE_BRINGUP 6 V），Pass0/Pass1/第三段共用 */
#define M1_ID_CAL_PI_V_LIMIT_V     M1_PI_V_LIMIT_V
#else
#define M1_ID_CAL_I_REF_ABS_MAX    M1_I_REF_ABS_MAX
#define M1_ID_CAL_PI_V_LIMIT_V     M1_PI_V_LIMIT_V
#endif

#define M1_ID_CAL_PI_V_LIMIT_MIN   (-M1_ID_CAL_PI_V_LIMIT_V)
#define M1_ID_CAL_PI_INT_LIMIT_V   M1_ID_CAL_PI_V_LIMIT_V
#define M1_ID_CAL_PI_INT_LIMIT_MIN (-M1_ID_CAL_PI_INT_LIMIT_V)

#define M1_ID_CAL_IQ_REF_A          0.0f
#define M1_ID_CAL_I_MIN_A           0.05f
/** Pass0 标定扫表：138 点 = 低 I 加密 + 中高 I（DUAL_FULL）；IDENT 模式覆写为 30 点/1.5 A */
#ifndef M1_ID_CAL_AMP_TABLE_LEN
#define M1_ID_CAL_AMP_TABLE_LEN     138u
#endif
/** 低 I 加密区上界（A）；与 amp 表 0.40 A 末档、ID_DWELL_LOW_ID_A 对齐 */
#ifndef M1_ID_CAL_LOW_I_DENSE_END_A
#define M1_ID_CAL_LOW_I_DENSE_END_A 0.40f
#endif
#ifndef M1_ID_CAL_LOW_I_STEP_A
#define M1_ID_CAL_LOW_I_STEP_A      0.003125f
#endif
#ifndef M1_ID_CAL_I_MAX_A
#define M1_ID_CAL_I_MAX_A           3.0f
#endif
/**
 * Pass1 LUT 验收：默认与 Pass0 同表（EXT_LEN=0）；>0 时在 Pass0 表后追加续扫档。
 * 各档 dwell 统一 M1_ID_CAL_VERIFY_ID_DWELL_S（联调万用表时可改为 3 s）。
 */
#ifndef M1_ID_CAL_VERIFY_I_MAX_A
#define M1_ID_CAL_VERIFY_I_MAX_A    3.0f
#endif
#ifndef M1_ID_CAL_VERIFY_EXT_LEN
#define M1_ID_CAL_VERIFY_EXT_LEN    0u
#endif
#define M1_ID_CAL_VERIFY_AMP_TABLE_LEN  (M1_ID_CAL_AMP_TABLE_LEN + M1_ID_CAL_VERIFY_EXT_LEN)
#ifndef M1_ID_CAL_ID_DWELL_S
#define M1_ID_CAL_ID_DWELL_S        3.0f
#endif
/** Id_ref ≤ 此值时用 M1_ID_CAL_ID_DWELL_LOW_S（与 M1_ID_CAL_ID_DWELL_S 同），否则用 M1_ID_CAL_ID_DWELL_S */
#ifndef M1_ID_CAL_ID_DWELL_LOW_ID_A
#define M1_ID_CAL_ID_DWELL_LOW_ID_A  0.40f
#endif
#ifndef M1_ID_CAL_ID_DWELL_LOW_S
#define M1_ID_CAL_ID_DWELL_LOW_S     M1_ID_CAL_ID_DWELL_S
#endif
/** Pass1 LUT 验收：各 Id 档 dwell 统一（s），不再用 LOW_S 加长 */
#ifndef M1_ID_CAL_VERIFY_ID_DWELL_S
#define M1_ID_CAL_VERIFY_ID_DWELL_S  M1_ID_CAL_ID_DWELL_S
#endif
/** 上电 Id=0 稳定（s）；240125 已验证 */
#define M1_ID_CAL_INIT_HOLD_S       0.5f
/**
 * Pass0 末档→Pass1：Id_ref=0 衰减 (s)，LUT 仍 OFF，再 commit；仅 LUT_VERIFY 双扫生效。
 */
#ifndef M1_ID_CAL_PASS0_DECAY_S
#define M1_ID_CAL_PASS0_DECAY_S     0.8f
#endif
/** Pass1 LUT 验收扫表 init hold (s)，commit 后略长以便 settle */
#ifndef M1_ID_CAL_VERIFY_INIT_HOLD_S
#define M1_ID_CAL_VERIFY_INIT_HOLD_S  1.0f
#endif

/** capture：|Id-Id_ref| 门限 (A) */
#ifndef M1_ID_CAL_CAPTURE_EPS_A
#define M1_ID_CAL_CAPTURE_EPS_A     0.03f
#endif

/** Ud_residual 超过此值 (V) 置 outlier 标志，默认仍写入表 */
#ifndef M1_ID_CAL_OUTLIER_V
#define M1_ID_CAL_OUTLIER_V         1.0f
#endif

/**
 * commit 前 capture 表处理：sort 始终开启；dedupe 默认关。
 * 2045：dedupe 32→31 后低 Id Pass1 劣于 0827，待 A/B 验证；高 Id 不受影响。
 */
#ifndef M1_ID_CAL_LUT_DEDUP_ENABLE
#define M1_ID_CAL_LUT_DEDUP_ENABLE  0
#endif
#ifndef M1_ID_CAL_LUT_DEDUP_AMP_EPS_A
#define M1_ID_CAL_LUT_DEDUP_AMP_EPS_A  0.005f
#endif

/**
 * 段 2=0：只 RAM capture，不 deadband_set_lut
 * 段 3=1：deadband_cal_commit() 注册 LUT
 */
#ifndef M1_ID_CAL_COMMIT_LUT
#define M1_ID_CAL_COMMIT_LUT        1
#endif

/**
 * 段 3：标定 commit 后同次上电自动再扫 Id（LUT ON，不 capture，0.05～3.0 A）。
 * 须 M1_ID_CAL_COMMIT_LUT=1。
 */
#ifndef M1_ID_CAL_LUT_VERIFY_SWEEP
#define M1_ID_CAL_LUT_VERIFY_SWEEP  1
#endif

/**
 * =1：Pass0 扫完即 DONE（deadband 全程 OFF，不 commit / 不 LUT 突发 / 无 Pass1 / 无 Iq 探路）。
 * 由 M1_BRINGUP_MODE_ID_CAL_PASS0_ONLY / MULTI_ANGLE_PASS0 自动置 1。
 */
#ifndef M1_ID_CAL_PASS0_ONLY_ENABLE
#define M1_ID_CAL_PASS0_ONLY_ENABLE  0
#endif

#if M1_ID_CAL_PASS0_ONLY_ENABLE && M1_ID_CAL_LUT_VERIFY_SWEEP
#error "M1_ID_CAL_PASS0_ONLY_ENABLE requires M1_ID_CAL_LUT_VERIFY_SWEEP=0"
#endif
#if M1_ID_CAL_PASS0_ONLY_ENABLE && M1_ID_CAL_IQ_PROBE_ENABLE
#error "M1_ID_CAL_PASS0_ONLY_ENABLE requires M1_ID_CAL_IQ_PROBE_ENABLE=0"
#endif

#if M1_ID_CAL_LUT_VERIFY_SWEEP && !M1_ID_CAL_COMMIT_LUT
#error "M1_ID_CAL_LUT_VERIFY_SWEEP requires M1_ID_CAL_COMMIT_LUT=1"
#endif

/**
 * Pass1 验收路径：=0（默认）d 表 + Ud 注入（锁轴验 capture，不与 abc duty 打架）；
 * =1 Pass1 也走 plut + apply_duty（仅 Step1 对照，易堵转）。
 */
#ifndef M1_ID_CAL_PASS1_USE_APPLY_DUTY
#define M1_ID_CAL_PASS1_USE_APPLY_DUTY  0
#endif

/**
 * Pass0 commit 后 Iq 旋转探路（同次上电）。
 *   Id=0，Iq=M1_ID_CAL_IQ_PROBE_A（默认 0.5 A），编码器 θ Park，M1_STARTUP_ENABLE=0 手拨启动；
 *   默认 OFF/FIXED 均为 0 s → commit 后直接 phase LUT ON（open_seq 51）。
 *   可选 A/B：M1_ID_CAL_IQ_PROBE_OFF_S>0 先 deadband OFF（50），
 *             M1_ID_CAL_IQ_PROBE_FIXED_S>0 再 FIXED（53），再 LUT ON。
 *   PI/电压限幅用 M1_CLOSURE_BRINGUP（6V/0.5A）；VOFA ch3=Iq ch4=Id ch5=θ。
 * 须 M1_ID_CAL_COMMIT_LUT=1；空载联调，LUT ON 段结束后手动断使能/停录。
 */
#ifndef M1_ID_CAL_IQ_PROBE_ENABLE
#define M1_ID_CAL_IQ_PROBE_ENABLE   1   /* Phase 3：Pass1 后 Iq 旋转探路 */
#endif
#ifndef M1_ID_CAL_IQ_PROBE_A
#define M1_ID_CAL_IQ_PROBE_A        M1_IQ_REF_A
#endif
/** Iq 探路第一段：LUT OFF 时长 (s) */
#ifndef M1_ID_CAL_IQ_PROBE_OFF_S
#define M1_ID_CAL_IQ_PROBE_OFF_S    10.0f
#endif
/** Iq 探路第二段：FIXED 符号补偿 ON (s)；0=跳过，OFF 后直接 LUT ON */
#ifndef M1_ID_CAL_IQ_PROBE_FIXED_S
#define M1_ID_CAL_IQ_PROBE_FIXED_S  5.0f
#endif
/**
 * Iq 探路段是否跑 Id PI（=0：仅 Uq/Iq 环，Pass1 大 Id 扫表后避免 Ud 积分锁死转子）。
 */
#ifndef M1_ID_CAL_IQ_PROBE_ID_PI_ENABLE
#define M1_ID_CAL_IQ_PROBE_ID_PI_ENABLE  0
#endif
/** @deprecated 已由 OFF→ON 双段取代；保留宏避免旧配置编译失败 */
#ifndef M1_ID_CAL_IQ_PROBE_DEADBAND_LUT
#define M1_ID_CAL_IQ_PROBE_DEADBAND_LUT  1
#endif

/**
 * Pass0 commit 后 Id 慢 ramp Rs 辨识（论文 2.5 累加 LS；LUT abc duty；只上报）。
 * open_seq：54=ramp 中，55=成功，56=失败。
 */
#ifndef M1_RS_IDENT_ENABLE
#define M1_RS_IDENT_ENABLE              0
#endif
/** 1=跳过 Rs ramp，VASI 直接用 M1_RS_OHM（与 M1_RS_IDENT_ENABLE 互斥） */
#ifndef M1_RS_IDENT_USE_FIXED_NOMINAL
#define M1_RS_IDENT_USE_FIXED_NOMINAL   0
#endif
#if M1_RS_IDENT_USE_FIXED_NOMINAL && M1_RS_IDENT_ENABLE
#error "M1_RS_IDENT_USE_FIXED_NOMINAL and M1_RS_IDENT_ENABLE are mutually exclusive"
#endif
#ifndef M1_RS_IDENT_I_MAX_A
#define M1_RS_IDENT_I_MAX_A             3.0f
#endif
#ifndef M1_RS_IDENT_RAMP_A_PER_S
#define M1_RS_IDENT_RAMP_A_PER_S        0.5f
#endif
#ifndef M1_RS_IDENT_I_MIN_FIT_A
#define M1_RS_IDENT_I_MIN_FIT_A         2.0f
#endif
#ifndef M1_RS_IDENT_EPS_TRACK_A
#define M1_RS_IDENT_EPS_TRACK_A         0.03f
#endif
#ifndef M1_RS_IDENT_REPEAT_N
#define M1_RS_IDENT_REPEAT_N            2u
#endif
#ifndef M1_RS_IDENT_INTER_ROUND_S
#define M1_RS_IDENT_INTER_ROUND_S       1.0f
#endif
#ifndef M1_RS_IDENT_MIN_SAMPLES
#define M1_RS_IDENT_MIN_SAMPLES         200u
#endif

#if M1_RS_IDENT_ENABLE && !M1_ID_CAL_COMMIT_LUT && !M1_ID_CAL_RS_LD_LQ_ONLY_ENABLE
#error "M1_RS_IDENT_ENABLE requires M1_ID_CAL_COMMIT_LUT=1 (or RS_LD_LQ_ONLY)"
#endif
#if M1_RS_IDENT_ENABLE && M1_ID_CAL_IQ_PROBE_ENABLE
#error "M1_RS_IDENT_ENABLE and M1_ID_CAL_IQ_PROBE_ENABLE are mutually exclusive"
#endif
#if M1_RS_IDENT_ENABLE && M1_IDENT_ENABLE
#error "M1_RS_IDENT_ENABLE requires M1_IDENT_ENABLE=0"
#endif

/**
 * Pass0+Rs 后 VASI 9 点 Ld/Lq 曲面（Id/Iq 偏置 0.5~1.5 A 线性区；U_inj 按 L 自适应）。
 * open_seq：57=进行中，58=9 格跑完，59=中途 abort（ABORT_ON_THETA_DRIFT=1 时 θ 漂移）；
 * 163=Rs/L leg 间 decay；
 * MULTI=1 时 160+leg=ALIGN，170+leg×40=VASI（stride 40 防 uint8 溢出）。
 */
#ifndef M1_LD_LQ_IDENT_ENABLE
#define M1_LD_LQ_IDENT_ENABLE           0
#endif
#ifndef M1_LD_LQ_PRE_DECAY_S
#define M1_LD_LQ_PRE_DECAY_S            0.8f
#endif
#ifndef M1_LD_LQ_IDENT_BIAS_RAMP_S
/** leg 入口 G0 SETTLE 内 Id/Iq 线性 ramp（Rs decay 后 0→首格）；格点间阶跃 */
#define M1_LD_LQ_IDENT_BIAS_RAMP_S      0.12f
#endif
#ifndef M1_LD_LQ_OPEN_SEQ_VASI_BASE
#define M1_LD_LQ_OPEN_SEQ_VASI_BASE     170u
#endif
#ifndef M1_LD_LQ_OPEN_SEQ_VASI_STRIDE
#define M1_LD_LQ_OPEN_SEQ_VASI_STRIDE   40u
#endif
#ifndef M1_LD_LQ_OPEN_SEQ_PRE_DECAY
#define M1_LD_LQ_OPEN_SEQ_PRE_DECAY     163u
#endif
#ifndef M1_LD_LQ_IDENT_OPEN_LOOP_ENABLE
/** 1=VASI 注入段冻结 SETTLE 末 Ud/Uq（论文 §2.3 开环 HF 注入）；0=闭环 PI+叠 u_inj（旧） */
#define M1_LD_LQ_IDENT_OPEN_LOOP_ENABLE  0
#endif
#ifndef M1_LD_LQ_IDENT_BODE_BIAS_GRID_ENABLE
/** 1=2 格 Id=0,Iq=0.25/1.25 A（对齐 BODE_OFF）；0=9/15 格曲面 */
#define M1_LD_LQ_IDENT_BODE_BIAS_GRID_ENABLE  0
#endif
#ifndef M1_LD_LQ_IDENT_FINE_GRID_ENABLE
/** 1=方案 A：Id 0.5/0.75/1.0 × Iq 0/0.25/0.5/0.75/1.0（15 格，0–1 A 加密） */
#define M1_LD_LQ_IDENT_FINE_GRID_ENABLE  0
#endif
#if M1_LD_LQ_IDENT_BODE_BIAS_GRID_ENABLE && M1_LD_LQ_IDENT_FINE_GRID_ENABLE
#error "M1_LD_LQ_IDENT_BODE_BIAS_GRID_ENABLE and FINE_GRID_ENABLE are mutually exclusive"
#endif
#if M1_LD_LQ_IDENT_BODE_BIAS_GRID_ENABLE
#undef M1_LD_LQ_ID_BIAS_N
#define M1_LD_LQ_ID_BIAS_N              1u
#undef M1_LD_LQ_IQ_BIAS_N
#define M1_LD_LQ_IQ_BIAS_N              2u
#elif M1_LD_LQ_IDENT_FINE_GRID_ENABLE
#undef M1_LD_LQ_ID_BIAS_N
#define M1_LD_LQ_ID_BIAS_N              3u
#undef M1_LD_LQ_IQ_BIAS_N
#define M1_LD_LQ_IQ_BIAS_N              5u
#endif
#ifndef M1_LD_LQ_IDENT_FINE_ONLY
/** 1=仅 1 kHz fine（跳过 500 Hz coarse 与 2 kHz f2）；telem 仍为 proto 2.0，coarse 列无效 */
#define M1_LD_LQ_IDENT_FINE_ONLY         0
#endif
#if M1_LD_LQ_IDENT_FINE_ONLY
#undef M1_LD_LQ_IDENT_F2_ENABLE
#define M1_LD_LQ_IDENT_F2_ENABLE        0
#endif
#ifndef M1_LD_LQ_ID_BIAS_N
#define M1_LD_LQ_ID_BIAS_N              3u
#endif
#ifndef M1_LD_LQ_IQ_BIAS_N
#define M1_LD_LQ_IQ_BIAS_N              3u
#endif
#ifndef M1_LD_LQ_IDENT_F_COARSE_HZ
#define M1_LD_LQ_IDENT_F_COARSE_HZ      2000.0f
#endif
#ifndef M1_LD_LQ_IDENT_F_FINE_HZ
#define M1_LD_LQ_IDENT_F_FINE_HZ        5000.0f
#endif
/** 1=每轴 coarse→fine→f2 三档；0=仅 500 Hz+1 kHz（proto 2.0） */
#ifndef M1_LD_LQ_IDENT_F2_ENABLE
#define M1_LD_LQ_IDENT_F2_ENABLE        0
#endif
#if M1_LD_LQ_IDENT_F2_ENABLE
#ifndef M1_LD_LQ_IDENT_F_F2_HZ
#define M1_LD_LQ_IDENT_F_F2_HZ          2000.0f
#endif
#endif
/** @deprecated 兼容旧脚本；等同 F_FINE */
#ifndef M1_LD_LQ_IDENT_F_HZ
#define M1_LD_LQ_IDENT_F_HZ             M1_LD_LQ_IDENT_F_FINE_HZ
#endif
#ifndef M1_LD_LQ_IDENT_L_NOM_H
/** VASI U_inj 标定用名义电感 (Ld+Lq)/2 */
#define M1_LD_LQ_IDENT_L_NOM_H          70e-6f
#endif
#ifndef M1_LD_LQ_IDENT_DI_TARGET_A
/** 目标 HF 纹波 ΔI（叠在偏置上，避免过零） */
#define M1_LD_LQ_IDENT_DI_TARGET_A      0.12f
#endif
#ifndef M1_LD_LQ_IDENT_I_RIPPLE_MARGIN_A
/** 偏置轴电流与零的最小距离 |I_bias|−|ΔI| ≥ 此值 */
#define M1_LD_LQ_IDENT_I_RIPPLE_MARGIN_A  0.15f
#endif
#ifndef M1_LD_LQ_IDENT_U_INJ_MIN_V
#define M1_LD_LQ_IDENT_U_INJ_MIN_V      0.12f
#endif
#ifndef M1_LD_LQ_IDENT_U_INJ_MAX_V
#define M1_LD_LQ_IDENT_U_INJ_MAX_V      0.50f
#endif
#ifndef M1_LD_LQ_IDENT_SETTLE_S
#define M1_LD_LQ_IDENT_SETTLE_S         0.5f
#endif
/** @deprecated 变幅改 U_INJ_MIN/MAX + L 自适应；保留 0 以兼容 */
#ifndef M1_LD_LQ_IDENT_V_FRAC_MIN
#define M1_LD_LQ_IDENT_V_FRAC_MIN       0.0f
#endif
#ifndef M1_LD_LQ_IDENT_V_FRAC_MAX
#define M1_LD_LQ_IDENT_V_FRAC_MAX       0.0f
#endif
#ifndef M1_LD_LQ_IDENT_AMP_STEPS
#define M1_LD_LQ_IDENT_AMP_STEPS        7u
#endif
#ifndef M1_LD_LQ_IDENT_CYCLES_PER_AMP
#define M1_LD_LQ_IDENT_CYCLES_PER_AMP   10u
#endif
#ifndef M1_LD_LQ_IDENT_THETA_DRIFT_MECH_DEG
#define M1_LD_LQ_IDENT_THETA_DRIFT_MECH_DEG  8.0f
#endif
/** 1=编码器漂移超限时 ld_lq_abort()；0=只记 dbg，9 点 grid 强制跑完（离线分析用） */
#ifndef M1_LD_LQ_IDENT_ABORT_ON_THETA_DRIFT
#define M1_LD_LQ_IDENT_ABORT_ON_THETA_DRIFT  0
#endif
#ifndef M1_LD_LQ_IDENT_L_MIN_H
#define M1_LD_LQ_IDENT_L_MIN_H          20e-6f
#endif
#ifndef M1_LD_LQ_IDENT_EPS_TRACK_A
#define M1_LD_LQ_IDENT_EPS_TRACK_A      0.10f
#endif
#ifndef M1_LD_LQ_IDENT_ZERO_BIAS_SUM_A
#define M1_LD_LQ_IDENT_ZERO_BIAS_SUM_A  0.15f
#endif
#ifndef M1_LD_LQ_IDENT_SETTLE_MAX_S
#define M1_LD_LQ_IDENT_SETTLE_MAX_S       2.0f
#endif
#ifndef M1_LD_LQ_IDENT_MIN_DI_A
#define M1_LD_LQ_IDENT_MIN_DI_A         0.05f
#endif
/** 1=ψ 积分用 (U−U_bias) 去掉偏置 DC，对齐论文对称 ± 注入 */
#ifndef M1_LD_LQ_IDENT_PSI_USE_U_AC
#define M1_LD_LQ_IDENT_PSI_USE_U_AC     1
#endif
/** 1=VASI INJ_LD/LQ 段 deadband LUT runtime；SETTLE/PRE_DECAY 仍 OFF */
#ifndef M1_LD_LQ_IDENT_INJECT_LUT_ENABLE
#define M1_LD_LQ_IDENT_INJECT_LUT_ENABLE  0
#endif
#if M1_LD_LQ_IDENT_INJECT_LUT_ENABLE && !M1_LD_LQ_IDENT_ENABLE
#error "M1_LD_LQ_IDENT_INJECT_LUT_ENABLE requires M1_LD_LQ_IDENT_ENABLE=1"
#endif
#if M1_LD_LQ_IDENT_INJECT_LUT_ENABLE && !M1_DEADBAND_ENABLE
#error "M1_LD_LQ_IDENT_INJECT_LUT_ENABLE requires M1_DEADBAND_ENABLE=1"
#endif

/** deadband_flow 编译/启动条件（含仅 OPEN_UD 验表） */
#ifndef M1_DEADBAND_FLOW_ENABLE
#define M1_DEADBAND_FLOW_ENABLE         \
    (M1_IDENT_ENABLE || M1_ID_LOCK_CAL_SWEEP || M1_SPEED_IDENT_ENABLE || \
     M1_OPEN_UD_PRE_ID_CAL_ENABLE || M1_OPEN_UQ_PRE_ID_CAL_ENABLE)
#endif
/** 正负半周 |u_ac| 最小比值，低于此丢弃该 cycle */
#ifndef M1_LD_LQ_IDENT_U_SYM_RATIO_MIN
#define M1_LD_LQ_IDENT_U_SYM_RATIO_MIN  0.65f
#endif
/** 半周平均 |u_ac| 低于此视为无效注入 */
#ifndef M1_LD_LQ_IDENT_U_SYM_MIN_DV_V
#define M1_LD_LQ_IDENT_U_SYM_MIN_DV_V  0.015f
#endif
/** fine(1 kHz) 额外 v_inj 上限，避免大幅值非线性 */
#ifndef M1_LD_LQ_IDENT_FINE_V_INJ_MAX_V
#define M1_LD_LQ_IDENT_FINE_V_INJ_MAX_V 0.20f
#endif
#ifndef M1_LD_LQ_IDENT_L_MAX_H
#define M1_LD_LQ_IDENT_L_MAX_H          0.001f
#endif
#ifndef M1_LD_LQ_IDENT_MIN_LD_OK
#define M1_LD_LQ_IDENT_MIN_LD_OK        7u
#endif
#ifndef M1_LD_LQ_IDENT_MIN_LQ_OK
#define M1_LD_LQ_IDENT_MIN_LQ_OK        7u
#endif
/** 1=Rs 后 30/150/270° 各跑一遍 9 点 VASI（ALIGN Ud 换角，同 Pass0 三角） */
#ifndef M1_LD_LQ_MULTI_ANGLE_ENABLE
#define M1_LD_LQ_MULTI_ANGLE_ENABLE   0
#endif
#ifndef M1_LD_LQ_IDENT_ANGLE_COUNT
#define M1_LD_LQ_IDENT_ANGLE_COUNT      3u
#endif
#ifndef M1_LD_LQ_ALIGN_S
#define M1_LD_LQ_ALIGN_S                M1_ID_CAL_ALIGN_S
#endif
/** L 辨识多角度电角（默认同 Pass0 三角 30/150/270°） */
#ifndef M1_LD_LQ_THETA0_EL_RAD
#define M1_LD_LQ_THETA0_EL_RAD          M1_ID_CAL_THETA_EL_RAD
#endif
#ifndef M1_LD_LQ_THETA1_EL_RAD
#define M1_LD_LQ_THETA1_EL_RAD          2.6179938779914940f  /* 150° */
#endif
#ifndef M1_LD_LQ_THETA2_EL_RAD
#define M1_LD_LQ_THETA2_EL_RAD          4.7123889803846900f  /* 270° */
#endif
#if M1_LD_LQ_MULTI_ANGLE_ENABLE && !M1_ID_CAL_FIX_THETA_ENABLE
#error "M1_LD_LQ_MULTI_ANGLE_ENABLE requires M1_ID_CAL_FIX_THETA_ENABLE=1"
#endif
#if M1_LD_LQ_MULTI_ANGLE_ENABLE && (M1_LD_LQ_IDENT_ANGLE_COUNT < 1u)
#error "M1_LD_LQ_IDENT_ANGLE_COUNT must be >= 1"
#endif

#if M1_LD_LQ_IDENT_ENABLE && !M1_RS_IDENT_ENABLE && !M1_RS_IDENT_USE_FIXED_NOMINAL
#error "M1_LD_LQ_IDENT_ENABLE requires M1_RS_IDENT_ENABLE=1 or M1_RS_IDENT_USE_FIXED_NOMINAL=1"
#endif

/**
 * Pass0 commit 后 Rs+VASI：默认单轮 deadband OFF。
 * M1_RS_L_IDENT_DUAL_LUT_ROUND_ENABLE=1 时再跑 LUT 轮对照（open_seq +100）。
 */
#ifndef M1_RS_L_IDENT_DUAL_LUT_ROUND_ENABLE
#define M1_RS_L_IDENT_DUAL_LUT_ROUND_ENABLE  0
#endif
#ifndef M1_RS_L_IDENT_LUT_ROUND_OPEN_SEQ_OFFSET
#define M1_RS_L_IDENT_LUT_ROUND_OPEN_SEQ_OFFSET  100u
#endif
#if M1_RS_L_IDENT_DUAL_LUT_ROUND_ENABLE && (!M1_RS_IDENT_ENABLE || !M1_LD_LQ_IDENT_ENABLE)
#error "M1_RS_L_IDENT_DUAL_LUT_ROUND_ENABLE requires M1_RS_IDENT_ENABLE and M1_LD_LQ_IDENT_ENABLE"
#endif

#ifndef M1_ID_CAL_RS_LD_LQ_ONLY_ENABLE
#define M1_ID_CAL_RS_LD_LQ_ONLY_ENABLE  0
#endif

#if M1_LD_LQ_IDENT_ENABLE && !M1_ID_CAL_COMMIT_LUT && !M1_ID_CAL_RS_LD_LQ_ONLY_ENABLE
#error "M1_LD_LQ_IDENT_ENABLE requires M1_ID_CAL_COMMIT_LUT=1 (or RS_LD_LQ_ONLY)"
#endif
#if M1_LD_LQ_IDENT_ENABLE && M1_ID_CAL_IQ_PROBE_ENABLE
#error "M1_LD_LQ_IDENT_ENABLE and M1_ID_CAL_IQ_PROBE_ENABLE are mutually exclusive"
#endif

#ifndef M1_ID_CAL_IQ_PROBE_ONLY_ENABLE
#define M1_ID_CAL_IQ_PROBE_ONLY_ENABLE  0
#endif
#ifndef M1_ID_CAL_IQ_PROBE_LUT_AFTER_OFF
#define M1_ID_CAL_IQ_PROBE_LUT_AFTER_OFF  1
#endif
#if M1_ID_CAL_IQ_PROBE_ONLY_ENABLE && !M1_ID_CAL_IQ_PROBE_ENABLE
#error "M1_ID_CAL_IQ_PROBE_ONLY_ENABLE requires M1_ID_CAL_IQ_PROBE_ENABLE=1"
#endif
#if M1_ID_CAL_IQ_PROBE_ONLY_ENABLE && M1_ID_CAL_PASS0_ONLY_ENABLE
#error "M1_ID_CAL_IQ_PROBE_ONLY_ENABLE requires M1_ID_CAL_PASS0_ONLY_ENABLE=0"
#endif
#if M1_ID_CAL_IQ_PROBE_ONLY_ENABLE && (M1_RS_IDENT_ENABLE || M1_LD_LQ_IDENT_ENABLE)
#error "M1_ID_CAL_IQ_PROBE_ONLY_ENABLE is incompatible with RS/L ident"
#endif
#if M1_ID_CAL_RS_LD_LQ_ONLY_ENABLE && !M1_ID_LOCK_CAL_SWEEP
#error "M1_ID_CAL_RS_LD_LQ_ONLY_ENABLE requires M1_ID_LOCK_CAL_SWEEP=1"
#endif
#if M1_ID_CAL_RS_LD_LQ_ONLY_ENABLE && \
    (!M1_LD_LQ_IDENT_ENABLE || (!M1_RS_IDENT_ENABLE && !M1_RS_IDENT_USE_FIXED_NOMINAL))
#error "M1_ID_CAL_RS_LD_LQ_ONLY_ENABLE requires M1_LD_LQ_IDENT_ENABLE and (M1_RS_IDENT_ENABLE or M1_RS_IDENT_USE_FIXED_NOMINAL)"
#endif
#if M1_ID_CAL_RS_LD_LQ_ONLY_ENABLE && M1_ID_CAL_PASS0_ONLY_ENABLE
#error "M1_ID_CAL_RS_LD_LQ_ONLY_ENABLE requires M1_ID_CAL_PASS0_ONLY_ENABLE=0"
#endif
#if M1_ID_CAL_RS_LD_LQ_ONLY_ENABLE && M1_ID_CAL_IQ_PROBE_ENABLE
#error "M1_ID_CAL_RS_LD_LQ_ONLY_ENABLE is incompatible with M1_ID_CAL_IQ_PROBE_ENABLE"
#endif
#if M1_ID_CAL_IQ_PROBE_ENABLE && !M1_ID_CAL_COMMIT_LUT && !M1_ID_CAL_IQ_PROBE_ONLY_ENABLE
#error "M1_ID_CAL_IQ_PROBE_ENABLE requires M1_ID_CAL_COMMIT_LUT=1 (or IQ_PROBE_ONLY)"
#endif

/**
 * 标定 Park 角固定 30° 电角（π/6）；Iq=0 时 d↔a 换算最简。
 * =0：沿用编码器 theta_enc_park（旧行为）。
 * 编码器仍采样；仅 Park/InvPark/SVPWM 用固定角。
 */
#ifndef M1_ID_CAL_FIX_THETA_ENABLE
#define M1_ID_CAL_FIX_THETA_ENABLE   1
#endif
#ifndef M1_ID_CAL_THETA_EL_RAD
#define M1_ID_CAL_THETA_EL_RAD       0.5235987755982988f  /* pi/6 */
#endif
/** cos(30°)；Step 2 commit d→phase 表时使用 */
#ifndef M1_ID_CAL_D_TO_PHASE_COS
#define M1_ID_CAL_D_TO_PHASE_COS     0.8660254037844386f
#endif

/**
 * FIX_THETA=1 时上电先开环 Ud 对齐到 M1_ID_CAL_THETA_EL_RAD，再 Id 扫表。
 * 须 M1_ID_CAL_FIX_THETA_ENABLE=1。
 */
#ifndef M1_ID_CAL_ALIGN_ENABLE
#define M1_ID_CAL_ALIGN_ENABLE       1
#endif
#ifndef M1_ID_CAL_ALIGN_UD_V
#define M1_ID_CAL_ALIGN_UD_V         3.0f
#endif
#ifndef M1_ID_CAL_ALIGN_S
#define M1_ID_CAL_ALIGN_S            0.5f
#endif

/**
 * 论文 §4.4 双特殊角标定：Pass0-A @30° + Pass0-B @0°，abc 样本合并建 phase 表。
 * =0：仅 Pass0-A（30°），与 Phase 1 行为相同。
 */
#ifndef M1_ID_CAL_DUAL_ANGLE_ENABLE
#define M1_ID_CAL_DUAL_ANGLE_ENABLE  1
#endif
#ifndef M1_ID_CAL_THETA_PASS0_A_RAD
#define M1_ID_CAL_THETA_PASS0_A_RAD  M1_ID_CAL_THETA_EL_RAD  /* 30° 位置 1 */
#endif
#ifndef M1_ID_CAL_THETA_PASS0_B_RAD
#define M1_ID_CAL_THETA_PASS0_B_RAD  0.0f                    /* 0° 位置 2 */
#endif
#ifndef M1_ID_CAL_ALIGN_B_UD_V
#define M1_ID_CAL_ALIGN_B_UD_V       M1_ID_CAL_ALIGN_UD_V
#endif
#ifndef M1_ID_CAL_ALIGN_B_S
#define M1_ID_CAL_ALIGN_B_S          M1_ID_CAL_ALIGN_S
#endif
/**
 * Pass0-B @0° 各档 dwell (s)。>0 时覆盖 ID_DWELL_*；0=与 Pass0-A 相同。
 * 058 录波 0° 高 Id 欠流 7～10% 时，可试 1.0～2.0 s 再验证 capture。
 */
#ifndef M1_ID_CAL_PASS0_B_DWELL_S
#define M1_ID_CAL_PASS0_B_DWELL_S    0.0f
#endif

/**
 * =1：Pass0 多电角扫 Id（deadband OFF + capture）。
 * COUNT=3：30/150/270° 三角（强相轮换，查三相不平衡）；
 * COUNT=5：0/30/60/90/120° 五角（MULTI_ANGLE_PASS0 模式）。
 * geo 池 = AMP_TABLE_LEN×3×COUNT；s_dlut 锚仍仅 30° 档写入。
 * 录波后：`python tools/multi_angle_geo_analysis.py <csv> --angles 30,150,270`
 */
#ifndef M1_ID_CAL_MULTI_ANGLE_ENABLE
#define M1_ID_CAL_MULTI_ANGLE_ENABLE  0
#endif
#if M1_ID_CAL_MULTI_ANGLE_ENABLE
#ifndef M1_ID_CAL_PASS0_ANGLE_COUNT
#define M1_ID_CAL_PASS0_ANGLE_COUNT   5
#endif
#ifndef M1_ID_CAL_THETA_PASS0_0_RAD
#define M1_ID_CAL_THETA_PASS0_0_RAD   0.0f
#endif
#ifndef M1_ID_CAL_THETA_PASS0_150_RAD
#define M1_ID_CAL_THETA_PASS0_150_RAD 2.6179938779914940f  /* 5pi/6, 150° */
#endif
#ifndef M1_ID_CAL_THETA_PASS0_270_RAD
#define M1_ID_CAL_THETA_PASS0_270_RAD 4.7123889803846900f  /* 3pi/2, 270° */
#endif
#ifndef M1_ID_CAL_THETA_PASS0_60_RAD
#define M1_ID_CAL_THETA_PASS0_60_RAD  1.0471975511965976f  /* pi/3, 60° */
#endif
#ifndef M1_ID_CAL_THETA_PASS0_90_RAD
#define M1_ID_CAL_THETA_PASS0_90_RAD  1.5707963267948966f  /* pi/2, 90° */
#endif
#ifndef M1_ID_CAL_THETA_PASS0_120_RAD
#define M1_ID_CAL_THETA_PASS0_120_RAD 2.0943951023931953f  /* 2pi/3, 120° */
#endif
#undef M1_DEADBAND_GEO_SAMPLE_MAX
#define M1_DEADBAND_GEO_SAMPLE_MAX    \
    (M1_ID_CAL_AMP_TABLE_LEN * 3u * (uint32_t)M1_ID_CAL_PASS0_ANGLE_COUNT)
#if M1_ID_CAL_PASS0_ANGLE_COUNT < 2
#error "M1_ID_CAL_PASS0_ANGLE_COUNT must be >= 2"
#endif
#endif /* M1_ID_CAL_MULTI_ANGLE_ENABLE */

#if M1_ID_CAL_DUAL_ANGLE_ENABLE && !M1_ID_CAL_FIX_THETA_ENABLE
#error "M1_ID_CAL_DUAL_ANGLE_ENABLE requires M1_ID_CAL_FIX_THETA_ENABLE=1"
#endif
#if M1_ID_CAL_DUAL_ANGLE_ENABLE && \
    (!M1_ID_CAL_LUT_VERIFY_SWEEP || !M1_ID_CAL_COMMIT_LUT) && \
    !M1_ID_CAL_PASS0_ONLY_ENABLE && \
    !M1_ID_CAL_IQ_PROBE_ENABLE && !M1_RS_IDENT_ENABLE
#error "M1_ID_CAL_DUAL_ANGLE_ENABLE requires LUT_VERIFY+COMMIT, PASS0_ONLY, IQ_PROBE, or RS_IDENT+COMMIT"
#endif

/** Pass0 末衰减：双角 leg 切换 / commit 前 Id→0 */
#if M1_ID_CAL_PASS0_ONLY_ENABLE || \
    (M1_ID_CAL_COMMIT_LUT && (M1_ID_CAL_LUT_VERIFY_SWEEP || M1_ID_CAL_IQ_PROBE_ENABLE || \
                              M1_RS_IDENT_ENABLE || M1_IDENT_ID_CAL_BEFORE_STEP))
#define M1_ID_CAL_PASS0_DECAY_ENABLE  1
#else
#define M1_ID_CAL_PASS0_DECAY_ENABLE  0
#endif

#if M1_ID_CAL_ALIGN_ENABLE && !M1_ID_CAL_FIX_THETA_ENABLE
#error "M1_ID_CAL_ALIGN_ENABLE requires M1_ID_CAL_FIX_THETA_ENABLE=1"
#endif

/** 双角 Pass0 abc 样本池上限；IDENT 单角覆写为 AMP×3 */
#ifndef M1_DEADBAND_GEO_SAMPLE_MAX
#define M1_DEADBAND_GEO_SAMPLE_MAX        (M1_ID_CAL_AMP_TABLE_LEN * 3u * 2u)
#endif

/**
 * =0：telem_lut_dump 不编译（stub）；日常固件关。
 */
#ifndef M1_VOFA_LUT_DUMP_ENABLE
#define M1_VOFA_LUT_DUMP_ENABLE     1
#endif

/**
 * =0：telem_ident_dump 不编译（stub）；Rs/Ld-Lq 辨识结果不突发 VOFA。
 */
#ifndef M1_VOFA_IDENT_DUMP_ENABLE
#define M1_VOFA_IDENT_DUMP_ENABLE   0
#endif

#else /* !M1_ID_LOCK_CAL_SWEEP */

#ifndef M1_ID_CAL_FIX_THETA_ENABLE
#define M1_ID_CAL_FIX_THETA_ENABLE   0
#endif

#ifndef M1_VOFA_LUT_DUMP_ENABLE
#define M1_VOFA_LUT_DUMP_ENABLE     0
#endif

#ifndef M1_VOFA_IDENT_DUMP_ENABLE
#define M1_VOFA_IDENT_DUMP_ENABLE   0
#endif

#endif /* M1_ID_LOCK_CAL_SWEEP */

/* --- 死区补偿（固定符号法） --- */
/**
 * 电流环 deadband A/B 三档（开环 0623 已扫 400/591）：
 *   OFF（M1_DEADBAND_ENABLE=0）— 不过补，基线见 vofa+202606240039
 *   400 ns — 开环折中，≈0.192 V/相（改 M1_DEADTIME_NS 再编译）
 *   591 ns — 本档过补，≈0.284 V/相（当前默认 ON，闭环复测用）
 */
#ifndef M1_DEADBAND_ENABLE
#define M1_DEADBAND_ENABLE      0
#endif

/** 有效死区时间（ns）；标定模式强制 OFF；电流环 A/B 见 240039/240051 报告 */
#define M1_DEADTIME_NS          591u

/** PWM 周期（s），与 M1_CTRL_TS_S / TIM8 20 kHz 一致 */
#define M1_PWM_PERIOD_S         M1_CTRL_TS_S

/** 每相固定补偿电压：Vbus × t_dead / T_pwm ≈ 0.284 V @ 24 V, 591 ns */
#define M1_DEADBAND_V_COMP_V    (M1_VBUS_V * (float)M1_DEADTIME_NS * 1.0e-9f / M1_PWM_PERIOD_S)

/** 归一化占空比补偿量 = t_dead / T_pwm */
#define M1_DEADBAND_DUTY_COMP   (M1_DEADBAND_V_COMP_V / M1_VBUS_V)

/** 过零区：|i| 低于此值不补偿（仅 M1_DEADBAND_I_ZERO_DISABLE=0 时生效） */
#define M1_DEADBAND_I_ZERO_A    0.05f

/**
 * A/B 联调：=1 关闭过零死区，i≠0 即按符号全幅补偿。
 * 低电流请优先用 M1_DEADBAND_LUT_APPLY_MIN_A。
 */
#ifndef M1_DEADBAND_I_ZERO_DISABLE
#define M1_DEADBAND_I_ZERO_DISABLE   0
#endif

/** LUT 注入：1=d 轴 Ud（0830 过渡）；0=abc 单相表（Phase A 终态） */
#ifndef M1_DEADBAND_LUT_APPLY_UD
#define M1_DEADBAND_LUT_APPLY_UD  1   /* Phase 1：Ud 补偿能力；路径由 runtime 标志控制 */
#endif

/**
 * phase abc 注入：三相 duty 补偿后去零序 u₀=(Δa+Δb+Δc)/3（论文 4.4 运行时修正）。
 * 锁轴时 u₀≈0；旋转时强制三相补偿之和为零。
 */
#ifndef M1_DEADBAND_LUT_ZERO_SEQ_ENABLE
#define M1_DEADBAND_LUT_ZERO_SEQ_ENABLE  1
#endif

/**
 * LUT 运行时硬门槛：|i_phase| < APPLY_MIN_A 时 comp=0，否则全查表。
 * 硬切会在 ~0.4 A 形成补偿阶跃，污染 dq → 保持 0。
 * 低电流区用 LOW_FLAT 或 capture 曲线本身，勿用本开关。
 */
#ifndef M1_DEADBAND_LUT_APPLY_MIN_ENABLE
#define M1_DEADBAND_LUT_APPLY_MIN_ENABLE   0
#endif
#ifndef M1_DEADBAND_LUT_APPLY_MIN_A
#define M1_DEADBAND_LUT_APPLY_MIN_A        0.40f
#endif

/**
 * phase 表 apply_duty 运行缩放（仅 lut_domain=phase，不影响 d 表 Ud 辨识路径）。
 * 锁轴 Pass0 学的 |u'| 往往大于 Iq 旋转过零修正所需；联调可从 0.2～0.3 扫。
 */
#ifndef M1_DEADBAND_LUT_RUNTIME_SCALE
#define M1_DEADBAND_LUT_RUNTIME_SCALE       1.0f
#endif

/**
 * =1：commit 时 vals[] *= V_FIXED/lut(Iq_probe×cos30°)，runtime scale 恒 1（论文 #8）。
 * 与 M1_DEADBAND_LUT_RUNTIME_SCALE_AUTO 互斥。
 */
#ifndef M1_DEADBAND_LUT_COMMIT_NORMALIZE
#define M1_DEADBAND_LUT_COMMIT_NORMALIZE    0
#endif

#if M1_DEADBAND_LUT_COMMIT_NORMALIZE && M1_DEADBAND_LUT_RUNTIME_SCALE_AUTO
#error "M1_DEADBAND_LUT_COMMIT_NORMALIZE and M1_DEADBAND_LUT_RUNTIME_SCALE_AUTO are mutually exclusive"
#endif

#ifndef M1_DEADBAND_RUNTIME_GEO_ENABLE
#define M1_DEADBAND_RUNTIME_GEO_ENABLE      0
#endif

#ifndef M1_DEADBAND_GEO_TWO_CLUSTER_ENABLE
#define M1_DEADBAND_GEO_TWO_CLUSTER_ENABLE  0
#endif

/** TWO_CLUSTER runtime 分簇电角（NORMAL 运行期也需；与 Pass0 标定角一致） */
#ifndef M1_ID_CAL_THETA_EL_RAD
#define M1_ID_CAL_THETA_EL_RAD       0.5235987755982988f  /* pi/6 = 30° */
#endif
#ifndef M1_ID_CAL_THETA_PASS0_A_RAD
#define M1_ID_CAL_THETA_PASS0_A_RAD  M1_ID_CAL_THETA_EL_RAD
#endif
#ifndef M1_ID_CAL_THETA_PASS0_B_RAD
#define M1_ID_CAL_THETA_PASS0_B_RAD  0.0f
#endif
#ifndef M1_ID_CAL_THETA_MATCH_RAD
#define M1_ID_CAL_THETA_MATCH_RAD    0.02f
#endif

#if M1_DEADBAND_RUNTIME_GEO_ENABLE && !M1_DEADBAND_LUT_APPLY_UD
#error "M1_DEADBAND_RUNTIME_GEO_ENABLE requires M1_DEADBAND_LUT_APPLY_UD=1 (d-table at commit)"
#endif

/**
 * commit 时把 phase LUT 低 Id 区改为线性（去掉 0.15~0.2 A 陡升）。
 * 0=保留 capture 曲线（2308 对照）；1=平坦化 Id∈[首点, M1_DEADBAND_LUT_LOW_FLAT_ID_A]。
 * Phase 1 对照（2026-06-27）：0=关平坦化，验证 2015 中 Id 失败是否由 LOW_FLAT 放大。
 */
#ifndef M1_DEADBAND_LUT_LOW_FLAT_ENABLE
#define M1_DEADBAND_LUT_LOW_FLAT_ENABLE   0
#endif
#ifndef M1_DEADBAND_LUT_LOW_FLAT_ID_A
#define M1_DEADBAND_LUT_LOW_FLAT_ID_A     0.30f
#endif

/**
 * Phase 3 geo 建表（论文 §4.4 ②③④⑤）：
 * 0=×0.866；1=双角 s_geo_samples 合并拟合 s_plut（须 DUAL_ANGLE=1）。
 */
#ifndef M1_DEADBAND_GEO_BUILD_ENABLE
#define M1_DEADBAND_GEO_BUILD_ENABLE      1
#endif

/**
 * =1：merge plut 时 val 仅用 Pass0-A（30°）样本；amp 仍取 30° max|i|。
 * 0010 双角 median 在 0° Ud 偏高时污染小 I / 造成 val 跳变。
 */
#ifndef M1_DEADBAND_GEO_MERGE_VAL30_ONLY
#define M1_DEADBAND_GEO_MERGE_VAL30_ONLY    0
#endif

/** geo 样本池：Pass0-B 单点 |u'| 低于此值丢弃（坏 capture / 3A 欠流） */
#ifndef M1_ID_CAL_GEO_U_MIN_V
#define M1_ID_CAL_GEO_U_MIN_V               0.10f
#endif

/**
 * commit 时 =1 建 fa/fb/fc 三表；=0 仅 proposed 共享 plut（30° amp + 双角 median val）。
 * P0 签收先用 0（离线金标准 ~1.47 V/相）；1146 triplet val≈1.95 V 暂不启用。
 */
#ifndef M1_DEADBAND_GEO_TRIPLET_ENABLE
#define M1_DEADBAND_GEO_TRIPLET_ENABLE    0
#endif

/** GEO_BUILD=0 时旁路对比 geo vs ×0.866，更新 deadband_geo_diff_* */
#ifndef M1_DEADBAND_GEO_DIFF_LOG_ENABLE
#define M1_DEADBAND_GEO_DIFF_LOG_ENABLE   1
#endif

/**
 * 实验 A/B（Id cal 前）：ALIGN(68) → 开环 Uq 或 Ud 阶梯 0/0.2/0.5/1/2/4 V（70..75）→ 77。
 * flow：OPEN_UD_LADDER 或 OPEN_UQ_LADDER → Id Pass0 → ident Bode → …
 * dwell=M1_OPEN_PRE_ID_LADDER_DWELL_S（默认 0.3 s）；θ 固定 30°。
 * 实验 A/B VOFA×12（D=2, 10kHz）：ch0=Ud_out ch1=Uq_out ch2=Vd_est ch3=Vq_est
 *   ch4=Id ch5=Iq ch6=CCR1 ch7=CCR2 ch8=CCR3 ch9=Uref ch10=sector ch11=open_seq_phase
 */
#ifndef M1_OPEN_UQ_PRE_ID_CAL_ENABLE
#define M1_OPEN_UQ_PRE_ID_CAL_ENABLE  0
#endif
#ifndef M1_OPEN_UD_PRE_ID_CAL_ENABLE
#define M1_OPEN_UD_PRE_ID_CAL_ENABLE  0
#endif
/** 1=Pass0/IdCal 完成后自动接 Ud 开环阶梯（recipe: ID_CAL → OPEN_UD）；0=阶梯在 IdCal 前（ident before step） */
#ifndef M1_OPEN_UD_AFTER_ID_CAL
#define M1_OPEN_UD_AFTER_ID_CAL         0
#endif
#if M1_OPEN_UD_AFTER_ID_CAL && !M1_OPEN_UD_PRE_ID_CAL_ENABLE
#error "M1_OPEN_UD_AFTER_ID_CAL requires M1_OPEN_UD_PRE_ID_CAL_ENABLE=1"
#endif
/** 1=Ud 阶梯完成后同次上电接 VASI Ld/Lq（recipe 末步 DEADBAND_FLOW_KIND_LD_LQ_IDENT） */
#ifndef M1_DEADBAND_FLOW_LD_LQ_AFTER_OPEN_UD
#define M1_DEADBAND_FLOW_LD_LQ_AFTER_OPEN_UD  0
#endif
#if M1_DEADBAND_FLOW_LD_LQ_AFTER_OPEN_UD && !M1_LD_LQ_IDENT_ENABLE
#error "M1_DEADBAND_FLOW_LD_LQ_AFTER_OPEN_UD requires M1_LD_LQ_IDENT_ENABLE=1"
#endif
#if M1_DEADBAND_FLOW_LD_LQ_AFTER_OPEN_UD && !M1_OPEN_UD_AFTER_ID_CAL
#error "M1_DEADBAND_FLOW_LD_LQ_AFTER_OPEN_UD requires M1_OPEN_UD_AFTER_ID_CAL=1"
#endif
#if M1_OPEN_UQ_PRE_ID_CAL_ENABLE && M1_OPEN_UD_PRE_ID_CAL_ENABLE
#error "M1_OPEN_UQ_PRE_ID_CAL_ENABLE and M1_OPEN_UD_PRE_ID_CAL_ENABLE are mutually exclusive"
#endif
/** 1=Ud/Uq 开环 ladder 用 LUT runtime abc（Pass0 后验表）；0=deadband OFF */
#ifndef M1_OPEN_PRE_ID_LADDER_LUT_RUNTIME
#define M1_OPEN_PRE_ID_LADDER_LUT_RUNTIME  0
#endif
#if M1_OPEN_PRE_ID_LADDER_LUT_RUNTIME && !M1_DEADBAND_ENABLE
#error "M1_OPEN_PRE_ID_LADDER_LUT_RUNTIME requires M1_DEADBAND_ENABLE=1"
#endif

#if M1_OPEN_UQ_PRE_ID_CAL_ENABLE || M1_OPEN_UD_PRE_ID_CAL_ENABLE
/** ALIGN(68) → 0/0.2/0.5/1/2/4 V 阶梯(70..75) → 77 结束；实验 A=Uq，实验 B=Ud */
#ifndef M1_OPEN_PRE_ID_LADDER_ALIGN_ENABLE
#define M1_OPEN_PRE_ID_LADDER_ALIGN_ENABLE      1
#endif
#ifndef M1_OPEN_PRE_ID_LADDER_FIX_THETA_ENABLE
#define M1_OPEN_PRE_ID_LADDER_FIX_THETA_ENABLE  1
#endif
#ifndef M1_OPEN_PRE_ID_LADDER_THETA_EL_RAD
#define M1_OPEN_PRE_ID_LADDER_THETA_EL_RAD      M1_ID_CAL_THETA_EL_RAD  /* 30° */
#endif
#ifndef M1_OPEN_PRE_ID_LADDER_ALIGN_UD_V
#define M1_OPEN_PRE_ID_LADDER_ALIGN_UD_V        M1_ID_CAL_ALIGN_UD_V    /* 3 V */
#endif
#ifndef M1_OPEN_PRE_ID_LADDER_ALIGN_S
#define M1_OPEN_PRE_ID_LADDER_ALIGN_S           M1_ID_CAL_ALIGN_S       /* 0.5 s */
#endif
#ifndef M1_OPEN_PRE_ID_LADDER_V4
#define M1_OPEN_PRE_ID_LADDER_V4                4.0f
#endif
#ifndef M1_OPEN_PRE_ID_LADDER_N
#define M1_OPEN_PRE_ID_LADDER_N                 6u
#endif
#ifndef M1_OPEN_PRE_ID_LADDER_DWELL_S
#define M1_OPEN_PRE_ID_LADDER_DWELL_S            0.3f
#endif
#if M1_OPEN_PRE_ID_LADDER_ALIGN_ENABLE && !M1_OPEN_PRE_ID_LADDER_FIX_THETA_ENABLE
#error "M1_OPEN_PRE_ID_LADDER_ALIGN_ENABLE requires M1_OPEN_PRE_ID_LADDER_FIX_THETA_ENABLE=1"
#endif
#define M1_OPEN_PRE_ID_LADDER_ALIGN_PHASE       68u
#define M1_OPEN_PRE_ID_LADDER_PHASE_BASE        70u
#define M1_OPEN_PRE_ID_LADDER_DONE_PHASE        (M1_OPEN_PRE_ID_LADDER_PHASE_BASE + M1_OPEN_PRE_ID_LADDER_N + 1u)
#ifndef M1_OPEN_PRE_ID_LADDER_AB_ENABLE
#define M1_OPEN_PRE_ID_LADDER_AB_ENABLE         0
#endif
#if M1_OPEN_PRE_ID_LADDER_AB_ENABLE && !M1_DEADBAND_ENABLE
#error "M1_OPEN_PRE_ID_LADDER_AB_ENABLE requires M1_DEADBAND_ENABLE=1"
#endif
#define M1_OPEN_PRE_ID_LADDER_PHASE_ON_BASE     (M1_OPEN_PRE_ID_LADDER_PHASE_BASE + 10u)
#define M1_OPEN_PRE_ID_LADDER_DONE_ON_PHASE     (M1_OPEN_PRE_ID_LADDER_PHASE_ON_BASE + M1_OPEN_PRE_ID_LADDER_N + 1u)
/** @deprecated 实验 A 别名 */
#define M1_OPEN_UQ_LADDER_ALIGN_ENABLE      M1_OPEN_PRE_ID_LADDER_ALIGN_ENABLE
#define M1_OPEN_UQ_LADDER_FIX_THETA_ENABLE  M1_OPEN_PRE_ID_LADDER_FIX_THETA_ENABLE
#define M1_OPEN_UQ_LADDER_THETA_EL_RAD      M1_OPEN_PRE_ID_LADDER_THETA_EL_RAD
#define M1_OPEN_UQ_LADDER_ALIGN_UD_V        M1_OPEN_PRE_ID_LADDER_ALIGN_UD_V
#define M1_OPEN_UQ_LADDER_ALIGN_S           M1_OPEN_PRE_ID_LADDER_ALIGN_S
#define M1_OPEN_UQ_LADDER_ALIGN_PHASE       M1_OPEN_PRE_ID_LADDER_ALIGN_PHASE
#define M1_OPEN_UQ_LADDER_DWELL_S           M1_OPEN_PRE_ID_LADDER_DWELL_S
#define M1_OPEN_UQ_LADDER_PHASE_BASE        M1_OPEN_PRE_ID_LADDER_PHASE_BASE
#define M1_OPEN_UQ_LADDER_DONE_PHASE        M1_OPEN_PRE_ID_LADDER_DONE_PHASE
#endif

/**
 * 开环 Uq 阶梯 + 死区 A/B（2026-06-23 综合录波）
 *
 * M1_OPEN_UQ_DEADBAND_AB_SWEEP=1 时 15 s 内 6 段（各 2.5 s）：
 *   phase 0: Uq=2.0 V, deadband OFF
 *   phase 1: Uq=2.0 V, deadband ON  (M1_DEADTIME_NS)
 *   phase 2: Uq=2.5 V, deadband OFF
 *   phase 3: Uq=2.5 V, deadband ON
 *   phase 4: Uq=3.0 V, deadband OFF
 *   phase 5: Uq=3.0 V, deadband ON
 * Watch: dbg.open_seq_phase = 0..5；VOFA ch4=扇区(M1_VOFA_SECTOR_DIAG=1)
 *
 * =0 时沿用旧逻辑：2/2.5/3 V 各 M1_OPEN_UQ_SWEEP_STEP_S（默认 5 s），死区由 M1_DEADBAND_ENABLE 编译决定。
 */
#ifndef M1_OPEN_UQ_DEADBAND_AB_SWEEP
#define M1_OPEN_UQ_DEADBAND_AB_SWEEP  0
#endif

/** A/B  sweep 每半段时长（s）；6 段合计 15 s */
#define M1_OPEN_UQ_HALF_STEP_S        2.5f

#define M1_OPEN_UQ_SWEEP_STEP_S       5.0f
#define M1_OPEN_UQ_SWEEP_V0           2.0f
#define M1_OPEN_UQ_SWEEP_V1           2.5f
#define M1_OPEN_UQ_SWEEP_V2           3.0f

/** @deprecated 用 M1_OPEN_UQ_SWEEP_V0 */
#define M1_OPEN_UQ_RUN_V          M1_OPEN_UQ_SWEEP_V0

/**
 * ADC2 注入 rank 交换试验（须与 binding 同步，见 docs/VOFA联调记录_20260617_扇区诊断全CSV.md §6）：
 * 0=默认 JDR1=PC3(ic) JDR3=PC4(ia)，binding [2,1,0]
 * 1=交换 JDR1=PC4(ia) JDR3=PC3(ic)，binding [0,1,2]（ic 最后采）
 */
#ifndef M1_ADC_RANK_SWAP_IAIC
#define M1_ADC_RANK_SWAP_IAIC     0
#endif

/* --- 扇区条件电流重构：仅扇区 1～2 重建 ic（180010 路线；CCR4 扫时用 0） --- */
#ifndef M1_CURRENT_RECON_ENABLE
#define M1_CURRENT_RECON_ENABLE   1
#endif

/** |i| 低于此值（A）时不重构，避免零漂放大 */
#ifndef M1_CURRENT_RECON_MIN_A
#define M1_CURRENT_RECON_MIN_A    0.05f
#endif

/* --- Iq 环辨识（堵转 + 磁粉制动器；与 Id 扫表互斥，由 M1_BRINGUP_MODE 开关） --- */
/**
 * =1：堵转 CURRENT_LOOP；可选先 Id Pass0 commit，再 Bode 或 Iq 阶跃。
 *     Park 角：编码器 θ（FIX_THETA=0）；M1_IDENT_OVERRIDE_LIMITS=1。
 *     IDENT 模式：Id Pass0 建表 → HOLD 2s → 阶跃×8(低0+高1A，OFF/LUT各2) → Bode×4(0.25/1.25A)。
 * VOFA：Id 段 ch3=Ud ch4=Id_ref；Ident ch3=Iq ch4=Iq_ref ch5=Uq_pi。
 * open_seq：Id 1..N→9；60=HOLD 61..66=Step-OFF 74..79=Step-LUT 62=Bode-OFF 63=Bode-LUT 73=DONE。
 * 选 M1_BRINGUP_MODE_IDENT_IQ_STEP 时自动 =1。
 */
#ifndef M1_IDENT_ENABLE
#define M1_IDENT_ENABLE             0
#endif

#ifndef M1_IDENT_POST_BODE_OPEN_UQ_ENABLE
#define M1_IDENT_POST_BODE_OPEN_UQ_ENABLE  0
#endif
#ifndef M1_IDENT_POST_BODE_OPEN_UQ_V
#define M1_IDENT_POST_BODE_OPEN_UQ_V         M1_OPEN_UQ_SWEEP_V0
#endif
#ifndef M1_IDENT_POST_BODE_OPEN_UQ_S
#define M1_IDENT_POST_BODE_OPEN_UQ_S         5.0f
#endif

#if M1_IDENT_ENABLE
#if M1_ID_LOCK_CAL_SWEEP && !M1_IDENT_ID_CAL_BEFORE_STEP
#error "M1_IDENT_ENABLE requires M1_ID_LOCK_CAL_SWEEP=0 (or M1_IDENT_ID_CAL_BEFORE_STEP=1)"
#endif

#ifndef M1_IDENT_ID_CAL_BEFORE_STEP
#define M1_IDENT_ID_CAL_BEFORE_STEP   0
#endif

#ifndef M1_IDENT_IQ_STEP_ENABLE
#define M1_IDENT_IQ_STEP_ENABLE     1
#endif
#ifndef M1_IDENT_IQ_BODE_ENABLE
#define M1_IDENT_IQ_BODE_ENABLE     0   /* 先阶跃；=1 时 STEP 后接 BODE */
#endif
#if !M1_IDENT_IQ_STEP_ENABLE && !M1_IDENT_IQ_BODE_ENABLE
#error "M1_IDENT_ENABLE requires M1_IDENT_IQ_STEP_ENABLE and/or M1_IDENT_IQ_BODE_ENABLE"
#endif

#ifndef M1_IDENT_FIX_THETA_ENABLE
#define M1_IDENT_FIX_THETA_ENABLE   0   /* 0=编码器 θ（堵转默认）；1=写死 30° */
#endif
#ifndef M1_IDENT_THETA_EL_RAD
#define M1_IDENT_THETA_EL_RAD       0.5235987755982988f  /* 仅 FIX_THETA=1 时用 */
#endif

/** 制动器加载后稳定等待 (s) */
#ifndef M1_IDENT_HOLD_S
#define M1_IDENT_HOLD_S               2.0f
#endif
/** 阶跃总轮数 / OFF / FIXED：IDENT 模式在上方 #elif 已设为 18 / 6 / 6；此处仅为未覆盖时的兜底 */
#ifndef M1_IDENT_STEP_ROUNDS
#define M1_IDENT_STEP_ROUNDS          18u
#endif
#ifndef M1_IDENT_STEP_OFF_ROUNDS
#define M1_IDENT_STEP_OFF_ROUNDS      6u
#endif
#ifndef M1_IDENT_STEP_FIXED_ROUNDS
#define M1_IDENT_STEP_FIXED_ROUNDS    6u
#endif
/** 1 且 FIXED_ROUNDS=0：OFF 之后全用 LUT；FIXED_ROUNDS>0 时第三段恒为 LUT */
#ifndef M1_IDENT_STEP_LUT_AFTER_OFF
#define M1_IDENT_STEP_LUT_AFTER_OFF   0
#endif
#if M1_IDENT_IQ_STEP_ENABLE
#if (M1_IDENT_STEP_OFF_ROUNDS > M1_IDENT_STEP_ROUNDS)
#error "M1_IDENT_STEP_OFF_ROUNDS must be <= M1_IDENT_STEP_ROUNDS"
#endif
#if (M1_IDENT_STEP_OFF_ROUNDS + M1_IDENT_STEP_FIXED_ROUNDS > M1_IDENT_STEP_ROUNDS)
#error "M1_IDENT_STEP_OFF_ROUNDS + M1_IDENT_STEP_FIXED_ROUNDS must be <= M1_IDENT_STEP_ROUNDS"
#endif
#if ((M1_IDENT_STEP_ROUNDS - M1_IDENT_STEP_OFF_ROUNDS - M1_IDENT_STEP_FIXED_ROUNDS) > 0u) && \
    !M1_DEADBAND_LUT_BAKED_ENABLE && !M1_IDENT_ID_CAL_BEFORE_STEP
#error "LUT segment requires M1_DEADBAND_LUT_BAKED_ENABLE or M1_IDENT_ID_CAL_BEFORE_STEP"
#endif
#endif /* M1_IDENT_IQ_STEP_ENABLE */
#ifndef M1_IDENT_STEP_I0_A
#define M1_IDENT_STEP_I0_A            0.0f
#endif
#ifndef M1_IDENT_STEP_I1_A
#define M1_IDENT_STEP_I1_A            0.3f   /* 小阶跃 */
#endif
#ifndef M1_IDENT_STEP_I2_A
#define M1_IDENT_STEP_I2_A            0.5f   /* 中阶跃 */
#endif
#ifndef M1_IDENT_STEP_I3_A
#define M1_IDENT_STEP_I3_A            1.0f   /* 低段大阶跃 */
#endif
#ifndef M1_IDENT_STEP_I_BASE_HI_A
#define M1_IDENT_STEP_I_BASE_HI_A     1.0f   /* 高段 1 A 基线 */
#endif
#ifndef M1_IDENT_STEP_I5_A
#define M1_IDENT_STEP_I5_A            1.3f
#endif
#ifndef M1_IDENT_STEP_I6_A
#define M1_IDENT_STEP_I6_A            1.5f
#endif
#ifndef M1_IDENT_STEP_I7_A
#define M1_IDENT_STEP_I7_A            2.0f
#endif
#ifndef M1_IDENT_STEP_BANDS
#define M1_IDENT_STEP_BANDS           1u
#endif
#ifndef M1_IDENT_STEP_ROUNDS_PER_PROFILE
#define M1_IDENT_STEP_ROUNDS_PER_PROFILE  M1_IDENT_STEP_OFF_ROUNDS
#endif
#if (M1_IDENT_STEP_BANDS > 1u) && \
    (M1_IDENT_STEP_ROUNDS != (M1_IDENT_STEP_BANDS * 2u * M1_IDENT_STEP_ROUNDS_PER_PROFILE))
#error "M1_IDENT_STEP_ROUNDS must equal M1_IDENT_STEP_BANDS * 2 * M1_IDENT_STEP_ROUNDS_PER_PROFILE"
#endif
/** 各拍 dwell (s)；非零档与回零档相同 */
#ifndef M1_IDENT_STEP_DWELL_S
#define M1_IDENT_STEP_DWELL_S         0.5f
#endif
#ifndef M1_IDENT_STEP_ZERO_DWELL_S
#define M1_IDENT_STEP_ZERO_DWELL_S    0.5f
#endif

/** 1=阶跃辨识绕过 M1_CLOSURE_BRINGUP 的 I_ref 钳位与 6 V PI 限幅 */
#ifndef M1_IDENT_OVERRIDE_LIMITS
#define M1_IDENT_OVERRIDE_LIMITS      1
#endif
#if M1_IDENT_OVERRIDE_LIMITS
#define M1_IDENT_PI_V_LIMIT_V         M1_PI_V_MAX
#else
#define M1_IDENT_PI_V_LIMIT_V         M1_PI_V_LIMIT_V
#endif
#define M1_IDENT_PI_V_LIMIT_MIN       (-M1_IDENT_PI_V_LIMIT_V)
#define M1_IDENT_PI_INT_LIMIT_V       M1_IDENT_PI_V_LIMIT_V
#define M1_IDENT_PI_INT_LIMIT_MIN     (-M1_IDENT_PI_V_LIMIT_V)

/** Bode 轴：0=Iq sin（Id=0）；1=Id sin（Iq=0，θ 锁轴） */
#ifndef M1_IDENT_BODE_AXIS_ID
#define M1_IDENT_BODE_AXIS_ID         0
#endif

/** Bode：i_ref = bias + amp*sin(2πft)，几何扫频 f *= RATIO */
#ifndef M1_IDENT_BODE_I_BIAS_A
#define M1_IDENT_BODE_I_BIAS_A        0.25f
#endif
#ifndef M1_IDENT_BODE_I_AMP_A
#define M1_IDENT_BODE_I_AMP_A         0.05f
#endif
#ifndef M1_IDENT_BODE_I_BIAS_HI_A
#define M1_IDENT_BODE_I_BIAS_HI_A     1.25f   /* 高 I Bode；无 1.0 A 档 */
#endif
#ifndef M1_IDENT_BODE_I_AMP_HI_A
#define M1_IDENT_BODE_I_AMP_HI_A      0.10f
#endif
#ifndef M1_IDENT_BODE_LUT_ENABLE
#define M1_IDENT_BODE_LUT_ENABLE      1
#endif
#ifndef M1_IDENT_BODE_BANDS
#define M1_IDENT_BODE_BANDS           1u
#endif
#if M1_IDENT_BODE_LUT_ENABLE
#if (M1_IDENT_BODE_BANDS > 1u) && (M1_IDENT_BODE_ROUNDS != (M1_IDENT_BODE_BANDS * 2u))
#error "M1_IDENT_BODE_ROUNDS must equal M1_IDENT_BODE_BANDS * 2"
#endif
#else
#if (M1_IDENT_BODE_BANDS > 1u) && (M1_IDENT_BODE_ROUNDS != M1_IDENT_BODE_BANDS)
#error "OFF-only Bode: M1_IDENT_BODE_ROUNDS must equal M1_IDENT_BODE_BANDS"
#endif
#endif
#ifndef M1_IDENT_BODE_F0_HZ
#define M1_IDENT_BODE_F0_HZ           10.0f
#endif
#ifndef M1_IDENT_BODE_F1_HZ
/** legacy 兜底：仅文档/脚本参考；运行时频表以 ident_bode_freq_table.h 为准（F1=800 → 38 点/846 Hz） */
#define M1_IDENT_BODE_F1_HZ           800.0f
#endif
#ifndef M1_IDENT_BODE_F_RATIO
#define M1_IDENT_BODE_F_RATIO         1.15f
#endif
#ifndef M1_IDENT_BODE_F_SPLIT_HZ
#define M1_IDENT_BODE_F_SPLIT_HZ      500.0f
#endif
#ifndef M1_IDENT_BODE_F_RATIO_HI
#define M1_IDENT_BODE_F_RATIO_HI      1.06f
#endif
#ifndef M1_IDENT_BODE_CYCLES_PER_FREQ
#define M1_IDENT_BODE_CYCLES_PER_FREQ 20.0f
#endif
/** f >= F_SPLIT 固定观测窗：1=Ncyc=T_obs*f；0=用 CYCLES_HI 常数 */
#ifndef M1_IDENT_BODE_USE_T_OBS_HI
#define M1_IDENT_BODE_USE_T_OBS_HI    1
#endif
#ifndef M1_IDENT_BODE_T_OBS_HI_S
#define M1_IDENT_BODE_T_OBS_HI_S      0.1f   /* 100 ms；@1 kHz → 100 cycles */
#endif
#ifndef M1_IDENT_BODE_CYCLES_HI
#define M1_IDENT_BODE_CYCLES_HI       50.0f  /* 仅 USE_T_OBS_HI=0 时生效 */
#endif
/** Bode 分段：1×OFF + 1×FIXED + 1×LUT，各跑 F0→F1 一整遍 */
#ifndef M1_IDENT_BODE_ROUNDS
#define M1_IDENT_BODE_ROUNDS          3u
#endif
#ifndef M1_IDENT_BODE_OFF_ROUNDS
#define M1_IDENT_BODE_OFF_ROUNDS      1u
#endif
#ifndef M1_IDENT_BODE_FIXED_ROUNDS
#define M1_IDENT_BODE_FIXED_ROUNDS    1u
#endif
#if M1_IDENT_IQ_BODE_ENABLE && M1_IDENT_BODE_LUT_ENABLE
#if (M1_IDENT_BODE_OFF_ROUNDS + M1_IDENT_BODE_FIXED_ROUNDS > M1_IDENT_BODE_ROUNDS)
#error "M1_IDENT_BODE_OFF_ROUNDS + M1_IDENT_BODE_FIXED_ROUNDS must be <= M1_IDENT_BODE_ROUNDS"
#endif
#if ((M1_IDENT_BODE_ROUNDS - M1_IDENT_BODE_OFF_ROUNDS - M1_IDENT_BODE_FIXED_ROUNDS) > 0u) && \
    !M1_DEADBAND_LUT_BAKED_ENABLE && !M1_IDENT_ID_CAL_BEFORE_STEP
#error "Bode LUT segment requires M1_IDENT_ID_CAL_BEFORE_STEP or M1_DEADBAND_LUT_BAKED_ENABLE"
#endif
#endif /* M1_IDENT_IQ_BODE_ENABLE && M1_IDENT_BODE_LUT_ENABLE */
#endif /* M1_IDENT_ENABLE */

/* --- 速度环辨识（旋转、deadband OFF；与 Id/Iq ident 互斥） --- */
#ifndef M1_SPEED_IDENT_ENABLE
#define M1_SPEED_IDENT_ENABLE           0
#endif

#if M1_SPEED_IDENT_ENABLE
#if M1_IDENT_ENABLE
#error "M1_SPEED_IDENT_ENABLE and M1_IDENT_ENABLE are mutually exclusive"
#endif
#if M1_ID_LOCK_CAL_SWEEP
#error "M1_SPEED_IDENT_ENABLE requires M1_ID_LOCK_CAL_SWEEP=0"
#endif
#if !M1_SPEED_LOOP_ENABLE
#error "M1_SPEED_IDENT_ENABLE requires M1_SPEED_LOOP_ENABLE=1"
#endif

#ifndef M1_SPEED_IDENT_STEP_ENABLE
#define M1_SPEED_IDENT_STEP_ENABLE      1
#endif
#ifndef M1_SPEED_IDENT_BODE_ENABLE
#define M1_SPEED_IDENT_BODE_ENABLE      1
#endif
#if !M1_SPEED_IDENT_STEP_ENABLE && !M1_SPEED_IDENT_BODE_ENABLE
#error "M1_SPEED_IDENT_ENABLE requires STEP and/or BODE"
#endif

#ifndef M1_SPEED_IDENT_HOLD_S
#define M1_SPEED_IDENT_HOLD_S           8.0f
#endif
#ifndef M1_SPEED_IDENT_RPM_START
#define M1_SPEED_IDENT_RPM_START        100.0f
#endif
#ifndef M1_SPEED_IDENT_STEP_ROUNDS
#define M1_SPEED_IDENT_STEP_ROUNDS      1u
#endif
#ifndef M1_SPEED_IDENT_STEP_RPM0
#define M1_SPEED_IDENT_STEP_RPM0        500.0f
#endif
#ifndef M1_SPEED_IDENT_STEP_RPM1
#define M1_SPEED_IDENT_STEP_RPM1        300.0f
#endif
#ifndef M1_SPEED_IDENT_STEP_RPM2
#define M1_SPEED_IDENT_STEP_RPM2        700.0f
#endif
#ifndef M1_SPEED_IDENT_STEP_RPM3
#define M1_SPEED_IDENT_STEP_RPM3        900.0f
#endif
#ifndef M1_SPEED_IDENT_STEP_DWELL_S
#define M1_SPEED_IDENT_STEP_DWELL_S     8.0f
#endif
#ifndef M1_SPEED_IDENT_STEP_ZERO_DWELL_S
#define M1_SPEED_IDENT_STEP_ZERO_DWELL_S  3.0f
#endif
#ifndef M1_SPEED_IDENT_BODE_RPM_BIAS
#define M1_SPEED_IDENT_BODE_RPM_BIAS    500.0f
#endif
#ifndef M1_SPEED_IDENT_BODE_RPM_AMP
#define M1_SPEED_IDENT_BODE_RPM_AMP     30.0f
#endif
#ifndef M1_SPEED_IDENT_BODE_CYCLES_PER_FREQ
#define M1_SPEED_IDENT_BODE_CYCLES_PER_FREQ  10.0f
#endif
#endif /* M1_SPEED_IDENT_ENABLE */

#endif
