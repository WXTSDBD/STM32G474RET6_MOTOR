/**
 * @file m1_hfi_standstill.profile.h
 * @date 2026-10-06
 * @brief 无感静止起动交付 profile：打开 HFI、运动旁路、观测速度反馈、编码器可选。
 *
 * 切回有感只改 bringup_active.h 里选哪份 profile。解调、PLL、注入仍在 hfi_sqwave.c。
 * 本文件只铺宏，不含函数。
 *
 * Iq 限幅、ω 斜坡、Iq slew 的产品口径见
 * docs/规范_外环限幅与斜坡_出厂整定_2026-10-11.md；下列安培/斜坡数为台架临时。
 *
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#ifndef CONFIG_PROFILES_M1_HFI_STANDSTILL_PROFILE_H
#define CONFIG_PROFILES_M1_HFI_STANDSTILL_PROFILE_H

#ifndef M1_HFI_GATE
#define M1_HFI_GATE                     141
#endif
#if M1_HFI_GATE != 141
#error "M1_HFI_GATE must be 141 (delivery profile)"
#endif

#define M1_ID_LOCK_CAL_SWEEP            0
#define M1_IDENT_ENABLE                 0
#define M1_ID_CAL_ALIGN_ENABLE          0
#define M1_ID_CAL_COMMIT_LUT            0
#define M1_LD_LQ_IDENT_ENABLE           0
#define M1_RS_IDENT_ENABLE              0
#define M1_VOFA_IDENT_DUMP_ENABLE       0

#define M1_SPEED_IDENT_ENABLE           0
#define M1_SPEED_IDENT_STEP_ENABLE      0
#define M1_SPEED_IDENT_BODE_ENABLE      0
#define M1_SPEED_PROFILE_ENABLE         0

#undef M1_SPEED_LOOP_ENABLE
#define M1_SPEED_LOOP_ENABLE            1
#undef M1_SPEED_LOOP_BOOT
#define M1_SPEED_LOOP_BOOT              0
/*
 * TODO(factory)：ω 斜坡 / Iq slew / 制动电流 — 出厂自算；现行联调临时，勿随 VBUS 臆改。
 * 慢慢刹优先平滑（2225）：反向全程同一慢 DECEL；关 DECEL_BRAKE，避免终值一刀负 Iq 空载失步。
 * DECEL=800 < 空载滑行~1500~1900 → 多靠滑行，手感匀；要更快再开轻刹地板。
 */
#undef M1_SPEED_OMEGA_RAMP_ENABLE
#define M1_SPEED_OMEGA_RAMP_ENABLE      1
#undef M1_SPEED_OMEGA_RAMP_RPM_S
#define M1_SPEED_OMEGA_RAMP_RPM_S       500.0f
#undef M1_SPEED_OMEGA_RAMP_DECEL_RPM_S
#define M1_SPEED_OMEGA_RAMP_DECEL_RPM_S 800.0f
#undef M1_SPEED_IQ_SLEW_ENABLE
#define M1_SPEED_IQ_SLEW_ENABLE         1
#undef M1_SPEED_IQ_SLEW_A_PER_S
#define M1_SPEED_IQ_SLEW_A_PER_S        40.0f
#undef M1_SPEED_DECEL_BRAKE_ENABLE
#define M1_SPEED_DECEL_BRAKE_ENABLE     0
#undef M1_SPEED_DECEL_BRAKE_A
#define M1_SPEED_DECEL_BRAKE_A          2.0f
#undef M1_SPEED_DECEL_BRAKE_MARGIN_RPM
#define M1_SPEED_DECEL_BRAKE_MARGIN_RPM 120.0f
#undef M1_SPEED_DECEL_BRAKE_MIN_RPM
#define M1_SPEED_DECEL_BRAKE_MIN_RPM    120.0f

#undef M1_OPEN_UD_PRE_ID_CAL_ENABLE
#define M1_OPEN_UD_PRE_ID_CAL_ENABLE    0
#undef M1_OPEN_UQ_PRE_ID_CAL_ENABLE
#define M1_OPEN_UQ_PRE_ID_CAL_ENABLE    0

#undef M1_STARTUP_ENABLE
#define M1_STARTUP_ENABLE               0

#define M1_DEADBAND_ENABLE              0
#define M1_DEADBAND_LUT_BAKED_ENABLE    0
#define M1_DEADBAND_NVM_ON_BOOT         0

#undef M1_IF_ENABLE
#define M1_IF_ENABLE                    0
#undef M1_IF_TO_OBS_ENABLE
#define M1_IF_TO_OBS_ENABLE             0

/*
 * 回收 GATE 141 纯无感基线（拔线验交接）。有感对照改回 SENSED_PAIRS=1 即可，
 * 有感路径代码未删。速度环吃 HFI/SMO；踢后残速判南北（见 QKICK_POL_*）。
 */
#undef M1_ENC_OPTIONAL_ENABLE
#define M1_ENC_OPTIONAL_ENABLE          0   /* 金样实验：要跑"有感一轮"，编码器必须编进来 */

#undef M1_IQ_REF_A
#define M1_IQ_REF_A                     0.0f

#undef M1_HFI_ENABLE
#define M1_HFI_ENABLE                   1
#undef M1_HFI_MOTION_BYPASS_ENABLE
#define M1_HFI_MOTION_BYPASS_ENABLE     1
#undef M1_HFI_IPD_SWEEP_ENABLE
#define M1_HFI_IPD_SWEEP_ENABLE         0
#undef M1_HFI_QKICK_AFTER_LOCK_ENABLE
#define M1_HFI_QKICK_AFTER_LOCK_ENABLE  0
#undef M1_HFI_DQ_IDENT_ENABLE
#define M1_HFI_DQ_IDENT_ENABLE          0

#undef M1_HFI_PLL_EPS_DEAD
#define M1_HFI_PLL_EPS_DEAD             0.0f
#undef M1_HFI_VH_V
#define M1_HFI_VH_V                     0.40f
#undef M1_HFI_SEQ_ENABLE
#define M1_HFI_SEQ_ENABLE               1
#undef M1_HFI_FH_HZ
#define M1_HFI_FH_HZ                    10000.0f

#undef M1_HFI_INIT_FROM_ENC
#define M1_HFI_INIT_FROM_ENC            0
#undef M1_HFI_PLL_INIT_OFF_RAD
#define M1_HFI_PLL_INIT_OFF_RAD         0.0f

#undef M1_HFI_PLL_ENABLE
#define M1_HFI_PLL_ENABLE               1
#undef M1_HFI_PLL_KP
#define M1_HFI_PLL_KP                   39.2f
#undef M1_HFI_PLL_KI
#define M1_HFI_PLL_KI                   389.0f
#undef M1_HFI_PLL_W_MAX
#define M1_HFI_PLL_W_MAX                200.0f
#undef M1_HFI_PLL_INT_MAX
#define M1_HFI_PLL_INT_MAX              M1_HFI_PLL_W_MAX
#undef M1_HFI_SPEED_FB_ENABLE
#define M1_HFI_SPEED_FB_ENABLE          0
#undef M1_HFI_EPS_SIGN
#define M1_HFI_EPS_SIGN                 1.0f
/* ISR：PLL 吃 VESC 误差；关 atan2，s_eps 复用 s_e_pll（AUTH/对照）。 */
#undef M1_HFI_ATAN2_ENABLE
#define M1_HFI_ATAN2_ENABLE             0
#undef M1_HFI_A_CMD
#define M1_HFI_A_CMD                    (0.10f)

#undef M1_EMF_VEQ_ENABLE
#define M1_EMF_VEQ_ENABLE               0
#undef M1_EMF_SMO_ENABLE
#define M1_EMF_SMO_ENABLE               0
#undef M1_EMF_PLL_ENABLE
#define M1_EMF_PLL_ENABLE               0
#undef M1_OBS_SOFT_SWITCH_ENABLE
#define M1_OBS_SOFT_SWITCH_ENABLE       0

#undef M1_PLL_ENABLE
#define M1_PLL_ENABLE                   1
#undef M1_VOFA_UNIFIED_12CH
#define M1_VOFA_UNIFIED_12CH            1
#undef M1_VOFA_OPT12
#define M1_VOFA_OPT12                   1 /* OPT12 布局，ch9 已改成 id（其余列与之前完全一致） */
#undef M1_VOFA_SPEED_ISR_CH
#define M1_VOFA_SPEED_ISR_CH            1 /* SPEED 布局 ch10 = isr_delta（用来分有感/无感段） */
#undef M1_TELEM_BRINGUP_K
#define M1_TELEM_BRINGUP_K              12u
#undef M1_TELEM_BRINGUP_DECIMATION
#define M1_TELEM_BRINGUP_DECIMATION     2u /* 10 kHz；人要求 D=2，勿改回 8 */

/* ISR dbg 瘦身：默认关；勿再默默改回满拷（会抬无感 foc） */
#undef M1_DBG_ISR_MIRROR_HFI
#define M1_DBG_ISR_MIRROR_HFI           0
#undef M1_DBG_ISR_MIRROR_OBS
#define M1_DBG_ISR_MIRROR_OBS           0
#undef M1_DBG_ISR_MIRROR_SVPWM
#define M1_DBG_ISR_MIRROR_SVPWM         0
#undef M1_DBG_ISR_MIRROR_ACDC
#define M1_DBG_ISR_MIRROR_ACDC          0
#undef M1_DBG_ISR_MIRROR_PLL_DIAG
#define M1_DBG_ISR_MIRROR_PLL_DIAG      0

#undef M1_VOFA_HFI_12CH
#define M1_VOFA_HFI_12CH                0
#undef M1_VOFA_IF_12CH
#define M1_VOFA_IF_12CH                 0
#undef M1_VOFA_OBS_SMO_12CH
#define M1_VOFA_OBS_SMO_12CH            0
#undef M1_VOFA_OBS_PLL_12CH
#define M1_VOFA_OBS_PLL_12CH            0
#undef M1_VOFA_OBS_VEQ_12CH
#define M1_VOFA_OBS_VEQ_12CH            0

/* -------------------------------------------------------------------------- */

#if M1_HFI_GATE == 141
/* GATE 141：无感一对 ±TARGET → 刹到 0（SENSED_PAIRS=0）。RAMP/SLEW 开。 */
/* ±2000 实验参数已从宏迁到 hfi_sqwave.c 的 static const（阶段2 表化）。 */
#undef M1_VOFA_HFI_ISR_CH
#define M1_VOFA_HFI_ISR_CH              1
/*
 * 交接：回到 10-06 冻结（obs_cfg 默认）。UP 1300 / DOWN 1000 / 关注入 1500。
 * 勿再用 10-10 ISR 试验的 500/600 早交接。
 */
#undef M1_HFI_PUB_UP_RPM
#define M1_HFI_PUB_UP_RPM               1300.0f
#undef M1_HFI_PUB_DOWN_RPM
#define M1_HFI_PUB_DOWN_RPM             1000.0f
#undef M1_HFI_PUB_INJ_OFF_RPM
#define M1_HFI_PUB_INJ_OFF_RPM          1500.0f
#undef M1_HFI_PUB_INJ_ON_RPM
#define M1_HFI_PUB_INJ_ON_RPM           1400.0f
#undef M1_HFI_PUB_DN_QUAL_N
#define M1_HFI_PUB_DN_QUAL_N            400u
#undef M1_HFI_PUB_FREEZE_ENABLE
#define M1_HFI_PUB_FREEZE_ENABLE        0
#undef M1_HFI_PUB_FREEZE_RPM_MIN
#define M1_HFI_PUB_FREEZE_RPM_MIN       200.0f
#undef M1_HFI_PUB_FREEZE_RPM_MAX
#define M1_HFI_PUB_FREEZE_RPM_MAX       900.0f
#undef M1_HFI_PUB_FREEZE_DRPM
#define M1_HFI_PUB_FREEZE_DRPM          15.0f
#undef M1_HFI_PUB_FREEZE_N
#define M1_HFI_PUB_FREEZE_N             400u
/*
 * ISR 第 2 步：低速只 HFI，跳过 SMO/PLL。
 * 1803：交接 |w|800–1200 且 ss=0 时双开 ~10.3k；把门槛 200→400，
 * 拉长「只 HFI」段，缩短未发布前的 HFI+SMO 重叠（离开门槛仍 reset SMO）。
 */
#undef M1_HFI_MUX_LO_ENABLE
#define M1_HFI_MUX_LO_ENABLE            0
#undef M1_HFI_MUX_LO_RPM
#define M1_HFI_MUX_LO_RPM               400.0f
/* 正余弦固定 LUT（motor_trig_cfg 默认）；勿用 -DMOTOR_TRIG_BACKEND=2。 */
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              1
#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             5.0f
/* REV141 用 ±TARGET 表，勿开单调阶梯（LADDER=1 会变成 100→200→…台阶）。 */
#undef M1_HFI_RUN_LADDER_ENABLE
#define M1_HFI_RUN_LADDER_ENABLE        0
#undef M1_HFI_RUN_RPM_START
#define M1_HFI_RUN_RPM_START            100.0f
#undef M1_HFI_RUN_RPM_STEP
#define M1_HFI_RUN_RPM_STEP             100.0f
#undef M1_HFI_RUN_RPM_MAX
#define M1_HFI_RUN_RPM_MAX              2000.0f
#undef M1_HFI_RUN_STEP_S
#define M1_HFI_RUN_STEP_S               5.0f
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 100.0f
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               5.0f
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 500.0f
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               20.0f
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (-1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (-1.0f)
#undef M1_HFI_QKICK_AFTER_LOCK_ENABLE
#define M1_HFI_QKICK_AFTER_LOCK_ENABLE  1
#undef M1_HFI_QKICK_THEN_HFI_ENABLE
#define M1_HFI_QKICK_THEN_HFI_ENABLE    1
#undef M1_HFI_QKICK_PRE_S
#define M1_HFI_QKICK_PRE_S              2.0f
#undef M1_HFI_QKICK_IQ_A
#define M1_HFI_QKICK_IQ_A               1.6f
#undef M1_HFI_QKICK_KICK_N
#define M1_HFI_QKICK_KICK_N             6000u
#undef M1_HFI_QKICK_HOLD_S
#define M1_HFI_QKICK_HOLD_S             0.6f
/* 无感：LOG 内用踢后残速判南北；窗口夹在 HOLD 内。 */
#undef M1_HFI_QKICK_POL_DECIDE_S
#define M1_HFI_QKICK_POL_DECIDE_S       0.50f
#undef M1_HFI_QKICK_POL_W_RPM
#define M1_HFI_QKICK_POL_W_RPM          8.0f
#undef M1_HFI_INJECT_POST_LOOP
#define M1_HFI_INJECT_POST_LOOP         1
#undef M1_HFI_DEMOD_INJ_AXIS
#define M1_HFI_DEMOD_INJ_AXIS           1
#undef M1_HFI_ID_ON_FROM_RUN_ENABLE
#define M1_HFI_ID_ON_FROM_RUN_ENABLE    1
#undef M1_HFI_SPEED_FB_ENABLE
#define M1_HFI_SPEED_FB_ENABLE          1
#undef M1_HFI_SPEED_OBS_INT
#define M1_HFI_SPEED_OBS_INT            1
#undef M1_HFI_PLL_KP
#define M1_HFI_PLL_KP                   1200.0f
#undef M1_HFI_PLL_VESC_ERR_ENABLE
#define M1_HFI_PLL_VESC_ERR_ENABLE      1
#undef M1_HFI_PLL_VESC_MAX_ERR
#define M1_HFI_PLL_VESC_MAX_ERR         (0.30f)
#undef M1_HFI_PLL_VESC_ERR_SIGN
#define M1_HFI_PLL_VESC_ERR_SIGN        (-1.0f)
#undef M1_HFI_IQ_AUTH_ENABLE
#define M1_HFI_IQ_AUTH_ENABLE           1
#undef M1_HFI_IQ_AUTH_X_GOOD
#define M1_HFI_IQ_AUTH_X_GOOD           (0.218f)
#undef M1_HFI_IQ_AUTH_X_BAD
#define M1_HFI_IQ_AUTH_X_BAD            (0.205f)
#undef M1_HFI_IQ_AUTH_EPS_FALSE_MAX
#define M1_HFI_IQ_AUTH_EPS_FALSE_MAX    (0.20f)
#undef M1_HFI_IQ_AUTH_HOLD_N
#define M1_HFI_IQ_AUTH_HOLD_N           2000u
#undef M1_HFI_IQ_AUTH_CLEAR_N
#define M1_HFI_IQ_AUTH_CLEAR_N          1000u
#undef M1_HFI_IQ_AUTH_IQ_LO
#define M1_HFI_IQ_AUTH_IQ_LO            (0.0f)
/* 141 冻结 AUTH：2 A / slew 4。无感签收后再抬。 */
#undef M1_HFI_IQ_AUTH_IQ_HI
#define M1_HFI_IQ_AUTH_IQ_HI            (2.0f)
#undef M1_HFI_IQ_AUTH_SLEW_A_S
#define M1_HFI_IQ_AUTH_SLEW_A_S         (4.0f)
#undef M1_HFI_IQ_AUTH_FEED_ENABLE
#define M1_HFI_IQ_AUTH_FEED_ENABLE      0
#undef M1_HFI_DEMOD_HP_ENABLE
#define M1_HFI_DEMOD_HP_ENABLE          1
#undef M1_HFI_DEMOD_HP_A
#define M1_HFI_DEMOD_HP_A               (0.05f)
#undef M1_HFI_DEMOD_SKIP_OUTLIER
#define M1_HFI_DEMOD_SKIP_OUTLIER       0
#undef M1_I_REF_ABS_MAX
#define M1_I_REF_ABS_MAX                11.0f
#undef M1_SPEED_IQ_REF_ABS_MAX
#define M1_SPEED_IQ_REF_ABS_MAX         11.0f
#undef M1_SPEED_PI_OUT_MAX
#define M1_SPEED_PI_OUT_MAX             M1_SPEED_IQ_REF_ABS_MAX
#undef M1_SPEED_PI_OUT_MIN
#define M1_SPEED_PI_OUT_MIN             (-M1_SPEED_IQ_REF_ABS_MAX)
#undef M1_SPEED_PI_INT_MAX
#define M1_SPEED_PI_INT_MAX             M1_SPEED_IQ_REF_ABS_MAX
#undef M1_SPEED_PI_INT_MIN
#define M1_SPEED_PI_INT_MIN             (-M1_SPEED_IQ_REF_ABS_MAX)
#undef M1_SPEED_PI_KI
#define M1_SPEED_PI_KI                  (6.5e-6f)
#undef M1_SPEED_PI_BETA
#define M1_SPEED_PI_BETA                (1.0f)
#undef M1_SPEED_PI_KP
#define M1_SPEED_PI_KP                  (0.005f)
/* 电压方程正转交不出 HFI，解调留在 foc。退回 SMO，PLL 吃 SMO 的 eαβ。 */
#undef M1_EMF_VEQ_ENABLE
#define M1_EMF_VEQ_ENABLE               0
#undef M1_EMF_SMO_ENABLE
#define M1_EMF_SMO_ENABLE               1
#undef M1_EMF_PLL_ENABLE
#define M1_EMF_PLL_ENABLE               1
#undef M1_EMF_PLL_USE_SMO
#define M1_EMF_PLL_USE_SMO              1
#undef M1_HFI_SMO_SUB_VH_ENABLE
#define M1_HFI_SMO_SUB_VH_ENABLE        1
#undef M1_EMF_SMO_K
#define M1_EMF_SMO_K                    20.0f
#undef M1_EMF_SMO_SAT_A
#define M1_EMF_SMO_SAT_A                0.30f
#undef M1_EMF_SMO_LPF_ENABLE
#define M1_EMF_SMO_LPF_ENABLE           1
#undef M1_EMF_SMO_LPF_HZ
#define M1_EMF_SMO_LPF_HZ               200.0f
#undef M1_EMF_SMO_LPF_SCHED_ENABLE
#define M1_EMF_SMO_LPF_SCHED_ENABLE     0
#undef M1_EMF_SMO_LPF_LINEAR_ENABLE
#define M1_EMF_SMO_LPF_LINEAR_ENABLE    1
#undef M1_EMF_SMO_LPF_LINEAR_K
#define M1_EMF_SMO_LPF_LINEAR_K         1.2f
#undef M1_EMF_SMO_LPF_LINEAR_FC_MIN
#define M1_EMF_SMO_LPF_LINEAR_FC_MIN    100.0f
/* 实验 E2（2026-10-24）：EMF 低通线性调度上限 280 → 420 Hz。
 * 原 280 Hz 处钳位，而 fe=233.3 Hz 正好是 2000 rpm（fc=1.2·fe 恰在此碰上限）；
 * 若真速比报告高或 2000 段 fe 已越 233，这项会把相角误差放大，改回 280 即复位。 */
#undef M1_EMF_SMO_LPF_LINEAR_FC_MAX
#define M1_EMF_SMO_LPF_LINEAR_FC_MAX    420.0f
#undef M1_EMF_SMO_THETA_OFF_RAD
#define M1_EMF_SMO_THETA_OFF_RAD        (0.0f)
#undef M1_EMF_PLL_FN_HZ
#define M1_EMF_PLL_FN_HZ                45.0f
#undef M1_EMF_PLL_ZETA
#define M1_EMF_PLL_ZETA                 0.707106781f
/* 实验 E1（2026-10-24）：EMF-PLL 的 ω 与积分限幅 2000 → 3000 rad/s。
 * 原 2000 rad/s = 2728 rpm 机械；若真速比 wfb 报告值高（角被封），
 * 改完 wfb 会跳到真速、速度环会主动刹车。改回 2000 即复位。 */
#undef M1_EMF_PLL_OMEGA_LIMIT_RAD_S
#define M1_EMF_PLL_OMEGA_LIMIT_RAD_S    3000.0f
#undef M1_EMF_PLL_INTEGRATOR_LIMIT_RAD_S
#define M1_EMF_PLL_INTEGRATOR_LIMIT_RAD_S 3000.0f
/* T1：去 |e| 归一化（无感段）；有感段已跳过 EMF-PLL。θerr/抖动退化则回滚为 1。 */
#undef M1_EMF_PLL_NORM_ENABLE
#define M1_EMF_PLL_NORM_ENABLE          0
#undef M1_EMF_LPF_PHASE_FF_ENABLE
#define M1_EMF_LPF_PHASE_FF_ENABLE      1
#undef M1_EMF_PLL_THETA_OFF_RAD
#define M1_EMF_PLL_THETA_OFF_RAD        (0.1147f)

#endif /* M1_HFI_GATE == 141 */

/* ===== 外环搬到 TIM6 2 kHz 中断（与 20 kHz FOC 内环剥离，降时序抖动） ===== */
#undef M1_OUTER_IN_TIM6
#define M1_OUTER_IN_TIM6                1   /* 1=外环在 TIM6 ISR；0=回退到 20 kHz 分频（旧行为） */

/* ===== 实验框架（宏 → 表）：编进固件以备验证，上电仍走原 HFI 141 路径 ===== */
#undef M1_EXP_FRAMEWORK_ENABLE
#define M1_EXP_FRAMEWORK_ENABLE         1   /* 表化实验已注册（exp_tables.c） */
#undef M1_EXP_BOOT_UD_CAL
#define M1_EXP_BOOT_UD_CAL              1   /* 阶段1 pilot：表驱动 UD_CAL 标定 */
#undef M1_EXP_BOOT_REV141
#define M1_EXP_BOOT_REV141              0   /* 阶段2 pilot（±2000 进表）暂时关 */

#endif /* CONFIG_PROFILES_M1_HFI_STANDSTILL_PROFILE_H */

