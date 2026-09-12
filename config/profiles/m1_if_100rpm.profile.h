/**
 * @file m1_if_100rpm.profile.h
 * @brief I/F → SMO：通用 |ω| 路径；方向只改 M1_IF_DIR_SIGN（+1/−1）
 *
 * 时序：RAMP→|1000|；≥920→BLEND→OBS→①浅刹；
 * DIR_SEQ：正转① soak → Iq=0 滑行 → 近零 → 再 I/F 反转到 −1000。
 * Iq：I/F 与速度环交接均用 +|Iq|；DIR 只乘转速目标（勿在 BLEND 把 Iq 乘成负）。
 *
 * VOFA×12：
 *   ch0 ω_enc  ch1 ω_ref  ch2 Iq_ref  ch3 θ_err
 *   ch4 ω_obs  ch5 speed_fb  ch6 Iq  ch7 emag
 *   ch8 Uq  ch9 θ_park  ch10 spd_on(0→1)  ch11 ss_state+0.1α
 * open_seq：246=巡航①  230=滑行  231=近零待反起  232=反转腿结束
 */
#ifndef CONFIG_PROFILES_M1_IF_100RPM_PROFILE_H
#define CONFIG_PROFILES_M1_IF_100RPM_PROFILE_H

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

#undef M1_OPEN_UD_PRE_ID_CAL_ENABLE
#define M1_OPEN_UD_PRE_ID_CAL_ENABLE    0
#undef M1_OPEN_UQ_PRE_ID_CAL_ENABLE
#define M1_OPEN_UQ_PRE_ID_CAL_ENABLE    0

#undef M1_STARTUP_ENABLE
#define M1_STARTUP_ENABLE               0

#define M1_DEADBAND_ENABLE              0
#define M1_DEADBAND_LUT_BAKED_ENABLE    0
#define M1_DEADBAND_NVM_ON_BOOT         0

/* --- I/F：爬到 ±1000；软切从 920 起开门，爬升途中抓机会 --- */
#undef M1_IF_ENABLE
#define M1_IF_ENABLE                    1
#undef M1_IF_TO_OBS_ENABLE
#define M1_IF_TO_OBS_ENABLE             1
/* 编码器有则监督、无则照跑（控制不吃 ω_enc/θ_enc） */
#undef M1_ENC_OPTIONAL_ENABLE
#define M1_ENC_OPTIONAL_ENABLE          1
/* 方向唯一旋钮：+1 正转 / -1 反转（只乘 ω 目标；Iq 交接保持 +|Iq|）
 * DIR_SEQ 开时：先正后反，必须 +1；反转由序列运行时改 motor_if 目标。 */
#undef M1_IF_DIR_SIGN
#define M1_IF_DIR_SIGN                  (1.0f)
#undef M1_IF_ALIGN_S
#define M1_IF_ALIGN_S                   0.6f /* 中速丢步：对齐做实再爬 */
#undef M1_IF_IQ_A
#define M1_IF_IQ_A                      4.0f /* I/F |Iq|；速度环再 × DIR */
#undef M1_IF_ID_A
#define M1_IF_ID_A                      0.0f
#undef M1_IF_TARGET_RPM
#define M1_IF_TARGET_RPM                (M1_IF_DIR_SIGN * 1000.0f)
#undef M1_IF_RAMP_S
#define M1_IF_RAMP_S                    12.0f /* 原 40s 录包头太长；≈83 rpm/s */
/* 200–500 rpm 易丢步：中速带抬 Iq（按 |ω|）；缩短斜坡后更需要 */
#undef M1_IF_IQ_MID_BOOST_ENABLE
#define M1_IF_IQ_MID_BOOST_ENABLE       1
#undef M1_IF_IQ_MID_A
#define M1_IF_IQ_MID_A                  5.5f
#undef M1_IF_IQ_MID_RPM_LO
#define M1_IF_IQ_MID_RPM_LO             180.0f
#undef M1_IF_IQ_MID_RPM_HI
#define M1_IF_IQ_MID_RPM_HI             520.0f
#undef M1_IF_HOLD_S
#define M1_IF_HOLD_S                    0.0f
#undef M1_IF_OBS_SPEED_REF_RPM
#define M1_IF_OBS_SPEED_REF_RPM         (M1_IF_DIR_SIGN * 1000.0f) /* HOLD_IF_CMD 钉 ω_IF */
#undef M1_IF_OBS_ANGLE_ONLY_ENABLE
#define M1_IF_OBS_ANGLE_ONLY_ENABLE     0 /* 开弱速度环 */
#undef M1_IF_OBS_HOLD_SPEED_ENABLE
#define M1_IF_OBS_HOLD_SPEED_ENABLE     1
#undef M1_IF_OBS_HOLD_IF_CMD_ENABLE
#define M1_IF_OBS_HOLD_IF_CMD_ENABLE    1 /* ω_ref 钉 I/F 指令，不认飞车测速 */
#undef M1_IF_HANDOFF_IQ_A
#define M1_IF_HANDOFF_IQ_A              4.0f  /* |Iq|；速度环交接 × DIR */
#undef M1_IF_OBS_BLEND_SPEED_ENABLE
#define M1_IF_OBS_BLEND_SPEED_ENABLE    1 /* BLEND 起弱速度环，抑融角飞车 */
#undef M1_IF_OBS_BLEND_KEEP_IF_IQ
#define M1_IF_OBS_BLEND_KEEP_IF_IQ      0 /* 与 BLEND_SPEED 互斥：Iq 交给外环 */
#undef M1_IF_OBS_EW_CLAMP_ENABLE
#define M1_IF_OBS_EW_CLAMP_ENABLE       1 /* 仅挡 enc 尖峰，勿长期捂住真过速 */
#undef M1_IF_OBS_EW_CLAMP_RPM
#define M1_IF_OBS_EW_CLAMP_RPM          80.0f
#undef M1_IF_OBS_EW_CLAMP_S
#define M1_IF_OBS_EW_CLAMP_S            0.08f /* 短窗；过速交给弱 PI + 浅刹车 */
/* 交接浅刹车：① 阶段保持；② 起解除 */
#undef M1_IF_OBS_SOFT_BRAKE_ENABLE
#define M1_IF_OBS_SOFT_BRAKE_ENABLE     1
#undef M1_IF_OBS_SOFT_REGEN_IQ_A
#define M1_IF_OBS_SOFT_REGEN_IQ_A       0.25f /* 浅刹 |regen|；驱动侧仍开 */
#undef M1_IF_OBS_IQ_MIN_A
#define M1_IF_OBS_IQ_MIN_A              (-M1_IF_OBS_SOFT_REGEN_IQ_A) /* 兼容旧名 */
#undef M1_IF_OBS_SOFT_BRAKE_S
#define M1_IF_OBS_SOFT_BRAKE_S          12.0f /* 兜底；① 会续期 */
/* 三步巡航（1347 禁止一步放开） */
#undef M1_IF_OBS_CRUISE_ENABLE
#define M1_IF_OBS_CRUISE_ENABLE         1
#undef M1_IF_OBS_CRUISE_SETTLE_S
#define M1_IF_OBS_CRUISE_SETTLE_S       1.5f /* spd_on → ① */
#undef M1_IF_OBS_CRUISE_STAGE1_S
#define M1_IF_OBS_CRUISE_STAGE1_S       2.5f /* ① 只改 ω_ref */
#undef M1_IF_OBS_CRUISE_STAGE2_S
#define M1_IF_OBS_CRUISE_STAGE2_S       1.5f /* ② 对称 Iq */
#undef M1_IF_OBS_CRUISE_RPM
#define M1_IF_OBS_CRUISE_RPM            (M1_IF_DIR_SIGN * 1000.0f)
/* 正→不管停→反：关阶跃/探针，锁①；零速后再 I/F −1000 */
#undef M1_IF_OBS_DIR_SEQ_ENABLE
#define M1_IF_OBS_DIR_SEQ_ENABLE        1
#undef M1_IF_OBS_DIR_SEQ_FWD_HOLD_S
#define M1_IF_OBS_DIR_SEQ_FWD_HOLD_S    3.0f
#undef M1_IF_OBS_DIR_SEQ_ZERO_RPM
#define M1_IF_OBS_DIR_SEQ_ZERO_RPM      80.0f
#undef M1_IF_OBS_DIR_SEQ_ZERO_HOLD_S
#define M1_IF_OBS_DIR_SEQ_ZERO_HOLD_S   0.5f
#undef M1_IF_OBS_DIR_SEQ_COAST_MIN_S
#define M1_IF_OBS_DIR_SEQ_COAST_MIN_S   2.0f
#undef M1_IF_OBS_DIR_SEQ_COAST_MAX_S
#define M1_IF_OBS_DIR_SEQ_COAST_MAX_S   8.0f
#undef M1_IF_OBS_DIR_SEQ_EMAG_STOP
#define M1_IF_OBS_DIR_SEQ_EMAG_STOP     0.55f
#undef M1_IF_OBS_DIR_SEQ_REV_HOLD_S
#define M1_IF_OBS_DIR_SEQ_REV_HOLD_S    3.0f
/* 巡航后硬阶跃：DIR_SEQ 时关 */
#undef M1_IF_OBS_CRUISE_STEP_ENABLE
#define M1_IF_OBS_CRUISE_STEP_ENABLE    0
#undef M1_IF_OBS_CRUISE_STEP_SOAK_S
#define M1_IF_OBS_CRUISE_STEP_SOAK_S    2.0f
#undef M1_IF_OBS_CRUISE_STEP_HOLD_S
#define M1_IF_OBS_CRUISE_STEP_HOLD_S    4.0f
#undef M1_IF_OBS_CRUISE_STEP_LO_RPM
#define M1_IF_OBS_CRUISE_STEP_LO_RPM    (M1_IF_DIR_SIGN * 900.0f)
#undef M1_IF_OBS_CRUISE_STEP_HI_RPM
#define M1_IF_OBS_CRUISE_STEP_HI_RPM    (M1_IF_DIR_SIGN * 1300.0f)
/* ③ 探针：DIR_SEQ 时关 */
#undef M1_IF_OBS_CRUISE_S3_PROBE_ENABLE
#define M1_IF_OBS_CRUISE_S3_PROBE_ENABLE 0
#undef M1_IF_OBS_CRUISE_S3_PROBE_SOAK_S
#define M1_IF_OBS_CRUISE_S3_PROBE_SOAK_S 1.5f
#undef M1_IF_OBS_CRUISE_S3_PROBE_HOLD_S
#define M1_IF_OBS_CRUISE_S3_PROBE_HOLD_S 4.0f /* 与大阶跃表同 hold */
#undef M1_IF_OBS_CRUISE_PI_KP
#define M1_IF_OBS_CRUISE_PI_KP          0.008f
#undef M1_IF_OBS_CRUISE_PI_KI
#define M1_IF_OBS_CRUISE_PI_KI          0.001f
#undef M1_IF_OBS_CRUISE_IQ_ABS_MAX
#define M1_IF_OBS_CRUISE_IQ_ABS_MAX     8.0f
/* DIR_SEQ：正/反巡航都锁①（浅刹），勿进②/③ */
#undef M1_IF_OBS_CRUISE_LOCK_STAGE1_ENABLE
#define M1_IF_OBS_CRUISE_LOCK_STAGE1_ENABLE 1
#undef M1_IF_OBS_CRUISE_LOCK_STAGE2_ENABLE
#define M1_IF_OBS_CRUISE_LOCK_STAGE2_ENABLE 0
#undef M1_IF_OBS_CRUISE_LOCK_STAGE3_ENABLE
#define M1_IF_OBS_CRUISE_LOCK_STAGE3_ENABLE 1
#undef M1_IF_OBS_CRUISE_GATE_ENABLE
#define M1_IF_OBS_CRUISE_GATE_ENABLE    1
#undef M1_IF_OBS_CRUISE_GATE_T_MIN_S
#define M1_IF_OBS_CRUISE_GATE_T_MIN_S   2.0f
#undef M1_IF_OBS_CRUISE_GATE_T_MIN2_S
#define M1_IF_OBS_CRUISE_GATE_T_MIN2_S  1.5f
#undef M1_IF_OBS_CRUISE_GATE_HOLD_S
#define M1_IF_OBS_CRUISE_GATE_HOLD_S    1.0f /* 离线：连续 ≥1s */
#undef M1_IF_OBS_CRUISE_GATE_ERR_RPM
#define M1_IF_OBS_CRUISE_GATE_ERR_RPM   60.0f
#undef M1_IF_OBS_CRUISE_GATE_DOMEGA_MAX
#define M1_IF_OBS_CRUISE_GATE_DOMEGA_MAX 1200.0f
#undef M1_IF_OBS_CRUISE_GATE_DOMEGA_WIN_S
#define M1_IF_OBS_CRUISE_GATE_DOMEGA_WIN_S 0.10f
#undef M1_IF_OBS_CRUISE_GATE_EW_LPF_HZ
#define M1_IF_OBS_CRUISE_GATE_EW_LPF_HZ  20.0f
#undef M1_IF_OBS_CRUISE_GATE_FAIL_LEAK
#define M1_IF_OBS_CRUISE_GATE_FAIL_LEAK  2.0f
#undef M1_IF_OBS_CRUISE_AMP_GATE_ENABLE
#define M1_IF_OBS_CRUISE_AMP_GATE_ENABLE 1
#undef M1_IF_OBS_CRUISE_AMP_STD_MAX
#define M1_IF_OBS_CRUISE_AMP_STD_MAX     38.0f
#undef M1_IF_OBS_CRUISE_AMP_PTP_MAX
#define M1_IF_OBS_CRUISE_AMP_PTP_MAX     98.0f
#undef M1_IF_OBS_CRUISE_AMP_EW_MAX
#define M1_IF_OBS_CRUISE_AMP_EW_MAX      58.0f
#undef M1_IF_OBS_CRUISE_AMP_EMA_HZ
#define M1_IF_OBS_CRUISE_AMP_EMA_HZ      2.0f
#undef M1_IF_OBS_CRUISE_IQ_MIN_SLEW_A_S
#define M1_IF_OBS_CRUISE_IQ_MIN_SLEW_A_S 0.15f /* |regen| 斜坡 0.6→0.9 ≈2s */
#undef M1_IF_OBS_CRUISE_DRIVE_IQ_A
#define M1_IF_OBS_CRUISE_DRIVE_IQ_A     3.5f /* 驱动 |Iq|；符号 = DIR */
#undef M1_IF_OBS_CRUISE_STAGE2_REGEN_IQ_A
#define M1_IF_OBS_CRUISE_STAGE2_REGEN_IQ_A 0.6f
#undef M1_IF_OBS_CRUISE_STAGE3_REGEN_IQ_A
#define M1_IF_OBS_CRUISE_STAGE3_REGEN_IQ_A 0.9f
#undef M1_IF_OBS_DAMP_ENABLE
#define M1_IF_OBS_DAMP_ENABLE           1
#undef M1_IF_OBS_DAMP_HP_ENABLE
#define M1_IF_OBS_DAMP_HP_ENABLE        1
#undef M1_IF_OBS_DAMP_BD
#define M1_IF_OBS_DAMP_BD               0.003f /* 1652：0.002→0.003，抑 −0.9 极限环 */
#undef M1_IF_OBS_DAMP_LPF_HZ
#define M1_IF_OBS_DAMP_LPF_HZ           0.35f
/* 交接后 ω 斜坡：巡航时 200 rpm/s 爬向 1000 */
#undef M1_SPEED_OMEGA_RAMP_ENABLE
#define M1_SPEED_OMEGA_RAMP_ENABLE      1
#undef M1_SPEED_OMEGA_RAMP_RPM_S
#define M1_SPEED_OMEGA_RAMP_RPM_S       200.0f
#undef M1_SPEED_IQ_SLEW_ENABLE
#define M1_SPEED_IQ_SLEW_ENABLE         1
#undef M1_SPEED_IQ_SLEW_A_PER_S
#define M1_SPEED_IQ_SLEW_A_PER_S        12.0f /* BLEND 减流；巡航仍可用 */
#undef M1_SPEED_IQ_REF_ABS_MAX
#define M1_SPEED_IQ_REF_ABS_MAX         3.5f /* 仅交接初值；巡航改 pi 限幅 */
#undef M1_SPEED_PI_OUT_MAX
#define M1_SPEED_PI_OUT_MAX             M1_SPEED_IQ_REF_ABS_MAX
#undef M1_SPEED_PI_OUT_MIN
#define M1_SPEED_PI_OUT_MIN             (-M1_SPEED_IQ_REF_ABS_MAX)
#undef M1_SPEED_PI_INT_MAX
#define M1_SPEED_PI_INT_MAX             M1_SPEED_IQ_REF_ABS_MAX
#undef M1_SPEED_PI_INT_MIN
#define M1_SPEED_PI_INT_MIN             (-M1_SPEED_IQ_REF_ABS_MAX)

/* --- SMO → EMF-PLL + 软切（同 speed_1000 签收配方） --- */
#undef M1_EMF_VEQ_ENABLE
#define M1_EMF_VEQ_ENABLE               0
#undef M1_EMF_SMO_ENABLE
#define M1_EMF_SMO_ENABLE               1
#undef M1_EMF_PLL_ENABLE
#define M1_EMF_PLL_ENABLE               1
#undef M1_EMF_PLL_USE_SMO
#define M1_EMF_PLL_USE_SMO              1

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
#undef M1_EMF_SMO_LPF_LINEAR_FC_MAX
#define M1_EMF_SMO_LPF_LINEAR_FC_MAX    280.0f
#undef M1_EMF_SMO_THETA_OFF_RAD
#define M1_EMF_SMO_THETA_OFF_RAD        (0.0f)

#undef M1_EMF_PLL_FN_HZ
#define M1_EMF_PLL_FN_HZ                45.0f
#undef M1_EMF_PLL_ZETA
#define M1_EMF_PLL_ZETA                 0.707106781f
#undef M1_EMF_PLL_NORM_ENABLE
#define M1_EMF_PLL_NORM_ENABLE          1
#undef M1_EMF_LPF_PHASE_FF_ENABLE
#define M1_EMF_LPF_PHASE_FF_ENABLE      1
#undef M1_EMF_PLL_THETA_OFF_RAD
#define M1_EMF_PLL_THETA_OFF_RAD        (0.1147f)

#undef M1_OBS_SOFT_SWITCH_ENABLE
#define M1_OBS_SOFT_SWITCH_ENABLE       1
/* 先角后速：OBS 站稳再切 ω̂（避免 BLEND 过冲时硬切速） */
#undef M1_OBS_SS_SPEED_SWITCH_ENABLE
#define M1_OBS_SS_SPEED_SWITCH_ENABLE   1
#undef M1_OBS_SS_SPD_DEFER_ENABLE
#define M1_OBS_SS_SPD_DEFER_ENABLE      1
#undef M1_OBS_SS_SPD_DWELL_S
#define M1_OBS_SS_SPD_DWELL_S           2.0f /* 进 OBS 后再等 2s */
#undef M1_OBS_SS_SPD_HOLD_S
#define M1_OBS_SS_SPD_HOLD_S            0.25f /* 1330：10% 门限在晃速下凑不满 0.5s */
#undef M1_OBS_SS_SPD_RPM_ERR_FRAC
#define M1_OBS_SS_SPD_RPM_ERR_FRAC      0.20f /* |ω̂−ω_ref|；空载残余摆动约 ±150 */
#undef M1_OBS_SS_SPD_DOMEGA_MAX
#define M1_OBS_SS_SPD_DOMEGA_MAX        2500.0f /* 100ms 窗真加速度量级 */
#undef M1_OBS_SS_SPD_DOMEGA_WIN_S
#define M1_OBS_SS_SPD_DOMEGA_WIN_S      0.10f
#undef M1_OBS_SS_SPD_ENC_MATCH_ENABLE
#define M1_OBS_SS_SPD_ENC_MATCH_ENABLE  0 /* 门限纯无感：不看 ω_enc */
/* 切入后不回退 */
#undef M1_OBS_SS_FALLBACK_ENABLE
#define M1_OBS_SS_FALLBACK_ENABLE       0
#undef M1_OBS_SS_FALLBACK_ON_ERR_ENABLE
#define M1_OBS_SS_FALLBACK_ON_ERR_ENABLE 0
#undef M1_OBS_SS_RPM_ENTER
#define M1_OBS_SS_RPM_ENTER             920.0f /* ≥920 即可武装；I/F 仍在爬向 1000 */
#undef M1_OBS_SS_RPM_EXIT
#define M1_OBS_SS_RPM_EXIT              350.0f
#undef M1_OBS_SS_ERR_ENTER_RAD
#define M1_OBS_SS_ERR_ENTER_RAD         0.34906585f /* 有感路径仍用；I/F 切角已不依赖 */
#undef M1_OBS_SS_ERR_EXIT_RAD
#define M1_OBS_SS_ERR_EXIT_RAD          0.5235988f
#undef M1_OBS_SS_EMAG_MIN
#define M1_OBS_SS_EMAG_MIN              0.9f
#undef M1_OBS_SS_IQ_ABS_MAX
#define M1_OBS_SS_IQ_ABS_MAX            12.0f
/* ARM：转速+emag 攒满即 BLEND（不再等 θ_if） */
#undef M1_OBS_SS_ARM_S
#define M1_OBS_SS_ARM_S                 0.20f
#undef M1_OBS_SS_ARM_GRACE_S
#define M1_OBS_SS_ARM_GRACE_S           0.10f
#undef M1_OBS_SS_BLEND_S
#define M1_OBS_SS_BLEND_S               0.20f

#undef M1_OBS_SPD_PLL_ENABLE
#define M1_OBS_SPD_PLL_ENABLE           1
#undef M1_OBS_SPD_PLL_FN_HZ
#define M1_OBS_SPD_PLL_FN_HZ            12.0f
#undef M1_OBS_SPD_PLL_ZETA
#define M1_OBS_SPD_PLL_ZETA             0.707106781f
#undef M1_OBS_SPD_FB_LPF_HZ
#define M1_OBS_SPD_FB_LPF_HZ            25

#undef M1_SPEED_PI_KP
#define M1_SPEED_PI_KP                  0.0008f /* 再弱：抑 ~1Hz 大摆 */
#undef M1_SPEED_PI_KI
#define M1_SPEED_PI_KI                  0.00005f
#undef M1_OBS_THETA_NOTCH_ENABLE
#define M1_OBS_THETA_NOTCH_ENABLE       0

#undef M1_VOFA_UNIFIED_12CH
#define M1_VOFA_UNIFIED_12CH            1
#undef M1_TELEM_BRINGUP_K
#define M1_TELEM_BRINGUP_K              12u
#undef M1_TELEM_BRINGUP_DECIMATION
#define M1_TELEM_BRINGUP_DECIMATION     2u
#undef M1_VOFA_IF_12CH
#define M1_VOFA_IF_12CH                 1
#undef M1_VOFA_OBS_SMO_12CH
#define M1_VOFA_OBS_SMO_12CH            0
#undef M1_VOFA_OBS_PLL_12CH
#define M1_VOFA_OBS_PLL_12CH            0
#undef M1_VOFA_OBS_VEQ_12CH
#define M1_VOFA_OBS_VEQ_12CH            0
#undef M1_VOFA_FLUX_ID_6CH
#define M1_VOFA_FLUX_ID_6CH             0

#undef M1_CLOSURE_BRINGUP
#define M1_CLOSURE_BRINGUP              0

#endif /* CONFIG_PROFILES_M1_IF_100RPM_PROFILE_H */
