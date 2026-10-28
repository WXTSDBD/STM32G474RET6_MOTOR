/**
 * @file m1_if_100rpm.profile.h
 * @date 2026-10-06
 * @brief I-f 爬到目标后再软切观测器。

 *
 * 只铺宏。方向用 M1_IF_DIR_SIGN。切实验改 bringup_active.h。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#ifndef CONFIG_PROFILES_M1_IF_100RPM_PROFILE_H
#define CONFIG_PROFILES_M1_IF_100RPM_PROFILE_H

#define M1_ID_LOCK_CAL_SWEEP            0
#define M1_IDENT_ENABLE                 0
#define M1_ID_CAL_ALIGN_ENABLE          0
#define M1_ID_CAL_COMMIT_LUT            0
#define M1_L_HFI_IDENT_ENABLE           0
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
#undef M1_IF_OBS_EW_CLAMP_ENABLE
#define M1_IF_OBS_EW_CLAMP_ENABLE       1 /* 仅挡 enc 尖峰，勿长期捂住真过速 */
/* 交接浅刹车：① 阶段保持；② 起解除 */
#undef M1_IF_OBS_SOFT_BRAKE_ENABLE
#define M1_IF_OBS_SOFT_BRAKE_ENABLE     1
#undef M1_IF_OBS_SOFT_REGEN_IQ_A
#define M1_IF_OBS_SOFT_REGEN_IQ_A       0.25f /* 浅刹 |regen|；驱动侧仍开 */
#undef M1_IF_OBS_IQ_MIN_A
#define M1_IF_OBS_IQ_MIN_A              (-M1_IF_OBS_SOFT_REGEN_IQ_A) /* 兼容旧名 */
/* 三步巡航（1347 禁止一步放开） */
#undef M1_IF_OBS_CRUISE_ENABLE
#define M1_IF_OBS_CRUISE_ENABLE         1
/* 正→不管停→反：关阶跃/探针，锁①；零速后再 I/F −1000 */
#undef M1_IF_OBS_DIR_SEQ_ENABLE
#define M1_IF_OBS_DIR_SEQ_ENABLE        1
/* 巡航后硬阶跃：DIR_SEQ 时关 */
/* ③ 探针：DIR_SEQ 时关 */
/* DIR_SEQ：正/反巡航都锁①（浅刹），勿进②/③ */
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
/* 切入后不回退 */
/* ARM：转速+emag 攒满即 BLEND（不再等 θ_if） */

#undef M1_OBS_SPD_PLL_ENABLE
#define M1_OBS_SPD_PLL_ENABLE           1

#undef M1_SPEED_PI_KP
#define M1_SPEED_PI_KP                  0.0008f /* 再弱：抑 ~1Hz 大摆 */
#undef M1_SPEED_PI_KI
#define M1_SPEED_PI_KI                  0.00005f

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

#undef M1_CLOSURE_BRINGUP
#define M1_CLOSURE_BRINGUP              0

#endif /* CONFIG_PROFILES_M1_IF_100RPM_PROFILE_H */
