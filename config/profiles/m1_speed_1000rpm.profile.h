/**
 * @file m1_speed_1000rpm.profile.h
 * @brief 直达 1000 rpm：有感爬升 → 软切角+速到 SMO（θ̂ + θ̂→motor_pll@2kHz）
 *
 * 时序：
 *   HOLD 6 s：100→1000；速度=编码器 PLL
 *   STEP：1000 软切 OBS 后，每 10 s −50 rpm 降至 400（下限探底）
 *   编码器仅监督：角差超限不回切（soak）
 *
 * VOFA×12：
 *   ch0 speed_fb  ch1 omega_ref  ch2 iq_ref  ch3 err_pll
 *   ch4 theta_park  ch5 obs_spd_pll  ch6 emag  ch7 theta_hat
 *   ch8 isr  ch9 foc  ch10 obs
 *   ch11 = ss_state + 0.1*alpha（3=OBS 角+速无感）
 */
#ifndef CONFIG_PROFILES_M1_SPEED_1000RPM_PROFILE_H
#define CONFIG_PROFILES_M1_SPEED_1000RPM_PROFILE_H

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

#define M1_SPEED_IDENT_HOLD_S           6.0f
#ifndef M1_SPEED_IDENT_RPM_START
#define M1_SPEED_IDENT_RPM_START        100.0f
#endif
/* HOLD→1000 软切；随后 1000→400，−50 rpm / 10 s */
#define M1_SPEED_IDENT_STEP_ROUNDS      1u
#define M1_SPEED_IDENT_STEP_RPM0        1000.0f
#define M1_SPEED_IDENT_STEP_RPM1        1000.0f
#define M1_SPEED_IDENT_STEP_RPM2        1000.0f
#define M1_SPEED_IDENT_STEP_RPM3        1000.0f
#define M1_SPEED_IDENT_STEP_RPM4        1000.0f
#define M1_SPEED_IDENT_STEP_RPM5        1000.0f
#define M1_SPEED_IDENT_STEP_RPM6        1000.0f
#define M1_SPEED_IDENT_STEP_DWELL_S     10.0f
#define M1_SPEED_IDENT_STEP_ZERO_DWELL_S 5.0f
#undef M1_SPEED_IDENT_OBS_FLOOR_PROBE
#define M1_SPEED_IDENT_OBS_FLOOR_PROBE  1
#undef M1_SPEED_IDENT_STEP_LADDER_ENABLE
#define M1_SPEED_IDENT_STEP_LADDER_ENABLE  1

#ifndef M1_SPEED_IDENT_PLL_SETTLE_S
#define M1_SPEED_IDENT_PLL_SETTLE_S     1.5f
#endif

#undef M1_VOFA_PLL_CH8_11
#define M1_VOFA_PLL_CH8_11              0
#undef M1_VOFA_SPEED_CH8_11
#define M1_VOFA_SPEED_CH8_11            0

#undef M1_VOFA_UNIFIED_12CH
#define M1_VOFA_UNIFIED_12CH            1
#undef M1_TELEM_BRINGUP_K
#define M1_TELEM_BRINGUP_K              12u
#undef M1_TELEM_BRINGUP_DECIMATION
#define M1_TELEM_BRINGUP_DECIMATION     2u
#undef M1_VOFA_FLUX_ID_6CH
#define M1_VOFA_FLUX_ID_6CH             0
#undef M1_VOFA_CH11_ENC_RAW
#define M1_VOFA_CH11_ENC_RAW            0

#undef M1_EMF_VEQ_ENABLE
#define M1_EMF_VEQ_ENABLE               0
#undef M1_EMF_SMO_ENABLE
#define M1_EMF_SMO_ENABLE               1
#undef M1_EMF_PLL_ENABLE
#define M1_EMF_PLL_ENABLE               1
#undef M1_EMF_PLL_USE_SMO
#define M1_EMF_PLL_USE_SMO              1
#undef M1_VOFA_OBS_VEQ_12CH
#define M1_VOFA_OBS_VEQ_12CH            0
#undef M1_VOFA_OBS_SMO_12CH
#define M1_VOFA_OBS_SMO_12CH            0
#undef M1_VOFA_OBS_SMO_RAW_12CH
#define M1_VOFA_OBS_SMO_RAW_12CH        0
#undef M1_VOFA_OBS_PLL_12CH
#define M1_VOFA_OBS_PLL_12CH            1

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
/* LPF 相位前馈 −atan(fe/fc)；常值 OFF 为有感残差（1526 离线：FF 后 mean(err)≈+6.57°） */
#undef M1_EMF_LPF_PHASE_FF_ENABLE
#define M1_EMF_LPF_PHASE_FF_ENABLE      1
#undef M1_EMF_PLL_THETA_OFF_RAD
#define M1_EMF_PLL_THETA_OFF_RAD        (0.1147f)

/* --- 软切：角 + 速度 --- */
#undef M1_OBS_SOFT_SWITCH_ENABLE
#define M1_OBS_SOFT_SWITCH_ENABLE       1
#undef M1_OBS_SS_SPEED_SWITCH_ENABLE
#define M1_OBS_SS_SPEED_SWITCH_ENABLE   1
/* 编码器监督：角差只上 VOFA，不因超差回切（过流/掉速仍可踢） */
#undef M1_OBS_SS_FALLBACK_ON_ERR_ENABLE
#define M1_OBS_SS_FALLBACK_ON_ERR_ENABLE 0
#undef M1_OBS_SS_RPM_ENTER
#define M1_OBS_SS_RPM_ENTER             900.0f
#undef M1_OBS_SS_RPM_EXIT
#define M1_OBS_SS_RPM_EXIT              350.0f /* 低于探底末档 400，避免未测完就掉速踢 */
#undef M1_OBS_SS_ERR_ENTER_RAD
#define M1_OBS_SS_ERR_ENTER_RAD         0.2617994f /* 15° @1000 切入 */
#undef M1_OBS_SS_ERR_EXIT_RAD
#define M1_OBS_SS_ERR_EXIT_RAD          0.5235988f
#undef M1_OBS_SS_EMAG_MIN
#define M1_OBS_SS_EMAG_MIN              0.8f
#undef M1_OBS_SS_IQ_ABS_MAX
#define M1_OBS_SS_IQ_ABS_MAX            10.0f
#undef M1_OBS_SS_ARM_S
#define M1_OBS_SS_ARM_S                 1.0f
#undef M1_OBS_SS_ARM_GRACE_S
#define M1_OBS_SS_ARM_GRACE_S           0.05f
#undef M1_OBS_SS_BLEND_S
#define M1_OBS_SS_BLEND_S               0.20f

/* θ̂→motor_pll @ 速度环节拍；离线扫参建议 Fn=12 */
#undef M1_OBS_SPD_PLL_ENABLE
#define M1_OBS_SPD_PLL_ENABLE           1
#undef M1_OBS_SPD_PLL_FN_HZ
#define M1_OBS_SPD_PLL_FN_HZ            12.0f
#undef M1_OBS_SPD_PLL_ZETA
#define M1_OBS_SPD_PLL_ZETA             0.707106781f
/* 观测速进 PI 前 LPF：回 25 Hz（15 Hz 试验更差：~8.5 Hz 狩猎） */
#undef M1_OBS_SPD_FB_LPF_HZ
#define M1_OBS_SPD_FB_LPF_HZ            25

/* 速度环 KI：探底用基线 0.002（异响试验的 ×0.6 先关掉） */
#undef M1_SPEED_PI_KI
#define M1_SPEED_PI_KI                  0.002f

#undef M1_CLOSURE_BRINGUP
#define M1_CLOSURE_BRINGUP              0

#endif /* CONFIG_PROFILES_M1_SPEED_1000RPM_PROFILE_H */
