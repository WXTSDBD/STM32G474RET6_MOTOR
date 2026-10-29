/**
 * @file bm_speed_ident.h
 * @date 2026-10-28
 * @brief Bringup 模式配方片段（由 motor_params_m1.h 按 M1_BRINGUP_MODE 纳入）。
 */
#ifndef CONFIG_BM_SPEED_IDENT_H
#define CONFIG_BM_SPEED_IDENT_H

#if M1_USE_HFI_STANDSTILL_PROFILE
/** HFI 静置旁路脚手架：。profiles/m1_hfi_standstill.profile.h */
#include "profiles/m1_hfi_standstill.profile.h"
#elif M1_USE_SENSED_POS_MIT_PROFILE
#include "profiles/m1_sensed_pos_mit.profile.h"
#elif M1_USE_SENSORLESS_POS_MIT_PROFILE
#include "profiles/m1_sensorless_pos_mit.profile.h"
#elif M1_USE_IF_100_PROFILE
/** 。I/F 拖到 100 rpm：见 profiles/m1_if_100rpm.profile.h */
#include "profiles/m1_if_100rpm.profile.h"
#elif M1_USE_SPEED_1000_PROFILE
/** 有感阶梯 + SMO→EMF-PLL 旁路（无。SMO）：。profiles/m1_speed_1000rpm.profile.h */
#include "profiles/m1_speed_1000rpm.profile.h"
#elif M1_USE_OBS_VEQ_PROFILE
/** 有感 1000 rpm + Veq 旁路观测：见 profiles/m1_obs_veq_1000rpm.profile.h */
#include "profiles/m1_obs_veq_1000rpm.profile.h"
#elif M1_USE_FLUX_ID_PROFILE
/** 有感 1000 rpm 磁链估：。profiles/m1_flux_id_1000rpm.profile.h */
#include "profiles/m1_flux_id_1000rpm.profile.h"
#else
/**
 * 7/5 1746 签收序列（联调报。§4.3）：
 *   HOLD 0.5s settle + 100。00 rpm。s）→ 500。00。00。00。00。00 。DONE @~29s
 * open_seq。20=HOLD  221..226=STEP 各相  230=BODE(。  239=DONE
 * ω_ref 斜坡 500 rpm/s 。7/8 0840 实验1（抑重载减。Mp）；轻载看硬阶跃可关 M1_SPEED_OMEGA_RAMP_ENABLE
 */
#define M1_ID_LOCK_CAL_SWEEP            0
#define M1_IDENT_ENABLE                 0
#define M1_ID_CAL_ALIGN_ENABLE          0
#define M1_ID_CAL_COMMIT_LUT            0
#define M1_L_HFI_IDENT_ENABLE           0
#define M1_RS_IDENT_ENABLE              0
#define M1_VOFA_IDENT_DUMP_ENABLE       0
#define M1_SPEED_IDENT_ENABLE           1
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
#define M1_SPEED_IDENT_STEP_DWELL_S     4.0f     /* 。300 。dwell。/5。。 s。*/
#define M1_SPEED_IDENT_STEP_ZERO_DWELL_S 3.0f   /* 。300 rpm dwell。/5 1746 签收。*/
/** HOLD 。PLL 重锁 + Iq=0 时长 [s]，抑制上。ch8 毛刺 / iq_ref 打满 */
#ifndef M1_SPEED_IDENT_PLL_SETTLE_S
#define M1_SPEED_IDENT_PLL_SETTLE_S     0.5f
#endif
#undef M1_VOFA_PLL_CH8_11
#define M1_VOFA_PLL_CH8_11              0
#endif /* M1_USE_SPEED_1000 / OBS_VEQ / FLUX_ID PROFILE */

#if M1_EMF_PLL_ENABLE && !(M1_EMF_VEQ_ENABLE || M1_EMF_SMO_ENABLE)
#error "M1_EMF_PLL_ENABLE requires Veq and/or SMO after profile include"
#endif
#if M1_EMF_PLL_USE_SMO && !M1_EMF_SMO_ENABLE
#error "M1_EMF_PLL_USE_SMO requires M1_EMF_SMO_ENABLE after profile include"
#endif
#if M1_EMF_PLL_ENABLE && !M1_EMF_PLL_USE_SMO && !M1_EMF_VEQ_ENABLE
#error "PLL on Veq requires M1_EMF_VEQ_ENABLE after profile include"
#endif
/**
 * 1=SMO 每拍 atan2 出 θ̂（对照/遥测）。
 * 须在 profile 之后：PLL 吃 eαβ 时默认关，省热路径 libm atan2。
 */
#ifndef M1_EMF_SMO_ATAN_ENABLE
#if M1_EMF_PLL_ENABLE && M1_EMF_PLL_USE_SMO
#define M1_EMF_SMO_ATAN_ENABLE          0
#else
#define M1_EMF_SMO_ATAN_ENABLE          1
#endif
#endif
/** 1=|e| 用 max+0.5·min；0=libm sqrtf。须在 profile 之后。 */
#ifndef M1_EMF_EMAG_FAST_ENABLE
#if M1_EMF_PLL_ENABLE || M1_EMF_SMO_ENABLE
#define M1_EMF_EMAG_FAST_ENABLE         1
#else
#define M1_EMF_EMAG_FAST_ENABLE         0
#endif
#endif
/**
 * 1=低速互斥：|ω| 低于 MUX_LO_RPM 时跳过 SMO/EMF-PLL（只跑 HFI）。
 * 须在 profile 之后。
 */
#if (M1_VOFA_OBS_PLL_12CH + M1_VOFA_OBS_SMO_12CH + M1_VOFA_OBS_VEQ_12CH + \
     M1_VOFA_OBS_SMO_RAW_12CH + M1_VOFA_HFI_12CH + M1_VOFA_IF_12CH) > 1
#error "Only one VOFA 12ch layout (OBS/IF/HFI) after profile include"
#endif
#if M1_USE_HFI_STANDSTILL_PROFILE && !M1_HFI_ENABLE
#error "M1_USE_HFI_STANDSTILL_PROFILE requires M1_HFI_ENABLE=1"
#endif
#ifndef M1_HFI_DEMOD_HP_A
#define M1_HFI_DEMOD_HP_A               (0.05f)
#endif
/* M1_HFI_PLL_VESC_ERR_ENABLE / M1_HFI_DEMOD_INJ_AXIS：无 .c 引用；profile 仍可 #define。 */
#ifndef M1_HFI_PLL_VESC_MAX_ERR
#define M1_HFI_PLL_VESC_MAX_ERR         (0.30f) /* 同 foc_hfi_max_err */
#endif
#ifndef M1_HFI_PLL_VESC_ERR_SIGN
#define M1_HFI_PLL_VESC_ERR_SIGN        (1.0f)
#endif
#ifndef M1_HFI_IQ_AUTH_ENABLE
#define M1_HFI_IQ_AUTH_ENABLE           0
#endif
#ifndef M1_HFI_IQ_AUTH_CLEAR_N
#define M1_HFI_IQ_AUTH_CLEAR_N          1u
#endif
#if M1_OBS_SOFT_SWITCH_ENABLE && !M1_EMF_PLL_ENABLE
#error "M1_OBS_SOFT_SWITCH_ENABLE requires M1_EMF_PLL_ENABLE after profile include"
#endif
#if M1_OBS_SPD_PLL_ENABLE && !M1_EMF_PLL_ENABLE
#error "M1_OBS_SPD_PLL_ENABLE requires M1_EMF_PLL_ENABLE after profile include"
#endif
#if M1_OBS_SPD_PLL_ENABLE && !M1_PLL_ENABLE
#error "M1_OBS_SPD_PLL_ENABLE requires M1_PLL_ENABLE (reuses motor_pll)"
#endif
#if M1_OBS_SS_SPEED_SWITCH_ENABLE && !M1_OBS_SOFT_SWITCH_ENABLE
#error "M1_OBS_SS_SPEED_SWITCH_ENABLE requires M1_OBS_SOFT_SWITCH_ENABLE"
#endif
#if M1_OBS_SS_SPEED_SWITCH_ENABLE && !M1_OBS_SPD_PLL_ENABLE
#error "M1_OBS_SS_SPEED_SWITCH_ENABLE requires M1_OBS_SPD_PLL_ENABLE"
#endif
#if M1_OBS_SS_SPD_DEFER_ENABLE && !M1_OBS_SS_SPEED_SWITCH_ENABLE
#error "M1_OBS_SS_SPD_DEFER_ENABLE requires M1_OBS_SS_SPEED_SWITCH_ENABLE"
#endif


#endif /* CONFIG_BM_SPEED_IDENT_H */
