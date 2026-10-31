/**
 * @file defaults_fold.h
 * @date 2026-10-31
 * @brief if (M1_*) 折叠用硬默认；配方未开时名字仍在。
 * @note 在 defaults_obs 之后、recipe 之前纳入；recipe 用 #undef+#define 覆盖。
 */
#ifndef CONFIG_DEFAULTS_FOLD_H
#define CONFIG_DEFAULTS_FOLD_H

/** VOFA JustFloat 通道数与分频（JEOC 20 kHz 基准）。 */
#define M1_TELEM_BRINGUP_K              12u
#define M1_TELEM_BRINGUP_DECIMATION     2u
#define M1_VOFA_UNIFIED_12CH            1
#define M1_VOFA_IDENT_DUTY_12CH         0

/** PI 带宽（Hz）；M1_PI_USE_FIXED_GAIN=0 时用。 */
#define M1_PI_FC_HZ                     1000.0f

/** 首次闭环联调限幅；速度环交付配方应 #undef 为 0。 */
#define M1_CLOSURE_BRINGUP              1

#define M1_IF_ENABLE                    0
#define M1_IF_IQ_A                      0.5f

#define M1_IDENT_OVERRIDE_LIMITS        0
#define M1_IDENT_ID_CAL_BEFORE_STEP     0
#define M1_IDENT_STEP_I0_A              0.0f

#define M1_ID_CAL_FIX_THETA_ENABLE      0
#define M1_ID_CAL_ALIGN_ENABLE          0
#define M1_ID_CAL_IQ_PROBE_ID_PI_ENABLE 0
#define M1_ID_CAL_ALIGN_UD_V            0.0f
#define M1_ID_CAL_IQ_REF_A              0.0f
#define M1_L_HFI_IDENT_ENABLE           0

#endif /* CONFIG_DEFAULTS_FOLD_H */
