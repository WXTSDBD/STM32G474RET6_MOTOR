/**
 * @file bm_bode_off_only.h
 * @date 2026-10-28
 * @brief Bringup 模式配方片段（由 motor_params_m1.h 按 M1_BRINGUP_MODE 纳入）。
 */
#ifndef CONFIG_BM_BODE_OFF_ONLY_H
#define CONFIG_BM_BODE_OFF_ONLY_H

/** 堵转 Iq Bode：无 Pass0/阶跃/LUT；HOLD 2s 。OFF-lo 0.25A 。OFF-hi 1.25A；VOFA×6 @20kHz
 *  频表 57 。F1=2500 Hz（单。~18 s @T_obs_hi=100 ms，两。~38 s）；勿用 legacy F1=800（仅 38 。846 Hz。*/
#undef M1_ID_LOCK_CAL_SWEEP
#define M1_ID_LOCK_CAL_SWEEP            0
#undef M1_IDENT_ID_CAL_BEFORE_STEP
#define M1_IDENT_ID_CAL_BEFORE_STEP     0
#undef M1_IDENT_ENABLE
#define M1_IDENT_ENABLE                 1
#undef M1_IDENT_IQ_STEP_ENABLE
#define M1_IDENT_IQ_STEP_ENABLE         0
#undef M1_IDENT_IQ_BODE_ENABLE
#define M1_IDENT_IQ_BODE_ENABLE         1
#undef M1_IDENT_STEP_ROUNDS
#define M1_IDENT_STEP_ROUNDS            0u
#undef M1_IDENT_STEP_OFF_ROUNDS
#define M1_IDENT_STEP_OFF_ROUNDS        0u
#undef M1_IDENT_STEP_FIXED_ROUNDS
#define M1_IDENT_STEP_FIXED_ROUNDS      0u
#undef M1_RS_IDENT_ENABLE
#define M1_RS_IDENT_ENABLE              0
#undef M1_L_HFI_IDENT_ENABLE
#define M1_L_HFI_IDENT_ENABLE           0
#undef M1_SPEED_IDENT_ENABLE
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
#undef M1_IDENT_BODE_I_BIAS_HI_A
#define M1_IDENT_BODE_I_BIAS_HI_A       1.25f
#undef M1_IDENT_BODE_CYCLES_HI
#define M1_IDENT_BODE_CYCLES_HI         50.0f  /* T_OBS_HI 优先；此。fallback */
#undef M1_IDENT_BODE_T_OBS_HI_S
#define M1_IDENT_BODE_T_OBS_HI_S        0.1f
#undef M1_DEADBAND_LUT_BAKED_ENABLE
#define M1_DEADBAND_LUT_BAKED_ENABLE    0
#undef M1_DEADBAND_ENABLE
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


#endif /* CONFIG_BM_BODE_OFF_ONLY_H */
