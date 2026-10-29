/**
 * @file defaults_id_cal.h
 * @date 2026-10-28
 * @brief Id 锁轴标定默认宏（仅 M1_ID_LOCK_CAL_SWEEP=1 时由 motor_params 纳入）。
 * @note 普通 #define；bringup_modes / profile 用 #undef+#define 覆盖。
 *       PASS0_RS：本头在 bm_pass0_rs 之前纳入。
 */
#ifndef CONFIG_DEFAULTS_ID_CAL_H
#define CONFIG_DEFAULTS_ID_CAL_H

/** 标定态临时放宽 I_ref / PI 限幅（Rs/l_hfi 仍用）。 */
#define M1_ID_CAL_OVERRIDE_LIMITS  1

#if M1_ID_CAL_OVERRIDE_LIMITS
#define M1_ID_CAL_I_REF_ABS_MAX    3.0f
/** 与电流环一致（M1_CLOSURE_BRINGUP）。 */
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
/** 上电 Id=0 稳定（s）；ALIGN→HOLD 仍读。 */
#define M1_ID_CAL_INIT_HOLD_S       0.5f

/*
 * 建表死区补偿已下线（2026-10-28）：产品只保留 FIXED（M1_DEADTIME_NS）。
 * 原 Pass0/geo/baked/Iq探路 源码在 `_scratch/archive_deadband_lut/`。
 * 下列宏一律闸死（bm 不得再打开）。
 */
#undef M1_ID_CAL_COMMIT_LUT
#define M1_ID_CAL_COMMIT_LUT                0
#undef M1_ID_CAL_LUT_VERIFY_SWEEP
#define M1_ID_CAL_LUT_VERIFY_SWEEP          0
#undef M1_ID_CAL_PASS0_ONLY_ENABLE
#define M1_ID_CAL_PASS0_ONLY_ENABLE         0
#undef M1_ID_CAL_IQ_PROBE_ENABLE
#define M1_ID_CAL_IQ_PROBE_ENABLE           0
#undef M1_ID_CAL_IQ_PROBE_ONLY_ENABLE
#define M1_ID_CAL_IQ_PROBE_ONLY_ENABLE      0
#undef M1_ID_CAL_IQ_PROBE_ID_PI_ENABLE
#define M1_ID_CAL_IQ_PROBE_ID_PI_ENABLE     0
#undef M1_ID_CAL_PASS1_USE_APPLY_DUTY
#define M1_ID_CAL_PASS1_USE_APPLY_DUTY      0
#undef M1_ID_CAL_DUAL_ANGLE_ENABLE
#define M1_ID_CAL_DUAL_ANGLE_ENABLE         0
#undef M1_ID_CAL_MULTI_ANGLE_ENABLE
#define M1_ID_CAL_MULTI_ANGLE_ENABLE        0
#undef M1_DEADBAND_LUT_BAKED_ENABLE
#define M1_DEADBAND_LUT_BAKED_ENABLE        0
#undef M1_DEADBAND_GEO_TWO_CLUSTER_ENABLE
#define M1_DEADBAND_GEO_TWO_CLUSTER_ENABLE  0
#undef M1_DEADBAND_GEO_BUILD_ENABLE
#define M1_DEADBAND_GEO_BUILD_ENABLE        0
#undef M1_DEADBAND_GEO_MERGE_VAL30_ONLY
#define M1_DEADBAND_GEO_MERGE_VAL30_ONLY    0
#undef M1_DEADBAND_GEO_DIFF_LOG_ENABLE
#define M1_DEADBAND_GEO_DIFF_LOG_ENABLE     0
#undef M1_DEADBAND_LUT_APPLY_UD
#define M1_DEADBAND_LUT_APPLY_UD            0
#undef M1_DEADBAND_LUT_ZERO_SEQ_ENABLE
#define M1_DEADBAND_LUT_ZERO_SEQ_ENABLE     0
#undef M1_DEADBAND_LUT_COMMIT_NORMALIZE
#define M1_DEADBAND_LUT_COMMIT_NORMALIZE    0
#undef M1_DEADBAND_NVM_ON_BOOT
#define M1_DEADBAND_NVM_ON_BOOT             0
#undef M1_OPEN_PRE_ID_LADDER_LUT_RUNTIME
#define M1_OPEN_PRE_ID_LADDER_LUT_RUNTIME   0
#undef M1_VOFA_LUT_DUMP_ENABLE
#define M1_VOFA_LUT_DUMP_ENABLE             0
#undef M1_RS_L_IDENT_DUAL_LUT_ROUND_ENABLE
#define M1_RS_L_IDENT_DUAL_LUT_ROUND_ENABLE 0
#undef M1_DEADBAND_FLOW_LD_LQ_AFTER_OPEN_UD
#define M1_DEADBAND_FLOW_LD_LQ_AFTER_OPEN_UD 0
#undef M1_OPEN_UD_PRE_ID_CAL_ENABLE
#define M1_OPEN_UD_PRE_ID_CAL_ENABLE 0
#undef M1_OPEN_UQ_PRE_ID_CAL_ENABLE
#define M1_OPEN_UQ_PRE_ID_CAL_ENABLE 0
#undef M1_OPEN_UD_AFTER_ID_CAL
#define M1_OPEN_UD_AFTER_ID_CAL 0
#undef M1_OPEN_PRE_ID_LADDER_AB_ENABLE
#define M1_OPEN_PRE_ID_LADDER_AB_ENABLE 0
#undef M1_OPEN_UQ_DEADBAND_AB_SWEEP
#define M1_OPEN_UQ_DEADBAND_AB_SWEEP 0

/**
 * Pass0 commit / Id / ramp Rs 辨识（论 2.5 累加 LS；只上报）。
 * open_seq：4=ramp 中，55=成功，6=失败。
 */
#define M1_RS_IDENT_ENABLE              0
/** 1=跳过 Rs ramp，直接用 M1_RS_OHM（与 M1_RS_IDENT_ENABLE 互斥）。 */
#define M1_RS_IDENT_USE_FIXED_NOMINAL   0
#define M1_RS_IDENT_I_MAX_A             3.0f
#define M1_RS_IDENT_RAMP_A_PER_S        0.5f
#define M1_RS_IDENT_I_MIN_FIT_A         2.0f
#define M1_RS_IDENT_EPS_TRACK_A         0.03f
#define M1_RS_IDENT_REPEAT_N            2u
#define M1_RS_IDENT_INTER_ROUND_S       1.0f
#define M1_RS_IDENT_MIN_SAMPLES         200u

/**
 * Rs 后 L 辨识：仅 VESC 旋转 HF+DFT（l_hfi_ident）。
 * open_seq：163=PRE_DECAY；其后由 l_hfi_ident_open_seq_phase 上报。
 */
#define M1_L_HFI_IDENT_ENABLE           0
#define M1_L_HFI_PRE_DECAY_S            0.8f
#define M1_L_HFI_OPEN_SEQ_PRE_DECAY     163u

#define M1_ID_CAL_RS_LD_LQ_ONLY_ENABLE  0

/**
 * 标定 Park 角固定 30° 电角（π/6）；编码器仍采样，仅 Park 用固定角。
 */
#define M1_ID_CAL_FIX_THETA_ENABLE   1
#define M1_ID_CAL_THETA_EL_RAD       0.5235987755982988f  /* pi/6 */

/**
 * FIX_THETA=1 时上电先开环 Ud 对齐，再进 Rs/l_hfi。
 */
#define M1_ID_CAL_ALIGN_ENABLE       1
#define M1_ID_CAL_ALIGN_UD_V         3.0f
#define M1_ID_CAL_ALIGN_S            0.5f

/**
 * =0：telem_ident_dump 不编译（stub）；Rs/Ld-Lq 辨识结果不突 VOFA。
 */
#define M1_VOFA_IDENT_DUMP_ENABLE   0

#endif /* CONFIG_DEFAULTS_ID_CAL_H */
