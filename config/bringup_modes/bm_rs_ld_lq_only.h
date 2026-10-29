/**
 * @file bm_rs_ld_lq_only.h
 * @date 2026-10-28
 * @brief Bringup 模式配方片段（由 motor_params_m1.h 按 M1_BRINGUP_MODE 纳入）。
 */
#ifndef CONFIG_BM_RS_LD_LQ_ONLY_H
#define CONFIG_BM_RS_LD_LQ_ONLY_H

/** Pass0 已完成：ALIGN→HOLD→l_hfi（Rs 固定 M1_RS_OHM）。*/
#undef M1_ID_LOCK_CAL_SWEEP
#define M1_ID_LOCK_CAL_SWEEP            1
#undef M1_IDENT_ENABLE
#define M1_IDENT_ENABLE                 0
#define M1_ID_CAL_RS_LD_LQ_ONLY_ENABLE  1
#define M1_ID_CAL_MULTI_ANGLE_ENABLE    0
#define M1_ID_CAL_PASS0_ONLY_ENABLE     0
#define M1_ID_CAL_LUT_VERIFY_SWEEP      0
#define M1_ID_CAL_IQ_PROBE_ENABLE       0
#undef M1_RS_IDENT_USE_FIXED_NOMINAL
#define M1_RS_IDENT_USE_FIXED_NOMINAL   1
#define M1_RS_IDENT_ENABLE              0
#define M1_L_HFI_IDENT_ENABLE           1
#define M1_ID_CAL_DUAL_ANGLE_ENABLE     0
#define M1_ID_CAL_FIX_THETA_ENABLE      1
#define M1_ID_CAL_ALIGN_ENABLE          1
#define M1_ID_CAL_COMMIT_LUT            0
#undef M1_VOFA_IDENT_DUMP_ENABLE
#define M1_VOFA_IDENT_DUMP_ENABLE         1
#undef M1_PLL_ENABLE
#define M1_PLL_ENABLE                   0
#undef M1_VOFA_PLL_CH8_11
#define M1_VOFA_PLL_CH8_11              0


#endif /* CONFIG_BM_RS_LD_LQ_ONLY_H */
