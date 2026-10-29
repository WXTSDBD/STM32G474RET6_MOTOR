/**
 * @file bm_pass0_rs.h
 * @date 2026-10-28
 * @brief Bringup 模式配方片段（由 motor_params_m1.h 按 M1_BRINGUP_MODE 纳入）。
 * @note PASS0：defaults_id_cal 已先纳入；本头一律 #undef 再覆盖，实验值与改前相同。
 */
#ifndef CONFIG_BM_PASS0_RS_H
#define CONFIG_BM_PASS0_RS_H

/** 只跑参数辨识：跳过 Pass0/LUT，ALIGN → Rs ramp → l_hfi×10（VESC 旋转HF+DFT）。*/
#undef M1_ID_LOCK_CAL_SWEEP
#define M1_ID_LOCK_CAL_SWEEP            1
#undef M1_IDENT_ENABLE
#define M1_IDENT_ENABLE                 0
#undef M1_ID_CAL_RS_LD_LQ_ONLY_ENABLE
#define M1_ID_CAL_RS_LD_LQ_ONLY_ENABLE  1
#undef M1_ID_CAL_MULTI_ANGLE_ENABLE
#define M1_ID_CAL_MULTI_ANGLE_ENABLE    0
#undef M1_ID_CAL_PASS0_ONLY_ENABLE
#define M1_ID_CAL_PASS0_ONLY_ENABLE     0
#undef M1_ID_CAL_LUT_VERIFY_SWEEP
#define M1_ID_CAL_LUT_VERIFY_SWEEP      0
#undef M1_ID_CAL_IQ_PROBE_ENABLE
#define M1_ID_CAL_IQ_PROBE_ENABLE       0
/** Rs 最小二乘；与 FIXED_NOMINAL 互斥。 */
#undef M1_RS_IDENT_USE_FIXED_NOMINAL
#define M1_RS_IDENT_USE_FIXED_NOMINAL   0
#undef M1_RS_IDENT_ENABLE
#define M1_RS_IDENT_ENABLE              1
#undef M1_L_HFI_IDENT_ENABLE
#define M1_L_HFI_IDENT_ENABLE           1
#undef M1_ID_CAL_DUAL_ANGLE_ENABLE
#define M1_ID_CAL_DUAL_ANGLE_ENABLE     0
#undef M1_ID_CAL_FIX_THETA_ENABLE
#define M1_ID_CAL_FIX_THETA_ENABLE      1
/** 开环 Ud 拉到 FIX_THETA 角，再 Rs 灌 Id。 */
#undef M1_ID_CAL_ALIGN_ENABLE
#define M1_ID_CAL_ALIGN_ENABLE          1
/** 3V 曾拉到 ~11A；1.5V 够贴角，电流约减半。 */
#undef M1_ID_CAL_ALIGN_UD_V
#define M1_ID_CAL_ALIGN_UD_V            1.5f
#undef M1_ID_CAL_COMMIT_LUT
#define M1_ID_CAL_COMMIT_LUT            0
#undef M1_RS_L_IDENT_DUAL_LUT_ROUND_ENABLE
#define M1_RS_L_IDENT_DUAL_LUT_ROUND_ENABLE  0
#undef M1_DEADBAND_ENABLE
#define M1_DEADBAND_ENABLE              0
/** HF-L 连跑 10 轮，轮间歇 2 s；每轮 mark55 连发本轮结果。 */
#undef M1_L_HFI_REPEAT_N
#define M1_L_HFI_REPEAT_N               10u
#undef M1_L_HFI_INTER_ROUND_S
#define M1_L_HFI_INTER_ROUND_S          2.0f
/*
 * 遥测：6ch @ 20kHz（DECIM=1）。12ch@10kHz 对 20kHz 双极半拍别名，离线凑不齐 Δi；
 * 6×20k≈560KB/s，与原 12×10k 同量级，6Mbps UART 吃得下。VOFA 通道数改 6。
 */
#undef M1_TELEM_BRINGUP_K
#define M1_TELEM_BRINGUP_K              6u
#undef M1_TELEM_BRINGUP_DECIMATION
#define M1_TELEM_BRINGUP_DECIMATION     1u
#include "profiles/m1_hfi_compile_nums.h"
#undef M1_HFI_ENABLE
#define M1_HFI_ENABLE                   1
#undef M1_HFI_MOTION_BYPASS_ENABLE
#define M1_HFI_MOTION_BYPASS_ENABLE     1
#undef M1_EMF_SMO_ENABLE
#define M1_EMF_SMO_ENABLE               1
#undef M1_EMF_PLL_ENABLE
#define M1_EMF_PLL_ENABLE               1
#undef M1_EMF_PLL_USE_SMO
#define M1_EMF_PLL_USE_SMO              1
#undef M1_VOFA_OPT12
#define M1_VOFA_OPT12                   0
#undef M1_VOFA_IDENT_DUTY_12CH
#define M1_VOFA_IDENT_DUTY_12CH         1

/* 配方互斥（defaults 先纳入时未开 Rs；覆盖后在此断言）。 */
#if M1_RS_IDENT_USE_FIXED_NOMINAL && M1_RS_IDENT_ENABLE
#error "M1_RS_IDENT_USE_FIXED_NOMINAL and M1_RS_IDENT_ENABLE are mutually exclusive"
#endif
#if M1_RS_IDENT_ENABLE && !M1_ID_CAL_COMMIT_LUT && !M1_ID_CAL_RS_LD_LQ_ONLY_ENABLE
#error "M1_RS_IDENT_ENABLE requires M1_ID_CAL_COMMIT_LUT=1 (or RS_LD_LQ_ONLY)"
#endif
#if M1_RS_IDENT_ENABLE && M1_IDENT_ENABLE
#error "M1_RS_IDENT_ENABLE requires M1_IDENT_ENABLE=0"
#endif
#if M1_L_HFI_IDENT_ENABLE && !M1_RS_IDENT_ENABLE && !M1_RS_IDENT_USE_FIXED_NOMINAL
#error "M1_L_HFI_IDENT_ENABLE requires M1_RS_IDENT_ENABLE=1 or M1_RS_IDENT_USE_FIXED_NOMINAL=1"
#endif
#if M1_L_HFI_IDENT_ENABLE && !M1_ID_CAL_COMMIT_LUT && !M1_ID_CAL_RS_LD_LQ_ONLY_ENABLE
#error "M1_L_HFI_IDENT_ENABLE requires M1_ID_CAL_COMMIT_LUT=1 (or RS_LD_LQ_ONLY)"
#endif
#if M1_ID_CAL_RS_LD_LQ_ONLY_ENABLE && !M1_ID_LOCK_CAL_SWEEP
#error "M1_ID_CAL_RS_LD_LQ_ONLY_ENABLE requires M1_ID_LOCK_CAL_SWEEP=1"
#endif
#if M1_ID_CAL_RS_LD_LQ_ONLY_ENABLE && \
    (!M1_L_HFI_IDENT_ENABLE || (!M1_RS_IDENT_ENABLE && !M1_RS_IDENT_USE_FIXED_NOMINAL))
#error "M1_ID_CAL_RS_LD_LQ_ONLY_ENABLE requires M1_L_HFI_IDENT_ENABLE and (M1_RS_IDENT_ENABLE or M1_RS_IDENT_USE_FIXED_NOMINAL)"
#endif
#if M1_ID_CAL_ALIGN_ENABLE && !M1_ID_CAL_FIX_THETA_ENABLE
#error "M1_ID_CAL_ALIGN_ENABLE requires M1_ID_CAL_FIX_THETA_ENABLE=1"
#endif

#endif /* CONFIG_BM_PASS0_RS_H */
