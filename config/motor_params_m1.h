/**
 * @file motor_params_m1.h
 * @brief M1 电机硬件常数：电流采样链路的标度（初始化预计算，热路径只乘不除）。
 *
 * ADC2 三相：外部放。×10，采样电。10 mΩ。2bit 单端 @ VDDA。
 */

#ifndef MOTOR_PARAMS_M1_H
#define MOTOR_PARAMS_M1_H

/** N5065 极对。*/
#define M1_POLE_PAIRS       7u

/** VDDA / ADC 参考（V。*/
#define M1_ADC_VREF_V       3.3f

/** 相电流采样电阻（Ω），10 mΩ */
#define M1_ADC_SHUNT_OHM    0.01f

/** 外部电流放大倍数 */
#define M1_ADC_AMP_GAIN     10.0f

/** 安培/LSB：Vref / (4096 × R_shunt × Gain)，热路径仅做 (raw-offset)*scale */
#define M1_ADC_SCALE_A_LSB  (M1_ADC_VREF_V / (4096.0f * M1_ADC_SHUNT_OHM * M1_ADC_AMP_GAIN))

/** 物理 JDR rank0/1/2 per-channel gain；统一 1.0（扇区采。KCL 修后再标定，勿用 run RMS 凑） */
#ifndef M1_ADC_GAIN_CH0
#define M1_ADC_GAIN_CH0     1.00f
#endif
#ifndef M1_ADC_GAIN_CH1
#define M1_ADC_GAIN_CH1     1.00f
#endif
#ifndef M1_ADC_GAIN_CH2
#define M1_ADC_GAIN_CH2     1.00f
#endif

/* --- 电气参数（LCR @ 1 kHz，AB 线；Rs 多轮 VASI 辨识 ~0.122 Ω。--- */
#define M1_RS_OHM           0.122f
#define M1_LD_H             59e-6f
#define M1_LQ_H             87e-6f

/** JEOC 电流环节拍（s。*/
#define M1_CTRL_TS_S        50e-6f

/**
 * Type-II PLL @ 20 kHz（bringup 试用；架构文档默。2 kHz，后续可。TIM6）。
 * M1_PLL_THETA_PARK_ENABLE=0 。Park 仍用编码。raw 电角，仅遥测 ω。
 *
 * 带宽整定（。0.707）：Ki=ωn²，Kp=2ζωn，ωn=2π·fn；fn [Hz] 为机械角域自然频率。
 *   fn。2  匀速最平滑，加减。低速切换慢（现。1242 CSV 表现。
 *   fn。0~80  速度切换 / 手拨加减速（推荐 bringup。
 *   fn。00+ 更跟手，20 kHz 。ω 噪声会变。
 */
#ifndef M1_PLL_ENABLE
#define M1_PLL_ENABLE                   1
#endif
#ifndef M1_PLL_FN_HZ
#define M1_PLL_FN_HZ                    80.0f
#endif
#ifndef M1_PLL_ZETA
#define M1_PLL_ZETA                     0.707106781f
#endif
#ifndef M1_PLL_WN_RAD_S
#define M1_PLL_WN_RAD_S                 (6.28318530718f * M1_PLL_FN_HZ)
#endif
#ifndef M1_PLL_KP
#define M1_PLL_KP                       (2.0f * M1_PLL_ZETA * M1_PLL_WN_RAD_S)
#endif
#ifndef M1_PLL_KI
#define M1_PLL_KI                       (M1_PLL_WN_RAD_S * M1_PLL_WN_RAD_S)
#endif
#ifndef M1_PLL_OMEGA_LIMIT_RAD_S
#define M1_PLL_OMEGA_LIMIT_RAD_S        650.0f   /* ~6200 rpm mech */
#endif
#ifndef M1_PLL_INTEGRATOR_LIMIT_RAD_S
#define M1_PLL_INTEGRATOR_LIMIT_RAD_S   M1_PLL_OMEGA_LIMIT_RAD_S
#endif
#ifndef M1_PLL_THETA_PARK_ENABLE
#define M1_PLL_THETA_PARK_ENABLE        0
#endif
/** 1=VOFA ch8。1 输出 PLL（ch8 ω_pll ch9 ω_diff ch10 θ_err ch11 Δω）；0=Id_ref/Iq_ref/duty_dev/open_seq */
#ifndef M1_VOFA_PLL_CH8_11
#define M1_VOFA_PLL_CH8_11              M1_PLL_ENABLE
#endif

/* ==========================================================================
 * 死区 LUT 三步联调 。只改 M1_DEADBAND_BRINGUP_PHASE 。Rebuild
 *
 *   M1_DB_BRINGUP_SPEED_OFF   。速度阶梯 profile，deadband OFF（基线）
 *   M1_DB_BRINGUP_PASS0         。Pass0 30° 建表 。commit 。VOFA LUT 突发
 *   M1_DB_BRINGUP_ONE_SHOT      。Pass0→OFF 阶梯→LUT 阶梯
 *   M1_DB_BRINGUP_SPEED_IDENT   。上电 deadband OFF 。速度阶跃（无 Bode，带载测。
 *
 * SPEED_IDENT：open_seq 220=HOLD 221..226=STEP 239=DONE
 * ONE_SHOT：open_seq 201=Pass0 / 200=OFF / 210=LUT
 * 分步 0/1/2 才需 parse_lut_vofa.py --emit-baked
 * ========================================================================== */
#define M1_DB_BRINGUP_SPEED_OFF         0
#define M1_DB_BRINGUP_PASS0             1
#define M1_DB_BRINGUP_SPEED_LUT         2
/** 一次上电：Pass0 锁轴建表 。速度 OFF 阶梯 。速度 LUT 阶梯（不导出 baked。*/
#define M1_DB_BRINGUP_ONE_SHOT          3
/** 上电 deadband OFF 。速度环阶跃（。Bode，带载测。*/
#define M1_DB_BRINGUP_SPEED_IDENT       4
#ifndef M1_DEADBAND_BRINGUP_PHASE
#define M1_DEADBAND_BRINGUP_PHASE       M1_DB_BRINGUP_SPEED_OFF
#endif
#if M1_DEADBAND_BRINGUP_PHASE == M1_DB_BRINGUP_ONE_SHOT
#define M1_DEADBAND_FLOW_ONE_SHOT         1
#else
#define M1_DEADBAND_FLOW_ONE_SHOT         0
#endif

/* Bringup mode（须。M1_SPEED_LOOP_ENABLE 之前选定。*/
#define M1_BRINGUP_MODE_NORMAL                0
#define M1_BRINGUP_MODE_IDENT_IQ_STEP         1
#define M1_BRINGUP_MODE_ID_CAL_DUAL_FULL      2
#define M1_BRINGUP_MODE_ID_CAL_PASS0_ONLY     3
#define M1_BRINGUP_MODE_MULTI_ANGLE_PASS0     4
#define M1_BRINGUP_MODE_ID_CAL_PASS0_RS       5
#define M1_BRINGUP_MODE_IQ_PROBE_OFF_ONLY     6
#define M1_BRINGUP_MODE_ID_CAL_PASS0_BUILD    7
#define M1_BRINGUP_MODE_SPEED_IDENT           8
#define M1_BRINGUP_MODE_BODE_OFF_ONLY         9
#define M1_BRINGUP_MODE_RS_LD_LQ_ONLY         10
#define M1_BRINGUP_MODE_BODE_ID_OFF_ONLY      11
/** 。Pass0 后：开。Ud 阶梯。LUT（NVM LUT ON；无 Pass0/ident。*/
#define M1_BRINGUP_MODE_OPEN_UD_LUT_VERIFY    12

/*
 * 联调实例选择：只。config/bringup_active.h
 * Bode 配方：config/profiles/m1_bode_id_fc1000.profile.h
 * 磁链稳速：config/profiles/m1_flux_id_1000rpm.profile.h
 */
#include "bringup_active.h"
#ifndef M1_USE_FLUX_ID_PROFILE
#define M1_USE_FLUX_ID_PROFILE          0
#endif
#ifndef M1_USE_OBS_VEQ_PROFILE
#define M1_USE_OBS_VEQ_PROFILE          0
#endif
#ifndef M1_USE_SPEED_1000_PROFILE
#define M1_USE_SPEED_1000_PROFILE       0
#endif
#ifndef M1_USE_IF_100_PROFILE
#define M1_USE_IF_100_PROFILE           0
#endif
#ifndef M1_USE_HFI_STANDSTILL_PROFILE
#define M1_USE_HFI_STANDSTILL_PROFILE   0
#endif
#ifndef M1_HFI_GATE
#define M1_HFI_GATE                     0
#endif
#if ((M1_USE_FLUX_ID_PROFILE != 0) + (M1_USE_OBS_VEQ_PROFILE != 0) + \
     (M1_USE_SPEED_1000_PROFILE != 0) + (M1_USE_IF_100_PROFILE != 0) + \
     (M1_USE_HFI_STANDSTILL_PROFILE != 0)) > 1
#error "M1_USE_SPEED_1000 / FLUX / OBS_VEQ / IF_100 / HFI_STANDSTILL profiles are mutually exclusive"
#endif
#ifndef M1_HFI_ENABLE
#define M1_HFI_ENABLE                   0
#endif
#ifndef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              0
#endif
#ifndef M1_HFI_MOTION_BYPASS_ENABLE
#define M1_HFI_MOTION_BYPASS_ENABLE     0
#endif
#ifndef M1_HFI_OMEGA_FF_SRC
#define M1_HFI_OMEGA_FF_SRC             0
#endif
#ifndef M1_VOFA_HFI_12CH
#define M1_VOFA_HFI_12CH                0
#endif
#ifndef M1_EMF_VEQ_ENABLE
#define M1_EMF_VEQ_ENABLE               0
#endif
#ifndef M1_EMF_SMO_ENABLE
#define M1_EMF_SMO_ENABLE               0
#endif
#ifndef M1_EMF_PLL_ENABLE
#define M1_EMF_PLL_ENABLE               0
#endif
#ifndef M1_EMF_SMO_LPF_SCHED_ENABLE
#define M1_EMF_SMO_LPF_SCHED_ENABLE     0
#endif
#ifndef M1_EMF_SMO_LPF_LINEAR_ENABLE
#define M1_EMF_SMO_LPF_LINEAR_ENABLE    0
#endif
#ifndef M1_EMF_LPF_PHASE_FF_ENABLE
#define M1_EMF_LPF_PHASE_FF_ENABLE      0 /* 1=θ̂ 。−atan(fe/fc) 模型前馈 */
#endif
#ifndef M1_OBS_SOFT_SWITCH_ENABLE
#define M1_OBS_SOFT_SWITCH_ENABLE       0
#endif
#ifndef M1_OBS_SS_SPEED_SWITCH_ENABLE
#define M1_OBS_SS_SPEED_SWITCH_ENABLE   0
#endif
/** 1=先角后速：。OBS 后速度仍吃编码器，门限满足再切观测。*/
#ifndef M1_OBS_SS_SPD_DEFER_ENABLE
#define M1_OBS_SS_SPD_DEFER_ENABLE      0
#endif
#ifndef M1_OBS_SS_SPD_DWELL_S
#define M1_OBS_SS_SPD_DWELL_S           1.5f /* 。OBS 后最短等待再武装切。*/
#endif
#ifndef M1_OBS_SS_SPD_HOLD_S
#define M1_OBS_SS_SPD_HOLD_S            0.4f /* 切速门限连续保。*/
#endif
#ifndef M1_OBS_SS_SPD_RPM_ERR_FRAC
#define M1_OBS_SS_SPD_RPM_ERR_FRAC      0.08f /* |ω̂−ω_ref|/ω_ref */
#endif
#ifndef M1_OBS_SS_SPD_ERR_RAD
#define M1_OBS_SS_SPD_ERR_RAD           0.2617994f /* 15°，切速时角仍须好 */
#endif
#ifndef M1_OBS_SS_SPD_DOMEGA_MAX
#define M1_OBS_SS_SPD_DOMEGA_MAX        2500.0f /* 长窗 |Δω̂|/Δt [rpm/s]，抑真猎。*/
#endif
#ifndef M1_OBS_SS_SPD_DOMEGA_WIN_S
#define M1_OBS_SS_SPD_DOMEGA_WIN_S      0.10f /* 加速度估计窗；勿用单拍 SPEED_TS */
#endif
#ifndef M1_OBS_SS_SPD_ENC_MATCH_ENABLE
#define M1_OBS_SS_SPD_ENC_MATCH_ENABLE  0 /* 1=另要。|ω̂−ω_enc|≤MATCH（联调） */
#endif
#ifndef M1_OBS_SS_SPD_ENC_MATCH_RPM
#define M1_OBS_SS_SPD_ENC_MATCH_RPM     50.0f
#endif
#ifndef M1_OBS_SS_SPD_REQUIRE_EMAG
#define M1_OBS_SS_SPD_REQUIRE_EMAG      0
#endif
#ifndef M1_OBS_SS_SPD_REQUIRE_ERR
#define M1_OBS_SS_SPD_REQUIRE_ERR       0
#endif
#ifndef M1_OBS_SS_FALLBACK_ON_ERR_ENABLE
#define M1_OBS_SS_FALLBACK_ON_ERR_ENABLE 1 /* 0=角差仅监督，不回切编码器 */
#endif
#ifndef M1_OBS_SS_FALLBACK_ENABLE
#define M1_OBS_SS_FALLBACK_ENABLE       1 /* 0=禁用全部回退（掉。过流亦不踢） */
#endif
#ifndef M1_OBS_SPD_PLL_ENABLE
#define M1_OBS_SPD_PLL_ENABLE           0
#endif
#ifndef M1_OBS_SPD_PLL_FN_HZ
#define M1_OBS_SPD_PLL_FN_HZ            12.0f
#endif
#ifndef M1_OBS_SPD_PLL_ZETA
#define M1_OBS_SPD_PLL_ZETA             0.707106781f
#endif
#ifndef M1_OBS_SPD_FB_LPF_HZ
#define M1_OBS_SPD_FB_LPF_HZ            0 /* [Hz] 0=off；观测速进速度环前一。LPF */
#endif
#ifndef M1_OBS_THETA_NOTCH_ENABLE
#define M1_OBS_THETA_NOTCH_ENABLE       0 /* 1=Park 前对 θ̂ 机械 1/rev 陷波 */
#endif
#ifndef M1_OBS_THETA_NOTCH_Q
#define M1_OBS_THETA_NOTCH_Q            10.0f
#endif
#ifndef M1_OBS_THETA_NOTCH_TRACK_HZ
#define M1_OBS_THETA_NOTCH_TRACK_HZ     3.0f /* 慢跟踪带宽，。<< 1/rev */
#endif
#ifndef M1_OBS_THETA_NOTCH_RPM_MIN
#define M1_OBS_THETA_NOTCH_RPM_MIN      400.0f
#endif
#ifndef M1_OBS_THETA_NOTCH_H2_ENABLE
#define M1_OBS_THETA_NOTCH_H2_ENABLE    0 /* 1=级联 2/rev */
#endif
#ifndef M1_VOFA_OBS_VEQ_12CH
#define M1_VOFA_OBS_VEQ_12CH            0
#endif
#ifndef M1_VOFA_OBS_SMO_12CH
#define M1_VOFA_OBS_SMO_12CH            0
#endif
#ifndef M1_VOFA_OBS_PLL_12CH
#define M1_VOFA_OBS_PLL_12CH            0
#endif
#ifndef M1_VOFA_OBS_SMO_RAW_12CH
#define M1_VOFA_OBS_SMO_RAW_12CH        0
#endif
#if M1_EMF_PLL_ENABLE && !(M1_EMF_VEQ_ENABLE || M1_EMF_SMO_ENABLE)
#error "M1_EMF_PLL_ENABLE requires M1_EMF_VEQ_ENABLE and/or M1_EMF_SMO_ENABLE (PLL feeds on eαβ)"
#endif
#if (M1_VOFA_OBS_PLL_12CH + M1_VOFA_OBS_SMO_12CH + M1_VOFA_OBS_VEQ_12CH + M1_VOFA_OBS_SMO_RAW_12CH) > 1
#error "Only one of M1_VOFA_OBS_PLL/SMO/VEQ/SMO_RAW_12CH may be 1"
#endif
#if M1_OBS_SOFT_SWITCH_ENABLE && !M1_EMF_PLL_ENABLE
#error "M1_OBS_SOFT_SWITCH_ENABLE requires M1_EMF_PLL_ENABLE"
#endif
/** 1=PLL 。SMO e（无。SMO 主路径）。=。Veq e。未定义时：。SMO 则优。SMO */
#ifndef M1_EMF_PLL_USE_SMO
#if M1_EMF_SMO_ENABLE
#define M1_EMF_PLL_USE_SMO              1
#else
#define M1_EMF_PLL_USE_SMO              0
#endif
#endif
#if M1_EMF_PLL_USE_SMO && !M1_EMF_SMO_ENABLE
#error "M1_EMF_PLL_USE_SMO=1 requires M1_EMF_SMO_ENABLE"
#endif
#if M1_EMF_PLL_ENABLE && !M1_EMF_PLL_USE_SMO && !M1_EMF_VEQ_ENABLE
#error "PLL without USE_SMO requires M1_EMF_VEQ_ENABLE"
#endif
#if (M1_BRINGUP_MODE == M1_BRINGUP_MODE_SPEED_IDENT)
/** 实验1 签收：。ramp 500 rpm/s（仅 SPEED_IDENT。*/
#ifndef M1_SPEED_OMEGA_RAMP_ENABLE
#define M1_SPEED_OMEGA_RAMP_ENABLE      1
#endif
#ifndef M1_SPEED_OMEGA_RAMP_RPM_S
#define M1_SPEED_OMEGA_RAMP_RPM_S       500.0f
#endif
#endif

/* --- E1 速度。@ 2 kHz。0 kHz 分频；限幅宏。M1_I_REF_ABS_MAX 之后。-- */
#ifndef M1_SPEED_LOOP_ENABLE
#if (M1_BRINGUP_MODE == M1_BRINGUP_MODE_BODE_OFF_ONLY) || \
    (M1_BRINGUP_MODE == M1_BRINGUP_MODE_BODE_ID_OFF_ONLY)
#define M1_SPEED_LOOP_ENABLE            0
#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_RS_LD_LQ_ONLY)
#define M1_SPEED_LOOP_ENABLE            0
#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_OPEN_UD_LUT_VERIFY)
#define M1_SPEED_LOOP_ENABLE            0
#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_SPEED_IDENT)
#define M1_SPEED_LOOP_ENABLE            1
#elif (M1_BRINGUP_MODE != M1_BRINGUP_MODE_NORMAL)
/** 所有辨。标定 bringup（除 SPEED_IDENT）均关外环，避免。Id 锁轴/VASI 。iq_ref */
#define M1_SPEED_LOOP_ENABLE            0
#elif M1_DEADBAND_BRINGUP_PHASE == M1_DB_BRINGUP_PASS0
#define M1_SPEED_LOOP_ENABLE            0
#else
#define M1_SPEED_LOOP_ENABLE            1
#endif
#endif
#if M1_SPEED_LOOP_ENABLE
#ifndef M1_SPEED_DECIM
#define M1_SPEED_DECIM                  10u
#endif
#define M1_SPEED_TS_S                   (M1_CTRL_TS_S * (float)M1_SPEED_DECIM)
#ifndef M1_SPEED_PI_FN_HZ
#define M1_SPEED_PI_FN_HZ               20.0f
#endif
#ifndef M1_SPEED_PI_ZETA
#define M1_SPEED_PI_ZETA                0.707106781f
#endif
#define M1_SPEED_PI_WN_RAD_S            (6.28318530718f * M1_SPEED_PI_FN_HZ)
#ifndef M1_SPEED_PI_KP
#define M1_SPEED_PI_KP                  0.015f
#endif
#ifndef M1_SPEED_PI_KI
#if (M1_BRINGUP_MODE == M1_BRINGUP_MODE_SPEED_IDENT)
#define M1_SPEED_PI_KI                  0.002f
#else
#define M1_SPEED_PI_KI                  0.002f
#endif
#endif
/** 2-DOF 速度 PI：Kp 作用。(β·ω_ref 。ω_fb)。.0=标准 PI。.3~0.5 抑阶跃超。*/
#ifndef M1_SPEED_PI_BETA
#if (M1_BRINGUP_MODE == M1_BRINGUP_MODE_SPEED_IDENT)
#define M1_SPEED_PI_BETA                0.4f
#else
#define M1_SPEED_PI_BETA                1.0f
#endif
#endif
#ifndef M1_SPEED_OMEGA_RAMP_ENABLE
#if (M1_BRINGUP_MODE == M1_BRINGUP_MODE_SPEED_IDENT)
#define M1_SPEED_OMEGA_RAMP_ENABLE      1   /* 实验1：抑减速硬阶跃 */
#else
#define M1_SPEED_OMEGA_RAMP_ENABLE      0
#endif
#endif
#ifndef M1_SPEED_OMEGA_RAMP_RPM_S
#define M1_SPEED_OMEGA_RAMP_RPM_S       500.0f
#endif
#ifndef M1_SPEED_IQ_SLEW_ENABLE
#define M1_SPEED_IQ_SLEW_ENABLE         0
#endif
#ifndef M1_SPEED_IQ_SLEW_A_PER_S
#define M1_SPEED_IQ_SLEW_A_PER_S        3.0f
#endif
#ifndef M1_FOC_ROTATION_FF_ENABLE
#define M1_FOC_ROTATION_FF_ENABLE       0
#endif
#ifndef M1_SPEED_REF_RPM_DEFAULT
#define M1_SPEED_REF_RPM_DEFAULT        300.0f
#endif
#ifndef M1_SPEED_LOOP_BOOT
#define M1_SPEED_LOOP_BOOT              1
#endif
#ifndef M1_VOFA_SPEED_CH8_11
#define M1_VOFA_SPEED_CH8_11            1
#endif

/** 1=位置 P 外环（θ_ref 。ω_ref 。速度 PI）；上电相对零位，不 NVM */
#ifndef M1_POS_LOOP_ENABLE
#define M1_POS_LOOP_ENABLE              1
#endif
#if M1_POS_LOOP_ENABLE
/** ω_ref = Kp * (θ_ref 。θ_mech) [rpm/rad]，多。unwrap 全误。*/
#ifndef M1_POS_KP_RPM_PER_RAD
#define M1_POS_KP_RPM_PER_RAD           80.0f
#endif
#ifndef M1_POS_OMEGA_MAX_RPM
#define M1_POS_OMEGA_MAX_RPM            400.0f
#endif
/** 1=θ_err 滞环停驱区：区内 ω_ref=0，可选冻速度 PI；抑制目标附近抖。*/
#ifndef M1_POS_ERR_HYST_ENABLE
#define M1_POS_ERR_HYST_ENABLE          1
#endif
#if M1_POS_ERR_HYST_ENABLE
#ifndef M1_POS_ERR_HYST_ENTER_RAD
#define M1_POS_ERR_HYST_ENTER_RAD       0.0087266f  /* 0.5° 进入停驱。*/
#endif
#ifndef M1_POS_ERR_HYST_EXIT_RAD
#define M1_POS_ERR_HYST_EXIT_RAD        0.0174533f  /* 1.0° 退出（> ENTER。*/
#endif
#ifndef M1_POS_ERR_HYST_FREEZE_SPEED_PI
#define M1_POS_ERR_HYST_FREEZE_SPEED_PI 1
#endif
#endif /* M1_POS_ERR_HYST_ENABLE */
/** 位置 P 分频：外。2kHz / DECIM 。P 更新率。=500Hz（相对速度 Fn。0Hz 足够）；仍抖可试 10=200Hz */
#ifndef M1_POS_DECIM
#define M1_POS_DECIM                      4u
#endif
#if M1_POS_DECIM < 1u
#error "M1_POS_DECIM must be >= 1"
#endif
#define M1_POS_TS_S                       (M1_SPEED_TS_S * (float)M1_POS_DECIM)
/** 1=上电 POSITION 并保。θ_ref=θ_mech(0)。=沿用 SPEED / ident 启动 */
#ifndef M1_POS_LOOP_BOOT
#define M1_POS_LOOP_BOOT                0
#endif
/** 1=位置 26 。+ MIT 同轨迹联测（NORMAL+SPEED_OFF 默认开。*/
#ifndef M1_POS_MIT_COMBO_ENABLE
#if (M1_BRINGUP_MODE == M1_BRINGUP_MODE_NORMAL) && \
    (M1_DEADBAND_BRINGUP_PHASE == M1_DB_BRINGUP_SPEED_OFF)
#define M1_POS_MIT_COMBO_ENABLE         1
#else
#define M1_POS_MIT_COMBO_ENABLE         0
#endif
#endif

/** 1=combo 含位。26 档；0=。MIT 同轨迹验收（暂停位置环） */
#ifndef M1_POS_MIT_COMBO_POS_ENABLE
#define M1_POS_MIT_COMBO_POS_ENABLE         0
#endif

/** 1=MIT 验收 VOFA ch8=ω_pll ch9=θ_err ch10=Iq_ref ch11=open_seq（随 MIT-only 默认开。*/
#ifndef M1_VOFA_MIT_CH8_11
#if M1_POS_MIT_COMBO_ENABLE && !M1_POS_MIT_COMBO_POS_ENABLE
#define M1_VOFA_MIT_CH8_11                  1
#else
#define M1_VOFA_MIT_CH8_11                  0
#endif
#endif

/** 1=上电 POSITION 阶跃序列（见 M1_POS_STEP_*）；。M1_SPEED_REVERSAL_TEST 互斥 */
#ifndef M1_POS_STEP_TEST_ENABLE
#if M1_POS_MIT_COMBO_ENABLE
#define M1_POS_STEP_TEST_ENABLE         1
#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_NORMAL) && \
    (M1_DEADBAND_BRINGUP_PHASE == M1_DB_BRINGUP_SPEED_OFF)
#define M1_POS_STEP_TEST_ENABLE         1
#else
#define M1_POS_STEP_TEST_ENABLE         0
#endif
#endif
#if M1_POS_STEP_TEST_ENABLE
#ifndef M1_POS_STEP_HOLD_S
#define M1_POS_STEP_HOLD_S              1.0f    /* 初始 hold @ θ(0) */
#endif
#ifndef M1_POS_STEP_DWELL_S
#define M1_POS_STEP_DWELL_S             1.5f    /* |Δθ| < 180° */
#endif
#ifndef M1_POS_STEP_DWELL_WIDE_S
#define M1_POS_STEP_DWELL_WIDE_S        2.0f    /* 180° 。|Δθ| < 360° */
#endif
#ifndef M1_POS_STEP_DWELL_MULT_S
#define M1_POS_STEP_DWELL_MULT_S        3.0f    /* 360° 。|Δθ| < 720° */
#endif
#ifndef M1_POS_STEP_DWELL_LONG_S
#define M1_POS_STEP_DWELL_LONG_S        4.0f    /* |Δθ| 。720° */
#endif
#ifndef M1_POS_STEP_SEQ_BASE
#define M1_POS_STEP_SEQ_BASE            229u    /* uint8。29..254=各档。6 档）。55=done */
#endif
/** 阶跃联调：Kp 30。5 。P+滞环抖；ω_max 暂保。250 rpm */
#ifndef M1_POS_STEP_KP_RPM_PER_RAD
#define M1_POS_STEP_KP_RPM_PER_RAD      15.0f
#endif
#ifndef M1_POS_STEP_OMEGA_MAX_RPM
#define M1_POS_STEP_OMEGA_MAX_RPM       250.0f
#endif
#undef M1_POS_KP_RPM_PER_RAD
#define M1_POS_KP_RPM_PER_RAD           M1_POS_STEP_KP_RPM_PER_RAD
#undef M1_POS_OMEGA_MAX_RPM
#define M1_POS_OMEGA_MAX_RPM            M1_POS_STEP_OMEGA_MAX_RPM
#ifndef M1_POS_BOOT_SETTLE_S
#define M1_POS_BOOT_SETTLE_S            0.0f    /* 0=关；settle 曾致 ω_ramp 。P 脱节 */
#endif
#endif /* M1_POS_STEP_TEST_ENABLE */

#if M1_POS_MIT_COMBO_ENABLE
/** MIT 。open_seq 130..155=各档。56=done（须 <256：dbg.open_seq 。uint8。*/
#ifndef M1_MIT_STEP_SEQ_BASE
#define M1_MIT_STEP_SEQ_BASE            130u
#endif
/** τ = Kp·(θ_des−。 + Kd·(ω_des−。 + τ_ff；ω_des=τ_ff=0 */
#ifndef M1_MIT_KP_NM_PER_RAD
#define M1_MIT_KP_NM_PER_RAD            10.0f
#endif
#ifndef M1_MIT_KD_NM_S_PER_RAD
#define M1_MIT_KD_NM_S_PER_RAD          1.0f
#endif
#ifndef M1_MIT_OMEGA_DES_RAD_S
#define M1_MIT_OMEGA_DES_RAD_S          0.0f
#endif
#ifndef M1_MIT_TAU_FF_NM
#define M1_MIT_TAU_FF_NM                0.0f
#endif
/** |τ| 限幅后再 /Kt，避。Kp 过大。iq_ref 。±I_max 振荡 */
#ifndef M1_MIT_TAU_ABS_MAX_NM
#define M1_MIT_TAU_ABS_MAX_NM           0.12f
#endif
/** MIT iq_cmd 斜坡 [A/s]；POS→MIT 切换时抑 ±I_max 硬阶。*/
#ifndef M1_MIT_IQ_SLEW_A_PER_S
#define M1_MIT_IQ_SLEW_A_PER_S          40.0f
#endif
/** 力矩→电流：τ [Nm] = Kt·Iq；待 Kt 标定后可。*/
#ifndef M1_MIT_KT_NM_A
#define M1_MIT_KT_NM_A                  0.025f
#endif
#if (M1_MIT_STEP_SEQ_BASE + 26u) > 255u
#error "M1_MIT_STEP_SEQ_BASE + 26 steps must fit uint8 open_seq (<256)"
#endif
#if M1_POS_MIT_COMBO_ENABLE && M1_SPEED_IDENT_ENABLE
#error "M1_POS_MIT_COMBO_ENABLE and M1_SPEED_IDENT_ENABLE are mutually exclusive"
#endif
#endif /* M1_POS_MIT_COMBO_ENABLE */

#if M1_POS_STEP_TEST_ENABLE && (M1_BRINGUP_MODE == M1_BRINGUP_MODE_NORMAL)
#undef M1_SPEED_OMEGA_RAMP_ENABLE
#define M1_SPEED_OMEGA_RAMP_ENABLE      1
#undef M1_SPEED_OMEGA_RAMP_RPM_S
#define M1_SPEED_OMEGA_RAMP_RPM_S       200.0f
#endif

#endif /* M1_POS_LOOP_ENABLE */

/**
 * deadband 效果测试（由 M1_DEADBAND_BRINGUP_PHASE 自动选择；勿手改除非单测 LUT。
 */
#define M1_SPEED_DEADBAND_TEST_OFF        0
#define M1_SPEED_DEADBAND_TEST_LUT        1
#ifndef M1_SPEED_DEADBAND_TEST
#if M1_DEADBAND_BRINGUP_PHASE == M1_DB_BRINGUP_SPEED_LUT
#define M1_SPEED_DEADBAND_TEST            M1_SPEED_DEADBAND_TEST_LUT
#else
#define M1_SPEED_DEADBAND_TEST            M1_SPEED_DEADBAND_TEST_OFF
#endif
#endif

/** @deprecated 。M1_SPEED_DEADBAND_TEST 推导，勿手改 */
#if M1_SPEED_DEADBAND_TEST == M1_SPEED_DEADBAND_TEST_OFF
#define M1_SPEED_LOOP_DEADBAND_OFF        1
#else
#define M1_SPEED_LOOP_DEADBAND_OFF        0
#endif

/** 1=上电自动阶梯。00。00。00。00。00 rpm，每。M1_SPEED_PROFILE_HOLD_S */
#ifndef M1_SPEED_PROFILE_ENABLE
#if M1_DEADBAND_BRINGUP_PHASE == M1_DB_BRINGUP_PASS0
#define M1_SPEED_PROFILE_ENABLE           0
#elif M1_DEADBAND_BRINGUP_PHASE == M1_DB_BRINGUP_SPEED_IDENT
#define M1_SPEED_PROFILE_ENABLE           0
#elif M1_DEADBAND_BRINGUP_PHASE == M1_DB_BRINGUP_SPEED_OFF
#define M1_SPEED_PROFILE_ENABLE           0   /* 。300 rpm，不跑阶。profile */
#else
#define M1_SPEED_PROFILE_ENABLE           1
#endif
#endif
#if M1_SPEED_PROFILE_ENABLE
#ifndef M1_SPEED_PROFILE_HOLD_S
#define M1_SPEED_PROFILE_HOLD_S           10.0f
#endif
#ifndef M1_SPEED_PROFILE_RPM_START
#define M1_SPEED_PROFILE_RPM_START        100.0f
#endif
#ifndef M1_SPEED_PROFILE_RPM_STEP
#define M1_SPEED_PROFILE_RPM_STEP         200.0f
#endif
#ifndef M1_SPEED_PROFILE_RPM_END
#define M1_SPEED_PROFILE_RPM_END          900.0f
#endif
#ifndef M1_SPEED_PROFILE_REPEAT
#define M1_SPEED_PROFILE_REPEAT           1   /* 1=900 后回。100 循环 */
#endif
#endif /* M1_SPEED_PROFILE_ENABLE */

/** 1=上电 ±RPM 交替（正反转观测）；SPEED_OFF 默认开，每。M1_SPEED_REVERSAL_HOLD_S */
#ifndef M1_SPEED_REVERSAL_TEST_ENABLE
#if M1_POS_STEP_TEST_ENABLE
#define M1_SPEED_REVERSAL_TEST_ENABLE     0
#elif M1_DEADBAND_BRINGUP_PHASE == M1_DB_BRINGUP_SPEED_OFF
#define M1_SPEED_REVERSAL_TEST_ENABLE     0
#else
#define M1_SPEED_REVERSAL_TEST_ENABLE     0
#endif
#endif
#if M1_SPEED_REVERSAL_TEST_ENABLE
#ifndef M1_SPEED_REVERSAL_RPM
#define M1_SPEED_REVERSAL_RPM             300.0f
#endif
#ifndef M1_SPEED_REVERSAL_HOLD_S
#define M1_SPEED_REVERSAL_HOLD_S          10.0f
#endif
#ifndef M1_SPEED_REVERSAL_REPEAT
#define M1_SPEED_REVERSAL_REPEAT          1   /* 1=+/- 循环。=各跑一档后保持末档 */
#endif
#endif /* M1_SPEED_REVERSAL_TEST_ENABLE */
#endif /* M1_SPEED_LOOP_ENABLE */

#ifndef M1_FOC_ROTATION_FF_ENABLE
#define M1_FOC_ROTATION_FF_ENABLE       0
#endif

/** VOFA JustFloat 通道数与分频（JEOC 20kHz 基准，D=2。0kHz / D=4。kHz / D=10。kHz 帧率。*/
#ifndef M1_TELEM_BRINGUP_K
#define M1_TELEM_BRINGUP_K              12u
#endif
#ifndef M1_TELEM_BRINGUP_DECIMATION
#define M1_TELEM_BRINGUP_DECIMATION     2u
#endif
#if M1_POS_MIT_COMBO_ENABLE
#undef M1_TELEM_BRINGUP_DECIMATION
#define M1_TELEM_BRINGUP_DECIMATION     4u    /* MIT。kHz。0kHz/4。*/
#endif

/**
 * 1=bringup 全阶段固。VOFA×12（Id cal / ident / 开环阶梯同布局，不再按 open_seq 。ch3。1）。
 * ch0=Ia ch1=Ib ch2=Ic ch3=Id ch4=Iq ch5=θ_el
 * M1_VOFA_IDENT_DUTY_12CH=1（辨。标定，默认）：ch6=Vd_est ch7=Vq_est ch8=Ta ch9=Tb ch10=Tc ch11=open_seq。
 *   VASI(open_seq=57) 。ch8=grid ch9=proc_code ch10=L_est_uH（ch6/7 仍为端电压估计）。
 * M1_VOFA_IDENT_DUTY_12CH=0：ch6=Ud_out ch7=Uq_out。
 *   M1_VOFA_MIT_CH8_11=1 。ch8=ω_pll ch9=θ_err ch10=Iq_ref ch11=open_seq（MIT 验收）；
 *   M1_VOFA_PLL_CH8_11=1 。ch8=ω_pll ch9=ω_diff ch10=θ_err ch11=Δω。
 *   M1_VOFA_PLL_CH8_11=0 。ch8=Id_ref ch9=Iq_ref ch10=duty_dev ch11=open_seq。
 */
#ifndef M1_VOFA_UNIFIED_12CH
#define M1_VOFA_UNIFIED_12CH            1
#endif

/** 1=Park/VOFA 。-θ_enc。=。SVPWM 。+θ（与 Core/Inc/main.h 同名宏兼容） */
#ifndef M1_THETA_NEGATE
#define M1_THETA_NEGATE     0
#endif

/** 母线电压（V），限幅。Vbus/。 */
#define M1_VBUS_V           24.0f
#define M1_PI_V_MAX         (M1_VBUS_V * 0.577350269f)
#define M1_PI_V_MIN         (-M1_PI_V_MAX)

/** PI 带宽（Hz）；M1_PI_USE_FIXED_GAIN=0 时用。L×ωc 整定（阶跃验收参。1000 Hz。*/
#define M1_PI_FC_HZ         1000.0f
#define M1_PI_WC_RADS       (2.0f * 3.14159265359f * M1_PI_FC_HZ)

/** 联调。=固定 Kp/Ki（试凑）。=。fc 整定 */
#ifndef M1_PI_USE_FIXED_GAIN
#define M1_PI_USE_FIXED_GAIN  0
#endif

#if M1_PI_USE_FIXED_GAIN
#define M1_PI_KP_ID         2.0f
#define M1_PI_KP_IQ         2.0f
#define M1_PI_KI            0.0002f
#else
#define M1_PI_KP_ID         (M1_LD_H * M1_PI_WC_RADS)
#define M1_PI_KP_IQ         (M1_LQ_H * M1_PI_WC_RADS)
#define M1_PI_KI_NOMINAL    (M1_RS_OHM * M1_PI_WC_RADS * M1_CTRL_TS_S)
#ifndef M1_PI_KI_SCALE
#define M1_PI_KI_SCALE      1.0f
#endif
#define M1_PI_KI            (M1_PI_KI_NOMINAL * M1_PI_KI_SCALE)
#endif

/**
 * 首次闭环联调。4V/0.8A 电源）：降低 dq 电压限幅，ref 硬钳位。
 * 稳定后设 M1_CLOSURE_BRINGUP=0 恢复满幅 SVPWM 线性区。
 */
#ifndef M1_CLOSURE_BRINGUP
#if M1_SPEED_LOOP_ENABLE
#define M1_CLOSURE_BRINGUP  0   /* 速度环联调：满幅 Iq/Uq */
#else
#define M1_CLOSURE_BRINGUP  1
#endif
#endif

#if M1_CLOSURE_BRINGUP
#define M1_PI_V_LIMIT_V     6.0f
#define M1_I_REF_ABS_MAX    0.5f
#else
#define M1_PI_V_LIMIT_V     M1_PI_V_MAX
#if (M1_BRINGUP_MODE == M1_BRINGUP_MODE_SPEED_IDENT)
#define M1_I_REF_ABS_MAX    11.0f
#else
#define M1_I_REF_ABS_MAX    5.0f
#endif
#endif

#define M1_PI_V_LIMIT_MIN   (-M1_PI_V_LIMIT_V)

/** 积分器限幅（V），与输出限幅同量级，防 windup */
#define M1_PI_INT_LIMIT_V   M1_PI_V_LIMIT_V
#define M1_PI_INT_LIMIT_MIN (-M1_PI_INT_LIMIT_V)

#if M1_SPEED_LOOP_ENABLE
/** 速度。Iq_ref 限幅 [A]（与辨识分轨；SPEED_IDENT 实验 11 A。*/
#ifndef M1_SPEED_IQ_REF_ABS_MAX
#if (M1_BRINGUP_MODE == M1_BRINGUP_MODE_SPEED_IDENT)
#define M1_SPEED_IQ_REF_ABS_MAX         11.0f
#else
#define M1_SPEED_IQ_REF_ABS_MAX         M1_I_REF_ABS_MAX
#endif
#endif
#define M1_SPEED_PI_OUT_MAX             M1_SPEED_IQ_REF_ABS_MAX
#define M1_SPEED_PI_OUT_MIN             (-M1_SPEED_IQ_REF_ABS_MAX)
#define M1_SPEED_PI_INT_MAX             M1_SPEED_IQ_REF_ABS_MAX
#define M1_SPEED_PI_INT_MIN             (-M1_SPEED_IQ_REF_ABS_MAX)
#endif

/** 电流。Iq 目标（A）；无启动策略时上电即用，手拨启动（Iq 探路/日常。.5 A。*/
#ifndef M1_IQ_REF_A
#define M1_IQ_REF_A         0.5f
#endif

/** 0=。ALIGN/DRAG，编码器 θ 直接闭环。=启动状态机 */
#ifndef M1_STARTUP_ENABLE
#define M1_STARTUP_ENABLE   0
#endif
#ifndef M1_IF_ENABLE
#define M1_IF_ENABLE                    0
#endif
#ifndef M1_IF_ALIGN_S
#define M1_IF_ALIGN_S                   0.2f
#endif
#ifndef M1_IF_IQ_A
#define M1_IF_IQ_A                      0.5f
#endif
#ifndef M1_IF_ID_A
#define M1_IF_ID_A                      0.0f
#endif
#ifndef M1_IF_TARGET_RPM
#define M1_IF_TARGET_RPM                100.0f
#endif
#ifndef M1_IF_RAMP_S
#define M1_IF_RAMP_S                    3.0f
#endif
#ifndef M1_IF_HOLD_S
#define M1_IF_HOLD_S                    0.0f
#endif
/** 1=RAMP 中速带。Iq，压 200。00 rpm 丢步 */
#ifndef M1_IF_IQ_MID_BOOST_ENABLE
#define M1_IF_IQ_MID_BOOST_ENABLE       0
#endif
#ifndef M1_IF_IQ_MID_A
#define M1_IF_IQ_MID_A                  5.5f
#endif
#ifndef M1_IF_IQ_MID_RPM_LO
#define M1_IF_IQ_MID_RPM_LO             180.0f
#endif
#ifndef M1_IF_IQ_MID_RPM_HI
#define M1_IF_IQ_MID_RPM_HI             520.0f
#endif
#ifndef M1_IF_TO_OBS_ENABLE
#define M1_IF_TO_OBS_ENABLE             0
#endif
/**
 * 1=编码器可选：控制。I/F→SMO，编码器只供 VOFA 监督。
 * 插着可看 ch0/θ_err；拔掉不挡启动与巡航（须 IF_TO_OBS）。
 */
#ifndef M1_ENC_OPTIONAL_ENABLE
#define M1_ENC_OPTIONAL_ENABLE          0
#endif
#ifndef M1_IF_OBS_SPEED_REF_RPM
#define M1_IF_OBS_SPEED_REF_RPM         1000.0f
#endif
/** 1=交接。ω_ref 钉当前测速；0=再抬。M1_IF_OBS_SPEED_REF_RPM */
#ifndef M1_IF_OBS_HOLD_SPEED_ENABLE
#define M1_IF_OBS_HOLD_SPEED_ENABLE     0
#endif
/** 1=HOLD 时钉 I/F 指令速（防认飞车）；须同。HOLD_SPEED=1 */
#ifndef M1_IF_OBS_HOLD_IF_CMD_ENABLE
#define M1_IF_OBS_HOLD_IF_CMD_ENABLE    0
#endif
/** 1=只切角：。OBS 后固。Iq、不跑速度环（角切泡开验收。*/
#ifndef M1_IF_OBS_ANGLE_ONLY_ENABLE
#define M1_IF_OBS_ANGLE_ONLY_ENABLE     0
#endif
/** 1=BLEND 期间保持 M1_IF_IQ_A（不收到 HANDOFF。*/
#ifndef M1_IF_OBS_BLEND_KEEP_IF_IQ
#define M1_IF_OBS_BLEND_KEEP_IF_IQ      0
#endif
/** 1=进入 BLEND 即弱速度环（ω_ref=ω_IF），抑角融阶段飞。*/
#ifndef M1_IF_OBS_BLEND_SPEED_ENABLE
#define M1_IF_OBS_BLEND_SPEED_ENABLE    0
#endif
/** 1=。OBS 后短时限幅速度 PI 。|e_ω|（抑 enc 测速尖峰） */
#ifndef M1_IF_OBS_EW_CLAMP_ENABLE
#define M1_IF_OBS_EW_CLAMP_ENABLE       0
#endif
#ifndef M1_IF_OBS_EW_CLAMP_RPM
#define M1_IF_OBS_EW_CLAMP_RPM          40.0f
#endif
#ifndef M1_IF_OBS_EW_CLAMP_S
#define M1_IF_OBS_EW_CLAMP_S            0.20f
#endif
/**
 * 交接短时「浅刹车」：。iq_ref 下限，防 1240 式大负电流砸停失锁。
 * 窗口结束后恢。M1_SPEED_PI_OUT_MIN（允许正常制动）。
 */
#ifndef M1_IF_OBS_SOFT_BRAKE_ENABLE
#define M1_IF_OBS_SOFT_BRAKE_ENABLE     0
#endif
#ifndef M1_IF_OBS_IQ_MIN_A
#define M1_IF_OBS_IQ_MIN_A              (-0.4f)
#endif
#ifndef M1_IF_OBS_SOFT_REGEN_IQ_A
#define M1_IF_OBS_SOFT_REGEN_IQ_A       (-M1_IF_OBS_IQ_MIN_A) /* |regen| 浅刹 */
#endif
#ifndef M1_IF_OBS_SOFT_BRAKE_S
#define M1_IF_OBS_SOFT_BRAKE_S          2.0f
#endif
/**
 * 机械转向。1 正转 / -1 反转。profile 只改此符号与 |转速| 目标。
 * 勿再使用 M1_IF_OBS_REVERSE_ENABLE 分叉逻辑。
 */
#ifndef M1_IF_DIR_SIGN
#define M1_IF_DIR_SIGN                  (1.0f)
#endif
/* 兼容旧宏：仅文档/检索；代码路径应走 DIR_SIGN */
#ifndef M1_IF_OBS_REVERSE_ENABLE
#define M1_IF_OBS_REVERSE_ENABLE        0
#endif
/**
 * 速切站稳后分三步进入巡航。347：一步放开会砸停）。
 *   。ω_ref→CRUISE_RPM，仍浅刹。。PI
 *   。撤浅刹车，对。±Iq
 *   。正常 Kp/Ki
 */
#ifndef M1_IF_OBS_CRUISE_ENABLE
#define M1_IF_OBS_CRUISE_ENABLE         0
#endif
#ifndef M1_IF_OBS_CRUISE_SETTLE_S
#define M1_IF_OBS_CRUISE_SETTLE_S       1.5f /* spd_on 。。*/
#endif
#ifndef M1_IF_OBS_CRUISE_STAGE1_S
#define M1_IF_OBS_CRUISE_STAGE1_S       2.5f /* 。持续时间 */
#endif
#ifndef M1_IF_OBS_CRUISE_STAGE2_S
#define M1_IF_OBS_CRUISE_STAGE2_S       1.5f /* 。持续时间 */
#endif
#ifndef M1_IF_OBS_CRUISE_RPM
#define M1_IF_OBS_CRUISE_RPM            M1_IF_OBS_SPEED_REF_RPM
#endif
/**
 * 正→滑行停→反：。正转巡航 soak 。Iq=0 不管。。|ω|。 。。I/F 反转。−|ω|。
 * @note 开后应。STEP/S3；LOCK_STAGE1=1 避免正转段进②。
 */
#ifndef M1_IF_OBS_DIR_SEQ_ENABLE
#define M1_IF_OBS_DIR_SEQ_ENABLE        0
#endif
#ifndef M1_IF_OBS_DIR_SEQ_FWD_HOLD_S
#define M1_IF_OBS_DIR_SEQ_FWD_HOLD_S    3.0f /* 正转巡航站稳后再松手 */
#endif
#ifndef M1_IF_OBS_DIR_SEQ_ZERO_RPM
#define M1_IF_OBS_DIR_SEQ_ZERO_RPM      80.0f /* |ω_fb| 低于此算近零 */
#endif
#ifndef M1_IF_OBS_DIR_SEQ_ZERO_HOLD_S
#define M1_IF_OBS_DIR_SEQ_ZERO_HOLD_S   0.5f /* 近零再保持片刻再反起 */
#endif
#ifndef M1_IF_OBS_DIR_SEQ_COAST_MIN_S
#define M1_IF_OBS_DIR_SEQ_COAST_MIN_S   2.0f /* 松手后再开近零判定，防 enc 假零 */
#endif
#ifndef M1_IF_OBS_DIR_SEQ_COAST_MAX_S
#define M1_IF_OBS_DIR_SEQ_COAST_MAX_S   8.0f /* ω̂ 假挂时超时强制反起（2144。*/
#endif
#ifndef M1_IF_OBS_DIR_SEQ_EMAG_STOP
#define M1_IF_OBS_DIR_SEQ_EMAG_STOP     0.55f /* Iq=0 。emag 掉到噪声≈已停；勿信。ω̂ */
#endif
#ifndef M1_IF_OBS_DIR_SEQ_REV_HOLD_S
#define M1_IF_OBS_DIR_SEQ_REV_HOLD_S    3.0f /* 反转巡航后结束（不再二次反） */
#endif
/**
 * 。。站稳后自。ω_ref 阶跃（验 SMO 可变速）。权威仍。LOCK_STAGE* 管。
 * 无弱磁：目标夹在 LO..HI（默。900..1300）。
 * 序列：soak@CRUISE 。HI 。CRUISE 。LO 。HI 。LO 。CRUISE。
 */
#ifndef M1_IF_OBS_CRUISE_STEP_ENABLE
#define M1_IF_OBS_CRUISE_STEP_ENABLE    0
#endif
#ifndef M1_IF_OBS_CRUISE_STEP_SOAK_S
#define M1_IF_OBS_CRUISE_STEP_SOAK_S    2.0f
#endif
#ifndef M1_IF_OBS_CRUISE_STEP_HOLD_S
#define M1_IF_OBS_CRUISE_STEP_HOLD_S    4.0f /* 大阶。减速靠 regen 需更长 */
#endif
#ifndef M1_IF_OBS_CRUISE_STEP_LO_RPM
#define M1_IF_OBS_CRUISE_STEP_LO_RPM    900.0f
#endif
#ifndef M1_IF_OBS_CRUISE_STEP_HI_RPM
#define M1_IF_OBS_CRUISE_STEP_HI_RPM    1300.0f
#endif
#ifndef M1_IF_OBS_CRUISE_STEP_RPM
#define M1_IF_OBS_CRUISE_STEP_RPM       100.0f /* 旧Δ；表驱后仅兼容保留 */
#endif
/**
 * 。iq_min 到位后同向硬减速探针：基→HI→LO→基，。Iq 。STAGE3 地板。
 * 须在阶跃表完成且已进③之后跑（与 STEP 表错开）。
 */
#ifndef M1_IF_OBS_CRUISE_S3_PROBE_ENABLE
#define M1_IF_OBS_CRUISE_S3_PROBE_ENABLE 0
#endif
#ifndef M1_IF_OBS_CRUISE_S3_PROBE_SOAK_S
#define M1_IF_OBS_CRUISE_S3_PROBE_SOAK_S 1.5f
#endif
#ifndef M1_IF_OBS_CRUISE_S3_PROBE_HOLD_S
#define M1_IF_OBS_CRUISE_S3_PROBE_HOLD_S 4.0f
#endif
#ifndef M1_IF_OBS_CRUISE_PI_KP
#define M1_IF_OBS_CRUISE_PI_KP          M1_SPEED_PI_KP
#endif
#ifndef M1_IF_OBS_CRUISE_PI_KI
#define M1_IF_OBS_CRUISE_PI_KI          M1_SPEED_PI_KI
#endif
#ifndef M1_IF_OBS_CRUISE_IQ_ABS_MAX
#define M1_IF_OBS_CRUISE_IQ_ABS_MAX     8.0f
#endif
/**
 * 1=巡航锁在①（open_seq=246）：不进。③，并持续续期浅刹车。
 * B0/B1 验收用；放开权威前必须为 0。
 */
#ifndef M1_IF_OBS_CRUISE_LOCK_STAGE1_ENABLE
#define M1_IF_OBS_CRUISE_LOCK_STAGE1_ENABLE 0
#endif
/**
 * 1=进②后不再升档（停在 iq_min=STAGE2）。
 */
#ifndef M1_IF_OBS_CRUISE_LOCK_STAGE2_ENABLE
#define M1_IF_OBS_CRUISE_LOCK_STAGE2_ENABLE 0
#endif
/**
 * 1=进③（第二档 iq_min）后不再进正。PI。
 */
#ifndef M1_IF_OBS_CRUISE_LOCK_STAGE3_ENABLE
#define M1_IF_OBS_CRUISE_LOCK_STAGE3_ENABLE 0
#endif
/**
 * ①→。/ ②→。门控（时间只。T_min；不满足则永不硬升）。
 */
#ifndef M1_IF_OBS_CRUISE_GATE_ENABLE
#define M1_IF_OBS_CRUISE_GATE_ENABLE    0
#endif
#ifndef M1_IF_OBS_CRUISE_GATE_T_MIN_S
#define M1_IF_OBS_CRUISE_GATE_T_MIN_S   2.0f
#endif
/** 。内最短停留再门控升③ */
#ifndef M1_IF_OBS_CRUISE_GATE_T_MIN2_S
#define M1_IF_OBS_CRUISE_GATE_T_MIN2_S  1.5f
#endif
#ifndef M1_IF_OBS_CRUISE_GATE_HOLD_S
#define M1_IF_OBS_CRUISE_GATE_HOLD_S    0.5f
#endif
#ifndef M1_IF_OBS_CRUISE_GATE_ERR_RPM
#define M1_IF_OBS_CRUISE_GATE_ERR_RPM   60.0f /* 1442。0 。20kHz 单拍易抖。*/
#endif
#ifndef M1_IF_OBS_CRUISE_GATE_DOMEGA_MAX
#define M1_IF_OBS_CRUISE_GATE_DOMEGA_MAX 1200.0f /* rpm/s，长。*/
#endif
#ifndef M1_IF_OBS_CRUISE_GATE_DOMEGA_WIN_S
#define M1_IF_OBS_CRUISE_GATE_DOMEGA_WIN_S 0.10f
#endif
/** 门控。|eω| 一阶滤波截止；。20 kHz 单拍尖峰 */
#ifndef M1_IF_OBS_CRUISE_GATE_EW_LPF_HZ
#define M1_IF_OBS_CRUISE_GATE_EW_LPF_HZ  20.0f
#endif
/**
 * 不合格时 hold 泄漏倍率（相。dt）：hold -= LEAK*dt，而非清零。
 * 1=对称减；3=掉得比涨快，仍抗单拍毛刺。
 */
#ifndef M1_IF_OBS_CRUISE_GATE_FAIL_LEAK
#define M1_IF_OBS_CRUISE_GATE_FAIL_LEAK  2.0f
#endif
/**
 * 晃幅门控（离。1453/1503）：升权威前要求 std / 近似峰峰 / |eω| 达标。
 * 1503 证明。|eω| 均值不够。
 */
#ifndef M1_IF_OBS_CRUISE_AMP_GATE_ENABLE
#define M1_IF_OBS_CRUISE_AMP_GATE_ENABLE 0
#endif
#ifndef M1_IF_OBS_CRUISE_AMP_STD_MAX
#define M1_IF_OBS_CRUISE_AMP_STD_MAX     38.0f
#endif
#ifndef M1_IF_OBS_CRUISE_AMP_PTP_MAX
#define M1_IF_OBS_CRUISE_AMP_PTP_MAX     98.0f /* ≈p5–p95 代理。.5·std */
#endif
#ifndef M1_IF_OBS_CRUISE_AMP_EW_MAX
#define M1_IF_OBS_CRUISE_AMP_EW_MAX      58.0f
#endif
#ifndef M1_IF_OBS_CRUISE_AMP_EMA_HZ
#define M1_IF_OBS_CRUISE_AMP_EMA_HZ      2.0f
#endif
/** iq_min 斜坡 [A/s]：权威禁止阶。*/
#ifndef M1_IF_OBS_CRUISE_IQ_MIN_SLEW_A_S
#define M1_IF_OBS_CRUISE_IQ_MIN_SLEW_A_S 0.2f /* 。.25→−0.6 。1.75s */
#endif
/** 驱动 |Iq| 与分。|regen|（符号由 M1_IF_DIR_SIGN 映射。*/
#ifndef M1_IF_OBS_CRUISE_DRIVE_IQ_A
#define M1_IF_OBS_CRUISE_DRIVE_IQ_A     3.5f
#endif
#ifndef M1_IF_OBS_CRUISE_STAGE2_REGEN_IQ_A
#define M1_IF_OBS_CRUISE_STAGE2_REGEN_IQ_A 0.6f
#endif
#ifndef M1_IF_OBS_CRUISE_STAGE3_REGEN_IQ_A
#define M1_IF_OBS_CRUISE_STAGE3_REGEN_IQ_A 0.9f
#endif
/** 兼容旧名（按正向语义）；运行时请。DRIVE/REGEN + DIR */
#ifndef M1_IF_OBS_CRUISE_STAGE2_IQ_MIN_A
#define M1_IF_OBS_CRUISE_STAGE2_IQ_MIN_A (-M1_IF_OBS_CRUISE_STAGE2_REGEN_IQ_A)
#endif
#ifndef M1_IF_OBS_CRUISE_STAGE2_IQ_MAX_A
#define M1_IF_OBS_CRUISE_STAGE2_IQ_MAX_A M1_IF_OBS_CRUISE_DRIVE_IQ_A
#endif
#ifndef M1_IF_OBS_CRUISE_STAGE3_IQ_MIN_A
#define M1_IF_OBS_CRUISE_STAGE3_IQ_MIN_A (-M1_IF_OBS_CRUISE_STAGE3_REGEN_IQ_A)
#endif
#ifndef M1_IF_OBS_CRUISE_STAGE3_IQ_MAX_A
#define M1_IF_OBS_CRUISE_STAGE3_IQ_MAX_A M1_IF_OBS_CRUISE_DRIVE_IQ_A
#endif
/**
 * 速度环主动阻尼（空载收晃）。
 * HP=1：iq -= Bd·(ω−ω_lpf)——只阻尼交流，不与浅刹车抢欠速半周（1422 绝对粘滞。Fail）。
 * HP=0：iq -= Bd·ω_lpf——绝对粘滞；非对。iq_min 下勿用。
 * LPF 截止须明显低于猎振频率（~1 Hz 。建议 0.3 Hz）。
 */
#ifndef M1_IF_OBS_DAMP_ENABLE
#define M1_IF_OBS_DAMP_ENABLE           0
#endif
#ifndef M1_IF_OBS_DAMP_HP_ENABLE
#define M1_IF_OBS_DAMP_HP_ENABLE        1
#endif
#ifndef M1_IF_OBS_DAMP_BD
#define M1_IF_OBS_DAMP_BD               0.002f /* A/rpm；HP 时乘。(ω−ω_lpf) 。*/
#endif
#ifndef M1_IF_OBS_DAMP_LPF_HZ
#define M1_IF_OBS_DAMP_LPF_HZ           0.35f /* 提均值；。<< 猎振。*/
#endif
/** BLEND 。/ 交接 bumpless 目标 Iq（从 M1_IF_IQ_A 线性收到此值） */
#ifndef M1_IF_HANDOFF_IQ_A
#define M1_IF_HANDOFF_IQ_A              0.8f
#endif
#ifndef M1_VOFA_IF_12CH
#define M1_VOFA_IF_12CH                 0
#endif
#if M1_IF_TO_OBS_ENABLE && !M1_IF_ENABLE
#error "M1_IF_TO_OBS_ENABLE requires M1_IF_ENABLE=1"
#endif
#if M1_IF_TO_OBS_ENABLE && !M1_OBS_SOFT_SWITCH_ENABLE
/* soft switch may be enabled later by profile; checked after profile include */
#endif

/* ==========================================================================
 * 联调模式（只改这一处，其余 M1_ID_* / M1_IDENT_* 由下面自动推导）
 *
 * 电感 Ld/Lq 三步联调（无端电压表）：
 *   。M1_BRINGUP_MODE_ID_CAL_PASS0_BUILD  。Pass0 闭环建表 。commit 。NVM
 *   。M1_BRINGUP_MODE_OPEN_UD_LUT_VERIFY  。开。Ud 阶梯 + LUT runtime 验表
 *   。M1_BRINGUP_MODE_RS_LD_LQ_ONLY       。VASI（INJECT 。LUT ON；Rs 固定。
 *
 *   M1_BRINGUP_MODE_NORMAL              日常 Iq 环，无自动序。
 *   M1_BRINGUP_MODE_IDENT_IQ_STEP       Pass0 30° 30。Id。.5A) 。Iq稳。OFF/LUT。.3A OFF 20s 。LUT ON。
 *   M1_BRINGUP_MODE_ID_CAL_DUAL_FULL    Pass0 三角 30°+150°+270° 。commit 。Pass1 验表
 *   M1_BRINGUP_MODE_ID_CAL_PASS0_ONLY   Pass0 双角 only（无 Pass1 / 。Iq 探路。
 *   M1_BRINGUP_MODE_ID_CAL_PASS0_RS      Pass0 30° 建表 。commit 。单轮 OFF Rs+VASI Ld/Lq
 *   M1_BRINGUP_MODE_RS_LD_LQ_ONLY        。VASI（Rs 固定；INJECT 。LUT ON；SETTLE OFF。
 *   M1_BRINGUP_MODE_OPEN_UD_LUT_VERIFY  。开。Ud 阶梯。LUT（Pass0 后单独烧录）
 *   M1_BRINGUP_MODE_ID_CAL_PASS0_BUILD   。Pass0 30° 建表 。commit 。NVM
 *   M1_BRINGUP_MODE_MULTI_ANGLE_PASS0   Pass0 五角 0/30/60/90/120° only（多角度 raw 录波。
 *   M1_BRINGUP_MODE_IQ_PROBE_OFF_ONLY   跳过建表：上电即 Iq+deadband OFF（PLL 联调。
 *   M1_BRINGUP_MODE_ID_CAL_PASS0_BUILD  Pass0 30° 建表 。commit 。VOFA LUT 突发 。DONE
 *   M1_BRINGUP_MODE_SPEED_IDENT         上电 deadband OFF 。速度阶跃 。Bode
 *   M1_BRINGUP_MODE_BODE_OFF_ONLY       无建。阶跃：HOLD 。Iq Bode OFF-lo/hi @6ch 20kHz
 *   M1_BRINGUP_MODE_BODE_ID_OFF_ONLY    同上，Id 。sin（Iq=0，。30°）ch3=Id ch4=Id_ref
 * （枚举与 M1_BRINGUP_MODE 默认见文件前部，M1_SPEED_LOOP_ENABLE 之前。
 * ========================================================================== */

#if (M1_BRINGUP_MODE == M1_BRINGUP_MODE_NORMAL)
#define M1_ID_LOCK_CAL_SWEEP            0
#define M1_IDENT_ENABLE                 0
#define M1_ID_CAL_MULTI_ANGLE_ENABLE    0
#define M1_ID_CAL_PASS0_ONLY_ENABLE     0
#define M1_ID_CAL_LUT_VERIFY_SWEEP     0
#define M1_ID_CAL_IQ_PROBE_ENABLE       0
#if M1_SPEED_LOOP_ENABLE && (M1_SPEED_DEADBAND_TEST == M1_SPEED_DEADBAND_TEST_OFF)
/** Run A：速度。deadband 效果测试 。LUT OFF */
#define M1_DEADBAND_LUT_BAKED_ENABLE    0
#define M1_DEADBAND_ENABLE              0
#elif M1_SPEED_LOOP_ENABLE && (M1_SPEED_DEADBAND_TEST == M1_SPEED_DEADBAND_TEST_LUT)
/** Run B：速度。deadband 效果测试 。baked LUT ON（abc duty，runtime_apply_ud=0。*/
#undef M1_DEADBAND_GEO_TWO_CLUSTER_ENABLE
#define M1_DEADBAND_GEO_TWO_CLUSTER_ENABLE  0
#define M1_DEADBAND_NVM_ON_BOOT         0
#define M1_DEADBAND_LUT_BAKED_ENABLE    1
#define M1_DEADBAND_LUT_RUNTIME_SCALE   1.0f
#define M1_DEADBAND_ENABLE              1
#else
/** 产品：两。plut + NVM 上电 LUT ON（标定一次后。NORMAL。*/
#ifndef M1_DEADBAND_GEO_TWO_CLUSTER_ENABLE
#define M1_DEADBAND_GEO_TWO_CLUSTER_ENABLE  1
#endif
#ifndef M1_DEADBAND_NVM_ON_BOOT
#define M1_DEADBAND_NVM_ON_BOOT         0
#endif
#define M1_DEADBAND_LUT_BAKED_ENABLE    1
#if M1_DEADBAND_LUT_BAKED_ENABLE
#ifndef M1_DEADBAND_GEO_TWO_CLUSTER_ENABLE
#define M1_DEADBAND_GEO_TWO_CLUSTER_ENABLE  0
#endif
#ifndef M1_DEADBAND_LUT_RUNTIME_SCALE
#define M1_DEADBAND_LUT_RUNTIME_SCALE   1.0f
#endif
#elif !defined(M1_DEADBAND_LUT_RUNTIME_SCALE)
#define M1_DEADBAND_LUT_RUNTIME_SCALE   0.25f
#endif
#ifndef M1_DEADBAND_ENABLE
#define M1_DEADBAND_ENABLE              1
#endif
#endif /* speed loop deadband off */

#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_ID_CAL_PASS0_BUILD)
/** 。Pass0 。。Ud 阶梯 OFF→ON 。。VASI Ld/Lq(INJECT LUT) 。LUT/ident 突发；同次上。RAM only */
#define M1_ID_LOCK_CAL_SWEEP            1
#define M1_IDENT_ENABLE                 0
#define M1_ID_CAL_MULTI_ANGLE_ENABLE    0
#define M1_ID_CAL_PASS0_ONLY_ENABLE     0
#define M1_ID_CAL_DUAL_ANGLE_ENABLE     0
#define M1_ID_CAL_FIX_THETA_ENABLE      1
#define M1_ID_CAL_ALIGN_ENABLE          1
#define M1_ID_CAL_COMMIT_LUT            1
#define M1_ID_CAL_LUT_VERIFY_SWEEP      0
#define M1_ID_CAL_IQ_PROBE_ENABLE       0
#define M1_RS_IDENT_ENABLE              0
#undef M1_RS_IDENT_USE_FIXED_NOMINAL
#define M1_RS_IDENT_USE_FIXED_NOMINAL   1
#define M1_LD_LQ_IDENT_ENABLE           1
#undef M1_LD_LQ_IDENT_ABORT_ON_THETA_DRIFT
#define M1_LD_LQ_IDENT_ABORT_ON_THETA_DRIFT  0
#undef M1_LD_LQ_MULTI_ANGLE_ENABLE
#define M1_LD_LQ_MULTI_ANGLE_ENABLE     0
#undef M1_LD_LQ_PRE_DECAY_S
#define M1_LD_LQ_PRE_DECAY_S            1.0f
#undef M1_LD_LQ_IDENT_OPEN_LOOP_ENABLE
#define M1_LD_LQ_IDENT_OPEN_LOOP_ENABLE   1
#undef M1_LD_LQ_IDENT_BODE_BIAS_GRID_ENABLE
#define M1_LD_LQ_IDENT_BODE_BIAS_GRID_ENABLE  0
#undef M1_LD_LQ_IDENT_FINE_GRID_ENABLE
#define M1_LD_LQ_IDENT_FINE_GRID_ENABLE  1
#undef M1_LD_LQ_IDENT_FINE_ONLY
#define M1_LD_LQ_IDENT_FINE_ONLY         0
/** 15 。+ 500 Hz coarse 。1 kHz fine（关 F2 / FINE_ONLY。*/
#undef M1_LD_LQ_IDENT_F_COARSE_HZ
#define M1_LD_LQ_IDENT_F_COARSE_HZ        500.0f
#undef M1_LD_LQ_IDENT_F_FINE_HZ
#define M1_LD_LQ_IDENT_F_FINE_HZ        1000.0f
#undef M1_LD_LQ_IDENT_F2_ENABLE
#define M1_LD_LQ_IDENT_F2_ENABLE        0
#undef M1_LD_LQ_IDENT_INJECT_LUT_ENABLE
#define M1_LD_LQ_IDENT_INJECT_LUT_ENABLE  1
#undef M1_LD_LQ_IDENT_L_NOM_H
#define M1_LD_LQ_IDENT_L_NOM_H            85e-6f
#undef M1_LD_LQ_IDENT_SETTLE_S
#define M1_LD_LQ_IDENT_SETTLE_S           1.0f
#undef M1_LD_LQ_IDENT_CYCLES_PER_AMP
#define M1_LD_LQ_IDENT_CYCLES_PER_AMP     15u
#undef M1_VOFA_IDENT_DUMP_ENABLE
#define M1_VOFA_IDENT_DUMP_ENABLE         1
#undef M1_DEADBAND_FLOW_LD_LQ_AFTER_OPEN_UD
#define M1_DEADBAND_FLOW_LD_LQ_AFTER_OPEN_UD  1
#undef M1_ID_CAL_AMP_TABLE_LEN
#define M1_ID_CAL_AMP_TABLE_LEN         30u
#undef M1_ID_CAL_I_MAX_A
#define M1_ID_CAL_I_MAX_A               1.5f
#undef M1_ID_CAL_ID_DWELL_S
#define M1_ID_CAL_ID_DWELL_S            0.5f
#undef M1_ID_CAL_ID_DWELL_LOW_S
#define M1_ID_CAL_ID_DWELL_LOW_S        0.5f
#undef M1_ID_CAL_I_REF_ABS_MAX
#define M1_ID_CAL_I_REF_ABS_MAX         3.5f
#undef M1_TELEM_BRINGUP_K
#define M1_TELEM_BRINGUP_K              12u
#undef M1_VOFA_LUT_DUMP_ENABLE
#if M1_DEADBAND_FLOW_ONE_SHOT
#define M1_VOFA_LUT_DUMP_ENABLE         0
#else
#define M1_VOFA_LUT_DUMP_ENABLE         1
#endif
#undef M1_DEADBAND_I_ZERO_DISABLE
#define M1_DEADBAND_I_ZERO_DISABLE      1
#define M1_DEADBAND_RUNTIME_GEO_ENABLE    0
#undef M1_DEADBAND_GEO_MERGE_VAL30_ONLY
#define M1_DEADBAND_GEO_MERGE_VAL30_ONLY    1
#undef M1_DEADBAND_GEO_TWO_CLUSTER_ENABLE
#define M1_DEADBAND_GEO_TWO_CLUSTER_ENABLE  0
#define M1_DEADBAND_LUT_BAKED_ENABLE    0
#define M1_DEADBAND_LUT_RUNTIME_SCALE   1.0f
#undef M1_DEADBAND_LUT_COMMIT_NORMALIZE
#define M1_DEADBAND_LUT_COMMIT_NORMALIZE  0
#undef M1_DEADBAND_LUT_APPLY_UD
#define M1_DEADBAND_LUT_APPLY_UD        0
#define M1_DEADBAND_ENABLE              1
#undef M1_DEADBAND_NVM_COMMIT_ENABLE
#define M1_DEADBAND_NVM_COMMIT_ENABLE   0
#undef M1_OPEN_UQ_PRE_ID_CAL_ENABLE
#define M1_OPEN_UQ_PRE_ID_CAL_ENABLE    0
#undef M1_OPEN_UD_PRE_ID_CAL_ENABLE
#define M1_OPEN_UD_PRE_ID_CAL_ENABLE    1
#undef M1_OPEN_UD_AFTER_ID_CAL
#define M1_OPEN_UD_AFTER_ID_CAL         1
#undef M1_OPEN_PRE_ID_LADDER_LUT_RUNTIME
#define M1_OPEN_PRE_ID_LADDER_LUT_RUNTIME  1
#undef M1_OPEN_PRE_ID_LADDER_AB_ENABLE
#define M1_OPEN_PRE_ID_LADDER_AB_ENABLE    1
#undef M1_OPEN_PRE_ID_LADDER_DWELL_S
#define M1_OPEN_PRE_ID_LADDER_DWELL_S       0.3f
#undef M1_VOFA_UNIFIED_12CH
#define M1_VOFA_UNIFIED_12CH            1
#undef M1_VOFA_IDENT_DUTY_12CH
#define M1_VOFA_IDENT_DUTY_12CH         1
#undef M1_PLL_ENABLE
#define M1_PLL_ENABLE                   0

#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_IDENT_IQ_STEP)
/** Id Pass0 30° 30。。commit 。Iq=0.3A OFF 20s 。LUT ON 稳态（无阶。Bode。*/
#define M1_IDENT_ID_CAL_BEFORE_STEP     1
#define M1_ID_LOCK_CAL_SWEEP            1
#define M1_IDENT_ENABLE                 0
#define M1_ID_CAL_MULTI_ANGLE_ENABLE    0
#define M1_ID_CAL_PASS0_ONLY_ENABLE     0
#define M1_ID_CAL_LUT_VERIFY_SWEEP      0
#define M1_ID_CAL_IQ_PROBE_ENABLE       1
#define M1_ID_CAL_IQ_PROBE_A            0.3f
#define M1_ID_CAL_IQ_PROBE_OFF_S        20.0f
#define M1_ID_CAL_IQ_PROBE_FIXED_S      0.0f
#define M1_ID_CAL_IQ_PROBE_ID_PI_ENABLE 0
#define M1_ID_CAL_DUAL_ANGLE_ENABLE     0
#define M1_ID_CAL_FIX_THETA_ENABLE      1
#define M1_ID_CAL_ALIGN_ENABLE          0
#define M1_ID_CAL_COMMIT_LUT            1
#undef M1_ID_CAL_AMP_TABLE_LEN
#define M1_ID_CAL_AMP_TABLE_LEN         30u
#undef M1_ID_CAL_I_MAX_A
#define M1_ID_CAL_I_MAX_A               1.5f
/** Pass0 单角 30°。.05~1.5 A @50 mA/档，0.5 s/档（。5 s。*/
#undef M1_ID_CAL_ID_DWELL_S
#define M1_ID_CAL_ID_DWELL_S            0.5f
#undef M1_ID_CAL_ID_DWELL_LOW_S
#define M1_ID_CAL_ID_DWELL_LOW_S        0.5f
#undef M1_DEADBAND_GEO_SAMPLE_MAX
#define M1_DEADBAND_GEO_SAMPLE_MAX      (M1_ID_CAL_AMP_TABLE_LEN * 3u)
#define M1_IDENT_IQ_STEP_ENABLE         1
#define M1_IDENT_IQ_BODE_ENABLE         1
/** 阶跃：低 4 。0 起点 + 。4 。1 A 基线（OFF/LUT 。2）；每轮 6 。× 1 s */
#undef M1_IDENT_STEP_BANDS
#define M1_IDENT_STEP_BANDS             2u
#undef M1_IDENT_STEP_ROUNDS_PER_PROFILE
#define M1_IDENT_STEP_ROUNDS_PER_PROFILE 2u
#undef M1_IDENT_STEP_ROUNDS
#define M1_IDENT_STEP_ROUNDS            (M1_IDENT_STEP_BANDS * 2u * M1_IDENT_STEP_ROUNDS_PER_PROFILE)
#undef M1_IDENT_STEP_OFF_ROUNDS
#define M1_IDENT_STEP_OFF_ROUNDS        M1_IDENT_STEP_ROUNDS_PER_PROFILE
#undef M1_IDENT_STEP_FIXED_ROUNDS
#define M1_IDENT_STEP_FIXED_ROUNDS      0u
#undef M1_IDENT_STEP_I0_A
#define M1_IDENT_STEP_I0_A              0.0f   /* 低段回零 */
#undef M1_IDENT_STEP_I1_A
#define M1_IDENT_STEP_I1_A              0.3f   /* 低段：小 */
#undef M1_IDENT_STEP_I2_A
#define M1_IDENT_STEP_I2_A              0.5f   /* 低段：中 */
#undef M1_IDENT_STEP_I3_A
#define M1_IDENT_STEP_I3_A              1.0f   /* 低段：大 */
#undef M1_IDENT_STEP_I_BASE_HI_A
#define M1_IDENT_STEP_I_BASE_HI_A       1.0f   /* 高段基线 */
#undef M1_IDENT_STEP_I5_A
#define M1_IDENT_STEP_I5_A              1.3f   /* 高段。.0。.3 */
#undef M1_IDENT_STEP_I6_A
#define M1_IDENT_STEP_I6_A              1.5f   /* 高段。.0。.5 */
#undef M1_IDENT_STEP_I7_A
#define M1_IDENT_STEP_I7_A              2.0f   /* 高段。.0。.0 */
/** 阶跃各档 dwell 1 s（含回基。回零），便于 Ts/稳态验。*/
#undef M1_IDENT_STEP_DWELL_S
#define M1_IDENT_STEP_DWELL_S           1.0f
#undef M1_IDENT_STEP_ZERO_DWELL_S
#define M1_IDENT_STEP_ZERO_DWELL_S      1.0f
/** Bode 方案 D。.25/1.25 A 。OFF/LUT = 4 轮；57 。10。570 Hz。0 cycles/f */
#undef M1_IDENT_BODE_BANDS
#define M1_IDENT_BODE_BANDS             2u
#undef M1_IDENT_BODE_ROUNDS
#define M1_IDENT_BODE_ROUNDS            (M1_IDENT_BODE_BANDS * 2u)
#undef M1_IDENT_BODE_OFF_ROUNDS
#define M1_IDENT_BODE_OFF_ROUNDS        1u
#undef M1_IDENT_BODE_FIXED_ROUNDS
#define M1_IDENT_BODE_FIXED_ROUNDS      0u
#undef M1_IDENT_BODE_I_BIAS_A
#define M1_IDENT_BODE_I_BIAS_A          0.25f
#undef M1_IDENT_BODE_I_AMP_A
#define M1_IDENT_BODE_I_AMP_A           0.05f
#undef M1_IDENT_BODE_I_BIAS_HI_A
#define M1_IDENT_BODE_I_BIAS_HI_A       1.25f  /* 。I 。Bode（不。1.0 A。*/
#undef M1_IDENT_BODE_I_AMP_HI_A
#define M1_IDENT_BODE_I_AMP_HI_A        0.10f
#undef M1_IDENT_BODE_CYCLES_PER_FREQ
#define M1_IDENT_BODE_CYCLES_PER_FREQ   20.0f
#undef M1_IDENT_BODE_F0_HZ
#define M1_IDENT_BODE_F0_HZ             10.0f
#undef M1_IDENT_BODE_F1_HZ
#define M1_IDENT_BODE_F1_HZ             1500.0f
#undef M1_IDENT_BODE_F_RATIO
#define M1_IDENT_BODE_F_RATIO           1.15f   /* 10 ~ F_SPLIT 以下 */
#undef M1_IDENT_BODE_F_SPLIT_HZ
#define M1_IDENT_BODE_F_SPLIT_HZ        200.0f
#undef M1_IDENT_BODE_F_RATIO_HI
#define M1_IDENT_BODE_F_RATIO_HI        1.06f   /* F_SPLIT ~ F1，方。D 密扫 crossover 。*/
/** 阶跃/Bode 。Pass0 。30° 锁轴（Iq@30°，非编码器旋转） */
#undef M1_IDENT_FIX_THETA_ENABLE
#define M1_IDENT_FIX_THETA_ENABLE       1
#undef M1_IDENT_THETA_EL_RAD
#define M1_IDENT_THETA_EL_RAD           M1_ID_CAL_THETA_EL_RAD
#define M1_DEADBAND_LUT_BAKED_ENABLE    0
#ifndef M1_DEADBAND_GEO_TWO_CLUSTER_ENABLE
#define M1_DEADBAND_GEO_TWO_CLUSTER_ENABLE  0
#endif
#define M1_DEADBAND_LUT_RUNTIME_SCALE   1.0f
#ifndef M1_DEADBAND_LUT_COMMIT_NORMALIZE
#define M1_DEADBAND_LUT_COMMIT_NORMALIZE  1
#endif
#ifndef M1_DEADBAND_NVM_COMMIT_ENABLE
#define M1_DEADBAND_NVM_COMMIT_ENABLE   0
#endif
#undef M1_DEADBAND_LUT_APPLY_UD
#define M1_DEADBAND_LUT_APPLY_UD        0
#undef M1_OPEN_UQ_PRE_ID_CAL_ENABLE
#define M1_OPEN_UQ_PRE_ID_CAL_ENABLE       0
#undef M1_OPEN_UD_PRE_ID_CAL_ENABLE
#define M1_OPEN_UD_PRE_ID_CAL_ENABLE       0
#define M1_IDENT_POST_BODE_OPEN_UQ_ENABLE  0

#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_ID_CAL_DUAL_FULL)
/** Pass0 三角 30/150/270° capture(138。 。commit 。Pass1 abc duty 验表（Pass1 可暂不录波） */
#define M1_ID_LOCK_CAL_SWEEP            1
#define M1_IDENT_ENABLE                 0
#define M1_ID_CAL_MULTI_ANGLE_ENABLE    1
#undef M1_ID_CAL_PASS0_ANGLE_COUNT
#define M1_ID_CAL_PASS0_ANGLE_COUNT     3
#define M1_ID_CAL_PASS0_ONLY_ENABLE     0
#define M1_ID_CAL_DUAL_ANGLE_ENABLE     1
#define M1_ID_CAL_FIX_THETA_ENABLE      1
#define M1_ID_CAL_ALIGN_ENABLE          1
#define M1_ID_CAL_COMMIT_LUT            1
#define M1_ID_CAL_LUT_VERIFY_SWEEP      1
#undef M1_ID_CAL_PASS1_USE_APPLY_DUTY
#define M1_ID_CAL_PASS1_USE_APPLY_DUTY  1
#define M1_ID_CAL_IQ_PROBE_ENABLE       0
#define M1_ID_CAL_IQ_PROBE_OFF_S        5.0f
#define M1_ID_CAL_IQ_PROBE_FIXED_S      0.0f
#define M1_ID_CAL_IQ_PROBE_ID_PI_ENABLE 0
#undef M1_ID_CAL_ID_DWELL_S
#define M1_ID_CAL_ID_DWELL_S            0.5f
#undef M1_ID_CAL_ID_DWELL_LOW_S
#define M1_ID_CAL_ID_DWELL_LOW_S        0.5f
#undef M1_ID_CAL_VERIFY_ID_DWELL_S
#define M1_ID_CAL_VERIFY_ID_DWELL_S     0.5f
/** Pass0-B @0° 专用 dwell。=。Pass0-A 相同。58 。Id 欠流验证。.0 s */
#undef M1_ID_CAL_PASS0_B_DWELL_S
#define M1_ID_CAL_PASS0_B_DWELL_S       1.0f
#undef M1_OPEN_UQ_PRE_ID_CAL_ENABLE
#define M1_OPEN_UQ_PRE_ID_CAL_ENABLE    0
#undef M1_OPEN_UD_PRE_ID_CAL_ENABLE
#define M1_OPEN_UD_PRE_ID_CAL_ENABLE    0
#undef M1_TELEM_BRINGUP_K
#define M1_TELEM_BRINGUP_K              12u
#undef M1_VOFA_LUT_DUMP_ENABLE
#define M1_VOFA_LUT_DUMP_ENABLE         1
/** 标定联调：关。|i|<I_ZERO 硬切，小电流连续查表 */
#undef M1_DEADBAND_I_ZERO_DISABLE
#define M1_DEADBAND_I_ZERO_DISABLE      1
/** 第一。A：d 。+ 当前 θ 。Park→abc duty。603 过补，先关回 plut+scale */
#define M1_DEADBAND_RUNTIME_GEO_ENABLE    0
/** 论文式单。merge plut。010 双角 median 。I 。。val 仅用 30° */
#undef M1_DEADBAND_GEO_MERGE_VAL30_ONLY
#define M1_DEADBAND_GEO_MERGE_VAL30_ONLY    1
/** Pass0-B 。capture（|u'|<阈值）不进 geo 。*/
#undef M1_ID_CAL_GEO_U_MIN_V
#define M1_ID_CAL_GEO_U_MIN_V               0.10f
#undef M1_DEADBAND_GEO_TWO_CLUSTER_ENABLE
#define M1_DEADBAND_GEO_TWO_CLUSTER_ENABLE  0
#define M1_DEADBAND_LUT_RUNTIME_SCALE   1.0f
#undef M1_DEADBAND_LUT_COMMIT_NORMALIZE
#define M1_DEADBAND_LUT_COMMIT_NORMALIZE  0
/** commit 后写 Flash（与 FOC 互斥，后续加状态机再开）；完整标定测试阶段。*/
#ifndef M1_DEADBAND_NVM_COMMIT_ENABLE
#define M1_DEADBAND_NVM_COMMIT_ENABLE   0
#endif
/** 0=归一化在 commit。=旧路。runtime AUTO scale */
#ifndef M1_DEADBAND_LUT_RUNTIME_SCALE_AUTO
#define M1_DEADBAND_LUT_RUNTIME_SCALE_AUTO  0
#endif

#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_IQ_PROBE_OFF_ONLY)
/** PLL/联调：无 Pass0；上电即 Id=0 Iq=探路电流 deadband OFF，编码器 θ Park，不。LUT */
#define M1_ID_LOCK_CAL_SWEEP                1
#define M1_IDENT_ENABLE                     0
#define M1_ID_CAL_IQ_PROBE_ONLY_ENABLE      1
#define M1_ID_CAL_IQ_PROBE_ENABLE           1
#define M1_ID_CAL_IQ_PROBE_LUT_AFTER_OFF    0
#define M1_ID_CAL_IQ_PROBE_A                0.4f
#define M1_ID_CAL_IQ_PROBE_OFF_S            0.0f   /* 0=一。OFF，手动停。*/
#define M1_ID_CAL_IQ_PROBE_FIXED_S          0.0f
#define M1_ID_CAL_IQ_PROBE_ID_PI_ENABLE     1
#define M1_ID_CAL_COMMIT_LUT                0
#define M1_ID_CAL_FIX_THETA_ENABLE          0
#define M1_ID_CAL_ALIGN_ENABLE              0
#define M1_ID_CAL_DUAL_ANGLE_ENABLE         0
#define M1_ID_CAL_MULTI_ANGLE_ENABLE        0
#define M1_ID_CAL_PASS0_ONLY_ENABLE         0
#define M1_ID_CAL_LUT_VERIFY_SWEEP          0
#define M1_RS_IDENT_ENABLE                  0
#define M1_LD_LQ_IDENT_ENABLE               0
#define M1_DEADBAND_LUT_BAKED_ENABLE        0
#ifndef M1_DEADBAND_ENABLE
#define M1_DEADBAND_ENABLE                  1
#endif

#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_ID_CAL_PASS0_RS)
/** Pass0 30° 30。。commit 。单轮 OFF Rs+VASI（死。OFF 辨识；DONE 后切 LUT runtime。*/
#define M1_ID_LOCK_CAL_SWEEP            1
#define M1_IDENT_ENABLE                 0
#define M1_ID_CAL_MULTI_ANGLE_ENABLE    0
#define M1_ID_CAL_PASS0_ONLY_ENABLE     0
#define M1_ID_CAL_LUT_VERIFY_SWEEP      0
#define M1_ID_CAL_IQ_PROBE_ENABLE       0
#define M1_RS_IDENT_ENABLE              1
#define M1_LD_LQ_IDENT_ENABLE           1
#undef M1_LD_LQ_IDENT_ABORT_ON_THETA_DRIFT
/** 0=θ 漂移只记 dbg。 。VASI 强制跑完（离线再筛）。=超限 ld_lq_abort */
#define M1_LD_LQ_IDENT_ABORT_ON_THETA_DRIFT  0
#undef M1_LD_LQ_MULTI_ANGLE_ENABLE
#define M1_LD_LQ_MULTI_ANGLE_ENABLE     0
#undef M1_LD_LQ_IDENT_ANGLE_COUNT
#define M1_LD_LQ_IDENT_ANGLE_COUNT      3u
#undef M1_LD_LQ_ALIGN_S
#define M1_LD_LQ_ALIGN_S                1.0f
#undef M1_LD_LQ_PRE_DECAY_S
#define M1_LD_LQ_PRE_DECAY_S            1.0f
#define M1_ID_CAL_DUAL_ANGLE_ENABLE     0
#define M1_ID_CAL_FIX_THETA_ENABLE      1
#define M1_ID_CAL_ALIGN_ENABLE          0
#define M1_ID_CAL_COMMIT_LUT            1
#undef M1_ID_CAL_AMP_TABLE_LEN
#define M1_ID_CAL_AMP_TABLE_LEN         30u
#undef M1_ID_CAL_I_MAX_A
#define M1_ID_CAL_I_MAX_A               1.5f
#undef M1_ID_CAL_ID_DWELL_S
#define M1_ID_CAL_ID_DWELL_S            0.5f
#undef M1_ID_CAL_ID_DWELL_LOW_S
#define M1_ID_CAL_ID_DWELL_LOW_S        0.5f
#undef M1_ID_CAL_I_REF_ABS_MAX
#define M1_ID_CAL_I_REF_ABS_MAX         3.5f
#undef M1_DEADBAND_GEO_SAMPLE_MAX
#define M1_DEADBAND_GEO_SAMPLE_MAX      (M1_ID_CAL_AMP_TABLE_LEN * 3u)
#undef M1_RS_IDENT_REPEAT_N
#define M1_RS_IDENT_REPEAT_N            2u
#undef M1_ID_CAL_PASS1_USE_APPLY_DUTY
#define M1_ID_CAL_PASS1_USE_APPLY_DUTY  0
/** 0=。OFF 。Rs+VASI。=commit 后再。LUT 轮对照（open_seq +100。*/
#undef M1_RS_L_IDENT_DUAL_LUT_ROUND_ENABLE
#define M1_RS_L_IDENT_DUAL_LUT_ROUND_ENABLE  0

#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_RS_LD_LQ_ONLY)
/** Pass0 已完成：ALIGN@30° 。HOLD 。VASI Ld/Lq 。DONE 。LUT runtime（Rs 固定 M1_RS_OHM。*/
#define M1_ID_LOCK_CAL_SWEEP            1
#define M1_IDENT_ENABLE                 0
#define M1_ID_CAL_RS_LD_LQ_ONLY_ENABLE  1
#define M1_ID_CAL_MULTI_ANGLE_ENABLE    0
#define M1_ID_CAL_PASS0_ONLY_ENABLE     0
#define M1_ID_CAL_LUT_VERIFY_SWEEP      0
#define M1_ID_CAL_IQ_PROBE_ENABLE       0
#undef M1_RS_IDENT_USE_FIXED_NOMINAL
#define M1_RS_IDENT_USE_FIXED_NOMINAL   1
#define M1_RS_IDENT_ENABLE              0
#define M1_LD_LQ_IDENT_ENABLE           1
#undef M1_LD_LQ_IDENT_ABORT_ON_THETA_DRIFT
#define M1_LD_LQ_IDENT_ABORT_ON_THETA_DRIFT  0
#undef M1_LD_LQ_MULTI_ANGLE_ENABLE
#define M1_LD_LQ_MULTI_ANGLE_ENABLE     0
#undef M1_LD_LQ_PRE_DECAY_S
#define M1_LD_LQ_PRE_DECAY_S            1.0f
#define M1_ID_CAL_DUAL_ANGLE_ENABLE     0
#define M1_ID_CAL_FIX_THETA_ENABLE      1
#define M1_ID_CAL_ALIGN_ENABLE          1
#define M1_ID_CAL_COMMIT_LUT            0
#undef M1_RS_IDENT_REPEAT_N
#define M1_RS_IDENT_REPEAT_N            2u
#undef M1_RS_L_IDENT_DUAL_LUT_ROUND_ENABLE
#define M1_RS_L_IDENT_DUAL_LUT_ROUND_ENABLE  0
#undef M1_VOFA_IDENT_DUMP_ENABLE
#define M1_VOFA_IDENT_DUMP_ENABLE         1
#undef M1_PLL_ENABLE
#define M1_PLL_ENABLE                   0
#undef M1_VOFA_PLL_CH8_11
#define M1_VOFA_PLL_CH8_11              0
#undef M1_DEADBAND_NVM_ON_BOOT
#define M1_DEADBAND_NVM_ON_BOOT         0
#undef M1_DEADBAND_ENABLE
#define M1_DEADBAND_ENABLE              1
/** 论文 §2.3：SETTLE 。PI 到偏。。注入段关 PI、冻。Ud/Uq + 对称 HF 方波（合力矩。。*/
#undef M1_LD_LQ_IDENT_OPEN_LOOP_ENABLE
#define M1_LD_LQ_IDENT_OPEN_LOOP_ENABLE   1
/** 0=论文 9 格；1=方案 A 15 。0。 A 加密 */
#undef M1_LD_LQ_IDENT_BODE_BIAS_GRID_ENABLE
#define M1_LD_LQ_IDENT_BODE_BIAS_GRID_ENABLE  0
#undef M1_LD_LQ_IDENT_FINE_GRID_ENABLE
#define M1_LD_LQ_IDENT_FINE_GRID_ENABLE  1
#undef M1_LD_LQ_IDENT_FINE_ONLY
#define M1_LD_LQ_IDENT_FINE_ONLY         0
/** 15 。+ 500 Hz coarse 。1 kHz fine（关 F2 / FINE_ONLY。*/
#undef M1_LD_LQ_IDENT_F_COARSE_HZ
#define M1_LD_LQ_IDENT_F_COARSE_HZ        500.0f
#undef M1_LD_LQ_IDENT_F_FINE_HZ
#define M1_LD_LQ_IDENT_F_FINE_HZ        1000.0f
#undef M1_LD_LQ_IDENT_F2_ENABLE
#define M1_LD_LQ_IDENT_F2_ENABLE        0
/** INJECT 开环段 abc duty LUT；SETTLE/PRE_DECAY 。OFF（PI 自补偿） */
#undef M1_LD_LQ_IDENT_INJECT_LUT_ENABLE
#define M1_LD_LQ_IDENT_INJECT_LUT_ENABLE  1

#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_OPEN_UD_LUT_VERIFY)
/** 。开。Ud 0/0.2/0.5/1/2/4 V 阶梯，deadband LUT runtime（验 Pass0 表） */
#define M1_ID_LOCK_CAL_SWEEP            0
#define M1_IDENT_ENABLE                 0
#define M1_SPEED_LOOP_ENABLE            0
#define M1_OPEN_UD_PRE_ID_CAL_ENABLE    1
#define M1_OPEN_UQ_PRE_ID_CAL_ENABLE    0
#undef M1_OPEN_PRE_ID_LADDER_LUT_RUNTIME
#define M1_OPEN_PRE_ID_LADDER_LUT_RUNTIME  1
#define M1_OPEN_PRE_ID_LADDER_FIX_THETA_ENABLE  1
#undef M1_DEADBAND_NVM_ON_BOOT
#define M1_DEADBAND_NVM_ON_BOOT         0
#define M1_DEADBAND_ENABLE              1
#undef M1_VOFA_UNIFIED_12CH
#define M1_VOFA_UNIFIED_12CH            1
#undef M1_PLL_ENABLE
#define M1_PLL_ENABLE                   0

#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_ID_CAL_PASS0_ONLY)
#define M1_ID_LOCK_CAL_SWEEP            1
#define M1_IDENT_ENABLE                 0
#define M1_ID_CAL_MULTI_ANGLE_ENABLE    0
#define M1_ID_CAL_PASS0_ONLY_ENABLE     1
#define M1_ID_CAL_LUT_VERIFY_SWEEP     0
#define M1_ID_CAL_IQ_PROBE_ENABLE       0

#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_MULTI_ANGLE_PASS0)
#define M1_ID_LOCK_CAL_SWEEP            1
#define M1_IDENT_ENABLE                 0
#define M1_ID_CAL_MULTI_ANGLE_ENABLE    1
#define M1_ID_CAL_PASS0_ONLY_ENABLE     1
#define M1_ID_CAL_LUT_VERIFY_SWEEP     0
#define M1_ID_CAL_IQ_PROBE_ENABLE       0

#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_BODE_OFF_ONLY)
/** 堵转 Iq Bode：无 Pass0/阶跃/LUT；HOLD 2s 。OFF-lo 0.25A 。OFF-hi 1.25A；VOFA×6 @20kHz
 *  频表 57 。F1=2500 Hz（单。~18 s @T_obs_hi=100 ms，两。~38 s）；勿用 legacy F1=800（仅 38 。846 Hz。*/
#define M1_ID_LOCK_CAL_SWEEP            0
#define M1_IDENT_ID_CAL_BEFORE_STEP     0
#define M1_IDENT_ENABLE                 1
#define M1_IDENT_IQ_STEP_ENABLE         0
#define M1_IDENT_IQ_BODE_ENABLE         1
#undef M1_IDENT_STEP_ROUNDS
#define M1_IDENT_STEP_ROUNDS            0u
#undef M1_IDENT_STEP_OFF_ROUNDS
#define M1_IDENT_STEP_OFF_ROUNDS        0u
#undef M1_IDENT_STEP_FIXED_ROUNDS
#define M1_IDENT_STEP_FIXED_ROUNDS      0u
#define M1_RS_IDENT_ENABLE              0
#define M1_LD_LQ_IDENT_ENABLE           0
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
#undef M1_IDENT_BODE_I_BIAS_A
#define M1_IDENT_BODE_I_BIAS_A          0.25f
#undef M1_IDENT_BODE_I_AMP_A
#define M1_IDENT_BODE_I_AMP_A           0.05f
#undef M1_IDENT_BODE_I_BIAS_HI_A
#define M1_IDENT_BODE_I_BIAS_HI_A       1.25f
#undef M1_IDENT_BODE_I_AMP_HI_A
#define M1_IDENT_BODE_I_AMP_HI_A        0.10f
#undef M1_IDENT_BODE_CYCLES_PER_FREQ
#define M1_IDENT_BODE_CYCLES_PER_FREQ   20.0f
#undef M1_IDENT_BODE_CYCLES_HI
#define M1_IDENT_BODE_CYCLES_HI         50.0f  /* T_OBS_HI 优先；此。fallback */
#undef M1_IDENT_BODE_F0_HZ
#define M1_IDENT_BODE_F0_HZ             10.0f
#undef M1_IDENT_BODE_F1_HZ
#define M1_IDENT_BODE_F1_HZ             2500.0f
#undef M1_IDENT_BODE_F_RATIO
#define M1_IDENT_BODE_F_RATIO           1.15f
#undef M1_IDENT_BODE_F_SPLIT_HZ
#define M1_IDENT_BODE_F_SPLIT_HZ        500.0f
#undef M1_IDENT_BODE_USE_T_OBS_HI
#define M1_IDENT_BODE_USE_T_OBS_HI      1
#undef M1_IDENT_BODE_T_OBS_HI_S
#define M1_IDENT_BODE_T_OBS_HI_S        0.1f
#undef M1_IDENT_BODE_F_RATIO_HI
#define M1_IDENT_BODE_F_RATIO_HI        1.06f
#undef M1_IDENT_BODE_AXIS_ID
#define M1_IDENT_BODE_AXIS_ID           0
#undef M1_IDENT_FIX_THETA_ENABLE
#define M1_IDENT_FIX_THETA_ENABLE       1
#undef M1_IDENT_THETA_EL_RAD
#define M1_IDENT_THETA_EL_RAD           M1_ID_CAL_THETA_EL_RAD
#undef M1_IDENT_HOLD_S
#define M1_IDENT_HOLD_S                 2.0f
#undef M1_IDENT_OVERRIDE_LIMITS
#define M1_IDENT_OVERRIDE_LIMITS        1
#define M1_DEADBAND_LUT_BAKED_ENABLE    0
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
#define M1_IDENT_POST_BODE_OPEN_UQ_ENABLE  0

#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_BODE_ID_OFF_ONLY)
/** Id Bode 签收：参数见 profiles/m1_bode_id_fc1000.profile.h（fc=1000。*/
#include "profiles/m1_bode_id_fc1000.profile.h"

#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_SPEED_IDENT)
#if M1_USE_HFI_STANDSTILL_PROFILE
/** HFI 静置旁路脚手架：。profiles/m1_hfi_standstill.profile.h */
#include "profiles/m1_hfi_standstill.profile.h"
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
#undef M1_VOFA_SPEED_CH8_11
#define M1_VOFA_SPEED_CH8_11            1
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
#if (M1_VOFA_OBS_PLL_12CH + M1_VOFA_OBS_SMO_12CH + M1_VOFA_OBS_VEQ_12CH + \
     M1_VOFA_OBS_SMO_RAW_12CH + M1_VOFA_HFI_12CH + M1_VOFA_IF_12CH) > 1
#error "Only one VOFA 12ch layout (OBS/IF/HFI) after profile include"
#endif
#if M1_USE_HFI_STANDSTILL_PROFILE && !M1_HFI_ENABLE
#error "M1_USE_HFI_STANDSTILL_PROFILE requires M1_HFI_ENABLE=1"
#endif
#if M1_USE_HFI_STANDSTILL_PROFILE && (M1_HFI_GATE != 1) && (M1_HFI_GATE != 2) && \
    (M1_HFI_GATE != 3) && (M1_HFI_GATE != 4) && (M1_HFI_GATE != 5) && \
    (M1_HFI_GATE != 6) && (M1_HFI_GATE != 7) && (M1_HFI_GATE != 8) && \
    (M1_HFI_GATE != 9) && (M1_HFI_GATE != 10) && (M1_HFI_GATE != 11) && \
    (M1_HFI_GATE != 12) && (M1_HFI_GATE != 13) && (M1_HFI_GATE != 14) && \
    (M1_HFI_GATE != 15) && (M1_HFI_GATE != 16) && (M1_HFI_GATE != 17) && \
    (M1_HFI_GATE != 18) && (M1_HFI_GATE != 19) && (M1_HFI_GATE != 20) && \
    (M1_HFI_GATE != 21) && (M1_HFI_GATE != 22) && (M1_HFI_GATE != 23) && \
    (M1_HFI_GATE != 24) && (M1_HFI_GATE != 25) && (M1_HFI_GATE != 26) && \
    (M1_HFI_GATE != 27) && (M1_HFI_GATE != 28) && (M1_HFI_GATE != 29) && \
    (M1_HFI_GATE != 30) && (M1_HFI_GATE != 31) && (M1_HFI_GATE != 32) && \
    (M1_HFI_GATE != 33) && (M1_HFI_GATE != 34) && (M1_HFI_GATE != 35) && \
    (M1_HFI_GATE != 36) && (M1_HFI_GATE != 37) && (M1_HFI_GATE != 38) && \
    (M1_HFI_GATE != 39) && (M1_HFI_GATE != 40) && (M1_HFI_GATE != 41) && \
    (M1_HFI_GATE != 42) && (M1_HFI_GATE != 43) &&     (M1_HFI_GATE != 44) && \
    (M1_HFI_GATE != 45) && \
    (M1_HFI_GATE != 46) && \
    (M1_HFI_GATE != 47) && \
    (M1_HFI_GATE != 48) && \
    (M1_HFI_GATE != 49) && \
    (M1_HFI_GATE != 50) && \
    (M1_HFI_GATE != 51) && \
    (M1_HFI_GATE != 52) && \
    (M1_HFI_GATE != 53) && \
    (M1_HFI_GATE != 54) && \
    (M1_HFI_GATE != 55) && \
    (M1_HFI_GATE != 56) && \
    (M1_HFI_GATE != 57) && \
    (M1_HFI_GATE != 58) && \
    (M1_HFI_GATE != 59) && \
    (M1_HFI_GATE != 60) && \
    (M1_HFI_GATE != 61) && \
    (M1_HFI_GATE != 62) && \
    (M1_HFI_GATE != 63) && \
    (M1_HFI_GATE != 64) && \
    (M1_HFI_GATE != 65) && \
    (M1_HFI_GATE != 66) && \
    (M1_HFI_GATE != 67) && \
    (M1_HFI_GATE != 68) && (M1_HFI_GATE != 69) && (M1_HFI_GATE != 70) && (M1_HFI_GATE != 71) && (M1_HFI_GATE != 72) && (M1_HFI_GATE != 73) && (M1_HFI_GATE != 74) && (M1_HFI_GATE != 75) && (M1_HFI_GATE != 76) && (M1_HFI_GATE != 77) && (M1_HFI_GATE != 78) && (M1_HFI_GATE != 79) && (M1_HFI_GATE != 80) && (M1_HFI_GATE != 81) && (M1_HFI_GATE != 82) && (M1_HFI_GATE != 83) && (M1_HFI_GATE != 84) && (M1_HFI_GATE != 85) && (M1_HFI_GATE != 86) && (M1_HFI_GATE != 87) &&     (M1_HFI_GATE != 88) && (M1_HFI_GATE != 89) &&     (M1_HFI_GATE != 90) && (M1_HFI_GATE != 91) && (M1_HFI_GATE != 92) &&     (M1_HFI_GATE != 93) &&     (M1_HFI_GATE != 94) &&     (M1_HFI_GATE != 95) &&     (M1_HFI_GATE != 96) && (M1_HFI_GATE != 97) && (M1_HFI_GATE != 98) &&     (M1_HFI_GATE != 99) && (M1_HFI_GATE != 100) && (M1_HFI_GATE != 101) && (M1_HFI_GATE != 102) && (M1_HFI_GATE != 103) && (M1_HFI_GATE != 104) && (M1_HFI_GATE != 105) && (M1_HFI_GATE != 106) && (M1_HFI_GATE != 107) &&     (M1_HFI_GATE != 108) && (M1_HFI_GATE != 109) &&     (M1_HFI_GATE != 110) && \
    (M1_HFI_GATE != 111) && (M1_HFI_GATE != 112) && (M1_HFI_GATE != 113) && \
    (M1_HFI_GATE != 114) && (M1_HFI_GATE != 115) && (M1_HFI_GATE != 116) && \
    (M1_HFI_GATE != 117) && (M1_HFI_GATE != 118) && (M1_HFI_GATE != 119) && \
    (M1_HFI_GATE != 120) && (M1_HFI_GATE != 121) && (M1_HFI_GATE != 122) && \
    (M1_HFI_GATE != 123) && (M1_HFI_GATE != 124) && (M1_HFI_GATE != 125) && \
    (M1_HFI_GATE != 126) &&     (M1_HFI_GATE != 127) &&     (M1_HFI_GATE != 128) && (M1_HFI_GATE != 129) && (M1_HFI_GATE != 130) && (M1_HFI_GATE != 131) && (M1_HFI_GATE != 132) && (M1_HFI_GATE != 133) && (M1_HFI_GATE != 134) && (M1_HFI_GATE != 135) &&     (M1_HFI_GATE != 136) && (M1_HFI_GATE != 137) &&     (M1_HFI_GATE != 138) && (M1_HFI_GATE != 139) && (M1_HFI_GATE != 140) && (M1_HFI_GATE != 141)
#error "M1_HFI_GATE must be 1..141 (S1..S2j / HFI-SMO / V1-V5 / VESC尺)"
#endif
#ifndef M1_HFI_ID_PI_OFF_ENABLE
#define M1_HFI_ID_PI_OFF_ENABLE         0
#endif
#ifndef M1_HFI_ID_PI_SOFT_N
#define M1_HFI_ID_PI_SOFT_N             20000u /* 放行 Ud 软开 1.0 s @ 20 kHz */
#endif
#ifndef M1_HFI_VESC_ID_HANDOFF_ENABLE
#define M1_HFI_VESC_ID_HANDOFF_ENABLE   0 /* 1：窗内 VH↓ 同时放行 Id PI */
#endif
#ifndef M1_HFI_VESC_HANDOFF_N
#define M1_HFI_VESC_HANDOFF_N          10000u /* 0.5 s @20 kHz */
#endif
#ifndef M1_HFI_HFI_ID_SOFT_ENABLE
#define M1_HFI_HFI_ID_SOFT_ENABLE      0 /* 1：纯 HFI 锁后软开 Id，VH 不动 */
#endif
#ifndef M1_HFI_ID_ON_FROM_RUN_ENABLE
#define M1_HFI_ID_ON_FROM_RUN_ENABLE   0 /* 1：进 RUN 即 Id*=0 常开（无 soft） */
#endif
#ifndef M1_HFI_ID_PI_LPF_ENABLE
#define M1_HFI_ID_PI_LPF_ENABLE         0 /* 1：Id PI 反馈用 LPF(Id)，解调仍裸 Id */
#endif
#ifndef M1_HFI_ID_PI_LPF_A
#define M1_HFI_ID_PI_LPF_A              (0.05f)
#endif
#ifndef M1_HFI_DEMOD_HP_ENABLE
#define M1_HFI_DEMOD_HP_ENABLE          0 /* 1：半周 di 用高通 Id/Iq */
#endif
#ifndef M1_HFI_DEMOD_HP_A
#define M1_HFI_DEMOD_HP_A               (0.05f)
#endif
#ifndef M1_HFI_DEMOD_SKIP_OUTLIER
#define M1_HFI_DEMOD_SKIP_OUTLIER       0 /* 1：不像凸极的半周不更新 x/ε/PLL */
#endif
#ifndef M1_HFI_DEMOD_SKIP_Y_ABS
#define M1_HFI_DEMOD_SKIP_Y_ABS         (0.08f)
#endif
#ifndef M1_HFI_DEMOD_SKIP_X_ABS
#define M1_HFI_DEMOD_SKIP_X_ABS         (0.36f)
#endif
#ifndef M1_HFI_DEMOD_SKIP_DI_Q_ABS
#define M1_HFI_DEMOD_SKIP_DI_Q_ABS      (0.12f)
#endif
#ifndef M1_HFI_PLL_VESC_ERR_ENABLE
#define M1_HFI_PLL_VESC_ERR_ENABLE      0 /* 1：PLL 吃 V4 的 di_q/Vh/Δ(1/L)，再限幅 */
#endif
#ifndef M1_HFI_PLL_VESC_MAX_ERR
#define M1_HFI_PLL_VESC_MAX_ERR         (0.30f) /* 同 foc_hfi_max_err */
#endif
#ifndef M1_HFI_PLL_VESC_ERR_SIGN
#define M1_HFI_PLL_VESC_ERR_SIGN        (1.0f)
#endif
#ifndef M1_HFI_LQ_WELL_FLIP_ENABLE
#define M1_HFI_LQ_WELL_FLIP_ENABLE      0 /* 1：x 低且 y≈0 则 θ̂ 一次 +π/2 */
#endif
#ifndef M1_HFI_LQ_WELL_X_MAX
#define M1_HFI_LQ_WELL_X_MAX            (0.185f)
#endif
#ifndef M1_HFI_LQ_WELL_Y_ABS
#define M1_HFI_LQ_WELL_Y_ABS            (0.015f)
#endif
#ifndef M1_HFI_LQ_WELL_HOLD_N
#define M1_HFI_LQ_WELL_HOLD_N           20u
#endif
#ifndef M1_HFI_LQ_WELL_IQ_MIN
#define M1_HFI_LQ_WELL_IQ_MIN           (0.0f)
#endif
#ifndef M1_HFI_LQ_WELL_KEEP_W
#define M1_HFI_LQ_WELL_KEEP_W           0
#endif
#ifndef M1_HFI_DEMOD_AB_ENABLE
#define M1_HFI_DEMOD_AB_ENABLE          0 /* 1：αβ 半周差分再 Park(θ̂) */
#endif
#ifndef M1_HFI_DEMOD_AB_MID_ENABLE
#define M1_HFI_DEMOD_AB_MID_ENABLE      0 /* 1：αβ 三端点中点差分再 Park(中点 θ̂) */
#endif
#ifndef M1_HFI_INJECT_AB_ENABLE
#define M1_HFI_INJECT_AB_ENABLE         0 /* 1：反 Park 后沿 θ̂ 叠 ±Vh 到 αβ */
#endif
#ifndef M1_HFI_INJECT_POST_LOOP
#define M1_HFI_INJECT_POST_LOOP         0 /* 1：环后沿 θ̂ 叠 Vh（用 s_vh_sign） */
#endif
#ifndef M1_HFI_DEMOD_INJ_AXIS
#define M1_HFI_DEMOD_INJ_AXIS           0 /* 1：注入轴 αβ 电流解调 */
#endif
/*
 * HFI↔SMO 加速交接（38/53。2）：
 * 55：残 Vh。KILL→VH_END。 或微地板）。OPEN_ID：残 Vh 下开 Id。
 * 62：开 Id 。W_HOLD（速度反馈守卫，通用路径）。
 */
#ifndef M1_HFI_SMO_HAND_ENABLE
#define M1_HFI_SMO_HAND_ENABLE          0
#endif
#ifndef M1_HFI_SMO_SUB_VH_ENABLE
#define M1_HFI_SMO_SUB_VH_ENABLE        0 /* 1：SMO 用 u−u_hfi，HFI 段不停观测 */
#endif
#ifndef M1_HFI_ROTATE_PI_ENABLE
#define M1_HFI_ROTATE_PI_ENABLE         0 /* 1：Park 切 SMO 时旋 Id/Iq PI */
#endif
#ifndef M1_HFI_HAND_ID_OVERLAP_ENABLE
#define M1_HFI_HAND_ID_OVERLAP_ENABLE   0
#endif
#ifndef M1_HFI_HAND_OPEN_ID_ENABLE
#define M1_HFI_HAND_OPEN_ID_ENABLE      0
#endif
#ifndef M1_HFI_HAND_KILL_VH_ENABLE
#define M1_HFI_HAND_KILL_VH_ENABLE      0
#endif
#ifndef M1_HFI_HAND_IQ_HOLD_ON_IDUP
#define M1_HFI_HAND_IQ_HOLD_ON_IDUP     0
#endif
#ifndef M1_HFI_HAND_W_HOLD_ON_IDUP
#define M1_HFI_HAND_W_HOLD_ON_IDUP      0
#endif
#ifndef M1_HFI_HAND_W_REL_N
#define M1_HFI_HAND_W_REL_N            4000u
#endif
#ifndef M1_HFI_HAND_W_SLEW_ENABLE
#define M1_HFI_HAND_W_SLEW_ENABLE       0
#endif
#ifndef M1_HFI_HAND_W_SLEW_RPM_S
#define M1_HFI_HAND_W_SLEW_RPM_S        (200.0f)
#endif
#ifndef M1_HFI_HAND_W_SLEW_IDUP_ONLY
#define M1_HFI_HAND_W_SLEW_IDUP_ONLY    0
#endif
#ifndef M1_HFI_HAND_W_SLEW_SMO_N
#define M1_HFI_HAND_W_SLEW_SMO_N       4000u
#endif
#ifndef M1_HFI_HAND_VH0_SOFT_ENABLE
#define M1_HFI_HAND_VH0_SOFT_ENABLE     0
#endif
#ifndef M1_HFI_HAND_STOP_AFTER
#define M1_HFI_HAND_STOP_AFTER          0
#endif
#ifndef M1_HFI_HAND_VH_FLOOR
#define M1_HFI_HAND_VH_FLOOR            (0.25f)
#endif
#ifndef M1_HFI_HAND_VH_END
#define M1_HFI_HAND_VH_END              (0.0f) /* KILL 终点 scale */
#endif
#ifndef M1_HFI_HAND_ID_WEAK
#define M1_HFI_HAND_ID_WEAK             (0.12f)
#endif
#ifndef M1_HFI_HAND_HOLD_N
#define M1_HFI_HAND_HOLD_N             10000u
#endif
#ifndef M1_HFI_HAND_FADE_N
#define M1_HFI_HAND_FADE_N             4000u
#endif
#ifndef M1_HFI_HAND_VH0_N
#define M1_HFI_HAND_VH0_N              20000u
#endif
#ifndef M1_HFI_HAND_IDUP_N
#define M1_HFI_HAND_IDUP_N             20000u
#endif
#ifndef M1_HFI_HAND_REV_ENABLE
#define M1_HFI_HAND_REV_ENABLE          0 /* 1：SMO 减速过 REV_RPM 。HFI */
#endif
#ifndef M1_HFI_HAND_REV_WAKE_ENABLE
#define M1_HFI_HAND_REV_WAKE_ENABLE     0 /* 1：RVH 。hold + RQUAL */
#endif
#ifndef M1_HFI_HAND_REV_VH_WAKE
#define M1_HFI_HAND_REV_VH_WAKE         M1_HFI_HAND_VH_FLOOR /* 68=1.0 满注入 */
#endif
#ifndef M1_HFI_HAND_REV_RVH_HOLD_ENABLE
#define M1_HFI_HAND_REV_RVH_HOLD_ENABLE 0 /* 1：RVH 钉 θ̂；69 */
#endif
#ifndef M1_HFI_HAND_REV_RESEED_N
#define M1_HFI_HAND_REV_RESEED_N       2000u
#endif
#ifndef M1_HFI_HAND_RQUAL_X_MAX_ENABLE
#define M1_HFI_HAND_RQUAL_X_MAX_ENABLE  1
#endif
#ifndef M1_HFI_HAND_RQUAL_X_MAX
#define M1_HFI_HAND_RQUAL_X_MAX         (0.45f)
#endif
#ifndef M1_HFI_HAND_REV_OBS_ENABLE
#define M1_HFI_HAND_REV_OBS_ENABLE      0 /* 1: RVH->ROBS observe only */
#endif
#ifndef M1_HFI_HAND_REV_VH_MIRROR_ENABLE
#define M1_HFI_HAND_REV_VH_MIRROR_ENABLE 0
#endif
#ifndef M1_HFI_HAND_DECEL_BRAKE_ENABLE
#define M1_HFI_HAND_DECEL_BRAKE_ENABLE  0 /* 1: SMO decel |Iq| floor (74) */
#endif
#ifndef M1_HFI_HAND_DECEL_BRAKE_IQ_A
#define M1_HFI_HAND_DECEL_BRAKE_IQ_A    (1.5f) /* |Iq|_min */
#endif
#ifndef M1_HFI_HAND_DECEL_BRAKE_END_RPM
#define M1_HFI_HAND_DECEL_BRAKE_END_RPM (920.0f) /* release on SMO ω */
#endif
#ifndef M1_HFI_HAND_DECEL_BRAKE_ARM_RPM
#define M1_HFI_HAND_DECEL_BRAKE_ARM_RPM (1400.0f)
#endif
#ifndef M1_HFI_HAND_DECEL_BRAKE_DROP_RPM
#define M1_HFI_HAND_DECEL_BRAKE_DROP_RPM (80.0f)
#endif
#ifndef M1_HFI_HAND_SPD_X_KILL_HI
#define M1_HFI_HAND_SPD_X_KILL_HI       (0.62f)
#endif
#ifndef M1_HFI_HAND_REV_RPM
#define M1_HFI_HAND_REV_RPM             (1400.0f)
#endif
#ifndef M1_HFI_HAND_REV_ARM_RPM
#define M1_HFI_HAND_REV_ARM_RPM         (1450.0f)
#endif
#ifndef M1_HFI_HAND_RVH_N
#define M1_HFI_HAND_RVH_N              20000u
#endif
#ifndef M1_HFI_HAND_RQUAL_N
#define M1_HFI_HAND_RQUAL_N            8000u
#endif
#ifndef M1_HFI_HAND_RQUAL_TIMEOUT_N
#define M1_HFI_HAND_RQUAL_TIMEOUT_N    60000u
#endif
#ifndef M1_HFI_HAND_RANG_N
#define M1_HFI_HAND_RANG_N             20000u
#endif
#ifndef M1_HFI_HAND_RFADE_N
#define M1_HFI_HAND_RFADE_N            16000u
#endif
#ifndef M1_HFI_HAND_RSPD_N
#define M1_HFI_HAND_RSPD_N             40000u
#endif
#ifndef M1_HFI_QKICK_FORCE_PI
#define M1_HFI_QKICK_FORCE_PI           0
#endif
#ifndef M1_HFI_QKICK_PRE_GATE_ENABLE
#define M1_HFI_QKICK_PRE_GATE_ENABLE    0
#endif
#ifndef M1_HFI_IQ_AUTH_ENABLE
#define M1_HFI_IQ_AUTH_ENABLE           0
#endif
#ifndef M1_HFI_IQ_AUTH_CLEAR_N
#define M1_HFI_IQ_AUTH_CLEAR_N          1u
#endif
#ifndef M1_HFI_IQ_AUTH_FEED_ENABLE
#define M1_HFI_IQ_AUTH_FEED_ENABLE      0
#endif
#ifndef M1_HFI_IQ_AUTH_FEED_A
#define M1_HFI_IQ_AUTH_FEED_A           0.0f
#endif
#ifndef M1_HFI_IQ_AUTH_FEED_HOLD
#define M1_HFI_IQ_AUTH_FEED_HOLD        0 /* 1：FEED 已起后 auth 掉旗不掐 Iq* */
#endif
#ifndef M1_HFI_IQ_AUTH_FEED_LEGACY_BRANCH
#define M1_HFI_IQ_AUTH_FEED_LEGACY_BRANCH 0
#endif
#if M1_VOFA_HFI_12CH && !M1_HFI_ENABLE
#error "M1_VOFA_HFI_12CH requires M1_HFI_ENABLE=1"
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

#else
#error "Unknown M1_BRINGUP_MODE — use M1_BRINGUP_MODE_* in motor_params_m1.h"
#endif

/** 辨识/标定 bringup：关 PLL，避。ch8。1 。ω/θ 占用导致 open_seq 不可。*/
#if (M1_BRINGUP_MODE != M1_BRINGUP_MODE_NORMAL) && \
    (M1_BRINGUP_MODE != M1_BRINGUP_MODE_SPEED_IDENT)
#undef M1_PLL_ENABLE
#define M1_PLL_ENABLE                   0
#undef M1_VOFA_PLL_CH8_11
#define M1_VOFA_PLL_CH8_11              0
#endif

/** 1=辨识 VOFA ch6。 端电压估。+ ch8。0 abc duty（可离线核对 ψ）；0=。Ud/Uq + Id_ref 布局 */
#ifndef M1_VOFA_IDENT_DUTY_12CH
#if (M1_BRINGUP_MODE != M1_BRINGUP_MODE_NORMAL) && \
    (M1_BRINGUP_MODE != M1_BRINGUP_MODE_SPEED_IDENT) && \
    (M1_BRINGUP_MODE != M1_BRINGUP_MODE_BODE_OFF_ONLY) && \
    (M1_BRINGUP_MODE != M1_BRINGUP_MODE_BODE_ID_OFF_ONLY)
#define M1_VOFA_IDENT_DUTY_12CH         1
#else
#define M1_VOFA_IDENT_DUTY_12CH         0
#endif
#endif

#ifndef M1_DEADBAND_LUT_BAKED_ENABLE
#define M1_DEADBAND_LUT_BAKED_ENABLE  0
#endif

/* 当前模式名（调试/VOFA 备注用；勿用 #pragma message，Keil 。TU 重复告警。*/
#if (M1_BRINGUP_MODE == M1_BRINGUP_MODE_MULTI_ANGLE_PASS0)
#define M1_BRINGUP_MODE_NAME  "MULTI_ANGLE_PASS0"
#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_IDENT_IQ_STEP)
#define M1_BRINGUP_MODE_NAME  "IDENT_IQ_STEP"
#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_ID_CAL_DUAL_FULL)
#define M1_BRINGUP_MODE_NAME  "ID_CAL_DUAL_FULL"
#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_ID_CAL_PASS0_RS)
#define M1_BRINGUP_MODE_NAME  "ID_CAL_PASS0_RS"
#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_RS_LD_LQ_ONLY)
#define M1_BRINGUP_MODE_NAME  "RS_LD_LQ_ONLY"
#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_OPEN_UD_LUT_VERIFY)
#define M1_BRINGUP_MODE_NAME  "OPEN_UD_LUT_VERIFY"
#elif M1_DEADBAND_FLOW_ONE_SHOT
#define M1_BRINGUP_MODE_NAME  "ONE_SHOT"
#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_ID_CAL_PASS0_BUILD)
#define M1_BRINGUP_MODE_NAME  "ID_CAL_PASS0_BUILD"
#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_ID_CAL_PASS0_ONLY)
#define M1_BRINGUP_MODE_NAME  "ID_CAL_PASS0_ONLY"
#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_IQ_PROBE_OFF_ONLY)
#define M1_BRINGUP_MODE_NAME  "IQ_PROBE_OFF_ONLY"
#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_BODE_OFF_ONLY)
#define M1_BRINGUP_MODE_NAME  "BODE_OFF_ONLY"
#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_BODE_ID_OFF_ONLY)
#define M1_BRINGUP_MODE_NAME  "BODE_ID_OFF_ONLY"
#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_SPEED_IDENT)
#define M1_BRINGUP_MODE_NAME  "SPEED_IDENT"
#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_NORMAL) && M1_SPEED_LOOP_ENABLE && \
      (M1_DEADBAND_BRINGUP_PHASE == M1_DB_BRINGUP_SPEED_LUT)
#define M1_BRINGUP_MODE_NAME  "NORMAL_SPEED_DB_LUT"
#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_NORMAL) && M1_SPEED_LOOP_ENABLE && \
      (M1_DEADBAND_BRINGUP_PHASE == M1_DB_BRINGUP_SPEED_OFF) && \
      M1_SPEED_REVERSAL_TEST_ENABLE
#define M1_BRINGUP_MODE_NAME  "NORMAL_SPEED_REV_TEST"
#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_NORMAL) && M1_SPEED_LOOP_ENABLE && \
      (M1_DEADBAND_BRINGUP_PHASE == M1_DB_BRINGUP_SPEED_OFF) && \
      M1_POS_MIT_COMBO_ENABLE && !M1_POS_MIT_COMBO_POS_ENABLE
#define M1_BRINGUP_MODE_NAME  "NORMAL_MIT_ONLY"
#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_NORMAL) && M1_SPEED_LOOP_ENABLE && \
      (M1_DEADBAND_BRINGUP_PHASE == M1_DB_BRINGUP_SPEED_OFF) && \
      M1_POS_MIT_COMBO_ENABLE
#define M1_BRINGUP_MODE_NAME  "NORMAL_POS_MIT"
#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_NORMAL) && M1_SPEED_LOOP_ENABLE && \
      (M1_DEADBAND_BRINGUP_PHASE == M1_DB_BRINGUP_SPEED_OFF)
#define M1_BRINGUP_MODE_NAME  "NORMAL_SPEED_DB_OFF"
#else
#define M1_BRINGUP_MODE_NAME  "NORMAL"
#endif

/**
 * Id 锁轴标定状态机。D Id 扫表 。。1 。Ud_pi；段 2+ 在线。LUT）。
 *
 * =1：强。CURRENT_LOOP、Iq_ref=0、deadband OFF；自动扫 Id。
 * 。M1_OPEN_UQ_DEADBAND_AB_SWEEP 互斥。
 * VOFA×12（M1_VOFA_UNIFIED_12CH=1）：ch0=Ia ch1=Ib ch2=Ic ch3=Id ch4=Iq ch5=θ
 *       M1_VOFA_IDENT_DUTY_12CH=1：ch6=Vd_est ch7=Vq_est ch8=Ta ch9=Tb ch10=Tc ch11=open_seq。
 *       M1_VOFA_IDENT_DUTY_12CH=0：ch6=Ud_out ch7=Uq_out。
 *       M1_VOFA_PLL_CH8_11=1 。ch8=ω_pll ch9=ω_diff ch10=θ_err ch11=Δω。
 *       M1_VOFA_PLL_CH8_11=0 。ch8=Id_ref ch9=Iq_ref ch10=duty_dev ch11=open_seq。
 *       VASI(open_seq=57) 。ch8=grid ch9=proc_code ch10=L_est_uH ch11=open_seq（ch6/7 不变）。
 * M1_LD_LQ_IDENT_INJECT_LUT_ENABLE=1：仅 INJ_LD/LQ 。LUT runtime；SETTLE/PRE_DECAY OFF。
 * M1_OPEN_PRE_ID_LADDER_LUT_RUNTIME=1：开。Ud/Uq ladder 。LUT runtime（AB=0 时整。ON）。
 * M1_OPEN_PRE_ID_LADDER_AB_ENABLE=1：Ud/Uq 阶梯。OFF(70..75) 。LUT ON(80..85)。7/87=轮末。
 * dbg.open_seq_phase。8=ALIGN(30° Ud)。4=Rs ramp。5=Rs OK。6=Rs FAIL。
 *   57=L ident 中；58=L OK。9=L FAIL。63=Rs/L decay。
 *   M1_RS_L_IDENT_DUAL_LUT_ROUND_ENABLE=1 。open_seq +100。57/158…）；ch10=0/1 。OFF/LUT 轮；
 *   9=DONE。
 *   0=init。..N=标定档，39=Pass0 衰减。0/41..=Pass1 LUT 验收。2=JustFloat LUT 突发。
 *   50=Iq 探路 LUT OFF。1=Iq 探路 LUT ON。=DONE。
 *   DONE 。M1_VOFA_IDENT_DUMP_ENABLE=1：I0≈−777777 突发 Rs+9 。Ld/Lq（proto 1.0）；
 *   120+leg×50+step=多角 Pass0（三。leg0..2 。120..269，五。。120..369）。
 *
 * 。M1_BRINGUP_MODE 推导；勿。M1_IDENT_ENABLE 同开。
 */
#ifndef M1_ID_LOCK_CAL_SWEEP
#define M1_ID_LOCK_CAL_SWEEP  0
#endif

#if M1_ID_LOCK_CAL_SWEEP
#if M1_OPEN_UQ_DEADBAND_AB_SWEEP
#error "M1_ID_LOCK_CAL_SWEEP and M1_OPEN_UQ_DEADBAND_AB_SWEEP are mutually exclusive"
#endif

/** 标定态临时放。I_ref / PI 限幅（与 M1_CLOSURE_BRINGUP 日常 0.5 A 解耦） */
#ifndef M1_ID_CAL_OVERRIDE_LIMITS
#define M1_ID_CAL_OVERRIDE_LIMITS  1
#endif

#if M1_ID_CAL_OVERRIDE_LIMITS
#ifndef M1_ID_CAL_I_REF_ABS_MAX
#define M1_ID_CAL_I_REF_ABS_MAX    3.0f
#endif
/** 。2347 电流环一致（M1_CLOSURE_BRINGUP 6 V），Pass0/Pass1/第三段共。*/
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
/** Pass0 标定扫表。38 。= 。I 加密 + 中高 I（DUAL_FULL）；IDENT 模式覆写。30 。1.5 A */
#ifndef M1_ID_CAL_AMP_TABLE_LEN
#define M1_ID_CAL_AMP_TABLE_LEN     138u
#endif
/** 。I 加密区上界（A）；。amp 。0.40 A 末档、ID_DWELL_LOW_ID_A 对齐 */
#ifndef M1_ID_CAL_LOW_I_DENSE_END_A
#define M1_ID_CAL_LOW_I_DENSE_END_A 0.40f
#endif
#ifndef M1_ID_CAL_LOW_I_STEP_A
#define M1_ID_CAL_LOW_I_STEP_A      0.003125f
#endif
#ifndef M1_ID_CAL_I_MAX_A
#define M1_ID_CAL_I_MAX_A           3.0f
#endif
/**
 * Pass1 LUT 验收：默认与 Pass0 同表（EXT_LEN=0）；>0 时在 Pass0 表后追加续扫档。
 * 各档 dwell 统一 M1_ID_CAL_VERIFY_ID_DWELL_S（联调万用表时可改为 3 s）。
 */
#ifndef M1_ID_CAL_VERIFY_I_MAX_A
#define M1_ID_CAL_VERIFY_I_MAX_A    3.0f
#endif
#ifndef M1_ID_CAL_VERIFY_EXT_LEN
#define M1_ID_CAL_VERIFY_EXT_LEN    0u
#endif
#define M1_ID_CAL_VERIFY_AMP_TABLE_LEN  (M1_ID_CAL_AMP_TABLE_LEN + M1_ID_CAL_VERIFY_EXT_LEN)
#ifndef M1_ID_CAL_ID_DWELL_S
#define M1_ID_CAL_ID_DWELL_S        3.0f
#endif
/** Id_ref 。此值时。M1_ID_CAL_ID_DWELL_LOW_S（与 M1_ID_CAL_ID_DWELL_S 同），否则用 M1_ID_CAL_ID_DWELL_S */
#ifndef M1_ID_CAL_ID_DWELL_LOW_ID_A
#define M1_ID_CAL_ID_DWELL_LOW_ID_A  0.40f
#endif
#ifndef M1_ID_CAL_ID_DWELL_LOW_S
#define M1_ID_CAL_ID_DWELL_LOW_S     M1_ID_CAL_ID_DWELL_S
#endif
/** Pass1 LUT 验收：各 Id 。dwell 统一（s），不再。LOW_S 加长 */
#ifndef M1_ID_CAL_VERIFY_ID_DWELL_S
#define M1_ID_CAL_VERIFY_ID_DWELL_S  M1_ID_CAL_ID_DWELL_S
#endif
/** 上电 Id=0 稳定（s）；240125 已验。*/
#define M1_ID_CAL_INIT_HOLD_S       0.5f
/**
 * Pass0 末档→Pass1：Id_ref=0 衰减 (s)，LUT 。OFF，再 commit；仅 LUT_VERIFY 双扫生效。
 */
#ifndef M1_ID_CAL_PASS0_DECAY_S
#define M1_ID_CAL_PASS0_DECAY_S     0.8f
#endif
/** Pass1 LUT 验收扫表 init hold (s)，commit 后略长以。settle */
#ifndef M1_ID_CAL_VERIFY_INIT_HOLD_S
#define M1_ID_CAL_VERIFY_INIT_HOLD_S  1.0f
#endif

/** capture：|Id-Id_ref| 门限 (A) */
#ifndef M1_ID_CAL_CAPTURE_EPS_A
#define M1_ID_CAL_CAPTURE_EPS_A     0.03f
#endif

/** Ud_residual 超过此。(V) 。outlier 标志，默认仍写入。*/
#ifndef M1_ID_CAL_OUTLIER_V
#define M1_ID_CAL_OUTLIER_V         1.0f
#endif

/**
 * commit 。capture 表处理：sort 始终开启；dedupe 默认关。
 * 2045：dedupe 32。1 后低 Id Pass1 劣于 0827，待 A/B 验证；高 Id 不受影响。
 */
#ifndef M1_ID_CAL_LUT_DEDUP_ENABLE
#define M1_ID_CAL_LUT_DEDUP_ENABLE  0
#endif
#ifndef M1_ID_CAL_LUT_DEDUP_AMP_EPS_A
#define M1_ID_CAL_LUT_DEDUP_AMP_EPS_A  0.005f
#endif

/**
 * 。2=0：只 RAM capture，不 deadband_set_lut
 * 。3=1：deadband_cal_commit() 注册 LUT
 */
#ifndef M1_ID_CAL_COMMIT_LUT
#define M1_ID_CAL_COMMIT_LUT        1
#endif

/**
 * 。3：标。commit 后同次上电自动再。Id（LUT ON，不 capture。.05。.0 A）。
 * 。M1_ID_CAL_COMMIT_LUT=1。
 */
#ifndef M1_ID_CAL_LUT_VERIFY_SWEEP
#define M1_ID_CAL_LUT_VERIFY_SWEEP  1
#endif

/**
 * =1：Pass0 扫完。DONE（deadband 全程 OFF，不 commit / 。LUT 突发 / 。Pass1 / 。Iq 探路）。
 * 。M1_BRINGUP_MODE_ID_CAL_PASS0_ONLY / MULTI_ANGLE_PASS0 自动。1。
 */
#ifndef M1_ID_CAL_PASS0_ONLY_ENABLE
#define M1_ID_CAL_PASS0_ONLY_ENABLE  0
#endif

#if M1_ID_CAL_PASS0_ONLY_ENABLE && M1_ID_CAL_LUT_VERIFY_SWEEP
#error "M1_ID_CAL_PASS0_ONLY_ENABLE requires M1_ID_CAL_LUT_VERIFY_SWEEP=0"
#endif
#if M1_ID_CAL_PASS0_ONLY_ENABLE && M1_ID_CAL_IQ_PROBE_ENABLE
#error "M1_ID_CAL_PASS0_ONLY_ENABLE requires M1_ID_CAL_IQ_PROBE_ENABLE=0"
#endif

#if M1_ID_CAL_LUT_VERIFY_SWEEP && !M1_ID_CAL_COMMIT_LUT
#error "M1_ID_CAL_LUT_VERIFY_SWEEP requires M1_ID_CAL_COMMIT_LUT=1"
#endif

/**
 * Pass1 验收路径。0（默认）d 。+ Ud 注入（锁轴验 capture，不。abc duty 打架）；
 * =1 Pass1 也走 plut + apply_duty（仅 Step1 对照，易堵转）。
 */
#ifndef M1_ID_CAL_PASS1_USE_APPLY_DUTY
#define M1_ID_CAL_PASS1_USE_APPLY_DUTY  0
#endif

/**
 * Pass0 commit 。Iq 旋转探路（同次上电）。
 *   Id=0，Iq=M1_ID_CAL_IQ_PROBE_A（默。0.5 A），编码。θ Park，M1_STARTUP_ENABLE=0 手拨启动。
 *   默认 OFF/FIXED 均为 0 s 。commit 后直。phase LUT ON（open_seq 51）。
 *   可。A/B：M1_ID_CAL_IQ_PROBE_OFF_S>0 。deadband OFF。0），
 *             M1_ID_CAL_IQ_PROBE_FIXED_S>0 。FIXED。3），。LUT ON。
 *   PI/电压限幅。M1_CLOSURE_BRINGUP。V/0.5A）；VOFA ch3=Iq ch4=Id ch5=θ。
 * 。M1_ID_CAL_COMMIT_LUT=1；空载联调，LUT ON 段结束后手动断使。停录。
 */
#ifndef M1_ID_CAL_IQ_PROBE_ENABLE
#define M1_ID_CAL_IQ_PROBE_ENABLE   1   /* Phase 3：Pass1 。Iq 旋转探路 */
#endif
#ifndef M1_ID_CAL_IQ_PROBE_A
#define M1_ID_CAL_IQ_PROBE_A        M1_IQ_REF_A
#endif
/** Iq 探路第一段：LUT OFF 时长 (s) */
#ifndef M1_ID_CAL_IQ_PROBE_OFF_S
#define M1_ID_CAL_IQ_PROBE_OFF_S    10.0f
#endif
/** Iq 探路第二段：FIXED 符号补偿 ON (s)。=跳过，OFF 后直。LUT ON */
#ifndef M1_ID_CAL_IQ_PROBE_FIXED_S
#define M1_ID_CAL_IQ_PROBE_FIXED_S  5.0f
#endif
/**
 * Iq 探路段是否跑 Id PI。0：仅 Uq/Iq 环，Pass1 。Id 扫表后避。Ud 积分锁死转子）。
 */
#ifndef M1_ID_CAL_IQ_PROBE_ID_PI_ENABLE
#define M1_ID_CAL_IQ_PROBE_ID_PI_ENABLE  0
#endif
/** @deprecated 已由 OFF→ON 双段取代；保留宏避免旧配置编译失。*/
#ifndef M1_ID_CAL_IQ_PROBE_DEADBAND_LUT
#define M1_ID_CAL_IQ_PROBE_DEADBAND_LUT  1
#endif

/**
 * Pass0 commit 。Id 。ramp Rs 辨识（论。2.5 累加 LS；LUT abc duty；只上报）。
 * open_seq。4=ramp 中，55=成功。6=失败。
 */
#ifndef M1_RS_IDENT_ENABLE
#define M1_RS_IDENT_ENABLE              0
#endif
/** 1=跳过 Rs ramp，VASI 直接。M1_RS_OHM（与 M1_RS_IDENT_ENABLE 互斥。*/
#ifndef M1_RS_IDENT_USE_FIXED_NOMINAL
#define M1_RS_IDENT_USE_FIXED_NOMINAL   0
#endif
#if M1_RS_IDENT_USE_FIXED_NOMINAL && M1_RS_IDENT_ENABLE
#error "M1_RS_IDENT_USE_FIXED_NOMINAL and M1_RS_IDENT_ENABLE are mutually exclusive"
#endif
#ifndef M1_RS_IDENT_I_MAX_A
#define M1_RS_IDENT_I_MAX_A             3.0f
#endif
#ifndef M1_RS_IDENT_RAMP_A_PER_S
#define M1_RS_IDENT_RAMP_A_PER_S        0.5f
#endif
#ifndef M1_RS_IDENT_I_MIN_FIT_A
#define M1_RS_IDENT_I_MIN_FIT_A         2.0f
#endif
#ifndef M1_RS_IDENT_EPS_TRACK_A
#define M1_RS_IDENT_EPS_TRACK_A         0.03f
#endif
#ifndef M1_RS_IDENT_REPEAT_N
#define M1_RS_IDENT_REPEAT_N            2u
#endif
#ifndef M1_RS_IDENT_INTER_ROUND_S
#define M1_RS_IDENT_INTER_ROUND_S       1.0f
#endif
#ifndef M1_RS_IDENT_MIN_SAMPLES
#define M1_RS_IDENT_MIN_SAMPLES         200u
#endif

#if M1_RS_IDENT_ENABLE && !M1_ID_CAL_COMMIT_LUT && !M1_ID_CAL_RS_LD_LQ_ONLY_ENABLE
#error "M1_RS_IDENT_ENABLE requires M1_ID_CAL_COMMIT_LUT=1 (or RS_LD_LQ_ONLY)"
#endif
#if M1_RS_IDENT_ENABLE && M1_ID_CAL_IQ_PROBE_ENABLE
#error "M1_RS_IDENT_ENABLE and M1_ID_CAL_IQ_PROBE_ENABLE are mutually exclusive"
#endif
#if M1_RS_IDENT_ENABLE && M1_IDENT_ENABLE
#error "M1_RS_IDENT_ENABLE requires M1_IDENT_ENABLE=0"
#endif

/**
 * Pass0+Rs 。VASI 9 。Ld/Lq 曲面（Id/Iq 偏置 0.5~1.5 A 线性区；U_inj 。L 自适应）。
 * open_seq。7=进行中，58=9 格跑完，59=中。abort（ABORT_ON_THETA_DRIFT=1 。θ 漂移）；
 * 163=Rs/L leg 。decay。
 * MULTI=1 。160+leg=ALIGN。70+leg×40=VASI（stride 40 。uint8 溢出）。
 */
#ifndef M1_LD_LQ_IDENT_ENABLE
#define M1_LD_LQ_IDENT_ENABLE           0
#endif
#ifndef M1_LD_LQ_PRE_DECAY_S
#define M1_LD_LQ_PRE_DECAY_S            0.8f
#endif
#ifndef M1_LD_LQ_IDENT_BIAS_RAMP_S
/** leg 入口 G0 SETTLE 。Id/Iq 线。ramp（Rs decay 。0→首格）；格点间阶跃 */
#define M1_LD_LQ_IDENT_BIAS_RAMP_S      0.12f
#endif
#ifndef M1_LD_LQ_OPEN_SEQ_VASI_BASE
#define M1_LD_LQ_OPEN_SEQ_VASI_BASE     170u
#endif
#ifndef M1_LD_LQ_OPEN_SEQ_VASI_STRIDE
#define M1_LD_LQ_OPEN_SEQ_VASI_STRIDE   40u
#endif
#ifndef M1_LD_LQ_OPEN_SEQ_PRE_DECAY
#define M1_LD_LQ_OPEN_SEQ_PRE_DECAY     163u
#endif
#ifndef M1_LD_LQ_IDENT_OPEN_LOOP_ENABLE
/** 1=VASI 注入段冻。SETTLE 。Ud/Uq（论。§2.3 开。HF 注入）；0=闭环 PI+。u_inj（旧。*/
#define M1_LD_LQ_IDENT_OPEN_LOOP_ENABLE  0
#endif
#ifndef M1_LD_LQ_IDENT_BODE_BIAS_GRID_ENABLE
/** 1=2 。Id=0,Iq=0.25/1.25 A（对。BODE_OFF）；0=9/15 格曲。*/
#define M1_LD_LQ_IDENT_BODE_BIAS_GRID_ENABLE  0
#endif
#ifndef M1_LD_LQ_IDENT_FINE_GRID_ENABLE
/** 1=方案 A：Id 0.5/0.75/1.0 × Iq 0/0.25/0.5/0.75/1.0。5 格，0。 A 加密。*/
#define M1_LD_LQ_IDENT_FINE_GRID_ENABLE  0
#endif
#if M1_LD_LQ_IDENT_BODE_BIAS_GRID_ENABLE && M1_LD_LQ_IDENT_FINE_GRID_ENABLE
#error "M1_LD_LQ_IDENT_BODE_BIAS_GRID_ENABLE and FINE_GRID_ENABLE are mutually exclusive"
#endif
#if M1_LD_LQ_IDENT_BODE_BIAS_GRID_ENABLE
#undef M1_LD_LQ_ID_BIAS_N
#define M1_LD_LQ_ID_BIAS_N              1u
#undef M1_LD_LQ_IQ_BIAS_N
#define M1_LD_LQ_IQ_BIAS_N              2u
#elif M1_LD_LQ_IDENT_FINE_GRID_ENABLE
#undef M1_LD_LQ_ID_BIAS_N
#define M1_LD_LQ_ID_BIAS_N              3u
#undef M1_LD_LQ_IQ_BIAS_N
#define M1_LD_LQ_IQ_BIAS_N              5u
#endif
#ifndef M1_LD_LQ_IDENT_FINE_ONLY
/** 1=。1 kHz fine（跳。500 Hz coarse 。2 kHz f2）；telem 仍为 proto 2.0，coarse 列无。*/
#define M1_LD_LQ_IDENT_FINE_ONLY         0
#endif
#if M1_LD_LQ_IDENT_FINE_ONLY
#undef M1_LD_LQ_IDENT_F2_ENABLE
#define M1_LD_LQ_IDENT_F2_ENABLE        0
#endif
#ifndef M1_LD_LQ_ID_BIAS_N
#define M1_LD_LQ_ID_BIAS_N              3u
#endif
#ifndef M1_LD_LQ_IQ_BIAS_N
#define M1_LD_LQ_IQ_BIAS_N              3u
#endif
#ifndef M1_LD_LQ_IDENT_F_COARSE_HZ
#define M1_LD_LQ_IDENT_F_COARSE_HZ      2000.0f
#endif
#ifndef M1_LD_LQ_IDENT_F_FINE_HZ
#define M1_LD_LQ_IDENT_F_FINE_HZ        5000.0f
#endif
/** 1=每轴 coarse→fine→f2 三档。=。500 Hz+1 kHz（proto 2.0。*/
#ifndef M1_LD_LQ_IDENT_F2_ENABLE
#define M1_LD_LQ_IDENT_F2_ENABLE        0
#endif
#if M1_LD_LQ_IDENT_F2_ENABLE
#ifndef M1_LD_LQ_IDENT_F_F2_HZ
#define M1_LD_LQ_IDENT_F_F2_HZ          2000.0f
#endif
#endif
/** @deprecated 兼容旧脚本；等同 F_FINE */
#ifndef M1_LD_LQ_IDENT_F_HZ
#define M1_LD_LQ_IDENT_F_HZ             M1_LD_LQ_IDENT_F_FINE_HZ
#endif
#ifndef M1_LD_LQ_IDENT_L_NOM_H
/** VASI U_inj 标定用名义电。(Ld+Lq)/2 */
#define M1_LD_LQ_IDENT_L_NOM_H          70e-6f
#endif
#ifndef M1_LD_LQ_IDENT_DI_TARGET_A
/** 目标 HF 纹波 ΔI（叠在偏置上，避免过零） */
#define M1_LD_LQ_IDENT_DI_TARGET_A      0.12f
#endif
#ifndef M1_LD_LQ_IDENT_I_RIPPLE_MARGIN_A
/** 偏置轴电流与零的最小距。|I_bias|−|ΔI| 。此。*/
#define M1_LD_LQ_IDENT_I_RIPPLE_MARGIN_A  0.15f
#endif
#ifndef M1_LD_LQ_IDENT_U_INJ_MIN_V
#define M1_LD_LQ_IDENT_U_INJ_MIN_V      0.12f
#endif
#ifndef M1_LD_LQ_IDENT_U_INJ_MAX_V
#define M1_LD_LQ_IDENT_U_INJ_MAX_V      0.50f
#endif
#ifndef M1_LD_LQ_IDENT_SETTLE_S
#define M1_LD_LQ_IDENT_SETTLE_S         0.5f
#endif
/** @deprecated 变幅。U_INJ_MIN/MAX + L 自适应；保。0 以兼。*/
#ifndef M1_LD_LQ_IDENT_V_FRAC_MIN
#define M1_LD_LQ_IDENT_V_FRAC_MIN       0.0f
#endif
#ifndef M1_LD_LQ_IDENT_V_FRAC_MAX
#define M1_LD_LQ_IDENT_V_FRAC_MAX       0.0f
#endif
#ifndef M1_LD_LQ_IDENT_AMP_STEPS
#define M1_LD_LQ_IDENT_AMP_STEPS        7u
#endif
#ifndef M1_LD_LQ_IDENT_CYCLES_PER_AMP
#define M1_LD_LQ_IDENT_CYCLES_PER_AMP   10u
#endif
#ifndef M1_LD_LQ_IDENT_THETA_DRIFT_MECH_DEG
#define M1_LD_LQ_IDENT_THETA_DRIFT_MECH_DEG  8.0f
#endif
/** 1=编码器漂移超限时 ld_lq_abort()。=只记 dbg。 。grid 强制跑完（离线分析用。*/
#ifndef M1_LD_LQ_IDENT_ABORT_ON_THETA_DRIFT
#define M1_LD_LQ_IDENT_ABORT_ON_THETA_DRIFT  0
#endif
#ifndef M1_LD_LQ_IDENT_L_MIN_H
#define M1_LD_LQ_IDENT_L_MIN_H          20e-6f
#endif
#ifndef M1_LD_LQ_IDENT_EPS_TRACK_A
#define M1_LD_LQ_IDENT_EPS_TRACK_A      0.10f
#endif
#ifndef M1_LD_LQ_IDENT_ZERO_BIAS_SUM_A
#define M1_LD_LQ_IDENT_ZERO_BIAS_SUM_A  0.15f
#endif
#ifndef M1_LD_LQ_IDENT_SETTLE_MAX_S
#define M1_LD_LQ_IDENT_SETTLE_MAX_S       2.0f
#endif
#ifndef M1_LD_LQ_IDENT_MIN_DI_A
#define M1_LD_LQ_IDENT_MIN_DI_A         0.05f
#endif
/** 1=ψ 积分。(U−U_bias) 去掉偏置 DC，对齐论文对。± 注入 */
#ifndef M1_LD_LQ_IDENT_PSI_USE_U_AC
#define M1_LD_LQ_IDENT_PSI_USE_U_AC     1
#endif
/** 1=VASI INJ_LD/LQ 。deadband LUT runtime；SETTLE/PRE_DECAY 。OFF */
#ifndef M1_LD_LQ_IDENT_INJECT_LUT_ENABLE
#define M1_LD_LQ_IDENT_INJECT_LUT_ENABLE  0
#endif
#if M1_LD_LQ_IDENT_INJECT_LUT_ENABLE && !M1_LD_LQ_IDENT_ENABLE
#error "M1_LD_LQ_IDENT_INJECT_LUT_ENABLE requires M1_LD_LQ_IDENT_ENABLE=1"
#endif
#if M1_LD_LQ_IDENT_INJECT_LUT_ENABLE && !M1_DEADBAND_ENABLE
#error "M1_LD_LQ_IDENT_INJECT_LUT_ENABLE requires M1_DEADBAND_ENABLE=1"
#endif

/** deadband_flow 编译/启动条件（含。OPEN_UD 验表。*/
#ifndef M1_DEADBAND_FLOW_ENABLE
#define M1_DEADBAND_FLOW_ENABLE         \
    (M1_IDENT_ENABLE || M1_ID_LOCK_CAL_SWEEP || M1_SPEED_IDENT_ENABLE || \
     M1_OPEN_UD_PRE_ID_CAL_ENABLE || M1_OPEN_UQ_PRE_ID_CAL_ENABLE)
#endif
/** 正负半周 |u_ac| 最小比值，低于此丢弃该 cycle */
#ifndef M1_LD_LQ_IDENT_U_SYM_RATIO_MIN
#define M1_LD_LQ_IDENT_U_SYM_RATIO_MIN  0.65f
#endif
/** 半周平均 |u_ac| 低于此视为无效注。*/
#ifndef M1_LD_LQ_IDENT_U_SYM_MIN_DV_V
#define M1_LD_LQ_IDENT_U_SYM_MIN_DV_V  0.015f
#endif
/** fine(1 kHz) 额外 v_inj 上限，避免大幅值非线。*/
#ifndef M1_LD_LQ_IDENT_FINE_V_INJ_MAX_V
#define M1_LD_LQ_IDENT_FINE_V_INJ_MAX_V 0.20f
#endif
#ifndef M1_LD_LQ_IDENT_L_MAX_H
#define M1_LD_LQ_IDENT_L_MAX_H          0.001f
#endif
#ifndef M1_LD_LQ_IDENT_MIN_LD_OK
#define M1_LD_LQ_IDENT_MIN_LD_OK        7u
#endif
#ifndef M1_LD_LQ_IDENT_MIN_LQ_OK
#define M1_LD_LQ_IDENT_MIN_LQ_OK        7u
#endif
/** 1=Rs 。30/150/270° 各跑一。9 。VASI（ALIGN Ud 换角，同 Pass0 三角。*/
#ifndef M1_LD_LQ_MULTI_ANGLE_ENABLE
#define M1_LD_LQ_MULTI_ANGLE_ENABLE   0
#endif
#ifndef M1_LD_LQ_IDENT_ANGLE_COUNT
#define M1_LD_LQ_IDENT_ANGLE_COUNT      3u
#endif
#ifndef M1_LD_LQ_ALIGN_S
#define M1_LD_LQ_ALIGN_S                M1_ID_CAL_ALIGN_S
#endif
/** L 辨识多角度电角（默认。Pass0 三角 30/150/270°。*/
#ifndef M1_LD_LQ_THETA0_EL_RAD
#define M1_LD_LQ_THETA0_EL_RAD          M1_ID_CAL_THETA_EL_RAD
#endif
#ifndef M1_LD_LQ_THETA1_EL_RAD
#define M1_LD_LQ_THETA1_EL_RAD          2.6179938779914940f  /* 150° */
#endif
#ifndef M1_LD_LQ_THETA2_EL_RAD
#define M1_LD_LQ_THETA2_EL_RAD          4.7123889803846900f  /* 270° */
#endif
#if M1_LD_LQ_MULTI_ANGLE_ENABLE && !M1_ID_CAL_FIX_THETA_ENABLE
#error "M1_LD_LQ_MULTI_ANGLE_ENABLE requires M1_ID_CAL_FIX_THETA_ENABLE=1"
#endif
#if M1_LD_LQ_MULTI_ANGLE_ENABLE && (M1_LD_LQ_IDENT_ANGLE_COUNT < 1u)
#error "M1_LD_LQ_IDENT_ANGLE_COUNT must be >= 1"
#endif

#if M1_LD_LQ_IDENT_ENABLE && !M1_RS_IDENT_ENABLE && !M1_RS_IDENT_USE_FIXED_NOMINAL
#error "M1_LD_LQ_IDENT_ENABLE requires M1_RS_IDENT_ENABLE=1 or M1_RS_IDENT_USE_FIXED_NOMINAL=1"
#endif

/**
 * Pass0 commit 。Rs+VASI：默认单。deadband OFF。
 * M1_RS_L_IDENT_DUAL_LUT_ROUND_ENABLE=1 时再。LUT 轮对照（open_seq +100）。
 */
#ifndef M1_RS_L_IDENT_DUAL_LUT_ROUND_ENABLE
#define M1_RS_L_IDENT_DUAL_LUT_ROUND_ENABLE  0
#endif
#ifndef M1_RS_L_IDENT_LUT_ROUND_OPEN_SEQ_OFFSET
#define M1_RS_L_IDENT_LUT_ROUND_OPEN_SEQ_OFFSET  100u
#endif
#if M1_RS_L_IDENT_DUAL_LUT_ROUND_ENABLE && (!M1_RS_IDENT_ENABLE || !M1_LD_LQ_IDENT_ENABLE)
#error "M1_RS_L_IDENT_DUAL_LUT_ROUND_ENABLE requires M1_RS_IDENT_ENABLE and M1_LD_LQ_IDENT_ENABLE"
#endif

#ifndef M1_ID_CAL_RS_LD_LQ_ONLY_ENABLE
#define M1_ID_CAL_RS_LD_LQ_ONLY_ENABLE  0
#endif

#if M1_LD_LQ_IDENT_ENABLE && !M1_ID_CAL_COMMIT_LUT && !M1_ID_CAL_RS_LD_LQ_ONLY_ENABLE
#error "M1_LD_LQ_IDENT_ENABLE requires M1_ID_CAL_COMMIT_LUT=1 (or RS_LD_LQ_ONLY)"
#endif
#if M1_LD_LQ_IDENT_ENABLE && M1_ID_CAL_IQ_PROBE_ENABLE
#error "M1_LD_LQ_IDENT_ENABLE and M1_ID_CAL_IQ_PROBE_ENABLE are mutually exclusive"
#endif

#ifndef M1_ID_CAL_IQ_PROBE_ONLY_ENABLE
#define M1_ID_CAL_IQ_PROBE_ONLY_ENABLE  0
#endif
#ifndef M1_ID_CAL_IQ_PROBE_LUT_AFTER_OFF
#define M1_ID_CAL_IQ_PROBE_LUT_AFTER_OFF  1
#endif
#if M1_ID_CAL_IQ_PROBE_ONLY_ENABLE && !M1_ID_CAL_IQ_PROBE_ENABLE
#error "M1_ID_CAL_IQ_PROBE_ONLY_ENABLE requires M1_ID_CAL_IQ_PROBE_ENABLE=1"
#endif
#if M1_ID_CAL_IQ_PROBE_ONLY_ENABLE && M1_ID_CAL_PASS0_ONLY_ENABLE
#error "M1_ID_CAL_IQ_PROBE_ONLY_ENABLE requires M1_ID_CAL_PASS0_ONLY_ENABLE=0"
#endif
#if M1_ID_CAL_IQ_PROBE_ONLY_ENABLE && (M1_RS_IDENT_ENABLE || M1_LD_LQ_IDENT_ENABLE)
#error "M1_ID_CAL_IQ_PROBE_ONLY_ENABLE is incompatible with RS/L ident"
#endif
#if M1_ID_CAL_RS_LD_LQ_ONLY_ENABLE && !M1_ID_LOCK_CAL_SWEEP
#error "M1_ID_CAL_RS_LD_LQ_ONLY_ENABLE requires M1_ID_LOCK_CAL_SWEEP=1"
#endif
#if M1_ID_CAL_RS_LD_LQ_ONLY_ENABLE && \
    (!M1_LD_LQ_IDENT_ENABLE || (!M1_RS_IDENT_ENABLE && !M1_RS_IDENT_USE_FIXED_NOMINAL))
#error "M1_ID_CAL_RS_LD_LQ_ONLY_ENABLE requires M1_LD_LQ_IDENT_ENABLE and (M1_RS_IDENT_ENABLE or M1_RS_IDENT_USE_FIXED_NOMINAL)"
#endif
#if M1_ID_CAL_RS_LD_LQ_ONLY_ENABLE && M1_ID_CAL_PASS0_ONLY_ENABLE
#error "M1_ID_CAL_RS_LD_LQ_ONLY_ENABLE requires M1_ID_CAL_PASS0_ONLY_ENABLE=0"
#endif
#if M1_ID_CAL_RS_LD_LQ_ONLY_ENABLE && M1_ID_CAL_IQ_PROBE_ENABLE
#error "M1_ID_CAL_RS_LD_LQ_ONLY_ENABLE is incompatible with M1_ID_CAL_IQ_PROBE_ENABLE"
#endif
#if M1_ID_CAL_IQ_PROBE_ENABLE && !M1_ID_CAL_COMMIT_LUT && !M1_ID_CAL_IQ_PROBE_ONLY_ENABLE
#error "M1_ID_CAL_IQ_PROBE_ENABLE requires M1_ID_CAL_COMMIT_LUT=1 (or IQ_PROBE_ONLY)"
#endif

/**
 * 标定 Park 角固。30° 电角（π/6）；Iq=0 。d↔a 换算最简。
 * =0：沿用编码器 theta_enc_park（旧行为）。
 * 编码器仍采样；仅 Park/InvPark/SVPWM 用固定角。
 */
#ifndef M1_ID_CAL_FIX_THETA_ENABLE
#define M1_ID_CAL_FIX_THETA_ENABLE   1
#endif
#ifndef M1_ID_CAL_THETA_EL_RAD
#define M1_ID_CAL_THETA_EL_RAD       0.5235987755982988f  /* pi/6 */
#endif
/** cos(30°)；Step 2 commit d→phase 表时使用 */
#ifndef M1_ID_CAL_D_TO_PHASE_COS
#define M1_ID_CAL_D_TO_PHASE_COS     0.8660254037844386f
#endif

/**
 * FIX_THETA=1 时上电先开。Ud 对齐。M1_ID_CAL_THETA_EL_RAD，再 Id 扫表。
 * 。M1_ID_CAL_FIX_THETA_ENABLE=1。
 */
#ifndef M1_ID_CAL_ALIGN_ENABLE
#define M1_ID_CAL_ALIGN_ENABLE       1
#endif
#ifndef M1_ID_CAL_ALIGN_UD_V
#define M1_ID_CAL_ALIGN_UD_V         3.0f
#endif
#ifndef M1_ID_CAL_ALIGN_S
#define M1_ID_CAL_ALIGN_S            0.5f
#endif

/**
 * 论文 §4.4 双特殊角标定：Pass0-A @30° + Pass0-B @0°，abc 样本合并。phase 表。
 * =0：仅 Pass0-A。0°），。Phase 1 行为相同。
 */
#ifndef M1_ID_CAL_DUAL_ANGLE_ENABLE
#define M1_ID_CAL_DUAL_ANGLE_ENABLE  1
#endif
#ifndef M1_ID_CAL_THETA_PASS0_A_RAD
#define M1_ID_CAL_THETA_PASS0_A_RAD  M1_ID_CAL_THETA_EL_RAD  /* 30° 位置 1 */
#endif
#ifndef M1_ID_CAL_THETA_PASS0_B_RAD
#define M1_ID_CAL_THETA_PASS0_B_RAD  0.0f                    /* 0° 位置 2 */
#endif
#ifndef M1_ID_CAL_ALIGN_B_UD_V
#define M1_ID_CAL_ALIGN_B_UD_V       M1_ID_CAL_ALIGN_UD_V
#endif
#ifndef M1_ID_CAL_ALIGN_B_S
#define M1_ID_CAL_ALIGN_B_S          M1_ID_CAL_ALIGN_S
#endif
/**
 * Pass0-B @0° 各档 dwell (s)。0 时覆。ID_DWELL_*。=。Pass0-A 相同。
 * 058 录波 0° 。Id 欠流 7。0% 时，可试 1.0。.0 s 再验。capture。
 */
#ifndef M1_ID_CAL_PASS0_B_DWELL_S
#define M1_ID_CAL_PASS0_B_DWELL_S    0.0f
#endif

/**
 * =1：Pass0 多电角扫 Id（deadband OFF + capture）。
 * COUNT=3。0/150/270° 三角（强相轮换，查三相不平衡）；
 * COUNT=5。/30/60/90/120° 五角（MULTI_ANGLE_PASS0 模式）。
 * geo 。= AMP_TABLE_LEN×3×COUNT；s_dlut 锚仍。30° 档写入。
 * 录波后：`python tools/multi_angle_geo_analysis.py <csv> --angles 30,150,270`
 */
#ifndef M1_ID_CAL_MULTI_ANGLE_ENABLE
#define M1_ID_CAL_MULTI_ANGLE_ENABLE  0
#endif
#if M1_ID_CAL_MULTI_ANGLE_ENABLE
#ifndef M1_ID_CAL_PASS0_ANGLE_COUNT
#define M1_ID_CAL_PASS0_ANGLE_COUNT   5
#endif
#ifndef M1_ID_CAL_THETA_PASS0_0_RAD
#define M1_ID_CAL_THETA_PASS0_0_RAD   0.0f
#endif
#ifndef M1_ID_CAL_THETA_PASS0_150_RAD
#define M1_ID_CAL_THETA_PASS0_150_RAD 2.6179938779914940f  /* 5pi/6, 150° */
#endif
#ifndef M1_ID_CAL_THETA_PASS0_270_RAD
#define M1_ID_CAL_THETA_PASS0_270_RAD 4.7123889803846900f  /* 3pi/2, 270° */
#endif
#ifndef M1_ID_CAL_THETA_PASS0_60_RAD
#define M1_ID_CAL_THETA_PASS0_60_RAD  1.0471975511965976f  /* pi/3, 60° */
#endif
#ifndef M1_ID_CAL_THETA_PASS0_90_RAD
#define M1_ID_CAL_THETA_PASS0_90_RAD  1.5707963267948966f  /* pi/2, 90° */
#endif
#ifndef M1_ID_CAL_THETA_PASS0_120_RAD
#define M1_ID_CAL_THETA_PASS0_120_RAD 2.0943951023931953f  /* 2pi/3, 120° */
#endif
#undef M1_DEADBAND_GEO_SAMPLE_MAX
#define M1_DEADBAND_GEO_SAMPLE_MAX    \
    (M1_ID_CAL_AMP_TABLE_LEN * 3u * (uint32_t)M1_ID_CAL_PASS0_ANGLE_COUNT)
#if M1_ID_CAL_PASS0_ANGLE_COUNT < 2
#error "M1_ID_CAL_PASS0_ANGLE_COUNT must be >= 2"
#endif
#endif /* M1_ID_CAL_MULTI_ANGLE_ENABLE */

#if M1_ID_CAL_DUAL_ANGLE_ENABLE && !M1_ID_CAL_FIX_THETA_ENABLE
#error "M1_ID_CAL_DUAL_ANGLE_ENABLE requires M1_ID_CAL_FIX_THETA_ENABLE=1"
#endif
#if M1_ID_CAL_DUAL_ANGLE_ENABLE && \
    (!M1_ID_CAL_LUT_VERIFY_SWEEP || !M1_ID_CAL_COMMIT_LUT) && \
    !M1_ID_CAL_PASS0_ONLY_ENABLE && \
    !M1_ID_CAL_IQ_PROBE_ENABLE && !M1_RS_IDENT_ENABLE
#error "M1_ID_CAL_DUAL_ANGLE_ENABLE requires LUT_VERIFY+COMMIT, PASS0_ONLY, IQ_PROBE, or RS_IDENT+COMMIT"
#endif

/** Pass0 末衰减：双角 leg 切换 / commit 。Id。 */
#if M1_ID_CAL_PASS0_ONLY_ENABLE || \
    (M1_ID_CAL_COMMIT_LUT && (M1_ID_CAL_LUT_VERIFY_SWEEP || M1_ID_CAL_IQ_PROBE_ENABLE || \
                              M1_RS_IDENT_ENABLE || M1_IDENT_ID_CAL_BEFORE_STEP))
#define M1_ID_CAL_PASS0_DECAY_ENABLE  1
#else
#define M1_ID_CAL_PASS0_DECAY_ENABLE  0
#endif

#if M1_ID_CAL_ALIGN_ENABLE && !M1_ID_CAL_FIX_THETA_ENABLE
#error "M1_ID_CAL_ALIGN_ENABLE requires M1_ID_CAL_FIX_THETA_ENABLE=1"
#endif

/** 双角 Pass0 abc 样本池上限；IDENT 单角覆写。AMP×3 */
#ifndef M1_DEADBAND_GEO_SAMPLE_MAX
#define M1_DEADBAND_GEO_SAMPLE_MAX        (M1_ID_CAL_AMP_TABLE_LEN * 3u * 2u)
#endif

/**
 * =0：telem_lut_dump 不编译（stub）；日常固件关。
 */
#ifndef M1_VOFA_LUT_DUMP_ENABLE
#define M1_VOFA_LUT_DUMP_ENABLE     1
#endif

/**
 * =0：telem_ident_dump 不编译（stub）；Rs/Ld-Lq 辨识结果不突。VOFA。
 */
#ifndef M1_VOFA_IDENT_DUMP_ENABLE
#define M1_VOFA_IDENT_DUMP_ENABLE   0
#endif

#else /* !M1_ID_LOCK_CAL_SWEEP */

#ifndef M1_ID_CAL_FIX_THETA_ENABLE
#define M1_ID_CAL_FIX_THETA_ENABLE   0
#endif

#ifndef M1_VOFA_LUT_DUMP_ENABLE
#define M1_VOFA_LUT_DUMP_ENABLE     0
#endif

#ifndef M1_VOFA_IDENT_DUMP_ENABLE
#define M1_VOFA_IDENT_DUMP_ENABLE   0
#endif

#endif /* M1_ID_LOCK_CAL_SWEEP */

/* --- 死区补偿（固定符号法。--- */
/**
 * 电流。deadband A/B 三档（开。0623 已扫 400/591）：
 *   OFF（M1_DEADBAND_ENABLE=0）。不过补，基线。vofa+202606240039
 *   400 ns 。开环折中，。.192 V/相（。M1_DEADTIME_NS 再编译）
 *   591 ns 。本档过补，≈0.284 V/相（当前默认 ON，闭环复测用。
 */
#ifndef M1_DEADBAND_ENABLE
#define M1_DEADBAND_ENABLE      0
#endif

/** 有效死区时间（ns）；标定模式强制 OFF；电流环 A/B 。240039/240051 报告 */
#define M1_DEADTIME_NS          591u

/** PWM 周期（s），。M1_CTRL_TS_S / TIM8 20 kHz 一。*/
#define M1_PWM_PERIOD_S         M1_CTRL_TS_S

/** 每相固定补偿电压：Vbus × t_dead / T_pwm 。0.284 V @ 24 V, 591 ns */
#define M1_DEADBAND_V_COMP_V    (M1_VBUS_V * (float)M1_DEADTIME_NS * 1.0e-9f / M1_PWM_PERIOD_S)

/** 归一化占空比补偿。= t_dead / T_pwm */
#define M1_DEADBAND_DUTY_COMP   (M1_DEADBAND_V_COMP_V / M1_VBUS_V)

/** 过零区：|i| 低于此值不补偿（仅 M1_DEADBAND_I_ZERO_DISABLE=0 时生效） */
#define M1_DEADBAND_I_ZERO_A    0.05f

/**
 * A/B 联调。1 关闭过零死区，i。 即按符号全幅补偿。
 * 低电流请优先。M1_DEADBAND_LUT_APPLY_MIN_A。
 */
#ifndef M1_DEADBAND_I_ZERO_DISABLE
#define M1_DEADBAND_I_ZERO_DISABLE   0
#endif

/** LUT 注入。=d 。Ud。830 过渡）；0=abc 单相表（Phase A 终态） */
#ifndef M1_DEADBAND_LUT_APPLY_UD
#define M1_DEADBAND_LUT_APPLY_UD  1   /* Phase 1：Ud 补偿能力；路径由 runtime 标志控制 */
#endif

/**
 * phase abc 注入：三。duty 补偿后去零序 u₀=(Δa+Δb+Δc)/3（论。4.4 运行时修正）。
 * 锁轴。u₀。；旋转时强制三相补偿之和为零。
 */
#ifndef M1_DEADBAND_LUT_ZERO_SEQ_ENABLE
#define M1_DEADBAND_LUT_ZERO_SEQ_ENABLE  1
#endif

/**
 * LUT 运行时硬门槛：|i_phase| < APPLY_MIN_A 。comp=0，否则全查表。
 * 硬切会在 ~0.4 A 形成补偿阶跃，污。dq 。保持 0。
 * 低电流区。LOW_FLAT 。capture 曲线本身，勿用本开关。
 */
#ifndef M1_DEADBAND_LUT_APPLY_MIN_ENABLE
#define M1_DEADBAND_LUT_APPLY_MIN_ENABLE   0
#endif
#ifndef M1_DEADBAND_LUT_APPLY_MIN_A
#define M1_DEADBAND_LUT_APPLY_MIN_A        0.40f
#endif

/**
 * phase 。apply_duty 运行缩放（仅 lut_domain=phase，不影响 d 。Ud 辨识路径）。
 * 锁轴 Pass0 学的 |u'| 往往大于 Iq 旋转过零修正所需；联调可。0.2。.3 扫。
 */
#ifndef M1_DEADBAND_LUT_RUNTIME_SCALE
#define M1_DEADBAND_LUT_RUNTIME_SCALE       1.0f
#endif

/**
 * =1：commit 。vals[] *= V_FIXED/lut(Iq_probe×cos30°)，runtime scale 。1（论。#8）。
 * 。M1_DEADBAND_LUT_RUNTIME_SCALE_AUTO 互斥。
 */
#ifndef M1_DEADBAND_LUT_COMMIT_NORMALIZE
#define M1_DEADBAND_LUT_COMMIT_NORMALIZE    0
#endif

#if M1_DEADBAND_LUT_COMMIT_NORMALIZE && M1_DEADBAND_LUT_RUNTIME_SCALE_AUTO
#error "M1_DEADBAND_LUT_COMMIT_NORMALIZE and M1_DEADBAND_LUT_RUNTIME_SCALE_AUTO are mutually exclusive"
#endif

#ifndef M1_DEADBAND_RUNTIME_GEO_ENABLE
#define M1_DEADBAND_RUNTIME_GEO_ENABLE      0
#endif

#ifndef M1_DEADBAND_GEO_TWO_CLUSTER_ENABLE
#define M1_DEADBAND_GEO_TWO_CLUSTER_ENABLE  0
#endif

/** TWO_CLUSTER runtime 分簇电角（NORMAL 运行期也需；与 Pass0 标定角一致） */
#ifndef M1_ID_CAL_THETA_EL_RAD
#define M1_ID_CAL_THETA_EL_RAD       0.5235987755982988f  /* pi/6 = 30° */
#endif
#ifndef M1_ID_CAL_THETA_PASS0_A_RAD
#define M1_ID_CAL_THETA_PASS0_A_RAD  M1_ID_CAL_THETA_EL_RAD
#endif
#ifndef M1_ID_CAL_THETA_PASS0_B_RAD
#define M1_ID_CAL_THETA_PASS0_B_RAD  0.0f
#endif
#ifndef M1_ID_CAL_THETA_MATCH_RAD
#define M1_ID_CAL_THETA_MATCH_RAD    0.02f
#endif

#if M1_DEADBAND_RUNTIME_GEO_ENABLE && !M1_DEADBAND_LUT_APPLY_UD
#error "M1_DEADBAND_RUNTIME_GEO_ENABLE requires M1_DEADBAND_LUT_APPLY_UD=1 (d-table at commit)"
#endif

/**
 * commit 时把 phase LUT 。Id 区改为线性（去掉 0.15~0.2 A 陡升）。
 * 0=保留 capture 曲线。308 对照）；1=平坦。Id∈[首点, M1_DEADBAND_LUT_LOW_FLAT_ID_A]。
 * Phase 1 对照。026-06-27）：0=关平坦化，验。2015 。Id 失败是否。LOW_FLAT 放大。
 */
#ifndef M1_DEADBAND_LUT_LOW_FLAT_ENABLE
#define M1_DEADBAND_LUT_LOW_FLAT_ENABLE   0
#endif
#ifndef M1_DEADBAND_LUT_LOW_FLAT_ID_A
#define M1_DEADBAND_LUT_LOW_FLAT_ID_A     0.30f
#endif

/**
 * Phase 3 geo 建表（论。§4.4 ②③④⑤）：
 * 0=×0.866。=双角 s_geo_samples 合并拟合 s_plut（须 DUAL_ANGLE=1）。
 */
#ifndef M1_DEADBAND_GEO_BUILD_ENABLE
#define M1_DEADBAND_GEO_BUILD_ENABLE      1
#endif

/**
 * =1：merge plut 。val 仅用 Pass0-A。0°）样本；amp 仍取 30° max|i|。
 * 0010 双角 median 。0° Ud 偏高时污染小 I / 造成 val 跳变。
 */
#ifndef M1_DEADBAND_GEO_MERGE_VAL30_ONLY
#define M1_DEADBAND_GEO_MERGE_VAL30_ONLY    0
#endif

/** geo 样本池：Pass0-B 单点 |u'| 低于此值丢弃（。capture / 3A 欠流。*/
#ifndef M1_ID_CAL_GEO_U_MIN_V
#define M1_ID_CAL_GEO_U_MIN_V               0.10f
#endif

/**
 * commit 。=1 。fa/fb/fc 三表。0 。proposed 共享 plut。0° amp + 双角 median val）。
 * P0 签收先用 0（离线金标准 ~1.47 V/相）。146 triplet val。.95 V 暂不启用。
 */
#ifndef M1_DEADBAND_GEO_TRIPLET_ENABLE
#define M1_DEADBAND_GEO_TRIPLET_ENABLE    0
#endif

/** GEO_BUILD=0 时旁路对。geo vs ×0.866，更。deadband_geo_diff_* */
#ifndef M1_DEADBAND_GEO_DIFF_LOG_ENABLE
#define M1_DEADBAND_GEO_DIFF_LOG_ENABLE   1
#endif

/**
 * 实验 A/B（Id cal 前）：ALIGN(68) 。开。Uq 。Ud 阶梯 0/0.2/0.5/1/2/4 V。0..75）→ 77。
 * flow：OPEN_UD_LADDER 。OPEN_UQ_LADDER 。Id Pass0 。ident Bode 。。
 * dwell=M1_OPEN_PRE_ID_LADDER_DWELL_S（默。0.3 s）；θ 固定 30°。
 * 实验 A/B VOFA×12（D=2, 10kHz）：ch0=Ud_out ch1=Uq_out ch2=Vd_est ch3=Vq_est
 *   ch4=Id ch5=Iq ch6=CCR1 ch7=CCR2 ch8=CCR3 ch9=Uref ch10=sector ch11=open_seq_phase
 */
#ifndef M1_OPEN_UQ_PRE_ID_CAL_ENABLE
#define M1_OPEN_UQ_PRE_ID_CAL_ENABLE  0
#endif
#ifndef M1_OPEN_UD_PRE_ID_CAL_ENABLE
#define M1_OPEN_UD_PRE_ID_CAL_ENABLE  0
#endif
/** 1=Pass0/IdCal 完成后自动接 Ud 开环阶梯（recipe: ID_CAL 。OPEN_UD）；0=阶梯。IdCal 前（ident before step。*/
#ifndef M1_OPEN_UD_AFTER_ID_CAL
#define M1_OPEN_UD_AFTER_ID_CAL         0
#endif
#if M1_OPEN_UD_AFTER_ID_CAL && !M1_OPEN_UD_PRE_ID_CAL_ENABLE
#error "M1_OPEN_UD_AFTER_ID_CAL requires M1_OPEN_UD_PRE_ID_CAL_ENABLE=1"
#endif
/** 1=Ud 阶梯完成后同次上电接 VASI Ld/Lq（recipe 末步 DEADBAND_FLOW_KIND_LD_LQ_IDENT。*/
#ifndef M1_DEADBAND_FLOW_LD_LQ_AFTER_OPEN_UD
#define M1_DEADBAND_FLOW_LD_LQ_AFTER_OPEN_UD  0
#endif
#if M1_DEADBAND_FLOW_LD_LQ_AFTER_OPEN_UD && !M1_LD_LQ_IDENT_ENABLE
#error "M1_DEADBAND_FLOW_LD_LQ_AFTER_OPEN_UD requires M1_LD_LQ_IDENT_ENABLE=1"
#endif
#if M1_DEADBAND_FLOW_LD_LQ_AFTER_OPEN_UD && !M1_OPEN_UD_AFTER_ID_CAL
#error "M1_DEADBAND_FLOW_LD_LQ_AFTER_OPEN_UD requires M1_OPEN_UD_AFTER_ID_CAL=1"
#endif
#if M1_OPEN_UQ_PRE_ID_CAL_ENABLE && M1_OPEN_UD_PRE_ID_CAL_ENABLE
#error "M1_OPEN_UQ_PRE_ID_CAL_ENABLE and M1_OPEN_UD_PRE_ID_CAL_ENABLE are mutually exclusive"
#endif
#if M1_IF_ENABLE && (M1_STARTUP_ENABLE || M1_OPEN_UD_PRE_ID_CAL_ENABLE || \
                     M1_OPEN_UQ_PRE_ID_CAL_ENABLE)
#error "M1_IF_ENABLE mutually exclusive with Uq/Ud open-loop drag (STARTUP / OPEN_UD / OPEN_UQ)"
#endif
#if M1_IF_ENABLE && M1_OBS_SOFT_SWITCH_ENABLE && !M1_IF_TO_OBS_ENABLE
#error "M1_IF_ENABLE + OBS_SS requires M1_IF_TO_OBS_ENABLE=1"
#endif
#if M1_IF_TO_OBS_ENABLE && !M1_OBS_SOFT_SWITCH_ENABLE
#error "M1_IF_TO_OBS_ENABLE requires M1_OBS_SOFT_SWITCH_ENABLE=1"
#endif
#if M1_IF_TO_OBS_ENABLE && !M1_EMF_PLL_ENABLE
#error "M1_IF_TO_OBS_ENABLE requires M1_EMF_PLL_ENABLE=1"
#endif
#if M1_USE_IF_100_PROFILE && !M1_IF_ENABLE
#error "M1_USE_IF_100_PROFILE requires M1_IF_ENABLE=1"
#endif
#if M1_USE_HFI_STANDSTILL_PROFILE && !M1_HFI_ENABLE
#error "M1_USE_HFI_STANDSTILL_PROFILE requires M1_HFI_ENABLE=1 (late check)"
#endif
#if M1_IF_OBS_DIR_SEQ_ENABLE && !M1_IF_OBS_CRUISE_ENABLE
#error "M1_IF_OBS_DIR_SEQ_ENABLE requires M1_IF_OBS_CRUISE_ENABLE=1"
#endif
#if M1_IF_OBS_DIR_SEQ_ENABLE && !M1_IF_TO_OBS_ENABLE
#error "M1_IF_OBS_DIR_SEQ_ENABLE requires M1_IF_TO_OBS_ENABLE=1"
#endif
#if M1_IF_OBS_DIR_SEQ_ENABLE && (M1_IF_OBS_CRUISE_STEP_ENABLE || M1_IF_OBS_CRUISE_S3_PROBE_ENABLE)
#error "M1_IF_OBS_DIR_SEQ_ENABLE: disable CRUISE_STEP and S3_PROBE"
#endif
/** 1=Ud/Uq 开。ladder 。LUT runtime abc（Pass0 后验表）。=deadband OFF */
#ifndef M1_OPEN_PRE_ID_LADDER_LUT_RUNTIME
#define M1_OPEN_PRE_ID_LADDER_LUT_RUNTIME  0
#endif
#if M1_OPEN_PRE_ID_LADDER_LUT_RUNTIME && !M1_DEADBAND_ENABLE
#error "M1_OPEN_PRE_ID_LADDER_LUT_RUNTIME requires M1_DEADBAND_ENABLE=1"
#endif

#if M1_OPEN_UQ_PRE_ID_CAL_ENABLE || M1_OPEN_UD_PRE_ID_CAL_ENABLE
/** ALIGN(68) 。0/0.2/0.5/1/2/4 V 阶梯(70..75) 。77 结束；实。A=Uq，实。B=Ud */
#ifndef M1_OPEN_PRE_ID_LADDER_ALIGN_ENABLE
#define M1_OPEN_PRE_ID_LADDER_ALIGN_ENABLE      1
#endif
#ifndef M1_OPEN_PRE_ID_LADDER_FIX_THETA_ENABLE
#define M1_OPEN_PRE_ID_LADDER_FIX_THETA_ENABLE  1
#endif
#ifndef M1_OPEN_PRE_ID_LADDER_THETA_EL_RAD
#define M1_OPEN_PRE_ID_LADDER_THETA_EL_RAD      M1_ID_CAL_THETA_EL_RAD  /* 30° */
#endif
#ifndef M1_OPEN_PRE_ID_LADDER_ALIGN_UD_V
#define M1_OPEN_PRE_ID_LADDER_ALIGN_UD_V        M1_ID_CAL_ALIGN_UD_V    /* 3 V */
#endif
#ifndef M1_OPEN_PRE_ID_LADDER_ALIGN_S
#define M1_OPEN_PRE_ID_LADDER_ALIGN_S           M1_ID_CAL_ALIGN_S       /* 0.5 s */
#endif
#ifndef M1_OPEN_PRE_ID_LADDER_V4
#define M1_OPEN_PRE_ID_LADDER_V4                4.0f
#endif
#ifndef M1_OPEN_PRE_ID_LADDER_N
#define M1_OPEN_PRE_ID_LADDER_N                 6u
#endif
#ifndef M1_OPEN_PRE_ID_LADDER_DWELL_S
#define M1_OPEN_PRE_ID_LADDER_DWELL_S            0.3f
#endif
#if M1_OPEN_PRE_ID_LADDER_ALIGN_ENABLE && !M1_OPEN_PRE_ID_LADDER_FIX_THETA_ENABLE
#error "M1_OPEN_PRE_ID_LADDER_ALIGN_ENABLE requires M1_OPEN_PRE_ID_LADDER_FIX_THETA_ENABLE=1"
#endif
#define M1_OPEN_PRE_ID_LADDER_ALIGN_PHASE       68u
#define M1_OPEN_PRE_ID_LADDER_PHASE_BASE        70u
#define M1_OPEN_PRE_ID_LADDER_DONE_PHASE        (M1_OPEN_PRE_ID_LADDER_PHASE_BASE + M1_OPEN_PRE_ID_LADDER_N + 1u)
#ifndef M1_OPEN_PRE_ID_LADDER_AB_ENABLE
#define M1_OPEN_PRE_ID_LADDER_AB_ENABLE         0
#endif
#if M1_OPEN_PRE_ID_LADDER_AB_ENABLE && !M1_DEADBAND_ENABLE
#error "M1_OPEN_PRE_ID_LADDER_AB_ENABLE requires M1_DEADBAND_ENABLE=1"
#endif
#define M1_OPEN_PRE_ID_LADDER_PHASE_ON_BASE     (M1_OPEN_PRE_ID_LADDER_PHASE_BASE + 10u)
#define M1_OPEN_PRE_ID_LADDER_DONE_ON_PHASE     (M1_OPEN_PRE_ID_LADDER_PHASE_ON_BASE + M1_OPEN_PRE_ID_LADDER_N + 1u)
/** @deprecated 实验 A 别名 */
#define M1_OPEN_UQ_LADDER_ALIGN_ENABLE      M1_OPEN_PRE_ID_LADDER_ALIGN_ENABLE
#define M1_OPEN_UQ_LADDER_FIX_THETA_ENABLE  M1_OPEN_PRE_ID_LADDER_FIX_THETA_ENABLE
#define M1_OPEN_UQ_LADDER_THETA_EL_RAD      M1_OPEN_PRE_ID_LADDER_THETA_EL_RAD
#define M1_OPEN_UQ_LADDER_ALIGN_UD_V        M1_OPEN_PRE_ID_LADDER_ALIGN_UD_V
#define M1_OPEN_UQ_LADDER_ALIGN_S           M1_OPEN_PRE_ID_LADDER_ALIGN_S
#define M1_OPEN_UQ_LADDER_ALIGN_PHASE       M1_OPEN_PRE_ID_LADDER_ALIGN_PHASE
#define M1_OPEN_UQ_LADDER_DWELL_S           M1_OPEN_PRE_ID_LADDER_DWELL_S
#define M1_OPEN_UQ_LADDER_PHASE_BASE        M1_OPEN_PRE_ID_LADDER_PHASE_BASE
#define M1_OPEN_UQ_LADDER_DONE_PHASE        M1_OPEN_PRE_ID_LADDER_DONE_PHASE
#endif

/**
 * 开。Uq 阶梯 + 死区 A/B。026-06-23 综合录波。
 *
 * M1_OPEN_UQ_DEADBAND_AB_SWEEP=1 。15 s 。6 段（。2.5 s）：
 *   phase 0: Uq=2.0 V, deadband OFF
 *   phase 1: Uq=2.0 V, deadband ON  (M1_DEADTIME_NS)
 *   phase 2: Uq=2.5 V, deadband OFF
 *   phase 3: Uq=2.5 V, deadband ON
 *   phase 4: Uq=3.0 V, deadband OFF
 *   phase 5: Uq=3.0 V, deadband ON
 * Watch: dbg.open_seq_phase = 0..5；VOFA ch4=扇区(M1_VOFA_SECTOR_DIAG=1)
 *
 * =0 时沿用旧逻辑。/2.5/3 V 。M1_OPEN_UQ_SWEEP_STEP_S（默。5 s），死区。M1_DEADBAND_ENABLE 编译决定。
 */
#ifndef M1_OPEN_UQ_DEADBAND_AB_SWEEP
#define M1_OPEN_UQ_DEADBAND_AB_SWEEP  0
#endif

/** A/B  sweep 每半段时长（s）；6 段合。15 s */
#define M1_OPEN_UQ_HALF_STEP_S        2.5f

#define M1_OPEN_UQ_SWEEP_STEP_S       5.0f
#define M1_OPEN_UQ_SWEEP_V0           2.0f
#define M1_OPEN_UQ_SWEEP_V1           2.5f
#define M1_OPEN_UQ_SWEEP_V2           3.0f

/** @deprecated 。M1_OPEN_UQ_SWEEP_V0 */
#define M1_OPEN_UQ_RUN_V          M1_OPEN_UQ_SWEEP_V0

/**
 * ADC2 注入 rank 交换试验（须。binding 同步，见 docs/VOFA联调记录_20260617_扇区诊断全CSV.md §6）：
 * 0=默认 JDR1=PC3(ic) JDR3=PC4(ia)，binding [2,1,0]
 * 1=交换 JDR1=PC4(ia) JDR3=PC3(ic)，binding [0,1,2]（ic 最后采。
 */
#ifndef M1_ADC_RANK_SWAP_IAIC
#define M1_ADC_RANK_SWAP_IAIC     0
#endif

/* --- 扇区条件电流重构：仅扇区 1。 重建 ic。80010 路线；CCR4 扫时。0。--- */
#ifndef M1_CURRENT_RECON_ENABLE
#define M1_CURRENT_RECON_ENABLE   1
#endif

/** |i| 低于此值（A）时不重构，避免零漂放大 */
#ifndef M1_CURRENT_RECON_MIN_A
#define M1_CURRENT_RECON_MIN_A    0.05f
#endif

/* --- Iq 环辨识（堵转 + 磁粉制动器；。Id 扫表互斥，由 M1_BRINGUP_MODE 开关） --- */
/**
 * =1：堵。CURRENT_LOOP；可选先 Id Pass0 commit，再 Bode 。Iq 阶跃。
 *     Park 角：编码。θ（FIX_THETA=0）；M1_IDENT_OVERRIDE_LIMITS=1。
 *     IDENT 模式：Id Pass0 建表 。HOLD 2s 。阶跃×8(。+。A，OFF/LUT。) 。Bode×4(0.25/1.25A)。
 * VOFA：Id 。ch3=Ud ch4=Id_ref；Ident ch3=Iq ch4=Iq_ref ch5=Uq_pi。
 * open_seq：Id 1..N。。0=HOLD 61..68=Step-OFF 74..79=Step-LUT 62=Bode-OFF 63=Bode-LUT 73=DONE。
 * 。M1_BRINGUP_MODE_IDENT_IQ_STEP 时自。=1。
 */
#ifndef M1_IDENT_ENABLE
#define M1_IDENT_ENABLE             0
#endif

#ifndef M1_IDENT_POST_BODE_OPEN_UQ_ENABLE
#define M1_IDENT_POST_BODE_OPEN_UQ_ENABLE  0
#endif
#ifndef M1_IDENT_POST_BODE_OPEN_UQ_V
#define M1_IDENT_POST_BODE_OPEN_UQ_V         M1_OPEN_UQ_SWEEP_V0
#endif
#ifndef M1_IDENT_POST_BODE_OPEN_UQ_S
#define M1_IDENT_POST_BODE_OPEN_UQ_S         5.0f
#endif

#if M1_IDENT_ENABLE
#if M1_ID_LOCK_CAL_SWEEP && !M1_IDENT_ID_CAL_BEFORE_STEP
#error "M1_IDENT_ENABLE requires M1_ID_LOCK_CAL_SWEEP=0 (or M1_IDENT_ID_CAL_BEFORE_STEP=1)"
#endif

#ifndef M1_IDENT_ID_CAL_BEFORE_STEP
#define M1_IDENT_ID_CAL_BEFORE_STEP   0
#endif

#ifndef M1_IDENT_IQ_STEP_ENABLE
#define M1_IDENT_IQ_STEP_ENABLE     1
#endif
#ifndef M1_IDENT_IQ_BODE_ENABLE
#define M1_IDENT_IQ_BODE_ENABLE     0   /* 先阶跃；=1 。STEP 后接 BODE */
#endif
#if !M1_IDENT_IQ_STEP_ENABLE && !M1_IDENT_IQ_BODE_ENABLE
#error "M1_IDENT_ENABLE requires M1_IDENT_IQ_STEP_ENABLE and/or M1_IDENT_IQ_BODE_ENABLE"
#endif

#ifndef M1_IDENT_FIX_THETA_ENABLE
#define M1_IDENT_FIX_THETA_ENABLE   0   /* 0=编码。θ（堵转默认）。=写死 30° */
#endif
#ifndef M1_IDENT_THETA_EL_RAD
#define M1_IDENT_THETA_EL_RAD       0.5235987755982988f  /* 。FIX_THETA=1 时用 */
#endif

/** 制动器加载后稳定等待 (s) */
#ifndef M1_IDENT_HOLD_S
#define M1_IDENT_HOLD_S               2.0f
#endif
/** 阶跃总轮。/ OFF / FIXED：IDENT 模式在上。#elif 已设。18 / 6 / 6；此处仅为未覆盖时的兜底 */
#ifndef M1_IDENT_STEP_ROUNDS
#define M1_IDENT_STEP_ROUNDS          18u
#endif
#ifndef M1_IDENT_STEP_OFF_ROUNDS
#define M1_IDENT_STEP_OFF_ROUNDS      6u
#endif
#ifndef M1_IDENT_STEP_FIXED_ROUNDS
#define M1_IDENT_STEP_FIXED_ROUNDS    6u
#endif
/** 1 。FIXED_ROUNDS=0：OFF 之后全用 LUT；FIXED_ROUNDS>0 时第三段恒为 LUT */
#ifndef M1_IDENT_STEP_LUT_AFTER_OFF
#define M1_IDENT_STEP_LUT_AFTER_OFF   0
#endif
#if M1_IDENT_IQ_STEP_ENABLE
#if (M1_IDENT_STEP_OFF_ROUNDS > M1_IDENT_STEP_ROUNDS)
#error "M1_IDENT_STEP_OFF_ROUNDS must be <= M1_IDENT_STEP_ROUNDS"
#endif
#if (M1_IDENT_STEP_OFF_ROUNDS + M1_IDENT_STEP_FIXED_ROUNDS > M1_IDENT_STEP_ROUNDS)
#error "M1_IDENT_STEP_OFF_ROUNDS + M1_IDENT_STEP_FIXED_ROUNDS must be <= M1_IDENT_STEP_ROUNDS"
#endif
#if ((M1_IDENT_STEP_ROUNDS - M1_IDENT_STEP_OFF_ROUNDS - M1_IDENT_STEP_FIXED_ROUNDS) > 0u) && \
    !M1_DEADBAND_LUT_BAKED_ENABLE && !M1_IDENT_ID_CAL_BEFORE_STEP
#error "LUT segment requires M1_DEADBAND_LUT_BAKED_ENABLE or M1_IDENT_ID_CAL_BEFORE_STEP"
#endif
#endif /* M1_IDENT_IQ_STEP_ENABLE */
#ifndef M1_IDENT_STEP_I0_A
#define M1_IDENT_STEP_I0_A            0.0f
#endif
#ifndef M1_IDENT_STEP_I1_A
#define M1_IDENT_STEP_I1_A            0.3f   /* 小阶。*/
#endif
#ifndef M1_IDENT_STEP_I2_A
#define M1_IDENT_STEP_I2_A            0.5f   /* 中阶。*/
#endif
#ifndef M1_IDENT_STEP_I3_A
#define M1_IDENT_STEP_I3_A            1.0f   /* 低段大阶。*/
#endif
#ifndef M1_IDENT_STEP_I_BASE_HI_A
#define M1_IDENT_STEP_I_BASE_HI_A     1.0f   /* 高段 1 A 基线 */
#endif
#ifndef M1_IDENT_STEP_I5_A
#define M1_IDENT_STEP_I5_A            1.3f
#endif
#ifndef M1_IDENT_STEP_I6_A
#define M1_IDENT_STEP_I6_A            1.5f
#endif
#ifndef M1_IDENT_STEP_I7_A
#define M1_IDENT_STEP_I7_A            2.0f
#endif
#ifndef M1_IDENT_STEP_BANDS
#define M1_IDENT_STEP_BANDS           1u
#endif
#ifndef M1_IDENT_STEP_ROUNDS_PER_PROFILE
#define M1_IDENT_STEP_ROUNDS_PER_PROFILE  M1_IDENT_STEP_OFF_ROUNDS
#endif
#if (M1_IDENT_STEP_BANDS > 1u) && \
    (M1_IDENT_STEP_ROUNDS != (M1_IDENT_STEP_BANDS * 2u * M1_IDENT_STEP_ROUNDS_PER_PROFILE))
#error "M1_IDENT_STEP_ROUNDS must equal M1_IDENT_STEP_BANDS * 2 * M1_IDENT_STEP_ROUNDS_PER_PROFILE"
#endif
/** 各拍 dwell (s)；非零档与回零档相同 */
#ifndef M1_IDENT_STEP_DWELL_S
#define M1_IDENT_STEP_DWELL_S         0.5f
#endif
#ifndef M1_IDENT_STEP_ZERO_DWELL_S
#define M1_IDENT_STEP_ZERO_DWELL_S    0.5f
#endif

/** 1=阶跃辨识绕过 M1_CLOSURE_BRINGUP 。I_ref 钳位。6 V PI 限幅 */
#ifndef M1_IDENT_OVERRIDE_LIMITS
#define M1_IDENT_OVERRIDE_LIMITS      1
#endif
#if M1_IDENT_OVERRIDE_LIMITS
#define M1_IDENT_PI_V_LIMIT_V         M1_PI_V_MAX
#else
#define M1_IDENT_PI_V_LIMIT_V         M1_PI_V_LIMIT_V
#endif
#define M1_IDENT_PI_V_LIMIT_MIN       (-M1_IDENT_PI_V_LIMIT_V)
#define M1_IDENT_PI_INT_LIMIT_V       M1_IDENT_PI_V_LIMIT_V
#define M1_IDENT_PI_INT_LIMIT_MIN     (-M1_IDENT_PI_V_LIMIT_V)

/** Bode 轴：0=Iq sin（Id=0）；1=Id sin（Iq=0，。锁轴。*/
#ifndef M1_IDENT_BODE_AXIS_ID
#define M1_IDENT_BODE_AXIS_ID         0
#endif

/** Bode：i_ref = bias + amp*sin(2πft)，几何扫。f *= RATIO */
#ifndef M1_IDENT_BODE_I_BIAS_A
#define M1_IDENT_BODE_I_BIAS_A        0.25f
#endif
#ifndef M1_IDENT_BODE_I_AMP_A
#define M1_IDENT_BODE_I_AMP_A         0.05f
#endif
#ifndef M1_IDENT_BODE_I_BIAS_HI_A
#define M1_IDENT_BODE_I_BIAS_HI_A     1.25f   /* 。I Bode；无 1.0 A 。*/
#endif
#ifndef M1_IDENT_BODE_I_AMP_HI_A
#define M1_IDENT_BODE_I_AMP_HI_A      0.10f
#endif
#ifndef M1_IDENT_BODE_LUT_ENABLE
#define M1_IDENT_BODE_LUT_ENABLE      1
#endif
#ifndef M1_IDENT_BODE_BANDS
#define M1_IDENT_BODE_BANDS           1u
#endif
#if M1_IDENT_BODE_LUT_ENABLE
#if (M1_IDENT_BODE_BANDS > 1u) && (M1_IDENT_BODE_ROUNDS != (M1_IDENT_BODE_BANDS * 2u))
#error "M1_IDENT_BODE_ROUNDS must equal M1_IDENT_BODE_BANDS * 2"
#endif
#else
#if (M1_IDENT_BODE_BANDS > 1u) && (M1_IDENT_BODE_ROUNDS != M1_IDENT_BODE_BANDS)
#error "OFF-only Bode: M1_IDENT_BODE_ROUNDS must equal M1_IDENT_BODE_BANDS"
#endif
#endif
#ifndef M1_IDENT_BODE_F0_HZ
#define M1_IDENT_BODE_F0_HZ           10.0f
#endif
#ifndef M1_IDENT_BODE_F1_HZ
/** legacy 兜底：仅文档/脚本参考；运行时频表以 ident_bode_freq_table.h 为准（F1=800 。38 。846 Hz。*/
#define M1_IDENT_BODE_F1_HZ           800.0f
#endif
#ifndef M1_IDENT_BODE_F_RATIO
#define M1_IDENT_BODE_F_RATIO         1.15f
#endif
#ifndef M1_IDENT_BODE_F_SPLIT_HZ
#define M1_IDENT_BODE_F_SPLIT_HZ      500.0f
#endif
#ifndef M1_IDENT_BODE_F_RATIO_HI
#define M1_IDENT_BODE_F_RATIO_HI      1.06f
#endif
#ifndef M1_IDENT_BODE_CYCLES_PER_FREQ
#define M1_IDENT_BODE_CYCLES_PER_FREQ 20.0f
#endif
/** f >= F_SPLIT 固定观测窗：1=Ncyc=T_obs*f。=。CYCLES_HI 常数 */
#ifndef M1_IDENT_BODE_USE_T_OBS_HI
#define M1_IDENT_BODE_USE_T_OBS_HI    1
#endif
#ifndef M1_IDENT_BODE_T_OBS_HI_S
#define M1_IDENT_BODE_T_OBS_HI_S      0.1f   /* 100 ms；@1 kHz 。100 cycles */
#endif
#ifndef M1_IDENT_BODE_CYCLES_HI
#define M1_IDENT_BODE_CYCLES_HI       50.0f  /* 。USE_T_OBS_HI=0 时生。*/
#endif
/** Bode 分段。×OFF + 1×FIXED + 1×LUT，各。F0→F1 一整遍 */
#ifndef M1_IDENT_BODE_ROUNDS
#define M1_IDENT_BODE_ROUNDS          3u
#endif
#ifndef M1_IDENT_BODE_OFF_ROUNDS
#define M1_IDENT_BODE_OFF_ROUNDS      1u
#endif
#ifndef M1_IDENT_BODE_FIXED_ROUNDS
#define M1_IDENT_BODE_FIXED_ROUNDS    1u
#endif
#if M1_IDENT_IQ_BODE_ENABLE && M1_IDENT_BODE_LUT_ENABLE
#if (M1_IDENT_BODE_OFF_ROUNDS + M1_IDENT_BODE_FIXED_ROUNDS > M1_IDENT_BODE_ROUNDS)
#error "M1_IDENT_BODE_OFF_ROUNDS + M1_IDENT_BODE_FIXED_ROUNDS must be <= M1_IDENT_BODE_ROUNDS"
#endif
#if ((M1_IDENT_BODE_ROUNDS - M1_IDENT_BODE_OFF_ROUNDS - M1_IDENT_BODE_FIXED_ROUNDS) > 0u) && \
    !M1_DEADBAND_LUT_BAKED_ENABLE && !M1_IDENT_ID_CAL_BEFORE_STEP
#error "Bode LUT segment requires M1_IDENT_ID_CAL_BEFORE_STEP or M1_DEADBAND_LUT_BAKED_ENABLE"
#endif
#endif /* M1_IDENT_IQ_BODE_ENABLE && M1_IDENT_BODE_LUT_ENABLE */
#endif /* M1_IDENT_ENABLE */

/* --- 速度环辨识（旋转、deadband OFF；与 Id/Iq ident 互斥。--- */
#ifndef M1_SPEED_IDENT_ENABLE
#define M1_SPEED_IDENT_ENABLE           0
#endif

#if M1_SPEED_IDENT_ENABLE
#if M1_IDENT_ENABLE
#error "M1_SPEED_IDENT_ENABLE and M1_IDENT_ENABLE are mutually exclusive"
#endif
#if M1_ID_LOCK_CAL_SWEEP
#error "M1_SPEED_IDENT_ENABLE requires M1_ID_LOCK_CAL_SWEEP=0"
#endif
#if !M1_SPEED_LOOP_ENABLE
#error "M1_SPEED_IDENT_ENABLE requires M1_SPEED_LOOP_ENABLE=1"
#endif

#ifndef M1_SPEED_IDENT_STEP_ENABLE
#define M1_SPEED_IDENT_STEP_ENABLE      1
#endif
#ifndef M1_SPEED_IDENT_STEP_LADDER_ENABLE
#define M1_SPEED_IDENT_STEP_LADDER_ENABLE  0
#endif
#ifndef M1_SPEED_IDENT_OBS_FLOOR_PROBE
#define M1_SPEED_IDENT_OBS_FLOOR_PROBE    0 /* 1=1000。00 。0rpm 。OBS 下限 */
#endif
#ifndef M1_SPEED_IDENT_BODE_ENABLE
#define M1_SPEED_IDENT_BODE_ENABLE      1
#endif
#if !M1_SPEED_IDENT_STEP_ENABLE && !M1_SPEED_IDENT_BODE_ENABLE
#error "M1_SPEED_IDENT_ENABLE requires STEP and/or BODE"
#endif

#ifndef M1_SPEED_IDENT_HOLD_S
#define M1_SPEED_IDENT_HOLD_S           8.0f
#endif
#ifndef M1_SPEED_IDENT_RPM_START
#define M1_SPEED_IDENT_RPM_START        100.0f
#endif
#ifndef M1_SPEED_IDENT_STEP_ROUNDS
#define M1_SPEED_IDENT_STEP_ROUNDS      1u
#endif
#ifndef M1_SPEED_IDENT_STEP_RPM0
#define M1_SPEED_IDENT_STEP_RPM0        500.0f
#endif
#ifndef M1_SPEED_IDENT_STEP_RPM1
#define M1_SPEED_IDENT_STEP_RPM1        300.0f
#endif
#ifndef M1_SPEED_IDENT_STEP_RPM2
#define M1_SPEED_IDENT_STEP_RPM2        700.0f
#endif
#ifndef M1_SPEED_IDENT_STEP_RPM3
#define M1_SPEED_IDENT_STEP_RPM3        900.0f
#endif
#ifndef M1_SPEED_IDENT_STEP_RPM4
#define M1_SPEED_IDENT_STEP_RPM4        800.0f
#endif
#ifndef M1_SPEED_IDENT_STEP_RPM5
#define M1_SPEED_IDENT_STEP_RPM5        900.0f
#endif
#ifndef M1_SPEED_IDENT_STEP_RPM6
#define M1_SPEED_IDENT_STEP_RPM6        1000.0f
#endif
#ifndef M1_SPEED_IDENT_STEP_DWELL_S
#define M1_SPEED_IDENT_STEP_DWELL_S     8.0f
#endif
#ifndef M1_SPEED_IDENT_STEP_ZERO_DWELL_S
#define M1_SPEED_IDENT_STEP_ZERO_DWELL_S  3.0f
#endif
#ifndef M1_SPEED_IDENT_BODE_RPM_BIAS
#define M1_SPEED_IDENT_BODE_RPM_BIAS    500.0f
#endif
#ifndef M1_SPEED_IDENT_BODE_RPM_AMP
#define M1_SPEED_IDENT_BODE_RPM_AMP     30.0f
#endif
#ifndef M1_SPEED_IDENT_BODE_CYCLES_PER_FREQ
#define M1_SPEED_IDENT_BODE_CYCLES_PER_FREQ  10.0f
#endif
#endif /* M1_SPEED_IDENT_ENABLE */

/**
 * 全部 profile 之后。
 * I/F 帧恒。+|Iq| 拖动（正/。ω 皆然。121：反。I/F 。+4）。
 * 速度环交。bumpless 。Continuity 同号，勿。DIR（乘 DIR 会在 BLEND 。+4→−3.5 反扭矩砸速）。
 * DIR 只作用在 ω 目标；转子系若需 −Iq 保负速，。PI 在限权内自行斜过去。
 */
#undef M1_IF_OBS_SPEED_IQ_SIGN
#define M1_IF_OBS_SPEED_IQ_SIGN         (1.0f)
#undef M1_IF_OBS_SPEED_IQ_BOOT_A
#define M1_IF_OBS_SPEED_IQ_BOOT_A       (M1_IF_IQ_A)
#undef M1_IF_OBS_HANDOFF_IQ_BOOT_A
#define M1_IF_OBS_HANDOFF_IQ_BOOT_A     (M1_IF_HANDOFF_IQ_A)

#endif
