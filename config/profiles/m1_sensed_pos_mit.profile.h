/**
 * @file m1_sensed_pos_mit.profile.h
 * @date 2026-10-06
 * @brief 位置环 / MIT：PACK_MK 刚度 A/B + 并行力矩源 MIT。
 *
 * 打开 M1_USE_SENSED_POS_MIT_PROFILE。与 HFI 141 互斥。
 * PACK_MK：Kp_scale=1/2/3/4/6 × iq_ff；×6 用高档阻尼抑 gap 振。
 * M1_MIT_TORQUE_LAW=1：iq*=Kp·Δθ+Kd·Δω+iq_ff；回退嵌套则改 0。
 */
#ifndef CONFIG_PROFILES_M1_SENSED_POS_MIT_PROFILE_H
#define CONFIG_PROFILES_M1_SENSED_POS_MIT_PROFILE_H

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

#undef M1_SPEED_LOOP_ENABLE
#define M1_SPEED_LOOP_ENABLE            1
#undef M1_SPEED_LOOP_BOOT
#define M1_SPEED_LOOP_BOOT              1
#undef M1_SPEED_REF_RPM_DEFAULT
/** 解耦前馈 A/B：段表给 0/500；默认 0。 */
#define M1_SPEED_REF_RPM_DEFAULT        0.0f
#undef M1_SPEED_OMEGA_RAMP_ENABLE
/** 500 rpm/s：0→500 约 1 s，落在 3 s 加速窗内便于看 Id。 */
#define M1_SPEED_OMEGA_RAMP_ENABLE      1
#undef M1_SPEED_OMEGA_RAMP_RPM_S
#define M1_SPEED_OMEGA_RAMP_RPM_S       500.0f
#undef M1_SPEED_PI_BETA
#define M1_SPEED_PI_BETA                1.0f
/** 嵌套支撑 ~10 Hz 位置：速度环略抬（原 0.015 偏软）。 */
#undef M1_SPEED_PI_KP
#define M1_SPEED_PI_KP                  0.030f
#undef M1_SPEED_PI_KI
#define M1_SPEED_PI_KI                  0.004f
#undef M1_I_REF_ABS_MAX
#define M1_I_REF_ABS_MAX                5.0f
#undef M1_SPEED_IQ_REF_ABS_MAX
#define M1_SPEED_IQ_REF_ABS_MAX         5.0f
#undef M1_SPEED_PI_OUT_MAX
#define M1_SPEED_PI_OUT_MAX             5.0f
#undef M1_SPEED_PI_OUT_MIN
#define M1_SPEED_PI_OUT_MIN             (-5.0f)
#undef M1_SPEED_PI_INT_MAX
#define M1_SPEED_PI_INT_MAX             5.0f
#undef M1_SPEED_PI_INT_MIN
#define M1_SPEED_PI_INT_MIN             (-5.0f)

#undef M1_OPEN_UD_PRE_ID_CAL_ENABLE
#define M1_OPEN_UD_PRE_ID_CAL_ENABLE    0
#undef M1_OPEN_UQ_PRE_ID_CAL_ENABLE
#define M1_OPEN_UQ_PRE_ID_CAL_ENABLE    0

#define M1_DEADBAND_ENABLE              0
#define M1_DEADBAND_LUT_BAKED_ENABLE    0
#define M1_DEADBAND_NVM_ON_BOOT         0

#undef M1_STARTUP_ENABLE
#define M1_STARTUP_ENABLE               0
#undef M1_IF_ENABLE
#define M1_IF_ENABLE                    0
#undef M1_IF_TO_OBS_ENABLE
#define M1_IF_TO_OBS_ENABLE             0

#undef M1_ENC_OPTIONAL_ENABLE
#define M1_ENC_OPTIONAL_ENABLE          0

#undef M1_HFI_ENABLE
#define M1_HFI_ENABLE                   0
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              0
#undef M1_HFI_MOTION_BYPASS_ENABLE
#define M1_HFI_MOTION_BYPASS_ENABLE     0
#undef M1_HFI_SPEED_FB_ENABLE
#define M1_HFI_SPEED_FB_ENABLE          0

#undef M1_EMF_VEQ_ENABLE
#define M1_EMF_VEQ_ENABLE               0
#undef M1_EMF_SMO_ENABLE
#define M1_EMF_SMO_ENABLE               0
#undef M1_EMF_PLL_ENABLE
#define M1_EMF_PLL_ENABLE               0
#undef M1_OBS_SOFT_SWITCH_ENABLE
#define M1_OBS_SOFT_SWITCH_ENABLE       0
#undef M1_OBS_SPD_PLL_ENABLE
#define M1_OBS_SPD_PLL_ENABLE           0

#undef M1_PLL_ENABLE
#define M1_PLL_ENABLE                   1

#undef M1_POS_LOOP_ENABLE
#define M1_POS_LOOP_ENABLE              1
#undef M1_POS_LOOP_BOOT
#define M1_POS_LOOP_BOOT                0
#undef M1_POS_STEP_TEST_ENABLE
#define M1_POS_STEP_TEST_ENABLE         0
#undef M1_POS_MIT_COMBO_ENABLE
#define M1_POS_MIT_COMBO_ENABLE         0
#undef M1_POS_ERR_HYST_ENABLE
#define M1_POS_ERR_HYST_ENABLE          0
#undef M1_POS_DECIM
#define M1_POS_DECIM                    1u
/** Kp≈60×f_pos；f_pos=10 Hz → 600 rpm/rad。 */
#undef M1_POS_KP_RPM_PER_RAD
#define M1_POS_KP_RPM_PER_RAD           600.0f
#undef M1_POS_OMEGA_MAX_RPM
#define M1_POS_OMEGA_MAX_RPM            400.0f
#undef M1_SPEED_REVERSAL_TEST_ENABLE
#define M1_SPEED_REVERSAL_TEST_ENABLE   0

#undef M1_OUTER_NEST_ENABLE
#define M1_OUTER_NEST_ENABLE            1
#undef M1_OUTER_EXPT
#define M1_OUTER_EXPT                   M1_OUTER_EXPT_SIGNOFF
#undef M1_MIT_KP_SCALE
/** 默认档；PACK_MK 运行时覆盖为 2/4/6。 */
#define M1_MIT_KP_SCALE                 4.0f
#undef M1_MIT_KD_RPM
#define M1_MIT_KD_RPM                   0.0f
#undef M1_MIT_KI_DISABLE
#define M1_MIT_KI_DISABLE               1
/** 1=并行力矩源 MIT；0=回退嵌套（位置 P→速度 PI）。 */
#undef M1_MIT_TORQUE_LAW
#define M1_MIT_TORQUE_LAW               1
#undef M1_MIT_TORQUE_LAW_20KHZ
#define M1_MIT_TORQUE_LAW_20KHZ         1
/** 电流域刚度基值；×scale 后约对齐旧嵌套实测（scale2≈44 A/rad）。 */
#undef M1_MIT_KP_A_PER_RAD
#define M1_MIT_KP_A_PER_RAD             22.0f
#undef M1_MIT_KD_A_S_PER_RAD
#define M1_MIT_KD_A_S_PER_RAD           0.0f
/** ×6 专用阻尼（A·s/rad）；由 0052 反推 ζ≈0.35。 */
#undef M1_MIT_KD_HIGH_A_S_PER_RAD
#define M1_MIT_KD_HIGH_A_S_PER_RAD      0.4f
#undef M1_MIT_KD_HIGH_SCALE_MIN
#define M1_MIT_KD_HIGH_SCALE_MIN        6.0f
#undef M1_OUTER_THETA_FB_SRC
#define M1_OUTER_THETA_FB_SRC           0
#undef M1_FRIC_FF_ENABLE
#define M1_FRIC_FF_ENABLE               0
#undef M1_FRIC_FF_A
#define M1_FRIC_FF_A                    0.35f
#undef M1_FRIC_FF_SAT_RAD
#define M1_FRIC_FF_SAT_RAD              (2.0f * 0.01745329252f)
#undef M1_OUTER_SIGNOFF_PACK1
#define M1_OUTER_SIGNOFF_PACK1          0
#undef M1_OUTER_SIGNOFF_PACK2
#define M1_OUTER_SIGNOFF_PACK2          0
#undef M1_OUTER_SIGNOFF_PACK_MK
#define M1_OUTER_SIGNOFF_PACK_MK        1
#undef M1_OUTER_SIGNOFF_PACK_BW
#define M1_OUTER_SIGNOFF_PACK_BW        0
#undef M1_OUTER_MK_HOLD_S
#define M1_OUTER_MK_HOLD_S              5.0f
#undef M1_OUTER_MK_GAP_S
#define M1_OUTER_MK_GAP_S               2.5f
#undef M1_EXP_FRAMEWORK_ENABLE
#define M1_EXP_FRAMEWORK_ENABLE         1
#undef M1_EXP_FF_AB_ENABLE
#define M1_EXP_FF_AB_ENABLE             0
#undef M1_FOC_ROTATION_FF_ENABLE
#define M1_FOC_ROTATION_FF_ENABLE       0

#undef M1_VOFA_UNIFIED_12CH
#define M1_VOFA_UNIFIED_12CH            1
#undef M1_TELEM_BRINGUP_K
#define M1_TELEM_BRINGUP_K              12u
/** 20 kHz / 4 = 5 kHz：刚度保持 + mark/iq_ref。 */
#undef M1_TELEM_BRINGUP_DECIMATION
#define M1_TELEM_BRINGUP_DECIMATION     4u
#undef M1_VOFA_PLL_CH8_11
#define M1_VOFA_PLL_CH8_11              0
#undef M1_VOFA_SPEED_CH8_11
#define M1_VOFA_SPEED_CH8_11            0
#undef M1_VOFA_SPEED_ISR_CH
#define M1_VOFA_SPEED_ISR_CH            0
#undef M1_VOFA_MIT_CH8_11
#define M1_VOFA_MIT_CH8_11              1
#undef M1_VOFA_SIGNOFF_CH
#define M1_VOFA_SIGNOFF_CH              1
/** 0=ch10=iq_ref ch11=mark（刚度分析要用）；1=enc_raw 会丢掉 mark。 */
#undef M1_VOFA_SIGNOFF_ENC_RAW
#define M1_VOFA_SIGNOFF_ENC_RAW         0
#undef M1_VOFA_KTH_SPEED_DIAG
#define M1_VOFA_KTH_SPEED_DIAG          0
#undef M1_CLOSURE_BRINGUP
#define M1_CLOSURE_BRINGUP              0

#endif /* CONFIG_PROFILES_M1_SENSED_POS_MIT_PROFILE_H */
