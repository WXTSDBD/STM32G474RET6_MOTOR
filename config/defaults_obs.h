/**
 * @file defaults_obs.h
 * @date 2026-10-29
 * @brief 观测器门控与数值默认。普通 #define；recipe 用 #undef+#define 覆盖。
 * @note 须在 recipe 之前纳入。模块是否编进见 build_config.h。
 */
#ifndef CONFIG_DEFAULTS_OBS_H
#define CONFIG_DEFAULTS_OBS_H

#define M1_HFI_ENABLE                   0
#define M1_HFI_MOTION_BYPASS_ENABLE     0
#define M1_HFI_SPEED_FB_ENABLE          0
#define M1_EMF_VEQ_ENABLE               0
#define M1_EMF_SMO_ENABLE               0
#define M1_EMF_PLL_ENABLE               0
#define M1_EMF_SMO_LPF_LINEAR_ENABLE    0
#define M1_EMF_LPF_PHASE_FF_ENABLE      0
#define M1_OBS_SOFT_SWITCH_ENABLE       0
#define M1_OBS_SS_SPEED_SWITCH_ENABLE   0
#define M1_EMF_PLL_USE_SMO              0

/** 发布门槛（机械 rpm）与退回资格拍数。 */
#define M1_HFI_PUB_UP_RPM               1300.0f
#define M1_HFI_PUB_DOWN_RPM             1000.0f
#define M1_HFI_PUB_INJ_OFF_RPM          1500.0f
#define M1_HFI_PUB_INJ_ON_RPM           1400.0f
#define M1_HFI_PUB_DN_QUAL_N            400u
/** 4 个真闸门：recipe 可覆盖。 */
#define M1_HFI_PUB_FREEZE_ENABLE        1
#define M1_HFI_PUB_FREEZE_RPM_MIN       200.0f
#define M1_HFI_PUB_FREEZE_RPM_MAX       900.0f
#define M1_HFI_PUB_FREEZE_DRPM          15.0f
#define M1_HFI_PUB_FREEZE_N             400u

#define M1_HFI_QKICK_HAT0_N             160u
#define M1_HFI_QKICK_BRAKE_N            400u
#define M1_HFI_QKICK_POL_W_MIN_RAD_S    5.0f

#define M1_EMF_SMO_R_OHM                M1_RS_OHM
#define M1_EMF_SMO_L_H                  M1_LD_H
#define M1_EMF_SMO_K                    20.0f
#define M1_EMF_SMO_SAT_A                0.30f
#define M1_EMF_SMO_LPF_HZ               200.0f
#define M1_EMF_SMO_LPF_ENABLE           1
#define M1_EMF_SMO_ATAN_ENABLE          0
#define M1_EMF_EMAG_FAST_ENABLE         0
#define M1_EMF_SMO_THETA_OFF_RAD        (-0.4054f)
#define M1_EMF_SMO_LPF_LINEAR_K         1.5f
#define M1_EMF_SMO_LPF_LINEAR_FC_MIN    100.0f
#define M1_EMF_SMO_LPF_LINEAR_FC_MAX    280.0f

#define M1_EMF_PLL_FN_HZ                100.0f
#define M1_EMF_PLL_ZETA                 0.707106781f
#define M1_EMF_PLL_THETA_OFF_RAD        (-0.4054f)
#define M1_EMF_PLL_NORM_ENABLE          1
#define M1_EMF_PLL_NORM_EPS             0.05f
#define M1_EMF_PLL_OMEGA_LIMIT_RAD_S    2000.0f
#define M1_EMF_PLL_INTEGRATOR_LIMIT_RAD_S M1_EMF_PLL_OMEGA_LIMIT_RAD_S

#define M1_EMF_VEQ_R_OHM                M1_RS_OHM
#define M1_EMF_VEQ_L_H                  M1_LD_H
#define M1_EMF_VEQ_LPF_HZ               200.0f
#define M1_EMF_VEQ_LPF_ENABLE           1
#define M1_EMF_VEQ_THETA_OFF_RAD        (-0.4054f)
#define M1_EMF_VEQ_PSI_WMIN             20.0f

#endif /* CONFIG_DEFAULTS_OBS_H */
