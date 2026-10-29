/**
 * @file defaults_obs.h
 * @date 2026-10-29
 * @brief 观测器门控默认宏（HFI / EMF-SMO / PLL / 软切换）。普通 #define。
 * @note 须在 bringup_modes / profile 之前纳入；覆盖方用 #undef+#define。
 * @note 模块是否编进工程见 build_config.h 的 BUILD_HFI / BUILD_SMO；
 *       本头只给运行配方默认。交叉断言放在 bringup_modes 之后。
 */
#ifndef CONFIG_DEFAULTS_OBS_H
#define CONFIG_DEFAULTS_OBS_H

/** 1=编进 HFI 与 Composite。默认 0；profile / bm 可覆盖。关掉时 Composite 走空桩。 */
#define M1_HFI_ENABLE                   0
/** 1=无感运动旁路：速度环用观测转速。 */
#define M1_HFI_MOTION_BYPASS_ENABLE     0
/** 1=速度反馈走观测 PLL，不吃编码器。须与运动旁路一起开。 */
#define M1_HFI_SPEED_FB_ENABLE          0
#define M1_EMF_VEQ_ENABLE               0
#define M1_EMF_SMO_ENABLE               0
#define M1_EMF_PLL_ENABLE               0
#define M1_EMF_SMO_LPF_LINEAR_ENABLE    0
#define M1_EMF_LPF_PHASE_FF_ENABLE      0 /* 1=θ̂ 模型前馈 */
#define M1_OBS_SOFT_SWITCH_ENABLE       0
#define M1_OBS_SS_SPEED_SWITCH_ENABLE   0
#if M1_EMF_PLL_ENABLE && !(M1_EMF_VEQ_ENABLE || M1_EMF_SMO_ENABLE)
#error "M1_EMF_PLL_ENABLE requires M1_EMF_VEQ_ENABLE and/or M1_EMF_SMO_ENABLE (PLL feeds on eαβ)"
#endif
#if M1_OBS_SOFT_SWITCH_ENABLE && !M1_EMF_PLL_ENABLE
#error "M1_OBS_SOFT_SWITCH_ENABLE requires M1_EMF_PLL_ENABLE"
#endif
/** 1=PLL 吃 SMO e；0=吃 Veq e。未覆盖时：开了 SMO 则优先生效 SMO。 */
#if M1_EMF_SMO_ENABLE
#define M1_EMF_PLL_USE_SMO              1
#else
#define M1_EMF_PLL_USE_SMO              0
#endif
#if M1_EMF_PLL_USE_SMO && !M1_EMF_SMO_ENABLE
#error "M1_EMF_PLL_USE_SMO=1 requires M1_EMF_SMO_ENABLE"
#endif
#if M1_EMF_PLL_ENABLE && !M1_EMF_PLL_USE_SMO && !M1_EMF_VEQ_ENABLE
#error "PLL without USE_SMO requires M1_EMF_VEQ_ENABLE"
#endif

#endif /* CONFIG_DEFAULTS_OBS_H */
