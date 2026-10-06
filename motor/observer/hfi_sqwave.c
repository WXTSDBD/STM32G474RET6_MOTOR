/**
 * @file hfi_sqwave.c
 * @brief HFI 旁路 / δ 扫描 / IPD 扫位（Ud 摆位+多轮脉冲，供离线分析。
 */
#include "hfi_sqwave.h"
#include "observer/obs_cfg.h"
#include "motor_math.h"

#include <math.h>
#include <stddef.h>

#ifndef M1_HFI_ENABLE
#define M1_HFI_ENABLE 0
#endif


#ifndef M1_HFI_MOTION_BYPASS_ENABLE
#define M1_HFI_MOTION_BYPASS_ENABLE     0
#endif
#ifndef M1_ENC_OPTIONAL_ENABLE
#define M1_ENC_OPTIONAL_ENABLE          0
#endif
#ifndef M1_HFI_VH_V
#define M1_HFI_VH_V                     2.0f
#endif
#ifndef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             0.5f
#endif
#ifndef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 150.0f
#endif
#ifndef M1_HFI_GATE
#define M1_HFI_GATE                     0
#endif
#ifndef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               6.0f
#endif
#ifndef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 300.0f
#endif
#ifndef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               6.0f
#endif
#ifndef M1_HFI_RUN_RPM3
#define M1_HFI_RUN_RPM3                 0.0f
#endif
#ifndef M1_HFI_RUN_RPM3_S
#define M1_HFI_RUN_RPM3_S               0.0f
#endif
#ifndef M1_HFI_RUN_RPM4
#define M1_HFI_RUN_RPM4                 0.0f
#endif
#ifndef M1_HFI_RUN_RPM4_S
#define M1_HFI_RUN_RPM4_S               0.0f
#endif
#ifndef M1_HFI_RUN_RPM5
#define M1_HFI_RUN_RPM5                 0.0f
#endif
#ifndef M1_HFI_RUN_RPM5_S
#define M1_HFI_RUN_RPM5_S               0.0f
#endif
#ifndef M1_HFI_RUN_LADDER_ENABLE
#define M1_HFI_RUN_LADDER_ENABLE        0
#endif
#ifndef M1_HFI_RUN_RPM_START
#define M1_HFI_RUN_RPM_START            100.0f
#endif
#ifndef M1_HFI_RUN_RPM_STEP
#define M1_HFI_RUN_RPM_STEP             100.0f
#endif
#ifndef M1_HFI_RUN_RPM_MAX
#define M1_HFI_RUN_RPM_MAX              800.0f
#endif
#ifndef M1_HFI_RUN_STEP_S
#define M1_HFI_RUN_STEP_S               5.0f
#endif
/* 有感标定：Park=enc、速环。enc，HFI 只影子；多档转速供离线扫前。PLL */
#ifndef M1_HFI_A0_MOVE_UD_V
#define M1_HFI_A0_MOVE_UD_V             2.0f
#endif
#ifndef M1_HFI_A0_MOVE_S
#define M1_HFI_A0_MOVE_S                1.5f
#endif
#ifndef M1_HFI_A0_SETTLE_S
#define M1_HFI_A0_SETTLE_S              1.0f
#endif
#ifndef M1_HFI_A0_POS_N
#define M1_HFI_A0_POS_N                 3u
#endif
#ifndef M1_HFI_A0_STEP_DEG
#define M1_HFI_A0_STEP_DEG              60.0f
#endif
#ifndef M1_HFI_PLL_ENABLE
#define M1_HFI_PLL_ENABLE               1
#endif
#ifndef M1_HFI_PLL_KP
#define M1_HFI_PLL_KP                   80.0f
#endif
#ifndef M1_HFI_PLL_KI
#define M1_HFI_PLL_KI                   800.0f
#endif
#ifndef M1_HFI_PLL_W_MAX
#define M1_HFI_PLL_W_MAX                80.0f
#endif
#ifndef M1_HFI_PLL_INIT_OFF_RAD
#define M1_HFI_PLL_INIT_OFF_RAD         0.0f
#endif
#ifndef M1_HFI_DEMOD_PROBE_ENABLE
#define M1_HFI_DEMOD_PROBE_ENABLE       0
#endif
#ifndef M1_HFI_DEMOD_FREEZE_T0_S
#define M1_HFI_DEMOD_FREEZE_T0_S        1.20f
#endif
#ifndef M1_HFI_DEMOD_FREEZE_T1_S
#define M1_HFI_DEMOD_FREEZE_T1_S        1.60f
#endif
#ifndef M1_HFI_DEMOD_FREEZE_T2_S
#define M1_HFI_DEMOD_FREEZE_T2_S        6.00f
#endif
#ifndef M1_HFI_DEMOD_FREEZE_T3_S
#define M1_HFI_DEMOD_FREEZE_T3_S        6.40f
#endif
#ifndef M1_HFI_POLARITY_HOLD_N
#define M1_HFI_POLARITY_HOLD_N          400u
#endif
#ifndef M1_HFI_EPS_SIGN
#define M1_HFI_EPS_SIGN                 1.0f
#endif
#ifndef M1_HFI_ATAN2_ENABLE
#define M1_HFI_ATAN2_ENABLE             0
#endif
#ifndef M1_HFI_DEMOD_HP_ENABLE
#define M1_HFI_DEMOD_HP_ENABLE          0
#endif
#ifndef M1_HFI_DEMOD_HP_A
#define M1_HFI_DEMOD_HP_A               (0.05f)
#endif
#ifndef M1_HFI_DEMOD_AB_ENABLE
#define M1_HFI_DEMOD_AB_ENABLE          0
#endif
#ifndef M1_HFI_DEMOD_AB_MID_ENABLE
#define M1_HFI_DEMOD_AB_MID_ENABLE      0
#endif
#if M1_HFI_DEMOD_AB_ENABLE && M1_HFI_DEMOD_AB_MID_ENABLE
#error "M1_HFI_DEMOD_AB_ENABLE and M1_HFI_DEMOD_AB_MID_ENABLE are exclusive"
#endif
#ifndef M1_HFI_INJECT_AB_ENABLE
#define M1_HFI_INJECT_AB_ENABLE         0
#endif
#ifndef M1_HFI_INJECT_POST_LOOP
#define M1_HFI_INJECT_POST_LOOP         0 /* 1：Vh 沿 θ̂ 在 αβ 叠，不进 Ud_pi */
#endif
#ifndef M1_HFI_DEMOD_INJ_AXIS
#define M1_HFI_DEMOD_INJ_AXIS           0 /* 1：注入轴 iαβ 半周差分，不吃 Park Id */
#endif
#ifndef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                1.0f
#endif
#ifndef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                1.0f
#endif
#ifndef M1_HFI_A_CMD
#define M1_HFI_A_CMD                    0.0f
#endif
#ifndef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              0
#endif
#ifndef OBS_CTRL_TS_S
#define OBS_CTRL_TS_S                    50e-6f
#endif
#ifndef M1_HFI_FH_HZ
#define M1_HFI_FH_HZ                    10000.0f
#endif
#ifndef M1_HFI_OMEGA_FF_SRC
#define M1_HFI_OMEGA_FF_SRC             1
#endif
/*
 * 正交选轴。247 标定）：。d 。eps_d<0，假 q 。eps_d>0。
 * |eps| 小且 eps_d>+D_TH 。θ̂+=π/2；eps_d<-D_TH 确认窗满 。axis_ok。
 * 未确认前。Iq（见 AXIS_SEL_IQ_MAX）。
 */
#ifndef M1_HFI_AXIS_SEL_ENABLE
#define M1_HFI_AXIS_SEL_ENABLE          0
#endif
#ifndef M1_HFI_AXIS_SEL_D_SIGN
#define M1_HFI_AXIS_SEL_D_SIGN          1.0f
#endif
#ifndef M1_HFI_AXIS_SEL_D_TH
#define M1_HFI_AXIS_SEL_D_TH            0.050f
#endif
#ifndef M1_HFI_AXIS_SEL_EPS_MAX
#define M1_HFI_AXIS_SEL_EPS_MAX         0.020f
#endif
#ifndef M1_HFI_AXIS_SEL_LOCK_N
#define M1_HFI_AXIS_SEL_LOCK_N          2000u /* 100 ms @20 kHz */
#endif
#ifndef M1_HFI_AXIS_SEL_CONFIRM_N
#define M1_HFI_AXIS_SEL_CONFIRM_N       2000u /* 100 ms 。d 确认 */
#endif
#ifndef M1_HFI_AXIS_SEL_COOLDOWN_N
#define M1_HFI_AXIS_SEL_COOLDOWN_N      4000u /* 200 ms，防连翻 */
#endif
#ifndef M1_HFI_AXIS_SEL_IQ_MAX
#define M1_HFI_AXIS_SEL_IQ_MAX          0.35f
#endif
/* 无感盆地质量 。Iq 权威（S3c0b）。不。enc。*/
#ifndef M1_HFI_IQ_AUTH_ENABLE
#define M1_HFI_IQ_AUTH_ENABLE           0
#endif
#ifndef M1_HFI_IQ_AUTH_X_GOOD
#define M1_HFI_IQ_AUTH_X_GOOD           0.218f
#endif
#ifndef M1_HFI_IQ_AUTH_X_BAD
#define M1_HFI_IQ_AUTH_X_BAD            0.205f
#endif
#ifndef M1_HFI_IQ_AUTH_EPS_FALSE_MAX
#define M1_HFI_IQ_AUTH_EPS_FALSE_MAX    0.20f
#endif
#ifndef M1_HFI_IQ_AUTH_HOLD_N
#define M1_HFI_IQ_AUTH_HOLD_N           2000u
#endif
#ifndef M1_HFI_IQ_AUTH_CLEAR_N
#define M1_HFI_IQ_AUTH_CLEAR_N          1u /* 1=立即清零（GATE。1）；C3b=1000 */
#endif
#ifndef M1_HFI_IQ_AUTH_IQ_LO
#define M1_HFI_IQ_AUTH_IQ_LO            0.25f
#endif
#ifndef M1_HFI_IQ_AUTH_IQ_HI
#define M1_HFI_IQ_AUTH_IQ_HI            3.50f
#endif
#ifndef M1_HFI_IQ_AUTH_SLEW_A_S
#define M1_HFI_IQ_AUTH_SLEW_A_S         4.0f
#endif
#ifndef M1_HFI_IQ_AUTH_FEED_ENABLE
#define M1_HFI_IQ_AUTH_FEED_ENABLE      0 /* 1=qual_ok 后馈 Iq（C4。*/
#endif
#ifndef M1_HFI_IQ_AUTH_FEED_SIGN
#define M1_HFI_IQ_AUTH_FEED_SIGN        1.0f
#endif
#ifndef M1_HFI_IQ_AUTH_FEED_A
#define M1_HFI_IQ_AUTH_FEED_A           0.0f /* >0：ok 时馈此幅值；0=改用 auth_abs */
#endif
#ifndef M1_HFI_IQ_AUTH_FEED_DELAY_S
#define M1_HFI_IQ_AUTH_FEED_DELAY_S     0.0f /* ok 后再等此时长才开始馈 */
#endif
#ifndef M1_HFI_IQ_AUTH_FEED_RAMP_S
#define M1_HFI_IQ_AUTH_FEED_RAMP_S      0.0f /* >0：从 0 斜坡。FEED_A。=阶跃 */
#endif
#ifndef M1_HFI_IQ_AUTH_FEED_HOLD
#define M1_HFI_IQ_AUTH_FEED_HOLD        0 /* 1：已开始馈流后，x 抖动不清指令 */
#endif
#ifndef M1_HFI_IQ_AUTH_FEED_COMPARE_AB
#define M1_HFI_IQ_AUTH_FEED_COMPARE_AB  0 /* 1：第1靴A延时阶跃，第2靴B斜坡 */
#endif
#ifndef M1_HFI_IQ_AUTH_FEED_A_DELAY_S
#define M1_HFI_IQ_AUTH_FEED_A_DELAY_S   0.20f
#endif
#ifndef M1_HFI_IQ_AUTH_FEED_A_RAMP_S
#define M1_HFI_IQ_AUTH_FEED_A_RAMP_S    0.0f
#endif
#ifndef M1_HFI_IQ_AUTH_FEED_B_DELAY_S
#define M1_HFI_IQ_AUTH_FEED_B_DELAY_S   0.0f
#endif
#ifndef M1_HFI_IQ_AUTH_FEED_B_RAMP_S
#define M1_HFI_IQ_AUTH_FEED_B_RAMP_S    0.80f
#endif
#ifndef M1_HFI_IQ_AUTH_FEED_LEGACY_BRANCH
#define M1_HFI_IQ_AUTH_FEED_LEGACY_BRANCH 0 /* 1=旧误接（1552/1612）；0=解调后馈 */
#endif
#ifndef M1_HFI_FEED_COAST_ENABLE
#define M1_HFI_FEED_COAST_ENABLE        0 /* 1：RUN 内掐 FEED 滑行对照 */
#endif
#ifndef M1_HFI_FEED_COAST_T0_S
#define M1_HFI_FEED_COAST_T0_S          6.0f /* 相对踢后 RUN 。stage_t */
#endif
#ifndef M1_HFI_FEED_COAST_T1_S
#define M1_HFI_FEED_COAST_T1_S          8.0f /* [T0,T1) 强制 Iq*=0 */
#endif
#if M1_HFI_IQ_AUTH_FEED_ENABLE && !M1_HFI_IQ_AUTH_ENABLE
#error "M1_HFI_IQ_AUTH_FEED_ENABLE requires M1_HFI_IQ_AUTH_ENABLE=1"
#endif
#if M1_HFI_FEED_COAST_ENABLE && !M1_HFI_IQ_AUTH_FEED_ENABLE
#error "M1_HFI_FEED_COAST_ENABLE requires M1_HFI_IQ_AUTH_FEED_ENABLE=1"
#endif
/* 1=axis_ok 后禁止再翻（静置验收 / 。2306 连翻。*/
#ifndef M1_HFI_AXIS_SEL_FREEZE_ON_OK
#define M1_HFI_AXIS_SEL_FREEZE_ON_OK    1
#endif
/* 未确认前最多翻几次。=不限。*/
#ifndef M1_HFI_AXIS_SEL_FLIP_MAX
#define M1_HFI_AXIS_SEL_FLIP_MAX        4u
#endif
/* SRC=3：ω_ff=(1−。·pll_int + α·ω_cmd；。M1_HFI_OMEGA_FF_REF_W（S1 离零。*/
#ifndef M1_HFI_OMEGA_FF_REF_W
#define M1_HFI_OMEGA_FF_REF_W           0.15f
#endif
#ifndef M1_HFI_OMEGA_SEED_ENABLE
#define M1_HFI_OMEGA_SEED_ENABLE        0
#endif
/* 固定 Iq 电流模式。ARM_ENABLE=0：全程保持该电流，不开速度。*/
#ifndef M1_HFI_IQ_PULL_ENABLE
#define M1_HFI_IQ_PULL_ENABLE           0
#endif
#ifndef M1_HFI_IQ_PULL_ARM_ENABLE
#define M1_HFI_IQ_PULL_ARM_ENABLE       1
#endif
#ifndef M1_HFI_IQ_PULL_DELAY_S
#define M1_HFI_IQ_PULL_DELAY_S          0.20f /* qual_ok 后再出力 */
#endif
#ifndef M1_HFI_PLL_HOLD_X_BELOW_A
#define M1_HFI_PLL_HOLD_X_BELOW_A       0 /* 1：x≤A 时不。±90° 。ε 写入 θ̂ */
#endif
#ifndef M1_HFI_IQ_PULL_A
#define M1_HFI_IQ_PULL_A                0.80f
#endif
#ifndef M1_HFI_IQ_PULL_S
#define M1_HFI_IQ_PULL_S                2.0f
#endif
#ifndef M1_HFI_IQ_PULL_ARM_RPM
#define M1_HFI_IQ_PULL_ARM_RPM          100.0f /* |pll_int| 超过此值才开始计保持 */
#endif
#ifndef M1_HFI_IQ_PULL_HOLD_S
#define M1_HFI_IQ_PULL_HOLD_S           0.10f /* 超过 ARM_RPM 后保持此时长再交速度。*/
#endif
#ifndef M1_HFI_IQ_PULL_ARM_MIN_S
#define M1_HFI_IQ_PULL_ARM_MIN_S        0.50f
#endif
/* GATE52：。 。0 斜坡自起（速度。+ SPEED_FB=HFI），禁止阶跃 ω* */
#ifndef M1_HFI_SPD_RAMP_ENABLE
#define M1_HFI_SPD_RAMP_ENABLE          0
#endif
#ifndef M1_HFI_SPD_HOLD0_S
#define M1_HFI_SPD_HOLD0_S              0.50f /* ω*=0 静置锁相 */
#endif
#ifndef M1_HFI_SPD_RAMP_S
#define M1_HFI_SPD_RAMP_S               2.0f /* 0→首档斜坡时。*/
#endif
#ifndef M1_HFI_PLL_INT_MAX
#define M1_HFI_PLL_INT_MAX              M1_HFI_PLL_W_MAX
#endif
#ifndef M1_HFI_PLL_EPS_DEAD
#define M1_HFI_PLL_EPS_DEAD             0.0f /* 0=关闭；启。profile 。~0.02 */
#endif
#ifndef M1_HFI_PLL_SKIP_X_BELOW_A
#define M1_HFI_PLL_SKIP_X_BELOW_A       0
#endif
#ifndef M1_HFI_PLL_VESC_ERR_ENABLE
#define M1_HFI_PLL_VESC_ERR_ENABLE      0
#endif
#ifndef M1_HFI_PLL_VESC_MAX_ERR
#define M1_HFI_PLL_VESC_MAX_ERR         (0.30f)
#endif
#ifndef M1_HFI_PLL_VESC_ERR_SIGN
#define M1_HFI_PLL_VESC_ERR_SIGN        (1.0f)
#endif
#ifndef M1_HFI_LQ_WELL_FLIP_ENABLE
#define M1_HFI_LQ_WELL_FLIP_ENABLE      0
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
#ifndef M1_HFI_PLL_INT_LEAK
#define M1_HFI_PLL_INT_LEAK             0.0f /* 死区内每拍漏泄系。*/
#endif
#ifndef M1_HFI_PLL_HOLD_IQ_A
#define M1_HFI_PLL_HOLD_IQ_A            0.65f
#endif
#ifndef M1_HFI_PLL_HOLD_REL_IQ_A
#define M1_HFI_PLL_HOLD_REL_IQ_A        0.60f
#endif
#ifndef M1_HFI_PLL_RETRACK_RPM
#define M1_HFI_PLL_RETRACK_RPM          20.0f
#endif
#ifndef M1_HFI_PLL_HOLD_RPM_LO
#define M1_HFI_PLL_HOLD_RPM_LO          8.0f
#endif
#ifndef M1_HFI_PLL_RETRACK_N
#define M1_HFI_PLL_RETRACK_N            2000u
#endif
#ifndef M1_HFI_PLL_RETRACK_USE_ENC
#define M1_HFI_PLL_RETRACK_USE_ENC      1
#endif
#ifndef M1_HFI_PLL_EPS_DEAD_SWEEP_N
#define M1_HFI_PLL_EPS_DEAD_SWEEP_N     1u
#endif
#ifndef M1_HFI_PLL_EPS_DEAD_0
#define M1_HFI_PLL_EPS_DEAD_0           M1_HFI_PLL_EPS_DEAD
#endif
#ifndef M1_HFI_PLL_EPS_DEAD_1
#define M1_HFI_PLL_EPS_DEAD_1           M1_HFI_PLL_EPS_DEAD
#endif
#ifndef M1_HFI_PLL_EPS_DEAD_2
#define M1_HFI_PLL_EPS_DEAD_2           M1_HFI_PLL_EPS_DEAD
#endif
#ifndef M1_HFI_PLL_EPS_DEAD_3
#define M1_HFI_PLL_EPS_DEAD_3           M1_HFI_PLL_EPS_DEAD
#endif
#ifndef M1_HFI_PLL_EPS_DEAD_4
#define M1_HFI_PLL_EPS_DEAD_4           M1_HFI_PLL_EPS_DEAD
#endif
#ifndef M1_HFI_PLL_EPS_DEAD_5
#define M1_HFI_PLL_EPS_DEAD_5           M1_HFI_PLL_EPS_DEAD
#endif
#ifndef M1_HFI_START_LADDER_VH_0
#define M1_HFI_START_LADDER_VH_0        M1_HFI_VH_V
#endif
#ifndef M1_HFI_START_LADDER_VH_1
#define M1_HFI_START_LADDER_VH_1        M1_HFI_VH_V
#endif
#ifndef M1_HFI_START_LADDER_VH_2
#define M1_HFI_START_LADDER_VH_2        M1_HFI_VH_V
#endif
#ifndef M1_HFI_START_LADDER_VH_3
#define M1_HFI_START_LADDER_VH_3        M1_HFI_VH_V
#endif
#ifndef M1_HFI_START_LADDER_VH_4
#define M1_HFI_START_LADDER_VH_4        M1_HFI_VH_V
#endif
#ifndef M1_HFI_START_LADDER_VH_5
#define M1_HFI_START_LADDER_VH_5        M1_HFI_VH_V
#endif
#ifndef M1_HFI_BIAS_CAL_ENABLE
#define M1_HFI_BIAS_CAL_ENABLE          0
#endif
#ifndef M1_HFI_BIAS_CAL_LEARN_S
#define M1_HFI_BIAS_CAL_LEARN_S         2.0f
#endif
#ifndef M1_HFI_BIAS_CAL_LP
#define M1_HFI_BIAS_CAL_LP              0.002f
#endif
#ifndef M1_HFI_BIAS_CAL_WMIN_RPM
#define M1_HFI_BIAS_CAL_WMIN_RPM        10.0f
#endif
#ifndef OBS_POLE_PAIRS
#define OBS_POLE_PAIRS                   7u
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
/* |θ_err| 超过此值计为异常；>PI_RAD 。π，否则翻 π/2（消 ~90° 假锁。*/
#ifndef M1_HFI_POLARITY_ERR_RAD
#define M1_HFI_POLARITY_ERR_RAD         ((float)M_PI * 0.5f)
#endif
#ifndef M1_HFI_POLARITY_PI_RAD
#define M1_HFI_POLARITY_PI_RAD          ((float)M_PI * 2.0f / 3.0f)
#endif
#ifndef M1_HFI_LOCK_ENABLE
#define M1_HFI_LOCK_ENABLE              0
#endif
#ifndef M1_HFI_LOCK_ERR_RAD
#define M1_HFI_LOCK_ERR_RAD             0.175f
#endif
#ifndef M1_HFI_UNLOCK_ERR_RAD
#define M1_HFI_UNLOCK_ERR_RAD           0.525f
#endif
#ifndef M1_HFI_LOCK_HOLD_N
#define M1_HFI_LOCK_HOLD_N              400u
#endif
#ifndef M1_HFI_UNLOCK_HOLD_N
#define M1_HFI_UNLOCK_HOLD_N            200u
#endif
#ifndef M1_HFI_INIT_FROM_ENC
#define M1_HFI_INIT_FROM_ENC            1
#endif
/* 工业 IPD：ALIGN(2θ) 。SETTLE 。P0 。SETTLE 。P1 。相对裕度判决 */
#ifndef M1_HFI_IQ_RAMP_PRE_S
#define M1_HFI_IQ_RAMP_PRE_S            2.0f
#endif
#ifndef M1_HFI_IQ_RAMP_S
#define M1_HFI_IQ_RAMP_S                12.0f
#endif
#ifndef M1_HFI_IQ_RAMP_HOLD_S
#define M1_HFI_IQ_RAMP_HOLD_S           8.0f
#endif
#ifndef M1_HFI_IQ_RAMP_A_MAX
#define M1_HFI_IQ_RAMP_A_MAX            2.0f
#endif
#ifndef M1_HFI_QKICK_AFTER_LOCK_ENABLE
#define M1_HFI_QKICK_AFTER_LOCK_ENABLE  0
#endif
#ifndef M1_HFI_QKICK_THEN_HFI_ENABLE
#define M1_HFI_QKICK_THEN_HFI_ENABLE    0 /* 1：S3b 踢完 hold→HFI RUN（沿用踢。θ̂。*/
#endif
#ifndef M1_HFI_ID_PI_OFF_ENABLE
#define M1_HFI_ID_PI_OFF_ENABLE         0 /* 1：踢。LOG/RUN 旁路 Id PI（C4k。*/
#endif
/* 放行 Id 。Ud 。0 爬到满的拍数。0 kHz）。700：硬开会打。SMO。*/
#ifndef M1_HFI_ID_PI_SOFT_N
#define M1_HFI_ID_PI_SOFT_N             20000u /* 1.0 s */
#endif
#ifndef M1_HFI_VESC_ID_HANDOFF_ENABLE
#define M1_HFI_VESC_ID_HANDOFF_ENABLE   0
#endif
#ifndef M1_HFI_HFI_ID_SOFT_ENABLE
#define M1_HFI_HFI_ID_SOFT_ENABLE      0 /* 1: pure-HFI soft-open Id after qual */
#endif
#ifndef M1_HFI_ID_ON_FROM_RUN_ENABLE
#define M1_HFI_ID_ON_FROM_RUN_ENABLE   0 /* 1: Id*=0 from RUN entry, no soft */
#endif
/* PRE 假锁门禁：|ε|。0° 。(ε小且|x|。 。。+π/2；|ε|持续小才准踢（C4n。*/
#ifndef M1_HFI_QKICK_PRE_S
#define M1_HFI_QKICK_PRE_S              0.2f
#endif
#ifndef M1_HFI_QKICK_HAT0_N
#define M1_HFI_QKICK_HAT0_N             160u /* 8 ms：Iq 阶跃过后再采 θ̂0 */
#endif
#ifndef M1_HFI_DQ_GATE_S
#define M1_HFI_DQ_GATE_S                0.5f
#endif
#ifndef M1_HFI_DQ_BLANK_S
#define M1_HFI_DQ_BLANK_S               0.10f /* ignore IDLE ε=0; PI/Vh settle */
#endif
#ifndef M1_HFI_DQ_VH_RAMP_S
#define M1_HFI_DQ_VH_RAMP_S             0.05f
#endif
#ifndef M1_HFI_DQ_GAP_S
#define M1_HFI_DQ_GAP_S                 0.30f
#endif
#ifndef M1_HFI_DQ_SENTINEL_S
#define M1_HFI_DQ_SENTINEL_S            1.0f
#endif
#ifndef M1_HFI_DQ_SENTINEL_A
#define M1_HFI_DQ_SENTINEL_A            0.10f
#endif
#ifndef M1_HFI_DQ_LOCK_TMO_S
#define M1_HFI_DQ_LOCK_TMO_S            15.0f
#endif
#ifndef M1_HFI_DQ_EPS_MAX_RAD
#define M1_HFI_DQ_EPS_MAX_RAD           0.2618f /* 15° */
#endif
#ifndef M1_HFI_DQ_STILL_RAD
#define M1_HFI_DQ_STILL_RAD             0.03491f /* 2° el */
#endif
#ifndef M1_HFI_DQ_MOVE_RAD
#define M1_HFI_DQ_MOVE_RAD              0.08727f /* 5° el */
#endif
#ifndef M1_HFI_DQ_MED_N
#define M1_HFI_DQ_MED_N                 32u /* 1.6 ms @20 kHz */
#endif
#if M1_HFI_DQ_IDENT_ENABLE && (M1_HFI_DQ_MED_N > 32u)
#error "M1_HFI_DQ_MED_N must be <= 32"
#endif
#ifndef M1_HFI_QKICK_FORCE_PI
#define M1_HFI_QKICK_FORCE_PI           0
#endif
#define M1_HFI_QKICK_ANY                1
#ifndef M1_HFI_QKICK_HOLD_S
#define M1_HFI_QKICK_HOLD_S             8.0f
#endif
/* Step3：CRAWL 内零。Iq 阶跃 0。A。→−A。，每。SEG_S；总时长仍。CRAWL_S */
/* CRAWL：踢后慢。Iq。→A_MAX），。Bv、无速度。*/
#ifndef M1_HFI_QKICK_PARK_ENC
#define M1_HFI_QKICK_PARK_ENC           0
#endif
/* S3c1：仅 CRAWL 段力。Park=enc；踢/LOG 。θ̂ */
#ifndef M1_HFI_QKICK_POL_DECIDE_S
#define M1_HFI_QKICK_POL_DECIDE_S       1.5f
#endif
#ifndef M1_HFI_QKICK_POL_W_RPM
#define M1_HFI_QKICK_POL_W_RPM          8.0f
#endif
/* Step4：CRAWL 内短 I–f 离零（Park=θ_if，恒 Iq，ω_cmd 慢斜坡）；不开 M1_IF 全路。*/
/* 踢后正式起动：HOLD→CAPTURE(固定Iq+电子阻尼)→HANDOVER→RUN */
/* CAPTURE：Iq = Iq_cmd 。Bv·ω（空轴用电子阻尼代机。B；非角误。PI。*/
/* 0=CAPTURE 结束。DONE，不开无感速环。551：交接即反向飞车。*/
#ifndef M1_HFI_QKICK_GATE_EPS_LIGHT
#define M1_HFI_QKICK_GATE_EPS_LIGHT     0.35f
#endif
#ifndef M1_HFI_QKICK_GATE_LIGHT_N
#define M1_HFI_QKICK_GATE_LIGHT_N       400u /* 20 ms @20kHz */
#endif
#ifndef M1_HFI_QKICK_GATE_EPS_HEAVY
#define M1_HFI_QKICK_GATE_EPS_HEAVY     0.45f
#endif
#ifndef M1_HFI_QKICK_GATE_W_RPM_HEAVY
#define M1_HFI_QKICK_GATE_W_RPM_HEAVY   90.0f
#endif
#ifndef M1_HFI_QKICK_GATE_HEAVY_N
#define M1_HFI_QKICK_GATE_HEAVY_N       1600u /* 80 ms */
#endif
#ifndef M1_HFI_QKICK_GATE_FAKE_N
#define M1_HFI_QKICK_GATE_FAKE_N        2000u /* 100 ms：假。积分顶满+eps 死区 */
#endif
/* 踢后进速度环前的零速预。[s]（旧 S1；START 路径不用。*/
#ifndef M1_HFI_QKICK_IQ_A
#define M1_HFI_QKICK_IQ_A               1.0f
#endif
#ifndef M1_HFI_QKICK_KICK_N
#define M1_HFI_QKICK_KICK_N             400u /* 20 ms @20kHz */
#endif
#ifndef M1_HFI_QKICK_BRAKE_N
#define M1_HFI_QKICK_BRAKE_N            400u /* 20 ms；AFTER_LOCK 切相可能仍读 */
#endif
#ifndef M1_HFI_QKICK_DTH_MIN_RAD
#define M1_HFI_QKICK_DTH_MIN_RAD        0.03f /* ~1.7°el：无运动。*/
#endif
#ifndef M1_HFI_QKICK_SETTLE_MAX_N
#define M1_HFI_QKICK_SETTLE_MAX_N       1000u
#endif
#ifndef M1_HFI_QKICK_BEFORE_SETTLE_N
#define M1_HFI_QKICK_BEFORE_SETTLE_N    0u /* BEFORE_HFI：踢。HF 破粘拍数。=立刻。*/
#endif
#ifndef M1_HFI_QKICK_I_TH_A
#define M1_HFI_QKICK_I_TH_A             0.08f
#endif
/* 脉冲电压表：一次实验扫多档，LOG 。ch5=该档 Ud [V] */

enum {
    HFI_IPD_ALIGN = 0,
    HFI_IPD_SETTLE0,
    HFI_IPD_P0,
    HFI_IPD_SETTLE1,
    HFI_IPD_P1
};

static hfi_stage_t s_stage;
static hfi_lock_t s_lock;
static uint16_t s_lock_cnt;
static float s_theta_cmd;
static float s_theta_hat;
static float s_theta_err;
static float s_eps;
static float s_e_pll; /* VESC V4 残差；未开宏时等于 s_eps */
static float s_di_q;
static float s_di_d;     /* 半周差分 di_d，未。prev_sign */
static float s_x_raw;    /* XY_X_SIGN * prev_sign * di_d */
static float s_y_raw;    /* XY_Y_SIGN * prev_sign * di_q */
static float s_vh_sign;
static float s_vh_v; /* 运行时注入幅值；扫腿时按表切。*/
static float s_vh_scale = 1.0f; /* 交接。 全注入，0 关掉。不。Vh 标称。*/
static uint8_t s_id_pi_release; /* 1：允。Id PI（交。Park 已到 SMO。*/
static uint16_t s_id_pi_soft_n; /* 放行。Ud 软开计数（自动爬坡） */
static float s_id_pi_soft_cmd;  /* >=0：交接外给定权重。0：走自动爬坡 */
static uint8_t s_torque_ov;      /* 1：力矩角由交接写入，不是 θ̂ */
static float s_torque_theta;
static uint8_t s_hat_hold;       /* 1：。不再写入 θ̂ */
static float s_omega_coast;      /* 保持。θ̂ 按这个电角速度。[rad/s] */
static float s_stage_t;
static float s_theta_enc;
static float s_ud_inj;
static float s_uq_inj;
static float s_id_inj_prev;
static float s_iq_inj_prev;
static uint8_t s_inj_prev_valid;
static float s_id_inj_slow;
static float s_iq_inj_slow;
#if M1_HFI_DEMOD_INJ_AXIS
static float s_ia_now;
static float s_ib_now;
#endif
#if M1_HFI_DEMOD_INJ_AXIS
/* 上一拍注入轴。两拍 iαβ 都投到这组正余弦上，PLL 更新之前写入。 */
static float s_pair_c;
static float s_pair_s;
static float s_ia_pair;
static float s_ib_pair;
static float s_pair_di_q;
static uint8_t s_pair_ready;
#endif
/* θ̂ 自身转速的低通。118 用它做积分上限，119 的积分直接等于它。编码器不参与。 */
static float s_speed_est;
static float s_sign;
static float s_eps_lp;
static float s_x_lp;     /* x_raw 向量 LPF；初。A_cmd，避。atan2(0,-A) */
static float s_iq_auth_abs;   /* 当前 |Iq| 天花。*/
static uint16_t s_iq_auth_good_n;
static uint16_t s_iq_auth_bad_n;
static uint8_t s_iq_auth_ok;  /* 1=质量过线，目。HI */
static uint8_t s_iq_auth_hold; /* 1：注入故意关掉，天花板留。HI */
static float s_y_lp;
static float s_pll_int;
static float s_pll_eps_dead; /* 运行时死区；扫档时按表切。*/
static uint16_t s_polarity_cnt;
static float s_omega_ff_el;
static float s_omega_el;
static float s_omega_trim_el;
static float s_sh_int; /* 影子：无 ω_ff，跟主环 θ̂（凸极），不。Park */
static float s_sh_w;
static float s_sh_th;
static uint8_t s_sh_seed;
enum {
    HFI_QK_SEED = 0,
    HFI_QK_KICK = 1,
    HFI_QK_BRAKE = 2
};
static uint16_t s_qk_seed_i;    /* SWEEP:0/1；AFTER_LOCK:0正常/1强制π */
static uint16_t s_qk_cnt;
static uint8_t s_qk_phase;
static uint8_t s_qk_ov;
static float s_qk_ud;
static float s_qk_uq;
static float s_qk_iq_ref;
static float s_qk_id_ref;
static float s_qk_iabs;
static float s_qk_enc0;
static float s_qk_hat0;         /* AFTER_LOCK：踢。θ̂0（无感尺子） */
static float s_qk_dth;
static float s_qk_verdict;      /* +1 keep / -1 flipped / 0 nomotion */
static uint8_t s_qk_pi_reset;
static float s_qk_enc_prev;
static uint8_t s_qk_enc_prev_valid;
static uint8_t s_qk_done;       /* 本开机只踢一。*/
static uint8_t s_qk_pol_done;   /* CRAWL 。ω̂ 极性已。*/

/**
 * @brief VESC HFI V4：q 轴 di / (Vh·(1/Lq−1/Ld))
 * @note S1 也算，即使本拍不写 θ̂。限幅由 MAX_ERR 定，静态尺用大值。
 */
static float hfi_vesc_ang_err(float y_raw)
{
    const float inv_ld_lq = (1.0f / OBS_LQ_H) - (1.0f / OBS_LD_H);
    float vh = s_vh_v * s_vh_scale;
    float den;

    if (vh < 0.05f) {
        vh = 0.05f;
    }
    den = vh * inv_ld_lq;
    return motor_vesc_ang_err(y_raw, OBS_CTRL_TS_S, den,
                              M1_HFI_PLL_VESC_ERR_SIGN, M1_HFI_PLL_VESC_MAX_ERR);
}

/**
 * @brief 对外/Park 用的 θ̂（偏置在冻结时已并入 s_theta_hat。
 */
static float hfi_theta_hat_out(void)
{
    return s_theta_hat;
}

/**
 * @brief 当前 Park 是否已用 θ̂（仅 LOCKED；未开 lock 门时 RUN 即用 θ̂。
 */
static uint8_t hfi_park_uses_hat(void)
{
    if ((s_stage == HFI_STAGE_MEAS) || (s_stage == HFI_STAGE_LOG) ||
        (s_stage == HFI_STAGE_CRAWL) || (s_stage == HFI_STAGE_RUN)) {
        return 1u;
    }
#if M1_ENC_OPTIONAL_ENABLE
    /* IDLE/DONE 也不吃浮空 SPI */
    return 1u;
#else
    return 0u;
#endif
}

/**
 * @brief 当前 Park 坐标系角度（解调旋转用）
 */
static float hfi_park_frame_theta(void)
{
    if (hfi_park_uses_hat() != 0u) {
        return hfi_theta_hat_out();
    }
    return s_theta_enc;
}

/** 交接把力矩角转开时，解调不能再把 Park 当成 θ̂。*/
static uint8_t hfi_demod_on_hat(void)
{
    if (s_torque_ov != 0u) {
        return 0u;
    }
    return hfi_park_uses_hat();
}

static float hfi_demod_frame_theta(void)
{
    if (s_torque_ov != 0u) {
        return s_torque_theta;
    }
    return hfi_park_frame_theta();
}

/**
 * @brief 。|θ_err|（调试真值）更新捕锁状。
 * @note 产品路径再换成假。置信；失锁不。ω_ref 重种假。
 */
static void hfi_lock_update(void)
{
    /* LOCK_ENABLE=0：台架不走 |θ_err| 捕锁状态机 */
}

/**
 * @brief 清注入电。
 */
static void hfi_clear_inj(void)
{
    s_ud_inj = 0.0f;
    s_uq_inj = 0.0f;
    s_vh_sign = 0.0f;
}

/**
 * @brief 。θ̂ 轴上叠加 ±Vh
 * @note Park=enc 时电压在 enc 系，需。(θ̂−θ_enc)；Park=θ̂ 时直。Ud=±Vh
 */
static void hfi_set_inj_on_hat(void)
{
    s_vh_sign = s_sign;
    /*
     * Park 系直接叠 ±Vh：θ。。CAPTURE 。θ_cmd（电压已。Park 坐标）。
     * 。CAPTURE/。I–f 的「Park≠θ。。Park≠cmd」才旋到 enc。
     */
    if ((hfi_demod_on_hat() != 0u)
        ) {
        s_ud_inj = s_vh_v * s_vh_scale * s_sign;
        s_uq_inj = 0.0f;
    } else {
        /* Park≠θ̂：。hat 。d 轴的 ±Vh 表达。Park 系。。θ̂−θ_park */
        const float th = s_theta_hat - hfi_demod_frame_theta();
        const float c = cosf(th);
        const float s = sinf(th);

        s_ud_inj = s_vh_v * s_vh_scale * s_sign * c;
        s_uq_inj = s_vh_v * s_vh_scale * s_sign * s;
    }
}

/**
 * @brief RUN 总时。[s]
 */
static float hfi_run_total_s(void)
{
    /* 到了 ±1500 再保持 0.5 s 才换向，五轮。到不了则最多 40 s。 */
    return 40.0f;
}


/**
 * @brief 旁路 RUN 速度指令
 */
static int s_rev141_idx;
static float s_rev141_hold = -1.0f;
static uint8_t s_rev141_done;

static float hfi_run_speed_ref_rpm(void)
{
    {
        float cmd;
        float meas;
        const float rpm_scale =
            60.0f / (2.0f * 3.14159265f * (float)OBS_POLE_PAIRS);

        /*
         * 指令先停在 ±1500。速度观测到了这一档，再保持 0.5 s 才换向。
         * 正反各 5 次。保持时间用 s_stage_t，本函数一拍里会被调用多次。
         */
        if (s_rev141_idx > 9) {
            s_rev141_done = 1u;
            return -1500.0f;
        }
        cmd = ((s_rev141_idx % 2) == 0) ? 1500.0f : -1500.0f;
        if (s_rev141_hold >= 0.0f) {
            if ((s_stage_t - s_rev141_hold) >= 0.5f) {
                s_rev141_idx++;
                s_rev141_hold = -1.0f;
                if (s_rev141_idx > 9) {
                    s_rev141_done = 1u;
                    return cmd;
                }
                cmd = ((s_rev141_idx % 2) == 0) ? 1500.0f : -1500.0f;
            }
            return cmd;
        }
        meas = s_speed_est * rpm_scale;
        if (((cmd > 0.0f) && (meas >= 1500.0f)) ||
            ((cmd < 0.0f) && (meas <= -1500.0f))) {
            s_rev141_hold = s_stage_t;
        }
        return cmd;
    }
}

/**
 * @brief 机械 rpm 。电角速度 [rad/s]
 */
static float hfi_omega_el_from_rpm(float rpm_mech)
{
    return rpm_mech * (2.0f * (float)M_PI / 60.0f) * (float)OBS_POLE_PAIRS;
}


static void hfi_xy_lp_reset(void)
{
    s_x_lp = M1_HFI_A_CMD;
    s_y_lp = 0.0f;
    s_eps_lp = 0.0f;
    s_id_inj_slow = 0.0f;
    s_iq_inj_slow = 0.0f;
    s_iq_auth_abs = M1_HFI_IQ_AUTH_IQ_LO;
    s_iq_auth_good_n = 0u;
    s_iq_auth_bad_n = 0u;
    s_iq_auth_ok = 0u;
}


/**
 * 无感盆地质量 。|Iq| 天花板。
 * |x|≥GOOD：像。d；|x|≤BAD 。|ε| 小：假锁（A−B）→ 。Iq。
 * 清零需连续 CLEAR_N 拍（默认 1=立即；C3b=50 ms），抗真锁微掉线。
 * 不用 enc。
 */
static void hfi_iq_auth_note_bad(uint8_t *good_now)
{
    s_iq_auth_good_n = 0u;
    if (s_iq_auth_bad_n < 0xFFFFu) {
        s_iq_auth_bad_n++;
    }
    if (s_iq_auth_bad_n >= (uint16_t)M1_HFI_IQ_AUTH_CLEAR_N) {
        s_iq_auth_ok = 0u;
    }
    *good_now = s_iq_auth_ok;
}

static void hfi_iq_auth_step(void)
{
    float ax = s_x_lp;
    float ae = s_eps;
    float target;
    float step;
    uint8_t good_now;

    if (ax < 0.0f) {
        ax = -ax;
    }
    if (ae < 0.0f) {
        ae = -ae;
    }

    if (s_iq_auth_hold != 0u) {
        /* 交接收注入：x 会掉。0，这不是假锁。天花板留在 HI。*/
        s_iq_auth_bad_n = 0u;
        s_iq_auth_ok = 1u;
        good_now = 1u;
    } else if ((ax <= M1_HFI_IQ_AUTH_X_BAD) && (ae <= M1_HFI_IQ_AUTH_EPS_FALSE_MAX)) {
        hfi_iq_auth_note_bad(&good_now);
    } else if (ax >= M1_HFI_IQ_AUTH_X_GOOD) {
        s_iq_auth_bad_n = 0u;
        if (s_iq_auth_good_n < 0xFFFFu) {
            s_iq_auth_good_n++;
        }
        if (s_iq_auth_good_n >= (uint16_t)M1_HFI_IQ_AUTH_HOLD_N) {
            s_iq_auth_ok = 1u;
        }
        good_now = s_iq_auth_ok;
    } else {
        /* 中间带：保持。ok，但升起累计清零，避免边缘抖动凑。*/
        s_iq_auth_good_n = 0u;
        if (ax <= M1_HFI_IQ_AUTH_X_BAD) {
            hfi_iq_auth_note_bad(&good_now);
        } else {
            s_iq_auth_bad_n = 0u;
            good_now = s_iq_auth_ok;
        }
    }

    target = (good_now != 0u) ? M1_HFI_IQ_AUTH_IQ_HI : M1_HFI_IQ_AUTH_IQ_LO;
    step = M1_HFI_IQ_AUTH_SLEW_A_S * OBS_CTRL_TS_S;
    if (s_iq_auth_abs < target) {
        s_iq_auth_abs += step;
        if (s_iq_auth_abs > target) {
            s_iq_auth_abs = target;
        }
    } else if (s_iq_auth_abs > target) {
        s_iq_auth_abs -= step;
        if (s_iq_auth_abs < target) {
            s_iq_auth_abs = target;
        }
    }
}


/**
 * @brief 半周差分解调；update_angle=1 时跑 PLL 更新 θ̂
 * @note Park=enc：id/iq 。enc 系，旋到 θ̂；Park=θ̂：id/iq 已在 θ̂ 。
 */

static void hfi_demod_step(float id, float iq, uint8_t update_angle)
{
    float id_inj;
    float iq_inj;

    if ((s_hat_hold != 0u) && (s_vh_scale <= 0.0f)) {
        /* 注入已关：不再解调。θ。只按保持速度走。*/
        s_omega_el = s_omega_coast;
        s_theta_hat = motor_wrap_pi(s_theta_hat + s_omega_coast * OBS_CTRL_TS_S);
        return;
    }

#if M1_HFI_DEMOD_INJ_AXIS
    /*
     * s_pair_c/s 是上一拍入口的 θ̂，也就是打出「本拍电流」的那根注入轴。
     * 本拍和上一拍的 iαβ 都投到这根轴上，用高通之前的电流。算完再允许 PLL 改 θ̂。
     */
    s_pair_ready = 0u;
    if (s_inj_prev_valid != 0u) {
        const float iq_now = -s_ia_now * s_pair_s + s_ib_now * s_pair_c;
        const float iq_prev = -s_ia_pair * s_pair_s + s_ib_pair * s_pair_c;

        s_pair_di_q = iq_now - iq_prev;
        s_pair_ready = 1u;
    }
    s_pair_c = cosf(s_theta_hat);
    s_pair_s = sinf(s_theta_hat);
    s_ia_pair = s_ia_now;
    s_ib_pair = s_ib_now;
#endif

#if M1_HFI_DEMOD_INJ_AXIS
    /* 注入轴电流：iαβ 投到 θ̂，不经过 Park Id（Id PI 开着也不吃基波）。 */
    {
        const float c = cosf(s_theta_hat);
        const float s = sinf(s_theta_hat);

        id_inj = s_ia_now * c + s_ib_now * s;
        iq_inj = -s_ia_now * s + s_ib_now * c;
        (void)id;
        (void)iq;
    }
#else
    /* Park=θ̂：id/iq 已在 θ̂ 系；否则从 Park 系旋到 θ̂ 再解调 */
    if (hfi_demod_on_hat() != 0u) {
        id_inj = id;
        iq_inj = iq;
    } else {
        const float th = s_theta_hat - hfi_demod_frame_theta();
        const float c = cosf(th);
        const float s = sinf(th);

        id_inj = id * c + iq * s;
        iq_inj = -id * s + iq * c;
    }
#endif

    /* 半周 di 吃高频；基波/FEED 慢斜率不进 x,y。电流环仍用裸 Id。 */
    /* 踢段不要把慢环预载到 FEED 量级 */
    if (s_stage != HFI_STAGE_MEAS)
    {
    if (s_inj_prev_valid == 0u) {
        s_id_inj_slow = id_inj;
        s_iq_inj_slow = iq_inj;
    } else {
        s_id_inj_slow += M1_HFI_DEMOD_HP_A * (id_inj - s_id_inj_slow);
        s_iq_inj_slow += M1_HFI_DEMOD_HP_A * (iq_inj - s_iq_inj_slow);
    }
    id_inj = id_inj - s_id_inj_slow;
    iq_inj = iq_inj - s_iq_inj_slow;
    }

    if (s_inj_prev_valid != 0u) {
        float di_d;
        float di_q;
        uint8_t di_ok = 1u;
        di_d = id_inj - s_id_inj_prev;
        di_q = iq_inj - s_iq_inj_prev;
        if (di_ok != 0u) {
        const float prev_sign = -s_sign;
        const float a = 0.05f;

        s_di_d = di_d;
        s_di_q = di_q;
        s_x_raw = M1_HFI_XY_X_SIGN * prev_sign * di_d;
        s_y_raw = M1_HFI_XY_Y_SIGN * prev_sign * di_q;
        s_e_pll = hfi_vesc_ang_err(s_y_raw);
#if M1_HFI_DEMOD_INJ_AXIS && M1_HFI_PLL_VESC_ERR_ENABLE
        if (s_pair_ready != 0u) {
            const float y_pair = M1_HFI_XY_Y_SIGN * prev_sign * s_pair_di_q;

            s_e_pll = hfi_vesc_ang_err(y_pair);
        }
#endif
        /* 踢段只记 raw，LPF/AUTH/PLL 冻结 */
        if (s_stage != HFI_STAGE_MEAS)
        {
        /* P0-1：滤向量再取角，避免 ±π 支割上把 ±90° 抹平 */
        s_x_lp += a * (s_x_raw - s_x_lp);
        s_y_lp += a * (s_y_raw - s_y_lp);
        s_eps = 0.5f * atan2f(s_y_lp, s_x_lp - M1_HFI_A_CMD);
        s_eps_lp = s_eps;
        hfi_iq_auth_step();
        if (update_angle != 0u) {
#if M1_HFI_PLL_ENABLE
            {
                float w_ff;
                float dw;
                float e_pll = s_eps;
                float ae_eps;

                /* atan2 的 s_eps 仍给 PRE/AUTH/VOFA。115/116：PLL 直接吃 s_eps。 */
                e_pll = s_e_pll;
                ae_eps = e_pll;
                if (ae_eps < 0.0f) {
                    ae_eps = -ae_eps;
                }
                /*
                 * 死区：Iq 漏解调的小偏置不。Ki 爬假速（1626：eps。.01 积出 ~15 rpm）。
                 * 出死区才积分；死区内漏泄让停轴后 pll_int 回落。
                 * x≤A：atan2 。±90°，只反映 y 的符号。
                 * HOLD：这一。Kp 和积分都不写，避。±90° 。θ̂ 从转子上拖走。
                 * SKIP：只停积分，角度仍加 Kp·eps。829：积分停住后堵转）。
                 */
                {
                    /*
                     * 118 的低通。积分不再按 e·(Kp/400) 爬，直接等于上一拍 θ̂ 转速。
                     * 角度速率 = Kp·e + I，不夹 200 rad/s。e 闪一下时转速留在积分里。
                     */
                    (void)ae_eps;
                    s_pll_int = s_speed_est;
                    dw = M1_HFI_PLL_KP * e_pll + s_pll_int;
                }

                w_ff = 0.0f;
                s_omega_el = w_ff + dw;
                /* 与 VESC UTILS_LP_FAST(..., 0.01) 相同 */
                s_speed_est += 0.01f * (s_omega_el - s_speed_est);
                s_omega_trim_el = dw;
                /*
                 * 旁路 PLL：自。θ_sh，误。wrap(θ̂−θ_sh)，无 enc 前馈。
                 * 主环 θ̂ 已锁凸极，相当于干净的凸极角老师；影子模。
                 * 「无前馈。Type-II 能否跟住旋转凸极」。不。Park。
                 */
                if (s_sh_seed == 0u) {
                    s_sh_th = s_theta_hat;
                    s_sh_int = 0.0f;
                    s_sh_w = 0.0f;
                    s_sh_seed = 1u;
                } else {
                    float e_sh = motor_wrap_pi(s_theta_hat - s_sh_th);
                    float ae_sh = e_sh;

                    if (ae_sh < 0.0f) {
                        ae_sh = -ae_sh;
                    }
                    if ((s_pll_eps_dead <= 0.0f) || (ae_sh >= s_pll_eps_dead)) {
                        s_sh_int += M1_HFI_PLL_KI * e_sh * OBS_CTRL_TS_S;
                    }
                    s_sh_int = motor_clampf(s_sh_int, M1_HFI_PLL_INT_MAX);
                    s_sh_w = motor_clampf(M1_HFI_PLL_KP * e_sh + s_sh_int,
                                        M1_HFI_PLL_INT_MAX);
                    s_sh_th = motor_wrap_pi(s_sh_th + s_sh_w * OBS_CTRL_TS_S);
                }
                s_theta_hat = motor_wrap_pi(s_theta_hat + s_omega_el * OBS_CTRL_TS_S);
                s_theta_err = motor_wrap_pi(hfi_theta_hat_out() - s_theta_enc);

                hfi_lock_update();
            }
#endif
        }
        }
        }
    } else {
        s_inj_prev_valid = 1u;
    }

    s_id_inj_prev = id_inj;
    s_iq_inj_prev = iq_inj;
    if (s_hat_hold != 0u) {
        /* 收注入：ε 已不可用。θ。按保持速度继续走，不再停在原地。*/
        s_omega_el = s_omega_coast;
        s_theta_hat = motor_wrap_pi(s_theta_hat + s_omega_coast * OBS_CTRL_TS_S);
    }
    s_sign = -s_sign;
}




/** RUN 起 Id 常开：release + soft=1（Id*=0），允许与满 VH 共存。 */
static void hfi_id_on_from_run_arm(void)
{
    s_id_pi_release = 1u;
    s_id_pi_soft_n = 0u;
    s_id_pi_soft_cmd = 1.0f;
}

/**
 * @brief 进入旁路 RUN
 * @note SEED 模式：pll_int←ω_ref；Step4：θ。可沿。IPD，不再强。enc 播种
 */
static void hfi_enter_run(void)
{
    s_stage = HFI_STAGE_RUN;
    s_stage_t = 0.0f;
    s_rev141_idx = 0;
    s_rev141_hold = -1.0f;
    s_rev141_done = 0u;
    s_sign = 1.0f;
    s_inj_prev_valid = 0u;
    hfi_xy_lp_reset();
    s_di_q = 0.0f;
    s_eps = 0.0f;
    s_polarity_cnt = 0u;
    s_lock = HFI_LOCK_CAPTURE;
    s_lock_cnt = 0u;
    /* IDLE/DONE 强制 0V 。PI 会顶满；。RUN 必须卸掉，否则首。Ud~十数 V 。HFI */
    s_qk_pi_reset = 1u;
#if M1_HFI_INIT_FROM_ENC && !M1_ENC_OPTIONAL_ENABLE
    s_theta_hat = motor_wrap_pi(s_theta_enc + M1_HFI_PLL_INIT_OFF_RAD);
#else
    s_theta_hat = motor_wrap_pi(M1_HFI_PLL_INIT_OFF_RAD);
#endif
    s_pll_int = 0.0f;
    s_omega_el = 0.0f; /* 禁止把上层残留的 enc ω 种进 PLL */
    s_omega_trim_el = 0.0f;
    hfi_id_on_from_run_arm();
}

/**
 * @brief 极性踢后进 HFI RUN：保。decide 后的 θ̂，禁止再。enc 播种。
 * @note BEFORE_HFI：踢完直进；THEN_HFI：LOG hold 后再进。
 */
static void hfi_enter_run_after_kick(void)
{
    s_stage = HFI_STAGE_RUN;
    s_stage_t = 0.0f;
    s_rev141_idx = 0;
    s_rev141_hold = -1.0f;
    s_rev141_done = 0u;
    s_sign = 1.0f;
    s_inj_prev_valid = 0u;
    s_di_q = 0.0f;
    s_polarity_cnt = 0u;
    s_lock = HFI_LOCK_CAPTURE;
    s_lock_cnt = 0u;
    s_pll_int = 0.0f;
    s_omega_el = 0.0f;
    s_omega_trim_el = 0.0f;
    s_qk_ov = 0u;
    s_qk_ud = 0.0f;
    s_qk_uq = 0.0f;
    s_qk_iq_ref = 0.0f;
    s_qk_id_ref = 0.0f;
    s_qk_pi_reset = 1u;
    s_theta_err = motor_wrap_pi(s_theta_hat - s_theta_enc);
    s_theta_cmd = s_theta_hat;
    hfi_set_inj_on_hat();
    hfi_id_on_from_run_arm();
}






static void hfi_qk_set_zero_u(void)
{
    s_qk_ov = 1u;
    s_qk_ud = 0.0f;
    s_qk_uq = 0.0f;
    s_qk_iq_ref = 0.0f;
    s_qk_id_ref = 0.0f;
    hfi_clear_inj();
}


/**
 * @brief 踢段结束：用 Δθ_enc 判南北，并把冻住期间的轴位移补回 θ̂。
 * @note MEAS 。PLL。正。θ̂+=dth；反。θ̂+=dth+π。246：只。π 会落。q 再掉回南）。
 */
static void hfi_qk_decide_after_lock(void)
{
    float ae = s_qk_dth;

    if (ae < 0.0f) {
        ae = -ae;
    }
    /* +Iq 期望正转。反。= 锁在 d+π。不。= 拒判。*/
    if (ae < M1_HFI_QKICK_DTH_MIN_RAD) {
#if M1_ENC_OPTIONAL_ENABLE
        /* 拔线 SPI 钉死：不拒判，默认北。插着仍走 Δθ_enc。 */
        s_qk_verdict = 1.0f;
        s_lock = HFI_LOCK_LOCKED;
#else
        s_qk_verdict = 0.0f;
        s_lock = HFI_LOCK_FAULT;
#endif
    } else if (s_qk_dth > 0.0f) {
        /* 北：补踢段位移，不翻 π */
        s_qk_verdict = 1.0f;
        s_theta_hat = motor_wrap_pi(s_theta_hat + s_qk_dth);
        s_pll_int = 0.0f;
        s_omega_el = 0.0f;
        s_lock = HFI_LOCK_LOCKED;
    } else {
        /* 南：补位。+ 。π 。落在。d 附近再进 LOG */
        s_qk_verdict = -1.0f;
        s_theta_hat = motor_wrap_pi(s_theta_hat + s_qk_dth + (float)M_PI);
        s_pll_int = 0.0f;
        s_omega_el = 0.0f;
        s_lock = HFI_LOCK_LOCKED;
    }
    s_theta_err = motor_wrap_pi(s_theta_hat - s_theta_enc);
    s_di_q = s_qk_verdict;
    s_eps = s_qk_dth;
    s_vh_sign = (s_qk_verdict < 0.0f) ? -1.0f : 1.0f;
    s_omega_trim_el = M1_HFI_QKICK_IQ_A;
    s_theta_cmd = s_theta_hat;
}

static void hfi_qk_enter_from_lock(void)
{
    s_qk_done = 1u;
    s_qk_seed_i = 0u;
#if M1_HFI_QKICK_FORCE_PI
    s_theta_hat = motor_wrap_pi(s_theta_hat + (float)M_PI);
    s_qk_seed_i = 1u; /* 标记：踢前故意错 π */
#endif
    s_theta_err = motor_wrap_pi(s_theta_hat - s_theta_enc);
    s_theta_cmd = s_theta_hat;
    s_stage = HFI_STAGE_MEAS;
    s_stage_t = 0.0f;
    s_qk_cnt = 0u;
    s_qk_phase = (uint8_t)HFI_QK_SEED;
    s_qk_dth = 0.0f;
    s_qk_hat0 = s_theta_hat;
    s_qk_verdict = 0.0f;
    s_qk_enc_prev_valid = 0u;
    s_qk_iq_ref = 0.0f;
    s_qk_ov = 0u;
    /* 踢段。PLL（on_current update_angle=0），不跟泄漏。*/
    s_omega_trim_el = M1_HFI_QKICK_IQ_A;
    s_vh_sign = 1.0f;
}





static void hfi_qk_finish_meas_to_next(void)
{
    s_eps = s_qk_dth;
    s_di_q = s_qk_verdict;
    s_theta_err = motor_wrap_pi(s_theta_hat - s_theta_enc);
    /* LOG：Iq=0 + 注入 + PLL 解冻（同 S3a 静置），勿清 inj / 勿强。0V */
    s_qk_ov = 0u;
    s_qk_ud = 0.0f;
    s_qk_uq = 0.0f;
    s_qk_iq_ref = 0.0f;
    s_qk_id_ref = 0.0f;
    s_qk_pi_reset = 1u;
    s_pll_int = 0.0f;
    s_omega_el = 0.0f;
    s_inj_prev_valid = 0u;
    hfi_set_inj_on_hat();
    s_stage = HFI_STAGE_LOG;
    s_stage_t = 0.0f;
    s_qk_phase = (uint8_t)HFI_QK_SEED;
}

static void hfi_qk_on_meas(void)
{
    hfi_set_inj_on_hat();
    s_theta_err = motor_wrap_pi(s_theta_hat - s_theta_enc);
    s_qk_cnt++;


    switch (s_qk_phase) {
    case HFI_QK_SEED:
        s_qk_enc0 = s_theta_enc;
        s_qk_hat0 = s_theta_hat;
        s_qk_dth = 0.0f;
        s_qk_iq_ref = M1_HFI_QKICK_IQ_A;
        s_qk_ov = 0u;
        s_qk_pi_reset = 1u;
        s_qk_phase = (uint8_t)HFI_QK_KICK;
        s_qk_cnt = 0u;
        break;

    case HFI_QK_KICK:
        s_qk_iq_ref = M1_HFI_QKICK_IQ_A;
        s_qk_ov = 0u;
        if (s_qk_cnt == M1_HFI_QKICK_HAT0_N) {
            s_qk_hat0 = s_theta_hat;
        }
        s_qk_dth = motor_wrap_pi(s_theta_enc - s_qk_enc0);
        s_eps = s_qk_dth;
        if (s_qk_cnt >= M1_HFI_QKICK_KICK_N) {
            hfi_qk_decide_after_lock();
            hfi_qk_finish_meas_to_next();
        }
        break;

    case HFI_QK_BRAKE:
    default:
        s_qk_iq_ref = -M1_HFI_QKICK_IQ_A;
        s_qk_ov = 0u;
        if (s_qk_cnt >= M1_HFI_QKICK_BRAKE_N) {
            hfi_qk_finish_meas_to_next();
        }
        break;
    }
}

void hfi_sqwave_init(void)
{
    hfi_sqwave_reset();
}

void hfi_sqwave_reset(void)
{
    s_stage = HFI_STAGE_IDLE;
    s_lock = HFI_LOCK_CAPTURE;
    s_lock_cnt = 0u;
    s_theta_cmd = 0.0f;
    s_theta_hat = 0.0f;
    s_theta_err = 0.0f;
    s_eps = 0.0f;
    s_di_q = 0.0f;
    s_di_d = 0.0f;
    s_x_raw = 0.0f;
    s_y_raw = 0.0f;
    s_vh_sign = 0.0f;
    s_vh_v = M1_HFI_VH_V;
    s_vh_scale = 1.0f;
    s_id_pi_release = 0u;
    s_id_pi_soft_n = 0u;
    s_id_pi_soft_cmd = -1.0f;
    s_torque_ov = 0u;
    s_torque_theta = 0.0f;
    s_hat_hold = 0u;
    s_omega_coast = 0.0f;
    s_stage_t = 0.0f;
    s_theta_enc = 0.0f;
    hfi_clear_inj();
    s_id_inj_prev = 0.0f;
    s_iq_inj_prev = 0.0f;
    s_inj_prev_valid = 0u;
    s_sign = 1.0f;
    hfi_xy_lp_reset();
    s_pll_int = 0.0f;
    s_speed_est = 0.0f;
    s_pll_eps_dead = M1_HFI_PLL_EPS_DEAD;
    s_polarity_cnt = 0u;
    s_omega_ff_el = 0.0f;
    s_omega_el = 0.0f;
    s_omega_trim_el = 0.0f;
    s_sh_int = 0.0f;
    s_sh_w = 0.0f;
    s_sh_th = 0.0f;
    s_sh_seed = 0u;
    s_qk_seed_i = 0u;
    s_qk_cnt = 0u;
    s_qk_phase = (uint8_t)HFI_QK_SEED;
    s_qk_ov = 0u;
    s_qk_ud = 0.0f;
    s_qk_uq = 0.0f;
    s_qk_iq_ref = 0.0f;
    s_qk_id_ref = 0.0f;
    s_qk_iabs = 0.0f;
    s_qk_enc0 = 0.0f;
    s_qk_hat0 = 0.0f;
    s_qk_dth = 0.0f;
    s_qk_verdict = 0.0f;
    s_qk_pi_reset = 0u;
    s_qk_enc_prev = 0.0f;
    s_qk_enc_prev_valid = 0u;
    s_qk_done = 0u;
    s_qk_pol_done = 0u;
    (void)M1_HFI_FH_HZ;
}

void hfi_sqwave_set_omega_ff_el(float omega_el_rad_s)
{
    s_omega_ff_el = omega_el_rad_s;
}

void hfi_sqwave_on_angle(float theta_enc_el, float dt)
{
    const float run_total_s = hfi_run_total_s();

    s_theta_enc = theta_enc_el;

    if (dt <= 0.0f) {
        dt = OBS_CTRL_TS_S;
    }
    s_stage_t += dt;

    s_theta_err = motor_wrap_pi(s_theta_hat - theta_enc_el);
    switch (s_stage) {
    case HFI_STAGE_IDLE:
        hfi_clear_inj();
        if (s_stage_t >= M1_HFI_BOOT_DELAY_S) {
            hfi_enter_run();
        }
        break;
    case HFI_STAGE_RUN:
        hfi_set_inj_on_hat();
        /* S3b：预。PRE_S 后踢；PRE_GATE：须 pre_ok，否则逃。等待，超时拒。*/
        if ((s_qk_done == 0u) && (s_stage_t >= M1_HFI_QKICK_PRE_S)) {
            hfi_qk_enter_from_lock();
            break;
        }
        if ((s_rev141_done != 0u) || (s_stage_t >= run_total_s)) {
            s_stage = HFI_STAGE_DONE;
            s_stage_t = 0.0f;
            hfi_clear_inj();
            s_omega_el = 0.0f;
            s_omega_trim_el = 0.0f;
            s_lock = HFI_LOCK_CAPTURE;
            s_lock_cnt = 0u;
        }
        break;
    case HFI_STAGE_MEAS:
        hfi_qk_on_meas();
        break;
    case HFI_STAGE_LOG:
        /* 注入 + 电流。Iq=0；eps 。on_current 解调写入，勿。dth 盖掉 */
        hfi_set_inj_on_hat();
        s_qk_iq_ref = 0.0f;
        s_qk_id_ref = 0.0f;
        s_qk_ov = 0u;
        s_omega_trim_el = s_qk_verdict;
        s_theta_err = motor_wrap_pi(s_theta_hat - s_theta_enc);
        if (s_stage_t >= M1_HFI_QKICK_HOLD_S) {
            /* S3b 踢结。。。hold 。HFI RUN（qk_done=1，不会再踢） */
            hfi_enter_run_after_kick();
        }
        break;
    case HFI_STAGE_DONE:
    default:
        hfi_clear_inj();
        hfi_qk_set_zero_u();
        break;
    }
}

void hfi_sqwave_on_current(float id, float iq, float i_alpha, float i_beta)
{
#if M1_HFI_DEMOD_INJ_AXIS
    s_ia_now = i_alpha;
    s_ib_now = i_beta;
#else
    (void)i_alpha;
    (void)i_beta;
#endif
    {
        float iabs = id * id + iq * iq;

        if (iabs > 0.0f) {
            iabs = sqrtf(iabs);
        } else {
            iabs = 0.0f;
        }
        s_qk_iabs = iabs;
    }
    if (s_stage == HFI_STAGE_RUN) {
        {
            uint8_t upd = 1u;

            if (s_hat_hold != 0u) {
                upd = 0u;
            }
            hfi_demod_step(id, iq, upd);
        }
    } else if (s_stage == HFI_STAGE_MEAS) {
        hfi_demod_step(id, iq, 0u); /* 踢段冻 θ̂，避 Iq 泄漏 */
    } else if (s_stage == HFI_STAGE_LOG) {
        hfi_demod_step(id, iq, 1u); /* 踢后静置再锁。205 冻角差） */
    } else if (s_stage == HFI_STAGE_CRAWL) {
        hfi_demod_step(id, iq, 1u);
    }
}

float hfi_sqwave_park_theta(float theta_enc_el)
{
    if ((s_stage == HFI_STAGE_MEAS) || (s_stage == HFI_STAGE_LOG)) {
        return s_theta_hat;
    }
    if (s_stage == HFI_STAGE_CRAWL) {
        return hfi_theta_hat_out();
    }
    if (hfi_park_uses_hat() != 0u) {
        return hfi_theta_hat_out();
    }
    return theta_enc_el;
}

uint8_t hfi_sqwave_override_voltage(float *ud, float *uq)
{
    if (ud == NULL || uq == NULL) {
        return 0u;
    }
    if (s_stage == HFI_STAGE_RUN) {
        return 0u;
    }
    if ((s_stage == HFI_STAGE_MEAS) || (s_stage == HFI_STAGE_CRAWL) ||
        (s_stage == HFI_STAGE_LOG)) {
        return 0u; /* 。蠕动/LOG 再锁：电流环 + 注入 */
    }
    if ((s_stage == HFI_STAGE_DONE) || (s_stage == HFI_STAGE_IDLE)) {
        *ud = 0.0f;
        *uq = 0.0f;
        return 1u;
    }
    *ud = 0.0f;
    *uq = 0.0f;
    return 1u;
}

void hfi_sqwave_get_inj(float *ud_inj, float *uq_inj)
{
    /* INJECT_POST_LOOP：dq 注入口置 0，定子注入走 get_inj_ab */
    if (ud_inj != NULL) {
        *ud_inj = 0.0f;
    }
    if (uq_inj != NULL) {
        *uq_inj = 0.0f;
    }
}

void hfi_sqwave_get_inj_ab(float *u_alpha_inj, float *u_beta_inj)
{
    float ua = 0.0f;
    float ub = 0.0f;

    /* 用本半周锁定的 s_vh_sign，勿用解调后已翻转的 s_sign */
    if (s_vh_sign != 0.0f) {
        const float mag = s_vh_v * s_vh_scale * s_vh_sign;
        const float c = cosf(s_theta_hat);
        const float s = sinf(s_theta_hat);

        ua = mag * c;
        ub = mag * s;
    }
    if (u_alpha_inj != NULL) {
        *u_alpha_inj = ua;
    }
    if (u_beta_inj != NULL) {
        *u_beta_inj = ub;
    }
}

float hfi_sqwave_get_iq_ref(void)
{
    /* 踢段：s_qk_iq_ref */
    return s_qk_iq_ref;
}

float hfi_sqwave_get_id_ref(void)
{
    return s_qk_id_ref;
}


uint8_t hfi_sqwave_speed_run_active(void)
{
    /* C4r：THEN_HFI 踢完再开速度环；勿开 QKICK_SPEED（与 THEN_HFI 互斥。*/
    return ((s_stage == HFI_STAGE_RUN) && (s_qk_done != 0u)) ? 1u : 0u;
}

float hfi_sqwave_get_speed_ref_rpm(void)
{
    if (s_stage == HFI_STAGE_RUN) {
        return hfi_run_speed_ref_rpm();
    }
    return 0.0f;
}

uint8_t hfi_sqwave_if_leave_active(void)
{
    return 0u;
}

hfi_stage_t hfi_sqwave_get_stage(void)
{
    return s_stage;
}

hfi_lock_t hfi_sqwave_get_lock(void)
{
    return s_lock;
}

float hfi_sqwave_get_theta_cmd(void)
{
    return s_theta_cmd;
}

float hfi_sqwave_get_theta_hat(void)
{
    return hfi_theta_hat_out();
}

float hfi_sqwave_get_theta_err(void)
{
    return s_theta_err;
}

float hfi_sqwave_get_eps(void)
{
    return s_eps;
}

float hfi_sqwave_get_pll_vesc_err(void)
{
    return s_e_pll;
}

float hfi_sqwave_get_di_q(void)
{
    return s_di_q;
}

float hfi_sqwave_get_di_d(void)
{
    return s_di_d;
}

float hfi_sqwave_get_x_raw(void)
{
    return s_x_raw;
}

float hfi_sqwave_get_y_raw(void)
{
    return s_y_raw;
}

float hfi_sqwave_get_x_lp(void)
{
    return s_x_lp;
}

float hfi_sqwave_get_y_lp(void)
{
    return s_y_lp;
}

uint8_t hfi_sqwave_demod_probe_freeze(void)
{
    return 0u;
}

uint8_t hfi_sqwave_feed_coast_active(void)
{
    return 0u;
}

float hfi_sqwave_get_vh_sign(void)
{
    return s_vh_sign;
}

float hfi_sqwave_get_omega_el(void)
{
    return s_omega_el;
}

float hfi_sqwave_get_omega_trim_el(void)
{
    return s_omega_trim_el;
}

uint8_t hfi_sqwave_take_polarity_flip(void)
{
    return 0u;
}

void hfi_sqwave_set_inj_scale(float scale)
{
    if (scale < 0.0f) {
        scale = 0.0f;
    } else if (scale > 1.0f) {
        scale = 1.0f;
    }
    s_vh_scale = scale;
}

void hfi_sqwave_set_id_pi_release(uint8_t enable)
{
    uint8_t on;

    on = (enable != 0u) ? 1u : 0u;
    if ((on != 0u) && (s_id_pi_release == 0u)) {
        /* 上升沿：重新软开，避免中途再放行带着满增。*/
        s_id_pi_soft_n = 0u;
    }
    if (on == 0u) {
        s_id_pi_soft_n = 0u;
        s_id_pi_soft_cmd = -1.0f;
    }
    s_id_pi_release = on;
}

void hfi_sqwave_set_id_pi_soft_cmd(float scale)
{
    if (scale < 0.0f) {
        s_id_pi_soft_cmd = -1.0f;
        return;
    }
    if (scale > 1.0f) {
        scale = 1.0f;
    }
    s_id_pi_soft_cmd = scale;
}

/**
 * @brief Id→Ud 权重。外给定优先；否则放行后自动 0。。
 * @note 外给定模式不自增，由交接状态机每拍。set_id_pi_soft_cmd。
 */
float hfi_sqwave_get_id_pi_soft(void)
{
    return 1.0f;
}

float hfi_sqwave_id_pi_soft_scale(void)
{
    return 1.0f;
}

void hfi_sqwave_set_torque_theta(float theta, uint8_t enable)
{
    s_torque_ov = (enable != 0u) ? 1u : 0u;
    s_torque_theta = theta;
}

void hfi_sqwave_set_hat_hold(uint8_t hold)
{
    uint8_t on;

    on = (hold != 0u) ? 1u : 0u;
    if ((on != 0u) && (s_hat_hold == 0u)) {
        /* 刚进入保持：记下积分转速，不含这一拍的 Kp·ε。*/
        s_omega_coast = s_pll_int;
    }
    s_hat_hold = on;
}

void hfi_sqwave_set_hat_coast_el(float omega_el)
{
    if (s_hat_hold != 0u) {
        s_omega_coast = omega_el;
    }
}

void hfi_sqwave_seed_hat(float theta_el, float omega_el)
{
    s_theta_hat = motor_wrap_pi(theta_el);
    s_pll_int = omega_el;
    s_omega_el = omega_el;
    s_omega_coast = omega_el;
    s_hat_hold = 0u;
    s_theta_err = 0.0f;
    /* 下一拍积分直接等于这次播进去的转速 */
    s_speed_est = omega_el;
}

void hfi_sqwave_set_iq_auth_hold(uint8_t hold)
{
    s_iq_auth_hold = (hold != 0u) ? 1u : 0u;
}

void hfi_sqwave_flip_hat_pi(void)
{
    s_theta_hat = motor_wrap_pi(s_theta_hat + (float)M_PI);
    s_pll_int = 0.0f;
    s_omega_el = 0.0f;
    s_sh_seed = 0u;
}

float hfi_sqwave_get_pll_int_el(void)
{
    return s_pll_int;
}

float hfi_sqwave_get_omega_shadow_el(void)
{
    return s_sh_w;
}

float hfi_sqwave_get_eps_d(void)
{
    return 0.0f;
}

float hfi_sqwave_get_axis_flip_n(void)
{
    return 0.0f;
}

float hfi_sqwave_get_lq_well_flip_n(void)
{
    return 0.0f;
}

uint8_t hfi_sqwave_axis_ok(void)
{
    return 1u;
}

float hfi_sqwave_get_iq_auth_abs(void)
{
    return s_iq_auth_abs;
}

uint8_t hfi_sqwave_iq_auth_ok(void)
{
    return s_iq_auth_ok;
}

uint8_t hfi_sqwave_qk_pre_ok(void)
{
    return 1u;
}

float hfi_sqwave_get_qk_pre_flip_n(void)
{
    return 0.0f;
}

float hfi_sqwave_get_eps_dead(void)
{
    {
        float d = s_pll_eps_dead;

        return d;
    }
}

uint8_t hfi_sqwave_get_ipd_phase(void)
{
    return 0u;
}

float hfi_sqwave_get_ipd_pulse_ud(void)
{
    return s_qk_iq_ref;
}

uint8_t hfi_sqwave_get_qkick_phase(void)
{
    return s_qk_phase;
}

float hfi_sqwave_get_qkick_seed(void)
{
    return (float)s_qk_seed_i;
}

float hfi_sqwave_get_qkick_dth(void)
{
    return s_qk_dth;
}

float hfi_sqwave_get_qkick_verdict(void)
{
    return s_qk_verdict;
}

uint8_t hfi_sqwave_consume_pi_reset(void)
{
    if (s_qk_pi_reset != 0u) {
        s_qk_pi_reset = 0u;
        return 1u;
    }
    return 0u;
}

uint8_t hfi_sqwave_id_pi_bypass(void)
{
    return 0u;
}

uint8_t hfi_sqwave_get_sensed_cal_loop(void)
{
    return 0u;
}

static hfi_telem_snap_t s_telem;
static hfi_telem_snap_t s_harvest;
static uint8_t s_harvest_ok;
static volatile uint32_t s_telem_seq;

static void hfi_telem_dmb(void)
{
    __asm volatile("dmb" ::: "memory");
}

static void hfi_telem_fill(hfi_telem_snap_t *s)
{
    s->eps = hfi_sqwave_get_eps();
    s->pll_vesc_err = hfi_sqwave_get_pll_vesc_err();
    s->x_lp = hfi_sqwave_get_x_lp();
    s->y_lp = hfi_sqwave_get_y_lp();
    s->x_raw = hfi_sqwave_get_x_raw();
    s->y_raw = hfi_sqwave_get_y_raw();
    s->di_d = hfi_sqwave_get_di_d();
    s->di_q = hfi_sqwave_get_di_q();
    s->pll_int_el = hfi_sqwave_get_pll_int_el();
    s->qkick_dth = hfi_sqwave_get_qkick_dth();
    s->qkick_verdict = hfi_sqwave_get_qkick_verdict();
    s->qk_pre_flip_n = hfi_sqwave_get_qk_pre_flip_n();
    s->lq_well_flip_n = hfi_sqwave_get_lq_well_flip_n();
    s->vh_sign = hfi_sqwave_get_vh_sign();
    s->id_pi_soft = hfi_sqwave_get_id_pi_soft();
    hfi_sqwave_get_inj(&s->ud_inj, &s->uq_inj);
    s->sensed_cal_loop = hfi_sqwave_get_sensed_cal_loop();
    s->iq_auth_ok = hfi_sqwave_iq_auth_ok();
    s->qk_pre_ok = hfi_sqwave_qk_pre_ok();
    s->demod_probe_freeze = hfi_sqwave_demod_probe_freeze();
    s->feed_coast_active = hfi_sqwave_feed_coast_active();
}

/*
 * T3 时序契约（ISR）：
 * - snap 时刻钉在 observer_read_view→本函数；进 hfi_telem_snap_t 的量须在此之前更新完毕。
 * - 本拍随后到 observer_telem_publish 之间，禁止再调会改这些 getter 源的路径。
 * - publish 只提交 s_harvest（seqlock 拷贝），不再二次取数。
 * - s_harvest_ok==0 时 publish 内重填是漏 harvest 的错误路径，勿当正常用法。
 */
void hfi_sqwave_telem_harvest(hfi_telem_snap_t *out)
{
    hfi_telem_fill(&s_harvest);
    s_harvest_ok = 1u;
    if (out != 0) {
        *out = s_harvest;
    }
}

void hfi_sqwave_telem_publish(void)
{
    uint32_t seq = s_telem_seq;

    if (s_harvest_ok == 0u) {
        /* 错误路径：漏 harvest 时才重填，snap 时刻会挪到 ISR 末 */
        hfi_telem_fill(&s_harvest);
    }
    s_telem_seq = seq + 1u;
    hfi_telem_dmb();
    s_telem = s_harvest;
    hfi_telem_dmb();
    s_telem_seq = seq + 2u;
    s_harvest_ok = 0u;
}

void hfi_sqwave_telem_read(hfi_telem_snap_t *out)
{
    uint32_t s1;
    uint32_t s2;
    uint8_t n;

    if (out == NULL) {
        return;
    }
    n = 0u;
    do {
        s1 = s_telem_seq;
        hfi_telem_dmb();
        *out = s_telem;
        hfi_telem_dmb();
        s2 = s_telem_seq;
        n++;
    } while (((s1 != s2) || ((s1 & 1u) != 0u)) && (n < 8u));
}

