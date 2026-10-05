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

#if M1_HFI_ENABLE

#ifndef M1_HFI_MOTION_BYPASS_ENABLE
#define M1_HFI_MOTION_BYPASS_ENABLE     0
#endif
#ifndef M1_HFI_SPD_CLOSE_ENABLE
#define M1_HFI_SPD_CLOSE_ENABLE         0
#endif
#if M1_HFI_SPD_CLOSE_ENABLE
#ifndef M1_HFI_SPD_CLOSE_ERR_RAD
#define M1_HFI_SPD_CLOSE_ERR_RAD        (0.5235988f)
#endif
#ifndef M1_HFI_SPD_CLOSE_HOLD_S
#define M1_HFI_SPD_CLOSE_HOLD_S         (0.10f)
#endif
#ifndef M1_HFI_SPD_CLOSE_MIN_RPM
#define M1_HFI_SPD_CLOSE_MIN_RPM        (80.0f)
#endif
#ifndef M1_HFI_SPD_CLOSE_TARGET_RPM
#define M1_HFI_SPD_CLOSE_TARGET_RPM     (100.0f)
#endif
#ifndef M1_HFI_SPD_CLOSE_RAMP_RPM_S
#define M1_HFI_SPD_CLOSE_RAMP_RPM_S     (50.0f)
#endif
#endif
#ifndef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE       0
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
#ifndef M1_HFI_SENSED_CAL_ENABLE
#define M1_HFI_SENSED_CAL_ENABLE        0
#endif
#ifndef M1_HFI_SENSED_CAL_STEP_S
#define M1_HFI_SENSED_CAL_STEP_S        5.0f
#endif
#ifndef M1_HFI_SENSED_CAL_LOOPS
#define M1_HFI_SENSED_CAL_LOOPS         2u
#endif
#ifndef M1_HFI_SENSED_CAL_NRPM
#define M1_HFI_SENSED_CAL_NRPM          6u
#endif
#ifndef M1_HFI_SENSED_CAL_RPM0
#define M1_HFI_SENSED_CAL_RPM0          0.0f
#endif
#ifndef M1_HFI_SENSED_CAL_RPM1
#define M1_HFI_SENSED_CAL_RPM1          20.0f
#endif
#ifndef M1_HFI_SENSED_CAL_RPM2
#define M1_HFI_SENSED_CAL_RPM2          40.0f
#endif
#ifndef M1_HFI_SENSED_CAL_RPM3
#define M1_HFI_SENSED_CAL_RPM3          60.0f
#endif
#ifndef M1_HFI_SENSED_CAL_RPM4
#define M1_HFI_SENSED_CAL_RPM4          100.0f
#endif
#ifndef M1_HFI_SENSED_CAL_RPM5
#define M1_HFI_SENSED_CAL_RPM5          150.0f
#endif
#ifndef M1_HFI_DELTA_MIN_DEG
#define M1_HFI_DELTA_MIN_DEG            (-45.0f)
#endif
#ifndef M1_HFI_DELTA_MAX_DEG
#define M1_HFI_DELTA_MAX_DEG            (45.0f)
#endif
#ifndef M1_HFI_DELTA_STEP_DEG
#define M1_HFI_DELTA_STEP_DEG           (5.0f)
#endif
#ifndef M1_HFI_DELTA_HOLD_S
#define M1_HFI_DELTA_HOLD_S             (1.5f)
#endif
#ifndef M1_HFI_DELTA_TAIL_S
#define M1_HFI_DELTA_TAIL_S             (5.0f) /* 扫完 LOG 尾巴，方便录。*/
#endif
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
#ifndef M1_HFI_POLARITY_ENC_ENABLE
#define M1_HFI_POLARITY_ENC_ENABLE      1
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
#ifndef M1_HFI_PLL_HOLD_ENABLE
#define M1_HFI_PLL_HOLD_ENABLE          0
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
#ifndef M1_HFI_PLL_EPS_DEAD_SWEEP_ENABLE
#define M1_HFI_PLL_EPS_DEAD_SWEEP_ENABLE 0
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
#ifndef M1_HFI_POLARITY_IPD_ENABLE
#define M1_HFI_POLARITY_IPD_ENABLE      0
#endif
#ifndef M1_HFI_INIT_FROM_ENC
#define M1_HFI_INIT_FROM_ENC            1
#endif
/* 工业 IPD：ALIGN(2θ) 。SETTLE 。P0 。SETTLE 。P1 。相对裕度判决 */
#ifndef M1_HFI_IPD_ALIGN_N
#define M1_HFI_IPD_ALIGN_N              4000u /* 200 ms 最。ALIGN */
#endif
#ifndef M1_HFI_IPD_ALIGN_MAX_N
#define M1_HFI_IPD_ALIGN_MAX_N          8000u /* 400 ms 超时失败 */
#endif
#ifndef M1_HFI_IPD_ALIGN_ERR_RAD
#define M1_HFI_IPD_ALIGN_ERR_RAD        0.35f /* ~20°：台架用 enc 。ALIGN 质量 */
#endif
#ifndef M1_HFI_IPD_ALIGN_USE_ENC
#define M1_HFI_IPD_ALIGN_USE_ENC        1 /* 0=。|eps| 门（纯无感） */
#endif
#ifndef M1_HFI_IPD_ALIGN_EPS
#define M1_HFI_IPD_ALIGN_EPS            0.03f
#endif
#ifndef M1_HFI_IPD_ALIGN_HOLD_N
#define M1_HFI_IPD_ALIGN_HOLD_N         400u /* 20 ms 持续对准 */
#endif
#ifndef M1_HFI_IQ_RAMP_ENABLE
#define M1_HFI_IQ_RAMP_ENABLE           0
#endif
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
#ifndef M1_HFI_IPD_SETTLE_MAX_N
#define M1_HFI_IPD_SETTLE_MAX_N         1000u /* 50 ms 清流上限 */
#endif
#ifndef M1_HFI_IPD_I_TH_A
#define M1_HFI_IPD_I_TH_A               0.08f
#endif
#ifndef M1_HFI_IPD_PULSE_N
#define M1_HFI_IPD_PULSE_N              60u /* 3 ms @20kHz */
#endif
#ifndef M1_HFI_IPD_UD_V
#define M1_HFI_IPD_UD_V                 1.5f
#endif
#ifndef M1_HFI_IPD_REL_MIN
#define M1_HFI_IPD_REL_MIN              0.12f /* |p1-p0|/max 相对裕度 */
#endif
#ifndef M1_HFI_IPD_SWEEP_ENABLE
#define M1_HFI_IPD_SWEEP_ENABLE         0
#endif
#if M1_HFI_IPD_SWEEP_ENABLE && !M1_HFI_POLARITY_IPD_ENABLE
#error "M1_HFI_IPD_SWEEP_ENABLE requires M1_HFI_POLARITY_IPD_ENABLE=1"
#endif
#if M1_HFI_IPD_SWEEP_ENABLE && M1_HFI_DELTA_SWEEP_ENABLE
#error "M1_HFI_IPD_SWEEP_ENABLE conflicts with M1_HFI_DELTA_SWEEP_ENABLE"
#endif
#ifndef M1_HFI_QKICK_SWEEP_ENABLE
#define M1_HFI_QKICK_SWEEP_ENABLE       0
#endif
#if M1_HFI_QKICK_SWEEP_ENABLE && M1_HFI_IPD_SWEEP_ENABLE
#error "M1_HFI_QKICK_SWEEP_ENABLE conflicts with M1_HFI_IPD_SWEEP_ENABLE"
#endif
#if M1_HFI_QKICK_SWEEP_ENABLE && M1_HFI_DELTA_SWEEP_ENABLE
#error "M1_HFI_QKICK_SWEEP_ENABLE conflicts with M1_HFI_DELTA_SWEEP_ENABLE"
#endif
#if M1_HFI_QKICK_SWEEP_ENABLE && M1_HFI_MOTION_BYPASS_ENABLE
#error "M1_HFI_QKICK_SWEEP_ENABLE conflicts with M1_HFI_MOTION_BYPASS_ENABLE"
#endif
#ifndef M1_HFI_QKICK_AFTER_LOCK_ENABLE
#define M1_HFI_QKICK_AFTER_LOCK_ENABLE  0
#endif
#ifndef M1_HFI_QKICK_BEFORE_HFI_ENABLE
#define M1_HFI_QKICK_BEFORE_HFI_ENABLE  0 /* 1：IDLE→先踢→。HFI RUN（非 AFTER_LOCK。*/
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
#ifndef M1_HFI_HAND_ID_OVERLAP_ENABLE
#define M1_HFI_HAND_ID_OVERLAP_ENABLE   0
#endif
#ifndef M1_HFI_HAND_OPEN_ID_ENABLE
#define M1_HFI_HAND_OPEN_ID_ENABLE      0
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
#ifndef M1_HFI_QKICK_PRE_GATE_ENABLE
#define M1_HFI_QKICK_PRE_GATE_ENABLE    0
#endif
#if M1_HFI_QKICK_PRE_GATE_ENABLE && !M1_HFI_QKICK_AFTER_LOCK_ENABLE
#error "M1_HFI_QKICK_PRE_GATE_ENABLE requires M1_HFI_QKICK_AFTER_LOCK_ENABLE=1"
#endif
#if M1_HFI_QKICK_PRE_GATE_ENABLE && M1_HFI_QKICK_BEFORE_HFI_ENABLE
#error "M1_HFI_QKICK_PRE_GATE_ENABLE conflicts with M1_HFI_QKICK_BEFORE_HFI_ENABLE"
#endif
#ifndef M1_HFI_QKICK_PRE_GATE_EPS_OK_RAD
#define M1_HFI_QKICK_PRE_GATE_EPS_OK_RAD      0.35f /* ~20°：像 ±。d */
#endif
#ifndef M1_HFI_QKICK_PRE_GATE_EPS_Q_RAD
#define M1_HFI_QKICK_PRE_GATE_EPS_Q_RAD       1.05f /* ~60°：atan2 。±90° */
#endif
#ifndef M1_HFI_QKICK_PRE_GATE_EPS_FALSE_MAX
#define M1_HFI_QKICK_PRE_GATE_EPS_FALSE_MAX   0.25f /* ε 自洽。x 。。A−B 假锁 */
#endif
#ifndef M1_HFI_QKICK_PRE_GATE_X_BAD
#define M1_HFI_QKICK_PRE_GATE_X_BAD           0.06f /* 1802 PRE x_lp。.038 */
#endif
#ifndef M1_HFI_QKICK_PRE_GATE_HOLD_N
#define M1_HFI_QKICK_PRE_GATE_HOLD_N          2000u /* 100 ms @20 kHz */
#endif
#ifndef M1_HFI_QKICK_PRE_GATE_FALSE_N
#define M1_HFI_QKICK_PRE_GATE_FALSE_N         2000u
#endif
#ifndef M1_HFI_QKICK_PRE_GATE_COOLDOWN_N
#define M1_HFI_QKICK_PRE_GATE_COOLDOWN_N      4000u /* 翻轴。200 ms */
#endif
#ifndef M1_HFI_QKICK_PRE_GATE_FLIP_MAX
#define M1_HFI_QKICK_PRE_GATE_FLIP_MAX        4u
#endif
#ifndef M1_HFI_QKICK_PRE_GATE_TIMEOUT_S
#define M1_HFI_QKICK_PRE_GATE_TIMEOUT_S       8.0f /* 仍假锁则拒踢 DONE */
#endif
#ifndef M1_HFI_QKICK_PRE_S
#define M1_HFI_QKICK_PRE_S              0.2f
#endif
#ifndef M1_HFI_QKICK_HAT0_N
#define M1_HFI_QKICK_HAT0_N             160u /* 8 ms：Iq 阶跃过后再采 θ̂0 */
#endif
#if M1_HFI_QKICK_AFTER_LOCK_ENABLE && !M1_HFI_MOTION_BYPASS_ENABLE
#error "M1_HFI_QKICK_AFTER_LOCK_ENABLE requires M1_HFI_MOTION_BYPASS_ENABLE=1"
#endif
#if M1_HFI_QKICK_AFTER_LOCK_ENABLE && M1_HFI_QKICK_SWEEP_ENABLE
#error "M1_HFI_QKICK_AFTER_LOCK_ENABLE conflicts with M1_HFI_QKICK_SWEEP_ENABLE"
#endif
#if M1_HFI_QKICK_AFTER_LOCK_ENABLE && M1_HFI_POLARITY_IPD_ENABLE
#error "M1_HFI_QKICK_AFTER_LOCK_ENABLE conflicts with M1_HFI_POLARITY_IPD_ENABLE"
#endif
#if M1_HFI_IQ_RAMP_ENABLE && M1_HFI_QKICK_AFTER_LOCK_ENABLE
#error "M1_HFI_IQ_RAMP_ENABLE conflicts with M1_HFI_QKICK_AFTER_LOCK_ENABLE"
#endif
#if M1_HFI_QKICK_BEFORE_HFI_ENABLE && !M1_HFI_QKICK_AFTER_LOCK_ENABLE
#error "M1_HFI_QKICK_BEFORE_HFI_ENABLE requires M1_HFI_QKICK_AFTER_LOCK_ENABLE=1"
#endif
#if M1_HFI_QKICK_BEFORE_HFI_ENABLE && M1_HFI_QKICK_CRAWL_ENABLE
#error "M1_HFI_QKICK_BEFORE_HFI_ENABLE conflicts with M1_HFI_QKICK_CRAWL_ENABLE"
#endif
#if M1_HFI_QKICK_BEFORE_HFI_ENABLE && M1_HFI_QKICK_START_ENABLE
#error "M1_HFI_QKICK_BEFORE_HFI_ENABLE conflicts with M1_HFI_QKICK_START_ENABLE"
#endif
#if M1_HFI_QKICK_BEFORE_HFI_ENABLE && M1_HFI_QKICK_SPEED_ENABLE
#error "M1_HFI_QKICK_BEFORE_HFI_ENABLE conflicts with M1_HFI_QKICK_SPEED_ENABLE"
#endif
#if M1_HFI_QKICK_THEN_HFI_ENABLE && !M1_HFI_QKICK_AFTER_LOCK_ENABLE
#error "M1_HFI_QKICK_THEN_HFI_ENABLE requires M1_HFI_QKICK_AFTER_LOCK_ENABLE=1"
#endif
#if M1_HFI_QKICK_THEN_HFI_ENABLE && M1_HFI_QKICK_BEFORE_HFI_ENABLE
#error "M1_HFI_QKICK_THEN_HFI_ENABLE conflicts with M1_HFI_QKICK_BEFORE_HFI_ENABLE"
#endif
#if M1_HFI_QKICK_THEN_HFI_ENABLE && M1_HFI_QKICK_CRAWL_ENABLE
#error "M1_HFI_QKICK_THEN_HFI_ENABLE conflicts with M1_HFI_QKICK_CRAWL_ENABLE"
#endif
#if M1_HFI_QKICK_THEN_HFI_ENABLE && M1_HFI_QKICK_START_ENABLE
#error "M1_HFI_QKICK_THEN_HFI_ENABLE conflicts with M1_HFI_QKICK_START_ENABLE"
#endif
#if M1_HFI_QKICK_THEN_HFI_ENABLE && M1_HFI_QKICK_SPEED_ENABLE
#error "M1_HFI_QKICK_THEN_HFI_ENABLE conflicts with M1_HFI_QKICK_SPEED_ENABLE"
#endif
#ifndef M1_HFI_DQ_IDENT_ENABLE
#define M1_HFI_DQ_IDENT_ENABLE          0
#endif
#if M1_HFI_DQ_IDENT_ENABLE && !M1_HFI_QKICK_AFTER_LOCK_ENABLE
#error "M1_HFI_DQ_IDENT_ENABLE requires M1_HFI_QKICK_AFTER_LOCK_ENABLE=1"
#endif
#if M1_HFI_DQ_IDENT_ENABLE && M1_HFI_DELTA_SWEEP_ENABLE
/* L2b：扫完最后一。enter_run，θ。沿用 enc+δ */
#endif
#if M1_HFI_DQ_IDENT_ENABLE && M1_HFI_QKICK_SWEEP_ENABLE
#error "M1_HFI_DQ_IDENT_ENABLE conflicts with M1_HFI_QKICK_SWEEP_ENABLE"
#endif
#if M1_HFI_DQ_IDENT_ENABLE && M1_HFI_POLARITY_IPD_ENABLE
#error "M1_HFI_DQ_IDENT_ENABLE conflicts with M1_HFI_POLARITY_IPD_ENABLE"
#endif
#if M1_HFI_DQ_IDENT_ENABLE && M1_HFI_QKICK_BRAKE_ENABLE
#error "M1_HFI_DQ_IDENT_ENABLE requires M1_HFI_QKICK_BRAKE_ENABLE=0"
#endif
#if M1_HFI_DQ_IDENT_ENABLE && M1_HFI_QKICK_SPEED_ENABLE
#error "M1_HFI_DQ_IDENT_ENABLE conflicts with M1_HFI_QKICK_SPEED_ENABLE (gated speed is internal)"
#endif
#if M1_HFI_DQ_IDENT_ENABLE && M1_HFI_QKICK_START_ENABLE
#error "M1_HFI_DQ_IDENT_ENABLE conflicts with M1_HFI_QKICK_START_ENABLE"
#endif
#if M1_HFI_DQ_IDENT_ENABLE && M1_HFI_QKICK_CRAWL_ENABLE
#error "M1_HFI_DQ_IDENT_ENABLE conflicts with M1_HFI_QKICK_CRAWL_ENABLE"
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
#if M1_HFI_IQ_RAMP_ENABLE && !M1_HFI_POLARITY_IPD_ENABLE
#error "M1_HFI_IQ_RAMP_ENABLE requires M1_HFI_POLARITY_IPD_ENABLE=1"
#endif
#ifndef M1_HFI_QKICK_FORCE_PI
#define M1_HFI_QKICK_FORCE_PI           0
#endif
#if (M1_HFI_QKICK_SWEEP_ENABLE || M1_HFI_QKICK_AFTER_LOCK_ENABLE)
#define M1_HFI_QKICK_ANY                1
#else
#define M1_HFI_QKICK_ANY                0
#endif
#ifndef M1_HFI_QKICK_HOLD_S
#define M1_HFI_QKICK_HOLD_S             8.0f
#endif
#ifndef M1_HFI_QKICK_CRAWL_ENABLE
#define M1_HFI_QKICK_CRAWL_ENABLE       0
#endif
#ifndef M1_HFI_QKICK_CRAWL_IQ_A
#define M1_HFI_QKICK_CRAWL_IQ_A         0.8f
#endif
#ifndef M1_HFI_QKICK_CRAWL_S
#define M1_HFI_QKICK_CRAWL_S            5.0f
#endif
/* Step3：CRAWL 内零。Iq 阶跃 0。A。→−A。，每。SEG_S；总时长仍。CRAWL_S */
#ifndef M1_HFI_QKICK_IQ_STEP_ENABLE
#define M1_HFI_QKICK_IQ_STEP_ENABLE     0
#endif
#ifndef M1_HFI_QKICK_IQ_STEP_SEG_S
#define M1_HFI_QKICK_IQ_STEP_SEG_S      2.0f
#endif
/* CRAWL：踢后慢。Iq。→A_MAX），。Bv、无速度。*/
#ifndef M1_HFI_QKICK_IQ_RAMP_ENABLE
#define M1_HFI_QKICK_IQ_RAMP_ENABLE     0
#endif
#ifndef M1_HFI_QKICK_IQ_RAMP_PRE_S
#define M1_HFI_QKICK_IQ_RAMP_PRE_S      2.0f
#endif
#ifndef M1_HFI_QKICK_IQ_RAMP_S
#define M1_HFI_QKICK_IQ_RAMP_S          12.0f
#endif
#ifndef M1_HFI_QKICK_IQ_RAMP_HOLD_S
#define M1_HFI_QKICK_IQ_RAMP_HOLD_S     8.0f
#endif
#ifndef M1_HFI_QKICK_IQ_RAMP_A_MAX
#define M1_HFI_QKICK_IQ_RAMP_A_MAX      2.0f
#endif
#ifndef M1_HFI_QKICK_IQ_LADDER_ENABLE
#define M1_HFI_QKICK_IQ_LADDER_ENABLE   0
#endif
#ifndef M1_HFI_QKICK_IQ_LADDER_N
#define M1_HFI_QKICK_IQ_LADDER_N        6
#endif
#ifndef M1_HFI_QKICK_IQ_LADDER_SEG_S
#define M1_HFI_QKICK_IQ_LADDER_SEG_S    3.0f
#endif
#ifndef M1_HFI_QKICK_IQ_LADDER_0
#define M1_HFI_QKICK_IQ_LADDER_0        0.50f
#endif
#ifndef M1_HFI_QKICK_IQ_LADDER_1
#define M1_HFI_QKICK_IQ_LADDER_1        0.70f
#endif
#ifndef M1_HFI_QKICK_IQ_LADDER_2
#define M1_HFI_QKICK_IQ_LADDER_2        0.90f
#endif
#ifndef M1_HFI_QKICK_IQ_LADDER_3
#define M1_HFI_QKICK_IQ_LADDER_3        1.10f
#endif
#ifndef M1_HFI_QKICK_IQ_LADDER_4
#define M1_HFI_QKICK_IQ_LADDER_4        1.30f
#endif
#ifndef M1_HFI_QKICK_IQ_LADDER_5
#define M1_HFI_QKICK_IQ_LADDER_5        1.50f
#endif
#ifndef M1_HFI_QKICK_IQ_LADDER_ARITH_ENABLE
#define M1_HFI_QKICK_IQ_LADDER_ARITH_ENABLE 0
#endif
#ifndef M1_HFI_QKICK_IQ_LADDER_A0
#define M1_HFI_QKICK_IQ_LADDER_A0       0.50f
#endif
#ifndef M1_HFI_QKICK_IQ_LADDER_DA
#define M1_HFI_QKICK_IQ_LADDER_DA       0.10f
#endif
#ifndef M1_HFI_QKICK_PARK_ENC
#define M1_HFI_QKICK_PARK_ENC           0
#endif
/* S3c1：仅 CRAWL 段力。Park=enc；踢/LOG 。θ̂ */
#ifndef M1_HFI_QKICK_CRAWL_PARK_ENC
#define M1_HFI_QKICK_CRAWL_PARK_ENC     0
#endif
#ifndef M1_HFI_QKICK_IQ_SLOW_ENABLE
#define M1_HFI_QKICK_IQ_SLOW_ENABLE     0
#endif
#ifndef M1_HFI_QKICK_IQ_SLOW_A0
#define M1_HFI_QKICK_IQ_SLOW_A0         0.50f
#endif
#ifndef M1_HFI_QKICK_IQ_SLOW_A1
#define M1_HFI_QKICK_IQ_SLOW_A1         0.70f
#endif
#ifndef M1_HFI_QKICK_IQ_SLOW_A2
#define M1_HFI_QKICK_IQ_SLOW_A2         0.90f
#endif
#ifndef M1_HFI_QKICK_IQ_SLOW_HOLD0_S
#define M1_HFI_QKICK_IQ_SLOW_HOLD0_S    3.0f
#endif
#ifndef M1_HFI_QKICK_IQ_SLOW_RAMP1_S
#define M1_HFI_QKICK_IQ_SLOW_RAMP1_S    8.0f
#endif
#ifndef M1_HFI_QKICK_IQ_SLOW_HOLD1_S
#define M1_HFI_QKICK_IQ_SLOW_HOLD1_S    3.0f
#endif
#ifndef M1_HFI_QKICK_IQ_SLOW_RAMP2_S
#define M1_HFI_QKICK_IQ_SLOW_RAMP2_S    4.0f
#endif
#ifndef M1_HFI_QKICK_IQ_SLOW_HOLD2_S
#define M1_HFI_QKICK_IQ_SLOW_HOLD2_S    3.0f
#endif
#ifndef M1_HFI_QKICK_POL_DECIDE_S
#define M1_HFI_QKICK_POL_DECIDE_S       1.5f
#endif
#ifndef M1_HFI_QKICK_POL_W_RPM
#define M1_HFI_QKICK_POL_W_RPM          8.0f
#endif
/* Step4：CRAWL 内短 I–f 离零（Park=θ_if，恒 Iq，ω_cmd 慢斜坡）；不开 M1_IF 全路。*/
#ifndef M1_HFI_QKICK_IF_ENABLE
#define M1_HFI_QKICK_IF_ENABLE          0
#endif
#ifndef M1_HFI_QKICK_IF_IQ_A
#define M1_HFI_QKICK_IF_IQ_A            0.8f
#endif
#ifndef M1_HFI_QKICK_IF_TARGET_RPM
#define M1_HFI_QKICK_IF_TARGET_RPM      40.0f
#endif
#ifndef M1_HFI_QKICK_IF_PRE_S
#define M1_HFI_QKICK_IF_PRE_S           1.0f
#endif
#ifndef M1_HFI_QKICK_IF_RAMP_S
#define M1_HFI_QKICK_IF_RAMP_S          4.0f
#endif
#ifndef M1_HFI_QKICK_IF_HOLD_S
#define M1_HFI_QKICK_IF_HOLD_S          2.0f
#endif
#ifndef M1_HFI_QKICK_IF_POST_S
#define M1_HFI_QKICK_IF_POST_S          3.0f
#endif
#ifndef M1_HFI_QKICK_IF_LADDER_ENABLE
#define M1_HFI_QKICK_IF_LADDER_ENABLE   0
#endif
#ifndef M1_HFI_QKICK_IF_LADDER_N
#define M1_HFI_QKICK_IF_LADDER_N        6
#endif
#ifndef M1_HFI_QKICK_IF_LADDER_RAMP_S
#define M1_HFI_QKICK_IF_LADDER_RAMP_S   0.80f
#endif
#ifndef M1_HFI_QKICK_IF_LADDER_HOLD_S
#define M1_HFI_QKICK_IF_LADDER_HOLD_S   2.50f
#endif
#ifndef M1_HFI_QKICK_IF_LAD_IQ0
#define M1_HFI_QKICK_IF_LAD_IQ0         M1_HFI_QKICK_IF_IQ_A
#endif
#ifndef M1_HFI_QKICK_IF_LAD_RPM0
#define M1_HFI_QKICK_IF_LAD_RPM0        5.0f
#endif
#ifndef M1_HFI_QKICK_IF_LAD_IQ1
#define M1_HFI_QKICK_IF_LAD_IQ1         M1_HFI_QKICK_IF_IQ_A
#endif
#ifndef M1_HFI_QKICK_IF_LAD_RPM1
#define M1_HFI_QKICK_IF_LAD_RPM1        18.0f
#endif
#ifndef M1_HFI_QKICK_IF_LAD_IQ2
#define M1_HFI_QKICK_IF_LAD_IQ2         0.55f
#endif
#ifndef M1_HFI_QKICK_IF_LAD_RPM2
#define M1_HFI_QKICK_IF_LAD_RPM2        5.0f
#endif
#ifndef M1_HFI_QKICK_IF_LAD_IQ3
#define M1_HFI_QKICK_IF_LAD_IQ3         0.55f
#endif
#ifndef M1_HFI_QKICK_IF_LAD_RPM3
#define M1_HFI_QKICK_IF_LAD_RPM3        18.0f
#endif
#ifndef M1_HFI_QKICK_IF_LAD_IQ4
#define M1_HFI_QKICK_IF_LAD_IQ4         0.60f
#endif
#ifndef M1_HFI_QKICK_IF_LAD_RPM4
#define M1_HFI_QKICK_IF_LAD_RPM4        5.0f
#endif
#ifndef M1_HFI_QKICK_IF_LAD_IQ5
#define M1_HFI_QKICK_IF_LAD_IQ5         0.60f
#endif
#ifndef M1_HFI_QKICK_IF_LAD_RPM5
#define M1_HFI_QKICK_IF_LAD_RPM5        18.0f
#endif
#ifndef M1_HFI_QKICK_SPEED_ENABLE
#define M1_HFI_QKICK_SPEED_ENABLE       0
#endif
/* 踢后正式起动：HOLD→CAPTURE(固定Iq+电子阻尼)→HANDOVER→RUN */
#ifndef M1_HFI_QKICK_START_ENABLE
#define M1_HFI_QKICK_START_ENABLE       0
#endif
#ifndef M1_HFI_QKICK_START_HOLD_S
#define M1_HFI_QKICK_START_HOLD_S       1.0f
#endif
/* CAPTURE：Iq = Iq_cmd 。Bv·ω（空轴用电子阻尼代机。B；非角误。PI。*/
#ifndef M1_HFI_QKICK_START_CAP_IQ_A
#define M1_HFI_QKICK_START_CAP_IQ_A     1.2f
#endif
#ifndef M1_HFI_QKICK_START_CAP_IQ_KICK_A
#define M1_HFI_QKICK_START_CAP_IQ_KICK_A M1_HFI_QKICK_START_CAP_IQ_A
#endif
#ifndef M1_HFI_QKICK_START_CAP_IQ_KICK_S
#define M1_HFI_QKICK_START_CAP_IQ_KICK_S 0.0f /* 0=全程。CAP_IQ / HOLD */
#endif
#ifndef M1_HFI_QKICK_START_CAP_IQ_HOLD_A
#define M1_HFI_QKICK_START_CAP_IQ_HOLD_A M1_HFI_QKICK_START_CAP_IQ_A
#endif
#ifndef M1_HFI_QKICK_START_BV_A_RPM
#define M1_HFI_QKICK_START_BV_A_RPM     0.020f /* ~0.8A @40rpm 。净力矩。0.4A */
#endif
#ifndef M1_HFI_QKICK_START_BV_USE_ENC
#define M1_HFI_QKICK_START_BV_USE_ENC   0 /* 1=台架。ω_enc。=产品。ω̂ */
#endif
#ifndef M1_HFI_QKICK_START_CAP_RPM
#define M1_HFI_QKICK_START_CAP_RPM      40.0f
#endif
#ifndef M1_HFI_QKICK_START_CAP_RAMP_S
#define M1_HFI_QKICK_START_CAP_RAMP_S   6.0f
#endif
#ifndef M1_HFI_QKICK_START_CAP_HOLD_S
#define M1_HFI_QKICK_START_CAP_HOLD_S   2.0f
#endif
#ifndef M1_HFI_QKICK_START_EXIT_RPM
#define M1_HFI_QKICK_START_EXIT_RPM     12.0f /* 真速不够禁止交。*/
#endif
#ifndef M1_HFI_QKICK_START_EXIT_N
#define M1_HFI_QKICK_START_EXIT_N       2000u /* 100 ms @20kHz */
#endif
#ifndef M1_HFI_QKICK_START_HANDOVER_S
#define M1_HFI_QKICK_START_HANDOVER_S   0.4f
#endif
#ifndef M1_HFI_QKICK_START_COOL_S
#define M1_HFI_QKICK_START_COOL_S       0.8f
#endif
/* 0=CAPTURE 结束。DONE，不开无感速环。551：交接即反向飞车。*/
#ifndef M1_HFI_QKICK_START_ARM_RUN
#define M1_HFI_QKICK_START_ARM_RUN      0
#endif
#ifndef M1_HFI_QKICK_START_FLIP_RPM
#define M1_HFI_QKICK_START_FLIP_RPM     8.0f
#endif
#ifndef M1_HFI_QKICK_START_FLIP_N
#define M1_HFI_QKICK_START_FLIP_N       4000u /* 200 ms @20kHz */
#endif
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
#ifndef M1_HFI_QKICK_SPEED_PRE_S
#define M1_HFI_QKICK_SPEED_PRE_S        1.0f
#endif
#if M1_HFI_QKICK_SPEED_ENABLE && !M1_HFI_QKICK_AFTER_LOCK_ENABLE
#error "M1_HFI_QKICK_SPEED_ENABLE requires M1_HFI_QKICK_AFTER_LOCK_ENABLE=1"
#endif
#if M1_HFI_QKICK_START_ENABLE && !M1_HFI_QKICK_AFTER_LOCK_ENABLE
#error "M1_HFI_QKICK_START_ENABLE requires M1_HFI_QKICK_AFTER_LOCK_ENABLE=1"
#endif
#if M1_HFI_QKICK_START_ENABLE && !M1_HFI_QKICK_CRAWL_ENABLE
#error "M1_HFI_QKICK_START_ENABLE requires M1_HFI_QKICK_CRAWL_ENABLE=1"
#endif
#if M1_HFI_QKICK_START_ENABLE && !M1_HFI_QKICK_SPEED_ENABLE
#error "M1_HFI_QKICK_START_ENABLE requires M1_HFI_QKICK_SPEED_ENABLE=1 (final RUN)"
#endif
#if M1_HFI_QKICK_SPEED_ENABLE && M1_HFI_QKICK_CRAWL_ENABLE && !M1_HFI_QKICK_START_ENABLE
#error "M1_HFI_QKICK_SPEED_ENABLE conflicts with M1_HFI_QKICK_CRAWL_ENABLE"
#endif
#if M1_HFI_SENSED_CAL_ENABLE && M1_HFI_QKICK_START_ENABLE
#error "M1_HFI_SENSED_CAL_ENABLE conflicts with M1_HFI_QKICK_START_ENABLE"
#endif
#if M1_HFI_SENSED_CAL_ENABLE && M1_HFI_QKICK_CRAWL_ENABLE
#error "M1_HFI_SENSED_CAL_ENABLE requires M1_HFI_QKICK_CRAWL_ENABLE=0"
#endif
#if M1_HFI_SENSED_CAL_ENABLE && !M1_HFI_QKICK_SPEED_ENABLE
#error "M1_HFI_SENSED_CAL_ENABLE requires M1_HFI_QKICK_SPEED_ENABLE=1"
#endif
#if M1_HFI_SENSED_CAL_ENABLE && !M1_HFI_QKICK_AFTER_LOCK_ENABLE
#error "M1_HFI_SENSED_CAL_ENABLE requires M1_HFI_QKICK_AFTER_LOCK_ENABLE=1"
#endif
#if M1_HFI_QKICK_IQ_STEP_ENABLE && !M1_HFI_QKICK_CRAWL_ENABLE
#error "M1_HFI_QKICK_IQ_STEP_ENABLE requires M1_HFI_QKICK_CRAWL_ENABLE=1"
#endif
#if M1_HFI_QKICK_IF_ENABLE && !M1_HFI_QKICK_CRAWL_ENABLE
#error "M1_HFI_QKICK_IF_ENABLE requires M1_HFI_QKICK_CRAWL_ENABLE=1"
#endif
#if M1_HFI_QKICK_IF_ENABLE && M1_HFI_QKICK_IQ_STEP_ENABLE
#error "M1_HFI_QKICK_IF_ENABLE conflicts with M1_HFI_QKICK_IQ_STEP_ENABLE"
#endif
#if M1_HFI_QKICK_IF_LADDER_ENABLE && !M1_HFI_QKICK_IF_ENABLE
#error "M1_HFI_QKICK_IF_LADDER_ENABLE requires M1_HFI_QKICK_IF_ENABLE=1"
#endif
#if M1_HFI_QKICK_IF_LADDER_ENABLE && ((M1_HFI_QKICK_IF_LADDER_N < 1) || (M1_HFI_QKICK_IF_LADDER_N > 6))
#error "M1_HFI_QKICK_IF_LADDER_N must be 1..6"
#endif
#if M1_HFI_QKICK_IQ_RAMP_ENABLE && !M1_HFI_QKICK_CRAWL_ENABLE
#error "M1_HFI_QKICK_IQ_RAMP_ENABLE requires M1_HFI_QKICK_CRAWL_ENABLE=1"
#endif
#if M1_HFI_QKICK_IQ_RAMP_ENABLE && (M1_HFI_QKICK_IF_ENABLE || M1_HFI_QKICK_IQ_STEP_ENABLE || M1_HFI_QKICK_START_ENABLE)
#error "M1_HFI_QKICK_IQ_RAMP_ENABLE conflicts with IF/IQ_STEP/START"
#endif
#if M1_HFI_QKICK_IQ_LADDER_ENABLE && !M1_HFI_QKICK_CRAWL_ENABLE
#error "M1_HFI_QKICK_IQ_LADDER_ENABLE requires M1_HFI_QKICK_CRAWL_ENABLE=1"
#endif
#if M1_HFI_QKICK_IQ_LADDER_ENABLE && !M1_HFI_QKICK_AFTER_LOCK_ENABLE
#error "M1_HFI_QKICK_IQ_LADDER_ENABLE requires M1_HFI_QKICK_AFTER_LOCK_ENABLE=1"
#endif
#if M1_HFI_QKICK_IQ_LADDER_ENABLE && (M1_HFI_QKICK_IF_ENABLE || M1_HFI_QKICK_IQ_STEP_ENABLE || M1_HFI_QKICK_START_ENABLE || M1_HFI_QKICK_IQ_RAMP_ENABLE)
#error "M1_HFI_QKICK_IQ_LADDER_ENABLE conflicts with IF/IQ_STEP/START/RAMP"
#endif
#if M1_HFI_QKICK_IQ_SLOW_ENABLE && !M1_HFI_QKICK_CRAWL_ENABLE
#error "M1_HFI_QKICK_IQ_SLOW_ENABLE requires M1_HFI_QKICK_CRAWL_ENABLE=1"
#endif
#if M1_HFI_QKICK_IQ_SLOW_ENABLE && !M1_HFI_QKICK_AFTER_LOCK_ENABLE
#error "M1_HFI_QKICK_IQ_SLOW_ENABLE requires M1_HFI_QKICK_AFTER_LOCK_ENABLE=1"
#endif
#if M1_HFI_QKICK_IQ_SLOW_ENABLE && (M1_HFI_QKICK_IF_ENABLE || M1_HFI_QKICK_IQ_STEP_ENABLE || M1_HFI_QKICK_START_ENABLE || M1_HFI_QKICK_IQ_RAMP_ENABLE || M1_HFI_QKICK_IQ_LADDER_ENABLE)
#error "M1_HFI_QKICK_IQ_SLOW_ENABLE conflicts with IF/IQ_STEP/START/RAMP/LADDER"
#endif
#if M1_HFI_PLL_HOLD_ENABLE && !M1_HFI_QKICK_AFTER_LOCK_ENABLE
#error "M1_HFI_PLL_HOLD_ENABLE requires M1_HFI_QKICK_AFTER_LOCK_ENABLE=1"
#endif
#if M1_HFI_QKICK_START_ENABLE && (M1_HFI_QKICK_IF_ENABLE || M1_HFI_QKICK_IQ_STEP_ENABLE)
#error "M1_HFI_QKICK_START_ENABLE conflicts with IF/IQ_STEP"
#endif
#ifndef M1_HFI_QKICK_MOVE_UD_V
#define M1_HFI_QKICK_MOVE_UD_V          2.0f
#endif
#ifndef M1_HFI_QKICK_MOVE_N
#define M1_HFI_QKICK_MOVE_N             6000u /* 300 ms @20kHz */
#endif
#ifndef M1_HFI_QKICK_POS_N
#define M1_HFI_QKICK_POS_N              12u
#endif
#ifndef M1_HFI_QKICK_ROUNDS
#define M1_HFI_QKICK_ROUNDS             2u
#endif
#ifndef M1_HFI_QKICK_LOG_S
#define M1_HFI_QKICK_LOG_S              0.25f
#endif
#ifndef M1_HFI_QKICK_IQ_A
#define M1_HFI_QKICK_IQ_A               1.0f
#endif
#ifndef M1_HFI_QKICK_KICK_N
#define M1_HFI_QKICK_KICK_N             400u /* 20 ms @20kHz */
#endif
#ifndef M1_HFI_QKICK_BRAKE_ENABLE
#define M1_HFI_QKICK_BRAKE_ENABLE       1
#endif
#ifndef M1_HFI_QKICK_BRAKE_N
#define M1_HFI_QKICK_BRAKE_N            400u /* 20 ms */
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
#ifndef M1_HFI_IPD_MOVE_UD_V
#define M1_HFI_IPD_MOVE_UD_V            2.0f
#endif
#ifndef M1_HFI_IPD_MOVE_N
#define M1_HFI_IPD_MOVE_N               6000u /* 300 ms @20kHz Ud 摆位 */
#endif
#ifndef M1_HFI_IPD_POS_N
#define M1_HFI_IPD_POS_N                12u /* 。30° 一。*/
#endif
#ifndef M1_HFI_IPD_ROUNDS
#define M1_HFI_IPD_ROUNDS               3u
#endif
#ifndef M1_HFI_IPD_LOG_S
#define M1_HFI_IPD_LOG_S                0.25f /* 每轮结果保持，供 CSV 切片 */
#endif
/* 脉冲电压表：一次实验扫多档，LOG 。ch5=该档 Ud [V] */
#ifndef M1_HFI_IPD_UD_TAB_N
#define M1_HFI_IPD_UD_TAB_N             4u
#endif
#ifndef M1_HFI_IPD_UD_TAB_0
#define M1_HFI_IPD_UD_TAB_0             0.4f
#endif
#ifndef M1_HFI_IPD_UD_TAB_1
#define M1_HFI_IPD_UD_TAB_1             0.5f
#endif
#ifndef M1_HFI_IPD_UD_TAB_2
#define M1_HFI_IPD_UD_TAB_2             0.6f
#endif
#ifndef M1_HFI_IPD_UD_TAB_3
#define M1_HFI_IPD_UD_TAB_3             0.8f
#endif

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
#if (M1_HFI_GATE == 51) || (M1_HFI_GATE == 52)
/* GATE51/52 共用阶梯状态机；表内容。GATE 分开 */
#if M1_HFI_GATE == 52
/* S2j。00。00 / +50；VH 恒定（由 profile M1_HFI_VH_V。*/
static const float s_run_rpm_seq[] = {
    100.0f, 150.0f, 200.0f, 250.0f, 300.0f, 350.0f, 400.0f,
    450.0f, 500.0f, 550.0f, 600.0f, 650.0f, 700.0f, 750.0f,
    800.0f,
};
#else
/* S2i。00。00/+100，之。+50。000；VH 三档 RO，仅换档。*/
#ifndef M1_HFI_VH_LO
#define M1_HFI_VH_LO                    0.40f
#endif
#ifndef M1_HFI_VH_MID
#define M1_HFI_VH_MID                   0.50f
#endif
#ifndef M1_HFI_VH_HI
#define M1_HFI_VH_HI                    0.60f
#endif
static const float s_run_rpm_seq[] = {
    100.0f, 200.0f, 300.0f, 400.0f,
    450.0f, 500.0f, 550.0f, 600.0f, 650.0f,
    700.0f, 750.0f, 800.0f, 850.0f, 900.0f,
    950.0f, 1000.0f,
};
static const float s_run_vh_seq[] = {
    M1_HFI_VH_LO, M1_HFI_VH_LO, M1_HFI_VH_LO,
    M1_HFI_VH_MID, M1_HFI_VH_MID, M1_HFI_VH_MID, M1_HFI_VH_MID,
    M1_HFI_VH_HI, M1_HFI_VH_HI, M1_HFI_VH_HI, M1_HFI_VH_HI,
    M1_HFI_VH_HI, M1_HFI_VH_HI, M1_HFI_VH_HI, M1_HFI_VH_HI,
    M1_HFI_VH_HI,
};
typedef char hfi51_vh_len_ok[
    (sizeof(s_run_vh_seq) == sizeof(s_run_rpm_seq)) ? 1 : -1];
#endif
#define HFI_RUN_LADDER_PHASES \
    ((uint8_t)(sizeof(s_run_rpm_seq) / sizeof(s_run_rpm_seq[0])))
static uint8_t s_run_phase;
static uint32_t s_run_phase_tick;
static uint32_t s_run_dwell_ticks;
static uint8_t s_run_ladder_done;
#if M1_HFI_SPD_RAMP_ENABLE && (M1_HFI_GATE == 52)
static uint8_t s_spd_ramp_done;
static uint32_t s_spd_ramp_tick;
static uint32_t s_spd_hold0_ticks;
static uint32_t s_spd_ramp_ticks;
#endif
#endif
#if M1_HFI_IQ_PULL_ENABLE
static uint8_t s_iq_pull_done; /* 1=固定电流段结束，允许开 HFI 速度。*/
#if M1_HFI_IQ_PULL_ARM_ENABLE
static float s_iq_pull_above_t; /* |ω̂| 越过门槛。stage 时刻。0 表示未越。*/
#endif
#if M1_HFI_IQ_AUTH_ENABLE
static float s_iq_pull_ok_t; /* qual_ok 已持续时长；掉线清零 */
#endif
#endif
static float s_theta_enc;
static float s_ud_inj;
static float s_uq_inj;
static float s_id_inj_prev;
static float s_iq_inj_prev;
static uint8_t s_inj_prev_valid;
#if M1_HFI_DEMOD_HP_ENABLE
static float s_id_inj_slow;
static float s_iq_inj_slow;
#endif
#if M1_HFI_DEMOD_AB_ENABLE || M1_HFI_DEMOD_AB_MID_ENABLE || M1_HFI_DEMOD_INJ_AXIS
static float s_ia_now;
static float s_ib_now;
#endif
#if ((M1_HFI_GATE == 117) || (M1_HFI_GATE == 118) || (M1_HFI_GATE == 119) || (M1_HFI_GATE == 120) || (M1_HFI_GATE == 121) || (M1_HFI_GATE == 122) || (M1_HFI_GATE == 123) || (M1_HFI_GATE == 124) || (M1_HFI_GATE == 125) || (M1_HFI_GATE == 126) || (M1_HFI_GATE == 127) || (M1_HFI_GATE == 128) || (M1_HFI_GATE == 129) || (M1_HFI_GATE == 130) || (M1_HFI_GATE == 131) || (M1_HFI_GATE == 138) || (M1_HFI_GATE == 132) || (M1_HFI_GATE == 133) || (M1_HFI_GATE == 134) || (M1_HFI_GATE == 135) || (M1_HFI_GATE == 136) || (M1_HFI_GATE == 137) || (M1_HFI_GATE == 139) || (M1_HFI_GATE == 140) || (M1_HFI_GATE == 141)) && M1_HFI_DEMOD_INJ_AXIS
/* 上一拍注入轴。两拍 iαβ 都投到这组正余弦上，PLL 更新之前写入。 */
static float s_pair_c;
static float s_pair_s;
static float s_ia_pair;
static float s_ib_pair;
static float s_pair_di_q;
static uint8_t s_pair_ready;
#endif
#if (M1_HFI_GATE == 118) || (M1_HFI_GATE == 119) || (M1_HFI_GATE == 120) || (M1_HFI_GATE == 121) || (M1_HFI_GATE == 122) || (M1_HFI_GATE == 123) || (M1_HFI_GATE == 124) || (M1_HFI_GATE == 125) || (M1_HFI_GATE == 126) || (M1_HFI_GATE == 127) || (M1_HFI_GATE == 128) || (M1_HFI_GATE == 129) || (M1_HFI_GATE == 130) || (M1_HFI_GATE == 131) || (M1_HFI_GATE == 138) || (M1_HFI_GATE == 132) || (M1_HFI_GATE == 133) || (M1_HFI_GATE == 134) || (M1_HFI_GATE == 135) || (M1_HFI_GATE == 136) || (M1_HFI_GATE == 137) || (M1_HFI_GATE == 139) || (M1_HFI_GATE == 140) || (M1_HFI_GATE == 141)
/* θ̂ 自身转速的低通。118 用它做积分上限，119 的积分直接等于它。编码器不参与。 */
static float s_speed_est;
#endif
#if M1_HFI_DEMOD_AB_ENABLE || M1_HFI_DEMOD_AB_MID_ENABLE
static float s_ia_prev;
static float s_ib_prev;
#endif
#if M1_HFI_DEMOD_AB_MID_ENABLE
static float s_ia_older;
static float s_ib_older;
static float s_th_ab_prev;
static uint8_t s_ab_mid_n;
#endif
static float s_sign;
static float s_eps_lp;
static float s_x_lp;     /* x_raw 向量 LPF；初。A_cmd，避。atan2(0,-A) */
#if M1_HFI_IQ_AUTH_ENABLE
static float s_iq_auth_abs;   /* 当前 |Iq| 天花。*/
static uint16_t s_iq_auth_good_n;
static uint16_t s_iq_auth_bad_n;
static uint8_t s_iq_auth_ok;  /* 1=质量过线，目。HI */
static uint8_t s_iq_auth_hold; /* 1：注入故意关掉，天花板留。HI */
#endif
#if M1_HFI_IQ_AUTH_FEED_ENABLE
static float s_iq_feed_ok_t;  /* qual_ok 已持续时。[s] */
static float s_iq_feed_cmd;   /* 当前馈流指令幅。*/
static uint8_t s_iq_feed_mode; /* 0=A 延时阶跃。=B 斜坡 */
static uint8_t s_iq_feed_ab_n; /* COMPARE_AB：已完成 enter_run 次数 */
#endif
#if M1_HFI_SPD_CLOSE_ENABLE
static uint8_t s_spd_close_on;
static float s_spd_close_rpm;
static float s_spd_close_target;
static float s_spd_close_mark_t; /* <0：角。转速窗口未开。*/
#endif
static float s_y_lp;
static float s_pll_int;
static float s_pll_eps_dead; /* 运行时死区；扫档时按表切。*/
#if M1_HFI_AXIS_SEL_ENABLE
static float s_eps_d_lp;     /* 正交 di_d 解调 */
static uint16_t s_axis_lock_n;
static uint16_t s_axis_good_n; /* eps_d 持续为真 d 符号 */
static uint16_t s_axis_cd;
static uint16_t s_axis_flip_n;
static uint8_t s_axis_ok; /* 1=已确认真 d，放开 Iq */
#endif
#if M1_HFI_LQ_WELL_FLIP_ENABLE
static uint16_t s_lq_well_n;
static uint8_t s_lq_well_flip_n; /* 本 RUN 已翻次数；1 即冻结 */
#endif
#if M1_HFI_PLL_HOLD_ENABLE
static uint8_t s_pll_hold;       /* 1=HOLD θ̂。=TRACK */
#if M1_HFI_PLL_RETRACK_USE_ENC
static float s_w_mot_rpm;        /* 仅对照门用；本轮 enc 不进交接 */
static float s_enc_mot_prev;
static uint8_t s_enc_mot_valid;
#endif
static uint16_t s_retrack_n;
#endif
#if M1_HFI_PLL_EPS_DEAD_SWEEP_ENABLE && M1_HFI_QKICK_START_ENABLE
static uint8_t s_eps_dead_i;
#endif
#if M1_HFI_BIAS_CAL_ENABLE
static float s_theta_bias; /* 学习/冻结。δ=wrap(θ̂−θ_enc)；冻结后仅存。*/
static uint8_t s_bias_frozen;
#endif
static uint16_t s_polarity_cnt;
static float s_omega_ff_el;
static float s_omega_el;
static float s_omega_trim_el;
static float s_sh_int; /* 影子：无 ω_ff，跟主环 θ̂（凸极），不。Park */
static float s_sh_w;
static float s_sh_th;
static uint8_t s_sh_seed;
#if M1_HFI_POLARITY_IPD_ENABLE
static uint8_t s_ipd_phase;
static uint16_t s_ipd_cnt;
static uint16_t s_ipd_align_hold;
static float s_ipd_peak0; /* P0：|id_h| 峰（SETTLE 后从近零起） */
static float s_ipd_peak1;
static float s_ipd_th0;
static float s_ipd_iabs; /* 最。|i_dq|，供 settle 。*/
static uint8_t s_ipd_ov;
static float s_ipd_ud;
static float s_ipd_uq;
#if M1_HFI_IPD_SWEEP_ENABLE
static uint16_t s_ipd_pos_i;
static uint16_t s_ipd_round;
static uint16_t s_ipd_move_cnt;
static uint16_t s_ipd_ud_i;     /* 脉冲电压表下。*/
static float s_ipd_pulse_ud;    /* 本格脉冲 Ud [V] */
#endif
#endif
#if M1_HFI_QKICK_ANY
enum {
    HFI_QK_SEED = 0,
    HFI_QK_KICK = 1,
    HFI_QK_BRAKE = 2
};
#if M1_HFI_QKICK_SWEEP_ENABLE
static uint16_t s_qk_pos_i;
static uint16_t s_qk_round;
static uint16_t s_qk_move_cnt;
#endif
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
#if M1_HFI_QKICK_AFTER_LOCK_ENABLE
static uint8_t s_qk_done;       /* 本开机只踢一。*/
#if M1_HFI_QKICK_PRE_GATE_ENABLE
static uint8_t s_qk_pre_ok;     /* 1=PRE 。±d，允许踢 */
static uint16_t s_qk_pre_good_n;
static uint16_t s_qk_pre_false_n;
static uint16_t s_qk_pre_cd;
static uint16_t s_qk_pre_flip_n;
#endif
static uint8_t s_qk_pol_done;   /* CRAWL 。ω̂ 极性已。*/
#if M1_HFI_DQ_IDENT_ENABLE
enum {
    HFI_DQ_BLANK = 0,
    HFI_DQ_WAIT_EPS,
    HFI_DQ_PULSE_D,
    HFI_DQ_GAP,
    HFI_DQ_PULSE_Q,
    HFI_DQ_SENTINEL,
    HFI_DQ_SPEED,
    HFI_DQ_FAIL
};
static uint8_t s_dq_step;
static uint8_t s_dq_axis; /* 0=Id(θ̂) 1=Iq(θ̂+90) */
static float s_park_off;
static float s_dq_dth_d;
static float s_dq_dth_q;
static float s_dq_enc0;
static float s_dq_med_buf[32];
static uint16_t s_dq_med_n;
static float s_dq_gate_enc0;
static float s_dq_good_t;
static uint8_t s_dq_enc0_ok;
#endif
#if M1_HFI_SENSED_CAL_ENABLE
static uint8_t s_sensed_cal_loop; /* 有感标定圈号 0..LOOPS-1 */
#endif
#endif
#if M1_HFI_QKICK_IF_ENABLE
static uint8_t s_if_active;     /* CRAWL 。I–f：Park=θ_if */
static float s_if_rpm_cmd;      /* I–f 指令机械转。[rpm] */
#endif
#if M1_HFI_QKICK_START_ENABLE
enum {
    HFI_START_HOLD = 0,
    HFI_START_CAPTURE = 1,
    HFI_START_HANDOVER = 2
};
static uint8_t s_start_phase;
static uint8_t s_start_run_armed; /* 1=已交接，允许速度。*/
static float s_start_rpm_cmd;
static float s_start_cool_s;
static float s_start_ang_int;
static float s_start_enc_prev;
static uint8_t s_start_enc_prev_valid;
static float s_start_wenc_rpm;
static uint16_t s_start_exit_n;
static float s_start_iq_sign;
static uint8_t s_start_flip_done;
static uint16_t s_start_flip_n;
static uint16_t s_gate_light_n;
static uint16_t s_gate_heavy_n;
static uint16_t s_gate_fake_n;
#endif
#endif
#if M1_HFI_DELTA_SWEEP_ENABLE
static int16_t s_delta_i;
static float s_delta_rad;
static uint8_t s_a0_pos; /* 0..2，Ud 摆到 0/60/120° el */
#endif

#if M1_HFI_PLL_VESC_ERR_ENABLE
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
#endif

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
#if M1_HFI_PARK_ENABLE
#if M1_HFI_DELTA_SWEEP_ENABLE
    /* 。δ：Park=θ̂=enc+δ。关。:1365 翻号，并与包 3 RUN 同帧。*/
    if (s_stage == HFI_STAGE_MEAS) {
        return 1u;
    }
#endif
#if M1_HFI_SENSED_CAL_ENABLE
    /*
     * 有感标定：力。Park=enc；极性踢 MEAS/LOG 仍用 θ̂。
     * HFI PLL 继续估角，不进力矩环。
     */
    if ((s_stage == HFI_STAGE_MEAS) || (s_stage == HFI_STAGE_LOG)) {
        return 1u;
    }
    return 0u;
#endif
#if M1_HFI_POLARITY_IPD_ENABLE
    /* IPD ALIGN/脉冲：Park=θ̂，不。enc */
    if (s_stage == HFI_STAGE_MOVE) {
        return 1u;
    }
#endif
#if M1_HFI_QKICK_AFTER_LOCK_ENABLE
#if M1_HFI_QKICK_PARK_ENC
    return 0u; /* 有感：力。Park=enc，HFI 只估对照 */
#endif
#if M1_HFI_QKICK_START_ENABLE
    if (s_stage == HFI_STAGE_CRAWL) {
        /* 角度闭环全程 Park=θ̂（含 CAPTURE。*/
        return 1u;
    }
    if ((s_stage == HFI_STAGE_RUN) && (s_qk_done != 0u) &&
        (s_start_run_armed != 0u)) {
        return 1u;
    }
#endif
    if ((s_stage == HFI_STAGE_MEAS) || (s_stage == HFI_STAGE_LOG)) {
        return 1u;
    }
    if (s_stage == HFI_STAGE_CRAWL) {
#if M1_HFI_QKICK_CRAWL_PARK_ENC
        return 0u; /* S3c1：爬段力。enc，HFI 旁路估角 */
#endif
#if M1_HFI_QKICK_IF_ENABLE
        if (s_if_active != 0u) {
            return 0u;
        }
#endif
        return 1u;
    }
    if (s_stage == HFI_STAGE_RUN) {
        return 1u; /* 全程 θ̂，不。enc 。*/
    }
#if M1_HFI_QKICK_SPEED_ENABLE
    if ((s_stage == HFI_STAGE_RUN) && (s_qk_done != 0u)) {
#if M1_HFI_QKICK_START_ENABLE
        return (s_start_run_armed != 0u) ? 1u : 0u;
#else
        return 1u;
#endif
    }
#endif
#endif
#if M1_HFI_LOCK_ENABLE
    return ((s_stage == HFI_STAGE_RUN) && (s_lock == HFI_LOCK_LOCKED)) ? 1u : 0u;
#else
    return (s_stage == HFI_STAGE_RUN) ? 1u : 0u;
#endif
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
#if M1_HFI_QKICK_IF_ENABLE
    if ((s_stage == HFI_STAGE_CRAWL) && (s_if_active != 0u)) {
        return s_theta_cmd; /* θ_if */
    }
#endif
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
#if M1_HFI_LOCK_ENABLE
    float ae = s_theta_err;

    if (ae < 0.0f) {
        ae = -ae;
    }
#if M1_HFI_QKICK_AFTER_LOCK_ENABLE
    /* 2θ 锁门：θ。。enc 。0 。π 都算对准 */
    if (ae > (0.5f * (float)M_PI)) {
        ae = (float)M_PI - ae;
    }
#endif

    if ((s_stage != HFI_STAGE_RUN) && (s_stage != HFI_STAGE_CRAWL)) {
        s_lock = HFI_LOCK_CAPTURE;
        s_lock_cnt = 0u;
        return;
    }

    if (s_lock == HFI_LOCK_LOCKED) {
        if (ae > M1_HFI_UNLOCK_ERR_RAD) {
            s_lock_cnt++;
            if (s_lock_cnt >= M1_HFI_UNLOCK_HOLD_N) {
                s_lock = HFI_LOCK_FAULT;
                s_lock_cnt = 0u;
                /*
                 * 未武。RUN：失锁必清积分，。CRAWL 假速自持。
                 * 武装后保。∫（速度环仍。ω̂）。
                 */
#if M1_HFI_QKICK_START_ENABLE
                if (s_start_run_armed == 0u) {
                    s_pll_int = 0.0f;
                }
#elif M1_HFI_QKICK_SPEED_ENABLE
                if (s_qk_done == 0u) {
                    s_pll_int = 0.0f;
                }
#else
                s_pll_int = 0.0f;
#endif
            }
        } else {
            s_lock_cnt = 0u;
        }
    } else {
        /* CAPTURE / FAULT：小误差持续则进 LOCKED */
        if (ae < M1_HFI_LOCK_ERR_RAD) {
            s_lock_cnt++;
            if (s_lock_cnt >= M1_HFI_LOCK_HOLD_N) {
                s_lock = HFI_LOCK_LOCKED;
                s_lock_cnt = 0u;
            }
        } else {
            s_lock_cnt = 0u;
            if (s_lock == HFI_LOCK_FAULT) {
                s_lock = HFI_LOCK_CAPTURE;
            }
        }
    }
#else
    (void)0;
#endif
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
#if M1_HFI_QKICK_START_ENABLE
        || ((s_stage == HFI_STAGE_CRAWL) && (s_start_phase != HFI_START_HOLD))
#endif
#if M1_HFI_QKICK_IF_ENABLE
        || ((s_stage == HFI_STAGE_CRAWL) && (s_if_active != 0u))
#endif
        ) {
        s_ud_inj = s_vh_v * s_vh_scale * s_sign;
        s_uq_inj = 0.0f;
#if M1_HFI_DQ_IDENT_ENABLE
        if (s_park_off != 0.0f) {
            const float c = cosf(s_park_off);
            const float s = sinf(s_park_off);
            const float ud = s_ud_inj;
            const float uq = s_uq_inj;

            s_ud_inj = ud * c + uq * s;
            s_uq_inj = -ud * s + uq * c;
        }
#endif
    } else {
        /* Park≠θ̂：。hat 。d 轴的 ±Vh 表达。Park 系。。θ̂−θ_park */
        const float th = s_theta_hat - hfi_demod_frame_theta();
        const float c = cosf(th);
        const float s = sinf(th);

        s_ud_inj = s_vh_v * s_vh_scale * s_sign * c;
        s_uq_inj = s_vh_v * s_vh_scale * s_sign * s;
    }
}

#if M1_HFI_RUN_LADDER_ENABLE && (M1_HFI_GATE != 131)
static int hfi_run_ladder_nsteps(void)
{
#if M1_HFI_GATE == 132
    /* 100→500 五档，再 400→0 五档。 */
    return 10;
#elif M1_HFI_GATE == 137
    /* 100、200、100。每档 5 s。 */
    return 3;
#elif (M1_HFI_GATE == 133) || (M1_HFI_GATE == 134) || (M1_HFI_GATE == 135) || (M1_HFI_GATE == 136)
    return 10;
#else
    const float span = M1_HFI_RUN_RPM_MAX - M1_HFI_RUN_RPM_START;

    if ((span < 0.0f) || (M1_HFI_RUN_RPM_STEP <= 0.0f)) {
        return 1;
    }
    return (int)(span / M1_HFI_RUN_RPM_STEP + 0.5f) + 1;
#endif
}
#endif

/**
 * @brief RUN 总时。[s]
 */
static float hfi_run_total_s(void)
{
#if M1_HFI_SENSED_CAL_ENABLE
    return M1_HFI_SENSED_CAL_STEP_S * (float)M1_HFI_SENSED_CAL_NRPM;
#elif M1_HFI_RUN_LADDER_ENABLE
#if M1_HFI_GATE == 131
    /* 100、200 各 5 s，再 50 rpm/s 爬到 1500 并停 5 s，然后 15 档降到 0。 */
    return 10.0f + ((1500.0f - 200.0f) / 50.0f) + 5.0f + (15.0f * 5.0f);
#elif M1_HFI_GATE == 138
    /* +1500 与 −1500 各停 3 s，反复 5 次。指令阶跃，不爬坡。 */
    return 5.0f * 6.0f;
#elif M1_HFI_GATE == 141
    /* 到了 ±1500 再保持 0.5 s 才换向，五轮。到不了则最多 40 s。 */
    return 40.0f;
#elif M1_HFI_GATE == 139
    /* 100 rpm 停 15 s，200 rpm 停 10 s。 */
    return 25.0f;
#elif M1_HFI_GATE == 140
    /* 100 rpm 停 15 s，200 rpm 停 10 s。 */
    return 25.0f;
#else
    return (float)hfi_run_ladder_nsteps() * M1_HFI_RUN_STEP_S;
#endif
#else
#if M1_HFI_QKICK_START_ENABLE
    return M1_HFI_RUN_RPM1_S + M1_HFI_RUN_RPM2_S;
#elif M1_HFI_QKICK_SPEED_ENABLE && M1_HFI_QKICK_AFTER_LOCK_ENABLE
    return M1_HFI_QKICK_SPEED_PRE_S + M1_HFI_RUN_RPM1_S + M1_HFI_RUN_RPM2_S;
#else
    return M1_HFI_RUN_RPM1_S + M1_HFI_RUN_RPM2_S;
#endif
#endif
}

/**
 * @brief S2d 两档指令（与 GATE 46 同一写法）。t 相对 RUN 起点。
 * @return 。 为本档转速；<0 表示已过 100/200 。
 */
static float hfi_run_speed_ref_s2d(float t)
{
    if (t < M1_HFI_RUN_RPM1_S) {
        return M1_HFI_RUN_RPM1;
    }
    if (t < (M1_HFI_RUN_RPM1_S + M1_HFI_RUN_RPM2_S)) {
        return M1_HFI_RUN_RPM2;
    }
    return -1.0f;
}

#if (M1_HFI_GATE == 51) || (M1_HFI_GATE == 52)
/**
 * @brief 。RUN：重置相位；51 。VH 表档 0。2 用恒 VH。
 */
static void hfi_run_ladder_init(void)
{
    s_run_phase = 0u;
    s_run_phase_tick = 0u;
    s_run_ladder_done = 0u;
    s_run_dwell_ticks =
        (uint32_t)(M1_HFI_RUN_STEP_S / OBS_CTRL_TS_S + 0.5f);
    if (s_run_dwell_ticks < 1u) {
        s_run_dwell_ticks = 1u;
    }
#if M1_HFI_GATE == 51
    s_vh_v = s_run_vh_seq[0];
#else
    s_vh_v = M1_HFI_VH_V;
#endif
#if M1_HFI_SPD_RAMP_ENABLE && (M1_HFI_GATE == 52)
    s_spd_ramp_done = 0u;
    s_spd_ramp_tick = 0u;
    s_spd_hold0_ticks =
        (uint32_t)(M1_HFI_SPD_HOLD0_S / OBS_CTRL_TS_S + 0.5f);
    s_spd_ramp_ticks =
        (uint32_t)(M1_HFI_SPD_RAMP_S / OBS_CTRL_TS_S + 0.5f);
    if (s_spd_ramp_ticks < 1u) {
        s_spd_ramp_ticks = 1u;
    }
#endif
}
#endif

#if M1_HFI_IQ_PULL_ENABLE
/**
 * @brief |pll_int| 超过门槛并保。HOLD 后才开速度环。
 * @note ARM_ENABLE=0 时不交接。用 s_stage_t 计时，同一拍多次调用不会把 100 ms 算短。
 */
static void hfi_iq_pull_try_arm(void)
{
#if M1_HFI_IQ_PULL_ARM_ENABLE
    float rpm;
    float ae;

    if (s_iq_pull_done != 0u) {
        return;
    }
    if (s_stage != HFI_STAGE_RUN) {
        return;
    }
    rpm = s_pll_int * (60.0f / (2.0f * 3.14159265f * (float)OBS_POLE_PAIRS));
    ae = (rpm < 0.0f) ? -rpm : rpm;
    if (ae >= M1_HFI_IQ_PULL_ARM_RPM) {
        if (s_iq_pull_above_t < 0.0f) {
            s_iq_pull_above_t = s_stage_t;
        }
        if ((s_stage_t - s_iq_pull_above_t) >= M1_HFI_IQ_PULL_HOLD_S) {
            s_iq_pull_done = 1u;
#if (M1_HFI_GATE == 51) || (M1_HFI_GATE == 52)
            /* 固定电流段不推进阶梯；交接时。100 rpm 档重新计。*/
            hfi_run_ladder_init();
#endif
        }
    } else {
        s_iq_pull_above_t = -1.0f;
    }
#endif
}
#endif

#if (M1_HFI_GATE == 51) || (M1_HFI_GATE == 52)
/**
 * @brief 。omega_ref。热路径只读 rpm。1 仅换档写 vh 表。
 * @note GATE52+SPD_RAMP：先 ω*=0 静置，再斜坡到首档，然后才走阶梯（速度环自起）。
 */
static float hfi_run_ladder_poll(void)
{
#if M1_HFI_SPD_RAMP_ENABLE && (M1_HFI_GATE == 52)
    if (s_spd_ramp_done == 0u) {
        uint32_t k;

        s_spd_ramp_tick++;
        if (s_spd_ramp_tick <= s_spd_hold0_ticks) {
            return 0.0f;
        }
        k = s_spd_ramp_tick - s_spd_hold0_ticks;
        if (k >= s_spd_ramp_ticks) {
            s_spd_ramp_done = 1u;
            s_run_phase = 0u;
            s_run_phase_tick = 0u;
            return s_run_rpm_seq[0];
        }
        /* 整数节拍斜坡：rpm = rpm0 * k / N（外环频率，。PWM 热路径） */
        return s_run_rpm_seq[0] * ((float)k / (float)s_spd_ramp_ticks);
    }
#endif
    {
        const float rpm = s_run_rpm_seq[s_run_phase];

        if (s_run_ladder_done != 0u) {
            return rpm;
        }
        s_run_phase_tick++;
        if (s_run_phase_tick < s_run_dwell_ticks) {
            return rpm;
        }
        s_run_phase_tick = 0u;
        if ((uint8_t)(s_run_phase + 1u) < HFI_RUN_LADDER_PHASES) {
            s_run_phase++;
#if M1_HFI_GATE == 51
            s_vh_v = s_run_vh_seq[s_run_phase];
#endif
            return s_run_rpm_seq[s_run_phase];
        }
        s_run_ladder_done = 1u;
        return rpm;
    }
}
#endif

/**
 * @brief 旁路 RUN 速度指令
 */
#if M1_HFI_GATE == 141
static int s_rev141_idx;
static float s_rev141_hold = -1.0f;
static uint8_t s_rev141_done;
#endif

static float hfi_run_speed_ref_rpm(void)
{
#if M1_HFI_SENSED_CAL_ENABLE
    {
        static const float k_rpm[6] = {
            M1_HFI_SENSED_CAL_RPM0, M1_HFI_SENSED_CAL_RPM1,
            M1_HFI_SENSED_CAL_RPM2, M1_HFI_SENSED_CAL_RPM3,
            M1_HFI_SENSED_CAL_RPM4, M1_HFI_SENSED_CAL_RPM5
        };
        const float hold = M1_HFI_SENSED_CAL_STEP_S;
        int idx;
        int n = (int)M1_HFI_SENSED_CAL_NRPM;

        if (n > 6) {
            n = 6;
        }
        if (n < 1) {
            n = 1;
        }
        if (hold <= 0.0f) {
            return k_rpm[0];
        }
        idx = (int)(s_stage_t / hold);
        if (idx < 0) {
            idx = 0;
        }
        if (idx >= n) {
            idx = n - 1;
        }
        return k_rpm[idx];
    }
#elif M1_HFI_RUN_LADDER_ENABLE
#if M1_HFI_GATE == 131
    {
        float rpm;
        /*
         * 100、200 各停 5 s。之后 50 rpm/s 爬到 1500，停 5 s。
         * 再按 100 rpm / 5 s 降到 0。1000 这一档交回 HFI。
         */
        if (s_stage_t < 5.0f) {
            return 100.0f;
        }
        if (s_stage_t < 10.0f) {
            return 200.0f;
        }
        if (s_stage_t < 36.0f) {
            rpm = 200.0f + (s_stage_t - 10.0f) * 50.0f;
            if (rpm > 1500.0f) {
                rpm = 1500.0f;
            }
            return rpm;
        }
        if (s_stage_t < 41.0f) {
            return 1500.0f;
        }
        {
            int down = (int)((s_stage_t - 41.0f) / 5.0f);

            if (down < 0) {
                down = 0;
            }
            if (down > 14) {
                down = 14;
            }
            return 1400.0f - (float)down * 100.0f;
        }
    }
#elif M1_HFI_GATE == 138
    {
        int idx = (int)(s_stage_t / 3.0f);

        /* 指令直接在 +1500 与 −1500 之间阶跃。各停 3 s，各 5 次。 */
        if (idx < 0) {
            idx = 0;
        }
        if (idx > 9) {
            idx = 9;
        }
        if ((idx % 2) == 0) {
            return 1500.0f;
        }
        return -1500.0f;
    }
#elif M1_HFI_GATE == 141
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
#elif (M1_HFI_GATE == 139) || (M1_HFI_GATE == 140)
    /* 100 rpm 停 15 s，接着 200 rpm 停 10 s。 */
    if (s_stage_t < 15.0f) {
        return 100.0f;
    }
    return 200.0f;
#elif M1_HFI_GATE == 137
    {
        int idx = (int)(s_stage_t / 5.0f);

        /* 100 → 200 → 100。每档 5 s。 */
        if (idx < 0) {
            idx = 0;
        }
        if (idx > 2) {
            idx = 2;
        }
        if (idx == 1) {
            return 200.0f;
        }
        return 100.0f;
    }
#elif (M1_HFI_GATE == 132) || (M1_HFI_GATE == 133) || (M1_HFI_GATE == 134) || (M1_HFI_GATE == 135) || (M1_HFI_GATE == 136)
    {
        int idx = (int)(s_stage_t / 5.0f);

        /* 100→500，再 400→0。每档 5 s。 */
        if (idx < 0) {
            idx = 0;
        }
        if (idx > 9) {
            idx = 9;
        }
        if (idx < 5) {
            return 100.0f + (float)idx * 100.0f;
        }
        return 400.0f - (float)(idx - 5) * 100.0f;
    }
#else
    {
        const float hold = M1_HFI_RUN_STEP_S;
        int idx;
        int n;
        float rpm;

        if (hold <= 0.0f) {
            return M1_HFI_RUN_RPM_START;
        }
        idx = (int)(s_stage_t / hold);
        if (idx < 0) {
            idx = 0;
        }
        n = hfi_run_ladder_nsteps();
        if (idx >= n) {
            idx = n - 1;
        }
        rpm = M1_HFI_RUN_RPM_START + (float)idx * M1_HFI_RUN_RPM_STEP;
        if (rpm > M1_HFI_RUN_RPM_MAX) {
            rpm = M1_HFI_RUN_RPM_MAX;
        }
        return rpm;
    }
#endif
#elif (M1_HFI_GATE == 51) || (M1_HFI_GATE == 52)
    return s_run_rpm_seq[s_run_phase];
#else
    {
        float t = s_stage_t;
        float rpm;

#if M1_HFI_QKICK_START_ENABLE
        /* CAPTURE 已建速，RUN 段不。PRE=0 */
        if ((s_qk_done != 0u) && (s_start_run_armed != 0u)) {
            /* fall through with t = s_stage_t */
        } else
#endif
#if M1_HFI_QKICK_SPEED_ENABLE && M1_HFI_QKICK_AFTER_LOCK_ENABLE
        /* 。S1：踢后先零速预。*/
        if (s_qk_done != 0u) {
            if (t < M1_HFI_QKICK_SPEED_PRE_S) {
                return 0.0f;
            }
            t -= M1_HFI_QKICK_SPEED_PRE_S;
        }
#endif
        rpm = hfi_run_speed_ref_s2d(t);
        if (rpm >= 0.0f) {
            return rpm;
        }
        return 0.0f;
    }
#endif
}

/**
 * @brief 机械 rpm 。电角速度 [rad/s]
 */
static float hfi_omega_el_from_rpm(float rpm_mech)
{
    return rpm_mech * (2.0f * (float)M_PI / 60.0f) * (float)OBS_POLE_PAIRS;
}

#if M1_HFI_DELTA_SWEEP_ENABLE
/**
 * @brief δ 扫描总步数（含端点）
 */
static int16_t hfi_delta_nsteps(void)
{
    const float span = M1_HFI_DELTA_MAX_DEG - M1_HFI_DELTA_MIN_DEG;
    float steps = span / M1_HFI_DELTA_STEP_DEG;

    if (steps < 0.0f) {
        steps = -steps;
    }
    return (int16_t)(steps + 0.5f) + 1;
}

/**
 * @brief 。i 步的 δ [rad]
 */
static float hfi_delta_at_step(int16_t i)
{
    const float deg = M1_HFI_DELTA_MIN_DEG + (float)i * M1_HFI_DELTA_STEP_DEG;

    return deg * ((float)M_PI / 180.0f);
}
#endif

static void hfi_xy_lp_reset(void)
{
    s_x_lp = M1_HFI_A_CMD;
    s_y_lp = 0.0f;
    s_eps_lp = 0.0f;
#if M1_HFI_DEMOD_HP_ENABLE
    s_id_inj_slow = 0.0f;
    s_iq_inj_slow = 0.0f;
#endif
#if M1_HFI_DEMOD_AB_ENABLE || M1_HFI_DEMOD_AB_MID_ENABLE
    s_ia_prev = 0.0f;
    s_ib_prev = 0.0f;
#endif
#if M1_HFI_DEMOD_AB_MID_ENABLE
    s_ia_older = 0.0f;
    s_ib_older = 0.0f;
    s_th_ab_prev = 0.0f;
    s_ab_mid_n = 0u;
#endif
#if M1_HFI_IQ_AUTH_ENABLE
    s_iq_auth_abs = M1_HFI_IQ_AUTH_IQ_LO;
    s_iq_auth_good_n = 0u;
    s_iq_auth_bad_n = 0u;
    s_iq_auth_ok = 0u;
#endif
#if M1_HFI_IQ_AUTH_FEED_ENABLE
    s_iq_feed_ok_t = 0.0f;
    s_iq_feed_cmd = 0.0f;
#endif
#if M1_HFI_QKICK_PRE_GATE_ENABLE
    s_qk_pre_ok = 0u;
    s_qk_pre_good_n = 0u;
    s_qk_pre_false_n = 0u;
    s_qk_pre_cd = 0u;
    /* flip_n 。enter_run 保留至踢前；reset/init 清零 */
#endif
}

#if M1_HFI_QKICK_PRE_GATE_ENABLE
/**
 * PRE 假锁门禁（atan2 指纹，不。enc）。
 * 假：|ε|≥EPS_Q，或 (|ε|小且|x|≤X_BAD) 。连续 FALSE_N 。θ̂+=π/2。
 * 真：|ε|<EPS_OK 连续 HOLD_N 。pre_ok，此后冻结再翻；仅此时准踢。
 */
static void hfi_qk_pre_gate_step(void)
{
    float ae = s_eps;
    float ax = s_x_lp;
    uint8_t is_false;

    if (ae < 0.0f) {
        ae = -ae;
    }
    if (ax < 0.0f) {
        ax = -ax;
    }
    if (s_qk_pre_cd > 0u) {
        s_qk_pre_cd--;
    }

    is_false = 0u;
    if (ae >= M1_HFI_QKICK_PRE_GATE_EPS_Q_RAD) {
        is_false = 1u;
    } else if ((ae <= M1_HFI_QKICK_PRE_GATE_EPS_FALSE_MAX) &&
               (ax <= M1_HFI_QKICK_PRE_GATE_X_BAD)) {
        is_false = 1u;
    }

    if (s_qk_pre_ok != 0u) {
        /* 已确认：保持；。再爆则清 ok 重走 */
        if (is_false != 0u) {
            s_qk_pre_ok = 0u;
            s_qk_pre_good_n = 0u;
        }
        return;
    }

    if (is_false != 0u) {
        s_qk_pre_good_n = 0u;
        if (s_qk_pre_cd > 0u) {
            return; /* 冷却中不累计、不连翻 */
        }
        if (s_qk_pre_false_n < 0xFFFFu) {
            s_qk_pre_false_n++;
        }
        if ((s_qk_pre_false_n >= (uint16_t)M1_HFI_QKICK_PRE_GATE_FALSE_N) &&
            (s_qk_pre_flip_n < (uint16_t)M1_HFI_QKICK_PRE_GATE_FLIP_MAX)) {
            s_theta_hat = motor_wrap_pi(s_theta_hat + 0.5f * (float)M_PI);
            s_pll_int = 0.0f;
            s_omega_el = 0.0f;
            s_qk_pre_cd = (uint16_t)M1_HFI_QKICK_PRE_GATE_COOLDOWN_N;
            s_qk_pre_false_n = 0u;
            s_qk_pre_good_n = 0u;
            s_qk_pre_flip_n++;
            s_theta_err = motor_wrap_pi(s_theta_hat - s_theta_enc);
        }
        return;
    }

    s_qk_pre_false_n = 0u;
    if (ae < M1_HFI_QKICK_PRE_GATE_EPS_OK_RAD) {
        if (s_qk_pre_good_n < 0xFFFFu) {
            s_qk_pre_good_n++;
        }
        if (s_qk_pre_good_n >= (uint16_t)M1_HFI_QKICK_PRE_GATE_HOLD_N) {
            s_qk_pre_ok = 1u;
        }
    } else {
        s_qk_pre_good_n = 0u;
    }
}
#endif

#if M1_HFI_IQ_AUTH_ENABLE
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
#endif

#if M1_HFI_IQ_AUTH_FEED_ENABLE
/**
 * @brief qual_ok 后延。斜坡。Iq（C4。
 * @return 馈流指令；未放行时为 0
 * @note 每控制拍只应调用一次（推进 delay 计时。
 */
static float hfi_iq_auth_feed_step(void)
{
    float delay_s;
    float ramp_s;
    float target;
    float step;

    if (s_stage != HFI_STAGE_RUN) {
        s_iq_feed_ok_t = 0.0f;
        s_iq_feed_cmd = 0.0f;
        return 0.0f;
    }

#if M1_HFI_IQ_AUTH_FEED_COMPARE_AB
    if (s_iq_feed_mode == 0u) {
        delay_s = M1_HFI_IQ_AUTH_FEED_A_DELAY_S;
        ramp_s = M1_HFI_IQ_AUTH_FEED_A_RAMP_S;
    } else {
        delay_s = M1_HFI_IQ_AUTH_FEED_B_DELAY_S;
        ramp_s = M1_HFI_IQ_AUTH_FEED_B_RAMP_S;
    }
#else
    delay_s = M1_HFI_IQ_AUTH_FEED_DELAY_S;
    ramp_s = M1_HFI_IQ_AUTH_FEED_RAMP_S;
#endif

    /*
     * 已放出馈流：x 。GOOD/BAD 门槛附近抖动时不要把电流掐回 0。
     * 1343。.8 A 断约 0.6 s，。只有几度，θ。被拖到约 170°。
     */
#if M1_HFI_IQ_AUTH_FEED_HOLD
    if ((s_iq_auth_ok == 0u) && (s_iq_feed_ok_t < delay_s)) {
        s_iq_feed_ok_t = 0.0f;
        s_iq_feed_cmd = 0.0f;
        return 0.0f;
    }
#else
    if (s_iq_auth_ok == 0u) {
        s_iq_feed_ok_t = 0.0f;
        s_iq_feed_cmd = 0.0f;
        return 0.0f;
    }
#endif

    s_iq_feed_ok_t += OBS_CTRL_TS_S;
    if (s_iq_feed_ok_t < delay_s) {
        s_iq_feed_cmd = 0.0f;
        return 0.0f;
    }

    if (M1_HFI_IQ_AUTH_FEED_A > 0.0f) {
        target = M1_HFI_IQ_AUTH_FEED_A;
    } else {
        target = s_iq_auth_abs;
    }

    if (ramp_s <= 0.0f) {
        s_iq_feed_cmd = target;
    } else {
        step = target * (OBS_CTRL_TS_S / ramp_s);
        if (step < 0.0f) {
            step = -step;
        }
        if (s_iq_feed_cmd < target) {
            s_iq_feed_cmd += step;
            if (s_iq_feed_cmd > target) {
                s_iq_feed_cmd = target;
            }
        } else {
            s_iq_feed_cmd = target;
        }
    }
#if M1_HFI_FEED_COAST_ENABLE
    /* 中段掐流滑行：不。auth/feed 状态，出窗后立刻恢。FEED_A */
    if ((s_stage_t >= M1_HFI_FEED_COAST_T0_S) &&
        (s_stage_t < M1_HFI_FEED_COAST_T1_S)) {
        return 0.0f;
    }
#endif
    return M1_HFI_IQ_AUTH_FEED_SIGN * s_iq_feed_cmd;
}
#endif

/**
 * @brief 半周差分解调；update_angle=1 时跑 PLL 更新 θ̂
 * @note Park=enc：id/iq 。enc 系，旋到 θ̂；Park=θ̂：id/iq 已在 θ̂ 。
 */
#if (M1_HFI_GATE == 38) || (M1_HFI_GATE == 53) || (M1_HFI_GATE == 54) || \
    (M1_HFI_GATE == 55) || (M1_HFI_GATE == 56) || (M1_HFI_GATE == 57) || (M1_HFI_GATE == 58) || (M1_HFI_GATE == 59) || (M1_HFI_GATE == 60) || (M1_HFI_GATE == 61) || (M1_HFI_GATE == 62) || (M1_HFI_GATE == 63) || (M1_HFI_GATE == 64) || (M1_HFI_GATE == 65) || (M1_HFI_GATE == 66) || (M1_HFI_GATE == 67) || (M1_HFI_GATE == 68) || (M1_HFI_GATE == 69) || (M1_HFI_GATE == 70) || (M1_HFI_GATE == 71) || (M1_HFI_GATE == 72) || (M1_HFI_GATE == 73) || (M1_HFI_GATE == 74) || (M1_HFI_GATE == 75) || (M1_HFI_GATE == 76) || (M1_HFI_GATE == 77) || (M1_HFI_GATE == 78) || (M1_HFI_GATE == 79) || (M1_HFI_GATE == 80) || (M1_HFI_GATE == 91) || (M1_HFI_GATE == 92) || (M1_HFI_GATE == 93)
static uint16_t s_pol_bad_n;
static uint16_t s_pol_arm_n;
static uint8_t s_pol_done;
static uint8_t s_pol_flip_pulse;

/**
 * @brief 速度环前 2 s：指令与 ω̂ 反号。|ω̂|>40 rpm 持续 0.1 s，θ。。π，只一次。
 * @note 用积分项，不。Kp·ε。窗口过后不再翻，避免中途手抓被当成极性错。
 */
static void hfi_run_polarity_once(void)
{
    float cmd;
    float w_rpm;
    const float rpm_scale =
        60.0f / (6.28318530718f * (float)OBS_POLE_PAIRS);

    if (hfi_sqwave_speed_run_active() == 0u) {
        s_pol_bad_n = 0u;
        s_pol_arm_n = 0u;
        s_pol_done = 0u;
        return;
    }
    if (s_pol_done != 0u) {
        return;
    }
    if (s_pol_arm_n < 40000u) {
        s_pol_arm_n++;
    }
    if (s_pol_arm_n >= 40000u) {
        s_pol_done = 1u;
        s_pol_bad_n = 0u;
        return;
    }
    cmd = hfi_sqwave_get_speed_ref_rpm();
    w_rpm = s_pll_int * rpm_scale;
    if (((cmd > 0.0f) && (w_rpm < -40.0f)) ||
        ((cmd < 0.0f) && (w_rpm > 40.0f))) {
        if (s_pol_bad_n < 2000u) {
            s_pol_bad_n++;
        }
        if (s_pol_bad_n >= 2000u) {
            s_theta_hat = motor_wrap_pi(s_theta_hat + (float)M_PI);
            s_pll_int = 0.0f;
            s_omega_el = 0.0f;
            s_sh_seed = 0u;
            s_pol_done = 1u;
            s_pol_flip_pulse = 1u;
            s_pol_bad_n = 0u;
        }
    } else {
        s_pol_bad_n = 0u;
    }
}
#endif

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

#if ((M1_HFI_GATE == 117) || (M1_HFI_GATE == 118) || (M1_HFI_GATE == 119) || (M1_HFI_GATE == 120) || (M1_HFI_GATE == 121) || (M1_HFI_GATE == 122) || (M1_HFI_GATE == 123) || (M1_HFI_GATE == 124) || (M1_HFI_GATE == 125) || (M1_HFI_GATE == 126) || (M1_HFI_GATE == 127) || (M1_HFI_GATE == 128) || (M1_HFI_GATE == 129) || (M1_HFI_GATE == 130) || (M1_HFI_GATE == 131) || (M1_HFI_GATE == 138) || (M1_HFI_GATE == 132) || (M1_HFI_GATE == 133) || (M1_HFI_GATE == 134) || (M1_HFI_GATE == 135) || (M1_HFI_GATE == 136) || (M1_HFI_GATE == 137) || (M1_HFI_GATE == 139) || (M1_HFI_GATE == 140) || (M1_HFI_GATE == 141)) && M1_HFI_DEMOD_INJ_AXIS
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

#if M1_HFI_DEMOD_HP_ENABLE
    /* 半周 di 吃高频；基波/FEED 慢斜率不进 x,y。电流环仍用裸 Id。 */
#if (M1_HFI_GATE == 98) || (M1_HFI_GATE == 103) || (M1_HFI_GATE == 104) || \
    (M1_HFI_GATE == 105) || (M1_HFI_GATE == 106) || (M1_HFI_GATE == 107) || \
    (M1_HFI_GATE == 108) || (M1_HFI_GATE == 109) || (M1_HFI_GATE == 110) || \
    (M1_HFI_GATE == 111) || (M1_HFI_GATE == 112) || (M1_HFI_GATE == 113) || \
    (M1_HFI_GATE == 114) || (M1_HFI_GATE == 115) || (M1_HFI_GATE == 116) || \
    (M1_HFI_GATE == 117) || (M1_HFI_GATE == 118) || (M1_HFI_GATE == 119) || (M1_HFI_GATE == 120) || (M1_HFI_GATE == 121) || (M1_HFI_GATE == 122) || (M1_HFI_GATE == 123) || (M1_HFI_GATE == 124) || (M1_HFI_GATE == 125) || (M1_HFI_GATE == 126) || (M1_HFI_GATE == 127) || (M1_HFI_GATE == 128) || (M1_HFI_GATE == 129) || (M1_HFI_GATE == 130) || (M1_HFI_GATE == 131) || (M1_HFI_GATE == 138) || (M1_HFI_GATE == 132) || (M1_HFI_GATE == 133) || (M1_HFI_GATE == 134) || (M1_HFI_GATE == 135) || (M1_HFI_GATE == 136) || (M1_HFI_GATE == 137) || (M1_HFI_GATE == 139) || (M1_HFI_GATE == 140) || (M1_HFI_GATE == 141)
    /* 踢段 1.6 A 不要把慢环预载到 FEED 量级，否则 LOG 一刀把 x 打掉。 */
    if (s_stage != HFI_STAGE_MEAS)
#endif
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
#endif

    if (s_inj_prev_valid != 0u) {
        float di_d;
        float di_q;
        uint8_t di_ok = 1u;
#if M1_HFI_DEMOD_AB_MID_ENABLE
        if (s_ab_mid_n < 2u) {
            di_ok = 0u;
            di_d = 0.0f;
            di_q = 0.0f;
        } else {
            const float dth = motor_wrap_pi(s_theta_hat - s_th_ab_prev);
            const float thm = motor_wrap_pi(s_th_ab_prev + 0.5f * dth);
            const float c = cosf(thm);
            const float s = sinf(thm);
            const float da = 0.5f * (s_ia_now + s_ia_older) - s_ia_prev;
            const float db = 0.5f * (s_ib_now + s_ib_older) - s_ib_prev;

            /* 三端点：同极性平均减对极，基波 αβ 斜坡在 Park 前抵消 */
            di_d = da * c + db * s;
            di_q = -da * s + db * c;
        }
#elif M1_HFI_DEMOD_AB_ENABLE
        {
            const float dia = s_ia_now - s_ia_prev;
            const float dib = s_ib_now - s_ib_prev;
            const float cab = cosf(s_theta_hat);
            const float sab = sinf(s_theta_hat);

            di_d = dia * cab + dib * sab;
            di_q = -dia * sab + dib * cab;
        }
#else
        di_d = id_inj - s_id_inj_prev;
        di_q = iq_inj - s_iq_inj_prev;
#endif
        if (di_ok != 0u) {
        const float prev_sign = -s_sign;
        const float a = 0.05f;

        s_di_d = di_d;
        s_di_q = di_q;
        s_x_raw = M1_HFI_XY_X_SIGN * prev_sign * di_d;
        s_y_raw = M1_HFI_XY_Y_SIGN * prev_sign * di_q;
#if M1_HFI_PLL_VESC_ERR_ENABLE
        s_e_pll = hfi_vesc_ang_err(s_y_raw);
#endif
#if ((M1_HFI_GATE == 117) || (M1_HFI_GATE == 118) || (M1_HFI_GATE == 119) || (M1_HFI_GATE == 120) || (M1_HFI_GATE == 121) || (M1_HFI_GATE == 122) || (M1_HFI_GATE == 123) || (M1_HFI_GATE == 124) || (M1_HFI_GATE == 125) || (M1_HFI_GATE == 126) || (M1_HFI_GATE == 127) || (M1_HFI_GATE == 128) || (M1_HFI_GATE == 129) || (M1_HFI_GATE == 130) || (M1_HFI_GATE == 131) || (M1_HFI_GATE == 138) || (M1_HFI_GATE == 132) || (M1_HFI_GATE == 133) || (M1_HFI_GATE == 134) || (M1_HFI_GATE == 135) || (M1_HFI_GATE == 136) || (M1_HFI_GATE == 137) || (M1_HFI_GATE == 139) || (M1_HFI_GATE == 140) || (M1_HFI_GATE == 141)) && M1_HFI_DEMOD_INJ_AXIS && M1_HFI_PLL_VESC_ERR_ENABLE
        if (s_pair_ready != 0u) {
            const float y_pair = M1_HFI_XY_Y_SIGN * prev_sign * s_pair_di_q;

            s_e_pll = hfi_vesc_ang_err(y_pair);
        }
#endif
#if (M1_HFI_GATE == 98) || (M1_HFI_GATE == 103) || (M1_HFI_GATE == 104) || \
    (M1_HFI_GATE == 105) || (M1_HFI_GATE == 106) || (M1_HFI_GATE == 107) || \
    (M1_HFI_GATE == 108) || (M1_HFI_GATE == 109) || (M1_HFI_GATE == 110) || \
    (M1_HFI_GATE == 111) || (M1_HFI_GATE == 112) || (M1_HFI_GATE == 113) || \
    (M1_HFI_GATE == 114) || (M1_HFI_GATE == 115) || (M1_HFI_GATE == 116) || \
    (M1_HFI_GATE == 117) || (M1_HFI_GATE == 118) || (M1_HFI_GATE == 119) || (M1_HFI_GATE == 120) || (M1_HFI_GATE == 121) || (M1_HFI_GATE == 122) || (M1_HFI_GATE == 123) || (M1_HFI_GATE == 124) || (M1_HFI_GATE == 125) || (M1_HFI_GATE == 126) || (M1_HFI_GATE == 127) || (M1_HFI_GATE == 128) || (M1_HFI_GATE == 129) || (M1_HFI_GATE == 130) || (M1_HFI_GATE == 131) || (M1_HFI_GATE == 138) || (M1_HFI_GATE == 132) || (M1_HFI_GATE == 133) || (M1_HFI_GATE == 134) || (M1_HFI_GATE == 135) || (M1_HFI_GATE == 136) || (M1_HFI_GATE == 137) || (M1_HFI_GATE == 139) || (M1_HFI_GATE == 140) || (M1_HFI_GATE == 141)
        /* 踢段 Iq 会冲掉 PRE 的 x。只记 raw，LPF/AUTH/PLL 冻结；半周符号仍翻转。 */
        if (s_stage != HFI_STAGE_MEAS)
#endif
        {
#if (M1_HFI_GATE == 98) && M1_HFI_DEMOD_SKIP_OUTLIER
        /* 不像凸极的半周丢掉：不写 x/ε、不进 PLL。不把 atan2 砍小。 */
        {
            float ay = s_y_raw;
            float axr = s_x_raw;
            float aq = di_q;
            uint8_t dirty = 0u;

            if (ay < 0.0f) {
                ay = -ay;
            }
            if (axr < 0.0f) {
                axr = -axr;
            }
            if (aq < 0.0f) {
                aq = -aq;
            }
            if (ay > M1_HFI_DEMOD_SKIP_Y_ABS) {
                dirty = 1u;
            }
            if (axr > M1_HFI_DEMOD_SKIP_X_ABS) {
                dirty = 1u;
            }
            if (aq > M1_HFI_DEMOD_SKIP_DI_Q_ABS) {
                dirty = 1u;
            }
            if (dirty == 0u) {
#endif
        /* P0-1：滤向量再取角，避免 ±π 支割上把 ±90° 抹平 */
        s_x_lp += a * (s_x_raw - s_x_lp);
        s_y_lp += a * (s_y_raw - s_y_lp);
#if M1_HFI_ATAN2_ENABLE
        s_eps = 0.5f * atan2f(s_y_lp, s_x_lp - M1_HFI_A_CMD);
            s_eps_lp = s_eps;
#else
        {
            float eps_sign = M1_HFI_EPS_SIGN;
            float eps_raw;

            eps_raw = eps_sign * di_q * prev_sign;
#if M1_HFI_AXIS_SEL_ENABLE
            {
                const float eps_d_raw =
                    eps_sign * di_d * prev_sign * M1_HFI_AXIS_SEL_D_SIGN;

                s_eps_d_lp += a * (eps_d_raw - s_eps_d_lp);
            }
#endif
            s_eps_lp += a * (eps_raw - s_eps_lp);
            s_eps = s_eps_lp;
        }
#endif
#if M1_HFI_IQ_AUTH_ENABLE
        hfi_iq_auth_step();
#endif
#if M1_HFI_HFI_ID_SOFT_ENABLE
#if !M1_HFI_IQ_AUTH_ENABLE
#error "M1_HFI_HFI_ID_SOFT_ENABLE requires M1_HFI_IQ_AUTH_ENABLE=1"
#endif
        /* GATE81: after lock/qual, soft-open Id while VH stays full (no SMO). */
        if ((s_stage == HFI_STAGE_RUN) && (s_qk_done != 0u) &&
            (s_iq_auth_ok != 0u) && (s_id_pi_release == 0u)) {
            s_id_pi_release = 1u;
            s_id_pi_soft_n = 0u;
            s_id_pi_soft_cmd = -1.0f; /* auto ramp via ID_PI_SOFT_N */
        }
#endif
#if M1_HFI_QKICK_PRE_GATE_ENABLE
        /* 。PRE（踢。RUN）跑门禁；MEAS/LOG/踢后 RUN 不翻。*/
        if ((update_angle != 0u) && (s_stage == HFI_STAGE_RUN) &&
            (s_qk_done == 0u)) {
            hfi_qk_pre_gate_step();
        }
#endif

        if (update_angle != 0u) {
#if M1_HFI_PLL_ENABLE
            {
                float w_ff;
                float dw;
                float e_pol;
                float e_pll = s_eps;
                float ae_eps;

#if M1_HFI_PLL_VESC_ERR_ENABLE && (M1_HFI_GATE != 115) && (M1_HFI_GATE != 116)
                /* atan2 的 s_eps 仍给 PRE/AUTH/VOFA。115/116：PLL 直接吃 s_eps。 */
                e_pll = s_e_pll;
#endif
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
#if M1_HFI_PLL_HOLD_X_BELOW_A
                if (s_x_lp <= M1_HFI_A_CMD) {
                    dw = 0.0f;
                } else
#endif
#if M1_HFI_PLL_SKIP_X_BELOW_A
                if (s_x_lp <= M1_HFI_A_CMD) {
                    dw = motor_clampf(M1_HFI_PLL_KP * e_pll + s_pll_int,
                                    M1_HFI_PLL_INT_MAX);
                } else
#endif
                {
#if M1_HFI_GATE == 114
                    /*
                     * 限幅只夹积分转速。比例项另加到角度速率上，不和积分挤进同一个夹紧。
                     * 残差已经贴在 MAX_ERR 上时这一拍不积分（失锁区不把速度积到限幅）。
                     */
                    if (ae_eps < M1_HFI_PLL_VESC_MAX_ERR) {
                        if ((s_pll_eps_dead > 0.0f) && (ae_eps < s_pll_eps_dead)) {
                            if (M1_HFI_PLL_INT_LEAK > 0.0f) {
                                s_pll_int *= (1.0f - M1_HFI_PLL_INT_LEAK);
                            }
                        } else {
                            s_pll_int += M1_HFI_PLL_KI * e_pll * OBS_CTRL_TS_S;
                        }
                    }
                    s_pll_int = motor_clampf(s_pll_int, M1_HFI_PLL_INT_MAX);
                    dw = s_pll_int + M1_HFI_PLL_KP * e_pll;
#elif M1_HFI_GATE == 116
                    /*
                     * 115 的 ε。|ε|≥0.5 时不积分，避免失锁区把速度积到限幅。
                     * 限幅只夹积分。比例项加在角度速率上。
                     */
                    if (ae_eps < 0.5f) {
                        if ((s_pll_eps_dead > 0.0f) && (ae_eps < s_pll_eps_dead)) {
                            if (M1_HFI_PLL_INT_LEAK > 0.0f) {
                                s_pll_int *= (1.0f - M1_HFI_PLL_INT_LEAK);
                            }
                        } else {
                            s_pll_int += M1_HFI_PLL_KI * e_pll * OBS_CTRL_TS_S;
                        }
                    }
                    s_pll_int = motor_clampf(s_pll_int, M1_HFI_PLL_INT_MAX);
                    dw = s_pll_int + M1_HFI_PLL_KP * e_pll;
#elif M1_HFI_GATE == 118
                    /*
                     * VESC foc_hfi_adjust_angle 的低速半段。
                     * gain_int=4000*gain 即 Kp，gain_int2=10*gain=Kp/400。
                     * |I| 不超过上一拍 θ̂ 转速，角度速率不再夹 200 rad/s。
                     */
                    {
                        float lim = s_speed_est;

                        (void)ae_eps;
                        s_pll_int += e_pll * (M1_HFI_PLL_KP * (1.0f / 400.0f));
                        if (lim < 0.0f) {
                            lim = -lim;
                        }
                        s_pll_int = motor_clampf(s_pll_int, lim);
                        dw = M1_HFI_PLL_KP * e_pll + s_pll_int;
                    }
#elif (M1_HFI_GATE == 119) || (M1_HFI_GATE == 120) || (M1_HFI_GATE == 121) || (M1_HFI_GATE == 122) || (M1_HFI_GATE == 123) || (M1_HFI_GATE == 124) || (M1_HFI_GATE == 125) || (M1_HFI_GATE == 126) || (M1_HFI_GATE == 127) || (M1_HFI_GATE == 128) || (M1_HFI_GATE == 129) || (M1_HFI_GATE == 130) || (M1_HFI_GATE == 131) || (M1_HFI_GATE == 138) || (M1_HFI_GATE == 132) || (M1_HFI_GATE == 133) || (M1_HFI_GATE == 134) || (M1_HFI_GATE == 135) || (M1_HFI_GATE == 136) || (M1_HFI_GATE == 137) || (M1_HFI_GATE == 139) || (M1_HFI_GATE == 140) || (M1_HFI_GATE == 141)
                    /*
                     * 118 的低通。积分不再按 e·(Kp/400) 爬，直接等于上一拍 θ̂ 转速。
                     * 角度速率 = Kp·e + I，不夹 200 rad/s。e 闪一下时转速留在积分里。
                     */
                    (void)ae_eps;
                    s_pll_int = s_speed_est;
                    dw = M1_HFI_PLL_KP * e_pll + s_pll_int;
#else
                    if ((s_pll_eps_dead > 0.0f) && (ae_eps < s_pll_eps_dead)) {
                        if (M1_HFI_PLL_INT_LEAK > 0.0f) {
                            s_pll_int *= (1.0f - M1_HFI_PLL_INT_LEAK);
                        }
                    } else {
                        s_pll_int += M1_HFI_PLL_KI * e_pll * OBS_CTRL_TS_S;
                    }
                    s_pll_int = motor_clampf(s_pll_int, M1_HFI_PLL_INT_MAX);
                    dw = motor_clampf(M1_HFI_PLL_KP * e_pll + s_pll_int, M1_HFI_PLL_INT_MAX);
#endif
                }

#if M1_HFI_OMEGA_FF_SRC == 1
                w_ff = s_omega_ff_el;
#elif M1_HFI_OMEGA_FF_SRC == 2
                w_ff = s_pll_int;
                dw = motor_clampf(M1_HFI_PLL_KP * e_pll, M1_HFI_PLL_W_MAX);
#elif M1_HFI_OMEGA_FF_SRC == 3
                /*
                 * S1：自举为。+ 小权。ω_cmd（由上层。ω_ref）。
                 * 禁止全权 ω_ref。334）；慢斜坡时 α 小，θ̂ 不甩转子。
                 */
                {
                    const float a = M1_HFI_OMEGA_FF_REF_W;
                    float a_cl = a;

                    if (a_cl < 0.0f) {
                        a_cl = 0.0f;
                    }
                    if (a_cl > 0.5f) {
                        a_cl = 0.5f;
                    }
                    w_ff = (1.0f - a_cl) * s_pll_int + a_cl * s_omega_ff_el;
                    dw = motor_clampf(M1_HFI_PLL_KP * e_pll, M1_HFI_PLL_W_MAX);
                }
#else
                w_ff = 0.0f;
#endif
                s_omega_el = w_ff + dw;
#if (M1_HFI_GATE == 118) || (M1_HFI_GATE == 119) || (M1_HFI_GATE == 120) || \
    (M1_HFI_GATE == 122) || (M1_HFI_GATE == 123) || (M1_HFI_GATE == 125) || (M1_HFI_GATE == 126) || (M1_HFI_GATE == 127) || (M1_HFI_GATE == 128) || (M1_HFI_GATE == 129) || (M1_HFI_GATE == 130) || (M1_HFI_GATE == 131) || (M1_HFI_GATE == 138) || (M1_HFI_GATE == 132) || (M1_HFI_GATE == 133) || (M1_HFI_GATE == 134) || (M1_HFI_GATE == 135) || (M1_HFI_GATE == 136) || (M1_HFI_GATE == 137) || (M1_HFI_GATE == 139) || (M1_HFI_GATE == 140) || (M1_HFI_GATE == 141)
                /* 与 VESC UTILS_LP_FAST(..., 0.01) 相同。118 做上限，119 下一拍直接当积分。 */
                s_speed_est += 0.01f * (s_omega_el - s_speed_est);
#elif M1_HFI_GATE == 121
                /* 15 Hz、阻尼 0.7：时间常数 15 ms。20 kHz 下系数是 Ts/0.015。 */
                s_speed_est += (1.0f / 300.0f) * (s_omega_el - s_speed_est);
#elif M1_HFI_GATE == 124
                /*
                 * 速度观测 20 ms（约 8 Hz）。角度仍用上一行的 ω=Kp·e+I。
                 * 这一路就是速度环和以后 SMO 交接要读的转速。20 kHz 下系数是 Ts/0.020。
                 */
                s_speed_est += (1.0f / 400.0f) * (s_omega_el - s_speed_est);
#endif
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
#if M1_HFI_LQ_WELL_FLIP_ENABLE
                /*
                 * 84/109：y≈0 在 0° 与 90° 都会出现；x 在 90° 掉到 ~1/Lq。
                 * 只翻一次，避免 2306 连翻。不掐 Iq。
                 * KEEP_W=0：翻完清积分（111：122 rpm 时 θ̂ 停转）。
                 */
                if ((s_stage == HFI_STAGE_RUN) && (s_qk_done != 0u) &&
                    (s_lq_well_flip_n == 0u)) {
                    float ay = s_y_lp;
                    float iq_feed = 0.0f;

                    if (ay < 0.0f) {
                        ay = -ay;
                    }
#if M1_HFI_IQ_AUTH_FEED_ENABLE
                    iq_feed = s_iq_feed_cmd;
#endif
                    if (iq_feed < M1_HFI_LQ_WELL_IQ_MIN) {
                        s_lq_well_n = 0u;
                    } else if ((s_x_lp < M1_HFI_LQ_WELL_X_MAX) &&
                               (ay < M1_HFI_LQ_WELL_Y_ABS)) {
                        if (s_lq_well_n < 65535u) {
                            s_lq_well_n++;
                        }
                    } else {
                        s_lq_well_n = 0u;
                    }
                    if (s_lq_well_n >= M1_HFI_LQ_WELL_HOLD_N) {
                        s_theta_hat = motor_wrap_pi(
                            s_theta_hat + 0.5f * (float)M_PI);
#if !M1_HFI_LQ_WELL_KEEP_W
                        s_pll_int = 0.0f;
                        s_omega_el = 0.0f;
#endif
                        s_lq_well_flip_n = 1u;
                        s_lq_well_n = 0u;
                    }
                }
#endif
#if (M1_HFI_GATE == 38) || (M1_HFI_GATE == 53) || (M1_HFI_GATE == 54) || \
    (M1_HFI_GATE == 55) || (M1_HFI_GATE == 56) || (M1_HFI_GATE == 57) || (M1_HFI_GATE == 58) || (M1_HFI_GATE == 59) || (M1_HFI_GATE == 60) || (M1_HFI_GATE == 61) || (M1_HFI_GATE == 62) || (M1_HFI_GATE == 63) || (M1_HFI_GATE == 64) || (M1_HFI_GATE == 65) || (M1_HFI_GATE == 66) || (M1_HFI_GATE == 67) || (M1_HFI_GATE == 68) || (M1_HFI_GATE == 69) || (M1_HFI_GATE == 70) || (M1_HFI_GATE == 71) || (M1_HFI_GATE == 72) || (M1_HFI_GATE == 73) || (M1_HFI_GATE == 74) || (M1_HFI_GATE == 75) || (M1_HFI_GATE == 76) || (M1_HFI_GATE == 77) || (M1_HFI_GATE == 78) || (M1_HFI_GATE == 79) || (M1_HFI_GATE == 80) || (M1_HFI_GATE == 91) || (M1_HFI_GATE == 92) || (M1_HFI_GATE == 93)
                hfi_run_polarity_once();
#endif
#if M1_HFI_AXIS_SEL_ENABLE
                /*
                 * 2247：真 d 。eps_d<0；假 q 。eps_d>0。
                 * |e| 小且 eps_d>+D_TH 。。+90°；eps_d<-D_TH 持续 。axis_ok。
                 * FREEZE_ON_OK：确认后不再翻（2306 连翻根因）。
                 */
                {
                    const float ed = s_eps_d_lp;
                    const float d_th = M1_HFI_AXIS_SEL_D_TH;

                    if (s_axis_cd > 0u) {
                        s_axis_cd--;
                    }
                    if (ae_eps < M1_HFI_AXIS_SEL_EPS_MAX) {
                        if (s_axis_lock_n < 65535u) {
                            s_axis_lock_n++;
                        }
                    } else {
                        s_axis_lock_n = 0u;
                    }
                    if (ed < (-d_th)) {
                        if (s_axis_good_n < 65535u) {
                            s_axis_good_n++;
                        }
                    } else {
                        s_axis_good_n = 0u;
                    }
                    if (s_axis_good_n >= M1_HFI_AXIS_SEL_CONFIRM_N) {
                        s_axis_ok = 1u;
                    }
#if M1_HFI_AXIS_SEL_FREEZE_ON_OK
                    if (s_axis_ok == 0u)
#endif
                    {
                        const uint8_t flip_ok =
#if M1_HFI_AXIS_SEL_FLIP_MAX > 0u
                            (s_axis_flip_n < (uint16_t)M1_HFI_AXIS_SEL_FLIP_MAX)
#else
                            1u
#endif
                            ;

                        if (flip_ok &&
                            (s_axis_cd == 0u) &&
                            (s_axis_lock_n >= M1_HFI_AXIS_SEL_LOCK_N) &&
                            (ed > d_th)) {
                            s_theta_hat =
                                motor_wrap_pi(s_theta_hat + 0.5f * (float)M_PI);
                            s_axis_cd = M1_HFI_AXIS_SEL_COOLDOWN_N;
                            s_axis_lock_n = 0u;
                            s_axis_good_n = 0u;
                            s_axis_ok = 0u;
                            s_axis_flip_n++;
                        }
                    }
                }
#endif
                s_theta_err = motor_wrap_pi(hfi_theta_hat_out() - s_theta_enc);

#if M1_HFI_POLARITY_ENC_ENABLE
                e_pol = s_theta_err;
                if (e_pol < 0.0f) {
                    e_pol = -e_pol;
                }
                if (e_pol > M1_HFI_POLARITY_ERR_RAD) {
                    s_polarity_cnt++;
                    if (s_polarity_cnt >= M1_HFI_POLARITY_HOLD_N) {
                        if (e_pol > M1_HFI_POLARITY_PI_RAD) {
                            s_theta_hat = motor_wrap_pi(s_theta_hat + (float)M_PI);
                        } else if (s_theta_err >= 0.0f) {
                            s_theta_hat = motor_wrap_pi(s_theta_hat - 0.5f * (float)M_PI);
                        } else {
                            s_theta_hat = motor_wrap_pi(s_theta_hat + 0.5f * (float)M_PI);
                        }
                        /* Step0：纠角后只清 trim，禁。pll_int←ω_ref 假。*/
                        s_pll_int = 0.0f;
                        s_polarity_cnt = 0u;
                        s_theta_err = motor_wrap_pi(s_theta_hat - s_theta_enc);
                    }
                } else {
                    s_polarity_cnt = 0u;
                }
#endif
                hfi_lock_update();
            }
#endif
        }
#if (M1_HFI_GATE == 98) && M1_HFI_DEMOD_SKIP_OUTLIER
        }
        }
#endif
        }
        }
    } else {
        s_inj_prev_valid = 1u;
    }

    s_id_inj_prev = id_inj;
    s_iq_inj_prev = iq_inj;
#if M1_HFI_DEMOD_AB_MID_ENABLE
    s_ia_older = s_ia_prev;
    s_ib_older = s_ib_prev;
    s_th_ab_prev = s_theta_hat;
    if (s_ab_mid_n < 3u) {
        s_ab_mid_n++;
    }
#endif
#if M1_HFI_DEMOD_AB_ENABLE || M1_HFI_DEMOD_AB_MID_ENABLE
    s_ia_prev = s_ia_now;
    s_ib_prev = s_ib_now;
#endif
    if (s_hat_hold != 0u) {
        /* 收注入：ε 已不可用。θ。按保持速度继续走，不再停在原地。*/
        s_omega_el = s_omega_coast;
        s_theta_hat = motor_wrap_pi(s_theta_hat + s_omega_coast * OBS_CTRL_TS_S);
    }
    s_sign = -s_sign;
}

#if M1_HFI_PLL_HOLD_ENABLE && M1_HFI_PLL_RETRACK_USE_ENC
/**
 * @brief 台架：dθ_enc 只作「已转起来」门，不。Park
 */
static void hfi_pll_hold_motion_step(float dt)
{
    float dth;
    float w;
    const float rpm_scale =
        60.0f / (2.0f * (float)M_PI * (float)OBS_POLE_PAIRS);

    if (dt <= 0.0f) {
        dt = OBS_CTRL_TS_S;
    }
    if (s_enc_mot_valid != 0u) {
        dth = motor_wrap_pi(s_theta_enc - s_enc_mot_prev);
        w = (dth / dt) * rpm_scale;
        s_w_mot_rpm += 0.02f * (w - s_w_mot_rpm);
    } else {
        s_w_mot_rpm = 0.0f;
    }
    s_enc_mot_prev = s_theta_enc;
    s_enc_mot_valid = 1u;
}
#endif

#if M1_HFI_DEMOD_PROBE_ENABLE
/**
 * @brief RUN 内两段冻 θ̂：解调照算，PLL 不积分。
 * @return 1=本拍不更。θ̂
 */
static uint8_t hfi_demod_probe_freeze(void)
{
    if ((s_stage == HFI_STAGE_RUN) && (s_qk_done != 0u)) {
        if ((s_stage_t >= M1_HFI_DEMOD_FREEZE_T0_S) &&
            (s_stage_t < M1_HFI_DEMOD_FREEZE_T1_S)) {
            return 1u;
        }
        if ((s_stage_t >= M1_HFI_DEMOD_FREEZE_T2_S) &&
            (s_stage_t < M1_HFI_DEMOD_FREEZE_T3_S)) {
            return 1u;
        }
    }
    return 0u;
}
#endif

#if M1_HFI_PLL_HOLD_ENABLE
static uint8_t hfi_pll_want_update(void)
{
    float a_iq;
#if M1_HFI_PLL_RETRACK_USE_ENC
    float a_w;
#endif

    a_iq = s_qk_iq_ref;
    if (a_iq < 0.0f) {
        a_iq = -a_iq;
    }
#if M1_HFI_PLL_RETRACK_USE_ENC
    a_w = s_w_mot_rpm;
    if (a_w < 0.0f) {
        a_w = -a_w;
    }
#endif

#if M1_HFI_QKICK_IF_ENABLE
    if (s_if_active != 0u) {
        s_pll_hold = 1u;
        s_retrack_n = 0u;
        return 0u;
    }
#endif

    if (s_pll_hold != 0u) {
        if (a_iq < M1_HFI_PLL_HOLD_REL_IQ_A) {
            s_pll_hold = 0u;
            s_retrack_n = 0u;
            return 1u;
        }
#if M1_HFI_PLL_RETRACK_USE_ENC
        if (a_w >= M1_HFI_PLL_RETRACK_RPM) {
            if (s_retrack_n < 0xFFFFu) {
                s_retrack_n++;
            }
            if (s_retrack_n >= M1_HFI_PLL_RETRACK_N) {
                s_pll_hold = 0u;
                s_retrack_n = 0u;
                return 1u;
            }
        } else {
            s_retrack_n = 0u;
        }
#endif
        return 0u;
    }

#if M1_HFI_PLL_RETRACK_USE_ENC
    if ((a_iq >= M1_HFI_PLL_HOLD_IQ_A) && (a_w < M1_HFI_PLL_HOLD_RPM_LO)) {
#else
    if (a_iq >= M1_HFI_PLL_HOLD_IQ_A) {
#endif
        s_pll_hold = 1u;
        s_retrack_n = 0u;
        s_pll_int = 0.0f;
        return 0u;
    }
    return 1u;
}
#endif

#if M1_HFI_ID_ON_FROM_RUN_ENABLE
/** RUN 起 Id 常开：release + soft=1（Id*=0），允许与满 VH 共存。 */
static void hfi_id_on_from_run_arm(void)
{
    s_id_pi_release = 1u;
    s_id_pi_soft_n = 0u;
    s_id_pi_soft_cmd = 1.0f;
}
#endif

/**
 * @brief 进入旁路 RUN
 * @note SEED 模式：pll_int←ω_ref；Step4：θ。可沿。IPD，不再强。enc 播种
 */
static void hfi_enter_run(void)
{
    s_stage = HFI_STAGE_RUN;
    s_stage_t = 0.0f;
#if M1_HFI_GATE == 141
    s_rev141_idx = 0;
    s_rev141_hold = -1.0f;
    s_rev141_done = 0u;
#endif
#if (M1_HFI_GATE == 51) || (M1_HFI_GATE == 52)
    hfi_run_ladder_init();
#endif
#if M1_HFI_IQ_PULL_ENABLE
    s_iq_pull_done = 0u;
#if M1_HFI_IQ_PULL_ARM_ENABLE
    s_iq_pull_above_t = -1.0f;
#endif
#if M1_HFI_IQ_AUTH_ENABLE
    s_iq_pull_ok_t = 0.0f;
#endif
#endif
    s_sign = 1.0f;
    s_inj_prev_valid = 0u;
    hfi_xy_lp_reset();
    s_di_q = 0.0f;
    s_eps = 0.0f;
    s_polarity_cnt = 0u;
    s_lock = HFI_LOCK_CAPTURE;
    s_lock_cnt = 0u;
#if M1_HFI_QKICK_ANY
    /* IDLE/DONE 强制 0V 。PI 会顶满；。RUN 必须卸掉，否则首。Ud~十数 V 。HFI */
    s_qk_pi_reset = 1u;
#endif
#if M1_HFI_IQ_AUTH_FEED_ENABLE
#if M1_HFI_IQ_AUTH_FEED_COMPARE_AB
    /* 。。A（延。阶跃），。靴起 B（斜坡） */
    s_iq_feed_mode = (s_iq_feed_ab_n == 0u) ? 0u : 1u;
    if (s_iq_feed_ab_n < 0xFFu) {
        s_iq_feed_ab_n++;
    }
#else
    s_iq_feed_mode = 0u;
#endif
    s_iq_feed_ok_t = 0.0f;
    s_iq_feed_cmd = 0.0f;
#endif
#if M1_HFI_INIT_FROM_ENC && !M1_HFI_POLARITY_IPD_ENABLE
    s_theta_hat = motor_wrap_pi(s_theta_enc + M1_HFI_PLL_INIT_OFF_RAD);
#endif
#if M1_HFI_OMEGA_SEED_ENABLE
    s_pll_int = motor_clampf(hfi_omega_el_from_rpm(hfi_run_speed_ref_rpm()), M1_HFI_PLL_INT_MAX);
    s_omega_el = s_pll_int;
#else
    s_pll_int = 0.0f;
#if M1_HFI_OMEGA_FF_SRC == 0
    s_omega_el = 0.0f; /* 禁止把上层残留的 enc ω 种进 PLL */
#else
    s_omega_el = s_omega_ff_el;
#endif
#endif
    s_omega_trim_el = 0.0f;
#if M1_HFI_DQ_IDENT_ENABLE
    s_qk_pi_reset = 1u;
    s_vh_v = 0.0f;
    s_eps = M1_HFI_DQ_EPS_MAX_RAD + 0.2f;
    s_dq_step = (uint8_t)HFI_DQ_BLANK;
    s_dq_good_t = 0.0f;
    s_dq_enc0_ok = 0u;
    s_dq_med_n = 0u;
#endif
#if M1_HFI_ID_ON_FROM_RUN_ENABLE
    hfi_id_on_from_run_arm();
#endif
}

#if M1_HFI_QKICK_BEFORE_HFI_ENABLE || M1_HFI_QKICK_THEN_HFI_ENABLE
/**
 * @brief 极性踢后进 HFI RUN：保。decide 后的 θ̂，禁止再。enc 播种。
 * @note BEFORE_HFI：踢完直进；THEN_HFI：LOG hold 后再进。
 */
static void hfi_enter_run_after_kick(void)
{
    s_stage = HFI_STAGE_RUN;
    s_stage_t = 0.0f;
#if M1_HFI_GATE == 141
    s_rev141_idx = 0;
    s_rev141_hold = -1.0f;
    s_rev141_done = 0u;
#endif
    s_sign = 1.0f;
    s_inj_prev_valid = 0u;
#if (M1_HFI_GATE != 98) && (M1_HFI_GATE != 103) && (M1_HFI_GATE != 104) && \
    (M1_HFI_GATE != 105) && (M1_HFI_GATE != 106) && (M1_HFI_GATE != 107) && \
    (M1_HFI_GATE != 108) && (M1_HFI_GATE != 109) && (M1_HFI_GATE != 120) && (M1_HFI_GATE != 121) && (M1_HFI_GATE != 122) && (M1_HFI_GATE != 123) && (M1_HFI_GATE != 124) && (M1_HFI_GATE != 125) && (M1_HFI_GATE != 126) && (M1_HFI_GATE != 127) && (M1_HFI_GATE != 128) && (M1_HFI_GATE != 129) && (M1_HFI_GATE != 130) && (M1_HFI_GATE != 131) && (M1_HFI_GATE != 132) && (M1_HFI_GATE != 133) && (M1_HFI_GATE != 134) && (M1_HFI_GATE != 135) && (M1_HFI_GATE != 136) && (M1_HFI_GATE != 137) && (M1_HFI_GATE != 138) && (M1_HFI_GATE != 139) && (M1_HFI_GATE != 140) && (M1_HFI_GATE != 141)
    hfi_xy_lp_reset();
#endif
    s_di_q = 0.0f;
#if (M1_HFI_GATE != 98) && (M1_HFI_GATE != 103) && (M1_HFI_GATE != 104) && \
    (M1_HFI_GATE != 105) && (M1_HFI_GATE != 106) && (M1_HFI_GATE != 107) && \
    (M1_HFI_GATE != 108) && (M1_HFI_GATE != 109) && (M1_HFI_GATE != 120) && (M1_HFI_GATE != 121) && (M1_HFI_GATE != 122) && (M1_HFI_GATE != 123) && (M1_HFI_GATE != 124) && (M1_HFI_GATE != 125) && (M1_HFI_GATE != 126) && (M1_HFI_GATE != 127) && (M1_HFI_GATE != 128) && (M1_HFI_GATE != 129) && (M1_HFI_GATE != 130) && (M1_HFI_GATE != 131) && (M1_HFI_GATE != 132) && (M1_HFI_GATE != 133) && (M1_HFI_GATE != 134) && (M1_HFI_GATE != 135) && (M1_HFI_GATE != 136) && (M1_HFI_GATE != 137) && (M1_HFI_GATE != 138) && (M1_HFI_GATE != 139) && (M1_HFI_GATE != 140) && (M1_HFI_GATE != 141)
    s_eps = 0.0f;
#endif
    s_polarity_cnt = 0u;
    s_lock = HFI_LOCK_CAPTURE;
    s_lock_cnt = 0u;
#if M1_HFI_OMEGA_SEED_ENABLE
    /* 用当。ω* 播种，避。THEN_HFI 进速度段时 Type-II 。0 冷起 */
    s_pll_int = motor_clampf(hfi_omega_el_from_rpm(hfi_run_speed_ref_rpm()),
                           M1_HFI_PLL_INT_MAX);
    s_omega_el = s_pll_int;
#else
    s_pll_int = 0.0f;
    s_omega_el = 0.0f;
#endif
    s_omega_trim_el = 0.0f;
    s_qk_ov = 0u;
    s_qk_ud = 0.0f;
    s_qk_uq = 0.0f;
    s_qk_iq_ref = 0.0f;
    s_qk_id_ref = 0.0f;
    s_qk_pi_reset = 1u;
#if M1_HFI_IQ_AUTH_FEED_ENABLE
    s_iq_feed_ok_t = 0.0f;
    s_iq_feed_cmd = 0.0f;
#if !M1_HFI_IQ_AUTH_FEED_COMPARE_AB
    s_iq_feed_mode = 0u;
#endif
#endif
    s_theta_err = motor_wrap_pi(s_theta_hat - s_theta_enc);
    s_theta_cmd = s_theta_hat;
    hfi_set_inj_on_hat();
#if M1_HFI_ID_ON_FROM_RUN_ENABLE
    hfi_id_on_from_run_arm();
#endif
}
#endif

#if M1_HFI_DQ_IDENT_ENABLE && M1_HFI_DELTA_SWEEP_ENABLE
/**
 * @brief 扫完最后一档：θ̂ 保持 enc+δ，开 PLL，不。LPF、不。Vh
 */
static void hfi_enter_run_from_delta(void)
{
    s_stage = HFI_STAGE_RUN;
    s_stage_t = 0.0f;
    s_pll_int = 0.0f;
    s_omega_el = 0.0f;
    s_omega_trim_el = 0.0f;
    s_lock = HFI_LOCK_CAPTURE;
    s_lock_cnt = 0u;
    s_polarity_cnt = 0u;
    s_dq_step = (uint8_t)HFI_DQ_WAIT_EPS;
    s_dq_good_t = 0.0f;
    s_vh_v = M1_HFI_VH_V;
    s_qk_id_ref = 0.0f;
    s_qk_iq_ref = 0.0f;
    s_qk_pi_reset = 0u;
}
#endif

#if M1_HFI_POLARITY_IPD_ENABLE
/**
 * @brief 本拍脉冲 Ud：扫表或固定。
 */
static float hfi_ipd_pulse_ud_now(void)
{
#if M1_HFI_IPD_SWEEP_ENABLE
    switch (s_ipd_ud_i) {
    case 0u:
        return M1_HFI_IPD_UD_TAB_0;
    case 1u:
        return M1_HFI_IPD_UD_TAB_1;
    case 2u:
        return M1_HFI_IPD_UD_TAB_2;
    case 3u:
        return M1_HFI_IPD_UD_TAB_3;
    default:
        return M1_HFI_IPD_UD_TAB_0;
    }
#else
    return M1_HFI_IPD_UD_V;
#endif
}

/**
 * @brief IPD 失败：进 DONE，不带错极进速度。
 * @note vh_sign=0 表示拒绝判决；di_q/eps 保留 peak0/peak1 。VOFA
 */
static void hfi_ipd_fail(void)
{
    s_ipd_ov = 0u;
    s_ipd_ud = 0.0f;
    s_ipd_uq = 0.0f;
    hfi_clear_inj();
    s_vh_sign = 0.0f;
    s_di_q = s_ipd_peak0;
    s_eps = s_ipd_peak1;
    s_omega_el = s_ipd_peak0; /* VOFA ch8：peak0 */
    s_omega_trim_el = 0.0f;
    s_lock = HFI_LOCK_FAULT;
    s_lock_cnt = 0u;
#if M1_HFI_IPD_SWEEP_ENABLE
    /* 扫位模式：本格失败也。LOG，继续下一位置；θ。拉回 θ0 免假 180° */
    s_theta_hat = s_ipd_th0;
    s_theta_err = motor_wrap_pi(s_theta_hat - s_theta_enc);
    s_omega_trim_el = s_ipd_pulse_ud; /* ch5：本格脉冲电。*/
    s_stage = HFI_STAGE_LOG;
    s_stage_t = 0.0f;
#else
    s_stage = HFI_STAGE_DONE;
    s_stage_t = 0.0f;
#endif
}

/**
 * @brief 强制电压。0（清。settle。
 */
static void hfi_ipd_set_zero_u(void)
{
    s_ipd_ov = 1u;
    s_ipd_ud = 0.0f;
    s_ipd_uq = 0.0f;
    hfi_clear_inj();
}

/**
 * @brief 进入工业 IPD：ALIGN 。SETTLE 。双脉。。相对裕度。π
 */
static void hfi_enter_ipd(void)
{
    s_stage = HFI_STAGE_MOVE;
    s_stage_t = 0.0f;
    s_ipd_phase = (uint8_t)HFI_IPD_ALIGN;
    s_ipd_cnt = 0u;
    s_ipd_align_hold = 0u;
    s_ipd_peak0 = 0.0f;
    s_ipd_peak1 = 0.0f;
    s_ipd_th0 = 0.0f;
    s_ipd_iabs = 0.0f;
    s_ipd_ov = 0u;
    s_ipd_ud = 0.0f;
    s_ipd_uq = 0.0f;
    s_sign = 1.0f;
    s_inj_prev_valid = 0u;
    hfi_xy_lp_reset();
    s_di_q = 0.0f;
    s_eps = 0.0f;
    s_pll_int = 0.0f;
    s_polarity_cnt = 0u;
    s_lock = HFI_LOCK_CAPTURE;
    s_lock_cnt = 0u;
    s_omega_el = 0.0f;
    s_omega_trim_el = 0.0f;
#if M1_HFI_INIT_FROM_ENC
    s_theta_hat = motor_wrap_pi(s_theta_enc + M1_HFI_PLL_INIT_OFF_RAD);
#else
    s_theta_hat = 0.0f;
#endif
    s_theta_err = motor_wrap_pi(s_theta_hat - s_theta_enc);
}

/**
 * @brief θ̂ 。+Ud 旋到 enc Park（脉冲开环）
 */
static void hfi_ipd_set_pulse_ud(float ud_hat)
{
    s_ipd_ov = 1u;
    if (hfi_park_uses_hat() != 0u) {
        /* Park=θ̂：脉冲直接打。d 。*/
        s_ipd_ud = ud_hat;
        s_ipd_uq = 0.0f;
    } else {
        const float th = s_theta_hat - s_theta_enc;
        const float c = cosf(th);
        const float s = sinf(th);

        s_ipd_ud = ud_hat * c;
        s_ipd_uq = ud_hat * s;
    }
    s_ud_inj = 0.0f;
    s_uq_inj = 0.0f;
    s_vh_sign = 0.0f;
}

/**
 * @brief SETTLE 结束？|i| 足够小或超时
 */
static uint8_t hfi_ipd_settle_done(void)
{
    if (s_ipd_iabs <= M1_HFI_IPD_I_TH_A) {
        return 1u;
    }
    if (s_ipd_cnt >= M1_HFI_IPD_SETTLE_MAX_N) {
        return 1u;
    }
    return 0u;
}

/**
 * @brief 相对裕度判决；失败则 DONE
 */
static void hfi_ipd_decide(void)
{
    const float p0 = s_ipd_peak0;
    const float p1 = s_ipd_peak1;
    float mx = p0;
    float dlt;
    float rel;

    if (p1 > mx) {
        mx = p1;
    }
    if (mx < 1.0e-3f) {
        mx = 1.0e-3f;
    }
    dlt = p1 - p0;
    if (dlt < 0.0f) {
        dlt = -dlt;
    }
    rel = dlt / mx;

    s_di_q = p0;
    s_eps = p1;

    /* 对比不够：拒绝起动，避免噪声明翻 */
    if (rel < M1_HFI_IPD_REL_MIN) {
        hfi_ipd_fail();
        return;
    }

    /*
     * 更大 |id| 。更易饱和侧。取峰更大一侧为 +d。
     * 若与机型定义相反，只改此处符号，勿动 ALIGN。
     */
    if (p1 > p0) {
        s_theta_hat = motor_wrap_pi(s_ipd_th0 + (float)M_PI);
        s_vh_sign = 1.0f;
    } else {
        s_theta_hat = s_ipd_th0;
        s_vh_sign = -1.0f;
    }
    s_ipd_ov = 0u;
    s_omega_el = s_ipd_peak0; /* VOFA ch8：peak0；ch4 eps=peak1 */
    s_theta_err = motor_wrap_pi(s_theta_hat - s_theta_enc);
#if M1_HFI_IPD_SWEEP_ENABLE
    s_omega_trim_el = s_ipd_pulse_ud; /* ch5：本格脉冲电。*/
    s_lock = HFI_LOCK_LOCKED; /* LOG 段：1=判决有效 */
    s_stage = HFI_STAGE_LOG;
    s_stage_t = 0.0f;
    hfi_clear_inj();
#else
    hfi_enter_run();
#endif
}

/**
 * @brief IPD 状态机（on_angle 每拍。
 */
static void hfi_ipd_on_angle(void)
{
    float ae;

    s_theta_err = motor_wrap_pi(s_theta_hat - s_theta_enc);
    s_ipd_cnt++;

    switch (s_ipd_phase) {
    case HFI_IPD_ALIGN:
        s_ipd_ov = 0u;
        hfi_set_inj_on_hat();
#if M1_HFI_IPD_ALIGN_USE_ENC
        ae = s_theta_err;
        if (ae < 0.0f) {
            ae = -ae;
        }
#else
        /* 纯无感：|eps| 准直流即认为凸极已锁 */
        ae = s_eps;
        if (ae < 0.0f) {
            ae = -ae;
        }
#endif
        /* 最。ALIGN 后，门限持续满足才冻。*/
        if (s_ipd_cnt >= M1_HFI_IPD_ALIGN_N) {
#if M1_HFI_IPD_ALIGN_USE_ENC
            if (ae < M1_HFI_IPD_ALIGN_ERR_RAD) {
#else
            if (ae < M1_HFI_IPD_ALIGN_EPS) {
#endif
                s_ipd_align_hold++;
            } else {
                s_ipd_align_hold = 0u;
            }
            if (s_ipd_align_hold >= M1_HFI_IPD_ALIGN_HOLD_N) {
                s_ipd_th0 = s_theta_hat;
                s_pll_int = 0.0f;
                s_omega_el = 0.0f;
                s_omega_trim_el = 0.0f;
                s_theta_hat = s_ipd_th0;
                s_ipd_phase = (uint8_t)HFI_IPD_SETTLE0;
                s_ipd_cnt = 0u;
                hfi_ipd_set_zero_u();
            } else if (s_ipd_cnt >= M1_HFI_IPD_ALIGN_MAX_N) {
                hfi_ipd_fail();
            }
        } else if (s_ipd_cnt >= M1_HFI_IPD_ALIGN_MAX_N) {
            hfi_ipd_fail();
        }
        break;

    case HFI_IPD_SETTLE0:
        s_theta_hat = s_ipd_th0;
        hfi_ipd_set_zero_u();
        s_di_q = s_ipd_iabs;
        if (hfi_ipd_settle_done() != 0u) {
            s_ipd_phase = (uint8_t)HFI_IPD_P0;
            s_ipd_cnt = 0u;
            s_ipd_peak0 = 0.0f;
            hfi_ipd_set_pulse_ud(hfi_ipd_pulse_ud_now());
        }
        break;

    case HFI_IPD_P0:
        s_theta_hat = s_ipd_th0;
        hfi_ipd_set_pulse_ud(hfi_ipd_pulse_ud_now());
        if (s_ipd_cnt >= M1_HFI_IPD_PULSE_N) {
            s_ipd_phase = (uint8_t)HFI_IPD_SETTLE1;
            s_ipd_cnt = 0u;
            s_di_q = s_ipd_peak0;
            hfi_ipd_set_zero_u();
        }
        break;

    case HFI_IPD_SETTLE1:
        s_theta_hat = s_ipd_th0;
        hfi_ipd_set_zero_u();
        s_eps = s_ipd_iabs;
        if (hfi_ipd_settle_done() != 0u) {
            s_theta_hat = motor_wrap_pi(s_ipd_th0 + (float)M_PI);
            s_ipd_phase = (uint8_t)HFI_IPD_P1;
            s_ipd_cnt = 0u;
            s_ipd_peak1 = 0.0f;
            hfi_ipd_set_pulse_ud(hfi_ipd_pulse_ud_now());
        }
        break;

    case HFI_IPD_P1:
    default:
        s_theta_hat = motor_wrap_pi(s_ipd_th0 + (float)M_PI);
        hfi_ipd_set_pulse_ud(hfi_ipd_pulse_ud_now());
        if (s_ipd_cnt >= M1_HFI_IPD_PULSE_N) {
            s_eps = s_ipd_peak1;
            s_di_q = s_ipd_peak0;
            hfi_ipd_decide();
        }
        break;
    }
}

/**
 * @brief IPD 电流：更。|i|；脉冲段。|id_h| 。
 */
static void hfi_ipd_on_current(float id, float iq)
{
    float iabs = id * id + iq * iq;

    if (iabs > 0.0f) {
        iabs = sqrtf(iabs);
    } else {
        iabs = 0.0f;
    }
    s_ipd_iabs = iabs;

    if ((s_ipd_phase != (uint8_t)HFI_IPD_P0) &&
        (s_ipd_phase != (uint8_t)HFI_IPD_P1)) {
        return;
    }
    {
        float id_h;
        float a;

        if (hfi_park_uses_hat() != 0u) {
            /* Park=θ̂：id 已是 hat 。*/
            id_h = id;
        } else {
            const float th = s_theta_hat - s_theta_enc;
            const float c = cosf(th);
            const float s = sinf(th);

            id_h = id * c + iq * s;
        }
        a = id_h;

        if (a < 0.0f) {
            a = -a;
        }
        if (s_ipd_phase == (uint8_t)HFI_IPD_P0) {
            if (a > s_ipd_peak0) {
                s_ipd_peak0 = a;
            }
        } else if (a > s_ipd_peak1) {
            s_ipd_peak1 = a;
        }
        s_di_q = id_h;
    }
}
#endif /* M1_HFI_POLARITY_IPD_ENABLE */

#if M1_HFI_DELTA_SWEEP_ENABLE
/**
 * @brief 进入 / 切到下一。δ（开环，不积分角。
 */
static void hfi_enter_delta_step(int16_t step_i)
{
    s_delta_i = step_i;
    s_delta_rad = hfi_delta_at_step(step_i);
    s_theta_cmd = s_delta_rad;
    s_stage = HFI_STAGE_MEAS;
    s_stage_t = 0.0f;
    s_sign = 1.0f;
    s_inj_prev_valid = 0u;
    hfi_xy_lp_reset();
    s_di_d = 0.0f;
    s_di_q = 0.0f;
    s_x_raw = 0.0f;
    s_y_raw = 0.0f;
    s_eps = 0.0f;
#if M1_HFI_AXIS_SEL_ENABLE
    s_eps_d_lp = 0.0f;
    s_axis_lock_n = 0u;
    s_axis_good_n = 0u;
    s_axis_cd = 0u;
    s_axis_ok = 0u;
#endif
    s_pll_int = 0.0f;
    s_polarity_cnt = 0u;
    s_omega_el = 0.0f;
    s_omega_trim_el = 0.0f;
    s_theta_hat = motor_wrap_pi(s_theta_enc + s_delta_rad);
    s_theta_err = s_delta_rad;
}

static void hfi_enter_a0_move(void)
{
    const float step = M1_HFI_A0_STEP_DEG * (float)M_PI / 180.0f;

    s_theta_cmd = motor_wrap_pi((float)s_a0_pos * step);
    s_theta_hat = s_theta_cmd;
    s_theta_err = motor_wrap_pi(s_theta_hat - s_theta_enc);
    s_stage = HFI_STAGE_MOVE;
    s_stage_t = 0.0f;
    hfi_clear_inj();
    s_pll_int = 0.0f;
    s_omega_el = 0.0f;
    s_omega_trim_el = 0.0f;
}
#endif


#if M1_HFI_IPD_SWEEP_ENABLE
/**
 * @brief Ud 摆位到下一电角度格子（不经 HFI ALIGN。
 */
static void hfi_ipd_sweep_enter_move(void)
{
    const float step = (2.0f * (float)M_PI) / (float)M1_HFI_IPD_POS_N;

    s_ipd_pulse_ud = hfi_ipd_pulse_ud_now();
    s_theta_cmd = motor_wrap_pi((float)s_ipd_pos_i * step);
    s_theta_hat = s_theta_cmd;
    s_theta_err = motor_wrap_pi(s_theta_hat - s_theta_enc);
    s_stage = HFI_STAGE_MOVE;
    s_stage_t = 0.0f;
    s_ipd_move_cnt = 0u;
    s_ipd_ov = 1u;
    s_ipd_ud = M1_HFI_IPD_MOVE_UD_V;
    s_ipd_uq = 0.0f;
    hfi_clear_inj();
    s_lock = HFI_LOCK_CAPTURE;
    s_vh_sign = 0.0f;
    s_eps = 0.0f;
    s_di_q = 0.0f;
    s_omega_el = 0.0f;
    s_omega_trim_el = s_ipd_pulse_ud;
    s_ipd_peak0 = 0.0f;
    s_ipd_peak1 = 0.0f;
}
#endif

#if M1_HFI_QKICK_ANY
static void hfi_qk_set_zero_u(void)
{
    s_qk_ov = 1u;
    s_qk_ud = 0.0f;
    s_qk_uq = 0.0f;
    s_qk_iq_ref = 0.0f;
    s_qk_id_ref = 0.0f;
    hfi_clear_inj();
}

#if M1_HFI_QKICK_SWEEP_ENABLE
static uint8_t hfi_qk_settle_done(void)
{
    if (s_qk_iabs <= M1_HFI_QKICK_I_TH_A) {
        return 1u;
    }
    if (s_qk_cnt >= M1_HFI_QKICK_SETTLE_MAX_N) {
        return 1u;
    }
    return 0u;
}

static void hfi_qk_decide(void)
{
    const float expect = (s_qk_seed_i == 0u) ? 1.0f : -1.0f;
    float ae = s_qk_dth;

    if (ae < 0.0f) {
        ae = -ae;
    }
    if (ae < M1_HFI_QKICK_DTH_MIN_RAD) {
        s_qk_verdict = 0.0f;
        s_lock = HFI_LOCK_FAULT;
    } else if ((s_qk_dth * expect) > 0.0f) {
        s_qk_verdict = 1.0f;
        s_lock = HFI_LOCK_LOCKED;
    } else {
        s_qk_verdict = -1.0f;
        s_lock = HFI_LOCK_FAULT;
    }
    s_eps = s_qk_dth;
    s_di_q = s_qk_verdict;
    s_vh_sign = expect;
    s_omega_trim_el = M1_HFI_QKICK_IQ_A;
}

static void hfi_qk_enter_move(void)
{
    const float step = (2.0f * (float)M_PI) / (float)M1_HFI_QKICK_POS_N;

    s_theta_cmd = motor_wrap_pi((float)s_qk_pos_i * step);
    s_theta_hat = s_theta_cmd;
    s_theta_err = motor_wrap_pi(s_theta_hat - s_theta_enc);
    s_stage = HFI_STAGE_MOVE;
    s_stage_t = 0.0f;
    s_qk_move_cnt = 0u;
    s_qk_cnt = 0u;
    s_qk_phase = (uint8_t)HFI_QK_SEED;
    s_qk_ov = 1u;
    s_qk_ud = M1_HFI_QKICK_MOVE_UD_V;
    s_qk_uq = 0.0f;
    s_qk_iq_ref = 0.0f;
    s_qk_dth = 0.0f;
    s_qk_verdict = 0.0f;
    s_qk_enc_prev_valid = 0u;
    hfi_clear_inj();
    s_lock = HFI_LOCK_CAPTURE;
    s_vh_sign = (s_qk_seed_i == 0u) ? 1.0f : -1.0f;
    s_eps = 0.0f;
    s_di_q = 0.0f;
    s_omega_el = 0.0f;
    s_omega_trim_el = M1_HFI_QKICK_IQ_A;
}
#endif /* QKICK_SWEEP */

#if M1_HFI_QKICK_AFTER_LOCK_ENABLE
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
        s_qk_verdict = 0.0f;
        s_lock = HFI_LOCK_FAULT;
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

#if M1_HFI_DQ_IDENT_ENABLE
static float hfi_dq_median_rel(const float *x, uint16_t n)
{
    float t[32];
    float v;
    float x0;
    uint16_t i;
    uint16_t j;

    if ((n == 0u) || (x == NULL)) {
        return 0.0f;
    }
    if (n > 32u) {
        n = 32u;
    }
    x0 = x[0];
    for (i = 0u; i < n; i++) {
        t[i] = motor_wrap_pi(x[i] - x0);
    }
    for (i = 1u; i < n; i++) {
        v = t[i];
        j = i;
        while ((j > 0u) && (t[j - 1u] > v)) {
            t[j] = t[j - 1u];
            j--;
        }
        t[j] = v;
    }
    return motor_wrap_pi(x0 + t[n / 2u]);
}

static void hfi_dq_med_reset(void)
{
    s_dq_med_n = 0u;
}

static void hfi_dq_med_push(float enc)
{
    if (s_dq_med_n < M1_HFI_DQ_MED_N) {
        s_dq_med_buf[s_dq_med_n] = enc;
        s_dq_med_n++;
    }
}

static float hfi_dq_med_value(void)
{
    return hfi_dq_median_rel(s_dq_med_buf, s_dq_med_n);
}

static void hfi_dq_enter_sentinel(void)
{
    s_dq_step = (uint8_t)HFI_DQ_SENTINEL;
    s_stage = HFI_STAGE_RUN;
    s_stage_t = 0.0f;
    s_qk_id_ref = 0.0f;
    s_qk_iq_ref = M1_HFI_DQ_SENTINEL_A;
    s_qk_done = 0u;
}

static void hfi_dq_enter_pulse(uint8_t axis)
{
    s_dq_axis = axis;
    s_qk_done = 0u;
    s_qk_seed_i = 0u;
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
    s_qk_id_ref = 0.0f;
    s_qk_ov = 0u;
    hfi_dq_med_reset();
    s_dq_enc0 = s_theta_enc;
    s_omega_trim_el = M1_HFI_QKICK_IQ_A;
    s_vh_sign = 1.0f;
}

static void hfi_dq_enter_pulse(uint8_t axis);
static void hfi_dq_enter_sentinel(void);
static void hfi_dq_decide(void);
static void hfi_dq_on_pulse_done(void);

static void hfi_dq_ramp_vh(float t_on)
{
    if (t_on >= M1_HFI_DQ_VH_RAMP_S) {
        s_vh_v = M1_HFI_VH_V;
    } else if (t_on <= 0.0f) {
        s_vh_v = 0.0f;
    } else {
        s_vh_v = M1_HFI_VH_V * (t_on / M1_HFI_DQ_VH_RAMP_S);
    }
}

static uint8_t hfi_dq_ident_on_run(void)
{
    float ae;

    ae = s_eps;
    if (ae < 0.0f) {
        ae = -ae;
    }

    switch (s_dq_step) {
    case HFI_DQ_BLANK:
        hfi_dq_ramp_vh(s_stage_t);
        if (s_stage_t >= M1_HFI_DQ_BLANK_S) {
            s_dq_step = (uint8_t)HFI_DQ_WAIT_EPS;
            s_stage_t = 0.0f;
            s_dq_good_t = 0.0f;
            s_dq_enc0_ok = 0u;
            s_vh_v = M1_HFI_VH_V;
            hfi_dq_med_reset();
        }
        return 0u;

    case HFI_DQ_WAIT_EPS:
        s_vh_v = M1_HFI_VH_V;
        /* 本枪只看 ε/x_lp：满 LOCK_TMO 。DONE，提前锁也不打脉。*/
        if (s_stage_t >= M1_HFI_DQ_LOCK_TMO_S) {
            s_dq_step = (uint8_t)HFI_DQ_FAIL;
            s_qk_id_ref = 0.0f;
            s_qk_iq_ref = 0.0f;
            hfi_qk_set_zero_u();
            s_stage = HFI_STAGE_DONE;
            s_stage_t = 0.0f;
            return 1u;
        }
        if (ae > M1_HFI_DQ_EPS_MAX_RAD) {
            s_dq_good_t = 0.0f;
        } else {
            s_dq_good_t += OBS_CTRL_TS_S;
        }
        return 0u;

    case HFI_DQ_GAP:
        s_qk_id_ref = 0.0f;
        s_qk_iq_ref = 0.0f;
        if (s_stage_t >= M1_HFI_DQ_GAP_S) {
            s_dq_step = (uint8_t)HFI_DQ_PULSE_Q;
            hfi_dq_enter_pulse(1u);
            return 1u;
        }
        return 0u;

    case HFI_DQ_SENTINEL:
        s_qk_id_ref = 0.0f;
        s_qk_iq_ref = M1_HFI_DQ_SENTINEL_A;
        if (s_stage_t >= M1_HFI_DQ_SENTINEL_S) {
            s_dq_step = (uint8_t)HFI_DQ_SPEED;
            s_qk_iq_ref = 0.0f;
            s_qk_done = 1u;
            s_stage_t = 0.0f;
        }
        return 0u;

    case HFI_DQ_SPEED:
        s_qk_id_ref = 0.0f;
        s_qk_iq_ref = 0.0f;
        if (s_stage_t >= M1_HFI_RUN_RPM1_S) {
            s_stage = HFI_STAGE_DONE;
            s_stage_t = 0.0f;
            hfi_qk_set_zero_u();
            return 1u;
        }
        return 0u;

    default:
        return 0u;
    }
}

static void hfi_dq_decide(void)
{
    float ad;
    float aq;
    uint8_t d_move;
    uint8_t q_move;

    ad = s_dq_dth_d;
    aq = s_dq_dth_q;
    if (ad < 0.0f) {
        ad = -ad;
    }
    if (aq < 0.0f) {
        aq = -aq;
    }
    d_move = (ad >= M1_HFI_DQ_MOVE_RAD) ? 1u : 0u;
    q_move = (aq >= M1_HFI_DQ_MOVE_RAD) ? 1u : 0u;
    if ((d_move != 0u) && (aq <= M1_HFI_DQ_STILL_RAD) && (q_move == 0u)) {
        s_park_off = (s_dq_dth_d >= 0.0f) ? (-0.5f * (float)M_PI)
                                          : (0.5f * (float)M_PI);
    } else if ((q_move != 0u) && (ad <= M1_HFI_DQ_STILL_RAD) && (d_move == 0u)) {
        s_park_off = (s_dq_dth_q >= 0.0f) ? 0.0f : (float)M_PI;
    } else {
        s_park_off = 0.0f; /* 判不出也起速，Δ=0 */
    }
    hfi_dq_enter_sentinel();
}

static void hfi_dq_on_pulse_done(void)
{
    float enc1;
    float dth;

    enc1 = hfi_dq_med_value();
    dth = motor_wrap_pi(enc1 - s_dq_enc0);
    s_qk_dth = dth;
    if (s_dq_axis == 0u) {
        s_dq_dth_d = dth;
        s_dq_step = (uint8_t)HFI_DQ_GAP;
        s_stage = HFI_STAGE_RUN;
        s_stage_t = 0.0f;
        s_qk_id_ref = 0.0f;
        s_qk_iq_ref = 0.0f;
    } else {
        s_dq_dth_q = dth;
        hfi_dq_decide();
    }
}
#endif /* DQ_IDENT */

#if M1_HFI_QKICK_START_ENABLE
static float hfi_eps_dead_at(uint8_t i)
{
    switch (i) {
    case 0u:
        return M1_HFI_PLL_EPS_DEAD_0;
    case 1u:
        return M1_HFI_PLL_EPS_DEAD_1;
    case 2u:
        return M1_HFI_PLL_EPS_DEAD_2;
    case 3u:
        return M1_HFI_PLL_EPS_DEAD_3;
    case 4u:
        return M1_HFI_PLL_EPS_DEAD_4;
    case 5u:
        return M1_HFI_PLL_EPS_DEAD_5;
    default:
        return M1_HFI_PLL_EPS_DEAD;
    }
}

static float hfi_start_vh_at(uint8_t i)
{
    switch (i) {
    case 0u:
        return M1_HFI_START_LADDER_VH_0;
    case 1u:
        return M1_HFI_START_LADDER_VH_1;
    case 2u:
        return M1_HFI_START_LADDER_VH_2;
    case 3u:
        return M1_HFI_START_LADDER_VH_3;
    case 4u:
        return M1_HFI_START_LADDER_VH_4;
    case 5u:
        return M1_HFI_START_LADDER_VH_5;
    default:
        return M1_HFI_VH_V;
    }
}

static void hfi_start_apply_eps_dead(void)
{
#if M1_HFI_PLL_EPS_DEAD_SWEEP_ENABLE
    if (s_eps_dead_i >= (uint8_t)M1_HFI_PLL_EPS_DEAD_SWEEP_N) {
        s_eps_dead_i = (uint8_t)(M1_HFI_PLL_EPS_DEAD_SWEEP_N - 1u);
    }
    s_pll_eps_dead = hfi_eps_dead_at(s_eps_dead_i);
    s_vh_v = hfi_start_vh_at(s_eps_dead_i);
#else
    s_pll_eps_dead = M1_HFI_PLL_EPS_DEAD;
    s_vh_v = M1_HFI_VH_V;
#endif
    s_pll_int = 0.0f;
    s_omega_el = 0.0f;
}

static void hfi_start_enter_hold(void)
{
    s_stage = HFI_STAGE_CRAWL;
    s_stage_t = 0.0f;
    s_start_phase = (uint8_t)HFI_START_HOLD;
    s_start_run_armed = 0u;
    s_start_rpm_cmd = 0.0f;
    s_qk_iq_ref = 0.0f;
    s_qk_ov = 0u;
    s_qk_pi_reset = 1u;
    s_inj_prev_valid = 0u;
    s_sign = 1.0f;
    hfi_xy_lp_reset();
    s_omega_trim_el = 0.0f;
    s_start_ang_int = 0.0f;
    s_start_enc_prev_valid = 0u;
    s_start_wenc_rpm = 0.0f;
    s_start_exit_n = 0u;
    s_start_iq_sign = 1.0f;
    s_start_flip_done = 0u;
    s_start_flip_n = 0u;
    s_gate_light_n = 0u;
    s_gate_heavy_n = 0u;
    s_gate_fake_n = 0u;
    hfi_start_apply_eps_dead();
    s_theta_cmd = s_theta_hat;
    s_theta_err = motor_wrap_pi(s_theta_hat - s_theta_enc);
    s_eps = s_qk_dth;
    s_di_q = s_qk_verdict;
    if (s_qk_verdict != 0.0f) {
        s_lock = HFI_LOCK_LOCKED;
    }
}

static void hfi_start_enter_capture(void)
{
    s_start_phase = (uint8_t)HFI_START_CAPTURE;
    s_stage_t = 0.0f;
    s_start_rpm_cmd = 0.0f;
    s_qk_iq_ref = 0.0f;
    s_qk_pi_reset = 1u;
    s_theta_cmd = s_theta_hat; /* θ_ref 从当。θ̂ 起步 */
    s_start_ang_int = 0.0f;
    s_start_enc_prev_valid = 0u;
    s_start_wenc_rpm = 0.0f;
    s_start_exit_n = 0u;
    s_start_flip_n = 0u;
    s_gate_light_n = 0u;
    s_gate_heavy_n = 0u;
    s_gate_fake_n = 0u;
    /* 。CAPTURE 立即出力，勿继承上一门的 cool（否则破粘窗口被吃掉。*/
    s_start_cool_s = 0.0f;
#if M1_HFI_BIAS_CAL_ENABLE
    s_theta_bias = 0.0f;
    s_bias_frozen = 0u;
#endif
}

#if M1_HFI_BIAS_CAL_ENABLE
/**
 * @brief 影子标定：爬速后。δ；冻结时一次。θ̂。δ。054：只。Park 会崩 PLL。
 */
static void hfi_bias_cal_step(float t_cap)
{
    float aw;
    float e;
    const float t_learn0 = M1_HFI_QKICK_START_CAP_RAMP_S;
    const float t_freeze = t_learn0 + M1_HFI_BIAS_CAL_LEARN_S;

    if (s_bias_frozen != 0u) {
        return;
    }

    aw = s_start_wenc_rpm;
    if (aw < 0.0f) {
        aw = -aw;
    }

    if ((t_cap >= t_learn0) && (aw >= M1_HFI_BIAS_CAL_WMIN_RPM)) {
        e = motor_wrap_pi(s_theta_hat - s_theta_enc);
        s_theta_bias += M1_HFI_BIAS_CAL_LP * (e - s_theta_bias);
    }

    if (t_cap >= t_freeze) {
        /* 扳估计角，Park/注入/PLL 仍共轴；清半周避。di_q 假阶。*/
        s_theta_hat = motor_wrap_pi(s_theta_hat - s_theta_bias);
        s_theta_err = motor_wrap_pi(s_theta_hat - s_theta_enc);
        s_inj_prev_valid = 0u;
        hfi_xy_lp_reset();
        s_eps = 0.0f;
        s_bias_frozen = 1u;
    }
}
#endif

static void hfi_start_enter_handover(void)
{
    s_start_phase = (uint8_t)HFI_START_HANDOVER;
    s_stage_t = 0.0f;
    s_gate_light_n = 0u;
    s_gate_heavy_n = 0u;
    s_gate_fake_n = 0u;
}

/**
 * @brief 台架真速：。enc 差分低通，。CAPTURE 出门门控
 */
static void hfi_start_update_wenc(void)
{
    float dth;
    float w_rpm;
    const float rpm_scale =
        60.0f / (2.0f * (float)M_PI * (float)OBS_POLE_PAIRS);

    if (s_start_enc_prev_valid == 0u) {
        s_start_enc_prev = s_theta_enc;
        s_start_enc_prev_valid = 1u;
        s_start_wenc_rpm = 0.0f;
        return;
    }
    dth = motor_wrap_pi(s_theta_enc - s_start_enc_prev);
    s_start_enc_prev = s_theta_enc;
    w_rpm = (dth / OBS_CTRL_TS_S) * rpm_scale;
    /* ~50 Hz 一阶：α。.015 @20 kHz */
    s_start_wenc_rpm += 0.015f * (w_rpm - s_start_wenc_rpm);
}

/**
 * @brief CAPTURE 力矩：短促破粘后小维持；Iq = Iq_cmd 。Bv·ω，限。±峰。
 * @note BV_USE_ENC=1：台架用真速当 B，Park 仍为 θ̂。：吃 ω̂（产品形态）
 */
static void hfi_start_damped_iq_step(void)
{
    float iq_cmd_abs;
    float iq_cmd;
    float w_rpm;
    float iq;
    float lim;
    const float rpm_scale =
        60.0f / (2.0f * (float)M_PI * (float)OBS_POLE_PAIRS);

    if ((M1_HFI_QKICK_START_CAP_IQ_KICK_S > 0.0f) &&
        (s_stage_t < M1_HFI_QKICK_START_CAP_IQ_KICK_S)) {
        iq_cmd_abs = M1_HFI_QKICK_START_CAP_IQ_KICK_A;
    } else {
        iq_cmd_abs = M1_HFI_QKICK_START_CAP_IQ_HOLD_A;
    }
    lim = M1_HFI_QKICK_START_CAP_IQ_A;
    if (iq_cmd_abs > lim) {
        lim = iq_cmd_abs;
    }
    iq_cmd = s_start_iq_sign * iq_cmd_abs;

#if M1_HFI_QKICK_START_BV_USE_ENC
    (void)rpm_scale;
    w_rpm = s_start_wenc_rpm;
#else
    w_rpm = s_omega_el * rpm_scale;
#endif
    iq = iq_cmd - M1_HFI_QKICK_START_BV_A_RPM * w_rpm;
    iq = motor_clampf(iq, lim);
    s_qk_iq_ref = iq;
    s_omega_trim_el = iq;
}

static void hfi_start_maybe_flip_iq(void)
{
    float w = s_start_wenc_rpm;

    if (s_start_flip_done != 0u) {
        return;
    }
    /* 真速与 Iq 命令反号且够大：翻一。Iq（极。Park 。enc 不一致） */
    if ((s_start_iq_sign * w) < (-M1_HFI_QKICK_START_FLIP_RPM)) {
        s_start_flip_n++;
        if (s_start_flip_n >= M1_HFI_QKICK_START_FLIP_N) {
            s_start_iq_sign = -s_start_iq_sign;
            s_start_flip_done = 1u;
            s_start_flip_n = 0u;
        }
    } else {
        s_start_flip_n = 0u;
    }
}

static void hfi_start_finish_no_run(void)
{
    s_qk_iq_ref = 0.0f;
    s_start_rpm_cmd = 0.0f;
    s_start_run_armed = 0u;
#if M1_HFI_PLL_EPS_DEAD_SWEEP_ENABLE
    /* 下一档死区再跑一。CAPTURE，避免反复上。*/
    if ((uint16_t)s_eps_dead_i + 1u < (uint16_t)M1_HFI_PLL_EPS_DEAD_SWEEP_N) {
        s_eps_dead_i++;
        s_start_cool_s = M1_HFI_QKICK_START_COOL_S;
        hfi_start_enter_hold();
        return;
    }
#endif
    s_stage = HFI_STAGE_DONE;
    s_stage_t = 0.0f;
    hfi_clear_inj();
    s_omega_el = 0.0f;
    s_omega_trim_el = 0.0f;
    s_pll_int = 0.0f;
}

/**
 * @brief 分级门控：假。重门只软冷却（Iq=0、清积分），不清 CAPTURE 计时
 * @return 1=本拍已软冷却（上层勿再推 Iq）；0=继续
 * @note 2025：假速→enter_hold 会清 stage_t，扫档卡。dead=0 死循。
 */
static uint8_t hfi_start_gate_step(void)
{
    float ae = s_eps;
    float w_rpm;
    float pint_rpm;
    float int_lim_rpm;
    const float rpm_scale =
        60.0f / (2.0f * (float)M_PI * (float)OBS_POLE_PAIRS);

    if (ae < 0.0f) {
        ae = -ae;
    }
    w_rpm = s_omega_el * rpm_scale;
    if (w_rpm < 0.0f) {
        w_rpm = -w_rpm;
    }
    pint_rpm = s_pll_int * rpm_scale;
    if (pint_rpm < 0.0f) {
        pint_rpm = -pint_rpm;
    }
    int_lim_rpm = M1_HFI_PLL_INT_MAX * rpm_scale;

    {
        float dead_gate = s_pll_eps_dead;

        if (dead_gate <= 0.0f) {
            dead_gate = 0.02f;
        }
        if ((ae < dead_gate) && (pint_rpm > (0.85f * int_lim_rpm))) {
            s_gate_fake_n++;
            if (s_gate_fake_n >= M1_HFI_QKICK_GATE_FAKE_N) {
                s_pll_int = 0.0f;
                s_omega_el = 0.0f;
                s_qk_iq_ref = 0.0f;
                s_gate_fake_n = 0u;
                /* 软冷却：留在 CAPTURE，stage_t 继续。*/
                s_start_cool_s = M1_HFI_QKICK_START_COOL_S;
                return 1u;
            }
        } else {
            s_gate_fake_n = 0u;
        }
    }

    if (ae > M1_HFI_QKICK_GATE_EPS_LIGHT) {
        s_gate_light_n++;
        if (s_gate_light_n >= M1_HFI_QKICK_GATE_LIGHT_N) {
            s_pll_int = motor_clampf(s_pll_int, 0.5f * M1_HFI_PLL_INT_MAX);
            s_gate_light_n = 0u;
        }
    } else {
        s_gate_light_n = 0u;
    }

    if ((ae > M1_HFI_QKICK_GATE_EPS_HEAVY) &&
        (w_rpm > M1_HFI_QKICK_GATE_W_RPM_HEAVY)) {
        s_gate_heavy_n++;
        if (s_gate_heavy_n >= M1_HFI_QKICK_GATE_HEAVY_N) {
            s_pll_int = 0.0f;
            s_omega_el = 0.0f;
            s_qk_iq_ref = 0.0f;
            s_gate_heavy_n = 0u;
            s_start_cool_s = M1_HFI_QKICK_START_COOL_S;
            return 1u;
        }
    } else {
        s_gate_heavy_n = 0u;
    }
    return 0u;
}

static void hfi_start_on_crawl(void)
{
    float w_el;
    float seed_rpm;
    float t_cap;
    float aw;

    hfi_set_inj_on_hat();
    s_theta_err = motor_wrap_pi(hfi_theta_hat_out() - s_theta_enc);
    /* 保留解调 eps（勿用踢。dth 覆盖，否则门。VOFA 都是。eps。*/
    s_di_q = s_qk_verdict;
    hfi_start_update_wenc();

    if (s_start_cool_s > 0.0f) {
        s_start_cool_s -= OBS_CTRL_TS_S;
        if (s_start_cool_s < 0.0f) {
            s_start_cool_s = 0.0f;
        }
        s_qk_iq_ref = 0.0f;
        s_start_rpm_cmd = 0.0f;
        s_theta_cmd = s_theta_hat;
        return;
    }

    if ((s_start_phase == HFI_START_CAPTURE) ||
        (s_start_phase == HFI_START_HANDOVER)) {
        if (hfi_start_gate_step() != 0u) {
            return;
        }
    }

    switch (s_start_phase) {
    case HFI_START_HOLD:
        s_qk_iq_ref = 0.0f;
        s_start_rpm_cmd = 0.0f;
        s_theta_cmd = s_theta_hat;
        s_omega_trim_el = 0.0f;
        if (s_stage_t >= M1_HFI_QKICK_START_HOLD_S) {
            hfi_start_enter_capture();
        }
        break;

    case HFI_START_CAPTURE:
        t_cap = s_stage_t;
        if (t_cap < M1_HFI_QKICK_START_CAP_RAMP_S) {
            s_start_rpm_cmd = M1_HFI_QKICK_START_CAP_RPM *
                (t_cap / M1_HFI_QKICK_START_CAP_RAMP_S);
        } else {
            s_start_rpm_cmd = M1_HFI_QKICK_START_CAP_RPM;
        }
        /* VOFA：θ_cmd 跟指令斜坡；力矩不跟角差。Park=θ̂；。0 Type-II */
        w_el = hfi_omega_el_from_rpm(s_start_rpm_cmd);
        s_theta_cmd = motor_wrap_pi(s_theta_cmd + w_el * OBS_CTRL_TS_S);
        hfi_start_damped_iq_step();
        hfi_start_maybe_flip_iq();
#if M1_HFI_BIAS_CAL_ENABLE
        hfi_bias_cal_step(t_cap);
#endif

        aw = s_start_wenc_rpm;
        if (aw < 0.0f) {
            aw = -aw;
        }
        if ((t_cap >= 1.0f) && (aw >= M1_HFI_QKICK_START_EXIT_RPM)) {
            if (s_start_exit_n < 0xFFFFu) {
                s_start_exit_n++;
            }
        } else {
            s_start_exit_n = 0u;
        }

        if (t_cap >= (M1_HFI_QKICK_START_CAP_RAMP_S +
                      M1_HFI_QKICK_START_CAP_HOLD_S)) {
#if M1_HFI_QKICK_START_ARM_RUN
            if (s_start_exit_n >= M1_HFI_QKICK_START_EXIT_N) {
                hfi_start_enter_handover();
            } else {
                s_start_cool_s = M1_HFI_QKICK_START_COOL_S;
                hfi_start_enter_hold();
            }
#else
            (void)aw;
            hfi_start_finish_no_run();
#endif
        }
        break;

    case HFI_START_HANDOVER:
        /* 保持阻尼 Iq；种子速度用真速（台架。*/
        s_start_rpm_cmd = M1_HFI_QKICK_START_CAP_RPM;
        w_el = hfi_omega_el_from_rpm(s_start_rpm_cmd);
        s_theta_cmd = motor_wrap_pi(s_theta_cmd + w_el * OBS_CTRL_TS_S);
        hfi_start_damped_iq_step();
        if (s_stage_t >= M1_HFI_QKICK_START_HANDOVER_S) {
            aw = s_start_wenc_rpm;
            if (aw < 0.0f) {
                aw = -aw;
            }
            if (aw >= M1_HFI_QKICK_START_EXIT_RPM) {
                seed_rpm = s_start_wenc_rpm;
            } else {
                seed_rpm = M1_HFI_QKICK_START_CAP_RPM;
            }
            w_el = hfi_omega_el_from_rpm(seed_rpm);
            s_pll_int = w_el;
            s_omega_el = w_el;
            s_start_run_armed = 1u;
            s_qk_iq_ref = 0.0f;
            s_stage = HFI_STAGE_RUN;
            s_stage_t = 0.0f;
            s_qk_pi_reset = 1u;
            s_lock = HFI_LOCK_LOCKED;
            s_lock_cnt = 0u;
            s_gate_light_n = 0u;
            s_gate_heavy_n = 0u;
            s_gate_fake_n = 0u;
        }
        break;

    default:
        hfi_start_enter_hold();
        break;
    }
}
#endif

#if M1_HFI_QKICK_CRAWL_ENABLE
/**
 * @brief CRAWL 段：Step3 Iq 阶跃 / Step4 。I–f / 常。Iq
 * @return 当前 Iq_ref；Step4 同时更新 s_if_* / s_theta_cmd
 */
static float hfi_qk_crawl_iq_ref(float t)
{
#if M1_HFI_QKICK_IF_ENABLE
#if M1_HFI_QKICK_IF_LADDER_ENABLE
    const float lad_iq[6] = {
        M1_HFI_QKICK_IF_LAD_IQ0, M1_HFI_QKICK_IF_LAD_IQ1,
        M1_HFI_QKICK_IF_LAD_IQ2, M1_HFI_QKICK_IF_LAD_IQ3,
        M1_HFI_QKICK_IF_LAD_IQ4, M1_HFI_QKICK_IF_LAD_IQ5
    };
    const float lad_rpm[6] = {
        M1_HFI_QKICK_IF_LAD_RPM0, M1_HFI_QKICK_IF_LAD_RPM1,
        M1_HFI_QKICK_IF_LAD_RPM2, M1_HFI_QKICK_IF_LAD_RPM3,
        M1_HFI_QKICK_IF_LAD_RPM4, M1_HFI_QKICK_IF_LAD_RPM5
    };
    const int n_lad = (int)M1_HFI_QKICK_IF_LADDER_N;
    const float t_pre = M1_HFI_QKICK_IF_PRE_S;
    const float t_ramp = M1_HFI_QKICK_IF_LADDER_RAMP_S;
    const float t_hold = M1_HFI_QKICK_IF_LADDER_HOLD_S;
    const float t_post = M1_HFI_QKICK_IF_POST_S;
    const float seg = t_ramp + t_hold;
    const float t_if_end = t_pre + seg * (float)n_lad;
    const float t_post_end = t_if_end + t_post;
    float rpm = 0.0f;
    float iq = M1_HFI_QKICK_IF_IQ_A;
    float w_el;
    float tloc;
    float u;
    float rpm_prev;
    int i;

    if (t < t_pre) {
        s_if_active = 0u;
        s_if_rpm_cmd = 0.0f;
        s_theta_cmd = s_theta_hat;
        return iq;
    }

    if (t < t_if_end) {
        i = (int)((t - t_pre) / seg);
        if (i < 0) {
            i = 0;
        }
        if (i >= n_lad) {
            i = n_lad - 1;
        }
        tloc = (t - t_pre) - seg * (float)i;
        rpm_prev = (i <= 0) ? 0.0f : lad_rpm[i - 1];
        if (tloc < t_ramp) {
            u = (t_ramp > 1.0e-6f) ? (tloc / t_ramp) : 1.0f;
            rpm = rpm_prev + (lad_rpm[i] - rpm_prev) * u;
        } else {
            rpm = lad_rpm[i];
        }
        iq = lad_iq[i];
        if (s_if_active == 0u) {
            s_theta_cmd = s_theta_hat;
            s_if_active = 1u;
            s_qk_pi_reset = 1u;
        }
        s_if_rpm_cmd = rpm;
        w_el = hfi_omega_el_from_rpm(rpm);
        s_theta_cmd = motor_wrap_pi(s_theta_cmd + w_el * OBS_CTRL_TS_S);
        return iq;
    }

    if (s_if_active != 0u) {
        s_theta_hat = s_theta_cmd;
        s_pll_int = motor_clampf(hfi_omega_el_from_rpm(s_if_rpm_cmd), M1_HFI_PLL_INT_MAX);
        s_omega_el = s_pll_int;
        s_if_active = 0u;
#if M1_HFI_PLL_HOLD_ENABLE
        s_pll_hold = 0u;
        s_retrack_n = 0u;
#endif
    }
    s_if_rpm_cmd = 0.0f;
    s_theta_cmd = s_theta_hat;
    if (t < t_post_end) {
        return M1_HFI_QKICK_IF_IQ_A;
    }
    return 0.0f;
#else
    const float t_pre = M1_HFI_QKICK_IF_PRE_S;
    const float t_ramp = M1_HFI_QKICK_IF_RAMP_S;
    const float t_hold = M1_HFI_QKICK_IF_HOLD_S;
    const float t_post = M1_HFI_QKICK_IF_POST_S;
    const float t1 = t_pre;
    const float t2 = t1 + t_ramp;
    const float t3 = t2 + t_hold;
    const float t4 = t3 + t_post;
    float rpm = 0.0f;
    float w_el;

    if (t < t1) {
        /* PRE：Park=θ̂ TRACK */
        s_if_active = 0u;
        s_if_rpm_cmd = 0.0f;
        s_theta_cmd = s_theta_hat;
        return M1_HFI_QKICK_IF_IQ_A;
    }

    if (t < t3) {
        if (t < t2) {
            const float u = (t_ramp > 1.0e-6f) ? ((t - t1) / t_ramp) : 1.0f;
            rpm = M1_HFI_QKICK_IF_TARGET_RPM * u;
        } else {
            rpm = M1_HFI_QKICK_IF_TARGET_RPM;
        }
        if (s_if_active == 0u) {
            s_theta_cmd = s_theta_hat;
            s_if_active = 1u;
            s_qk_pi_reset = 1u;
        }
        s_if_rpm_cmd = rpm;
        w_el = hfi_omega_el_from_rpm(rpm);
        s_theta_cmd = motor_wrap_pi(s_theta_cmd + w_el * OBS_CTRL_TS_S);
        return M1_HFI_QKICK_IF_IQ_A;
    }

    if (s_if_active != 0u) {
        /* 定时交接：θ。。θ_if，与 enc 无关 */
        s_theta_hat = s_theta_cmd;
        s_pll_int = motor_clampf(hfi_omega_el_from_rpm(s_if_rpm_cmd), M1_HFI_PLL_INT_MAX);
        s_omega_el = s_pll_int;
        s_if_active = 0u;
#if M1_HFI_PLL_HOLD_ENABLE
        s_pll_hold = 0u;
        s_retrack_n = 0u;
#endif
    }
    s_if_rpm_cmd = 0.0f;
    s_theta_cmd = s_theta_hat;
    if (t < t4) {
        return M1_HFI_QKICK_IF_IQ_A;
    }
    return 0.0f;
#endif
#elif M1_HFI_QKICK_IQ_STEP_ENABLE
    const float seg = M1_HFI_QKICK_IQ_STEP_SEG_S;
    const float a = M1_HFI_QKICK_CRAWL_IQ_A;
    int i;

    if (seg <= 0.0f) {
        return 0.0f;
    }
    i = (int)(t / seg);
    if (i < 0) {
        i = 0;
    }
    /* 0 。+A 。0 。−A 。0 */
    switch (i) {
    case 1:
        return a;
    case 3:
        return -a;
    default:
        return 0.0f;
    }
#elif M1_HFI_QKICK_IQ_SLOW_ENABLE
    {
        const float a0 = M1_HFI_QKICK_IQ_SLOW_A0;
        const float a1 = M1_HFI_QKICK_IQ_SLOW_A1;
        const float a2 = M1_HFI_QKICK_IQ_SLOW_A2;
        const float t0 = M1_HFI_QKICK_IQ_SLOW_HOLD0_S;
        const float t1 = t0 + M1_HFI_QKICK_IQ_SLOW_RAMP1_S;
        const float t2 = t1 + M1_HFI_QKICK_IQ_SLOW_HOLD1_S;
        const float t3 = t2 + M1_HFI_QKICK_IQ_SLOW_RAMP2_S;
        const float t4 = t3 + M1_HFI_QKICK_IQ_SLOW_HOLD2_S;
        float u;
        float dt;

        if (t < t0) {
            return a0;
        }
        if (t < t1) {
            dt = M1_HFI_QKICK_IQ_SLOW_RAMP1_S;
            u = (dt > 1.0e-6f) ? ((t - t0) / dt) : 1.0f;
            return a0 + (a1 - a0) * u;
        }
        if (t < t2) {
            return a1;
        }
        if (t < t3) {
            dt = M1_HFI_QKICK_IQ_SLOW_RAMP2_S;
            u = (dt > 1.0e-6f) ? ((t - t2) / dt) : 1.0f;
            return a1 + (a2 - a1) * u;
        }
        if (t < t4) {
            return a2;
        }
        return 0.0f;
    }
#elif M1_HFI_QKICK_IQ_LADDER_ENABLE
    {
        int i;
        const float seg = M1_HFI_QKICK_IQ_LADDER_SEG_S;

        if (seg <= 0.0f) {
            return 0.0f;
        }
        i = (int)(t / seg);
        if (i < 0) {
            i = 0;
        }
        if (i >= (int)M1_HFI_QKICK_IQ_LADDER_N) {
            return 0.0f;
        }
#if M1_HFI_QKICK_IQ_LADDER_ARITH_ENABLE
        return M1_HFI_QKICK_IQ_LADDER_A0 + M1_HFI_QKICK_IQ_LADDER_DA * (float)i;
#else
        switch (i) {
        case 0:
            return M1_HFI_QKICK_IQ_LADDER_0;
        case 1:
            return M1_HFI_QKICK_IQ_LADDER_1;
        case 2:
            return M1_HFI_QKICK_IQ_LADDER_2;
        case 3:
            return M1_HFI_QKICK_IQ_LADDER_3;
        case 4:
            return M1_HFI_QKICK_IQ_LADDER_4;
        default:
            return M1_HFI_QKICK_IQ_LADDER_5;
        }
#endif
    }
#elif M1_HFI_QKICK_IQ_RAMP_ENABLE
    {
        const float t0 = M1_HFI_QKICK_IQ_RAMP_PRE_S;
        const float t1 = t0 + M1_HFI_QKICK_IQ_RAMP_S;
        const float t2 = t1 + M1_HFI_QKICK_IQ_RAMP_HOLD_S;

        if (t < t0) {
            return 0.0f;
        }
        if (t < t1) {
            return M1_HFI_QKICK_IQ_RAMP_A_MAX * ((t - t0) / M1_HFI_QKICK_IQ_RAMP_S);
        }
        if (t < t2) {
            return M1_HFI_QKICK_IQ_RAMP_A_MAX;
        }
        return 0.0f;
    }
#else
    (void)t;
    return M1_HFI_QKICK_CRAWL_IQ_A;
#endif
}

#if (M1_HFI_QKICK_IQ_LADDER_ENABLE || M1_HFI_QKICK_IQ_SLOW_ENABLE || M1_HFI_QKICK_IF_ENABLE)
/**
 * @brief 首档。ω̂：明显反转才。π；|ω̂| 太小当静摩擦，不。
 */
static void hfi_qk_maybe_pol_from_w(void)
{
    float w_rpm;
    const float rpm_scale =
        60.0f / (2.0f * (float)M_PI * (float)OBS_POLE_PAIRS);

    if (s_qk_pol_done != 0u) {
        return;
    }
    if (s_stage_t < M1_HFI_QKICK_POL_DECIDE_S) {
        return;
    }
    w_rpm = s_omega_el * rpm_scale;
    s_qk_pol_done = 1u;
    if (w_rpm < (-M1_HFI_QKICK_POL_W_RPM)) {
        s_theta_hat = motor_wrap_pi(s_theta_hat + (float)M_PI);
        s_pll_int = 0.0f;
        s_qk_verdict = -1.0f;
    } else if (w_rpm > M1_HFI_QKICK_POL_W_RPM) {
        s_qk_verdict = 1.0f;
    } else {
        s_qk_verdict = 0.0f;
    }
}
#endif

static void hfi_qk_enter_crawl(void)
{
    s_stage = HFI_STAGE_CRAWL;
    s_stage_t = 0.0f;
#if M1_HFI_QKICK_IF_ENABLE
    s_if_active = 0u;
    s_if_rpm_cmd = 0.0f;
#endif
    s_qk_iq_ref = hfi_qk_crawl_iq_ref(0.0f);
    s_qk_ov = 0u;
    s_qk_pi_reset = 1u;
    s_inj_prev_valid = 0u;
    s_sign = 1.0f;
    hfi_xy_lp_reset();
    hfi_set_inj_on_hat();
    /* 保留 pll_int/ω：零。HOLD/阶跃/I–f 都不清角积分，避免进段跳。*/
    s_omega_trim_el = s_qk_iq_ref;
    s_eps = s_qk_dth;
    s_di_q = s_qk_verdict;
    s_theta_err = motor_wrap_pi(s_theta_hat - s_theta_enc);
    s_theta_cmd = s_theta_hat;
    s_lock = HFI_LOCK_LOCKED;
    s_lock_cnt = 0u;
    s_qk_pol_done = 0u;
}
#endif

#if M1_HFI_QKICK_SPEED_ENABLE
/**
 * @brief P3：极。OK 后重。RUN，开速度环（ω_fb=HFI，Park=θ̂。
 */
static void hfi_qk_enter_speed_run(void)
{
    s_stage = HFI_STAGE_RUN;
    s_stage_t = 0.0f;
    s_qk_iq_ref = 0.0f; /* Iq 交速度。*/
    s_qk_ov = 0u;
    s_qk_pi_reset = 1u;
    s_inj_prev_valid = 0u;
    s_sign = 1.0f;
    hfi_xy_lp_reset();
    /* S1：保。pll_int/ω 作自举底座，不清零（避免 Type-II 冷启动） */
    s_omega_trim_el = 0.0f;
    s_eps = s_qk_dth;
    s_di_q = s_qk_verdict;
    s_theta_err = motor_wrap_pi(s_theta_hat - s_theta_enc);
    s_theta_cmd = s_theta_hat;
    s_lock = HFI_LOCK_LOCKED;
    s_lock_cnt = 0u;
#if M1_HFI_SENSED_CAL_ENABLE
    s_sensed_cal_loop = 0u;
#endif
}
#endif
#endif /* AFTER_LOCK */

static void hfi_qk_finish_meas_to_next(void)
{
    s_eps = s_qk_dth;
    s_di_q = s_qk_verdict;
    s_theta_err = motor_wrap_pi(s_theta_hat - s_theta_enc);
#if M1_HFI_QKICK_BEFORE_HFI_ENABLE
    /* 产品序：踢完直接 HFI RUN，不。LOG（避。AFTER_LOCK 的先 HFI 再踢。*/
    hfi_enter_run_after_kick();
    return;
#elif M1_HFI_QKICK_AFTER_LOCK_ENABLE && M1_HFI_QKICK_START_ENABLE
    if (s_qk_verdict != 0.0f) {
#if M1_HFI_PLL_EPS_DEAD_SWEEP_ENABLE
        s_eps_dead_i = 0u; /* 踢后从死区表。0 档扫。*/
#endif
        hfi_start_enter_hold();
        return;
    }
#elif M1_HFI_QKICK_AFTER_LOCK_ENABLE && M1_HFI_QKICK_SPEED_ENABLE
    /* 踢后必进速度环；verdict 只对照，不挡起步 */
    hfi_qk_enter_speed_run();
    return;
#elif M1_HFI_QKICK_AFTER_LOCK_ENABLE && M1_HFI_QKICK_CRAWL_ENABLE
    /* 极。OK 才爬；踢不动则只。LOG */
    if (s_qk_verdict != 0.0f) {
        hfi_qk_enter_crawl();
        return;
    }
#endif
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
#if M1_HFI_QKICK_AFTER_LOCK_ENABLE
#if M1_HFI_QKICK_BEFORE_HFI_ENABLE
    /* 产品序先踢：MEAS 不叠 Vh，纯 Iq；注入只在踢。enter_run_after_kick 打开 */
    hfi_clear_inj();
#else
    hfi_set_inj_on_hat();
#endif
#endif
    s_theta_err = motor_wrap_pi(s_theta_hat - s_theta_enc);
    s_qk_cnt++;

#if M1_HFI_QKICK_SWEEP_ENABLE
    {
        float dth;

        if (s_qk_enc_prev_valid != 0u) {
            dth = motor_wrap_pi(s_theta_enc - s_qk_enc_prev);
            s_omega_el = dth / OBS_CTRL_TS_S;
        } else {
            s_omega_el = 0.0f;
        }
        s_qk_enc_prev = s_theta_enc;
        s_qk_enc_prev_valid = 1u;
    }
#endif

    switch (s_qk_phase) {
#if M1_HFI_DQ_IDENT_ENABLE
    case HFI_QK_SEED:
        s_qk_id_ref = 0.0f;
        s_qk_iq_ref = 0.0f;
        s_qk_ov = 0u;
        hfi_dq_med_push(s_theta_enc);
        if (s_dq_med_n >= M1_HFI_DQ_MED_N) {
            s_dq_enc0 = hfi_dq_med_value();
            hfi_dq_med_reset();
            s_qk_hat0 = s_theta_hat;
            s_qk_pi_reset = 1u;
            s_qk_phase = (uint8_t)HFI_QK_KICK;
            s_qk_cnt = 0u;
        }
        break;

    case HFI_QK_KICK:
        if (s_dq_axis == 0u) {
            s_qk_id_ref = M1_HFI_QKICK_IQ_A;
            s_qk_iq_ref = 0.0f;
        } else {
            s_qk_id_ref = 0.0f;
            s_qk_iq_ref = M1_HFI_QKICK_IQ_A;
        }
        s_qk_ov = 0u;
        if ((s_qk_cnt + M1_HFI_DQ_MED_N) > M1_HFI_QKICK_KICK_N) {
            hfi_dq_med_push(s_theta_enc);
        }
        if (s_qk_cnt >= M1_HFI_QKICK_KICK_N) {
            hfi_dq_on_pulse_done();
        }
        break;

    default:
        hfi_dq_on_pulse_done();
        break;
#else
    case HFI_QK_SEED:
#if M1_HFI_QKICK_SWEEP_ENABLE
        if (s_qk_seed_i == 0u) {
            s_theta_hat = s_theta_cmd;
        } else {
            s_theta_hat = motor_wrap_pi(s_theta_cmd + (float)M_PI);
        }
#endif
#if M1_HFI_QKICK_BEFORE_HFI_ENABLE
        /*
         * C4g 2142：IDLE 直踢。Iq±1.6 已跟上但 Δθ=0；S3b 。2s HF 预热能踢动。
         * 此处只加。HF settle（PLL 仍冻），不走路会漂角。PRE 跟踪。
         */
        if (M1_HFI_QKICK_BEFORE_SETTLE_N > 0u) {
            s_qk_iq_ref = 0.0f;
            s_qk_id_ref = 0.0f;
            s_qk_ov = 0u;
            if (s_qk_cnt < M1_HFI_QKICK_BEFORE_SETTLE_N) {
                break;
            }
        }
#endif
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
#if M1_HFI_QKICK_AFTER_LOCK_ENABLE
        if (s_qk_cnt == M1_HFI_QKICK_HAT0_N) {
            s_qk_hat0 = s_theta_hat;
        }
        s_qk_dth = motor_wrap_pi(s_theta_enc - s_qk_enc0);
        s_eps = s_qk_dth;
#else
        s_qk_dth = motor_wrap_pi(s_theta_enc - s_qk_enc0);
        s_eps = s_qk_dth;
#endif
        if (s_qk_cnt >= M1_HFI_QKICK_KICK_N) {
#if M1_HFI_QKICK_AFTER_LOCK_ENABLE
            hfi_qk_decide_after_lock();
#elif M1_HFI_QKICK_SWEEP_ENABLE
            hfi_qk_decide();
#endif
#if M1_HFI_QKICK_BRAKE_ENABLE
            s_qk_phase = (uint8_t)HFI_QK_BRAKE;
            s_qk_cnt = 0u;
            s_qk_iq_ref = -M1_HFI_QKICK_IQ_A;
#else
            hfi_qk_finish_meas_to_next();
#endif
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
#endif
    }
}
#endif /* QKICK_ANY */

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
#if (M1_HFI_GATE == 118) || (M1_HFI_GATE == 119) || (M1_HFI_GATE == 120) || (M1_HFI_GATE == 121) || (M1_HFI_GATE == 122) || (M1_HFI_GATE == 123) || (M1_HFI_GATE == 124) || (M1_HFI_GATE == 125) || (M1_HFI_GATE == 126) || (M1_HFI_GATE == 127) || (M1_HFI_GATE == 128) || (M1_HFI_GATE == 129) || (M1_HFI_GATE == 130) || (M1_HFI_GATE == 131) || (M1_HFI_GATE == 138) || (M1_HFI_GATE == 132) || (M1_HFI_GATE == 133) || (M1_HFI_GATE == 134) || (M1_HFI_GATE == 135) || (M1_HFI_GATE == 136) || (M1_HFI_GATE == 137) || (M1_HFI_GATE == 139) || (M1_HFI_GATE == 140) || (M1_HFI_GATE == 141)
    s_speed_est = 0.0f;
#endif
    s_pll_eps_dead = M1_HFI_PLL_EPS_DEAD;
#if M1_HFI_AXIS_SEL_ENABLE
    s_eps_d_lp = 0.0f;
    s_axis_lock_n = 0u;
    s_axis_good_n = 0u;
    s_axis_cd = 0u;
    s_axis_flip_n = 0u;
    s_axis_ok = 0u;
#endif
#if M1_HFI_LQ_WELL_FLIP_ENABLE
    s_lq_well_n = 0u;
    s_lq_well_flip_n = 0u;
#endif
#if M1_HFI_PLL_HOLD_ENABLE
    s_pll_hold = 0u;
#if M1_HFI_PLL_RETRACK_USE_ENC
    s_w_mot_rpm = 0.0f;
    s_enc_mot_prev = 0.0f;
    s_enc_mot_valid = 0u;
#endif
    s_retrack_n = 0u;
#endif
#if M1_HFI_PLL_EPS_DEAD_SWEEP_ENABLE && M1_HFI_QKICK_START_ENABLE
    s_eps_dead_i = 0u;
    s_vh_v = hfi_start_vh_at(0u);
    s_pll_eps_dead = hfi_eps_dead_at(0u);
#endif
#if M1_HFI_BIAS_CAL_ENABLE
    s_theta_bias = 0.0f;
    s_bias_frozen = 0u;
#endif
    s_polarity_cnt = 0u;
    s_omega_ff_el = 0.0f;
    s_omega_el = 0.0f;
    s_omega_trim_el = 0.0f;
    s_sh_int = 0.0f;
    s_sh_w = 0.0f;
    s_sh_th = 0.0f;
    s_sh_seed = 0u;
#if M1_HFI_POLARITY_IPD_ENABLE
    s_ipd_phase = (uint8_t)HFI_IPD_ALIGN;
    s_ipd_cnt = 0u;
    s_ipd_align_hold = 0u;
    s_ipd_peak0 = 0.0f;
    s_ipd_peak1 = 0.0f;
    s_ipd_th0 = 0.0f;
    s_ipd_iabs = 0.0f;
    s_ipd_ov = 0u;
    s_ipd_ud = 0.0f;
    s_ipd_uq = 0.0f;
#if M1_HFI_IPD_SWEEP_ENABLE
    s_ipd_pos_i = 0u;
    s_ipd_round = 0u;
    s_ipd_move_cnt = 0u;
    s_ipd_ud_i = 0u;
    s_ipd_pulse_ud = M1_HFI_IPD_UD_TAB_0;
#endif
#endif
#if M1_HFI_QKICK_ANY
#if M1_HFI_QKICK_SWEEP_ENABLE
    s_qk_pos_i = 0u;
    s_qk_round = 0u;
    s_qk_move_cnt = 0u;
#endif
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
#if M1_HFI_QKICK_AFTER_LOCK_ENABLE
    s_qk_done = 0u;
    s_qk_pol_done = 0u;
#if M1_HFI_SPD_CLOSE_ENABLE
    s_spd_close_on = 0u;
    s_spd_close_rpm = 0.0f;
    s_spd_close_target = 0.0f;
    s_spd_close_mark_t = -1.0f;
#endif
#if M1_HFI_QKICK_PRE_GATE_ENABLE
    s_qk_pre_ok = 0u;
    s_qk_pre_good_n = 0u;
    s_qk_pre_false_n = 0u;
    s_qk_pre_cd = 0u;
    s_qk_pre_flip_n = 0u;
#endif
#if M1_HFI_DQ_IDENT_ENABLE
    s_dq_step = (uint8_t)HFI_DQ_BLANK;
    s_dq_axis = 0u;
    s_park_off = 0.0f;
    s_dq_dth_d = 0.0f;
    s_dq_dth_q = 0.0f;
    s_dq_enc0 = 0.0f;
    s_dq_med_n = 0u;
    s_dq_gate_enc0 = 0.0f;
    s_dq_good_t = 0.0f;
    s_dq_enc0_ok = 0u;
#endif
#if M1_HFI_SENSED_CAL_ENABLE
    s_sensed_cal_loop = 0u;
#endif
#endif
#if M1_HFI_QKICK_IF_ENABLE
    s_if_active = 0u;
    s_if_rpm_cmd = 0.0f;
#endif
#if M1_HFI_QKICK_START_ENABLE
    s_start_phase = (uint8_t)HFI_START_HOLD;
    s_start_run_armed = 0u;
    s_start_rpm_cmd = 0.0f;
    s_start_cool_s = 0.0f;
    s_start_ang_int = 0.0f;
    s_start_enc_prev = 0.0f;
    s_start_enc_prev_valid = 0u;
    s_start_wenc_rpm = 0.0f;
    s_start_exit_n = 0u;
    s_start_iq_sign = 1.0f;
    s_start_flip_done = 0u;
    s_start_flip_n = 0u;
    s_gate_light_n = 0u;
    s_gate_heavy_n = 0u;
    s_gate_fake_n = 0u;
#endif
#endif
#if M1_HFI_DELTA_SWEEP_ENABLE
    s_delta_i = 0;
    s_delta_rad = 0.0f;
    s_a0_pos = 0u;
#endif
#if M1_HFI_IQ_AUTH_FEED_ENABLE
    s_iq_feed_ok_t = 0.0f;
    s_iq_feed_cmd = 0.0f;
    s_iq_feed_mode = 0u;
    s_iq_feed_ab_n = 0u;
#endif
    (void)M1_HFI_FH_HZ;
}

void hfi_sqwave_set_omega_ff_el(float omega_el_rad_s)
{
    s_omega_ff_el = omega_el_rad_s;
}

void hfi_sqwave_on_angle(float theta_enc_el, float dt)
{
#if (M1_HFI_GATE != 51) && (M1_HFI_GATE != 52)
    const float run_total_s = hfi_run_total_s();
#endif

    s_theta_enc = theta_enc_el;

    if (dt <= 0.0f) {
        dt = OBS_CTRL_TS_S;
    }
    s_stage_t += dt;
#if M1_HFI_PLL_HOLD_ENABLE && M1_HFI_PLL_RETRACK_USE_ENC
    hfi_pll_hold_motion_step(dt);
#endif

#if M1_HFI_DELTA_SWEEP_ENABLE
    switch (s_stage) {
    case HFI_STAGE_IDLE:
        hfi_clear_inj();
        s_theta_hat = s_theta_enc;
        s_theta_err = 0.0f;
        s_theta_cmd = 0.0f;
        if (s_stage_t >= M1_HFI_BOOT_DELAY_S) {
            hfi_enter_delta_step(0); /* 上电直接扫，不经 Ud */
        }
        break;
    case HFI_STAGE_MOVE:
        s_theta_hat = s_theta_cmd;
        s_theta_err = motor_wrap_pi(s_theta_hat - s_theta_enc);
        hfi_clear_inj();
        if (s_stage_t >= M1_HFI_A0_MOVE_S) {
            s_stage = HFI_STAGE_SETTLE;
            s_stage_t = 0.0f;
        }
        break;
    case HFI_STAGE_SETTLE:
        hfi_clear_inj();
        s_theta_hat = s_theta_enc;
        s_theta_err = 0.0f;
        if (s_stage_t >= M1_HFI_A0_SETTLE_S) {
            hfi_enter_delta_step(0);
        }
        break;
    case HFI_STAGE_MEAS:
        /* 每拍强制开环角，防止任何路径改 θ̂ */
        s_theta_hat = motor_wrap_pi(s_theta_enc + s_delta_rad);
        s_theta_err = s_delta_rad;
        s_theta_cmd = s_delta_rad;
        hfi_set_inj_on_hat();
        if (s_stage_t >= M1_HFI_DELTA_HOLD_S) {
            const int16_t next = (int16_t)(s_delta_i + 1);

            if (next >= hfi_delta_nsteps()) {
#if M1_HFI_DQ_IDENT_ENABLE
                hfi_enter_run_from_delta();
#else
                /* S1。3 档必须跑完。不。RUN，不因锁/超时中止。*/
                s_stage = HFI_STAGE_LOG;
                s_stage_t = 0.0f;
                hfi_clear_inj();
#endif
            } else {
                hfi_enter_delta_step(next);
            }
        }
        break;
    case HFI_STAGE_RUN:
        hfi_set_inj_on_hat();
#if M1_HFI_DQ_IDENT_ENABLE
        s_theta_err = motor_wrap_pi(s_theta_hat - s_theta_enc);
        if (hfi_dq_ident_on_run() != 0u) {
            break;
        }
#else
        s_theta_err = motor_wrap_pi(s_theta_hat - s_theta_enc);
        if (s_stage_t >= 8.0f) {
            s_a0_pos++;
            if (s_a0_pos >= (uint8_t)M1_HFI_A0_POS_N) {
                s_stage = HFI_STAGE_DONE;
                s_stage_t = 0.0f;
                hfi_clear_inj();
            } else {
                hfi_enter_a0_move(); /* 下一电角 60° / 120° */
            }
        }
#endif
        break;
    case HFI_STAGE_LOG:
        /* 注入已关；保留末。x/y 供尾巴录波。分析脚本只。stage=3。*/
        hfi_clear_inj();
        s_theta_hat = motor_wrap_pi(s_theta_enc + s_delta_rad);
        s_theta_err = s_delta_rad;
        s_theta_cmd = s_delta_rad;
        if (s_stage_t >= M1_HFI_DELTA_TAIL_S) {
            s_stage = HFI_STAGE_DONE;
            s_stage_t = 0.0f;
        }
        break;
    case HFI_STAGE_DONE:
    default:
        hfi_clear_inj();
        s_theta_hat = motor_wrap_pi(s_theta_enc + s_delta_rad);
        s_theta_err = s_delta_rad;
        s_theta_cmd = s_delta_rad;
        break;
    }
#elif M1_HFI_IPD_SWEEP_ENABLE
    (void)run_total_s;
    s_theta_err = motor_wrap_pi(s_theta_hat - theta_enc_el);
    switch (s_stage) {
    case HFI_STAGE_IDLE:
        hfi_clear_inj();
        s_ipd_ov = 1u;
        s_ipd_ud = 0.0f;
        s_ipd_uq = 0.0f;
        if (s_stage_t >= M1_HFI_BOOT_DELAY_S) {
            s_ipd_pos_i = 0u;
            s_ipd_round = 0u;
            s_ipd_ud_i = 0u;
            hfi_ipd_sweep_enter_move();
        }
        break;
    case HFI_STAGE_MOVE:
        /* Park=θ_cmd，开。Ud 摆转。*/
        s_theta_hat = s_theta_cmd;
        s_theta_err = motor_wrap_pi(s_theta_hat - s_theta_enc);
        s_ipd_ov = 1u;
        s_ipd_ud = M1_HFI_IPD_MOVE_UD_V;
        s_ipd_uq = 0.0f;
        hfi_clear_inj();
        s_ipd_move_cnt++;
        if (s_ipd_move_cnt >= M1_HFI_IPD_MOVE_N) {
            s_stage = HFI_STAGE_SETTLE;
            s_stage_t = 0.0f;
            s_ipd_cnt = 0u;
            hfi_ipd_set_zero_u();
        }
        break;
    case HFI_STAGE_SETTLE:
        s_theta_hat = s_theta_cmd;
        s_theta_err = motor_wrap_pi(s_theta_hat - s_theta_enc);
        hfi_ipd_set_zero_u();
        s_di_q = s_ipd_iabs;
        s_ipd_cnt++;
        if (hfi_ipd_settle_done() != 0u) {
            /* 跳过 HFI ALIGN：th0=摆位角，直接双脉。*/
            s_ipd_pulse_ud = hfi_ipd_pulse_ud_now();
            s_ipd_th0 = s_theta_cmd;
            s_theta_hat = s_ipd_th0;
            s_stage = HFI_STAGE_MEAS;
            s_stage_t = 0.0f;
            s_ipd_phase = (uint8_t)HFI_IPD_P0;
            s_ipd_cnt = 0u;
            s_ipd_peak0 = 0.0f;
            s_ipd_peak1 = 0.0f;
            s_omega_trim_el = s_ipd_pulse_ud;
            hfi_ipd_set_pulse_ud(s_ipd_pulse_ud);
        }
        break;
    case HFI_STAGE_MEAS:
        hfi_ipd_on_angle();
        break;
    case HFI_STAGE_LOG:
        s_ipd_ov = 1u;
        s_ipd_ud = 0.0f;
        s_ipd_uq = 0.0f;
        hfi_clear_inj();
        s_di_q = s_ipd_peak0;
        s_eps = s_ipd_peak1;
        s_omega_el = s_ipd_peak0;
        s_omega_trim_el = s_ipd_pulse_ud; /* VOFA ch5：脉冲档电压 */
        s_theta_err = motor_wrap_pi(s_theta_hat - s_theta_enc);
        if (s_stage_t >= M1_HFI_IPD_LOG_S) {
            /* 顺序：位。。电压。。轮次 */
            s_ipd_pos_i++;
            if (s_ipd_pos_i >= M1_HFI_IPD_POS_N) {
                s_ipd_pos_i = 0u;
                s_ipd_ud_i++;
                if (s_ipd_ud_i >= M1_HFI_IPD_UD_TAB_N) {
                    s_ipd_ud_i = 0u;
                    s_ipd_round++;
                }
            }
            if (s_ipd_round >= M1_HFI_IPD_ROUNDS) {
                s_stage = HFI_STAGE_DONE;
                s_stage_t = 0.0f;
                s_ipd_ov = 0u;
            } else {
                hfi_ipd_sweep_enter_move();
            }
        }
        break;
    case HFI_STAGE_DONE:
    default:
        s_ipd_ov = 1u;
        s_ipd_ud = 0.0f;
        s_ipd_uq = 0.0f;
        hfi_clear_inj();
        break;
    }

#elif M1_HFI_QKICK_SWEEP_ENABLE
    (void)run_total_s;
    s_theta_err = motor_wrap_pi(s_theta_hat - theta_enc_el);
    switch (s_stage) {
    case HFI_STAGE_IDLE:
        hfi_qk_set_zero_u();
        if (s_stage_t >= M1_HFI_BOOT_DELAY_S) {
            s_qk_pos_i = 0u;
            s_qk_seed_i = 0u;
            s_qk_round = 0u;
            hfi_qk_enter_move();
        }
        break;
    case HFI_STAGE_MOVE:
        s_theta_hat = s_theta_cmd;
        s_theta_err = motor_wrap_pi(s_theta_hat - s_theta_enc);
        s_qk_ov = 1u;
        s_qk_ud = M1_HFI_QKICK_MOVE_UD_V;
        s_qk_uq = 0.0f;
        s_qk_iq_ref = 0.0f;
        hfi_clear_inj();
        s_qk_move_cnt++;
        if (s_qk_move_cnt >= M1_HFI_QKICK_MOVE_N) {
            s_stage = HFI_STAGE_SETTLE;
            s_stage_t = 0.0f;
            s_qk_cnt = 0u;
            hfi_qk_set_zero_u();
        }
        break;
    case HFI_STAGE_SETTLE:
        s_theta_hat = s_theta_cmd;
        s_theta_err = motor_wrap_pi(s_theta_hat - s_theta_enc);
        hfi_qk_set_zero_u();
        s_qk_cnt++;
        if (hfi_qk_settle_done() != 0u) {
            s_stage = HFI_STAGE_MEAS;
            s_stage_t = 0.0f;
            s_qk_phase = (uint8_t)HFI_QK_SEED;
            s_qk_cnt = 0u;
            s_qk_enc_prev_valid = 0u;
        }
        break;
    case HFI_STAGE_MEAS:
        hfi_qk_on_meas();
        break;
    case HFI_STAGE_LOG:
        hfi_qk_set_zero_u();
        s_eps = s_qk_dth;
        s_di_q = s_qk_verdict;
        s_omega_trim_el = M1_HFI_QKICK_IQ_A;
        s_theta_err = motor_wrap_pi(s_theta_hat - s_theta_enc);
        if (s_stage_t >= M1_HFI_QKICK_LOG_S) {
            /* 位置 。种子 。轮次 */
            s_qk_seed_i++;
            if (s_qk_seed_i >= 2u) {
                s_qk_seed_i = 0u;
                s_qk_pos_i++;
                if (s_qk_pos_i >= M1_HFI_QKICK_POS_N) {
                    s_qk_pos_i = 0u;
                    s_qk_round++;
                }
            }
            if (s_qk_round >= M1_HFI_QKICK_ROUNDS) {
                s_stage = HFI_STAGE_DONE;
                s_stage_t = 0.0f;
                hfi_qk_set_zero_u();
            } else {
                hfi_qk_enter_move();
            }
        }
        break;
    case HFI_STAGE_DONE:
    default:
        hfi_qk_set_zero_u();
        break;
    }

#elif M1_HFI_MOTION_BYPASS_ENABLE
    s_theta_err = motor_wrap_pi(s_theta_hat - theta_enc_el);
    switch (s_stage) {
    case HFI_STAGE_IDLE:
        hfi_clear_inj();
        if (s_stage_t >= M1_HFI_BOOT_DELAY_S) {
#if M1_HFI_POLARITY_IPD_ENABLE
            hfi_enter_ipd();
#elif M1_HFI_QKICK_BEFORE_HFI_ENABLE
            /* 先踢：enc 播种 θ̂ 。MEAS，禁止先。HFI PRE */
            if (s_qk_done == 0u) {
#if M1_HFI_INIT_FROM_ENC
                s_theta_hat = motor_wrap_pi(s_theta_enc + M1_HFI_PLL_INIT_OFF_RAD);
#else
                s_theta_hat = 0.0f;
#endif
                s_theta_cmd = s_theta_hat;
                s_theta_err = motor_wrap_pi(s_theta_hat - s_theta_enc);
                s_pll_int = 0.0f;
                s_omega_el = 0.0f;
                hfi_qk_enter_from_lock();
            } else {
                hfi_enter_run_after_kick();
            }
#else
            hfi_enter_run();
#endif
        }
        break;
#if M1_HFI_POLARITY_IPD_ENABLE
    case HFI_STAGE_MOVE:
        hfi_ipd_on_angle();
        break;
#endif
    case HFI_STAGE_RUN:
        hfi_set_inj_on_hat();
#if M1_HFI_DQ_IDENT_ENABLE
        if (hfi_dq_ident_on_run() != 0u) {
            break;
        }
#elif M1_HFI_QKICK_AFTER_LOCK_ENABLE && !M1_HFI_QKICK_BEFORE_HFI_ENABLE
        /* S3b：预。PRE_S 后踢；PRE_GATE：须 pre_ok，否则逃。等待，超时拒。*/
        if ((s_qk_done == 0u) && (s_stage_t >= M1_HFI_QKICK_PRE_S)) {
#if M1_HFI_QKICK_PRE_GATE_ENABLE
            if (s_qk_pre_ok != 0u) {
                hfi_qk_enter_from_lock();
                break;
            }
            if (s_stage_t >= M1_HFI_QKICK_PRE_GATE_TIMEOUT_S) {
                s_lock = HFI_LOCK_FAULT;
                s_qk_done = 1u;
                s_stage = HFI_STAGE_DONE;
                s_stage_t = 0.0f;
                hfi_clear_inj();
                s_omega_el = 0.0f;
                s_omega_trim_el = 0.0f;
                break;
            }
            break; /* 假锁未解除：继续 PRE，不准踢 */
#else
            hfi_qk_enter_from_lock();
            break;
#endif
        }
#endif
#if M1_HFI_QKICK_START_ENABLE
        if (s_start_run_armed != 0u) {
            if (hfi_start_gate_step() != 0u) {
                break; /* 已退。HOLD */
            }
        }
#endif
#if (M1_HFI_GATE == 51) || (M1_HFI_GATE == 52)
        if (s_run_ladder_done != 0u) {
#elif M1_HFI_GATE == 141
        if ((s_rev141_done != 0u) || (s_stage_t >= run_total_s)) {
#else
        if (s_stage_t >= run_total_s) {
#endif
#if M1_HFI_DQ_IDENT_ENABLE
            break; /* 超时。DQ 状态机自己。*/
#elif M1_HFI_SENSED_CAL_ENABLE
            /* 同一上电可多圈阶梯，方便一。CSV 覆盖多次启动 */
            if ((s_qk_done != 0u) &&
                ((uint16_t)s_sensed_cal_loop + 1u < (uint16_t)M1_HFI_SENSED_CAL_LOOPS)) {
                s_sensed_cal_loop++;
                s_stage_t = 0.0f;
                break;
            }
#endif
#if M1_HFI_IQ_AUTH_FEED_COMPARE_AB
            /* C4d：同一上电 A→IDLE→B，勿 NRST（RAM 清零会再。A。*/
            if (s_iq_feed_ab_n < 2u) {
                s_stage = HFI_STAGE_IDLE;
                s_stage_t = 0.0f;
                hfi_clear_inj();
                s_omega_el = 0.0f;
                s_omega_trim_el = 0.0f;
                s_lock = HFI_LOCK_CAPTURE;
                s_lock_cnt = 0u;
                s_iq_feed_ok_t = 0.0f;
                s_iq_feed_cmd = 0.0f;
#if M1_HFI_QKICK_START_ENABLE
                s_start_run_armed = 0u;
#endif
                break;
            }
#endif
            s_stage = HFI_STAGE_DONE;
            s_stage_t = 0.0f;
            hfi_clear_inj();
            s_omega_el = 0.0f;
            s_omega_trim_el = 0.0f;
            s_lock = HFI_LOCK_CAPTURE;
            s_lock_cnt = 0u;
#if M1_HFI_QKICK_START_ENABLE
            s_start_run_armed = 0u;
#endif
        }
        break;
#if M1_HFI_QKICK_AFTER_LOCK_ENABLE
    case HFI_STAGE_MEAS:
        hfi_qk_on_meas();
        break;
#if M1_HFI_QKICK_CRAWL_ENABLE
    case HFI_STAGE_CRAWL:
#if M1_HFI_QKICK_START_ENABLE
        hfi_start_on_crawl();
#else
        hfi_set_inj_on_hat();
        s_qk_iq_ref = hfi_qk_crawl_iq_ref(s_stage_t);
#if (M1_HFI_QKICK_IQ_LADDER_ENABLE || M1_HFI_QKICK_IQ_SLOW_ENABLE || M1_HFI_QKICK_IF_ENABLE)
        hfi_qk_maybe_pol_from_w();
#endif
        s_qk_ov = 0u;
        s_di_q = s_qk_verdict;
        s_omega_trim_el = s_qk_iq_ref;
        s_theta_err = motor_wrap_pi(s_theta_hat - s_theta_enc);
        if (s_stage_t >= M1_HFI_QKICK_CRAWL_S) {
            /* 。LOG：Iq=0 + 注入 + PLL（勿 set_zero_u 。inj。*/
            s_qk_iq_ref = 0.0f;
            s_qk_id_ref = 0.0f;
            s_qk_ov = 0u;
            s_qk_pi_reset = 1u;
            s_inj_prev_valid = 0u;
            hfi_set_inj_on_hat();
            s_stage = HFI_STAGE_LOG;
            s_stage_t = 0.0f;
        }
#endif
        break;
#endif
    case HFI_STAGE_LOG:
        /* 注入 + 电流。Iq=0；eps 。on_current 解调写入，勿。dth 盖掉 */
        hfi_set_inj_on_hat();
        s_qk_iq_ref = 0.0f;
        s_qk_id_ref = 0.0f;
        s_qk_ov = 0u;
        s_omega_trim_el = s_qk_verdict;
        s_theta_err = motor_wrap_pi(s_theta_hat - s_theta_enc);
        if (s_stage_t >= M1_HFI_QKICK_HOLD_S) {
#if M1_HFI_QKICK_THEN_HFI_ENABLE
            /* S3b 踢结。。。hold 。HFI RUN（qk_done=1，不会再踢） */
            hfi_enter_run_after_kick();
#else
            s_stage = HFI_STAGE_DONE;
            s_stage_t = 0.0f;
#endif
        }
        break;
#endif
    case HFI_STAGE_DONE:
    default:
        hfi_clear_inj();
#if M1_HFI_QKICK_AFTER_LOCK_ENABLE
        hfi_qk_set_zero_u();
#endif
        break;
    }
#else
    (void)run_total_s;
    (void)dt;
    s_stage = HFI_STAGE_IDLE;
    hfi_clear_inj();
#endif
}

void hfi_sqwave_on_current(float id, float iq, float i_alpha, float i_beta)
{
#if M1_HFI_DEMOD_AB_ENABLE || M1_HFI_DEMOD_AB_MID_ENABLE || M1_HFI_DEMOD_INJ_AXIS
    s_ia_now = i_alpha;
    s_ib_now = i_beta;
#else
    (void)i_alpha;
    (void)i_beta;
#endif
#if M1_HFI_DELTA_SWEEP_ENABLE
    if (s_stage == HFI_STAGE_MEAS) {
        hfi_demod_step(id, iq, 0u); /* 扫档：只解调，不更新 θ̂ */
    } else if (s_stage == HFI_STAGE_RUN) {
        hfi_demod_step(id, iq, 1u); /* 本位置锁。*/
    }
#elif M1_HFI_IPD_SWEEP_ENABLE
    if ((s_stage == HFI_STAGE_SETTLE) || (s_stage == HFI_STAGE_MEAS)) {
        hfi_ipd_on_current(id, iq);
    }
#elif M1_HFI_QKICK_ANY
    {
        float iabs = id * id + iq * iq;

        if (iabs > 0.0f) {
            iabs = sqrtf(iabs);
        } else {
            iabs = 0.0f;
        }
        s_qk_iabs = iabs;
    }
#if M1_HFI_QKICK_AFTER_LOCK_ENABLE
    if (s_stage == HFI_STAGE_RUN) {
#if M1_HFI_DQ_IDENT_ENABLE
        {
            float id_h = id;
            float iq_h = iq;

            if (s_park_off != 0.0f) {
                const float c = cosf(s_park_off);
                const float s = sinf(s_park_off);

                id_h = id * c + iq * s;
                iq_h = -id * s + iq * c;
            }
            hfi_demod_step(id_h, iq_h, 1u);
        }
#elif M1_HFI_PLL_HOLD_ENABLE
        hfi_demod_step(id, iq, hfi_pll_want_update());
#else
        {
            uint8_t upd = 1u;

#if M1_HFI_DEMOD_PROBE_ENABLE
            if (hfi_demod_probe_freeze() != 0u) {
                upd = 0u;
            }
#endif
            if (s_hat_hold != 0u) {
                upd = 0u;
            }
            hfi_demod_step(id, iq, upd);
        }
#endif
    } else if (s_stage == HFI_STAGE_MEAS) {
        hfi_demod_step(id, iq, 0u); /* 踢段。θ̂，避。Iq 泄漏 */
    } else if (s_stage == HFI_STAGE_LOG) {
        hfi_demod_step(id, iq, 1u); /* 踢后静置再锁。205 冻角差） */
    } else if (s_stage == HFI_STAGE_CRAWL) {
#if M1_HFI_PLL_HOLD_ENABLE
        hfi_demod_step(id, iq, hfi_pll_want_update());
#else
        hfi_demod_step(id, iq, 1u);
#endif
    }
#if M1_HFI_IQ_AUTH_FEED_ENABLE
    /* 踢完 RUN：每拍推。FEED→s_qk_iq_ref（勿。get_iq_ref 双计 delay。*/
    if ((s_stage == HFI_STAGE_RUN) && (s_qk_done != 0u)) {
        s_qk_iq_ref = hfi_iq_auth_feed_step();
    }
#endif
#endif
#elif M1_HFI_MOTION_BYPASS_ENABLE
    if (s_stage == HFI_STAGE_RUN) {
        hfi_demod_step(id, iq, 1u); /* P1：解冻，静置闭环 PLL */
#if M1_HFI_POLARITY_IPD_ENABLE
    } else if (s_stage == HFI_STAGE_MOVE) {
        if (s_ipd_phase == (uint8_t)HFI_IPD_ALIGN) {
            hfi_demod_step(id, iq, 1u); /* 静置锁凸极轴 */
        } else {
            hfi_ipd_on_current(id, iq);
        }
#endif
    }
#else
    (void)id;
    (void)iq;
#endif
}

float hfi_sqwave_park_theta(float theta_enc_el)
{
#if M1_HFI_IPD_SWEEP_ENABLE
    /* Ud 摆位：Park=θ_cmd；脉。记录：Park=enc */
    if (s_stage == HFI_STAGE_MOVE) {
        return s_theta_cmd;
    }
    return theta_enc_el;
#elif M1_HFI_QKICK_SWEEP_ENABLE
    /* MOVE/SETTLE：Park=θ_cmd；KICK/BRAKE/LOG：Park=θ̂（含故意 π。*/
    if ((s_stage == HFI_STAGE_MOVE) || (s_stage == HFI_STAGE_SETTLE)) {
        return s_theta_cmd;
    }
    if ((s_stage == HFI_STAGE_MEAS) || (s_stage == HFI_STAGE_LOG)) {
        return s_theta_hat;
    }
    return theta_enc_el;
#elif M1_HFI_DELTA_SWEEP_ENABLE
    /* MOVE：Park=θ_cmd 才能 Ud 拉到目标电角；MEAS/RUN：Park=θ̂ */
    if (s_stage == HFI_STAGE_MOVE) {
        return s_theta_cmd;
    }
    if ((s_stage == HFI_STAGE_MEAS) || (s_stage == HFI_STAGE_RUN)) {
        return hfi_theta_hat_out();
    }
    return theta_enc_el;
#elif M1_HFI_PARK_ENABLE
#if M1_HFI_DQ_IDENT_ENABLE
    if ((s_stage == HFI_STAGE_MEAS) || (s_stage == HFI_STAGE_RUN)) {
        return motor_wrap_pi(s_theta_hat + s_park_off);
    }
#endif
#if M1_HFI_QKICK_AFTER_LOCK_ENABLE
#if M1_HFI_QKICK_PARK_ENC
    return theta_enc_el;
#endif
    if ((s_stage == HFI_STAGE_MEAS) || (s_stage == HFI_STAGE_LOG)) {
        return s_theta_hat;
    }
    if (s_stage == HFI_STAGE_CRAWL) {
#if M1_HFI_QKICK_CRAWL_PARK_ENC
        return theta_enc_el; /* S3c1：爬段力。Park=enc */
#endif
#if M1_HFI_QKICK_START_ENABLE
        /* START：Park=θ̂（冻结后。θ̂−δ） */
        return hfi_theta_hat_out();
#endif
#if M1_HFI_QKICK_IF_ENABLE
        if (s_if_active != 0u) {
            return s_theta_cmd; /* θ_if */
        }
#endif
        return hfi_theta_hat_out();
    }
#endif
    /* 。LOCK_ENABLE 时未锁退 enc；本。AFTER_LOCK 已在上面。θ̂ */
    if (hfi_park_uses_hat() != 0u) {
        return hfi_theta_hat_out();
    }
#endif
    return theta_enc_el;
}

uint8_t hfi_sqwave_override_voltage(float *ud, float *uq)
{
    if (ud == NULL || uq == NULL) {
        return 0u;
    }
#if M1_HFI_IPD_SWEEP_ENABLE
    if ((s_stage == HFI_STAGE_MOVE) ||
        ((s_ipd_ov != 0u) &&
         ((s_stage == HFI_STAGE_SETTLE) || (s_stage == HFI_STAGE_MEAS) ||
          (s_stage == HFI_STAGE_LOG) || (s_stage == HFI_STAGE_DONE) ||
          (s_stage == HFI_STAGE_IDLE)))) {
        *ud = s_ipd_ud;
        *uq = s_ipd_uq;
        return 1u;
    }
    *ud = 0.0f;
    *uq = 0.0f;
    return 1u;
#elif M1_HFI_QKICK_SWEEP_ENABLE
    /* MOVE/SETTLE/LOG/IDLE/DONE：强制电压；MEAS(KICK/BRAKE)：交电流。*/
    if (s_stage == HFI_STAGE_MEAS) {
        return 0u;
    }
    if ((s_stage == HFI_STAGE_MOVE) ||
        ((s_qk_ov != 0u) &&
         ((s_stage == HFI_STAGE_SETTLE) || (s_stage == HFI_STAGE_LOG) ||
          (s_stage == HFI_STAGE_DONE) || (s_stage == HFI_STAGE_IDLE)))) {
        *ud = s_qk_ud;
        *uq = s_qk_uq;
        return 1u;
    }
    *ud = 0.0f;
    *uq = 0.0f;
    return 1u;
#elif M1_HFI_DELTA_SWEEP_ENABLE
    /* MEAS/RUN：电流环 + 注入；MOVE：开。Ud 摆位；其余强。0 */
    if ((s_stage == HFI_STAGE_MEAS) || (s_stage == HFI_STAGE_RUN)) {
        return 0u;
    }
    if (s_stage == HFI_STAGE_MOVE) {
        *ud = M1_HFI_A0_MOVE_UD_V;
        *uq = 0.0f;
        return 1u;
    }
    *ud = 0.0f;
    *uq = 0.0f;
    return 1u;
#elif M1_HFI_MOTION_BYPASS_ENABLE
    if (s_stage == HFI_STAGE_RUN) {
        return 0u;
    }
#if M1_HFI_QKICK_AFTER_LOCK_ENABLE
    if ((s_stage == HFI_STAGE_MEAS) || (s_stage == HFI_STAGE_CRAWL) ||
        (s_stage == HFI_STAGE_LOG)) {
        return 0u; /* 。蠕动/LOG 再锁：电流环 + 注入 */
    }
    if ((s_stage == HFI_STAGE_DONE) || (s_stage == HFI_STAGE_IDLE)) {
        *ud = 0.0f;
        *uq = 0.0f;
        return 1u;
    }
#endif
#if M1_HFI_POLARITY_IPD_ENABLE
    if ((s_stage == HFI_STAGE_MOVE) && (s_ipd_ov != 0u)) {
        *ud = s_ipd_ud;
        *uq = s_ipd_uq;
        return 1u;
    }
    if (s_stage == HFI_STAGE_MOVE) {
        /* ALIGN：走电流。+ HFI 注入 */
        return 0u;
    }
#endif
    *ud = 0.0f;
    *uq = 0.0f;
    return 1u;
#else
    *ud = 0.0f;
    *uq = 0.0f;
    return 1u;
#endif
}

void hfi_sqwave_get_inj(float *ud_inj, float *uq_inj)
{
#if M1_HFI_INJECT_AB_ENABLE || M1_HFI_INJECT_POST_LOOP
    if (ud_inj != NULL) {
        *ud_inj = 0.0f;
    }
    if (uq_inj != NULL) {
        *uq_inj = 0.0f;
    }
#else
    if (ud_inj != NULL) {
        *ud_inj = s_ud_inj;
    }
    if (uq_inj != NULL) {
        *uq_inj = s_uq_inj;
    }
#endif
}

void hfi_sqwave_get_inj_ab(float *u_alpha_inj, float *u_beta_inj)
{
    float ua = 0.0f;
    float ub = 0.0f;

#if M1_HFI_INJECT_POST_LOOP
    /* 用本半周锁定的 s_vh_sign，勿用解调后已翻转的 s_sign（GATE88 翻号）。 */
    if (s_vh_sign != 0.0f) {
        const float mag = s_vh_v * s_vh_scale * s_vh_sign;
        const float c = cosf(s_theta_hat);
        const float s = sinf(s_theta_hat);

        ua = mag * c;
        ub = mag * s;
    }
#elif M1_HFI_INJECT_AB_ENABLE
    if (s_vh_sign != 0.0f) {
        const float mag = s_vh_v * s_vh_scale * s_sign;
        const float c = cosf(s_theta_hat);
        const float s = sinf(s_theta_hat);

        ua = mag * c;
        ub = mag * s;
    }
#endif
    if (u_alpha_inj != NULL) {
        *u_alpha_inj = ua;
    }
    if (u_beta_inj != NULL) {
        *u_beta_inj = ub;
    }
}

float hfi_sqwave_get_iq_ref(void)
{
#if M1_HFI_QKICK_ANY
    /* 踢段：s_qk_iq_ref；踢。FEED：on_current 已写。FEED */
    return s_qk_iq_ref;
#elif M1_HFI_IQ_RAMP_ENABLE
    {
        float t;
        float t0;
        float t1;

        if (s_stage != HFI_STAGE_RUN) {
            return 0.0f;
        }
        t = s_stage_t;
        t0 = M1_HFI_IQ_RAMP_PRE_S;
        t1 = t0 + M1_HFI_IQ_RAMP_S;
        if (t < t0) {
            return 0.0f;
        }
        if (t < t1) {
            return M1_HFI_IQ_RAMP_A_MAX * ((t - t0) / M1_HFI_IQ_RAMP_S);
        }
        if (t < (t1 + M1_HFI_IQ_RAMP_HOLD_S)) {
            return M1_HFI_IQ_RAMP_A_MAX;
        }
        return 0.0f;
    }
#elif M1_HFI_IQ_PULL_ENABLE
    hfi_iq_pull_try_arm();
    if ((s_stage == HFI_STAGE_RUN) && (s_iq_pull_done == 0u)) {
#if M1_HFI_IQ_PULL_ARM_ENABLE
        /* 超时仍未到门槛：停转矩，不开速度环（避免 100。 再顶电流。*/
        if (s_stage_t >= M1_HFI_IQ_PULL_S) {
            return 0.0f;
        }
#endif
#if M1_HFI_IQ_AUTH_ENABLE
        /* 。C4 相同：x 没到 GOOD 之前不出力；掉线后收。*/
        if (s_iq_auth_ok == 0u) {
            s_iq_pull_ok_t = 0.0f;
            return 0.0f;
        }
        s_iq_pull_ok_t += OBS_CTRL_TS_S;
        if (s_iq_pull_ok_t < M1_HFI_IQ_PULL_DELAY_S) {
            return 0.0f;
        }
#endif
        return M1_HFI_IQ_PULL_A;
    }
    return 0.0f;
#elif M1_HFI_IQ_AUTH_FEED_ENABLE
    /* 。QKICK 。C4：motor_current 解调后单次调。*/
    return hfi_iq_auth_feed_step();
#else
    return 0.0f;
#endif
}

float hfi_sqwave_get_id_ref(void)
{
#if M1_HFI_QKICK_ANY
    return s_qk_id_ref;
#else
    return 0.0f;
#endif
}

#if M1_HFI_SPD_CLOSE_ENABLE
/**
 * @brief 馈流拉起后，角度回到门槛且轴已转起来，才允许速度环。
 * @note 。s_stage_t 计时，同一拍多次调用不会把保持时间算短。
 */
static uint8_t hfi_spd_close_active(void)
{
    float ae;
    float rpm;
    float ar;

    if ((s_stage != HFI_STAGE_RUN) || (s_qk_done == 0u)) {
        s_spd_close_on = 0u;
        s_spd_close_mark_t = -1.0f;
        return 0u;
    }
    if (s_spd_close_on != 0u) {
        return 1u;
    }
    ae = s_theta_err;
    if (ae < 0.0f) {
        ae = -ae;
    }
    rpm = s_omega_el * (60.0f / (2.0f * 3.14159265f * (float)OBS_POLE_PAIRS));
    ar = rpm;
    if (ar < 0.0f) {
        ar = -ar;
    }
    if ((ae < M1_HFI_SPD_CLOSE_ERR_RAD) && (ar >= M1_HFI_SPD_CLOSE_MIN_RPM)) {
        if (s_spd_close_mark_t < 0.0f) {
            s_spd_close_mark_t = s_stage_t;
        }
        if ((s_stage_t - s_spd_close_mark_t) >= M1_HFI_SPD_CLOSE_HOLD_S) {
            s_spd_close_on = 1u;
            s_spd_close_rpm = rpm;
            if (rpm >= 0.0f) {
                s_spd_close_target = M1_HFI_SPD_CLOSE_TARGET_RPM;
            } else {
                s_spd_close_target = -M1_HFI_SPD_CLOSE_TARGET_RPM;
            }
            return 1u;
        }
    } else {
        s_spd_close_mark_t = -1.0f;
    }
    return 0u;
}

/**
 * @brief ω* 从交接转速斜坡到第一档。只。get_speed_ref 里推进。
 */
static float hfi_spd_close_ref_rpm(void)
{
    float step;
    float delta;

    if (s_spd_close_on == 0u) {
        return 0.0f;
    }
    step = M1_HFI_SPD_CLOSE_RAMP_RPM_S * OBS_CTRL_TS_S;
    delta = s_spd_close_target - s_spd_close_rpm;
    if (delta > step) {
        s_spd_close_rpm += step;
    } else if (delta < -step) {
        s_spd_close_rpm -= step;
    } else {
        s_spd_close_rpm = s_spd_close_target;
    }
    return s_spd_close_rpm;
}
#endif

uint8_t hfi_sqwave_speed_run_active(void)
{
#if M1_HFI_DELTA_SWEEP_ENABLE || M1_HFI_IPD_SWEEP_ENABLE || M1_HFI_QKICK_SWEEP_ENABLE
    return 0u;
#elif M1_HFI_IQ_RAMP_ENABLE
    /* 慢抬 Iq：电流环。iq_ref，不开速度。*/
    return 0u;
#elif M1_HFI_QKICK_AFTER_LOCK_ENABLE
#if M1_HFI_DQ_IDENT_ENABLE
    return ((s_stage == HFI_STAGE_RUN) && (s_dq_step == (uint8_t)HFI_DQ_SPEED))
               ? 1u
               : 0u;
#elif M1_HFI_QKICK_SPEED_ENABLE
    /*
     * P3a。/ START：极性已定且（START 已交接）。RUN 开速度环。
     */
#if M1_HFI_QKICK_START_ENABLE
    return ((s_stage == HFI_STAGE_RUN) && (s_qk_done != 0u) &&
            (s_start_run_armed != 0u))
               ? 1u
               : 0u;
#else
    return ((s_stage == HFI_STAGE_RUN) && (s_qk_done != 0u)) ? 1u : 0u;
#endif
#elif M1_HFI_RUN_LADDER_ENABLE
    /* C4r：THEN_HFI 踢完再开速度环；勿开 QKICK_SPEED（与 THEN_HFI 互斥。*/
    return ((s_stage == HFI_STAGE_RUN) && (s_qk_done != 0u)) ? 1u : 0u;
#elif (M1_HFI_GATE == 65) || (M1_HFI_GATE == 66) || (M1_HFI_GATE == 67) || (M1_HFI_GATE == 68) || (M1_HFI_GATE == 69) || (M1_HFI_GATE == 70) || (M1_HFI_GATE == 71) || (M1_HFI_GATE == 72) || (M1_HFI_GATE == 73) || (M1_HFI_GATE == 74) || (M1_HFI_GATE == 75) || (M1_HFI_GATE == 76) || (M1_HFI_GATE == 77) || (M1_HFI_GATE == 78) || (M1_HFI_GATE == 79) || (M1_HFI_GATE == 80) || (M1_HFI_GATE == 91) || (M1_HFI_GATE == 92) || (M1_HFI_GATE == 93)
    /*
     * 65/66：无 ladder，用 RPM1→RPM2 两段巡航。
     * 以前 LADDER=0 会落到下。return 0，踢。iq_ref 一。0。446）。
     */
    return ((s_stage == HFI_STAGE_RUN) && (s_qk_done != 0u)) ? 1u : 0u;
#else
#if M1_HFI_SPD_CLOSE_ENABLE
    /* 0.8 A 拉起之后，角度和转速都过线才把 Iq 交给速度。*/
    return hfi_spd_close_active();
#else
    /* P1/P2：锁定后蠕动，不开速度。*/
    return 0u;
#endif
#endif
#elif M1_HFI_MOTION_BYPASS_ENABLE
#if (M1_HFI_GATE == 3) || (M1_HFI_GATE == 4) || (M1_HFI_GATE == 5) || \
    (M1_HFI_GATE == 6) || (M1_HFI_GATE == 9) || (M1_HFI_GATE == 10) || \
    (M1_HFI_GATE == 11) || (M1_HFI_GATE == 12) || (M1_HFI_GATE == 13) || \
    (M1_HFI_GATE == 14) || (M1_HFI_GATE == 15) || (M1_HFI_GATE == 16) || \
    (M1_HFI_GATE == 17) || (M1_HFI_GATE == 18) || (M1_HFI_GATE == 19) || \
    (M1_HFI_GATE == 96) || (M1_HFI_GATE == 97) || (M1_HFI_GATE == 98) || \
    (M1_HFI_GATE == 102) || (M1_HFI_GATE == 103) || (M1_HFI_GATE == 104) || \
    (M1_HFI_GATE == 106) || (M1_HFI_GATE == 107) || (M1_HFI_GATE == 108) || \
    (M1_HFI_GATE == 109) || (M1_HFI_GATE == 110) || (M1_HFI_GATE == 111) || \
    (M1_HFI_GATE == 112) || (M1_HFI_GATE == 113) || (M1_HFI_GATE == 114) || \
    (M1_HFI_GATE == 115) || (M1_HFI_GATE == 116) || (M1_HFI_GATE == 117) || \
    (M1_HFI_GATE == 118) || (M1_HFI_GATE == 119)
    return 0u; /* S3a..C4e / V3–V5 / VESC 静置：不开速度环 */
#else
#if M1_HFI_IQ_PULL_ENABLE
    /* 固定 Iq 拉起未结束：速度环关；结束后。SPEED_FB=HFI */
    if (s_stage != HFI_STAGE_RUN) {
        return 0u;
    }
    hfi_iq_pull_try_arm();
    return (s_iq_pull_done != 0u) ? 1u : 0u;
#else
    /* S1/S2/S3c0a(7)/S3c0b(8)：RUN 开速度环；ω_ff 仍由 OMEGA_FF_SRC 管（须为 0。*/
    return (s_stage == HFI_STAGE_RUN) ? 1u : 0u;
#endif
#endif
#else
    return 0u;
#endif
}

float hfi_sqwave_get_speed_ref_rpm(void)
{
#if M1_HFI_DELTA_SWEEP_ENABLE
    return 0.0f;
#else
#if M1_HFI_QKICK_START_ENABLE
    if (s_stage == HFI_STAGE_CRAWL) {
        return s_start_rpm_cmd;
    }
#endif
#if M1_HFI_QKICK_IF_ENABLE
    if (s_stage == HFI_STAGE_CRAWL) {
        return s_if_rpm_cmd;
    }
#endif
#if M1_HFI_MOTION_BYPASS_ENABLE
    if (s_stage == HFI_STAGE_RUN) {
#if M1_HFI_DQ_IDENT_ENABLE
        if (s_dq_step != (uint8_t)HFI_DQ_SPEED) {
            return 0.0f;
        }
#endif
#if M1_HFI_SPD_CLOSE_ENABLE
        return hfi_spd_close_ref_rpm();
#elif (M1_HFI_GATE == 51) || (M1_HFI_GATE == 52)
#if M1_HFI_IQ_PULL_ENABLE
        /* 拉起段禁。ladder_poll，避免未开速度环时档位空转 */
        if (s_iq_pull_done == 0u) {
            return 0.0f;
        }
#endif
        /* 。speed_ident 。omega_ref 同层：此处推进档位，on_angle 不碰 */
        return hfi_run_ladder_poll();
#else
        return hfi_run_speed_ref_rpm();
#endif
    }
#endif
    return 0.0f;
#endif
}

uint8_t hfi_sqwave_if_leave_active(void)
{
#if M1_HFI_QKICK_IF_ENABLE
    return ((s_stage == HFI_STAGE_CRAWL) && (s_if_active != 0u)) ? 1u : 0u;
#else
    return 0u;
#endif
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
#if M1_HFI_PLL_VESC_ERR_ENABLE
    return s_e_pll;
#else
    return s_eps;
#endif
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
#if M1_HFI_DEMOD_PROBE_ENABLE
    return hfi_demod_probe_freeze();
#else
    return 0u;
#endif
}

uint8_t hfi_sqwave_feed_coast_active(void)
{
#if M1_HFI_FEED_COAST_ENABLE
    if ((s_stage == HFI_STAGE_RUN) && (s_qk_done != 0u) &&
        (s_stage_t >= M1_HFI_FEED_COAST_T0_S) &&
        (s_stage_t < M1_HFI_FEED_COAST_T1_S)) {
        return 1u;
    }
#endif
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
#if (M1_HFI_GATE == 38) || (M1_HFI_GATE == 53) || (M1_HFI_GATE == 54) || \
    (M1_HFI_GATE == 55) || (M1_HFI_GATE == 56) || (M1_HFI_GATE == 57) || (M1_HFI_GATE == 58) || (M1_HFI_GATE == 59) || (M1_HFI_GATE == 60) || (M1_HFI_GATE == 61) || (M1_HFI_GATE == 62) || (M1_HFI_GATE == 63) || (M1_HFI_GATE == 64) || (M1_HFI_GATE == 65) || (M1_HFI_GATE == 66) || (M1_HFI_GATE == 67) || (M1_HFI_GATE == 68) || (M1_HFI_GATE == 69) || (M1_HFI_GATE == 70) || (M1_HFI_GATE == 71) || (M1_HFI_GATE == 72) || (M1_HFI_GATE == 73) || (M1_HFI_GATE == 74) || (M1_HFI_GATE == 75) || (M1_HFI_GATE == 76) || (M1_HFI_GATE == 77) || (M1_HFI_GATE == 78) || (M1_HFI_GATE == 79) || (M1_HFI_GATE == 80) || (M1_HFI_GATE == 91) || (M1_HFI_GATE == 92) || (M1_HFI_GATE == 93)
    uint8_t pulse = s_pol_flip_pulse;

    s_pol_flip_pulse = 0u;
    return pulse;
#else
    return 0u;
#endif
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
#if M1_HFI_ID_PI_OFF_ENABLE
    if (s_id_pi_release == 0u) {
        return 0.0f;
    }
    if (s_id_pi_soft_cmd >= 0.0f) {
        return s_id_pi_soft_cmd;
    }
    if (M1_HFI_ID_PI_SOFT_N == 0u) {
        return 1.0f;
    }
    return (float)s_id_pi_soft_n / (float)M1_HFI_ID_PI_SOFT_N;
#else
    return 1.0f;
#endif
}

float hfi_sqwave_id_pi_soft_scale(void)
{
#if M1_HFI_ID_PI_OFF_ENABLE
    if (s_id_pi_release == 0u) {
        return 0.0f;
    }
    if (s_id_pi_soft_cmd >= 0.0f) {
        return s_id_pi_soft_cmd;
    }
    if (M1_HFI_ID_PI_SOFT_N == 0u) {
        return 1.0f;
    }
    if (s_id_pi_soft_n < M1_HFI_ID_PI_SOFT_N) {
        s_id_pi_soft_n++;
    }
    return (float)s_id_pi_soft_n / (float)M1_HFI_ID_PI_SOFT_N;
#else
    return 1.0f;
#endif
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
#if (M1_HFI_GATE == 118) || (M1_HFI_GATE == 119) || (M1_HFI_GATE == 120) || \
    (M1_HFI_GATE == 121) || (M1_HFI_GATE == 122) || (M1_HFI_GATE == 123) || \
    (M1_HFI_GATE == 124) || (M1_HFI_GATE == 125) || (M1_HFI_GATE == 126) || \
    (M1_HFI_GATE == 127) || (M1_HFI_GATE == 128) || (M1_HFI_GATE == 129) || (M1_HFI_GATE == 130) || (M1_HFI_GATE == 131) || (M1_HFI_GATE == 138) || (M1_HFI_GATE == 132) || (M1_HFI_GATE == 133) || (M1_HFI_GATE == 134) || (M1_HFI_GATE == 135) || (M1_HFI_GATE == 136) || (M1_HFI_GATE == 137) || (M1_HFI_GATE == 139) || (M1_HFI_GATE == 140) || (M1_HFI_GATE == 141)
    /* 下一拍积分直接等于这次播进去的转速，避免低通还留着上一拍 HFI。 */
    s_speed_est = omega_el;
#endif
}

void hfi_sqwave_set_iq_auth_hold(uint8_t hold)
{
#if M1_HFI_IQ_AUTH_ENABLE
    s_iq_auth_hold = (hold != 0u) ? 1u : 0u;
#else
    (void)hold;
#endif
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
#if M1_HFI_AXIS_SEL_ENABLE
    return s_eps_d_lp;
#else
    return 0.0f;
#endif
}

float hfi_sqwave_get_axis_flip_n(void)
{
#if M1_HFI_AXIS_SEL_ENABLE
    return (float)s_axis_flip_n;
#elif M1_HFI_LQ_WELL_FLIP_ENABLE
    return (float)s_lq_well_flip_n;
#else
    return 0.0f;
#endif
}

float hfi_sqwave_get_lq_well_flip_n(void)
{
#if M1_HFI_LQ_WELL_FLIP_ENABLE
    return (float)s_lq_well_flip_n;
#else
    return 0.0f;
#endif
}

uint8_t hfi_sqwave_axis_ok(void)
{
#if M1_HFI_AXIS_SEL_ENABLE
    return s_axis_ok;
#else
    return 1u;
#endif
}

float hfi_sqwave_get_iq_auth_abs(void)
{
#if M1_HFI_IQ_AUTH_ENABLE
    return s_iq_auth_abs;
#else
    return 11.0f;
#endif
}

uint8_t hfi_sqwave_iq_auth_ok(void)
{
#if M1_HFI_IQ_AUTH_ENABLE
    return s_iq_auth_ok;
#else
    return 1u;
#endif
}

uint8_t hfi_sqwave_qk_pre_ok(void)
{
#if M1_HFI_QKICK_PRE_GATE_ENABLE
    return s_qk_pre_ok;
#else
    return 1u;
#endif
}

float hfi_sqwave_get_qk_pre_flip_n(void)
{
#if M1_HFI_QKICK_PRE_GATE_ENABLE
    return (float)s_qk_pre_flip_n;
#else
    return 0.0f;
#endif
}

float hfi_sqwave_get_eps_dead(void)
{
#if M1_HFI_PLL_EPS_DEAD_SWEEP_ENABLE && M1_HFI_QKICK_START_ENABLE
    /* VOFA：dead + 0.1·leg */
    return s_pll_eps_dead + 0.1f * (float)s_eps_dead_i;
#elif M1_HFI_BIAS_CAL_ENABLE
    /* VOFA：dead。0.1 表示偏置已冻结并施加 */
    return s_pll_eps_dead + ((s_bias_frozen != 0u) ? 0.1f : 0.0f);
#else
    {
        float d = s_pll_eps_dead;

#if M1_HFI_PLL_HOLD_ENABLE
        if (s_pll_hold != 0u) {
            d += 0.5f; /* VOFA stage。.5=HOLD。.0=TRACK */
        }
#endif
        return d;
    }
#endif
}

uint8_t hfi_sqwave_get_ipd_phase(void)
{
#if M1_HFI_POLARITY_IPD_ENABLE
    return s_ipd_phase;
#else
    return 0u;
#endif
}

float hfi_sqwave_get_ipd_pulse_ud(void)
{
#if M1_HFI_POLARITY_IPD_ENABLE && M1_HFI_IPD_SWEEP_ENABLE
    return s_ipd_pulse_ud;
#elif M1_HFI_QKICK_ANY
    return s_qk_iq_ref;
#else
    return 0.0f;
#endif
}

uint8_t hfi_sqwave_get_qkick_phase(void)
{
#if M1_HFI_QKICK_ANY
    return s_qk_phase;
#else
    return 0u;
#endif
}

float hfi_sqwave_get_qkick_seed(void)
{
#if M1_HFI_QKICK_ANY
    return (float)s_qk_seed_i;
#else
    return 0.0f;
#endif
}

float hfi_sqwave_get_qkick_dth(void)
{
#if M1_HFI_QKICK_ANY
    return s_qk_dth;
#else
    return 0.0f;
#endif
}

float hfi_sqwave_get_qkick_verdict(void)
{
#if M1_HFI_QKICK_ANY
    return s_qk_verdict;
#else
    return 0.0f;
#endif
}

uint8_t hfi_sqwave_consume_pi_reset(void)
{
#if M1_HFI_QKICK_ANY
    if (s_qk_pi_reset != 0u) {
        s_qk_pi_reset = 0u;
        return 1u;
    }
#endif
    return 0u;
}

uint8_t hfi_sqwave_id_pi_bypass(void)
{
#if M1_HFI_ID_PI_OFF_ENABLE
    /*
     * 注入开着且未放行：旁路，避免 Ud_pi 。±Vh。
     * OVERLAP/OPEN_ID：放行后即使。Vh>0 也走 Id（与开环注入重叠）。
     * 否则：仅 scale。 且放行后才开 Id。
     */
    if (s_stage == HFI_STAGE_LOG) {
        return 1u;
    }
#if M1_HFI_ID_ON_FROM_RUN_ENABLE
    /* RUN 全程（含踢前）Id 常开；LOG 仍旁路。 */
    if (s_stage == HFI_STAGE_RUN) {
        return (s_id_pi_release != 0u) ? 0u : 1u;
    }
#elif M1_HFI_QKICK_ANY
    if ((s_stage == HFI_STAGE_RUN) && (s_qk_done != 0u)) {
#if M1_HFI_HAND_ID_OVERLAP_ENABLE || M1_HFI_HAND_OPEN_ID_ENABLE || \
    M1_HFI_VESC_ID_HANDOFF_ENABLE || M1_HFI_HFI_ID_SOFT_ENABLE
        if (s_id_pi_release != 0u) {
            return 0u;
        }
#else
        if ((s_vh_scale <= 0.0f) && (s_id_pi_release != 0u)) {
            return 0u;
        }
#endif
        return 1u;
    }
#endif
#endif
    return 0u;
}

uint8_t hfi_sqwave_get_sensed_cal_loop(void)
{
#if M1_HFI_SENSED_CAL_ENABLE
    return s_sensed_cal_loop;
#else
    return 0u;
#endif
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

#else /* !M1_HFI_ENABLE */

void hfi_sqwave_init(void) {}
void hfi_sqwave_reset(void) {}
void hfi_sqwave_set_omega_ff_el(float omega_el_rad_s)
{
    (void)omega_el_rad_s;
}
void hfi_sqwave_on_angle(float theta_enc_el, float dt)
{
    (void)theta_enc_el;
    (void)dt;
}
void hfi_sqwave_on_current(float id, float iq, float i_alpha, float i_beta)
{
    (void)id;
    (void)iq;
    (void)i_alpha;
    (void)i_beta;
}
float hfi_sqwave_park_theta(float theta_enc_el)
{
    return theta_enc_el;
}
uint8_t hfi_sqwave_override_voltage(float *ud, float *uq)
{
    (void)ud;
    (void)uq;
    return 0u;
}
void hfi_sqwave_get_inj(float *ud_inj, float *uq_inj)
{
    if (ud_inj != NULL) {
        *ud_inj = 0.0f;
    }
    if (uq_inj != NULL) {
        *uq_inj = 0.0f;
    }
}
void hfi_sqwave_get_inj_ab(float *u_alpha_inj, float *u_beta_inj)
{
    if (u_alpha_inj != NULL) {
        *u_alpha_inj = 0.0f;
    }
    if (u_beta_inj != NULL) {
        *u_beta_inj = 0.0f;
    }
}
float hfi_sqwave_get_iq_ref(void)
{
    return 0.0f;
}
float hfi_sqwave_get_id_ref(void)
{
    return 0.0f;
}
uint8_t hfi_sqwave_speed_run_active(void)
{
    return 0u;
}
float hfi_sqwave_get_speed_ref_rpm(void)
{
    return 0.0f;
}
uint8_t hfi_sqwave_if_leave_active(void)
{
    return 0u;
}
hfi_stage_t hfi_sqwave_get_stage(void)
{
    return HFI_STAGE_IDLE;
}
hfi_lock_t hfi_sqwave_get_lock(void)
{
    return HFI_LOCK_CAPTURE;
}
float hfi_sqwave_get_theta_cmd(void)
{
    return 0.0f;
}
float hfi_sqwave_get_theta_hat(void)
{
    return 0.0f;
}
float hfi_sqwave_get_theta_err(void)
{
    return 0.0f;
}
float hfi_sqwave_get_eps(void)
{
    return 0.0f;
}
float hfi_sqwave_get_pll_vesc_err(void)
{
    return 0.0f;
}
float hfi_sqwave_get_di_q(void)
{
    return 0.0f;
}
float hfi_sqwave_get_di_d(void)
{
    return 0.0f;
}
float hfi_sqwave_get_x_raw(void)
{
    return 0.0f;
}
float hfi_sqwave_get_y_raw(void)
{
    return 0.0f;
}
float hfi_sqwave_get_x_lp(void)
{
    return 0.0f;
}
float hfi_sqwave_get_y_lp(void)
{
    return 0.0f;
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
    return 0.0f;
}
float hfi_sqwave_get_omega_el(void)
{
    return 0.0f;
}
float hfi_sqwave_get_omega_trim_el(void)
{
    return 0.0f;
}
uint8_t hfi_sqwave_take_polarity_flip(void)
{
    return 0u;
}
void hfi_sqwave_set_inj_scale(float scale)
{
    (void)scale;
}
void hfi_sqwave_set_id_pi_release(uint8_t enable)
{
    (void)enable;
}
void hfi_sqwave_set_id_pi_soft_cmd(float scale)
{
    (void)scale;
}
float hfi_sqwave_id_pi_soft_scale(void)
{
    return 1.0f;
}
float hfi_sqwave_get_id_pi_soft(void)
{
    return 1.0f;
}
void hfi_sqwave_set_torque_theta(float theta, uint8_t enable)
{
    (void)theta;
    (void)enable;
}
void hfi_sqwave_set_hat_hold(uint8_t hold)
{
    (void)hold;
}
void hfi_sqwave_set_hat_coast_el(float omega_el)
{
    (void)omega_el;
}
void hfi_sqwave_seed_hat(float theta_el, float omega_el)
{
    (void)theta_el;
    (void)omega_el;
}
void hfi_sqwave_set_iq_auth_hold(uint8_t hold)
{
    (void)hold;
}
void hfi_sqwave_flip_hat_pi(void) {}
float hfi_sqwave_get_pll_int_el(void)
{
    return 0.0f;
}
float hfi_sqwave_get_omega_shadow_el(void)
{
    return 0.0f;
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
    return 11.0f;
}
uint8_t hfi_sqwave_iq_auth_ok(void)
{
    return 1u;
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
    return 0.0f;
}
uint8_t hfi_sqwave_get_ipd_phase(void)
{
    return 0u;
}
float hfi_sqwave_get_ipd_pulse_ud(void)
{
    return 0.0f;
}
uint8_t hfi_sqwave_get_qkick_phase(void)
{
    return 0u;
}
float hfi_sqwave_get_qkick_seed(void)
{
    return 0.0f;
}
float hfi_sqwave_get_qkick_dth(void)
{
    return 0.0f;
}
float hfi_sqwave_get_qkick_verdict(void)
{
    return 0.0f;
}
uint8_t hfi_sqwave_consume_pi_reset(void)
{
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

void hfi_sqwave_telem_publish(void) {}

void hfi_sqwave_telem_harvest(hfi_telem_snap_t *out)
{
    if (out == NULL) {
        return;
    }
    hfi_sqwave_telem_read(out);
}

void hfi_sqwave_telem_read(hfi_telem_snap_t *out)
{
    if (out == NULL) {
        return;
    }
    out->eps = 0.0f;
    out->pll_vesc_err = 0.0f;
    out->x_lp = 0.0f;
    out->y_lp = 0.0f;
    out->x_raw = 0.0f;
    out->y_raw = 0.0f;
    out->di_d = 0.0f;
    out->di_q = 0.0f;
    out->pll_int_el = 0.0f;
    out->qkick_dth = 0.0f;
    out->qkick_verdict = 0.0f;
    out->qk_pre_flip_n = 0.0f;
    out->lq_well_flip_n = 0.0f;
    out->vh_sign = 0.0f;
    out->id_pi_soft = 1.0f;
    out->ud_inj = 0.0f;
    out->uq_inj = 0.0f;
    out->sensed_cal_loop = 0u;
    out->iq_auth_ok = 1u;
    out->qk_pre_ok = 1u;
    out->demod_probe_freeze = 0u;
    out->feed_coast_active = 0u;
}

#endif /* M1_HFI_ENABLE */
