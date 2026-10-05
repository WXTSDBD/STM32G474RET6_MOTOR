/**
 * @file m1_hfi_standstill.profile.h
 * @brief HFI 四步重验收：公共骨架 + �?M1_HFI_GATE 展开�?
 *
 * 切实验只�?config/bringup_active.h 里的 M1_HFI_GATE�?
 * 解调 / PLL / 注入仍走 hfi_sqwave.c 现有状态机，本文件只铺宏�?
 *
 *   GATE=1  S1 开环尺�?005 PASS�?
 *   GATE=2  S2 有感 100 rpm 旁路�?136 PASS�?
 *   GATE=3  S3a 静置闭环 Park=θ̂�?146 PASS�?
 *   GATE=4  S3b q 轴踢�?252 PASS�?
 *   GATE=5  S3c0 Park=θ̂+Iq爬（1305 FAIL，可复现�?
 *   GATE=6  S3c1 对照：爬段力�?Park=enc
 *   GATE=7  S3c0a 相对 S2 只换 Park→θ̂（ω_ff=0�?
 *   GATE=8  S3c0b �?S3c0a + 无感 x 质量�?Iq
 *   GATE=9  C1 捕获瞬态（控制�?S3a�?�? 冻结�?
 *   GATE=10 C2 坏种子（PLL_INIT_OFF=+90°�?
 *   GATE=11 C3 无感质量放行标志（Iq=0，不开速度环）
 *   GATE=12 C3b 质量旗清零滞回（相对 C3 只加 CLEAR_N�?
 *   GATE=13 C4v1 过线后极�?Iq（旧误接 FEED→QKICK 支；1552/1612 FAIL�?
 *   GATE=14 C4   �?knobs，解调后�?Iq（过线前路径≡C3b�?
 *   GATE=15 C4a  相对 C3b 只开 FEED 接线（HI=0，永不出力）
 *   GATE=16 C4b  相对 C4a 只改 HI=0.25�?730 FAIL�?
 *   GATE=17 C4c  HI=0 + FEED_A=0.25（出力与权威天花板拆开�?
 *   GATE=18 C4d  �?C4c + 一上电 A(延时阶跃)→B(斜坡) 对照（C 方案不做�?
 *   GATE=19 C4e  控制≡C4a；仅 VOFA �?x_raw/Ud_inj/di_d 供离线定�?
 *   GATE=20 C4f  C4e+q�?AFTER_LOCK�?132 FAIL：先 HFI 再踢；冻结）
 *   GATE=21 C4g  先踢�?HFI（QKICK_BEFORE_HFI）；2142：Iq 跟住�?Δθ=0；冻�?
 *   GATE=22 C4h  �?C4g + 踢前 HF settle�?224 FAIL）；冻结
 *   GATE=23 C4i  �?C4g + MEAS 清注入（1234 FAIL）；冻结
 *   GATE=24 C4j  S3b �?+ hold 后再 HFI RUN（THEN_HFI）；冻结
 *   GATE=25 C4k  �?C4j + 踢后 LOG/RUN �?Id PI（Ud_pi=0）；冻结
 *   GATE=26 C4l  �?C4k + 踢前 θ̂+=π（FORCE_PI�?402 FAIL；冻结）
 *   GATE=27 C4m  �?C4k + A_cmd=0.238（拆 q 轴假锁；勿叠 FORCE_PI）；冻结
 *   GATE=28 C4n  �?C4k + PRE 假锁门禁（逃�?+ �?pre_ok 才踢）；冻结
 *   GATE=29 C4o  �?C4n + �?BRAKE（踢�?Iq=0 滑行�?LOG）；静置捕获签收
 *   GATE=30 C4p  �?C4o + 过线�?FEED_A=0.5（HI=0�?945/46 PASS 轻出力不掉锁�?
 *   GATE=31 C4q  �?C4p + FEED_A=0.8（探静摩擦；唯一增量�?
 *   GATE=32 C4r  �?C4q 捕获 + 过线后速度环阶�?100�?000 / 100 rpm
 *   GATE=33 C4s  �?C4r + AUTH HI=11 A（对�?SPEED_IDENT/SMO 电流顶）
 *   GATE=34 C4t  �?C4s + �?PLL_W_MAX + ω* 小权重前馈（冻结；指令前馈静�?空转�?
 *   GATE=35 C4u  �?C4t 但关 ω* 前馈/播种；仅保留 W_MAX=1100
 *   GATE=36 C4v  �?C4u + 巡航 100�?00 rpm / 10 s（观察角漂）
 *   GATE=37 C4w  控制≡C4v；VOFA 改巡航有用通道（HFI ω / enc rpm / Iq*�?
 *   GATE=38 C4x  P1：捕获同 C4w；关速度环；FEED_A=0.8×10 s（验带流锁相�?
 *   GATE=39 C4y  �?C4x；FEED_A=1.8（加载对照：�?Iq 压转速看角偏�?
 *   GATE=40 C4z  控制≡C4y；RUN 内两段冻 θ̂ + 解调遥测（y/di_d/di_q�?
 *   GATE=41 C4aa 控制≡C4x（FEED=0.8）；�?C4z 冻角 + 解调遥测；冻�?
 *   GATE=42 C4ab 控制≡C4x；VOFA=�?FEED 分段诊断（不冻角�?
 *   GATE=43 C4ac 控制≡C4x；RUN �?6�? s �?FEED 滑行 2 s（同速对照）
 *   GATE=44 S2b  �?S2；RUN 40 s，便于有�?100 rpm 中途加负载
 *   GATE=45 S2c  �?S2；有感阶�?0�?000 / +100 rpm、每�?8 s
 *   GATE=46 S2d  速度环吃编码器；Park=θ̂�?00 rpm 然后 200 rpm，各 20 s
 *   GATE=47 S2e  �?S2d；阶�?100�?000 / +100 rpm、每�?8 s；ch8=积分转�?
 *   GATE=48 S2f  �?S2e；x≤A 不写积分�?838�?00 rpm 丢步；冻结）
 *   GATE=49 S2g  �?S2e 阶梯；PLL �?S2d（每�?Ki+Kp，无 skip�?
 *   GATE=50 S2h  tacc 五段�?917 100 rpm 失锁；冻结）
 *   GATE=51 S2i  +50 �?3.5s；VH 三档 RO 换档写（0.40/0.50/0.60�?
 *   GATE=52 S2j  转矩拉过 100 rpm×100 ms �?HFI 速度�?100�?00/+50
 *   GATE�?2 �?#error
 */
#ifndef CONFIG_PROFILES_M1_HFI_STANDSTILL_PROFILE_H
#define CONFIG_PROFILES_M1_HFI_STANDSTILL_PROFILE_H

#ifndef M1_HFI_GATE
#define M1_HFI_GATE                     2
#endif
#if (M1_HFI_GATE < 1) || (M1_HFI_GATE > 141)
#error "M1_HFI_GATE: 1..141 ready; higher not implemented yet"
#endif

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
#define M1_SPEED_LOOP_BOOT              0
#undef M1_SPEED_OMEGA_RAMP_ENABLE
#define M1_SPEED_OMEGA_RAMP_ENABLE      0

#undef M1_OPEN_UD_PRE_ID_CAL_ENABLE
#define M1_OPEN_UD_PRE_ID_CAL_ENABLE    0
#undef M1_OPEN_UQ_PRE_ID_CAL_ENABLE
#define M1_OPEN_UQ_PRE_ID_CAL_ENABLE    0

#undef M1_STARTUP_ENABLE
#define M1_STARTUP_ENABLE               0

#define M1_DEADBAND_ENABLE              0
#define M1_DEADBAND_LUT_BAKED_ENABLE    0
#define M1_DEADBAND_NVM_ON_BOOT         0

#undef M1_IF_ENABLE
#define M1_IF_ENABLE                    0
#undef M1_IF_TO_OBS_ENABLE
#define M1_IF_TO_OBS_ENABLE             0

#undef M1_ENC_OPTIONAL_ENABLE
#define M1_ENC_OPTIONAL_ENABLE          0

#undef M1_IQ_REF_A
#define M1_IQ_REF_A                     0.0f

#undef M1_HFI_ENABLE
#define M1_HFI_ENABLE                   1
#undef M1_HFI_MOTION_BYPASS_ENABLE
#define M1_HFI_MOTION_BYPASS_ENABLE     1
#undef M1_HFI_IPD_SWEEP_ENABLE
#define M1_HFI_IPD_SWEEP_ENABLE         0
#undef M1_HFI_QKICK_SWEEP_ENABLE
#define M1_HFI_QKICK_SWEEP_ENABLE       0
#undef M1_HFI_QKICK_AFTER_LOCK_ENABLE
#define M1_HFI_QKICK_AFTER_LOCK_ENABLE  0
#undef M1_HFI_DQ_IDENT_ENABLE
#define M1_HFI_DQ_IDENT_ENABLE          0
#undef M1_HFI_QKICK_SPEED_ENABLE
#define M1_HFI_QKICK_SPEED_ENABLE       0
#undef M1_HFI_QKICK_BRAKE_ENABLE
#define M1_HFI_QKICK_BRAKE_ENABLE       0
#undef M1_HFI_QKICK_CRAWL_ENABLE
#define M1_HFI_QKICK_CRAWL_ENABLE       0
#undef M1_HFI_QKICK_START_ENABLE
#define M1_HFI_QKICK_START_ENABLE       0
#undef M1_HFI_SENSED_CAL_ENABLE
#define M1_HFI_SENSED_CAL_ENABLE        0

#undef M1_HFI_PLL_EPS_DEAD
#define M1_HFI_PLL_EPS_DEAD             0.0f
#undef M1_HFI_VH_V
#define M1_HFI_VH_V                     0.40f
#undef M1_HFI_BIAS_CAL_ENABLE
#define M1_HFI_BIAS_CAL_ENABLE          0
#undef M1_HFI_SEQ_ENABLE
#define M1_HFI_SEQ_ENABLE               1
#undef M1_HFI_FH_HZ
#define M1_HFI_FH_HZ                    10000.0f

#undef M1_HFI_POLARITY_ENC_ENABLE
#define M1_HFI_POLARITY_ENC_ENABLE      0
#undef M1_HFI_POLARITY_IPD_ENABLE
#define M1_HFI_POLARITY_IPD_ENABLE      0
#undef M1_HFI_INIT_FROM_ENC
#define M1_HFI_INIT_FROM_ENC            1
#undef M1_HFI_PLL_INIT_OFF_RAD
#define M1_HFI_PLL_INIT_OFF_RAD         0.0f

#undef M1_HFI_PLL_ENABLE
#define M1_HFI_PLL_ENABLE               1
#undef M1_HFI_AXIS_SEL_ENABLE
#define M1_HFI_AXIS_SEL_ENABLE          0
#undef M1_HFI_PLL_KP
#define M1_HFI_PLL_KP                   39.2f
#undef M1_HFI_PLL_KI
#define M1_HFI_PLL_KI                   389.0f
#undef M1_HFI_PLL_W_MAX
#define M1_HFI_PLL_W_MAX                200.0f
#undef M1_HFI_PLL_INT_MAX
#define M1_HFI_PLL_INT_MAX              M1_HFI_PLL_W_MAX
#undef M1_HFI_PLL_INT_LEAK
#define M1_HFI_PLL_INT_LEAK             0.0f
#undef M1_HFI_PLL_HOLD_ENABLE
#define M1_HFI_PLL_HOLD_ENABLE          0
#undef M1_HFI_LOCK_ENABLE
#define M1_HFI_LOCK_ENABLE              0
#undef M1_HFI_SPEED_FB_ENABLE
#define M1_HFI_SPEED_FB_ENABLE          0
#undef M1_HFI_OMEGA_FF_SRC
#define M1_HFI_OMEGA_FF_SRC             0
#undef M1_HFI_OMEGA_FF_FROM_REF
#define M1_HFI_OMEGA_FF_FROM_REF        0
#undef M1_HFI_OMEGA_SEED_ENABLE
#define M1_HFI_OMEGA_SEED_ENABLE        0
#undef M1_HFI_EPS_SIGN
#define M1_HFI_EPS_SIGN                 1.0f
#undef M1_HFI_ATAN2_ENABLE
#define M1_HFI_ATAN2_ENABLE             1
#undef M1_HFI_A_CMD
#define M1_HFI_A_CMD                    (0.10f)

#undef M1_EMF_VEQ_ENABLE
#define M1_EMF_VEQ_ENABLE               0
#undef M1_EMF_SMO_ENABLE
#define M1_EMF_SMO_ENABLE               0
#undef M1_EMF_PLL_ENABLE
#define M1_EMF_PLL_ENABLE               0
#undef M1_OBS_SOFT_SWITCH_ENABLE
#define M1_OBS_SOFT_SWITCH_ENABLE       0

#undef M1_PLL_ENABLE
#define M1_PLL_ENABLE                   1
#undef M1_VOFA_UNIFIED_12CH
#define M1_VOFA_UNIFIED_12CH            1
#undef M1_TELEM_BRINGUP_K
#define M1_TELEM_BRINGUP_K              12u
#undef M1_TELEM_BRINGUP_DECIMATION
#define M1_TELEM_BRINGUP_DECIMATION     8u /* 2.5 kHz；原 D=2=10kHz，减遥测 ISR/UART 负载 */

#undef M1_VOFA_HFI_12CH
#define M1_VOFA_HFI_12CH                1
#undef M1_VOFA_IF_12CH
#define M1_VOFA_IF_12CH                 0
#undef M1_VOFA_OBS_SMO_12CH
#define M1_VOFA_OBS_SMO_12CH            0
#undef M1_VOFA_OBS_PLL_12CH
#define M1_VOFA_OBS_PLL_12CH            0
#undef M1_VOFA_OBS_VEQ_12CH
#define M1_VOFA_OBS_VEQ_12CH            0

/* -------------------------------------------------------------------------- */
#if M1_HFI_GATE == 1
/* S1：开�?δ 尺。PLL 不更�?θ̂。扫�?LOG→DONE。XY=+1 原始落盘�?005 决策�?flip_xy）�?*/
#undef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE       1
#undef M1_HFI_DELTA_MIN_DEG
#define M1_HFI_DELTA_MIN_DEG            (-90.0f)
#undef M1_HFI_DELTA_MAX_DEG
#define M1_HFI_DELTA_MAX_DEG            (90.0f)
#undef M1_HFI_DELTA_STEP_DEG
#define M1_HFI_DELTA_STEP_DEG           (15.0f)
#undef M1_HFI_DELTA_HOLD_S
#define M1_HFI_DELTA_HOLD_S             (3.0f)
#undef M1_HFI_DELTA_TAIL_S
#define M1_HFI_DELTA_TAIL_S             (5.0f)
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              1
#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             8.0f
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 0.0f
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               1.0f
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 0.0f
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               1.0f
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (1.0f)

#elif M1_HFI_GATE == 94
/* V1 Rel GATE1：开环 δ 尺。只改注入/解调路径，Park/RUN/A_cmd/XY 不动。 */
#undef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE       1
#undef M1_HFI_DELTA_MIN_DEG
#define M1_HFI_DELTA_MIN_DEG            (-90.0f)
#undef M1_HFI_DELTA_MAX_DEG
#define M1_HFI_DELTA_MAX_DEG            (90.0f)
#undef M1_HFI_DELTA_STEP_DEG
#define M1_HFI_DELTA_STEP_DEG           (15.0f)
#undef M1_HFI_DELTA_HOLD_S
#define M1_HFI_DELTA_HOLD_S             (3.0f)
#undef M1_HFI_DELTA_TAIL_S
#define M1_HFI_DELTA_TAIL_S             (5.0f)
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              1
#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             8.0f
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 0.0f
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               1.0f
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 0.0f
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               1.0f
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (1.0f)
#undef M1_HFI_INJECT_POST_LOOP
#define M1_HFI_INJECT_POST_LOOP         1
#undef M1_HFI_DEMOD_INJ_AXIS
#define M1_HFI_DEMOD_INJ_AXIS           1
#undef M1_HFI_INJECT_AB_ENABLE
#define M1_HFI_INJECT_AB_ENABLE         0

#elif M1_HFI_GATE == 2
/* S2：有�?100 rpm。Park=enc，ω_ff=0�?
 * 落盘：S1 flip_xy（XY=�?）。A_cmd=+0.10�?123：开�?0.238 �?闭环 x，�?切口）�?
 * 注入/解调旋转已与 Park=θ̂ 对齐；勿再给 S2 另备 XY�?*/
#undef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE       0
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              0
#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             5.0f
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 100.0f
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               25.0f
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 100.0f
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               0.0f
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (-1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (-1.0f)
#undef M1_HFI_A_CMD
#define M1_HFI_A_CMD                    (0.10f)

#elif M1_HFI_GATE == 95
/* V2 Rel GATE2：有感 100 rpm。Park=enc，ω_ff=0。注入/解调同 94。Id PI 开。 */
#undef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE       0
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              0
#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             5.0f
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 100.0f
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               25.0f
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 100.0f
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               0.0f
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (-1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (-1.0f)
#undef M1_HFI_A_CMD
#define M1_HFI_A_CMD                    (0.10f)
#undef M1_HFI_INJECT_POST_LOOP
#define M1_HFI_INJECT_POST_LOOP         1
#undef M1_HFI_DEMOD_INJ_AXIS
#define M1_HFI_DEMOD_INJ_AXIS           1
#undef M1_HFI_INJECT_AB_ENABLE
#define M1_HFI_INJECT_AB_ENABLE         0
#undef M1_HFI_ID_PI_OFF_ENABLE
#define M1_HFI_ID_PI_OFF_ENABLE         0
#undef M1_HFI_ID_ON_FROM_RUN_ENABLE
#define M1_HFI_ID_ON_FROM_RUN_ENABLE    1
#undef M1_HFI_OMEGA_FF_SRC
#define M1_HFI_OMEGA_FF_SRC             0
#undef M1_HFI_OMEGA_FF_FROM_REF
#define M1_HFI_OMEGA_FF_FROM_REF        0

#elif M1_HFI_GATE == 96
/* V3 Rel GATE3：静置 Iq=0，Park=θ̂。1813：须 speed_run_active=0，勿开速环。 */
#undef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE       0
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              1
#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             5.0f
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 0.0f
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               15.0f
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 0.0f
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               0.0f
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (-1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (-1.0f)
#undef M1_HFI_A_CMD
#define M1_HFI_A_CMD                    (0.10f)
#undef M1_HFI_INJECT_POST_LOOP
#define M1_HFI_INJECT_POST_LOOP         1
#undef M1_HFI_DEMOD_INJ_AXIS
#define M1_HFI_DEMOD_INJ_AXIS           1
#undef M1_HFI_INJECT_AB_ENABLE
#define M1_HFI_INJECT_AB_ENABLE         0
#undef M1_HFI_ID_PI_OFF_ENABLE
#define M1_HFI_ID_PI_OFF_ENABLE         0
#undef M1_HFI_ID_ON_FROM_RUN_ENABLE
#define M1_HFI_ID_ON_FROM_RUN_ENABLE    1
#undef M1_HFI_OMEGA_FF_SRC
#define M1_HFI_OMEGA_FF_SRC             0
#undef M1_HFI_OMEGA_FF_FROM_REF
#define M1_HFI_OMEGA_FF_FROM_REF        0

#elif M1_HFI_GATE == 97
/* V4 Rel GATE4：q 踢 1.6 A×300 ms。关对称刹车。注入/解调同 94。Id PI 开、踢时不清 Id 积分。 */
#undef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE       0
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              1
#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             5.0f
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 0.0f
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               8.0f
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 0.0f
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               0.0f
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (-1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (-1.0f)
#undef M1_HFI_A_CMD
#define M1_HFI_A_CMD                    (0.10f)
#undef M1_HFI_POLARITY_IPD_ENABLE
#define M1_HFI_POLARITY_IPD_ENABLE      0
#undef M1_HFI_INIT_FROM_ENC
#define M1_HFI_INIT_FROM_ENC            1
#undef M1_HFI_QKICK_AFTER_LOCK_ENABLE
#define M1_HFI_QKICK_AFTER_LOCK_ENABLE  1
#undef M1_HFI_QKICK_PRE_S
#define M1_HFI_QKICK_PRE_S              2.0f
#undef M1_HFI_QKICK_IQ_A
#define M1_HFI_QKICK_IQ_A               1.6f
#undef M1_HFI_QKICK_KICK_N
#define M1_HFI_QKICK_KICK_N             6000u
#undef M1_HFI_QKICK_BRAKE_ENABLE
#define M1_HFI_QKICK_BRAKE_ENABLE       0
#undef M1_HFI_QKICK_BRAKE_N
#define M1_HFI_QKICK_BRAKE_N            6000u
#undef M1_HFI_QKICK_HOLD_S
#define M1_HFI_QKICK_HOLD_S             5.0f
#undef M1_HFI_QKICK_SPEED_ENABLE
#define M1_HFI_QKICK_SPEED_ENABLE       0
#undef M1_HFI_QKICK_CRAWL_ENABLE
#define M1_HFI_QKICK_CRAWL_ENABLE       0
#undef M1_HFI_QKICK_FORCE_PI
#define M1_HFI_QKICK_FORCE_PI           0
#undef M1_HFI_INJECT_POST_LOOP
#define M1_HFI_INJECT_POST_LOOP         1
#undef M1_HFI_DEMOD_INJ_AXIS
#define M1_HFI_DEMOD_INJ_AXIS           1
#undef M1_HFI_INJECT_AB_ENABLE
#define M1_HFI_INJECT_AB_ENABLE         0
#undef M1_HFI_ID_PI_OFF_ENABLE
#define M1_HFI_ID_PI_OFF_ENABLE         0
#undef M1_HFI_ID_ON_FROM_RUN_ENABLE
#define M1_HFI_ID_ON_FROM_RUN_ENABLE    1
#undef M1_HFI_OMEGA_FF_SRC
#define M1_HFI_OMEGA_FF_SRC             0
#undef M1_HFI_OMEGA_FF_FROM_REF
#define M1_HFI_OMEGA_FF_FROM_REF        0

#elif M1_HFI_GATE == 98
/* V5 Rel C4p/GATE30：C4o 时间线 + 过线 FEED 0.5 A。Id PI 开。注入/解调同 94。
 * 1833：踢段不解 x；THEN_HFI 保留 LOG 末盆地。
 * 1843：FEED 阶跃进半周差，开 DEMOD_HP（同 86）。
 * 1853：关掉丢掉半周。VESC 残差不在本档（先走 99 静态尺）。 */
#undef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE       0
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              1
#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             5.0f
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 0.0f
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               15.0f
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 0.0f
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               0.0f
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (-1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (-1.0f)
#undef M1_HFI_A_CMD
#define M1_HFI_A_CMD                    (0.10f)
#undef M1_HFI_PLL_INIT_OFF_RAD
#define M1_HFI_PLL_INIT_OFF_RAD         0.0f
#undef M1_HFI_INIT_FROM_ENC
#define M1_HFI_INIT_FROM_ENC            1
#undef M1_HFI_POLARITY_IPD_ENABLE
#define M1_HFI_POLARITY_IPD_ENABLE      0
#undef M1_HFI_QKICK_AFTER_LOCK_ENABLE
#define M1_HFI_QKICK_AFTER_LOCK_ENABLE  1
#undef M1_HFI_QKICK_BEFORE_HFI_ENABLE
#define M1_HFI_QKICK_BEFORE_HFI_ENABLE  0
#undef M1_HFI_QKICK_THEN_HFI_ENABLE
#define M1_HFI_QKICK_THEN_HFI_ENABLE    1
#undef M1_HFI_QKICK_PRE_S
#define M1_HFI_QKICK_PRE_S              2.0f
#undef M1_HFI_QKICK_IQ_A
#define M1_HFI_QKICK_IQ_A               1.6f
#undef M1_HFI_QKICK_KICK_N
#define M1_HFI_QKICK_KICK_N             6000u
#undef M1_HFI_QKICK_BRAKE_ENABLE
#define M1_HFI_QKICK_BRAKE_ENABLE       0
#undef M1_HFI_QKICK_BRAKE_N
#define M1_HFI_QKICK_BRAKE_N            6000u
#undef M1_HFI_QKICK_HOLD_S
#define M1_HFI_QKICK_HOLD_S             0.4f
#undef M1_HFI_QKICK_SPEED_ENABLE
#define M1_HFI_QKICK_SPEED_ENABLE       0
#undef M1_HFI_QKICK_CRAWL_ENABLE
#define M1_HFI_QKICK_CRAWL_ENABLE       0
#undef M1_HFI_QKICK_START_ENABLE
#define M1_HFI_QKICK_START_ENABLE       0
#undef M1_HFI_ID_PI_OFF_ENABLE
#define M1_HFI_ID_PI_OFF_ENABLE         0
#undef M1_HFI_ID_ON_FROM_RUN_ENABLE
#define M1_HFI_ID_ON_FROM_RUN_ENABLE    1
#undef M1_HFI_QKICK_FORCE_PI
#define M1_HFI_QKICK_FORCE_PI           0
#undef M1_HFI_QKICK_PRE_GATE_ENABLE
#define M1_HFI_QKICK_PRE_GATE_ENABLE    1
#undef M1_HFI_IQ_AUTH_ENABLE
#define M1_HFI_IQ_AUTH_ENABLE           1
#undef M1_HFI_IQ_AUTH_X_GOOD
#define M1_HFI_IQ_AUTH_X_GOOD           (0.218f)
#undef M1_HFI_IQ_AUTH_X_BAD
#define M1_HFI_IQ_AUTH_X_BAD            (0.205f)
#undef M1_HFI_IQ_AUTH_EPS_FALSE_MAX
#define M1_HFI_IQ_AUTH_EPS_FALSE_MAX    (0.20f)
#undef M1_HFI_IQ_AUTH_HOLD_N
#define M1_HFI_IQ_AUTH_HOLD_N           2000u
#undef M1_HFI_IQ_AUTH_CLEAR_N
#define M1_HFI_IQ_AUTH_CLEAR_N          1000u
#undef M1_HFI_IQ_AUTH_IQ_LO
#define M1_HFI_IQ_AUTH_IQ_LO            (0.0f)
#undef M1_HFI_IQ_AUTH_IQ_HI
#define M1_HFI_IQ_AUTH_IQ_HI            (0.0f)
#undef M1_HFI_IQ_AUTH_SLEW_A_S
#define M1_HFI_IQ_AUTH_SLEW_A_S         (10.0f)
#undef M1_HFI_IQ_AUTH_FEED_ENABLE
#define M1_HFI_IQ_AUTH_FEED_ENABLE      1
#undef M1_HFI_IQ_AUTH_FEED_SIGN
#define M1_HFI_IQ_AUTH_FEED_SIGN        (1.0f)
#undef M1_HFI_IQ_AUTH_FEED_A
#define M1_HFI_IQ_AUTH_FEED_A           (0.50f)
#undef M1_HFI_IQ_AUTH_FEED_DELAY_S
#define M1_HFI_IQ_AUTH_FEED_DELAY_S     (0.20f)
#undef M1_HFI_IQ_AUTH_FEED_RAMP_S
#define M1_HFI_IQ_AUTH_FEED_RAMP_S      (0.0f)
#undef M1_HFI_IQ_AUTH_FEED_LEGACY_BRANCH
#define M1_HFI_IQ_AUTH_FEED_LEGACY_BRANCH 0
#undef M1_HFI_INJECT_POST_LOOP
#define M1_HFI_INJECT_POST_LOOP         1
#undef M1_HFI_DEMOD_INJ_AXIS
#define M1_HFI_DEMOD_INJ_AXIS           1
#undef M1_HFI_DEMOD_HP_ENABLE
#define M1_HFI_DEMOD_HP_ENABLE          1
#undef M1_HFI_DEMOD_HP_A
#define M1_HFI_DEMOD_HP_A               (0.05f) /* ~160 Hz；FEED 慢 Iq 不进 x,y */
#undef M1_HFI_DEMOD_SKIP_OUTLIER
#define M1_HFI_DEMOD_SKIP_OUTLIER       0
#undef M1_HFI_PLL_VESC_ERR_ENABLE
#define M1_HFI_PLL_VESC_ERR_ENABLE      0
#undef M1_HFI_INJECT_AB_ENABLE
#define M1_HFI_INJECT_AB_ENABLE         0
#undef M1_HFI_OMEGA_FF_SRC
#define M1_HFI_OMEGA_FF_SRC             0
#undef M1_HFI_OMEGA_FF_FROM_REF
#define M1_HFI_OMEGA_FF_FROM_REF        0

#elif M1_HFI_GATE == 99
/* VESC 尺 Rel GATE94/S1：开环 δ，Park=enc+δ，PLL 不写 θ̂。
 * 只加 V4 残差（Ld/Lq/Vh/f）。限幅先放开，用 CSV 对 δ 整定符号和斜率。 */
#undef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE       1
#undef M1_HFI_DELTA_MIN_DEG
#define M1_HFI_DELTA_MIN_DEG            (-90.0f)
#undef M1_HFI_DELTA_MAX_DEG
#define M1_HFI_DELTA_MAX_DEG            (90.0f)
#undef M1_HFI_DELTA_STEP_DEG
#define M1_HFI_DELTA_STEP_DEG           (15.0f)
#undef M1_HFI_DELTA_HOLD_S
#define M1_HFI_DELTA_HOLD_S             (3.0f)
#undef M1_HFI_DELTA_TAIL_S
#define M1_HFI_DELTA_TAIL_S             (5.0f)
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              1
#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             8.0f
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 0.0f
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               1.0f
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 0.0f
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               1.0f
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (1.0f)
#undef M1_HFI_INJECT_POST_LOOP
#define M1_HFI_INJECT_POST_LOOP         1
#undef M1_HFI_DEMOD_INJ_AXIS
#define M1_HFI_DEMOD_INJ_AXIS           1
#undef M1_HFI_INJECT_AB_ENABLE
#define M1_HFI_INJECT_AB_ENABLE         0
#undef M1_HFI_PLL_VESC_ERR_ENABLE
#define M1_HFI_PLL_VESC_ERR_ENABLE      1
#undef M1_HFI_PLL_VESC_MAX_ERR
#define M1_HFI_PLL_VESC_MAX_ERR         (10.0f) /* 本档不砍，看对 δ 的斜率 */
#undef M1_HFI_PLL_VESC_ERR_SIGN
#define M1_HFI_PLL_VESC_ERR_SIGN        (1.0f)

#elif M1_HFI_GATE == 100
/* VESC 旁路 Rel GATE95：Park=enc，100 rpm，ω_ff=0。
 * 残差同 99；XY 用 S2 的 −1，SIGN 取反以保持 e 与 δ 关系。限幅仍放开。 */
#undef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE       0
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              0
#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             5.0f
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 100.0f
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               25.0f
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 100.0f
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               0.0f
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (-1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (-1.0f)
#undef M1_HFI_A_CMD
#define M1_HFI_A_CMD                    (0.10f)
#undef M1_HFI_INJECT_POST_LOOP
#define M1_HFI_INJECT_POST_LOOP         1
#undef M1_HFI_DEMOD_INJ_AXIS
#define M1_HFI_DEMOD_INJ_AXIS           1
#undef M1_HFI_INJECT_AB_ENABLE
#define M1_HFI_INJECT_AB_ENABLE         0
#undef M1_HFI_ID_PI_OFF_ENABLE
#define M1_HFI_ID_PI_OFF_ENABLE         0
#undef M1_HFI_ID_ON_FROM_RUN_ENABLE
#define M1_HFI_ID_ON_FROM_RUN_ENABLE    1
#undef M1_HFI_OMEGA_FF_SRC
#define M1_HFI_OMEGA_FF_SRC             0
#undef M1_HFI_OMEGA_FF_FROM_REF
#define M1_HFI_OMEGA_FF_FROM_REF        0
#undef M1_HFI_PLL_VESC_ERR_ENABLE
#define M1_HFI_PLL_VESC_ERR_ENABLE      1
#undef M1_HFI_PLL_VESC_MAX_ERR
#define M1_HFI_PLL_VESC_MAX_ERR         (10.0f)
#undef M1_HFI_PLL_VESC_ERR_SIGN
#define M1_HFI_PLL_VESC_ERR_SIGN        (-1.0f)

#elif M1_HFI_GATE == 101
/* Rel 100 + 旧 q 踢极性：MEAS Park=θ̂ 踢 1.6 A；反转则 θ̂+=π。
 * RUN Park=enc、100 rpm（SENSED_CAL 一档）。V4 残差同 100。 */
#undef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE       0
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              1
#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             5.0f
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 100.0f
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               25.0f
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 100.0f
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               0.0f
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (-1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (-1.0f)
#undef M1_HFI_A_CMD
#define M1_HFI_A_CMD                    (0.10f)
#undef M1_HFI_INIT_FROM_ENC
#define M1_HFI_INIT_FROM_ENC            1
#undef M1_HFI_QKICK_AFTER_LOCK_ENABLE
#define M1_HFI_QKICK_AFTER_LOCK_ENABLE  1
#undef M1_HFI_QKICK_PRE_S
#define M1_HFI_QKICK_PRE_S              2.0f
#undef M1_HFI_QKICK_IQ_A
#define M1_HFI_QKICK_IQ_A               1.6f
#undef M1_HFI_QKICK_KICK_N
#define M1_HFI_QKICK_KICK_N             6000u
#undef M1_HFI_QKICK_BRAKE_ENABLE
#define M1_HFI_QKICK_BRAKE_ENABLE       0
#undef M1_HFI_QKICK_HOLD_S
#define M1_HFI_QKICK_HOLD_S             0.4f
#undef M1_HFI_QKICK_SPEED_ENABLE
#define M1_HFI_QKICK_SPEED_ENABLE       1
#undef M1_HFI_QKICK_CRAWL_ENABLE
#define M1_HFI_QKICK_CRAWL_ENABLE       0
#undef M1_HFI_QKICK_START_ENABLE
#define M1_HFI_QKICK_START_ENABLE       0
#undef M1_HFI_QKICK_BEFORE_HFI_ENABLE
#define M1_HFI_QKICK_BEFORE_HFI_ENABLE  0
#undef M1_HFI_SENSED_CAL_ENABLE
#define M1_HFI_SENSED_CAL_ENABLE        1
#undef M1_HFI_SENSED_CAL_LOOPS
#define M1_HFI_SENSED_CAL_LOOPS         1u
#undef M1_HFI_SENSED_CAL_NRPM
#define M1_HFI_SENSED_CAL_NRPM          1u
#undef M1_HFI_SENSED_CAL_STEP_S
#define M1_HFI_SENSED_CAL_STEP_S        25.0f
#undef M1_HFI_SENSED_CAL_RPM0
#define M1_HFI_SENSED_CAL_RPM0          100.0f
#undef M1_HFI_INJECT_POST_LOOP
#define M1_HFI_INJECT_POST_LOOP         1
#undef M1_HFI_DEMOD_INJ_AXIS
#define M1_HFI_DEMOD_INJ_AXIS           1
#undef M1_HFI_INJECT_AB_ENABLE
#define M1_HFI_INJECT_AB_ENABLE         0
#undef M1_HFI_ID_PI_OFF_ENABLE
#define M1_HFI_ID_PI_OFF_ENABLE         0
#undef M1_HFI_ID_ON_FROM_RUN_ENABLE
#define M1_HFI_ID_ON_FROM_RUN_ENABLE    1
#undef M1_HFI_OMEGA_FF_SRC
#define M1_HFI_OMEGA_FF_SRC             0
#undef M1_HFI_OMEGA_FF_FROM_REF
#define M1_HFI_OMEGA_FF_FROM_REF        0
#undef M1_HFI_SPEED_FB_ENABLE
#define M1_HFI_SPEED_FB_ENABLE          0
#undef M1_HFI_PLL_VESC_ERR_ENABLE
#define M1_HFI_PLL_VESC_ERR_ENABLE      1
#undef M1_HFI_PLL_VESC_MAX_ERR
#define M1_HFI_PLL_VESC_MAX_ERR         (10.0f)
#undef M1_HFI_PLL_VESC_ERR_SIGN
#define M1_HFI_PLL_VESC_ERR_SIGN        (-1.0f)

#elif M1_HFI_GATE == 102
/* Rel 96：踢完极性后静置 Park=θ̂、Iq*=0。不开速度环。对照 1820。 */
#undef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE       0
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              1
#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             5.0f
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 0.0f
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               15.0f
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 0.0f
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               0.0f
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (-1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (-1.0f)
#undef M1_HFI_A_CMD
#define M1_HFI_A_CMD                    (0.10f)
#undef M1_HFI_INIT_FROM_ENC
#define M1_HFI_INIT_FROM_ENC            1
#undef M1_HFI_QKICK_AFTER_LOCK_ENABLE
#define M1_HFI_QKICK_AFTER_LOCK_ENABLE  1
#undef M1_HFI_QKICK_THEN_HFI_ENABLE
#define M1_HFI_QKICK_THEN_HFI_ENABLE    1
#undef M1_HFI_QKICK_PRE_S
#define M1_HFI_QKICK_PRE_S              2.0f
#undef M1_HFI_QKICK_IQ_A
#define M1_HFI_QKICK_IQ_A               1.6f
#undef M1_HFI_QKICK_KICK_N
#define M1_HFI_QKICK_KICK_N             6000u
#undef M1_HFI_QKICK_BRAKE_ENABLE
#define M1_HFI_QKICK_BRAKE_ENABLE       0
#undef M1_HFI_QKICK_HOLD_S
#define M1_HFI_QKICK_HOLD_S             0.4f
#undef M1_HFI_QKICK_SPEED_ENABLE
#define M1_HFI_QKICK_SPEED_ENABLE       0
#undef M1_HFI_QKICK_CRAWL_ENABLE
#define M1_HFI_QKICK_CRAWL_ENABLE       0
#undef M1_HFI_SENSED_CAL_ENABLE
#define M1_HFI_SENSED_CAL_ENABLE        0
#undef M1_HFI_INJECT_POST_LOOP
#define M1_HFI_INJECT_POST_LOOP         1
#undef M1_HFI_DEMOD_INJ_AXIS
#define M1_HFI_DEMOD_INJ_AXIS           1
#undef M1_HFI_INJECT_AB_ENABLE
#define M1_HFI_INJECT_AB_ENABLE         0
#undef M1_HFI_ID_PI_OFF_ENABLE
#define M1_HFI_ID_PI_OFF_ENABLE         0
#undef M1_HFI_ID_ON_FROM_RUN_ENABLE
#define M1_HFI_ID_ON_FROM_RUN_ENABLE    1
#undef M1_HFI_OMEGA_FF_SRC
#define M1_HFI_OMEGA_FF_SRC             0
#undef M1_HFI_OMEGA_FF_FROM_REF
#define M1_HFI_OMEGA_FF_FROM_REF        0
#undef M1_HFI_PLL_VESC_ERR_ENABLE
#define M1_HFI_PLL_VESC_ERR_ENABLE      1
#undef M1_HFI_PLL_VESC_MAX_ERR
#define M1_HFI_PLL_VESC_MAX_ERR         (10.0f)
#undef M1_HFI_PLL_VESC_ERR_SIGN
#define M1_HFI_PLL_VESC_ERR_SIGN        (-1.0f)

#elif (M1_HFI_GATE == 103) || (M1_HFI_GATE == 104) || (M1_HFI_GATE == 106) || \
      (M1_HFI_GATE == 107) || (M1_HFI_GATE == 108) || (M1_HFI_GATE == 109) || \
      (M1_HFI_GATE == 110) || (M1_HFI_GATE == 111) || (M1_HFI_GATE == 112) || \
      (M1_HFI_GATE == 113) || (M1_HFI_GATE == 114) || (M1_HFI_GATE == 115) || \
      (M1_HFI_GATE == 116) || (M1_HFI_GATE == 117) || (M1_HFI_GATE == 118) || \
      (M1_HFI_GATE == 119)
/* Rel C4p/C4q：102 时间线 + 过线 FEED。Id PI 开，V4 残差仍开。
 * 104：MAX_ERR 0.30。106：FEED_A 0.8 阶跃。107：RAMP 0.8 s。
 * 108：FEED_HOLD。109：控制≡108，只换 VOFA。
 * 110：108 + Lq 井一次 +π/2。不开速度环。
 * 113：控制≡108。PLL Kp 39.2→1200。Ki、MAX_ERR、Wmax 不动，不翻轴。
 * 114：控制≡108，Kp 仍 39.2。限幅只夹积分；|e| 贴满 MAX_ERR 时不积分。
 * 115：控制≡114，Kp 仍 39.2。PLL 改吃 ½atan2，A_cmd=0.216。y 残差只留遥测。
 * 116：控制≡115。|ε|<0.5 才积分；限幅只夹积分；角度速率=积分+Kp·ε。
 * 117：控制≡113。e 改为同一注入轴上、高通前的一对 Δiq。
 * 118：控制≡117。积分每拍 += e·(Kp/400)，|I| 不超过 θ̂ 自身转速，角度速率不再夹 200。
 * 119：控制≡118。积分直接等于 θ̂ 转速低通，不再按 e·(Kp/400) 爬。 */
#undef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE       0
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              1
#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             5.0f
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 0.0f
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               15.0f
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 0.0f
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               0.0f
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (-1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (-1.0f)
#undef M1_HFI_A_CMD
#if (M1_HFI_GATE == 115) || (M1_HFI_GATE == 116)
#define M1_HFI_A_CMD                    (0.216f)
#else
#define M1_HFI_A_CMD                    (0.10f)
#endif
#undef M1_HFI_INIT_FROM_ENC
#define M1_HFI_INIT_FROM_ENC            1
#undef M1_HFI_QKICK_AFTER_LOCK_ENABLE
#define M1_HFI_QKICK_AFTER_LOCK_ENABLE  1
#undef M1_HFI_QKICK_THEN_HFI_ENABLE
#define M1_HFI_QKICK_THEN_HFI_ENABLE    1
#undef M1_HFI_QKICK_PRE_S
#define M1_HFI_QKICK_PRE_S              2.0f
#undef M1_HFI_QKICK_IQ_A
#define M1_HFI_QKICK_IQ_A               1.6f
#undef M1_HFI_QKICK_KICK_N
#define M1_HFI_QKICK_KICK_N             6000u
#undef M1_HFI_QKICK_BRAKE_ENABLE
#define M1_HFI_QKICK_BRAKE_ENABLE       0
#undef M1_HFI_QKICK_HOLD_S
#define M1_HFI_QKICK_HOLD_S             0.4f
#undef M1_HFI_QKICK_SPEED_ENABLE
#define M1_HFI_QKICK_SPEED_ENABLE       0
#undef M1_HFI_QKICK_CRAWL_ENABLE
#define M1_HFI_QKICK_CRAWL_ENABLE       0
#undef M1_HFI_SENSED_CAL_ENABLE
#define M1_HFI_SENSED_CAL_ENABLE        0
#undef M1_HFI_INJECT_POST_LOOP
#define M1_HFI_INJECT_POST_LOOP         1
#undef M1_HFI_DEMOD_INJ_AXIS
#define M1_HFI_DEMOD_INJ_AXIS           1
#undef M1_HFI_INJECT_AB_ENABLE
#define M1_HFI_INJECT_AB_ENABLE         0
#undef M1_HFI_ID_PI_OFF_ENABLE
#define M1_HFI_ID_PI_OFF_ENABLE         0
#undef M1_HFI_ID_ON_FROM_RUN_ENABLE
#define M1_HFI_ID_ON_FROM_RUN_ENABLE    1
#undef M1_HFI_OMEGA_FF_SRC
#define M1_HFI_OMEGA_FF_SRC             0
#undef M1_HFI_OMEGA_FF_FROM_REF
#define M1_HFI_OMEGA_FF_FROM_REF        0
#undef M1_HFI_PLL_VESC_ERR_ENABLE
#define M1_HFI_PLL_VESC_ERR_ENABLE      1
#undef M1_HFI_PLL_VESC_MAX_ERR
#if (M1_HFI_GATE == 104) || (M1_HFI_GATE == 106) || (M1_HFI_GATE == 107) || \
    (M1_HFI_GATE == 108) || (M1_HFI_GATE == 109) || (M1_HFI_GATE == 110) || \
    (M1_HFI_GATE == 111) || (M1_HFI_GATE == 112) || (M1_HFI_GATE == 113) || \
    (M1_HFI_GATE == 114) || (M1_HFI_GATE == 115) || (M1_HFI_GATE == 116) || \
    (M1_HFI_GATE == 117) || (M1_HFI_GATE == 118) || (M1_HFI_GATE == 119)
#define M1_HFI_PLL_VESC_MAX_ERR         (0.30f)
#else
#define M1_HFI_PLL_VESC_MAX_ERR         (10.0f)
#endif
#undef M1_HFI_PLL_VESC_ERR_SIGN
#define M1_HFI_PLL_VESC_ERR_SIGN        (-1.0f)
#undef M1_HFI_IQ_AUTH_ENABLE
#define M1_HFI_IQ_AUTH_ENABLE           1
#undef M1_HFI_IQ_AUTH_X_GOOD
#define M1_HFI_IQ_AUTH_X_GOOD           (0.218f)
#undef M1_HFI_IQ_AUTH_X_BAD
#define M1_HFI_IQ_AUTH_X_BAD            (0.205f)
#undef M1_HFI_IQ_AUTH_EPS_FALSE_MAX
#define M1_HFI_IQ_AUTH_EPS_FALSE_MAX    (0.20f)
#undef M1_HFI_IQ_AUTH_HOLD_N
#define M1_HFI_IQ_AUTH_HOLD_N           2000u
#undef M1_HFI_IQ_AUTH_CLEAR_N
#define M1_HFI_IQ_AUTH_CLEAR_N          1000u
#undef M1_HFI_IQ_AUTH_IQ_LO
#define M1_HFI_IQ_AUTH_IQ_LO            (0.0f)
#undef M1_HFI_IQ_AUTH_IQ_HI
#define M1_HFI_IQ_AUTH_IQ_HI            (0.0f)
#undef M1_HFI_IQ_AUTH_SLEW_A_S
#define M1_HFI_IQ_AUTH_SLEW_A_S         (10.0f)
#undef M1_HFI_IQ_AUTH_FEED_ENABLE
#define M1_HFI_IQ_AUTH_FEED_ENABLE      1
#undef M1_HFI_IQ_AUTH_FEED_SIGN
#define M1_HFI_IQ_AUTH_FEED_SIGN        (1.0f)
#undef M1_HFI_IQ_AUTH_FEED_A
#if (M1_HFI_GATE == 106) || (M1_HFI_GATE == 107) || (M1_HFI_GATE == 108) || \
    (M1_HFI_GATE == 109) || (M1_HFI_GATE == 110) || (M1_HFI_GATE == 111) || \
    (M1_HFI_GATE == 112) || (M1_HFI_GATE == 113) || (M1_HFI_GATE == 114) || \
    (M1_HFI_GATE == 115) || (M1_HFI_GATE == 116) ||     (M1_HFI_GATE == 117) || \
    (M1_HFI_GATE == 118) || (M1_HFI_GATE == 119)
#define M1_HFI_IQ_AUTH_FEED_A           (0.80f)
#else
#define M1_HFI_IQ_AUTH_FEED_A           (0.50f)
#endif
#undef M1_HFI_IQ_AUTH_FEED_DELAY_S
#define M1_HFI_IQ_AUTH_FEED_DELAY_S     (0.20f)
#undef M1_HFI_IQ_AUTH_FEED_RAMP_S
#if (M1_HFI_GATE == 107) || (M1_HFI_GATE == 108) || (M1_HFI_GATE == 109) || \
    (M1_HFI_GATE == 110) || (M1_HFI_GATE == 111) || (M1_HFI_GATE == 112) || \
    (M1_HFI_GATE == 113) ||     (M1_HFI_GATE == 114) || (M1_HFI_GATE == 115) || \
    (M1_HFI_GATE == 116) || (M1_HFI_GATE == 117) || (M1_HFI_GATE == 118) || \
    (M1_HFI_GATE == 119)
#define M1_HFI_IQ_AUTH_FEED_RAMP_S      (0.80f)
#else
#define M1_HFI_IQ_AUTH_FEED_RAMP_S      (0.0f)
#endif
#undef M1_HFI_IQ_AUTH_FEED_HOLD
#if (M1_HFI_GATE == 108) || (M1_HFI_GATE == 109) || (M1_HFI_GATE == 110) || \
    (M1_HFI_GATE == 111) || (M1_HFI_GATE == 112) || (M1_HFI_GATE == 113) || \
    (M1_HFI_GATE == 114) || (M1_HFI_GATE == 115) || (M1_HFI_GATE == 116) || \
    (M1_HFI_GATE == 117) || (M1_HFI_GATE == 118) || (M1_HFI_GATE == 119)
#define M1_HFI_IQ_AUTH_FEED_HOLD        1
#else
#define M1_HFI_IQ_AUTH_FEED_HOLD        0
#endif
#undef M1_HFI_IQ_AUTH_FEED_LEGACY_BRANCH
#define M1_HFI_IQ_AUTH_FEED_LEGACY_BRANCH 0
#undef M1_HFI_DEMOD_HP_ENABLE
#define M1_HFI_DEMOD_HP_ENABLE          1
#undef M1_HFI_DEMOD_HP_A
#define M1_HFI_DEMOD_HP_A               (0.05f)
#undef M1_HFI_DEMOD_SKIP_OUTLIER
#define M1_HFI_DEMOD_SKIP_OUTLIER       0
#if (M1_HFI_GATE == 110) || (M1_HFI_GATE == 111) || (M1_HFI_GATE == 112)
/* 84/109：x(0°)/x(90°)≈Lq/Ld；y 在两口井都≈0。一次翻轴，防 2306 连翻。 */
#undef M1_HFI_LQ_WELL_FLIP_ENABLE
#define M1_HFI_LQ_WELL_FLIP_ENABLE      1
#undef M1_HFI_LQ_WELL_X_MAX
#define M1_HFI_LQ_WELL_X_MAX            (0.185f)
#undef M1_HFI_LQ_WELL_Y_ABS
#define M1_HFI_LQ_WELL_Y_ABS            (0.015f)
#undef M1_HFI_LQ_WELL_HOLD_N
#define M1_HFI_LQ_WELL_HOLD_N           20u
#if (M1_HFI_GATE == 111) || (M1_HFI_GATE == 112)
/* 2138：LOG→RUN 时 x 未收敛、Iq*=0 误翻。只认 FEED 指令，不认踢段 1.6 A。 */
#undef M1_HFI_LQ_WELL_IQ_MIN
#define M1_HFI_LQ_WELL_IQ_MIN           (0.50f)
#endif
#if M1_HFI_GATE == 112
/* 2148：122 rpm 时清积分，θ̂ 停转又掉回井。翻轴只加角。 */
#undef M1_HFI_LQ_WELL_KEEP_W
#define M1_HFI_LQ_WELL_KEEP_W           1
#endif
#endif
#if (M1_HFI_GATE == 113) || (M1_HFI_GATE == 117) || (M1_HFI_GATE == 118) || \
    (M1_HFI_GATE == 119)
/* 113：只改 Kp。117：同一 Kp，e 改为同轴原始一对。118：PLL 结构改成 VESC 低速跟踪。
 * 119：同一 Kp。积分直接等于 θ̂ 转速低通。 */
#undef M1_HFI_PLL_KP
#define M1_HFI_PLL_KP                   1200.0f
#endif

#elif (M1_HFI_GATE == 120) || (M1_HFI_GATE == 121) || (M1_HFI_GATE == 122) || \
      (M1_HFI_GATE == 123) || (M1_HFI_GATE == 124) || (M1_HFI_GATE == 125) || \
      (M1_HFI_GATE == 126) || (M1_HFI_GATE == 127) || (M1_HFI_GATE == 128) || (M1_HFI_GATE == 129) || (M1_HFI_GATE == 130) || (M1_HFI_GATE == 131) || (M1_HFI_GATE == 138) || (M1_HFI_GATE == 132) || (M1_HFI_GATE == 133) || (M1_HFI_GATE == 134) || (M1_HFI_GATE == 135) || (M1_HFI_GATE == 136) || (M1_HFI_GATE == 137) || (M1_HFI_GATE == 139) || (M1_HFI_GATE == 140) || (M1_HFI_GATE == 141)
/* 120：速度环吃编码器。121：速度环吃 ω=I+Kp·e，PLL 约 15 Hz。
 * 122：速度环吃 HFI 低通转速。PLL 是 Kp=1200、5 ms。上限 500 rpm。
 * 123：同 122，上限 1500 rpm。每档 100 rpm / 5 s。ω_ff=0。电流顶 2 A。
 * 124：同 123。速度观测 20 ms，角度仍是 Kp=1200。
 * 125：同 123。速度观测回到 5 ms，速度环 Kp=0.005。
 * 126：同 125，上限 2000 rpm。
 * 127：同 125。SMO 只旁路，吃 u−u_hfi。Park 仍是 HFI，注入一直开。
 * 128：同 127。升到 1300 rpm 发布 SMO 角，降到 1000 rpm 交回。
 *     300 rpm 以下补每转 12 次和 14 次的转矩纹波。
 * 129：同 128，但不做补偿。发布角跟速度指令比。
 *     1500 关注入，1400 再开。100→1500 后再降到 900 巡航。
 * 130：同 129。速度环吃发布角的变化率。
 * 131：同 129。100、200 各停 5 s，再 50 rpm/s 爬到 1500，然后按台阶降到 0。
 * 132：同 131 的 HFI。速度反馈在 250 rpm 以下陷波每转 12 次和 14 次。
 *     阶梯 100→500 再降到 0，每档 100 rpm / 5 s。
 * 133：同 132。只在 100 rpm 档补齿槽转矩，每路 ≤0.20 A。
 * 134：同 132 的阶梯。实测转速 0..200 rpm 给速度反馈补 5 ms 相位，不跟指令。
 * 135：同 132 的阶梯。实测 60..140 rpm 按角度超前每转 12 次和 14 次，增益为 1。
 * 136：同 135。解调前减去 150 ms 平均转速。
 * 137：同 136，再超前每转 24 次。阶梯只留 100、200、100。
 * 138：指令在 +1500 与 −1500 之间阶跃，各停 3 s，反复 5 次。
 *     发布和注入跟实测转速的绝对值。
 * 139：100 rpm 停 15 s，再 200 rpm 停 10 s。速度反馈仍是 5 ms。
 *     实测转速 160 以下 Kp=0.015，到 200 回到 0.005。
 * 140：同 139 的阶梯。100 rpm 指令时 Kp=0.015，按实测转速陷掉每转 24 次和 28 次。
 *     指令到 200 rpm 关陷波，Kp 回到 0.005。
 * 141：同 138 的交接。速度观测到 ±1500 后再保持 0.5 s 才换向，反复 5 轮。
 *     发布和注入仍跟实测转速的绝对值。 */
#undef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE       0
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              1
#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             5.0f
#undef M1_HFI_RUN_LADDER_ENABLE
#define M1_HFI_RUN_LADDER_ENABLE        1
#undef M1_HFI_RUN_RPM_START
#define M1_HFI_RUN_RPM_START            100.0f
#undef M1_HFI_RUN_RPM_STEP
#define M1_HFI_RUN_RPM_STEP             100.0f
#undef M1_HFI_RUN_RPM_MAX
#if (M1_HFI_GATE == 123) || (M1_HFI_GATE == 124) || (M1_HFI_GATE == 125) || \
    (M1_HFI_GATE == 127) || (M1_HFI_GATE == 128) || (M1_HFI_GATE == 129) || (M1_HFI_GATE == 130) || (M1_HFI_GATE == 131) || (M1_HFI_GATE == 138) || (M1_HFI_GATE == 141)
#define M1_HFI_RUN_RPM_MAX              1500.0f
#elif M1_HFI_GATE == 126
#define M1_HFI_RUN_RPM_MAX              2000.0f
#else
#define M1_HFI_RUN_RPM_MAX              500.0f
#endif
#undef M1_HFI_RUN_STEP_S
#define M1_HFI_RUN_STEP_S               5.0f
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 100.0f
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               5.0f
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 500.0f
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               20.0f
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (-1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (-1.0f)
#undef M1_HFI_A_CMD
#define M1_HFI_A_CMD                    (0.10f)
#undef M1_HFI_INIT_FROM_ENC
#define M1_HFI_INIT_FROM_ENC            1
#undef M1_HFI_QKICK_AFTER_LOCK_ENABLE
#define M1_HFI_QKICK_AFTER_LOCK_ENABLE  1
#undef M1_HFI_QKICK_THEN_HFI_ENABLE
#define M1_HFI_QKICK_THEN_HFI_ENABLE    1
#undef M1_HFI_QKICK_PRE_S
#define M1_HFI_QKICK_PRE_S              2.0f
#undef M1_HFI_QKICK_IQ_A
#define M1_HFI_QKICK_IQ_A               1.6f
#undef M1_HFI_QKICK_KICK_N
#define M1_HFI_QKICK_KICK_N             6000u
#undef M1_HFI_QKICK_BRAKE_ENABLE
#define M1_HFI_QKICK_BRAKE_ENABLE       0
#undef M1_HFI_QKICK_HOLD_S
#define M1_HFI_QKICK_HOLD_S             0.4f
#undef M1_HFI_QKICK_SPEED_ENABLE
#define M1_HFI_QKICK_SPEED_ENABLE       0
#undef M1_HFI_QKICK_CRAWL_ENABLE
#define M1_HFI_QKICK_CRAWL_ENABLE       0
#undef M1_HFI_SENSED_CAL_ENABLE
#define M1_HFI_SENSED_CAL_ENABLE        0
#undef M1_HFI_INJECT_POST_LOOP
#define M1_HFI_INJECT_POST_LOOP         1
#undef M1_HFI_DEMOD_INJ_AXIS
#define M1_HFI_DEMOD_INJ_AXIS           1
#undef M1_HFI_INJECT_AB_ENABLE
#define M1_HFI_INJECT_AB_ENABLE         0
#undef M1_HFI_ID_PI_OFF_ENABLE
#define M1_HFI_ID_PI_OFF_ENABLE         0
#undef M1_HFI_ID_ON_FROM_RUN_ENABLE
#define M1_HFI_ID_ON_FROM_RUN_ENABLE    1
#undef M1_HFI_OMEGA_FF_SRC
#define M1_HFI_OMEGA_FF_SRC             0
#undef M1_HFI_OMEGA_FF_FROM_REF
#define M1_HFI_OMEGA_FF_FROM_REF        0
#undef M1_HFI_SPEED_FB_ENABLE
#if (M1_HFI_GATE == 121) || (M1_HFI_GATE == 122) || (M1_HFI_GATE == 123) || (M1_HFI_GATE == 124) || (M1_HFI_GATE == 125) || (M1_HFI_GATE == 126) || (M1_HFI_GATE == 127) || (M1_HFI_GATE == 128) || (M1_HFI_GATE == 129) || (M1_HFI_GATE == 130) || (M1_HFI_GATE == 131) || (M1_HFI_GATE == 138) || (M1_HFI_GATE == 132) || (M1_HFI_GATE == 133) || (M1_HFI_GATE == 134) || (M1_HFI_GATE == 135) || (M1_HFI_GATE == 136) || (M1_HFI_GATE == 137) || (M1_HFI_GATE == 139) || (M1_HFI_GATE == 140) || (M1_HFI_GATE == 141)
#define M1_HFI_SPEED_FB_ENABLE          1
#else
#define M1_HFI_SPEED_FB_ENABLE          0
#endif
#undef M1_HFI_SPEED_OBS_INT
#if (M1_HFI_GATE == 122) || (M1_HFI_GATE == 123) || (M1_HFI_GATE == 124) || (M1_HFI_GATE == 125) || (M1_HFI_GATE == 126) || (M1_HFI_GATE == 127) || (M1_HFI_GATE == 128) || (M1_HFI_GATE == 129) || (M1_HFI_GATE == 130) || (M1_HFI_GATE == 131) || (M1_HFI_GATE == 138) || (M1_HFI_GATE == 132) || (M1_HFI_GATE == 133) || (M1_HFI_GATE == 134) || (M1_HFI_GATE == 135) || (M1_HFI_GATE == 136) || (M1_HFI_GATE == 137) || (M1_HFI_GATE == 139) || (M1_HFI_GATE == 140) || (M1_HFI_GATE == 141)
#define M1_HFI_SPEED_OBS_INT            1
#else
#define M1_HFI_SPEED_OBS_INT            0
#endif
#undef M1_HFI_PLL_KP
#if M1_HFI_GATE == 121
#define M1_HFI_PLL_KP                   260.0f
#else
#define M1_HFI_PLL_KP                   1200.0f
#endif
#undef M1_HFI_PLL_VESC_ERR_ENABLE
#define M1_HFI_PLL_VESC_ERR_ENABLE      1
#undef M1_HFI_PLL_VESC_MAX_ERR
#define M1_HFI_PLL_VESC_MAX_ERR         (0.30f)
#undef M1_HFI_PLL_VESC_ERR_SIGN
#define M1_HFI_PLL_VESC_ERR_SIGN        (-1.0f)
#undef M1_HFI_IQ_AUTH_ENABLE
#define M1_HFI_IQ_AUTH_ENABLE           1
#undef M1_HFI_IQ_AUTH_X_GOOD
#define M1_HFI_IQ_AUTH_X_GOOD           (0.218f)
#undef M1_HFI_IQ_AUTH_X_BAD
#define M1_HFI_IQ_AUTH_X_BAD            (0.205f)
#undef M1_HFI_IQ_AUTH_EPS_FALSE_MAX
#define M1_HFI_IQ_AUTH_EPS_FALSE_MAX    (0.20f)
#undef M1_HFI_IQ_AUTH_HOLD_N
#define M1_HFI_IQ_AUTH_HOLD_N           2000u
#undef M1_HFI_IQ_AUTH_CLEAR_N
#define M1_HFI_IQ_AUTH_CLEAR_N          1000u
#undef M1_HFI_IQ_AUTH_IQ_LO
#define M1_HFI_IQ_AUTH_IQ_LO            (0.0f)
#undef M1_HFI_IQ_AUTH_IQ_HI
#define M1_HFI_IQ_AUTH_IQ_HI            (2.0f)
#undef M1_HFI_IQ_AUTH_SLEW_A_S
#define M1_HFI_IQ_AUTH_SLEW_A_S         (4.0f)
#undef M1_HFI_IQ_AUTH_FEED_ENABLE
#define M1_HFI_IQ_AUTH_FEED_ENABLE      0
#undef M1_HFI_DEMOD_HP_ENABLE
#define M1_HFI_DEMOD_HP_ENABLE          1
#undef M1_HFI_DEMOD_HP_A
#define M1_HFI_DEMOD_HP_A               (0.05f)
#undef M1_HFI_DEMOD_SKIP_OUTLIER
#define M1_HFI_DEMOD_SKIP_OUTLIER       0
#undef M1_SPEED_IQ_REF_ABS_MAX
#define M1_SPEED_IQ_REF_ABS_MAX         2.0f
#undef M1_SPEED_PI_OUT_MAX
#define M1_SPEED_PI_OUT_MAX             M1_SPEED_IQ_REF_ABS_MAX
#undef M1_SPEED_PI_OUT_MIN
#define M1_SPEED_PI_OUT_MIN             (-M1_SPEED_IQ_REF_ABS_MAX)
#undef M1_SPEED_PI_INT_MAX
#define M1_SPEED_PI_INT_MAX             M1_SPEED_IQ_REF_ABS_MAX
#undef M1_SPEED_PI_INT_MIN
#define M1_SPEED_PI_INT_MIN             (-M1_SPEED_IQ_REF_ABS_MAX)
#undef M1_SPEED_PI_KI
#define M1_SPEED_PI_KI                  (6.5e-6f)
#undef M1_SPEED_PI_BETA
#define M1_SPEED_PI_BETA                (1.0f)
#if (M1_HFI_GATE == 125) || (M1_HFI_GATE == 126) || (M1_HFI_GATE == 127) || (M1_HFI_GATE == 128) || (M1_HFI_GATE == 129) || (M1_HFI_GATE == 130) || (M1_HFI_GATE == 131) || (M1_HFI_GATE == 138) || (M1_HFI_GATE == 132) || (M1_HFI_GATE == 133) || (M1_HFI_GATE == 134) || (M1_HFI_GATE == 135) || (M1_HFI_GATE == 136) || (M1_HFI_GATE == 137) || (M1_HFI_GATE == 139) || (M1_HFI_GATE == 140) || (M1_HFI_GATE == 141)
#undef M1_SPEED_PI_KP
#define M1_SPEED_PI_KP                  (0.005f)
#endif
#if (M1_HFI_GATE == 127) || (M1_HFI_GATE == 128) || (M1_HFI_GATE == 129) || (M1_HFI_GATE == 130) || (M1_HFI_GATE == 131) || (M1_HFI_GATE == 138) || (M1_HFI_GATE == 132) || (M1_HFI_GATE == 133) || (M1_HFI_GATE == 134) || (M1_HFI_GATE == 135) || (M1_HFI_GATE == 136) || (M1_HFI_GATE == 137) || (M1_HFI_GATE == 141)
/* 签收过的 SMO 增益。127 只旁路。128 到速度窗才换发布角。旧交接状态机不开。 */
#undef M1_EMF_SMO_ENABLE
#define M1_EMF_SMO_ENABLE               1
#undef M1_EMF_PLL_ENABLE
#define M1_EMF_PLL_ENABLE               1
#undef M1_EMF_PLL_USE_SMO
#define M1_EMF_PLL_USE_SMO              1
#undef M1_OBS_SOFT_SWITCH_ENABLE
#define M1_OBS_SOFT_SWITCH_ENABLE       0
#undef M1_HFI_SMO_HAND_ENABLE
#define M1_HFI_SMO_HAND_ENABLE          0
#undef M1_HFI_SMO_SUB_VH_ENABLE
#define M1_HFI_SMO_SUB_VH_ENABLE        1
#undef M1_EMF_SMO_K
#define M1_EMF_SMO_K                    20.0f
#undef M1_EMF_SMO_SAT_A
#define M1_EMF_SMO_SAT_A                0.30f
#undef M1_EMF_SMO_LPF_ENABLE
#define M1_EMF_SMO_LPF_ENABLE           1
#undef M1_EMF_SMO_LPF_HZ
#define M1_EMF_SMO_LPF_HZ               200.0f
#undef M1_EMF_SMO_LPF_SCHED_ENABLE
#define M1_EMF_SMO_LPF_SCHED_ENABLE     0
#undef M1_EMF_SMO_LPF_LINEAR_ENABLE
#define M1_EMF_SMO_LPF_LINEAR_ENABLE    1
#undef M1_EMF_SMO_LPF_LINEAR_K
#define M1_EMF_SMO_LPF_LINEAR_K         1.2f
#undef M1_EMF_SMO_LPF_LINEAR_FC_MIN
#define M1_EMF_SMO_LPF_LINEAR_FC_MIN    100.0f
#undef M1_EMF_SMO_LPF_LINEAR_FC_MAX
#define M1_EMF_SMO_LPF_LINEAR_FC_MAX    280.0f
#undef M1_EMF_SMO_THETA_OFF_RAD
#define M1_EMF_SMO_THETA_OFF_RAD        (0.0f)
#undef M1_EMF_PLL_FN_HZ
#define M1_EMF_PLL_FN_HZ                45.0f
#undef M1_EMF_PLL_ZETA
#define M1_EMF_PLL_ZETA                 0.707106781f
#undef M1_EMF_PLL_NORM_ENABLE
#define M1_EMF_PLL_NORM_ENABLE          1
#undef M1_EMF_LPF_PHASE_FF_ENABLE
#define M1_EMF_LPF_PHASE_FF_ENABLE      1
#undef M1_EMF_PLL_THETA_OFF_RAD
#define M1_EMF_PLL_THETA_OFF_RAD        (0.1147f)
#endif

#elif M1_HFI_GATE == 105
/* Rel C4r：104 的 VESC 锁/限幅，踢完交速度环、只巡 100 rpm。
 * FEED 关（Iq 给速度 PI）；SPEED_FB=HFI pll_int；ω_ff=0。
 * AUTH 天花板 3.5 A（C4r 起步，不作 1000 梯）。对照 2018。 */
#undef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE       0
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              1
#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             5.0f
#undef M1_HFI_RUN_LADDER_ENABLE
#define M1_HFI_RUN_LADDER_ENABLE        1
#undef M1_HFI_RUN_RPM_START
#define M1_HFI_RUN_RPM_START            100.0f
#undef M1_HFI_RUN_RPM_STEP
#define M1_HFI_RUN_RPM_STEP             100.0f
#undef M1_HFI_RUN_RPM_MAX
#define M1_HFI_RUN_RPM_MAX              100.0f
#undef M1_HFI_RUN_STEP_S
#define M1_HFI_RUN_STEP_S               15.0f
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 100.0f
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               15.0f
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 100.0f
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               0.0f
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (-1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (-1.0f)
#undef M1_HFI_A_CMD
#define M1_HFI_A_CMD                    (0.10f)
#undef M1_HFI_INIT_FROM_ENC
#define M1_HFI_INIT_FROM_ENC            1
#undef M1_HFI_QKICK_AFTER_LOCK_ENABLE
#define M1_HFI_QKICK_AFTER_LOCK_ENABLE  1
#undef M1_HFI_QKICK_THEN_HFI_ENABLE
#define M1_HFI_QKICK_THEN_HFI_ENABLE    1
#undef M1_HFI_QKICK_PRE_S
#define M1_HFI_QKICK_PRE_S              2.0f
#undef M1_HFI_QKICK_IQ_A
#define M1_HFI_QKICK_IQ_A               1.6f
#undef M1_HFI_QKICK_KICK_N
#define M1_HFI_QKICK_KICK_N             6000u
#undef M1_HFI_QKICK_BRAKE_ENABLE
#define M1_HFI_QKICK_BRAKE_ENABLE       0
#undef M1_HFI_QKICK_HOLD_S
#define M1_HFI_QKICK_HOLD_S             0.4f
#undef M1_HFI_QKICK_SPEED_ENABLE
#define M1_HFI_QKICK_SPEED_ENABLE       0
#undef M1_HFI_QKICK_CRAWL_ENABLE
#define M1_HFI_QKICK_CRAWL_ENABLE       0
#undef M1_HFI_SENSED_CAL_ENABLE
#define M1_HFI_SENSED_CAL_ENABLE        0
#undef M1_HFI_INJECT_POST_LOOP
#define M1_HFI_INJECT_POST_LOOP         1
#undef M1_HFI_DEMOD_INJ_AXIS
#define M1_HFI_DEMOD_INJ_AXIS           1
#undef M1_HFI_INJECT_AB_ENABLE
#define M1_HFI_INJECT_AB_ENABLE         0
#undef M1_HFI_ID_PI_OFF_ENABLE
#define M1_HFI_ID_PI_OFF_ENABLE         0
#undef M1_HFI_ID_ON_FROM_RUN_ENABLE
#define M1_HFI_ID_ON_FROM_RUN_ENABLE    1
#undef M1_HFI_OMEGA_FF_SRC
#define M1_HFI_OMEGA_FF_SRC             0
#undef M1_HFI_OMEGA_FF_FROM_REF
#define M1_HFI_OMEGA_FF_FROM_REF        0
#undef M1_HFI_SPEED_FB_ENABLE
#define M1_HFI_SPEED_FB_ENABLE          1
#undef M1_HFI_SPEED_OBS_INT
#define M1_HFI_SPEED_OBS_INT            1
#undef M1_HFI_PLL_VESC_ERR_ENABLE
#define M1_HFI_PLL_VESC_ERR_ENABLE      1
#undef M1_HFI_PLL_VESC_MAX_ERR
#define M1_HFI_PLL_VESC_MAX_ERR         (0.30f)
#undef M1_HFI_PLL_VESC_ERR_SIGN
#define M1_HFI_PLL_VESC_ERR_SIGN        (-1.0f)
#undef M1_HFI_IQ_AUTH_ENABLE
#define M1_HFI_IQ_AUTH_ENABLE           1
#undef M1_HFI_IQ_AUTH_X_GOOD
#define M1_HFI_IQ_AUTH_X_GOOD           (0.218f)
#undef M1_HFI_IQ_AUTH_X_BAD
#define M1_HFI_IQ_AUTH_X_BAD            (0.205f)
#undef M1_HFI_IQ_AUTH_EPS_FALSE_MAX
#define M1_HFI_IQ_AUTH_EPS_FALSE_MAX    (0.20f)
#undef M1_HFI_IQ_AUTH_HOLD_N
#define M1_HFI_IQ_AUTH_HOLD_N           2000u
#undef M1_HFI_IQ_AUTH_CLEAR_N
#define M1_HFI_IQ_AUTH_CLEAR_N          1000u
#undef M1_HFI_IQ_AUTH_IQ_LO
#define M1_HFI_IQ_AUTH_IQ_LO            (0.25f)
#undef M1_HFI_IQ_AUTH_IQ_HI
#define M1_HFI_IQ_AUTH_IQ_HI            (3.50f)
#undef M1_HFI_IQ_AUTH_SLEW_A_S
#define M1_HFI_IQ_AUTH_SLEW_A_S         (4.0f)
#undef M1_HFI_IQ_AUTH_FEED_ENABLE
#define M1_HFI_IQ_AUTH_FEED_ENABLE      0
#undef M1_HFI_DEMOD_HP_ENABLE
#define M1_HFI_DEMOD_HP_ENABLE          1
#undef M1_HFI_DEMOD_HP_A
#define M1_HFI_DEMOD_HP_A               (0.05f)
#undef M1_HFI_DEMOD_SKIP_OUTLIER
#define M1_HFI_DEMOD_SKIP_OUTLIER       0
#undef M1_SPEED_IQ_REF_ABS_MAX
#define M1_SPEED_IQ_REF_ABS_MAX         3.5f
#undef M1_SPEED_PI_OUT_MAX
#define M1_SPEED_PI_OUT_MAX             M1_SPEED_IQ_REF_ABS_MAX
#undef M1_SPEED_PI_OUT_MIN
#define M1_SPEED_PI_OUT_MIN             (-M1_SPEED_IQ_REF_ABS_MAX)
#undef M1_SPEED_PI_INT_MAX
#define M1_SPEED_PI_INT_MAX             M1_SPEED_IQ_REF_ABS_MAX
#undef M1_SPEED_PI_INT_MIN
#define M1_SPEED_PI_INT_MIN             (-M1_SPEED_IQ_REF_ABS_MAX)

#elif M1_HFI_GATE == 44
/* S2b：控制同 S2（Park=enc，速度�?100 rpm，ω_ff=0）�?
 * 唯一增量：RUN 25�?0 s，空载锁定后再加负载，看 Iq�?.8 �?θ_err。回退 GATE=2�?*/
#undef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE       0
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              0
#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             5.0f
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 100.0f
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               40.0f
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 100.0f
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               0.0f
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (-1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (-1.0f)
#undef M1_HFI_A_CMD
#define M1_HFI_A_CMD                    (0.10f)

#elif M1_HFI_GATE == 45
/* S2c：Park=enc，速度环吃编码器。阶�?0,100,�?1000 rpm，每�?8 s�?
 * HFI 只估角。W_MAX 抬到 1100，避免约 270 rpm 以上 PLL 限速把 θ_err 拖歪�?
 * ω_ff 仍为 0。回退 GATE=2�?*/
#undef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE       0
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              0
#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             5.0f
#undef M1_HFI_RUN_LADDER_ENABLE
#define M1_HFI_RUN_LADDER_ENABLE        1
#undef M1_HFI_RUN_RPM_START
#define M1_HFI_RUN_RPM_START            0.0f
#undef M1_HFI_RUN_RPM_STEP
#define M1_HFI_RUN_RPM_STEP             100.0f
#undef M1_HFI_RUN_RPM_MAX
#define M1_HFI_RUN_RPM_MAX              1000.0f
#undef M1_HFI_RUN_STEP_S
#define M1_HFI_RUN_STEP_S               8.0f
#undef M1_HFI_PLL_W_MAX
#define M1_HFI_PLL_W_MAX                1100.0f
#undef M1_HFI_PLL_INT_MAX
#define M1_HFI_PLL_INT_MAX              M1_HFI_PLL_W_MAX
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (-1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (-1.0f)
#undef M1_HFI_A_CMD
#define M1_HFI_A_CMD                    (0.10f)

#elif M1_HFI_GATE == 46
/* S2d：速度环吃编码器，电流 Park=θ̂�?00 rpm 20 s，然�?200 rpm 20 s�?
 * 相对 S2 只改 Park 和两档转速。SPEED_FB=0，ω_ff=0，无 FEED，PLL 增益不动�?
 * 回退 GATE=44�?*/
#undef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE       0
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              1
#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             5.0f
#undef M1_HFI_RUN_LADDER_ENABLE
#define M1_HFI_RUN_LADDER_ENABLE        0
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 100.0f
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               20.0f
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 200.0f
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               20.0f
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (-1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (-1.0f)
#undef M1_HFI_A_CMD
#define M1_HFI_A_CMD                    (0.10f)
#undef M1_HFI_SPEED_FB_ENABLE
#define M1_HFI_SPEED_FB_ENABLE          0
/* 速度观测 = PLL 积分，不�?Kp·eps。限�?807 rad/s �?1100 rpm 机械�? 对极）�?
 * 角度积分仍用 Kp·eps+∫�?700：积�?20 ms 中位�?enc �?0�?3 rpm，原�?ω̂ p95 �?>100 rpm�?*/
#undef M1_HFI_PLL_W_MAX
#define M1_HFI_PLL_W_MAX                807.0f
#undef M1_HFI_PLL_INT_MAX
#define M1_HFI_PLL_INT_MAX              M1_HFI_PLL_W_MAX
#undef M1_HFI_SPEED_OBS_INT
#define M1_HFI_SPEED_OBS_INT            1

#elif M1_HFI_GATE == 47
/* S2e：控制同 S2d。速度环吃编码器，电流 Park=θ̂，ch8=PLL 积分�?
 * 阶梯 100,200,�?1000 rpm，每�?8 s�?0 档，RUN 80 s）。限�?1100 rpm�?
 * 回退 GATE=46�?*/
#undef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE       0
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              1
#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             5.0f
#undef M1_HFI_RUN_LADDER_ENABLE
#define M1_HFI_RUN_LADDER_ENABLE        1
#undef M1_HFI_RUN_RPM_START
#define M1_HFI_RUN_RPM_START            100.0f
#undef M1_HFI_RUN_RPM_STEP
#define M1_HFI_RUN_RPM_STEP             100.0f
#undef M1_HFI_RUN_RPM_MAX
#define M1_HFI_RUN_RPM_MAX              1000.0f
#undef M1_HFI_RUN_STEP_S
#define M1_HFI_RUN_STEP_S               8.0f
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (-1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (-1.0f)
#undef M1_HFI_A_CMD
#define M1_HFI_A_CMD                    (0.10f)
#undef M1_HFI_SPEED_FB_ENABLE
#define M1_HFI_SPEED_FB_ENABLE          0
#undef M1_HFI_PLL_W_MAX
#define M1_HFI_PLL_W_MAX                807.0f
#undef M1_HFI_PLL_INT_MAX
#define M1_HFI_PLL_INT_MAX              M1_HFI_PLL_W_MAX
#undef M1_HFI_SPEED_OBS_INT
#define M1_HFI_SPEED_OBS_INT            1

#elif M1_HFI_GATE == 48
/* S2f：同 S2e。x≤A 不写 Ki�?829 冻角�?838 100 rpm 丢步、换档积分爬不动）�?
 * 冻结。回退 GATE=46�?*/
#undef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE       0
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              1
#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             5.0f
#undef M1_HFI_RUN_LADDER_ENABLE
#define M1_HFI_RUN_LADDER_ENABLE        1
#undef M1_HFI_RUN_RPM_START
#define M1_HFI_RUN_RPM_START            100.0f
#undef M1_HFI_RUN_RPM_STEP
#define M1_HFI_RUN_RPM_STEP             100.0f
#undef M1_HFI_RUN_RPM_MAX
#define M1_HFI_RUN_RPM_MAX              1000.0f
#undef M1_HFI_RUN_STEP_S
#define M1_HFI_RUN_STEP_S               8.0f
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (-1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (-1.0f)
#undef M1_HFI_A_CMD
#define M1_HFI_A_CMD                    (0.10f)
#undef M1_HFI_SPEED_FB_ENABLE
#define M1_HFI_SPEED_FB_ENABLE          0
#undef M1_HFI_PLL_W_MAX
#define M1_HFI_PLL_W_MAX                807.0f
#undef M1_HFI_PLL_INT_MAX
#define M1_HFI_PLL_INT_MAX              M1_HFI_PLL_W_MAX
#undef M1_HFI_SPEED_OBS_INT
#define M1_HFI_SPEED_OBS_INT            1
#undef M1_HFI_PLL_SKIP_X_BELOW_A
#define M1_HFI_PLL_SKIP_X_BELOW_A       1

#elif M1_HFI_GATE == 49
/* S2g：阶梯同 S2e�?00�?000 / 8 s）。PLL �?S2d：每�?Ki+Kp，无 x≤A skip�?
 * 速度环吃编码器，Park=θ̂。回退 GATE=46�?*/
#undef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE       0
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              1
#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             5.0f
#undef M1_HFI_RUN_LADDER_ENABLE
#define M1_HFI_RUN_LADDER_ENABLE        1
#undef M1_HFI_RUN_RPM_START
#define M1_HFI_RUN_RPM_START            100.0f
#undef M1_HFI_RUN_RPM_STEP
#define M1_HFI_RUN_RPM_STEP             100.0f
#undef M1_HFI_RUN_RPM_MAX
#define M1_HFI_RUN_RPM_MAX              1000.0f
#undef M1_HFI_RUN_STEP_S
#define M1_HFI_RUN_STEP_S               8.0f
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (-1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (-1.0f)
#undef M1_HFI_A_CMD
#define M1_HFI_A_CMD                    (0.10f)
#undef M1_HFI_SPEED_FB_ENABLE
#define M1_HFI_SPEED_FB_ENABLE          0
#undef M1_HFI_PLL_W_MAX
#define M1_HFI_PLL_W_MAX                807.0f
#undef M1_HFI_PLL_INT_MAX
#define M1_HFI_PLL_INT_MAX              M1_HFI_PLL_W_MAX
#undef M1_HFI_SPEED_OBS_INT
#define M1_HFI_SPEED_OBS_INT            1
#undef M1_HFI_PLL_SKIP_X_BELOW_A
#define M1_HFI_PLL_SKIP_X_BELOW_A       0

#elif M1_HFI_GATE == 50
/* S2h：tacc 五段改写�?917 FAIL）。冻结。回退 GATE=46�?*/
#undef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE       0
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              1
#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             5.0f
#undef M1_HFI_RUN_LADDER_ENABLE
#define M1_HFI_RUN_LADDER_ENABLE        0
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 100.0f
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               20.0f
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 200.0f
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               20.0f
#undef M1_HFI_RUN_RPM3
#define M1_HFI_RUN_RPM3                 300.0f
#undef M1_HFI_RUN_RPM3_S
#define M1_HFI_RUN_RPM3_S               20.0f
#undef M1_HFI_RUN_RPM4
#define M1_HFI_RUN_RPM4                 400.0f
#undef M1_HFI_RUN_RPM4_S
#define M1_HFI_RUN_RPM4_S               20.0f
#undef M1_HFI_RUN_RPM5
#define M1_HFI_RUN_RPM5                 500.0f
#undef M1_HFI_RUN_RPM5_S
#define M1_HFI_RUN_RPM5_S               20.0f
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (-1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (-1.0f)
#undef M1_HFI_A_CMD
#define M1_HFI_A_CMD                    (0.10f)
#undef M1_HFI_SPEED_FB_ENABLE
#define M1_HFI_SPEED_FB_ENABLE          0
#undef M1_HFI_PLL_W_MAX
#define M1_HFI_PLL_W_MAX                807.0f
#undef M1_HFI_PLL_INT_MAX
#define M1_HFI_PLL_INT_MAX              M1_HFI_PLL_W_MAX
#undef M1_HFI_SPEED_OBS_INT
#define M1_HFI_SPEED_OBS_INT            1

#elif M1_HFI_GATE == 51
/* S2i：控制同 S2d。RO �?100�?00/+100，其�?+50�?000；各�?dwell 3.5 s�?
 * VH 三档 RO：≤300�?.40�?00�?50�?.50，≥600�?.60；仅换档�?s_vh_v。回退 GATE=46�?*/
#undef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE       0
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              1
#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             5.0f
#undef M1_HFI_VH_V
#define M1_HFI_VH_V                     0.40f
#undef M1_HFI_VH_LO
#define M1_HFI_VH_LO                    0.40f
#undef M1_HFI_VH_MID
#define M1_HFI_VH_MID                   0.50f
#undef M1_HFI_VH_HI
#define M1_HFI_VH_HI                    0.60f
#undef M1_HFI_RUN_LADDER_ENABLE
#define M1_HFI_RUN_LADDER_ENABLE        0
#undef M1_HFI_RUN_RPM_START
#define M1_HFI_RUN_RPM_START            100.0f
#undef M1_HFI_RUN_RPM_STEP
#define M1_HFI_RUN_RPM_STEP             100.0f
#undef M1_HFI_RUN_RPM_MAX
#define M1_HFI_RUN_RPM_MAX              1000.0f
#undef M1_HFI_RUN_STEP_S
#define M1_HFI_RUN_STEP_S               3.5f
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 M1_HFI_RUN_RPM_START
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               M1_HFI_RUN_STEP_S
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 M1_HFI_RUN_RPM_START
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               0.0f
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (-1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (-1.0f)
#undef M1_HFI_A_CMD
#define M1_HFI_A_CMD                    (0.10f)
#undef M1_HFI_SPEED_FB_ENABLE
#define M1_HFI_SPEED_FB_ENABLE          0
#undef M1_HFI_PLL_W_MAX
#define M1_HFI_PLL_W_MAX                807.0f
#undef M1_HFI_PLL_INT_MAX
#define M1_HFI_PLL_INT_MAX              M1_HFI_PLL_W_MAX
#undef M1_HFI_SPEED_OBS_INT
#define M1_HFI_SPEED_OBS_INT            1

#elif M1_HFI_GATE == 52
/* S2j：Park=θ̂，VH=0.40。先�?Iq=0 锁住（x≥GOOD 连续 HOLD），再延时后
 * 固定 Iq=0.8 A。x≤A �?PLL 不跟 ±90° �?ε。速度环关。回退 46/51�?*/
#undef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE       0
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              1
#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             5.0f
#undef M1_HFI_VH_V
#define M1_HFI_VH_V                     0.40f
#undef M1_HFI_RUN_LADDER_ENABLE
#define M1_HFI_RUN_LADDER_ENABLE        0
#undef M1_HFI_RUN_RPM_START
#define M1_HFI_RUN_RPM_START            100.0f
#undef M1_HFI_RUN_RPM_STEP
#define M1_HFI_RUN_RPM_STEP             50.0f
#undef M1_HFI_RUN_RPM_MAX
#define M1_HFI_RUN_RPM_MAX              800.0f
#undef M1_HFI_RUN_STEP_S
#define M1_HFI_RUN_STEP_S               3.5f
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 M1_HFI_RUN_RPM_START
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               M1_HFI_RUN_STEP_S
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 M1_HFI_RUN_RPM_START
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               0.0f
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (-1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (-1.0f)
#undef M1_HFI_A_CMD
#define M1_HFI_A_CMD                    (0.10f)
#undef M1_HFI_SPEED_FB_ENABLE
#define M1_HFI_SPEED_FB_ENABLE          1 /* 速度段吃 HFI ω，不�?enc */
#undef M1_HFI_PLL_W_MAX
#define M1_HFI_PLL_W_MAX                807.0f
#undef M1_HFI_PLL_INT_MAX
#define M1_HFI_PLL_INT_MAX              M1_HFI_PLL_W_MAX
#undef M1_HFI_SPEED_OBS_INT
#define M1_HFI_SPEED_OBS_INT            1
#undef M1_HFI_IQ_PULL_ENABLE
#define M1_HFI_IQ_PULL_ENABLE           1 /* 电流环定 Iq，速度环关 */
#undef M1_HFI_IQ_PULL_A
#define M1_HFI_IQ_PULL_A                0.80f
#undef M1_HFI_IQ_PULL_ARM_ENABLE
#define M1_HFI_IQ_PULL_ARM_ENABLE       0 /* 固定电流，不交速度�?*/
#undef M1_HFI_IQ_PULL_DELAY_S
#define M1_HFI_IQ_PULL_DELAY_S          0.20f /* qual_ok 后再等，对齐 C4 FEED */
#undef M1_HFI_IQ_AUTH_ENABLE
#define M1_HFI_IQ_AUTH_ENABLE           1
#undef M1_HFI_IQ_AUTH_X_GOOD
#define M1_HFI_IQ_AUTH_X_GOOD           (0.218f)
#undef M1_HFI_IQ_AUTH_X_BAD
#define M1_HFI_IQ_AUTH_X_BAD            (0.205f)
#undef M1_HFI_IQ_AUTH_EPS_FALSE_MAX
#define M1_HFI_IQ_AUTH_EPS_FALSE_MAX    (0.20f)
#undef M1_HFI_IQ_AUTH_HOLD_N
#define M1_HFI_IQ_AUTH_HOLD_N           2000u /* 100 ms */
#undef M1_HFI_IQ_AUTH_CLEAR_N
#define M1_HFI_IQ_AUTH_CLEAR_N          1000u /* x 塌了收回 0.8 A */
#undef M1_HFI_PLL_HOLD_X_BELOW_A
#define M1_HFI_PLL_HOLD_X_BELOW_A       1 /* x≤A：�?�?±90°，不�?θ̂ */
#undef M1_HFI_SPD_RAMP_ENABLE
#define M1_HFI_SPD_RAMP_ENABLE          0
#undef M1_HFI_OMEGA_FF_SRC
#define M1_HFI_OMEGA_FF_SRC             0
#undef M1_HFI_OMEGA_SEED_ENABLE
#define M1_HFI_OMEGA_SEED_ENABLE        0
/* 起步限流：避免斜坡段 PI 仍顶满毁锁（对齐 C4r 量级�?*/
#undef M1_SPEED_IQ_REF_ABS_MAX
#define M1_SPEED_IQ_REF_ABS_MAX         3.5f
#undef M1_SPEED_PI_OUT_MAX
#define M1_SPEED_PI_OUT_MAX             M1_SPEED_IQ_REF_ABS_MAX
#undef M1_SPEED_PI_OUT_MIN
#define M1_SPEED_PI_OUT_MIN             (-M1_SPEED_IQ_REF_ABS_MAX)
#undef M1_SPEED_PI_INT_MAX
#define M1_SPEED_PI_INT_MAX             M1_SPEED_IQ_REF_ABS_MAX
#undef M1_SPEED_PI_INT_MIN
#define M1_SPEED_PI_INT_MIN             (-M1_SPEED_IQ_REF_ABS_MAX)
#undef M1_SPEED_IQ_SLEW_ENABLE
#define M1_SPEED_IQ_SLEW_ENABLE         1
#undef M1_SPEED_IQ_SLEW_A_PER_S
#define M1_SPEED_IQ_SLEW_A_PER_S        8.0f

#elif M1_HFI_GATE == 3
/* S3a：静置、Iq=0、Park=θ̂。落盘同 S2：XY=�?，A_cmd=+0.10�?*/
#undef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE       0
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              1
#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             5.0f
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 0.0f
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               15.0f
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 0.0f
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               0.0f
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (-1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (-1.0f)
#undef M1_HFI_A_CMD
#define M1_HFI_A_CMD                    (0.10f)

#elif M1_HFI_GATE == 4
/* S3b：q 轴踢（QKICK_AFTER_LOCK）。本机饱�?IPD 分不开极性�?
 * IDLE 5 s �?RUN �?2θ（≥PRE）→ MEAS +Iq �?�?BRAKE �?LOG（再解冻 PLL）→ DONE�?
 * decide：正�?θ̂+=Δθ；反�?θ̂+=Δθ+π。勿开 IPD。复测改�?GATE=4�?*/
#undef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE       0
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              1
#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             5.0f
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 0.0f
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               8.0f
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 0.0f
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               0.0f
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (-1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (-1.0f)
#undef M1_HFI_A_CMD
#define M1_HFI_A_CMD                    (0.10f)
#undef M1_HFI_POLARITY_IPD_ENABLE
#define M1_HFI_POLARITY_IPD_ENABLE      0
#undef M1_HFI_INIT_FROM_ENC
#define M1_HFI_INIT_FROM_ENC            1
#undef M1_HFI_QKICK_AFTER_LOCK_ENABLE
#define M1_HFI_QKICK_AFTER_LOCK_ENABLE  1
#undef M1_HFI_QKICK_PRE_S
#define M1_HFI_QKICK_PRE_S              2.0f
#undef M1_HFI_QKICK_IQ_A
#define M1_HFI_QKICK_IQ_A               1.6f
#undef M1_HFI_QKICK_KICK_N
#define M1_HFI_QKICK_KICK_N             6000u /* 300 ms */
#undef M1_HFI_QKICK_BRAKE_ENABLE
#define M1_HFI_QKICK_BRAKE_ENABLE       1
#undef M1_HFI_QKICK_BRAKE_N
#define M1_HFI_QKICK_BRAKE_N            6000u
#undef M1_HFI_QKICK_HOLD_S
#define M1_HFI_QKICK_HOLD_S             5.0f
#undef M1_HFI_QKICK_SPEED_ENABLE
#define M1_HFI_QKICK_SPEED_ENABLE       0
#undef M1_HFI_QKICK_CRAWL_ENABLE
#define M1_HFI_QKICK_CRAWL_ENABLE       0

#elif M1_HFI_GATE == 5
/* S3c0：踢�?Park=θ̂ + 常�?Iq 爬（1305 FAIL：掉 q、反转）。可复现对照�?
 * 下一�?S3c1=GATE=6（力�?Park=enc）。回退 GATE=4→S3b�?*/
#undef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE       0
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              1
#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             5.0f
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 0.0f
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               8.0f
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 0.0f
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               0.0f
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (-1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (-1.0f)
#undef M1_HFI_A_CMD
#define M1_HFI_A_CMD                    (0.10f)
#undef M1_HFI_POLARITY_IPD_ENABLE
#define M1_HFI_POLARITY_IPD_ENABLE      0
#undef M1_HFI_INIT_FROM_ENC
#define M1_HFI_INIT_FROM_ENC            1
#undef M1_HFI_QKICK_AFTER_LOCK_ENABLE
#define M1_HFI_QKICK_AFTER_LOCK_ENABLE  1
#undef M1_HFI_QKICK_PRE_S
#define M1_HFI_QKICK_PRE_S              2.0f
#undef M1_HFI_QKICK_IQ_A
#define M1_HFI_QKICK_IQ_A               1.6f
#undef M1_HFI_QKICK_KICK_N
#define M1_HFI_QKICK_KICK_N             6000u
#undef M1_HFI_QKICK_BRAKE_ENABLE
#define M1_HFI_QKICK_BRAKE_ENABLE       1
#undef M1_HFI_QKICK_BRAKE_N
#define M1_HFI_QKICK_BRAKE_N            6000u
#undef M1_HFI_QKICK_HOLD_S
#define M1_HFI_QKICK_HOLD_S             3.0f
#undef M1_HFI_QKICK_SPEED_ENABLE
#define M1_HFI_QKICK_SPEED_ENABLE       0
#undef M1_HFI_QKICK_CRAWL_ENABLE
#define M1_HFI_QKICK_CRAWL_ENABLE       1
#undef M1_HFI_QKICK_CRAWL_IQ_A
#define M1_HFI_QKICK_CRAWL_IQ_A         0.8f
#undef M1_HFI_QKICK_CRAWL_S
#define M1_HFI_QKICK_CRAWL_S            8.0f
#undef M1_HFI_QKICK_IQ_STEP_ENABLE
#define M1_HFI_QKICK_IQ_STEP_ENABLE     0
#undef M1_HFI_QKICK_IQ_RAMP_ENABLE
#define M1_HFI_QKICK_IQ_RAMP_ENABLE     0
#undef M1_HFI_QKICK_IQ_LADDER_ENABLE
#define M1_HFI_QKICK_IQ_LADDER_ENABLE   0
#undef M1_HFI_QKICK_IQ_SLOW_ENABLE
#define M1_HFI_QKICK_IQ_SLOW_ENABLE     0
#undef M1_HFI_QKICK_IF_ENABLE
#define M1_HFI_QKICK_IF_ENABLE          0
#undef M1_HFI_QKICK_START_ENABLE
#define M1_HFI_QKICK_START_ENABLE       0
#undef M1_HFI_QKICK_CRAWL_PARK_ENC
#define M1_HFI_QKICK_CRAWL_PARK_ENC     0

#elif M1_HFI_GATE == 6
/* S3c1：同 S3c0 �?0.8A 爬，�?CRAWL 力矩 Park=enc，HFI 只旁路估角�?
 * 对照自指。回退 GATE=5→S3c0，GATE=4→S3b�?*/
#undef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE       0
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              1
#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             5.0f
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 0.0f
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               8.0f
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 0.0f
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               0.0f
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (-1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (-1.0f)
#undef M1_HFI_A_CMD
#define M1_HFI_A_CMD                    (0.10f)
#undef M1_HFI_POLARITY_IPD_ENABLE
#define M1_HFI_POLARITY_IPD_ENABLE      0
#undef M1_HFI_INIT_FROM_ENC
#define M1_HFI_INIT_FROM_ENC            1
#undef M1_HFI_QKICK_AFTER_LOCK_ENABLE
#define M1_HFI_QKICK_AFTER_LOCK_ENABLE  1
#undef M1_HFI_QKICK_PRE_S
#define M1_HFI_QKICK_PRE_S              2.0f
#undef M1_HFI_QKICK_IQ_A
#define M1_HFI_QKICK_IQ_A               1.6f
#undef M1_HFI_QKICK_KICK_N
#define M1_HFI_QKICK_KICK_N             6000u
#undef M1_HFI_QKICK_BRAKE_ENABLE
#define M1_HFI_QKICK_BRAKE_ENABLE       1
#undef M1_HFI_QKICK_BRAKE_N
#define M1_HFI_QKICK_BRAKE_N            6000u
#undef M1_HFI_QKICK_HOLD_S
#define M1_HFI_QKICK_HOLD_S             3.0f
#undef M1_HFI_QKICK_SPEED_ENABLE
#define M1_HFI_QKICK_SPEED_ENABLE       0
#undef M1_HFI_QKICK_CRAWL_ENABLE
#define M1_HFI_QKICK_CRAWL_ENABLE       1
#undef M1_HFI_QKICK_CRAWL_IQ_A
#define M1_HFI_QKICK_CRAWL_IQ_A         0.8f
#undef M1_HFI_QKICK_CRAWL_S
#define M1_HFI_QKICK_CRAWL_S            8.0f
#undef M1_HFI_QKICK_CRAWL_PARK_ENC
#define M1_HFI_QKICK_CRAWL_PARK_ENC     1
#undef M1_HFI_QKICK_IQ_STEP_ENABLE
#define M1_HFI_QKICK_IQ_STEP_ENABLE     0
#undef M1_HFI_QKICK_IQ_RAMP_ENABLE
#define M1_HFI_QKICK_IQ_RAMP_ENABLE     0
#undef M1_HFI_QKICK_IQ_LADDER_ENABLE
#define M1_HFI_QKICK_IQ_LADDER_ENABLE   0
#undef M1_HFI_QKICK_IQ_SLOW_ENABLE
#define M1_HFI_QKICK_IQ_SLOW_ENABLE     0
#undef M1_HFI_QKICK_IF_ENABLE
#define M1_HFI_QKICK_IF_ENABLE          0
#undef M1_HFI_QKICK_START_ENABLE
#define M1_HFI_QKICK_START_ENABLE       0

#elif M1_HFI_GATE == 7
/* S3c0a：相�?S2 只改 Park→θ̂。速度环~100 rpm，ω_ff=0，无 QKICK�?
 * 禁重开 enc 转速前馈（1638 假过）。回退 GATE=2→S2�?*/
#undef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE       0
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              1
#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             5.0f
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 100.0f
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               25.0f
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 100.0f
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               0.0f
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (-1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (-1.0f)
#undef M1_HFI_A_CMD
#define M1_HFI_A_CMD                    (0.10f)
#undef M1_HFI_OMEGA_FF_SRC
#define M1_HFI_OMEGA_FF_SRC             0
#undef M1_HFI_OMEGA_FF_FROM_REF
#define M1_HFI_OMEGA_FF_FROM_REF        0
#undef M1_HFI_QKICK_AFTER_LOCK_ENABLE
#define M1_HFI_QKICK_AFTER_LOCK_ENABLE  0
#undef M1_HFI_QKICK_CRAWL_ENABLE
#define M1_HFI_QKICK_CRAWL_ENABLE       0
#undef M1_HFI_INIT_FROM_ENC
#define M1_HFI_INIT_FROM_ENC            1

#elif M1_HFI_GATE == 8
/* S3c0b：同 S3c0a，加无感质量分限 Iq�?
 * |x| �?A �?放权；钉 A−B �?ε 小（假锁）→ Iq 天花板压低�?
 * 判据不用 enc。回退 GATE=7�?*/
#undef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE       0
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              1
#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             5.0f
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 100.0f
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               25.0f
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 100.0f
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               0.0f
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (-1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (-1.0f)
#undef M1_HFI_A_CMD
#define M1_HFI_A_CMD                    (0.10f)
#undef M1_HFI_OMEGA_FF_SRC
#define M1_HFI_OMEGA_FF_SRC             0
#undef M1_HFI_OMEGA_FF_FROM_REF
#define M1_HFI_OMEGA_FF_FROM_REF        0
#undef M1_HFI_QKICK_AFTER_LOCK_ENABLE
#define M1_HFI_QKICK_AFTER_LOCK_ENABLE  0
#undef M1_HFI_QKICK_CRAWL_ENABLE
#define M1_HFI_QKICK_CRAWL_ENABLE       0
#undef M1_HFI_INIT_FROM_ENC
#define M1_HFI_INIT_FROM_ENC            1
#undef M1_HFI_IQ_AUTH_ENABLE
#define M1_HFI_IQ_AUTH_ENABLE           1
#undef M1_HFI_IQ_AUTH_X_GOOD
#define M1_HFI_IQ_AUTH_X_GOOD           (0.218f) /* |x|�?�?�?A */
#undef M1_HFI_IQ_AUTH_X_BAD
#define M1_HFI_IQ_AUTH_X_BAD            (0.205f) /* |x|�?�?�?A−B */
#undef M1_HFI_IQ_AUTH_EPS_FALSE_MAX
#define M1_HFI_IQ_AUTH_EPS_FALSE_MAX    (0.20f)  /* rad；假�?ε 上限 */
#undef M1_HFI_IQ_AUTH_HOLD_N
#define M1_HFI_IQ_AUTH_HOLD_N           2000u    /* 100 ms @20 kHz */
#undef M1_HFI_IQ_AUTH_IQ_LO
#define M1_HFI_IQ_AUTH_IQ_LO            (0.25f)
#undef M1_HFI_IQ_AUTH_IQ_HI
#define M1_HFI_IQ_AUTH_IQ_HI            (3.50f)  /* 捕获期勿给满 11 A */
#undef M1_HFI_IQ_AUTH_SLEW_A_S
#define M1_HFI_IQ_AUTH_SLEW_A_S         (4.0f)

#elif M1_HFI_GATE == 9
/* C1：控制同 GATE=3（S3a）。专看上�?�?RUN �?0�? s 瞬态�?
 * 1�? 冻结；回退 GATE=3。勿在此叠速度�?Iq 权威�?*/
#undef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE       0
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              1
#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             5.0f
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 0.0f
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               15.0f
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 0.0f
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               0.0f
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (-1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (-1.0f)
#undef M1_HFI_A_CMD
#define M1_HFI_A_CMD                    (0.10f)
#undef M1_HFI_IQ_AUTH_ENABLE
#define M1_HFI_IQ_AUTH_ENABLE           0

#elif M1_HFI_GATE == 10
/* C2：控制同 S3a/C1，唯一增量：θ�?种子 = enc + 90°（故意坏初值）�?
 * Iq=0，不开速度环�?�? 冻结；回退 GATE=9 �?3�?*/
#undef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE       0
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              1
#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             5.0f
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 0.0f
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               15.0f
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 0.0f
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               0.0f
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (-1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (-1.0f)
#undef M1_HFI_A_CMD
#define M1_HFI_A_CMD                    (0.10f)
#undef M1_HFI_INIT_FROM_ENC
#define M1_HFI_INIT_FROM_ENC            1
#undef M1_HFI_PLL_INIT_OFF_RAD
#define M1_HFI_PLL_INIT_OFF_RAD         (1.57079632679f) /* +90 deg */
#undef M1_HFI_IQ_AUTH_ENABLE
#define M1_HFI_IQ_AUTH_ENABLE           0

#elif M1_HFI_GATE == 11
/* C3：控制同 S3a，唯一增量：开无感质量标志（IQ_AUTH 只产 ok，不出力）�?
 * Iq=0，不开速度环。VOFA ch1=qual_ok�?�?0 冻结；回退 GATE=10/9/3�?*/
#undef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE       0
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              1
#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             5.0f
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 0.0f
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               15.0f
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 0.0f
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               0.0f
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (-1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (-1.0f)
#undef M1_HFI_A_CMD
#define M1_HFI_A_CMD                    (0.10f)
#undef M1_HFI_PLL_INIT_OFF_RAD
#define M1_HFI_PLL_INIT_OFF_RAD         0.0f
#undef M1_HFI_IQ_AUTH_ENABLE
#define M1_HFI_IQ_AUTH_ENABLE           1
#undef M1_HFI_IQ_AUTH_X_GOOD
#define M1_HFI_IQ_AUTH_X_GOOD           (0.218f)
#undef M1_HFI_IQ_AUTH_X_BAD
#define M1_HFI_IQ_AUTH_X_BAD            (0.205f)
#undef M1_HFI_IQ_AUTH_EPS_FALSE_MAX
#define M1_HFI_IQ_AUTH_EPS_FALSE_MAX    (0.20f)
#undef M1_HFI_IQ_AUTH_HOLD_N
#define M1_HFI_IQ_AUTH_HOLD_N           2000u
#undef M1_HFI_IQ_AUTH_IQ_LO
#define M1_HFI_IQ_AUTH_IQ_LO            (0.0f) /* C3 不出�?*/
#undef M1_HFI_IQ_AUTH_IQ_HI
#define M1_HFI_IQ_AUTH_IQ_HI            (0.0f)
#undef M1_HFI_IQ_AUTH_SLEW_A_S
#define M1_HFI_IQ_AUTH_SLEW_A_S         (10.0f)
/* CLEAR_N 默认 1：与历史 GATE=8/11 立即清零一致（本分支不改） */

#elif M1_HFI_GATE == 12
/* C3b：控制同 C3（Iq=0 + qual_ok），唯一增量：假锁清零需连续 CLEAR_N�?
 * 1536：真 d �?|x| 偶发掉到 ~0.21 即清�?�?过脆�?�?1 冻结；回退 GATE=11�?*/
#undef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE       0
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              1
#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             5.0f
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 0.0f
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               15.0f
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 0.0f
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               0.0f
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (-1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (-1.0f)
#undef M1_HFI_A_CMD
#define M1_HFI_A_CMD                    (0.10f)
#undef M1_HFI_PLL_INIT_OFF_RAD
#define M1_HFI_PLL_INIT_OFF_RAD         0.0f
#undef M1_HFI_IQ_AUTH_ENABLE
#define M1_HFI_IQ_AUTH_ENABLE           1
#undef M1_HFI_IQ_AUTH_X_GOOD
#define M1_HFI_IQ_AUTH_X_GOOD           (0.218f)
#undef M1_HFI_IQ_AUTH_X_BAD
#define M1_HFI_IQ_AUTH_X_BAD            (0.205f)
#undef M1_HFI_IQ_AUTH_EPS_FALSE_MAX
#define M1_HFI_IQ_AUTH_EPS_FALSE_MAX    (0.20f)
#undef M1_HFI_IQ_AUTH_HOLD_N
#define M1_HFI_IQ_AUTH_HOLD_N           2000u
#undef M1_HFI_IQ_AUTH_CLEAR_N
#define M1_HFI_IQ_AUTH_CLEAR_N          1000u /* 50 ms @20 kHz；C3 为立即清�?*/
#undef M1_HFI_IQ_AUTH_IQ_LO
#define M1_HFI_IQ_AUTH_IQ_LO            (0.0f)
#undef M1_HFI_IQ_AUTH_IQ_HI
#define M1_HFI_IQ_AUTH_IQ_HI            (0.0f)
#undef M1_HFI_IQ_AUTH_SLEW_A_S
#define M1_HFI_IQ_AUTH_SLEW_A_S         (10.0f)

#elif M1_HFI_GATE == 13
/* C4v1（冻结对照）：knobs �?C4，但 FEED 误接 QKICK 支（1552/1612 FAIL）�?
 * 回退复现：GATE=13。正�?C4 �?GATE=14�?*/
#undef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE       0
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              1
#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             5.0f
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 0.0f
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               15.0f
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 0.0f
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               0.0f
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (-1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (-1.0f)
#undef M1_HFI_A_CMD
#define M1_HFI_A_CMD                    (0.10f)
#undef M1_HFI_PLL_INIT_OFF_RAD
#define M1_HFI_PLL_INIT_OFF_RAD         0.0f
#undef M1_HFI_IQ_AUTH_ENABLE
#define M1_HFI_IQ_AUTH_ENABLE           1
#undef M1_HFI_IQ_AUTH_X_GOOD
#define M1_HFI_IQ_AUTH_X_GOOD           (0.218f)
#undef M1_HFI_IQ_AUTH_X_BAD
#define M1_HFI_IQ_AUTH_X_BAD            (0.205f)
#undef M1_HFI_IQ_AUTH_EPS_FALSE_MAX
#define M1_HFI_IQ_AUTH_EPS_FALSE_MAX    (0.20f)
#undef M1_HFI_IQ_AUTH_HOLD_N
#define M1_HFI_IQ_AUTH_HOLD_N           2000u
#undef M1_HFI_IQ_AUTH_CLEAR_N
#define M1_HFI_IQ_AUTH_CLEAR_N          1000u
#undef M1_HFI_IQ_AUTH_IQ_LO
#define M1_HFI_IQ_AUTH_IQ_LO            (0.0f)
#undef M1_HFI_IQ_AUTH_IQ_HI
#define M1_HFI_IQ_AUTH_IQ_HI            (0.25f)
#undef M1_HFI_IQ_AUTH_SLEW_A_S
#define M1_HFI_IQ_AUTH_SLEW_A_S         (2.0f)
#undef M1_HFI_IQ_AUTH_FEED_ENABLE
#define M1_HFI_IQ_AUTH_FEED_ENABLE      1
#undef M1_HFI_IQ_AUTH_FEED_SIGN
#define M1_HFI_IQ_AUTH_FEED_SIGN        (1.0f)
#undef M1_HFI_IQ_AUTH_FEED_LEGACY_BRANCH
#define M1_HFI_IQ_AUTH_FEED_LEGACY_BRANCH 1 /* 旧误接：保留可复�?*/

#elif M1_HFI_GATE == 14
/* C4：控制同 C3b，唯一增量：qual_ok 后馈 0.25 A（解调后�?iq_ref）�?
 * 过线前电流环路径≡C3b（字�?id/iq/ω=0）�?�?3 冻结；回退 GATE=12 �?13�?*/
#undef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE       0
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              1
#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             5.0f
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 0.0f
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               15.0f
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 0.0f
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               0.0f
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (-1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (-1.0f)
#undef M1_HFI_A_CMD
#define M1_HFI_A_CMD                    (0.10f)
#undef M1_HFI_PLL_INIT_OFF_RAD
#define M1_HFI_PLL_INIT_OFF_RAD         0.0f
#undef M1_HFI_IQ_AUTH_ENABLE
#define M1_HFI_IQ_AUTH_ENABLE           1
#undef M1_HFI_IQ_AUTH_X_GOOD
#define M1_HFI_IQ_AUTH_X_GOOD           (0.218f)
#undef M1_HFI_IQ_AUTH_X_BAD
#define M1_HFI_IQ_AUTH_X_BAD            (0.205f)
#undef M1_HFI_IQ_AUTH_EPS_FALSE_MAX
#define M1_HFI_IQ_AUTH_EPS_FALSE_MAX    (0.20f)
#undef M1_HFI_IQ_AUTH_HOLD_N
#define M1_HFI_IQ_AUTH_HOLD_N           2000u
#undef M1_HFI_IQ_AUTH_CLEAR_N
#define M1_HFI_IQ_AUTH_CLEAR_N          1000u
#undef M1_HFI_IQ_AUTH_IQ_LO
#define M1_HFI_IQ_AUTH_IQ_LO            (0.0f)
#undef M1_HFI_IQ_AUTH_IQ_HI
#define M1_HFI_IQ_AUTH_IQ_HI            (0.25f)
#undef M1_HFI_IQ_AUTH_SLEW_A_S
#define M1_HFI_IQ_AUTH_SLEW_A_S         (2.0f)
#undef M1_HFI_IQ_AUTH_FEED_ENABLE
#define M1_HFI_IQ_AUTH_FEED_ENABLE      1
#undef M1_HFI_IQ_AUTH_FEED_SIGN
#define M1_HFI_IQ_AUTH_FEED_SIGN        (1.0f)
#undef M1_HFI_IQ_AUTH_FEED_LEGACY_BRANCH
#define M1_HFI_IQ_AUTH_FEED_LEGACY_BRANCH 0 /* 解调后馈；过线前≡C3b */

#elif M1_HFI_GATE == 15
/* C4a：控制同 C3b，唯一增量：FEED=1（解调后�?iq_ref）�?
 * HI=0 �?即使 qual_ok 也不出力。用来单独验 FEED 接线是否毁捕获�?
 * 1�?4 冻结；回退 GATE=12�?*/
#undef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE       0
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              1
#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             5.0f
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 0.0f
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               15.0f
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 0.0f
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               0.0f
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (-1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (-1.0f)
#undef M1_HFI_A_CMD
#define M1_HFI_A_CMD                    (0.10f)
#undef M1_HFI_PLL_INIT_OFF_RAD
#define M1_HFI_PLL_INIT_OFF_RAD         0.0f
#undef M1_HFI_IQ_AUTH_ENABLE
#define M1_HFI_IQ_AUTH_ENABLE           1
#undef M1_HFI_IQ_AUTH_X_GOOD
#define M1_HFI_IQ_AUTH_X_GOOD           (0.218f)
#undef M1_HFI_IQ_AUTH_X_BAD
#define M1_HFI_IQ_AUTH_X_BAD            (0.205f)
#undef M1_HFI_IQ_AUTH_EPS_FALSE_MAX
#define M1_HFI_IQ_AUTH_EPS_FALSE_MAX    (0.20f)
#undef M1_HFI_IQ_AUTH_HOLD_N
#define M1_HFI_IQ_AUTH_HOLD_N           2000u
#undef M1_HFI_IQ_AUTH_CLEAR_N
#define M1_HFI_IQ_AUTH_CLEAR_N          1000u
#undef M1_HFI_IQ_AUTH_IQ_LO
#define M1_HFI_IQ_AUTH_IQ_LO            (0.0f)
#undef M1_HFI_IQ_AUTH_IQ_HI
#define M1_HFI_IQ_AUTH_IQ_HI            (0.0f) /* 相对 C3b 不改 HI */
#undef M1_HFI_IQ_AUTH_SLEW_A_S
#define M1_HFI_IQ_AUTH_SLEW_A_S         (10.0f)
#undef M1_HFI_IQ_AUTH_FEED_ENABLE
#define M1_HFI_IQ_AUTH_FEED_ENABLE      1 /* 唯一增量 */
#undef M1_HFI_IQ_AUTH_FEED_SIGN
#define M1_HFI_IQ_AUTH_FEED_SIGN        (1.0f)
#undef M1_HFI_IQ_AUTH_FEED_LEGACY_BRANCH
#define M1_HFI_IQ_AUTH_FEED_LEGACY_BRANCH 0

#elif M1_HFI_GATE == 16
/* C4b：同 C4a（FEED 解调后写），唯一增量：HI=0.25（SLEW �?10，同 C3b）�?
 * 1723 C4a PASS �?�?HI 是否毁捕�?/ 是否正常出力�?�?5 冻结；回退 GATE=15�?*/
#undef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE       0
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              1
#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             5.0f
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 0.0f
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               15.0f
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 0.0f
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               0.0f
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (-1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (-1.0f)
#undef M1_HFI_A_CMD
#define M1_HFI_A_CMD                    (0.10f)
#undef M1_HFI_PLL_INIT_OFF_RAD
#define M1_HFI_PLL_INIT_OFF_RAD         0.0f
#undef M1_HFI_IQ_AUTH_ENABLE
#define M1_HFI_IQ_AUTH_ENABLE           1
#undef M1_HFI_IQ_AUTH_X_GOOD
#define M1_HFI_IQ_AUTH_X_GOOD           (0.218f)
#undef M1_HFI_IQ_AUTH_X_BAD
#define M1_HFI_IQ_AUTH_X_BAD            (0.205f)
#undef M1_HFI_IQ_AUTH_EPS_FALSE_MAX
#define M1_HFI_IQ_AUTH_EPS_FALSE_MAX    (0.20f)
#undef M1_HFI_IQ_AUTH_HOLD_N
#define M1_HFI_IQ_AUTH_HOLD_N           2000u
#undef M1_HFI_IQ_AUTH_CLEAR_N
#define M1_HFI_IQ_AUTH_CLEAR_N          1000u
#undef M1_HFI_IQ_AUTH_IQ_LO
#define M1_HFI_IQ_AUTH_IQ_LO            (0.0f)
#undef M1_HFI_IQ_AUTH_IQ_HI
#define M1_HFI_IQ_AUTH_IQ_HI            (0.25f) /* 唯一增量 vs C4a */
#undef M1_HFI_IQ_AUTH_SLEW_A_S
#define M1_HFI_IQ_AUTH_SLEW_A_S         (10.0f) /* 仍同 C3b，勿�?SLEW=2 */
#undef M1_HFI_IQ_AUTH_FEED_ENABLE
#define M1_HFI_IQ_AUTH_FEED_ENABLE      1
#undef M1_HFI_IQ_AUTH_FEED_SIGN
#define M1_HFI_IQ_AUTH_FEED_SIGN        (1.0f)
#undef M1_HFI_IQ_AUTH_FEED_LEGACY_BRANCH
#define M1_HFI_IQ_AUTH_FEED_LEGACY_BRANCH 0

#elif M1_HFI_GATE == 17
/* C4c：同 C4a（FEED + HI=0 捕获），唯一增量：ok 后馈 FEED_A=0.25�?
 * 不把 HI 抬到 0.25�?730：HI=0.25 毁捕获）�?�?6 冻结；回退 GATE=15/16�?*/
#undef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE       0
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              1
#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             5.0f
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 0.0f
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               15.0f
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 0.0f
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               0.0f
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (-1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (-1.0f)
#undef M1_HFI_A_CMD
#define M1_HFI_A_CMD                    (0.10f)
#undef M1_HFI_PLL_INIT_OFF_RAD
#define M1_HFI_PLL_INIT_OFF_RAD         0.0f
#undef M1_HFI_IQ_AUTH_ENABLE
#define M1_HFI_IQ_AUTH_ENABLE           1
#undef M1_HFI_IQ_AUTH_X_GOOD
#define M1_HFI_IQ_AUTH_X_GOOD           (0.218f)
#undef M1_HFI_IQ_AUTH_X_BAD
#define M1_HFI_IQ_AUTH_X_BAD            (0.205f)
#undef M1_HFI_IQ_AUTH_EPS_FALSE_MAX
#define M1_HFI_IQ_AUTH_EPS_FALSE_MAX    (0.20f)
#undef M1_HFI_IQ_AUTH_HOLD_N
#define M1_HFI_IQ_AUTH_HOLD_N           2000u
#undef M1_HFI_IQ_AUTH_CLEAR_N
#define M1_HFI_IQ_AUTH_CLEAR_N          1000u
#undef M1_HFI_IQ_AUTH_IQ_LO
#define M1_HFI_IQ_AUTH_IQ_LO            (0.0f)
#undef M1_HFI_IQ_AUTH_IQ_HI
#define M1_HFI_IQ_AUTH_IQ_HI            (0.0f) /* 捕获期天花板保持 0 */
#undef M1_HFI_IQ_AUTH_SLEW_A_S
#define M1_HFI_IQ_AUTH_SLEW_A_S         (10.0f)
#undef M1_HFI_IQ_AUTH_FEED_ENABLE
#define M1_HFI_IQ_AUTH_FEED_ENABLE      1
#undef M1_HFI_IQ_AUTH_FEED_SIGN
#define M1_HFI_IQ_AUTH_FEED_SIGN        (1.0f)
#undef M1_HFI_IQ_AUTH_FEED_A
#define M1_HFI_IQ_AUTH_FEED_A           (0.25f) /* ok 后固定馈流；不走 HI */
#undef M1_HFI_IQ_AUTH_FEED_LEGACY_BRANCH
#define M1_HFI_IQ_AUTH_FEED_LEGACY_BRANCH 0

#elif M1_HFI_GATE == 18
/* C4d：同 C4c（HI=0 + FEED_A=0.25），唯一增量：一上电�?A �?B�?
 * A：qual_ok 后再�?0.20 s 阶跃�?0.25（HOLD 100 ms 已含在旗里）�?
 * B：ok �?0.80 s 斜坡�?0.25。不�?C（降 FEED_A）�?�?7 冻结�?*/
#undef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE       0
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              1
#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             5.0f
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 0.0f
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               15.0f
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 0.0f
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               0.0f
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (-1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (-1.0f)
#undef M1_HFI_A_CMD
#define M1_HFI_A_CMD                    (0.10f)
#undef M1_HFI_PLL_INIT_OFF_RAD
#define M1_HFI_PLL_INIT_OFF_RAD         0.0f
#undef M1_HFI_IQ_AUTH_ENABLE
#define M1_HFI_IQ_AUTH_ENABLE           1
#undef M1_HFI_IQ_AUTH_X_GOOD
#define M1_HFI_IQ_AUTH_X_GOOD           (0.218f)
#undef M1_HFI_IQ_AUTH_X_BAD
#define M1_HFI_IQ_AUTH_X_BAD            (0.205f)
#undef M1_HFI_IQ_AUTH_EPS_FALSE_MAX
#define M1_HFI_IQ_AUTH_EPS_FALSE_MAX    (0.20f)
#undef M1_HFI_IQ_AUTH_HOLD_N
#define M1_HFI_IQ_AUTH_HOLD_N           2000u
#undef M1_HFI_IQ_AUTH_CLEAR_N
#define M1_HFI_IQ_AUTH_CLEAR_N          1000u
#undef M1_HFI_IQ_AUTH_IQ_LO
#define M1_HFI_IQ_AUTH_IQ_LO            (0.0f)
#undef M1_HFI_IQ_AUTH_IQ_HI
#define M1_HFI_IQ_AUTH_IQ_HI            (0.0f)
#undef M1_HFI_IQ_AUTH_SLEW_A_S
#define M1_HFI_IQ_AUTH_SLEW_A_S         (10.0f)
#undef M1_HFI_IQ_AUTH_FEED_ENABLE
#define M1_HFI_IQ_AUTH_FEED_ENABLE      1
#undef M1_HFI_IQ_AUTH_FEED_SIGN
#define M1_HFI_IQ_AUTH_FEED_SIGN        (1.0f)
#undef M1_HFI_IQ_AUTH_FEED_A
#define M1_HFI_IQ_AUTH_FEED_A           (0.25f)
#undef M1_HFI_IQ_AUTH_FEED_COMPARE_AB
#define M1_HFI_IQ_AUTH_FEED_COMPARE_AB  1
#undef M1_HFI_IQ_AUTH_FEED_A_DELAY_S
#define M1_HFI_IQ_AUTH_FEED_A_DELAY_S   (0.20f)
#undef M1_HFI_IQ_AUTH_FEED_A_RAMP_S
#define M1_HFI_IQ_AUTH_FEED_A_RAMP_S    (0.0f)
#undef M1_HFI_IQ_AUTH_FEED_B_DELAY_S
#define M1_HFI_IQ_AUTH_FEED_B_DELAY_S   (0.0f)
#undef M1_HFI_IQ_AUTH_FEED_B_RAMP_S
#define M1_HFI_IQ_AUTH_FEED_B_RAMP_S    (0.80f)
#undef M1_HFI_IQ_AUTH_FEED_LEGACY_BRANCH
#define M1_HFI_IQ_AUTH_FEED_LEGACY_BRANCH 0

#elif M1_HFI_GATE == 19
/* C4e：控�?�?C4a（FEED+HI=0，Iq=0）。唯一增量：VOFA �?x_raw/Ud_inj/di_d�?
 * 1�?8 冻结。离线：_analyze_hfi_c4e_diag.py */
#undef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE       0
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              1
#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             5.0f
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 0.0f
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               15.0f
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 0.0f
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               0.0f
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (-1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (-1.0f)
#undef M1_HFI_A_CMD
#define M1_HFI_A_CMD                    (0.10f)
#undef M1_HFI_PLL_INIT_OFF_RAD
#define M1_HFI_PLL_INIT_OFF_RAD         0.0f
#undef M1_HFI_IQ_AUTH_ENABLE
#define M1_HFI_IQ_AUTH_ENABLE           1
#undef M1_HFI_IQ_AUTH_X_GOOD
#define M1_HFI_IQ_AUTH_X_GOOD           (0.218f)
#undef M1_HFI_IQ_AUTH_X_BAD
#define M1_HFI_IQ_AUTH_X_BAD            (0.205f)
#undef M1_HFI_IQ_AUTH_EPS_FALSE_MAX
#define M1_HFI_IQ_AUTH_EPS_FALSE_MAX    (0.20f)
#undef M1_HFI_IQ_AUTH_HOLD_N
#define M1_HFI_IQ_AUTH_HOLD_N           2000u
#undef M1_HFI_IQ_AUTH_CLEAR_N
#define M1_HFI_IQ_AUTH_CLEAR_N          1000u
#undef M1_HFI_IQ_AUTH_IQ_LO
#define M1_HFI_IQ_AUTH_IQ_LO            (0.0f)
#undef M1_HFI_IQ_AUTH_IQ_HI
#define M1_HFI_IQ_AUTH_IQ_HI            (0.0f)
#undef M1_HFI_IQ_AUTH_SLEW_A_S
#define M1_HFI_IQ_AUTH_SLEW_A_S         (10.0f)
#undef M1_HFI_IQ_AUTH_FEED_ENABLE
#define M1_HFI_IQ_AUTH_FEED_ENABLE      1
#undef M1_HFI_IQ_AUTH_FEED_SIGN
#define M1_HFI_IQ_AUTH_FEED_SIGN        (1.0f)
#undef M1_HFI_IQ_AUTH_FEED_LEGACY_BRANCH
#define M1_HFI_IQ_AUTH_FEED_LEGACY_BRANCH 0

#elif M1_HFI_GATE == 20
/* C4f：相�?C4e 唯一增量 = 打开 S3b 已签收的 q 踢（QKICK_AFTER_LOCK）�?
 * 流程：IDLE→RUN(PRE)→MEAS 踢→BRAKE→LOG(Iq=0+HFI)→DONE。不开速度环、不�?FEED�?
 * 踢参数对�?GATE=4�?252 PASS）。回退 GATE=19→C4e / 4→S3b�?�?9 冻结�?*/
#undef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE       0
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              1
#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             5.0f
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 0.0f
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               8.0f
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 0.0f
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               0.0f
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (-1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (-1.0f)
#undef M1_HFI_A_CMD
#define M1_HFI_A_CMD                    (0.10f)
#undef M1_HFI_PLL_INIT_OFF_RAD
#define M1_HFI_PLL_INIT_OFF_RAD         0.0f
#undef M1_HFI_INIT_FROM_ENC
#define M1_HFI_INIT_FROM_ENC            1
#undef M1_HFI_POLARITY_IPD_ENABLE
#define M1_HFI_POLARITY_IPD_ENABLE      0
/* --- 唯一增量：S3b q 踢宏 --- */
#undef M1_HFI_QKICK_AFTER_LOCK_ENABLE
#define M1_HFI_QKICK_AFTER_LOCK_ENABLE  1
#undef M1_HFI_QKICK_PRE_S
#define M1_HFI_QKICK_PRE_S              2.0f
#undef M1_HFI_QKICK_IQ_A
#define M1_HFI_QKICK_IQ_A               1.6f
#undef M1_HFI_QKICK_KICK_N
#define M1_HFI_QKICK_KICK_N             6000u /* 300 ms */
#undef M1_HFI_QKICK_BRAKE_ENABLE
#define M1_HFI_QKICK_BRAKE_ENABLE       1
#undef M1_HFI_QKICK_BRAKE_N
#define M1_HFI_QKICK_BRAKE_N            6000u
#undef M1_HFI_QKICK_HOLD_S
#define M1_HFI_QKICK_HOLD_S             5.0f
#undef M1_HFI_QKICK_SPEED_ENABLE
#define M1_HFI_QKICK_SPEED_ENABLE       0
#undef M1_HFI_QKICK_CRAWL_ENABLE
#define M1_HFI_QKICK_CRAWL_ENABLE       0
#undef M1_HFI_QKICK_START_ENABLE
#define M1_HFI_QKICK_START_ENABLE       0
/* --- 保留 C4e：质量旗 + FEED 接线�?HI=0（踢后仍不出力）--- */
#undef M1_HFI_IQ_AUTH_ENABLE
#define M1_HFI_IQ_AUTH_ENABLE           1
#undef M1_HFI_IQ_AUTH_X_GOOD
#define M1_HFI_IQ_AUTH_X_GOOD           (0.218f)
#undef M1_HFI_IQ_AUTH_X_BAD
#define M1_HFI_IQ_AUTH_X_BAD            (0.205f)
#undef M1_HFI_IQ_AUTH_EPS_FALSE_MAX
#define M1_HFI_IQ_AUTH_EPS_FALSE_MAX    (0.20f)
#undef M1_HFI_IQ_AUTH_HOLD_N
#define M1_HFI_IQ_AUTH_HOLD_N           2000u
#undef M1_HFI_IQ_AUTH_CLEAR_N
#define M1_HFI_IQ_AUTH_CLEAR_N          1000u
#undef M1_HFI_IQ_AUTH_IQ_LO
#define M1_HFI_IQ_AUTH_IQ_LO            (0.0f)
#undef M1_HFI_IQ_AUTH_IQ_HI
#define M1_HFI_IQ_AUTH_IQ_HI            (0.0f)
#undef M1_HFI_IQ_AUTH_SLEW_A_S
#define M1_HFI_IQ_AUTH_SLEW_A_S         (10.0f)
#undef M1_HFI_IQ_AUTH_FEED_ENABLE
#define M1_HFI_IQ_AUTH_FEED_ENABLE      1
#undef M1_HFI_IQ_AUTH_FEED_SIGN
#define M1_HFI_IQ_AUTH_FEED_SIGN        (1.0f)
#undef M1_HFI_IQ_AUTH_FEED_LEGACY_BRANCH
#define M1_HFI_IQ_AUTH_FEED_LEGACY_BRANCH 0

#elif M1_HFI_GATE == 21
/* C4g：相�?C4f 唯一增量 = QKICK_BEFORE_HFI（IDLE→踢→HFI RUN，禁�?PRE 先跟踪）�?
 * 踢参数仍对齐 S3b。回退 GATE=20→旧 AFTER_LOCK / 4→S3b�?�?0 冻结�?*/
#undef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE       0
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              1
#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             5.0f
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 0.0f
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               15.0f
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 0.0f
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               0.0f
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (-1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (-1.0f)
#undef M1_HFI_A_CMD
#define M1_HFI_A_CMD                    (0.10f)
#undef M1_HFI_PLL_INIT_OFF_RAD
#define M1_HFI_PLL_INIT_OFF_RAD         0.0f
#undef M1_HFI_INIT_FROM_ENC
#define M1_HFI_INIT_FROM_ENC            1
#undef M1_HFI_POLARITY_IPD_ENABLE
#define M1_HFI_POLARITY_IPD_ENABLE      0
#undef M1_HFI_QKICK_AFTER_LOCK_ENABLE
#define M1_HFI_QKICK_AFTER_LOCK_ENABLE  1 /* 复用�?MEAS 实现 */
#undef M1_HFI_QKICK_BEFORE_HFI_ENABLE
#define M1_HFI_QKICK_BEFORE_HFI_ENABLE  1 /* 唯一增量：先踢再 HFI */
#undef M1_HFI_QKICK_PRE_S
#define M1_HFI_QKICK_PRE_S              2.0f /* BEFORE_HFI 下不使用 */
#undef M1_HFI_QKICK_IQ_A
#define M1_HFI_QKICK_IQ_A               1.6f
#undef M1_HFI_QKICK_KICK_N
#define M1_HFI_QKICK_KICK_N             6000u
#undef M1_HFI_QKICK_BRAKE_ENABLE
#define M1_HFI_QKICK_BRAKE_ENABLE       1
#undef M1_HFI_QKICK_BRAKE_N
#define M1_HFI_QKICK_BRAKE_N            6000u
#undef M1_HFI_QKICK_HOLD_S
#define M1_HFI_QKICK_HOLD_S             5.0f /* BEFORE_HFI 不走 LOG，保留宏 */
#undef M1_HFI_QKICK_SPEED_ENABLE
#define M1_HFI_QKICK_SPEED_ENABLE       0
#undef M1_HFI_QKICK_CRAWL_ENABLE
#define M1_HFI_QKICK_CRAWL_ENABLE       0
#undef M1_HFI_QKICK_START_ENABLE
#define M1_HFI_QKICK_START_ENABLE       0
#undef M1_HFI_IQ_AUTH_ENABLE
#define M1_HFI_IQ_AUTH_ENABLE           1
#undef M1_HFI_IQ_AUTH_X_GOOD
#define M1_HFI_IQ_AUTH_X_GOOD           (0.218f)
#undef M1_HFI_IQ_AUTH_X_BAD
#define M1_HFI_IQ_AUTH_X_BAD            (0.205f)
#undef M1_HFI_IQ_AUTH_EPS_FALSE_MAX
#define M1_HFI_IQ_AUTH_EPS_FALSE_MAX    (0.20f)
#undef M1_HFI_IQ_AUTH_HOLD_N
#define M1_HFI_IQ_AUTH_HOLD_N           2000u
#undef M1_HFI_IQ_AUTH_CLEAR_N
#define M1_HFI_IQ_AUTH_CLEAR_N          1000u
#undef M1_HFI_IQ_AUTH_IQ_LO
#define M1_HFI_IQ_AUTH_IQ_LO            (0.0f)
#undef M1_HFI_IQ_AUTH_IQ_HI
#define M1_HFI_IQ_AUTH_IQ_HI            (0.0f)
#undef M1_HFI_IQ_AUTH_SLEW_A_S
#define M1_HFI_IQ_AUTH_SLEW_A_S         (10.0f)
#undef M1_HFI_IQ_AUTH_FEED_ENABLE
#define M1_HFI_IQ_AUTH_FEED_ENABLE      1
#undef M1_HFI_IQ_AUTH_FEED_SIGN
#define M1_HFI_IQ_AUTH_FEED_SIGN        (1.0f)
#undef M1_HFI_IQ_AUTH_FEED_LEGACY_BRANCH
#define M1_HFI_IQ_AUTH_FEED_LEGACY_BRANCH 0

#elif M1_HFI_GATE == 22
/* C4h：相�?C4g 唯一增量 = 踢前 HF settle（冻 PLL，破�?电流环就绪）�?
 * 2142：Iq±1.6 已跟、Δ�?0；对�?S3b �?2s HF 预热。不做漂�?PRE�?
 * 回退 GATE=21→C4g / 4→S3b�?�?1 冻结�?*/
#undef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE       0
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              1
#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             5.0f
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 0.0f
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               15.0f
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 0.0f
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               0.0f
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (-1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (-1.0f)
#undef M1_HFI_A_CMD
#define M1_HFI_A_CMD                    (0.10f)
#undef M1_HFI_PLL_INIT_OFF_RAD
#define M1_HFI_PLL_INIT_OFF_RAD         0.0f
#undef M1_HFI_INIT_FROM_ENC
#define M1_HFI_INIT_FROM_ENC            1
#undef M1_HFI_POLARITY_IPD_ENABLE
#define M1_HFI_POLARITY_IPD_ENABLE      0
#undef M1_HFI_QKICK_AFTER_LOCK_ENABLE
#define M1_HFI_QKICK_AFTER_LOCK_ENABLE  1
#undef M1_HFI_QKICK_BEFORE_HFI_ENABLE
#define M1_HFI_QKICK_BEFORE_HFI_ENABLE  1
#undef M1_HFI_QKICK_BEFORE_SETTLE_N
#define M1_HFI_QKICK_BEFORE_SETTLE_N    4000u /* 唯一增量�?00 ms @20kHz HF settle */
#undef M1_HFI_QKICK_PRE_S
#define M1_HFI_QKICK_PRE_S              2.0f
#undef M1_HFI_QKICK_IQ_A
#define M1_HFI_QKICK_IQ_A               1.6f
#undef M1_HFI_QKICK_KICK_N
#define M1_HFI_QKICK_KICK_N             6000u
#undef M1_HFI_QKICK_BRAKE_ENABLE
#define M1_HFI_QKICK_BRAKE_ENABLE       1
#undef M1_HFI_QKICK_BRAKE_N
#define M1_HFI_QKICK_BRAKE_N            6000u
#undef M1_HFI_QKICK_HOLD_S
#define M1_HFI_QKICK_HOLD_S             5.0f
#undef M1_HFI_QKICK_SPEED_ENABLE
#define M1_HFI_QKICK_SPEED_ENABLE       0
#undef M1_HFI_QKICK_CRAWL_ENABLE
#define M1_HFI_QKICK_CRAWL_ENABLE       0
#undef M1_HFI_QKICK_START_ENABLE
#define M1_HFI_QKICK_START_ENABLE       0
#undef M1_HFI_IQ_AUTH_ENABLE
#define M1_HFI_IQ_AUTH_ENABLE           1
#undef M1_HFI_IQ_AUTH_X_GOOD
#define M1_HFI_IQ_AUTH_X_GOOD           (0.218f)
#undef M1_HFI_IQ_AUTH_X_BAD
#define M1_HFI_IQ_AUTH_X_BAD            (0.205f)
#undef M1_HFI_IQ_AUTH_EPS_FALSE_MAX
#define M1_HFI_IQ_AUTH_EPS_FALSE_MAX    (0.20f)
#undef M1_HFI_IQ_AUTH_HOLD_N
#define M1_HFI_IQ_AUTH_HOLD_N           2000u
#undef M1_HFI_IQ_AUTH_CLEAR_N
#define M1_HFI_IQ_AUTH_CLEAR_N          1000u
#undef M1_HFI_IQ_AUTH_IQ_LO
#define M1_HFI_IQ_AUTH_IQ_LO            (0.0f)
#undef M1_HFI_IQ_AUTH_IQ_HI
#define M1_HFI_IQ_AUTH_IQ_HI            (0.0f)
#undef M1_HFI_IQ_AUTH_SLEW_A_S
#define M1_HFI_IQ_AUTH_SLEW_A_S         (10.0f)
#undef M1_HFI_IQ_AUTH_FEED_ENABLE
#define M1_HFI_IQ_AUTH_FEED_ENABLE      1
#undef M1_HFI_IQ_AUTH_FEED_SIGN
#define M1_HFI_IQ_AUTH_FEED_SIGN        (1.0f)
#undef M1_HFI_IQ_AUTH_FEED_LEGACY_BRANCH
#define M1_HFI_IQ_AUTH_FEED_LEGACY_BRANCH 0

#elif M1_HFI_GATE == 23
/* C4i：相�?C4g 唯一增量 = BEFORE_HFI �?MEAS 清注入（�?Iq 踢，踢完再开 HFI）�?
 * 不做 settle（那是提前开 HF）。回退 GATE=21→C4g / 4→S3b�?�?2 冻结�?*/
#undef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE       0
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              1
#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             5.0f
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 0.0f
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               15.0f
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 0.0f
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               0.0f
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (-1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (-1.0f)
#undef M1_HFI_A_CMD
#define M1_HFI_A_CMD                    (0.10f)
#undef M1_HFI_PLL_INIT_OFF_RAD
#define M1_HFI_PLL_INIT_OFF_RAD         0.0f
#undef M1_HFI_INIT_FROM_ENC
#define M1_HFI_INIT_FROM_ENC            1
#undef M1_HFI_POLARITY_IPD_ENABLE
#define M1_HFI_POLARITY_IPD_ENABLE      0
#undef M1_HFI_QKICK_AFTER_LOCK_ENABLE
#define M1_HFI_QKICK_AFTER_LOCK_ENABLE  1
#undef M1_HFI_QKICK_BEFORE_HFI_ENABLE
#define M1_HFI_QKICK_BEFORE_HFI_ENABLE  1
#undef M1_HFI_QKICK_BEFORE_SETTLE_N
#define M1_HFI_QKICK_BEFORE_SETTLE_N    0u /* 不做提前 HF */
#undef M1_HFI_QKICK_PRE_S
#define M1_HFI_QKICK_PRE_S              2.0f
#undef M1_HFI_QKICK_IQ_A
#define M1_HFI_QKICK_IQ_A               1.6f
#undef M1_HFI_QKICK_KICK_N
#define M1_HFI_QKICK_KICK_N             6000u
#undef M1_HFI_QKICK_BRAKE_ENABLE
#define M1_HFI_QKICK_BRAKE_ENABLE       1
#undef M1_HFI_QKICK_BRAKE_N
#define M1_HFI_QKICK_BRAKE_N            6000u
#undef M1_HFI_QKICK_HOLD_S
#define M1_HFI_QKICK_HOLD_S             5.0f
#undef M1_HFI_QKICK_SPEED_ENABLE
#define M1_HFI_QKICK_SPEED_ENABLE       0
#undef M1_HFI_QKICK_CRAWL_ENABLE
#define M1_HFI_QKICK_CRAWL_ENABLE       0
#undef M1_HFI_QKICK_START_ENABLE
#define M1_HFI_QKICK_START_ENABLE       0
#undef M1_HFI_IQ_AUTH_ENABLE
#define M1_HFI_IQ_AUTH_ENABLE           1
#undef M1_HFI_IQ_AUTH_X_GOOD
#define M1_HFI_IQ_AUTH_X_GOOD           (0.218f)
#undef M1_HFI_IQ_AUTH_X_BAD
#define M1_HFI_IQ_AUTH_X_BAD            (0.205f)
#undef M1_HFI_IQ_AUTH_EPS_FALSE_MAX
#define M1_HFI_IQ_AUTH_EPS_FALSE_MAX    (0.20f)
#undef M1_HFI_IQ_AUTH_HOLD_N
#define M1_HFI_IQ_AUTH_HOLD_N           2000u
#undef M1_HFI_IQ_AUTH_CLEAR_N
#define M1_HFI_IQ_AUTH_CLEAR_N          1000u
#undef M1_HFI_IQ_AUTH_IQ_LO
#define M1_HFI_IQ_AUTH_IQ_LO            (0.0f)
#undef M1_HFI_IQ_AUTH_IQ_HI
#define M1_HFI_IQ_AUTH_IQ_HI            (0.0f)
#undef M1_HFI_IQ_AUTH_SLEW_A_S
#define M1_HFI_IQ_AUTH_SLEW_A_S         (10.0f)
#undef M1_HFI_IQ_AUTH_FEED_ENABLE
#define M1_HFI_IQ_AUTH_FEED_ENABLE      1
#undef M1_HFI_IQ_AUTH_FEED_SIGN
#define M1_HFI_IQ_AUTH_FEED_SIGN        (1.0f)
#undef M1_HFI_IQ_AUTH_FEED_LEGACY_BRANCH
#define M1_HFI_IQ_AUTH_FEED_LEGACY_BRANCH 0

#elif M1_HFI_GATE == 24
/* C4j：沿�?S3b（AFTER_LOCK 踢），唯一增量 = 踢后�?hold �?HFI RUN（THEN_HFI）�?
 * IDLE→RUN(PRE)→MEAS 踢→BRAKE→LOG 0.4s→RUN 15s（沿用踢�?θ̂）→DONE�?
 * 不做 BEFORE_HFI。回退 GATE=4→S3b�?�?3 冻结�?*/
#undef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE       0
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              1
#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             5.0f
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 0.0f
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               15.0f /* 踢后第二�?HFI RUN */
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 0.0f
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               0.0f
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (-1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (-1.0f)
#undef M1_HFI_A_CMD
#define M1_HFI_A_CMD                    (0.10f)
#undef M1_HFI_PLL_INIT_OFF_RAD
#define M1_HFI_PLL_INIT_OFF_RAD         0.0f
#undef M1_HFI_INIT_FROM_ENC
#define M1_HFI_INIT_FROM_ENC            1
#undef M1_HFI_POLARITY_IPD_ENABLE
#define M1_HFI_POLARITY_IPD_ENABLE      0
#undef M1_HFI_QKICK_AFTER_LOCK_ENABLE
#define M1_HFI_QKICK_AFTER_LOCK_ENABLE  1
#undef M1_HFI_QKICK_BEFORE_HFI_ENABLE
#define M1_HFI_QKICK_BEFORE_HFI_ENABLE  0
#undef M1_HFI_QKICK_THEN_HFI_ENABLE
#define M1_HFI_QKICK_THEN_HFI_ENABLE    1 /* 唯一增量 */
#undef M1_HFI_QKICK_PRE_S
#define M1_HFI_QKICK_PRE_S              2.0f
#undef M1_HFI_QKICK_IQ_A
#define M1_HFI_QKICK_IQ_A               1.6f
#undef M1_HFI_QKICK_KICK_N
#define M1_HFI_QKICK_KICK_N             6000u
#undef M1_HFI_QKICK_BRAKE_ENABLE
#define M1_HFI_QKICK_BRAKE_ENABLE       1
#undef M1_HFI_QKICK_BRAKE_N
#define M1_HFI_QKICK_BRAKE_N            6000u
#undef M1_HFI_QKICK_HOLD_S
#define M1_HFI_QKICK_HOLD_S             0.4f /* 踢后几百 ms 再进 HFI RUN */
#undef M1_HFI_QKICK_SPEED_ENABLE
#define M1_HFI_QKICK_SPEED_ENABLE       0
#undef M1_HFI_QKICK_CRAWL_ENABLE
#define M1_HFI_QKICK_CRAWL_ENABLE       0
#undef M1_HFI_QKICK_START_ENABLE
#define M1_HFI_QKICK_START_ENABLE       0

#elif M1_HFI_GATE == 25
/* C4k：相�?C4j 唯一增量 = 踢后 LOG + 第二�?RUN �?Id PI（Ud_pi=0，只�?±Vh）�?
 * q 环照开；MEAS 踢段仍走 Id PI。回退 GATE=24→C4j�?�?4 冻结�?*/
#undef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE       0
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              1
#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             5.0f
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 0.0f
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               15.0f
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 0.0f
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               0.0f
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (-1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (-1.0f)
#undef M1_HFI_A_CMD
#define M1_HFI_A_CMD                    (0.10f)
#undef M1_HFI_PLL_INIT_OFF_RAD
#define M1_HFI_PLL_INIT_OFF_RAD         0.0f
#undef M1_HFI_INIT_FROM_ENC
#define M1_HFI_INIT_FROM_ENC            1
#undef M1_HFI_POLARITY_IPD_ENABLE
#define M1_HFI_POLARITY_IPD_ENABLE      0
#undef M1_HFI_QKICK_AFTER_LOCK_ENABLE
#define M1_HFI_QKICK_AFTER_LOCK_ENABLE  1
#undef M1_HFI_QKICK_BEFORE_HFI_ENABLE
#define M1_HFI_QKICK_BEFORE_HFI_ENABLE  0
#undef M1_HFI_QKICK_THEN_HFI_ENABLE
#define M1_HFI_QKICK_THEN_HFI_ENABLE    1
#undef M1_HFI_QKICK_PRE_S
#define M1_HFI_QKICK_PRE_S              2.0f
#undef M1_HFI_QKICK_IQ_A
#define M1_HFI_QKICK_IQ_A               1.6f
#undef M1_HFI_QKICK_KICK_N
#define M1_HFI_QKICK_KICK_N             6000u
#undef M1_HFI_QKICK_BRAKE_ENABLE
#define M1_HFI_QKICK_BRAKE_ENABLE       1
#undef M1_HFI_QKICK_BRAKE_N
#define M1_HFI_QKICK_BRAKE_N            6000u
#undef M1_HFI_QKICK_HOLD_S
#define M1_HFI_QKICK_HOLD_S             0.4f
#undef M1_HFI_QKICK_SPEED_ENABLE
#define M1_HFI_QKICK_SPEED_ENABLE       0
#undef M1_HFI_QKICK_CRAWL_ENABLE
#define M1_HFI_QKICK_CRAWL_ENABLE       0
#undef M1_HFI_QKICK_START_ENABLE
#define M1_HFI_QKICK_START_ENABLE       0
#undef M1_HFI_ID_PI_OFF_ENABLE
#define M1_HFI_ID_PI_OFF_ENABLE         1 /* 唯一增量：踢后看凸极�?d 环让�?*/

#elif M1_HFI_GATE == 26
/* C4l：相�?C4k 唯一增量 = 踢前 θ̂+=π（QKICK_FORCE_PI）�?
 * 1330：干净 PRE �?θ̂≈enc、Iq±1.6 跟住�?Δθ=0�?241 能动�?PRE �?θ_err�?75°�?
 * 本枪只把进踢瞬间的力矩轴翻到南，验证「北极静置锁」是否就是踢不动的根因�?
 * 回退 GATE=25→C4k�?�?5 冻结�?*/
#undef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE       0
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              1
#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             5.0f
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 0.0f
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               15.0f
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 0.0f
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               0.0f
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (-1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (-1.0f)
#undef M1_HFI_A_CMD
#define M1_HFI_A_CMD                    (0.10f)
#undef M1_HFI_PLL_INIT_OFF_RAD
#define M1_HFI_PLL_INIT_OFF_RAD         0.0f
#undef M1_HFI_INIT_FROM_ENC
#define M1_HFI_INIT_FROM_ENC            1
#undef M1_HFI_POLARITY_IPD_ENABLE
#define M1_HFI_POLARITY_IPD_ENABLE      0
#undef M1_HFI_QKICK_AFTER_LOCK_ENABLE
#define M1_HFI_QKICK_AFTER_LOCK_ENABLE  1
#undef M1_HFI_QKICK_BEFORE_HFI_ENABLE
#define M1_HFI_QKICK_BEFORE_HFI_ENABLE  0
#undef M1_HFI_QKICK_THEN_HFI_ENABLE
#define M1_HFI_QKICK_THEN_HFI_ENABLE    1
#undef M1_HFI_QKICK_PRE_S
#define M1_HFI_QKICK_PRE_S              2.0f
#undef M1_HFI_QKICK_IQ_A
#define M1_HFI_QKICK_IQ_A               1.6f
#undef M1_HFI_QKICK_KICK_N
#define M1_HFI_QKICK_KICK_N             6000u
#undef M1_HFI_QKICK_BRAKE_ENABLE
#define M1_HFI_QKICK_BRAKE_ENABLE       1
#undef M1_HFI_QKICK_BRAKE_N
#define M1_HFI_QKICK_BRAKE_N            6000u
#undef M1_HFI_QKICK_HOLD_S
#define M1_HFI_QKICK_HOLD_S             0.4f
#undef M1_HFI_QKICK_SPEED_ENABLE
#define M1_HFI_QKICK_SPEED_ENABLE       0
#undef M1_HFI_QKICK_CRAWL_ENABLE
#define M1_HFI_QKICK_CRAWL_ENABLE       0
#undef M1_HFI_QKICK_START_ENABLE
#define M1_HFI_QKICK_START_ENABLE       0
#undef M1_HFI_ID_PI_OFF_ENABLE
#define M1_HFI_ID_PI_OFF_ENABLE         1
#undef M1_HFI_QKICK_FORCE_PI
#define M1_HFI_QKICK_FORCE_PI           1 /* 唯一增量：进踢前 θ̂+=π */

#elif M1_HFI_GATE == 27
/* C4m：相�?C4k 唯一增量 = A_cmd=0.238（S1 开�?A，XY=�? 后取正）�?
 * 1402：FORCE_PI 已排除南北；PRE |x|�?.21≈A−B �?ε�? �?A_cmd=0.10 �?q 轴也变成 ε=0�?
 * 本枪�?A_cmd，使 q �?x−A_cmd<0 �?ε→±π/2，�?PLL 离开假锁再踢�?
 * 不做 FORCE_PI。回退 GATE=25→C4k�?�?6 冻结�?*/
#undef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE       0
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              1
#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             5.0f
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 0.0f
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               15.0f
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 0.0f
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               0.0f
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (-1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (-1.0f)
#undef M1_HFI_A_CMD
#define M1_HFI_A_CMD                    (0.238f) /* 唯一增量 vs C4k(0.10) */
#undef M1_HFI_PLL_INIT_OFF_RAD
#define M1_HFI_PLL_INIT_OFF_RAD         0.0f
#undef M1_HFI_INIT_FROM_ENC
#define M1_HFI_INIT_FROM_ENC            1
#undef M1_HFI_POLARITY_IPD_ENABLE
#define M1_HFI_POLARITY_IPD_ENABLE      0
#undef M1_HFI_QKICK_AFTER_LOCK_ENABLE
#define M1_HFI_QKICK_AFTER_LOCK_ENABLE  1
#undef M1_HFI_QKICK_BEFORE_HFI_ENABLE
#define M1_HFI_QKICK_BEFORE_HFI_ENABLE  0
#undef M1_HFI_QKICK_THEN_HFI_ENABLE
#define M1_HFI_QKICK_THEN_HFI_ENABLE    1
#undef M1_HFI_QKICK_PRE_S
#define M1_HFI_QKICK_PRE_S              2.0f
#undef M1_HFI_QKICK_IQ_A
#define M1_HFI_QKICK_IQ_A               1.6f
#undef M1_HFI_QKICK_KICK_N
#define M1_HFI_QKICK_KICK_N             6000u
#undef M1_HFI_QKICK_BRAKE_ENABLE
#define M1_HFI_QKICK_BRAKE_ENABLE       1
#undef M1_HFI_QKICK_BRAKE_N
#define M1_HFI_QKICK_BRAKE_N            6000u
#undef M1_HFI_QKICK_HOLD_S
#define M1_HFI_QKICK_HOLD_S             0.4f
#undef M1_HFI_QKICK_SPEED_ENABLE
#define M1_HFI_QKICK_SPEED_ENABLE       0
#undef M1_HFI_QKICK_CRAWL_ENABLE
#define M1_HFI_QKICK_CRAWL_ENABLE       0
#undef M1_HFI_QKICK_START_ENABLE
#define M1_HFI_QKICK_START_ENABLE       0
#undef M1_HFI_ID_PI_OFF_ENABLE
#define M1_HFI_ID_PI_OFF_ENABLE         1
#undef M1_HFI_QKICK_FORCE_PI
#define M1_HFI_QKICK_FORCE_PI           0

#elif M1_HFI_GATE == 28
/* C4n：相�?C4k 唯一增量 = PRE 假锁门禁（QKICK_PRE_GATE）�?
 * 1802：PRE |ε|�?0° / x�?.038 仍定时踢 �?极性判决前提坏�?
 * 本枪：假锁翻 +π/2 逃逸；|ε|持续小才准踢；超时拒�?DONE�?
 * 不做失败重踢。回退 GATE=25→C4k�?�?7 冻结�?*/
#undef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE       0
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              1
#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             5.0f
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 0.0f
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               15.0f
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 0.0f
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               0.0f
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (-1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (-1.0f)
#undef M1_HFI_A_CMD
#define M1_HFI_A_CMD                    (0.10f)
#undef M1_HFI_PLL_INIT_OFF_RAD
#define M1_HFI_PLL_INIT_OFF_RAD         0.0f
#undef M1_HFI_INIT_FROM_ENC
#define M1_HFI_INIT_FROM_ENC            1
#undef M1_HFI_POLARITY_IPD_ENABLE
#define M1_HFI_POLARITY_IPD_ENABLE      0
#undef M1_HFI_QKICK_AFTER_LOCK_ENABLE
#define M1_HFI_QKICK_AFTER_LOCK_ENABLE  1
#undef M1_HFI_QKICK_BEFORE_HFI_ENABLE
#define M1_HFI_QKICK_BEFORE_HFI_ENABLE  0
#undef M1_HFI_QKICK_THEN_HFI_ENABLE
#define M1_HFI_QKICK_THEN_HFI_ENABLE    1
#undef M1_HFI_QKICK_PRE_S
#define M1_HFI_QKICK_PRE_S              2.0f
#undef M1_HFI_QKICK_IQ_A
#define M1_HFI_QKICK_IQ_A               1.6f
#undef M1_HFI_QKICK_KICK_N
#define M1_HFI_QKICK_KICK_N             6000u
#undef M1_HFI_QKICK_BRAKE_ENABLE
#define M1_HFI_QKICK_BRAKE_ENABLE       1
#undef M1_HFI_QKICK_BRAKE_N
#define M1_HFI_QKICK_BRAKE_N            6000u
#undef M1_HFI_QKICK_HOLD_S
#define M1_HFI_QKICK_HOLD_S             0.4f
#undef M1_HFI_QKICK_SPEED_ENABLE
#define M1_HFI_QKICK_SPEED_ENABLE       0
#undef M1_HFI_QKICK_CRAWL_ENABLE
#define M1_HFI_QKICK_CRAWL_ENABLE       0
#undef M1_HFI_QKICK_START_ENABLE
#define M1_HFI_QKICK_START_ENABLE       0
#undef M1_HFI_ID_PI_OFF_ENABLE
#define M1_HFI_ID_PI_OFF_ENABLE         1
#undef M1_HFI_QKICK_FORCE_PI
#define M1_HFI_QKICK_FORCE_PI           0
#undef M1_HFI_QKICK_PRE_GATE_ENABLE
#define M1_HFI_QKICK_PRE_GATE_ENABLE    1 /* 唯一增量：假锁门�?+ �?pre_ok �?*/

#elif M1_HFI_GATE == 29
/* C4o：相�?C4n 唯一增量 = �?QKICK_BRAKE�?
 * 1857：踢末对齐尚可，−Iq 刹车把转子转�?~90° �?θ̂ 冻住 �?err 被拉�?�?POST ~174°�?
 * 本枪踢完直接 Iq=0 �?LOG 滑行，不做反向回摆。回退 GATE=28→C4n�?�?8 冻结�?*/
#undef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE       0
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              1
#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             5.0f
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 0.0f
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               15.0f
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 0.0f
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               0.0f
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (-1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (-1.0f)
#undef M1_HFI_A_CMD
#define M1_HFI_A_CMD                    (0.10f)
#undef M1_HFI_PLL_INIT_OFF_RAD
#define M1_HFI_PLL_INIT_OFF_RAD         0.0f
#undef M1_HFI_INIT_FROM_ENC
#define M1_HFI_INIT_FROM_ENC            1
#undef M1_HFI_POLARITY_IPD_ENABLE
#define M1_HFI_POLARITY_IPD_ENABLE      0
#undef M1_HFI_QKICK_AFTER_LOCK_ENABLE
#define M1_HFI_QKICK_AFTER_LOCK_ENABLE  1
#undef M1_HFI_QKICK_BEFORE_HFI_ENABLE
#define M1_HFI_QKICK_BEFORE_HFI_ENABLE  0
#undef M1_HFI_QKICK_THEN_HFI_ENABLE
#define M1_HFI_QKICK_THEN_HFI_ENABLE    1
#undef M1_HFI_QKICK_PRE_S
#define M1_HFI_QKICK_PRE_S              2.0f
#undef M1_HFI_QKICK_IQ_A
#define M1_HFI_QKICK_IQ_A               1.6f
#undef M1_HFI_QKICK_KICK_N
#define M1_HFI_QKICK_KICK_N             6000u
#undef M1_HFI_QKICK_BRAKE_ENABLE
#define M1_HFI_QKICK_BRAKE_ENABLE       0 /* 唯一增量：踢完滑行，�?−Iq 回摆 */
#undef M1_HFI_QKICK_BRAKE_N
#define M1_HFI_QKICK_BRAKE_N            6000u
#undef M1_HFI_QKICK_HOLD_S
#define M1_HFI_QKICK_HOLD_S             0.4f
#undef M1_HFI_QKICK_SPEED_ENABLE
#define M1_HFI_QKICK_SPEED_ENABLE       0
#undef M1_HFI_QKICK_CRAWL_ENABLE
#define M1_HFI_QKICK_CRAWL_ENABLE       0
#undef M1_HFI_QKICK_START_ENABLE
#define M1_HFI_QKICK_START_ENABLE       0
#undef M1_HFI_ID_PI_OFF_ENABLE
#define M1_HFI_ID_PI_OFF_ENABLE         1
#undef M1_HFI_QKICK_FORCE_PI
#define M1_HFI_QKICK_FORCE_PI           0
#undef M1_HFI_QKICK_PRE_GATE_ENABLE
#define M1_HFI_QKICK_PRE_GATE_ENABLE    1

#elif M1_HFI_GATE == 30
/* C4p：相�?C4o 唯一增量 = 过线�?FEED_A=0.5（HI=0，不走权威天花板出力）�?
 * 捕获期与 C4o 同；踢完 LOG→RUN �?qual_ok 再馈 0.5 A（电流环下沿 / 静摩擦档）�?
 * 1730：勿�?HI。回退 GATE=29�?�?9 冻结�?*/
#undef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE       0
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              1
#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             5.0f
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 0.0f
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               15.0f
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 0.0f
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               0.0f
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (-1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (-1.0f)
#undef M1_HFI_A_CMD
#define M1_HFI_A_CMD                    (0.10f)
#undef M1_HFI_PLL_INIT_OFF_RAD
#define M1_HFI_PLL_INIT_OFF_RAD         0.0f
#undef M1_HFI_INIT_FROM_ENC
#define M1_HFI_INIT_FROM_ENC            1
#undef M1_HFI_POLARITY_IPD_ENABLE
#define M1_HFI_POLARITY_IPD_ENABLE      0
#undef M1_HFI_QKICK_AFTER_LOCK_ENABLE
#define M1_HFI_QKICK_AFTER_LOCK_ENABLE  1
#undef M1_HFI_QKICK_BEFORE_HFI_ENABLE
#define M1_HFI_QKICK_BEFORE_HFI_ENABLE  0
#undef M1_HFI_QKICK_THEN_HFI_ENABLE
#define M1_HFI_QKICK_THEN_HFI_ENABLE    1
#undef M1_HFI_QKICK_PRE_S
#define M1_HFI_QKICK_PRE_S              2.0f
#undef M1_HFI_QKICK_IQ_A
#define M1_HFI_QKICK_IQ_A               1.6f
#undef M1_HFI_QKICK_KICK_N
#define M1_HFI_QKICK_KICK_N             6000u
#undef M1_HFI_QKICK_BRAKE_ENABLE
#define M1_HFI_QKICK_BRAKE_ENABLE       0
#undef M1_HFI_QKICK_BRAKE_N
#define M1_HFI_QKICK_BRAKE_N            6000u
#undef M1_HFI_QKICK_HOLD_S
#define M1_HFI_QKICK_HOLD_S             0.4f
#undef M1_HFI_QKICK_SPEED_ENABLE
#define M1_HFI_QKICK_SPEED_ENABLE       0
#undef M1_HFI_QKICK_CRAWL_ENABLE
#define M1_HFI_QKICK_CRAWL_ENABLE       0
#undef M1_HFI_QKICK_START_ENABLE
#define M1_HFI_QKICK_START_ENABLE       0
#undef M1_HFI_ID_PI_OFF_ENABLE
#define M1_HFI_ID_PI_OFF_ENABLE         1
#undef M1_HFI_QKICK_FORCE_PI
#define M1_HFI_QKICK_FORCE_PI           0
#undef M1_HFI_QKICK_PRE_GATE_ENABLE
#define M1_HFI_QKICK_PRE_GATE_ENABLE    1
#undef M1_HFI_IQ_AUTH_ENABLE
#define M1_HFI_IQ_AUTH_ENABLE           1
#undef M1_HFI_IQ_AUTH_X_GOOD
#define M1_HFI_IQ_AUTH_X_GOOD           (0.218f)
#undef M1_HFI_IQ_AUTH_X_BAD
#define M1_HFI_IQ_AUTH_X_BAD            (0.205f)
#undef M1_HFI_IQ_AUTH_EPS_FALSE_MAX
#define M1_HFI_IQ_AUTH_EPS_FALSE_MAX    (0.20f)
#undef M1_HFI_IQ_AUTH_HOLD_N
#define M1_HFI_IQ_AUTH_HOLD_N           2000u
#undef M1_HFI_IQ_AUTH_CLEAR_N
#define M1_HFI_IQ_AUTH_CLEAR_N          1000u
#undef M1_HFI_IQ_AUTH_IQ_LO
#define M1_HFI_IQ_AUTH_IQ_LO            (0.0f)
#undef M1_HFI_IQ_AUTH_IQ_HI
#define M1_HFI_IQ_AUTH_IQ_HI            (0.0f) /* 捕获期天花板保持 0；出力走 FEED_A */
#undef M1_HFI_IQ_AUTH_SLEW_A_S
#define M1_HFI_IQ_AUTH_SLEW_A_S         (10.0f)
#undef M1_HFI_IQ_AUTH_FEED_ENABLE
#define M1_HFI_IQ_AUTH_FEED_ENABLE      1
#undef M1_HFI_IQ_AUTH_FEED_SIGN
#define M1_HFI_IQ_AUTH_FEED_SIGN        (1.0f)
#undef M1_HFI_IQ_AUTH_FEED_A
#define M1_HFI_IQ_AUTH_FEED_A           (0.50f) /* 唯一增量：过线后�?0.5 A */
#undef M1_HFI_IQ_AUTH_FEED_DELAY_S
#define M1_HFI_IQ_AUTH_FEED_DELAY_S     (0.20f)
#undef M1_HFI_IQ_AUTH_FEED_RAMP_S
#define M1_HFI_IQ_AUTH_FEED_RAMP_S      (0.0f)
#undef M1_HFI_IQ_AUTH_FEED_LEGACY_BRANCH
#define M1_HFI_IQ_AUTH_FEED_LEGACY_BRANCH 0

#elif M1_HFI_GATE == 31
/* C4q：相�?C4p 唯一增量 = FEED_A 0.5�?.8�?
 * 1945/46�?.5 A 角准但不转（静摩擦）；本枪探能否持续滑动。回退 GATE=30�?�?0 冻结�?*/
#undef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE       0
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              1
#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             5.0f
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 0.0f
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               15.0f
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 0.0f
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               0.0f
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (-1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (-1.0f)
#undef M1_HFI_A_CMD
#define M1_HFI_A_CMD                    (0.10f)
#undef M1_HFI_PLL_INIT_OFF_RAD
#define M1_HFI_PLL_INIT_OFF_RAD         0.0f
#undef M1_HFI_INIT_FROM_ENC
#define M1_HFI_INIT_FROM_ENC            1
#undef M1_HFI_POLARITY_IPD_ENABLE
#define M1_HFI_POLARITY_IPD_ENABLE      0
#undef M1_HFI_QKICK_AFTER_LOCK_ENABLE
#define M1_HFI_QKICK_AFTER_LOCK_ENABLE  1
#undef M1_HFI_QKICK_BEFORE_HFI_ENABLE
#define M1_HFI_QKICK_BEFORE_HFI_ENABLE  0
#undef M1_HFI_QKICK_THEN_HFI_ENABLE
#define M1_HFI_QKICK_THEN_HFI_ENABLE    1
#undef M1_HFI_QKICK_PRE_S
#define M1_HFI_QKICK_PRE_S              2.0f
#undef M1_HFI_QKICK_IQ_A
#define M1_HFI_QKICK_IQ_A               1.6f
#undef M1_HFI_QKICK_KICK_N
#define M1_HFI_QKICK_KICK_N             6000u
#undef M1_HFI_QKICK_BRAKE_ENABLE
#define M1_HFI_QKICK_BRAKE_ENABLE       0
#undef M1_HFI_QKICK_BRAKE_N
#define M1_HFI_QKICK_BRAKE_N            6000u
#undef M1_HFI_QKICK_HOLD_S
#define M1_HFI_QKICK_HOLD_S             0.4f
#undef M1_HFI_QKICK_SPEED_ENABLE
#define M1_HFI_QKICK_SPEED_ENABLE       0
#undef M1_HFI_QKICK_CRAWL_ENABLE
#define M1_HFI_QKICK_CRAWL_ENABLE       0
#undef M1_HFI_QKICK_START_ENABLE
#define M1_HFI_QKICK_START_ENABLE       0
#undef M1_HFI_ID_PI_OFF_ENABLE
#define M1_HFI_ID_PI_OFF_ENABLE         1
#undef M1_HFI_QKICK_FORCE_PI
#define M1_HFI_QKICK_FORCE_PI           0
#undef M1_HFI_QKICK_PRE_GATE_ENABLE
#define M1_HFI_QKICK_PRE_GATE_ENABLE    1
#undef M1_HFI_IQ_AUTH_ENABLE
#define M1_HFI_IQ_AUTH_ENABLE           1
#undef M1_HFI_IQ_AUTH_X_GOOD
#define M1_HFI_IQ_AUTH_X_GOOD           (0.218f)
#undef M1_HFI_IQ_AUTH_X_BAD
#define M1_HFI_IQ_AUTH_X_BAD            (0.205f)
#undef M1_HFI_IQ_AUTH_EPS_FALSE_MAX
#define M1_HFI_IQ_AUTH_EPS_FALSE_MAX    (0.20f)
#undef M1_HFI_IQ_AUTH_HOLD_N
#define M1_HFI_IQ_AUTH_HOLD_N           2000u
#undef M1_HFI_IQ_AUTH_CLEAR_N
#define M1_HFI_IQ_AUTH_CLEAR_N          1000u
#undef M1_HFI_IQ_AUTH_IQ_LO
#define M1_HFI_IQ_AUTH_IQ_LO            (0.0f)
#undef M1_HFI_IQ_AUTH_IQ_HI
#define M1_HFI_IQ_AUTH_IQ_HI            (0.0f)
#undef M1_HFI_IQ_AUTH_SLEW_A_S
#define M1_HFI_IQ_AUTH_SLEW_A_S         (10.0f)
#undef M1_HFI_IQ_AUTH_FEED_ENABLE
#define M1_HFI_IQ_AUTH_FEED_ENABLE      1
#undef M1_HFI_IQ_AUTH_FEED_SIGN
#define M1_HFI_IQ_AUTH_FEED_SIGN        (1.0f)
#undef M1_HFI_IQ_AUTH_FEED_A
#define M1_HFI_IQ_AUTH_FEED_A           (0.80f) /* 唯一增量�?.5�?.8 A */
#undef M1_HFI_IQ_AUTH_FEED_DELAY_S
#define M1_HFI_IQ_AUTH_FEED_DELAY_S     (0.20f)
#undef M1_HFI_IQ_AUTH_FEED_RAMP_S
#define M1_HFI_IQ_AUTH_FEED_RAMP_S      (0.0f)
#undef M1_HFI_IQ_AUTH_FEED_LEGACY_BRANCH
#define M1_HFI_IQ_AUTH_FEED_LEGACY_BRANCH 0

#elif M1_HFI_GATE == 32
/* C4r：捕获同 C4q（PRE_GATE、关 BRAKE、THEN_HFI、ID_PI_OFF）�?
 * 唯一增量：踢�?RUN 开现有速度�?+ RUN_LADDER 100�?000 rpm / 100 / 2.5 s�?
 * �?FEED（Iq 交速度 PI）；ω_ff=0；速反馈用 HFI ω（SPEED_FB）�?
 * AUTH 沿用 S3c0b 天花板，防未跟住时顶 11 A。回退 GATE=31�?�?1 冻结�?*/
#undef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE       0
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              1
#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             5.0f
#undef M1_HFI_RUN_LADDER_ENABLE
#define M1_HFI_RUN_LADDER_ENABLE        1
#undef M1_HFI_RUN_RPM_START
#define M1_HFI_RUN_RPM_START            100.0f
#undef M1_HFI_RUN_RPM_STEP
#define M1_HFI_RUN_RPM_STEP             100.0f
#undef M1_HFI_RUN_RPM_MAX
#define M1_HFI_RUN_RPM_MAX              1000.0f
#undef M1_HFI_RUN_STEP_S
#define M1_HFI_RUN_STEP_S               2.5f
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 0.0f
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               2.5f
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 0.0f
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               0.0f
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (-1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (-1.0f)
#undef M1_HFI_A_CMD
#define M1_HFI_A_CMD                    (0.10f)
#undef M1_HFI_PLL_INIT_OFF_RAD
#define M1_HFI_PLL_INIT_OFF_RAD         0.0f
#undef M1_HFI_INIT_FROM_ENC
#define M1_HFI_INIT_FROM_ENC            1
#undef M1_HFI_POLARITY_IPD_ENABLE
#define M1_HFI_POLARITY_IPD_ENABLE      0
#undef M1_HFI_OMEGA_FF_SRC
#define M1_HFI_OMEGA_FF_SRC             0
#undef M1_HFI_OMEGA_FF_FROM_REF
#define M1_HFI_OMEGA_FF_FROM_REF        0
#undef M1_HFI_SPEED_FB_ENABLE
#define M1_HFI_SPEED_FB_ENABLE          1 /* 速度段吃 HFI ω，不�?enc PLL */
#undef M1_HFI_QKICK_AFTER_LOCK_ENABLE
#define M1_HFI_QKICK_AFTER_LOCK_ENABLE  1
#undef M1_HFI_QKICK_BEFORE_HFI_ENABLE
#define M1_HFI_QKICK_BEFORE_HFI_ENABLE  0
#undef M1_HFI_QKICK_THEN_HFI_ENABLE
#define M1_HFI_QKICK_THEN_HFI_ENABLE    1
#undef M1_HFI_QKICK_SPEED_ENABLE
#define M1_HFI_QKICK_SPEED_ENABLE       0 /* �?THEN_HFI 互斥；速度�?LADDER �?*/
#undef M1_HFI_QKICK_PRE_S
#define M1_HFI_QKICK_PRE_S              2.0f
#undef M1_HFI_QKICK_IQ_A
#define M1_HFI_QKICK_IQ_A               1.6f
#undef M1_HFI_QKICK_KICK_N
#define M1_HFI_QKICK_KICK_N             6000u
#undef M1_HFI_QKICK_BRAKE_ENABLE
#define M1_HFI_QKICK_BRAKE_ENABLE       0
#undef M1_HFI_QKICK_BRAKE_N
#define M1_HFI_QKICK_BRAKE_N            6000u
#undef M1_HFI_QKICK_HOLD_S
#define M1_HFI_QKICK_HOLD_S             0.4f
#undef M1_HFI_QKICK_CRAWL_ENABLE
#define M1_HFI_QKICK_CRAWL_ENABLE       0
#undef M1_HFI_QKICK_START_ENABLE
#define M1_HFI_QKICK_START_ENABLE       0
#undef M1_HFI_ID_PI_OFF_ENABLE
#define M1_HFI_ID_PI_OFF_ENABLE         1
#undef M1_HFI_QKICK_FORCE_PI
#define M1_HFI_QKICK_FORCE_PI           0
#undef M1_HFI_QKICK_PRE_GATE_ENABLE
#define M1_HFI_QKICK_PRE_GATE_ENABLE    1
#undef M1_HFI_IQ_AUTH_ENABLE
#define M1_HFI_IQ_AUTH_ENABLE           1
#undef M1_HFI_IQ_AUTH_X_GOOD
#define M1_HFI_IQ_AUTH_X_GOOD           (0.218f)
#undef M1_HFI_IQ_AUTH_X_BAD
#define M1_HFI_IQ_AUTH_X_BAD            (0.205f)
#undef M1_HFI_IQ_AUTH_EPS_FALSE_MAX
#define M1_HFI_IQ_AUTH_EPS_FALSE_MAX    (0.20f)
#undef M1_HFI_IQ_AUTH_HOLD_N
#define M1_HFI_IQ_AUTH_HOLD_N           2000u
#undef M1_HFI_IQ_AUTH_CLEAR_N
#define M1_HFI_IQ_AUTH_CLEAR_N          1000u
#undef M1_HFI_IQ_AUTH_IQ_LO
#define M1_HFI_IQ_AUTH_IQ_LO            (0.25f)
#undef M1_HFI_IQ_AUTH_IQ_HI
#define M1_HFI_IQ_AUTH_IQ_HI            (3.50f) /* 沿用 S3c0b；捕获期速度环未开 */
#undef M1_HFI_IQ_AUTH_SLEW_A_S
#define M1_HFI_IQ_AUTH_SLEW_A_S         (4.0f)
#undef M1_HFI_IQ_AUTH_FEED_ENABLE
#define M1_HFI_IQ_AUTH_FEED_ENABLE      0 /* Iq 交速度�?*/

#elif M1_HFI_GATE == 33
/* C4s：相�?C4r 唯一增量 = AUTH IQ_HI 3.5�?1 A�?
 * 对齐 SPEED_IDENT �?M1_I_REF_ABS_MAX / SPEED_IQ（本工程 SMO 路径同顶 ~10�?2 A）�?
 * 2008�?.5 A 钉死 �?轴卡 ~273 rpm。回退 GATE=32�?�?2 冻结�?*/
#undef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE       0
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              1
#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             5.0f
#undef M1_HFI_RUN_LADDER_ENABLE
#define M1_HFI_RUN_LADDER_ENABLE        1
#undef M1_HFI_RUN_RPM_START
#define M1_HFI_RUN_RPM_START            100.0f
#undef M1_HFI_RUN_RPM_STEP
#define M1_HFI_RUN_RPM_STEP             100.0f
#undef M1_HFI_RUN_RPM_MAX
#define M1_HFI_RUN_RPM_MAX              1000.0f
#undef M1_HFI_RUN_STEP_S
#define M1_HFI_RUN_STEP_S               2.5f
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 0.0f
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               2.5f
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 0.0f
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               0.0f
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (-1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (-1.0f)
#undef M1_HFI_A_CMD
#define M1_HFI_A_CMD                    (0.10f)
#undef M1_HFI_PLL_INIT_OFF_RAD
#define M1_HFI_PLL_INIT_OFF_RAD         0.0f
#undef M1_HFI_INIT_FROM_ENC
#define M1_HFI_INIT_FROM_ENC            1
#undef M1_HFI_POLARITY_IPD_ENABLE
#define M1_HFI_POLARITY_IPD_ENABLE      0
#undef M1_HFI_OMEGA_FF_SRC
#define M1_HFI_OMEGA_FF_SRC             0
#undef M1_HFI_OMEGA_FF_FROM_REF
#define M1_HFI_OMEGA_FF_FROM_REF        0
#undef M1_HFI_SPEED_FB_ENABLE
#define M1_HFI_SPEED_FB_ENABLE          1
#undef M1_HFI_QKICK_AFTER_LOCK_ENABLE
#define M1_HFI_QKICK_AFTER_LOCK_ENABLE  1
#undef M1_HFI_QKICK_BEFORE_HFI_ENABLE
#define M1_HFI_QKICK_BEFORE_HFI_ENABLE  0
#undef M1_HFI_QKICK_THEN_HFI_ENABLE
#define M1_HFI_QKICK_THEN_HFI_ENABLE    1
#undef M1_HFI_QKICK_SPEED_ENABLE
#define M1_HFI_QKICK_SPEED_ENABLE       0
#undef M1_HFI_QKICK_PRE_S
#define M1_HFI_QKICK_PRE_S              2.0f
#undef M1_HFI_QKICK_IQ_A
#define M1_HFI_QKICK_IQ_A               1.6f
#undef M1_HFI_QKICK_KICK_N
#define M1_HFI_QKICK_KICK_N             6000u
#undef M1_HFI_QKICK_BRAKE_ENABLE
#define M1_HFI_QKICK_BRAKE_ENABLE       0
#undef M1_HFI_QKICK_BRAKE_N
#define M1_HFI_QKICK_BRAKE_N            6000u
#undef M1_HFI_QKICK_HOLD_S
#define M1_HFI_QKICK_HOLD_S             0.4f
#undef M1_HFI_QKICK_CRAWL_ENABLE
#define M1_HFI_QKICK_CRAWL_ENABLE       0
#undef M1_HFI_QKICK_START_ENABLE
#define M1_HFI_QKICK_START_ENABLE       0
#undef M1_HFI_ID_PI_OFF_ENABLE
#define M1_HFI_ID_PI_OFF_ENABLE         1
#undef M1_HFI_QKICK_FORCE_PI
#define M1_HFI_QKICK_FORCE_PI           0
#undef M1_HFI_QKICK_PRE_GATE_ENABLE
#define M1_HFI_QKICK_PRE_GATE_ENABLE    1
#undef M1_HFI_IQ_AUTH_ENABLE
#define M1_HFI_IQ_AUTH_ENABLE           1
#undef M1_HFI_IQ_AUTH_X_GOOD
#define M1_HFI_IQ_AUTH_X_GOOD           (0.218f)
#undef M1_HFI_IQ_AUTH_X_BAD
#define M1_HFI_IQ_AUTH_X_BAD            (0.205f)
#undef M1_HFI_IQ_AUTH_EPS_FALSE_MAX
#define M1_HFI_IQ_AUTH_EPS_FALSE_MAX    (0.20f)
#undef M1_HFI_IQ_AUTH_HOLD_N
#define M1_HFI_IQ_AUTH_HOLD_N           2000u
#undef M1_HFI_IQ_AUTH_CLEAR_N
#define M1_HFI_IQ_AUTH_CLEAR_N          1000u
#undef M1_HFI_IQ_AUTH_IQ_LO
#define M1_HFI_IQ_AUTH_IQ_LO            (0.25f)
#undef M1_HFI_IQ_AUTH_IQ_HI
#define M1_HFI_IQ_AUTH_IQ_HI            (11.0f) /* 唯一增量：对�?SMO/SPEED 电流�?*/
#undef M1_HFI_IQ_AUTH_SLEW_A_S
#define M1_HFI_IQ_AUTH_SLEW_A_S         (4.0f)
#undef M1_HFI_IQ_AUTH_FEED_ENABLE
#define M1_HFI_IQ_AUTH_FEED_ENABLE      0

#elif M1_HFI_GATE == 34
/* C4t：相�?C4s �?
 * 1) PLL_W_MAX/INT_MAX�?00�?100 rad/s 电角（~273→~1500 rpm 机）�?
 * 2) OMEGA_FF_SRC=3：�? 小权重（误路径；2028 空转）。冻结。回退 GATE=33�?*/
#undef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE       0
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              1
#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             5.0f
#undef M1_HFI_RUN_LADDER_ENABLE
#define M1_HFI_RUN_LADDER_ENABLE        1
#undef M1_HFI_RUN_RPM_START
#define M1_HFI_RUN_RPM_START            100.0f
#undef M1_HFI_RUN_RPM_STEP
#define M1_HFI_RUN_RPM_STEP             100.0f
#undef M1_HFI_RUN_RPM_MAX
#define M1_HFI_RUN_RPM_MAX              1000.0f
#undef M1_HFI_RUN_STEP_S
#define M1_HFI_RUN_STEP_S               2.5f
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 0.0f
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               2.5f
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 0.0f
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               0.0f
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (-1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (-1.0f)
#undef M1_HFI_A_CMD
#define M1_HFI_A_CMD                    (0.10f)
#undef M1_HFI_PLL_INIT_OFF_RAD
#define M1_HFI_PLL_INIT_OFF_RAD         0.0f
#undef M1_HFI_INIT_FROM_ENC
#define M1_HFI_INIT_FROM_ENC            1
#undef M1_HFI_POLARITY_IPD_ENABLE
#define M1_HFI_POLARITY_IPD_ENABLE      0
#undef M1_HFI_PLL_W_MAX
#define M1_HFI_PLL_W_MAX                1100.0f /* ~1500 rpm 机；阶梯 1000 留裕�?*/
#undef M1_HFI_PLL_INT_MAX
#define M1_HFI_PLL_INT_MAX              M1_HFI_PLL_W_MAX
#undef M1_HFI_OMEGA_FF_SRC
#define M1_HFI_OMEGA_FF_SRC             3 /* ω* 小权重；上层�?rpm_cmd */
#undef M1_HFI_OMEGA_FF_REF_W
#define M1_HFI_OMEGA_FF_REF_W           0.15f
#undef M1_HFI_OMEGA_FF_FROM_REF
#define M1_HFI_OMEGA_FF_FROM_REF        0 /* SRC=3 已走 ω_cmd，勿开 enc �?*/
#undef M1_HFI_OMEGA_SEED_ENABLE
#define M1_HFI_OMEGA_SEED_ENABLE        1 /* �?RUN �?ω* 播种 pll_int */
#undef M1_HFI_SPEED_FB_ENABLE
#define M1_HFI_SPEED_FB_ENABLE          1
#undef M1_HFI_QKICK_AFTER_LOCK_ENABLE
#define M1_HFI_QKICK_AFTER_LOCK_ENABLE  1
#undef M1_HFI_QKICK_BEFORE_HFI_ENABLE
#define M1_HFI_QKICK_BEFORE_HFI_ENABLE  0
#undef M1_HFI_QKICK_THEN_HFI_ENABLE
#define M1_HFI_QKICK_THEN_HFI_ENABLE    1
#undef M1_HFI_QKICK_SPEED_ENABLE
#define M1_HFI_QKICK_SPEED_ENABLE       0
#undef M1_HFI_QKICK_PRE_S
#define M1_HFI_QKICK_PRE_S              2.0f
#undef M1_HFI_QKICK_IQ_A
#define M1_HFI_QKICK_IQ_A               1.6f
#undef M1_HFI_QKICK_KICK_N
#define M1_HFI_QKICK_KICK_N             6000u
#undef M1_HFI_QKICK_BRAKE_ENABLE
#define M1_HFI_QKICK_BRAKE_ENABLE       0
#undef M1_HFI_QKICK_BRAKE_N
#define M1_HFI_QKICK_BRAKE_N            6000u
#undef M1_HFI_QKICK_HOLD_S
#define M1_HFI_QKICK_HOLD_S             0.4f
#undef M1_HFI_QKICK_CRAWL_ENABLE
#define M1_HFI_QKICK_CRAWL_ENABLE       0
#undef M1_HFI_QKICK_START_ENABLE
#define M1_HFI_QKICK_START_ENABLE       0
#undef M1_HFI_ID_PI_OFF_ENABLE
#define M1_HFI_ID_PI_OFF_ENABLE         1
#undef M1_HFI_QKICK_FORCE_PI
#define M1_HFI_QKICK_FORCE_PI           0
#undef M1_HFI_QKICK_PRE_GATE_ENABLE
#define M1_HFI_QKICK_PRE_GATE_ENABLE    1
#undef M1_HFI_IQ_AUTH_ENABLE
#define M1_HFI_IQ_AUTH_ENABLE           1
#undef M1_HFI_IQ_AUTH_X_GOOD
#define M1_HFI_IQ_AUTH_X_GOOD           (0.218f)
#undef M1_HFI_IQ_AUTH_X_BAD
#define M1_HFI_IQ_AUTH_X_BAD            (0.205f)
#undef M1_HFI_IQ_AUTH_EPS_FALSE_MAX
#define M1_HFI_IQ_AUTH_EPS_FALSE_MAX    (0.20f)
#undef M1_HFI_IQ_AUTH_HOLD_N
#define M1_HFI_IQ_AUTH_HOLD_N           2000u
#undef M1_HFI_IQ_AUTH_CLEAR_N
#define M1_HFI_IQ_AUTH_CLEAR_N          1000u
#undef M1_HFI_IQ_AUTH_IQ_LO
#define M1_HFI_IQ_AUTH_IQ_LO            (0.25f)
#undef M1_HFI_IQ_AUTH_IQ_HI
#define M1_HFI_IQ_AUTH_IQ_HI            (11.0f)
#undef M1_HFI_IQ_AUTH_SLEW_A_S
#define M1_HFI_IQ_AUTH_SLEW_A_S         (4.0f)
#undef M1_HFI_IQ_AUTH_FEED_ENABLE
#define M1_HFI_IQ_AUTH_FEED_ENABLE      0

#elif M1_HFI_GATE == 35
/* C4u：相�?C4t 唯一增量 = 关指令前馈�?
 * OMEGA_FF_SRC=3�?；OMEGA_SEED=1�?。保�?PLL_W_MAX=1100�?
 * 2028：�? 前馈空转 θ̂ + SPEED_FB=HFI 假闭环。回退 GATE=34（或 33）�?�?4 冻结�?*/
#undef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE       0
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              1
#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             5.0f
#undef M1_HFI_RUN_LADDER_ENABLE
#define M1_HFI_RUN_LADDER_ENABLE        1
#undef M1_HFI_RUN_RPM_START
#define M1_HFI_RUN_RPM_START            100.0f
#undef M1_HFI_RUN_RPM_STEP
#define M1_HFI_RUN_RPM_STEP             100.0f
#undef M1_HFI_RUN_RPM_MAX
#define M1_HFI_RUN_RPM_MAX              1000.0f
#undef M1_HFI_RUN_STEP_S
#define M1_HFI_RUN_STEP_S               2.5f
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 0.0f
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               2.5f
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 0.0f
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               0.0f
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (-1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (-1.0f)
#undef M1_HFI_A_CMD
#define M1_HFI_A_CMD                    (0.10f)
#undef M1_HFI_PLL_INIT_OFF_RAD
#define M1_HFI_PLL_INIT_OFF_RAD         0.0f
#undef M1_HFI_INIT_FROM_ENC
#define M1_HFI_INIT_FROM_ENC            1
#undef M1_HFI_POLARITY_IPD_ENABLE
#define M1_HFI_POLARITY_IPD_ENABLE      0
#undef M1_HFI_PLL_W_MAX
#define M1_HFI_PLL_W_MAX                1100.0f /* �?C4t；破 ~273 rpm �?*/
#undef M1_HFI_PLL_INT_MAX
#define M1_HFI_PLL_INT_MAX              M1_HFI_PLL_W_MAX
#undef M1_HFI_OMEGA_FF_SRC
#define M1_HFI_OMEGA_FF_SRC             0 /* �?ω*；纯 Type-II */
#undef M1_HFI_OMEGA_FF_FROM_REF
#define M1_HFI_OMEGA_FF_FROM_REF        0
#undef M1_HFI_OMEGA_SEED_ENABLE
#define M1_HFI_OMEGA_SEED_ENABLE        0 /* �?ω* 播种 */
#undef M1_HFI_SPEED_FB_ENABLE
#define M1_HFI_SPEED_FB_ENABLE          1
#undef M1_HFI_QKICK_AFTER_LOCK_ENABLE
#define M1_HFI_QKICK_AFTER_LOCK_ENABLE  1
#undef M1_HFI_QKICK_BEFORE_HFI_ENABLE
#define M1_HFI_QKICK_BEFORE_HFI_ENABLE  0
#undef M1_HFI_QKICK_THEN_HFI_ENABLE
#define M1_HFI_QKICK_THEN_HFI_ENABLE    1
#undef M1_HFI_QKICK_SPEED_ENABLE
#define M1_HFI_QKICK_SPEED_ENABLE       0
#undef M1_HFI_QKICK_PRE_S
#define M1_HFI_QKICK_PRE_S              2.0f
#undef M1_HFI_QKICK_IQ_A
#define M1_HFI_QKICK_IQ_A               1.6f
#undef M1_HFI_QKICK_KICK_N
#define M1_HFI_QKICK_KICK_N             6000u
#undef M1_HFI_QKICK_BRAKE_ENABLE
#define M1_HFI_QKICK_BRAKE_ENABLE       0
#undef M1_HFI_QKICK_BRAKE_N
#define M1_HFI_QKICK_BRAKE_N            6000u
#undef M1_HFI_QKICK_HOLD_S
#define M1_HFI_QKICK_HOLD_S             0.4f
#undef M1_HFI_QKICK_CRAWL_ENABLE
#define M1_HFI_QKICK_CRAWL_ENABLE       0
#undef M1_HFI_QKICK_START_ENABLE
#define M1_HFI_QKICK_START_ENABLE       0
#undef M1_HFI_ID_PI_OFF_ENABLE
#define M1_HFI_ID_PI_OFF_ENABLE         1
#undef M1_HFI_QKICK_FORCE_PI
#define M1_HFI_QKICK_FORCE_PI           0
#undef M1_HFI_QKICK_PRE_GATE_ENABLE
#define M1_HFI_QKICK_PRE_GATE_ENABLE    1
#undef M1_HFI_IQ_AUTH_ENABLE
#define M1_HFI_IQ_AUTH_ENABLE           1
#undef M1_HFI_IQ_AUTH_X_GOOD
#define M1_HFI_IQ_AUTH_X_GOOD           (0.218f)
#undef M1_HFI_IQ_AUTH_X_BAD
#define M1_HFI_IQ_AUTH_X_BAD            (0.205f)
#undef M1_HFI_IQ_AUTH_EPS_FALSE_MAX
#define M1_HFI_IQ_AUTH_EPS_FALSE_MAX    (0.20f)
#undef M1_HFI_IQ_AUTH_HOLD_N
#define M1_HFI_IQ_AUTH_HOLD_N           2000u
#undef M1_HFI_IQ_AUTH_CLEAR_N
#define M1_HFI_IQ_AUTH_CLEAR_N          1000u
#undef M1_HFI_IQ_AUTH_IQ_LO
#define M1_HFI_IQ_AUTH_IQ_LO            (0.25f)
#undef M1_HFI_IQ_AUTH_IQ_HI
#define M1_HFI_IQ_AUTH_IQ_HI            (11.0f)
#undef M1_HFI_IQ_AUTH_SLEW_A_S
#define M1_HFI_IQ_AUTH_SLEW_A_S         (4.0f)
#undef M1_HFI_IQ_AUTH_FEED_ENABLE
#define M1_HFI_IQ_AUTH_FEED_ENABLE      0

#elif (M1_HFI_GATE == 36) || (M1_HFI_GATE == 37)
/* C4v(36)：相�?C4u 唯一增量 = RUN 100�?00 / 10 s�?
 * C4w(37)：控�?�?C4v；唯一增量 = 巡航 VOFA（debug 侧）�?
 * AUTH/W_MAX/�?ω* �?C4u。回退 36 �?35�?�?5 冻结�?*/
#undef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE       0
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              1
#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             5.0f
#undef M1_HFI_RUN_LADDER_ENABLE
#define M1_HFI_RUN_LADDER_ENABLE        1
#undef M1_HFI_RUN_RPM_START
#define M1_HFI_RUN_RPM_START            100.0f
#undef M1_HFI_RUN_RPM_STEP
#define M1_HFI_RUN_RPM_STEP             100.0f
#undef M1_HFI_RUN_RPM_MAX
#define M1_HFI_RUN_RPM_MAX              200.0f /* 只巡�?100�?00 */
#undef M1_HFI_RUN_STEP_S
#define M1_HFI_RUN_STEP_S               10.0f /* 每档 10 s */
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 0.0f
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               2.5f
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 0.0f
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               0.0f
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (-1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (-1.0f)
#undef M1_HFI_A_CMD
#define M1_HFI_A_CMD                    (0.10f)
#undef M1_HFI_PLL_INIT_OFF_RAD
#define M1_HFI_PLL_INIT_OFF_RAD         0.0f
#undef M1_HFI_INIT_FROM_ENC
#define M1_HFI_INIT_FROM_ENC            1
#undef M1_HFI_POLARITY_IPD_ENABLE
#define M1_HFI_POLARITY_IPD_ENABLE      0
#undef M1_HFI_PLL_W_MAX
#define M1_HFI_PLL_W_MAX                1100.0f
#undef M1_HFI_PLL_INT_MAX
#define M1_HFI_PLL_INT_MAX              M1_HFI_PLL_W_MAX
#undef M1_HFI_OMEGA_FF_SRC
#define M1_HFI_OMEGA_FF_SRC             0
#undef M1_HFI_OMEGA_FF_FROM_REF
#define M1_HFI_OMEGA_FF_FROM_REF        0
#undef M1_HFI_OMEGA_SEED_ENABLE
#define M1_HFI_OMEGA_SEED_ENABLE        0
#undef M1_HFI_SPEED_FB_ENABLE
#define M1_HFI_SPEED_FB_ENABLE          1
#undef M1_HFI_QKICK_AFTER_LOCK_ENABLE
#define M1_HFI_QKICK_AFTER_LOCK_ENABLE  1
#undef M1_HFI_QKICK_BEFORE_HFI_ENABLE
#define M1_HFI_QKICK_BEFORE_HFI_ENABLE  0
#undef M1_HFI_QKICK_THEN_HFI_ENABLE
#define M1_HFI_QKICK_THEN_HFI_ENABLE    1
#undef M1_HFI_QKICK_SPEED_ENABLE
#define M1_HFI_QKICK_SPEED_ENABLE       0
#undef M1_HFI_QKICK_PRE_S
#define M1_HFI_QKICK_PRE_S              2.0f
#undef M1_HFI_QKICK_IQ_A
#define M1_HFI_QKICK_IQ_A               1.6f
#undef M1_HFI_QKICK_KICK_N
#define M1_HFI_QKICK_KICK_N             6000u
#undef M1_HFI_QKICK_BRAKE_ENABLE
#define M1_HFI_QKICK_BRAKE_ENABLE       0
#undef M1_HFI_QKICK_BRAKE_N
#define M1_HFI_QKICK_BRAKE_N            6000u
#undef M1_HFI_QKICK_HOLD_S
#define M1_HFI_QKICK_HOLD_S             0.4f
#undef M1_HFI_QKICK_CRAWL_ENABLE
#define M1_HFI_QKICK_CRAWL_ENABLE       0
#undef M1_HFI_QKICK_START_ENABLE
#define M1_HFI_QKICK_START_ENABLE       0
#undef M1_HFI_ID_PI_OFF_ENABLE
#define M1_HFI_ID_PI_OFF_ENABLE         1
#undef M1_HFI_QKICK_FORCE_PI
#define M1_HFI_QKICK_FORCE_PI           0
#undef M1_HFI_QKICK_PRE_GATE_ENABLE
#define M1_HFI_QKICK_PRE_GATE_ENABLE    1
#undef M1_HFI_IQ_AUTH_ENABLE
#define M1_HFI_IQ_AUTH_ENABLE           1
#undef M1_HFI_IQ_AUTH_X_GOOD
#define M1_HFI_IQ_AUTH_X_GOOD           (0.218f)
#undef M1_HFI_IQ_AUTH_X_BAD
#define M1_HFI_IQ_AUTH_X_BAD            (0.205f)
#undef M1_HFI_IQ_AUTH_EPS_FALSE_MAX
#define M1_HFI_IQ_AUTH_EPS_FALSE_MAX    (0.20f)
#undef M1_HFI_IQ_AUTH_HOLD_N
#define M1_HFI_IQ_AUTH_HOLD_N           2000u
#undef M1_HFI_IQ_AUTH_CLEAR_N
#define M1_HFI_IQ_AUTH_CLEAR_N          1000u
#undef M1_HFI_IQ_AUTH_IQ_LO
#define M1_HFI_IQ_AUTH_IQ_LO            (0.25f)
#undef M1_HFI_IQ_AUTH_IQ_HI
#define M1_HFI_IQ_AUTH_IQ_HI            (11.0f) /* 不降；看真实电流/角漂 */
#undef M1_HFI_IQ_AUTH_SLEW_A_S
#define M1_HFI_IQ_AUTH_SLEW_A_S         (4.0f)
#undef M1_HFI_IQ_AUTH_FEED_ENABLE
#define M1_HFI_IQ_AUTH_FEED_ENABLE      0

#elif (M1_HFI_GATE == 38) || (M1_HFI_GATE == 41) || (M1_HFI_GATE == 42) || \
      (M1_HFI_GATE == 43) || (M1_HFI_GATE == 53) || (M1_HFI_GATE == 54) || \
      (M1_HFI_GATE == 55) || (M1_HFI_GATE == 56) || (M1_HFI_GATE == 57) || \
      (M1_HFI_GATE == 58) || (M1_HFI_GATE == 59) || (M1_HFI_GATE == 60) || \
      (M1_HFI_GATE == 61) || (M1_HFI_GATE == 62) || (M1_HFI_GATE == 63) || \
      (M1_HFI_GATE == 64) || (M1_HFI_GATE == 65) || (M1_HFI_GATE == 66) || (M1_HFI_GATE == 67) || (M1_HFI_GATE == 68) || (M1_HFI_GATE == 69) || (M1_HFI_GATE == 70) || (M1_HFI_GATE == 71) || (M1_HFI_GATE == 72) || (M1_HFI_GATE == 73) || (M1_HFI_GATE == 74) || (M1_HFI_GATE == 75) || (M1_HFI_GATE == 76) || (M1_HFI_GATE == 77) || (M1_HFI_GATE == 78) || (M1_HFI_GATE == 79) || (M1_HFI_GATE == 80) || (M1_HFI_GATE == 91) || (M1_HFI_GATE == 92) || (M1_HFI_GATE == 93) || (M1_HFI_GATE == 81) || (M1_HFI_GATE == 82) || (M1_HFI_GATE == 83) || (M1_HFI_GATE == 84) || (M1_HFI_GATE == 85) || (M1_HFI_GATE == 86) || (M1_HFI_GATE == 87) || (M1_HFI_GATE == 88) || (M1_HFI_GATE == 89) || (M1_HFI_GATE == 90)
/* C4x(38)：踢后速度�?0�?800 rpm @ 50 rpm/s，探高速。电流上�?11 A。FEED 关�?
 * 本轮不从编码器播种，拔线�?θ̂ �?0 起由凸极锁轴，速度反馈与扇区重构也不吃编码器�?
 * 55：残 Vh�?6/57：灭 Vh�?8：残 Vh 下开 Id�?2：开 Id �?W_HOLD�?*/
#undef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE       0
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              1
#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             5.0f
#undef M1_HFI_RUN_LADDER_ENABLE
#define M1_HFI_RUN_LADDER_ENABLE        0 /* 关速度阶梯 */
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 0.0f
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               10.0f /* FEED 观察�?10 s */
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 0.0f
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               0.0f
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (-1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (-1.0f)
#undef M1_HFI_A_CMD
#define M1_HFI_A_CMD                    (0.10f)
#undef M1_HFI_PLL_INIT_OFF_RAD
#define M1_HFI_PLL_INIT_OFF_RAD         0.0f
#undef M1_HFI_INIT_FROM_ENC
#define M1_HFI_INIT_FROM_ENC            1
#undef M1_HFI_POLARITY_IPD_ENABLE
#define M1_HFI_POLARITY_IPD_ENABLE      0
#undef M1_HFI_PLL_W_MAX
#define M1_HFI_PLL_W_MAX                1100.0f /* �?C4w；勿�?FEED 混收 */
#undef M1_HFI_PLL_INT_MAX
#define M1_HFI_PLL_INT_MAX              M1_HFI_PLL_W_MAX
#undef M1_HFI_OMEGA_FF_SRC
#define M1_HFI_OMEGA_FF_SRC             0
#undef M1_HFI_OMEGA_FF_FROM_REF
#define M1_HFI_OMEGA_FF_FROM_REF        0
#undef M1_HFI_OMEGA_SEED_ENABLE
#define M1_HFI_OMEGA_SEED_ENABLE        0
#undef M1_HFI_SPEED_FB_ENABLE
#define M1_HFI_SPEED_FB_ENABLE          0 /* 不开 HFI 转速反�?*/
#undef M1_HFI_QKICK_AFTER_LOCK_ENABLE
#define M1_HFI_QKICK_AFTER_LOCK_ENABLE  1
#undef M1_HFI_QKICK_BEFORE_HFI_ENABLE
#define M1_HFI_QKICK_BEFORE_HFI_ENABLE  0
#undef M1_HFI_QKICK_THEN_HFI_ENABLE
#define M1_HFI_QKICK_THEN_HFI_ENABLE    1
#undef M1_HFI_QKICK_SPEED_ENABLE
#define M1_HFI_QKICK_SPEED_ENABLE       0
#undef M1_HFI_QKICK_PRE_S
#define M1_HFI_QKICK_PRE_S              2.0f
#undef M1_HFI_QKICK_IQ_A
#define M1_HFI_QKICK_IQ_A               1.6f
#undef M1_HFI_QKICK_KICK_N
#define M1_HFI_QKICK_KICK_N             6000u
#undef M1_HFI_QKICK_BRAKE_ENABLE
#define M1_HFI_QKICK_BRAKE_ENABLE       0
#undef M1_HFI_QKICK_BRAKE_N
#define M1_HFI_QKICK_BRAKE_N            6000u
#undef M1_HFI_QKICK_HOLD_S
#define M1_HFI_QKICK_HOLD_S             0.4f
#undef M1_HFI_QKICK_CRAWL_ENABLE
#define M1_HFI_QKICK_CRAWL_ENABLE       0
#undef M1_HFI_QKICK_START_ENABLE
#define M1_HFI_QKICK_START_ENABLE       0
#undef M1_HFI_ID_PI_OFF_ENABLE
#define M1_HFI_ID_PI_OFF_ENABLE         1
/* 交接 SM�?8/53�?8 开�?*/
#if (M1_HFI_GATE == 38) || (M1_HFI_GATE == 53) || (M1_HFI_GATE == 54) || \
    (M1_HFI_GATE == 55) || (M1_HFI_GATE == 56) || (M1_HFI_GATE == 57) || \
    (M1_HFI_GATE == 58) || (M1_HFI_GATE == 59) || (M1_HFI_GATE == 60) || \
    (M1_HFI_GATE == 61) || (M1_HFI_GATE == 62) || (M1_HFI_GATE == 63) || \
    (M1_HFI_GATE == 64) || (M1_HFI_GATE == 65) || (M1_HFI_GATE == 66) || (M1_HFI_GATE == 67) || (M1_HFI_GATE == 68) || (M1_HFI_GATE == 69) || (M1_HFI_GATE == 70) || (M1_HFI_GATE == 71) || (M1_HFI_GATE == 72) || (M1_HFI_GATE == 73) || (M1_HFI_GATE == 74) || (M1_HFI_GATE == 75) || (M1_HFI_GATE == 76) || (M1_HFI_GATE == 77) || (M1_HFI_GATE == 78) || (M1_HFI_GATE == 79) || (M1_HFI_GATE == 80) || (M1_HFI_GATE == 91) || (M1_HFI_GATE == 92) || (M1_HFI_GATE == 93)
#undef M1_HFI_SMO_HAND_ENABLE
#define M1_HFI_SMO_HAND_ENABLE          1
#else
#undef M1_HFI_SMO_HAND_ENABLE
#define M1_HFI_SMO_HAND_ENABLE          0
#endif
/* 默认真源序标志；�?GATE 再覆盖�?*/
#undef M1_HFI_HAND_ID_OVERLAP_ENABLE
#define M1_HFI_HAND_ID_OVERLAP_ENABLE   1
#undef M1_HFI_HAND_OPEN_ID_ENABLE
#define M1_HFI_HAND_OPEN_ID_ENABLE      0
#undef M1_HFI_HAND_KILL_VH_ENABLE
#define M1_HFI_HAND_KILL_VH_ENABLE      0
#undef M1_HFI_HAND_IQ_HOLD_ON_IDUP
#define M1_HFI_HAND_IQ_HOLD_ON_IDUP     0
#undef M1_HFI_HAND_W_HOLD_ON_IDUP
#define M1_HFI_HAND_W_HOLD_ON_IDUP      0
#undef M1_HFI_HAND_W_REL_N
#define M1_HFI_HAND_W_REL_N            4000u
#undef M1_HFI_HAND_W_SLEW_ENABLE
#define M1_HFI_HAND_W_SLEW_ENABLE       0
#undef M1_HFI_HAND_W_SLEW_RPM_S
#define M1_HFI_HAND_W_SLEW_RPM_S        (200.0f)
#undef M1_HFI_HAND_W_SLEW_IDUP_ONLY
#define M1_HFI_HAND_W_SLEW_IDUP_ONLY    0
#undef M1_HFI_HAND_W_SLEW_SMO_N
#define M1_HFI_HAND_W_SLEW_SMO_N       4000u
#undef M1_HFI_HAND_VH0_SOFT_ENABLE
#define M1_HFI_HAND_VH0_SOFT_ENABLE     0
#undef M1_HFI_HAND_STOP_AFTER
#define M1_HFI_HAND_STOP_AFTER          0
#undef M1_HFI_HAND_VH_FLOOR
#define M1_HFI_HAND_VH_FLOOR            (0.25f)
#undef M1_HFI_HAND_VH_END
#define M1_HFI_HAND_VH_END              (0.0f) /* KILL 终点�?9 改为微地�?*/
#undef M1_HFI_HAND_FADE_N
#define M1_HFI_HAND_FADE_N             4000u
#undef M1_HFI_HAND_ID_WEAK
#define M1_HFI_HAND_ID_WEAK             (0.12f)
#undef M1_HFI_HAND_HOLD_N
#define M1_HFI_HAND_HOLD_N             10000u /* 0.5 s */
#undef M1_HFI_HAND_VH0_N
#define M1_HFI_HAND_VH0_N              20000u /* 1.0 s 只收 Vh */
#undef M1_HFI_HAND_IDUP_N
#define M1_HFI_HAND_IDUP_N             20000u /* 1.0 s 只抬 Id */
#undef M1_HFI_QKICK_FORCE_PI
#define M1_HFI_QKICK_FORCE_PI           0
#undef M1_HFI_QKICK_PRE_GATE_ENABLE
#define M1_HFI_QKICK_PRE_GATE_ENABLE    1
#undef M1_HFI_IQ_AUTH_ENABLE
#define M1_HFI_IQ_AUTH_ENABLE           1
#undef M1_HFI_IQ_AUTH_X_GOOD
#define M1_HFI_IQ_AUTH_X_GOOD           (0.218f)
#undef M1_HFI_IQ_AUTH_X_BAD
#define M1_HFI_IQ_AUTH_X_BAD            (0.205f)
#undef M1_HFI_IQ_AUTH_EPS_FALSE_MAX
#define M1_HFI_IQ_AUTH_EPS_FALSE_MAX    (0.20f)
#undef M1_HFI_IQ_AUTH_HOLD_N
#define M1_HFI_IQ_AUTH_HOLD_N           2000u
#undef M1_HFI_IQ_AUTH_CLEAR_N
#define M1_HFI_IQ_AUTH_CLEAR_N          1000u
#undef M1_HFI_IQ_AUTH_IQ_LO
#define M1_HFI_IQ_AUTH_IQ_LO            (0.0f)
#undef M1_HFI_IQ_AUTH_IQ_HI
#define M1_HFI_IQ_AUTH_IQ_HI            (0.0f) /* �?C4q：出力只�?FEED_A */
#undef M1_HFI_IQ_AUTH_SLEW_A_S
#define M1_HFI_IQ_AUTH_SLEW_A_S         (10.0f)
#undef M1_HFI_IQ_AUTH_FEED_ENABLE
#define M1_HFI_IQ_AUTH_FEED_ENABLE      1
#undef M1_HFI_IQ_AUTH_FEED_SIGN
#define M1_HFI_IQ_AUTH_FEED_SIGN        (1.0f)
#undef M1_HFI_IQ_AUTH_FEED_A
#define M1_HFI_IQ_AUTH_FEED_A           (0.80f)
#undef M1_HFI_IQ_AUTH_FEED_DELAY_S
#define M1_HFI_IQ_AUTH_FEED_DELAY_S     (0.20f)
#undef M1_HFI_IQ_AUTH_FEED_RAMP_S
#define M1_HFI_IQ_AUTH_FEED_RAMP_S      (0.0f)
#undef M1_HFI_IQ_AUTH_FEED_LEGACY_BRANCH
#define M1_HFI_IQ_AUTH_FEED_LEGACY_BRANCH 0
#if (M1_HFI_GATE == 38) || (M1_HFI_GATE == 53) || (M1_HFI_GATE == 54) || \
    (M1_HFI_GATE == 55) || (M1_HFI_GATE == 56) || (M1_HFI_GATE == 57) || \
    (M1_HFI_GATE == 58) || (M1_HFI_GATE == 59) || (M1_HFI_GATE == 60) || \
    (M1_HFI_GATE == 61) || (M1_HFI_GATE == 62) || (M1_HFI_GATE == 63) || \
    (M1_HFI_GATE == 64) || (M1_HFI_GATE == 65) || (M1_HFI_GATE == 66) || (M1_HFI_GATE == 67) || (M1_HFI_GATE == 68) || (M1_HFI_GATE == 69) || (M1_HFI_GATE == 70) || (M1_HFI_GATE == 71) || (M1_HFI_GATE == 72) || (M1_HFI_GATE == 73) || (M1_HFI_GATE == 74) || (M1_HFI_GATE == 75) || (M1_HFI_GATE == 76) || (M1_HFI_GATE == 77) || (M1_HFI_GATE == 78) || (M1_HFI_GATE == 79) || (M1_HFI_GATE == 80) || (M1_HFI_GATE == 91) || (M1_HFI_GATE == 92) || (M1_HFI_GATE == 93)
/* 1715/1627/1435：闭�?(x,y) 圆心 0.216�?*/
#undef M1_HFI_A_CMD
#define M1_HFI_A_CMD                    (0.216f)
/* 拔编码器：θ�?不从 enc 播种。极性有 180° 模糊，预锁门�?ε/x 自己找轴�?*/
#undef M1_HFI_INIT_FROM_ENC
#define M1_HFI_INIT_FROM_ENC            0
/* 探高速：ω* 0�?800 rpm @ 50 rpm/s，到顶后再守 8 s。PLL 钳位抬到�?2300 rpm，避�?1500 先截住�?*/
#undef M1_HFI_SPEED_FB_ENABLE
#define M1_HFI_SPEED_FB_ENABLE          1
#undef M1_HFI_IQ_AUTH_FEED_ENABLE
#define M1_HFI_IQ_AUTH_FEED_ENABLE      0
#undef M1_HFI_IQ_AUTH_IQ_HI
#define M1_HFI_IQ_AUTH_IQ_HI            (11.0f)
#undef M1_HFI_PLL_W_MAX
#define M1_HFI_PLL_W_MAX                (1700.0f)
#undef M1_HFI_PLL_INT_MAX
#define M1_HFI_PLL_INT_MAX              M1_HFI_PLL_W_MAX
#undef M1_HFI_RUN_LADDER_ENABLE
#define M1_HFI_RUN_LADDER_ENABLE        1
#undef M1_HFI_RUN_RPM_START
#define M1_HFI_RUN_RPM_START            (1800.0f)
#undef M1_HFI_RUN_RPM_STEP
#define M1_HFI_RUN_RPM_STEP             (0.0f)
#undef M1_HFI_RUN_RPM_MAX
#define M1_HFI_RUN_RPM_MAX              (1800.0f)
#undef M1_HFI_RUN_STEP_S
#define M1_HFI_RUN_STEP_S               (44.0f)
#undef M1_SPEED_OMEGA_RAMP_ENABLE
#define M1_SPEED_OMEGA_RAMP_ENABLE      1
#undef M1_SPEED_OMEGA_RAMP_RPM_S
#define M1_SPEED_OMEGA_RAMP_RPM_S       (50.0f)
#undef M1_SPEED_PI_KI
#define M1_SPEED_PI_KI                  (6.5e-6f)
#undef M1_SPEED_PI_BETA
#define M1_SPEED_PI_BETA                (1.0f)
/* 旁路：SMO 用签收增益跟着算。软切保持关闭；交接�?SMO_HAND �?Park/速度权重�?*/
#undef M1_EMF_SMO_ENABLE
#define M1_EMF_SMO_ENABLE               1
#undef M1_EMF_PLL_ENABLE
#define M1_EMF_PLL_ENABLE               1
#undef M1_EMF_PLL_USE_SMO
#define M1_EMF_PLL_USE_SMO              1
#undef M1_OBS_SOFT_SWITCH_ENABLE
#define M1_OBS_SOFT_SWITCH_ENABLE       0
#undef M1_EMF_SMO_K
#define M1_EMF_SMO_K                    20.0f
#undef M1_EMF_SMO_SAT_A
#define M1_EMF_SMO_SAT_A                0.30f
#undef M1_EMF_SMO_LPF_ENABLE
#define M1_EMF_SMO_LPF_ENABLE           1
#undef M1_EMF_SMO_LPF_HZ
#define M1_EMF_SMO_LPF_HZ               200.0f
#undef M1_EMF_SMO_LPF_SCHED_ENABLE
#define M1_EMF_SMO_LPF_SCHED_ENABLE     0
#undef M1_EMF_SMO_LPF_LINEAR_ENABLE
#define M1_EMF_SMO_LPF_LINEAR_ENABLE    1
#undef M1_EMF_SMO_LPF_LINEAR_K
#define M1_EMF_SMO_LPF_LINEAR_K         1.2f
#undef M1_EMF_SMO_LPF_LINEAR_FC_MIN
#define M1_EMF_SMO_LPF_LINEAR_FC_MIN    100.0f
#undef M1_EMF_SMO_LPF_LINEAR_FC_MAX
#define M1_EMF_SMO_LPF_LINEAR_FC_MAX    280.0f
#undef M1_EMF_SMO_THETA_OFF_RAD
#define M1_EMF_SMO_THETA_OFF_RAD        (0.0f)
#undef M1_EMF_PLL_FN_HZ
#define M1_EMF_PLL_FN_HZ                45.0f
#undef M1_EMF_PLL_ZETA
#define M1_EMF_PLL_ZETA                 0.707106781f
#undef M1_EMF_PLL_NORM_ENABLE
#define M1_EMF_PLL_NORM_ENABLE          1
#undef M1_EMF_LPF_PHASE_FF_ENABLE
#define M1_EMF_LPF_PHASE_FF_ENABLE      1
#undef M1_EMF_PLL_THETA_OFF_RAD
#define M1_EMF_PLL_THETA_OFF_RAD        (0.1147f)
#endif
#if M1_HFI_GATE == 53
/* �?1 轮：相对 38 只停�?VH0（Vh=0、Id=WEAK）�?*/
#undef M1_HFI_HAND_STOP_AFTER
#define M1_HFI_HAND_STOP_AFTER          1
#endif
#if M1_HFI_GATE == 54
/* 相对 53：抬地板 + �?HOLD（残 Vh 常驻；不�?VH0）�?*/
#undef M1_HFI_HAND_VH_FLOOR
#define M1_HFI_HAND_VH_FLOOR            (0.40f)
#undef M1_HFI_HAND_STOP_AFTER
#define M1_HFI_HAND_STOP_AFTER          2
#endif
#if M1_HFI_GATE == 55
/* 1646 复现：不开 Id；ANG 完进 SMO 态，�?Vh=0.25 常驻。【基线�?*/
#undef M1_HFI_HAND_ID_OVERLAP_ENABLE
#define M1_HFI_HAND_ID_OVERLAP_ENABLE   0
#undef M1_HFI_HAND_OPEN_ID_ENABLE
#define M1_HFI_HAND_OPEN_ID_ENABLE      0
#undef M1_HFI_HAND_KILL_VH_ENABLE
#define M1_HFI_HAND_KILL_VH_ENABLE      0
#undef M1_HFI_HAND_VH_FLOOR
#define M1_HFI_HAND_VH_FLOOR            (0.25f)
#undef M1_HFI_HAND_STOP_AFTER
#define M1_HFI_HAND_STOP_AFTER          0
#endif
#if M1_HFI_GATE == 56
/* 相对 55：角交后线性灭 Vh�?.0 s），仍不开 Id�?143：钉 0 �?Iq 噪�?*/
#undef M1_HFI_HAND_ID_OVERLAP_ENABLE
#define M1_HFI_HAND_ID_OVERLAP_ENABLE   0
#undef M1_HFI_HAND_OPEN_ID_ENABLE
#define M1_HFI_HAND_OPEN_ID_ENABLE      0
#undef M1_HFI_HAND_KILL_VH_ENABLE
#define M1_HFI_HAND_KILL_VH_ENABLE      1
#undef M1_HFI_HAND_VH0_SOFT_ENABLE
#define M1_HFI_HAND_VH0_SOFT_ENABLE     0
#undef M1_HFI_HAND_VH_FLOOR
#define M1_HFI_HAND_VH_FLOOR            (0.25f)
#undef M1_HFI_HAND_VH0_N
#define M1_HFI_HAND_VH0_N              40000u /* 2.0 s 线�?floor�? */
#undef M1_HFI_HAND_STOP_AFTER
#define M1_HFI_HAND_STOP_AFTER          0
#endif
#if M1_HFI_GATE == 57
/* 相对 56：smoothstep 软灭；FADE 0.8 s；VH0 3.0 s。仍不开 Id�?*/
#undef M1_HFI_HAND_ID_OVERLAP_ENABLE
#define M1_HFI_HAND_ID_OVERLAP_ENABLE   0
#undef M1_HFI_HAND_OPEN_ID_ENABLE
#define M1_HFI_HAND_OPEN_ID_ENABLE      0
#undef M1_HFI_HAND_KILL_VH_ENABLE
#define M1_HFI_HAND_KILL_VH_ENABLE      1
#undef M1_HFI_HAND_VH0_SOFT_ENABLE
#define M1_HFI_HAND_VH0_SOFT_ENABLE     1
#undef M1_HFI_HAND_VH_FLOOR
#define M1_HFI_HAND_VH_FLOOR            (0.25f)
#undef M1_HFI_HAND_FADE_N
#define M1_HFI_HAND_FADE_N             16000u /* 0.8 s soft 1→地�?*/
#undef M1_HFI_HAND_VH0_N
#define M1_HFI_HAND_VH0_N              60000u /* 3.0 s soft 地板�? */
#undef M1_HFI_HAND_STOP_AFTER
#define M1_HFI_HAND_STOP_AFTER          0
#endif
#if M1_HFI_GATE == 58
/* 相对 55：OPEN_ID。ANG→HOLD→残 Vh �?soft Id 0�?→SMO。不�?Vh�?*/
#undef M1_HFI_HAND_ID_OVERLAP_ENABLE
#define M1_HFI_HAND_ID_OVERLAP_ENABLE   0
#undef M1_HFI_HAND_OPEN_ID_ENABLE
#define M1_HFI_HAND_OPEN_ID_ENABLE      1
#undef M1_HFI_HAND_KILL_VH_ENABLE
#define M1_HFI_HAND_KILL_VH_ENABLE      0
#undef M1_HFI_HAND_VH_FLOOR
#define M1_HFI_HAND_VH_FLOOR            (0.25f)
#undef M1_HFI_HAND_HOLD_N
#define M1_HFI_HAND_HOLD_N             10000u /* 0.5 s 消化 Park */
#undef M1_HFI_HAND_IDUP_N
#define M1_HFI_HAND_IDUP_N             20000u /* 1.0 s soft Id */
#undef M1_HFI_HAND_STOP_AFTER
#define M1_HFI_HAND_STOP_AFTER          0
#endif
#if M1_HFI_GATE == 59
/* 相对 57：唯一增量 END=0.05（≈0.02 V），不开 Id。【灭 Vh 基线 / 1238 PASS�?*/
#undef M1_HFI_HAND_ID_OVERLAP_ENABLE
#define M1_HFI_HAND_ID_OVERLAP_ENABLE   0
#undef M1_HFI_HAND_OPEN_ID_ENABLE
#define M1_HFI_HAND_OPEN_ID_ENABLE      0
#undef M1_HFI_HAND_KILL_VH_ENABLE
#define M1_HFI_HAND_KILL_VH_ENABLE      1
#undef M1_HFI_HAND_VH0_SOFT_ENABLE
#define M1_HFI_HAND_VH0_SOFT_ENABLE     1
#undef M1_HFI_HAND_VH_FLOOR
#define M1_HFI_HAND_VH_FLOOR            (0.25f)
#undef M1_HFI_HAND_VH_END
#define M1_HFI_HAND_VH_END              (0.05f) /* 微地�?scale�?.40×0.05�?.02 V */
#undef M1_HFI_HAND_FADE_N
#define M1_HFI_HAND_FADE_N             16000u
#undef M1_HFI_HAND_VH0_N
#define M1_HFI_HAND_VH0_N              60000u /* 3.0 s soft 0.25�?.05 */
#undef M1_HFI_HAND_STOP_AFTER
#define M1_HFI_HAND_STOP_AFTER          0
#endif
#if M1_HFI_GATE == 60
/* 相对 59：微地板后再 soft 开 Id。ANG→VH0→IDUP→SMO�?*/
#undef M1_HFI_HAND_ID_OVERLAP_ENABLE
#define M1_HFI_HAND_ID_OVERLAP_ENABLE   0
#undef M1_HFI_HAND_OPEN_ID_ENABLE
#define M1_HFI_HAND_OPEN_ID_ENABLE      1
#undef M1_HFI_HAND_KILL_VH_ENABLE
#define M1_HFI_HAND_KILL_VH_ENABLE      1
#undef M1_HFI_HAND_IQ_HOLD_ON_IDUP
#define M1_HFI_HAND_IQ_HOLD_ON_IDUP     0
#undef M1_HFI_HAND_VH0_SOFT_ENABLE
#define M1_HFI_HAND_VH0_SOFT_ENABLE     1
#undef M1_HFI_HAND_VH_FLOOR
#define M1_HFI_HAND_VH_FLOOR            (0.25f)
#undef M1_HFI_HAND_VH_END
#define M1_HFI_HAND_VH_END              (0.05f)
#undef M1_HFI_HAND_FADE_N
#define M1_HFI_HAND_FADE_N             16000u
#undef M1_HFI_HAND_VH0_N
#define M1_HFI_HAND_VH0_N              60000u
#undef M1_HFI_HAND_IDUP_N
#define M1_HFI_HAND_IDUP_N             20000u /* 1.0 s soft Id @ 微地�?*/
#undef M1_HFI_HAND_STOP_AFTER
#define M1_HFI_HAND_STOP_AFTER          0
#endif
#if M1_HFI_GATE == 61
/* 相对 60：冻 iq_ref（诊断对照，非产品路径）�?*/
#undef M1_HFI_HAND_ID_OVERLAP_ENABLE
#define M1_HFI_HAND_ID_OVERLAP_ENABLE   0
#undef M1_HFI_HAND_OPEN_ID_ENABLE
#define M1_HFI_HAND_OPEN_ID_ENABLE      1
#undef M1_HFI_HAND_KILL_VH_ENABLE
#define M1_HFI_HAND_KILL_VH_ENABLE      1
#undef M1_HFI_HAND_IQ_HOLD_ON_IDUP
#define M1_HFI_HAND_IQ_HOLD_ON_IDUP     1
#undef M1_HFI_HAND_W_HOLD_ON_IDUP
#define M1_HFI_HAND_W_HOLD_ON_IDUP      0
#undef M1_HFI_HAND_VH0_SOFT_ENABLE
#define M1_HFI_HAND_VH0_SOFT_ENABLE     1
#undef M1_HFI_HAND_VH_FLOOR
#define M1_HFI_HAND_VH_FLOOR            (0.25f)
#undef M1_HFI_HAND_VH_END
#define M1_HFI_HAND_VH_END              (0.05f)
#undef M1_HFI_HAND_FADE_N
#define M1_HFI_HAND_FADE_N             16000u
#undef M1_HFI_HAND_VH0_N
#define M1_HFI_HAND_VH0_N              60000u
#undef M1_HFI_HAND_IDUP_N
#define M1_HFI_HAND_IDUP_N             20000u
#undef M1_HFI_HAND_STOP_AFTER
#define M1_HFI_HAND_STOP_AFTER          0
#endif
#if M1_HFI_GATE == 62
/* 相对 60：开 Id 窗速度环吃 VH0 �?ω；SMO 0.2 s 软释放。不�?iq_ref。【通用�?*/
#undef M1_HFI_HAND_ID_OVERLAP_ENABLE
#define M1_HFI_HAND_ID_OVERLAP_ENABLE   0
#undef M1_HFI_HAND_OPEN_ID_ENABLE
#define M1_HFI_HAND_OPEN_ID_ENABLE      1
#undef M1_HFI_HAND_KILL_VH_ENABLE
#define M1_HFI_HAND_KILL_VH_ENABLE      1
#undef M1_HFI_HAND_IQ_HOLD_ON_IDUP
#define M1_HFI_HAND_IQ_HOLD_ON_IDUP     0
#undef M1_HFI_HAND_W_HOLD_ON_IDUP
#define M1_HFI_HAND_W_HOLD_ON_IDUP      1
#undef M1_HFI_HAND_W_REL_N
#define M1_HFI_HAND_W_REL_N            4000u /* 0.2 s */
#undef M1_HFI_HAND_VH0_SOFT_ENABLE
#define M1_HFI_HAND_VH0_SOFT_ENABLE     1
#undef M1_HFI_HAND_VH_FLOOR
#define M1_HFI_HAND_VH_FLOOR            (0.25f)
#undef M1_HFI_HAND_VH_END
#define M1_HFI_HAND_VH_END              (0.05f)
#undef M1_HFI_HAND_FADE_N
#define M1_HFI_HAND_FADE_N             16000u
#undef M1_HFI_HAND_VH0_N
#define M1_HFI_HAND_VH0_N              60000u
#undef M1_HFI_HAND_IDUP_N
#define M1_HFI_HAND_IDUP_N             20000u
#undef M1_HFI_HAND_STOP_AFTER
#define M1_HFI_HAND_STOP_AFTER          0
#endif
#if M1_HFI_GATE == 63
/* 相对 60：ANG 起全程限斜率�?319：VH0 门口 FAIL）�?*/
#undef M1_HFI_HAND_ID_OVERLAP_ENABLE
#define M1_HFI_HAND_ID_OVERLAP_ENABLE   0
#undef M1_HFI_HAND_OPEN_ID_ENABLE
#define M1_HFI_HAND_OPEN_ID_ENABLE      1
#undef M1_HFI_HAND_KILL_VH_ENABLE
#define M1_HFI_HAND_KILL_VH_ENABLE      1
#undef M1_HFI_HAND_IQ_HOLD_ON_IDUP
#define M1_HFI_HAND_IQ_HOLD_ON_IDUP     0
#undef M1_HFI_HAND_W_HOLD_ON_IDUP
#define M1_HFI_HAND_W_HOLD_ON_IDUP      0
#undef M1_HFI_HAND_W_SLEW_ENABLE
#define M1_HFI_HAND_W_SLEW_ENABLE       1
#undef M1_HFI_HAND_W_SLEW_IDUP_ONLY
#define M1_HFI_HAND_W_SLEW_IDUP_ONLY    0
#undef M1_HFI_HAND_W_SLEW_RPM_S
#define M1_HFI_HAND_W_SLEW_RPM_S        (200.0f)
#undef M1_HFI_HAND_VH0_SOFT_ENABLE
#define M1_HFI_HAND_VH0_SOFT_ENABLE     1
#undef M1_HFI_HAND_VH_FLOOR
#define M1_HFI_HAND_VH_FLOOR            (0.25f)
#undef M1_HFI_HAND_VH_END
#define M1_HFI_HAND_VH_END              (0.05f)
#undef M1_HFI_HAND_FADE_N
#define M1_HFI_HAND_FADE_N             16000u
#undef M1_HFI_HAND_VH0_N
#define M1_HFI_HAND_VH0_N              60000u
#undef M1_HFI_HAND_IDUP_N
#define M1_HFI_HAND_IDUP_N             20000u
#undef M1_HFI_HAND_STOP_AFTER
#define M1_HFI_HAND_STOP_AFTER          0
#endif
#if M1_HFI_GATE == 64
/* 相对 60：限斜率�?IDUP + SMO 0.2 s；ANG/VH0 跟活速�?*/
#undef M1_HFI_HAND_ID_OVERLAP_ENABLE
#define M1_HFI_HAND_ID_OVERLAP_ENABLE   0
#undef M1_HFI_HAND_OPEN_ID_ENABLE
#define M1_HFI_HAND_OPEN_ID_ENABLE      1
#undef M1_HFI_HAND_KILL_VH_ENABLE
#define M1_HFI_HAND_KILL_VH_ENABLE      1
#undef M1_HFI_HAND_IQ_HOLD_ON_IDUP
#define M1_HFI_HAND_IQ_HOLD_ON_IDUP     0
#undef M1_HFI_HAND_W_HOLD_ON_IDUP
#define M1_HFI_HAND_W_HOLD_ON_IDUP      0
#undef M1_HFI_HAND_W_SLEW_ENABLE
#define M1_HFI_HAND_W_SLEW_ENABLE       1
#undef M1_HFI_HAND_W_SLEW_IDUP_ONLY
#define M1_HFI_HAND_W_SLEW_IDUP_ONLY    1
#undef M1_HFI_HAND_W_SLEW_RPM_S
#define M1_HFI_HAND_W_SLEW_RPM_S        (200.0f)
#undef M1_HFI_HAND_W_SLEW_SMO_N
#define M1_HFI_HAND_W_SLEW_SMO_N       4000u
#undef M1_HFI_HAND_VH0_SOFT_ENABLE
#define M1_HFI_HAND_VH0_SOFT_ENABLE     1
#undef M1_HFI_HAND_VH_FLOOR
#define M1_HFI_HAND_VH_FLOOR            (0.25f)
#undef M1_HFI_HAND_VH_END
#define M1_HFI_HAND_VH_END              (0.05f)
#undef M1_HFI_HAND_FADE_N
#define M1_HFI_HAND_FADE_N             16000u
#undef M1_HFI_HAND_VH0_N
#define M1_HFI_HAND_VH0_N              60000u
#undef M1_HFI_HAND_IDUP_N
#define M1_HFI_HAND_IDUP_N             20000u
#undef M1_HFI_HAND_STOP_AFTER
#define M1_HFI_HAND_STOP_AFTER          0
#endif
#if M1_HFI_GATE == 65
/* 相对 59：交�?Id 不动。�? 1800 守完后切 800，外环仍 50 rpm/s 对称斜坡�?*/
#undef M1_HFI_HAND_ID_OVERLAP_ENABLE
#define M1_HFI_HAND_ID_OVERLAP_ENABLE   0
#undef M1_HFI_HAND_OPEN_ID_ENABLE
#define M1_HFI_HAND_OPEN_ID_ENABLE      0
#undef M1_HFI_HAND_KILL_VH_ENABLE
#define M1_HFI_HAND_KILL_VH_ENABLE      1
#undef M1_HFI_HAND_VH0_SOFT_ENABLE
#define M1_HFI_HAND_VH0_SOFT_ENABLE     1
#undef M1_HFI_HAND_VH_FLOOR
#define M1_HFI_HAND_VH_FLOOR            (0.25f)
#undef M1_HFI_HAND_VH_END
#define M1_HFI_HAND_VH_END              (0.05f)
#undef M1_HFI_HAND_FADE_N
#define M1_HFI_HAND_FADE_N             16000u
#undef M1_HFI_HAND_VH0_N
#define M1_HFI_HAND_VH0_N              60000u
#undef M1_HFI_HAND_STOP_AFTER
#define M1_HFI_HAND_STOP_AFTER          0
#undef M1_HFI_RUN_LADDER_ENABLE
#define M1_HFI_RUN_LADDER_ENABLE        0
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 (1800.0f)
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               (44.0f) /* �?0�?800 斜坡 + 短守 */
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 (800.0f) /* 不停�?0；SMO 仍应够用 */
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               (28.0f) /* 20 s 斜坡 + ~8 s �?*/
#endif
#if M1_HFI_GATE == 66
/* 相对 59：开 SMO→HFI 反向。节时：爬到 1500 短守即减速，�?1400 �?HFI。不开 Id�?*/
#undef M1_HFI_HAND_ID_OVERLAP_ENABLE
#define M1_HFI_HAND_ID_OVERLAP_ENABLE   0
#undef M1_HFI_HAND_OPEN_ID_ENABLE
#define M1_HFI_HAND_OPEN_ID_ENABLE      0
#undef M1_HFI_HAND_KILL_VH_ENABLE
#define M1_HFI_HAND_KILL_VH_ENABLE      1
#undef M1_HFI_HAND_VH0_SOFT_ENABLE
#define M1_HFI_HAND_VH0_SOFT_ENABLE     1
#undef M1_HFI_HAND_VH_FLOOR
#define M1_HFI_HAND_VH_FLOOR            (0.25f)
#undef M1_HFI_HAND_VH_END
#define M1_HFI_HAND_VH_END              (0.05f)
#undef M1_HFI_HAND_FADE_N
#define M1_HFI_HAND_FADE_N             16000u
#undef M1_HFI_HAND_VH0_N
#define M1_HFI_HAND_VH0_N              60000u
#undef M1_HFI_HAND_STOP_AFTER
#define M1_HFI_HAND_STOP_AFTER          0
#undef M1_HFI_HAND_REV_ENABLE
#define M1_HFI_HAND_REV_ENABLE          1
#undef M1_HFI_HAND_REV_RPM
#define M1_HFI_HAND_REV_RPM             (1400.0f)
#undef M1_HFI_HAND_REV_ARM_RPM
#define M1_HFI_HAND_REV_ARM_RPM         (1450.0f)
#undef M1_HFI_HAND_RVH_N
#define M1_HFI_HAND_RVH_N              20000u /* 1 s END→FLOOR */
#undef M1_HFI_HAND_RANG_N
#define M1_HFI_HAND_RANG_N             20000u /* 1 s ang 1�? */
#undef M1_HFI_HAND_RFADE_N
#define M1_HFI_HAND_RFADE_N            16000u /* 0.8 s FLOOR�? */
#undef M1_HFI_HAND_RSPD_N
#define M1_HFI_HAND_RSPD_N             40000u /* 2 s alpha 1�? */
#undef M1_HFI_RUN_LADDER_ENABLE
#define M1_HFI_RUN_LADDER_ENABLE        0
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 (1500.0f) /* 不必�?1800；前向在 900�?100 已交�?*/
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               (36.0f) /* ~30 s 斜坡 + ~6 s �?*/
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 (600.0f) /* 反向触发后继续降，看 HFI 能否接着�?*/
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               (26.0f) /* 18 s 斜坡 + ~8 s �?*/
#endif
#if M1_HFI_GATE == 67
/* 相对 66：反�?RVH �?hold �?θ̂ + RQUAL 过门再交角。RVH 时长对齐 VH0=3 s�?*/
#undef M1_HFI_HAND_ID_OVERLAP_ENABLE
#define M1_HFI_HAND_ID_OVERLAP_ENABLE   0
#undef M1_HFI_HAND_OPEN_ID_ENABLE
#define M1_HFI_HAND_OPEN_ID_ENABLE      0
#undef M1_HFI_HAND_KILL_VH_ENABLE
#define M1_HFI_HAND_KILL_VH_ENABLE      1
#undef M1_HFI_HAND_VH0_SOFT_ENABLE
#define M1_HFI_HAND_VH0_SOFT_ENABLE     1
#undef M1_HFI_HAND_VH_FLOOR
#define M1_HFI_HAND_VH_FLOOR            (0.25f)
#undef M1_HFI_HAND_VH_END
#define M1_HFI_HAND_VH_END              (0.05f)
#undef M1_HFI_HAND_FADE_N
#define M1_HFI_HAND_FADE_N             16000u
#undef M1_HFI_HAND_VH0_N
#define M1_HFI_HAND_VH0_N              60000u
#undef M1_HFI_HAND_STOP_AFTER
#define M1_HFI_HAND_STOP_AFTER          0
#undef M1_HFI_HAND_REV_ENABLE
#define M1_HFI_HAND_REV_ENABLE          1
#undef M1_HFI_HAND_REV_WAKE_ENABLE
#define M1_HFI_HAND_REV_WAKE_ENABLE     1 /* 唯一增量相对 66 */
#undef M1_HFI_HAND_REV_RPM
#define M1_HFI_HAND_REV_RPM             (1400.0f)
#undef M1_HFI_HAND_REV_ARM_RPM
#define M1_HFI_HAND_REV_ARM_RPM         (1450.0f)
#undef M1_HFI_HAND_RVH_N
#define M1_HFI_HAND_RVH_N              60000u /* 3 s END→FLOOR，对�?VH0 */
#undef M1_HFI_HAND_RQUAL_N
#define M1_HFI_HAND_RQUAL_N            8000u /* 0.4 s �?*/
#undef M1_HFI_HAND_RQUAL_TIMEOUT_N
#define M1_HFI_HAND_RQUAL_TIMEOUT_N    60000u /* 3 s 锁不上退 SMO */
#undef M1_HFI_HAND_RANG_N
#define M1_HFI_HAND_RANG_N             20000u
#undef M1_HFI_HAND_RFADE_N
#define M1_HFI_HAND_RFADE_N            16000u
#undef M1_HFI_HAND_RSPD_N
#define M1_HFI_HAND_RSPD_N             40000u
#undef M1_HFI_RUN_LADDER_ENABLE
#define M1_HFI_RUN_LADDER_ENABLE        0
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 (1500.0f)
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               (36.0f)
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 (600.0f)
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               (26.0f)
#endif
#if M1_HFI_GATE == 68
/* 相对 67：唤�?Vh 拉满 1.0；RQUAL 去掉 dw 门（1506 误杀）。结构沿�?WAKE�?*/
#undef M1_HFI_HAND_ID_OVERLAP_ENABLE
#define M1_HFI_HAND_ID_OVERLAP_ENABLE   0
#undef M1_HFI_HAND_OPEN_ID_ENABLE
#define M1_HFI_HAND_OPEN_ID_ENABLE      0
#undef M1_HFI_HAND_KILL_VH_ENABLE
#define M1_HFI_HAND_KILL_VH_ENABLE      1
#undef M1_HFI_HAND_VH0_SOFT_ENABLE
#define M1_HFI_HAND_VH0_SOFT_ENABLE     1
#undef M1_HFI_HAND_VH_FLOOR
#define M1_HFI_HAND_VH_FLOOR            (0.25f)
#undef M1_HFI_HAND_VH_END
#define M1_HFI_HAND_VH_END              (0.05f)
#undef M1_HFI_HAND_FADE_N
#define M1_HFI_HAND_FADE_N             16000u
#undef M1_HFI_HAND_VH0_N
#define M1_HFI_HAND_VH0_N              60000u
#undef M1_HFI_HAND_STOP_AFTER
#define M1_HFI_HAND_STOP_AFTER          0
#undef M1_HFI_HAND_REV_ENABLE
#define M1_HFI_HAND_REV_ENABLE          1
#undef M1_HFI_HAND_REV_WAKE_ENABLE
#define M1_HFI_HAND_REV_WAKE_ENABLE     1
#undef M1_HFI_HAND_REV_VH_WAKE
#define M1_HFI_HAND_REV_VH_WAKE         (1.0f) /* 唯一主增量：满注入唤�?*/
#undef M1_HFI_HAND_REV_RPM
#define M1_HFI_HAND_REV_RPM             (1400.0f)
#undef M1_HFI_HAND_REV_ARM_RPM
#define M1_HFI_HAND_REV_ARM_RPM         (1450.0f)
#undef M1_HFI_HAND_RVH_N
#define M1_HFI_HAND_RVH_N              60000u /* 3 s END�? */
#undef M1_HFI_HAND_RQUAL_N
#define M1_HFI_HAND_RQUAL_N            8000u
#undef M1_HFI_HAND_RQUAL_TIMEOUT_N
#define M1_HFI_HAND_RQUAL_TIMEOUT_N    60000u
#undef M1_HFI_HAND_RANG_N
#define M1_HFI_HAND_RANG_N             20000u
#undef M1_HFI_HAND_RFADE_N
#define M1_HFI_HAND_RFADE_N            4000u /* 已满幅，短过即可 */
#undef M1_HFI_HAND_RSPD_N
#define M1_HFI_HAND_RSPD_N             40000u
#undef M1_HFI_RUN_LADDER_ENABLE
#define M1_HFI_RUN_LADDER_ENABLE        0
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 (1500.0f)
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               (36.0f)
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 (600.0f)
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               (26.0f)
#endif
#if M1_HFI_GATE == 69
/* 相对 68：RVH 钉 θ̂=SMO（hold+coast+reseed），满 Vh 后再 RQUAL 放 PLL；去 x 上界。 */
#undef M1_HFI_HAND_ID_OVERLAP_ENABLE
#define M1_HFI_HAND_ID_OVERLAP_ENABLE   0
#undef M1_HFI_HAND_OPEN_ID_ENABLE
#define M1_HFI_HAND_OPEN_ID_ENABLE      0
#undef M1_HFI_HAND_KILL_VH_ENABLE
#define M1_HFI_HAND_KILL_VH_ENABLE      1
#undef M1_HFI_HAND_VH0_SOFT_ENABLE
#define M1_HFI_HAND_VH0_SOFT_ENABLE     1
#undef M1_HFI_HAND_VH_FLOOR
#define M1_HFI_HAND_VH_FLOOR            (0.25f)
#undef M1_HFI_HAND_VH_END
#define M1_HFI_HAND_VH_END              (0.05f)
#undef M1_HFI_HAND_FADE_N
#define M1_HFI_HAND_FADE_N             16000u
#undef M1_HFI_HAND_VH0_N
#define M1_HFI_HAND_VH0_N              60000u
#undef M1_HFI_HAND_STOP_AFTER
#define M1_HFI_HAND_STOP_AFTER          0
#undef M1_HFI_HAND_REV_ENABLE
#define M1_HFI_HAND_REV_ENABLE          1
#undef M1_HFI_HAND_REV_WAKE_ENABLE
#define M1_HFI_HAND_REV_WAKE_ENABLE     1
#undef M1_HFI_HAND_REV_VH_WAKE
#define M1_HFI_HAND_REV_VH_WAKE         (1.0f)
#undef M1_HFI_HAND_REV_RVH_HOLD_ENABLE
#define M1_HFI_HAND_REV_RVH_HOLD_ENABLE 1 /* 主增量：RVH 钉角 */
#undef M1_HFI_HAND_RQUAL_X_MAX_ENABLE
#define M1_HFI_HAND_RQUAL_X_MAX_ENABLE  0 /* 去 x 上界（1515） */
#undef M1_HFI_HAND_REV_RPM
#define M1_HFI_HAND_REV_RPM             (1400.0f)
#undef M1_HFI_HAND_REV_ARM_RPM
#define M1_HFI_HAND_REV_ARM_RPM         (1450.0f)
#undef M1_HFI_HAND_RVH_N
#define M1_HFI_HAND_RVH_N              60000u
#undef M1_HFI_HAND_RQUAL_N
#define M1_HFI_HAND_RQUAL_N            8000u
#undef M1_HFI_HAND_RQUAL_TIMEOUT_N
#define M1_HFI_HAND_RQUAL_TIMEOUT_N    60000u
#undef M1_HFI_HAND_RANG_N
#define M1_HFI_HAND_RANG_N             20000u
#undef M1_HFI_HAND_RFADE_N
#define M1_HFI_HAND_RFADE_N            4000u
#undef M1_HFI_HAND_RSPD_N
#define M1_HFI_HAND_RSPD_N             40000u
#undef M1_HFI_RUN_LADDER_ENABLE
#define M1_HFI_RUN_LADDER_ENABLE        0
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 (1500.0f)
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               (36.0f)
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 (600.0f)
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               (26.0f)
#endif
#if M1_HFI_GATE == 70
/* Rel 59 forward + observe: decel to 900, Vh END->0.4, HFI bypass only. No angle/speed handoff. */
#undef M1_HFI_HAND_ID_OVERLAP_ENABLE
#define M1_HFI_HAND_ID_OVERLAP_ENABLE   0
#undef M1_HFI_HAND_OPEN_ID_ENABLE
#define M1_HFI_HAND_OPEN_ID_ENABLE      0
#undef M1_HFI_HAND_KILL_VH_ENABLE
#define M1_HFI_HAND_KILL_VH_ENABLE      1
#undef M1_HFI_HAND_VH0_SOFT_ENABLE
#define M1_HFI_HAND_VH0_SOFT_ENABLE     1
#undef M1_HFI_HAND_VH_FLOOR
#define M1_HFI_HAND_VH_FLOOR            (0.25f)
#undef M1_HFI_HAND_VH_END
#define M1_HFI_HAND_VH_END              (0.05f)
#undef M1_HFI_HAND_FADE_N
#define M1_HFI_HAND_FADE_N             16000u
#undef M1_HFI_HAND_VH0_N
#define M1_HFI_HAND_VH0_N              60000u
#undef M1_HFI_HAND_STOP_AFTER
#define M1_HFI_HAND_STOP_AFTER          0
#undef M1_HFI_HAND_REV_ENABLE
#define M1_HFI_HAND_REV_ENABLE          1
#undef M1_HFI_HAND_REV_WAKE_ENABLE
#define M1_HFI_HAND_REV_WAKE_ENABLE     1
#undef M1_HFI_HAND_REV_OBS_ENABLE
#define M1_HFI_HAND_REV_OBS_ENABLE      1 /* main: observe only */
#undef M1_HFI_HAND_REV_VH_WAKE
#define M1_HFI_HAND_REV_VH_WAKE         (0.40f) /* product residual scale */
#undef M1_HFI_HAND_REV_RVH_HOLD_ENABLE
#define M1_HFI_HAND_REV_RVH_HOLD_ENABLE 1 /* pin hat during Vh ramp */
#undef M1_HFI_HAND_SPD_X_KILL_HI
#define M1_HFI_HAND_SPD_X_KILL_HI       (0.70f) /* avoid 1530 SPD abort */
#undef M1_HFI_HAND_REV_RPM
#define M1_HFI_HAND_REV_RPM             (900.0f)
#undef M1_HFI_HAND_REV_ARM_RPM
#define M1_HFI_HAND_REV_ARM_RPM         (950.0f)
#undef M1_HFI_HAND_RVH_N
#define M1_HFI_HAND_RVH_N              60000u /* 3 s END->0.4 */
#undef M1_HFI_RUN_LADDER_ENABLE
#define M1_HFI_RUN_LADDER_ENABLE        0
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 (1500.0f)
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               (36.0f)
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 (900.0f)
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               (26.0f)
#endif
#if M1_HFI_GATE == 71
/* Rel 70: Vh rise mirrors forward VH0+FADE (END->FLOOR->WAKE=0.4). Still OBS only. */
#undef M1_HFI_HAND_ID_OVERLAP_ENABLE
#define M1_HFI_HAND_ID_OVERLAP_ENABLE   0
#undef M1_HFI_HAND_OPEN_ID_ENABLE
#define M1_HFI_HAND_OPEN_ID_ENABLE      0
#undef M1_HFI_HAND_KILL_VH_ENABLE
#define M1_HFI_HAND_KILL_VH_ENABLE      1
#undef M1_HFI_HAND_VH0_SOFT_ENABLE
#define M1_HFI_HAND_VH0_SOFT_ENABLE     1
#undef M1_HFI_HAND_VH_FLOOR
#define M1_HFI_HAND_VH_FLOOR            (0.25f)
#undef M1_HFI_HAND_VH_END
#define M1_HFI_HAND_VH_END              (0.05f)
#undef M1_HFI_HAND_FADE_N
#define M1_HFI_HAND_FADE_N             16000u
#undef M1_HFI_HAND_VH0_N
#define M1_HFI_HAND_VH0_N              60000u
#undef M1_HFI_HAND_STOP_AFTER
#define M1_HFI_HAND_STOP_AFTER          0
#undef M1_HFI_HAND_REV_ENABLE
#define M1_HFI_HAND_REV_ENABLE          1
#undef M1_HFI_HAND_REV_WAKE_ENABLE
#define M1_HFI_HAND_REV_WAKE_ENABLE     1
#undef M1_HFI_HAND_REV_OBS_ENABLE
#define M1_HFI_HAND_REV_OBS_ENABLE      1
#undef M1_HFI_HAND_REV_VH_MIRROR_ENABLE
#define M1_HFI_HAND_REV_VH_MIRROR_ENABLE 1 /* main vs 70 */
#undef M1_HFI_HAND_REV_VH_WAKE
#define M1_HFI_HAND_REV_VH_WAKE         (0.40f)
#undef M1_HFI_HAND_REV_RVH_HOLD_ENABLE
#define M1_HFI_HAND_REV_RVH_HOLD_ENABLE 1
#undef M1_HFI_HAND_SPD_X_KILL_HI
#define M1_HFI_HAND_SPD_X_KILL_HI       (0.70f)
#undef M1_HFI_HAND_REV_RPM
#define M1_HFI_HAND_REV_RPM             (900.0f)
#undef M1_HFI_HAND_REV_ARM_RPM
#define M1_HFI_HAND_REV_ARM_RPM         (950.0f)
#undef M1_HFI_RUN_LADDER_ENABLE
#define M1_HFI_RUN_LADDER_ENABLE        0
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 (1500.0f)
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               (36.0f)
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 (900.0f)
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               (26.0f)
#endif
#if M1_HFI_GATE == 72
/* Accel baseline (freeze): HFI→SMO hand, climb+hold 1500. Decel 1500→900 still in profile
 * (known LOS ~1050 on 2147); do not treat decel hang as failing this gate's accel PASS.
 * Rel 71: REV off. BRAKE floor stays 0 (default). */
#undef M1_HFI_HAND_ID_OVERLAP_ENABLE
#define M1_HFI_HAND_ID_OVERLAP_ENABLE   0
#undef M1_HFI_HAND_OPEN_ID_ENABLE
#define M1_HFI_HAND_OPEN_ID_ENABLE      0
#undef M1_HFI_HAND_KILL_VH_ENABLE
#define M1_HFI_HAND_KILL_VH_ENABLE      1
#undef M1_HFI_HAND_VH0_SOFT_ENABLE
#define M1_HFI_HAND_VH0_SOFT_ENABLE     1
#undef M1_HFI_HAND_VH_FLOOR
#define M1_HFI_HAND_VH_FLOOR            (0.25f)
#undef M1_HFI_HAND_VH_END
#define M1_HFI_HAND_VH_END              (0.05f)
#undef M1_HFI_HAND_FADE_N
#define M1_HFI_HAND_FADE_N             16000u
#undef M1_HFI_HAND_VH0_N
#define M1_HFI_HAND_VH0_N              60000u
#undef M1_HFI_HAND_STOP_AFTER
#define M1_HFI_HAND_STOP_AFTER          0
#undef M1_HFI_HAND_REV_ENABLE
#define M1_HFI_HAND_REV_ENABLE          0 /* main vs 71: no reverse */
#undef M1_HFI_HAND_SPD_X_KILL_HI
#define M1_HFI_HAND_SPD_X_KILL_HI       (0.70f)
#undef M1_HFI_RUN_LADDER_ENABLE
#define M1_HFI_RUN_LADDER_ENABLE        0
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 (1500.0f)
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               (36.0f)
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 (900.0f)
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               (26.0f)
#endif
#if M1_HFI_GATE == 73
/* Rel 72: SMO decel |Iq| floor=1.5 A. Arm on 1500, open on ω* drop (step OK). */
#undef M1_HFI_HAND_ID_OVERLAP_ENABLE
#define M1_HFI_HAND_ID_OVERLAP_ENABLE   0
#undef M1_HFI_HAND_OPEN_ID_ENABLE
#define M1_HFI_HAND_OPEN_ID_ENABLE      0
#undef M1_HFI_HAND_KILL_VH_ENABLE
#define M1_HFI_HAND_KILL_VH_ENABLE      1
#undef M1_HFI_HAND_VH0_SOFT_ENABLE
#define M1_HFI_HAND_VH0_SOFT_ENABLE     1
#undef M1_HFI_HAND_VH_FLOOR
#define M1_HFI_HAND_VH_FLOOR            (0.25f)
#undef M1_HFI_HAND_VH_END
#define M1_HFI_HAND_VH_END              (0.05f)
#undef M1_HFI_HAND_FADE_N
#define M1_HFI_HAND_FADE_N             16000u
#undef M1_HFI_HAND_VH0_N
#define M1_HFI_HAND_VH0_N              60000u
#undef M1_HFI_HAND_STOP_AFTER
#define M1_HFI_HAND_STOP_AFTER          0
#undef M1_HFI_HAND_REV_ENABLE
#define M1_HFI_HAND_REV_ENABLE          0
#undef M1_HFI_HAND_DECEL_BRAKE_ENABLE
#define M1_HFI_HAND_DECEL_BRAKE_ENABLE  1 /* main vs 72 */
#undef M1_HFI_HAND_DECEL_BRAKE_IQ_A
#define M1_HFI_HAND_DECEL_BRAKE_IQ_A    (1.5f) /* |Iq|_min */
#undef M1_HFI_HAND_DECEL_BRAKE_END_RPM
#define M1_HFI_HAND_DECEL_BRAKE_END_RPM (920.0f) /* release on SMO ω, not ω* */
#undef M1_HFI_HAND_DECEL_BRAKE_ARM_RPM
#define M1_HFI_HAND_DECEL_BRAKE_ARM_RPM (1400.0f)
#undef M1_HFI_HAND_SPD_X_KILL_HI
#define M1_HFI_HAND_SPD_X_KILL_HI       (0.70f)
#undef M1_HFI_RUN_LADDER_ENABLE
#define M1_HFI_RUN_LADDER_ENABLE        0
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 (1500.0f)
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               (36.0f)
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 (900.0f)
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               (26.0f)
#endif
#if M1_HFI_GATE == 74
/* Rel 72: SMO |Iq| floor=1.5 A when omega* << SMO-omega (decel intent). Safe at 1500 cruise. */
#undef M1_HFI_HAND_ID_OVERLAP_ENABLE
#define M1_HFI_HAND_ID_OVERLAP_ENABLE   0
#undef M1_HFI_HAND_OPEN_ID_ENABLE
#define M1_HFI_HAND_OPEN_ID_ENABLE      0
#undef M1_HFI_HAND_KILL_VH_ENABLE
#define M1_HFI_HAND_KILL_VH_ENABLE      1
#undef M1_HFI_HAND_VH0_SOFT_ENABLE
#define M1_HFI_HAND_VH0_SOFT_ENABLE     1
#undef M1_HFI_HAND_VH_FLOOR
#define M1_HFI_HAND_VH_FLOOR            (0.25f)
#undef M1_HFI_HAND_VH_END
#define M1_HFI_HAND_VH_END              (0.05f)
#undef M1_HFI_HAND_FADE_N
#define M1_HFI_HAND_FADE_N             16000u
#undef M1_HFI_HAND_VH0_N
#define M1_HFI_HAND_VH0_N              60000u
#undef M1_HFI_HAND_STOP_AFTER
#define M1_HFI_HAND_STOP_AFTER          0
#undef M1_HFI_HAND_REV_ENABLE
#define M1_HFI_HAND_REV_ENABLE          0
#undef M1_HFI_HAND_DECEL_BRAKE_ENABLE
#define M1_HFI_HAND_DECEL_BRAKE_ENABLE  1 /* main vs 72 */
#undef M1_HFI_HAND_DECEL_BRAKE_IQ_A
#define M1_HFI_HAND_DECEL_BRAKE_IQ_A    (1.5f)
#undef M1_HFI_HAND_DECEL_BRAKE_END_RPM
#define M1_HFI_HAND_DECEL_BRAKE_END_RPM (920.0f)
#undef M1_HFI_HAND_DECEL_BRAKE_ARM_RPM
#define M1_HFI_HAND_DECEL_BRAKE_ARM_RPM (1400.0f)
#undef M1_HFI_HAND_DECEL_BRAKE_DROP_RPM
#define M1_HFI_HAND_DECEL_BRAKE_DROP_RPM (80.0f)
#undef M1_HFI_HAND_SPD_X_KILL_HI
#define M1_HFI_HAND_SPD_X_KILL_HI       (0.70f)
#undef M1_HFI_RUN_LADDER_ENABLE
#define M1_HFI_RUN_LADDER_ENABLE        0
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 (1500.0f)
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               (36.0f)
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 (900.0f)
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               (26.0f)
#endif
#if M1_HFI_GATE == 75
/* Rel 72: signed brake floor −1.5 A when ω* << SMO-ω. Fixes 74 same-sign |Iq| floor. */
#undef M1_HFI_HAND_ID_OVERLAP_ENABLE
#define M1_HFI_HAND_ID_OVERLAP_ENABLE   0
#undef M1_HFI_HAND_OPEN_ID_ENABLE
#define M1_HFI_HAND_OPEN_ID_ENABLE      0
#undef M1_HFI_HAND_KILL_VH_ENABLE
#define M1_HFI_HAND_KILL_VH_ENABLE      1
#undef M1_HFI_HAND_VH0_SOFT_ENABLE
#define M1_HFI_HAND_VH0_SOFT_ENABLE     1
#undef M1_HFI_HAND_VH_FLOOR
#define M1_HFI_HAND_VH_FLOOR            (0.25f)
#undef M1_HFI_HAND_VH_END
#define M1_HFI_HAND_VH_END              (0.05f)
#undef M1_HFI_HAND_FADE_N
#define M1_HFI_HAND_FADE_N             16000u
#undef M1_HFI_HAND_VH0_N
#define M1_HFI_HAND_VH0_N              60000u
#undef M1_HFI_HAND_STOP_AFTER
#define M1_HFI_HAND_STOP_AFTER          0
#undef M1_HFI_HAND_REV_ENABLE
#define M1_HFI_HAND_REV_ENABLE          0
#undef M1_HFI_HAND_DECEL_BRAKE_ENABLE
#define M1_HFI_HAND_DECEL_BRAKE_ENABLE  1 /* main vs 72: signed brake floor */
#undef M1_HFI_HAND_DECEL_BRAKE_IQ_A
#define M1_HFI_HAND_DECEL_BRAKE_IQ_A    (1.5f) /* iq* <= -1.5 when ω>0 */
#undef M1_HFI_HAND_DECEL_BRAKE_END_RPM
#define M1_HFI_HAND_DECEL_BRAKE_END_RPM (920.0f)
#undef M1_HFI_HAND_DECEL_BRAKE_ARM_RPM
#define M1_HFI_HAND_DECEL_BRAKE_ARM_RPM (1400.0f)
#undef M1_HFI_HAND_DECEL_BRAKE_DROP_RPM
#define M1_HFI_HAND_DECEL_BRAKE_DROP_RPM (80.0f)
#undef M1_HFI_HAND_SPD_X_KILL_HI
#define M1_HFI_HAND_SPD_X_KILL_HI       (0.70f)
#undef M1_HFI_RUN_LADDER_ENABLE
#define M1_HFI_RUN_LADDER_ENABLE        0
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 (1500.0f)
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               (36.0f)
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 (900.0f)
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               (26.0f)
#endif
#if M1_HFI_GATE == 76
/* D1 / Rel 72: accel ramp unchanged 50 rpm/s; decel-only slower 20 rpm/s.
 * RPM2 dwell lengthened so 1500→900 ramp (~30 s) + short hold fit. No brake floor. */
#undef M1_HFI_HAND_ID_OVERLAP_ENABLE
#define M1_HFI_HAND_ID_OVERLAP_ENABLE   0
#undef M1_HFI_HAND_OPEN_ID_ENABLE
#define M1_HFI_HAND_OPEN_ID_ENABLE      0
#undef M1_HFI_HAND_KILL_VH_ENABLE
#define M1_HFI_HAND_KILL_VH_ENABLE      1
#undef M1_HFI_HAND_VH0_SOFT_ENABLE
#define M1_HFI_HAND_VH0_SOFT_ENABLE     1
#undef M1_HFI_HAND_VH_FLOOR
#define M1_HFI_HAND_VH_FLOOR            (0.25f)
#undef M1_HFI_HAND_VH_END
#define M1_HFI_HAND_VH_END              (0.05f)
#undef M1_HFI_HAND_FADE_N
#define M1_HFI_HAND_FADE_N             16000u
#undef M1_HFI_HAND_VH0_N
#define M1_HFI_HAND_VH0_N              60000u
#undef M1_HFI_HAND_STOP_AFTER
#define M1_HFI_HAND_STOP_AFTER          0
#undef M1_HFI_HAND_REV_ENABLE
#define M1_HFI_HAND_REV_ENABLE          0
#undef M1_HFI_HAND_SPD_X_KILL_HI
#define M1_HFI_HAND_SPD_X_KILL_HI       (0.70f)
#undef M1_SPEED_OMEGA_RAMP_DECEL_RPM_S
#define M1_SPEED_OMEGA_RAMP_DECEL_RPM_S (20.0f) /* main vs 72: slower decel only */
#undef M1_HFI_RUN_LADDER_ENABLE
#define M1_HFI_RUN_LADDER_ENABLE        0
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 (1500.0f)
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               (36.0f)
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 (900.0f)
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               (55.0f) /* 30 s ramp @20 + hold */
#endif
#if M1_HFI_GATE == 77
/* Rel 72: after hand, VH_END=0 (no micro-floor inject). Single var vs 72.
 * Early 56/57 END=0 saw Iq noise; retest whether 1100 LOS clears. */
#undef M1_HFI_HAND_ID_OVERLAP_ENABLE
#define M1_HFI_HAND_ID_OVERLAP_ENABLE   0
#undef M1_HFI_HAND_OPEN_ID_ENABLE
#define M1_HFI_HAND_OPEN_ID_ENABLE      0
#undef M1_HFI_HAND_KILL_VH_ENABLE
#define M1_HFI_HAND_KILL_VH_ENABLE      1
#undef M1_HFI_HAND_VH0_SOFT_ENABLE
#define M1_HFI_HAND_VH0_SOFT_ENABLE     1
#undef M1_HFI_HAND_VH_FLOOR
#define M1_HFI_HAND_VH_FLOOR            (0.25f)
#undef M1_HFI_HAND_VH_END
#define M1_HFI_HAND_VH_END              (0.0f) /* main vs 72: true kill */
#undef M1_HFI_HAND_FADE_N
#define M1_HFI_HAND_FADE_N             16000u
#undef M1_HFI_HAND_VH0_N
#define M1_HFI_HAND_VH0_N              60000u
#undef M1_HFI_HAND_STOP_AFTER
#define M1_HFI_HAND_STOP_AFTER          0
#undef M1_HFI_HAND_REV_ENABLE
#define M1_HFI_HAND_REV_ENABLE          0
#undef M1_HFI_HAND_SPD_X_KILL_HI
#define M1_HFI_HAND_SPD_X_KILL_HI       (0.70f)
#undef M1_HFI_RUN_LADDER_ENABLE
#define M1_HFI_RUN_LADDER_ENABLE        0
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 (1500.0f)
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               (36.0f)
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 (900.0f)
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               (26.0f)
#endif
#if M1_HFI_GATE == 78
/* S1 / Rel 72: control identical. Observe-only VESC rpm window (mech):
 * want_smo set @>=1050, clear @<=950. No Park/VH/Id change. */
#undef M1_HFI_HAND_ID_OVERLAP_ENABLE
#define M1_HFI_HAND_ID_OVERLAP_ENABLE   0
#undef M1_HFI_HAND_OPEN_ID_ENABLE
#define M1_HFI_HAND_OPEN_ID_ENABLE      0
#undef M1_HFI_HAND_KILL_VH_ENABLE
#define M1_HFI_HAND_KILL_VH_ENABLE      1
#undef M1_HFI_HAND_VH0_SOFT_ENABLE
#define M1_HFI_HAND_VH0_SOFT_ENABLE     1
#undef M1_HFI_HAND_VH_FLOOR
#define M1_HFI_HAND_VH_FLOOR            (0.25f)
#undef M1_HFI_HAND_VH_END
#define M1_HFI_HAND_VH_END              (0.05f)
#undef M1_HFI_HAND_FADE_N
#define M1_HFI_HAND_FADE_N             16000u
#undef M1_HFI_HAND_VH0_N
#define M1_HFI_HAND_VH0_N              60000u
#undef M1_HFI_HAND_STOP_AFTER
#define M1_HFI_HAND_STOP_AFTER          0
#undef M1_HFI_HAND_REV_ENABLE
#define M1_HFI_HAND_REV_ENABLE          0
#undef M1_HFI_HAND_SPD_X_KILL_HI
#define M1_HFI_HAND_SPD_X_KILL_HI       (0.70f)
#undef M1_HFI_VESC_WIN_HI_RPM
#define M1_HFI_VESC_WIN_HI_RPM          (1050.0f)
#undef M1_HFI_VESC_WIN_LO_RPM
#define M1_HFI_VESC_WIN_LO_RPM          (950.0f)
#undef M1_HFI_RUN_LADDER_ENABLE
#define M1_HFI_RUN_LADDER_ENABLE        0
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 (1500.0f)
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               (36.0f)
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 (900.0f)
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               (26.0f)
#endif
#if M1_HFI_GATE == 79
/* S2 / Rel 78: same window + VH_END=0.05 hand path. When want_smo=1 and
 * hand=SMO: force inj_scale=0 and keep Id PI bypassed (≠GATE77: VH0 still
 * lands on micro-floor; kill only after rpm window). */
#undef M1_HFI_HAND_ID_OVERLAP_ENABLE
#define M1_HFI_HAND_ID_OVERLAP_ENABLE   0
#undef M1_HFI_HAND_OPEN_ID_ENABLE
#define M1_HFI_HAND_OPEN_ID_ENABLE      0
#undef M1_HFI_HAND_KILL_VH_ENABLE
#define M1_HFI_HAND_KILL_VH_ENABLE      1
#undef M1_HFI_HAND_VH0_SOFT_ENABLE
#define M1_HFI_HAND_VH0_SOFT_ENABLE     1
#undef M1_HFI_HAND_VH_FLOOR
#define M1_HFI_HAND_VH_FLOOR            (0.25f)
#undef M1_HFI_HAND_VH_END
#define M1_HFI_HAND_VH_END              (0.05f)
#undef M1_HFI_HAND_FADE_N
#define M1_HFI_HAND_FADE_N             16000u
#undef M1_HFI_HAND_VH0_N
#define M1_HFI_HAND_VH0_N              60000u
#undef M1_HFI_HAND_STOP_AFTER
#define M1_HFI_HAND_STOP_AFTER          0
#undef M1_HFI_HAND_REV_ENABLE
#define M1_HFI_HAND_REV_ENABLE          0
#undef M1_HFI_HAND_SPD_X_KILL_HI
#define M1_HFI_HAND_SPD_X_KILL_HI       (0.70f)
#undef M1_HFI_VESC_WIN_HI_RPM
#define M1_HFI_VESC_WIN_HI_RPM          (1050.0f)
#undef M1_HFI_VESC_WIN_LO_RPM
#define M1_HFI_VESC_WIN_LO_RPM          (950.0f)
#undef M1_HFI_RUN_LADDER_ENABLE
#define M1_HFI_RUN_LADDER_ENABLE        0
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 (1500.0f)
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               (36.0f)
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 (900.0f)
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               (26.0f)
#endif
#if M1_HFI_GATE == 80
/* S2b / Rel 79: soft handoff. want=1 & SMO: fade VH_END→0 while releasing
 * Id PI (bumpless id*→0). No OPEN_ID/IDUP — accel path ≡72 until window. */
#undef M1_HFI_HAND_ID_OVERLAP_ENABLE
#define M1_HFI_HAND_ID_OVERLAP_ENABLE   0
#undef M1_HFI_HAND_OPEN_ID_ENABLE
#define M1_HFI_HAND_OPEN_ID_ENABLE      0
#undef M1_HFI_HAND_KILL_VH_ENABLE
#define M1_HFI_HAND_KILL_VH_ENABLE      1
#undef M1_HFI_HAND_VH0_SOFT_ENABLE
#define M1_HFI_HAND_VH0_SOFT_ENABLE     1
#undef M1_HFI_HAND_VH_FLOOR
#define M1_HFI_HAND_VH_FLOOR            (0.25f)
#undef M1_HFI_HAND_VH_END
#define M1_HFI_HAND_VH_END              (0.05f)
#undef M1_HFI_HAND_FADE_N
#define M1_HFI_HAND_FADE_N             16000u
#undef M1_HFI_HAND_VH0_N
#define M1_HFI_HAND_VH0_N              60000u
#undef M1_HFI_HAND_STOP_AFTER
#define M1_HFI_HAND_STOP_AFTER          0
#undef M1_HFI_HAND_REV_ENABLE
#define M1_HFI_HAND_REV_ENABLE          0
#undef M1_HFI_HAND_SPD_X_KILL_HI
#define M1_HFI_HAND_SPD_X_KILL_HI       (0.70f)
#undef M1_HFI_VESC_WIN_HI_RPM
#define M1_HFI_VESC_WIN_HI_RPM          (1050.0f)
#undef M1_HFI_VESC_WIN_LO_RPM
#define M1_HFI_VESC_WIN_LO_RPM          (950.0f)
#undef M1_HFI_VESC_ID_HANDOFF_ENABLE
#define M1_HFI_VESC_ID_HANDOFF_ENABLE   1
#undef M1_HFI_VESC_HANDOFF_N
#define M1_HFI_VESC_HANDOFF_N          10000u /* 0.5 s @20 kHz: VH↓ + Id soft↑ */
#undef M1_HFI_RUN_LADDER_ENABLE
#define M1_HFI_RUN_LADDER_ENABLE        0
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 (1500.0f)
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               (36.0f)
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 (900.0f)
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               (26.0f)
#endif
#if M1_HFI_GATE == 81
/* Rel C4x FEED (control == 41/42): after iq_auth_ok soft-open Id to 0,
 * VH stays 1.0, SMO_HAND stays 0. Capture still uses ID_PI_OFF bypass. */
#undef M1_HFI_HFI_ID_SOFT_ENABLE
#define M1_HFI_HFI_ID_SOFT_ENABLE       1
#undef M1_HFI_ID_PI_SOFT_N
#define M1_HFI_ID_PI_SOFT_N             20000u /* 1.0 s @20 kHz */
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               15.0f /* FEED soak a bit longer */
#endif
#if M1_HFI_GATE == 82
/* Rel 81 / C4x FEED: Id on from RUN entry (Id*=0, soft=1), no ramp.
 * Baseline for demod-with-Id-on. VH full, no SMO. */
#undef M1_HFI_ID_ON_FROM_RUN_ENABLE
#define M1_HFI_ID_ON_FROM_RUN_ENABLE    1
#undef M1_HFI_HFI_ID_SOFT_ENABLE
#define M1_HFI_HFI_ID_SOFT_ENABLE       0
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               15.0f
#endif
#if M1_HFI_GATE == 83
/* Rel 82: after FEED starts, keep Iq* even if auth drops (no 0↔0.8 chop). */
#undef M1_HFI_ID_ON_FROM_RUN_ENABLE
#define M1_HFI_ID_ON_FROM_RUN_ENABLE    1
#undef M1_HFI_HFI_ID_SOFT_ENABLE
#define M1_HFI_HFI_ID_SOFT_ENABLE       0
#undef M1_HFI_IQ_AUTH_FEED_HOLD
#define M1_HFI_IQ_AUTH_FEED_HOLD        1
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               15.0f
#endif
#if M1_HFI_GATE == 84
/* Rel 83 control; VOFA = edge demod intermediates for PC offline replay. */
#undef M1_HFI_ID_ON_FROM_RUN_ENABLE
#define M1_HFI_ID_ON_FROM_RUN_ENABLE    1
#undef M1_HFI_HFI_ID_SOFT_ENABLE
#define M1_HFI_HFI_ID_SOFT_ENABLE       0
#undef M1_HFI_IQ_AUTH_FEED_HOLD
#define M1_HFI_IQ_AUTH_FEED_HOLD        1
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               15.0f
#endif
#if M1_HFI_GATE == 85
/* Rel 84: Id PI feedback = LPF(Id); demod/HFI still raw Id.
 * Aim: Ud_pi tracks DC Id only → closer to old ID_PI_OFF + FEED 0.8. */
#undef M1_HFI_ID_ON_FROM_RUN_ENABLE
#define M1_HFI_ID_ON_FROM_RUN_ENABLE    1
#undef M1_HFI_HFI_ID_SOFT_ENABLE
#define M1_HFI_HFI_ID_SOFT_ENABLE       0
#undef M1_HFI_IQ_AUTH_FEED_HOLD
#define M1_HFI_IQ_AUTH_FEED_HOLD        1
#undef M1_HFI_ID_PI_LPF_ENABLE
#define M1_HFI_ID_PI_LPF_ENABLE         1
#undef M1_HFI_ID_PI_LPF_A
#define M1_HFI_ID_PI_LPF_A              (0.05f) /* ~160 Hz @20 kHz */
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               15.0f
#endif
#if M1_HFI_GATE == 86
/* Rel 84 control (Id on, FEED_HOLD, raw Id PI). Demod HP Id/Iq before di. */
#undef M1_HFI_ID_ON_FROM_RUN_ENABLE
#define M1_HFI_ID_ON_FROM_RUN_ENABLE    1
#undef M1_HFI_HFI_ID_SOFT_ENABLE
#define M1_HFI_HFI_ID_SOFT_ENABLE       0
#undef M1_HFI_IQ_AUTH_FEED_HOLD
#define M1_HFI_IQ_AUTH_FEED_HOLD        1
#undef M1_HFI_ID_PI_LPF_ENABLE
#define M1_HFI_ID_PI_LPF_ENABLE         0
#undef M1_HFI_DEMOD_HP_ENABLE
#define M1_HFI_DEMOD_HP_ENABLE          1
#undef M1_HFI_DEMOD_HP_A
#define M1_HFI_DEMOD_HP_A               (0.05f) /* ~160 Hz; drop FEED slow Id/Iq */
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               15.0f
#endif
#if M1_HFI_GATE == 87
/* Rel 84 control. Demod: Δiαβ then Park(θ̂). HP off (86 NO_GAIN). */
#undef M1_HFI_ID_ON_FROM_RUN_ENABLE
#define M1_HFI_ID_ON_FROM_RUN_ENABLE    1
#undef M1_HFI_HFI_ID_SOFT_ENABLE
#define M1_HFI_HFI_ID_SOFT_ENABLE       0
#undef M1_HFI_IQ_AUTH_FEED_HOLD
#define M1_HFI_IQ_AUTH_FEED_HOLD        1
#undef M1_HFI_ID_PI_LPF_ENABLE
#define M1_HFI_ID_PI_LPF_ENABLE         0
#undef M1_HFI_DEMOD_HP_ENABLE
#define M1_HFI_DEMOD_HP_ENABLE          0
#undef M1_HFI_DEMOD_AB_ENABLE
#define M1_HFI_DEMOD_AB_ENABLE          1
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               15.0f
#endif
#if M1_HFI_GATE == 88
/* Rel 84 dq demod. Hat-axis ±Vh in αβ, Park into Ud/Uq for SVPWM. */
#undef M1_HFI_ID_ON_FROM_RUN_ENABLE
#define M1_HFI_ID_ON_FROM_RUN_ENABLE    1
#undef M1_HFI_HFI_ID_SOFT_ENABLE
#define M1_HFI_HFI_ID_SOFT_ENABLE       0
#undef M1_HFI_IQ_AUTH_FEED_HOLD
#define M1_HFI_IQ_AUTH_FEED_HOLD        1
#undef M1_HFI_ID_PI_LPF_ENABLE
#define M1_HFI_ID_PI_LPF_ENABLE         0
#undef M1_HFI_DEMOD_HP_ENABLE
#define M1_HFI_DEMOD_HP_ENABLE          0
#undef M1_HFI_DEMOD_AB_ENABLE
#define M1_HFI_DEMOD_AB_ENABLE          0
#undef M1_HFI_INJECT_AB_ENABLE
#define M1_HFI_INJECT_AB_ENABLE         1
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               15.0f
#endif
#if M1_HFI_GATE == 89
/* Rel 84: Id PI never bypassed (incl. LOG/kick). Id*=0. No αβ inject. */
#undef M1_HFI_ID_ON_FROM_RUN_ENABLE
#define M1_HFI_ID_ON_FROM_RUN_ENABLE    1
#undef M1_HFI_HFI_ID_SOFT_ENABLE
#define M1_HFI_HFI_ID_SOFT_ENABLE       0
#undef M1_HFI_IQ_AUTH_FEED_HOLD
#define M1_HFI_IQ_AUTH_FEED_HOLD        1
#undef M1_HFI_ID_PI_OFF_ENABLE
#define M1_HFI_ID_PI_OFF_ENABLE         0
#undef M1_HFI_ID_PI_LPF_ENABLE
#define M1_HFI_ID_PI_LPF_ENABLE         0
#undef M1_HFI_DEMOD_HP_ENABLE
#define M1_HFI_DEMOD_HP_ENABLE          0
#undef M1_HFI_DEMOD_AB_ENABLE
#define M1_HFI_DEMOD_AB_ENABLE          0
#undef M1_HFI_INJECT_AB_ENABLE
#define M1_HFI_INJECT_AB_ENABLE         0
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               15.0f
#endif
#if M1_HFI_GATE == 90
/* Rel 84 control (Id on RUN, FEED_HOLD, ID_PI_OFF=1 as C4x). Demod: αβ
 * midpoint 0.5*(now+older)-prev then Park(mid θ̂). Not 89, not 87. */
#undef M1_HFI_ID_ON_FROM_RUN_ENABLE
#define M1_HFI_ID_ON_FROM_RUN_ENABLE    1
#undef M1_HFI_HFI_ID_SOFT_ENABLE
#define M1_HFI_HFI_ID_SOFT_ENABLE       0
#undef M1_HFI_IQ_AUTH_FEED_HOLD
#define M1_HFI_IQ_AUTH_FEED_HOLD        1
#undef M1_HFI_ID_PI_OFF_ENABLE
#define M1_HFI_ID_PI_OFF_ENABLE         1
#undef M1_HFI_ID_PI_LPF_ENABLE
#define M1_HFI_ID_PI_LPF_ENABLE         0
#undef M1_HFI_DEMOD_HP_ENABLE
#define M1_HFI_DEMOD_HP_ENABLE          0
#undef M1_HFI_DEMOD_AB_ENABLE
#define M1_HFI_DEMOD_AB_ENABLE          0
#undef M1_HFI_DEMOD_AB_MID_ENABLE
#define M1_HFI_DEMOD_AB_MID_ENABLE      1
#undef M1_HFI_INJECT_AB_ENABLE
#define M1_HFI_INJECT_AB_ENABLE         0
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               15.0f
#endif
#if M1_HFI_GATE == 91
/* Rel 72 accel hand. Only: SMO u−u_hfi, HFI 段不停观测. */
#undef M1_HFI_HAND_ID_OVERLAP_ENABLE
#define M1_HFI_HAND_ID_OVERLAP_ENABLE   0
#undef M1_HFI_HAND_OPEN_ID_ENABLE
#define M1_HFI_HAND_OPEN_ID_ENABLE      0
#undef M1_HFI_HAND_KILL_VH_ENABLE
#define M1_HFI_HAND_KILL_VH_ENABLE      1
#undef M1_HFI_HAND_VH0_SOFT_ENABLE
#define M1_HFI_HAND_VH0_SOFT_ENABLE     1
#undef M1_HFI_HAND_VH_FLOOR
#define M1_HFI_HAND_VH_FLOOR            (0.25f)
#undef M1_HFI_HAND_VH_END
#define M1_HFI_HAND_VH_END              (0.05f)
#undef M1_HFI_HAND_FADE_N
#define M1_HFI_HAND_FADE_N             16000u
#undef M1_HFI_HAND_VH0_N
#define M1_HFI_HAND_VH0_N              60000u
#undef M1_HFI_HAND_STOP_AFTER
#define M1_HFI_HAND_STOP_AFTER          0
#undef M1_HFI_HAND_REV_ENABLE
#define M1_HFI_HAND_REV_ENABLE          0
#undef M1_HFI_HAND_SPD_X_KILL_HI
#define M1_HFI_HAND_SPD_X_KILL_HI       (0.70f)
#undef M1_HFI_RUN_LADDER_ENABLE
#define M1_HFI_RUN_LADDER_ENABLE        0
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 (1500.0f)
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               (36.0f)
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 (900.0f)
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               (26.0f)
#undef M1_HFI_ID_ON_FROM_RUN_ENABLE
#define M1_HFI_ID_ON_FROM_RUN_ENABLE    0
#undef M1_HFI_HFI_ID_SOFT_ENABLE
#define M1_HFI_HFI_ID_SOFT_ENABLE       0
#undef M1_HFI_DEMOD_AB_MID_ENABLE
#define M1_HFI_DEMOD_AB_MID_ENABLE      0
#undef M1_HFI_DEMOD_AB_ENABLE
#define M1_HFI_DEMOD_AB_ENABLE          0
#undef M1_HFI_DEMOD_HP_ENABLE
#define M1_HFI_DEMOD_HP_ENABLE          0
#undef M1_HFI_INJECT_AB_ENABLE
#define M1_HFI_INJECT_AB_ENABLE         0
#undef M1_HFI_SMO_SUB_VH_ENABLE
#define M1_HFI_SMO_SUB_VH_ENABLE        1
#endif
#if M1_HFI_GATE == 92
/* Rel 91: one-shot rotate Id/Iq PI when Park fully switches to SMO. */
#undef M1_HFI_HAND_ID_OVERLAP_ENABLE
#define M1_HFI_HAND_ID_OVERLAP_ENABLE   0
#undef M1_HFI_HAND_OPEN_ID_ENABLE
#define M1_HFI_HAND_OPEN_ID_ENABLE      0
#undef M1_HFI_HAND_KILL_VH_ENABLE
#define M1_HFI_HAND_KILL_VH_ENABLE      1
#undef M1_HFI_HAND_VH0_SOFT_ENABLE
#define M1_HFI_HAND_VH0_SOFT_ENABLE     1
#undef M1_HFI_HAND_VH_FLOOR
#define M1_HFI_HAND_VH_FLOOR            (0.25f)
#undef M1_HFI_HAND_VH_END
#define M1_HFI_HAND_VH_END              (0.05f)
#undef M1_HFI_HAND_FADE_N
#define M1_HFI_HAND_FADE_N             16000u
#undef M1_HFI_HAND_VH0_N
#define M1_HFI_HAND_VH0_N              60000u
#undef M1_HFI_HAND_STOP_AFTER
#define M1_HFI_HAND_STOP_AFTER          0
#undef M1_HFI_HAND_REV_ENABLE
#define M1_HFI_HAND_REV_ENABLE          0
#undef M1_HFI_HAND_SPD_X_KILL_HI
#define M1_HFI_HAND_SPD_X_KILL_HI       (0.70f)
#undef M1_HFI_RUN_LADDER_ENABLE
#define M1_HFI_RUN_LADDER_ENABLE        0
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 (1500.0f)
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               (36.0f)
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 (900.0f)
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               (26.0f)
#undef M1_HFI_ID_ON_FROM_RUN_ENABLE
#define M1_HFI_ID_ON_FROM_RUN_ENABLE    0
#undef M1_HFI_HFI_ID_SOFT_ENABLE
#define M1_HFI_HFI_ID_SOFT_ENABLE       0
#undef M1_HFI_DEMOD_AB_MID_ENABLE
#define M1_HFI_DEMOD_AB_MID_ENABLE      0
#undef M1_HFI_DEMOD_AB_ENABLE
#define M1_HFI_DEMOD_AB_ENABLE          0
#undef M1_HFI_DEMOD_HP_ENABLE
#define M1_HFI_DEMOD_HP_ENABLE          0
#undef M1_HFI_INJECT_AB_ENABLE
#define M1_HFI_INJECT_AB_ENABLE         0
#undef M1_HFI_SMO_SUB_VH_ENABLE
#define M1_HFI_SMO_SUB_VH_ENABLE        1
#undef M1_HFI_ROTATE_PI_ENABLE
#define M1_HFI_ROTATE_PI_ENABLE         1
#endif
#if M1_HFI_GATE == 93
/* Rel 92：文档§7 步1。HFI/交接 Id PI 不旁路，Id*=0。不改解调、不 IDUP。 */
#undef M1_HFI_HAND_ID_OVERLAP_ENABLE
#define M1_HFI_HAND_ID_OVERLAP_ENABLE   0
#undef M1_HFI_HAND_OPEN_ID_ENABLE
#define M1_HFI_HAND_OPEN_ID_ENABLE      0
#undef M1_HFI_HAND_KILL_VH_ENABLE
#define M1_HFI_HAND_KILL_VH_ENABLE      1
#undef M1_HFI_HAND_VH0_SOFT_ENABLE
#define M1_HFI_HAND_VH0_SOFT_ENABLE     1
#undef M1_HFI_HAND_VH_FLOOR
#define M1_HFI_HAND_VH_FLOOR            (0.25f)
#undef M1_HFI_HAND_VH_END
#define M1_HFI_HAND_VH_END              (0.05f)
#undef M1_HFI_HAND_FADE_N
#define M1_HFI_HAND_FADE_N             16000u
#undef M1_HFI_HAND_VH0_N
#define M1_HFI_HAND_VH0_N              60000u
#undef M1_HFI_HAND_STOP_AFTER
#define M1_HFI_HAND_STOP_AFTER          0
#undef M1_HFI_HAND_REV_ENABLE
#define M1_HFI_HAND_REV_ENABLE          0
#undef M1_HFI_HAND_SPD_X_KILL_HI
#define M1_HFI_HAND_SPD_X_KILL_HI       (0.70f)
#undef M1_HFI_RUN_LADDER_ENABLE
#define M1_HFI_RUN_LADDER_ENABLE        0
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 (1500.0f)
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               (36.0f)
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 (900.0f)
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               (26.0f)
#undef M1_HFI_ID_ON_FROM_RUN_ENABLE
#define M1_HFI_ID_ON_FROM_RUN_ENABLE    1
#undef M1_HFI_ID_PI_OFF_ENABLE
#define M1_HFI_ID_PI_OFF_ENABLE         0
#undef M1_HFI_HFI_ID_SOFT_ENABLE
#define M1_HFI_HFI_ID_SOFT_ENABLE       0
#undef M1_HFI_DEMOD_AB_MID_ENABLE
#define M1_HFI_DEMOD_AB_MID_ENABLE      0
#undef M1_HFI_DEMOD_AB_ENABLE
#define M1_HFI_DEMOD_AB_ENABLE          0
#undef M1_HFI_DEMOD_HP_ENABLE
#define M1_HFI_DEMOD_HP_ENABLE          0
#undef M1_HFI_INJECT_AB_ENABLE
#define M1_HFI_INJECT_AB_ENABLE         0
#undef M1_HFI_SMO_SUB_VH_ENABLE
#define M1_HFI_SMO_SUB_VH_ENABLE        1
#undef M1_HFI_ROTATE_PI_ENABLE
#define M1_HFI_ROTATE_PI_ENABLE         1
#endif
#if M1_HFI_GATE == 41
#undef M1_HFI_DEMOD_PROBE_ENABLE
#define M1_HFI_DEMOD_PROBE_ENABLE       1
#undef M1_HFI_DEMOD_FREEZE_T0_S
#define M1_HFI_DEMOD_FREEZE_T0_S        (1.20f)
#undef M1_HFI_DEMOD_FREEZE_T1_S
#define M1_HFI_DEMOD_FREEZE_T1_S        (1.60f)
#undef M1_HFI_DEMOD_FREEZE_T2_S
#define M1_HFI_DEMOD_FREEZE_T2_S        (6.00f)
#undef M1_HFI_DEMOD_FREEZE_T3_S
#define M1_HFI_DEMOD_FREEZE_T3_S        (6.40f)
#endif
#if M1_HFI_GATE == 43
/* 唯一增量：踢�?RUN 6.0�?.0 s 强制 Iq*=0（惯性滑行），前后仍 FEED_A */
#undef M1_HFI_FEED_COAST_ENABLE
#define M1_HFI_FEED_COAST_ENABLE        1
#undef M1_HFI_FEED_COAST_T0_S
#define M1_HFI_FEED_COAST_T0_S          (6.00f)
#undef M1_HFI_FEED_COAST_T1_S
#define M1_HFI_FEED_COAST_T1_S          (8.00f)
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               12.0f /* 滑行后再�?4 s FEED */
#endif

#elif (M1_HFI_GATE == 39) || (M1_HFI_GATE == 40)
/* C4y(39)：相�?C4x 唯一增量 = FEED_A 0.8�?.8�?
 * C4z(40)：控制同 C4y；另开两段冻角 + 解调遥测。回退 39�?*/
#undef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE       0
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              1
#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             5.0f
#undef M1_HFI_RUN_LADDER_ENABLE
#define M1_HFI_RUN_LADDER_ENABLE        0
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 0.0f
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               10.0f
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 0.0f
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               0.0f
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (-1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (-1.0f)
#undef M1_HFI_A_CMD
#define M1_HFI_A_CMD                    (0.10f)
#undef M1_HFI_PLL_INIT_OFF_RAD
#define M1_HFI_PLL_INIT_OFF_RAD         0.0f
#undef M1_HFI_INIT_FROM_ENC
#define M1_HFI_INIT_FROM_ENC            1
#undef M1_HFI_POLARITY_IPD_ENABLE
#define M1_HFI_POLARITY_IPD_ENABLE      0
#undef M1_HFI_PLL_W_MAX
#define M1_HFI_PLL_W_MAX                1100.0f
#undef M1_HFI_PLL_INT_MAX
#define M1_HFI_PLL_INT_MAX              M1_HFI_PLL_W_MAX
#undef M1_HFI_OMEGA_FF_SRC
#define M1_HFI_OMEGA_FF_SRC             0
#undef M1_HFI_OMEGA_FF_FROM_REF
#define M1_HFI_OMEGA_FF_FROM_REF        0
#undef M1_HFI_OMEGA_SEED_ENABLE
#define M1_HFI_OMEGA_SEED_ENABLE        0
#undef M1_HFI_SPEED_FB_ENABLE
#define M1_HFI_SPEED_FB_ENABLE          0
#undef M1_HFI_QKICK_AFTER_LOCK_ENABLE
#define M1_HFI_QKICK_AFTER_LOCK_ENABLE  1
#undef M1_HFI_QKICK_BEFORE_HFI_ENABLE
#define M1_HFI_QKICK_BEFORE_HFI_ENABLE  0
#undef M1_HFI_QKICK_THEN_HFI_ENABLE
#define M1_HFI_QKICK_THEN_HFI_ENABLE    1
#undef M1_HFI_QKICK_SPEED_ENABLE
#define M1_HFI_QKICK_SPEED_ENABLE       0
#undef M1_HFI_QKICK_PRE_S
#define M1_HFI_QKICK_PRE_S              2.0f
#undef M1_HFI_QKICK_IQ_A
#define M1_HFI_QKICK_IQ_A               1.6f
#undef M1_HFI_QKICK_KICK_N
#define M1_HFI_QKICK_KICK_N             6000u
#undef M1_HFI_QKICK_BRAKE_ENABLE
#define M1_HFI_QKICK_BRAKE_ENABLE       0
#undef M1_HFI_QKICK_BRAKE_N
#define M1_HFI_QKICK_BRAKE_N            6000u
#undef M1_HFI_QKICK_HOLD_S
#define M1_HFI_QKICK_HOLD_S             0.4f
#undef M1_HFI_QKICK_CRAWL_ENABLE
#define M1_HFI_QKICK_CRAWL_ENABLE       0
#undef M1_HFI_QKICK_START_ENABLE
#define M1_HFI_QKICK_START_ENABLE       0
#undef M1_HFI_ID_PI_OFF_ENABLE
#define M1_HFI_ID_PI_OFF_ENABLE         1
#undef M1_HFI_QKICK_FORCE_PI
#define M1_HFI_QKICK_FORCE_PI           0
#undef M1_HFI_QKICK_PRE_GATE_ENABLE
#define M1_HFI_QKICK_PRE_GATE_ENABLE    1
#undef M1_HFI_IQ_AUTH_ENABLE
#define M1_HFI_IQ_AUTH_ENABLE           1
#undef M1_HFI_IQ_AUTH_X_GOOD
#define M1_HFI_IQ_AUTH_X_GOOD           (0.218f)
#undef M1_HFI_IQ_AUTH_X_BAD
#define M1_HFI_IQ_AUTH_X_BAD            (0.205f)
#undef M1_HFI_IQ_AUTH_EPS_FALSE_MAX
#define M1_HFI_IQ_AUTH_EPS_FALSE_MAX    (0.20f)
#undef M1_HFI_IQ_AUTH_HOLD_N
#define M1_HFI_IQ_AUTH_HOLD_N           2000u
#undef M1_HFI_IQ_AUTH_CLEAR_N
#define M1_HFI_IQ_AUTH_CLEAR_N          1000u
#undef M1_HFI_IQ_AUTH_IQ_LO
#define M1_HFI_IQ_AUTH_IQ_LO            (0.0f)
#undef M1_HFI_IQ_AUTH_IQ_HI
#define M1_HFI_IQ_AUTH_IQ_HI            (0.0f)
#undef M1_HFI_IQ_AUTH_SLEW_A_S
#define M1_HFI_IQ_AUTH_SLEW_A_S         (10.0f)
#undef M1_HFI_IQ_AUTH_FEED_ENABLE
#define M1_HFI_IQ_AUTH_FEED_ENABLE      1
#undef M1_HFI_IQ_AUTH_FEED_SIGN
#define M1_HFI_IQ_AUTH_FEED_SIGN        (1.0f)
#undef M1_HFI_IQ_AUTH_FEED_A
#define M1_HFI_IQ_AUTH_FEED_A           (1.80f) /* 唯一增量�?.8�?.8 */
#undef M1_HFI_IQ_AUTH_FEED_DELAY_S
#define M1_HFI_IQ_AUTH_FEED_DELAY_S     (0.20f)
#undef M1_HFI_IQ_AUTH_FEED_RAMP_S
#define M1_HFI_IQ_AUTH_FEED_RAMP_S      (0.0f)
#undef M1_HFI_IQ_AUTH_FEED_LEGACY_BRANCH
#define M1_HFI_IQ_AUTH_FEED_LEGACY_BRANCH 0
#if M1_HFI_GATE == 40
/* C4z：控制同 C4y。RUN �?1.2�?.6 s�?.0�?.4 s �?θ̂，解调仍算�?*/
#undef M1_HFI_DEMOD_PROBE_ENABLE
#define M1_HFI_DEMOD_PROBE_ENABLE       1
#undef M1_HFI_DEMOD_FREEZE_T0_S
#define M1_HFI_DEMOD_FREEZE_T0_S        (1.20f)
#undef M1_HFI_DEMOD_FREEZE_T1_S
#define M1_HFI_DEMOD_FREEZE_T1_S        (1.60f)
#undef M1_HFI_DEMOD_FREEZE_T2_S
#define M1_HFI_DEMOD_FREEZE_T2_S        (6.00f)
#undef M1_HFI_DEMOD_FREEZE_T3_S
#define M1_HFI_DEMOD_FREEZE_T3_S        (6.40f)
#endif

#endif /* M1_HFI_GATE */

#endif /* CONFIG_PROFILES_M1_HFI_STANDSTILL_PROFILE_H */
