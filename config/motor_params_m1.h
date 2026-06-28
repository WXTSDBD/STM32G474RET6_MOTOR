/**
 * @file motor_params_m1.h
 * @brief M1 电机硬件常数：电流采样链路的标度（初始化预计算，热路径只乘不除）。
 *
 * ADC2 三相：外部放大 ×10，采样电阻 10 mΩ，12bit 单端 @ VDDA。
 */

#ifndef MOTOR_PARAMS_M1_H
#define MOTOR_PARAMS_M1_H

/** N5065 极对数 */
#define M1_POLE_PAIRS       7u

/** VDDA / ADC 参考（V） */
#define M1_ADC_VREF_V       3.3f

/** 相电流采样电阻（Ω），10 mΩ */
#define M1_ADC_SHUNT_OHM    0.01f

/** 外部电流放大倍数 */
#define M1_ADC_AMP_GAIN     10.0f

/** 安培/LSB：Vref / (4096 × R_shunt × Gain)，热路径仅做 (raw-offset)*scale */
#define M1_ADC_SCALE_A_LSB  (M1_ADC_VREF_V / (4096.0f * M1_ADC_SHUNT_OHM * M1_ADC_AMP_GAIN))

/** 物理 JDR rank0/1/2 per-channel gain；统一 1.0（扇区采样/KCL 修后再标定，勿用 run RMS 凑） */
#ifndef M1_ADC_GAIN_CH0
#define M1_ADC_GAIN_CH0     1.00f
#endif
#ifndef M1_ADC_GAIN_CH1
#define M1_ADC_GAIN_CH1     1.00f
#endif
#ifndef M1_ADC_GAIN_CH2
#define M1_ADC_GAIN_CH2     1.00f
#endif

/* --- 电气参数（LCR @ 1 kHz，AB 线） --- */
#define M1_RS_OHM           0.115f
#define M1_LD_H             59e-6f
#define M1_LQ_H             87e-6f

/** JEOC 电流环节拍（s） */
#define M1_CTRL_TS_S        50e-6f

/** 1=Park/VOFA 用 -θ_enc；0=与 SVPWM 同 +θ（与 Core/Inc/main.h 同名宏兼容） */
#ifndef M1_THETA_NEGATE
#define M1_THETA_NEGATE     0
#endif

/** 母线电压（V），限幅用 Vbus/√3 */
#define M1_VBUS_V           24.0f
#define M1_PI_V_MAX         (M1_VBUS_V * 0.577350269f)
#define M1_PI_V_MIN         (-M1_PI_V_MAX)

/** PI 带宽（Hz）；M1_PI_USE_FIXED_GAIN=0 时用于 L×ωc 整定（阶跃验收参考 1000 Hz） */
#define M1_PI_FC_HZ         1000.0f
#define M1_PI_WC_RADS       (2.0f * 3.14159265359f * M1_PI_FC_HZ)

/** 联调：1=固定 Kp/Ki（试凑）；0=按 fc 整定 */
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
 * 首次闭环联调（24V/0.8A 电源）：降低 dq 电压限幅，ref 硬钳位。
 * 稳定后设 M1_CLOSURE_BRINGUP=0 恢复满幅 SVPWM 线性区。
 */
#ifndef M1_CLOSURE_BRINGUP
#define M1_CLOSURE_BRINGUP  1
#endif

#if M1_CLOSURE_BRINGUP
#define M1_PI_V_LIMIT_V     6.0f
#define M1_I_REF_ABS_MAX    0.5f
#else
#define M1_PI_V_LIMIT_V     M1_PI_V_MAX
#define M1_I_REF_ABS_MAX    2.0f
#endif

#define M1_PI_V_LIMIT_MIN   (-M1_PI_V_LIMIT_V)

/** 积分器限幅（V），与输出限幅同量级，防 windup */
#define M1_PI_INT_LIMIT_V   M1_PI_V_LIMIT_V
#define M1_PI_INT_LIMIT_MIN (-M1_PI_INT_LIMIT_V)

/** 电流环 Iq 目标（A）；无启动策略时上电即用，手拨启动（Iq 探路/日常：0.5 A） */
#ifndef M1_IQ_REF_A
#define M1_IQ_REF_A         0.5f
#endif

/** 0=无 ALIGN/DRAG，编码器 θ 直接闭环；1=启动状态机 */
#ifndef M1_STARTUP_ENABLE
#define M1_STARTUP_ENABLE   0
#endif

/* ==========================================================================
 * 联调模式（只改这一处，其余 M1_ID_* / M1_IDENT_* 由下面自动推导）
 *
 *   M1_BRINGUP_MODE_NORMAL              日常 Iq 环，无自动序列
 *   M1_BRINGUP_MODE_IDENT_IQ_STEP       堵转：Id Pass0 建表 → 3×Bode(OFF/FIXED/LUT)
 *   M1_BRINGUP_MODE_ID_CAL_DUAL_FULL    Pass0 双角 30°+0° → commit → Iq 探路 OFF→LUT ON
 *   M1_BRINGUP_MODE_ID_CAL_PASS0_ONLY   Pass0 双角 only（无 Pass1 / 无 Iq 探路）
 *   M1_BRINGUP_MODE_MULTI_ANGLE_PASS0   Pass0 五角 0/30/60/90/120° only（多角度 raw 录波）
 * ========================================================================== */
#define M1_BRINGUP_MODE_NORMAL                0
#define M1_BRINGUP_MODE_IDENT_IQ_STEP         1
#define M1_BRINGUP_MODE_ID_CAL_DUAL_FULL      2
#define M1_BRINGUP_MODE_ID_CAL_PASS0_ONLY     3
#define M1_BRINGUP_MODE_MULTI_ANGLE_PASS0     4

#ifndef M1_BRINGUP_MODE
#define M1_BRINGUP_MODE  M1_BRINGUP_MODE_IDENT_IQ_STEP
#endif

#if (M1_BRINGUP_MODE == M1_BRINGUP_MODE_NORMAL)
#define M1_ID_LOCK_CAL_SWEEP            0
#define M1_IDENT_ENABLE                 0
#define M1_ID_CAL_MULTI_ANGLE_ENABLE    0
#define M1_ID_CAL_PASS0_ONLY_ENABLE     0
#define M1_ID_CAL_LUT_VERIFY_SWEEP     0
#define M1_ID_CAL_IQ_PROBE_ENABLE       0
/** 产品：两簇 plut + NVM 上电 LUT ON（标定一次后切 NORMAL） */
#ifndef M1_DEADBAND_GEO_TWO_CLUSTER_ENABLE
#define M1_DEADBAND_GEO_TWO_CLUSTER_ENABLE  1
#endif
#ifndef M1_DEADBAND_LUT_RUNTIME_SCALE
#define M1_DEADBAND_LUT_RUNTIME_SCALE   0.25f
#endif
#ifndef M1_DEADBAND_NVM_ON_BOOT
#define M1_DEADBAND_NVM_ON_BOOT         0
#endif
#define M1_DEADBAND_LUT_BAKED_ENABLE    1
#if M1_DEADBAND_LUT_BAKED_ENABLE
#ifndef M1_DEADBAND_GEO_TWO_CLUSTER_ENABLE
#define M1_DEADBAND_GEO_TWO_CLUSTER_ENABLE  0
#endif
#define M1_DEADBAND_LUT_RUNTIME_SCALE   1.0f
#endif
#ifndef M1_DEADBAND_ENABLE
#define M1_DEADBAND_ENABLE              1
#endif

#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_IDENT_IQ_STEP)
/** Id Pass0 建表 → commit → 3×Bode( OFF / FIXED / LUT 现场表 )；无 Iq 阶跃 */
#define M1_IDENT_ID_CAL_BEFORE_STEP     1
#define M1_ID_LOCK_CAL_SWEEP            1
#define M1_IDENT_ENABLE                 1
#define M1_ID_CAL_MULTI_ANGLE_ENABLE    0
#define M1_ID_CAL_PASS0_ONLY_ENABLE     0
#define M1_ID_CAL_LUT_VERIFY_SWEEP      0
#define M1_ID_CAL_IQ_PROBE_ENABLE       0
#define M1_ID_CAL_DUAL_ANGLE_ENABLE     0
#define M1_ID_CAL_FIX_THETA_ENABLE      0
#define M1_ID_CAL_ALIGN_ENABLE          0
#define M1_ID_CAL_COMMIT_LUT            1
#define M1_IDENT_IQ_STEP_ENABLE         0
#define M1_IDENT_IQ_BODE_ENABLE         1
#define M1_IDENT_BODE_ROUNDS            3u
#define M1_IDENT_BODE_OFF_ROUNDS        1u
#define M1_IDENT_BODE_FIXED_ROUNDS      1u
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
#define M1_IDENT_POST_BODE_OPEN_UQ_ENABLE  1
#define M1_IDENT_POST_BODE_OPEN_UQ_V       2.0f
#define M1_IDENT_POST_BODE_OPEN_UQ_S       5.0f

#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_ID_CAL_DUAL_FULL)
#define M1_ID_LOCK_CAL_SWEEP            1
#define M1_IDENT_ENABLE                 0
#define M1_ID_CAL_MULTI_ANGLE_ENABLE    0
#define M1_ID_CAL_PASS0_ONLY_ENABLE     0
#define M1_ID_CAL_LUT_VERIFY_SWEEP      0   /* Pass1 Ud 验表已签收，默认跳过 */
#define M1_ID_CAL_IQ_PROBE_ENABLE       1
#define M1_ID_CAL_IQ_PROBE_OFF_S        10.0f  /* Iq 探路：LUT OFF（open_seq 50） */
#define M1_ID_CAL_IQ_PROBE_FIXED_S      5.0f   /* Iq 探路：FIXED ON（open_seq 53） */
#define M1_ID_CAL_IQ_PROBE_ID_PI_ENABLE 1   /* Iq 段 Id 环拒扰，公平验 phase LUT */
/** 第一档 A：d 表 + 当前 θ 反 Park→abc duty；1603 过补，先关回 plut+scale */
#define M1_DEADBAND_RUNTIME_GEO_ENABLE    0
/** Phase 1：30°/0° 分簇 plut，runtime 按 θ mod 60° 选表；对比 1608 单表 */
#define M1_DEADBAND_GEO_TWO_CLUSTER_ENABLE  1
/** 论文 scale=1：幅度在 commit 归一化写入 plut val */
#define M1_DEADBAND_LUT_RUNTIME_SCALE   1.0f
#ifndef M1_DEADBAND_LUT_COMMIT_NORMALIZE
#define M1_DEADBAND_LUT_COMMIT_NORMALIZE  1
#endif
/** commit 后写 Flash（与 FOC 互斥，后续加状态机再开）；完整标定测试阶段关 */
#ifndef M1_DEADBAND_NVM_COMMIT_ENABLE
#define M1_DEADBAND_NVM_COMMIT_ENABLE   0
#endif
/** 0=归一化在 commit；1=旧路径 runtime AUTO scale */
#ifndef M1_DEADBAND_LUT_RUNTIME_SCALE_AUTO
#define M1_DEADBAND_LUT_RUNTIME_SCALE_AUTO  0
#endif

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

#else
#error "Unknown M1_BRINGUP_MODE — use M1_BRINGUP_MODE_* in motor_params_m1.h"
#endif

#ifndef M1_DEADBAND_LUT_BAKED_ENABLE
#define M1_DEADBAND_LUT_BAKED_ENABLE  0
#endif

/* 当前模式名（调试/VOFA 备注用；勿用 #pragma message，Keil 每 TU 重复告警） */
#if (M1_BRINGUP_MODE == M1_BRINGUP_MODE_MULTI_ANGLE_PASS0)
#define M1_BRINGUP_MODE_NAME  "MULTI_ANGLE_PASS0"
#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_IDENT_IQ_STEP)
#define M1_BRINGUP_MODE_NAME  "IDENT_IQ_STEP"
#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_ID_CAL_DUAL_FULL)
#define M1_BRINGUP_MODE_NAME  "ID_CAL_DUAL_FULL"
#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_ID_CAL_PASS0_ONLY)
#define M1_BRINGUP_MODE_NAME  "ID_CAL_PASS0_ONLY"
#else
#define M1_BRINGUP_MODE_NAME  "NORMAL"
#endif

/**
 * Id 锁轴标定状态机（1D Id 扫表 → 段 1 测 Ud_pi；段 2+ 在线建 LUT）。
 *
 * =1：强制 CURRENT_LOOP、Iq_ref=0、deadband OFF；自动扫 Id。
 * 与 M1_OPEN_UQ_DEADBAND_AB_SWEEP 互斥。
 * VOFA：Pass0/1 时 ch3=Ud_pi ch4=Id_ref ch5=θ(固定30°)；phase 50 时 ch3=Iq ch4=Id ch5=θ(编码器，同2347)。
 * dbg.open_seq_phase：38=30° ALIGN，0=init，1..N=标定档，39=Pass0 衰减，40/41..=LUT 验收，
 *   50=Iq 探路 LUT OFF，51=Iq 探路 LUT ON，9=DONE；120+leg×50+step=五角 Pass0。
 *
 * 由 M1_BRINGUP_MODE 推导；勿与 M1_IDENT_ENABLE 同开。
 */
#ifndef M1_ID_LOCK_CAL_SWEEP
#define M1_ID_LOCK_CAL_SWEEP  0
#endif

#if M1_ID_LOCK_CAL_SWEEP
#if M1_OPEN_UQ_DEADBAND_AB_SWEEP
#error "M1_ID_LOCK_CAL_SWEEP and M1_OPEN_UQ_DEADBAND_AB_SWEEP are mutually exclusive"
#endif

/** 标定态临时放宽 I_ref / PI 限幅（与 M1_CLOSURE_BRINGUP 日常 0.5 A 解耦） */
#ifndef M1_ID_CAL_OVERRIDE_LIMITS
#define M1_ID_CAL_OVERRIDE_LIMITS  1
#endif

#if M1_ID_CAL_OVERRIDE_LIMITS
#define M1_ID_CAL_I_REF_ABS_MAX    3.0f
/** 与 2347 电流环一致（M1_CLOSURE_BRINGUP 6 V），Pass0/Pass1/第三段共用 */
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
/** Pass0 标定扫表：32 点，0.05～1.50 A（4:3:1）；表见 motor_current.c */
#define M1_ID_CAL_AMP_TABLE_LEN     32u
#ifndef M1_ID_CAL_I_MAX_A
#define M1_ID_CAL_I_MAX_A           1.5f
#endif
/**
 * Pass1 LUT 验收：Pass0 同表 + 1.50→3.00 A 续扫（步进 0.1333 A，与末档一致）。
 * 各档 dwell 统一 M1_ID_CAL_VERIFY_ID_DWELL_S（默认 0.25 s）。
 * 仅 verify 段生效，capture/LUT 仍来自 Pass0 32 点。
 */
#ifndef M1_ID_CAL_VERIFY_I_MAX_A
#define M1_ID_CAL_VERIFY_I_MAX_A    3.0f
#endif
#ifndef M1_ID_CAL_VERIFY_EXT_LEN
#define M1_ID_CAL_VERIFY_EXT_LEN    11u
#endif
#define M1_ID_CAL_VERIFY_AMP_TABLE_LEN  (M1_ID_CAL_AMP_TABLE_LEN + M1_ID_CAL_VERIFY_EXT_LEN)
#define M1_ID_CAL_ID_DWELL_S        0.25f
/** Id_ref ≤ 此值时用 M1_ID_CAL_ID_DWELL_LOW_S（默认 0.25 s），否则用 M1_ID_CAL_ID_DWELL_S */
#ifndef M1_ID_CAL_ID_DWELL_LOW_ID_A
#define M1_ID_CAL_ID_DWELL_LOW_ID_A  0.40f
#endif
#ifndef M1_ID_CAL_ID_DWELL_LOW_S
#define M1_ID_CAL_ID_DWELL_LOW_S     0.25f
#endif
/** Pass1 LUT 验收：各 Id 档 dwell 统一（s），不再用 LOW_S 加长 */
#ifndef M1_ID_CAL_VERIFY_ID_DWELL_S
#define M1_ID_CAL_VERIFY_ID_DWELL_S  M1_ID_CAL_ID_DWELL_S
#endif
/** 上电 Id=0 稳定（s）；240125 已验证 */
#define M1_ID_CAL_INIT_HOLD_S       0.5f
/**
 * Pass0 末档→Pass1：Id_ref=0 衰减 (s)，LUT 仍 OFF，再 commit；仅 LUT_VERIFY 双扫生效。
 */
#ifndef M1_ID_CAL_PASS0_DECAY_S
#define M1_ID_CAL_PASS0_DECAY_S     0.8f
#endif
/** Pass1 LUT 验收扫表 init hold (s)，commit 后略长以便 settle */
#ifndef M1_ID_CAL_VERIFY_INIT_HOLD_S
#define M1_ID_CAL_VERIFY_INIT_HOLD_S  1.0f
#endif

/** capture：|Id-Id_ref| 门限 (A) */
#ifndef M1_ID_CAL_CAPTURE_EPS_A
#define M1_ID_CAL_CAPTURE_EPS_A     0.03f
#endif

/** Ud_residual 超过此值 (V) 置 outlier 标志，默认仍写入表 */
#ifndef M1_ID_CAL_OUTLIER_V
#define M1_ID_CAL_OUTLIER_V         1.0f
#endif

/**
 * commit 前 capture 表处理：sort 始终开启；dedupe 默认关。
 * 2045：dedupe 32→31 后低 Id Pass1 劣于 0827，待 A/B 验证；高 Id 不受影响。
 */
#ifndef M1_ID_CAL_LUT_DEDUP_ENABLE
#define M1_ID_CAL_LUT_DEDUP_ENABLE  0
#endif
#ifndef M1_ID_CAL_LUT_DEDUP_AMP_EPS_A
#define M1_ID_CAL_LUT_DEDUP_AMP_EPS_A  0.005f
#endif

/**
 * 段 2=0：只 RAM capture，不 deadband_set_lut
 * 段 3=1：deadband_cal_commit() 注册 LUT
 */
#ifndef M1_ID_CAL_COMMIT_LUT
#define M1_ID_CAL_COMMIT_LUT        1
#endif

/**
 * 段 3：标定 commit 后同次上电自动再扫 Id（LUT ON，不 capture，0.05～3.0 A）。
 * 须 M1_ID_CAL_COMMIT_LUT=1。
 */
#ifndef M1_ID_CAL_LUT_VERIFY_SWEEP
#define M1_ID_CAL_LUT_VERIFY_SWEEP  1
#endif

/**
 * =1：Pass0 扫完即 DONE（deadband 全程 OFF，不 commit / 不 LUT 突发 / 无 Pass1 / 无 Iq 探路）。
 * 由 M1_BRINGUP_MODE_ID_CAL_PASS0_ONLY / MULTI_ANGLE_PASS0 自动置 1。
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
 * Pass1 验收路径：=0（默认）d 表 + Ud 注入（锁轴验 capture，不与 abc duty 打架）；
 * =1 Pass1 也走 plut + apply_duty（仅 Step1 对照，易堵转）。
 */
#ifndef M1_ID_CAL_PASS1_USE_APPLY_DUTY
#define M1_ID_CAL_PASS1_USE_APPLY_DUTY  0
#endif

/**
 * Pass0 commit 后 Iq 旋转探路（同次上电）。
 *   Id=0，Iq=M1_ID_CAL_IQ_PROBE_A（默认 0.5 A），编码器 θ Park，M1_STARTUP_ENABLE=0 手拨启动；
 *   默认 OFF/FIXED 均为 0 s → commit 后直接 phase LUT ON（open_seq 51）。
 *   可选 A/B：M1_ID_CAL_IQ_PROBE_OFF_S>0 先 deadband OFF（50），
 *             M1_ID_CAL_IQ_PROBE_FIXED_S>0 再 FIXED（53），再 LUT ON。
 *   PI/电压限幅用 M1_CLOSURE_BRINGUP（6V/0.5A）；VOFA ch3=Iq ch4=Id ch5=θ。
 * 须 M1_ID_CAL_COMMIT_LUT=1；空载联调，LUT ON 段结束后手动断使能/停录。
 */
#ifndef M1_ID_CAL_IQ_PROBE_ENABLE
#define M1_ID_CAL_IQ_PROBE_ENABLE   1   /* Phase 3：Pass1 后 Iq 旋转探路 */
#endif
#ifndef M1_ID_CAL_IQ_PROBE_A
#define M1_ID_CAL_IQ_PROBE_A        M1_IQ_REF_A
#endif
/** Iq 探路第一段：LUT OFF 时长 (s) */
#ifndef M1_ID_CAL_IQ_PROBE_OFF_S
#define M1_ID_CAL_IQ_PROBE_OFF_S    10.0f
#endif
/** Iq 探路第二段：FIXED 符号补偿 ON (s)；0=跳过，OFF 后直接 LUT ON */
#ifndef M1_ID_CAL_IQ_PROBE_FIXED_S
#define M1_ID_CAL_IQ_PROBE_FIXED_S  5.0f
#endif
/**
 * Iq 探路段是否跑 Id PI（=0：仅 Uq/Iq 环，Pass1 大 Id 扫表后避免 Ud 积分锁死转子）。
 */
#ifndef M1_ID_CAL_IQ_PROBE_ID_PI_ENABLE
#define M1_ID_CAL_IQ_PROBE_ID_PI_ENABLE  0
#endif
/** @deprecated 已由 OFF→ON 双段取代；保留宏避免旧配置编译失败 */
#ifndef M1_ID_CAL_IQ_PROBE_DEADBAND_LUT
#define M1_ID_CAL_IQ_PROBE_DEADBAND_LUT  1
#endif

#if M1_ID_CAL_IQ_PROBE_ENABLE && !M1_ID_CAL_COMMIT_LUT
#error "M1_ID_CAL_IQ_PROBE_ENABLE requires M1_ID_CAL_COMMIT_LUT=1"
#endif

/**
 * 标定 Park 角固定 30° 电角（π/6）；Iq=0 时 d↔a 换算最简。
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
 * FIX_THETA=1 时上电先开环 Ud 对齐到 M1_ID_CAL_THETA_EL_RAD，再 Id 扫表。
 * 须 M1_ID_CAL_FIX_THETA_ENABLE=1。
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
 * 论文 §4.4 双特殊角标定：Pass0-A @30° + Pass0-B @0°，abc 样本合并建 phase 表。
 * =0：仅 Pass0-A（30°），与 Phase 1 行为相同。
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
 * =1：Pass0 在 0°/30°/60°/90°/120° 五电角各扫 32 档 Id（deadband OFF + capture）。
 * geo 样本池扩至 32×3×5；d 表锚仍仅 30° 档写入 s_dlut。
 * 录波后：`python tools/multi_angle_geo_analysis.py <csv>`
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
    !M1_ID_CAL_IQ_PROBE_ENABLE
#error "M1_ID_CAL_DUAL_ANGLE_ENABLE requires LUT_VERIFY+COMMIT, PASS0_ONLY, or IQ_PROBE+COMMIT"
#endif

/** Pass0 末衰减：双角 leg 切换 / commit 前 Id→0（Pass1 验表、Pass0-only、Pass0→Iq 均需要） */
#if M1_ID_CAL_PASS0_ONLY_ENABLE || \
    (M1_ID_CAL_COMMIT_LUT && (M1_ID_CAL_LUT_VERIFY_SWEEP || M1_ID_CAL_IQ_PROBE_ENABLE || \
                              M1_IDENT_ID_CAL_BEFORE_STEP))
#define M1_ID_CAL_PASS0_DECAY_ENABLE  1
#else
#define M1_ID_CAL_PASS0_DECAY_ENABLE  0
#endif

#if M1_ID_CAL_ALIGN_ENABLE && !M1_ID_CAL_FIX_THETA_ENABLE
#error "M1_ID_CAL_ALIGN_ENABLE requires M1_ID_CAL_FIX_THETA_ENABLE=1"
#endif

/** 双角 Pass0 abc 样本池上限：32 档 × 3 相 × 2 角 */
#ifndef M1_DEADBAND_GEO_SAMPLE_MAX
#define M1_DEADBAND_GEO_SAMPLE_MAX        (M1_ID_CAL_AMP_TABLE_LEN * 3u * 2u)
#endif

/**
 * =0：telem_lut_dump 不编译（stub）；日常固件关。
 */
#ifndef M1_VOFA_LUT_DUMP_ENABLE
#define M1_VOFA_LUT_DUMP_ENABLE     1
#endif

#else /* !M1_ID_LOCK_CAL_SWEEP */

#ifndef M1_ID_CAL_FIX_THETA_ENABLE
#define M1_ID_CAL_FIX_THETA_ENABLE   0
#endif

#ifndef M1_VOFA_LUT_DUMP_ENABLE
#define M1_VOFA_LUT_DUMP_ENABLE     0
#endif

#endif /* M1_ID_LOCK_CAL_SWEEP */

/* --- 死区补偿（固定符号法） --- */
/**
 * 电流环 deadband A/B 三档（开环 0623 已扫 400/591）：
 *   OFF（M1_DEADBAND_ENABLE=0）— 不过补，基线见 vofa+202606240039
 *   400 ns — 开环折中，≈0.192 V/相（改 M1_DEADTIME_NS 再编译）
 *   591 ns — 本档过补，≈0.284 V/相（当前默认 ON，闭环复测用）
 */
#ifndef M1_DEADBAND_ENABLE
#define M1_DEADBAND_ENABLE      0
#endif

/** 有效死区时间（ns）；标定模式强制 OFF；电流环 A/B 见 240039/240051 报告 */
#define M1_DEADTIME_NS          591u

/** PWM 周期（s），与 M1_CTRL_TS_S / TIM8 20 kHz 一致 */
#define M1_PWM_PERIOD_S         M1_CTRL_TS_S

/** 每相固定补偿电压：Vbus × t_dead / T_pwm ≈ 0.284 V @ 24 V, 591 ns */
#define M1_DEADBAND_V_COMP_V    (M1_VBUS_V * (float)M1_DEADTIME_NS * 1.0e-9f / M1_PWM_PERIOD_S)

/** 归一化占空比补偿量 = t_dead / T_pwm */
#define M1_DEADBAND_DUTY_COMP   (M1_DEADBAND_V_COMP_V / M1_VBUS_V)

/** 过零区：|i| 低于此值不补偿（仅 M1_DEADBAND_I_ZERO_DISABLE=0 时生效） */
#define M1_DEADBAND_I_ZERO_A    0.05f

/**
 * A/B 联调：=1 关闭过零死区，i≠0 即按符号全幅补偿。
 * 低电流请优先用 M1_DEADBAND_LUT_APPLY_MIN_A。
 */
#ifndef M1_DEADBAND_I_ZERO_DISABLE
#define M1_DEADBAND_I_ZERO_DISABLE   0
#endif

/** LUT 注入：1=d 轴 Ud（0830 过渡）；0=abc 单相表（Phase A 终态） */
#ifndef M1_DEADBAND_LUT_APPLY_UD
#define M1_DEADBAND_LUT_APPLY_UD  1   /* Phase 1：Ud 补偿能力；路径由 runtime 标志控制 */
#endif

/**
 * phase abc 注入：三相 duty 补偿后去零序 u₀=(Δa+Δb+Δc)/3（论文 4.4 运行时修正）。
 * 锁轴时 u₀≈0；旋转时强制三相补偿之和为零。
 */
#ifndef M1_DEADBAND_LUT_ZERO_SEQ_ENABLE
#define M1_DEADBAND_LUT_ZERO_SEQ_ENABLE  1
#endif

/**
 * LUT 运行时硬门槛：|i_phase| < APPLY_MIN_A 时 comp=0，否则全查表。
 * 硬切会在 ~0.4 A 形成补偿阶跃，污染 dq → 保持 0。
 * 低电流区用 LOW_FLAT 或 capture 曲线本身，勿用本开关。
 */
#ifndef M1_DEADBAND_LUT_APPLY_MIN_ENABLE
#define M1_DEADBAND_LUT_APPLY_MIN_ENABLE   0
#endif
#ifndef M1_DEADBAND_LUT_APPLY_MIN_A
#define M1_DEADBAND_LUT_APPLY_MIN_A        0.40f
#endif

/**
 * phase 表 apply_duty 运行缩放（仅 lut_domain=phase，不影响 d 表 Ud 辨识路径）。
 * 锁轴 Pass0 学的 |u'| 往往大于 Iq 旋转过零修正所需；联调可从 0.2～0.3 扫。
 */
#ifndef M1_DEADBAND_LUT_RUNTIME_SCALE
#define M1_DEADBAND_LUT_RUNTIME_SCALE       1.0f
#endif

/**
 * =1：commit 时 vals[] *= V_FIXED/lut(Iq_probe×cos30°)，runtime scale 恒 1（论文 #8）。
 * 与 M1_DEADBAND_LUT_RUNTIME_SCALE_AUTO 互斥。
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
 * commit 时把 phase LUT 低 Id 区改为线性（去掉 0.15~0.2 A 陡升）。
 * 0=保留 capture 曲线（2308 对照）；1=平坦化 Id∈[首点, M1_DEADBAND_LUT_LOW_FLAT_ID_A]。
 * Phase 1 对照（2026-06-27）：0=关平坦化，验证 2015 中 Id 失败是否由 LOW_FLAT 放大。
 */
#ifndef M1_DEADBAND_LUT_LOW_FLAT_ENABLE
#define M1_DEADBAND_LUT_LOW_FLAT_ENABLE   0
#endif
#ifndef M1_DEADBAND_LUT_LOW_FLAT_ID_A
#define M1_DEADBAND_LUT_LOW_FLAT_ID_A     0.30f
#endif

/**
 * Phase 3 geo 建表（论文 §4.4 ②③④⑤）：
 * 0=×0.866；1=双角 s_geo_samples 合并拟合 s_plut（须 DUAL_ANGLE=1）。
 */
#ifndef M1_DEADBAND_GEO_BUILD_ENABLE
#define M1_DEADBAND_GEO_BUILD_ENABLE      1
#endif

/**
 * commit 时 =1 建 fa/fb/fc 三表；=0 仅 proposed 共享 plut（30° amp + 双角 median val）。
 * P0 签收先用 0（离线金标准 ~1.47 V/相）；1146 triplet val≈1.95 V 暂不启用。
 */
#ifndef M1_DEADBAND_GEO_TRIPLET_ENABLE
#define M1_DEADBAND_GEO_TRIPLET_ENABLE    0
#endif

/** GEO_BUILD=0 时旁路对比 geo vs ×0.866，更新 deadband_geo_diff_* */
#ifndef M1_DEADBAND_GEO_DIFF_LOG_ENABLE
#define M1_DEADBAND_GEO_DIFF_LOG_ENABLE   1
#endif

/**
 * 开环 Uq 阶梯 + 死区 A/B（2026-06-23 综合录波）
 *
 * M1_OPEN_UQ_DEADBAND_AB_SWEEP=1 时 15 s 内 6 段（各 2.5 s）：
 *   phase 0: Uq=2.0 V, deadband OFF
 *   phase 1: Uq=2.0 V, deadband ON  (M1_DEADTIME_NS)
 *   phase 2: Uq=2.5 V, deadband OFF
 *   phase 3: Uq=2.5 V, deadband ON
 *   phase 4: Uq=3.0 V, deadband OFF
 *   phase 5: Uq=3.0 V, deadband ON
 * Watch: dbg.open_seq_phase = 0..5；VOFA ch4=扇区(M1_VOFA_SECTOR_DIAG=1)
 *
 * =0 时沿用旧逻辑：2/2.5/3 V 各 M1_OPEN_UQ_SWEEP_STEP_S（默认 5 s），死区由 M1_DEADBAND_ENABLE 编译决定。
 */
#ifndef M1_OPEN_UQ_DEADBAND_AB_SWEEP
#define M1_OPEN_UQ_DEADBAND_AB_SWEEP  0
#endif

/** A/B  sweep 每半段时长（s）；6 段合计 15 s */
#define M1_OPEN_UQ_HALF_STEP_S        2.5f

#define M1_OPEN_UQ_SWEEP_STEP_S       5.0f
#define M1_OPEN_UQ_SWEEP_V0           2.0f
#define M1_OPEN_UQ_SWEEP_V1           2.5f
#define M1_OPEN_UQ_SWEEP_V2           3.0f

/** @deprecated 用 M1_OPEN_UQ_SWEEP_V0 */
#define M1_OPEN_UQ_RUN_V          M1_OPEN_UQ_SWEEP_V0

/**
 * ADC2 注入 rank 交换试验（须与 binding 同步，见 docs/VOFA联调记录_20260617_扇区诊断全CSV.md §6）：
 * 0=默认 JDR1=PC3(ic) JDR3=PC4(ia)，binding [2,1,0]
 * 1=交换 JDR1=PC4(ia) JDR3=PC3(ic)，binding [0,1,2]（ic 最后采）
 */
#ifndef M1_ADC_RANK_SWAP_IAIC
#define M1_ADC_RANK_SWAP_IAIC     0
#endif

/* --- 扇区条件电流重构：仅扇区 1～2 重建 ic（180010 路线；CCR4 扫时用 0） --- */
#ifndef M1_CURRENT_RECON_ENABLE
#define M1_CURRENT_RECON_ENABLE   1
#endif

/** |i| 低于此值（A）时不重构，避免零漂放大 */
#ifndef M1_CURRENT_RECON_MIN_A
#define M1_CURRENT_RECON_MIN_A    0.05f
#endif

/* --- Iq 环辨识（堵转 + 磁粉制动器；与 Id 扫表互斥，由 M1_BRINGUP_MODE 开关） --- */
/**
 * =1：堵转 CURRENT_LOOP；可选先 Id Pass0 commit，再 Bode 或 Iq 阶跃。
 *     Park 角：编码器 θ（FIX_THETA=0）；M1_IDENT_OVERRIDE_LIMITS=1。
 *     当前 IDENT 模式：Id 建表 → HOLD 2s → Bode×3（OFF / FIXED / LUT 现场表），10→800 Hz。
 * VOFA：Id 段 ch3=Ud ch4=Id_ref；Bode 段 ch3=Iq ch4=Iq_ref ch5=Uq_pi。
 * open_seq：Id 1..N→39→9；60=HOLD 62=Bode-OFF 63=Bode-FIXED 64=Bode-LUT 73=DONE 80=开环Uq 81=结束。
 * 选 M1_BRINGUP_MODE_IDENT_IQ_STEP 时自动 =1。
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
#define M1_IDENT_IQ_BODE_ENABLE     0   /* 先阶跃；=1 时 STEP 后接 BODE */
#endif
#if !M1_IDENT_IQ_STEP_ENABLE && !M1_IDENT_IQ_BODE_ENABLE
#error "M1_IDENT_ENABLE requires M1_IDENT_IQ_STEP_ENABLE and/or M1_IDENT_IQ_BODE_ENABLE"
#endif

#ifndef M1_IDENT_FIX_THETA_ENABLE
#define M1_IDENT_FIX_THETA_ENABLE   0   /* 0=编码器 θ（堵转默认）；1=写死 30° */
#endif
#ifndef M1_IDENT_THETA_EL_RAD
#define M1_IDENT_THETA_EL_RAD       0.5235987755982988f  /* 仅 FIX_THETA=1 时用 */
#endif

/** 制动器加载后稳定等待 (s) */
#ifndef M1_IDENT_HOLD_S
#define M1_IDENT_HOLD_S               2.0f
#endif
/** 阶跃总轮数 / OFF / FIXED：IDENT 模式在上方 #elif 已设为 18 / 6 / 6；此处仅为未覆盖时的兜底 */
#ifndef M1_IDENT_STEP_ROUNDS
#define M1_IDENT_STEP_ROUNDS          18u
#endif
#ifndef M1_IDENT_STEP_OFF_ROUNDS
#define M1_IDENT_STEP_OFF_ROUNDS      6u
#endif
#ifndef M1_IDENT_STEP_FIXED_ROUNDS
#define M1_IDENT_STEP_FIXED_ROUNDS    6u
#endif
/** 1 且 FIXED_ROUNDS=0：OFF 之后全用 LUT；FIXED_ROUNDS>0 时第三段恒为 LUT */
#ifndef M1_IDENT_STEP_LUT_AFTER_OFF
#define M1_IDENT_STEP_LUT_AFTER_OFF   0
#endif
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
#ifndef M1_IDENT_STEP_I0_A
#define M1_IDENT_STEP_I0_A            0.0f
#endif
#ifndef M1_IDENT_STEP_I1_A
#define M1_IDENT_STEP_I1_A            0.3f   /* 小阶跃 */
#endif
#ifndef M1_IDENT_STEP_I2_A
#define M1_IDENT_STEP_I2_A            0.5f   /* 中阶跃 */
#endif
#ifndef M1_IDENT_STEP_I3_A
#define M1_IDENT_STEP_I3_A            1.0f   /* 大阶跃 */
#endif
/** 各拍 dwell (s)；非零档与回零档相同 */
#ifndef M1_IDENT_STEP_DWELL_S
#define M1_IDENT_STEP_DWELL_S         0.5f
#endif
#ifndef M1_IDENT_STEP_ZERO_DWELL_S
#define M1_IDENT_STEP_ZERO_DWELL_S    0.5f
#endif

/** 1=阶跃辨识绕过 M1_CLOSURE_BRINGUP 的 I_ref 钳位与 6 V PI 限幅 */
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

/** Bode：iq_ref = bias + amp*sin(2πft)，几何扫频 f *= RATIO */
#ifndef M1_IDENT_BODE_I_BIAS_A
#define M1_IDENT_BODE_I_BIAS_A        0.25f
#endif
#ifndef M1_IDENT_BODE_I_AMP_A
#define M1_IDENT_BODE_I_AMP_A         0.05f
#endif
#ifndef M1_IDENT_BODE_F0_HZ
#define M1_IDENT_BODE_F0_HZ           10.0f
#endif
#ifndef M1_IDENT_BODE_F1_HZ
#define M1_IDENT_BODE_F1_HZ           800.0f
#endif
#ifndef M1_IDENT_BODE_F_RATIO
#define M1_IDENT_BODE_F_RATIO         1.15f
#endif
#ifndef M1_IDENT_BODE_CYCLES_PER_FREQ
#define M1_IDENT_BODE_CYCLES_PER_FREQ 8.0f
#endif
/** Bode 分段：1×OFF + 1×FIXED + 1×LUT，各跑 F0→F1 一整遍 */
#ifndef M1_IDENT_BODE_ROUNDS
#define M1_IDENT_BODE_ROUNDS          3u
#endif
#ifndef M1_IDENT_BODE_OFF_ROUNDS
#define M1_IDENT_BODE_OFF_ROUNDS      1u
#endif
#ifndef M1_IDENT_BODE_FIXED_ROUNDS
#define M1_IDENT_BODE_FIXED_ROUNDS    1u
#endif
#if (M1_IDENT_BODE_OFF_ROUNDS + M1_IDENT_BODE_FIXED_ROUNDS > M1_IDENT_BODE_ROUNDS)
#error "M1_IDENT_BODE_OFF_ROUNDS + M1_IDENT_BODE_FIXED_ROUNDS must be <= M1_IDENT_BODE_ROUNDS"
#endif
#if ((M1_IDENT_BODE_ROUNDS - M1_IDENT_BODE_OFF_ROUNDS - M1_IDENT_BODE_FIXED_ROUNDS) > 0u) && \
    !M1_DEADBAND_LUT_BAKED_ENABLE && !M1_IDENT_ID_CAL_BEFORE_STEP
#error "Bode LUT segment requires M1_IDENT_ID_CAL_BEFORE_STEP or M1_DEADBAND_LUT_BAKED_ENABLE"
#endif
#endif /* M1_IDENT_ENABLE */

#endif
