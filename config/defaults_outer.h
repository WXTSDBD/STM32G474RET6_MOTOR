/**
 * @file defaults_outer.h
 * @date 2026-10-29
 * @brief 外环 / MIT / 摩擦 / 实验框架默认宏（普通 #define）。
 * @note 须在 bringup_active.h / profile 之前纳入；覆盖方用 #undef+#define。
 */
#ifndef CONFIG_DEFAULTS_OUTER_H
#define CONFIG_DEFAULTS_OUTER_H

/**
 * 外环执行位置：
 *   1 = 速度/位置/力矩等外环在 TIM6 2 kHz 中断里跑（与 20 kHz FOC 内环剥离，降抖动）
 *   0 = 旧行为：在 20 kHz FOC ISR 里按 M1_SPEED_DECIM 分频调用
 */
#define M1_OUTER_IN_TIM6                0

#define M1_OUTER_EXPT_NONE              0
#define M1_OUTER_EXPT_POS_STEP          2
#define M1_OUTER_EXPT_POS_REV           3
#define M1_OUTER_EXPT_MIT_HOLD          4
#define M1_OUTER_EXPT_MIT_REV           5
#define M1_OUTER_EXPT                   M1_OUTER_EXPT_NONE
#define M1_OUTER_NEST_ENABLE            0
/** MIT 相对 Kp_pos 的倍率。1=与位置环同一 P。 */
#define M1_MIT_KP_SCALE                 1.0f
/** MIT 在 ω_ref 上减 Kd·ω，单位 rpm/rpm。0=不加阻尼。 */
#define M1_MIT_KD_RPM                   0.0f

/** 签收档编号。与 POS_STEP/POS_REV/MIT_HOLD/MIT_REV 并列，一次只选一个。 */
#define M1_OUTER_EXPT_SIGNOFF           6

/** 1=超速守卫使能。无感冒烟可关，避免中途 seq=252 掐死。 */
#define M1_OUTER_OVERSPEED_GUARD_ENABLE 1

/** 1=速度 PI 的积分当拍为 0，即参数级关 Ki。MIT 档用，位置档保持 0。 */
#define M1_MIT_KI_DISABLE               0

/**
 * MIT 力矩律形态（运控/力控常用「并行」写法）。
 * 0=嵌套旧路径：位置 P → 速度 PI（Ki 可关）→ iq_ref（可回退）。
 * 1=并行力矩源：iq* = Kp·Δθ + Kd·Δω + iq_ff，不经速度 PI；电流环跟 Iq*。
 */
#define M1_MIT_TORQUE_LAW               0
/**
 * 1=并行 MIT 在电流环节拍（20 kHz）每拍刷新 iq_ref；时间表仍按 SPEED_DECIM。
 * 仅 M1_MIT_TORQUE_LAW=1 时有效。0=只在外环分频拍更新（2 kHz 若 DECIM=10）。
 */
#define M1_MIT_TORQUE_LAW_20KHZ         1
/**
 * 并行 MIT 刚度，单位 A/rad（电流域；与 iq_ff 同单位，便于刚度 A/B）。
 * 运控若下发 Nm：Kp_A = Kp_Nm / Kt（Kt 见 M1_MIT_KT_NM_A）。
 */
#define M1_MIT_KP_A_PER_RAD             22.0f
/** 并行 MIT 阻尼，单位 A·s/rad。低刚度档默认 0。 */
#define M1_MIT_KD_A_S_PER_RAD           0.0f
/**
 * 高刚度档专用阻尼，单位 A·s/rad。
 * PACK_MK 中 Kp_scale ≥ M1_MIT_KD_HIGH_SCALE_MIN 时改用本值。
 */
#define M1_MIT_KD_HIGH_A_S_PER_RAD      0.4f
/** 启用高档阻尼的最小 Kp_scale（含）。 */
#define M1_MIT_KD_HIGH_SCALE_MIN        6.0f
#if M1_MIT_TORQUE_LAW && !M1_OUTER_NEST_ENABLE
#error "M1_MIT_TORQUE_LAW=1 requires M1_OUTER_NEST_ENABLE=1"
#endif

/** 1=库仑摩擦方向前馈使能。出口叠加；标定段由时间表分段关掉。 */
#define M1_FRIC_FF_ENABLE               0
/**
 * 1=只跑 MIT 刚度 A/B：多档 iq_ff × Kp_scale∈{1,2,3,4,6}（一包自动）。
 */
#define M1_OUTER_SIGNOFF_PACK_MK        0

/**
 * 1=外环实验走 motor/experiment 注册表（E0）。
 * bringup_active / profile 用 #undef 覆盖。
 */
#define M1_EXP_FRAMEWORK_ENABLE         0
/** 1=上电跑 UD_CAL 段表。 */
#define M1_EXP_BOOT_UD_CAL              0
/** 1=上电跑 ±2000 段表。 */
#define M1_EXP_BOOT_REV141              0
/** 1=上电跑联合标定链（Pass0→Rs→L，EXP_ID_CALIB_CHAIN）。 */
#define M1_EXP_BOOT_CALIB_CHAIN         0

/**
 * 位置环位置反馈源：
 * 0=编码器解包角，1=编码器 PLL 角，2=HFI θ̂ 多圈解包（无感位控冒烟）。
 */
#define M1_OUTER_THETA_FB_SRC           0

/** 1=12 通道遥测切到签收档映射（θ_fb/θ_enc 并存，带 seq）。 */
#define M1_VOFA_SIGNOFF_CH              0
/** 1=OPT12 遥测布局。默认关，静置 profile 打开。 */
#define M1_VOFA_OPT12                   0

#endif /* CONFIG_DEFAULTS_OUTER_H */
