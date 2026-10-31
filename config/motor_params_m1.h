/**
 * @file motor_params_m1.h
 * @date 2026-10-06
 * @brief M1 电机与控制宏的默认值。profile 可覆盖。

 *
 * 本头条目很多。第一刀只保证文件头；改到哪条宏再补哪条注释。
 * 电流热路径用预计算的安培/LSB，不要在 ISR 里做除法。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#ifndef MOTOR_PARAMS_M1_H
#define MOTOR_PARAMS_M1_H

/** N5065 极对。profile 可覆盖；运行时副本见 g_m1_motor_cfg。 */
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
#define M1_ADC_GAIN_CH0     1.00f
#define M1_ADC_GAIN_CH1     1.00f
#define M1_ADC_GAIN_CH2     1.00f

/* --- 电气参数（LCR @ 1 kHz，AB 线；Rs 多轮 VASI 辨识 ~0.122 Ω。--- */
/* 铭牌：#ifndef 守卫；热路径优先读 g_m1_motor_cfg（包 8.3）。 */
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
#define M1_PLL_ENABLE                   1
#define M1_PLL_FN_HZ                    80.0f
#define M1_PLL_ZETA                     0.707106781f
#define M1_PLL_WN_RAD_S                 (6.28318530718f * M1_PLL_FN_HZ)
#define M1_PLL_KP                       (2.0f * M1_PLL_ZETA * M1_PLL_WN_RAD_S)
#define M1_PLL_KI                       (M1_PLL_WN_RAD_S * M1_PLL_WN_RAD_S)
#define M1_PLL_OMEGA_LIMIT_RAD_S        650.0f   /* ~6200 rpm mech */
#define M1_PLL_INTEGRATOR_LIMIT_RAD_S   M1_PLL_OMEGA_LIMIT_RAD_S
#define M1_PLL_THETA_PARK_ENABLE        0
/** 1=VOFA ch8。1 输出 PLL（ch8 ω_pll ch9 ω_diff ch10 θ_err ch11 Δω）；0=Id_ref/Iq_ref/duty_dev/open_seq */
#define M1_VOFA_PLL_CH8_11              M1_PLL_ENABLE

/* ==========================================================================
 * 死区联调相位（建表 LUT 已下线；PASS0/LUT/ONE_SHOT 枚举仅兼容旧 profile）。
 * 现网常用：SPEED_OFF / SPEED_IDENT。
 * ========================================================================== */
#define M1_DEADBAND_FLOW_ONE_SHOT       0  /* LUT 下线后恒关 */

/*
 * 联调实例选择：只改 config/bringup_active.h（ROUTE_ID + BOOT_EXP_ID）。
 * 外环/MIT/实验默认：defaults_outer.h（在 active 前，active 用 #undef 覆盖）
 * 观测门控默认：defaults_obs.h（在 recipe 前，recipe 用 #undef 覆盖）
 * 配方由 ACTIVE_ROUTE_ID 选 config/recipes/recipe_*.h。
 */
#include "defaults_outer.h"
#include "bringup_active.h"
/** 台架角源/零偏/机械角符号：须在本文件后续默认之前纳入。 */
#include "m1_axis_hw.h"
#include "route_ids.h"
#include "defaults_obs.h"
#include "defaults_fold.h"

/**
 * 配方单点纳入（route→recipe_*；defaults 由各 recipe 自带）。
 * 现网：route1=HFI±2000，route9=PASS0，route8=Bode Id。
 */
#include "recipes/recipe_active.h"

/**
 * ISR dbg 瘦身（默认关 = 省 foc 窗 cycles；Watch 深挖时单开）。
 * OPT12 无感只需 hfi_stage；满拷 read_view 曾把 foc 从 ~6100 抬回 ~7200。
 */
#if M1_POS_MIT_COMBO_ENABLE
#undef M1_TELEM_BRINGUP_DECIMATION
#define M1_TELEM_BRINGUP_DECIMATION     4u    /* MIT。kHz。0kHz/4。*/
#endif

/**
 * 1=bringup 全阶段固。VOFA×12（Id cal / ident / 开环阶梯同布局，不再按 open_seq 。ch3。1）。
 * ch0=Ia ch1=Ib ch2=Ic ch3=Id ch4=Iq ch5=θ_el
 * M1_VOFA_IDENT_DUTY_12CH=1（辨。标定，默认）：ch6=Vd_est ch7=Vq_est ch8=Ta ch9=Tb ch10=Tc ch11=open_seq。
 *   Pass0：ch8=id_ref ch9=id ch10=err；VASI：ch6/7=Ud/Uq ch8=Ld_uH ch9=Lq_uH ch10=L_avg。
 * M1_VOFA_IDENT_DUTY_12CH=0：ch6=Ud_out ch7=Uq_out。
 *   M1_VOFA_MIT_CH8_11=1 。ch8=ω_pll ch9=θ_err ch10=Iq_ref ch11=open_seq（MIT 验收）；
 *   M1_VOFA_PLL_CH8_11=1 。ch8=ω_pll ch9=ω_diff ch10=θ_err ch11=Δω。
 *   M1_VOFA_PLL_CH8_11=0 。ch8=Id_ref ch9=Iq_ref ch10=duty_dev ch11=open_seq。
 */

/** 机械角符号 / THETA_NEGATE：唯一源 m1_axis_hw.h。 */

/**
 * 母线电压（V）→ 仅电压限幅 PI_V_MAX；运行时副本见 g_m1_motor_cfg。
 * 改 24→48 等时不要顺手改 I_REF / 斜坡（电流额定与采样链另定）。
 */
#define M1_VBUS_V           24.0f
#define M1_PI_V_MAX         (M1_VBUS_V * 0.577350269f)

#define M1_PI_WC_RADS       (2.0f * 3.14159265359f * M1_PI_FC_HZ)

/** 联调。=固定 Kp/Ki（试凑）。=。fc 整定 */

#define M1_PI_KP_ID         (M1_LD_H * M1_PI_WC_RADS)
#define M1_PI_KP_IQ         (M1_LQ_H * M1_PI_WC_RADS)
#define M1_PI_KI_NOMINAL    (M1_RS_OHM * M1_PI_WC_RADS * M1_CTRL_TS_S)
#define M1_PI_KI_SCALE      1.0f
#define M1_PI_KI            (M1_PI_KI_NOMINAL * M1_PI_KI_SCALE)

/**
 * 首次闭环联调。4V/0.8A 电源）：降低 dq 电压限幅，ref 硬钳位。
 * 稳定后设 M1_CLOSURE_BRINGUP=0 恢复满幅 SVPWM 线性区。
 * 默认 fold=1；HFI/Bode recipe 写 0。速度环 Iq 限幅由 SPEED 配方硬写。
 */
#if M1_CLOSURE_BRINGUP
#define M1_PI_V_LIMIT_V     6.0f
#define M1_I_REF_ABS_MAX    0.5f
#else
#define M1_PI_V_LIMIT_V     M1_PI_V_MAX
/*
 * TODO(factory)：I_REF 出厂取 min(电机峰值/连续, 分流满量程, 驱动热设计)，单一源。
 * 11 A 为 HFI 联调临时，非额定；与 VBUS 无公式关系。
 * 见 docs/规范_外环限幅与斜坡_出厂整定_2026-10-11.md
 */
#define M1_I_REF_ABS_MAX    11.0f
#endif

#define M1_PI_V_LIMIT_MIN   (-M1_PI_V_LIMIT_V)

/** 积分器限幅（V），与输出限幅同量级，防 windup */
#define M1_PI_INT_LIMIT_V   M1_PI_V_LIMIT_V
#define M1_PI_INT_LIMIT_MIN (-M1_PI_INT_LIMIT_V)

/** 0=。ALIGN/DRAG，编码器 θ 直接闭环。=启动状态机 */
#define M1_STARTUP_ENABLE   0

/**
 * 1=编码器可选：控制不吃 SPI。插着只给遥测对照；拔掉仍走无感。
 * 默认 0；profile 可覆盖。
 */
#define M1_ENC_OPTIONAL_ENABLE          0

/* C：观测配方 vs 模块编译闸（BUILD_*）；配方开而模块未编则硬失败。 */
#include "build_config.h"
#if M1_HFI_ENABLE && !BUILD_HFI
#error "M1_HFI_ENABLE=1 requires BUILD_HFI=1"
#endif
#if (M1_EMF_SMO_ENABLE || M1_EMF_PLL_ENABLE) && !BUILD_SMO
#error "M1_EMF_SMO/PLL_ENABLE=1 requires BUILD_SMO=1"
#endif
#if M1_EMF_VEQ_ENABLE && !BUILD_VEQ
#error "M1_EMF_VEQ_ENABLE=1 requires BUILD_VEQ=1"
#endif

/** 1=跑 deadband_flow（Id/Rs/L 或 ident/speed_ident 配方） */
#define M1_DEADBAND_FLOW_ENABLE         \
    (M1_IDENT_ENABLE || M1_ID_LOCK_CAL_SWEEP || M1_SPEED_IDENT_ENABLE || \
     M1_OPEN_UD_PRE_ID_CAL_ENABLE || M1_OPEN_UQ_PRE_ID_CAL_ENABLE)

/* --- 死区补偿（仅固定符号法；建表 LUT 已下线）--- */

/** 有效死区时间（ns）；标定模式强制 OFF；运行时副本见 g_m1_motor_cfg。 */
#define M1_DEADTIME_NS          591u

/** PWM 周期（s），。M1_CTRL_TS_S / TIM8 20 kHz 一。*/
#define M1_PWM_PERIOD_S         M1_CTRL_TS_S

/**
 * PWM ARR 计数（与 CubeMX TIM1/TIM8 Init.Period 同值）。
 * 这是计数域，不是时间域。M1_PWM_PERIOD_S 是秒，二者不能互换。
 */
#define M1_PWM_ARR_COUNTS       3999u

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
#define M1_DEADBAND_I_ZERO_DISABLE   0

/* LUT/geo / Pass0 双角电角：见上方下线闸门与 FIX_THETA 段，此处不再重复。 */

/* V012 开环扫参电压台阶（motor_open_sweep SWEEP_KIND_V012 仍用） */
#define M1_OPEN_UQ_SWEEP_STEP_S       5.0f
#define M1_OPEN_UQ_SWEEP_V0           2.0f
#define M1_OPEN_UQ_SWEEP_V1           2.5f
#define M1_OPEN_UQ_SWEEP_V2           3.0f

/**
 * ADC2 注入 rank 交换试验（须与 binding 同步，见 docs/VOFA联调记录_20260617_扇区诊断全CSV.md §6）：
 * 0=默认 JDR1=PC3(ic) JDR3=PC4(ia)，binding [2,1,0]
 * 1=交换 JDR1=PC4(ia) JDR3=PC3(ic)，binding [0,1,2]（ic 最后采）
 */
#define M1_ADC_RANK_SWAP_IAIC     0

/* --- 扇区条件电流重构：仅扇区 1。 重建 ic。80010 路线；CCR4 扫时。0。--- */
#define M1_CURRENT_RECON_ENABLE   1

/** |i| 低于此值（A）时不重构，避免零漂放大 */
#define M1_CURRENT_RECON_MIN_A    0.05f

/* 配方附属 defaults：须在 PI 限幅之后（见 defaults_id_cal / ident_iq）。 */
#if M1_ID_LOCK_CAL_SWEEP
#include "defaults_id_cal.h"
#endif
#if M1_IDENT_ENABLE
#include "defaults_ident_iq.h"
#endif
#if M1_SPEED_IDENT_ENABLE
#include "defaults_speed_ident.h"
#endif

/*
 * Id-cal / ident PI 限幅别名：恒对齐电流环（defaults_* 若已写过则覆盖）。
 * 现网三 recipe：IDENT+OVERRIDE 仅在 CLOSURE=0 出现，此时 PI_V_LIMIT==V_MAX，与 OVERRIDE 同值。
 */
#undef M1_ID_CAL_PI_V_LIMIT_V
#define M1_ID_CAL_PI_V_LIMIT_V          M1_PI_V_LIMIT_V
#undef M1_ID_CAL_PI_V_LIMIT_MIN
#define M1_ID_CAL_PI_V_LIMIT_MIN        (-M1_ID_CAL_PI_V_LIMIT_V)
#undef M1_ID_CAL_PI_INT_LIMIT_V
#define M1_ID_CAL_PI_INT_LIMIT_V        M1_ID_CAL_PI_V_LIMIT_V
#undef M1_ID_CAL_PI_INT_LIMIT_MIN
#define M1_ID_CAL_PI_INT_LIMIT_MIN      (-M1_ID_CAL_PI_INT_LIMIT_V)
#undef M1_IDENT_PI_V_LIMIT_V
#define M1_IDENT_PI_V_LIMIT_V           M1_PI_V_LIMIT_V
#undef M1_IDENT_PI_V_LIMIT_MIN
#define M1_IDENT_PI_V_LIMIT_MIN         (-M1_IDENT_PI_V_LIMIT_V)
#undef M1_IDENT_PI_INT_LIMIT_V
#define M1_IDENT_PI_INT_LIMIT_V         M1_IDENT_PI_V_LIMIT_V
#undef M1_IDENT_PI_INT_LIMIT_MIN
#define M1_IDENT_PI_INT_LIMIT_MIN       (-M1_IDENT_PI_V_LIMIT_V)

/**
 * 全部 profile 之后。
 * I/F 帧恒。+|Iq| 拖动（正/。ω 皆然。121：反。I/F 。+4）。
 * 速度环交。bumpless 。Continuity 同号，勿。DIR（乘 DIR 会在 BLEND 。+4→−3.5 反扭矩砸速）。
 * DIR 只作用在 ω 目标；转子系若需 −Iq 保负速，。PI 在限权内自行斜过去。
 */
#undef M1_IF_OBS_SPEED_IQ_BOOT_A
#define M1_IF_OBS_SPEED_IQ_BOOT_A       (M1_IF_IQ_A)

#endif
