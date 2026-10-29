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
#define M1_DB_BRINGUP_SPEED_OFF         0
#define M1_DB_BRINGUP_PASS0             1  /* 旧：LUT Pass0，勿新开 */
#define M1_DB_BRINGUP_SPEED_LUT         2  /* 旧：LUT 速度阶梯，勿新开 */
#define M1_DB_BRINGUP_ONE_SHOT          3  /* 旧：Pass0→OFF→LUT，勿新开 */
#define M1_DB_BRINGUP_SPEED_IDENT       4
#define M1_DEADBAND_BRINGUP_PHASE       M1_DB_BRINGUP_SPEED_OFF
#define M1_DEADBAND_FLOW_ONE_SHOT       0  /* LUT 下线后恒关 */

/* Bringup mode（须。M1_SPEED_LOOP_ENABLE 之前选定。*/
#define M1_BRINGUP_MODE_NORMAL                0
#define M1_BRINGUP_MODE_ID_CAL_PASS0_RS       5
#define M1_BRINGUP_MODE_SPEED_IDENT           8
#define M1_BRINGUP_MODE_BODE_OFF_ONLY         9
#define M1_BRINGUP_MODE_RS_LD_LQ_ONLY         10
#define M1_BRINGUP_MODE_BODE_ID_OFF_ONLY      11
/** 。Pass0 后：开。Ud 阶梯。LUT（NVM LUT ON；无 Pass0/ident。*/

/*
 * 联调实例选择：只改 config/bringup_active.h
 * 外环/MIT/实验默认：defaults_outer.h（在 active 前，active 用 #undef 覆盖）
 * 观测门控默认：defaults_obs.h（在 profile/bm 前，bm 用 #undef 覆盖）
 */
#include "defaults_outer.h"
#include "bringup_active.h"
/** 台架角源/零偏/机械角符号：须在本文件后续默认之前纳入。 */
#include "bringup_bench.h"
#include "route_ids.h"
#include "route_use_from_id.h"
#include "defaults_obs.h"
#if (M1_BRINGUP_MODE == M1_BRINGUP_MODE_SPEED_IDENT)
/* TODO(factory)：斜坡速率出厂自算；500 为联调临时。见 docs/规范_外环限幅与斜坡_出厂整定_2026-10-11.md */
#define M1_SPEED_OMEGA_RAMP_ENABLE      1
#define M1_SPEED_OMEGA_RAMP_RPM_S       500.0f
#endif

/* --- E1 速度。@ 2 kHz。0 kHz 分频；限幅宏。M1_I_REF_ABS_MAX 之后。-- */
#if (M1_BRINGUP_MODE == M1_BRINGUP_MODE_BODE_OFF_ONLY) || \
    (M1_BRINGUP_MODE == M1_BRINGUP_MODE_BODE_ID_OFF_ONLY)
#define M1_SPEED_LOOP_ENABLE            0
#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_RS_LD_LQ_ONLY)
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
#if M1_SPEED_LOOP_ENABLE
#include "defaults_speed_loop.h"
#endif /* M1_SPEED_LOOP_ENABLE */


/** VOFA JustFloat 通道数与分频（JEOC 20kHz 基准，D=2。0kHz / D=4。kHz / D=10。kHz 帧率。*/
#define M1_TELEM_BRINGUP_K              12u
#define M1_TELEM_BRINGUP_DECIMATION     2u

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
#define M1_VOFA_UNIFIED_12CH            1

/** 机械角符号（±1）。KTH 台架默认在 bringup_bench.h 置 -1。 */
#ifndef M1_ENCODER_MECH_SIGN
#define M1_ENCODER_MECH_SIGN            (1.0f)
#endif

/** 1=Park/VOFA 。-θ_enc。=。SVPWM 。+θ（与 Core/Inc/main.h 同名宏兼容） */
#ifndef M1_THETA_NEGATE
#define M1_THETA_NEGATE     0
#endif

/**
 * 母线电压（V）→ 仅电压限幅 PI_V_MAX；运行时副本见 g_m1_motor_cfg。
 * 改 24→48 等时不要顺手改 I_REF / 斜坡（电流额定与采样链另定）。
 */
#define M1_VBUS_V           24.0f
#define M1_PI_V_MAX         (M1_VBUS_V * 0.577350269f)

/** PI 带宽（Hz）；M1_PI_USE_FIXED_GAIN=0 时用。L×ωc 整定（阶跃验收参。1000 Hz。*/
#define M1_PI_FC_HZ         1000.0f
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
 */
#if M1_SPEED_LOOP_ENABLE
#define M1_CLOSURE_BRINGUP  0   /* 速度环联调：满幅 Iq/Uq */
#else
#define M1_CLOSURE_BRINGUP  1
#endif

#if M1_CLOSURE_BRINGUP
#define M1_PI_V_LIMIT_V     6.0f
#define M1_I_REF_ABS_MAX    0.5f
#else
#define M1_PI_V_LIMIT_V     M1_PI_V_MAX
/*
 * TODO(factory)：I_REF 出厂取 min(电机峰值/连续, 分流满量程, 驱动热设计)，单一源。
 * 11/5 A 为联调分支临时，非额定；与 VBUS 无公式关系。
 * 见 docs/规范_外环限幅与斜坡_出厂整定_2026-10-11.md
 */
#if (M1_BRINGUP_MODE == M1_BRINGUP_MODE_SPEED_IDENT) && \
    (M1_USE_SENSED_POS_MIT_PROFILE == 0) && \
    (M1_USE_SENSORLESS_POS_MIT_PROFILE == 0)
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
/* TODO(factory)：速度环 Iq 上限应等于 I_REF 单一源；勿再平行写死 11。 */
#if (M1_BRINGUP_MODE == M1_BRINGUP_MODE_SPEED_IDENT) && \
    (M1_USE_SENSED_POS_MIT_PROFILE == 0) && \
    (M1_USE_SENSORLESS_POS_MIT_PROFILE == 0)
#define M1_SPEED_IQ_REF_ABS_MAX         11.0f
#else
#define M1_SPEED_IQ_REF_ABS_MAX         M1_I_REF_ABS_MAX
#endif
#define M1_SPEED_PI_OUT_MAX             M1_SPEED_IQ_REF_ABS_MAX
#define M1_SPEED_PI_OUT_MIN             (-M1_SPEED_IQ_REF_ABS_MAX)
#define M1_SPEED_PI_INT_MAX             M1_SPEED_IQ_REF_ABS_MAX
#define M1_SPEED_PI_INT_MIN             (-M1_SPEED_IQ_REF_ABS_MAX)
#endif

/** 电流。Iq 目标（A）；无启动策略时上电即用，手拨启动（Iq 探路/日常。.5 A。*/
/* M1_IQ_REF_A：无 .c / 无派生引用；profile 若仍 #define 可留。 */

/** 0=。ALIGN/DRAG，编码器 θ 直接闭环。=启动状态机 */
#define M1_STARTUP_ENABLE   0
#define M1_IF_ENABLE                    0
#define M1_IF_IQ_A                      0.5f
#if M1_IF_ENABLE
#include "defaults_if.h"
#endif /* M1_IF_ENABLE */

/**
 * 1=编码器可选：控制不吃 SPI。插着只给遥测对照；拔掉仍走无感。
 * 默认 0；profile 可覆盖。
 */
#define M1_ENC_OPTIONAL_ENABLE          0

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
#include "bringup_modes/bm_normal.h"
#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_ID_CAL_PASS0_RS)
/* A2：先 defaults（裸定义）再 bm（#undef 覆盖）；实验值不变。 */
#undef M1_ID_LOCK_CAL_SWEEP
#define M1_ID_LOCK_CAL_SWEEP            1
#undef M1_IDENT_ENABLE
#define M1_IDENT_ENABLE                 0
#include "defaults_id_cal.h"
#include "bringup_modes/bm_pass0_rs.h"
#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_RS_LD_LQ_ONLY)
#include "bringup_modes/bm_rs_ld_lq_only.h"
#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_BODE_OFF_ONLY)
/* A2：先 defaults_ident_iq 再 bm 覆盖。 */
#undef M1_ID_LOCK_CAL_SWEEP
#define M1_ID_LOCK_CAL_SWEEP            0
#undef M1_IDENT_ENABLE
#define M1_IDENT_ENABLE                 1
#include "defaults_ident_iq.h"
#include "bringup_modes/bm_bode_off_only.h"
#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_BODE_ID_OFF_ONLY)
#include "bringup_modes/bm_bode_id_off_only.h"
#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_SPEED_IDENT)
/* profile（如 hfi_standstill）可能关 SPEED_IDENT；defaults 在后面按门控纳入。 */
#include "bringup_modes/bm_speed_ident.h"
#else
#error "Unknown M1_BRINGUP_MODE — use M1_BRINGUP_MODE_* in motor_params_m1.h"
#endif

/* C：观测配方 vs 模块编译闸（BUILD_*）；配方开而模块未编则硬失败。 */
#include "build_config.h"
#if M1_HFI_ENABLE && !BUILD_HFI
#error "M1_HFI_ENABLE=1 requires BUILD_HFI=1"
#endif
#if (M1_EMF_SMO_ENABLE || M1_EMF_PLL_ENABLE) && !BUILD_SMO
#error "M1_EMF_SMO/PLL_ENABLE=1 requires BUILD_SMO=1"
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
#undef M1_VOFA_IDENT_DUTY_12CH
#if (M1_BRINGUP_MODE != M1_BRINGUP_MODE_NORMAL) && \
    (M1_BRINGUP_MODE != M1_BRINGUP_MODE_SPEED_IDENT) && \
    (M1_BRINGUP_MODE != M1_BRINGUP_MODE_BODE_OFF_ONLY) && \
    (M1_BRINGUP_MODE != M1_BRINGUP_MODE_BODE_ID_OFF_ONLY)
#define M1_VOFA_IDENT_DUTY_12CH         1
#else
#define M1_VOFA_IDENT_DUTY_12CH         0
#endif

/* M1_DEADBAND_LUT_BAKED_ENABLE：建表下线闸门里强制 0，不再给 #ifndef 默认。 */

/* 模式名 #pragma message 已删；空 #if/#elif 链一并去掉。 */

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
 *       Pass0(open_seq=1..30)：ch8=id_ref ch9=id ch10=id_ref−id；看阶跃。
 *       l_hfi：ch6/7=Ud/Uq；mark55 shout 为 Ld/Lq/Lavg_uH。
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
#include "defaults_id_cal.h"
#else /* !M1_ID_LOCK_CAL_SWEEP */

#ifndef M1_ID_CAL_FIX_THETA_ENABLE
#define M1_ID_CAL_FIX_THETA_ENABLE   0
#endif

#define M1_VOFA_LUT_DUMP_ENABLE     0

#ifndef M1_VOFA_IDENT_DUMP_ENABLE
#define M1_VOFA_IDENT_DUMP_ENABLE   0
#endif

#endif /* M1_ID_LOCK_CAL_SWEEP */

/** 1=跑 deadband_flow（Id/Rs/L 或 ident/speed_ident 配方） */
#define M1_DEADBAND_FLOW_ENABLE         \
    (M1_IDENT_ENABLE || M1_ID_LOCK_CAL_SWEEP || M1_SPEED_IDENT_ENABLE || \
     M1_OPEN_UD_PRE_ID_CAL_ENABLE || M1_OPEN_UQ_PRE_ID_CAL_ENABLE)

/* --- 死区补偿（仅固定符号法；建表 LUT 已下线）--- */
/**
 * 电流 deadband：
 *   OFF（M1_DEADBAND_ENABLE=0）不过补
 *   FIXED ON：591 ns ≈0.284 V/相（产品默认；由 service 上电切 FIXED）
 */
#ifndef M1_DEADBAND_ENABLE
#define M1_DEADBAND_ENABLE      1
#endif


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

/* OPEN_UQ/UD / LUT 阶梯：建表下线闸门已强制 0；I/F 互斥见 defaults_if.h。 */
#if M1_USE_IF_100_PROFILE && !M1_IF_ENABLE
#error "M1_USE_IF_100_PROFILE requires M1_IF_ENABLE=1"
#endif
#if M1_USE_HFI_STANDSTILL_PROFILE && !M1_HFI_ENABLE
#error "M1_USE_HFI_STANDSTILL_PROFILE requires M1_HFI_ENABLE=1 (late check)"
#endif
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


#if M1_IDENT_ENABLE
#include "defaults_ident_iq.h"
#endif /* M1_IDENT_ENABLE */

/* --- 速度环辨识（旋转、deadband OFF；与 Id/Iq ident 互斥。--- */
#ifndef M1_SPEED_IDENT_ENABLE
#define M1_SPEED_IDENT_ENABLE           0
#endif

#if M1_SPEED_IDENT_ENABLE
#include "defaults_speed_ident.h"
#endif /* M1_SPEED_IDENT_ENABLE */

/**
 * 全部 profile 之后。
 * I/F 帧恒。+|Iq| 拖动（正/。ω 皆然。121：反。I/F 。+4）。
 * 速度环交。bumpless 。Continuity 同号，勿。DIR（乘 DIR 会在 BLEND 。+4→−3.5 反扭矩砸速）。
 * DIR 只作用在 ω 目标；转子系若需 −Iq 保负速，。PI 在限权内自行斜过去。
 */
#undef M1_IF_OBS_SPEED_IQ_BOOT_A
#define M1_IF_OBS_SPEED_IQ_BOOT_A       (M1_IF_IQ_A)

#endif
