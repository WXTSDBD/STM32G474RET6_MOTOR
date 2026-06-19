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

/** 母线电压（V），限幅用 Vbus/√3 */
#define M1_VBUS_V           24.0f
#define M1_PI_V_MAX         (M1_VBUS_V * 0.577350269f)
#define M1_PI_V_MIN         (-M1_PI_V_MAX)

/** PI 带宽（Hz）；M1_PI_USE_FIXED_GAIN=0 时用于 L×ωc 整定 */
#define M1_PI_FC_HZ         200.0f
#define M1_PI_WC_RADS       (2.0f * 3.14159265359f * M1_PI_FC_HZ)

/** 联调：1=固定 Kp/Ki（试凑）；0=按 fc 整定 */
#ifndef M1_PI_USE_FIXED_GAIN
#define M1_PI_USE_FIXED_GAIN  1
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

/* --- 死区补偿（固定符号法，591 ns = MCU 140 ns + FD6288 200 ns + 余量） --- */
/** 联调 A/B：0=关 deadband；1=固定符号补偿 ~0.284V/相（试验结论：应 ON） */
#ifndef M1_DEADBAND_ENABLE
#define M1_DEADBAND_ENABLE      1
#endif

/** 有效死区时间（ns），含驱动器插入死区 */
#define M1_DEADTIME_NS          591u

/** PWM 周期（s），与 M1_CTRL_TS_S / TIM8 20 kHz 一致 */
#define M1_PWM_PERIOD_S         M1_CTRL_TS_S

/** 每相固定补偿电压：Vbus × t_dead / T_pwm ≈ 0.284 V @ 24 V */
#define M1_DEADBAND_V_COMP_V    (M1_VBUS_V * (float)M1_DEADTIME_NS * 1.0e-9f / M1_PWM_PERIOD_S)

/** 归一化占空比补偿量 = t_dead / T_pwm */
#define M1_DEADBAND_DUTY_COMP   (M1_DEADBAND_V_COMP_V / M1_VBUS_V)

/** 过零区：|i| 低于此值不补偿 */
#define M1_DEADBAND_I_ZERO_A    0.05f

/* --- 开环联调：先 Uq=0 静止采样，再切运行电压（offset vs PWM 分岔实验） --- */
#ifndef M1_OPEN_SEQ_ENABLE
#define M1_OPEN_SEQ_ENABLE      1
#endif

/** 静止段时长（s），@ 20 kHz JEOC */
#define M1_OPEN_IDLE_S            10.0f

/** 静止段结束后开环 Uq（V） */
#define M1_OPEN_UQ_RUN_V          2.0f

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

#endif
