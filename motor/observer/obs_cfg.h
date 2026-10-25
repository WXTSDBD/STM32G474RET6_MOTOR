/**
 * @file obs_cfg.h
 * @date 2026-10-07
 * @brief 观测器算法配置入口。不含函数原型。
 *
 * 算法 .c 应走 OBS_* 契约名，不要再直接 include motor_params_m1.h。
 * 铭牌运行时真相源是 `g_m1_motor_cfg`（init / 慢路径优先读表）。
 * OBS_* 宏仍等于 M1_*，供编译期常量与未迁热路径；节拍仍走宏。
 * 发布门槛、踢段门槛、SMO/PLL/电压方程系数的默认写在本头。
 * profile 已经写过的值优先，本头不再盖。
 *
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#ifndef MOTOR_OBSERVER_OBS_CFG_H
#define MOTOR_OBSERVER_OBS_CFG_H

#include <stdint.h>

#include "motor_cfg.h"
#include "motor_params_m1.h"

/** 定子电阻，单位 ohm。编译期默认；运行时优先 g_m1_motor_cfg.rs_ohm。 */
#ifndef OBS_RS_OHM
#define OBS_RS_OHM              M1_RS_OHM
#endif
/** d 轴电感，单位 H。 */
#ifndef OBS_LD_H
#define OBS_LD_H                M1_LD_H
#endif
/** q 轴电感，单位 H。 */
#ifndef OBS_LQ_H
#define OBS_LQ_H                M1_LQ_H
#endif
/** 极对数。电角与机械转速换算都靠它。 */
#ifndef OBS_POLE_PAIRS
#define OBS_POLE_PAIRS          M1_POLE_PAIRS
#endif
/** 控制节拍，单位 s。默认 50 us。 */
#ifndef OBS_CTRL_TS_S
#define OBS_CTRL_TS_S           M1_CTRL_TS_S
#endif

/**
 * 发布门槛，单位机械 rpm。默认；profile 可覆盖。
 * UP：未发布时用低速 ω 判断何时切到 SMO。
 * DOWN：已发布时用 SMO ω 判断何时退回 HFI。不要用卡住的 HFI 积分做退门。
 * INJ_OFF / INJ_ON：注入关断与重新打开的回滞。
 */
#ifndef M1_HFI_PUB_UP_RPM
#define M1_HFI_PUB_UP_RPM       1300.0f
#endif
#ifndef M1_HFI_PUB_DOWN_RPM
#define M1_HFI_PUB_DOWN_RPM     1000.0f
#endif
#ifndef M1_HFI_PUB_INJ_OFF_RPM
#define M1_HFI_PUB_INJ_OFF_RPM  1500.0f
#endif
#ifndef M1_HFI_PUB_INJ_ON_RPM
#define M1_HFI_PUB_INJ_ON_RPM   1400.0f
#endif
/** 退回 HFI：|ω_smo|≤DOWN 连续满足的拍数（20 kHz）。 */
#ifndef M1_HFI_PUB_DN_QUAL_N
#define M1_HFI_PUB_DN_QUAL_N    400u
#endif
/**
 * 1=SMO 在中速带「转速冻住」则强制回 HFI。
 * 假速钉死时 |ω_smo| 可能永远高于 DOWN，单靠 DOWN 退不回来。
 */
#ifndef M1_HFI_PUB_FREEZE_ENABLE
#define M1_HFI_PUB_FREEZE_ENABLE 1
#endif
#ifndef M1_HFI_PUB_FREEZE_RPM_MIN
#define M1_HFI_PUB_FREEZE_RPM_MIN 200.0f
#endif
#ifndef M1_HFI_PUB_FREEZE_RPM_MAX
#define M1_HFI_PUB_FREEZE_RPM_MAX 900.0f
#endif
/** 窗内 |Δrpm| 小于此值视为冻住（对照未滤波 ω）。 */
#ifndef M1_HFI_PUB_FREEZE_DRPM
#define M1_HFI_PUB_FREEZE_DRPM  15.0f
#endif
#ifndef M1_HFI_PUB_FREEZE_N
#define M1_HFI_PUB_FREEZE_N     400u
#endif

/** 踢段结束后再采 θ̂0 的拍数。20 kHz 下 160 拍为 8 ms。 */
#ifndef M1_HFI_QKICK_HAT0_N
#define M1_HFI_QKICK_HAT0_N             160u
#endif
/** AFTER_LOCK 切相仍可能读到的制动拍数。20 kHz 下 400 拍为 20 ms。 */
#ifndef M1_HFI_QKICK_BRAKE_N
#define M1_HFI_QKICK_BRAKE_N            400u
#endif
/**
 * 踢后极性用的残速门槛，单位 rad/s 电。
 * 无感段禁止用编码器位移判南北。正残速视为北，负残速翻 π。
 * 保持窗结束仍低于此值则默认北，不进故障锁。
 */
#ifndef M1_HFI_QKICK_POL_W_MIN_RAD_S
#define M1_HFI_QKICK_POL_W_MIN_RAD_S    5.0f
#endif

/** SMO 电阻，单位 ohm。未写时用铭牌。 */
#ifndef M1_EMF_SMO_R_OHM
#define M1_EMF_SMO_R_OHM            OBS_RS_OHM
#endif
/** SMO 电感，单位 H。未写时用铭牌 Ld。 */
#ifndef M1_EMF_SMO_L_H
#define M1_EMF_SMO_L_H              OBS_LD_H
#endif
/** SMO 滑模增益。 */
#ifndef M1_EMF_SMO_K
#define M1_EMF_SMO_K                20.0f
#endif
/** SMO 饱和限幅，单位 V。 */
#ifndef M1_EMF_SMO_SAT_A
#define M1_EMF_SMO_SAT_A            0.30f
#endif
/** SMO 反电势低通，单位 Hz。 */
#ifndef M1_EMF_SMO_LPF_HZ
#define M1_EMF_SMO_LPF_HZ           200.0f
#endif
/** 1=SMO 反电势走低通。 */
#ifndef M1_EMF_SMO_LPF_ENABLE
#define M1_EMF_SMO_LPF_ENABLE       1
#endif
/** SMO 角补偿，单位 rad。 */
#ifndef M1_EMF_SMO_THETA_OFF_RAD
#define M1_EMF_SMO_THETA_OFF_RAD    (-0.4054f)
#endif
/** 低通截止随转速线性抬升的斜率。 */
#ifndef M1_EMF_SMO_LPF_LINEAR_K
#define M1_EMF_SMO_LPF_LINEAR_K     1.5f
#endif
/** 线性调度下限，单位 Hz。 */
#ifndef M1_EMF_SMO_LPF_LINEAR_FC_MIN
#define M1_EMF_SMO_LPF_LINEAR_FC_MIN 100.0f
#endif
/** 线性调度上限，单位 Hz。profile 可覆盖。 */
#ifndef M1_EMF_SMO_LPF_LINEAR_FC_MAX
#define M1_EMF_SMO_LPF_LINEAR_FC_MAX 280.0f
#endif

/** PLL 自然频率，单位 Hz。 */
#ifndef M1_EMF_PLL_FN_HZ
#define M1_EMF_PLL_FN_HZ                100.0f
#endif
/** PLL 阻尼比。 */
#ifndef M1_EMF_PLL_ZETA
#define M1_EMF_PLL_ZETA                 0.707106781f
#endif
/** PLL 角补偿，单位 rad。 */
#ifndef M1_EMF_PLL_THETA_OFF_RAD
#define M1_EMF_PLL_THETA_OFF_RAD        (-0.4054f)
#endif
/** 1=PLL 误差按反电势幅值归一化。 */
#ifndef M1_EMF_PLL_NORM_ENABLE
#define M1_EMF_PLL_NORM_ENABLE          1
#endif
/** 归一化时 |e| 的下限，避免除零。 */
#ifndef M1_EMF_PLL_NORM_EPS
#define M1_EMF_PLL_NORM_EPS             0.05f
#endif
/** PLL 角速度限幅，单位 rad/s 电。 */
#ifndef M1_EMF_PLL_OMEGA_LIMIT_RAD_S
#define M1_EMF_PLL_OMEGA_LIMIT_RAD_S    2000.0f
#endif
/** PLL 积分限幅，单位 rad/s 电。未写时与角速度限幅相同。 */
#ifndef M1_EMF_PLL_INTEGRATOR_LIMIT_RAD_S
#define M1_EMF_PLL_INTEGRATOR_LIMIT_RAD_S M1_EMF_PLL_OMEGA_LIMIT_RAD_S
#endif

/** 电压方程电阻，单位 ohm。未写时用铭牌。 */
#ifndef M1_EMF_VEQ_R_OHM
#define M1_EMF_VEQ_R_OHM            OBS_RS_OHM
#endif
/** 电压方程电感，单位 H。未写时用铭牌 Ld。 */
#ifndef M1_EMF_VEQ_L_H
#define M1_EMF_VEQ_L_H              OBS_LD_H
#endif
/** 电压方程反电势低通，单位 Hz。 */
#ifndef M1_EMF_VEQ_LPF_HZ
#define M1_EMF_VEQ_LPF_HZ           200.0f
#endif
/** 1=电压方程反电势走低通。 */
#ifndef M1_EMF_VEQ_LPF_ENABLE
#define M1_EMF_VEQ_LPF_ENABLE       1
#endif
/** 电压方程角补偿，单位 rad。 */
#ifndef M1_EMF_VEQ_THETA_OFF_RAD
#define M1_EMF_VEQ_THETA_OFF_RAD    (-0.4054f)
#endif
/** 磁链估计的最低转速，单位机械 rpm。低于此不更新。 */
#ifndef M1_EMF_VEQ_PSI_WMIN
#define M1_EMF_VEQ_PSI_WMIN         20.0f
#endif

#endif /* MOTOR_OBSERVER_OBS_CFG_H */
