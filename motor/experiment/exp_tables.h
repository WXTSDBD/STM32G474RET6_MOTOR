/**
 * @file exp_tables.h
 * @date 2026-10-25
 * @brief 实验段表（宏 → const 数据）对外声明。
 *
 * 目的：把原先靠 `M1_*_ENABLE` / `M1_*_PARAM` 宏描述的实验，改成"一张 const 表 + 注册表一行"。
 * 本文件只声明 `g_exp_*`。±2000 在 config/experiments/exp_rev141.c，其余在 exp_tables.c。
 * 执行器仍只在 2 kHz 外环跑（exp_runner_tick）；本表默认不启用任何实验，
 * 上电跑什么由 exp_runner_select(id) 决定（调用点仍在原处，后续改为运行时选）。
 */
#ifndef MOTOR_EXPERIMENT_EXP_TABLES_H
#define MOTOR_EXPERIMENT_EXP_TABLES_H

#include "exp_def.h"

#ifdef __cplusplus
extern "C" {
#endif

/** ±2000 rpm 正反转（2 轮无感 + 1 轮有感）—— 原 M1_HFI_REV141_* */
extern const exp_def_t g_exp_rev141;
/** 编码器零偏：12 步 Id 锁轴（6 角度 × 正反）—— 原 M1_UD_PROBE_* */
extern const exp_def_t g_exp_ud_cal;
/** 速度阶梯（原 M1_SPEED_IDENT_STEP_RPM0..3） */
extern const exp_def_t g_exp_speed_ladder;
/** 位置阶跃（原 M1_POS_STEP_TEST_ENABLE） */
extern const exp_def_t g_exp_pos_step;
/** MIT 往返（原 M1_OUTER_EXPT_MIT_REV） */
extern const exp_def_t g_exp_mit_rev;
/** 速度反转纯测试（原 M1_SPEED_REVERSAL_TEST_ENABLE） */
extern const exp_def_t g_exp_speed_rev;
/** 开环 uq 扫描（原 M1_OPEN_UQ_SWEEP_*） */
extern const exp_def_t g_exp_uq_sweep;
/** IPD 扫频（原 M1_HFI_IPD_SWEEP_ENABLE） */
extern const exp_def_t g_exp_ipd_sweep;
/** 参数辨识（Rs/Ld/Lq/ψf，原 M1_IDENT_ENABLE 族；激励参数在表里，ops 待补） */
extern const exp_def_t g_exp_ident;
/** 电流环扫频/Bode（原 M1_BODE_* / M1_BRINGUP_MODE_BODE_*） */
extern const exp_def_t g_exp_bode;
/** VASI Ld/Lq 扫（原 M1_VASI_*） */
extern const exp_def_t g_exp_vasi;
/** I/F 开环拖动+巡航（原 M1_IF_ENABLE / M1_IF_OBS_CRUISE_*） */
extern const exp_def_t g_exp_if_drag;
/** 死区 LUT 在线标定（原 M1_DEADBAND_*） */
extern const exp_def_t g_exp_deadband_lut;
/** QKICK 极性判决（原 M1_HFI_QKICK_*，正反转的前置段） */
extern const exp_def_t g_exp_qkick;
/** 联合标定链：Pass0→Rs→LdLq（config/experiments/exp_calib_chain.c） */
extern const exp_def_t g_exp_calib_chain;

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_EXPERIMENT_EXP_TABLES_H */
