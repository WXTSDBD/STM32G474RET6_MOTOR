/**
 * @file exp_rev141_params.c
 * @date 2026-10-25
 * @brief ±2000 金样实验参数数据（原 `M1_HFI_REV141_*` 宏，阶段 2 表化）。
 *
 * 改实验参数只改这里，不再碰 profile / motor_params 的宏。
 */
#include "exp_rev141_params.h"

/** 与改造前 profile 覆盖值逐项一致：2 对无感 + 1 对有感，±2000 rpm。 */
const exp_rev141_params_t g_exp_rev141_params = {
    2u,       /* pairs */
    1u,       /* sensed_pairs */
    2000.0f,  /* target_rpm */
    80.0f,    /* stop_rpm */
    0.3f,     /* stop_hold_s */
    3.0f,     /* brake_timeout_s */
    0.02f,    /* enc_rpm_lp_a */
};
