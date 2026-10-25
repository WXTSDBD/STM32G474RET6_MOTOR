/**
 * @file exp_ff_ab.c
 * @date 2026-10-09
 * @brief 有感解耦前馈对照：同电先无 FF 再有 FF，速度 0→500→0。
 *
 * cfg0：ff_en=0；cfg1：ff_en=1。段 mark 写入 open_seq 便于切窗。
 * 判据（PC）：加速窗 id 峰值/RMS，B 应不差于 A。
 */

#include "exp_ff_ab.h"

#include <stddef.h>

#include "motor_params_m1.h"
#include "telem_table.h"


static const exp_seg_t s_ff_ab_segs_dummy[1] = {
    { 0u, EXP_ACT_NONE, NULL, 0.0f, 0.0f, 0.0f, 0u, 0u, 0u },
};

const exp_def_t g_exp_ff_ab = {
    EXP_ID_FF_AB,
    "ff_ab_0_500",
    EXP_KIND_SCHEDULE,
    s_ff_ab_segs_dummy,
    0u,
    NULL,
    0u,
    0u,
    NULL,
    0u,
};

