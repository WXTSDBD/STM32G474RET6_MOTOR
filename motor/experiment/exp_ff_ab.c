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

#if M1_EXP_FF_AB_ENABLE

/** A：无解耦前馈 */
static const exp_cfg_t s_ff_ab_cfg[] = {
    { 0u, 0u, 0u, 0u, 0.0f, 0.0f }, /* ff off */
    { 1u, 0u, 0u, 0u, 0.0f, 0.0f }, /* ff on  */
};

/**
 * 时间表（外环 2 kHz）：
 * A hold0 2s → A 500 3s → A hold0 2s → B hold0 1s → B 500 3s → B hold0 2s
 */
static const exp_seg_t s_ff_ab_segs[] = {
    { 2000u, EXP_ACT_OMEGA_REF, NULL, 0.0f,   0.0f, 0.0f, 0u, 10u, 0u },
    { 3000u, EXP_ACT_OMEGA_REF, NULL, 500.0f, 0.0f, 0.0f, 0u, 11u, 0u },
    { 2000u, EXP_ACT_OMEGA_REF, NULL, 0.0f,   0.0f, 0.0f, 0u, 12u, 0u },
    { 1000u, EXP_ACT_OMEGA_REF, NULL, 0.0f,   0.0f, 0.0f, 1u, 20u, 0u },
    { 3000u, EXP_ACT_OMEGA_REF, NULL, 500.0f, 0.0f, 0.0f, 1u, 21u, 0u },
    { 2000u, EXP_ACT_OMEGA_REF, NULL, 0.0f,   0.0f, 0.0f, 1u, 22u, 0u },
};

const exp_def_t g_exp_ff_ab = {
    EXP_ID_FF_AB,
    "ff_ab_0_500",
    EXP_KIND_SCHEDULE,
    s_ff_ab_segs,
    (uint16_t)(sizeof(s_ff_ab_segs) / sizeof(s_ff_ab_segs[0])),
    s_ff_ab_cfg,
    2u,
    TELEM_LAYOUT_SPEED,
    NULL,
};

#else /* 未编入：占位，避免空引用 */

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
};

#endif /* M1_EXP_FF_AB_ENABLE */
