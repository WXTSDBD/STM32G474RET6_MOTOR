/**
 * @file telem_ops_default.c
 * @date 2026-10-29
 * @brief 默认遥测布局占位（OPT12 id=4；真正填帧仍走 telem_table）。
 */
#include "core/motor_ops.h"

const telem_ops_t g_telem_opt = {
    4u,
    "opt12",
};
