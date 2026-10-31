/**
 * @file iref_zero_d.c
 * @date 2026-10-29
 * @brief 电流给定：id*=0，iq* 透传（S6.5）。
 *
 * HFI 旁路仍可在 20 kHz 后改写 id_ref；本表只覆盖外环写出的默认。
 */
#include "core/motor_ops.h"

#include <stddef.h>

/**
 * @brief id 置零，iq 跟指令。
 */
static void iref_zero_d_ref(motor_context_t *ctx, float iq_cmd,
                            float *id_ref, float *iq_ref)
{
    (void)ctx;
    *id_ref = 0.0f;
    *iq_ref = iq_cmd;
}

const iref_ops_t g_iref_zero_d = {
    iref_zero_d_ref,
};
