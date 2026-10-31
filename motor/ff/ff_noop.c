/**
 * @file ff_noop.c
 * @date 2026-10-29
 * @brief 空前馈：不改 ud/uq（S6.5 占位，现网无独立电压前馈链）。
 */
#include "core/motor_ops.h"

#include <stddef.h>

/**
 * @brief 前馈空操作。
 */
static void ff_noop_apply(motor_context_t *ctx, float *ud_out, float *uq_out,
                          float id, float iq)
{
    (void)ctx;
    (void)ud_out;
    (void)uq_out;
    (void)id;
    (void)iq;
}

const ff_ops_t g_ff_noop = {
    ff_noop_apply,
};
