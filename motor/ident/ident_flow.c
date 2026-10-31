/**
 * @file ident_flow.c
 * @date 2026-10-06
 * @brief 堵转辨识流程空壳（BUILD_IDENT=0）。
 */
#include "ident_flow.h"

#include <stddef.h>

void ident_flow_init(motor_context_t *ctx)
{
    (void)ctx;
}

void ident_flow_tick(motor_context_t *ctx)
{
    (void)ctx;
}

uint8_t ident_flow_deadband_fixed(void)
{
    return 0u;
}
