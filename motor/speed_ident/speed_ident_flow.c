/**
 * @file speed_ident_flow.c
 * @date 2026-10-06
 * @brief 速度环辨识流程空壳（BUILD_IDENT=0）。
 */
#include "speed_ident_flow.h"

#include <stddef.h>

void speed_ident_flow_init(motor_context_t *ctx)
{
    (void)ctx;
}

void speed_ident_flow_tick(motor_context_t *ctx)
{
    (void)ctx;
}

uint8_t speed_ident_flow_is_armed(void)
{
    return 0u;
}
