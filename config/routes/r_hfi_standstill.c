/**
 * @file r_hfi_standstill.c
 * @date 2026-10-25
 * @brief 现网静置高频注入这条机器的 route。
 *
 * 斜坡 500/800 rpm/s 从当前 profile 填进 gains，外环读这张表。
 * 电流环和速度环 PI 仍走原来的宏，不在本表。
 * 上电角度源是无感。有感指针由段表在 mark 不低于 0x30 时换上。
 */
#include "core/motor_route.h"

#include <stddef.h>

#include "motor_params_m1.h"

static const motor_gains_t s_gains = {
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
    0.0f,
    M1_SPEED_OMEGA_RAMP_RPM_S,
#if defined(M1_SPEED_OMEGA_RAMP_DECEL_RPM_S)
    M1_SPEED_OMEGA_RAMP_DECEL_RPM_S,
#else
    M1_SPEED_OMEGA_RAMP_RPM_S,
#endif
};

const motor_route_t g_route_hfi_standstill = {
    "m1_hfi_standstill",
    1u,
    NULL,
    &s_gains,
    &g_angle_hfi,
    NULL,
    NULL,
    NULL,
    NULL,
    0u,
    0u,
    { 0u, 0u },
};
