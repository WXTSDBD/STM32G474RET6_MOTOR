/**
 * @file r_flux_id_1000rpm.c
 * @date 2026-10-28
 * @brief 有感 1000 rpm 磁链估 route（对应 m1_flux_id_1000rpm.profile.h）。
 *
 * 数值抄自 profile + SPEED_IDENT 下 motor_params 默认（本 profile 未改 PI）。
 * 角度用编码器；外环/前馈/iref/telem 指针先空；未进 bringup 选择。
 */
#include "core/motor_route.h"

#include "plant/m1_n5065.h"

#include <stddef.h>

/** 速度环与 SPEED_IDENT 默认一致：Kp=0.015 / Ki=0.002 / Ilim=11 A，斜坡 500。 */
static const motor_gains_t s_gains = {
    { 1000.0f, 0.54663712f, 0.03832743f, 11.0f, 13.856406f },
    { 1000.0f, 0.37070793f, 0.03832743f, 11.0f, 13.856406f },
    { 0.0f, 0.015f, 0.002f, 11.0f, 0.0f },
    { 0.0f, 80.0f, 0.0f, 0.0f, 0.0f },
    0.0f,
    500.0f,
    500.0f,
};

const motor_route_t g_route_flux_id_1000rpm = {
    "m1_flux_id_1000rpm",
    7u,
    &g_plant_m1_n5065,
    &s_gains,
    &g_angle_enc,
    &g_outer_speed,
    &g_ff_noop,
    &g_iref_zero_d,
    &g_telem_opt,
    MOTOR_OUTER_SPEED,
    0u,
    { 0u, 0u },
};
