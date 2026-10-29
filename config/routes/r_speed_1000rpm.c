/**
 * @file r_speed_1000rpm.c
 * @date 2026-10-28
 * @brief 有感 1000 rpm 速度环交付 route（对应 m1_speed_1000rpm.profile.h）。
 *
 * 数值抄自 profile + SPEED_IDENT 下 motor_params 默认。角度用编码器。
 * 外环/前馈/iref/telem ops 尚未成表，指针先空；未进 bringup 选择。
 */
#include "core/motor_route.h"

#include "plant/m1_n5065.h"

#include <stddef.h>

/**
 * 速度环 Kp=0.015（默认）/ Ki=0.002（profile）/ Ilim=11 A。
 * 斜坡 500 rpm/s（SPEED_IDENT 默认；profile 未改 DECEL）。
 */
static const motor_gains_t s_gains = {
    { 1000.0f, 0.54663712f, 0.03832743f, 11.0f, 13.856406f },
    { 1000.0f, 0.37070793f, 0.03832743f, 11.0f, 13.856406f },
    { 0.0f, 0.015f, 0.002f, 11.0f, 0.0f },
    { 0.0f, 80.0f, 0.0f, 0.0f, 0.0f },
    0.0f,
    500.0f,
    500.0f,
};

const motor_route_t g_route_speed_1000rpm = {
    "m1_speed_1000rpm",
    5u,
    &g_plant_m1_n5065,
    &s_gains,
    &g_angle_enc,
    NULL,
    NULL,
    NULL,
    NULL,
    MOTOR_OUTER_SPEED,
    0u,
    { 0u, 0u },
};
