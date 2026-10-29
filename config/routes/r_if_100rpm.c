/**
 * @file r_if_100rpm.c
 * @date 2026-10-28
 * @brief I-f 爬升再软切观测 route（对应 m1_if_100rpm.profile.h）。
 *
 * 数值抄自 profile + motor_params 默认展开。尚无 I-f 专用 angle ops，
 * angle 指针先空；外环/前馈/iref/telem 亦空。未进 bringup 选择。
 */
#include "core/motor_route.h"

#include "plant/m1_n5065.h"

#include <stddef.h>

/**
 * 速度环 Kp=0.0008 / Ki=5e-5 / Ilim=3.5 A；斜坡 200 rpm/s。
 * 电流环 Ilim 取 SPEED_IDENT 默认 11 A（profile 未改 M1_I_REF_ABS_MAX）。
 */
static const motor_gains_t s_gains = {
    { 1000.0f, 0.54663712f, 0.03832743f, 11.0f, 13.856406f },
    { 1000.0f, 0.37070793f, 0.03832743f, 11.0f, 13.856406f },
    { 0.0f, 0.0008f, 0.00005f, 3.5f, 0.0f },
    { 0.0f, 80.0f, 0.0f, 0.0f, 0.0f },
    0.0f,
    200.0f,
    200.0f,
};

const motor_route_t g_route_if_100rpm = {
    "m1_if_100rpm",
    4u,
    &g_plant_m1_n5065,
    &s_gains,
    NULL,
    &g_outer_speed,
    NULL,
    NULL,
    NULL,
    MOTOR_OUTER_SPEED,
    0u,
    { 0u, 0u },
};
