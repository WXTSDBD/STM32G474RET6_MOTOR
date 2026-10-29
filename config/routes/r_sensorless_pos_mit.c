/**
 * @file r_sensorless_pos_mit.c
 * @date 2026-10-28
 * @brief 无感位置/MIT 冒烟档 route（对应 m1_sensorless_pos_mit.profile.h）。
 *
 * 数值抄自 profile + motor_params 默认展开。角度上电用 HFI。
 * 外环/前馈/iref/telem ops 尚未成表，指针先空；未进 bringup 选择。
 */
#include "core/motor_route.h"

#include "plant/m1_n5065.h"

#include <stddef.h>

/**
 * 速度环 Kp=0.005 / Ki=6.5e-6 / Ilim=5 A；位置 Kp=20 rpm/rad。
 * profile 关 ω 斜坡；表内斜坡填 0。
 */
static const motor_gains_t s_gains = {
    { 1000.0f, 0.54663712f, 0.03832743f, 5.0f, 13.856406f },
    { 1000.0f, 0.37070793f, 0.03832743f, 5.0f, 13.856406f },
    { 0.0f, 0.005f, 6.5e-6f, 5.0f, 0.0f },
    { 0.0f, 20.0f, 0.0f, 0.0f, 0.0f },
    0.0f,
    0.0f,
    0.0f,
};

const motor_route_t g_route_sensorless_pos_mit = {
    "m1_sensorless_pos_mit",
    3u,
    &g_plant_m1_n5065,
    &s_gains,
    &g_angle_hfi,
    NULL,
    NULL,
    NULL,
    NULL,
    MOTOR_OUTER_MIT,
    0u,
    { 0u, 0u },
};
