/**
 * @file r_sensed_pos_mit.c
 * @date 2026-10-28
 * @brief 有感位置/MIT 签收档 route（对应 m1_sensed_pos_mit.profile.h）。
 *
 * 数值抄自 profile + motor_params 默认展开。角度用编码器。
 * 外环/前馈/iref/telem ops 尚未成表，指针先空；未进 bringup 选择。
 */
#include "core/motor_route.h"

#include "plant/m1_n5065.h"

#include <stddef.h>

/**
 * 速度环 Kp=0.030 / Ki=0.004 / Ilim=5 A；位置 Kp=600 rpm/rad。
 * 斜坡 500 rpm/s；profile 未单独定 DECEL，与加速同值。
 */
static const motor_gains_t s_gains = {
    { 1000.0f, 0.54663712f, 0.03832743f, 5.0f, 13.856406f },
    { 1000.0f, 0.37070793f, 0.03832743f, 5.0f, 13.856406f },
    { 0.0f, 0.030f, 0.004f, 5.0f, 0.0f },
    { 0.0f, 600.0f, 0.0f, 0.0f, 0.0f },
    0.0f,
    500.0f,
    500.0f,
};

const motor_route_t g_route_sensed_pos_mit = {
    "m1_sensed_pos_mit",
    2u,
    &g_plant_m1_n5065,
    &s_gains,
    &g_angle_enc,
    &g_outer_mit,
    &g_ff_noop,
    &g_iref_zero_d,
    &g_telem_opt,
    MOTOR_OUTER_MIT,
    0u,
    { 0u, 0u },
};
