/**
 * @file r_bode_id_fc1000.c
 * @date 2026-10-28
 * @brief Id Bode 扫频档 route（对应 m1_bode_id_fc1000.profile.h）。
 *
 * 数值抄自 profile + BODE_ID 模式下 motor_params 展开：
 * 关速度环 → CLOSURE_BRINGUP 默认开 → Vlim=6 V、Ilim=0.5 A；fc=1000。
 * 标定档外环 DISABLED；outer 挂 g_outer_torque 占位（不调 tick）。
 */
#include "core/motor_route.h"

#include "plant/m1_n5065.h"

#include <stddef.h>

/**
 * 电流环 fc=1000，Vlim=6，Ilim=0.5；速度/位置环本档不跑，填 0。
 */
static const motor_gains_t s_gains = {
    { 1000.0f, 0.54663712f, 0.03832743f, 0.5f, 6.0f },
    { 1000.0f, 0.37070793f, 0.03832743f, 0.5f, 6.0f },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
    0.0f,
    0.0f,
    0.0f,
};

const motor_route_t g_route_bode_id_fc1000 = {
    "m1_bode_id_fc1000",
    8u,
    &g_plant_m1_n5065,
    &s_gains,
    &g_angle_enc,
    &g_outer_torque,
    &g_ff_noop,

    &g_iref_zero_d,

    &g_telem_opt,
    MOTOR_OUTER_DISABLED,
    0u,
    { 0u, 0u },
};
