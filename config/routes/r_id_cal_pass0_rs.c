/**
 * @file r_id_cal_pass0_rs.c
 * @date 2026-10-28
 * @brief PASS0_RS 标定档 route（bringup M1_ACTIVE_ROUTE_ID=9 现网选路）。
 *
 * 关速度环 → CLOSURE_BRINGUP=1 → Ilim=0.5 A、Vlim=6 V；电流环 fc=1000。
 * plant 共用 g_plant_m1_n5065；HFI/观测数值仍在 m1_hfi_compile_nums.h。
 */
#include "core/motor_route.h"

#include "plant/m1_n5065.h"

#include <stddef.h>

/**
 * 电流环 fc=1000，CLOSURE 限幅；速度/位置本档不跑，斜坡填 0。
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

const motor_route_t g_route_id_cal_pass0_rs = {
    "m1_id_cal_pass0_rs",
    9u,
    &g_plant_m1_n5065,
    &s_gains,
    &g_angle_hfi,
    NULL,
    NULL,
    NULL,
    NULL,
    MOTOR_OUTER_DISABLED,
    0u,
    { 0u, 0u },
};
