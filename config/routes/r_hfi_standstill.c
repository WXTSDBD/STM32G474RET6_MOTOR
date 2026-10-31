/**
 * @file r_hfi_standstill.c
 * @date 2026-10-25
 * @brief 现网静置高频注入这条机器的 route。
 *
 * 数值抄自 m1_hfi_standstill.profile.h 与 motor_params_m1.h 默认展开
 * （电流环 PI = Ld/Lq/Rs·ωc·Ts；速度环 KP/KI/斜坡见 profile）。
 * 斜坡：同向 500 rpm/s，减速和反向 800 rpm/s。
 * 上电角度源是无感。有感指针由段表在 mark 不低于 0x30 时换上。
 * 外环挂 g_outer_speed（2 kHz 速度 PI）。
 * plant 共用 g_plant_m1_n5065。
 */
#include "core/motor_route.h"

#include "plant/m1_n5065.h"

#include <stddef.h>

/**
 * iq/id：fc=1000，Kp=L·ωc，Ki=Rs·ωc·Ts，Ilim=11 A，Vlim=Vbus·√3/3。
 * spd：Kp=0.005，Ki=6.5e-6，Ilim=11 A。pos：宏默认 Kp=80。
 */
static const motor_gains_t s_gains = {
    { 1000.0f, 0.54663712f, 0.03832743f, 11.0f, 13.856406f },
    { 1000.0f, 0.37070793f, 0.03832743f, 11.0f, 13.856406f },
    { 0.0f, 0.005f, 6.5e-6f, 11.0f, 0.0f },
    { 0.0f, 80.0f, 0.0f, 0.0f, 0.0f },
    0.0f,
    500.0f,
    800.0f,
};

const motor_route_t g_route_hfi_standstill = {
    "m1_hfi_standstill",
    1u,
    &g_plant_m1_n5065,
    &s_gains,
    &g_angle_hfi,
    &g_outer_speed,
    &g_ff_noop,
    &g_iref_zero_d,
    &g_telem_opt,
    MOTOR_OUTER_SPEED,
    0u,
    { 0u, 0u },
};
