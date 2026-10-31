/**
 * @file outer_ops.c
 * @date 2026-10-29
 * @brief 四外环 ops 常量表（力矩 / 速度 / 位置 / MIT）。选路由绑表，热路径不解宏。
 */
#include "core/motor_ops.h"
#include "core/motor_route.h"

#include <stddef.h>

float outer_ops_torque_tick(motor_context_t *ctx,
                            float theta_fb_rad,
                            float omega_rpm);
float outer_ops_speed_tick(motor_context_t *ctx,
                           float theta_fb_rad,
                           float omega_rpm);
float outer_ops_pos_tick(motor_context_t *ctx,
                         float theta_fb_rad,
                         float omega_rpm);
void outer_ops_pos_enter(motor_context_t *ctx);
float outer_ops_mit_tick(motor_context_t *ctx,
                         float theta_fb_rad,
                         float omega_rpm);
void outer_ops_mit_enter(motor_context_t *ctx);

const outer_ops_t g_outer_torque = {
    outer_ops_torque_tick,
    NULL,
};

const outer_ops_t g_outer_speed = {
    outer_ops_speed_tick,
    NULL,
};

const outer_ops_t g_outer_pos = {
    outer_ops_pos_tick,
    outer_ops_pos_enter,
};

const outer_ops_t g_outer_mit = {
    outer_ops_mit_tick,
    outer_ops_mit_enter,
};
