/**
 * @file motor_bind.c
 * @date 2026-10-25
 * @brief 保存 route 和当前角度指针。
 *
 * 20 kHz 只读这两个指针。更换角度源走 motor_angle_use，由段边界调用。
 */
#include "core/motor_route.h"

#include <stddef.h>

/** 当前 route。 */
static const motor_route_t *s_route;
/** 当前角度源。段边界可以换成另一张表。 */
static const angle_ops_t *s_angle;
/** 本拍编码器电角，单位 rad。 */
static float s_enc_theta;

/**
 * @brief 绑定 route，并把角度指针设成 route 里的那张。
 * @param ctx 未使用。
 * @param route 目标 route。NULL 表示卸下。
 */
void motor_route_bind(motor_context_t *ctx, const motor_route_t *route)
{
    (void)ctx;
    s_route = route;
    s_angle = (route != NULL) ? route->angle : NULL;
}

/**
 * @brief 取当前 route。
 * @return 未绑定时为 NULL。
 */
const motor_route_t *motor_route_active(void)
{
    return s_route;
}

/**
 * @brief 段边界更换角度源。
 * @param ops 新的角度表。NULL 表示卸下。
 */
void motor_angle_use(const angle_ops_t *ops)
{
    s_angle = ops;
}

/**
 * @brief 取当前角度源。
 */
const angle_ops_t *motor_angle_active(void)
{
    return s_angle;
}

/**
 * @brief 记下本拍编码器电角。
 * @param theta_el 电角，单位 rad。
 */
void motor_angle_note_enc(float theta_el)
{
    s_enc_theta = theta_el;
}

/**
 * @brief 取本拍记下的编码器电角。
 */
float motor_angle_enc_theta(void)
{
    return s_enc_theta;
}

/**
 * @brief 1=当前源要跑无感解调。
 */
uint8_t motor_angle_sensorless(void)
{
    if (s_angle == NULL) {
        return 0u;
    }
    return (s_angle->post_park != NULL) ? 1u : 0u;
}
