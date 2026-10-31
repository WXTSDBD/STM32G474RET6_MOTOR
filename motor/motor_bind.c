/**
 * @file motor_bind.c
 * @date 2026-10-25
 * @brief 保存 route、角度与外环指针；上电按 M1_ACTIVE_ROUTE_ID 选表。
 *
 * 20 kHz / 2 kHz 只读这些指针。换角度走 motor_angle_use（段边界）；
 * 换外环走 motor_outer_use（模式切换时可调）。
 */
#include "core/motor_route.h"

#include <stddef.h>

#include "motor_params_m1.h"
#include "route_ids.h"

/** 当前 route。 */
static const motor_route_t *s_route;
/** 当前角度源。段边界可以换成另一张表。 */
static const angle_ops_t *s_angle;
/** 当前外环。SPEED/TORQUE 热路径优先走这张表。 */
static const outer_ops_t *s_outer;
/** 前馈 / 电流给定（route 绑定时装上）。 */
static const ff_ops_t *s_ff;
static const iref_ops_t *s_iref;
/** 本拍编码器电角，单位 rad。 */
static float s_enc_theta;

/**
 * @brief 绑定 route：写入 ctx 的 plant/gains，并装上角度/外环指针。
 * @param ctx 电机上下文；NULL 时只保存 route/angle/outer。
 * @param route 目标 route。NULL 表示卸下。
 */
void motor_route_bind(motor_context_t *ctx, const motor_route_t *route)
{
    s_route = route;
    s_angle = (route != NULL) ? route->angle : NULL;
    s_outer = (route != NULL) ? route->outer : NULL;
    s_ff = (route != NULL) ? route->ff : NULL;
    s_iref = (route != NULL) ? route->iref : NULL;
    if (ctx != NULL) {
        if (route != NULL) {
            ctx->plant = route->plant;
            ctx->gains = route->gains;
        } else {
            ctx->plant = NULL;
            ctx->gains = NULL;
        }
    }
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
 * @brief 上电选路（S4a）：bringup 的 M1_ACTIVE_ROUTE_ID 优先。
 * @note ACTIVE_ROUTE_ID 为编译期常量；未引用的 r_*.o 仍由链接器剔除。
 */
const motor_route_t *motor_route_for_boot(void)
{
    switch ((int)M1_ACTIVE_ROUTE_ID) {
    case (int)M1_ROUTE_ID_ID_CAL_PASS0_RS:
        return &g_route_id_cal_pass0_rs;
    case (int)M1_ROUTE_ID_HFI_STANDSTILL:
        return &g_route_hfi_standstill;
    case (int)M1_ROUTE_ID_SENSED_POS_MIT:
        return &g_route_sensed_pos_mit;
    case (int)M1_ROUTE_ID_SENSORLESS_POS_MIT:
        return &g_route_sensorless_pos_mit;
    case (int)M1_ROUTE_ID_IF_100RPM:
        return &g_route_if_100rpm;
    case (int)M1_ROUTE_ID_SPEED_1000RPM:
        return &g_route_speed_1000rpm;
    case (int)M1_ROUTE_ID_OBS_VEQ_1000RPM:
        return &g_route_obs_veq_1000rpm;
    case (int)M1_ROUTE_ID_FLUX_ID_1000RPM:
        return &g_route_flux_id_1000rpm;
    case (int)M1_ROUTE_ID_BODE_ID_FC1000:
        return &g_route_bode_id_fc1000;
    default:
        return &g_route_hfi_standstill;
    }
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

/**
 * @brief 更换外环表。
 * @param ops 新的外环表。NULL 表示卸下，热路径回落 switch。
 */
void motor_outer_use(const outer_ops_t *ops)
{
    s_outer = ops;
}

/**
 * @brief 取当前外环表。
 */
const outer_ops_t *motor_outer_active(void)
{
    return s_outer;
}

/**
 * @brief 取当前前馈表。
 */
const ff_ops_t *motor_ff_active(void)
{
    return s_ff;
}

/**
 * @brief 取当前电流给定表。
 */
const iref_ops_t *motor_iref_active(void)
{
    return s_iref;
}
