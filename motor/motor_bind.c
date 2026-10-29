/**
 * @file motor_bind.c
 * @date 2026-10-25
 * @brief 保存 route 和当前角度指针；上电按 M1_ACTIVE_ROUTE_ID 选表。
 *
 * 20 kHz 只读这两个指针。更换角度源走 motor_angle_use，由段边界调用。
 */
#include "core/motor_route.h"

#include <stddef.h>

#include "motor_params_m1.h"
#include "route_ids.h"

/** 当前 route。 */
static const motor_route_t *s_route;
/** 当前角度源。段边界可以换成另一张表。 */
static const angle_ops_t *s_angle;
/** 本拍编码器电角，单位 rad。 */
static float s_enc_theta;

/**
 * @brief 绑定 route：写入 ctx 的 plant/gains，并装上角度指针。
 * @param ctx 电机上下文；NULL 时只保存 route/angle。
 * @param route 目标 route。NULL 表示卸下。
 */
void motor_route_bind(motor_context_t *ctx, const motor_route_t *route)
{
    s_route = route;
    s_angle = (route != NULL) ? route->angle : NULL;
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
 * @note 各分支为编译期互斥；现网只留一条引用，其余 r_*.o 由链接器剔除。
 */
const motor_route_t *motor_route_for_boot(void)
{
#if M1_ACTIVE_ROUTE_ID == M1_ROUTE_ID_ID_CAL_PASS0_RS
    return &g_route_id_cal_pass0_rs;
#elif M1_ACTIVE_ROUTE_ID == M1_ROUTE_ID_HFI_STANDSTILL
    return &g_route_hfi_standstill;
#elif M1_ACTIVE_ROUTE_ID == M1_ROUTE_ID_SENSED_POS_MIT
    return &g_route_sensed_pos_mit;
#elif M1_ACTIVE_ROUTE_ID == M1_ROUTE_ID_SENSORLESS_POS_MIT
    return &g_route_sensorless_pos_mit;
#elif M1_ACTIVE_ROUTE_ID == M1_ROUTE_ID_IF_100RPM
    return &g_route_if_100rpm;
#elif M1_ACTIVE_ROUTE_ID == M1_ROUTE_ID_SPEED_1000RPM
    return &g_route_speed_1000rpm;
#elif M1_ACTIVE_ROUTE_ID == M1_ROUTE_ID_OBS_VEQ_1000RPM
    return &g_route_obs_veq_1000rpm;
#elif M1_ACTIVE_ROUTE_ID == M1_ROUTE_ID_FLUX_ID_1000RPM
    return &g_route_flux_id_1000rpm;
#elif M1_ACTIVE_ROUTE_ID == M1_ROUTE_ID_BODE_ID_FC1000
    return &g_route_bode_id_fc1000;
#elif M1_ACTIVE_ROUTE_ID == M1_ROUTE_ID_AUTO
#if M1_ID_CAL_RS_LD_LQ_ONLY_ENABLE
    return &g_route_id_cal_pass0_rs;
#else
    return &g_route_hfi_standstill;
#endif
#else
    return &g_route_hfi_standstill;
#endif
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
