/**
 * @file angle_route.c
 * @date 2026-10-25
 * @brief 两条角度源：无感 HFI，有感编码器。
 *
 * 无感源的 Park、转速和极性不使用编码器。
 * 有感源只在段边界被装上之后，Park 和速度反馈才取编码器。
 */
#include "core/motor_route.h"

#include <stddef.h>

#include "motor_current.h"
#include "observer/obs_angle.h"
#include "observer/observer_composite.h"

/**
 * @brief 无感：推进 HFI。编码器角只进对照。
 * @param ctx 未使用。
 * @param dt_s 节拍，单位 s。
 */
static void angle_hfi_update(motor_context_t *ctx, float dt_s)
{
    (void)ctx;
    obs_pre_park(motor_angle_enc_theta(), dt_s);
}

/**
 * @brief 无感 Park 角，单位 rad。
 * @param ctx 未使用。
 * @note 走 observer_ops 的取角。已发布时那里换成滑模角，不是编码器角。
 */
static float angle_hfi_theta(motor_context_t *ctx)
{
    const observer_ops_t *ops;

    (void)ctx;
    ops = observer_ops();
    if ((ops != NULL) && (ops->get_theta != NULL)) {
        return ops->get_theta();
    }
    return obs_get_theta();
}

/**
 * @brief 无感速度反馈。发布后用滑模转速，否则用 HFI 的 PLL 积分。
 * @param ctx 提供极对数。
 * @return 机械转速，单位 rpm。
 */
static float angle_hfi_rpm(motor_context_t *ctx)
{
    float rpm_scale;

    rpm_scale = 60.0f / (2.0f * 3.14159265f * (float)ctx->pole_pairs);
    if (observer_pub_ss() >= 1.0f) {
        return observer_pub_smo_rpm();
    }
    return observer_get_pll_int_el() * rpm_scale;
}

/**
 * @brief 无感解调。有感源没有这一项。
 */
static void angle_hfi_post(motor_context_t *ctx, float id, float iq,
                           float i_alpha, float i_beta)
{
    (void)ctx;
    obs_post_park(id, iq, i_alpha, i_beta);
}

/**
 * @brief 有感：只推阶段时钟和编码器转速低通。
 * @param ctx 未使用。
 * @param dt_s 节拍，单位 s。
 */
static void angle_enc_update(motor_context_t *ctx, float dt_s)
{
    (void)ctx;
    observer_sensed_keep(motor_angle_enc_theta(), dt_s);
}

/**
 * @brief 有感 Park 角，就是本拍编码器电角。
 * @param ctx 未使用。
 */
static float angle_enc_theta(motor_context_t *ctx)
{
    (void)ctx;
    return motor_angle_enc_theta();
}

/**
 * @brief 有感速度反馈，编码器 PLL。
 * @param ctx 未使用。
 */
static float angle_enc_rpm(motor_context_t *ctx)
{
    (void)ctx;
    return motor_current_get_enc_pll_omega_mech_rpm();
}

const angle_ops_t g_angle_hfi = {
    angle_hfi_update,
    angle_hfi_theta,
    angle_hfi_rpm,
    angle_hfi_post,
    NULL,
};

const angle_ops_t g_angle_enc = {
    angle_enc_update,
    angle_enc_theta,
    angle_enc_rpm,
    NULL,
    NULL,
};
