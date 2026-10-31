/**
 * @file exp_act_basic.c
 * @date 2026-10-09
 * @brief HOLD / OMEGA_REF 内建动作实现。
 */

#include "exp_act_basic.h"

#include <stddef.h>

#include "motor_outer_loop.h"
#include "telemetry/obs_preset.h"

/* table-driven experiment state: fixed angle / Id injection / angle source */
static uint8_t s_angle_src;      /* 0=observer, 1=encoder */
static uint8_t s_angle_src_on;
/**
 * @brief HOLD：不改指令，外环继续跟当前 omega_ref。
 */
static void exp_act_hold_enter(motor_context_t *ctx, const exp_seg_t *seg)
{
    (void)ctx;
    (void)seg;
}

/**
 * @brief HOLD 段内无额外更新。
 */
static void exp_act_hold_update(motor_context_t *ctx, const exp_seg_t *seg, float t_s)
{
    (void)ctx;
    (void)seg;
    (void)t_s;
}

/**
 * @brief 速度给定：p0 = 目标转速 rpm，写入 omega_ref 并同步斜坡。
 */
static void exp_act_omega_enter(motor_context_t *ctx, const exp_seg_t *seg)
{
    if ((ctx == NULL) || (seg == NULL)) {
        return;
    }
    ctx->omega_ref = seg->p0;
    motor_outer_set_omega_ramp_rpm(seg->p0);
    exp_act_angle_from_mark(seg->mark);
}

/**
 * @brief 速度给定段内保持 enter 写入的目标。
 */
static void exp_act_omega_update(motor_context_t *ctx, const exp_seg_t *seg, float t_s)
{
    (void)ctx;
    (void)seg;
    (void)t_s;
}

const exp_action_ops_t g_exp_act_hold = {
    exp_act_hold_enter,
    exp_act_hold_update,
};

const exp_action_ops_t g_exp_act_omega_ref = {
    exp_act_omega_enter,
    exp_act_omega_update,
};

/* ===== 固定 Park 角 + d 轴电流注入（Ud/Id 锁轴标定；阶段1 pilot） ===== */
static float s_theta_ref_deg;   /**< 固定电角，单位 deg。 */
static float s_theta_ref_id_a;  /**< 注入 Id，单位 A。 */
static uint8_t s_theta_ref_on;  /**< 1=正在锁轴注入。 */

static void exp_act_theta_enter(motor_context_t *ctx, const exp_seg_t *seg)
{
    if ((ctx == NULL) || (seg == NULL)) {
        return;
    }
    s_theta_ref_deg = seg->p0;
    s_theta_ref_id_a = seg->p1;
    s_theta_ref_on = 1u;
}

static void exp_act_theta_update(motor_context_t *ctx, const exp_seg_t *seg, float t_s)
{
    (void)ctx;
    (void)seg;
    (void)t_s;
}

const exp_action_ops_t g_exp_act_theta_ref = {
    exp_act_theta_enter,
    exp_act_theta_update,
};

uint8_t exp_act_theta_is_active(void)
{
    return s_theta_ref_on;
}

float exp_act_theta_get_rad(void)
{
    return s_theta_ref_deg * 0.01745329252f;
}

float exp_act_theta_get_id_a(void)
{
    return s_theta_ref_id_a;
}

void exp_act_theta_clear(void)
{
    s_theta_ref_on = 0u;
    s_angle_src_on = 0u;
}

uint8_t exp_act_angle_src_is_active(void)
{
    return s_angle_src_on;
}

uint8_t exp_act_angle_src(void)
{
    return s_angle_src;
}

/**
 * @brief 按段 mark 选择角度源。mark 不低于 0x30 用编码器，否则用观测角。
 * @param mark 段打点号。
 */
void exp_act_angle_from_mark(uint8_t mark)
{
    s_angle_src = (mark >= 0x30u) ? 1u : 0u;
    s_angle_src_on = 1u;
}

/**
 * @brief 切观测预设：p0=preset_id，p1=decim（0=默认）。
 */
static void exp_act_set_obs_enter(motor_context_t *ctx, const exp_seg_t *seg)
{
    uint8_t id;
    uint8_t decim;

    (void)ctx;
    if (seg == NULL) {
        return;
    }
    id = (uint8_t)seg->p0;
    decim = (uint8_t)seg->p1;
    (void)obs_preset_apply(id, decim);
}

static void exp_act_set_obs_update(motor_context_t *ctx, const exp_seg_t *seg,
                                   float t_s)
{
    (void)ctx;
    (void)seg;
    (void)t_s;
}

static const exp_action_ops_t g_exp_act_set_obs = {
    exp_act_set_obs_enter,
    exp_act_set_obs_update,
};

/**
 * @brief 按默认动作 id 取内建 ops。
 * @param act EXP_ACT_*。
 * @return ops；未知 id 为 NULL。
 */
const exp_action_ops_t *exp_act_basic_lookup(uint8_t act)
{
    switch (act) {
    case EXP_ACT_HOLD:
        return &g_exp_act_hold;
    case EXP_ACT_OMEGA_REF:
        return &g_exp_act_omega_ref;
    case EXP_ACT_THETA_REF:
        return &g_exp_act_theta_ref;
    case EXP_ACT_SET_OBS:
        return &g_exp_act_set_obs;
    default:
        return NULL;
    }
}
