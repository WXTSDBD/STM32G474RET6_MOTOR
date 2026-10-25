/**
 * @file exp_cfg.c
 * @date 2026-10-09
 * @brief 把 exp_cfg_t 写进 ctx->atrb（E3）。
 *
 * ff_en → 解耦前馈位；eso_en → ESO 位；deadband_en → 死区位。
 * 摩擦前馈本刀不进 cfg（仍由上电初值 / 手工改 atrb）；避免一次 A/B 误关两条。
 */

#include "exp_cfg.h"

#include <stddef.h>

/**
 * @brief 按 cfg 覆盖 atrb 中由实验管理的位。
 * @param ctx 控制上下文。不可为 NULL。
 * @param cfg 实验配置。NULL 则直接返回。
 * @note 0/1 语义：写死开或关，不是 keep。签收 SCRIPT 勿盲目套用全 0 表。
 */
void exp_cfg_apply(motor_context_t *ctx, const exp_cfg_t *cfg)
{
    uint8_t atrb;

    if ((ctx == NULL) || (cfg == NULL)) {
        return;
    }

    atrb = ctx->atrb;

    if (cfg->ff_en != 0u) {
        atrb |= (uint8_t)M1_ATRB_DECOUP_FF;
    } else {
        atrb = (uint8_t)(atrb & (uint8_t)~M1_ATRB_DECOUP_FF);
    }

    if (cfg->eso_en != 0u) {
        atrb |= (uint8_t)M1_ATRB_ESO;
    } else {
        atrb = (uint8_t)(atrb & (uint8_t)~M1_ATRB_ESO);
    }

    if (cfg->deadband_en != 0u) {
        atrb |= (uint8_t)M1_ATRB_DEADBAND;
    } else {
        atrb = (uint8_t)(atrb & (uint8_t)~M1_ATRB_DEADBAND);
    }

    (void)cfg->theta_src; /* 角源运行时切换后置，本刀不写 */
    (void)cfg->pi_kp;
    (void)cfg->pi_ki;

    ctx->atrb = atrb;
}
