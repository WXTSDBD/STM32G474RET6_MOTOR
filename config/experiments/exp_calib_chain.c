/**
 * @file exp_calib_chain.c
 * @date 2026-10-26
 * @brief 联合标定表：只跑 Rs + VESC 旋转 HF+DFT 测 L（无 Pass0、无 LUT、无 VASI）。
 *
 * bringup：ALIGN → Rs@OFF → l_hfi×N（duty 爬 + inv_L bin0/2）。
 * 遥测紧凑 6ch@20kHz，便于离线还原双极半拍。
 */
#include "experiment/exp_tables.h"

#include <stddef.h>

#include "deadband_id_cal.h"
#include "telem_table.h"

#define SEG(dur, act, a, b, c, mark) \
    { (uint16_t)(dur), (uint8_t)(act), NULL, (a), (b), (c), 0u, (uint8_t)(mark), 0u }

static const exp_cfg_t s_cfg_default[] = {
    { 0u, 0u, 0u, 0u, 0.0f, 0.0f },
};

static const exp_seg_t s_calib_segs[] = {
    SEG(0u, EXP_ACT_HOLD, 0.0f, 0.0f, 0.0f, 0x10),
    SEG(0u, EXP_ACT_HOLD, 0.0f, 0.0f, 0.0f, 0x20),
    SEG(0u, EXP_ACT_HOLD, 0.0f, 0.0f, 0.0f, 0x30),
    SEG(0u, EXP_ACT_HOLD, 0.0f, 0.0f, 0.0f, 0x40),
};

static uint8_t s_script_done;

/**
 * @brief 武装：IDENT layout；Pass0/LUT 已关，等 id_cal 自跑 Rs → HF-L。
 */
static void calib_chain_script_init(motor_context_t *ctx, const exp_cfg_t *cfg)
{
    (void)ctx;
    (void)cfg;
    s_script_done = 0u;
    telem_set_layout(TELEM_LAYOUT_IDENT_DUTY);
    /* flow boot_rs_ld_lq_only 已置 l_hfi；此处不挂 Pass0 continue。 */
    deadband_id_cal_set_pass0_continue(NULL);
}

/**
 * @brief 2 kHz：只看整链 DONE。
 */
static void calib_chain_script_tick(motor_context_t *ctx, float theta_fb_rad,
                                   float omega_rpm)
{
    (void)ctx;
    (void)theta_fb_rad;
    (void)omega_rpm;
    if (s_script_done != 0u) {
        return;
    }
    if (deadband_id_cal_is_done() != 0u) {
        s_script_done = 1u;
    }
}

/**
 * @brief 1=Rs+L 结束。
 */
static uint8_t calib_chain_script_done(const motor_context_t *ctx)
{
    (void)ctx;
    return s_script_done;
}

static const exp_script_ops_t s_calib_chain_script = {
    calib_chain_script_init,
    calib_chain_script_tick,
    calib_chain_script_done,
};

const exp_def_t g_exp_calib_chain = {
    EXP_ID_CALIB_CHAIN,
    "calib_rs_lhfi_only",
    EXP_KIND_SCRIPT,
    s_calib_segs,
    (uint16_t)(sizeof(s_calib_segs) / sizeof(s_calib_segs[0])),
    s_cfg_default,
    1u,
    TELEM_LAYOUT_IDENT_DUTY,
    &s_calib_chain_script,
    1u,
};
