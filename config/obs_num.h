/**
 * @file obs_num.h
 * @date 2026-10-25
 * @brief 滑模、反电势 PLL、踢段的 const 数。
 *
 * 热路径读这里的字段，不再直接读对应宏。
 * 高频注入 PLL 的 Kp/Ki 在 hfi_pll_cfg.c 的 g_hfi_pll_cfg，不重复放。
 * 数值由当前 profile 在编译期填进表，换 profile 会重编这张表。
 */
#ifndef CONFIG_OBS_NUM_H
#define CONFIG_OBS_NUM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 滑模与反电势 PLL。单位见各字段。 */
typedef struct {
    float smo_r_ohm;
    float smo_l_h;
    float smo_k;
    float smo_sat_a;
    float smo_theta_off_rad;
    float smo_lpf_hz;
    float smo_lpf_k;
    float smo_lpf_fc_min_hz;
    float smo_lpf_fc_max_hz;
    float pll_fn_hz;
    float pll_zeta;
    float pll_theta_off_rad;
    float pll_norm_eps;
    float pll_omega_limit_rad_s;
    float pll_integrator_limit_rad_s;
} obs_num_t;

/** 高频注入里已经收成表的踢段和注入幅值。 */
typedef struct {
    float vh_v;
    float qkick_iq_a;
    float pol_w_min_rad_s;
    float qkick_pre_s;
    float qkick_hold_s;
    uint16_t qkick_hat0_n;
    uint16_t qkick_kick_n;
    uint16_t qkick_brake_n;
} hfi_num_t;

extern const obs_num_t g_obs_num;
extern const hfi_num_t g_hfi_num;

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_OBS_NUM_H */
