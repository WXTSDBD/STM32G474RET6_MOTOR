/**
 * @file hfi_spd_obs.c
 * @brief SMO 转速滑动平均（GATE 141 活路径）。旧 GATE 窗/影子环已删。
 */
#include "observer/hfi_spd_obs.h"
#include "observer/obs_cfg.h"

#if M1_HFI_ENABLE && M1_HFI_MOTION_BYPASS_ENABLE && M1_EMF_SMO_ENABLE && \
    M1_EMF_PLL_ENABLE
static float s_smo_w_hist[HFI_SMO_W_MA_N];
static float s_smo_w_sum;
static uint16_t s_smo_w_i;
static uint16_t s_smo_w_fill;

void hfi_smo_w_ma_reset(void)
{
    s_smo_w_sum = 0.0f;
    s_smo_w_i = 0u;
    s_smo_w_fill = 0u;
}

float hfi_smo_w_ma_step(float rpm)
{
    if (s_smo_w_fill >= HFI_SMO_W_MA_N) {
        s_smo_w_sum -= s_smo_w_hist[s_smo_w_i];
    } else {
        s_smo_w_fill++;
    }
    s_smo_w_hist[s_smo_w_i] = rpm;
    s_smo_w_sum += rpm;
    s_smo_w_i++;
    if (s_smo_w_i >= HFI_SMO_W_MA_N) {
        s_smo_w_i = 0u;
    }
    return s_smo_w_sum / (float)s_smo_w_fill;
}
#endif
