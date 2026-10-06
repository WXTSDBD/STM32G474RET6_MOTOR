/**
 * @file hfi_spd_obs.c
 * @date 2026-10-06
 * @brief SMO 机械转速滑动平均。
 *
 * 给发布门槛用。不是观测器配置项。节拍限制见 hfi_spd_obs.h 文件头。
 *
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */
#include "observer/hfi_spd_obs.h"
#include "observer/obs_cfg.h"

#if M1_HFI_ENABLE && M1_HFI_MOTION_BYPASS_ENABLE && M1_EMF_SMO_ENABLE && \
    M1_EMF_PLL_ENABLE
/** 窗口采样，单位 rpm。 */
static float s_smo_w_hist[HFI_SMO_W_MA_N];
/** 窗口内求和，单位 rpm。 */
static float s_smo_w_sum;
/** 下一格下标。 */
static uint16_t s_smo_w_i;
/** 已经填入的格数，未满窗时当除数。 */
static uint16_t s_smo_w_fill;

/**
 * @brief 清窗口。不把历史数组逐格清零，下一步会覆盖。
 */
void hfi_smo_w_ma_reset(void)
{
    s_smo_w_sum = 0.0f;
    s_smo_w_i = 0u;
    s_smo_w_fill = 0u;
}

/**
 * @brief 推进一拍滑动平均。
 * @param rpm 本拍机械转速，单位 rpm。
 * @return 窗口平均，单位 rpm。
 */
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
