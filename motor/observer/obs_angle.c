/**
 * @file obs_angle.c
 * @date 2026-10-06
 * @brief 角路径薄壳：Park 前、取角、Park 后转到 HFI。
 *
 * 发布覆盖不在本文件。节拍限制见 obs_angle.h 文件头。
 * 能力闸门用 if (M1_HFI_ENABLE)。
 *
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */
#include "observer/obs_angle.h"

#include "observer/obs_cfg.h"
#include "observer/hfi_sqwave.h"

/** 本拍编码器电角，单位 rad。取 Park 时再交给 HFI。 */
static float s_theta_enc;

/**
 * @brief 空壳。PLL 由 Composite 持有，这里丢掉指针。
 * @param pll 忽略。
 */
void obs_angle_bind_pll(void *pll)
{
    (void)pll;
}

/**
 * @brief Park 前：记下电角并推进 HFI。
 * @param theta_enc 电角，单位 rad。
 * @param dt 节拍，单位 s。
 */
void obs_pre_park(float theta_enc, float dt)
{
    if (!M1_HFI_ENABLE) {
        (void)theta_enc;
        (void)dt;
        return;
    }
    s_theta_enc = theta_enc;
    hfi_sqwave_on_angle(theta_enc, dt);
}

/**
 * @brief 取本拍 Park 电角，单位 rad。不含发布覆盖。
 */
float obs_get_theta(void)
{
    if (!M1_HFI_ENABLE) {
        return 0.0f;
    }
    return hfi_sqwave_park_theta(s_theta_enc);
}

void obs_post_park(float id, float iq, float i_alpha, float i_beta)
{
    if (!M1_HFI_ENABLE) {
        (void)id;
        (void)iq;
        (void)i_alpha;
        (void)i_beta;
        return;
    }
    hfi_sqwave_on_current(id, iq, i_alpha, i_beta);
}
