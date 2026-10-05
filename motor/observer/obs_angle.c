/**
 * @file obs_angle.c
 * @brief P6 thin angle shell. Park 角；pub/HAND overlay 在 Composite get_theta。
 */
#include "observer/obs_angle.h"

#include "observer/obs_cfg.h"
#include "observer/hfi_sqwave.h"

#ifndef M1_HFI_ENABLE
#define M1_HFI_ENABLE 0
#endif

void obs_angle_bind_pll(void *pll)
{
    (void)pll;
}

#if M1_HFI_ENABLE

static float s_theta_enc;

void obs_pre_park(float theta_enc, float dt)
{
    s_theta_enc = theta_enc;
    hfi_sqwave_on_angle(theta_enc, dt);
}

float obs_get_theta(void)
{
    return hfi_sqwave_park_theta(s_theta_enc);
}

void obs_post_park(float id, float iq, float i_alpha, float i_beta)
{
    hfi_sqwave_on_current(id, iq, i_alpha, i_beta);
}

#else /* !M1_HFI_ENABLE */

void obs_pre_park(float theta_enc, float dt)
{
    (void)theta_enc;
    (void)dt;
}

float obs_get_theta(void)
{
    return 0.0f;
}

void obs_post_park(float id, float iq, float i_alpha, float i_beta)
{
    (void)id;
    (void)iq;
    (void)i_alpha;
    (void)i_beta;
}

#endif /* M1_HFI_ENABLE */
