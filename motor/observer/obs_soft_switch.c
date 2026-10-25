/**
 * @file obs_soft_switch.c
 * @date 2026-10-06
 * @brief 编码器或 I-f 角到观测角的软切状态机。
 *
 * 角先进观测，速度可以稍后切。节拍限制见 obs_soft_switch.h 文件头。
 *
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#include "obs_soft_switch.h"

#include "observer/obs_cfg.h"

void obs_soft_switch_init(void)
{
}

void obs_soft_switch_reset(void)
{
}

obs_ss_state_t obs_soft_switch_get_state(void)
{
    return OBS_SS_ENC;
}

float obs_soft_switch_get_alpha(void)
{
    return 0.0f;
}

uint8_t obs_soft_switch_speed_on_obs(void)
{
    return 0u;
}

uint8_t obs_soft_switch_speed_use_obs(void)
{
    return 0u;
}

float obs_soft_switch_apply(float theta_enc,
                            float theta_hat,
                            float theta_err,
                            float emag,
                            float omega_enc_rpm,
                            float omega_obs_rpm,
                            float omega_ref,
                            float iq,
                            float dt)
{
    (void)theta_hat;
    (void)theta_err;
    (void)emag;
    (void)omega_enc_rpm;
    (void)omega_obs_rpm;
    (void)omega_ref;
    (void)iq;
    (void)dt;
    return theta_enc;
}
