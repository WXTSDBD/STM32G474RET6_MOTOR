/**
 * @file obs_inj.c
 * @date 2026-10-06
 * @brief 电压相薄壳：改写 ud/uq 与读注入，转到 HFI。
 *
 * 不属于角 ops。节拍限制见 obs_inj.h 文件头。
 *
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */
#include "observer/obs_inj.h"

#include "observer/obs_cfg.h"
#include "observer/hfi_sqwave.h"

#ifndef M1_HFI_ENABLE
#define M1_HFI_ENABLE 0
#endif

#if M1_HFI_ENABLE

/**
 * @brief 需要时改写 dq 电压。指针不可为 NULL。
 * @return 1=本拍改写了。
 */
uint8_t obs_override_voltage(float *ud, float *uq)
{
    return hfi_sqwave_override_voltage(ud, uq);
}

/**
 * @brief 读 dq 注入，单位 V。
 */
void obs_get_inj(float *ud_inj, float *uq_inj)
{
    hfi_sqwave_get_inj(ud_inj, uq_inj);
}

void obs_get_inj_ab(float *u_alpha_inj, float *u_beta_inj)
{
    hfi_sqwave_get_inj_ab(u_alpha_inj, u_beta_inj);
}

#else /* !M1_HFI_ENABLE */

uint8_t obs_override_voltage(float *ud, float *uq)
{
    (void)ud;
    (void)uq;
    return 0u;
}

void obs_get_inj(float *ud_inj, float *uq_inj)
{
    if (ud_inj != 0) {
        *ud_inj = 0.0f;
    }
    if (uq_inj != 0) {
        *uq_inj = 0.0f;
    }
}

void obs_get_inj_ab(float *u_alpha_inj, float *u_beta_inj)
{
    if (u_alpha_inj != 0) {
        *u_alpha_inj = 0.0f;
    }
    if (u_beta_inj != 0) {
        *u_beta_inj = 0.0f;
    }
}

#endif /* M1_HFI_ENABLE */
