/**
 * @file obs_num.c
 * @date 2026-10-25
 * @brief 用当前 profile 的宏填观测器 const。
 *
 * 只在本文件读这些宏。算法文件改读 g_obs_num / g_hfi_num。
 */
#include "obs_num.h"

#include "observer/obs_cfg.h"

const obs_num_t g_obs_num = {
    M1_EMF_SMO_R_OHM,
    M1_EMF_SMO_L_H,
    M1_EMF_SMO_K,
    M1_EMF_SMO_SAT_A,
    M1_EMF_SMO_THETA_OFF_RAD,
    M1_EMF_SMO_LPF_HZ,
    M1_EMF_SMO_LPF_LINEAR_K,
    M1_EMF_SMO_LPF_LINEAR_FC_MIN,
    M1_EMF_SMO_LPF_LINEAR_FC_MAX,
    M1_EMF_PLL_FN_HZ,
    M1_EMF_PLL_ZETA,
    M1_EMF_PLL_THETA_OFF_RAD,
    M1_EMF_PLL_NORM_EPS,
    M1_EMF_PLL_OMEGA_LIMIT_RAD_S,
    M1_EMF_PLL_INTEGRATOR_LIMIT_RAD_S,
};

const hfi_num_t g_hfi_num = {
    M1_HFI_VH_V,
    M1_HFI_QKICK_IQ_A,
    M1_HFI_QKICK_POL_W_MIN_RAD_S,
    M1_HFI_QKICK_PRE_S,
    M1_HFI_QKICK_HOLD_S,
    (uint16_t)M1_HFI_QKICK_HAT0_N,
    (uint16_t)M1_HFI_QKICK_KICK_N,
    (uint16_t)M1_HFI_QKICK_BRAKE_N,
};
