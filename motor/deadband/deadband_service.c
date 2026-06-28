/**
 * @file deadband_service.c
 * @brief 死区补偿 Service 门面实现。
 */

#include "deadband_service.h"

#include "deadband_cal.h"
#include "factory_nvm.h"
#include "motor_params_m1.h"

#if M1_DEADBAND_LUT_BAKED_ENABLE
#include "deadband_lut_baked_m1.h"
#endif

static void deadband_service_apply_lut_runtime_abc(void)
{
    if (deadband_cal_len() >= 2u) {
        deadband_cal_switch_runtime_lut(0u);
    } else {
        deadband_set_mode(M1_DEADBAND_MODE_OFF);
    }
    deadband_set_runtime_apply_ud(0u);
}

void deadband_service_boot(deadband_service_boot_t *boot)
{
    uint8_t nvm_loaded = 0u;

    deadband_init();

#if M1_IDENT_ENABLE && !M1_IDENT_ID_CAL_BEFORE_STEP
    /* ident_module_init → deadband_service_reset_runtime */
#elif M1_ID_LOCK_CAL_SWEEP || (M1_IDENT_ENABLE && M1_IDENT_ID_CAL_BEFORE_STEP)
    deadband_service_apply_profile(DEADBAND_PROFILE_OFF);
#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_NORMAL) && M1_DEADBAND_ENABLE && \
    M1_DEADBAND_LUT_BAKED_ENABLE
    if (!deadband_lut_baked_m1_apply()) {
        deadband_service_apply_profile(DEADBAND_PROFILE_OFF);
    }
#elif (M1_BRINGUP_MODE == M1_BRINGUP_MODE_NORMAL) && M1_DEADBAND_ENABLE && \
    M1_DEADBAND_NVM_ON_BOOT
    nvm_loaded = factory_nvm_apply_deadband() ? 1u : 0u;
    if (nvm_loaded == 0u) {
        deadband_service_apply_profile(DEADBAND_PROFILE_OFF);
    }
#elif M1_OPEN_UQ_DEADBAND_AB_SWEEP
    deadband_service_apply_profile(DEADBAND_PROFILE_OFF);
#endif

    if (boot != NULL) {
#if (M1_BRINGUP_MODE != M1_BRINGUP_MODE_NORMAL) || !M1_DEADBAND_NVM_ON_BOOT
        boot->nvm_loaded = 0u;
#else
        boot->nvm_loaded = nvm_loaded;
#endif
    }
}

void deadband_service_reset_runtime(void)
{
    deadband_init();
    deadband_service_apply_profile(DEADBAND_PROFILE_OFF);
}

void deadband_service_apply_profile(deadband_profile_t profile)
{
    switch (profile) {
    case DEADBAND_PROFILE_OFF:
        deadband_set_mode(M1_DEADBAND_MODE_OFF);
        deadband_set_runtime_apply_ud(0u);
        break;

    case DEADBAND_PROFILE_FIXED:
        deadband_set_mode(M1_DEADBAND_MODE_FIXED);
        deadband_set_runtime_apply_ud(0u);
        break;

    case DEADBAND_PROFILE_LUT_RUNTIME:
        deadband_service_apply_lut_runtime_abc();
        break;

    case DEADBAND_PROFILE_LUT_BAKED:
#if M1_DEADBAND_LUT_BAKED_ENABLE
        if (!deadband_lut_baked_m1_apply()) {
            deadband_set_mode(M1_DEADBAND_MODE_OFF);
            deadband_set_runtime_apply_ud(0u);
        }
#else
        deadband_set_mode(M1_DEADBAND_MODE_OFF);
        deadband_set_runtime_apply_ud(0u);
#endif
        break;

    case DEADBAND_PROFILE_LUT_NVM:
        if (!factory_nvm_apply_deadband()) {
            deadband_set_mode(M1_DEADBAND_MODE_OFF);
        }
        deadband_set_runtime_apply_ud(0u);
        break;

    default:
        deadband_set_mode(M1_DEADBAND_MODE_OFF);
        deadband_set_runtime_apply_ud(0u);
        break;
    }
}

float deadband_service_ud_inject(float id_a)
{
#if M1_DEADBAND_LUT_APPLY_UD
    if (deadband_get_mode() == M1_DEADBAND_MODE_LUT) {
        return deadband_ud_comp_v(id_a);
    }
#else
    (void)id_a;
#endif
    return 0.0f;
}

m1_deadband_mode_t deadband_service_get_mode(void)
{
    return deadband_get_mode();
}
