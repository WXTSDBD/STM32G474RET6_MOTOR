/**
 * @file deadband_service.c
 * @date 2026-10-06
 * @brief 死区补偿门面：上电与 profile 切换；仅 OFF / FIXED。

 *
 * 建表 LUT / baked / NVM plut 已下线。ud_inject 恒 0。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#include "deadband_service.h"

#include <stddef.h>

#include "build_config.h"
#include "motor_params_m1.h"

/**
 * @brief 上电：init；BUILD_DEADBAND 且 ENABLE 时默认 FIXED。
 */
void deadband_service_boot(deadband_service_boot_t *boot)
{
    deadband_init();

#if BUILD_DEADBAND && M1_DEADBAND_ENABLE
    deadband_service_apply_profile(DEADBAND_PROFILE_FIXED);
#else
    deadband_service_apply_profile(DEADBAND_PROFILE_OFF);
#endif

    if (boot != NULL) {
        boot->nvm_loaded = 0u;
    }
}

/**
 * @brief 回到 OFF。辨识开始前用。
 */
void deadband_service_reset_runtime(void)
{
    deadband_init();
    deadband_service_apply_profile(DEADBAND_PROFILE_OFF);
}

void deadband_service_apply_profile(deadband_profile_t profile)
{
    switch (profile) {
    case DEADBAND_PROFILE_FIXED:
    case DEADBAND_PROFILE_LUT_RUNTIME: /* LUT 下线：与 FIXED 同归宿 */
    case DEADBAND_PROFILE_LUT_BAKED:
    case DEADBAND_PROFILE_LUT_NVM:
#if BUILD_DEADBAND && M1_DEADBAND_ENABLE
        deadband_set_mode(M1_DEADBAND_MODE_FIXED);
#else
        deadband_set_mode(M1_DEADBAND_MODE_OFF);
#endif
        deadband_set_runtime_apply_ud(0u);
        break;

    case DEADBAND_PROFILE_OFF:
    default:
        deadband_set_mode(M1_DEADBAND_MODE_OFF);
        deadband_set_runtime_apply_ud(0u);
        break;
    }
}

/**
 * @brief PI 后 d 轴注入已下线，恒 0。
 */
float deadband_service_ud_inject(float id_a)
{
    (void)id_a;
    return 0.0f;
}

m1_deadband_mode_t deadband_service_get_mode(void)
{
    return deadband_get_mode();
}
