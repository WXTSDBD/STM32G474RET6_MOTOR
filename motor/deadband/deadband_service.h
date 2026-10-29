/**
 * @file deadband_service.h
 * @date 2026-10-06
 * @brief 死区补偿门面：上电、切 profile；仅 OFF / FIXED。

 *
 * ud_inject 已下线（恒 0）。LUT profile 枚举保留，apply 时落到 FIXED。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#ifndef DEADBAND_SERVICE_H
#define DEADBAND_SERVICE_H

#include "deadband.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    DEADBAND_PROFILE_OFF = 0,
    DEADBAND_PROFILE_FIXED,
    /** 兼容旧调用；运行时等同 FIXED（建表已下线） */
    DEADBAND_PROFILE_LUT_RUNTIME,
    DEADBAND_PROFILE_LUT_BAKED,
    DEADBAND_PROFILE_LUT_NVM,
} deadband_profile_t;

typedef struct {
    /** 建表 NVM 已下线，恒 0 */
    uint8_t nvm_loaded;
} deadband_service_boot_t;

void deadband_service_boot(deadband_service_boot_t *boot);
void deadband_service_reset_runtime(void);
void deadband_service_apply_profile(deadband_profile_t profile);
float deadband_service_ud_inject(float id_a);
m1_deadband_mode_t deadband_service_get_mode(void);

#ifdef __cplusplus
}
#endif

#endif /* DEADBAND_SERVICE_H */
