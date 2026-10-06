/**
 * @file deadband_service.h
 * @date 2026-10-06
 * @brief 死区补偿门面：上电、切 profile、热路径 Ud 注入。

 *
 * ud_inject 只允许从电流环节拍、PI 之后调用。
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
    /** commit 后的 runtime plut（phase abc duty） */
    DEADBAND_PROFILE_LUT_RUNTIME,
    /** motor/deadband/deadband_lut_baked_m1 */
    DEADBAND_PROFILE_LUT_BAKED,
    /** Flash NVM（无有效记录时回退 OFF） */
    DEADBAND_PROFILE_LUT_NVM,
} deadband_profile_t;

typedef struct {
    /** 1 = factory_nvm_apply_deadband 成功 */
    uint8_t nvm_loaded;
} deadband_service_boot_t;

/** 上电一次：deadband_init + 按 M1_BRINGUP_MODE 选默认 profile */
void deadband_service_boot(deadband_service_boot_t *boot);

/** 序列 handoff：deadband_init + OFF（ident 开始前等） */
void deadband_service_reset_runtime(void);

/** 切换补偿 profile；Iq/Bode 路径统一 runtime_apply_ud=0 */
void deadband_service_apply_profile(deadband_profile_t profile);

/** PI 之后叠加 d 轴 LUT 补偿（V）；无补偿时返回 0 */
float deadband_service_ud_inject(float id_a);

m1_deadband_mode_t deadband_service_get_mode(void);

#ifdef __cplusplus
}
#endif

#endif /* DEADBAND_SERVICE_H */
