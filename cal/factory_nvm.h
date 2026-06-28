/**
 * @file factory_nvm.h
 * @brief 片内 Flash 尾区出厂数据（相序 binding + deadband phase LUT v2）。
 */

#ifndef FACTORY_NVM_H
#define FACTORY_NVM_H

#include <stdbool.h>
#include <stdint.h>

#include "motor_phase_binding.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FACTORY_NVM_MAGIC           0x46414354u
#define FACTORY_NVM_VERSION         2u
#define FACTORY_NVM_VERSION_V1      1u
#define FACTORY_NVM_DEADBAND_MAX    32u

typedef struct {
    uint8_t valid;
    uint8_t len;
    uint8_t two_cluster;
    uint8_t rsv;
    float runtime_scale;
    float amps_a[FACTORY_NVM_DEADBAND_MAX];
    float vals_a[FACTORY_NVM_DEADBAND_MAX];
    float amps_b[FACTORY_NVM_DEADBAND_MAX];
    float vals_b[FACTORY_NVM_DEADBAND_MAX];
} factory_nvm_deadband_t;

typedef struct {
    uint32_t magic;
    uint32_t version;
    motor_phase_binding_t phase;
    float encoder_add;
    factory_nvm_deadband_t deadband;
    uint32_t crc32;
} factory_nvm_record_t;

bool factory_nvm_load(factory_nvm_record_t *out);
bool factory_nvm_write_phase(const motor_phase_binding_t *phase);
bool factory_nvm_write_deadband(const factory_nvm_deadband_t *deadband);
bool factory_nvm_apply_phase_binding(void);
bool factory_nvm_apply_deadband(void);

#ifdef __cplusplus
}
#endif

#endif
