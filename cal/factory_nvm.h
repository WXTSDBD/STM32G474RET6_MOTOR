/**
 * @file factory_nvm.h
 * @brief 片内 Flash 尾区出厂数据（相序 binding + 预留 encoder add/LUT）。
 */

#ifndef FACTORY_NVM_H
#define FACTORY_NVM_H

#include <stdbool.h>
#include <stdint.h>

#include "motor_phase_binding.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FACTORY_NVM_MAGIC    0x46414354u
#define FACTORY_NVM_VERSION  1u

typedef struct {
    uint32_t magic;
    uint32_t version;
    motor_phase_binding_t phase;
    float encoder_add;
    uint32_t reserved;
    uint32_t crc32;
} factory_nvm_record_t;

bool factory_nvm_load(factory_nvm_record_t *out);
bool factory_nvm_write_phase(const motor_phase_binding_t *phase);
bool factory_nvm_apply_phase_binding(void);

#ifdef __cplusplus
}
#endif

#endif
