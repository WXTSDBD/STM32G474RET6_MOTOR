/**
 * @file factory_nvm.h
 * @date 2026-10-06
 * @brief 片内 Flash 尾区出厂数据（相序 binding + deadband phase LUT v2）。
 *
 * 地址真相源：`config/memory_map.h`。实现里的 `HAL_FLASH_*` 为厂商必然绑定。
 * load/write 在任务里调用。不要在电流环里擦写 Flash。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
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
    /** 1=本表有效。 */
    uint8_t valid;
    /** 每簇点数，须 ≥ 2。 */
    uint8_t len;
    /** 1=双簇，0=单簇。 */
    uint8_t two_cluster;
    /** 对齐保留。 */
    uint8_t rsv;
    /** 运行时幅值缩放，无量纲。 */
    float runtime_scale;
    /** 簇 A 电流幅值，单位 A。 */
    float amps_a[FACTORY_NVM_DEADBAND_MAX];
    /** 簇 A 补偿电压，单位 V。 */
    float vals_a[FACTORY_NVM_DEADBAND_MAX];
    /** 簇 B 电流幅值，单位 A。 */
    float amps_b[FACTORY_NVM_DEADBAND_MAX];
    /** 簇 B 补偿电压，单位 V。 */
    float vals_b[FACTORY_NVM_DEADBAND_MAX];
} factory_nvm_deadband_t;

typedef struct {
    /** FACTORY_NVM_MAGIC。 */
    uint32_t magic;
    /** 记录版本，现行为 2。 */
    uint32_t version;
    /** 出厂相序。 */
    motor_phase_binding_t phase;
    /** 编码器电角附加偏置，单位 rad。 */
    float encoder_add;
    /** 死区相表。 */
    factory_nvm_deadband_t deadband;
    /** 整条记录 CRC32。 */
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
