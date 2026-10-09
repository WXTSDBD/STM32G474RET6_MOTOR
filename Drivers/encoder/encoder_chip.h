/**
 * @file encoder_chip.h
 * @date 2026-10-08
 * @brief 编码器芯片静态注册表：id、SPI 模式、位宽、driver。
 *
 * 查表绑定实例；不是运行时工厂。新芯片只追加枚举值和表项。
 */

#ifndef ENCODER_CHIP_H
#define ENCODER_CHIP_H

#include <stddef.h>
#include <stdint.h>

#include "encoder.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ENC_CHIP_NONE = 0,
    ENC_CHIP_AS5047 = 1,
    ENC_CHIP_KTH7823 = 2,
} encoder_chip_id_t;

typedef struct {
    /** 芯片 id。 */
    encoder_chip_id_t id;
    /** 调试名。 */
    const char *name;
    /** SPI 模式：1 或 3。 */
    uint8_t spi_mode;
    /** 角度 raw 位宽。 */
    uint8_t resolution_bits;
    /** 热路径每采样帧数。 */
    uint8_t frames_per_sample;
    /** 芯片驱动。 */
    const encoder_driver_t *drv;
} encoder_chip_desc_t;

const encoder_chip_desc_t *encoder_chip_lookup(encoder_chip_id_t id);
const encoder_chip_desc_t *const *encoder_chip_table(size_t *n);

#ifdef __cplusplus
}
#endif

#endif
