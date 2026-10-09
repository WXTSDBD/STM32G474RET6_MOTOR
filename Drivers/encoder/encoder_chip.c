/**
 * @file encoder_chip.c
 * @date 2026-10-08
 * @brief 编码器芯片静态表与 lookup。
 */

#include "encoder_chip.h"

#include "as5047.h"
#include "kth7823.h"

static const encoder_chip_desc_t s_as5047_desc = {
    .id = ENC_CHIP_AS5047,
    .name = "AS5047",
    .spi_mode = 1u,
    .resolution_bits = 14u,
    .frames_per_sample = 2u,
    .drv = &as5047_encoder_driver,
};

static const encoder_chip_desc_t s_kth7823_desc = {
    .id = ENC_CHIP_KTH7823,
    .name = "KTH7823",
    .spi_mode = 3u,
    .resolution_bits = 16u,
    .frames_per_sample = 1u,
    .drv = &kth7823_encoder_driver,
};

static const encoder_chip_desc_t *const s_table[] = {
    &s_as5047_desc,
    &s_kth7823_desc,
};

/**
 * @brief 按 id 查芯片描述；没有则 NULL。
 */
const encoder_chip_desc_t *encoder_chip_lookup(encoder_chip_id_t id)
{
    size_t i;

    for (i = 0U; i < (sizeof(s_table) / sizeof(s_table[0])); i++) {
        if (s_table[i]->id == id) {
            return s_table[i];
        }
    }
    return NULL;
}

/**
 * @brief 返回静态表指针与长度。
 */
const encoder_chip_desc_t *const *encoder_chip_table(size_t *n)
{
    if (n != NULL) {
        *n = sizeof(s_table) / sizeof(s_table[0]);
    }
    return s_table;
}
