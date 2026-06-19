/**
 * @file factory_nvm.c
 * @brief 片内 Flash 尾区 0x0807F000（4 KB，scatter 预留）。
 */

#include "factory_nvm.h"

#include <stddef.h>
#include <string.h>

#include "stm32g4xx_hal_flash.h"
#include "stm32g4xx_hal_flash_ex.h"

#define FACTORY_NVM_BASE_ADDR  0x0807F000u
#define FACTORY_NVM_PAGE_SIZE  0x800u
#define FACTORY_NVM_PAGE_COUNT 2u

static uint32_t factory_nvm_crc32(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    uint32_t i;
    uint32_t bit;

    for (i = 0u; i < len; i++) {
        crc ^= (uint32_t)data[i];
        for (bit = 0u; bit < 8u; bit++) {
            if ((crc & 1u) != 0u) {
                crc = (crc >> 1u) ^ 0xEDB88320u;
            } else {
                crc >>= 1u;
            }
        }
    }

    return ~crc;
}

static uint32_t factory_nvm_record_crc(const factory_nvm_record_t *rec)
{
    factory_nvm_record_t tmp;

    if (rec == NULL) {
        return 0u;
    }

    tmp = *rec;
    tmp.crc32 = 0u;
    return factory_nvm_crc32((const uint8_t *)&tmp, sizeof(tmp));
}

static bool factory_nvm_read_raw(factory_nvm_record_t *out)
{
    const factory_nvm_record_t *flash_rec;

    if (out == NULL) {
        return false;
    }

    flash_rec = (const factory_nvm_record_t *)FACTORY_NVM_BASE_ADDR;
    *out = *flash_rec;

    if (out->magic != FACTORY_NVM_MAGIC || out->version != FACTORY_NVM_VERSION) {
        return false;
    }

    return (out->crc32 == factory_nvm_record_crc(out));
}

static bool factory_nvm_erase_tail(void)
{
    FLASH_EraseInitTypeDef erase;
    uint32_t page_error = 0u;
    HAL_StatusTypeDef st;

    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks = FLASH_BANK_2;
    erase.Page = (uint32_t)((FACTORY_NVM_BASE_ADDR - 0x08040000u) / FACTORY_NVM_PAGE_SIZE);
    erase.NbPages = FACTORY_NVM_PAGE_COUNT;

    if (HAL_FLASH_Unlock() != HAL_OK) {
        return false;
    }

    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
    st = HAL_FLASHEx_Erase(&erase, &page_error);
    (void)HAL_FLASH_Lock();

    return (st == HAL_OK && page_error == 0xFFFFFFFFu);
}

static bool factory_nvm_program_record(const factory_nvm_record_t *rec)
{
    const uint64_t *src;
    uint32_t addr;
    uint32_t words;
    uint32_t i;
    HAL_StatusTypeDef st;

    if (rec == NULL) {
        return false;
    }

    if (!factory_nvm_erase_tail()) {
        return false;
    }

    if (HAL_FLASH_Unlock() != HAL_OK) {
        return false;
    }

    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
    src = (const uint64_t *)rec;
    words = (uint32_t)((sizeof(factory_nvm_record_t) + 7u) / 8u);
    addr = FACTORY_NVM_BASE_ADDR;

    for (i = 0u; i < words; i++) {
        st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, src[i]);
        if (st != HAL_OK) {
            (void)HAL_FLASH_Lock();
            return false;
        }
        addr += 8u;
    }

    (void)HAL_FLASH_Lock();
    return true;
}

bool factory_nvm_load(factory_nvm_record_t *out)
{
    if (out == NULL) {
        return false;
    }

    return factory_nvm_read_raw(out);
}

bool factory_nvm_write_phase(const motor_phase_binding_t *phase)
{
    factory_nvm_record_t rec;

    if (phase == NULL || !motor_phase_binding_is_valid(phase)) {
        return false;
    }

    if (!factory_nvm_read_raw(&rec)) {
        memset(&rec, 0, sizeof(rec));
        rec.magic = FACTORY_NVM_MAGIC;
        rec.version = FACTORY_NVM_VERSION;
        rec.encoder_add = 0.0f;
    }

    rec.phase = *phase;
    rec.crc32 = factory_nvm_record_crc(&rec);

    return factory_nvm_program_record(&rec);
}

bool factory_nvm_apply_phase_binding(void)
{
    factory_nvm_record_t rec;

    if (!factory_nvm_read_raw(&rec)) {
        return false;
    }

    if (!motor_phase_binding_is_valid(&rec.phase)) {
        return false;
    }

    motor_phase_binding_set_active(&rec.phase, true);
    return true;
}
