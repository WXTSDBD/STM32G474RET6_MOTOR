/**
 * @file ident_module.c
 * @date 2026-10-06
 * @brief 堵转辨识激励空壳（BUILD_IDENT=0；开辨识时从 git 恢复实现）。
 *
 * 节拍限制见 ident_module.h 文件头。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */
#include "ident_module.h"

#include <stddef.h>

void ident_module_init(motor_context_t *ctx)
{
    (void)ctx;
}

void ident_module_tick(motor_context_t *ctx)
{
    (void)ctx;
}

ident_module_state_t ident_module_get_state(void)
{
    return IDENT_MOD_DONE;
}

uint8_t ident_module_get_round(void)
{
    return 0u;
}

uint8_t ident_module_get_phase_in_round(void)
{
    return 0u;
}

float ident_module_iq_ref_cmd(void)
{
    return 0.0f;
}

float ident_module_id_ref_cmd(void)
{
    return 0.0f;
}

float ident_module_bode_freq_hz(void)
{
    return 0.0f;
}

uint16_t ident_module_bode_freq_idx(void)
{
    return 0u;
}

uint8_t ident_module_step_round(void)
{
    return 0u;
}

uint8_t ident_module_bode_round(void)
{
    return 0u;
}

uint8_t ident_module_deadband_fixed(void)
{
    return 0u;
}
