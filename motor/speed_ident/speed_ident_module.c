/**
 * @file speed_ident_module.c
 * @date 2026-10-06
 * @brief 速度环辨识激励空壳（BUILD_IDENT=0；开辨识时从 git 恢复实现）。
 *
 * tick 随外环调用。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */
#include "speed_ident_module.h"

#include <stddef.h>

void speed_ident_module_init(motor_context_t *ctx)
{
    (void)ctx;
}

void speed_ident_module_tick(motor_context_t *ctx)
{
    (void)ctx;
}

speed_ident_module_state_t speed_ident_module_get_state(void)
{
    return SPEED_IDENT_DONE;
}

uint8_t speed_ident_module_get_round(void)
{
    return 0u;
}

uint8_t speed_ident_module_get_phase_in_round(void)
{
    return 0u;
}

float speed_ident_module_omega_ref_cmd(void)
{
    return 0.0f;
}

float speed_ident_module_bode_freq_hz(void)
{
    return 0.0f;
}

uint8_t speed_ident_module_step_round(void)
{
    return 0u;
}

uint8_t speed_ident_module_bode_round(void)
{
    return 0u;
}

uint8_t speed_ident_module_hold_iq_inhibit(void)
{
    return 0u;
}
