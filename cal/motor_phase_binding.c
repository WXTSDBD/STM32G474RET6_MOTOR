/**
 * @file motor_phase_binding.c
 */

#include "motor_phase_binding.h"

#include "dbg_monitor.h"
#include "pwm_port.h"

#include <stddef.h>

static motor_phase_binding_t s_binding;
static bool s_active;

void motor_phase_binding_set_identity(motor_phase_binding_t *b)
{
    uint8_t i;

    if (b == NULL) {
        return;
    }

    b->magic = MOTOR_PHASE_BINDING_MAGIC;
    for (i = 0u; i < 3u; i++) {
        b->pwm_ch_to_phase[i] = i;
        b->adc_rank_to_phase[i] = i;
        b->phase_sign[i] = 1;
    }
    b->reserved = 0u;
}

bool motor_phase_binding_is_valid(const motor_phase_binding_t *b)
{
    uint8_t i;
    uint8_t seen_pwm[3];
    uint8_t seen_adc[3];

    if (b == NULL || b->magic != MOTOR_PHASE_BINDING_MAGIC) {
        return false;
    }

    for (i = 0u; i < 3u; i++) {
        seen_pwm[i] = 0u;
        seen_adc[i] = 0u;
    }

    for (i = 0u; i < 3u; i++) {
        if (b->pwm_ch_to_phase[i] > 2u || b->adc_rank_to_phase[i] > 2u) {
            return false;
        }
        if (b->phase_sign[i] != 1 && b->phase_sign[i] != -1) {
            return false;
        }
        seen_pwm[b->pwm_ch_to_phase[i]] = 1u;
        seen_adc[b->adc_rank_to_phase[i]] = 1u;
    }

    return (seen_pwm[0] && seen_pwm[1] && seen_pwm[2] &&
            seen_adc[0] && seen_adc[1] && seen_adc[2]);
}

void motor_phase_binding_set_active(const motor_phase_binding_t *b, bool enable)
{
    if (enable && b != NULL && motor_phase_binding_is_valid(b)) {
        s_binding = *b;
        s_active = true;
    } else if (!enable) {
        s_active = false;
    }
}

bool motor_phase_binding_is_active(void)
{
    return s_active;
}

const motor_phase_binding_t *motor_phase_binding_get(void)
{
    return &s_binding;
}

void motor_phase_binding_map_abc(const float i_phys[3], float *ia, float *ib, float *ic)
{
    float logical[3];
    uint8_t i;
    uint8_t rank;

    if (i_phys == NULL) {
        return;
    }

    if (!s_active) {
        if (ia != NULL) {
            *ia = i_phys[0];
        }
        if (ib != NULL) {
            *ib = i_phys[1];
        }
        if (ic != NULL) {
            *ic = i_phys[2];
        }
        return;
    }

    for (i = 0u; i < 3u; i++) {
        rank = s_binding.adc_rank_to_phase[i];
        logical[i] = (float)s_binding.phase_sign[i] * i_phys[rank];
    }

    if (ia != NULL) {
        *ia = logical[0];
    }
    if (ib != NULL) {
        *ib = logical[1];
    }
    if (ic != NULL) {
        *ic = logical[2];
    }
}

void motor_phase_binding_write_ccr(TIM_HandleTypeDef *htim,
                                   float ta, float tb, float tc,
                                   uint16_t pwm_period)
{
    pwm_port_t port = {
        .ops = &pwm_port_ops_stm32g4_reg,
        .hw = htim,
        .user_ctx = NULL,
    };
    float duty_logical[3];
    float duty_phys[3];
    uint8_t i;
    uint8_t ch;
    uint32_t ccr1;
    uint32_t ccr2;
    uint32_t ccr3;

    if (htim == NULL) {
        return;
    }

    duty_logical[0] = ta;
    duty_logical[1] = tb;
    duty_logical[2] = tc;
    duty_phys[0] = ta;
    duty_phys[1] = tb;
    duty_phys[2] = tc;

    if (s_active) {
        for (i = 0u; i < 3u; i++) {
            ch = s_binding.pwm_ch_to_phase[i];
            duty_phys[ch] = duty_logical[i];
        }
    }

    ccr1 = (uint32_t)(duty_phys[0] * (float)pwm_period);
    ccr2 = (uint32_t)(duty_phys[1] * (float)pwm_period);
    ccr3 = (uint32_t)(duty_phys[2] * (float)pwm_period);
    dbg.foc_pwm_ccr1 = (float)ccr1;
    dbg.foc_pwm_ccr2 = (float)ccr2;
    dbg.foc_pwm_ccr3 = (float)ccr3;
    pwm_port_set_duty3(&port, ccr1, ccr2, ccr3);
}
