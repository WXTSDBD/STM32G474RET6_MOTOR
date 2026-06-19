/**
 * @file motor_phase_binding.h
 * @brief 出厂 PWM↔ADC 相序 binding；Clarke/SVPWM 公式不变，仅在边界 remap。
 */

#ifndef MOTOR_PHASE_BINDING_H
#define MOTOR_PHASE_BINDING_H

#include <stdbool.h>
#include <stdint.h>

#include "tim.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_PHASE_BINDING_MAGIC  0x50423131u

typedef struct {
    uint32_t magic;
    uint8_t pwm_ch_to_phase[3];
    uint8_t adc_rank_to_phase[3];
    int8_t phase_sign[3];
    uint8_t reserved;
} motor_phase_binding_t;

void motor_phase_binding_set_identity(motor_phase_binding_t *b);
bool motor_phase_binding_is_valid(const motor_phase_binding_t *b);

void motor_phase_binding_set_active(const motor_phase_binding_t *b, bool enable);
bool motor_phase_binding_is_active(void);
const motor_phase_binding_t *motor_phase_binding_get(void);

/**
 * @brief 物理 JDR 顺序电流 [0]=JDR1… → 逻辑 Ia/Ib/Ic。
 */
void motor_phase_binding_map_abc(const float i_phys[3], float *ia, float *ib, float *ic);

/** @brief 逻辑归一化占空 Ta/Tb/Tc → 写 TIM CCR（含 pwm 通道 remap）。 */
void motor_phase_binding_write_ccr(TIM_HandleTypeDef *htim,
                                   float ta, float tb, float tc,
                                   uint16_t pwm_period);

#ifdef __cplusplus
}
#endif

#endif
