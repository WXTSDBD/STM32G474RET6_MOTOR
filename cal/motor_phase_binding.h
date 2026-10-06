/**
 * @file motor_phase_binding.h
 * @date 2026-10-06
 * @brief 出厂 PWM↔ADC 相序 binding；Clarke/SVPWM 公式不变，仅在边界 remap。
 *
 * map_abc 和 write_ccr 从电流环节拍调用。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
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
    /** 固定魔数 MOTOR_PHASE_BINDING_MAGIC。 */
    uint32_t magic;
    /** PWM 通道下标 0..2 → 逻辑相 0=A,1=B,2=C。 */
    uint8_t pwm_ch_to_phase[3];
    /** ADC rank 0..2 → 逻辑相。 */
    uint8_t adc_rank_to_phase[3];
    /** 逻辑相电流符号，只能是 +1 或 -1。 */
    int8_t phase_sign[3];
    /** 对齐保留，写 0。 */
    uint8_t reserved;
} motor_phase_binding_t;

void motor_phase_binding_set_identity(motor_phase_binding_t *b);
bool motor_phase_binding_is_valid(const motor_phase_binding_t *b);

void motor_phase_binding_set_active(const motor_phase_binding_t *b, bool enable);
bool motor_phase_binding_is_active(void);
const motor_phase_binding_t *motor_phase_binding_get(void);

void motor_phase_binding_map_abc(const float i_phys[3], float *ia, float *ib, float *ic);

void motor_phase_binding_write_ccr(TIM_HandleTypeDef *htim,
                                   float ta, float tb, float tc,
                                   uint16_t pwm_period);

#ifdef __cplusplus
}
#endif

#endif
