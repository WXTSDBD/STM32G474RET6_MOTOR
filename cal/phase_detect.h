/**
 * @file phase_detect.h
 * @brief 出厂单相脉冲 rank/sign/gain 诊断（三档 Δ，zeroed LSB）。
 */

#ifndef PHASE_DETECT_H
#define PHASE_DETECT_H

#include <stdbool.h>
#include <stdint.h>

#include "adc.h"
#include "adc_sample.h"
#include "motor_phase_binding.h"
#include "tim.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 1：JEOC 走 phase_detect_jeoc_tick */
extern volatile uint8_t g_phase_cal_active;

/** 1：标定结束 hold（中性 PWM + VOFA，不进 FOC） */
extern volatile uint8_t g_cal_hold;

bool phase_detect_run(adc_sample_t *adc,
                      TIM_HandleTypeDef *htim,
                      motor_phase_binding_t *out,
                      bool write_flash);

void phase_detect_jeoc_tick(adc_sample_t *adc, ADC_HandleTypeDef *hadc, TIM_HandleTypeDef *htim);

void phase_detect_hold_jeoc_tick(adc_sample_t *adc, ADC_HandleTypeDef *hadc, TIM_HandleTypeDef *htim);

void phase_detect_fill_dbg(void);

#ifdef __cplusplus
}
#endif

#endif
