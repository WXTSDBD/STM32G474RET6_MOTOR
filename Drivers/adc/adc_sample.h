/**
 * @file adc_sample.h
 * @brief 三相 shunt 电流 ADC 采样：配置 schema 与对外 API（无 HAL 类型）。
 *
 * 板级 HAL 句柄由 config/bsp_axes + bridge_cubemx 填入 cfg 中的 void* 字段。
 * STM32 实现见 platform/adc_sample_stm32g4.c。
 */

#ifndef ADC_SAMPLE_H
#define ADC_SAMPLE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ADC_SAMPLE_TOPO_SCAN,
    ADC_SAMPLE_TOPO_PER_PHASE,
} adc_sample_topo_t;

typedef struct {
    void *hadc;
    uint8_t jdr_index;
    int32_t offset;
    float scale;
} adc_sample_ch_cfg_t;

typedef struct {
    adc_sample_topo_t topo;
    uint8_t n_ch;
    adc_sample_ch_cfg_t ch[3];
    void *trig_tim;
    uint32_t trig_ch;
    void *pwm_tim;
} adc_sample_config_t;

typedef struct {
    adc_sample_config_t *cfg;
    int16_t raw[3];
    float ia;
    float ib;
    float ic;
    float scale_k;
    volatile uint8_t cal_active;
    volatile uint32_t cal_done;
    uint16_t cal_discard;
    uint16_t cal_samples;
} adc_sample_t;

void adc_sample_init(adc_sample_t *s, adc_sample_config_t *cfg);

bool adc_sample_calibrate_offset(adc_sample_t *s,
                                 uint16_t discard,
                                 uint16_t samples,
                                 uint32_t timeout_ms);

void adc_sample_on_injected(adc_sample_t *s, void *hadc);

void adc_sample_jeoc_foc(adc_sample_t *s, void *hadc);

void adc_sample_update(adc_sample_t *s);

void adc_sample_get_abc(const adc_sample_t *s, float *ia, float *ib, float *ic);

#ifdef __cplusplus
}
#endif

#endif
