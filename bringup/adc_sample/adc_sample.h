/**

 * @file adc_sample.h

 * @brief 三相 shunt 电流 ADC 采样：配置 schema 与对外 API。

 *

 * 本模块不含板级 HAL 句柄（无 hadc2/htim8）；binding 由 config/bsp_axes + bridge 填入。

 * 函数说明见 adc_sample.c。

 */



#ifndef ADC_SAMPLE_H

#define ADC_SAMPLE_H



#include <stdbool.h>

#include <stdint.h>



#include "adc.h"

#include "tim.h"



#ifdef __cplusplus

extern "C" {

#endif



/** 注入采样拓扑：单 ADC 多 rank，或每相独立 ADC（后者待实现） */

typedef enum {

    ADC_SAMPLE_TOPO_SCAN,

    ADC_SAMPLE_TOPO_PER_PHASE,

} adc_sample_topo_t;



/** 单相通道：JDR 序号、标定 offset、A/LSB 标度（由 board/config 填写） */

typedef struct {

    ADC_HandleTypeDef *hadc;

    uint8_t jdr_index;

    int32_t offset;

    float scale;

} adc_sample_ch_cfg_t;



/** 一路 ADC 采样组的静态配置（指针字段由 bridge_cubemx 填写） */

typedef struct {

    adc_sample_topo_t topo;

    uint8_t n_ch;

    adc_sample_ch_cfg_t ch[3];

    TIM_HandleTypeDef *trig_tim;

    uint32_t trig_ch;

    TIM_HandleTypeDef *pwm_tim;

} adc_sample_config_t;



/** 运行时实例：原始码、安培值、标定过程状态 */

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



void adc_sample_on_injected(adc_sample_t *s, ADC_HandleTypeDef *hadc);



/** 20 kHz 热路径：直读 JDR1/2/3 → 减偏 × 每路 scale → ia/ib/ic（无除法） */

void adc_sample_jeoc_foc(adc_sample_t *s, ADC_HandleTypeDef *hadc);



void adc_sample_update(adc_sample_t *s);



void adc_sample_get_abc(const adc_sample_t *s, float *ia, float *ib, float *ic);



#ifdef __cplusplus

}

#endif



#endif


