/**
 * @file adc_sample_stm32g4.c
 * @brief STM32G4 三相电流 ADC 采样：JDR 寄存器读、零偏标定、FOC 热路径。
 *
 * 热路径 adc_sample_jeoc_foc：直读 ADC->JDR1/2/3，无 LL API。
 */

#include "adc_sample.h"

#include "hal_bridge.h"
#include "tim.h"

static int16_t adc_sample_read_jdr(ADC_HandleTypeDef *hadc, uint8_t jdr)
{
    switch (jdr) {
    case 1:
        return (int16_t)hadc->Instance->JDR1;
    case 2:
        return (int16_t)hadc->Instance->JDR2;
    case 3:
        return (int16_t)hadc->Instance->JDR3;
    case 4:
        return (int16_t)hadc->Instance->JDR4;
    default:
        return 0;
    }
}

static inline void adc_sample_read_jdr123(ADC_TypeDef *adc, int16_t raw[3])
{
    raw[0] = (int16_t)adc->JDR1;
    raw[1] = (int16_t)adc->JDR2;
    raw[2] = (int16_t)adc->JDR3;
}

static void adc_sample_read_scan(adc_sample_t *s)
{
    const adc_sample_config_t *cfg = s->cfg;
    ADC_HandleTypeDef *hadc = (ADC_HandleTypeDef *)cfg->ch[0].hadc;
    uint8_t i;

    if (hadc == NULL) {
        return;
    }

    for (i = 0; i < cfg->n_ch; i++) {
        s->raw[i] = adc_sample_read_jdr(hadc, cfg->ch[i].jdr_index);
    }
}

static inline void adc_sample_apply_scale(adc_sample_t *s)
{
    const adc_sample_config_t *cfg = s->cfg;
    int32_t d0;
    int32_t d1;
    int32_t d2;

    d0 = (int32_t)s->raw[0] - cfg->ch[0].offset;
    d1 = (int32_t)s->raw[1] - cfg->ch[1].offset;
    d2 = (int32_t)s->raw[2] - cfg->ch[2].offset;

    s->ia = (float)d0 * cfg->ch[0].scale;
    s->ib = (float)d1 * cfg->ch[1].scale;
    s->ic = (float)d2 * cfg->ch[2].scale;
}

static void adc_sample_cal_on_frame(adc_sample_t *s)
{
    adc_sample_config_t *cfg = s->cfg;
    uint32_t n = s->cal_done;
    uint32_t k;
    uint8_t i;
    int32_t delta;

    s->cal_done = n + 1U;

    if (n < (uint32_t)s->cal_discard) {
        return;
    }

    k = n + 1U - (uint32_t)s->cal_discard;
    for (i = 0; i < cfg->n_ch; i++) {
        delta = (int32_t)s->raw[i] - cfg->ch[i].offset;
        cfg->ch[i].offset += delta / (int32_t)k;
    }
}

void adc_sample_init(adc_sample_t *s, adc_sample_config_t *cfg)
{
    uint8_t i;

    s->cfg = cfg;
    s->cal_active = 0U;
    s->cal_done = 0U;
    s->cal_discard = 0U;
    s->cal_samples = 0U;
    s->scale_k = cfg->ch[0].scale;
    s->ia = 0.0f;
    s->ib = 0.0f;
    s->ic = 0.0f;

    for (i = 0; i < 3U; i++) {
        s->raw[i] = 0;
    }
}

bool adc_sample_calibrate_offset(adc_sample_t *s,
                                 uint16_t discard,
                                 uint16_t samples,
                                 uint32_t timeout_ms)
{
    adc_sample_config_t *cfg = s->cfg;
    TIM_HandleTypeDef *tim;
    uint32_t target;
    uint32_t t0;
    uint8_t i;

    if (s == NULL || cfg == NULL || samples == 0U) {
        return false;
    }

    tim = (TIM_HandleTypeDef *)cfg->trig_tim;
    if (tim == NULL || cfg->ch[0].hadc == NULL) {
        return false;
    }

    target = (uint32_t)discard + (uint32_t)samples;
    s->cal_active = 1U;
    s->cal_done = 0U;
    s->cal_discard = discard;
    s->cal_samples = samples;

    for (i = 0; i < cfg->n_ch; i++) {
        cfg->ch[i].offset = 0;
    }

    if (cfg->pwm_tim != NULL) {
        TIM_HandleTypeDef *pwm = (TIM_HandleTypeDef *)cfg->pwm_tim;
        __HAL_TIM_SET_COMPARE(pwm, TIM_CHANNEL_1, 0U);
        __HAL_TIM_SET_COMPARE(pwm, TIM_CHANNEL_2, 0U);
        __HAL_TIM_SET_COMPARE(pwm, TIM_CHANNEL_3, 0U);
    }

    if (HAL_TIM_PWM_Start(tim, cfg->trig_ch) != HAL_OK) {
        s->cal_active = 0U;
        return false;
    }
    if (HAL_TIM_Base_Start(tim) != HAL_OK) {
        HAL_TIM_PWM_Stop(tim, cfg->trig_ch);
        s->cal_active = 0U;
        return false;
    }

    t0 = HAL_GetTick();
    while (s->cal_done < target) {
        if ((HAL_GetTick() - t0) > timeout_ms) {
            HAL_TIM_Base_Stop(tim);
            HAL_TIM_PWM_Stop(tim, cfg->trig_ch);
            s->cal_active = 0U;
            return false;
        }
    }

    HAL_TIM_Base_Stop(tim);
    HAL_TIM_PWM_Stop(tim, cfg->trig_ch);

    s->cal_active = 0U;
    return true;
}

void adc_sample_on_injected(adc_sample_t *s, void *hadc)
{
    const adc_sample_config_t *cfg;
    ADC_HandleTypeDef *hadc_hal = (ADC_HandleTypeDef *)hadc;

    if (s == NULL || hadc_hal == NULL) {
        return;
    }

    cfg = s->cfg;
    if (cfg == NULL) {
        return;
    }

    if (cfg->topo == ADC_SAMPLE_TOPO_SCAN) {
        if (hadc_hal != (ADC_HandleTypeDef *)cfg->ch[0].hadc) {
            return;
        }
        adc_sample_read_scan(s);
    } else {
        return;
    }

    if (s->cal_active) {
        adc_sample_cal_on_frame(s);
    }
}

void adc_sample_jeoc_foc(adc_sample_t *s, void *hadc)
{
    ADC_HandleTypeDef *hadc_hal = (ADC_HandleTypeDef *)hadc;

    adc_sample_read_jdr123(hadc_hal->Instance, s->raw);
    adc_sample_apply_scale(s);
}

void adc_sample_update(adc_sample_t *s)
{
    if (s == NULL || s->cfg == NULL) {
        return;
    }

    adc_sample_apply_scale(s);
}

void adc_sample_get_abc(const adc_sample_t *s, float *ia, float *ib, float *ic)
{
    if (s == NULL) {
        return;
    }
    if (ia != NULL) {
        *ia = s->ia;
    }
    if (ib != NULL) {
        *ib = s->ib;
    }
    if (ic != NULL) {
        *ic = s->ic;
    }
}
