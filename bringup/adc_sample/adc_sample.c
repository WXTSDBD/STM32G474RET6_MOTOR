/**
 * @file adc_sample.c
 * @brief 三相电流 ADC 采样：JDR 读取、零偏标定（方案 B + 递推增量均值）、raw 转安培。
 *
 * 标定方案 B：仅启动 trig_tim 的 Base + CH4（与运行时 T8_CC4 触发一致），
 * 不启动 pwm_tim 的 CH1～3，不依赖 TIM UP 中断。
 * 标定算法：discard 后每帧 offset += (raw - offset) / k，与 sum/N 等价。
 * 运行时热路径 adc_sample_jeoc_foc：JDR1/2/3 直读 + int 减偏 + 每路 scale → ia/ib/ic。
 */

#include "adc_sample.h"

/**
 * @brief 按 JDR 序号读取注入转换结果（1～4 对应 JDR1～JDR4）。
 */
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

/**
 * @brief M1 SCAN 热路径：固定 JDR1/2/3 直读（无 switch/循环）。
 */
static inline void adc_sample_read_jdr123(ADC_TypeDef *adc, int16_t raw[3])
{
    raw[0] = (int16_t)adc->JDR1;
    raw[1] = (int16_t)adc->JDR2;
    raw[2] = (int16_t)adc->JDR3;
}

/**
 * @brief TOPO_SCAN：一次 JEOC 从同一 hadc 读齐 n_ch 路 JDR 到 raw[]（标定/通用路径）。
 */
static void adc_sample_read_scan(adc_sample_t *s)
{
    const adc_sample_config_t *cfg = s->cfg;
    ADC_HandleTypeDef *hadc = cfg->ch[0].hadc;
    uint8_t i;

    if (hadc == NULL) {
        return;
    }

    for (i = 0; i < cfg->n_ch; i++) {
        s->raw[i] = adc_sample_read_jdr(hadc, cfg->ch[i].jdr_index);
    }
}

/**
 * @brief int 减偏后乘 cfg->ch[i].scale，写入 ia/ib/ic。
 */
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

/**
 * @brief 标定态下每收到一帧：前 discard 帧丢弃，之后递推增量均值写入 offset。
 */
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

/**
 * @brief 绑定配置并清零运行时状态（不含 HAL 校准，不含零偏标定）。
 */
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

/**
 * @brief 零电流偏置标定：阻塞等待 discard+samples 帧，递推结果在 cfg->ch[i].offset。
 * @param discard  丢弃前 N 帧（定时器刚启动不稳定）
 * @param samples  参与递推的有效帧数
 * @param timeout_ms 等待 JEOC 帧数的超时（毫秒）
 * @return true 标定成功；false 参数/硬件/超时
 *
 * @note 调用前需已 HAL_ADCEx_InjectedStart_IT；调用期间勿开 pwm_tim 的 CH1～3 与 Base_IT。
 */
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

    tim = cfg->trig_tim;
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
        __HAL_TIM_SET_COMPARE(cfg->pwm_tim, TIM_CHANNEL_1, 0U);
        __HAL_TIM_SET_COMPARE(cfg->pwm_tim, TIM_CHANNEL_2, 0U);
        __HAL_TIM_SET_COMPARE(cfg->pwm_tim, TIM_CHANNEL_3, 0U);
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

/**
 * @brief ADC 注入转换完成回调入口（标定期 main 转发；含 cal_active 递推）。
 */
void adc_sample_on_injected(adc_sample_t *s, ADC_HandleTypeDef *hadc)
{
    const adc_sample_config_t *cfg;

    if (s == NULL || hadc == NULL) {
        return;
    }

    cfg = s->cfg;
    if (cfg == NULL) {
        return;
    }

    if (cfg->topo == ADC_SAMPLE_TOPO_SCAN) {
        if (hadc != cfg->ch[0].hadc) {
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

/**
 * @brief 20 kHz FOC 热路径：读 JDR → 减偏 × scale_k（标定分支不在此处理）。
 */
void adc_sample_jeoc_foc(adc_sample_t *s, ADC_HandleTypeDef *hadc)
{
    adc_sample_read_jdr123(hadc->Instance, s->raw);
    adc_sample_apply_scale(s);
}

/**
 * @brief raw 减 offset 乘 scale_k，结果写入 ia/ib/ic（非 ISR 调试入口）。
 */
void adc_sample_update(adc_sample_t *s)
{
    if (s == NULL || s->cfg == NULL) {
        return;
    }

    adc_sample_apply_scale(s);
}

/**
 * @brief 读取最近一次 update/jeoc_foc 得到的 Ia/Ib/Ic（安培）；指针可传 NULL 跳过。
 */
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