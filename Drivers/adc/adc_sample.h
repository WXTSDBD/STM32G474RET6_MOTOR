/**
 * @file adc_sample.h
 * @date 2026-10-06
 * @brief 三相采样电阻电流：配置和对外 API。不含 HAL 类型。
 *
 * 句柄由 bsp_axes 和 bridge 填进 void*。实现在 platform/adc_sample_stm32g4.c。
 * on_injected / jeoc_foc 只允许从 ADC 注入转换完成中断调用。
 *
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#ifndef ADC_SAMPLE_H
#define ADC_SAMPLE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 采样拓扑。SCAN：一个 ADC 扫三相。PER_PHASE：每相一个 ADC。
 */
typedef enum {
    ADC_SAMPLE_TOPO_SCAN,
    ADC_SAMPLE_TOPO_PER_PHASE,
} adc_sample_topo_t;

typedef struct {
    /** ADC 句柄，实际是 HAL 指针。 */
    void *hadc;
    /** 注入数据寄存器下标 1..3。 */
    uint8_t jdr_index;
    /** 零偏，LSB。 */
    int32_t offset;
    /** 安培/LSB。 */
    float scale;
} adc_sample_ch_cfg_t;

typedef struct {
    /** 扫描还是每相独立。 */
    adc_sample_topo_t topo;
    /** 通道数，M1 为 3。 */
    uint8_t n_ch;
    /** 三相通道配置。 */
    adc_sample_ch_cfg_t ch[3];
    /** 触发定时器句柄。 */
    void *trig_tim;
    /** 触发通道。 */
    uint32_t trig_ch;
    /** PWM 定时器句柄。 */
    void *pwm_tim;
} adc_sample_config_t;

typedef struct {
    /** 绑定的配置。 */
    adc_sample_config_t *cfg;
    /** 本拍 raw，已减偏置前。 */
    int16_t raw[3];
    /** a 相电流，单位 A。 */
    float ia;
    /** b 相电流，单位 A。 */
    float ib;
    /** c 相电流，单位 A。 */
    float ic;
    /** 公用安培/LSB。 */
    float scale_k;
    /** 1=正在采零偏。 */
    volatile uint8_t cal_active;
    /** 已采够的点数。 */
    volatile uint32_t cal_done;
    /** 开头丢掉的点数。 */
    uint16_t cal_discard;
    /** 参与平均的点数。 */
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
