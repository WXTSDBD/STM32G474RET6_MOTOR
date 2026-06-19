/**
 * @file bsp_axes.c
 * @brief M1/M2 轴表：语义 binding（JDR、scale、topo）+ 调用 bridge 填 HAL 指针。
 *
 * init 顺序：bind_defaults → bridge_cubemx_apply_axis → adc_sample_init（仅 enabled 轴）
 *           → board_encoder_m1_init。
 * M2 当前 enabled=false，仅占位；启用时在 bind_defaults 与 bridge 中补全即可。
 */

#include "bsp_axes.h"

#include "bridge_cubemx.h"

#include "board_encoder.h"
#include "motor_params_m1.h"

static bsp_axis_t s_axes[BSP_AXIS_COUNT];

/**
 * @brief 填写与 CubeMX 无关的语义配置（JDR rank、scale、topo、enc 指针）。
 *        hadc/trig_tim/pwm_tim 指针由 bridge 填写。
 */
static void bsp_axis_bind_defaults(bsp_axis_id_t id, bsp_axis_t *axis)
{
    adc_sample_config_t *cfg = &axis->adc_cfg;

    if (id == BSP_AXIS_M1) {
        axis->enabled = true;
        axis->enc = &enc_m1;
        axis->motor_ctx = NULL;

        cfg->topo = ADC_SAMPLE_TOPO_SCAN;
        cfg->n_ch = 3U;
        cfg->ch[0].jdr_index = 1U;
        cfg->ch[0].offset = 0;
        cfg->ch[0].scale = M1_ADC_SCALE_A_LSB * M1_ADC_GAIN_CH0;
        cfg->ch[1].jdr_index = 2U;
        cfg->ch[1].offset = 0;
        cfg->ch[1].scale = M1_ADC_SCALE_A_LSB * M1_ADC_GAIN_CH1;
        cfg->ch[2].jdr_index = 3U;
        cfg->ch[2].offset = 0;
        cfg->ch[2].scale = M1_ADC_SCALE_A_LSB * M1_ADC_GAIN_CH2;
    } else if (id == BSP_AXIS_M2) {
        axis->enabled = false;
        axis->enc = &enc_m2;
        axis->motor_ctx = NULL;

        cfg->topo = ADC_SAMPLE_TOPO_SCAN;
        cfg->n_ch = 3U;
        cfg->ch[0].jdr_index = 1U;
        cfg->ch[0].offset = 0;
        cfg->ch[0].scale = 1.0f;
        cfg->ch[1].jdr_index = 2U;
        cfg->ch[1].offset = 0;
        cfg->ch[1].scale = 1.0f;
        cfg->ch[2].jdr_index = 3U;
        cfg->ch[2].offset = 0;
        cfg->ch[2].scale = 1.0f;
    }
}

/**
 * @brief CubeMX 初始化完成后调用：装配各轴并初始化 encoder。
 *        应在 HAL_ADCEx_InjectedStart_IT 之后、零偏标定与 motor PWM 之前调用。
 */
void bsp_init(void)
{
    bsp_axis_id_t id;

    for (id = 0; id < BSP_AXIS_COUNT; id++) {
        bsp_axis_bind_defaults(id, &s_axes[id]);
        bridge_cubemx_apply_axis(id, &s_axes[id]);

        if (s_axes[id].enabled) {
            adc_sample_init(&s_axes[id].adc, &s_axes[id].adc_cfg);
        }
    }

    board_encoder_m1_init();
}

/**
 * @brief 按轴号取运行时轴对象；非法 id 返回 NULL。
 */
bsp_axis_t *bsp_axis(bsp_axis_id_t id)
{
    if (id >= BSP_AXIS_COUNT) {
        return NULL;
    }
    return &s_axes[id];
}

/**
 * @brief 对指定轴做零电流 ADC 偏置标定（转发 adc_sample_calibrate_offset）。
 * @return 轴未 enabled 或标定失败时 false
 */
bool bsp_axis_adc_calibrate_zero(bsp_axis_id_t id,
                                 uint16_t discard,
                                 uint16_t samples,
                                 uint32_t timeout_ms)
{
    bsp_axis_t *axis = bsp_axis(id);

    if (axis == NULL || !axis->enabled) {
        return false;
    }

    return adc_sample_calibrate_offset(&axis->adc, discard, samples, timeout_ms);
}
