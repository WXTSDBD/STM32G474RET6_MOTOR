/**
 * @file bridge_cubemx.c
 * @brief 将 CubeMX 导出的 HAL 句柄填入 bsp_axis->adc_cfg 与 pwm_tim。
 *
 * CubeMX Regenerate 后若句柄名变化，通常只需改本文件。
 *
 * 当前映射：
 *   M1 — hadc2 JDR1/2/3，TIM8 CH4 触发注入，TIM8 作 PWM
 *   M2 — 占位：TIM1；ADC hadc 待定义（enabled=false）
 */

#include "bridge_cubemx.h"

#include "adc.h"
#include "tim.h"

/**
 * @brief 按轴号写入 HAL 指针；不改变 JDR/scale 等语义项（由 bsp_axes 填写）。
 */
void bridge_cubemx_apply_axis(bsp_axis_id_t id, bsp_axis_t *axis)
{
    adc_sample_config_t *cfg;

    if (axis == NULL) {
        return;
    }

    cfg = &axis->adc_cfg;

    switch (id) {
    case BSP_AXIS_M1:
        cfg->ch[0].hadc = &hadc2;
        cfg->ch[1].hadc = &hadc2;
        cfg->ch[2].hadc = &hadc2;
        cfg->trig_tim = &htim8;
        cfg->trig_ch = TIM_CHANNEL_4;
        cfg->pwm_tim = &htim8;
        axis->pwm_tim = &htim8;
        break;

    case BSP_AXIS_M2:
        cfg->ch[0].hadc = NULL;
        cfg->ch[1].hadc = NULL;
        cfg->ch[2].hadc = NULL;
        cfg->trig_tim = &htim1;
        cfg->trig_ch = TIM_CHANNEL_4;
        cfg->pwm_tim = &htim1;
        axis->pwm_tim = &htim1;
        break;

    default:
        break;
    }
}
