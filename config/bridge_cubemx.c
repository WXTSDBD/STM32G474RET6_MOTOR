/**
 * @file bridge_cubemx.c
 * @date 2026-10-06
 * @brief 把 CubeMX 句柄填进轴配置和 board Port。

 *
 * 当前映射：
 *   M1 — hadc2 的 JDR1/2/3，TIM8 CH4 触发注入，TIM8 作 PWM
 *   M2 — 占位：TIM1；ADC 尚未绑定
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#include "bridge_cubemx.h"

#include "board_axis_ports.h"
#include "hal_bridge.h"
#include "tim.h"

void bridge_cubemx_apply_axis(bsp_axis_id_t id, bsp_axis_t *axis)
{
    adc_sample_config_t *cfg;

    if (axis == NULL) {
        return;
    }

    cfg = &axis->adc_cfg;
    axis->adc_foc = board_adc_foc_port(id);
    axis->pwm = board_pwm_port(id);
    board_axis_ports_bind_hal(id);

    switch (id) {
    case BSP_AXIS_M1:
        cfg->ch[0].hadc = &hadc2;
        cfg->ch[1].hadc = &hadc2;
        cfg->ch[2].hadc = &hadc2;
        cfg->trig_tim = &htim8;
        cfg->trig_ch = TIM_CHANNEL_4;
        cfg->pwm_tim = &htim8;
        break;

    case BSP_AXIS_M2:
        cfg->ch[0].hadc = NULL;
        cfg->ch[1].hadc = NULL;
        cfg->ch[2].hadc = NULL;
        cfg->trig_tim = &htim1;
        cfg->trig_ch = TIM_CHANNEL_4;
        cfg->pwm_tim = &htim1;
        break;

    default:
        break;
    }
}
