/**
 * @file board_axis_ports.c
 * @date 2026-10-06
 * @brief 把 CubeMX 句柄填进轴级 Port。

 *
 * 句柄名变了通常改 bridge 和本文件。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#include "board_axis_ports.h"

#include "hal_bridge.h"

static adc_foc_port_t s_m1_adc_foc = {
    .ops = &adc_foc_port_ops_stm32g4_reg,
    .hw = NULL,
};
static adc_foc_port_t s_m2_adc_foc = {
    .ops = &adc_foc_port_ops_stm32g4_reg,
    .hw = NULL,
};

static pwm_port_t s_m1_pwm = {
    .ops = &pwm_port_ops_stm32g4_reg,
    .hw = NULL,
};
static pwm_port_t s_m2_pwm = {
    .ops = &pwm_port_ops_stm32g4_reg,
    .hw = NULL,
};

adc_foc_port_t *board_adc_foc_port(bsp_axis_id_t id)
{
    switch (id) {
    case BSP_AXIS_M1:
        return &s_m1_adc_foc;
    case BSP_AXIS_M2:
        return &s_m2_adc_foc;
    default:
        return NULL;
    }
}

pwm_port_t *board_pwm_port(bsp_axis_id_t id)
{
    switch (id) {
    case BSP_AXIS_M1:
        return &s_m1_pwm;
    case BSP_AXIS_M2:
        return &s_m2_pwm;
    default:
        return NULL;
    }
}

void board_axis_ports_bind_hal(bsp_axis_id_t id)
{
    switch (id) {
    case BSP_AXIS_M1:
        s_m1_pwm.hw = &htim8;
        break;
    case BSP_AXIS_M2:
        s_m2_pwm.hw = &htim1;
        break;
    default:
        break;
    }
}
