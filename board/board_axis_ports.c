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
#include "motor_params_m1.h"

void Error_Handler(void);

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
/* CubeMX TIM1/TIM8 Init.Period 现网为 3999；改 .ioc 必须同步本宏。 */
_Static_assert(M1_PWM_ARR_COUNTS == 3999u,
               "M1_PWM_ARR_COUNTS must match CubeMX TIM Init.Period");
#endif

static adc_foc_port_t s_m1_adc_foc = {
    .ops = &adc_foc_port_ops_stm32g4_reg,
    .hw = NULL,
};
static adc_foc_port_t s_m2_adc_foc = {
    .ops = &adc_foc_port_ops_stm32g4_reg,
    .hw = NULL,
};

/* 板级 PWM 描述：换 TIM 通道接线只改 ch_map；换周期改 arr_counts。 */
static pwm_port_t s_m1_pwm = {
    .ops = &pwm_port_ops_stm32g4_reg,
    .hw = NULL,
    .ch_map = {0u, 1u, 2u},
    .arr_counts = (uint16_t)M1_PWM_ARR_COUNTS,
};
static pwm_port_t s_m2_pwm = {
    .ops = &pwm_port_ops_stm32g4_reg,
    .hw = NULL,
    .ch_map = {0u, 1u, 2u},
    .arr_counts = (uint16_t)M1_PWM_ARR_COUNTS,
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
        /* A6：宏 ↔ CubeMX Period ↔ 板级 arr_counts 三处一致。 */
        if ((htim8.Init.Period != (uint32_t)M1_PWM_ARR_COUNTS) ||
            (s_m1_pwm.arr_counts == 0u) ||
            (s_m1_pwm.arr_counts != (uint16_t)M1_PWM_ARR_COUNTS)) {
            Error_Handler();
        }
        break;
    case BSP_AXIS_M2:
        s_m2_pwm.hw = &htim1;
        if ((htim1.Init.Period != (uint32_t)M1_PWM_ARR_COUNTS) ||
            (s_m2_pwm.arr_counts == 0u) ||
            (s_m2_pwm.arr_counts != (uint16_t)M1_PWM_ARR_COUNTS)) {
            Error_Handler();
        }
        break;
    default:
        break;
    }
}
