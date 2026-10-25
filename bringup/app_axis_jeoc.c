/**
 * @file app_axis_jeoc.c
 * @date 2026-10-07
 * @brief M1 ADC2 JEOC：零偏标定 / 相序 / 编码器锁轴 / FOC 主路径。
 *
 * 节拍限制见 app_axis_jeoc.h 文件头。
 */

#include "app_axis_jeoc.h"

#include <stddef.h>

#include "adc_foc_port.h"
#include "adc_sample.h"
#include "app_uart_dma_debug.h"
#include "bsp_axes.h"
#include "dbg_monitor.h"
#include "encoder_cal.h"
#include "motor_current.h"
#include "phase_detect.h"
#include "bsp_dwt.h"

/**
 * @brief 刷新 dbg 里 M1 三相 ADC 偏移与零偏后读数。
 * @param m1 轴。不可为 NULL。
 */
static void app_axis_jeoc_snapshot_m1_adc(const bsp_axis_t *m1)
{
    uint8_t i;

    for (i = 0U; i < 3U; i++) {
        DBG.adc_offset[i] = m1->adc_cfg.ch[i].offset;
        DBG.adc_zeroed[i] =
            (int16_t)((int32_t)m1->adc.raw[i] - m1->adc_cfg.ch[i].offset);
        DBG.adc_reg[i] = m1->adc.raw[i];
    }
    DBG.adc_ia = m1->adc.ia;
    DBG.adc_ib = m1->adc.ib;
    DBG.adc_ic = m1->adc.ic;
}

/**
 * @brief ADC2 注入完成：按标定标志分流，否则跑 FOC 一拍。
 * @param hadc 完成的 ADC 句柄（void*）。不可为 NULL。
 */
static uint32_t s_jeoc_diag_bits;
static uint32_t s_jeoc_last_entry_cyc;

/**
 * @brief JEOC 诊断位：bit2 来自 ADC（JQOVF），bit3=漏拍。
 */
uint32_t app_axis_jeoc_diag_bits(void)
{
    return s_jeoc_diag_bits | (adc_sample_diag_bits() & 0x04u);
}

void app_axis_jeoc_on_injected(void *hadc)
{
    bsp_axis_t *m1;

    if (hadc == NULL) {
        return;
    }

    {
        /* §7.4 漏拍判据用入口间隔（不是 isr_delta 时长）：正常 8000 cycles，>1.75x 记漏拍。 */
        uint32_t now = DWT->CYCCNT;

        if ((s_jeoc_last_entry_cyc != 0u) &&
            ((now - s_jeoc_last_entry_cyc) > 14000u)) {
            s_jeoc_diag_bits |= 0x08u;
        }
        s_jeoc_last_entry_cyc = now;
    }

    m1 = bsp_axis(BSP_AXIS_M1);
    if (m1 == NULL) {
        return;
    }

    if (m1->adc.cal_active) {
        adc_sample_on_injected(&m1->adc, hadc);
    } else if (g_phase_cal_active) {
        phase_detect_jeoc_tick(&m1->adc, hadc, m1->pwm);
    } else if (g_cal_hold) {
        phase_detect_hold_jeoc_tick(&m1->adc, hadc, m1->pwm);
    } else if (g_encoder_cal_active) {
        encoder_cal_jeoc_tick(m1);
    } else {
        adc_foc_port_on_jeoc(m1->adc_foc, &m1->adc, hadc);
        motor_current_tick(m1);
    }

    app_axis_jeoc_snapshot_m1_adc(m1);
    if (g_phase_cal_active || g_cal_hold) {
        telem_bringup_tick();
        telem_bringup_try_send();
    }
}
