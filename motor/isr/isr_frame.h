/**
 * @file isr_frame.h
 * @date 2026-10-29
 * @brief 20 kHz 热路径帧：各 isr_*.c 共享本拍中间量，不靠宏选路。
 *
 * 入口 isr_entry_tick 按固定顺序调用 sample → startup → current → modulate → telem。
 * 静态状态仍住在 motor_current.c，本帧只搬运本拍数据。
 */
#ifndef MOTOR_ISR_ISR_FRAME_H
#define MOTOR_ISR_ISR_FRAME_H

#include "bsp_axes.h"
#include "motor_context.h"
#include "motor_startup.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 单拍 FOC 中间量（栈上，不进全局）。 */
typedef struct {
    bsp_axis_t *axis;
    motor_context_t *ctx;
    uint16_t enc_raw;
    float ia;
    float ib;
    float ic;
    float i_alpha;
    float i_beta;
    float theta_el_raw;
    float theta_enc_park;
    float theta_park;
    float sin_el;
    float cos_el;
    float id;
    float iq;
    float ud_out;
    float uq_out;
    float ua_hfi;
    float ub_hfi;
    motor_startup_step_t startup;
    uint32_t isr_t0;
    uint32_t foc_t0;
    uint32_t t_pre_obs;
    uint32_t t_post_obs;
    uint32_t t_post_svpwm;
} isr_frame_t;

void isr_entry_tick(bsp_axis_t *axis);
void isr_sample_run(isr_frame_t *f);
void isr_startup_run(isr_frame_t *f);
void isr_current_run(isr_frame_t *f);
void isr_obs_run(isr_frame_t *f);
void isr_modulate_run(isr_frame_t *f);
void isr_telem_run(isr_frame_t *f);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_ISR_ISR_FRAME_H */
