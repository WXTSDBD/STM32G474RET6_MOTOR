/**
 * @file speed_ident_flow.h
 * @date 2026-10-06
 * @brief 速度环辨识编排。死区关掉，由外环推进 ω_ref。

 *
 * tick 随外环调用，仍在 ADC 注入完成中断的分频拍里。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#ifndef SPEED_IDENT_FLOW_H
#define SPEED_IDENT_FLOW_H

#include "motor_context.h"

#ifdef __cplusplus
extern "C" {
#endif

void speed_ident_flow_init(motor_context_t *ctx);
void speed_ident_flow_tick(motor_context_t *ctx);
/** 1=init 已完成，外环应推进 ω_ref 序列（勿依赖 deadband_flow 配方步） */
uint8_t speed_ident_flow_is_armed(void);

#ifdef __cplusplus
}
#endif

#endif /* SPEED_IDENT_FLOW_H */
