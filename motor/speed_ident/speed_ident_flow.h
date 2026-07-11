/**
 * @file speed_ident_flow.h
 * @brief 速度环 ident 编排：deadband OFF + VOFA open_seq。
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
