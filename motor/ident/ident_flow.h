/**
 * @file ident_flow.h
 * @brief ident 编排：deadband profile 切换 + VOFA open_seq（Phase 4）。
 *
 * ident_module 只产生 Iq_ref；本层在状态/轮次变化时调用 deadband_service。
 */

#ifndef IDENT_FLOW_H
#define IDENT_FLOW_H

#include "motor_context.h"

#ifdef __cplusplus
extern "C" {
#endif

void ident_flow_init(motor_context_t *ctx);
void ident_flow_tick(motor_context_t *ctx);

/** 当前 STEP/BODE 段是否为 FIXED 补偿 */
uint8_t ident_flow_deadband_fixed(void);

#ifdef __cplusplus
}
#endif

#endif /* IDENT_FLOW_H */
