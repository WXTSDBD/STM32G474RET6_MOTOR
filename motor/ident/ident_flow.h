/**
 * @file ident_flow.h
 * @date 2026-10-06
 * @brief 堵转辨识编排：切死区 profile，推进 ident_module。

 *
 * tick 只允许从电流环节拍调用。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
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
