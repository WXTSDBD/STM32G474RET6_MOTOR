/**
 * @file motor_context.h
 * @brief M1 控制上下文：开环 / PI 影子 / 电流环（后续）。
 */

#ifndef MOTOR_CONTEXT_H
#define MOTOR_CONTEXT_H

#include <stdint.h>

#include "foc_pi.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    M1_CTRL_OPEN_LOOP = 0,
    M1_CTRL_OBSERVE_ONLY,
    M1_CTRL_CURRENT_LOOP,
} m1_ctrl_mode_t;

/** 2026-06-19 电流环联调：上电即 CURRENT_LOOP；开环死区扫参改 OBSERVE_ONLY */
#ifndef M1_CTRL_MODE_DEFAULT
#define M1_CTRL_MODE_DEFAULT M1_CTRL_CURRENT_LOOP
#endif

typedef struct {
    uint8_t pole_pairs;
    float uq_open;
    m1_ctrl_mode_t mode;
    float id_ref;
    float iq_ref;
    foc_pi_t pi_id;
    foc_pi_t pi_iq;
    float ud_pi;
    float uq_pi;
} motor_context_t;

#ifdef __cplusplus
}
#endif

#endif
