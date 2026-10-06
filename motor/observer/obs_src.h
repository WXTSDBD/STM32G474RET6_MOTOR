/**
 * @file obs_src.h
 * @brief 低速槽声明。给 emf_pll.c 用，避免 include observer_composite.h。
 *
 * 只放 observer_src_slot_t 与 observer_lo_src()。实现仍在 Composite。
 */
#ifndef MOTOR_OBSERVER_OBS_SRC_H
#define MOTOR_OBSERVER_OBS_SRC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 低速槽：θ, ω, running。回执不在这张表里。 */
typedef struct {
    float (*get_theta)(void);
    float (*get_omega)(void);
    uint8_t (*running)(void);
} observer_src_slot_t;

const observer_src_slot_t *observer_lo_src(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_OBSERVER_OBS_SRC_H */
