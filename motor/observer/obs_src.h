/**
 * @file obs_src.h
 * @date 2026-10-06
 * @brief 低速槽：θ、ω、是否在跑。给 emf_pll 用，避免再 include Composite。
 *
 * 实现仍在 observer_composite.c。回执不在这张表里。
 *
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#ifndef MOTOR_OBSERVER_OBS_SRC_H
#define MOTOR_OBSERVER_OBS_SRC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 低速槽：θ, ω, running。回执不在这张表里。 */
typedef struct {
    /** 当前低速估计电角，单位 rad。 */
    float (*get_theta)(void);
    /** 当前低速电角速度。现行 HFI 槽给出 PLL 积分项，单位 rad/s 电。 */
    float (*get_omega)(void);
    /** 1=低速源认为自己在跑，允许发布逻辑继续。 */
    uint8_t (*running)(void);
} observer_src_slot_t;

const observer_src_slot_t *observer_lo_src(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_OBSERVER_OBS_SRC_H */
