/**
 * @file deadband_lut_baked_m1.h
 * @brief 硬编码 phase LUT（VOFA vofa+202606281745，proposed merge + normalize，scale=1）。
 */

#ifndef DEADBAND_LUT_BAKED_M1_H
#define DEADBAND_LUT_BAKED_M1_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define M1_DEADBAND_LUT_BAKED_LEN  32u

extern const float m1_deadband_lut_baked_amps[M1_DEADBAND_LUT_BAKED_LEN];
extern const float m1_deadband_lut_baked_vals[M1_DEADBAND_LUT_BAKED_LEN];

/** 上电/阶跃：注册 LUT、MODE_LUT、runtime_scale=1，不写 Flash */
bool deadband_lut_baked_m1_apply(void);

#ifdef __cplusplus
}
#endif

#endif /* DEADBAND_LUT_BAKED_M1_H */
