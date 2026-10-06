/**
 * @file motor_math.h
 * @date 2026-10-06
 * @brief 热路径标量：折角、绝对值、牛顿开方、限幅。

 *
 * 函数都是 static inline，注释写在本头。折角区间是 [-π, π]。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#ifndef MOTOR_MATH_H
#define MOTOR_MATH_H

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/** Wrap to [-pi, pi] (same while-loop as former hfi_wrap_pi / hfi_*_wrap). */
static inline float motor_wrap_pi(float x)
{
    const float pi = (float)M_PI;
    const float twopi = 2.0f * pi;

    while (x > pi) {
        x -= twopi;
    }
    while (x < -pi) {
        x += twopi;
    }
    return x;
}

static inline float motor_absf(float x)
{
    return (x < 0.0f) ? -x : x;
}

/** Four Newton iterations; x<=0 returns 0. Same as former hfi_lead_sqrt / hfi_rip_sqrt. */
static inline float motor_sqrt4(float x)
{
    float y;
    int i;

    if (x <= 0.0f) {
        return 0.0f;
    }
    y = (x > 1.0f) ? x : 1.0f;
    for (i = 0; i < 4; i++) {
        y = 0.5f * (y + (x / y));
    }
    return y;
}

static inline float motor_clampf(float x, float lim)
{
    if (x > lim) {
        return lim;
    }
    if (x < -lim) {
        return -lim;
    }
    return x;
}

/**
 * VESC HFI V4 style: e = sign * (y_raw / ts) / den, then clamp.
 * den is typically vh * (1/Lq - 1/Ld); caller owns Vh / Ld / Lq.
 */
static inline float motor_vesc_ang_err(float y_raw, float ts, float den,
                                       float sign, float max_err)
{
    float e = 0.0f;

    if ((den > 1.0e-3f) || (den < -1.0e-3f)) {
        e = sign * (y_raw / ts) / den;
    }
    return motor_clampf(e, max_err);
}

#endif /* MOTOR_MATH_H */
