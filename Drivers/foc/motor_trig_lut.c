/**
 * @file motor_trig_lut.c
 * @date 2026-10-06
 * @brief 256 点正弦表插值。

 *
 * 表在 bringup/math_tables。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#include "motor_trig_cfg.h"

#if MOTOR_TRIG_BACKEND == MOTOR_TRIG_BACKEND_LUT

#include "motor_trig_backend.h"
#include "math_tables.h"

void motor_trig_lut_init(void)
{
}

void motor_trig_lut_sincos(float rad, float *cos_out, float *sin_out)
{
    math_tables_sincos_f32(rad, cos_out, sin_out);
}

#endif
