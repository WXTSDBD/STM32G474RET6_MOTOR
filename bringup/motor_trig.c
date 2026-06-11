/**
 * @file motor_trig.c
 * @brief 编译期分发：LUT 或 CORDIC，见 motor_trig_cfg.h。
 */

#include "motor_trig.h"
#include "motor_trig_cfg.h"
#include "motor_trig_backend.h"

void motor_trig_init(void)
{
#if MOTOR_TRIG_BACKEND == MOTOR_TRIG_BACKEND_CORDIC
    motor_trig_cordic_init();
#else
    motor_trig_lut_init();
#endif
}

void motor_trig_sincos(float rad, float *cos_out, float *sin_out)
{
#if MOTOR_TRIG_BACKEND == MOTOR_TRIG_BACKEND_CORDIC
    motor_trig_cordic_sincos(rad, cos_out, sin_out);
#else
    motor_trig_lut_sincos(rad, cos_out, sin_out);
#endif
}

float motor_trig_sin(float rad)
{
    float c;
    float s;

    motor_trig_sincos(rad, &c, &s);
    return s;
}
