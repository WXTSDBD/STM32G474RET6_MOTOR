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
