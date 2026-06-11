#ifndef MOTOR_TRIG_BACKEND_H
#define MOTOR_TRIG_BACKEND_H

void motor_trig_cordic_init(void);
void motor_trig_cordic_sincos(float rad, float *cos_out, float *sin_out);

void motor_trig_lut_init(void);
void motor_trig_lut_sincos(float rad, float *cos_out, float *sin_out);

#endif
