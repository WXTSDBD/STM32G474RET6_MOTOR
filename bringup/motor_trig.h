#ifndef MOTOR_TRIG_H
#define MOTOR_TRIG_H

#ifdef __cplusplus
extern "C" {
#endif

void motor_trig_init(void);
void motor_trig_sincos(float rad, float *cos_out, float *sin_out);
float motor_trig_sin(float rad);

#ifdef __cplusplus
}
#endif

#endif
