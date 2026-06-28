/**
 * @file pwm_port.h
 * @brief 三相 PWM duty 输出 Port 契约（无 HAL）。P1 起接 stm32g4 寄存器 backend。
 */

#ifndef PWM_PORT_H
#define PWM_PORT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct pwm_port pwm_port_t;

typedef struct pwm_port_ops {
    void (*set_duty3)(pwm_port_t *port, uint32_t ccr1, uint32_t ccr2, uint32_t ccr3);
    void (*enable)(pwm_port_t *port);
    void (*disable)(pwm_port_t *port);
} pwm_port_ops_t;

struct pwm_port {
    const pwm_port_ops_t *ops;
    void *hw;
    void *user_ctx;
};

extern const pwm_port_ops_t pwm_port_ops_stm32g4_reg;

void pwm_port_set_duty3(pwm_port_t *port, uint32_t ccr1, uint32_t ccr2, uint32_t ccr3);
void pwm_port_enable(pwm_port_t *port);
void pwm_port_disable(pwm_port_t *port);

#ifdef __cplusplus
}
#endif

#endif
