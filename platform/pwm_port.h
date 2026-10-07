/**
 * @file pwm_port.h
 * @date 2026-10-06
 * @brief 三相 PWM 占空比输出口。不含 HAL。

 *
 * set_duty3 只允许从电流环节拍调用。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
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
    /**
     * 逻辑相 A/B/C → TIM 通道索引（0=CCR1, 1=CCR2, 2=CCR3）。
     * 恒等 {0,1,2} 即现状接线；换板改这里，不改 motor/。
     */
    uint8_t ch_map[3];
    /** PWM ARR 计数（与时间域 M1_PWM_PERIOD_S 不同量纲）。 */
    uint16_t arr_counts;
};

extern const pwm_port_ops_t pwm_port_ops_stm32g4_reg;

void pwm_port_set_duty3(pwm_port_t *port, uint32_t ccr1, uint32_t ccr2, uint32_t ccr3);
void pwm_port_enable(pwm_port_t *port);
void pwm_port_disable(pwm_port_t *port);

#ifdef __cplusplus
}
#endif

#endif
