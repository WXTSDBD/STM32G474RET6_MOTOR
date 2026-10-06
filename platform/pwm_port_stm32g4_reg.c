/**
 * @file pwm_port_stm32g4_reg.c
 * @date 2026-10-06
 * @brief 把三相 CCR 写进 STM32G4 定时器。

 *
 * 节拍限制见 pwm_port.h 文件头。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#include "pwm_port.h"

#include "hal_bridge.h"

static void pwm_port_stm32g4_set_duty3(pwm_port_t *port,
                                       uint32_t ccr1,
                                       uint32_t ccr2,
                                       uint32_t ccr3)
{
    TIM_HandleTypeDef *htim;

    if (port == NULL) {
        return;
    }

    htim = (TIM_HandleTypeDef *)port->hw;
    if (htim == NULL || htim->Instance == NULL) {
        return;
    }

    htim->Instance->CCR1 = (uint16_t)ccr1;
    htim->Instance->CCR2 = (uint16_t)ccr2;
    htim->Instance->CCR3 = (uint16_t)ccr3;
}

static void pwm_port_stm32g4_enable(pwm_port_t *port)
{
    (void)port;
}

static void pwm_port_stm32g4_disable(pwm_port_t *port)
{
    (void)port;
}

const pwm_port_ops_t pwm_port_ops_stm32g4_reg = {
    .set_duty3 = pwm_port_stm32g4_set_duty3,
    .enable = pwm_port_stm32g4_enable,
    .disable = pwm_port_stm32g4_disable,
};

/**
 * @brief 写三相比较值。
 */
void pwm_port_set_duty3(pwm_port_t *port, uint32_t ccr1, uint32_t ccr2, uint32_t ccr3)
{
    if (port == NULL || port->ops == NULL || port->ops->set_duty3 == NULL) {
        return;
    }
    port->ops->set_duty3(port, ccr1, ccr2, ccr3);
}

/**
 * @brief 开 PWM 输出。
 */
void pwm_port_enable(pwm_port_t *port)
{
    if (port == NULL || port->ops == NULL || port->ops->enable == NULL) {
        return;
    }
    port->ops->enable(port);
}

void pwm_port_disable(pwm_port_t *port)
{
    if (port == NULL || port->ops == NULL || port->ops->disable == NULL) {
        return;
    }
    port->ops->disable(port);
}
