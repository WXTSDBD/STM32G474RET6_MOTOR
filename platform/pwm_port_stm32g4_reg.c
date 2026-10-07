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

/**
 * @brief 按 ch_map 把逻辑三相 CCR 写进 TIM CCRx。
 * @param port PWM 口。不可为 NULL，hw/Instance 已绑定。
 * @param ccr1 逻辑 A 相比较值。
 * @param ccr2 逻辑 B 相比较值。
 * @param ccr3 逻辑 C 相比较值。
 */
static void pwm_port_stm32g4_set_duty3(pwm_port_t *port,
                                       uint32_t ccr1,
                                       uint32_t ccr2,
                                       uint32_t ccr3)
{
    TIM_HandleTypeDef *htim;
    const uint32_t ccr_in[3] = {ccr1, ccr2, ccr3};
    uint8_t i;
    uint8_t ch;

    if (port == NULL) {
        return;
    }

    htim = (TIM_HandleTypeDef *)port->hw;
    if (htim == NULL || htim->Instance == NULL) {
        return;
    }

    for (i = 0u; i < 3u; i++) {
        ch = port->ch_map[i];
        if (ch == 0u) {
            htim->Instance->CCR1 = (uint16_t)ccr_in[i];
        } else if (ch == 1u) {
            htim->Instance->CCR2 = (uint16_t)ccr_in[i];
        } else if (ch == 2u) {
            htim->Instance->CCR3 = (uint16_t)ccr_in[i];
        }
    }
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
