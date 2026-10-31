/**
 * @file motor_if.c
 * @date 2026-10-06
 * @brief I-f 空壳（现网三 recipe 均 M1_IF_ENABLE=0）。
 *
 * 节拍限制见 motor_if.h 文件头。恢复 I-f 时从 git 取回活体。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#include "motor_if.h"

void motor_if_init(motor_context_t *ctx)
{
    (void)ctx;
}

void motor_if_arm(motor_context_t *ctx)
{
    (void)ctx;
}

void motor_if_release(void)
{
}

uint8_t motor_if_is_driving(void)
{
    return 0u;
}

void motor_if_set_target_rpm(float rpm)
{
    (void)rpm;
}

float motor_if_get_target_rpm(void)
{
    return 0.0f;
}

m1_if_state_t motor_if_get_state(void)
{
    return M1_IF_STOP;
}

motor_if_step_t motor_if_tick(motor_context_t *ctx, float theta_enc_park)
{
    motor_if_step_t out;

    (void)ctx;
    out.state = M1_IF_STOP;
    out.theta_park = theta_enc_park;
    out.iq_ref = 0.0f;
    out.id_ref = 0.0f;
    out.omega_cmd_rpm = 0.0f;
    out.omega_meas_rpm = 0.0f;
    out.theta_err_rad = 0.0f;
    return out;
}
