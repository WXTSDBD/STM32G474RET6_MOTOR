/**
 * @file motor_phase_binding.c
 * @date 2026-10-06
 * @brief 相序 binding 实现：PWM CCR 与 ADC 三相重映射。
 *
 * 节拍限制见 motor_phase_binding.h 文件头。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#include "motor_phase_binding.h"

#include "dbg_monitor.h"
#include "pwm_port.h"

#include <stddef.h>

static motor_phase_binding_t s_binding;
static bool s_active;

/**
 * @brief 填成恒等映射：通道 i 对应相 i，符号全 +1。
 * @param b 写出缓冲。不可为 NULL。
 */
void motor_phase_binding_set_identity(motor_phase_binding_t *b)
{
    uint8_t i;

    if (b == NULL) {
        return;
    }

    b->magic = MOTOR_PHASE_BINDING_MAGIC;
    for (i = 0u; i < 3u; i++) {
        b->pwm_ch_to_phase[i] = i;
        b->adc_rank_to_phase[i] = i;
        b->phase_sign[i] = 1;
    }
    b->reserved = 0u;
}

/**
 * @brief 检查魔数、置换完整、符号只能是 ±1。
 * @param b 待查表。
 * @return 合法为 true。
 */
bool motor_phase_binding_is_valid(const motor_phase_binding_t *b)
{
    uint8_t i;
    uint8_t seen_pwm[3];
    uint8_t seen_adc[3];

    if (b == NULL || b->magic != MOTOR_PHASE_BINDING_MAGIC) {
        return false;
    }

    for (i = 0u; i < 3u; i++) {
        seen_pwm[i] = 0u;
        seen_adc[i] = 0u;
    }

    for (i = 0u; i < 3u; i++) {
        if (b->pwm_ch_to_phase[i] > 2u || b->adc_rank_to_phase[i] > 2u) {
            return false;
        }
        if (b->phase_sign[i] != 1 && b->phase_sign[i] != -1) {
            return false;
        }
        seen_pwm[b->pwm_ch_to_phase[i]] = 1u;
        seen_adc[b->adc_rank_to_phase[i]] = 1u;
    }

    return (seen_pwm[0] && seen_pwm[1] && seen_pwm[2] &&
            seen_adc[0] && seen_adc[1] && seen_adc[2]);
}

/**
 * @brief 启用或关闭运行时 remap。
 * @param b 启用时必须合法。enable 为 false 时可传 NULL。
 * @param enable true=拷贝并启用。
 */
void motor_phase_binding_set_active(const motor_phase_binding_t *b, bool enable)
{
    if (enable && b != NULL && motor_phase_binding_is_valid(b)) {
        s_binding = *b;
        s_active = true;
    } else if (!enable) {
        s_active = false;
    }
}

/**
 * @brief 查询 remap 是否已启用。
 * @return 已启用为 true。
 */
bool motor_phase_binding_is_active(void)
{
    return s_active;
}

/**
 * @brief 取出当前静态 binding，未启用时内容仍可能是旧值。
 * @return 内部表指针，不要释放。
 */
const motor_phase_binding_t *motor_phase_binding_get(void)
{
    return &s_binding;
}

/**
 * @brief 物理 JDR 顺序电流换成逻辑 Ia/Ib/Ic。
 * @param i_phys 长度 3，单位 A。不可为 NULL。
 * @param ia 逻辑 A 相，可为 NULL。
 * @param ib 逻辑 B 相，可为 NULL。
 * @param ic 逻辑 C 相，可为 NULL。
 */
void motor_phase_binding_map_abc(const float i_phys[3], float *ia, float *ib, float *ic)
{
    float logical[3];
    uint8_t i;
    uint8_t rank;

    if (i_phys == NULL) {
        return;
    }

    if (!s_active) {
        if (ia != NULL) {
            *ia = i_phys[0];
        }
        if (ib != NULL) {
            *ib = i_phys[1];
        }
        if (ic != NULL) {
            *ic = i_phys[2];
        }
        return;
    }

    for (i = 0u; i < 3u; i++) {
        rank = s_binding.adc_rank_to_phase[i];
        logical[i] = (float)s_binding.phase_sign[i] * i_phys[rank];
    }

    if (ia != NULL) {
        *ia = logical[0];
    }
    if (ib != NULL) {
        *ib = logical[1];
    }
    if (ic != NULL) {
        *ic = logical[2];
    }
}

/**
 * @brief 逻辑占空写成 TIM CCR，启用时按 pwm_ch_to_phase 换通道。
 * @param htim PWM 定时器。不可为 NULL。
 * @param ta 逻辑 A 相归一化占空。
 * @param tb 逻辑 B 相归一化占空。
 * @param tc 逻辑 C 相归一化占空。
 * @param pwm_period 定时器周期计数。
 */
void motor_phase_binding_write_ccr(TIM_HandleTypeDef *htim,
                                   float ta, float tb, float tc,
                                   uint16_t pwm_period)
{
    pwm_port_t port = {
        .ops = &pwm_port_ops_stm32g4_reg,
        .hw = htim,
        .user_ctx = NULL,
    };
    float duty_logical[3];
    float duty_phys[3];
    uint8_t i;
    uint8_t ch;
    uint32_t ccr1;
    uint32_t ccr2;
    uint32_t ccr3;

    if (htim == NULL) {
        return;
    }

    duty_logical[0] = ta;
    duty_logical[1] = tb;
    duty_logical[2] = tc;
    duty_phys[0] = ta;
    duty_phys[1] = tb;
    duty_phys[2] = tc;

    if (s_active) {
        for (i = 0u; i < 3u; i++) {
            ch = s_binding.pwm_ch_to_phase[i];
            duty_phys[ch] = duty_logical[i];
        }
    }

    ccr1 = (uint32_t)(duty_phys[0] * (float)pwm_period);
    ccr2 = (uint32_t)(duty_phys[1] * (float)pwm_period);
    ccr3 = (uint32_t)(duty_phys[2] * (float)pwm_period);
    dbg.foc_pwm_ccr1 = (float)ccr1;
    dbg.foc_pwm_ccr2 = (float)ccr2;
    dbg.foc_pwm_ccr3 = (float)ccr3;
    pwm_port_set_duty3(&port, ccr1, ccr2, ccr3);
}
