/**
 * @file phase_detect.c
 * @date 2026-10-06
 * @brief 单相脉冲 rank/sign/gain 诊断：三档 Δ(400/600/800)，zeroed LSB 累加。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#include "phase_detect.h"

#include <stddef.h>
#include <string.h>

#include "dbg_monitor.h"
#include "factory_nvm.h"
#include "motor_params_m1.h"
#include "pwm_port.h"
#include "time_port.h"
#include "stm32g474xx.h"

#define PHASE_PWM_PERIOD      M1_PWM_ARR_COUNTS
#define PHASE_PWM_CENTER      2000u
#define PHASE_NEUTRAL_TICKS   4000u   /* 0.2 s @ 20 kHz */
#define PHASE_HALF_TICKS      6000u   /* 0.3 s 半周期 */
#define PHASE_CYCLES          2u
#define PHASE_DELTA_COUNT     3u
#define PHASE_S_MIN_LSB       8.0f
#define PHASE_RUN_TIMEOUT_MS  120000u

/* 与 Core/Inc/main.h USER CODE 中同名宏保持一致（本模块不再经 adc.h 间接包含）。 */
#define PHASE_CAL_FAIL_NONE     0u
#define PHASE_CAL_FAIL_OC       1u
#define PHASE_CAL_FAIL_SNR      2u
#define PHASE_CAL_FAIL_PERM     3u
#define PHASE_CAL_FAIL_FLASH    4u
#define PHASE_CAL_FAIL_TIMEOUT  5u

static const uint16_t s_delta_table[PHASE_DELTA_COUNT] = { 400u, 600u, 800u };

typedef enum {
    PHASE_ST_NEUTRAL = 0,
    PHASE_ST_PULSE_POS,
    PHASE_ST_PULSE_NEG,
    PHASE_ST_DONE,
    PHASE_ST_FAIL,
} phase_st_t;

volatile uint8_t g_phase_cal_active;
volatile uint8_t g_cal_hold;

static phase_st_t s_st;
static uint8_t s_pwm_idx;
static uint8_t s_delta_idx;
static uint8_t s_cycle_idx;
static uint32_t s_tick;
static uint32_t s_t0_ms;
static uint16_t s_delta_cur;

static float s_sum_pos[3];
static float s_sum_neg[3];
static float s_sum_neutral[3];
static uint32_t s_cnt_pos;
static uint32_t s_cnt_neg;
static uint32_t s_cnt_neutral;

static motor_phase_binding_t s_result;
static uint8_t s_pass_rank[3];
static int8_t s_pass_sign[3];
static uint8_t s_channels_done;
static uint8_t s_fail_reason;
static float s_last_snr;

static void phase_set_fail_reason(uint8_t reason)
{
    if (s_fail_reason == PHASE_CAL_FAIL_NONE) {
        s_fail_reason = reason;
    }
}

static void phase_pwm_neutral(pwm_port_t *pwm)
{
    if ((pwm == NULL) || (pwm->hw == NULL)) {
        return;
    }
    pwm_port_set_duty3(pwm,
                       (uint32_t)PHASE_PWM_CENTER,
                       (uint32_t)PHASE_PWM_CENTER,
                       (uint32_t)PHASE_PWM_CENTER);
}

static void phase_pwm_pulse(pwm_port_t *pwm, uint8_t pwm_ch, int16_t sign, uint16_t delta)
{
    uint32_t ccr[3];
    uint32_t pulse;

    if ((pwm == NULL) || (pwm->hw == NULL)) {
        return;
    }

    ccr[0] = (uint32_t)PHASE_PWM_CENTER;
    ccr[1] = (uint32_t)PHASE_PWM_CENTER;
    ccr[2] = (uint32_t)PHASE_PWM_CENTER;

    if (sign >= 0) {
        pulse = (uint32_t)PHASE_PWM_CENTER + (uint32_t)delta;
    } else if ((uint32_t)PHASE_PWM_CENTER > (uint32_t)delta) {
        pulse = (uint32_t)PHASE_PWM_CENTER - (uint32_t)delta;
    } else {
        pulse = 0u;
    }

    if (pulse > (uint32_t)PHASE_PWM_PERIOD) {
        pulse = (uint32_t)PHASE_PWM_PERIOD;
    }

    if (pwm_ch < 3u) {
        ccr[pwm_ch] = pulse;
    }

    pwm_port_set_duty3(pwm, ccr[0], ccr[1], ccr[2]);
}

static void phase_read_zeroed_lsb(const adc_sample_t *adc, float z[3])
{
    const adc_sample_config_t *cfg = adc->cfg;

    z[0] = (float)((int32_t)adc->raw[0] - cfg->ch[0].offset);
    z[1] = (float)((int32_t)adc->raw[1] - cfg->ch[1].offset);
    z[2] = (float)((int32_t)adc->raw[2] - cfg->ch[2].offset);
}

static void phase_update_vofa_dbg(void)
{
    dbg.phase_cal_pwm_idx = s_pwm_idx;
    dbg.phase_cal_delta_idx = s_delta_idx;
    if (s_st == PHASE_ST_NEUTRAL) {
        dbg.phase_cal_st = 0u;
    } else if (s_st == PHASE_ST_PULSE_POS) {
        dbg.phase_cal_st = 1u;
    } else if (s_st == PHASE_ST_PULSE_NEG) {
        dbg.phase_cal_st = 2u;
    } else {
        dbg.phase_cal_st = 3u;
    }
}

static void phase_reset_neutral_accum(void)
{
    memset(s_sum_neutral, 0, sizeof(s_sum_neutral));
    s_cnt_neutral = 0u;
}

static void phase_reset_bipolar_accum(void)
{
    memset(s_sum_pos, 0, sizeof(s_sum_pos));
    memset(s_sum_neg, 0, sizeof(s_sum_neg));
    s_cnt_pos = 0u;
    s_cnt_neg = 0u;
    s_cycle_idx = 0u;
}

static void phase_begin_half(int16_t sign)
{
    s_tick = 0u;
    if (sign > 0) {
        s_st = PHASE_ST_PULSE_POS;
    } else {
        s_st = PHASE_ST_PULSE_NEG;
    }
    phase_update_vofa_dbg();
}

static bool phase_rank_already_used(uint8_t rank)
{
    uint8_t i;

    for (i = 0u; i < s_pwm_idx; i++) {
        if (s_pass_rank[i] == rank) {
            return true;
        }
    }
    return false;
}

static void phase_store_tier_result(uint8_t pwm, uint8_t tier, const float s[3])
{
    uint8_t k;

    for (k = 0u; k < 3u; k++) {
        dbg.phase_cal_bipolar_lsb[pwm][tier][k] = s[k];
    }
}

static bool phase_eval_tier(uint8_t pwm, uint8_t tier, float s_out[3])
{
    float mean_pos[3];
    float mean_neg[3];
    uint8_t k;

    if (s_cnt_pos == 0u || s_cnt_neg == 0u) {
        return false;
    }

    for (k = 0u; k < 3u; k++) {
        mean_pos[k] = s_sum_pos[k] / (float)s_cnt_pos;
        mean_neg[k] = s_sum_neg[k] / (float)s_cnt_neg;
        s_out[k] = mean_pos[k] - mean_neg[k];
    }

    phase_store_tier_result(pwm, tier, s_out);
    return true;
}

static bool phase_eval_pwm_from_tier(uint8_t tier)
{
    float s[3];
    uint8_t dom = 0xFFu;
    uint8_t sec = 0xFFu;
    float sec_s = 0.0f;
    uint8_t k;

    for (k = 0u; k < 3u; k++) {
        s[k] = dbg.phase_cal_bipolar_lsb[s_pwm_idx][tier][k];
    }

    for (k = 0u; k < 3u; k++) {
        if (phase_rank_already_used(k)) {
            continue;
        }
        if (s[k] > PHASE_S_MIN_LSB) {
            if (dom == 0xFFu || s[k] > s[dom]) {
                dom = k;
            }
        }
    }

    if (dom == 0xFFu) {
        s_last_snr = 0.0f;
        return false;
    }

    for (k = 0u; k < 3u; k++) {
        if (k == dom || phase_rank_already_used(k)) {
            continue;
        }
        if (s[k] > PHASE_S_MIN_LSB) {
            if (sec == 0xFFu || s[k] > sec_s) {
                sec = k;
                sec_s = s[k];
            }
        }
    }

    if (sec == 0xFFu || sec_s < 1e-3f) {
        s_last_snr = 99.0f;
    } else {
        s_last_snr = s[dom] / sec_s;
    }

    s_pass_rank[s_pwm_idx] = dom;
    s_pass_sign[s_pwm_idx] = (s[dom] >= 0.0f) ? 1 : -1;
    dbg.phase_cal_dom_s_lsb[s_pwm_idx] = s[dom];
    return true;
}

static void phase_build_binding(motor_phase_binding_t *b)
{
    uint8_t i;

    motor_phase_binding_set_identity(b);
    for (i = 0u; i < 3u; i++) {
        b->pwm_ch_to_phase[i] = i;
        b->adc_rank_to_phase[i] = s_pass_rank[i];
        b->phase_sign[i] = s_pass_sign[i];
    }
}

static bool phase_binding_permutation_ok(const motor_phase_binding_t *b)
{
    uint8_t seen[3] = { 0u, 0u, 0u };
    uint8_t i;

    if (!motor_phase_binding_is_valid(b)) {
        return false;
    }

    for (i = 0u; i < 3u; i++) {
        seen[b->adc_rank_to_phase[i]] = 1u;
    }
    return (seen[0] && seen[1] && seen[2]);
}

static void phase_reset_run(void)
{
    uint8_t p;
    uint8_t t;
    uint8_t r;

    s_st = PHASE_ST_NEUTRAL;
    s_pwm_idx = 0u;
    s_delta_idx = 0u;
    s_cycle_idx = 0u;
    s_tick = 0u;
    s_delta_cur = s_delta_table[0];
    s_channels_done = 0u;
    s_fail_reason = PHASE_CAL_FAIL_NONE;
    s_last_snr = 0.0f;
    phase_reset_neutral_accum();
    phase_reset_bipolar_accum();
    memset(s_pass_rank, 0, sizeof(s_pass_rank));
    memset(s_pass_sign, 0, sizeof(s_pass_sign));
    motor_phase_binding_set_identity(&s_result);

    for (p = 0u; p < 3u; p++) {
        for (t = 0u; t < PHASE_DELTA_COUNT; t++) {
            for (r = 0u; r < 3u; r++) {
                dbg.phase_cal_bipolar_lsb[p][t][r] = 0.0f;
            }
        }
        dbg.phase_cal_dom_s_lsb[p] = 0.0f;
    }
    phase_update_vofa_dbg();
}

static void phase_advance_after_tier(void)
{
    float s_tmp[3];

    (void)phase_eval_tier(s_pwm_idx, s_delta_idx, s_tmp);

    s_delta_idx++;
    if (s_delta_idx < PHASE_DELTA_COUNT) {
        s_delta_cur = s_delta_table[s_delta_idx];
        s_st = PHASE_ST_NEUTRAL;
        s_tick = 0u;
        phase_update_vofa_dbg();
        return;
    }

    if (phase_eval_pwm_from_tier(1u)) {
        s_channels_done++;
        dbg.phase_cal_channels_done = s_channels_done;
    } else {
        phase_set_fail_reason(PHASE_CAL_FAIL_SNR);
    }

    s_pwm_idx++;
    s_delta_idx = 0u;
    s_delta_cur = s_delta_table[0];

    if (s_pwm_idx >= 3u) {
        phase_build_binding(&s_result);
        if (phase_binding_permutation_ok(&s_result)) {
            s_st = PHASE_ST_DONE;
        } else {
            phase_set_fail_reason(PHASE_CAL_FAIL_PERM);
            s_st = PHASE_ST_FAIL;
        }
    } else {
        s_st = PHASE_ST_NEUTRAL;
        s_tick = 0u;
    }
    phase_update_vofa_dbg();
}

/**
 * @brief 把标定失败原因和通道结果填进 VOFA。
 */
void phase_detect_fill_dbg(void)
{
    uint8_t i;

    dbg.phase_cal_fail_reason = s_fail_reason;
    dbg.phase_cal_channels_done = s_channels_done;
    dbg.phase_cal_last_snr = s_last_snr;
    for (i = 0u; i < 3u; i++) {
        dbg.pwm_ch_to_phase_dbg[i] = i;
        dbg.adc_rank_to_phase_dbg[i] = s_pass_rank[i];
        dbg.phase_sign_dbg[i] = s_pass_sign[i];
    }
    phase_update_vofa_dbg();
}

/**
 * @brief 标定期间每拍采样并推进单相脉冲状态机。
 * @param adc 采样实例。不可为 NULL。
 * @param hadc 注入完成的 ADC 句柄（void*，由 adc_sample 解释）。不可为 NULL。
 * @param pwm PWM 口。不可为 NULL。
 */
void phase_detect_jeoc_tick(adc_sample_t *adc, void *hadc, pwm_port_t *pwm)
{
    float z[3];

    if (adc == NULL || hadc == NULL || pwm == NULL || !g_phase_cal_active) {
        return;
    }

    adc_sample_jeoc_foc(adc, hadc);
    phase_read_zeroed_lsb(adc, z);

    if (s_st == PHASE_ST_PULSE_POS) {
        s_sum_pos[0] += z[0];
        s_sum_pos[1] += z[1];
        s_sum_pos[2] += z[2];
        s_cnt_pos++;
    } else if (s_st == PHASE_ST_PULSE_NEG) {
        s_sum_neg[0] += z[0];
        s_sum_neg[1] += z[1];
        s_sum_neg[2] += z[2];
        s_cnt_neg++;
    }

    switch (s_st) {
    case PHASE_ST_NEUTRAL:
        phase_pwm_neutral(pwm);
        if (s_tick == 0u) {
            phase_reset_neutral_accum();
        }
        s_sum_neutral[0] += z[0];
        s_sum_neutral[1] += z[1];
        s_sum_neutral[2] += z[2];
        s_cnt_neutral++;
        s_tick++;
        if (s_tick >= PHASE_NEUTRAL_TICKS) {
            phase_reset_bipolar_accum();
            phase_begin_half(1);
            phase_pwm_pulse(pwm, s_pwm_idx, 1, s_delta_cur);
        }
        break;

    case PHASE_ST_PULSE_POS:
        phase_pwm_pulse(pwm, s_pwm_idx, 1, s_delta_cur);
        s_tick++;
        if (s_tick >= PHASE_HALF_TICKS) {
            phase_begin_half(-1);
            phase_pwm_pulse(pwm, s_pwm_idx, -1, s_delta_cur);
        }
        break;

    case PHASE_ST_PULSE_NEG:
        phase_pwm_pulse(pwm, s_pwm_idx, -1, s_delta_cur);
        s_tick++;
        if (s_tick >= PHASE_HALF_TICKS) {
            s_cycle_idx++;
            if (s_cycle_idx < PHASE_CYCLES) {
                phase_begin_half(1);
                phase_pwm_pulse(pwm, s_pwm_idx, 1, s_delta_cur);
            } else {
                phase_advance_after_tier();
                phase_pwm_neutral(pwm);
            }
        }
        break;

    case PHASE_ST_DONE:
    case PHASE_ST_FAIL:
    default:
        phase_pwm_neutral(pwm);
        break;
    }
}

/**
 * @brief 标定结束后保持中性 PWM，只刷新采样给 VOFA。
 * @param adc 采样实例。不可为 NULL。
 * @param hadc 注入完成的 ADC 句柄（void*）。不可为 NULL。
 * @param pwm PWM 口。不可为 NULL。
 */
void phase_detect_hold_jeoc_tick(adc_sample_t *adc, void *hadc, pwm_port_t *pwm)
{
    if (adc == NULL || hadc == NULL || pwm == NULL || !g_cal_hold) {
        return;
    }

    phase_pwm_neutral(pwm);
    adc_sample_jeoc_foc(adc, hadc);
    dbg.phase_cal_st = 4u;
}

/**
 * @brief 阻塞跑完单相脉冲诊断，可选写入 Flash。
 * @param adc 采样实例。不可为 NULL。
 * @param pwm PWM 口。不可为 NULL。
 * @param out 写出 binding。不可为 NULL。
 * @param write_flash true 则成功后写 NVM。
 * @return 诊断成功为 true。
 * @note 本函数会空转等待，只能在任务里调用。
 */
bool phase_detect_run(adc_sample_t *adc,
                      pwm_port_t *pwm,
                      motor_phase_binding_t *out,
                      bool write_flash)
{
    bool ok;
    bool detected;

    if (adc == NULL || pwm == NULL || out == NULL) {
        return false;
    }

    g_cal_hold = 0u;
    phase_reset_run();
    g_phase_cal_active = 1u;
    s_t0_ms = time_port_ms();

    for (;;) {
        if ((time_port_ms() - s_t0_ms) > PHASE_RUN_TIMEOUT_MS) {
            phase_set_fail_reason(PHASE_CAL_FAIL_TIMEOUT);
            s_st = PHASE_ST_FAIL;
            break;
        }
        if (s_st == PHASE_ST_DONE || s_st == PHASE_ST_FAIL) {
            break;
        }
        __WFI();
    }

    g_phase_cal_active = 0u;
    phase_pwm_neutral(pwm);
    detected = (s_st == PHASE_ST_DONE && phase_binding_permutation_ok(&s_result));
    ok = detected;

    if (ok) {
        *out = s_result;
        if (write_flash) {
            ok = factory_nvm_write_phase(out);
            if (!ok) {
                phase_set_fail_reason(PHASE_CAL_FAIL_FLASH);
            }
        }
    } else {
        motor_phase_binding_set_identity(out);
    }

    phase_detect_fill_dbg();
    return ok;
}
