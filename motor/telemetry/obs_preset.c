/**
 * @file obs_preset.c
 * @date 2026-10-31
 * @brief 旧 layout → 名字表；RTOS 调 obs_session_set。
 */
#include "telemetry/obs_preset.h"
#include "telemetry/obs_reg.h"

#include "motor_params_m1.h"

typedef struct {
    uint8_t id;
    uint8_t n;
    const char *const names[OBS_CH_MAX];
} obs_preset_row_t;

static const obs_preset_row_t s_presets[] = {
    {
        OBS_PRESET_FOC_DQ, 12u,
        { "ia", "ib", "ic", "id", "iq", "theta_el",
          "ud", "uq", "id_ref", "iq_ref", "duty_dev", "mark" }
    },
    {
        OBS_PRESET_SIGNOFF, 12u,
        { "ia", "ib", "ic", "iq", "theta_el", "theta_mech",
          "theta_ref", "theta_err", "omega_pll", "omega_ref", "iq_ref", "mark" }
    },
    {
        OBS_PRESET_SPEED, 12u,
        { "ia", "ib", "ic", "id", "iq", "theta_el",
          "ud", "uq", "omega_pll", "omega_ref", "isr_delta", "mark" }
    },
    {
        OBS_PRESET_IDENT_DUTY, 12u,
        { "ia", "ib", "ic", "id", "iq", "theta_el",
          "ud", "uq", "id_ref", "iq_ref", "isr_delta", "mark" }
    },
    {
        OBS_PRESET_OPT, 12u,
        { "theta_park_err", "enc_raw", "omega_ref", "iq", "iq_ref", "ud",
          "uq", "isr_delta", "ud_pi", "id", "status", "omega_fb" }
    },
};

/**
 * @brief 按现网配方宏挑默认预设 id。
 */
uint8_t obs_preset_default_id(void)
{
    if (M1_VOFA_OPT12) {
        return OBS_PRESET_OPT;
    }
    if (M1_VOFA_SIGNOFF_CH) {
        return OBS_PRESET_SIGNOFF;
    }
    if (M1_VOFA_IDENT_DUTY_12CH && !M1_SPEED_LOOP_ENABLE) {
        return OBS_PRESET_IDENT_DUTY;
    }
    if (M1_SPEED_LOOP_ENABLE && M1_PLL_ENABLE) {
        return OBS_PRESET_SPEED;
    }
    return OBS_PRESET_FOC_DQ;
}

/**
 * @brief 查表挂会话；decim=0 用配方默认。
 */
uint8_t obs_preset_apply(uint8_t id, uint8_t decim)
{
    uint8_t i;
    uint8_t n;
    uint8_t d = decim;
    const obs_preset_row_t *row = 0;

    if (d == 0u) {
        d = (uint8_t)M1_TELEM_BRINGUP_DECIMATION;
    }
    if (d == 0u) {
        d = 2u;
    }

    for (i = 0u; i < (uint8_t)(sizeof(s_presets) / sizeof(s_presets[0])); i++) {
        if (s_presets[i].id == id) {
            row = &s_presets[i];
            break;
        }
    }
    if (row == 0) {
        return 3u;
    }

    n = row->n;
    if (n > M1_TELEM_BRINGUP_K) {
        n = (uint8_t)M1_TELEM_BRINGUP_K;
    }
    return obs_session_set(row->names, n, d);
}
