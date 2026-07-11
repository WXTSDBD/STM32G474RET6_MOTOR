/**
 * @file telem_lut_dump.c
 * @brief VOFA JustFloat LUT 表突发（M1_VOFA_LUT_DUMP_ENABLE 控制编译）。
 */

#include "telem_lut_dump.h"

#include <stddef.h>

#include "deadband_cal.h"
#include "motor_params_m1.h"

#if defined(M1_VOFA_LUT_DUMP_ENABLE) && (M1_VOFA_LUT_DUMP_ENABLE != 0) && \
    defined(M1_ID_LOCK_CAL_SWEEP) && (M1_ID_LOCK_CAL_SWEEP != 0)

typedef enum {
    TELEM_LUT_DUMP_IDLE = 0,
    TELEM_LUT_DUMP_HDR,
    TELEM_LUT_DUMP_DATA,
    TELEM_LUT_DUMP_TAIL,
    TELEM_LUT_DUMP_DONE,
} telem_lut_dump_state_t;

static telem_lut_dump_state_t s_dump_state;
static uint8_t s_data_frame_idx;

static float telem_lut_dump_vals_sum(void)
{
    uint8_t n = deadband_cal_len();
    uint8_t i;
    float sum = 0.0f;

    if (deadband_cal_phase_lut_triplet() != 0u) {
        for (i = 0u; i < 3u; i++) {
            const float *vals = deadband_cal_vals_phase_ph(i);
            uint8_t k;

            if (vals == NULL) {
                continue;
            }
            for (k = 0u; k < n; k++) {
                sum += vals[k];
            }
        }
    } else {
        const float *vals = deadband_cal_vals_phase();

        if (vals == NULL) {
            return 0.0f;
        }
        for (i = 0u; i < n; i++) {
            sum += vals[i];
        }
    }
    return sum;
}

static uint8_t telem_lut_dump_data_frame_count(uint8_t len)
{
    if (deadband_cal_phase_lut_triplet() != 0u) {
        return len;
    }
    return (uint8_t)((len + 2u) / 3u);
}

void telem_lut_dump_arm(void)
{
    if (deadband_cal_len() < 2u) {
        return;
    }

    s_dump_state = TELEM_LUT_DUMP_HDR;
    s_data_frame_idx = 0u;
}

uint8_t telem_lut_dump_is_busy(void)
{
    return (s_dump_state == TELEM_LUT_DUMP_HDR ||
            s_dump_state == TELEM_LUT_DUMP_DATA ||
            s_dump_state == TELEM_LUT_DUMP_TAIL) ?
           1u :
           0u;
}

int telem_lut_dump_next(float vals[], uint8_t lut_dump_ch)
{
    const float *amps;
    const float *lut_vals;
    uint8_t len;
    uint8_t base;
    uint8_t n_data;
    uint8_t k;

    if (vals == NULL || lut_dump_ch == 0u) {
        return 0;
    }

    if (s_dump_state == TELEM_LUT_DUMP_IDLE ||
        s_dump_state == TELEM_LUT_DUMP_DONE) {
        return 0;
    }

    amps = deadband_cal_amps_phase();
    lut_vals = deadband_cal_vals_phase();
    len = deadband_cal_len();
    if (amps == NULL || lut_vals == NULL || len < 2u) {
        s_dump_state = TELEM_LUT_DUMP_DONE;
        return 0;
    }

    switch (s_dump_state) {
    case TELEM_LUT_DUMP_HDR:
        vals[0] = M1_VOFA_LUT_MAGIC_HDR;
        vals[1] = (float)len;
        vals[2] = M1_RS_OHM;
        vals[3] = deadband_cal_outlier_seen() ? 1.0f : 0.0f;
#if M1_DEADBAND_LUT_APPLY_MIN_ENABLE
        vals[4] = 3.3f;
        vals[5] = M1_DEADBAND_LUT_APPLY_MIN_A;
#elif M1_DEADBAND_I_ZERO_DISABLE
        vals[4] = 3.2f;
        vals[5] = M1_DEADBAND_LUT_LOW_FLAT_ENABLE ? 1.0f : 0.0f;
#elif M1_DEADBAND_LUT_LOW_FLAT_ENABLE
        vals[4] = 3.1f;
        vals[5] = 1.0f;
#elif defined(M1_DEADBAND_GEO_BUILD_ENABLE) && (M1_DEADBAND_GEO_BUILD_ENABLE != 0)
        if (deadband_cal_phase_lut_triplet() != 0u) {
            vals[4] = 3.5f;
        } else {
            vals[4] = 3.4f;
        }
        vals[5] = (float)deadband_cal_geo_sample_len();
#else
        vals[4] = M1_VOFA_LUT_PROTO_VER;
        vals[5] = 0.0f;
#endif
        for (k = 6u; k < lut_dump_ch; k++) {
            vals[k] = 0.0f;
        }
        s_dump_state = TELEM_LUT_DUMP_DATA;
        s_data_frame_idx = 0u;
        return 1;

    case TELEM_LUT_DUMP_DATA:
        n_data = telem_lut_dump_data_frame_count(len);
        base = s_data_frame_idx;
        if (deadband_cal_phase_lut_triplet() != 0u) {
            const float *amps_b = deadband_cal_amps_phase_ph(1u);
            const float *amps_c = deadband_cal_amps_phase_ph(2u);
            const float *vals_b = deadband_cal_vals_phase_ph(1u);
            const float *vals_c = deadband_cal_vals_phase_ph(2u);

            vals[0] = (base < len) ? amps[base] : 0.0f;
            vals[1] = (base < len) ? lut_vals[base] : 0.0f;
            vals[2] = (base < len && amps_b != NULL) ? amps_b[base] : 0.0f;
            vals[3] = (base < len && vals_b != NULL) ? vals_b[base] : 0.0f;
            vals[4] = (base < len && amps_c != NULL) ? amps_c[base] : 0.0f;
            vals[5] = (base < len && vals_c != NULL) ? vals_c[base] : 0.0f;
        } else {
            base = (uint8_t)(s_data_frame_idx * 3u);
            vals[0] = (base < len) ? amps[base] : 0.0f;
            vals[1] = (base < len) ? lut_vals[base] : 0.0f;
            vals[2] = (base + 1u < len) ? amps[base + 1u] : 0.0f;
            vals[3] = (base + 1u < len) ? lut_vals[base + 1u] : 0.0f;
            vals[4] = (base + 2u < len) ? amps[base + 2u] : 0.0f;
            vals[5] = (base + 2u < len) ? lut_vals[base + 2u] : 0.0f;
        }
        for (k = 6u; k < lut_dump_ch; k++) {
            vals[k] = 0.0f;
        }
        s_data_frame_idx++;
        if (s_data_frame_idx >= n_data) {
            s_dump_state = TELEM_LUT_DUMP_TAIL;
        }
        return 1;

    case TELEM_LUT_DUMP_TAIL:
        vals[0] = M1_VOFA_LUT_MAGIC_TAIL;
        vals[1] = (float)len;
        vals[2] = telem_lut_dump_vals_sum();
        vals[3] = 0.0f;
        vals[4] = 0.0f;
        vals[5] = 0.0f;
        for (k = 6u; k < lut_dump_ch; k++) {
            vals[k] = 0.0f;
        }
        s_dump_state = TELEM_LUT_DUMP_DONE;
        return 1;

    default:
        return 0;
    }
}

#else /* M1_VOFA_LUT_DUMP_ENABLE && M1_ID_LOCK_CAL_SWEEP */

void telem_lut_dump_arm(void)
{
}

uint8_t telem_lut_dump_is_busy(void)
{
    return 0u;
}

int telem_lut_dump_next(float vals[], uint8_t lut_dump_ch)
{
    (void)vals;
    (void)lut_dump_ch;
    return 0;
}

#endif
