/**
 * @file telem_ident_dump.c
 * @brief VOFA JustFloat Rs/Ld-Lq 辨识结果突发。
 */

#include "telem_ident_dump.h"

#include <stddef.h>

#include "ld_lq_ident.h"
#include "motor_params_m1.h"
#include "rs_ident.h"

#if defined(M1_VOFA_IDENT_DUMP_ENABLE) && (M1_VOFA_IDENT_DUMP_ENABLE != 0) && \
    ((defined(M1_RS_IDENT_ENABLE) && (M1_RS_IDENT_ENABLE != 0)) || \
     (defined(M1_LD_LQ_IDENT_ENABLE) && (M1_LD_LQ_IDENT_ENABLE != 0)))

typedef enum {
    TELEM_IDENT_DUMP_IDLE = 0,
    TELEM_IDENT_DUMP_HDR,
    TELEM_IDENT_DUMP_DATA,
    TELEM_IDENT_DUMP_TAIL,
    TELEM_IDENT_DUMP_DONE,
} telem_ident_dump_state_t;

static telem_ident_dump_state_t s_dump_state;
static uint8_t s_data_frame_idx;
static rs_ident_result_t s_rs_snap;
static ld_lq_ident_result_t s_ld_lq_snap;
static uint8_t s_grid_n;

static float telem_ident_dump_checksum(void)
{
    uint8_t i;
    float sum = 0.0f;

    for (i = 0u; i < s_grid_n; i++) {
        if (s_ld_lq_snap.ld_valid[i] != 0u) {
            sum += s_ld_lq_snap.ld_h[i] * 1e6f;
        }
        if (s_ld_lq_snap.lq_valid[i] != 0u) {
            sum += s_ld_lq_snap.lq_h[i] * 1e6f;
        }
        if (s_ld_lq_snap.ld_valid_fine[i] != 0u) {
            sum += s_ld_lq_snap.ld_h_fine[i] * 1e6f;
        }
        if (s_ld_lq_snap.lq_valid_fine[i] != 0u) {
            sum += s_ld_lq_snap.lq_h_fine[i] * 1e6f;
        }
#if M1_LD_LQ_IDENT_F2_ENABLE
        if (s_ld_lq_snap.ld_valid_f2[i] != 0u) {
            sum += s_ld_lq_snap.ld_h_f2[i] * 1e6f;
        }
        if (s_ld_lq_snap.lq_valid_f2[i] != 0u) {
            sum += s_ld_lq_snap.lq_h_f2[i] * 1e6f;
        }
#endif
    }
    return sum;
}

void telem_ident_dump_arm(void)
{
#if M1_LD_LQ_IDENT_ENABLE
    ld_lq_ident_get_result(&s_ld_lq_snap);
    s_grid_n = M1_LD_LQ_GRID_N;
#else
    s_ld_lq_snap = (ld_lq_ident_result_t){0};
    s_grid_n = 0u;
#endif

#if M1_RS_IDENT_ENABLE
    rs_ident_get_result(&s_rs_snap);
#else
    s_rs_snap = (rs_ident_result_t){0};
#endif

    if (s_grid_n == 0u && s_rs_snap.ok == 0u) {
        return;
    }

    s_dump_state = TELEM_IDENT_DUMP_HDR;
    s_data_frame_idx = 0u;
}

uint8_t telem_ident_dump_is_busy(void)
{
    return (s_dump_state == TELEM_IDENT_DUMP_HDR ||
            s_dump_state == TELEM_IDENT_DUMP_DATA ||
            s_dump_state == TELEM_IDENT_DUMP_TAIL) ?
           1u :
           0u;
}

int telem_ident_dump_next(float vals[], uint8_t ident_dump_ch)
{
    uint8_t k;
    uint8_t gi;

    if (vals == NULL || ident_dump_ch == 0u) {
        return 0;
    }

    if (s_dump_state == TELEM_IDENT_DUMP_IDLE ||
        s_dump_state == TELEM_IDENT_DUMP_DONE) {
        return 0;
    }

    switch (s_dump_state) {
    case TELEM_IDENT_DUMP_HDR:
        vals[0] = M1_VOFA_IDENT_MAGIC_HDR;
        vals[1] = (float)s_grid_n;
        vals[2] = s_rs_snap.rs_ohm;
        vals[3] = s_rs_snap.ok ? 1.0f : 0.0f;
        vals[4] = M1_VOFA_IDENT_PROTO_VER;
        vals[5] = (float)s_ld_lq_snap.n_ld_ok;
        if (ident_dump_ch > 6u) {
            vals[6] = (float)s_ld_lq_snap.n_lq_ok;
        }
        if (ident_dump_ch > 7u) {
            vals[7] = (float)s_ld_lq_snap.n_ld_ok_fine;
        }
        if (ident_dump_ch > 8u) {
            vals[8] = (float)s_ld_lq_snap.n_lq_ok_fine;
        }
        if (ident_dump_ch > 9u) {
            vals[9] = s_ld_lq_snap.ok ? 1.0f : 0.0f;
        }
        if (ident_dump_ch > 10u) {
            vals[10] = s_ld_lq_snap.ok_fine ? 1.0f : 0.0f;
        }
#if M1_LD_LQ_IDENT_F2_ENABLE
        if (ident_dump_ch > 11u) {
            vals[11] = (float)s_ld_lq_snap.n_ld_ok_f2;
        }
        for (k = 12u; k < ident_dump_ch; k++) {
            vals[k] = 0.0f;
        }
#else
        for (k = 11u; k < ident_dump_ch; k++) {
            vals[k] = 0.0f;
        }
#endif
        if (s_grid_n > 0u) {
            s_dump_state = TELEM_IDENT_DUMP_DATA;
            s_data_frame_idx = 0u;
        } else {
            s_dump_state = TELEM_IDENT_DUMP_TAIL;
        }
        return 1;

    case TELEM_IDENT_DUMP_DATA:
        gi = s_data_frame_idx;
        vals[0] = s_ld_lq_snap.id_bias[gi];
        vals[1] = s_ld_lq_snap.iq_bias[gi];
        vals[2] = s_ld_lq_snap.ld_h[gi] * 1e6f;
        vals[3] = s_ld_lq_snap.lq_h[gi] * 1e6f;
        vals[4] = s_ld_lq_snap.ld_h_fine[gi] * 1e6f;
        vals[5] = s_ld_lq_snap.lq_h_fine[gi] * 1e6f;
        vals[6] = s_ld_lq_snap.ld_valid[gi] ? 1.0f : 0.0f;
        vals[7] = s_ld_lq_snap.lq_valid[gi] ? 1.0f : 0.0f;
        vals[8] = s_ld_lq_snap.ld_valid_fine[gi] ? 1.0f : 0.0f;
        vals[9] = s_ld_lq_snap.lq_valid_fine[gi] ? 1.0f : 0.0f;
#if M1_LD_LQ_IDENT_F2_ENABLE
        vals[10] = s_ld_lq_snap.ld_h_f2[gi] * 1e6f;
        vals[11] = s_ld_lq_snap.lq_h_f2[gi] * 1e6f;
        for (k = 12u; k < ident_dump_ch; k++) {
            vals[k] = 0.0f;
        }
#else
        for (k = 10u; k < ident_dump_ch; k++) {
            vals[k] = 0.0f;
        }
#endif
        s_data_frame_idx++;
        if (s_data_frame_idx >= s_grid_n) {
            s_dump_state = TELEM_IDENT_DUMP_TAIL;
        }
        return 1;

    case TELEM_IDENT_DUMP_TAIL:
        vals[0] = M1_VOFA_IDENT_MAGIC_TAIL;
        vals[1] = (float)s_grid_n;
        vals[2] = telem_ident_dump_checksum();
        vals[3] = s_ld_lq_snap.rs_used_ohm;
#if M1_LD_LQ_IDENT_F2_ENABLE
        vals[4] = (float)s_ld_lq_snap.n_lq_ok_f2;
        vals[5] = s_ld_lq_snap.ok_f2 ? 1.0f : 0.0f;
        for (k = 6u; k < ident_dump_ch; k++) {
            vals[k] = 0.0f;
        }
#else
        vals[4] = 0.0f;
        vals[5] = 0.0f;
        for (k = 6u; k < ident_dump_ch; k++) {
            vals[k] = 0.0f;
        }
#endif
        s_dump_state = TELEM_IDENT_DUMP_DONE;
        return 1;

    default:
        return 0;
    }
}

#else /* M1_VOFA_IDENT_DUMP_ENABLE */

void telem_ident_dump_arm(void)
{
}

uint8_t telem_ident_dump_is_busy(void)
{
    return 0u;
}

int telem_ident_dump_next(float vals[], uint8_t ident_dump_ch)
{
    (void)vals;
    (void)ident_dump_ch;
    return 0;
}

#endif
