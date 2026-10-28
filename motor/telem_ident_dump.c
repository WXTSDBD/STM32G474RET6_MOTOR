/**
 * @file telem_ident_dump.c
 * @date 2026-10-06
 * @brief 辨识结果突发状态机（Rs + l_hfi 标量；无 VASI 网格）。
 *
 * 节拍限制见 telem_ident_dump.h 文件头。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#include "telem_ident_dump.h"

#include <stddef.h>

#include "l_hfi_ident.h"
#include "motor_params_m1.h"
#include "rs_ident.h"

#if defined(M1_VOFA_IDENT_DUMP_ENABLE) && (M1_VOFA_IDENT_DUMP_ENABLE != 0) && \
    ((defined(M1_RS_IDENT_ENABLE) && (M1_RS_IDENT_ENABLE != 0)) || \
     (defined(M1_L_HFI_IDENT_ENABLE) && (M1_L_HFI_IDENT_ENABLE != 0)))

typedef enum {
    TELEM_IDENT_DUMP_IDLE = 0,
    TELEM_IDENT_DUMP_HDR,
    TELEM_IDENT_DUMP_TAIL,
    TELEM_IDENT_DUMP_DONE,
} telem_ident_dump_state_t;

static telem_ident_dump_state_t s_dump_state;
static rs_ident_result_t s_rs_snap;
static l_hfi_ident_result_t s_l_snap;

static float telem_ident_dump_checksum(void)
{
    return (s_l_snap.ld_h + s_l_snap.lq_h + s_l_snap.l_avg_h) * 1e6f;
}

void telem_ident_dump_arm(void)
{
#if M1_L_HFI_IDENT_ENABLE
    l_hfi_ident_get_result(&s_l_snap);
#else
    s_l_snap = (l_hfi_ident_result_t){0};
#endif

#if M1_RS_IDENT_ENABLE
    rs_ident_get_result(&s_rs_snap);
#else
    s_rs_snap = (rs_ident_result_t){0};
#endif

    if ((s_l_snap.ok == 0u) && (s_rs_snap.ok == 0u)) {
        return;
    }

    s_dump_state = TELEM_IDENT_DUMP_HDR;
}

uint8_t telem_ident_dump_is_busy(void)
{
    return (s_dump_state == TELEM_IDENT_DUMP_HDR ||
            s_dump_state == TELEM_IDENT_DUMP_TAIL) ?
           1u :
           0u;
}

int telem_ident_dump_next(float vals[], uint8_t ident_dump_ch)
{
    uint8_t k;

    if (vals == NULL || ident_dump_ch == 0u) {
        return 0;
    }

    if (s_dump_state == TELEM_IDENT_DUMP_IDLE ||
        s_dump_state == TELEM_IDENT_DUMP_DONE) {
        return 0;
    }

    switch (s_dump_state) {
    case TELEM_IDENT_DUMP_HDR:
        /* proto 4：无网格；I5..I8=Ld/Lq/Lavg_uH、l_ok */
        vals[0] = M1_VOFA_IDENT_MAGIC_HDR;
        vals[1] = 0.0f;
        vals[2] = s_rs_snap.rs_ohm;
        vals[3] = s_rs_snap.ok ? 1.0f : 0.0f;
        vals[4] = M1_VOFA_IDENT_PROTO_VER;
        vals[5] = s_l_snap.ld_h * 1e6f;
        if (ident_dump_ch > 6u) {
            vals[6] = s_l_snap.lq_h * 1e6f;
        }
        if (ident_dump_ch > 7u) {
            vals[7] = s_l_snap.l_avg_h * 1e6f;
        }
        if (ident_dump_ch > 8u) {
            vals[8] = s_l_snap.ok ? 1.0f : 0.0f;
        }
        for (k = 9u; k < ident_dump_ch; k++) {
            vals[k] = 0.0f;
        }
        s_dump_state = TELEM_IDENT_DUMP_TAIL;
        return 1;

    case TELEM_IDENT_DUMP_TAIL:
        vals[0] = M1_VOFA_IDENT_MAGIC_TAIL;
        vals[1] = 0.0f;
        vals[2] = telem_ident_dump_checksum();
        vals[3] = s_l_snap.rs_used_ohm;
        vals[4] = 0.0f;
        vals[5] = 0.0f;
        for (k = 6u; k < ident_dump_ch; k++) {
            vals[k] = 0.0f;
        }
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
