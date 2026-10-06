/**
 * @file rs_ident.h
 * @date 2026-10-06
 * @brief d 轴慢斜坡辨识定子电阻。

 *
 * tick 只允许从电流环节拍调用。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#ifndef RS_IDENT_H
#define RS_IDENT_H

#include <stdint.h>

#include "motor_params_m1.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float rs_ohm;
    float intercept_v;
    float rs_nominal;
    uint32_t n;
    uint8_t repeat_n;
    uint8_t ok;
} rs_ident_result_t;

#if M1_RS_IDENT_ENABLE

void rs_ident_init(void);
void rs_ident_arm(void);
float rs_ident_tick(float id_fb, float ud_pi);
uint8_t rs_ident_is_done(void);
uint8_t rs_ident_is_ok(void);
uint8_t rs_ident_open_seq_phase(void);
void rs_ident_get_result(rs_ident_result_t *out);
void rs_ident_sync_dbg(void);

#else

static inline void rs_ident_init(void) {}
static inline void rs_ident_arm(void) {}
static inline float rs_ident_tick(float id_fb, float ud_pi)
{
    (void)id_fb;
    (void)ud_pi;
    return 0.0f;
}
static inline uint8_t rs_ident_is_done(void) { return 1u; }
static inline uint8_t rs_ident_is_ok(void) { return 0u; }
static inline uint8_t rs_ident_open_seq_phase(void) { return 9u; }
static inline void rs_ident_get_result(rs_ident_result_t *out)
{
    if (out != 0) {
        *out = (rs_ident_result_t){0};
    }
}
static inline void rs_ident_sync_dbg(void) {}

#endif /* M1_RS_IDENT_ENABLE */

#ifdef __cplusplus
}
#endif

#endif /* RS_IDENT_H */
