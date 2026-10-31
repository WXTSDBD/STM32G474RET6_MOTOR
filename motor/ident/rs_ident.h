/**
 * @file rs_ident.h
 * @date 2026-10-06
 * @brief d 轴慢斜坡辨识定子电阻。
 *
 * tick 只允许从电流环节拍调用。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 *       API 始终声明；关 M1_RS_IDENT_ENABLE 时 .c 提供空桩。
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

void rs_ident_init(void);
void rs_ident_arm(void);
float rs_ident_tick(float id_fb, float ud_pi);
uint8_t rs_ident_is_done(void);
uint8_t rs_ident_is_ok(void);
uint8_t rs_ident_open_seq_phase(void);
void rs_ident_get_result(rs_ident_result_t *out);
void rs_ident_sync_dbg(void);

#ifdef __cplusplus
}
#endif

#endif /* RS_IDENT_H */
