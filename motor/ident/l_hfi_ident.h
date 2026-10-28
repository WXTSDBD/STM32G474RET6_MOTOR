/**
 * @file l_hfi_ident.h
 * @date 2026-10-26
 * @brief VESC 同构测 L：duty 爬升 + 旋转 HF + inv_L 环形缓冲 DFT bin0/bin2。
 *
 * 基波 Id/Iq 为 0；不在固定 q 轴灌持续方波。tick/on_sample 仅电流环节拍调用。
 * 每轮结束后 mark=55 连发本轮 Ld/Lq/Lavg（见 result_shout_*）。
 */
#ifndef L_HFI_IDENT_H
#define L_HFI_IDENT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float l_avg_h;
    float ld_lq_diff_h;
    float ld_h;
    float lq_h;
    float rs_used_ohm;
    float v_inj_used;
    uint8_t ok;
} l_hfi_ident_result_t;

void l_hfi_ident_init(void);
void l_hfi_ident_arm(float rs_ohm);
void l_hfi_ident_tick(float id_fb, float iq_fb);
/**
 * 在上一拍注入电压已作用后的电流上调用：翻号 / 填 inv_L / 推进角。
 * 调用后 ud/uq 为下一拍指令。
 */
void l_hfi_ident_on_sample(float id_fb, float iq_fb);
float l_hfi_ident_u_d(void);
float l_hfi_ident_u_q(void);
uint8_t l_hfi_ident_inject_active(void);
uint8_t l_hfi_ident_is_done(void);
uint8_t l_hfi_ident_is_ok(void);
uint8_t l_hfi_ident_open_seq_phase(void);
void l_hfi_ident_get_result(l_hfi_ident_result_t *out);
void l_hfi_ident_sync_dbg(void);
/** 1=正在连发本轮结果包（mark55）。 */
uint8_t l_hfi_ident_result_shout_active(void);
float l_hfi_ident_shout_ld_uH(void);
float l_hfi_ident_shout_lq_uH(void);
float l_hfi_ident_shout_lavg_uH(void);
float l_hfi_ident_shout_round(void);
float l_hfi_ident_shout_ok(void);

#ifdef __cplusplus
}
#endif

#endif /* L_HFI_IDENT_H */
