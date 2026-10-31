/**
 * @file isr_modulate.c
 * @date 2026-10-29
 * @brief 反 Park 已在外完成：本文件只写 SVPWM 并踢编码器 SPI。
 */
#include "isr/isr_frame.h"

#include "encoder.h"
#include "foc_svpwm.h"
#include "motor_params_m1.h"

/**
 * @brief 写三相占空并启动下一拍编码器传输。
 * @param f 本拍帧；axis/pwm 与 ud/uq/θ 已就绪。
 */
void isr_modulate_run(isr_frame_t *f)
{
    if ((f == NULL) || (f->axis == NULL)) {
        return;
    }

    foc_svpwm_apply_abc_sc(f->axis, f->uq_out, f->ud_out, f->theta_park,
                           f->sin_el, f->cos_el, f->ia, f->ib, f->ic,
                           f->id, f->iq);
    if (!M1_ENC_OPTIONAL_ENABLE) {
        if (f->axis->enc != NULL) {
            encoder_kick(f->axis->enc);
        }
    }
}
