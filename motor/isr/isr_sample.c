/**
 * @file isr_sample.c
 * @date 2026-10-29
 * @brief 采样后处理：写回 ctx 并 Clarke。ADC/相序/扇区重构由入口完成。
 */
#include "isr/isr_frame.h"

#include "foc_svpwm.h"

/**
 * @brief 由已就绪的 ia/ib/ic 写 ctx 并做 Clarke。
 * @param f 帧；读 ia/ib/ic，写 i_alpha/i_beta 与 ctx->ia/ib/ic。
 */
void isr_sample_run(isr_frame_t *f)
{
    if ((f == NULL) || (f->ctx == NULL)) {
        return;
    }

    f->ctx->ia = f->ia;
    f->ctx->ib = f->ib;
    f->ctx->ic = f->ic;
    Clarke_Transform(f->ia, f->ib, f->ic, &f->i_alpha, &f->i_beta);
}
