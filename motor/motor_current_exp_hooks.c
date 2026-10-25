/**
 * @file motor_current_exp_hooks.c
 * @date 2026-10-19
 * @brief 电流环实验旁路实现：宏关时为空桩，宏开时跑辨识/开环阶梯。
 *
 * 不改 GATE141 无感 ON 路径控制律；只把 OFF 嵌套挪出热路径源文件。
 * 节拍限制见 motor_current_exp_hooks.h 文件头。
 */

#include "motor_current_exp_hooks.h"

#include "motor_params_m1.h"

#if M1_IDENT_ENABLE && !M1_IDENT_ID_CAL_BEFORE_STEP && !M1_SPEED_IDENT_ENABLE
#include "ident_flow.h"
#endif
#if M1_IDENT_ENABLE || M1_DEADBAND_FLOW_ENABLE
#include "deadband_flow.h"
#endif
#if M1_LD_LQ_IDENT_ENABLE
#include "deadband_id_cal.h"
#include "ld_lq_ident.h"
#endif
#if (M1_OPEN_UQ_PRE_ID_CAL_ENABLE || M1_OPEN_UD_PRE_ID_CAL_ENABLE)
#include "motor_open_sweep.h"
#endif

#if M1_IDENT_ENABLE && !M1_IDENT_ID_CAL_BEFORE_STEP && !M1_SPEED_IDENT_ENABLE
/** 0=辨识流程还未 init。 */
static uint8_t s_ident_booted;
#endif

/**
 * @brief 辨识/死区流程节拍。SPEED_IDENT 仍留在 motor_current（要清观测器静态）。
 * @param ctx 控制上下文。不可为 NULL。
 */
void motor_current_exp_flow_tick(motor_context_t *ctx)
{
#if M1_IDENT_ENABLE && !M1_IDENT_ID_CAL_BEFORE_STEP && !M1_SPEED_IDENT_ENABLE
    if (ctx == 0) {
        return;
    }
    if (s_ident_booted == 0u) {
        ident_flow_init(ctx);
        s_ident_booted = 1u;
    }
    ident_flow_tick(ctx);
#elif M1_IDENT_ENABLE || M1_DEADBAND_FLOW_ENABLE
    if (ctx == 0) {
        return;
    }
    deadband_flow_tick(ctx);
#else
    (void)ctx;
#endif
}

/**
 * @brief Ld/Lq 辨识段改写 ud/uq。未开宏恒返回 0。
 * @param ctx 控制上下文。
 * @param id Id，单位 A。
 * @param iq Iq，单位 A。
 * @param ud 输出 d 轴电压，单位 V。不可为 NULL。
 * @param uq 输出 q 轴电压，单位 V。不可为 NULL。
 * @return 1=已改写且调用方应 break。
 */
uint8_t motor_current_exp_try_ld_lq_udq(motor_context_t *ctx, float id,
                                       float iq, float *ud, float *uq)
{
#if M1_LD_LQ_IDENT_ENABLE
    if ((ctx == 0) || (ud == 0) || (uq == 0)) {
        return 0u;
    }
    if (deadband_id_cal_in_ld_lq_ident() == 0u) {
        return 0u;
    }
#if M1_LD_LQ_IDENT_OPEN_LOOP_ENABLE
    if (ld_lq_ident_inject_active() != 0u) {
        *ud = ld_lq_ident_u_bias_d() + ld_lq_ident_u_inj_d();
        *uq = ld_lq_ident_u_bias_q() + ld_lq_ident_u_inj_q();
    } else {
        *ud = ctx->ud_pi;
        *uq = ctx->uq_pi;
    }
#else
    *ud = ctx->ud_pi + ld_lq_ident_u_inj_d();
    *uq = ctx->uq_pi + ld_lq_ident_u_inj_q();
#endif
    ld_lq_ident_integrate(id, iq, *ud, *uq);
    return 1u;
#else
    (void)ctx;
    (void)id;
    (void)iq;
    (void)ud;
    (void)uq;
    return 0u;
#endif
}

/**
 * @brief 开环预 Id 阶梯/对齐改写 ud/uq。未开宏恒返回 0。
 * @param ctx 控制上下文。
 * @param ud 输出 d 轴电压，单位 V。不可为 NULL。
 * @param uq 输出 q 轴电压，单位 V。不可为 NULL。
 * @return 1=已改写且调用方应 break。
 */
uint8_t motor_current_exp_try_open_udq(motor_context_t *ctx, float *ud,
                                      float *uq)
{
#if (M1_OPEN_UQ_PRE_ID_CAL_ENABLE || M1_OPEN_UD_PRE_ID_CAL_ENABLE) && \
    M1_OPEN_PRE_ID_LADDER_ALIGN_ENABLE
    if ((ctx == 0) || (ud == 0) || (uq == 0)) {
        return 0u;
    }
    if (motor_open_sweep_in_pre_id_align() != 0u) {
        *ud = M1_OPEN_PRE_ID_LADDER_ALIGN_UD_V;
        *uq = 0.0f;
        return 1u;
    }
#endif
#if M1_OPEN_UD_PRE_ID_CAL_ENABLE
    if ((ctx == 0) || (ud == 0) || (uq == 0)) {
        return 0u;
    }
    if (motor_open_sweep_in_pre_id_ud_ladder() != 0u) {
        *ud = ctx->ud_open;
        *uq = 0.0f;
        return 1u;
    }
#endif
#if !((M1_OPEN_UQ_PRE_ID_CAL_ENABLE || M1_OPEN_UD_PRE_ID_CAL_ENABLE) && \
      M1_OPEN_PRE_ID_LADDER_ALIGN_ENABLE) && !M1_OPEN_UD_PRE_ID_CAL_ENABLE
    (void)ctx;
    (void)ud;
    (void)uq;
#endif
    return 0u;
}
