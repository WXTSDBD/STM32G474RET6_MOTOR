/**
 * @file hfi_hand.c
 * @brief P5: HFI helpers moved from motor_current.c (call order unchanged).
 */
#include "observer/obs_cfg.h"
#include "motor_math.h"
#include "motor_context.h"
#include "motor_trig.h"
#include "dbg_monitor.h"
#include "foc_pi.h"
#include "observer/hfi_sqwave.h"
#include "observer/emf_pll.h"
#include "observer/hfi_current_priv.h"

#if M1_HFI_SMO_HAND_ENABLE && M1_EMF_PLL_ENABLE
/*
 * 交接状态机（可组合，一变量一轮）。
 *   55：残 Vh，不开 Id。
 *   KILL_VH：ANG 后收 Vh→VH_END。6/57：END=0 噪；59：微地板 PASS）。
 *   OPEN_ID：ANG→HOLD→残地板开 Id。8 FAIL）。
 *   OPEN_ID+KILL：ANG→VH0→IDUP(微地。→SMO。
 *   61：冻 iq_ref（诊断）。2：W_HOLD——开 Id 窗速度反馈。VH0 。ω，SMO 软释放。
 *   OVERLAP：旧序（勿作主路径）。
 */
/** @brief Hermite smoothstep。。，两端斜。0（比线性更软）。*/
float hfi_hand_smoothstep(float a)
{
    if (a <= 0.0f) {
        return 0.0f;
    }
    if (a >= 1.0f) {
        return 1.0f;
    }
    return a * a * (3.0f - 2.0f * a);
}

static uint8_t s_hand_state;
static uint8_t s_hand_armed;
static uint8_t s_hand_ok;
static uint16_t s_hand_n;
static emf_pll_t *s_pll_ref;
static uint16_t s_hand_bad;
static float s_hand_alpha; /* 速度权重 */
static float s_hand_ang;   /* 角度权重；ANG 段仍。VH_FLOOR */
static float s_hand_vh;
static float s_hand_th;
static float s_hand_w;
static float s_hand_dth; /* 收注入前记下的平滑角差，权重只转这个 */

uint8_t hfi_hand_ok(void)
{
    return s_hand_ok;
}

uint8_t hfi_hand_state(void)
{
    return s_hand_state;
}

float hfi_hand_ang(void)
{
    return s_hand_ang;
}

float hfi_hand_dth(void)
{
    return s_hand_dth;
}
#if M1_HFI_ROTATE_PI_ENABLE
static float s_rot_th_prev;
static uint8_t s_rot_th_ok;
static uint8_t s_rot_armed; /* ang<1 置位；ang 到 1 旋一次 */
#endif
#if M1_HFI_HAND_REV_ENABLE
static uint8_t s_hand_rev_arm; /* 1：已爬过 REV_ARM，允许减速反。*/
#if M1_HFI_HAND_REV_WAKE_ENABLE
static uint16_t s_hand_rev_wait; /* RQUAL 等待计数 */
static uint8_t s_hand_rev_seeded; /* RVH 已 seed 一次 */
#if M1_HFI_HAND_REV_VH_MIRROR_ENABLE
static uint8_t s_hand_rev_vh_phase; /* 0：反演 VH0；1：反演 FADE→WAKE */
#endif
#endif
#endif
#if M1_HFI_HAND_IQ_HOLD_ON_IDUP
static float s_hand_iq_hold; /* VH0 末锁定的 iq_ref；IDUP 期间钉住 */
#endif
#if M1_HFI_HAND_DECEL_BRAKE_ENABLE
static uint8_t s_hand_brake_arm; /* SMO 内已见过高 ω* */
static uint8_t s_hand_brake_on;  /* 减速 |Iq| 地板窗 */
static float s_hand_wref_prev;   /* 上一拍 ω* */
#endif
#if M1_HFI_HAND_W_HOLD_ON_IDUP
static float s_hand_w_hold; /* VH0 。SMO 平均转速；开 Id 窗给速度。*/
static uint16_t s_hand_w_rel_n; /* SMO 段软释放计数 */
#endif
#if M1_HFI_HAND_W_SLEW_ENABLE
static float s_hand_w_slew;
static uint8_t s_hand_w_slew_on;
static uint16_t s_hand_w_slew_n; /* SMO 续限斜率计数 */
#endif

#if M1_HFI_ROTATE_PI_ENABLE
/**
 * @brief 把上一拍 dq 电压旋到本拍 Park，积分无扰预加载
 * @note 只用于「换 d 轴定义」那一拍，不要每拍跟 θ̂ 旋
 */
void hfi_hand_rotate_current_pi(motor_context_t *ctx, float dth,
                                       float id, float iq)
{
    float c;
    float s;
    float ud;
    float uq;
    float ud2;
    float uq2;

    if (ctx == NULL) {
        return;
    }
    motor_trig_sincos(dth, &c, &s);
    ud = ctx->ud_pi;
    uq = ctx->uq_pi;
    ud2 = ud * c + uq * s;
    uq2 = -ud * s + uq * c;
    ctx->ud_pi = ud2;
    ctx->uq_pi = uq2;
    foc_pi_bumpless(&ctx->pi_id, ud2, ctx->id_ref, id);
    foc_pi_bumpless(&ctx->pi_iq, uq2, ctx->iq_ref, iq);
}
#endif

/** SMO 20 ms 平均转。。电角速度。收注入。θ̂ 只跟这个，不跟瞬。ω。*/
float hfi_hand_w_el(void)
{
    return s_hand_w * 0.104719755f * (float)OBS_POLE_PAIRS;
}

void hfi_smo_hand_idle(void)
{
    s_hand_state = HFI_HAND_HFI;
    s_hand_armed = 1u;
    s_hand_n = 0u;
    s_hand_bad = 0u;
    s_hand_alpha = 0.0f;
    s_hand_ang = 0.0f;
    s_hand_vh = 1.0f;
    s_hand_dth = 0.0f;
#if M1_HFI_ROTATE_PI_ENABLE
    s_rot_th_ok = 0u;
    s_rot_armed = 1u;
#endif
#if M1_HFI_HAND_REV_ENABLE
    s_hand_rev_arm = 0u;
#if M1_HFI_HAND_REV_WAKE_ENABLE
    s_hand_rev_wait = 0u;
    s_hand_rev_seeded = 0u;
#endif
#endif
#if M1_HFI_HAND_IQ_HOLD_ON_IDUP
    s_hand_iq_hold = 0.0f;
#endif
#if M1_HFI_HAND_DECEL_BRAKE_ENABLE
    s_hand_brake_arm = 0u;
    s_hand_brake_on = 0u;
    s_hand_wref_prev = 0.0f;
#endif
#if M1_HFI_HAND_W_HOLD_ON_IDUP
    s_hand_w_hold = 0.0f;
    s_hand_w_rel_n = 0u;
#endif
#if M1_HFI_HAND_W_SLEW_ENABLE
    s_hand_w_slew = 0.0f;
    s_hand_w_slew_on = 0u;
    s_hand_w_slew_n = 0u;
#endif
    hfi_sqwave_set_hat_hold(0u);
    hfi_sqwave_set_iq_auth_hold(0u);
    hfi_sqwave_set_inj_scale(1.0f);
    hfi_sqwave_set_id_pi_release(0u);
    hfi_sqwave_set_id_pi_soft_cmd(-1.0f);
    dbg.obs_ss_alpha = 0.0f;
    dbg.obs_ss_state = 0.0f;
#if (M1_HFI_GATE == 78) || (M1_HFI_GATE == 79) || (M1_HFI_GATE == 80)
    hfi_vesc_win_smo_set(0u);
    dbg.hfi_vesc_win_smo = 0.0f;
#if M1_HFI_GATE == 80
    hfi_vesc_ho_set(0u, 0u);
#endif
#endif
}

/** 速度还在混、注入还在时，角或解调离开切换前的带就退回 HFI。 */
void hfi_hand_abort(void)
{
    s_hand_state = HFI_HAND_HFI;
    s_hand_n = 0u;
    s_hand_bad = 0u;
    s_hand_alpha = 0.0f;
    s_hand_ang = 0.0f;
    s_hand_vh = 1.0f;
    s_hand_dth = 0.0f;
#if M1_HFI_ROTATE_PI_ENABLE
    s_rot_th_ok = 0u;
    s_rot_armed = 1u;
#endif
#if (M1_HFI_GATE == 78) || (M1_HFI_GATE == 79) || (M1_HFI_GATE == 80)
    hfi_vesc_win_smo_set(0u);
    dbg.hfi_vesc_win_smo = 0.0f;
#if M1_HFI_GATE == 80
    hfi_vesc_ho_set(0u, 0u);
#endif
#endif
#if M1_HFI_HAND_REV_ENABLE
    s_hand_rev_arm = 0u;
#if M1_HFI_HAND_REV_WAKE_ENABLE
    s_hand_rev_wait = 0u;
    s_hand_rev_seeded = 0u;
#endif
#endif
#if M1_HFI_HAND_IQ_HOLD_ON_IDUP
    s_hand_iq_hold = 0.0f;
#endif
#if M1_HFI_HAND_DECEL_BRAKE_ENABLE
    s_hand_brake_arm = 0u;
    s_hand_brake_on = 0u;
    s_hand_wref_prev = 0.0f;
#endif
#if M1_HFI_HAND_W_HOLD_ON_IDUP
    s_hand_w_hold = 0.0f;
    s_hand_w_rel_n = 0u;
#endif
#if M1_HFI_HAND_W_SLEW_ENABLE
    s_hand_w_slew = 0.0f;
    s_hand_w_slew_on = 0u;
    s_hand_w_slew_n = 0u;
#endif
    s_hand_armed = 0u;
    hfi_sqwave_set_hat_hold(0u);
    hfi_sqwave_set_iq_auth_hold(0u);
    hfi_sqwave_set_id_pi_release(0u);
    hfi_sqwave_set_id_pi_soft_cmd(-1.0f);
}

/**
 * @brief 注入还开着时看角差、ε、x。持。20 ms 或一次越出大界就退回。
 * @param ad |θ_smo−θ̂|
 * @param dw |ω_hfi−ω_smo| [rpm]
 * @param spd_blend 1=混速段：x 只走杀门、不进温和累计（1718：x 尖到 0.5 。0.36 温和门打回）
 * @return 1 本拍已退。
 */
uint8_t hfi_hand_watch(float ad, float dw, uint8_t spd_blend)
{
    float ae;
    float x;
    float x_kill_hi;
    uint8_t mild;
    uint8_t kill;

    ae = hfi_sqwave_get_eps();
    if (ae < 0.0f) {
        ae = -ae;
    }
    x = hfi_sqwave_get_x_lp();
    /* x 。900 rpm 附近中心。0.31，尖峰可。0.45。.53；混速勿。0.36 温和门。*/
    x_kill_hi = (spd_blend != 0u) ? M1_HFI_HAND_SPD_X_KILL_HI : 0.55f;
    kill = (uint8_t)((s_hand_ok == 0u) || (ad > HFI_HAND_ANG_KILL) ||
                     (ae > 0.45f) || (x < 0.18f) || (x > x_kill_hi) ||
                     (dw > 150.0f));
    if (spd_blend != 0u) {
        mild = (uint8_t)((ad > HFI_HAND_ANG_ABORT) || (ae > 0.25f) ||
                         (dw > 80.0f));
    } else {
        mild = (uint8_t)((ad > HFI_HAND_ANG_ABORT) || (ae > 0.25f) ||
                         (x < 0.22f) || (x > 0.36f) || (dw > 80.0f));
    }
    if (kill != 0u) {
        hfi_hand_abort();
        return 1u;
    }
    if (mild != 0u) {
        if (s_hand_bad < 65535u) {
            s_hand_bad++;
        }
        if (s_hand_bad >= HFI_HAND_BAD_N) {
            hfi_hand_abort();
            return 1u;
        }
    } else {
        s_hand_bad = 0u;
    }
    return 0u;
}

/**
 * @brief ε 停写。θ。。20 ms 平均转速走，不用带 Kp·ε 的瞬。ω。
 */
void hfi_hand_follow_smo(void)
{
    hfi_sqwave_set_hat_hold(1u);
    hfi_sqwave_set_hat_coast_el(hfi_hand_w_el());
    hfi_sqwave_set_iq_auth_hold(1u);
}

/**
 * @brief 20 kHz 交接。先交速度，注入关掉之后再交角度。
 * @param w_hfi 上一。HFI 全转。[rpm]，尚未混。SMO。
 */
void hfi_smo_hand_step(float w_hfi)
{
    float aw;
    float dth;
    float ad;
    float dw;
    float w_use;
    uint8_t gate;

    if (hfi_sqwave_speed_run_active() == 0u) {
        hfi_smo_hand_idle();
        return;
    }

    dth = motor_wrap_pi(s_hand_th - hfi_sqwave_get_theta_hat());
    ad = motor_absf(dth);
    dw = motor_absf(w_hfi - s_hand_w);
    gate = (uint8_t)((s_hand_ok != 0u) && (ad < HFI_HAND_ANG_OK) && (dw < 50.0f));
    w_use = (s_hand_alpha > 0.5f) ? s_hand_w : w_hfi;
    aw = motor_absf(w_use);
    if (aw < 800.0f) {
        s_hand_armed = 1u;
    }

    switch (s_hand_state) {
    case HFI_HAND_HFI:
        s_hand_alpha = 0.0f;
        s_hand_ang = 0.0f;
        s_hand_vh = 1.0f;
        s_hand_n = 0u;
        s_hand_bad = 0u;
        hfi_sqwave_set_hat_hold(0u);
        if ((s_hand_armed != 0u) && (aw >= 900.0f) && (aw < 1100.0f) &&
            (gate != 0u)) {
            s_hand_state = HFI_HAND_QUAL;
        }
        break;
    case HFI_HAND_QUAL:
        s_hand_alpha = 0.0f;
        s_hand_ang = 0.0f;
        s_hand_vh = 1.0f;
        if ((s_hand_ok == 0u) || (ad > HFI_HAND_ANG_KILL) || (dw > 150.0f) ||
            (aw < 880.0f) || (aw >= 1100.0f)) {
            s_hand_state = HFI_HAND_HFI;
            s_hand_n = 0u;
            if (aw >= 1100.0f) {
                s_hand_armed = 0u;
            }
        } else if (gate != 0u) {
            s_hand_n++;
            if (s_hand_n >= HFI_HAND_QUAL_N) {
                s_hand_state = HFI_HAND_SPD;
                s_hand_n = 0u;
                s_hand_bad = 0u;
                s_hand_armed = 0u;
            }
        }
        break;
    case HFI_HAND_SPD:
        /* 只混速度。Park 仍是 θ̂，注入满幅。*/
        s_hand_vh = 1.0f;
        s_hand_ang = 0.0f;
        if (hfi_hand_watch(ad, dw, 1u) != 0u) {
            break;
        }
        s_hand_n++;
        s_hand_alpha = (float)s_hand_n / (float)HFI_HAND_BLEND_N;
        if (s_hand_alpha >= 1.0f) {
            s_hand_alpha = 1.0f;
            s_hand_state = HFI_HAND_CONF;
            s_hand_n = 0u;
            s_hand_bad = 0u;
        }
        break;
    case HFI_HAND_CONF:
        /* 速度已是 SMO。角差稳住约 0.4 s 再收注入。单拍超。25° 不清计时。*/
        s_hand_alpha = 1.0f;
        s_hand_ang = 0.0f;
        s_hand_vh = 1.0f;
        hfi_sqwave_set_hat_hold(0u);
        hfi_sqwave_set_iq_auth_hold(0u);
        if ((s_hand_ok == 0u) || (ad > HFI_HAND_ANG_KILL) || (dw > 150.0f)) {
            s_hand_n = 0u;
        } else if (gate != 0u) {
            s_hand_n++;
            if (s_hand_n >= HFI_HAND_QUAL_N) {
                s_hand_state = HFI_HAND_FADE;
                s_hand_n = 0u;
            }
        }
        break;
    case HFI_HAND_FADE:
        /* Park 仍是 θ̂。第一拍记下平滑角差，之后 θ̂ 。20 ms 转速走。*/
        s_hand_alpha = 1.0f;
        s_hand_ang = 0.0f;
        if (s_hand_n == 0u) {
            float th_s = 0.0f;

            if (s_pll_ref != 0) {
                th_s = emf_pll_theta_smooth(s_pll_ref, hfi_hand_w_el());
            }
            s_hand_dth = motor_wrap_pi(th_s - hfi_sqwave_get_theta_hat());
        }
        hfi_hand_follow_smo();
        s_hand_n++;
        {
            float a = (float)s_hand_n / (float)M1_HFI_HAND_FADE_N;

#if M1_HFI_HAND_VH0_SOFT_ENABLE
            a = hfi_hand_smoothstep(a); /* 57：两端软，减。1→地板「台阶感。*/
#endif
            if (a > 1.0f) {
                a = 1.0f;
            }
            s_hand_vh = 1.0f - (1.0f - M1_HFI_HAND_VH_FLOOR) * a;
        }
        if (s_hand_n >= M1_HFI_HAND_FADE_N) {
            s_hand_vh = M1_HFI_HAND_VH_FLOOR;
            s_hand_state = HFI_HAND_ANG;
            s_hand_n = 0u;
        }
        break;
    case HFI_HAND_ANG:
        /* 残注入开着。 s 。Park 。θ̂ 转到平滑 SMO。*/
        s_hand_alpha = 1.0f;
        s_hand_vh = M1_HFI_HAND_VH_FLOOR;
        hfi_hand_follow_smo();
        s_hand_n++;
        s_hand_ang = (float)s_hand_n / (float)HFI_HAND_ANG_N;
        if (s_hand_ang >= 1.0f) {
            s_hand_ang = 1.0f;
#if M1_HFI_HAND_ID_OVERLAP_ENABLE
            s_hand_state = HFI_HAND_HOLD;
#elif M1_HFI_HAND_KILL_VH_ENABLE
            /* 59/60：先。Vh→END。0 再在微地板上开 Id */
            s_hand_state = HFI_HAND_VH0;
#elif M1_HFI_HAND_OPEN_ID_ENABLE
            s_hand_state = HFI_HAND_HOLD; /* 58：残地板上开 Id */
#else
            s_hand_state = HFI_HAND_SMO; /* 55 */
#endif
            s_hand_n = 0u;
        }
        break;
#if M1_HFI_HAND_ID_OVERLAP_ENABLE || M1_HFI_HAND_OPEN_ID_ENABLE
    case HFI_HAND_HOLD:
        /* ang=1 已切满。专消化 Park 台阶。724：同拍叠爆）。*/
        s_hand_alpha = 1.0f;
        s_hand_ang = 1.0f;
        s_hand_vh = M1_HFI_HAND_VH_FLOOR;
        hfi_hand_follow_smo();
#if M1_HFI_HAND_STOP_AFTER == 2
        if (s_hand_n >= M1_HFI_HAND_HOLD_N) {
            break;
        }
#endif
        s_hand_n++;
        if (s_hand_n >= M1_HFI_HAND_HOLD_N) {
#if M1_HFI_HAND_STOP_AFTER == 2
            /* stay in HOLD */
#elif M1_HFI_HAND_OPEN_ID_ENABLE && !M1_HFI_HAND_KILL_VH_ENABLE
            s_hand_state = HFI_HAND_IDUP; /* 58：残地板开 Id */
            s_hand_n = 0u;
#else
            s_hand_state = HFI_HAND_VH0; /* OVERLAP：先。Vh */
            s_hand_n = 0u;
#endif
        }
        break;
#endif
#if M1_HFI_HAND_ID_OVERLAP_ENABLE || M1_HFI_HAND_KILL_VH_ENABLE
    case HFI_HAND_VH0:
        /* FLOOR→END。本。Id 旁路；OPEN_ID 。VH0 之后再开。*/
        s_hand_alpha = 1.0f;
        s_hand_ang = 1.0f;
        hfi_hand_follow_smo();
#if M1_HFI_HAND_ID_OVERLAP_ENABLE && (M1_HFI_HAND_STOP_AFTER == 1)
        if (s_hand_n >= M1_HFI_HAND_VH0_N) {
            s_hand_vh = M1_HFI_HAND_VH_END;
            break;
        }
#endif
        s_hand_n++;
        {
            float a = (float)s_hand_n / (float)M1_HFI_HAND_VH0_N;

            if (a > 1.0f) {
                a = 1.0f;
            }
#if M1_HFI_HAND_VH0_SOFT_ENABLE
            a = hfi_hand_smoothstep(a);
#endif
            s_hand_vh = M1_HFI_HAND_VH_FLOOR +
                        (M1_HFI_HAND_VH_END - M1_HFI_HAND_VH_FLOOR) * a;
        }
        if (s_hand_n >= M1_HFI_HAND_VH0_N) {
            s_hand_vh = M1_HFI_HAND_VH_END;
#if M1_HFI_HAND_ID_OVERLAP_ENABLE
#if M1_HFI_HAND_STOP_AFTER == 1
            /* stay VH0 */
#else
            s_hand_state = HFI_HAND_IDUP;
            s_hand_n = 0u;
#endif
#elif M1_HFI_HAND_OPEN_ID_ENABLE
            s_hand_state = HFI_HAND_IDUP; /* 60：微地板后再开 Id */
            s_hand_n = 0u;
#else
            s_hand_state = HFI_HAND_SMO; /* 59：只收到 END */
            s_hand_n = 0u;
#endif
        }
        break;
#endif
#if M1_HFI_HAND_ID_OVERLAP_ENABLE || M1_HFI_HAND_OPEN_ID_ENABLE
    case HFI_HAND_IDUP:
        /* 。Id。0：钉 VH_END。8：钉 FLOOR。*/
        s_hand_alpha = 1.0f;
        s_hand_ang = 1.0f;
#if M1_HFI_HAND_KILL_VH_ENABLE
        s_hand_vh = M1_HFI_HAND_VH_END;
#else
        s_hand_vh = M1_HFI_HAND_VH_FLOOR;
#endif
        hfi_hand_follow_smo();
        s_hand_n++;
        if (s_hand_n >= M1_HFI_HAND_IDUP_N) {
            s_hand_state = HFI_HAND_SMO;
            s_hand_n = 0u;
        }
        break;
#endif
    case HFI_HAND_SMO:
        s_hand_alpha = 1.0f;
        s_hand_ang = 1.0f;
#if M1_HFI_HAND_KILL_VH_ENABLE
        s_hand_vh = M1_HFI_HAND_VH_END;
#elif M1_HFI_HAND_ID_OVERLAP_ENABLE
        s_hand_vh = M1_HFI_HAND_VH_END;
#else
        s_hand_vh = M1_HFI_HAND_VH_FLOOR;
#endif
        hfi_hand_follow_smo();
#if M1_HFI_HAND_REV_ENABLE
        /* 先爬。ARM，再减速过 REV 线才。HFI（省时：不必守满高速）。*/
        if (aw >= M1_HFI_HAND_REV_ARM_RPM) {
            s_hand_rev_arm = 1u;
        }
        if ((s_hand_rev_arm != 0u) && (aw <= M1_HFI_HAND_REV_RPM) &&
            (dbg.outer_omega_ref <= (M1_HFI_HAND_REV_RPM + 20.0f))) {
            s_hand_state = HFI_HAND_RVH;
            s_hand_n = 0u;
            s_hand_bad = 0u;
            s_hand_rev_arm = 0u;
#if M1_HFI_HAND_REV_WAKE_ENABLE
            s_hand_rev_seeded = 0u;
            s_hand_rev_wait = 0u;
#endif
#if M1_HFI_HAND_REV_VH_MIRROR_ENABLE
            s_hand_rev_vh_phase = 0u;
#endif
        }
#endif
        break;
#if M1_HFI_HAND_REV_ENABLE
    case HFI_HAND_RVH:
        /* vh：默认 END→WAKE；MIRROR=1 时按前向 VH0+FADE 反演。Park/α 仍 SMO。 */
        s_hand_alpha = 1.0f;
        s_hand_ang = 1.0f;
#if M1_HFI_HAND_REV_WAKE_ENABLE
#if M1_HFI_HAND_REV_RVH_HOLD_ENABLE
        if ((s_hand_rev_seeded == 0u) ||
            ((s_hand_n > 0u) &&
             ((s_hand_n % M1_HFI_HAND_REV_RESEED_N) == 0u))) {
            hfi_sqwave_seed_hat(s_hand_th, hfi_hand_w_el());
            s_hand_rev_seeded = 1u;
        }
        hfi_hand_follow_smo();
#else
        if (s_hand_rev_seeded == 0u) {
            hfi_sqwave_seed_hat(s_hand_th, hfi_hand_w_el());
            s_hand_rev_seeded = 1u;
        }
        hfi_sqwave_set_hat_hold(0u);
        hfi_sqwave_set_iq_auth_hold(1u);
#endif
#else
        hfi_hand_follow_smo();
#endif
        s_hand_n++;
#if M1_HFI_HAND_REV_VH_MIRROR_ENABLE
        /* 反演：先 VH0 逆（END→FLOOR），再 FADE 逆（FLOOR→WAKE），时长/smoothstep 同前向。 */
        {
            float a;
            float vh_wake = M1_HFI_HAND_REV_VH_WAKE;
            uint8_t done = 0u;

            if (vh_wake > 1.0f) {
                vh_wake = 1.0f;
            }
            if (s_hand_rev_vh_phase == 0u) {
                a = (float)s_hand_n / (float)M1_HFI_HAND_VH0_N;
                if (a > 1.0f) {
                    a = 1.0f;
                }
#if M1_HFI_HAND_VH0_SOFT_ENABLE
                a = hfi_hand_smoothstep(a);
#endif
                /* 前向 VH0：FLOOR+(END-FLOOR)*a → 反演 END+(FLOOR-END)*a */
                s_hand_vh = M1_HFI_HAND_VH_END +
                            (M1_HFI_HAND_VH_FLOOR - M1_HFI_HAND_VH_END) * a;
                if (s_hand_n >= M1_HFI_HAND_VH0_N) {
                    s_hand_vh = M1_HFI_HAND_VH_FLOOR;
                    if (vh_wake <= (M1_HFI_HAND_VH_FLOOR + 1.0e-4f)) {
                        done = 1u;
                    } else {
                        s_hand_rev_vh_phase = 1u;
                        s_hand_n = 0u;
                    }
                }
            } else {
                float span_full = 1.0f - M1_HFI_HAND_VH_FLOOR;
                float span_need = vh_wake - M1_HFI_HAND_VH_FLOOR;
                uint32_t n_fade = M1_HFI_HAND_FADE_N;

                if (span_full < 1.0e-4f) {
                    span_full = 1.0e-4f;
                }
                if (span_need < 0.0f) {
                    span_need = 0.0f;
                }
                /* 同斜率：只走 FADE 中 FLOOR→WAKE 这一段 */
                n_fade = (uint32_t)((float)M1_HFI_HAND_FADE_N * (span_need / span_full) + 0.5f);
                if (n_fade < 1u) {
                    n_fade = 1u;
                }
                a = (float)s_hand_n / (float)n_fade;
                if (a > 1.0f) {
                    a = 1.0f;
                }
#if M1_HFI_HAND_VH0_SOFT_ENABLE
                a = hfi_hand_smoothstep(a);
#endif
                /* 前向 FADE：1-(1-FLOOR)*a → 反演 FLOOR+(1-FLOOR)*a；截到 WAKE */
                s_hand_vh = M1_HFI_HAND_VH_FLOOR + span_need * a;
                if (s_hand_n >= n_fade) {
                    s_hand_vh = vh_wake;
                    done = 1u;
                }
            }
            if (done != 0u) {
#if M1_HFI_HAND_REV_WAKE_ENABLE
#if M1_HFI_HAND_REV_RVH_HOLD_ENABLE
                hfi_sqwave_seed_hat(s_hand_th, hfi_hand_w_el());
#endif
#if M1_HFI_HAND_REV_OBS_ENABLE
                s_hand_state = HFI_HAND_ROBS;
                s_hand_n = 0u;
                s_hand_bad = 0u;
#else
                s_hand_state = HFI_HAND_RQUAL;
                s_hand_n = 0u;
                s_hand_rev_wait = 0u;
                s_hand_bad = 0u;
#endif
#else
                s_hand_dth = motor_wrap_pi(s_hand_th - hfi_sqwave_get_theta_hat());
                s_hand_state = HFI_HAND_RANG;
                s_hand_n = 0u;
#endif
            }
        }
#else
        {
            float a = (float)s_hand_n / (float)M1_HFI_HAND_RVH_N;
            float vh_wake = M1_HFI_HAND_REV_VH_WAKE;

            if (a > 1.0f) {
                a = 1.0f;
            }
#if M1_HFI_HAND_VH0_SOFT_ENABLE
            a = hfi_hand_smoothstep(a);
#endif
            if (vh_wake > 1.0f) {
                vh_wake = 1.0f;
            }
            s_hand_vh = M1_HFI_HAND_VH_END +
                        (vh_wake - M1_HFI_HAND_VH_END) * a;
        }
        if (s_hand_n >= M1_HFI_HAND_RVH_N) {
            s_hand_vh = M1_HFI_HAND_REV_VH_WAKE;
            if (s_hand_vh > 1.0f) {
                s_hand_vh = 1.0f;
            }
#if M1_HFI_HAND_REV_WAKE_ENABLE
#if M1_HFI_HAND_REV_RVH_HOLD_ENABLE
            hfi_sqwave_seed_hat(s_hand_th, hfi_hand_w_el());
#endif
#if M1_HFI_HAND_REV_OBS_ENABLE
            s_hand_state = HFI_HAND_ROBS;
            s_hand_n = 0u;
            s_hand_bad = 0u;
#else
            s_hand_state = HFI_HAND_RQUAL;
            s_hand_n = 0u;
            s_hand_rev_wait = 0u;
            s_hand_bad = 0u;
#endif
#else
            s_hand_dth = motor_wrap_pi(s_hand_th - hfi_sqwave_get_theta_hat());
            s_hand_state = HFI_HAND_RANG;
            s_hand_n = 0u;
#endif
        }
#endif
        break;
#if M1_HFI_HAND_REV_OBS_ENABLE
    case HFI_HAND_ROBS:
        /* Park=SMO、α=SMO、vh=0.4；θ̂ 自由估，永不交角/交速。 */
        s_hand_alpha = 1.0f;
        s_hand_ang = 1.0f;
        s_hand_vh = M1_HFI_HAND_REV_VH_WAKE;
        if (s_hand_vh > 1.0f) {
            s_hand_vh = 1.0f;
        }
        hfi_sqwave_set_hat_hold(0u);
        hfi_sqwave_set_iq_auth_hold(1u);
        s_hand_n++;
        break;
#endif
#if M1_HFI_HAND_REV_WAKE_ENABLE
    case HFI_HAND_RQUAL:
        /* Park=SMO、vh=WAKE、θ。自由跟。门过再交角；超时退。SMO。*/
        s_hand_alpha = 1.0f;
        s_hand_ang = 1.0f;
        s_hand_vh = M1_HFI_HAND_REV_VH_WAKE;
        if (s_hand_vh > 1.0f) {
            s_hand_vh = 1.0f;
        }
        hfi_sqwave_set_hat_hold(0u);
        hfi_sqwave_set_iq_auth_hold(1u);
        {
            float x = hfi_sqwave_get_x_lp();
            float ae = hfi_sqwave_get_eps();
            uint8_t gate_ok;

            if (ae < 0.0f) {
                ae = -ae;
            }
            /* 。解调门；勿用 dw。506）。9：去。x 上界。515 误杀）。*/
            gate_ok = (uint8_t)((ad < HFI_HAND_ANG_OK) && (ae < 0.35f) &&
                                (x > 0.16f));
#if M1_HFI_HAND_RQUAL_X_MAX_ENABLE
            if (x >= M1_HFI_HAND_RQUAL_X_MAX) {
                gate_ok = 0u;
            }
#endif
            if (gate_ok != 0u) {
                s_hand_n++;
                s_hand_bad = 0u;
            } else {
                s_hand_n = 0u;
                if (s_hand_bad < 65535u) {
                    s_hand_bad++;
                }
            }
            if (s_hand_rev_wait < 65535u) {
                s_hand_rev_wait++;
            }
            if (s_hand_n >= M1_HFI_HAND_RQUAL_N) {
                s_hand_dth = motor_wrap_pi(s_hand_th - hfi_sqwave_get_theta_hat());
                s_hand_state = HFI_HAND_RANG;
                s_hand_n = 0u;
                s_hand_bad = 0u;
                s_hand_rev_wait = 0u;
            } else if (s_hand_rev_wait >= M1_HFI_HAND_RQUAL_TIMEOUT_N) {
                /* 锁不上：退。SMO 微地板，勿硬。HFI */
                s_hand_state = HFI_HAND_SMO;
                s_hand_vh = M1_HFI_HAND_VH_END;
                s_hand_n = 0u;
                s_hand_rev_seeded = 0u;
                s_hand_rev_wait = 0u;
                hfi_hand_follow_smo();
            }
        }
        break;
#endif
    case HFI_HAND_RANG:
        /* ang 1。。WAKE：θ。继续跟，vh 保持 WAKE。*/
        s_hand_alpha = 1.0f;
#if M1_HFI_HAND_REV_WAKE_ENABLE
        s_hand_vh = M1_HFI_HAND_REV_VH_WAKE;
        if (s_hand_vh > 1.0f) {
            s_hand_vh = 1.0f;
        }
        hfi_sqwave_set_hat_hold(0u);
        hfi_sqwave_set_iq_auth_hold(1u);
#else
        s_hand_vh = M1_HFI_HAND_VH_FLOOR;
        hfi_hand_follow_smo();
#endif
        s_hand_n++;
        {
            float a = (float)s_hand_n / (float)M1_HFI_HAND_RANG_N;

            if (a > 1.0f) {
                a = 1.0f;
            }
            s_hand_ang = 1.0f - a;
        }
        if (s_hand_n >= M1_HFI_HAND_RANG_N) {
            s_hand_ang = 0.0f;
#if !M1_HFI_HAND_REV_WAKE_ENABLE
            hfi_sqwave_seed_hat(s_hand_th, hfi_hand_w_el());
#endif
            hfi_sqwave_set_iq_auth_hold(0u);
            s_hand_state = HFI_HAND_RFADE;
            s_hand_n = 0u;
        }
        break;
    case HFI_HAND_RFADE:
        /* Park=θ̂，vh WAKE。（已满则。1）。*/
        s_hand_alpha = 1.0f;
        s_hand_ang = 0.0f;
        hfi_sqwave_set_hat_hold(0u);
        hfi_sqwave_set_iq_auth_hold(0u);
        s_hand_n++;
        {
            float a = (float)s_hand_n / (float)M1_HFI_HAND_RFADE_N;
            float vh0 = M1_HFI_HAND_REV_VH_WAKE;

            if (a > 1.0f) {
                a = 1.0f;
            }
#if M1_HFI_HAND_VH0_SOFT_ENABLE
            a = hfi_hand_smoothstep(a);
#endif
            if (vh0 > 1.0f) {
                vh0 = 1.0f;
            }
#if !M1_HFI_HAND_REV_WAKE_ENABLE
            vh0 = M1_HFI_HAND_VH_FLOOR;
#endif
            s_hand_vh = vh0 + (1.0f - vh0) * a;
        }
        if (s_hand_n >= M1_HFI_HAND_RFADE_N) {
            s_hand_vh = 1.0f;
            s_hand_state = HFI_HAND_RSPD;
            s_hand_n = 0u;
            s_hand_bad = 0u;
        }
        break;
    case HFI_HAND_RSPD:
        /* 对称 SPD 反向：满注入，alpha 1。（速度 SMO→HFI）。*/
        s_hand_vh = 1.0f;
        s_hand_ang = 0.0f;
        hfi_sqwave_set_hat_hold(0u);
        hfi_sqwave_set_iq_auth_hold(0u);
        if (hfi_hand_watch(ad, dw, 1u) != 0u) {
            /* 退回失败：abort 已把状态打。HFI */
            break;
        }
        s_hand_n++;
        {
            float a = (float)s_hand_n / (float)M1_HFI_HAND_RSPD_N;

            if (a > 1.0f) {
                a = 1.0f;
            }
            s_hand_alpha = 1.0f - a;
        }
        if (s_hand_n >= M1_HFI_HAND_RSPD_N) {
            s_hand_alpha = 0.0f;
            s_hand_state = HFI_HAND_HFI;
            s_hand_n = 0u;
            s_hand_armed = 0u; /* 。aw<800 才再允许前向 */
            s_hand_rev_arm = 0u;
        }
        break;
#endif
    default:
        s_hand_state = HFI_HAND_HFI;
        s_hand_alpha = 0.0f;
        s_hand_ang = 0.0f;
        s_hand_vh = 1.0f;
        break;
    }

    hfi_sqwave_set_inj_scale(s_hand_vh);
#if M1_HFI_HAND_ID_OVERLAP_ENABLE
    if (s_hand_state == HFI_HAND_ANG) {
        hfi_sqwave_set_id_pi_release(1u);
        hfi_sqwave_set_id_pi_soft_cmd(M1_HFI_HAND_ID_WEAK * s_hand_ang);
    } else if ((s_hand_state == HFI_HAND_HOLD) ||
               (s_hand_state == HFI_HAND_VH0)) {
        hfi_sqwave_set_id_pi_release(1u);
        hfi_sqwave_set_id_pi_soft_cmd(M1_HFI_HAND_ID_WEAK);
    } else if (s_hand_state == HFI_HAND_IDUP) {
        float a = (float)s_hand_n / (float)M1_HFI_HAND_IDUP_N;

        if (a > 1.0f) {
            a = 1.0f;
        }
        hfi_sqwave_set_id_pi_release(1u);
        hfi_sqwave_set_id_pi_soft_cmd(
            M1_HFI_HAND_ID_WEAK + (1.0f - M1_HFI_HAND_ID_WEAK) * a);
    } else if (s_hand_state == HFI_HAND_SMO) {
        hfi_sqwave_set_id_pi_release(1u);
        hfi_sqwave_set_id_pi_soft_cmd(1.0f);
    } else {
        hfi_sqwave_set_id_pi_release(0u);
        hfi_sqwave_set_id_pi_soft_cmd(-1.0f);
    }
#elif M1_HFI_HAND_OPEN_ID_ENABLE
    /* 60：VH0 旁路；IDUP soft；SMO→1。勿在 VH0 放行 Id */
    if (s_hand_state == HFI_HAND_IDUP) {
        float a = (float)s_hand_n / (float)M1_HFI_HAND_IDUP_N;

        if (a > 1.0f) {
            a = 1.0f;
        }
        a = hfi_hand_smoothstep(a);
        hfi_sqwave_set_id_pi_release(1u);
        hfi_sqwave_set_id_pi_soft_cmd(a);
    } else if (s_hand_state == HFI_HAND_SMO) {
        hfi_sqwave_set_id_pi_release(1u);
        hfi_sqwave_set_id_pi_soft_cmd(1.0f);
    } else {
        hfi_sqwave_set_id_pi_release(0u);
        hfi_sqwave_set_id_pi_soft_cmd(-1.0f);
    }
#else
#if M1_HFI_ID_ON_FROM_RUN_ENABLE || !M1_HFI_ID_PI_OFF_ENABLE
    /* 文档§7 步1：Id 环已开。KILL_VH 交接不得每拍再旁路。 */
    hfi_sqwave_set_id_pi_release(1u);
    hfi_sqwave_set_id_pi_soft_cmd(1.0f);
#else
    hfi_sqwave_set_id_pi_release(0u);
    hfi_sqwave_set_id_pi_soft_cmd(-1.0f);
#endif
#endif
#if (M1_HFI_GATE == 79) || (M1_HFI_GATE == 80)
    /* 窗：hand 内再刷（同拍贴转速）。FADE/ANG/VH0 不改。 */
    {
        float w = dbg.pll_omega_mech_rpm;

        if (w < 0.0f) {
            w = -w;
        }
        if (w < 20.0f) {
            w = s_hand_w;
            if (w < 0.0f) {
                w = -w;
            }
        }
        if (w < 20.0f) {
            w = dbg.hfi_omega_rpm;
            if (w < 0.0f) {
                w = -w;
            }
        }
        hfi_vesc_win_obs_update(w);
    }
#if M1_HFI_GATE == 79
    /* S2：want=1 且 SMO → 硬关残 Vh，Id 仍旁路（已证实会抖，仅对照）。 */
    if ((hfi_vesc_win_smo() != 0u) && (s_hand_ok != 0u) &&
        (s_hand_state == HFI_HAND_SMO)) {
        s_hand_vh = 0.0f;
        hfi_sqwave_set_inj_scale(0.0f);
        hfi_sqwave_set_id_pi_release(0u);
        hfi_sqwave_set_id_pi_soft_cmd(-1.0f);
    }
#elif M1_HFI_GATE == 80
    /* S2b：want=1 且 SMO → VH_END→0 软收，同时放行 Id PI（soft 0→1，id* 无扰→0）。 */
    if ((hfi_vesc_win_smo() != 0u) && (s_hand_ok != 0u) &&
        (s_hand_state == HFI_HAND_SMO)) {
        float a;

        if (hfi_vesc_ho_active() == 0u) {
            hfi_vesc_ho_set(1u, 0u);
        }
        if (hfi_vesc_ho_n() < M1_HFI_VESC_HANDOFF_N) {
            hfi_vesc_ho_set(1u, hfi_vesc_ho_n() + 1u);
        }
        a = (float)hfi_vesc_ho_n() / (float)M1_HFI_VESC_HANDOFF_N;
        if (a > 1.0f) {
            a = 1.0f;
        }
        a = hfi_hand_smoothstep(a);
        s_hand_vh = M1_HFI_HAND_VH_END * (1.0f - a);
        hfi_sqwave_set_inj_scale(s_hand_vh);
        hfi_sqwave_set_id_pi_release(1u);
        hfi_sqwave_set_id_pi_soft_cmd(a);
    } else if (hfi_vesc_ho_active() != 0u) {
        /* 出窗或离开 SMO：退回微地板 + Id 旁路（由上方默认分支已写 release=0） */
        hfi_vesc_ho_set(0u, 0u);
    }
#endif
#endif
    /* ch10: stage+hand；HFI→速度交→角度交（HOLD/VH0/IDUP） */
    dbg.obs_ss_alpha = 0.5f * s_hand_alpha + 0.5f * s_hand_ang;
    dbg.obs_ss_state = (float)s_hand_state;
}

#if M1_HFI_HAND_IQ_HOLD_ON_IDUP
/**
 * @brief IDUP 期间钉住 iq_ref，避免开 Id 。ω 毛刺让速度环下刹车。
 * @note 须在 outer_tick / iq_auth 之后调用。VH0 段持续采。hold。
 */
void hfi_hand_iq_hold_apply(motor_context_t *ctx)
{
    if ((ctx == NULL) || (s_hand_ok == 0u)) {
        return;
    }
    if (s_hand_state == HFI_HAND_IDUP) {
        ctx->iq_ref = s_hand_iq_hold;
        ctx->pi_speed.integrator = s_hand_iq_hold;
        dbg.outer_iq_ref = s_hand_iq_hold;
        dbg.foc_iq_ref = s_hand_iq_hold;
    } else if ((s_hand_state == HFI_HAND_VH0) ||
               (s_hand_state == HFI_HAND_ANG) ||
               (s_hand_state == HFI_HAND_HOLD)) {
        s_hand_iq_hold = ctx->iq_ref;
    }
}
#endif

#if M1_HFI_HAND_DECEL_BRAKE_ENABLE
/**
 * @brief SMO 减速段制动向 Iq 地板（产品形护角）。
 * @note 开窗：已爬高后，ω* 明显低于 SMO ω（减速意图）。守速不开。
 *       关窗：SMO ω 落到 END。速度环仍算；仅当 iq* 制动不足时抬到 -sign(ω)·floor。
 *       禁止按 |Iq| 保巡航正号（74 在开窗瞬间把 +0.5 抬到 +1.5 导致加速失步）。
 *       同步积分器到制动 iq，避免下一拍 PI 立刻顶回去。须在 iq_auth 后调用。
 */
void hfi_hand_decel_brake_apply(motor_context_t *ctx)
{
    float wref;
    float iq;
    float fl;
    float wfb;

    if ((ctx == NULL) || (s_hand_ok == 0u)) {
        s_hand_brake_arm = 0u;
        s_hand_brake_on = 0u;
        return;
    }
    if (s_hand_state != HFI_HAND_SMO) {
        s_hand_brake_arm = 0u;
        s_hand_brake_on = 0u;
        s_hand_wref_prev = ctx->omega_ref;
        return;
    }

    wref = ctx->omega_ref;
    wfb = s_hand_w;
    if (wfb >= M1_HFI_HAND_DECEL_BRAKE_ARM_RPM) {
        s_hand_brake_arm = 1u;
    }

    if (s_hand_brake_on == 0u) {
        if ((s_hand_brake_arm != 0u) &&
            (wref < (wfb - M1_HFI_HAND_DECEL_BRAKE_DROP_RPM))) {
            s_hand_brake_on = 1u;
        }
    } else if (wfb <= M1_HFI_HAND_DECEL_BRAKE_END_RPM) {
        s_hand_brake_on = 0u;
        s_hand_brake_arm = 0u;
    }

    if (s_hand_brake_on != 0u) {
        fl = M1_HFI_HAND_DECEL_BRAKE_IQ_A;
        if (fl < 0.0f) {
            fl = -fl;
        }
        iq = ctx->iq_ref;
        /* ω>0：制动 = 负 Iq；ω<0：制动 = 正 Iq。已更负/更正则不改。 */
        if (wfb >= 0.0f) {
            if (iq > (-fl)) {
                iq = -fl;
            }
        } else if (iq < fl) {
            iq = fl;
        }
        ctx->iq_ref = iq;
        ctx->pi_speed.integrator = iq;
        dbg.outer_iq_ref = iq;
        dbg.foc_iq_ref = iq;
    }

    s_hand_wref_prev = wref;
}
#endif

void hfi_hand_bind_pll(void *pll)
{
    s_pll_ref = (emf_pll_t *)pll;
}

void hfi_hand_clear_ok(void)
{
    s_hand_ok = 0u;
}

void hfi_hand_note_smo(float theta, float w_rpm)
{
    s_hand_th = theta;
    s_hand_w = w_rpm;
    s_hand_ok = 1u;
}

float hfi_hand_blend_speed_fb(float w_hfi)
{
    float w = w_hfi;

    hfi_smo_hand_step(w);
    if ((s_hand_alpha > 0.0f) && (s_hand_ok != 0u)) {
        w = (1.0f - s_hand_alpha) * w + s_hand_alpha * s_hand_w;
#if M1_HFI_HAND_W_HOLD_ON_IDUP
        if (s_hand_state == HFI_HAND_IDUP) {
            w = s_hand_w_hold;
            s_hand_w_rel_n = 0u;
        } else if ((s_hand_state == HFI_HAND_VH0) ||
                   (s_hand_state == HFI_HAND_ANG) ||
                   (s_hand_state == HFI_HAND_HOLD)) {
            s_hand_w_hold = s_hand_w;
            s_hand_w_rel_n = 0u;
        } else if (s_hand_state == HFI_HAND_SMO) {
            if (s_hand_w_rel_n < M1_HFI_HAND_W_REL_N) {
                float a = (float)s_hand_w_rel_n /
                          (float)M1_HFI_HAND_W_REL_N;

                a = hfi_hand_smoothstep(a);
                w = (1.0f - a) * s_hand_w_hold + a * s_hand_w;
                s_hand_w_rel_n++;
            }
        }
#endif
#if M1_HFI_HAND_W_SLEW_ENABLE
        {
            uint8_t slew_act = 0u;

#if M1_HFI_HAND_W_SLEW_IDUP_ONLY
            if ((s_hand_state == HFI_HAND_VH0) ||
                (s_hand_state == HFI_HAND_ANG) ||
                (s_hand_state == HFI_HAND_HOLD)) {
                s_hand_w_slew = w;
                s_hand_w_slew_on = 1u;
                s_hand_w_slew_n = 0u;
            } else if (s_hand_state == HFI_HAND_IDUP) {
                slew_act = 1u;
                s_hand_w_slew_n = 0u;
            } else if (s_hand_state == HFI_HAND_SMO) {
                if (s_hand_w_slew_n < M1_HFI_HAND_W_SLEW_SMO_N) {
                    slew_act = 1u;
                    s_hand_w_slew_n++;
                } else {
                    s_hand_w_slew_on = 0u;
                }
            } else {
                s_hand_w_slew_on = 0u;
                s_hand_w_slew_n = 0u;
            }
#else
            if ((s_hand_state == HFI_HAND_ANG) ||
                (s_hand_state == HFI_HAND_HOLD) ||
                (s_hand_state == HFI_HAND_VH0) ||
                (s_hand_state == HFI_HAND_IDUP) ||
                (s_hand_state == HFI_HAND_SMO)) {
                slew_act = 1u;
            } else {
                s_hand_w_slew_on = 0u;
            }
#endif
            if (slew_act != 0u) {
                if (s_hand_w_slew_on == 0u) {
                    s_hand_w_slew = w;
                    s_hand_w_slew_on = 1u;
                } else {
                    const float step =
                        M1_HFI_HAND_W_SLEW_RPM_S * OBS_CTRL_TS_S;
                    float d = w - s_hand_w_slew;

                    if (d > step) {
                        s_hand_w_slew += step;
                    } else if (d < -step) {
                        s_hand_w_slew -= step;
                    } else {
                        s_hand_w_slew = w;
                    }
                }
                w = s_hand_w_slew;
            }
        }
#endif
    }
    return w;
}

#if M1_HFI_ROTATE_PI_ENABLE
void hfi_hand_rotate_if_due(motor_context_t *ctx, float theta_park,
                            float id, float iq)
{
    if (s_hand_ang < 1.0f) {
        s_rot_armed = 1u;
    }
    if ((s_rot_armed != 0u) && (s_hand_ang >= 1.0f) &&
        (s_rot_th_ok != 0u)) {
        hfi_hand_rotate_current_pi(ctx,
                                   motor_wrap_pi(theta_park - s_rot_th_prev),
                                   id, iq);
        s_rot_armed = 0u;
    }
    s_rot_th_prev = theta_park;
    s_rot_th_ok = 1u;
}
#endif
#endif
