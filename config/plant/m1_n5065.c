/**
 * @file m1_n5065.c
 * @date 2026-10-28
 * @brief N5065 电机铭牌表（8 份 profile 共用同一电机）。
 *
 * 数值抄自 motor_params_m1.h：Rs/Ld/Lq/pp/Vbus/死区；
 * Kt 抄 M1_MIT_KT_NM_A；ψ/J/B 现网无宏，填 0 且 valid=0。
 * 已进 MDK；各 r_*.c 通过 g_plant_m1_n5065 共用本表。
 */
#include "m1_n5065.h"

const motor_plant_t g_plant_m1_n5065 = {
    0.122f,       /* rs_ohm：M1_RS_OHM */
    59e-6f,       /* ld_h：M1_LD_H */
    87e-6f,       /* lq_h：M1_LQ_H */
    0.063f,       /* kt_nm_a：M1_MIT_KT_NM_A */
    0.0f,         /* psi_wb：现网无宏 */
    0.0f,         /* j_kgm2：现网无宏 */
    0.0f,         /* b_nms：现网无宏 */
    7u,           /* pole_pairs：M1_POLE_PAIRS */
    0u,           /* valid：手册/宏默认，非本轮实测签收 */
    24.0f,        /* vbus_v：M1_VBUS_V */
    5.91e-7f,     /* deadtime_s：M1_DEADTIME_NS=591 → 591e-9 */
};
