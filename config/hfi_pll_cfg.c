/**
 * @file hfi_pll_cfg.c
 * @date 2026-10-25
 * @brief HFI PLL 参数数据（原 `M1_HFI_PLL_*` 宏，宏→表 阶段 2 切片）。
 *
 * 改 PLL 参数只改这里，不再碰 profile / motor_params 的宏。
 *
 * 取值来源：改造前用工程自带 armclang 对 hfi_sqwave.c 做预处理得到的实际展开值
 * （profile m1_hfi_standstill + GATE=141 生效）：
 *   KP 末次覆盖为 1200.0f（profile L127 的 39.2f 被 L287 覆盖），KI=389.0f，
 *   W_MAX=200.0f，INT_MAX=W_MAX=200.0f，EPS_DEAD=0.0f，INIT_OFF_RAD=0.0f。
 */
#include "hfi_pll_cfg.h"

/** 与改造前宏展开值逐项一致。 */
const hfi_pll_cfg_t g_hfi_pll_cfg = {
    1200.0f, /* kp */
    389.0f,  /* ki */
    200.0f,  /* w_max */
    200.0f,  /* int_max */
    0.0f,    /* eps_dead */
    0.0f,    /* init_off_rad */
};
