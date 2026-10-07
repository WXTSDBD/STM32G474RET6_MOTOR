/**
 * @file bringup_bench.h
 * @date 2026-10-07
 * @brief 台架 / 上电标定开关（原 Core/Inc/main.h USER CODE EC）。
 *
 * 产品默认：不覆盖 NVM binding、不上电跑标定。
 * 台架临时改本文件；勿把默认再改回「静默忽略出厂标定」。
 */

#ifndef CONFIG_BRINGUP_BENCH_H
#define CONFIG_BRINGUP_BENCH_H

#ifdef __cplusplus
extern "C" {
#endif

/** 1=上电锁转子标 encoder add；0=用 M1_ENCODER_OFFSET_RAD。 */
#ifndef M1_RUN_ENCODER_CAL
#define M1_RUN_ENCODER_CAL              0
#endif

/** 1=上电跑三档脉冲诊断（不写 Flash，标定后 hold）；0=走 Flash binding。 */
#ifndef M1_RUN_PHASE_CAL
#define M1_RUN_PHASE_CAL                0
#endif

/** 1=从 Flash 加载 phase binding（M1_RUN_PHASE_CAL=0 时生效）。 */
#ifndef M1_APPLY_PHASE_BINDING
#define M1_APPLY_PHASE_BINDING          1
#endif

/**
 * 1=强制 2325 Test3 binding，忽略 Flash。
 * 0=只用 Flash / identity。
 *
 * ★ 本板 Flash 里仍可能是被 2325 推翻的旧 rank/sign；清零会相序错 → 上电过流。
 * 迁到 config/ 可以，但默认必须保持 1，直到 NVM 重写成 2325 真值后再改 0。
 */
#ifndef M1_BINDING_OVERRIDE_2325
#define M1_BINDING_OVERRIDE_2325        1
#endif

#ifndef M1_ENCODER_OFFSET_RAD
#define M1_ENCODER_OFFSET_RAD           2.10f
#endif

/** 1=Park/VOFA 用 -θ（测试 B）；闭环前须为 0。 */
#ifndef M1_THETA_NEGATE
#define M1_THETA_NEGATE                 0
#endif

/** 1=VOFA ch4 输出 SVPWM 扇区 1..6（替换 Id）。 */
#ifndef M1_VOFA_SECTOR_DIAG
#define M1_VOFA_SECTOR_DIAG             0
#endif

/** 1=VOFA ch0..2 输出 foc_ia/b/c(A)；0=adc_zeroed LSB。 */
#ifndef M1_VOFA_FOC_ABC
#define M1_VOFA_FOC_ABC                 1
#endif

#define PHASE_CAL_FAIL_NONE             0u
#define PHASE_CAL_FAIL_OC               1u
#define PHASE_CAL_FAIL_SNR              2u
#define PHASE_CAL_FAIL_PERM             3u
#define PHASE_CAL_FAIL_FLASH            4u
#define PHASE_CAL_FAIL_TIMEOUT          5u

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_BRINGUP_BENCH_H */
