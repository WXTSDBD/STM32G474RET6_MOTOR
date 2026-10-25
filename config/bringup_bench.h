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

/** M1 角源：0=SPI1+AS5047，1=SPI3+KTH7823。 */
#ifndef M1_ENCODER_SRC_KTH7823_SPI3
#define M1_ENCODER_SRC_KTH7823_SPI3     1
#endif
#ifndef M1_ENCODER_SRC
#define M1_ENCODER_SRC                  M1_ENCODER_SRC_KTH7823_SPI3
#endif

/** 1=上电锁转子标 encoder add；0=用 M1_ENCODER_OFFSET_RAD。 */
#ifndef M1_RUN_ENCODER_CAL
/** 2358 Ud 标定已得 add≈3.1816；先冻，勿每电重标。 */
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
#if (M1_ENCODER_SRC == M1_ENCODER_SRC_KTH7823_SPI3)
/**
 * KTH7823 电角零偏（出厂值）。
 * 2026-10-25 用"纯有感恒速 2000rpm"实测 δ 反推（最干净：无 HFI/无观测器/无无感段）：
 *   在 offset=3.6586 下实测 δ=+9.92° ⇒ 正确值 = 3.6586 + 0.1732 = 3.8318 rad
 *   ⇒ 3.830 rad (219.4°)
 * 备注①：12 步 Id 锁轴探针给出 209.62°(3.6586)，被齿槽力带偏约 10°（3A×Kt≈0.09N·m 与齿槽同量级），
 *        不能直接用；锁轴法需要更大电流或反复正反平均。
 * 备注②：原值 3.181572 来源注释引用的 10-08 23:58 录波其实是一次 100rpm 拖动，不是 Ud 锁，来源不可信；
 *        实测它比正确值小约 37°。
 * 备注③：编码器存在 ±6° 电角的 1 次机械周期误差（偏心）。
 */
#define M1_ENCODER_OFFSET_RAD           3.830f
#else
#define M1_ENCODER_OFFSET_RAD           2.10f
#endif
#endif

/** 1=SIGNOFF 布局 ch11=enc_raw（录波给分析用）；0=ch11=mark。 */
#ifndef M1_VOFA_SIGNOFF_ENC_RAW
#if (M1_ENCODER_SRC == M1_ENCODER_SRC_KTH7823_SPI3)
#define M1_VOFA_SIGNOFF_ENC_RAW         1
#else
#define M1_VOFA_SIGNOFF_ENC_RAW         0
#endif
#endif

/**
 * 1=KTH raw 取负。2334 证明：开 DIR_REV 重标会把 add 打进坏区；
 * 且 2301 在 DIR_REV=0、add≈2.04 时已能出力疯转 → 默认关。
 */

/**
 * 机械角符号。2358：Ud 标定后 +Iq 时 raw 增、MECH_SIGN=-1 把 ω 翻成负 → 正反馈疯转。
 * 电角已对齐时保持 +1。
 */
#ifndef M1_ENCODER_MECH_SIGN
#define M1_ENCODER_MECH_SIGN            (1.0f)
#endif

/** 1=Park 用 -θ；与 MECH_SIGN 勿同时乱开。 */
#ifndef M1_THETA_NEGATE
#define M1_THETA_NEGATE                 0
#endif

/** 1=VOFA ch4 输出 SVPWM 扇区 1..6（替换 Id）。 */
#ifndef M1_VOFA_SECTOR_DIAG
#define M1_VOFA_SECTOR_DIAG             0
#endif

/** 1=VOFA ch0..2 输出 foc_ia/b/c(A)；0=adc_zeroed LSB。 */

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
