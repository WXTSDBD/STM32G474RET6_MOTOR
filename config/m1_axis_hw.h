/**
 * @file m1_axis_hw.h
 * @date 2026-10-31
 * @note 自 bringup_bench.h 迁入；数值未改。
 * @brief 台架 / 上电标定开关（原 Core/Inc/main.h USER CODE EC）。
 *
 * 本板固定：SPI3+KTH7823；产品默认不上电重标、强制 2325 binding。
 * 台架临时改数值；勿再套 #ifndef（本头即唯一源）。
 */
#ifndef CONFIG_M1_AXIS_HW_H
#define CONFIG_M1_AXIS_HW_H

#ifdef __cplusplus
extern "C" {
#endif

/** M1 角源：0=SPI1+AS5047，1=SPI3+KTH7823。 */
#define M1_ENCODER_SRC_KTH7823_SPI3     1
#define M1_ENCODER_SRC                  M1_ENCODER_SRC_KTH7823_SPI3

/** 1=上电锁转子标 encoder add；0=用 M1_ENCODER_OFFSET_RAD。 */
#define M1_RUN_ENCODER_CAL              0

/** 1=上电跑三档脉冲诊断（不写 Flash）；0=走 Flash binding。 */
#define M1_RUN_PHASE_CAL                0

/** 1=从 Flash 加载 phase binding（M1_RUN_PHASE_CAL=0 时生效）。 */
#define M1_APPLY_PHASE_BINDING          1

/**
 * 1=强制 2325 Test3 binding，忽略 Flash。
 * 本板 Flash 可能仍是旧 rank/sign；清零会相序错。
 */
#define M1_BINDING_OVERRIDE_2325        1

/**
 * KTH7823 电角零偏（出厂值）。
 * 2026-10-25 纯有感恒速 2000rpm 实测 δ 反推：3.830 rad。
 */
#define M1_ENCODER_OFFSET_RAD           3.830f

/** 1=SIGNOFF 布局 ch11=enc_raw；0=ch11=mark。 */
#define M1_VOFA_SIGNOFF_ENC_RAW         1

/** 机械角符号。电角已对齐时保持 +1。 */
#define M1_ENCODER_MECH_SIGN            (1.0f)

/** 1=Park 用 -θ；与 MECH_SIGN 勿同时乱开。 */
#define M1_THETA_NEGATE                 0

/** 1=VOFA ch4 输出 SVPWM 扇区 1..6（替换 Id）。 */
#define M1_VOFA_SECTOR_DIAG             0

#define PHASE_CAL_FAIL_NONE             0u
#define PHASE_CAL_FAIL_OC               1u
#define PHASE_CAL_FAIL_SNR              2u
#define PHASE_CAL_FAIL_PERM             3u
#define PHASE_CAL_FAIL_FLASH            4u
#define PHASE_CAL_FAIL_TIMEOUT          5u

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_M1_AXIS_HW_H */
