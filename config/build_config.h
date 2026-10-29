/**
 * @file build_config.h
 * @date 2026-10-25
 * @brief 唯一允许出现"编译期能力开关"的地方（"宏转表" 阶段 0）。
 *
 * 对应设计：docs/方案_宏转实验表_设计_2026-10-25.md §5.5。
 * 目标：全部工程只允许这 ≤15 个 BUILD_* 决定"编不编这套代码"；
 *       具体跑哪套算法由 route / ops 表 / 实验段表在**运行时**决定。
 *
 * S0.5/S7：BUILD_IDENT → ident_module/flow、speed_ident_*。
 * S8：BUILD_DEADBAND → FIXED 补偿；BUILD_DEADBAND_LUT → 建表 LUT（已下线）。
 * Rs/l_hfi/deadband_id_cal/flow 标定链不走 DEADBAND 开关。
 */
#ifndef CONFIG_BUILD_CONFIG_H
#define CONFIG_BUILD_CONFIG_H

/* ---- 能力（编不编进来） ---- */
#ifndef BUILD_HFI
#define BUILD_HFI               1   /* 高频注入（低速/零速） */
#endif
#ifndef BUILD_SMO
#define BUILD_SMO               1   /* 滑模 + EMF-PLL（中高速） */
#endif
#ifndef BUILD_DEADBAND
#define BUILD_DEADBAND          0   /* FIXED 死区；PASS0_RS 关，有感产品再开 */
#endif
#ifndef BUILD_DEADBAND_LUT
#define BUILD_DEADBAND_LUT      0   /* 建表 LUT / telem_lut_dump；永久下线 */
#endif
#ifndef BUILD_IDENT
#define BUILD_IDENT             0   /* Iq/速度辨识模块；PASS0_RS 不需要 */
#endif
#ifndef BUILD_FACTORY_TEST
#define BUILD_FACTORY_TEST      0   /* 出厂自测序列；0=空壳，不改现网行为 */
#endif
#ifndef BUILD_EXP_TABLES
#define BUILD_EXP_TABLES        1   /* 实验段表 */
#endif
#ifndef BUILD_TELEM_VOFA
#define BUILD_TELEM_VOFA        1   /* VOFA 12ch 遥测 */
#endif
#ifndef BUILD_CANOPEN
#define BUILD_CANOPEN           0   /* CANopen（后续阶段） */
#endif
#ifndef BUILD_OTA
#define BUILD_OTA               0   /* OTA（后续阶段） */
#endif
#ifndef BUILD_ENC_AS5047
#define BUILD_ENC_AS5047        0   /* AS5047 编码器 */
#endif
#ifndef BUILD_ENC_KTH7823
#define BUILD_ENC_KTH7823       1   /* KTH7823 编码器 */
#endif

#endif /* CONFIG_BUILD_CONFIG_H */
