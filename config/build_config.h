/**
 * @file build_config.h
 * @date 2026-10-25
 * @brief 编译期能力闸（编不编进模块）。普通 #define，唯一源。
 *
 * 运行配方由 route / ops / 实验表决定，不在此堆开关。
 */
#ifndef CONFIG_BUILD_CONFIG_H
#define CONFIG_BUILD_CONFIG_H

#define BUILD_HFI               1
#define BUILD_SMO               1
#define BUILD_VEQ               0
#define BUILD_DEADBAND          0
#define BUILD_IDENT             0
#define BUILD_EXP_TABLES        1
#define BUILD_TELEM_VOFA        1

#endif /* CONFIG_BUILD_CONFIG_H */
