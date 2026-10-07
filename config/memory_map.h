/**
 * @file memory_map.h
 * @date 2026-10-07
 * @brief 片内 Flash / NVM 地址真相源（与 .sct 对齐，本刀不动 scatter）。
 *
 * 改地址只改本头 + 同步 `.sct` / CubeMX；`factory_nvm` 读这些宏。
 * 电机参数段尚未划区（J3），预留注释占位。
 */

#ifndef CONFIG_MEMORY_MAP_H
#define CONFIG_MEMORY_MAP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Flash 起始。 */
#define MEM_FLASH_BASE           0x08000000u
/** App 链接长度（与 STM32G474RET6_MOTOR.sct LR_IROM1 一致）。 */
#define MEM_APP_SIZE             0x0007F000u
/** 出厂 NVM 基址（scatter 不链接该区）。 */
#define MEM_FACTORY_NVM_BASE     0x0807F000u
/** 出厂 NVM 总长：2 × 2 KB 页。 */
#define MEM_FACTORY_NVM_SIZE     0x00001000u
/** 单页大小（G4）。 */
#define MEM_FLASH_PAGE_SIZE      0x00000800u

/* 将来：电机铭牌段（J3）在 NVM 内再切；现无段。 */

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(MEM_FACTORY_NVM_BASE == (MEM_FLASH_BASE + MEM_APP_SIZE),
               "factory_nvm must sit at end of App region");
_Static_assert((MEM_FACTORY_NVM_SIZE % MEM_FLASH_PAGE_SIZE) == 0u,
               "factory_nvm size must be whole pages");
_Static_assert((MEM_FACTORY_NVM_BASE % MEM_FLASH_PAGE_SIZE) == 0u,
               "factory_nvm base must be page-aligned");
#endif

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_MEMORY_MAP_H */
