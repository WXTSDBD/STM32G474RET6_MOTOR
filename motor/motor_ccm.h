/**
 * @file motor_ccm.h
 * @date 2026-10-10
 * @brief G4 CCM 段属性：仅放 CPU 热状态，禁止 DMA 缓冲。
 *
 * scatter 须有 0x10000000 的 .bss.CCM 区。ADC/SPI DMA 缓冲勿用本宏。
 */

#ifndef MOTOR_CCM_H
#define MOTOR_CCM_H

#ifndef MOTOR_CCM_BSS
#define MOTOR_CCM_BSS __attribute__((section(".bss.CCM")))
#endif

#endif /* MOTOR_CCM_H */
