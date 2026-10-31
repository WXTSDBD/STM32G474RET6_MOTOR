/**
 * @file telem_lut_dump.c
 * @date 2026-10-06
 * @brief LUT 突发 dump 空壳（建表 LUT 已下线）。
 *
 * 节拍限制见 telem_lut_dump.h 文件头。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */
#include "telem_lut_dump.h"

#include <stddef.h>

void telem_lut_dump_arm(void)
{
}

uint8_t telem_lut_dump_is_busy(void)
{
    return 0u;
}

int telem_lut_dump_next(float vals[], uint8_t lut_dump_ch)
{
    (void)vals;
    (void)lut_dump_ch;
    return 0;
}
