/**
 * @file deadband_lut_baked_m1.h
 * @date 2026-10-06
 * @brief 烤表死区已下线：apply 恒失败。产品用 FIXED。

 *
 * 原表数据见 `_scratch/archive_deadband_lut/`。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#ifndef DEADBAND_LUT_BAKED_M1_H
#define DEADBAND_LUT_BAKED_M1_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

static inline bool deadband_lut_baked_m1_apply(void)
{
    return false;
}

#ifdef __cplusplus
}
#endif

#endif /* DEADBAND_LUT_BAKED_M1_H */
