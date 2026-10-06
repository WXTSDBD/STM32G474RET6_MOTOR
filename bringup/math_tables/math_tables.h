/**
 * @file math_tables.h
 * @date 2026-10-06
 * @brief 正弦表声明。数组在 math_tables_sincos.c。

 *
 * 纯数据，不解释每个点。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#ifndef MATH_TABLES_H
#define MATH_TABLES_H

#ifdef __cplusplus
extern "C" {
#endif

void math_tables_sincos_f32(float rad, float *cos_out, float *sin_out);

#ifdef __cplusplus
}
#endif

#endif
