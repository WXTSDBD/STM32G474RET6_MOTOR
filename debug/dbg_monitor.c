/**
 * @file dbg_monitor.c
 * @date 2026-10-06
 * @brief 调试镜像单例。

 *
 * 只有一份 dbg。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#include "dbg_monitor.h"

volatile DbgMon_t dbg;
