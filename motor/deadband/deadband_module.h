/**
 * @file deadband_module.h
 * @date 2026-10-06
 * @brief 死区模块总入口。按需再单独包含 deadband.h / deadband_cal.h。

 *
 * 本头不含函数，只收口 include。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#ifndef DEADBAND_MODULE_H
#define DEADBAND_MODULE_H

#include "deadband.h"
#include "deadband_cal.h"
#include "deadband_geo.h"
#include "deadband_lut_baked_m1.h"
#include "deadband_service.h"
#include "deadband_id_cal.h"
#include "deadband_flow.h"

#endif /* DEADBAND_MODULE_H */
