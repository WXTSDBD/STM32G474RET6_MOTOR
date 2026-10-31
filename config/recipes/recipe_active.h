/**
 * @file recipe_active.h
 * @date 2026-10-31
 * @brief 按 M1_ACTIVE_ROUTE_ID 纳入唯一配方头（单点选路）。
 *
 * 各 recipe 自行 #include 所需 defaults_*；本头只做 route→文件映射。
 */
#ifndef CONFIG_RECIPES_RECIPE_ACTIVE_H
#define CONFIG_RECIPES_RECIPE_ACTIVE_H

#include "route_ids.h"

#if (M1_ACTIVE_ROUTE_ID == M1_ROUTE_ID_ID_CAL_PASS0_RS)
#include "recipes/recipe_pass0_rs.h"
#elif (M1_ACTIVE_ROUTE_ID == M1_ROUTE_ID_BODE_ID_FC1000)
#include "recipes/recipe_bode_id_fc1000.h"
#elif (M1_ACTIVE_ROUTE_ID == M1_ROUTE_ID_HFI_STANDSTILL)
#include "recipes/recipe_hfi_standstill.h"
#else
#error "unsupported M1_ACTIVE_ROUTE_ID (only HFI_STANDSTILL / PASS0_RS / BODE_ID)"
#endif

#endif /* CONFIG_RECIPES_RECIPE_ACTIVE_H */
