/**
 * @file route_ids.h
 * @date 2026-10-28
 * @brief route_id 常量：与 config/routes/r_*.c 的 route_id 字段对齐。
 *
 * 现网配方只编译支持 1/8/9；其余 id 仍留给 bind/表项编号，勿再加 USE_*。
 */
#ifndef CONFIG_ROUTE_IDS_H
#define CONFIG_ROUTE_IDS_H

#define M1_ROUTE_ID_HFI_STANDSTILL       1u
#define M1_ROUTE_ID_SENSED_POS_MIT       2u
#define M1_ROUTE_ID_SENSORLESS_POS_MIT   3u
#define M1_ROUTE_ID_IF_100RPM            4u
#define M1_ROUTE_ID_SPEED_1000RPM        5u
#define M1_ROUTE_ID_OBS_VEQ_1000RPM      6u
#define M1_ROUTE_ID_FLUX_ID_1000RPM      7u
#define M1_ROUTE_ID_BODE_ID_FC1000       8u
#define M1_ROUTE_ID_ID_CAL_PASS0_RS      9u

#endif /* CONFIG_ROUTE_IDS_H */
