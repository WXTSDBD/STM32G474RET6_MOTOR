/**
 * @file route_ids.h
 * @date 2026-10-28
 * @brief route_id 常量：与 config/routes/r_*.c 里 route_id 字段对齐。
 *
 * bringup_active.h 用 M1_ACTIVE_ROUTE_ID 选一条；0=按 BRINGUP_MODE 推导。
 */
#ifndef CONFIG_ROUTE_IDS_H
#define CONFIG_ROUTE_IDS_H

#define M1_ROUTE_ID_AUTO                 0u
#define M1_ROUTE_ID_HFI_STANDSTILL       1u
#define M1_ROUTE_ID_SENSED_POS_MIT       2u
#define M1_ROUTE_ID_SENSORLESS_POS_MIT   3u
#define M1_ROUTE_ID_IF_100RPM            4u
#define M1_ROUTE_ID_SPEED_1000RPM        5u
#define M1_ROUTE_ID_OBS_VEQ_1000RPM      6u
#define M1_ROUTE_ID_FLUX_ID_1000RPM      7u
#define M1_ROUTE_ID_BODE_ID_FC1000       8u
#define M1_ROUTE_ID_ID_CAL_PASS0_RS      9u

#ifndef M1_ACTIVE_ROUTE_ID
#define M1_ACTIVE_ROUTE_ID               M1_ROUTE_ID_AUTO
#endif

#endif /* CONFIG_ROUTE_IDS_H */
