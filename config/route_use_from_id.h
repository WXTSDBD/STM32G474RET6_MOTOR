/**
 * @file route_use_from_id.h
 * @date 2026-10-28
 * @brief 由 M1_ACTIVE_ROUTE_ID 推导互斥的 M1_USE_*_PROFILE（S4c）。
 *
 * 须在 route_ids.h / bringup_active.h 之后纳入。AUTO / PASS0_RS / Bode 等
 * 不挂交付 profile 时全 0。bind 见 motor_route_for_boot。
 */
#ifndef CONFIG_ROUTE_USE_FROM_ID_H
#define CONFIG_ROUTE_USE_FROM_ID_H

#if M1_ACTIVE_ROUTE_ID == M1_ROUTE_ID_HFI_STANDSTILL
#define M1_USE_HFI_STANDSTILL_PROFILE       1
#define M1_USE_SENSED_POS_MIT_PROFILE       0
#define M1_USE_SENSORLESS_POS_MIT_PROFILE   0
#define M1_USE_IF_100_PROFILE               0
#define M1_USE_SPEED_1000_PROFILE           0
#define M1_USE_OBS_VEQ_PROFILE              0
#define M1_USE_FLUX_ID_PROFILE              0
#elif M1_ACTIVE_ROUTE_ID == M1_ROUTE_ID_SENSED_POS_MIT
#define M1_USE_HFI_STANDSTILL_PROFILE       0
#define M1_USE_SENSED_POS_MIT_PROFILE       1
#define M1_USE_SENSORLESS_POS_MIT_PROFILE   0
#define M1_USE_IF_100_PROFILE               0
#define M1_USE_SPEED_1000_PROFILE           0
#define M1_USE_OBS_VEQ_PROFILE              0
#define M1_USE_FLUX_ID_PROFILE              0
#elif M1_ACTIVE_ROUTE_ID == M1_ROUTE_ID_SENSORLESS_POS_MIT
#define M1_USE_HFI_STANDSTILL_PROFILE       0
#define M1_USE_SENSED_POS_MIT_PROFILE       0
#define M1_USE_SENSORLESS_POS_MIT_PROFILE   1
#define M1_USE_IF_100_PROFILE               0
#define M1_USE_SPEED_1000_PROFILE           0
#define M1_USE_OBS_VEQ_PROFILE              0
#define M1_USE_FLUX_ID_PROFILE              0
#elif M1_ACTIVE_ROUTE_ID == M1_ROUTE_ID_IF_100RPM
#define M1_USE_HFI_STANDSTILL_PROFILE       0
#define M1_USE_SENSED_POS_MIT_PROFILE       0
#define M1_USE_SENSORLESS_POS_MIT_PROFILE   0
#define M1_USE_IF_100_PROFILE               1
#define M1_USE_SPEED_1000_PROFILE           0
#define M1_USE_OBS_VEQ_PROFILE              0
#define M1_USE_FLUX_ID_PROFILE              0
#elif M1_ACTIVE_ROUTE_ID == M1_ROUTE_ID_SPEED_1000RPM
#define M1_USE_HFI_STANDSTILL_PROFILE       0
#define M1_USE_SENSED_POS_MIT_PROFILE       0
#define M1_USE_SENSORLESS_POS_MIT_PROFILE   0
#define M1_USE_IF_100_PROFILE               0
#define M1_USE_SPEED_1000_PROFILE           1
#define M1_USE_OBS_VEQ_PROFILE              0
#define M1_USE_FLUX_ID_PROFILE              0
#elif M1_ACTIVE_ROUTE_ID == M1_ROUTE_ID_OBS_VEQ_1000RPM
#define M1_USE_HFI_STANDSTILL_PROFILE       0
#define M1_USE_SENSED_POS_MIT_PROFILE       0
#define M1_USE_SENSORLESS_POS_MIT_PROFILE   0
#define M1_USE_IF_100_PROFILE               0
#define M1_USE_SPEED_1000_PROFILE           0
#define M1_USE_OBS_VEQ_PROFILE              1
#define M1_USE_FLUX_ID_PROFILE              0
#elif M1_ACTIVE_ROUTE_ID == M1_ROUTE_ID_FLUX_ID_1000RPM
#define M1_USE_HFI_STANDSTILL_PROFILE       0
#define M1_USE_SENSED_POS_MIT_PROFILE       0
#define M1_USE_SENSORLESS_POS_MIT_PROFILE   0
#define M1_USE_IF_100_PROFILE               0
#define M1_USE_SPEED_1000_PROFILE           0
#define M1_USE_OBS_VEQ_PROFILE              0
#define M1_USE_FLUX_ID_PROFILE              1
#else
/* AUTO / PASS0_RS / Bode(id8) 等：不挂交付 profile */
#define M1_USE_HFI_STANDSTILL_PROFILE       0
#define M1_USE_SENSED_POS_MIT_PROFILE       0
#define M1_USE_SENSORLESS_POS_MIT_PROFILE   0
#define M1_USE_IF_100_PROFILE               0
#define M1_USE_SPEED_1000_PROFILE           0
#define M1_USE_OBS_VEQ_PROFILE              0
#define M1_USE_FLUX_ID_PROFILE              0
#endif

#endif /* CONFIG_ROUTE_USE_FROM_ID_H */
