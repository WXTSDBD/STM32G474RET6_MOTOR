/**
 * @file bm_normal.h
 * @date 2026-10-28
 * @brief Bringup 模式配方片段（由 motor_params_m1.h 按 M1_BRINGUP_MODE 纳入）。
 */
#ifndef CONFIG_BM_NORMAL_H
#define CONFIG_BM_NORMAL_H

#define M1_ID_LOCK_CAL_SWEEP            0
#define M1_IDENT_ENABLE                 0
#define M1_ID_CAL_MULTI_ANGLE_ENABLE    0
#define M1_ID_CAL_PASS0_ONLY_ENABLE     0
#define M1_ID_CAL_LUT_VERIFY_SWEEP     0
#define M1_ID_CAL_IQ_PROBE_ENABLE       0
#if M1_SPEED_LOOP_ENABLE && (M1_SPEED_DEADBAND_TEST == M1_SPEED_DEADBAND_TEST_OFF)
/** Run A：速度环死区 OFF 基线 */
#define M1_DEADBAND_LUT_BAKED_ENABLE    0
#define M1_DEADBAND_ENABLE              0
#else
/** 产品：固定死区 ON（591 ns）；建表 LUT 已下线。 */
#undef M1_DEADBAND_GEO_TWO_CLUSTER_ENABLE
#define M1_DEADBAND_GEO_TWO_CLUSTER_ENABLE  0
#define M1_DEADBAND_NVM_ON_BOOT         0
#define M1_DEADBAND_LUT_BAKED_ENABLE    0
#define M1_DEADBAND_LUT_RUNTIME_SCALE   1.0f
#ifndef M1_DEADBAND_ENABLE
#define M1_DEADBAND_ENABLE              1
#endif
#endif /* speed loop deadband off */


#endif /* CONFIG_BM_NORMAL_H */
