/**
 * @file defaults_speed_loop.h
 * @date 2026-10-28
 * @brief 速度/位置环默认（仅 HFI 交付 route 纳入）。只留仍被读到的宏。
 */
#ifndef CONFIG_DEFAULTS_SPEED_LOOP_H
#define CONFIG_DEFAULTS_SPEED_LOOP_H

#define M1_SPEED_DECIM                  10u
#define M1_SPEED_TS_S                   (M1_CTRL_TS_S * (float)M1_SPEED_DECIM)
#define M1_SPEED_PI_KP                  0.015f
#define M1_SPEED_PI_KI                  0.002f
/** 2-DOF 速度 PI：β=0.4 抑阶跃超调。 */
#define M1_SPEED_PI_BETA                0.4f
#define M1_SPEED_OMEGA_RAMP_ENABLE      1
/* TODO(factory)：rpm/s 非物理默认值；出厂按 J、Kt、I_avail 写入。 */
#define M1_SPEED_OMEGA_RAMP_RPM_S       500.0f
#define M1_SPEED_IQ_SLEW_ENABLE         0
#define M1_SPEED_IQ_SLEW_A_PER_S        3.0f
#define M1_VOFA_SPEED_ISR_CH            0

#define M1_POS_LOOP_ENABLE              1
#define M1_POS_KP_RPM_PER_RAD           80.0f
#define M1_POS_OMEGA_MAX_RPM            400.0f
#define M1_POS_ERR_HYST_ENABLE          1
#define M1_POS_ERR_HYST_ENTER_RAD       0.0087266f
#define M1_POS_ERR_HYST_EXIT_RAD        0.0174533f
#define M1_POS_ERR_HYST_FREEZE_SPEED_PI 1
#define M1_POS_DECIM                    4u
#define M1_POS_MIT_COMBO_ENABLE         0
#define M1_POS_STEP_TEST_ENABLE         0
#define M1_SPEED_PROFILE_ENABLE         0

#endif /* CONFIG_DEFAULTS_SPEED_LOOP_H */
