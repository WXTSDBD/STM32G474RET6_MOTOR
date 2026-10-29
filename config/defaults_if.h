/**
 * @file defaults_if.h
 * @date 2026-10-28
 * @brief I/F 启动默认宏（仅 M1_IF_ENABLE=1 时由 motor_params 纳入）。
 * @note M1_IF_IQ_A 仍留在 params（关 I/F 时 boot 别名仍引用）。
 */
#ifndef CONFIG_DEFAULTS_IF_H
#define CONFIG_DEFAULTS_IF_H

#ifndef M1_IF_ALIGN_S
#define M1_IF_ALIGN_S                   0.2f
#endif
#ifndef M1_IF_ID_A
#define M1_IF_ID_A                      0.0f
#endif
#ifndef M1_IF_TARGET_RPM
#define M1_IF_TARGET_RPM                100.0f
#endif
#ifndef M1_IF_RAMP_S
#define M1_IF_RAMP_S                    3.0f
#endif
#ifndef M1_IF_HOLD_S
#define M1_IF_HOLD_S                    0.0f
#endif
/** 1=RAMP 中速带抬 Iq，压中速丢步 */
#ifndef M1_IF_IQ_MID_BOOST_ENABLE
#define M1_IF_IQ_MID_BOOST_ENABLE       0
#endif
#ifndef M1_IF_IQ_MID_A
#define M1_IF_IQ_MID_A                  5.5f
#endif
#ifndef M1_IF_IQ_MID_RPM_LO
#define M1_IF_IQ_MID_RPM_LO             180.0f
#endif
#ifndef M1_IF_IQ_MID_RPM_HI
#define M1_IF_IQ_MID_RPM_HI             520.0f
#endif
#ifndef M1_IF_TO_OBS_ENABLE
#define M1_IF_TO_OBS_ENABLE             0
#endif
#ifndef M1_IF_OBS_EW_CLAMP_ENABLE
#define M1_IF_OBS_EW_CLAMP_ENABLE       0
#endif
#ifndef M1_IF_OBS_SOFT_BRAKE_ENABLE
#define M1_IF_OBS_SOFT_BRAKE_ENABLE     0
#endif
/* M1_IF_OBS_IQ_MIN_A / SOFT_REGEN / DIR_SIGN / REVERSE：无 .c 引用；IF profile 自备。 */
#ifndef M1_IF_OBS_CRUISE_ENABLE
#define M1_IF_OBS_CRUISE_ENABLE         0
#endif
#ifndef M1_IF_OBS_DIR_SEQ_ENABLE
#define M1_IF_OBS_DIR_SEQ_ENABLE        0
#endif
#if M1_IF_TO_OBS_ENABLE && !M1_IF_ENABLE
#error "M1_IF_TO_OBS_ENABLE requires M1_IF_ENABLE=1"
#endif
#if M1_IF_ENABLE && (M1_STARTUP_ENABLE || M1_OPEN_UD_PRE_ID_CAL_ENABLE || \
                     M1_OPEN_UQ_PRE_ID_CAL_ENABLE)
#error "M1_IF_ENABLE mutually exclusive with Uq/Ud open-loop drag (STARTUP / OPEN_UD / OPEN_UQ)"
#endif
#if M1_IF_ENABLE && M1_OBS_SOFT_SWITCH_ENABLE && !M1_IF_TO_OBS_ENABLE
#error "M1_IF_ENABLE + OBS_SS requires M1_IF_TO_OBS_ENABLE=1"
#endif
#if M1_IF_TO_OBS_ENABLE && !M1_OBS_SOFT_SWITCH_ENABLE
#error "M1_IF_TO_OBS_ENABLE requires M1_OBS_SOFT_SWITCH_ENABLE=1"
#endif
#if M1_IF_TO_OBS_ENABLE && !M1_EMF_PLL_ENABLE
#error "M1_IF_TO_OBS_ENABLE requires M1_EMF_PLL_ENABLE=1"
#endif
#if M1_IF_OBS_DIR_SEQ_ENABLE && !M1_IF_OBS_CRUISE_ENABLE
#error "M1_IF_OBS_DIR_SEQ_ENABLE requires M1_IF_OBS_CRUISE_ENABLE=1"
#endif
#if M1_IF_OBS_DIR_SEQ_ENABLE && !M1_IF_TO_OBS_ENABLE
#error "M1_IF_OBS_DIR_SEQ_ENABLE requires M1_IF_TO_OBS_ENABLE=1"
#endif

#endif /* CONFIG_DEFAULTS_IF_H */
