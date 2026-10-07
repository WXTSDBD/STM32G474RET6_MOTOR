/**
 * @file app_axis_jeoc.h
 * @date 2026-10-07
 * @brief M1 ADC2 注入完成：标定分支与电流环节拍入口。
 *
 * 只允许从 HAL_ADCEx_InjectedConvCpltCallback 且 hadc 为 ADC2 时调用。
 * 诊断用 ADC1/3/5 仍留在 main.c。
 */

#ifndef BRINGUP_APP_AXIS_JEOC_H
#define BRINGUP_APP_AXIS_JEOC_H

#ifdef __cplusplus
extern "C" {
#endif

void app_axis_jeoc_on_injected(void *hadc);

#ifdef __cplusplus
}
#endif

#endif /* BRINGUP_APP_AXIS_JEOC_H */
