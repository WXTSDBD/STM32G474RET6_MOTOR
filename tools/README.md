# Keil AC6 + FreeRTOS (CubeMX)

详细说明见：[docs/Keil_AC6_FreeRTOS_CubeMX适配说明.md](../docs/Keil_AC6_FreeRTOS_CubeMX适配说明.md)

CubeMX generates FreeRTOS with the **RVDS** port (AC5). Keil **AC6** (ArmClang) needs the **GCC** port.

## Workflow

1. Close Keil
2. CubeMX **GENERATE CODE** (After script runs automatically)
3. Keil **Rebuild**

If build still fails, double-click `fix_keil_ac6_freertos.bat` and Rebuild again.

## What the script patches automatically

- Copy GCC port into `Middlewares/.../GCC/ARM_CM4F/`
- `uvprojx`: RVDS -> GCC, remove UTF-8 BOM
- `FreeRTOSConfig.h` USER CODE: `extern SystemCoreClock`, `configENABLE_FPU 1`

No manual edits needed after Generate.

## New project

1. Copy this entire `tools/` folder into the new repo
2. Set CubeMX **After Code Generation** to `tools\fix_keil_ac6_freertos.bat`
3. Generate -> Rebuild

## Files

- `freertos_port/GCC/ARM_CM4F/` - permanent GCC port backup (commit to git)
- `fix_keil_ac6_freertos.bat` - CubeMX "After Code Generation" hook
