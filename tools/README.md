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

## 脚本索引（自研常用）

| 脚本 | 用途 |
|------|------|
| `fix_keil_ac6_freertos.bat` | CubeMX Generate 后修补 AC6 + FreeRTOS GCC port |
| `quickcheck.ps1` | 单文件 / 小集合语法快检（若存在） |
| `regress_sensed_short.py` | 有感短回归：`--latest` 判 PASS/FAIL（seq 255/252、MREV、hold） |
| `check_gate_hygiene.ps1` | GATE / `#if == 数字` / 条件编译密度扫描（只报告） |
| `macro_map.py` | **包 8.1**：扫 params+profiles+读取点 → `docs/宏归属表_YYYY-MM-DD.{md,json}`（只读；验收 1004/592） |
| `classify_motor_params_undef.py` | **包 8.2 步1**：本文件 `#undef` 分类表 → `docs/宏归属表_undef_classify_*` |
| `undef_safe_batch.py` | **包 8.2 档1**：删「强制值==#ifndef 简单字面量默认」的 `#undef+#define`（`--dry-run` / `--apply`） |
| `scan_motor_params_macros.py` | 兼容入口 → 转发 `macro_map.py` |

录波 CSV 与分析脚本落盘见 `.cursor/skills/repo-hygiene/SKILL.md`（`VOFA+CSV/<日期>/`）。
