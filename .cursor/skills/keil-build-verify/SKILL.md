---
name: keil-build-verify
description: >-
  Run Keil MDK Rebuild for STM32G474RET6_MOTOR before delivering firmware
  changes; fix compile errors until 0 Error(s). Use when modifying Core/, motor/,
  config/, debug/, MDK-ARM/, or when the user asks to verify/build/compile Keil
  firmware.
---

# Keil 编译验证（STM32G474RET6_MOTOR）

修改会参与 MDK 编译的固件代码后，**必须先本地 Keil 编译**，确认 **0 Error(s)** 再向用户交付；有 Error 须修到通过为止。

## 何时必须编译

- 改了 `Core/`、`motor/`、`config/`、`debug/`、`Drivers/`（业务侧）、`MDK-ARM/`
- 改了 `motor_params_m1.h` 或会触发 `#error` 的宏组合
- 用户说「编译」「build」「验证」「烧录前检查」

可跳过：仅改 `tools/*.py`、`docs/`、`.cursor/` 且未动固件。

## 编译命令（PowerShell）

工程路径固定：

```text
D:\stm32\STM32G474RET6_MOTOR\MDK-ARM\STM32G474RET6_MOTOR.uvprojx
```

**须 `-Wait` 等编译结束**（全量 Rebuild 约 60–90 s）。勿用 `-j0` 并行，易触发 via 文件冲突。

```powershell
Set-Location "D:\stm32\STM32G474RET6_MOTOR\MDK-ARM"
Start-Process -FilePath "D:\keil5\UV4\UV4.exe" `
  -ArgumentList '-r','D:\stm32\STM32G474RET6_MOTOR\MDK-ARM\STM32G474RET6_MOTOR.uvprojx','-o','D:\stm32\STM32G474RET6_MOTOR\MDK-ARM\rebuild_log.txt' `
  -Wait -NoNewWindow
Get-Content "D:\stm32\STM32G474RET6_MOTOR\MDK-ARM\rebuild_log.txt" -Tail 5
```

小改动可用 `-b`（Build）代替 `-r`（Rebuild）以节省时间。

## 判定

- 读 `MDK-ARM/rebuild_log.txt` 末尾：须含 `0 Error(s)`（日志中间偶见 ArmClang 级联 stderr，以末尾汇总为准）。
- Warning 可保留，但 `#error` 触发的配置冲突必须修。
- 编译失败时：在 `rebuild_log.txt` 搜 `error:`，修完再 Rebuild，直到通过。

## 常见失败

- `motor_params_m1.h` 宏互斥 `#error`：对照当前 `M1_BRINGUP_MODE` 分支修宏。
- 新增宏在 `#if M1_IDENT_ENABLE` 外使用：在模式块与 `#ifndef` 兜底两处补齐。
- CubeMX Generate 后：跑 `tools/fix_keil_ac6_freertos.bat` 再 Rebuild。

## 交付给用户时

在回复中写明编译结果，例如：

```text
Keil Rebuild: 0 Error(s), N Warning(s)
```

若未通过，不要声称任务完成；先修错再编译。
