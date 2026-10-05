---
name: keil-build-verify
description: >-
  Before delivering any firmware change to the user, the agent MUST personally
  run Keil MDK build/rebuild and achieve 0 Error(s). Use when modifying Core/,
  motor/, config/, debug/, MDK-ARM/, or when the user asks to verify/build/compile.
---

# Keil 编译验证（STM32G474RET6_MOTOR）

## 硬约束（对人交付）

**给人类的固件代码，必须由你自己编译一遍且没有 Error。**

- 改完固件 → **你**跑 Keil → 确认 `0 Error(s)` → 再向用户说做完/可烧录。
- 禁止不编译就交付；禁止把「编不过」留给用户。
- 本回合新引入的 Warning 应修掉后再交付。

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

**须 `-Wait` 等编译结束**（全量 Rebuild 约 60–90 s）。勿用 `-j0` 并行。

```powershell
Set-Location "D:\stm32\STM32G474RET6_MOTOR\MDK-ARM"
Start-Process -FilePath "D:\keil5\UV4\UV4.exe" `
  -ArgumentList '-r','D:\stm32\STM32G474RET6_MOTOR\MDK-ARM\STM32G474RET6_MOTOR.uvprojx','-o','D:\stm32\STM32G474RET6_MOTOR\MDK-ARM\rebuild_log.txt' `
  -Wait -NoNewWindow
Get-Content "D:\stm32\STM32G474RET6_MOTOR\MDK-ARM\rebuild_log.txt" -Tail 5
```

小改动可用 `-b`（Build）代替 `-r`（Rebuild）。

## 判定

- 读 `MDK-ARM/rebuild_log.txt` 末尾：须含 `0 Error(s)`。
- 失败：搜 `error:`，修完再编，直到通过。

## 常见失败

- `motor_params_m1.h` 宏互斥 `#error`：对照当前 `M1_BRINGUP_MODE` 分支修宏。
- 新增宏在 `#if M1_IDENT_ENABLE` 外使用：在模式块与 `#ifndef` 兜底两处补齐。
- CubeMX Generate 后：跑 `tools/fix_keil_ac6_freertos.bat` 再 Rebuild。
- 注释里写 `*ud/*uq` 会触发 `-Wcomment`（`/*` 嵌套）：改成「ud/uq」。

## 交付给用户时

```text
Keil Rebuild: 0 Error(s), N Warning(s)
```

未通过不得声称任务完成。
