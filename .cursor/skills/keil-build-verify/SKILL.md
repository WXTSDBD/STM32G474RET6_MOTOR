---
name: keil-build-verify
description: >-
  Before delivering any firmware change to the user, the agent MUST personally
  run Keil MDK build/rebuild and achieve 0 Error(s). Use when modifying Core/,
  motor/, config/, debug/, MDK-ARM/, or when the user asks to verify/build/compile.
---

# Keil 编译验证（STM32G474RET6_MOTOR）

## 硬约束（对人交付）

**给人类的固件代码，必须由你自己用 Keil 编译一遍，且 `0 Error(s)` 且 `0 Warning(s)`。**

- 改完固件 → **你**跑 Keil → 确认 `0 Error(s), 0 Warning(s)` → 再向用户说做完/可烧录。
- 禁止不编译就交付；禁止留 Error；**禁止留 Warning**（含历史 Warning，本仓库以 0/0 验收）。
- 有报错就自己修，循环到 0/0 为止，不要把这个循环丢给用户。

## 固定工作流（每一轮都按这个走）

```text
1. 改代码（按 .cursor/skills/comment-style.md 写注释）
2. 自己跑 Keil Rebuild → 修到 0 Error(s), 0 Warning(s)
3. ★ 先交付可烧录产物 ★
   打 MDK-ARM/_pkg/<日期>_<实验名>/ 包，给出绝对路径、字节数、sha256、Program Size，
   并说明本轮改了什么、上电后会跑什么。到这一步就交棒。
4. 用户上台架跑（可能不是立刻有结果），把 CSV 放进 VOFA+CSV/<日期>/
5. Agent 再写分析脚本（或复用），放同日期目录，做分析并回报
```

## 顺序硬约束

- **先给 Keil 可烧录的东西，再去写分析脚本。** 第 3 步交付之前，不许先堆分析脚本。
- 交付内容按仓库惯例三件套＋：`STM32G474RET6_MOTOR.hex`、`rebuild_log.txt`、
  `size.txt`、`hex.sha256`、`bringup_active.h`、`<profile>.profile.h`。
- **不要催结果、不要假设立刻有 CSV**：用户要上台架，实验与取数有等待期。没有 CSV 就停在该步。
- 分析脚本落盘位置见 `.cursor/skills/repo-hygiene/SKILL.md`：**与它分析的 CSV 同一日期目录**
  （`VOFA+CSV/<YYYYMMDD>/`），只有跨实验复用的工具才放 `tools/`。

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

- 读 `MDK-ARM/rebuild_log.txt` 末尾：须含 `0 Error(s)` **且** `0 Warning(s)`。
- 失败：搜 `error:` / `warning:`，修完再编，直到通过。两者都不许留给用户。

## 常见失败

- `motor_params_m1.h` 宏互斥 `#error`：对照当前 `M1_BRINGUP_MODE` 分支修宏。
- 新增宏在 `#if M1_IDENT_ENABLE` 外使用：在模式块与 `#ifndef` 兜底两处补齐。
- CubeMX Generate 后：跑 `tools/fix_keil_ac6_freertos.bat` 再 Rebuild。
- 注释里写 `*ud/*uq` 会触发 `-Wcomment`（`/*` 嵌套）：改成「ud/uq」。
- 新增 `.c` 必须同时在 `MDK-ARM/*.uvprojx` 里加 `<File>` 条目，否则不参与编译。

## 交付给用户时

```text
Keil Rebuild: 0 Error(s), 0 Warning(s)
```

未通过不得声称任务完成。
