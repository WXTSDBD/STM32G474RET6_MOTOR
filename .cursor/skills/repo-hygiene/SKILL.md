---
name: repo-hygiene
description: >-
  仓库卫生与产物归置：录波 CSV 与分析脚本的落盘位置、临时文件不许堆在仓库根或
  MDK-ARM 根、清理流程、以及 git 操作必须经用户同意。移动/清理产物文件时使用。
---

# 仓库卫生（STM32G474RET6_MOTOR）

## 1. git：禁止自动操作

- **不许自动执行 `git add` / `commit` / `mv` / `rm` / `restore` / `checkout` / `stash` / `clean` / `push` / `reset`。**
- 需要提交、回滚、清理工作区时，**先说清要执行什么命令、为什么，等用户明确同意再动**。
- 只读查询可以随时跑：`git status`、`git diff`、`git log`、`git ls-files`。

## 2. 录波 CSV 落盘

- 一律 `VOFA+CSV/<YYYYMMDD>/`，按**录波当天**建目录；不存在就新建。
- 文件名沿用 `vofa+<YYYYMMDDHHMM>.csv`。
- **不许**放在仓库根，也不许放在 `MDK-ARM/` 根。

## 3. 分析脚本落盘

- 分析脚本与它分析的 CSV **放同一个日期目录**：`VOFA+CSV/<YYYYMMDD>/_analyze_xxx.py`。
- 脚本里的数据路径写 `VOFA+CSV/<YYYYMMDD>/xxx.csv`（相对仓库根），从仓库根运行。
- 脚本产出的 `.json` / `.txt` / 图，与脚本同目录同前缀，便于成对查找。
- 只有**跨多次实验复用**的工具才放 `tools/`。
- 日期确定不了的放 `VOFA+CSV/_mdk_arm_misc/`，**不要猜日期**。

## 4. 禁止堆放的位置

| 位置 | 只允许 |
|------|--------|
| 仓库根 | `.gitignore`、`*.ioc`、`.mxproject`、正式文档入口 |
| `MDK-ARM/` 根 | Keil 工程自身文件（`*.uvprojx`、`*.uvoptx`、`rebuild_log.txt`、`JLinkLog.txt`、`build_log.txt`） |

分析脚本、`.json`、`.csv`、临时 `.txt` 都不许出现在这两处。

## 5. 清理流程

1. **先空跑**：打印「源 → 目标」映射与数量，报告后再动手。
2. 目标同名文件**不覆盖**，跳过并报告。
3. 只移动**未被 git 跟踪**的文件；被跟踪的先报告，等用户定。
4. **删除必须问过用户**（未跟踪文件删掉不可恢复）。
5. 收尾报告：移了多少、分别去了哪、还有什么没动、有没有冲突。

## 6. 自检

```powershell
Set-Location D:\stm32\STM32G474RET6_MOTOR
Get-ChildItem . -File | Where-Object { $_.Extension -in '.csv','.py','.json' }
Get-ChildItem MDK-ARM -File -Filter '_*'
Get-ChildItem 'VOFA+CSV' -File
git ls-files 'MDK-ARM/_*'   # 跟踪中的不能乱动
```
