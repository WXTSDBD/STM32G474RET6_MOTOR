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

## 建模台与交付包（2026-10-17 起）

| 脚本 | 用途 |
|------|------|
| `simtool/` | **统一建模台（数据 → 模型 → 预测）**：分层、参数单一源、传函与裕度、SMO/HFI 分析台；总纲见 `tools/simtool/README.md` |
| `sim/` | 数据层：录波台账 / `Sig` 契约 / 包合格证（`inventory.py` → `out/ledger_*.csv`） |
| `pkg_snapshot.py` | 冻结交付包：补齐 `motor_params_m1.h` / `bringup_bench.h` / `git.txt` 等，让包**可回放**（解 README §7.4）。`--dry-run` 先看清单 |
| `simtool/arch.py` | **架构判据**（秒级）：行数预算 / 依赖方向 / 无环 / 无孤儿 / 参数单一源 / 入口自带自检。加文件或动 import 前后都跑它 |
| `simtool/silrun.py` | **固件 SIL**：ziglang 把真的 `emf_smo.c` / `emf_pll.c` 编成 DLL；200 rpm 结构死区、800/2000 rpm 幅值与 PLL 对账 |
| `simtool/sil/` | SIL 胶水（`sil_obs.c`）与 profile shim（不改固件，用 `-include` 选中无感配置） |
| `simtool/current.py` | **电流环数据裁决**：`ud/uq -> id/iq` 灰盒回放；回放/参数/延迟分别给 PASS/WARN/REJECT；`--batch` 扫台账全部电流环目标包（38 份） |
| `simtool/currentfit.py` | **电流环数学层**（L3.5）：稳态点提取 / 等效帧 `delta` 与 `R_eff/psi_eff` 拟合 / 展开到 50 us 控制拍的回放内核 |
| `simtool/anchor.py` | **回放等效锚点**（L2.5）：`delta/R_eff/psi_eff` 注册表 + `EquivalentPlant`；明确标注“不是物理参数” |
| `simtool/calibrate.py` | **锚点筛选**：从 `current --batch` JSON 选 A/B/C，落盘 `tools/model/anchors_m1.json` |
| `simtool/horizon.py` | **预测时域/自由发散**：每个起点实测重置的 1~200 ms 误差曲线；支持 `--anchor` 跨包诊断 |
| `sim/probe.py` | **只读布局自证**：ident_duty 的 duty→vd/vq 一致性、共模与 Vbus；不猜列号 |
| `simtool/ident_duty.py` | **锁轴静态辨识**：A/B/C 分级、IRLS Rs、表观 τ、批量与导出 |
| `model/electrical.py` | **电气对象纯数学**：RL 连续传函 / 精确 ZOH / 固件同式 PI / 连续与离散裕度 |
| `simtool/plantview.py` | **对象+控制器闭合模型**：锁轴证据 → 传函 / ZOH / 裕度 / 阶跃 |
| `simtool/obsview.py` | **HFI/SMO 统一视图**：C7/C0/C8/C4 与 SMO 地板/顶板/PM 并列 |
| `simtool/modelcard.py` | **统一模型卡**：只读 data_usability/plantview/obsview/sil 产物，输出数据覆盖/对象/电流环/观测器/SIL 的 JSON+Markdown |

录波 CSV 与分析脚本落盘见 `.cursor/skills/repo-hygiene/SKILL.md`（`VOFA+CSV/<日期>/`）。

## 2026-10-17 晚补充

- ziglang 0.16.0 已安装：python -m ziglang version 正常，可继续跑 simtool/silrun.py。
- 锁轴 A 级证据：Rs=0.123859 ohm、d 轴表观 Ld=72.33 uH@10kHz（扣 1 拍 59.94 uH）；Lq 本轮未复测（历史 VASI/LCR 87 uH）。
- 新增详细报告：docs/报告_锁轴辨识与对象模型_2026-10-17.md。
