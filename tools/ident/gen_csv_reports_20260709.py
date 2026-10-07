#!/usr/bin/env python3
"""Generate per-CSV analysis reports for VOFA+CSV/20260709."""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
from parse_ident_vofa import find_header_rows, load_csv, parse_burst  # noqa: E402

OUT_DIR = ROOT / "VOFA+CSV" / "20260709"
LD_LCR = 59.0
LQ_LCR = 87.0
RS_NOM = 0.122
FS = 10000.0


def pct(val: float, ref: float) -> str:
    if not np.isfinite(val) or ref <= 0:
        return "—"
    return f"{100.0 * (val / ref - 1.0):+.0f}%"


def flag_ok(val: float, ref: float, th: float = 15.0) -> str:
    if not np.isfinite(val):
        return "—"
    return "OK" if abs(100.0 * (val / ref - 1.0)) <= th else "—"


def timeline(d: np.ndarray) -> dict[str, dict]:
    os_ = d["I11"]
    tl: dict[str, dict] = {}
    for seq, name in [
        (38, "ALIGN"),
        (70, "Ud_AB"),
        (163, "PRE_DECAY"),
        (57, "VASI"),
        (58, "DONE"),
    ]:
        m = np.isclose(os_, seq, atol=0.5)
        if not np.any(m):
            continue
        idx = np.where(m)[0]
        tl[name] = {
            "t0": float(idx[0] / FS),
            "t1": float(idx[-1] / FS),
            "dur": float(len(idx) / FS),
        }
    return tl


def classify(tl: dict, bursts: list) -> str:
    if bursts and bursts[-1].get("rs", 0) > 0:
        return "full"
    if tl.get("VASI"):
        return "vasi_only"
    return "invalid"


def grid_table_md(grid: list, proto: float) -> list[str]:
    lines = [
        "| (Id,Iq) | Ld 500 | Ld 1k | Lq 500 | Lq 1k |",
        "|---------|--------|-------|--------|-------|",
    ]
    if proto >= 2.95:
        lines[0] = "| (Id,Iq) | Ld 500 | Ld 1k | Ld 2k | Lq 500 | Lq 1k | Lq 2k |"
        lines[1] = "|---------|--------|-------|-------|--------|-------|-------|"
    for g in grid:
        idb, iqb = g["id_bias"], g["iq_bias"]
        lc = f"{g['ld_coarse_uH']:.1f}" if g["ld_coarse_valid"] else "—"
        lf = f"{g['ld_fine_uH']:.1f}" if g["ld_fine_valid"] else "—"
        lqc = f"{g['lq_coarse_uH']:.1f}" if g["lq_coarse_valid"] else "—"
        lqf = f"{g['lq_fine_uH']:.1f}" if g["lq_fine_valid"] else "—"
        if proto >= 2.95:
            l2 = f"{g['ld_f2_uH']:.1f}" if g.get("ld_f2_valid") else "—"
            lq2 = f"{g['lq_f2_uH']:.1f}" if g.get("lq_f2_valid") else "—"
            lines.append(
                f"| ({idb:+.1f},{iqb:+.1f}) | {lc} | {lf} | {l2} | {lqc} | {lqf} | {lq2} |"
            )
        else:
            lines.append(f"| ({idb:+.1f},{iqb:+.1f}) | {lc} | {lf} | {lqc} | {lqf} |")
    return lines


def key_points_md(grid: list) -> list[str]:
    lines = ["| 格点 | 量 | 值 [µH] | vs LCR | ≤15% |", "|------|-----|---------|--------|------|"]
    for idb, iqb, key, ref in [
        (1.0, 0.0, "ld_fine", LD_LCR),
        (1.0, 0.5, "lq_fine", LQ_LCR),
        (0.5, 0.0, "ld_fine", LD_LCR),
        (1.5, 0.0, "ld_fine", LD_LCR),
    ]:
        for g in grid:
            if abs(g["id_bias"] - idb) > 0.01 or abs(g["iq_bias"] - iqb) > 0.01:
                continue
            vk = f"{key}_uH".replace("ld_fine", "ld_fine").replace("lq_fine", "lq_fine")
            if key == "ld_fine":
                val, ok = g["ld_fine_uH"], g["ld_fine_valid"]
            else:
                val, ok = g["lq_fine_uH"], g["lq_fine_valid"]
            if not ok:
                continue
            label = "Ld 1k" if "ld" in key else "Lq 1k"
            lines.append(
                f"| ({idb:+.1f},{iqb:+.1f}) | {label} | {val:.1f} | {pct(val, ref)} | {flag_ok(val, ref)} |"
            )
    return lines


def write_full_report(path: Path, tl: dict, burst: dict, n_bursts: int) -> None:
    grid = burst["grid"]
    proto = burst["proto"]
    tag = path.stem.replace("vofa+", "")
    out = OUT_DIR / f"分析报告_{path.name.replace('.csv', '')}_2026-07-10.md"

    ld15 = sum(
        1
        for g in grid
        if g["ld_fine_valid"] and abs(100 * (g["ld_fine_uH"] / LD_LCR - 1)) <= 15
    )
    lq15 = sum(
        1
        for g in grid
        if g["lq_fine_valid"] and abs(100 * (g["lq_fine_uH"] / LQ_LCR - 1)) <= 15
    )

    lines = [
        f"# 单条录波分析 — `{path.name}`",
        "",
        f"**日期**：2026-07-10  ",
        f"**归档**：`VOFA+CSV/20260709/`  ",
        f"**状态**：✅ 有效（VASI 完成 + ident burst 有效）  ",
        f"**汇总报告**：[验收报告_VASI_LdLq_参数辨识_2026-07-10.md](验收报告_VASI_LdLq_参数辨识_2026-07-10.md)",
        "",
        "---",
        "",
        "## 1. 录波概况",
        "",
        f"| 项 | 值 |",
        f"|----|-----|",
        f"| 行数 | {burst.get('rows', '—')} |",
        f"| ident burst 次数 | **{n_bursts}**（解析用**最后一个** @ row {burst['row']}） |",
        f"| proto | **{proto:.1f}** |",
        f"| `rs_used` | **{burst['rs']:.4f} Ω** |",
        f"| fine ok | Ld {burst.get('n_ld_f', '?')}/9，Lq {burst.get('n_lq_f', '?')}/9 |",
        "",
        "## 2. 时序（open_seq）",
        "",
        "| 阶段 | 起始 [s] | 结束 [s] | 时长 [s] |",
        "|------|----------|----------|----------|",
    ]
    for name in ["ALIGN", "Ud_AB", "PRE_DECAY", "VASI", "DONE"]:
        if name in tl:
            t = tl[name]
            lines.append(f"| {name} | {t['t0']:.1f} | {t['t1']:.1f} | {t['dur']:.1f} |")

    if n_bursts > 1:
        lines += [
            "",
            "> ⚠️ 存在 **Pass0 假 burst**（`rs_used=0`）；本报告数据均来自**最后一个** burst。",
        ]

    lines += [
        "",
        "## 3. MCU 9 格结果",
        "",
        *grid_table_md(grid, proto),
        "",
        "## 4. 关键格点 vs LCR（Ld=59 µH，Lq=87 µH）",
        "",
        *key_points_md(grid),
        "",
        f"**1 kHz fine ±15% 命中**：Ld **{ld15}/9**，Lq **{lq15}/9**",
        "",
        "## 5. 本条结论",
        "",
    ]

    # find key values
    ld10 = lq105 = None
    for g in grid:
        if abs(g["id_bias"] - 1) < 0.01 and abs(g["iq_bias"]) < 0.01 and g["ld_fine_valid"]:
            ld10 = g["ld_fine_uH"]
        if abs(g["id_bias"] - 1) < 0.01 and abs(g["iq_bias"] - 0.5) < 0.01 and g["lq_fine_valid"]:
            lq105 = g["lq_fine_uH"]

    if ld10 and lq105:
        lines.append(
            f"- **PI 参考点**：Ld@1k (1,0) = **{ld10:.1f} µH** ({pct(ld10, LD_LCR)})；"
            f"Lq@1k (1,0.5) = **{lq105:.1f} µH** ({pct(lq105, LQ_LCR)})"
        )
    lines += [
        "- **500 Hz coarse Ld** 仍系统性偏高，**勿用于 PI**",
        "- **1 kHz @ Iq=0** 的 Ld 相对 LCR 最可信",
    ]
    if proto >= 2.95:
        lines.append("- **2 kHz f2** 全线偏高，**淘汰**（见汇总报告 §5.4）")

    lines += [
        "",
        "## 6. 解析命令",
        "",
        "```powershell",
        f"python tools\\parse_ident_vofa.py VOFA+CSV\\20260709\\{path.name}",
        "```",
        "",
    ]
    out.write_text("\n".join(lines), encoding="utf-8")
    print("WROTE", out.name)


def write_vasi_only_report(path: Path, tl: dict) -> None:
    out = OUT_DIR / f"分析报告_{path.name.replace('.csv', '')}_2026-07-10.md"
    lines = [
        f"# 单条录波分析 — `{path.name}`",
        "",
        f"**日期**：2026-07-10  ",
        f"**归档**：`VOFA+CSV/20260709/`  ",
        f"**状态**：⚠️ **部分有效**（有 VASI 波形，**无 ident burst**）  ",
        "",
        "---",
        "",
        "## 1. 说明",
        "",
        "本文件 **未嵌入 I0≈−777777 ident burst**，无法直接读取 MCU 9 格 Ld/Lq 表。",
        "属于 **RS_LD_LQ_ONLY / 早期链式** 录波，仅可作波形/流程参考，**不纳入三轮验收统计**。",
        "",
        "## 2. 时序",
        "",
        "| 阶段 | 起始 [s] | 结束 [s] | 时长 [s] |",
        "|------|----------|----------|----------|",
    ]
    for name in ["ALIGN", "PRE_DECAY", "VASI", "DONE"]:
        if name in tl:
            t = tl[name]
            lines.append(f"| {name} | {t['t0']:.1f} | {t['t1']:.1f} | {t['dur']:.1f} |")
    lines += [
        "",
        "## 3. 建议",
        "",
        "- 验收请使用 **`016` / `017` / `042`**（含有效 burst）",
        "- 若需 MCU 网格，请重录并确认 `M1_VOFA_IDENT_DUMP_ENABLE=1` 且链式 DONE 后 burst 发出",
        "",
    ]
    out.write_text("\n".join(lines), encoding="utf-8")
    print("WROTE", out.name, "(vasi_only)")


def write_invalid_report(path: Path) -> None:
    out = OUT_DIR / f"分析报告_{path.name.replace('.csv', '')}_2026-07-10.md"
    lines = [
        f"# 单条录波分析 — `{path.name}`",
        "",
        f"**日期**：2026-07-10  ",
        f"**状态**：❌ **无效 / 未完成 VASI**",
        "",
        "---",
        "",
        "本录波 **无 open_seq=57 VASI 段** 且无 ident burst，不用于 Ld/Lq 验收。",
        "可能为中断录波、模式不匹配或早期调试片段。",
        "",
    ]
    out.write_text("\n".join(lines), encoding="utf-8")
    print("WROTE", out.name, "(invalid)")


def main() -> int:
    for path in sorted(OUT_DIR.glob("vofa+*.csv")):
        rows = load_csv(path)
        raw = np.genfromtxt(path, delimiter=",", names=True, dtype=float)
        tl = timeline(raw)
        burst_rows = find_header_rows(rows)
        bursts = []
        for hi in burst_rows:
            try:
                m = parse_burst(rows, hi)
                bursts.append(
                    {
                        "row": hi,
                        "proto": m["proto"],
                        "rs": m["rs_used_ohm"],
                        "n_ld_f": m.get("n_ld_ok_fine"),
                        "n_lq_f": m.get("n_lq_ok_fine"),
                        "grid": m["grid"],
                        "rows": len(rows),
                    }
                )
            except Exception:
                pass

        kind = classify(tl, bursts)
        if kind == "full":
            write_full_report(path, tl, bursts[-1], len(bursts))
        elif kind == "vasi_only":
            write_vasi_only_report(path, tl)
        else:
            write_invalid_report(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
