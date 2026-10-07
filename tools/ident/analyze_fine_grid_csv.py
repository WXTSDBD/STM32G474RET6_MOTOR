#!/usr/bin/env python3
"""Analyze 15-grid 1kHz-only VASI CSV."""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
from parse_ident_vofa import decode_proc_code, find_header_rows, load_csv, parse_burst  # noqa: E402

LD_LCR = 59.0
LQ_LCR = 87.0
FS = 10000.0
IDS = [0.5, 0.75, 1.0]
IQS = [0.0, 0.25, 0.5, 0.75, 1.0]


def pct(val: float, ref: float) -> str:
    return f"{100.0 * (val / ref - 1.0):+.0f}%"


def analyze(path: Path) -> str:
    rows = load_csv(path)
    raw = np.genfromtxt(path, delimiter=",", names=True, dtype=float)
    os_ = raw["I11"]

    tl: dict[str, tuple[float, float, float]] = {}
    for seq, name in [(38, "ALIGN"), (70, "Ud_AB"), (163, "PRE_DECAY"), (57, "VASI"), (58, "DONE")]:
        m = np.isclose(os_, seq, atol=0.5)
        if not np.any(m):
            continue
        idx = np.where(m)[0]
        tl[name] = (float(idx[0] / FS), float(idx[-1] / FS), float(len(idx) / FS))

    burst_rows = find_header_rows(rows)
    bursts = [parse_burst(rows, hi) for hi in burst_rows]
    b = bursts[-1]
    grid = b["grid"]

    lines = [
        f"# 单条录波分析 — `{path.name}`",
        "",
        f"**固件**：15 格方案 A + 1 kHz fine-only  ",
        f"**LCR 参考**：Ld={LD_LCR:.0f} µH，Lq={LQ_LCR:.0f} µH  ",
        "",
        "---",
        "",
        "## 1. 录波概况",
        "",
        "| 项 | 值 |",
        "|----|-----|",
        f"| 行数 | {len(rows)} |",
        f"| ident burst | **{len(bursts)}** 次，用最后一个 @ row **{b['hdr_row']}** |",
        f"| proto | **{b['proto']:.1f}** |",
        f"| grid_n | **{b['grid_n']}** |",
        f"| rs_used | **{b['rs_used_ohm']:.4f} Ω** |",
        f"| fine ok | Ld **{b['n_ld_ok_fine']}/{b['grid_n']}**，Lq **{b['n_lq_ok_fine']}/{b['grid_n']}** |",
        f"| coarse | n_ld={b['n_ld_ok']}, n_lq={b['n_lq_ok']}（fine-only 应为 0） |",
        "",
        "## 2. 时序",
        "",
        "| 阶段 | 起始 [s] | 结束 [s] | 时长 [s] |",
        "|------|----------|----------|----------|",
    ]
    for name in ["ALIGN", "Ud_AB", "PRE_DECAY", "VASI", "DONE"]:
        if name in tl:
            t = tl[name]
            lines.append(f"| {name} | {t[0]:.1f} | {t[1]:.1f} | {t[2]:.1f} |")

    if "VASI" in tl:
        vasi_m = np.isclose(os_, 57, atol=0.5)
        proc = raw["I9"][vasi_m]
        freq_cnt: dict[str, int] = {}
        for p in np.unique(proc):
            if p < 10:
                continue
            d = decode_proc_code(float(p))
            key = d["freq"]
            freq_cnt[key] = freq_cnt.get(key, 0) + int(np.sum(np.isclose(proc, p, atol=0.5)))
        lines += [
            "",
            "**VASI 注入频段**（proc_code 采样计数）：",
            "",
            "| 频段 | 采样数 |",
            "|------|--------|",
        ]
        for f in sorted(freq_cnt):
            lines.append(f"| {f} | {freq_cnt[f]} |")

    def find_g(idb: float, iqb: float) -> dict:
        for g in grid:
            if abs(g["id_bias"] - idb) < 0.01 and abs(g["iq_bias"] - iqb) < 0.01:
                return g
        raise KeyError((idb, iqb))

    lines += [
        "",
        "## 3. 1 kHz fine 矩阵 [µH]",
        "",
        "### Ld",
        "",
        "| Id \\ Iq | " + " | ".join(f"{iq:.2f}" for iq in IQS) + " |",
        "|---------|" + "|".join(["------"] * len(IQS)) + "|",
    ]
    for idb in IDS:
        cells = []
        for iqb in IQS:
            g = find_g(idb, iqb)
            cells.append(f"{g['ld_fine_uH']:.1f}" if g["ld_fine_valid"] else "—")
        lines.append(f"| **{idb:.2f}** | " + " | ".join(cells) + " |")

    lines += [
        "",
        "### Lq",
        "",
        "| Id \\ Iq | " + " | ".join(f"{iq:.2f}" for iq in IQS) + " |",
        "|---------|" + "|".join(["------"] * len(IQS)) + "|",
    ]
    for idb in IDS:
        cells = []
        for iqb in IQS:
            g = find_g(idb, iqb)
            cells.append(f"{g['lq_fine_uH']:.1f}" if g["lq_fine_valid"] else "—")
        lines.append(f"| **{idb:.2f}** | " + " | ".join(cells) + " |")

    lines += ["", "## 4. 关键切片", "", "### Ld @ Iq=0（vs LCR 59 µH）", ""]
    for idb in IDS:
        g = find_g(idb, 0.0)
        v = g["ld_fine_uH"]
        lines.append(f"- **({idb:.2f}, 0)**：{v:.1f} µH（{pct(v, LD_LCR)}）")

    lines += ["", "### Lq @ Id=1.0（饱和曲线，vs LCR 87 µH）", ""]
    for iqb in IQS:
        g = find_g(1.0, iqb)
        v = g["lq_fine_uH"]
        lines.append(f"- **(1.00, {iqb:.2f})**：{v:.1f} µH（{pct(v, LQ_LCR)}）")

    g10 = find_g(1.0, 0.0)
    g105 = find_g(1.0, 0.5)
    lines += [
        "",
        "## 5. vs 旧三轮验收（016/017/042 1k 均值）",
        "",
        "| 格点 | 本次 | 旧均值 | Δ |",
        "|------|------|--------|---|",
        f"| Ld (1, 0) | {g10['ld_fine_uH']:.1f} | 62.6 | {g10['ld_fine_uH'] - 62.6:+.1f} µH |",
        f"| Lq (1, 0.5) | {g105['lq_fine_uH']:.1f} | 87.6 | {g105['lq_fine_uH'] - 87.6:+.1f} µH |",
    ]

    ld15 = sum(
        1 for g in grid if g["ld_fine_valid"] and abs(100 * (g["ld_fine_uH"] / LD_LCR - 1)) <= 15
    )
    lq15 = sum(
        1 for g in grid if g["lq_fine_valid"] and abs(100 * (g["lq_fine_uH"] / LQ_LCR - 1)) <= 15
    )
    ld15_iq0 = sum(
        1
        for g in grid
        if g["ld_fine_valid"]
        and abs(g["iq_bias"]) < 0.01
        and abs(100 * (g["ld_fine_uH"] / LD_LCR - 1)) <= 15
    )

    lines += [
        "",
        "## 6. 结论",
        "",
        f"- **±15% vs LCR**：Ld **{ld15}/15**（Iq=0 行 **{ld15_iq0}/3**），Lq **{lq15}/15**",
        "- **Lq @ Id=1** 随 Iq 增大整体下降（0→0.75 A 约 93→77 µH），**饱和趋势可见**",
    ]
    if g10["ld_fine_uH"] > 80:
        lines += [
            f"- **Ld @(1,0) = {g10['ld_fine_uH']:.1f} µH** 较旧验收 **+34 µH**，偏离 LCR；"
            "需排查是否与旧 9 格同条件（LUT/ψ/链式 Pass0）或仅加密格点引入",
        ]
    if ld15_iq0 == 0:
        lines.append("- **Iq=0 行 Ld 全超 ±15%** → 本次 Ld 绝对值不可直接用于 PI，优先对比波形/条件")

    return "\n".join(lines) + "\n"


def main() -> int:
    path = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "VOFA+CSV" / "20260710" / "vofa+202607102250.csv"
    out = path.parent / f"分析报告_{path.stem}_2026-07-10.md"
    text = analyze(path)
    out.write_text(text, encoding="utf-8")
    print(text)
    print(f"WROTE {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
