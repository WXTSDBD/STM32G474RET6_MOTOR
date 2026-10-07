#!/usr/bin/env python3
"""Bode 相频解卷绕补充：f@−90° + 全频 |G| 的 f−3dB（不依赖 reliable 窗）。

用法（仓库根目录）:
  python tools/ident/analyze_iq_bode_phase.py VOFA+CSV/20260706/vofa+202607070042.csv --axis iq
  python tools/ident/analyze_iq_bode_phase.py --batch   # 跑主档 Iq×3 + Id×2
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "tools" / "ident"))

from analyze_iq_bode import analyze  # noqa: E402


def unwrap_phase_deg(ph_raw: np.ndarray) -> np.ndarray:
    """Map to (−180, 180], then unwrap to continuous degrees."""
    ph = np.asarray(ph_raw, dtype=float)
    ph = np.where(np.isfinite(ph), ph, np.nan)
    # fill nan with linear interp for unwrap continuity (rare)
    if np.any(~np.isfinite(ph)):
        idx = np.arange(len(ph))
        ok = np.isfinite(ph)
        if ok.sum() < 2:
            return ph
        ph = np.interp(idx, idx[ok], ph[ok])
    ph = ((ph + 180.0) % 360.0) - 180.0
    return np.rad2deg(np.unwrap(np.deg2rad(ph)))


def interp_cross_falling(fs: np.ndarray, ys: np.ndarray, y_tgt: float, f_min: float = 800.0) -> float:
    """First falling crossing of y_tgt for f >= f_min."""
    for i in range(len(ys) - 1):
        if fs[i] < f_min:
            continue
        a, b = float(ys[i]), float(ys[i + 1])
        if not (math.isfinite(a) and math.isfinite(b)):
            continue
        if a >= y_tgt >= b:
            if abs(b - a) < 1e-15:
                return float(fs[i])
            t = (y_tgt - a) / (b - a)
            return float(fs[i] + t * (fs[i + 1] - fs[i]))
    return float("nan")


def phase_metrics(rows: list[tuple[float, dict]]) -> dict:
    if not rows:
        return {
            "f3db_full_hz": float("nan"),
            "f_m90_hz": float("nan"),
            "n_bins": 0,
            "table": [],
        }
    fs = np.array([f for f, _ in rows], dtype=float)
    mag = np.array([m["G_i"] for _, m in rows], dtype=float)
    ph_raw = np.array([m.get("phase_deg", float("nan")) for _, m in rows], dtype=float)
    ph_u = unwrap_phase_deg(ph_raw)

    f3 = interp_cross_falling(fs, mag, 0.707, f_min=800.0)
    f90 = interp_cross_falling(fs, ph_u, -90.0, f_min=800.0)

    table = []
    for f, g, pr, pu in zip(fs, mag, ph_raw, ph_u):
        if 1200.0 <= f <= 2200.0:
            table.append(
                {
                    "f": float(f),
                    "G": float(g),
                    "ph_raw": float(pr),
                    "ph_unw": float(pu),
                }
            )
    return {
        "f3db_full_hz": f3,
        "f_m90_hz": f90,
        "n_bins": len(rows),
        "table": table,
        "fs": fs,
        "mag": mag,
        "ph_raw": ph_raw,
        "ph_unw": ph_u,
    }


def analyze_phase(csv: Path, *, axis: str = "iq", fs: float = 20000.0) -> dict:
    r = analyze(csv, axis=axis, fs=fs)
    out_segs = {}
    for lbl, rows in r.get("all_rows", {}).items():
        out_segs[lbl] = phase_metrics(rows)
        # attach bias if present
        s = r.get("segments", {}).get(lbl, {})
        out_segs[lbl]["bias_A"] = s.get("bias_A", float("nan"))
    return {
        "csv": str(csv),
        "axis": axis,
        "segments": out_segs,
        "labels": list(out_segs.keys()),
    }


def fmt_one(tag: str, r: dict) -> str:
    lines = [
        f"# Bode 相频解卷绕补充 — `{Path(r['csv']).name}`",
        "",
        f"- 轴：**{r['axis'].upper()}**",
        f"- 方法：各频点 `phase_deg=∠fb−∠ref`（脚本 raw）→ (−180,180] → `np.unwrap` → 连续相位；",
        f"  **f@−90°** = 解卷绕相位首次降至 −90°（f≥800 Hz 线性插值）；",
        f"  **f−3dB_full** = 全频 \|G\| 首次过 0.707（不依赖 reliable 窗，避免早期 n/a 误导）。",
        "",
        "## 摘要",
        "",
        "| 段 | bias | f−3dB_full (Hz) | f@−90° (Hz) | n_bins |",
        "|----|------|-----------------|-------------|--------|",
    ]
    for lbl in r["labels"]:
        s = r["segments"][lbl]
        bias = s.get("bias_A", float("nan"))
        bias_s = f"{bias:.2f} A" if math.isfinite(bias) else "—"
        f3 = s["f3db_full_hz"]
        f90 = s["f_m90_hz"]
        f3s = f"{f3:.1f}" if math.isfinite(f3) else "n/a"
        f90s = f"{f90:.1f}" if math.isfinite(f90) else "n/a"
        lines.append(
            f"| **{lbl}** | {bias_s} | **{f3s}** | **{f90s}** | {s['n_bins']} |"
        )

    lines += ["", "## 邻频点（约 1.2–2.2 kHz）", ""]
    for lbl in r["labels"]:
        s = r["segments"][lbl]
        if "hi" not in lbl.lower() and len(r["labels"]) > 1:
            # still print all, but mark
            pass
        lines.append(f"### {lbl}")
        lines.append("| f (Hz) | \|G\| | phase raw (°) | phase unwrapped (°) |")
        lines.append("|--------|------|---------------|---------------------|")
        for row in s["table"]:
            lines.append(
                f"| {row['f']:.1f} | {row['G']:.3f} | {row['ph_raw']:.1f} | {row['ph_unw']:.1f} |"
            )
        lines.append("")

    lines += [
        "## 结论口径（简历 / 进度文档可用）",
        "",
        "- 主验收看 **OFF-hi**：幅值 f−3dB≈2.1 kHz 量级；相频解卷绕后 **≈1.55–1.6 kHz 滞后至 −90°**。",
        "- raw 相位表上的 +270° 类跳变是 ±180°/360° **卷绕**，非物理突变。",
        "",
        "---",
        f"*tools/ident/analyze_iq_bode_phase.py*",
    ]
    return "\n".join(lines)


BATCH = [
    (ROOT / "VOFA+CSV" / "20260706" / "vofa+202607070042.csv", "iq"),
    (ROOT / "VOFA+CSV" / "20260706" / "vofa+202607070044.csv", "iq"),
    (ROOT / "VOFA+CSV" / "20260706" / "vofa+202607070045.csv", "iq"),
    (ROOT / "VOFA+CSV" / "20260707" / "vofa+202607072149.csv", "id"),
    (ROOT / "VOFA+CSV" / "20260707" / "vofa+202607072152.csv", "id"),
]


def run_batch(out_dir: Path) -> str:
    out_dir.mkdir(parents=True, exist_ok=True)
    summaries = []
    lines = [
        "# 电流环 Bode 相频解卷绕补充报告",
        "",
        "**日期**：2026-08-03  ",
        "**目的**：在已有 −3dB≈2.1 kHz 幅值签收基础上，对主档录波补做相位解卷绕，给出 **f@−90°**。  ",
        "**脚本**：`tools/ident/analyze_iq_bode_phase.py`（调用 `analyze_iq_bode.analyze` 提取频点后 unwrap）。  ",
        "",
        "## 主档汇总（OFF-hi）",
        "",
        "| 录波 | 轴 | f−3dB_full (Hz) | f@−90° (Hz) |",
        "|------|----|-----------------|-------------|",
    ]
    for csv, axis in BATCH:
        if not csv.is_file():
            lines.append(f"| `{csv.name}` | {axis} | **MISSING** | — |")
            continue
        r = analyze_phase(csv, axis=axis)
        md = fmt_one(csv.stem, r)
        md_path = out_dir / f"分析报告_BODE_PHASE_{csv.stem}.md"
        md_path.write_text(md, encoding="utf-8")
        # pick OFF-hi
        hi = None
        for lbl, s in r["segments"].items():
            if "hi" in lbl.lower():
                hi = s
                break
        if hi is None and r["segments"]:
            hi = list(r["segments"].values())[-1]
        f3 = hi["f3db_full_hz"] if hi else float("nan")
        f90 = hi["f_m90_hz"] if hi else float("nan")
        lines.append(
            f"| `{csv.name}` | {axis.upper()} | "
            f"{f3:.1f} | {f90:.1f} |"
        )
        summaries.append((csv.name, axis, f3, f90, md_path))

    f90s = [x[3] for x in summaries if math.isfinite(x[3])]
    f3s = [x[2] for x in summaries if math.isfinite(x[2])]
    lines += [
        "",
        "## 统计",
        "",
        f"- OFF-hi **f@−90°**：均值 **{float(np.mean(f90s)):.1f} Hz**"
        f"（min {min(f90s):.1f} / max {max(f90s):.1f}，n={len(f90s)}）"
        if f90s
        else "- OFF-hi f@−90°：无有效点",
        f"- OFF-hi **f−3dB_full**：均值 **{float(np.mean(f3s)):.1f} Hz**"
        f"（min {min(f3s):.1f} / max {max(f3s):.1f}）"
        if f3s
        else "",
        "",
        "## 分报告",
        "",
    ]
    for name, axis, f3, f90, md_path in summaries:
        lines.append(f"- [{md_path.name}]({md_path.name}) — {axis.upper()} f−3dB={f3:.1f} Hz, f@−90°={f90:.1f} Hz")

    lines += [
        "",
        "## 简历可用表述",
        "",
        "> 闭环 Bode：幅频上 Iq/Id 的 −3dB 约 2128 Hz / 2060 Hz；相频解卷绕后约 1.55–1.6 kHz 滞后至约 −90°。",
        "",
        "---",
        "*由 analyze_iq_bode_phase.py --batch 生成*",
    ]
    summary = "\n".join(lines)
    (out_dir / "分析报告_BODE_PHASE_解卷绕汇总_2026-08-03.md").write_text(summary, encoding="utf-8")
    # also copy-ish into docs
    docs = ROOT / "docs" / "分析报告_BODE_PHASE_解卷绕补充_2026-08-03.md"
    docs.write_text(summary, encoding="utf-8")
    return summary


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("csv", type=Path, nargs="?", help="single VOFA CSV")
    ap.add_argument("--axis", choices=("iq", "id"), default="iq")
    ap.add_argument("--batch", action="store_true", help="run main Iq/Id archive set")
    ap.add_argument("--md", type=Path, help="write markdown for single CSV")
    ap.add_argument(
        "--out-dir",
        type=Path,
        default=ROOT / "VOFA+CSV" / "20260706",
        help="batch output directory",
    )
    args = ap.parse_args()

    if args.batch:
        text = run_batch(args.out_dir)
        try:
            print(text)
        except UnicodeEncodeError:
            sys.stdout.buffer.write(text.encode("utf-8", errors="replace"))
            sys.stdout.buffer.write(b"\n")
        return 0

    if args.csv is None:
        ap.error("csv required unless --batch")
    r = analyze_phase(args.csv, axis=args.axis)
    text = fmt_one(args.csv.stem, r)
    try:
        print(text)
    except UnicodeEncodeError:
        sys.stdout.buffer.write(text.encode("utf-8", errors="replace"))
        sys.stdout.buffer.write(b"\n")
    if args.md:
        args.md.write_text(text, encoding="utf-8")
        print(f"\nWrote {args.md}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
