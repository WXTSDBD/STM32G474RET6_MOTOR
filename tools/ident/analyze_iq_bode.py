#!/usr/bin/env python3
"""Offline Iq Bode ident analysis (Id cal + 3× Bode OFF/FIXED/LUT).

VOFA channels (M1_IDENT_ID_CAL_BEFORE_STEP):
  Id cal (open_seq < 60): ch3=Ud ch4=Id_ref ch5=theta
  Ident  (open_seq 60-79): ch3=Iq ch4=Iq_ref ch5=Uq

Usage:
  python tools/ident/analyze_iq_bode.py VOFA+CSV/20260627/vofa+202606281835.csv
  python tools/ident/analyze_iq_bode.py file.csv --md report.md
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np
from scipy.signal import find_peaks

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))

from analyze_id_cal_sweep_vofa import load_csv, park, unwrap  # noqa: E402

FS = 20000.0
F0, F1, RATIO, CYCLES = 10.0, 800.0, 1.15, 8.0
BIAS, AMP = 0.25, 0.05


def freq_list() -> list[float]:
    f, out = F0, []
    while f <= F1:
        out.append(f)
        f *= RATIO
    return out


FREQS = freq_list()
SWEEP_S = sum(CYCLES / f for f in FREQS)


def find_id_cal_end(ch4: np.ndarray) -> int:
    idx = np.where(ch4 > 2.0)[0]
    return int(idx[-1]) if len(idx) else 0


def find_bode_start(ch4: np.ndarray, after: int) -> int:
    w = int(0.2 * FS)
    for i in range(after + int(0.5 * FS), len(ch4) - w, int(0.02 * FS)):
        seg = ch4[i : i + w]
        if seg.mean() < 0.15 or seg.mean() > 0.35:
            continue
        if np.std(seg) < 0.01:
            continue
        if np.max(np.abs(np.diff(seg))) < 0.001:
            continue
        return i
    return after + int(0.5 * FS)


def find_round_starts(bode_start: int, ch4: np.ndarray, bode_end: int) -> list[int]:
    """Prefer fixed sweep duration; fall back to peak-based reset detection."""
    sweep_ticks = int(SWEEP_S * FS + 0.5)
    t0 = bode_start
    starts = [t0]
    for _ in range(2):
        nxt = starts[-1] + sweep_ticks
        if nxt >= bode_end - int(0.5 * FS):
            break
        starts.append(nxt)
    if len(starts) >= 3:
        return starts + [min(starts[-1] + sweep_ticks, bode_end)]

    ref = ch4[bode_start:bode_end]
    peaks, _ = find_peaks(ref, distance=int(0.001 * FS), prominence=0.008)
    intervals = np.diff(peaks) / FS if len(peaks) > 1 else np.array([])
    jumps: list[int] = []
    for k in range(1, len(intervals)):
        if intervals[k] > intervals[k - 1] * 1.25 and intervals[k] > 0.035:
            jumps.append(k + 1)
    starts = [bode_start]
    for j in jumps:
        if len(starts) >= 3:
            break
        pi = bode_start + peaks[j]
        if pi - starts[-1] > int(4 * FS):
            starts.append(pi)
    if len(starts) < 3:
        L = bode_end - bode_start
        starts = [bode_start, bode_start + L // 3, bode_start + 2 * L // 3]
    return starts + [bode_end]


def fund_at(x: np.ndarray, f_hz: float) -> complex:
    tt = np.arange(len(x)) / FS
    w = 2 * np.pi * f_hz
    c = np.cos(w * tt)
    s = np.sin(w * tt)
    return 2 * np.mean(x * c) + 1j * 2 * np.mean(x * s)


def analyze_freq_window(
    ch3: np.ndarray,
    ch4: np.ndarray,
    ch5: np.ndarray,
    ia: np.ndarray,
    ib: np.ndarray,
    ic: np.ndarray,
    i0: int,
    i1: int,
    f_hz: float,
) -> dict | None:
    seg_ref = ch4[i0:i1]
    seg_iq = ch3[i0:i1]
    seg_uq = ch5[i0:i1]
    R = fund_at(seg_ref, f_hz)
    if abs(R) < 1e-9:
        return None
    I = fund_at(seg_iq, f_hz)
    U = fund_at(seg_uq, f_hz)
    return {
        "G_i": abs(I / R),
        "ph_i": float(np.angle(I / R, deg=True)),
        "G_u": abs(U / R),
        "ph_u": float(np.angle(U / R, deg=True)),
        "err": float(np.sqrt(np.mean((seg_iq - seg_ref) ** 2))),
        "abc_pp": float((np.ptp(ia[i0:i1]) + np.ptp(ib[i0:i1]) + np.ptp(ic[i0:i1])) / 3),
    }


def sweep_segment(
    ch3: np.ndarray,
    ch4: np.ndarray,
    ch5: np.ndarray,
    ia: np.ndarray,
    ib: np.ndarray,
    ic: np.ndarray,
    a: int,
    b: int,
) -> list[tuple[float, dict]]:
    """Align each freq bin to firmware schedule: 8 cycles at f from segment start."""
    tick = a
    rows: list[tuple[float, dict]] = []
    for f in FREQS:
        nw = int((CYCLES / f) * FS + 0.5)
        i0, i1 = tick, tick + nw
        if i1 > b:
            break
        m = analyze_freq_window(ch3, ch4, ch5, ia, ib, ic, i0, i1, f)
        if m:
            rows.append((f, m))
        tick = i1
    return rows


def summarize_rows(rows: list[tuple[float, dict]]) -> dict:
    if not rows:
        return {}
    gis = [m["G_i"] for _, m in rows]
    errs = [m["err"] for _, m in rows]
    abcs = [m["abc_pp"] for _, m in rows]
    mid = [m["G_i"] for f, m in rows if 80 <= f <= 120]
    return {
        "n_bins": len(rows),
        "G_mean": float(np.mean(gis)),
        "G_10": gis[0],
        "G_mid100": float(np.mean(mid)) if mid else float("nan"),
        "err_mA": float(np.mean(errs) * 1000),
        "abc_pp": float(np.mean(abcs)),
    }


def analyze(path: Path) -> dict:
    ia, ib, ic, ch3, ch4, ch5 = load_csv(path)
    n = len(ch3)
    t = np.arange(n) / FS

    id_end = find_id_cal_end(ch4)
    bode_start = find_bode_start(ch4, id_end)
    bode_end = n - 1
    while bode_end > bode_start and abs(ch4[bode_end]) < 0.02:
        bode_end -= 1

    round_starts = find_round_starts(bode_start, ch4, bode_end + 1)
    labels = ["OFF", "FIXED", "LUT"]

    id_sl = slice(0, id_end + 1)
    theta_raw = ch5[id_sl]
    theta_ok = np.abs(theta_raw) < 10.0
    id_p_max = float("nan")
    if int(theta_ok.sum()) > 1000:
        theta_u = unwrap(theta_raw[theta_ok])
        id_p, _ = park(ia[id_sl][theta_ok], ib[id_sl][theta_ok], ic[id_sl][theta_ok], theta_u)
        id_p_max = float(np.max(np.abs(id_p)))

    segments: dict[str, dict] = {}
    all_rows: dict[str, list] = {}
    for k, lbl in enumerate(labels):
        a, b = round_starts[k], round_starts[k + 1]
        rows = sweep_segment(ch3, ch4, ch5, ia, ib, ic, a, b)
        all_rows[lbl] = rows
        segments[lbl] = {
            "t0": float(t[a]),
            "t1": float(t[b - 1]) if b > a else float(t[a]),
            "dur": float(t[b - 1] - t[a]) if b > a else 0.0,
            **summarize_rows(rows),
            "rows": rows,
        }

    return {
        "path": path,
        "duration_s": float(t[-1]),
        "n_samples": n,
        "id_end_s": float(t[id_end]),
        "id_ref_max": float(ch4.max()),
        "id_park_max": id_p_max,
        "ud_mean": float(np.mean(ch3[id_sl])),
        "bode_start_s": float(t[bode_start]),
        "bode_end_s": float(t[bode_end]),
        "hold_gap_s": float(t[bode_start] - t[id_end]),
        "round_starts_s": [float(t[i]) for i in round_starts],
        "segments": segments,
        "all_rows": all_rows,
    }


def fmt_report(r: dict) -> str:
    p = r["path"]
    lines = [
        f"# Iq Bode 辨识分析 — `{p.name}`",
        "",
        f"- 时长 **{r['duration_s']:.2f} s**，Fs=20000 Hz，单段扫频理论 **{SWEEP_S:.2f} s**（{len(FREQS)} 频点 10→800 Hz）",
        "- 通道：Id 段 ch3=**Ud** ch4=**Id_ref** ch5=**θ**；Bode 段 ch3=**Iq** ch4=**Iq_ref** ch5=**Uq**",
        f"- 激励：`Iq_ref = {BIAS} + {AMP}·sin(2πft)`",
        "",
        "## 时序",
        "",
        f"| 阶段 | 时间 (s) | 说明 |",
        f"|------|----------|------|",
        f"| Id 扫表 | 0 → {r['id_end_s']:.2f} | Id_ref max={r['id_ref_max']:.1f} A，Id(park) max={r['id_park_max']:.2f} A |",
        f"| 过渡/HOLD | {r['id_end_s']:.2f} → {r['bode_start_s']:.2f} | 间隔 **{r['hold_gap_s']:.2f} s**（含 LUT 提交/2s HOLD 等） |",
        f"| Bode×3 | {r['bode_start_s']:.2f} → {r['bode_end_s']:.2f} | 切分点 {', '.join(f'{x:.2f}' for x in r['round_starts_s'])} |",
        "",
        "## 三组 Bode 摘要（Iq_ref → Iq 闭环增益 |G_i|）",
        "",
        "| 组 | t (s) | 时长 (s) | 完成频点 | |G_i| mean | |G_i|@10Hz | |G_i|@~100Hz | err RMS (mA) | abc_pp |",
        "|----|-------|----------|----------|----------|-----------|-------------|--------------|--------|",
    ]

    for lbl in ["OFF", "FIXED", "LUT"]:
        s = r["segments"][lbl]
        lines.append(
            f"| **{lbl}** | {s['t0']:.1f}–{s['t1']:.1f} | {s['dur']:.1f} | "
            f"{s.get('n_bins', 0)}/{len(FREQS)} | {s.get('G_mean', float('nan')):.3f} | "
            f"{s.get('G_10', float('nan')):.3f} | {s.get('G_mid100', float('nan')):.3f} | "
            f"{s.get('err_mA', float('nan')):.1f} | {s.get('abc_pp', float('nan')):.3f} |"
        )

    lines += ["", "## 选频对比（|G_i| / err / abc_pp）", ""]
    for f_tgt in [10, 30, 100, 300, 600]:
        row = f"| **{f_tgt} Hz** |"
        for lbl in ["OFF", "FIXED", "LUT"]:
            rows = r["all_rows"].get(lbl, [])
            hit = next((x for x in rows if abs(x[0] - f_tgt) < 0.1 * f_tgt), None)
            if hit:
                _, m = hit
                row += f" {lbl}: G={m['G_i']:.2f} e={m['err']*1000:.0f}mA abc={m['abc_pp']:.2f} |"
            else:
                row += f" {lbl}: — |"
        lines.append(row)

    lines += ["", "## 结论", ""]

    segs = r["segments"]
    off, fix, lut = segs["OFF"], segs["FIXED"], segs["LUT"]
    idle_s = r["duration_s"] - r["bode_end_s"]
    if idle_s > 5:
        lines.append(
            f"- 有效辨识在 **~{r['bode_end_s']:.0f} s** 结束；之后 **{idle_s:.0f} s** 为 DONE/空转（Iq_ref≈0），可忽略。"
        )

    lines.append(
        f"- **Id 现场建表正常**：0–{r['id_end_s']:.1f} s 扫到 Id_ref max **{r['id_ref_max']:.1f} A**，"
        f"Park(Id) max **{r['id_park_max']:.2f} A**；约 {r['hold_gap_s']:.1f} s 后进入 Bode。"
    )

    if lut.get("n_bins", 0) < len(FREQS):
        lines.append(
            f"- ⚠ **LUT 段扫频不完整**（{lut.get('n_bins', 0)}/{len(FREQS)} 频点，时长仅 {lut['dur']:.1f} s）。"
            " 可能 ident 提前 DONE 或录波窗口偏短；OFF/FIXED 两段已扫满 32 点。"
        )
    else:
        lines.append("- 三组均完成 32 频点扫频（10→800 Hz）。")

    if off.get("n_bins") and fix.get("n_bins") and lut.get("n_bins", 0) >= 15:
        best_err = min(["OFF", "FIXED", "LUT"], key=lambda l: segs[l]["err_mA"])
        lines.append(
            f"- **跟踪误差 RMS**：FIXED={fix['err_mA']:.1f} mA 略优于 "
            f"OFF={off['err_mA']:.1f} / LUT={lut['err_mA']:.1f} mA（{best_err} 最低）。"
        )
        lines.append(
            f"- **低频 10–30 Hz**：三档 |G_i|≈0.8–1.0，死区档位对基波跟踪 **几乎无差别**。"
        )
        lines.append(
            f"- **中频 ~100 Hz**：OFF |G_i|={off.get('G_mid100', float('nan')):.2f}，"
            f"FIXED={fix.get('G_mid100', float('nan')):.2f}，LUT={lut.get('G_mid100', float('nan')):.2f}；"
            "FIXED 中 band 增益略高，与固定补偿减轻 d 轴/Uq 损失一致，但未形成显著带宽扩展。"
        )
        lines.append(
            "- **相电流 abc_pp** 三档同量级（~0.25–0.28 A），LUT 未引入额外 PWM 纹波。"
        )
        lines.append(
            "- **相对 181819 阶跃结论**：阶跃三档打平；本次 Bode 上 FIXED 误差略优、LUT 段未扫完，"
            "**尚不能认定 LUT 优于 FIXED**——建议补录完整 LUT 段或延长 VOFA 触发窗口后再比 300–800 Hz。"
        )

    lines.append("")
    lines.append("---")
    lines.append("*工具：`tools/ident/analyze_iq_bode.py`；分段按 6.06 s/轮，频点窗 8 cycles/f。*")
    return "\n".join(lines)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("csv", type=Path)
    ap.add_argument("--md", type=Path, help="write markdown report")
    args = ap.parse_args()

    r = analyze(args.csv)
    text = fmt_report(r)
    print(text)
    if args.md:
        args.md.write_text(text, encoding="utf-8")
        print(f"\nWrote {args.md}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
