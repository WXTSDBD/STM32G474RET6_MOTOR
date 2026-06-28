#!/usr/bin/env python3
"""FFT / STFT 特征分析：Iq Bode 扫频 + Iq 阶跃响应。

Bode：STFT 看激励/反馈主频随时间；逐频窗 FFT 与单频相关法对比；|G(jw)|、-3dB。
阶跃：Tr/Ts + 阶跃段 FFT 估带宽/振铃主频；OFF/FIXED/LUT 分段。

通道：ch3=Iq  ch4=Iq_ref  ch5=Uq  Fs=20kHz

用法:
  python tools/ident/analyze_iq_fft_features.py VOFA+CSV/.../vofa+202606282007.csv --mode auto
  python tools/ident/analyze_iq_fft_features.py file.csv --mode bode --md out.md
  python tools/ident/analyze_iq_fft_features.py file.csv --mode step
"""
from __future__ import annotations

import argparse
import math
import sys
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np
from scipy.signal import find_peaks, stft

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "tools" / "ident"))

from analyze_id_cal_sweep_vofa import load_csv  # noqa: E402
from analyze_iq_bode import (  # noqa: E402
    AMP,
    BIAS,
    CYCLES,
    F0,
    F1,
    FREQS,
    FS,
    RATIO,
    SWEEP_S,
    find_bode_start,
    find_id_cal_end,
    find_round_starts,
    fund_at,
)
from analyze_iq_step import (  # noqa: E402
    DEFAULT_FIXED_ROUNDS,
    DEFAULT_OFF_ROUNDS,
    DEFAULT_ROUNDS,
    RISES_PER_ROUND,
    analyze_step,
    find_edges,
    is_rise_from_zero,
    next_edge_index,
)

G3 = 1.0 / math.sqrt(2.0)


def detect_mode(ch4: np.ndarray, after: int) -> str:
    """Heuristic: Bode has sustained 0.25±sin on ch4; step has repeated 0->I edges."""
    n = len(ch4)
    bode_try = find_bode_start(ch4, after)
    if bode_try < n - int(FS):
        w = int(0.5 * FS)
        seg = ch4[bode_try : bode_try + w]
        if 0.12 < seg.mean() < 0.38 and seg.std() > 0.015:
            return "bode"
    steps = find_edges(ch4)
    rises = sum(1 for _, k, _, _ in steps if is_rise_from_zero(k))
    if rises >= 6:
        return "step"
    return "bode" if bode_try > after + int(FS) else "unknown"


def find_bode_done(ch4: np.ndarray, bode_start: int) -> int:
    for i in range(int(25 * FS), len(ch4)):
        if ch4[i] == 0.0 and ch4[i - 1] > 0.15:
            return i
    return len(ch4) - 1


def stft_peak_freq(
    x: np.ndarray,
    fs: float = FS,
    f_lo: float = 8.0,
    f_hi: float = 900.0,
) -> tuple[np.ndarray, np.ndarray]:
    """Dominant positive-frequency peak per STFT column (Hz), band-limited."""
    nperseg = min(4096, max(512, len(x) // 40))
    if len(x) < nperseg * 2:
        nperseg = max(256, len(x) // 4)
    f, t, z = stft(x - np.mean(x), fs=fs, nperseg=nperseg, noverlap=nperseg * 3 // 4)
    mag = np.abs(z)
    fp = np.full(len(t), np.nan)
    for k in range(len(t)):
        band = (f >= f_lo) & (f <= f_hi)
        col = mag[band, k]
        ff = f[band]
        if col.size == 0 or np.max(col) < 1e-9:
            continue
        i = int(np.argmax(col))
        fp[k] = ff[i]
    return t, fp


def fft_mag_phase_at(x: np.ndarray, f_hz: float, fs: float = FS) -> tuple[float, float]:
    """Single-bin DFT magnitude & phase (same as fund_at)."""
    if len(x) < 8:
        return 0.0, 0.0
    tt = np.arange(len(x)) / fs
    w = 2 * np.pi * f_hz
    c = np.cos(w * tt)
    s = np.sin(w * tt)
    z = 2 * np.mean(x * c) + 1j * 2 * np.mean(x * s)
    return abs(z), float(np.angle(z, deg=True))


def fft_full_peaks(x: np.ndarray, fs: float = FS, fmin: float = 5.0, fmax: float = 2000.0) -> list[tuple[float, float]]:
    """Top FFT peaks in band (freq_Hz, magnitude)."""
    x = x - np.mean(x)
    n = len(x)
    if n < 64:
        return []
    win = np.hanning(n)
    spec = np.fft.rfft(x * win)
    freqs = np.fft.rfftfreq(n, 1.0 / fs)
    mag = np.abs(spec) * 2.0 / n
    mask = (freqs >= fmin) & (freqs <= fmax)
    if not np.any(mask):
        return []
    peaks, props = find_peaks(mag[mask], prominence=np.max(mag[mask]) * 0.08)
    idx = np.where(mask)[0][peaks]
    pairs = [(float(freqs[i]), float(mag[i])) for i in idx]
    pairs.sort(key=lambda p: p[1], reverse=True)
    return pairs[:5]


def bw3_from_gcurve(freqs: list[float], mags: list[float]) -> float:
    for i in range(len(mags) - 1):
        if mags[i] >= G3 and mags[i + 1] < G3:
            t = (G3 - mags[i]) / (mags[i + 1] - mags[i])
            return freqs[i] + t * (freqs[i + 1] - freqs[i])
    return float("nan")


def analyze_bode_round(
    ch3: np.ndarray,
    ch4: np.ndarray,
    ch5: np.ndarray,
    a: int,
    b: int,
    label: str,
) -> dict:
    ref = ch4[a:b]
    iq = ch3[a:b]
    dur = (b - a) / FS

    # STFT on ref (AC) and iq
    ref_ac = ref - BIAS
    t_stft, f_peak_ref = stft_peak_freq(ref_ac)
    t_stft_iq, f_peak_iq = stft_peak_freq(iq - np.mean(iq))

    # Schedule-aligned bins
    tick = a
    rows: list[dict] = []
    for f_exp in FREQS:
        nw = int((CYCLES / f_exp) * FS + 0.5)
        i0, i1 = tick, tick + nw
        if i1 > b:
            break
        seg_ref = ch4[i0:i1] - BIAS
        seg_iq = ch3[i0:i1]
        mag_r, ph_r = fft_mag_phase_at(seg_ref, f_exp)
        mag_i, ph_i = fft_mag_phase_at(seg_iq, f_exp)
        g_lin = mag_i / mag_r if mag_r > 1e-9 else float("nan")
        # wide FFT peaks on ref (sanity: should match f_exp)
        peaks_ref = fft_full_peaks(seg_ref, fmin=f_exp * 0.7, fmax=f_exp * 1.3)
        rows.append(
            {
                "f": f_exp,
                "G": g_lin,
                "G_db": 20 * math.log10(g_lin) if g_lin > 0 else float("nan"),
                "ph_deg": ph_i - ph_r,
                "fft_peak_ref": peaks_ref[0][0] if peaks_ref else float("nan"),
            }
        )
        tick = i1

    freqs = [r["f"] for r in rows]
    gmag = [r["G"] for r in rows]
    f_last = freqs[-1] if freqs else float("nan")

    # STFT: median peak in last 0.5 s (sweep end indicator)
    t_end = dur - 0.5
    if len(t_stft) > 0:
        mask_end = (t_stft >= max(0, t_end)) & np.isfinite(f_peak_ref) & (f_peak_ref < 200)
        f_end_ref = float(np.nanmedian(f_peak_ref[mask_end])) if np.any(mask_end) else float("nan")
    else:
        f_end_ref = float("nan")

    return {
        "label": label,
        "dur_s": dur,
        "n_bins": len(rows),
        "f_last": f_last,
        "f_end_stft": f_end_ref,
        "bw3_hz": bw3_from_gcurve(freqs, gmag),
        "G_10": gmag[0] if gmag else float("nan"),
        "rows": rows,
        "stft_t": t_stft,
        "stft_f_ref": f_peak_ref,
        "stft_f_iq": f_peak_iq,
    }


def fmt_stft_trace(t: np.ndarray, f: np.ndarray, n: int = 12) -> str:
    if len(t) == 0:
        return "(无)"
    idx = np.linspace(0, len(t) - 1, min(n, len(t)), dtype=int)
    parts = []
    for i in idx:
        if np.isfinite(f[i]):
            parts.append(f"{t[i]:5.2f}s→{f[i]:5.0f}Hz")
    return " | ".join(parts[:8]) + (" …" if len(parts) > 8 else "")


def analyze_step_fft(
    iq_fb: np.ndarray,
    t: np.ndarray,
    i0: int,
    to_v: float,
    i_end: int,
    tr_ms: float = float("nan"),
) -> dict | None:
    win = min(int(0.12 * FS), i_end - i0)
    if win < int(0.03 * FS):
        return None
    seg = iq_fb[i0 : i0 + win]
    # ring: FFT on tail after peak (skip initial rise)
    peak_i = int(np.argmax(seg))
    tail_start = peak_i + int(0.005 * FS)
    tail_end = min(len(seg), tail_start + int(0.06 * FS))
    if tail_end - tail_start < int(0.02 * FS):
        tail_start = max(0, peak_i)
        tail_end = min(len(seg), tail_start + int(0.05 * FS))
    err = seg[tail_start:tail_end] - to_v
    if len(err) < 64:
        err = seg - to_v
    err = err - np.linspace(err[0], err[-1], len(err))

    peaks = fft_full_peaks(err, fmin=15.0, fmax=600.0)
    ring_f = peaks[0][0] if peaks else float("nan")
    ring_mag = peaks[0][1] if peaks else float("nan")

    f_bw_step = 350.0 / tr_ms if tr_ms > 0.1 else float("nan")

    return {
        "ring_hz": ring_f,
        "ring_mag": ring_mag,
        "tr_ms": tr_ms,
        "f_bw_step_hz": f_bw_step,
        "peaks": peaks,
    }


def run_bode(path: Path) -> str:
    ia, ib, ic, ch3, ch4, ch5 = load_csv(path)
    n = len(ch3)
    t = np.arange(n) / FS
    id_end = find_id_cal_end(ch4)
    bode_start = find_bode_start(ch4, id_end)
    bode_end = find_bode_done(ch4, bode_start)
    starts = find_round_starts(bode_start, ch4, bode_end + 1)
    labels = ["OFF", "FIXED", "LUT"]

    lines = [
        f"# FFT/STFT 特征分析 — Bode — `{path.name}`\n",
        f"- Fs={FS:.0f} Hz，Bode **{t[bode_start]:.2f}–{t[bode_end]:.2f} s**（{ (bode_end-bode_start)/FS:.2f} s）",
        f"- 理论单轮 **{SWEEP_S:.2f} s / {len(FREQS)} 点**，激励 `Iq_ref={BIAS}±{AMP} A`\n",
        "## 方法说明\n",
        "1. **STFT 主频轨迹**：看 `Iq_ref` 正弦频率是否按 10→800 Hz ×1.15 走；截断时末段主频停在中低频。",
        "2. **逐频窗单频 FFT**（与 `analyze_iq_bode.py` 相同）：每点 8 周期 Hann 等效相关，得 |G|=|Iq|/|Iq_ref|。",
        "3. **宽窗 FFT 峰**：验证该窗内能量是否集中在预期频点。\n",
    ]

    rounds = []
    for k, lbl in enumerate(labels):
        a, b = starts[k], starts[k + 1]
        rounds.append(analyze_bode_round(ch3, ch4, ch5, a, b, lbl))

    lines.append("## 三轮摘要\n")
    lines.append("| 组 | 时长(s) | 完成频点 | 末点f(调度) | STFT末段主频 | |G|@10Hz | -3dB |")
    lines.append("|----|---------|----------|-------------|--------------|---------|------|")
    for r in rounds:
        lines.append(
            f"| **{r['label']}** | {r['dur_s']:.2f} | {r['n_bins']}/{len(FREQS)} | "
            f"{r['f_last']:.0f} Hz | {r['f_end_stft']:.0f} Hz | {r['G_10']:.3f} | "
            f"{r['bw3_hz']:.0f} Hz |"
        )
    lines.append("")

    for r in rounds:
        lines.append(f"### {r['label']} — STFT 主频轨迹（Iq_ref AC）\n")
        t0 = starts[labels.index(r["label"])] / FS
        stft_t = r["stft_t"] + t0
        lines.append(f"- {fmt_stft_trace(stft_t, r['stft_f_ref'])}\n")

        lines.append(f"### {r['label']} — 逐频 |G| 与 FFT 校验（前 12 + 末 4 点）\n")
        lines.append("| f(Hz) | |G| | dB | ∠(deg) | FFT峰(ref) |")
        lines.append("|-------|-----|-----|--------|------------|")
        show = r["rows"][:12] + (r["rows"][-4:] if len(r["rows"]) > 16 else [])
        seen = set()
        for row in show:
            if row["f"] in seen:
                continue
            seen.add(row["f"])
            fp = row["fft_peak_ref"]
            fp_s = f"{fp:.1f}" if np.isfinite(fp) else "—"
            lines.append(
                f"| {row['f']:.1f} | {row['G']:.3f} | {row['G_db']:+.1f} | {row['ph_deg']:+.1f} | {fp_s} |"
            )
        lines.append("")

    lines.append("## 解读提示\n")
    lut = rounds[2]
    if lut["n_bins"] < len(FREQS):
        lines.append(
            f"- **LUT 轮 STFT 末段主频 ~{lut['f_end_stft']:.0f} Hz**，调度末点 **{lut['f_last']:.0f} Hz** — "
            f"与 ident 提前 DONE（{lut['n_bins']}/{len(FREQS)} 点）一致。"
        )
    lines.append("- **-3dB** 来自 |G(f)| 曲线，表示闭环 `Iq/Iq_ref` 跟踪带宽，不是 PI 设计 fc。")
    lines.append("- 高频 |G|>1 多为 PWM/谐波泄漏，以 <150 Hz 段为准。\n")
    return "\n".join(lines)


def run_step(path: Path) -> str:
    ia, ib, ic, iq_fb, iq_ref, uq = load_csv(path)
    n = len(iq_ref)
    t = np.arange(n) / FS
    steps = find_edges(iq_ref)
    edge_idx = [s[0] for s in steps]

    lines = [
        f"# FFT/STFT 特征分析 — 阶跃 — `{path.name}`\n",
        f"- Fs={FS:.0f} Hz，时长 **{t[-1]:.2f} s**",
        "- 通道：ch3=**Iq** ch4=**Iq_ref** ch5=**Uq**\n",
        "## 方法说明\n",
        "1. **时域**：Tr（10→90%）、Ts、超调 σ%（同 `analyze_iq_step.py`）。",
        "2. **阶跃后 80 ms 误差 FFT**：去斜坡后找主振铃频率。",
        "3. **带宽粗估**：`f_bw ≈ 0.35 / Tr`（一阶阶跃近似，单位 Hz / Tr_ms→×1000）。\n",
    ]

    off_n = DEFAULT_OFF_ROUNDS * RISES_PER_ROUND
    fix_n = DEFAULT_FIXED_ROUNDS * RISES_PER_ROUND
    rise_idx = 0
    buckets: dict[str, list] = {"OFF": [], "FIXED": [], "LUT": []}

    for i, kind, fv, tv in steps:
        if not is_rise_from_zero(kind):
            continue
        rise_idx += 1
        pos = edge_idx.index(i)
        i_end = next_edge_index(edge_idx, pos, n)
        td = analyze_step(ia, ib, ic, iq_fb, uq, t, i, fv, tv, i_end)
        fd = analyze_step_fft(iq_fb, t, i, tv, i_end, tr_ms=td.get("tr_ms", float("nan")))
        if not td:
            continue
        if not fd:
            fd = {"ring_hz": float("nan"), "f_bw_step_hz": float("nan"), "tr_ms": td.get("tr_ms", float("nan"))}
        rec = {**td, **fd, "kind": kind}
        if rise_idx <= off_n:
            buckets["OFF"].append(rec)
        elif rise_idx <= off_n + fix_n:
            buckets["FIXED"].append(rec)
        else:
            buckets["LUT"].append(rec)

    lines.append("## 分段 FFT + 时域特征（0 起点上升沿均值）\n")
    lines.append("| 组 | 阶跃 | n | Tr(ms) | f_bw≈0.35/Tr(Hz) | 振铃主频(Hz) | σ% | err% |")
    lines.append("|----|------|---|--------|------------------|--------------|-----|------|")

    for lbl in ("OFF", "FIXED", "LUT"):
        if not buckets[lbl]:
            continue
        by_k: dict[str, list] = defaultdict(list)
        for r in buckets[lbl]:
            by_k[r["kind"]].append(r)
        for kind, grp in sorted(by_k.items(), key=lambda x: float(x[0].split("->")[1])):
            tr = float(np.nanmean([g["tr_ms"] for g in grp]))
            fb = float(np.nanmean([g["f_bw_step_hz"] for g in grp if np.isfinite(g["f_bw_step_hz"])]))
            ring = float(np.nanmean([g["ring_hz"] for g in grp if np.isfinite(g["ring_hz"])]))
            os_ = float(np.mean([g["overshoot_pct"] for g in grp]))
            err = float(np.mean([g["err_pct"] for g in grp]))
            lines.append(
                f"| **{lbl}** | {kind} | {len(grp)} | {tr:.2f} | {fb:.0f} | {ring:.0f} | {os_:+.1f} | {err:.2f} |"
            )
    lines.append("")

    lines.append("## 解读提示\n")
    lines.append("- **f_bw≈0.35/Tr** 与 Bode -3dB 同量级时，说明阶跃与扫频看到同一「有效带宽」。"
                 "若 Tr~10 ms → f_bw~35 Hz，与 Bode ~40 Hz 一致。")
    lines.append("- **振铃主频**：误差 FFT 峰；大超调时常在 30–100 Hz，反映欠阻尼而非真实开环 fc。\n")
    return "\n".join(lines)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("csv", type=Path)
    ap.add_argument("--mode", choices=("auto", "bode", "step"), default="auto")
    ap.add_argument("--md", type=Path, help="write markdown report")
    args = ap.parse_args()

    ia, ib, ic, ch3, ch4, ch5 = load_csv(args.csv)
    id_end = find_id_cal_end(ch4)
    mode = args.mode
    if mode == "auto":
        mode = detect_mode(ch4, id_end)

    if mode == "step":
        text = run_step(args.csv)
    elif mode == "bode":
        text = run_bode(args.csv)
    else:
        text = f"无法识别模式，请指定 --mode bode 或 step\n"

    print(text)
    if args.md:
        args.md.parent.mkdir(parents=True, exist_ok=True)
        args.md.write_text(text, encoding="utf-8")
        print(f"\nWrote {args.md}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
