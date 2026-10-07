#!/usr/bin/env python3
"""
Detailed speed-loop analysis for VOFA 12ch CSV (M1 speed profile bringup).

Typical speed-loop evaluation checklist (implemented here):
  1. Steady-state tracking: SSE mean/std, percentiles, % within band
  2. Step response (profile edges): rise time, overshoot, settling time, peak error
  3. Torque / load: Iq_mean, iq_ref_mean, saturation duty
  4. Inner-loop tracking: Iq vs iq_ref delay/gain (cross-corr), RMS tracking error
  5. Spectral: omega_err & Iq FFT peaks; compare to speed-loop rate (2 kHz) & PI Fn
  6. Limit-cycle / hunting: Iq sign changes, peak-to-peak vs load level
  7. d-axis coupling: Id std vs speed
  8. Fair multi-file compare on first profile cycle

Reuses segmentation/outlier logic from analyze_speed_loop_v2.py
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

# import shared helpers from v2
sys.path.insert(0, str(Path(__file__).resolve().parent))
from analyze_speed_loop_v2 import (  # noqa: E402
    CH,
    FS_HZ_DEFAULT,
    I_SAT_A,
    PROFILE_RPMS,
    HoldStats,
    analyze_file,
    build_outlier_mask,
    debounced_holds,
    first_cycle_holds,
    hold_stats,
    load_csv,
    steady_slice,
)

# firmware nominal (motor_params_m1.h)
SPEED_LOOP_HZ = 2000.0
SPEED_PI_FN_HZ = 20.0
SPEED_PI_KP = 0.015
SPEED_PI_KI = 0.002
PROFILE_HOLD_S = 10.0
I_REF_MAX_A = 5.0


@dataclass
class StepMetrics:
    from_rpm: int
    to_rpm: int
    t_step_s: float
    delta_rpm: float
    rise_time_s: float | None      # 10% -> 90% of delta
    overshoot_rpm: float
    overshoot_pct: float
    peak_err_rpm: float
    settle_time_s: float | None    # enter ±30 rpm band
    sse_mean_rpm: float            # mean err last 3s of 5s post-step window
    sse_std_rpm: float
    iq_ref_peak_a: float
    sat_during_step_pct: float


@dataclass
class SpectrumPeaks:
    signal: str
    rpm: int
    peaks_hz: list[tuple[float, float]]  # (freq_hz, magnitude)


@dataclass
class InnerLoopMetrics:
    rpm: int
    iq_track_rms_a: float
    iq_track_p95_a: float
    corr_iq_iqref: float
    lag_ms_at_max_xcorr: float
    iq_ref_to_iq_gain: float


@dataclass
class DetailedReport:
    path: str
    holds: list[HoldStats] = field(default_factory=list)
    steps: list[StepMetrics] = field(default_factory=list)
    spectra: list[SpectrumPeaks] = field(default_factory=list)
    inner: list[InnerLoopMetrics] = field(default_factory=list)


def _clean_indices(mask: np.ndarray, sl: slice) -> np.ndarray:
    idx = np.arange(sl.start, sl.stop)
    return idx[~mask[sl]]


def find_profile_steps(
    cols: dict[str, np.ndarray],
    fs: float,
    *,
    min_delta: float = 150.0,
) -> list[tuple[float, int, int]]:
    """Return [(t_step_s, from_rpm, to_rpm), ...] on debounced omega_ref."""
    wref = cols["omega_ref"]
    ladder = np.array(PROFILE_RPMS, dtype=float)
    near = np.argmin(np.abs(wref[:, None] - ladder[None, :]), axis=1)
    rpm = ladder[near]
    valid = np.abs(wref - rpm) <= 50.0
    rpm = np.where(valid, rpm, np.nan)

    steps: list[tuple[float, int, int]] = []
    debounce = int(0.2 * fs)
    i = 1
    while i < len(rpm):
        if np.isnan(rpm[i]) or np.isnan(rpm[i - 1]):
            i += 1
            continue
        if abs(rpm[i] - rpm[i - 1]) >= min_delta:
            t = i / fs
            fr, to = int(rpm[i - 1]), int(rpm[i])
            if steps and (t - steps[-1][0]) < 0.5:
                i += 1
                continue
            steps.append((t, fr, to))
            i += debounce
        i += 1
    return steps


def analyze_step(
    cols: dict[str, np.ndarray],
    mask: np.ndarray,
    t_step: float,
    from_rpm: int,
    to_rpm: int,
    fs: float,
    *,
    post_window_s: float = 5.0,
) -> StepMetrics:
    i0 = int(t_step * fs)
    i1 = min(len(cols["omega_pll"]), i0 + int(post_window_s * fs))
    w = cols["omega_pll"][i0:i1]
    wref = cols["omega_ref"][i0:i1]
    iqr = cols["iq_ref"][i0:i1]
    clean = ~mask[i0:i1]
    err = wref - w

    delta = float(to_rpm - from_rpm)
    base = float(from_rpm)
    target = float(to_rpm)

    # use cleaned samples where possible
    wc = w.copy()
    wc[~clean] = np.interp(np.flatnonzero(~clean), np.flatnonzero(clean), w[clean]) if np.any(clean) else w

    # rise time 10-90%
    y = wc - base
    norm = y / delta if abs(delta) > 1e-6 else y
    rise_time = None
    if abs(delta) > 1e-6:
        i10 = i90 = None
        for k in range(len(norm)):
            if i10 is None and norm[k] >= 0.10:
                i10 = k
            if norm[k] >= 0.90:
                i90 = k
                break
        if i10 is not None and i90 is not None and i90 > i10:
            rise_time = (i90 - i10) / fs

    overshoot_rpm = 0.0
    overshoot_pct = 0.0
    if delta > 0:
        overshoot_rpm = float(np.max(wc) - target)
    else:
        overshoot_rpm = float(target - np.min(wc))
    if abs(delta) > 1e-6:
        overshoot_pct = 100.0 * overshoot_rpm / abs(delta)

    peak_err = float(np.max(np.abs(err[clean]))) if np.any(clean) else float(np.max(np.abs(err)))

    settle_time = None
    band = 30.0
    for k in range(len(wc)):
        if abs(wc[k] - target) <= band:
            if np.all(np.abs(wc[k:] - target) <= band):
                settle_time = k / fs
                break

    tail = slice(max(0, len(wc) - int(3.0 * fs)), len(wc))
    err_tail = err[tail]
    err_tail = err_tail[~mask[i0:i1][tail]] if np.any(~mask[i0:i1][tail]) else err_tail

    sat = float(np.mean(np.abs(iqr) >= I_SAT_A - 0.01) * 100.0)

    return StepMetrics(
        from_rpm=from_rpm,
        to_rpm=to_rpm,
        t_step_s=t_step,
        delta_rpm=delta,
        rise_time_s=rise_time,
        overshoot_rpm=overshoot_rpm,
        overshoot_pct=overshoot_pct,
        peak_err_rpm=peak_err,
        settle_time_s=settle_time,
        sse_mean_rpm=float(np.mean(err_tail)),
        sse_std_rpm=float(np.std(err_tail)),
        iq_ref_peak_a=float(np.max(np.abs(iqr[clean])) if np.any(clean) else np.max(np.abs(iqr))),
        sat_during_step_pct=sat,
    )


def fft_peaks(x: np.ndarray, fs: float, *, n_peaks: int = 5, fmin: float = 0.2) -> list[tuple[float, float]]:
    x0 = x - np.mean(x)
    if len(x0) < 512:
        return []
    win = np.hanning(len(x0))
    spec = np.abs(np.fft.rfft(x0 * win))
    freqs = np.fft.rfftfreq(len(x0), 1.0 / fs)
    peaks: list[tuple[float, float]] = []
    for k in range(1, len(spec) - 1):
        if freqs[k] < fmin:
            continue
        if spec[k] >= spec[k - 1] and spec[k] > spec[k + 1]:
            peaks.append((float(freqs[k]), float(spec[k])))
    peaks.sort(key=lambda p: p[1], reverse=True)
    return peaks[:n_peaks]


def analyze_inner_loop(
    cols: dict[str, np.ndarray],
    mask: np.ndarray,
    rpm: int,
    i0: int,
    i1: int,
    fs: float,
    *,
    lead_in_s: float,
    tail_s: float,
) -> InnerLoopMetrics:
    sl = steady_slice(i0, i1, fs, lead_in_s=lead_in_s, tail_s=tail_s)
    iq = cols["Iq"][sl]
    iqr = cols["iq_ref"][sl]
    clean = ~mask[sl]
    if np.sum(clean) < 100:
        clean = np.ones(len(iq), dtype=bool)

    iq_c = iq[clean]
    iqr_c = iqr[clean]
    track = iq_c - iqr_c

    # cross-correlation lag (iq_ref leads)
    n = min(len(iq_c), 20000)
    a = iqr_c[:n] - np.mean(iqr_c[:n])
    b = iq_c[:n] - np.mean(iq_c[:n])
    corr = np.correlate(b, a, mode="full")
    lags = np.arange(-len(a) + 1, len(a))
    kmax = int(np.argmax(corr))
    lag_ms = lags[kmax] / fs * 1000.0

    # quasi-static gain
    denom = float(np.dot(iqr_c, iqr_c))
    gain = float(np.dot(iq_c, iqr_c) / denom) if denom > 1e-9 else 0.0

    c = np.corrcoef(iq_c, iqr_c)[0, 1] if len(iq_c) > 2 else 0.0

    return InnerLoopMetrics(
        rpm=rpm,
        iq_track_rms_a=float(np.sqrt(np.mean(track ** 2))),
        iq_track_p95_a=float(np.percentile(np.abs(track), 95)),
        corr_iq_iqref=float(c),
        lag_ms_at_max_xcorr=lag_ms,
        iq_ref_to_iq_gain=gain,
    )


def analyze_detailed(
    path: Path,
    fs: float,
    *,
    lead_in_s: float = 2.0,
    tail_s: float = 0.5,
) -> DetailedReport:
    cols = load_csv(path)
    mask = build_outlier_mask(cols)
    holds = first_cycle_holds(debounced_holds(cols["omega_ref"], fs))

    rep = DetailedReport(path=str(path))

    for rpm, i0, i1 in holds:
        if rpm not in PROFILE_RPMS:
            continue
        hs = hold_stats(cols, mask, rpm, i0, i1, fs,
                        lead_in_s=lead_in_s, tail_s=tail_s)
        rep.holds.append(hs)

        sl = steady_slice(i0, i1, fs, lead_in_s=lead_in_s, tail_s=tail_s)
        clean = ~mask[sl]
        err = cols["omega_ref"][sl] - cols["omega_pll"][sl]
        iq = cols["Iq"][sl]
        if np.sum(clean) > 512:
            err = err[clean]
            iq = iq[clean]

        rep.spectra.append(SpectrumPeaks("omega_err", rpm, fft_peaks(err, fs)))
        rep.spectra.append(SpectrumPeaks("Iq", rpm, fft_peaks(iq, fs)))
        rep.inner.append(analyze_inner_loop(
            cols, mask, rpm, i0, i1, fs, lead_in_s=lead_in_s, tail_s=tail_s))

    for t_step, fr, to in find_profile_steps(cols, fs):
        if fr in PROFILE_RPMS and to in PROFILE_RPMS:
            rep.steps.append(analyze_step(cols, mask, t_step, fr, to, fs))

    return rep


def format_markdown(reports: list[DetailedReport], labels: list[str]) -> str:
    lines: list[str] = []
    lines.append("# 速度环详细分析报告\n")
    lines.append("## 分析框架（速度环一般看什么）\n")
    lines.append("| 类别 | 指标 | 意义 |")
    lines.append("|------|------|------|")
    lines.append("| **稳态跟踪** | SSE mean/std, \\|e\\|<30 rpm 占比 | 静差与稳态波动 |")
    lines.append("| **阶跃响应** | tr, ts, overshoot, peak err | 带宽/阻尼是否合适 |")
    lines.append("| **扭矩链** | iq_ref mean, Iq mean, 饱和率 | 负载与限幅 |")
    lines.append("| **内环跟随** | Iq−iq_ref RMS, 相关, 滞后 | 电流环是否拖速度环后腿 |")
    lines.append("| **频域** | ω_err / Iq 主峰 | 极限环、机械共振、拍频 |")
    lines.append("| **d 轴** | Id std | 弱磁/解耦/角度误差 |")
    lines.append("")
    lines.append(
        f"**本机名义参数**: 速度环 {SPEED_LOOP_HZ:.0f} Hz, "
        f"PI Fn≈{SPEED_PI_FN_HZ} Hz, Kp={SPEED_PI_KP}, Ki={SPEED_PI_KI}, "
        f"I_lim=±{I_REF_MAX_A} A, profile hold={PROFILE_HOLD_S} s\n"
    )

    for rep, label in zip(reports, labels):
        name = Path(rep.path).name
        lines.append(f"---\n\n## {label} (`{name}`)\n")

        lines.append("### 1. 稳态 hold（skip 前2s + 后0.5s，已滤野点）\n")
        lines.append(
            "| rpm | ω mean | ω σ | |e|<20 | |e|<30 | e_p95 | "
            "Iq | Iq σ | iq_ref | sat% | Id σ | Iq<0% | ripple Hz |"
        )
        lines.append("|-----|--------|-----|-------|-------|-------|"
                     "-----|------|--------|------|------|-------|-----------|")
        for h in rep.holds:
            rip = next((s.peaks_hz[0][0] for s in rep.spectra
                        if s.signal == "Iq" and s.rpm == h.rpm and s.peaks_hz), None)
            rip_s = f"{rip:.1f}" if rip is not None else "-"
            lines.append(
                f"| {h.rpm} | {h.omega_pll_mean:.1f} | {h.omega_pll_std:.1f} | "
                f"{h.err_lt20_pct:.0f}% | {h.err_lt30_pct:.0f}% | {h.err_p95_abs:.0f} | "
                f"{h.iq_mean:.2f} | {h.iq_std:.2f} | {h.iq_ref_mean:.2f} | "
                f"{h.iq_ref_sat_pct:.1f} | {h.id_std:.3f} | {h.iq_neg_pct:.0f}% | {rip_s} |"
            )

        lines.append("\n### 2. 阶跃响应（profile 换档后 5 s 窗）\n")
        if rep.steps:
            lines.append(
                "| 阶跃 | t(s) | Δrpm | tr(s) | OS% | peak\\|e\\| | ts@30rpm | "
                "SSE μ(末3s) | SSE σ | iq_ref_pk | sat% |"
            )
            lines.append("|------|------|------|-------|-----|---------|"
                         "----------|---------|-------|-----------|------|")
            for s in rep.steps:
                tr = f"{s.rise_time_s:.2f}" if s.rise_time_s is not None else "-"
                ts = f"{s.settle_time_s:.2f}" if s.settle_time_s is not None else ">5"
                lines.append(
                    f"| {s.from_rpm}→{s.to_rpm} | {s.t_step_s:.1f} | {s.delta_rpm:+.0f} | "
                    f"{tr} | {s.overshoot_pct:.0f} | {s.peak_err_rpm:.0f} | {ts} | "
                    f"{s.sse_mean_rpm:+.1f} | {s.sse_std_rpm:.1f} | {s.iq_ref_peak_a:.2f} | "
                    f"{s.sat_during_step_pct:.1f} |"
                )
        else:
            lines.append("_未检测到有效阶跃_\n")

        lines.append("\n### 3. 内环跟随（Iq vs iq_ref）\n")
        lines.append("| rpm | track RMS | track p95 | corr | lag@xcorr ms | Iq/iq_ref gain |")
        lines.append("|-----|-----------|-----------|------|--------------|----------------|")
        for inn in rep.inner:
            lines.append(
                f"| {inn.rpm} | {inn.iq_track_rms_a:.3f} | {inn.iq_track_p95_a:.3f} | "
                f"{inn.corr_iq_iqref:.3f} | {inn.lag_ms_at_max_xcorr:+.1f} | "
                f"{inn.iq_ref_to_iq_gain:.3f} |"
            )

        lines.append("\n### 4. 频谱主峰（稳态段，Hanning FFT）\n")
        for rpm in PROFILE_RPMS:
            lines.append(f"**{rpm} rpm**")
            for sig in ("omega_err", "Iq"):
                sp = next((s for s in rep.spectra if s.rpm == rpm and s.signal == sig), None)
                if sp and sp.peaks_hz:
                    tops = ", ".join(f"{f:.2f}Hz({m:.1g})" for f, m in sp.peaks_hz[:3])
                    lines.append(f"- {sig}: {tops}")
            lines.append("")

        # brief diagnosis
        lines.append("### 5. 自动诊断摘要\n")
        diag = _auto_diagnose(rep)
        for d in diag:
            lines.append(f"- {d}")
        lines.append("")

    if len(reports) == 2:
        lines.append("---\n\n## A/B 对比要点\n")
        lines.extend(_compare_summary(reports[0], reports[1], labels[0], labels[1]))

    return "\n".join(lines)


def _auto_diagnose(rep: DetailedReport) -> list[str]:
    out: list[str] = []
    by_rpm = {h.rpm: h for h in rep.holds}

    if 100 in by_rpm and by_rpm[100].err_lt30_pct >= 95:
        out.append("100 rpm 稳态跟踪优秀。")
    if 300 in by_rpm:
        h = by_rpm[300]
        if h.iq_std > 1.0 and h.iq_neg_pct > 20:
            out.append(
                f"300 rpm：Iq 大幅双向波动（σ={h.iq_std:.2f}A, Iq<0 占 {h.iq_neg_pct:.0f}%），"
                "疑为速度环极限环或机械共振，非单纯测量噪声。"
            )
        elif h.iq_std < 0.5:
            out.append("300 rpm 电流波动正常。")

    sat_holds = [h for h in rep.holds if h.iq_ref_sat_pct > 1.0]
    if sat_holds:
        rpms = ", ".join(str(h.rpm) for h in sat_holds)
        out.append(f"稳态 iq_ref 饱和 >1% 的档位: {rpms} rpm — 考虑提高 I_lim 或降 Kp。")
    else:
        out.append("各档稳态未明显触电流饱和限幅。")

    for inn in rep.inner:
        if inn.iq_track_rms_a > 0.5 and inn.corr_iq_iqref > 0.5:
            out.append(
                f"{inn.rpm} rpm 内环跟踪 RMS={inn.iq_track_rms_a:.2f}A "
                f"(corr={inn.corr_iq_iqref:.2f}) — 电流环带宽/限幅可能限制速度环。"
            )

    for s in rep.steps:
        if s.settle_time_s is None and abs(s.delta_rpm) >= 150:
            out.append(
                f"阶跃 {s.from_rpm}→{s.to_rpm} 在 5s 内未 settling 到 ±30 rpm — "
                "带宽偏低或负载扰动大。"
            )
        if s.overshoot_pct > 25:
            out.append(f"阶跃 {s.from_rpm}→{s.to_rpm} 超调 {s.overshoot_pct:.0f}% — 阻尼偏低。")

    if not out:
        out.append("未发现显著异常；可继续加负载或扫 Kp/Ki。")
    return out


def _compare_summary(a: DetailedReport, b: DetailedReport, la: str, lb: str) -> list[str]:
    lines: list[str] = []
    ba, bb = {h.rpm: h for h in a.holds}, {h.rpm: h for h in b.holds}
    for rpm in PROFILE_RPMS:
        if rpm not in ba or rpm not in bb:
            continue
        d_iq = ba[rpm].iq_mean - bb[rpm].iq_mean
        d_es = ba[rpm].err_lt30_pct - bb[rpm].err_lt30_pct
        lines.append(
            f"- **{rpm} rpm**: ΔIq_mean({la}−{lb})={d_iq:+.2f}A; "
            f"Δ|e|<30%={d_es:+.1f}pp"
        )
    return lines


def main() -> int:
    ap = argparse.ArgumentParser(description="Detailed speed-loop CSV analysis")
    ap.add_argument("csv", type=Path, nargs="+")
    ap.add_argument("--fs", type=float, default=FS_HZ_DEFAULT)
    ap.add_argument("--lead-in", type=float, default=2.0)
    ap.add_argument("--tail", type=float, default=0.5)
    ap.add_argument("-o", "--out", type=Path, default=None, help="Markdown report path")
    ap.add_argument("--labels", nargs="*", default=None)
    args = ap.parse_args()

    labels = args.labels or [p.stem for p in args.csv]
    reports = [analyze_detailed(p, args.fs, lead_in_s=args.lead_in, tail_s=args.tail)
               for p in args.csv]

    md = format_markdown(reports, labels)

    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(md, encoding="utf-8")
        print(f"Written: {args.out}", file=sys.stderr)
    else:
        sys.stdout.buffer.write(md.encode("utf-8", errors="replace"))
        sys.stdout.buffer.write(b"\n")

    return 0


if __name__ == "__main__":
    sys.exit(main())
