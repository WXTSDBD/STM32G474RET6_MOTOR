#!/usr/bin/env python3
"""
ONE_SHOT VOFA CSV analyzer (Pass0 -> speed OFF -> speed LUT).

Segmentation:
  1. Pass0 ends at first sustained omega_ref=100 with rotating theta
  2. OFF ladder: fixed 10s windows from that index (primary, trusted)
  3. LUT ladder: fixed 10s windows from second sustained omega_ref=100

Hold stats use steady middle window (lead/tail 2s). debounced_holds is
cross-checked but not used for A/B when fixed windows are available.
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from analyze_speed_loop_v2 import (  # noqa: E402
    FS_HZ_DEFAULT,
    PROFILE_RPMS,
    HoldStats,
    build_outlier_mask,
    debounced_holds,
    detect_sustained_rpm,
    fixed_profile_holds,
    hold_stats,
    load_csv,
    sanitize_omega_ref,
)

THETA_30_RAD = 0.5235987755982988
CAL_THETA_TOL_RAD = 0.05
PROFILE_HOLD_S = 10.0
ALIGN_S = 0.5
PASS0_EXPECT_S = 16.0


@dataclass
class SegmentInfo:
    cal_end_i: int
    off_start_i: int
    lut_start_i: int
    cal_method: str
    off_method: str
    lut_method: str
    warnings: list[str] = field(default_factory=list)


@dataclass
class OneShotReport:
    csv_path: str
    duration_s: float
    fs_hz: float
    outlier_pct: float
    segment: SegmentInfo
    cal: dict[str, float]
    off_holds: list[HoldStats]
    lut_holds: list[HoldStats]
    debounced_off_n: int
    debounced_lut_n: int


def detect_cal_end(cols: dict[str, np.ndarray], fs: float) -> tuple[int, str]:
    wref = cols["omega_ref"]
    theta = cols["theta_el"]

    idx = detect_sustained_rpm(wref, fs, 100.0, search_from=0, sustain_s=0.3)
    if idx is not None:
        deb = int(0.2 * fs)
        if np.std(theta[idx : idx + deb]) > 0.02:
            return idx, "omega_ref=100 + rotating theta"

    locked = np.abs(theta - THETA_30_RAD) < CAL_THETA_TOL_RAD
    for i in range(1, len(theta)):
        if locked[i - 1] and not locked[i]:
            return i, "theta leaves 30 deg lock"
    return int(PASS0_EXPECT_S * fs), f"fallback {PASS0_EXPECT_S}s"


def detect_lut_start(
    cols: dict[str, np.ndarray],
    fs: float,
    off_start_i: int,
) -> tuple[int, str]:
    wref = cols["omega_ref"]
    ladder_s = len(PROFILE_RPMS) * PROFILE_HOLD_S
    min_search = off_start_i + int((ladder_s - 1.0) * fs)
    idx = detect_sustained_rpm(wref, fs, 100.0, search_from=min_search, sustain_s=0.3)
    if idx is not None:
        return idx, "second sustained omega_ref=100 after OFF ladder"
    return off_start_i + int(ladder_s * fs), f"fallback off_start+{ladder_s}s"


def cal_metrics(cols: dict[str, np.ndarray], cal_end_i: int, fs: float) -> dict[str, float]:
    sl = slice(0, cal_end_i)
    theta = cols["theta_el"][sl]
    ud = cols["Ud"][sl]
    id_ = cols["Id"][sl]
    align_sl = slice(0, min(cal_end_i, int(ALIGN_S * fs)))
    ud_align = cols["Ud"][align_sl]
    sweep_sl = slice(int(ALIGN_S * fs), cal_end_i)

    return {
        "duration_s": cal_end_i / fs,
        "theta_mean_rad": float(np.mean(theta)),
        "theta_std_rad": float(np.std(theta)),
        "align_ud_3v_pct": float(np.mean(np.abs(ud_align - 3.0) < 0.2) * 100.0),
        "sweep_id_peak_a": float(np.max(np.abs(cols["Id"][sweep_sl]))),
        "sweep_ud_mean_v": float(np.mean(cols["Ud"][sweep_sl])),
    }


def holds_from_fixed(
    cols: dict[str, np.ndarray],
    mask: np.ndarray,
    i0: int,
    i1: int,
    fs: float,
    *,
    lead_in_s: float,
    tail_s: float,
) -> list[HoldStats]:
    ladder_end = i0 + int(len(PROFILE_RPMS) * PROFILE_HOLD_S * fs)
    if ladder_end > i1:
        ladder_end = i1
    holds = fixed_profile_holds(i0, fs, hold_s=PROFILE_HOLD_S)
    out: list[HoldStats] = []
    for rpm, a, b in holds:
        if b > i1:
            break
        st = hold_stats(cols, mask, rpm, a, b, fs, lead_in_s=lead_in_s, tail_s=tail_s)
        out.append(st)
    return out


def validate_fixed_vs_raw(
    cols: dict[str, np.ndarray],
    holds: list[tuple[int, int, int]],
    fs: float,
) -> list[str]:
    wref = cols["omega_ref"]
    warns: list[str] = []
    for rpm, i0, i1 in holds:
        dur = (i1 - i0) / fs
        mid = wref[i0 + int(2 * fs) : i1 - int(2 * fs)]
        if len(mid) == 0:
            warns.append(f"{rpm} rpm: empty mid window")
            continue
        mean_ref = float(np.mean(mid))
        if abs(mean_ref - rpm) > 35.0:
            warns.append(
                f"{rpm} rpm window misaligned: mean omega_ref={mean_ref:.0f} (dur={dur:.1f}s)"
            )
    return warns


def compare_off_lut(off: list[HoldStats], lut: list[HoldStats]) -> list[str]:
    lines = [
        "",
        "## OFF vs LUT (LUT - OFF)",
        "",
        "| rpm | d_err_std | d_Iq_std | d_Iq_pp | d_|e|95 | note |",
        "|---:|---:|---:|---:|---:|---|",
    ]
    lo = {h.rpm: h for h in off}
    for h in lut:
        o = lo.get(h.rpm)
        if o is None:
            continue
        de = h.err_std - o.err_std
        di = h.iq_std - o.iq_std
        dp = h.iq_pp - o.iq_pp
        d95 = h.err_p95_abs - o.err_p95_abs
        note = []
        if h.rpm == 300:
            if di < -0.1:
                note.append("LUT Iq ripple lower")
            elif di > 0.1:
                note.append("LUT Iq ripple higher")
            else:
                note.append("300rpm hunt ~same")
        elif abs(de) < 1.5 and abs(di) < 0.15:
            note.append("~same")
        elif de > 2:
            note.append("LUT tracking worse")
        lines.append(
            f"| {h.rpm} | {de:+.1f} | {di:+.2f} | {dp:+.2f} | {d95:+.1f} | "
            f"{', '.join(note) if note else ''} |"
        )
    return lines


def holds_table(holds: list[HoldStats], title: str) -> list[str]:
    lines = [
        "",
        f"### {title}",
        "",
        "| rpm | t0(s) | w_mean | err_std | |e|95 | <30rpm% | Iq_mean | Iq_std | Iq_pp | ripple |",
        "|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for h in holds:
        rip = f"{h.dom_ripple_hz:.1f} Hz" if h.dom_ripple_hz else "-"
        lines.append(
            f"| {h.rpm} | {h.t_start_s:.1f} | {h.omega_pll_mean:.0f} | "
            f"{h.err_std:.1f} | {h.err_p95_abs:.1f} | {h.err_lt30_pct:.1f} | "
            f"{h.iq_mean:.2f} | {h.iq_std:.2f} | {h.iq_pp:.2f} | {rip} |"
        )
    return lines


def analyze_one_shot(
    path: Path,
    fs: float = FS_HZ_DEFAULT,
    *,
    lead_in_s: float = 2.0,
    tail_s: float = 2.0,
) -> OneShotReport:
    cols = load_csv(path)
    n = len(cols["omega_ref"])
    mask = build_outlier_mask(cols)

    cal_end_i, cal_m = detect_cal_end(cols, fs)
    off_start_i = cal_end_i
    lut_start_i, lut_m = detect_lut_start(cols, fs, off_start_i)

    seg = SegmentInfo(
        cal_end_i=cal_end_i,
        off_start_i=off_start_i,
        lut_start_i=lut_start_i,
        cal_method=cal_m,
        off_method=f"fixed {PROFILE_HOLD_S}s ladder from t={off_start_i / fs:.2f}s",
        lut_method=f"{lut_m} @ t={lut_start_i / fs:.2f}s",
    )

    off_end_i = off_start_i + int(len(PROFILE_RPMS) * PROFILE_HOLD_S * fs)
    if lut_start_i < off_end_i:
        seg.warnings.append(
            f"LUT start {lut_start_i/fs:.2f}s before OFF ladder end {off_end_i/fs:.2f}s; "
            "using full OFF window"
        )
        lut_start_i = off_end_i

    off_fixed = fixed_profile_holds(off_start_i, fs, hold_s=PROFILE_HOLD_S)
    lut_fixed = fixed_profile_holds(lut_start_i, fs, hold_s=PROFILE_HOLD_S)
    seg.warnings.extend(validate_fixed_vs_raw(cols, off_fixed, fs))
    seg.warnings.extend(validate_fixed_vs_raw(cols, lut_fixed, fs))

    if len(off_fixed) < len(PROFILE_RPMS):
        seg.warnings.append(f"OFF segment truncated: only {len(off_fixed)}/5 holds")
    if cal_end_i / fs < 10.0 or cal_end_i / fs > 25.0:
        seg.warnings.append(f"Pass0 duration {cal_end_i/fs:.1f}s outside expected ~16s")

    off_holds = holds_from_fixed(
        cols, mask, off_start_i, off_end_i, fs, lead_in_s=lead_in_s, tail_s=tail_s
    )
    lut_holds = holds_from_fixed(
        cols, mask, lut_start_i, n, fs, lead_in_s=lead_in_s, tail_s=tail_s
    )

    wref_san = sanitize_omega_ref(cols["omega_ref"], fs)
    deb_off = debounced_holds(wref_san[off_start_i:off_end_i], fs, min_hold_s=5.0)
    deb_lut = debounced_holds(wref_san[lut_start_i:n], fs, min_hold_s=5.0)
    if len(deb_off) < len(PROFILE_RPMS):
        seg.warnings.append(
            f"debounced_holds OFF still finds {len(de_off)}/5 (sanitized); using fixed windows"
        )

    return OneShotReport(
        csv_path=str(path),
        duration_s=n / fs,
        fs_hz=fs,
        outlier_pct=float(np.mean(mask) * 100.0),
        segment=seg,
        cal=cal_metrics(cols, cal_end_i, fs),
        off_holds=off_holds,
        lut_holds=lut_holds,
        debounced_off_n=len(deb_off),
        debounced_lut_n=len(deb_lut),
    )


def format_report(rep: OneShotReport) -> str:
    s = rep.segment
    lines = [
        f"# ONE_SHOT Analysis — {Path(rep.csv_path).name}",
        "",
        f"- Duration: **{rep.duration_s:.1f} s** @ {rep.fs_hz:.0f} Hz",
        f"- Outlier mask: **{rep.outlier_pct:.3f}%**",
        f"- Hold windows: **fixed {PROFILE_HOLD_S}s ladder** (steady {10-4}s after lead/tail 2s)",
        "",
        "## Segments",
        "",
        "| Phase | Time (s) | Duration | Detection |",
        "|---|---|---:|---|",
        f"| Pass0 | 0 – {s.cal_end_i/rep.fs_hz:.1f} | {s.cal_end_i/rep.fs_hz:.1f} | {s.cal_method} |",
        f"| Speed OFF | {s.off_start_i/rep.fs_hz:.1f} – {s.lut_start_i/rep.fs_hz:.1f} | "
        f"{(s.lut_start_i-s.off_start_i)/rep.fs_hz:.1f} | {s.off_method} |",
        f"| Speed LUT | {s.lut_start_i/rep.fs_hz:.1f} – {rep.duration_s:.1f} | "
        f"{(rep.duration_s - s.lut_start_i/rep.fs_hz):.1f} | {s.lut_method} |",
        "",
        "> CSV has no deadband_mode channel (ch11=omega_err). LUT segment inferred by time only.",
        "",
        "## Pass0",
        "",
        "| Metric | Value |",
        "|---|---:|",
    ]
    for k, v in rep.cal.items():
        lines.append(f"| {k} | {v:.4f} |" if isinstance(v, float) else f"| {k} | {v} |")

    lines += holds_table(rep.off_holds, "Speed OFF — deadband OFF (1st ladder)")
    lines += holds_table(rep.lut_holds, "Speed LUT — RAM LUT (1st ladder)")
    lines += compare_off_lut(rep.off_holds, rep.lut_holds)

    lines += [
        "",
        "## Cross-check",
        "",
        f"- debounced_holds (sanitized) OFF: **{rep.debounced_off_n}** holds",
        f"- debounced_holds (sanitized) LUT: **{rep.debounced_lut_n}** holds",
        f"- fixed windows OFF/LUT: **{len(rep.off_holds)}** / **{len(rep.lut_holds)}** holds",
        "",
    ]
    if s.warnings:
        lines += ["## Warnings", ""]
        for w in s.warnings:
            lines.append(f"- {w}")
        lines.append("")

    lines += [
        "## Summary",
        "",
        "1. **Flow OK** if Pass0 ~16s, then 5×10s OFF ladder, then LUT ladder at ~66s.",
        "2. Compare **300 rpm Iq_std / ripple** for deadband effect on hunting.",
        "3. Compare **err_std** across rpm for speed tracking.",
        "",
    ]
    return "\n".join(lines)


def main() -> int:
    ap = argparse.ArgumentParser(description="ONE_SHOT VOFA CSV analyzer")
    ap.add_argument("csv", type=Path, help="VOFA CSV path")
    ap.add_argument("--fs", type=float, default=FS_HZ_DEFAULT)
    ap.add_argument("--lead-in", type=float, default=2.0)
    ap.add_argument("--tail", type=float, default=2.0)
    ap.add_argument("-o", "--output", type=Path, default=None, help="markdown report path")
    args = ap.parse_args()

    print(f"Analyzing {args.csv} ...", flush=True)
    rep = analyze_one_shot(args.csv, args.fs, lead_in_s=args.lead_in, tail_s=args.tail)
    text = format_report(rep)

    out = args.output
    if out is None:
        out = args.csv.with_suffix(".oneshot.md")
    out.write_text(text, encoding="utf-8")
    print(text)
    print(f"\nWrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
