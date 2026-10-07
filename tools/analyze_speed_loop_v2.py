#!/usr/bin/env python3
"""
Speed-loop VOFA CSV analyzer (v2).

Fixes vs v1:
  - Debounced profile-step detection (ignore 1-sample omega_ref glitches)
  - Steady window: skip hold lead-in + step transition tail
  - Telemetery outlier mask (PLL/iq_ref/Uq spikes)
  - Recompute omega_err = omega_ref - omega_pll on clean data
  - Fair compare: first full profile cycle, same hold windows
  - Report raw + cleaned stats side by side
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np

# firmware: 20 kHz FOC, telem decim 2 -> 10 kHz
FS_HZ_DEFAULT = 10000.0
PROFILE_RPMS = (100, 300, 500, 700, 900)
I_SAT_A = 5.0

CH = [
    "Ia", "Ib", "Ic", "Id", "Iq", "theta_el", "Ud", "Uq",
    "omega_pll", "omega_ref", "iq_ref", "omega_err",
]


@dataclass
class HoldStats:
    rpm: int
    t_start_s: float
    t_end_s: float
    n_raw: int
    n_clean: int
    omega_pll_mean: float
    omega_pll_std: float
    omega_pll_p50: float
    err_mean: float
    err_std: float
    err_p95_abs: float
    err_lt20_pct: float
    err_lt30_pct: float
    err_lt50_pct: float
    iq_mean: float
    iq_std: float
    iq_std_raw: float
    iq_pp: float
    iq_p05: float
    iq_p95: float
    iq_ref_mean: float
    iq_ref_std: float
    iq_ref_sat_pct: float
    id_std: float
    uq_std: float
    iq_neg_pct: float
    dom_ripple_hz: float | None


@dataclass
class FileReport:
    path: str
    duration_s: float
    n_rows: int
    fs_hz: float
    outlier_pct: float
    profile_holds: list[HoldStats]
    startup_note: str


def load_csv(path: Path) -> dict[str, np.ndarray]:
    data = np.genfromtxt(path, delimiter=",", skip_header=1, dtype=np.float64)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    if data.shape[1] < 12:
        raise ValueError(f"{path}: need 12 columns, got {data.shape[1]}")
    return {CH[i]: data[:, i].copy() for i in range(12)}


def build_outlier_mask(cols: dict[str, np.ndarray]) -> np.ndarray:
    """Mark single-sample telemetry spikes that would poison stats."""
    w = cols["omega_pll"]
    wref = cols["omega_ref"]
    iqr = cols["iq_ref"]
    uq = cols["Uq"]
    ud = cols["Ud"]

    m = np.zeros(len(w), dtype=bool)
    # PLL teleport (|w|>2000 while ref in profile band)
    in_band = (wref >= 50.0) & (wref <= 950.0)
    m |= in_band & (np.abs(w) > 2000.0)
    # iq_ref impossible (>I_SAT*1.5 or >>100 when speed mode)
    m |= np.abs(iqr) > max(I_SAT_A * 1.5, 20.0)
    # voltage telem spikes
    m |= np.abs(uq) > 50.0
    m |= np.abs(ud) > 50.0
    # omega_ref not on profile ladder (after debounce, still catch isolated)
    ladder = np.array(PROFILE_RPMS, dtype=float)
    d = np.min(np.abs(wref[:, None] - ladder[None, :]), axis=1)
    m |= (wref > 5.0) & (d > 50.0)
    return m


def sanitize_omega_ref(
    omega_ref: np.ndarray,
    fs: float,
    *,
    ladder: tuple[int, ...] = PROFILE_RPMS,
    tol_rpm: float = 50.0,
    max_glitch_s: float = 0.2,
) -> np.ndarray:
    """
    Forward-fill brief invalid omega_ref samples (step-edge telemetry glitches).
    """
    ladder_arr = np.array(ladder, dtype=float)
    n = len(omega_ref)
    out = omega_ref.astype(np.float64, copy=True)
    idx_near = np.argmin(np.abs(out[:, None] - ladder_arr[None, :]), axis=1)
    snap = ladder_arr[idx_near]
    valid = np.abs(out - snap) <= tol_rpm

    max_glitch_n = max(1, int(max_glitch_s * fs))
    last_snap = np.nan
    i = 0
    while i < n:
        if valid[i]:
            last_snap = snap[i]
            i += 1
            continue
        j = i
        while j < n and not valid[j]:
            j += 1
        if (j - i) <= max_glitch_n and not np.isnan(last_snap):
            out[i:j] = last_snap
            valid[i:j] = True
        i = j if j > i else i + 1

    return out


def detect_sustained_rpm(
    omega_ref: np.ndarray,
    fs: float,
    target_rpm: float,
    *,
    search_from: int = 0,
    tol_rpm: float = 40.0,
    sustain_s: float = 0.3,
) -> int | None:
    """Index where omega_ref first stays near target_rpm for sustain_s."""
    n = len(omega_ref)
    need = max(1, int(sustain_s * fs))
    for i in range(search_from, n - need):
        seg = omega_ref[i : i + need]
        if np.max(np.abs(seg - target_rpm)) <= tol_rpm:
            return i
    return None


def fixed_profile_holds(
    i0: int,
    fs: float,
    *,
    hold_s: float = 10.0,
    ladder: tuple[int, ...] = PROFILE_RPMS,
) -> list[tuple[int, int, int]]:
    """Deterministic hold windows: i0 + k*hold_s for each profile rpm."""
    hold_n = int(hold_s * fs)
    return [(rpm, i0 + k * hold_n, i0 + (k + 1) * hold_n) for k, rpm in enumerate(ladder)]


def debounced_holds(
    omega_ref: np.ndarray,
    fs: float,
    *,
    min_hold_s: float = 3.0,
    debounce_s: float = 0.05,
    sanitize: bool = True,
) -> list[tuple[int, int, int]]:
    """
    Return [(rpm, i0, i1), ...] for contiguous debounced holds.
    Assign each sample to nearest PROFILE_RPMS if within 50 rpm.
    """
    wref = sanitize_omega_ref(omega_ref, fs) if sanitize else omega_ref
    n = len(wref)
    debounce_n = max(1, int(debounce_s * fs))
    ladder = np.array(PROFILE_RPMS, dtype=float)

    idx_near = np.argmin(np.abs(wref[:, None] - ladder[None, :]), axis=1)
    rpm_assign = ladder[idx_near]
    valid = np.abs(wref - rpm_assign) <= 50.0
    rpm_assign = np.where(valid, rpm_assign, -1.0).astype(int)

    holds: list[tuple[int, int, int]] = []
    i = 0
    while i < n:
        rpm = int(rpm_assign[i])
        if rpm < 0:
            i += 1
            continue
        j = i + 1
        while j < n and rpm_assign[j] == rpm:
            j += 1
        if (j - i) >= int(min_hold_s * fs):
            holds.append((rpm, i, j))
        i = j

    # merge holds separated by <= debounce_n glitch samples (same rpm)
    merged: list[tuple[int, int, int]] = []
    for rpm, i0, i1 in holds:
        if merged and merged[-1][0] == rpm and (i0 - merged[-1][2]) <= debounce_n:
            merged[-1] = (rpm, merged[-1][1], i1)
        else:
            merged.append((rpm, i0, i1))
    return merged


def steady_slice(i0: int, i1: int, fs: float, *, lead_in_s: float, tail_s: float) -> slice:
    lead = int(lead_in_s * fs)
    tail = int(tail_s * fs)
    s = i0 + lead
    e = i1 - tail
    if e <= s:
        s = i0 + int(0.2 * (i1 - i0))
        e = i0 + int(0.8 * (i1 - i0))
    return slice(s, e)


def dominant_ripple_hz(x: np.ndarray, fs: float) -> float | None:
    if len(x) < 256:
        return None
    x0 = x - np.mean(x)
    spec = np.abs(np.fft.rfft(x0))
    freqs = np.fft.rfftfreq(len(x0), 1.0 / fs)
    if len(freqs) < 3:
        return None
    k = 1 + int(np.argmax(spec[1:]))  # skip DC
    f = float(freqs[k])
    if f < 0.5 or f > fs / 4.0:
        return None
    return f


def hold_stats(
    cols: dict[str, np.ndarray],
    mask: np.ndarray,
    rpm: int,
    i0: int,
    i1: int,
    fs: float,
    *,
    lead_in_s: float,
    tail_s: float,
) -> HoldStats:
    sl_full = slice(i0, i1)
    sl = steady_slice(i0, i1, fs, lead_in_s=lead_in_s, tail_s=tail_s)

    w = cols["omega_pll"][sl]
    wref = cols["omega_ref"][sl]
    iq = cols["Iq"][sl]
    iq_raw = cols["Iq"][sl_full]
    id_ = cols["Id"][sl]
    iqr = cols["iq_ref"][sl]
    uq = cols["Uq"][sl]
    clean = ~mask[sl]
    err = wref - w  # recompute; do not trust ch11 during glitches

    if np.any(clean):
        err_c = err[clean]
        w_c = w[clean]
        iq_c = iq[clean]
        iqr_c = iqr[clean]
        id_c = id_[clean]
        uq_c = uq[clean]
    else:
        err_c = err
        w_c = w
        iq_c = iq
        iqr_c = iqr
        id_c = id_
        uq_c = uq

    dom = dominant_ripple_hz(iq_c, fs)

    return HoldStats(
        rpm=rpm,
        t_start_s=i0 / fs,
        t_end_s=i1 / fs,
        n_raw=i1 - i0,
        n_clean=int(np.sum(clean)),
        omega_pll_mean=float(np.mean(w_c)),
        omega_pll_std=float(np.std(w_c)),
        omega_pll_p50=float(np.percentile(w_c, 50)),
        err_mean=float(np.mean(err_c)),
        err_std=float(np.std(err_c)),
        err_p95_abs=float(np.percentile(np.abs(err_c), 95)),
        err_lt20_pct=float(np.mean(np.abs(err_c) < 20.0) * 100.0),
        err_lt30_pct=float(np.mean(np.abs(err_c) < 30.0) * 100.0),
        err_lt50_pct=float(np.mean(np.abs(err_c) < 50.0) * 100.0),
        iq_mean=float(np.mean(iq_c)),
        iq_std=float(np.std(iq_c)),
        iq_std_raw=float(np.std(iq_raw)),
        iq_pp=float(np.max(iq_c) - np.min(iq_c)),
        iq_p05=float(np.percentile(iq_c, 5)),
        iq_p95=float(np.percentile(iq_c, 95)),
        iq_ref_mean=float(np.mean(iqr_c)),
        iq_ref_std=float(np.std(iqr_c)),
        iq_ref_sat_pct=float(np.mean(np.abs(iqr_c) >= I_SAT_A - 0.01) * 100.0),
        id_std=float(np.std(id_c)),
        uq_std=float(np.std(uq_c)),
        iq_neg_pct=float(np.mean(iq_c < 0.0) * 100.0),
        dom_ripple_hz=dom,
    )


def first_cycle_holds(holds: list[tuple[int, int, int]]) -> list[tuple[int, int, int]]:
    """Keep first occurrence of each rpm in ladder order (one profile cycle)."""
    seen: set[int] = set()
    out: list[tuple[int, int, int]] = []
    for rpm, i0, i1 in holds:
        if rpm in seen:
            continue
        seen.add(rpm)
        out.append((rpm, i0, i1))
        if len(seen) == len(PROFILE_RPMS):
            break
    return sorted(out, key=lambda x: PROFILE_RPMS.index(x[0]))


def analyze_file(
    path: Path,
    fs: float,
    *,
    lead_in_s: float,
    tail_s: float,
    first_cycle_only: bool,
) -> FileReport:
    cols = load_csv(path)
    n = len(cols["Ia"])
    mask = build_outlier_mask(cols)
    holds = debounced_holds(cols["omega_ref"], fs)
    if first_cycle_only:
        holds_use = first_cycle_holds(holds)
    else:
        holds_use = holds

    stats: list[HoldStats] = []
    for rpm, i0, i1 in holds_use:
        if rpm not in PROFILE_RPMS:
            continue
        stats.append(
            hold_stats(cols, mask, rpm, i0, i1, fs,
                       lead_in_s=lead_in_s, tail_s=tail_s)
        )

    # startup diagnostic: first 2 s after first 100 rpm hold begins
    startup_note = ""
    h100 = [h for h in holds if h[0] == 100]
    if h100:
        i0 = h100[0][1]
        sl = slice(i0, min(i0 + int(2.0 * fs), n))
        w = cols["omega_pll"][sl]
        bad = mask[sl]
        startup_note = (
            f"first 100rpm hold +0~2s: omega_pll std={np.std(w[~bad]):.1f} rpm "
            f"(raw {np.std(w):.1f}), outliers {100*np.mean(bad):.1f}%"
        )

    return FileReport(
        path=str(path),
        duration_s=n / fs,
        n_rows=n,
        fs_hz=fs,
        outlier_pct=float(np.mean(mask) * 100.0),
        profile_holds=stats,
        startup_note=startup_note,
    )


def print_report(rep: FileReport, title: str) -> None:
    print(f"\n{'=' * 72}")
    print(title)
    print(f"  file: {Path(rep.path).name}")
    print(
        f"  duration={rep.duration_s:.1f}s  rows={rep.n_rows}  "
        f"fs={rep.fs_hz:.0f}Hz  outlier_mask={rep.outlier_pct:.3f}%"
    )
    if rep.startup_note:
        print(f"  startup: {rep.startup_note}")
    print()
    hdr = (
        f"{'rpm':>4} {'t_hold':>12} {'n_cln':>7} "
        f"{'ω_mean':>7} {'ω_std':>6} "
        f"{'|e|<20':>7} {'|e|<30':>7} {'e_p95':>6} "
        f"{'Iq_mu':>6} {'Iq_sd':>6} {'Iq_pp':>6} {'Iq<0':>6} "
        f"{'iq_rf':>6} {'sat%':>5} {'Id_sd':>6} {'ripHz':>6}"
    )
    print(hdr)
    for s in rep.profile_holds:
        th = f"{s.t_start_s:5.1f}-{s.t_end_s:5.1f}s"
        rip = f"{s.dom_ripple_hz:.1f}" if s.dom_ripple_hz is not None else "  -"
        print(
            f"{s.rpm:4d} {th:>12} {s.n_clean:7d} "
            f"{s.omega_pll_mean:7.1f} {s.omega_pll_std:6.1f} "
            f"{s.err_lt20_pct:6.1f}% {s.err_lt30_pct:6.1f}% {s.err_p95_abs:6.1f} "
            f"{s.iq_mean:6.2f} {s.iq_std:6.2f} {s.iq_pp:6.2f} {s.iq_neg_pct:5.1f}% "
            f"{s.iq_ref_mean:6.2f} {s.iq_ref_sat_pct:5.1f} {s.id_std:6.3f} {rip:>6}"
        )


def print_compare(a: FileReport, b: FileReport, label_a: str, label_b: str) -> None:
    print(f"\n{'=' * 72}")
    print(f"COMPARE first cycle (steady window)  A={label_a}  B={label_b}")
    print(f"{'rpm':>4}  {'metric':>12}  {label_a:>10}  {label_b:>10}  {'delta':>10}")
    by_a = {s.rpm: s for s in a.profile_holds}
    by_b = {s.rpm: s for s in b.profile_holds}
    metrics = [
        ("Iq_mean", lambda s: s.iq_mean, ".2f"),
        ("Iq_std", lambda s: s.iq_std, ".3f"),
        ("|e|<30%", lambda s: s.err_lt30_pct, ".1f"),
        ("omega_std", lambda s: s.omega_pll_std, ".1f"),
        ("iq_ref_sat%", lambda s: s.iq_ref_sat_pct, ".2f"),
    ]
    for rpm in PROFILE_RPMS:
        if rpm not in by_a or rpm not in by_b:
            continue
        sa, sb = by_a[rpm], by_b[rpm]
        for name, fn, fmt in metrics:
            va, vb = fn(sa), fn(sb)
            print(f"{rpm:4d}  {name:>12}  {va:10{fmt}}  {vb:10{fmt}}  {vb - va:+10{fmt}}")


def main() -> int:
    ap = argparse.ArgumentParser(description="Speed-loop VOFA CSV analyzer v2")
    ap.add_argument("csv", type=Path, nargs="+", help="one or more CSV files")
    ap.add_argument("--fs", type=float, default=FS_HZ_DEFAULT)
    ap.add_argument("--lead-in", type=float, default=2.0,
                    help="skip seconds after each hold start (default 2)")
    ap.add_argument("--tail", type=float, default=0.5,
                    help="skip seconds before hold end (default 0.5)")
    ap.add_argument("--all-holds", action="store_true",
                    help="include repeat cycles, not just first")
    ap.add_argument("--json", type=Path, default=None)
    args = ap.parse_args()

    reports: list[FileReport] = []
    for p in args.csv:
        rep = analyze_file(
            p, args.fs,
            lead_in_s=args.lead_in,
            tail_s=args.tail,
            first_cycle_only=not args.all_holds,
        )
        reports.append(rep)
        print_report(rep, f"REPORT: {p.name}")

    if len(reports) == 2:
        print_compare(
            reports[0], reports[1],
            Path(reports[0].path).stem,
            Path(reports[1].path).stem,
        )

    if args.json:
        payload = [asdict(r) for r in reports]
        args.json.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        print(f"\nWrote {args.json}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
