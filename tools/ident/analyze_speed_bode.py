#!/usr/bin/env python3
"""速度环 Bode 离线分析（M1_SPEED_IDENT_ENABLE / deadband OFF）。

VOFA：ch8=ω_pll  ch9=ω_ref  ch10=bode_f[Hz]（BODE 段）  ch11=open_seq(230)

用法:
  python tools/ident/analyze_speed_bode.py VOFA+CSV/xxx.csv
  python tools/ident/analyze_speed_bode.py file.csv --md report.md
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))

from vofa_io import fundamental, load_vofa, read_csv_matrix  # noqa: E402

FS = 10000.0
BODE_PHASE = 230
DEFAULT_CYCLES = 10.0
CH_OMEGA = 8
CH_OMEGA_REF = 9
CH_BODE_F = 10
CH_OPEN_SEQ = 11

# 与 speed_ident_module.c s_bode_freq_table 一致
FREQS = [
    0.5, 0.63, 0.7943, 1.0, 1.2589, 1.5849, 1.9953, 2.5119,
    3.1623, 3.9811, 5.0119, 6.3096, 7.9433, 10.0, 12.5893, 15.8489,
    19.9526, 25.1189, 31.6228, 39.8107, 50.0,
]


def series(frame, col: int) -> np.ndarray:
    _, mat = read_csv_matrix(frame.path)
    return mat[:, col]


def bode_segment(open_seq: np.ndarray) -> tuple[int, int]:
    mask = np.isclose(open_seq, float(BODE_PHASE), atol=0.5)
    idx = np.where(mask)[0]
    if len(idx) == 0:
        return 0, len(open_seq)
    return int(idx[0]), int(idx[-1]) + 1


def analyze_point(ref: np.ndarray, y: np.ndarray, fs: float, f_hz: float) -> tuple[float, float]:
    n_cycle = max(int(DEFAULT_CYCLES / f_hz * fs), int(fs / f_hz))
    n = min(len(ref), n_cycle)
    if n < int(0.5 * fs):
        return float("nan"), float("nan")
    r = ref[-n:]
    y = y[-n:]
    r = r - float(np.mean(r))
    y = y - float(np.mean(y))
    mag_r, ph_r = fundamental(r, fs, f_hz)
    mag_y, ph_y = fundamental(y, fs, f_hz)
    if mag_r < 1e-6:
        return float("nan"), float("nan")
    gain_db = 20.0 * math.log10(mag_y / mag_r)
    phase_deg = (ph_y - ph_r) * 180.0 / math.pi
    while phase_deg > 180.0:
        phase_deg -= 360.0
    while phase_deg < -180.0:
        phase_deg += 360.0
    return gain_db, phase_deg


def main() -> int:
    ap = argparse.ArgumentParser(description="Speed-loop Bode analysis")
    ap.add_argument("csv", type=Path)
    ap.add_argument("--md", type=Path, default=None)
    ap.add_argument("--fs", type=float, default=FS)
    args = ap.parse_args()

    frame = load_vofa(args.csv, fs=args.fs)
    omega = series(frame, CH_OMEGA)
    omega_ref = series(frame, CH_OMEGA_REF)
    bode_f = series(frame, CH_BODE_F)
    open_seq = series(frame, CH_OPEN_SEQ)

    i0, i1 = bode_segment(open_seq)
    print(f"BODE segment: {frame.t[i0]:.2f} .. {frame.t[i1 - 1]:.2f} s")

    rows: list[dict] = []
    cursor = i0
    for f_hz in FREQS:
        seg_len = int(max(DEFAULT_CYCLES / f_hz, 1.0 / f_hz) * args.fs)
        j0 = cursor
        j1 = min(i1, j0 + seg_len)
        if j1 - j0 < int(0.3 * args.fs):
            break
        f_mark = float(np.median(bode_f[j0:j1]))
        if f_mark > 0.1:
            f_use = f_mark
        else:
            f_use = f_hz
        g_db, ph = analyze_point(omega_ref[j0:j1], omega[j0:j1], args.fs, f_use)
        rows.append({"f_hz": f_use, "gain_db": g_db, "phase_deg": ph})
        print(f"  f={f_use:6.2f} Hz  |G|={g_db:+.2f} dB  ∠={ph:+.1f}°")
        cursor = j1

    if args.md:
        lines = ["# Speed Bode report\n", f"CSV: `{args.csv}`\n\n", "| f[Hz] | gain[dB] | phase[deg] |\n", "|-------|----------|------------|\n"]
        for r in rows:
            lines.append(f"| {r['f_hz']:.3f} | {r['gain_db']:+.2f} | {r['phase_deg']:+.1f} |\n")
        args.md.write_text("".join(lines), encoding="utf-8")
        print(f"Wrote {args.md}")

    return 0 if rows else 1


if __name__ == "__main__":
    raise SystemExit(main())
