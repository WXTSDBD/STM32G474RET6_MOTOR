#!/usr/bin/env python3
"""速度环阶跃响应离线分析（M1_SPEED_IDENT_ENABLE / deadband OFF）。

VOFA×12（M1_SPEED_IDENT_ENABLE）：
  ch8=ω_pll  ch9=ω_ref  ch10=iq_ref  ch11=open_seq
  open_seq：220=HOLD  221..226=STEP 各相  230=BODE  239=DONE

用法:
  python tools/ident/analyze_speed_step.py VOFA+CSV/xxx.csv
  python tools/ident/analyze_speed_step.py file.csv --md report.md
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))

from vofa_io import load_vofa, read_csv_matrix  # noqa: E402

FS = 10000.0
DYNAMIC_WIN_S = 2.0
SETTLE_BAND_RPM = 15.0
STEP_PHASE_LO = 221
STEP_PHASE_HI = 226
HOLD_PHASE = 220
CH_OMEGA = 8
CH_OMEGA_REF = 9
CH_OPEN_SEQ = 11


def series(frame, col: int) -> np.ndarray:
    _, mat = read_csv_matrix(frame.path)
    return mat[:, col]


def find_step_window(open_seq: np.ndarray) -> tuple[int, int]:
    mask = (open_seq >= float(STEP_PHASE_LO) - 0.5) & (
        open_seq <= float(STEP_PHASE_HI) + 0.5
    )
    idx = np.where(mask)[0]
    if len(idx) == 0:
        return 0, len(open_seq)
    return int(idx[0]), int(idx[-1]) + 1


def step_metrics(t: np.ndarray, y: np.ndarray, u: np.ndarray, baseline: float) -> list[dict]:
    edges = np.where(np.abs(np.diff(u)) > 5.0)[0]
    rows: list[dict] = []
    for ei in edges:
        i0 = ei + 1
        if i0 >= len(u):
            continue
        u0 = float(u[i0 - 1])
        u1 = float(u[i0])
        y0 = float(np.median(y[max(0, i0 - int(0.2 * FS)) : i0]))
        target = u1
        amp = abs(target - y0)
        if amp < 20.0:
            continue
        win = min(len(y), i0 + int(DYNAMIC_WIN_S * FS))
        seg_t = t[i0:win] - t[i0]
        seg_y = y[i0:win]
        peak = float(np.max(seg_y)) if target > y0 else float(np.min(seg_y))
        mp = 100.0 * (peak - target) / amp if amp > 1e-6 else 0.0
        tr_idx = np.where(np.abs(seg_y - (y0 + 0.9 * (target - y0))) <= SETTLE_BAND_RPM)[0]
        tr = float(seg_t[tr_idx[0]]) if len(tr_idx) else float("nan")
        err = float(np.median(seg_y[-int(0.5 * FS) :])) - target if win - i0 > int(0.5 * FS) else float("nan")
        rows.append(
            {
                "t_s": float(t[i0]),
                "u0": u0,
                "u1": u1,
                "y0": y0,
                "Mp_pct": mp,
                "tr_s": tr,
                "err_rpm": err,
            }
        )
    return rows


def main() -> int:
    ap = argparse.ArgumentParser(description="Speed-loop step response analysis")
    ap.add_argument("csv", type=Path)
    ap.add_argument("--md", type=Path, default=None)
    ap.add_argument("--fs", type=float, default=FS)
    args = ap.parse_args()

    frame = load_vofa(args.csv, fs=args.fs)
    omega = series(frame, CH_OMEGA)
    omega_ref = series(frame, CH_OMEGA_REF)
    open_seq = series(frame, CH_OPEN_SEQ)
    t = frame.t

    i0, i1 = find_step_window(open_seq)
    hold_mask = np.isclose(open_seq, float(HOLD_PHASE), atol=0.5)
    if np.any(hold_mask):
        print(f"HOLD segment: {t[hold_mask][0]:.2f} .. {t[hold_mask][-1]:.2f} s")

    rows = step_metrics(t[i0:i1], omega[i0:i1], omega_ref[i0:i1], baseline=500.0)
    print(f"STEP window: {t[i0]:.2f} .. {t[i1 - 1]:.2f} s  ({len(rows)} edges)")
    for i, r in enumerate(rows):
        print(
            f"  #{i+1} {r['u0']:.0f}->{r['u1']:.0f} rpm @ {r['t_s']:.2f}s  "
            f"Mp={r['Mp_pct']:+.1f}%  tr={r['tr_s']:.3f}s  err={r['err_rpm']:+.1f} rpm"
        )

    if args.md:
        lines = ["# Speed step report\n", f"CSV: `{args.csv}`\n\n", "| # | t[s] | step | Mp% | tr[s] | err[rpm] |\n", "|---|------|------|-----|-------|----------|\n"]
        for i, r in enumerate(rows):
            lines.append(
                f"| {i+1} | {r['t_s']:.2f} | {r['u0']:.0f}->{r['u1']:.0f} | {r['Mp_pct']:+.1f} | {r['tr_s']:.3f} | {r['err_rpm']:+.1f} |\n"
            )
        args.md.write_text("".join(lines), encoding="utf-8")
        print(f"Wrote {args.md}")

    return 0 if rows else 1


if __name__ == "__main__":
    raise SystemExit(main())
