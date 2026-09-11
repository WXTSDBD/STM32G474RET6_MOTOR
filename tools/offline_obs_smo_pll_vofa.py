#!/usr/bin/env python3
"""Offline SMO+atan vs SMO→PLL VOFA (M1_VOFA_OBS_PLL_12CH + M1_EMF_PLL_USE_SMO).

Layout:
  0 eα  1 eβ  2 err_atan  3 err_pll  4 θenc  5 ωe_pll
  6 |e|  7 θ̂_pll  8 isr  9 foc  10 obs  11 iα
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", type=Path)
    ap.add_argument("--fs", type=float, default=10000.0)
    args = ap.parse_args()

    raw = np.genfromtxt(args.csv, delimiter=",", skip_header=1)
    if raw.ndim != 2 or raw.shape[1] < 12:
        print("need 12 cols", file=sys.stderr)
        return 2

    fs = args.fs
    t = np.arange(len(raw)) / fs
    ea, eb = raw[:, 0], raw[:, 1]
    err_a, err_p = raw[:, 2], raw[:, 3]
    we = raw[:, 5]
    emag = raw[:, 6]
    isr, foc, obs = raw[:, 8], raw[:, 9], raw[:, 10]
    ia = raw[:, 11]
    rpm = we / (2 * np.pi) * 60.0 / 7.0

    print(f"file={args.csv.name} dur={t[-1]:.1f}s")
    print(
        f"cycle med/p95: isr={np.median(isr):.0f}/{np.percentile(isr,95):.0f} "
        f"obs={np.median(obs):.0f}/{np.percentile(obs,95):.0f} "
        f"ia_std={ia.std():.3f}"
    )
    print(f"{'rpm':>5} {'At p95':>7} {'Pl p95':>7} {'At med':>7} {'Pl med':>7} {'|e|':>6}")
    for tgt in (400, 500, 600, 700, 800, 900, 1000):
        near = np.abs(rpm - tgt) < 80
        if near.sum() < int(2 * fs):
            continue
        idx = np.where(near)[0]
        cuts = np.where(np.diff(idx) > 1)[0]
        starts = np.r_[idx[0], idx[cuts + 1]]
        ends = np.r_[idx[cuts], idx[-1]]
        best = None
        for s, e in zip(starts, ends):
            if (e - s + 1) >= int(2 * fs):
                best = (s, e)
        if best is None:
            continue
        s, e = best
        s2 = s + int(3 * fs)
        e2 = e - int(0.5 * fs)
        if e2 <= s2:
            continue
        m = slice(s2, e2 + 1)
        ea_, ep_ = err_a[m], err_p[m]
        print(
            f"{tgt:5d} {np.percentile(np.abs(ea_),95)*180/np.pi:7.2f} "
            f"{np.percentile(np.abs(ep_),95)*180/np.pi:7.2f} "
            f"{np.median(ea_)*180/np.pi:7.2f} {np.median(ep_)*180/np.pi:7.2f} "
            f"{np.mean(emag[m]):6.2f}"
        )
        _ = (ea, eb)  # silence unused if no windows
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
