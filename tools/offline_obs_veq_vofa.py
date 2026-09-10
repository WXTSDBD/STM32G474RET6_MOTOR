#!/usr/bin/env python3
"""
Offline check of MCU Veq observer VOFA (M1_VOFA_OBS_VEQ_12CH).

Layout:
  0 iα  1 iβ  2 uα  3 uβ  4 eα  5 eβ
  6 θ̂  7 θenc  8 θerr  9 |e|  10 ωe  11 ψinst

Usage:
  python tools/offline_obs_veq_vofa.py VOFA+CSV/.../vofa+....csv --fs 10000
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np


def wrap_pi(x: np.ndarray) -> np.ndarray:
    return (x + np.pi) % (2.0 * np.pi) - np.pi


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("csv", type=Path)
    ap.add_argument("--fs", type=float, default=10000.0, help="D=2 -> 10 kHz")
    ap.add_argument("--lead", type=float, default=1.0)
    ap.add_argument("--tail", type=float, default=0.5)
    ap.add_argument("--we-min", type=float, default=50.0, help="rad/s for steady")
    args = ap.parse_args()

    raw = np.genfromtxt(args.csv, delimiter=",", skip_header=1)
    if raw.ndim != 2 or raw.shape[1] < 12:
        print("need 12 columns", file=sys.stderr)
        return 2

    fs = args.fs
    ia, ib, ua, ub = raw[:, 0], raw[:, 1], raw[:, 2], raw[:, 3]
    ea, eb = raw[:, 4], raw[:, 5]
    th_hat, th_enc, th_err = raw[:, 6], raw[:, 7], raw[:, 8]
    emag, we, psi = raw[:, 9], raw[:, 10], raw[:, 11]
    n = len(raw)
    t = np.arange(n) / fs

    # steady: |we| high enough (1000rpm * 7 * 2pi/60 ≈ 733 rad/s)
    at = np.abs(we) > args.we_min
    mask = np.zeros(n, dtype=bool)
    i = 0
    while i < n:
        if not at[i]:
            i += 1
            continue
        j = i + 1
        while j < n and at[j]:
            j += 1
        if (j - i) >= int(2.0 * fs):
            s = i + int(args.lead * fs)
            e = j - int(args.tail * fs)
            if e > s:
                mask[s:e] = True
        i = j

    if not np.any(mask):
        print("no steady window (|ωe| gate); try lower --we-min")
        return 1

    err = th_err[mask]
    # also recompute wrap(hat-enc) in case firmware offset already applied
    err2 = wrap_pi(th_hat[mask] - th_enc[mask])
    # debias residual
    err0 = wrap_pi(err - np.median(err))

    print(f"file={args.csv.name} dur={t[-1]:.2f}s fs={fs:.0f} steady={mask.sum()/fs:.2f}s")
    print(f"|ωe| mean={np.mean(np.abs(we[mask])):.1f}  |e| mean={np.mean(emag[mask]):.3f}  ψ median={np.median(psi[mask]):.5f}")
    print(f"MCU θ_err (ch8): mean={np.mean(err)*180/np.pi:.2f}°  |e|p50={np.median(np.abs(err))*180/np.pi:.2f}°  "
          f"p95={np.percentile(np.abs(err),95)*180/np.pi:.2f}°  rms={np.sqrt(np.mean(err**2))*180/np.pi:.2f}°")
    print(f"MCU θ_err debiased: |e|p50={np.median(np.abs(err0))*180/np.pi:.2f}°  "
          f"p95={np.percentile(np.abs(err0),95)*180/np.pi:.2f}°  rms={np.sqrt(np.mean(err0**2))*180/np.pi:.2f}°")
    print(f"wrap(θ̂-θenc): |e|p50={np.median(np.abs(err2))*180/np.pi:.2f}°  "
          f"p95={np.percentile(np.abs(err2),95)*180/np.pi:.2f}°")

    # cross-check e vs u,i offline (same Veq, no LPF for rough check)
    R, L = 0.122, 59e-6
    ts = 1.0 / fs
    dia = np.gradient(ia, ts)
    dib = np.gradient(ib, ts)
    ea_o = ua - R * ia - L * dia
    eb_o = ub - R * ib - L * dib
    # compare MCU e to offline on steady
    print(f"|eα MCU-offline| mae={np.mean(np.abs(ea[mask]-ea_o[mask])):.3f} V  "
          f"|eβ| mae={np.mean(np.abs(eb[mask]-eb_o[mask])):.3f} V")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
