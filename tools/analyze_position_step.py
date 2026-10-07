#!/usr/bin/env python3
"""Analyze POSITION mode VOFA CSV (ch8=theta_err ch9=theta_mech ch10=theta_ref ch11=omega_ref)."""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

FS_HZ = 10000.0


def unwrap(th: np.ndarray) -> np.ndarray:
    o = np.empty_like(th)
    o[0] = th[0]
    for i in range(1, len(th)):
        dd = th[i] - th[i - 1]
        if dd > math.pi:
            dd -= 2 * math.pi
        elif dd < -math.pi:
            dd += 2 * math.pi
        o[i] = o[i - 1] + dd
    return o


def find_ref_steps(ref: np.ndarray, fs: float, min_hold_s: float = 0.8) -> list[tuple[int, int, float]]:
    steps: list[tuple[int, int, float]] = []
    last = ref[0]
    start = 0
    i = 1
    while i < len(ref):
        if abs(ref[i] - last) > 0.05:
            j = i
            while j < len(ref) and abs(ref[j] - ref[i]) < 0.01:
                j += 1
            if (j - i) > int(0.05 * fs) and abs(ref[i] - last) > 0.05:
                if (i - start) / fs >= min_hold_s or len(steps) == 0:
                    steps.append((start, i, last))
                start = i
                last = ref[i]
                i = j
                continue
        i += 1
    steps.append((start, len(ref) - 1, last))
    return steps


def dominant_tone(sig: np.ndarray, fs: float) -> tuple[float, float, list[tuple[float, float]]]:
    sig = sig - sig.mean()
    n = len(sig)
    if n < 64:
        return 0.0, 0.0, []
    win = np.hanning(n)
    spec = np.abs(np.fft.rfft(sig * win))
    freqs = np.fft.rfftfreq(n, 1.0 / fs)
    pk = int(np.argmax(spec[1:])) + 1
    amp = float(spec[pk] / n * 2.0)
    idxs = np.argsort(spec[1:])[-5:][::-1] + 1
    tops = [(float(freqs[i]), float(spec[i] / n * 2.0)) for i in idxs if freqs[i] >= 1.0]
    return float(freqs[pk]), amp, tops


def analyze(path: Path) -> int:
    d = np.genfromtxt(path, delimiter=",", names=True, dtype=float)
    n = len(d["I0"])
    fs = FS_HZ
    t = np.arange(n) / fs

    iq = d["I4"]
    theta_err = d["I8"]
    theta_mech = d["I9"]
    theta_ref = d["I10"]
    omega_ref = d["I11"]
    th_u = unwrap(theta_mech)
    theta0 = theta_ref[0]

    print(f"=== {path.name} POSITION test ===")
    print(f"Rows: {n:,}  Duration: {t[-1]:.1f} s  fs={fs} Hz")
    print("Layout: ch8=theta_err ch9=theta_mech ch10=theta_ref ch11=omega_ref")
    print()

    i0 = int(0.5 * fs)
    rev = (th_u[i0:].max() - th_u[i0:].min()) / (2 * math.pi)
    print(f"After t=0.5 s: theta_ref span {(theta_ref[i0:].max()-theta_ref[i0:].min()):.2f} rad")
    print(f"               theta_mech {rev:.1f} rev total")
    print(f"               |Iq| max={np.abs(iq).max():.2f} A  |omega_ref| max={np.abs(omega_ref).max():.1f} rpm")
    print()

    steps = find_ref_steps(theta_ref, fs)
    print(f"Detected {len(steps)} theta_ref segments")
    hdr = (
        f"{'idx':>3} {'t0':>6} {'t1':>6} {'dur':>5} {'dref_deg':>9} "
        f"{'err_m':>8} {'err_sd':>7} {'err_p95':>8} {'iq_pp':>7} {'wq_max':>7}"
    )
    print(hdr)
    seg_stats = []
    for k, (a, b, refv) in enumerate(steps):
        sl = slice(a + int(0.3 * fs), b - int(0.2 * fs))
        if sl.start >= sl.stop:
            sl = slice(a, b)
        te = theta_err[sl]
        wq = omega_ref[sl]
        iq_sl = iq[sl]
        dur = (b - a) / fs
        pp = float(iq_sl.max() - iq_sl.min())
        row = {
            "k": k,
            "a": a,
            "b": b,
            "dur": dur,
            "dref_deg": math.degrees(refv - theta0),
            "err_mean": float(te.mean()),
            "err_std": float(te.std()),
            "err_p95": float(np.percentile(np.abs(te), 95)),
            "iq_pp": pp,
            "wq_max": float(np.abs(wq).max()),
        }
        seg_stats.append(row)
        print(
            f"{k:3d} {t[a]:6.1f} {t[b]:6.1f} {dur:5.1f} {row['dref_deg']:+9.1f} "
            f"{row['err_mean']:+8.4f} {row['err_std']:7.4f} {row['err_p95']:8.4f} "
            f"{pp:7.3f} {row['wq_max']:7.1f}"
        )

    print()
    hold = (np.abs(omega_ref) < 20.0) & (t > 1.0)
    if hold.sum() > 1000:
        print("Near-hold (|omega_ref|<20 rpm, t>1 s):")
        print(
            f"  theta_err std={theta_err[hold].std():.4f} rad "
            f"({math.degrees(theta_err[hold].std()):.2f} deg)"
        )
        print(f"  Iq std={iq[hold].std():.3f} A  Iq pp={iq[hold].max()-iq[hold].min():.3f} A")

    best = None
    for a, b, refv in steps:
        if (b - a) / fs < 1.0:
            continue
        sl = slice(a + int(0.5 * fs), b - int(0.1 * fs))
        if sl.start >= sl.stop:
            continue
        if np.abs(omega_ref[sl]).mean() > 30:
            continue
        if best is None or (b - a) > (best[1] - best[0]):
            best = (a, b, refv)

    if best:
        a, b, refv = best
        sl = slice(a + int(0.3 * fs), b - int(0.1 * fs))
        f0, amp, tops = dominant_tone(theta_err[sl], fs)
        print()
        print(
            f"theta_err spectrum (steady seg t={t[a]:.1f}-{t[b]:.1f} s, "
            f"ref={math.degrees(refv-theta0):+.0f} deg):"
        )
        print(f"  dominant: {f0:.1f} Hz  {amp:.4f} rad ({math.degrees(amp):.2f} deg_pp)")
        for f, a2 in tops[:5]:
            print(f"    {f:5.1f} Hz  {a2:.4f} rad ({math.degrees(a2):.2f} deg)")

    dth = np.diff(th_u)
    glitch = np.abs(dth) > 0.5
    print()
    print(f"theta_mech glitches (|dtheta|>0.5 rad): {int(glitch.sum())} ({100*glitch.mean():.4f}%)")

    move = np.abs(omega_ref) > 50
    if move.sum() > 0:
        print(
            f"During |omega_ref|>50 rpm: Iq std={iq[move].std():.3f} A, "
            f"theta_err std={theta_err[move].std():.4f} rad"
        )

    # overshoot on first +90 deg step if present
    for row in seg_stats:
        if 85 <= row["dref_deg"] <= 95 and row["dur"] >= 1.0:
            sl = slice(row["a"], row["b"])
            te = theta_err[sl]
            print()
            print("+90 deg step: overshoot={:.3f} rad ({:.1f} deg)  settle |err|<0.05: check visually".format(
                float(-te.min()), math.degrees(-te.min())
            ))
            break

    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", type=Path)
    args = ap.parse_args()
    return analyze(args.csv)


if __name__ == "__main__":
    sys.exit(main())
