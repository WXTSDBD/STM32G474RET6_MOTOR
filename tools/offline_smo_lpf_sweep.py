#!/usr/bin/env python3
"""Offline SMO e-LPF sweep on RAW VOFA (M1_VOFA_OBS_SMO_RAW_12CH).

Layout (telem fs = 20kHz/DECIM, default 10 kHz):
  0 iα  1 iβ  2 uα  3 uβ  4 θ_enc  5 ω_mech_rpm
  6 eα_mcu  7 eβ_mcu  8 err_pll_mcu  9 |e|  10 θ̂_pll  11 lpf_hz

IMPORTANT: MCU SMO runs at 20 kHz. Default replay is ZOH upsample:
  each telem sample → --zoh-steps updates with ts=1/isr_hz (default 2×50µs).
  Bare 10 kHz single-step does NOT match MCU history — do not use for decisions.

Examples:
  python tools/offline_smo_lpf_sweep.py VOFA+CSV/20260910/vofa+....csv
  python tools/offline_smo_lpf_sweep.py raw.csv --mode fixed --fc 120,160,200
  python tools/offline_smo_lpf_sweep.py raw.csv --mode linear --k 1.5,1.7,2.0
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

try:
    from numba import njit

    HAS_NUMBA = True
except ImportError:
    HAS_NUMBA = False

POLE_PAIRS = 7.0
R_OHM = 0.122
L_H = 59e-6
SMO_K = 20.0
SAT_A = 0.30
THETA_OFF = -0.4054
PLL_FN_HZ = 60.0
PLL_ZETA = 0.707106781
PLL_NORM_EPS = 0.05
PLL_W_LIM = 2000.0
TWO_PI = 2.0 * math.pi
PI = math.pi

# Ladder dwells (skip climb) — RAW packs 2216/2217 style
WINDOWS = [
    (400, 10.0, 17.5),
    (550, 20.0, 27.5),
    (700, 30.0, 37.5),
    (850, 40.0, 47.5),
    (1000, 50.0, 57.5),
    (1100, 60.0, 67.5),
    (1200, 70.0, 79.0),
]


def parse_floats(s: str) -> list[float]:
    return [float(x) for x in s.split(",") if x.strip()]


def wrap_pi(x: float) -> float:
    while x > PI:
        x -= TWO_PI
    while x < -PI:
        x += TWO_PI
    return x


if HAS_NUMBA:

    @njit(cache=True)
    def _replay_nb(ia, ib, ua, ub, th, ts, fc_hz, steps):
        n = len(ia)
        disc_a = math.exp(-R_OHM * ts / L_H)
        disc_b = (1.0 - disc_a) / R_OHM
        inv = 1.0 / SAT_A
        wn = TWO_PI * PLL_FN_HZ
        kp = 2.0 * PLL_ZETA * wn
        ki = wn * wn
        ihat_a = 0.0
        ihat_b = 0.0
        ea = 0.0
        eb = 0.0
        smo = False
        pll = False
        pll_th = 0.0
        pll_w = 0.0
        pll_int = 0.0
        last_fc = -1.0
        lpf_a = 1.0
        err_p = np.zeros(n)
        for i in range(n):
            fc = fc_hz[i]
            if abs(fc - last_fc) > 1e-9:
                lpf_a = 1.0 - math.exp(-TWO_PI * max(fc, 1.0) * ts)
                if lpf_a < 0.0:
                    lpf_a = 0.0
                if lpf_a > 1.0:
                    lpf_a = 1.0
                last_fc = fc
            for _ in range(steps):
                if not smo:
                    ihat_a = ia[i]
                    ihat_b = ib[i]
                    smo = True
                    continue
                da = (ihat_a - ia[i]) * inv
                db = (ihat_b - ib[i]) * inv
                if da > 1.0:
                    da = 1.0
                elif da < -1.0:
                    da = -1.0
                if db > 1.0:
                    db = 1.0
                elif db < -1.0:
                    db = -1.0
                za = SMO_K * da
                zb = SMO_K * db
                ihat_a = disc_a * ihat_a + disc_b * (ua[i] - za)
                ihat_b = disc_a * ihat_b + disc_b * (ub[i] - zb)
                ea += lpf_a * (za - ea)
                eb += lpf_a * (zb - eb)
                emag = math.sqrt(ea * ea + eb * eb)
                if not pll:
                    pll_th = th[i] + THETA_OFF
                    while pll_th > PI:
                        pll_th -= TWO_PI
                    while pll_th < -PI:
                        pll_th += TWO_PI
                    pll_w = 0.0
                    pll_int = 0.0
                    pll = True
                    continue
                c = math.cos(pll_th)
                s = math.sin(pll_th)
                pd = (-ea * c - eb * s) / (emag + PLL_NORM_EPS)
                pll_int += ki * pd * ts
                if pll_int > PLL_W_LIM:
                    pll_int = PLL_W_LIM
                elif pll_int < -PLL_W_LIM:
                    pll_int = -PLL_W_LIM
                cmd = pll_int + kp * pd
                if cmd > PLL_W_LIM:
                    pll_w = PLL_W_LIM
                    pll_int = PLL_W_LIM - kp * pd
                    if pll_int > PLL_W_LIM:
                        pll_int = PLL_W_LIM
                    elif pll_int < -PLL_W_LIM:
                        pll_int = -PLL_W_LIM
                elif cmd < -PLL_W_LIM:
                    pll_w = -PLL_W_LIM
                    pll_int = -PLL_W_LIM - kp * pd
                    if pll_int > PLL_W_LIM:
                        pll_int = PLL_W_LIM
                    elif pll_int < -PLL_W_LIM:
                        pll_int = -PLL_W_LIM
                else:
                    pll_w = cmd
                pll_th += pll_w * ts
                while pll_th > PI:
                    pll_th -= TWO_PI
                while pll_th < -PI:
                    pll_th += TWO_PI
            th_hat = pll_th - THETA_OFF
            while th_hat > PI:
                th_hat -= TWO_PI
            while th_hat < -PI:
                th_hat += TWO_PI
            ep = th_hat - th[i]
            while ep > PI:
                ep -= TWO_PI
            while ep < -PI:
                ep += TWO_PI
            err_p[i] = ep
        return err_p

    def replay(ia, ib, ua, ub, th, ts, fc_hz, steps: int = 2):
        return _replay_nb(
            np.ascontiguousarray(ia, np.float64),
            np.ascontiguousarray(ib, np.float64),
            np.ascontiguousarray(ua, np.float64),
            np.ascontiguousarray(ub, np.float64),
            np.ascontiguousarray(th, np.float64),
            ts,
            np.ascontiguousarray(fc_hz, np.float64),
            int(steps),
        )

else:

    def replay(ia, ib, ua, ub, th, ts, fc_hz, steps: int = 2):
        n = len(ia)
        disc_a = math.exp(-R_OHM * ts / L_H)
        disc_b = (1.0 - disc_a) / R_OHM
        inv = 1.0 / SAT_A
        wn = TWO_PI * PLL_FN_HZ
        kp = 2.0 * PLL_ZETA * wn
        ki = wn * wn
        ihat_a = ihat_b = ea = eb = 0.0
        smo = pll = False
        pll_th = pll_w = pll_int = 0.0
        last_fc = -1.0
        lpf_a = 1.0
        err_p = np.zeros(n)
        for i in range(n):
            fc = float(fc_hz[i])
            if abs(fc - last_fc) > 1e-9:
                lpf_a = 1.0 - math.exp(-TWO_PI * max(fc, 1.0) * ts)
                lpf_a = min(max(lpf_a, 0.0), 1.0)
                last_fc = fc
            for _ in range(steps):
                if not smo:
                    ihat_a, ihat_b = float(ia[i]), float(ib[i])
                    smo = True
                    continue
                za = SMO_K * min(max((ihat_a - float(ia[i])) * inv, -1.0), 1.0)
                zb = SMO_K * min(max((ihat_b - float(ib[i])) * inv, -1.0), 1.0)
                ihat_a = disc_a * ihat_a + disc_b * (float(ua[i]) - za)
                ihat_b = disc_a * ihat_b + disc_b * (float(ub[i]) - zb)
                ea += lpf_a * (za - ea)
                eb += lpf_a * (zb - eb)
                emag = math.hypot(ea, eb)
                if not pll:
                    pll_th = wrap_pi(float(th[i]) + THETA_OFF)
                    pll_w = pll_int = 0.0
                    pll = True
                    continue
                pd = (-ea * math.cos(pll_th) - eb * math.sin(pll_th)) / (emag + PLL_NORM_EPS)
                pll_int = min(max(pll_int + ki * pd * ts, -PLL_W_LIM), PLL_W_LIM)
                cmd = pll_int + kp * pd
                pll_w = min(max(cmd, -PLL_W_LIM), PLL_W_LIM)
                pll_th = wrap_pi(pll_th + pll_w * ts)
            err_p[i] = wrap_pi(wrap_pi(pll_th - THETA_OFF) - float(th[i]))
        return err_p


def smooth_rpm(rpm: np.ndarray) -> np.ndarray:
    r = np.clip(rpm, -2000.0, 2000.0)
    try:
        from scipy.ndimage import median_filter

        return median_filter(r, size=401, mode="nearest")
    except Exception:
        step = 50
        idx = np.arange(0, len(r), step)
        m = np.array([np.median(r[max(0, i - 200) : i + 201]) for i in idx])
        return np.interp(np.arange(len(r)), idx, m)


def summarize(name: str, err_p: np.ndarray, fs: float, n: int) -> None:
    print(f"\n=== {name} ===")
    print(f"{'rpm':>5} {'Pl95':>7} {'Plmed':>7} {'debias95':>9}")
    for tgt, t0, t1 in WINDOWS:
        s = int(t0 * fs)
        e = min(int(t1 * fs), n)
        if e <= s:
            continue
        ep = err_p[s:e]
        med = float(np.median(ep))
        print(
            f"{tgt:5d} {np.percentile(np.abs(ep), 95) * 180 / PI:7.2f} "
            f"{med * 180 / PI:7.2f} "
            f"{np.percentile(np.abs(ep - med), 95) * 180 / PI:9.2f}"
        )


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", type=Path)
    ap.add_argument("--fs", type=float, default=10000.0, help="VOFA sample rate")
    ap.add_argument("--isr-hz", type=float, default=20000.0, help="FOC/SMO rate on MCU")
    ap.add_argument("--zoh-steps", type=int, default=2, help="ISR steps per telem sample (fs*steps≈isr)")
    ap.add_argument("--mode", choices=("fixed", "linear", "both"), default="both")
    ap.add_argument("--fc", type=str, default="120,145,160,180,200")
    ap.add_argument("--k", type=str, default="1.5,1.7,2.0")
    ap.add_argument("--fc-min", type=float, default=100.0)
    ap.add_argument("--fc-max", type=float, default=280.0)
    args = ap.parse_args()

    raw = np.genfromtxt(args.csv, delimiter=",", skip_header=1)
    if raw.ndim != 2 or raw.shape[1] < 12:
        print("need 12-col RAW CSV", file=sys.stderr)
        return 2

    fs = args.fs
    ts = 1.0 / args.isr_hz
    steps = args.zoh_steps
    ia, ib = raw[:, 0], raw[:, 1]
    ua, ub = raw[:, 2], raw[:, 3]
    th = raw[:, 4]
    rpm_s = smooth_rpm(raw[:, 5])
    n = len(raw)

    print(
        f"file={args.csv.name} n={n} dur={n/fs:.1f}s telem_fs={fs:.0f} "
        f"replay={steps}×{ts*1e6:.0f}µs numba={HAS_NUMBA}"
    )

    fe = np.abs(rpm_s) * POLE_PAIRS / 60.0

    if args.mode in ("fixed", "both"):
        for fc in parse_floats(args.fc):
            err_p = replay(ia, ib, ua, ub, th, ts, np.full(n, fc), steps)
            summarize(f"fixed_fc={fc:.0f}", err_p, fs, n)

    if args.mode in ("linear", "both"):
        for k in parse_floats(args.k):
            fc_arr = np.clip(k * fe, args.fc_min, args.fc_max)
            err_p = replay(ia, ib, ua, ub, th, ts, fc_arr, steps)
            summarize(
                f"linear_k={k:.2f}_clip[{args.fc_min:.0f},{args.fc_max:.0f}]",
                err_p,
                fs,
                n,
            )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
