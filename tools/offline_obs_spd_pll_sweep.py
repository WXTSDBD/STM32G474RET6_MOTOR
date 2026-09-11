#!/usr/bin/env python3
"""Offline sweep: EMF-PLL θ̂ → motor_pll (speed-loop Type-II) Fn.

Uses OBS PLL VOFA packs (not RAW):
  old layout (2316/2258):  ch3 err_pll  ch5 we_el  ch7 theta_hat
  new layout (2328):       ch0 enc_rpm  ch1 obs_spd  ch3 err_pll  ch7 theta_hat

Replays firmware motor_pll on electrical θ̂; ω_el → mech rpm.
Reference: motor_pll(Fn=80) on reconstructed θ_enc = wrap(θ̂ − err_pll).

Default ZOH ×2 (20 kHz) to match ISR.

Examples:
  python tools/offline_obs_spd_pll_sweep.py VOFA+CSV/20260910/vofa+202609102316.csv
  python tools/offline_obs_spd_pll_sweep.py VOFA+CSV/20260910/vofa+202609102258.csv --fn 10,15,20,25,40
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

PI = math.pi
TWO_PI = 2.0 * PI
POLE_PAIRS = 7.0
ZETA = 0.707106781
ENC_FN = 80.0  # encoder speed-PLL Fn (reference)
WLIM_EL = 650.0 * POLE_PAIRS


def wrap_pi(x: np.ndarray | float) -> np.ndarray | float:
    return (x + PI) % TWO_PI - PI


def load_pack(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray, str]:
    raw = np.genfromtxt(path, delimiter=",", skip_header=1)
    if raw.ndim != 2 or raw.shape[1] < 8:
        raise SystemExit(f"bad csv {path}")
    # Heuristic: new SPD layout has ch0 looking like rpm (~hundreds), old has eα (~volts)
    c0 = raw[:, 0]
    if np.nanmedian(np.abs(c0)) > 20.0:
        layout = "spd"
        err = raw[:, 3]
        hat = raw[:, 7]
        we_el = raw[:, 5] * (TWO_PI * POLE_PAIRS) / 60.0  # stored as rpm
    else:
        layout = "pll"
        err = raw[:, 3]
        hat = raw[:, 7]
        we_el = raw[:, 5]
    return hat.astype(np.float64), err.astype(np.float64), we_el.astype(np.float64), layout


def motor_pll_track(
    theta_meas: np.ndarray,
    dt: float,
    fn_hz: float,
    zeta: float = ZETA,
    wlim: float = WLIM_EL,
) -> np.ndarray:
    """Return omega [rad/s] same domain as theta_meas."""
    wn = TWO_PI * fn_hz
    kp = 2.0 * zeta * wn
    ki = wn * wn
    n = theta_meas.shape[0]
    omega = np.zeros(n, dtype=np.float64)
    th = float(theta_meas[0])
    integ = 0.0
    for i in range(n):
        meas = float(theta_meas[i])
        e = wrap_pi(meas - th)
        integ += ki * e * dt
        if integ > wlim:
            integ = wlim
        elif integ < -wlim:
            integ = -wlim
        cmd = integ + kp * e
        if cmd > wlim:
            w = wlim
            integ = wlim - kp * e
            if integ > wlim:
                integ = wlim
            elif integ < -wlim:
                integ = -wlim
        elif cmd < -wlim:
            w = -wlim
            integ = -wlim - kp * e
            if integ > wlim:
                integ = wlim
            elif integ < -wlim:
                integ = -wlim
        else:
            w = cmd
        th = th + w * dt
        omega[i] = w
    return omega


if HAS_NUMBA:

    @njit(cache=True)
    def _pll_numba(theta_meas, dt, kp, ki, wlim):
        n = theta_meas.shape[0]
        omega = np.zeros(n)
        th = theta_meas[0]
        integ = 0.0
        for i in range(n):
            meas = theta_meas[i]
            e = meas - th
            while e > PI:
                e -= TWO_PI
            while e < -PI:
                e += TWO_PI
            integ += ki * e * dt
            if integ > wlim:
                integ = wlim
            elif integ < -wlim:
                integ = -wlim
            cmd = integ + kp * e
            if cmd > wlim:
                w = wlim
                integ = wlim - kp * e
                if integ > wlim:
                    integ = wlim
                elif integ < -wlim:
                    integ = -wlim
            elif cmd < -wlim:
                w = -wlim
                integ = -wlim - kp * e
                if integ > wlim:
                    integ = wlim
                elif integ < -wlim:
                    integ = -wlim
            else:
                w = cmd
            th = th + w * dt
            omega[i] = w
        return omega

    def motor_pll_track(theta_meas, dt, fn_hz, zeta=ZETA, wlim=WLIM_EL):
        wn = TWO_PI * fn_hz
        kp = 2.0 * zeta * wn
        ki = wn * wn
        return _pll_numba(theta_meas.astype(np.float64), dt, kp, ki, wlim)


def zoh_dup(x: np.ndarray, steps: int) -> np.ndarray:
    if steps <= 1:
        return x
    return np.repeat(x, steps)


def rpm_from_wel(w_el: np.ndarray) -> np.ndarray:
    return w_el * 60.0 / (TWO_PI * POLE_PAIRS)


def score_window(obs: np.ndarray, ref: np.ndarray) -> dict:
    e = obs - ref
    return {
        "bias": float(np.median(e)),
        "std": float(np.std(e)),
        "p95": float(np.percentile(np.abs(e), 95)),
        "rms": float(np.sqrt(np.mean(e * e))),
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("csv", type=Path)
    ap.add_argument("--fs", type=float, default=10000.0, help="telem Hz")
    ap.add_argument("--zoh-steps", type=int, default=2, help="ISR upsample (2→20kHz)")
    ap.add_argument("--fn", type=str, default="8,10,12,15,20,25,30,40,50,60,80")
    ap.add_argument("--zeta", type=float, default=ZETA)
    ap.add_argument(
        "--window",
        type=str,
        default="auto",
        help="t0,t1 seconds or 'auto'",
    )
    args = ap.parse_args()

    hat, err, we_el, layout = load_pack(args.csv)
    n = hat.shape[0]
    t = np.arange(n) / args.fs
    print(f"file={args.csv.name} layout={layout} dur={t[-1]:.1f}s n={n}")

    if args.window == "auto":
        # Prefer late steady high-speed: last 8s if long, else middle
        if t[-1] >= 16.0:
            t0, t1 = 8.0, min(18.0, t[-1] - 0.2)
        elif t[-1] >= 40.0:
            # ladder: 1000 rpm dwell ~ hold+4*dwell for 6s dwell from 6 → ~30-36
            t0, t1 = 30.0, 36.0
            if t1 > t[-1]:
                t0, t1 = t[-1] - 8.0, t[-1] - 0.2
        else:
            t0, t1 = max(0.0, t[-1] - 4.0), t[-1] - 0.05
    else:
        parts = [float(x) for x in args.window.split(",")]
        t0, t1 = parts[0], parts[1]

    print(f"score window t=[{t0:.1f},{t1:.1f}]s  zoh={args.zoh_steps} zeta={args.zeta}")

    steps = max(1, args.zoh_steps)
    dt = 1.0 / (args.fs * steps)
    hat_u = zoh_dup(hat, steps)
    err_u = zoh_dup(err, steps)
    we_u = zoh_dup(we_el, steps)
    th_enc = wrap_pi(hat_u - err_u)

    # Reference: encoder angle through Fn=80 motor_pll (electrical → rpm)
    w_enc = motor_pll_track(th_enc, dt, ENC_FN, args.zeta)
    rpm_enc = rpm_from_wel(w_enc)
    rpm_raw = rpm_from_wel(we_u)

    # Map window to upsampled indices
    i0 = int(t0 * args.fs * steps)
    i1 = int(t1 * args.fs * steps)
    i0 = max(0, min(i0, len(rpm_enc) - 2))
    i1 = max(i0 + 1, min(i1, len(rpm_enc)))

    fns = [float(x) for x in args.fn.split(",") if x.strip()]
    print(
        f"{'Fn':>6} {'bias':>8} {'std':>8} {'p95':>8} {'rms':>8}  "
        f"| vs raw_emf p95"
    )
    rows = []
    for fn in fns:
        w_obs = motor_pll_track(hat_u, dt, fn, args.zeta)
        rpm_obs = rpm_from_wel(w_obs)
        s = score_window(rpm_obs[i0:i1], rpm_enc[i0:i1])
        s_raw = score_window(rpm_raw[i0:i1], rpm_enc[i0:i1])
        rows.append((fn, s, s_raw))
        print(
            f"{fn:6.1f} {s['bias']:+8.2f} {s['std']:8.2f} {s['p95']:8.2f} {s['rms']:8.2f}  "
            f"| raw_p95={s_raw['p95']:.1f}"
        )

    # Best by p95 then std
    rows_sorted = sorted(rows, key=lambda r: (r[1]["p95"], r[1]["std"]))
    best = rows_sorted[0]
    print(
        f"\nbest Fn={best[0]:.1f} Hz  "
        f"bias={best[1]['bias']:+.2f} std={best[1]['std']:.2f} "
        f"p95={best[1]['p95']:.2f} rpm"
    )
    print(
        f"enc ref = motor_pll(θ_enc, Fn={ENC_FN})  "
        f"raw EMF-PLL ω p95 err={best[2]['p95']:.1f} rpm"
    )
    print(
        "note: for speed-loop use, prefer low p95 with bias~0; "
        "too-low Fn lags on accel (check ramp separately)."
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
