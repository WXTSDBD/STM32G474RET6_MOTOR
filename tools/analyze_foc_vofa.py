#!/usr/bin/env python3
"""Analyze FOC VOFA CSV: Ia/Ib/Ic, Iq/Id, theta_el + theta_est from currents."""

import argparse
import csv
import math
import sys
from pathlib import Path

import numpy as np

POLE_PAIRS = 7
DEFAULT_FS_HZ = 200.0


def load_csv(path: Path):
    data = np.genfromtxt(path, delimiter=",", names=True, dtype=float)
    names = data.dtype.names
    if names is None or len(names) < 6:
        raise ValueError(f"Expected 6 columns, got {names}")
    ia, ib, ic = data[names[0]], data[names[1]], data[names[2]]
    iq, id_ = data[names[3]], data[names[4]]
    theta_el = data[names[5]]
    return ia, ib, ic, iq, id_, theta_el


def wrap_pi(x):
    return (x + math.pi) % (2.0 * math.pi) - math.pi


def wrap_array(x):
    return np.array([wrap_pi(v) for v in x])


def unwrap(theta):
    out = np.empty_like(theta)
    out[0] = theta[0]
    for i in range(1, len(theta)):
        d = theta[i] - theta[i - 1]
        if d > math.pi:
            d -= 2.0 * math.pi
        elif d < -math.pi:
            d += 2.0 * math.pi
        out[i] = out[i - 1] + d
    return out


def theta_est_from_abc(ia, ib, ic):
    i_beta = (ib - ic) / math.sqrt(3.0)
    return np.arctan2(i_beta, ia) - 0.5 * math.pi


def park_recompute(ia, ib, ic, theta_el, delta_rad=0.0):
    i_alpha = ia
    i_beta = (ib - ic) / math.sqrt(3.0)
    th = theta_el + delta_rad
    c, s = np.cos(th), np.sin(th)
    id_ = i_alpha * c + i_beta * s
    iq = -i_alpha * s + i_beta * c
    return id_, iq


def sweep_theta_correction(ia, ib, ic, theta_el, deg_min=-90.0, deg_max=90.0, steps=361):
    th_u = unwrap(theta_el)
    best = None
    for delta_deg in np.linspace(deg_min, deg_max, steps):
        id2, iq2 = park_recompute(ia, ib, ic, th_u, math.radians(delta_deg))
        id_rms = float(np.sqrt(np.mean(id2 ** 2)))
        iq_rms = float(np.sqrt(np.mean(iq2 ** 2)))
        ratio = id_rms / max(iq_rms, 1e-6)
        row = {
            "delta_deg": float(delta_deg),
            "id_rms": id_rms,
            "iq_rms": iq_rms,
            "ratio": ratio,
        }
        if best is None or ratio < best["ratio"]:
            best = row
    return best


def harmonic_1f(y, theta_u):
    c, s = np.cos(theta_u), np.sin(theta_u)
    return float(np.mean(y * c)), float(np.mean(y * s))


def analyze(path: Path, skip_s: float, export: Path | None, add_nominal: float, fs_hz: float):
    ia, ib, ic, iq, id_, theta_el = load_csv(path)
    n = len(ia)
    t = np.arange(n) / fs_hz

    skip_n = int(skip_s * fs_hz)
    if skip_n >= n - 100:
        skip_n = max(0, n // 10)

    sl = slice(skip_n, None)
    ia_s, ib_s, ic_s = ia[sl], ib[sl], ic[sl]
    iq_s, id_s = iq[sl], id_[sl]
    th_s = theta_el[sl]
    t_s = t[sl] - t[skip_n]

    th_unwrap = unwrap(th_s)
    th_est = theta_est_from_abc(ia_s, ib_s, ic_s)
    th_est_unwrap = unwrap(th_est)

    offset = float(np.median(th_unwrap - th_est_unwrap))
    th_est_aligned = th_est_unwrap + offset
    err = wrap_array(th_unwrap - th_est_aligned)

    iq_rms = float(np.sqrt(np.mean(iq_s ** 2)))
    id_rms = float(np.sqrt(np.mean(id_s ** 2)))
    iq_mean = float(np.mean(iq_s))
    id_mean = float(np.mean(id_s))

    if len(th_unwrap) > 1:
        dth = np.diff(th_unwrap)
        omega_el_med = float(np.median(dth * fs_hz))
        rpm_mech = omega_el_med / (2.0 * math.pi * POLE_PAIRS) * 60.0
    else:
        omega_el_med = 0.0
        rpm_mech = 0.0

    i_sum_rms = float(np.sqrt(np.mean((ia_s + ib_s + ic_s) ** 2)))
    th_range = float(th_unwrap[-1] - th_unwrap[0])
    duration = float(t_s[-1])

    id1c, id1s = harmonic_1f(id_s, th_unwrap)
    iq1c, iq1s = harmonic_1f(iq_s, th_unwrap)

    best = sweep_theta_correction(ia_s, ib_s, ic_s, th_s)
    add_suggest = add_nominal + math.radians(best["delta_deg"])

    print(f"=== FOC VOFA Analysis: {path.name} ===")
    print(f"Samples: {n}  Fs: {fs_hz:.0f} Hz  duration: {n/fs_hz:.2f}s  skip: {skip_s}s")
    print(f"Steady segment: {duration:.2f}s")
    print()
    print("--- Id / Iq (firmware Park, steady) ---")
    print(f"  Iq  mean={iq_mean:+.3f} A  rms={iq_rms:.3f} A  peak={float(np.max(np.abs(iq_s))):.3f} A")
    print(f"  Id  mean={id_mean:+.3f} A  rms={id_rms:.3f} A  peak={float(np.max(np.abs(id_s))):.3f} A")
    print(f"  |Id|/|Iq| rms = {id_rms/iq_rms if iq_rms > 1e-6 else float('inf'):.3f}")
    decoupled = id_rms < 0.35 * iq_rms and iq_rms > 0.1
    print(f"  Decoupled (|Id|<<|Iq|): {'YES' if decoupled else 'NO'}")
    print()
    print("--- Id/Iq 1f harmonic w.r.t theta_el (large => angle error) ---")
    print(f"  Id: cos={id1c:+.3f}  sin={id1s:+.3f}  amp={math.hypot(id1c,id1s):.3f} A")
    print(f"  Iq: cos={iq1c:+.3f}  sin={iq1s:+.3f}  amp={math.hypot(iq1c,iq1s):.3f} A")
    print()
    print("--- Theta ---")
    print(f"  theta_el unwrap: {th_range:.1f} rad ({th_range/(2*math.pi):.1f} e-rev)")
    print(f"  omega_el: {omega_el_med:.2f} rad/s  mech rpm: {rpm_mech:.1f}")
    print(f"  theta_est vs theta_el: offset={math.degrees(offset):.1f} deg")
    print(f"  err p95={math.degrees(float(np.percentile(np.abs(err), 95))):.1f} deg")
    print()
    print("--- Theta correction sweep (offline re-Park) ---")
    print(f"  best delta={best['delta_deg']:+.1f} deg")
    print(f"  -> Id_rms={best['id_rms']:.3f}  Iq_rms={best['iq_rms']:.3f}  ratio={best['ratio']:.3f}")
    print(f"  suggested add = {add_nominal:.3f} + {math.radians(best['delta_deg']):+.4f} rad = {add_suggest:.4f} rad")
    print()
    print("--- KCL Ia+Ib+Ic sum RMS ---")
    print(f"  {i_sum_rms:.4f} A")
    print()

    if abs(th_range) < 0.5:
        print("*** STALL: theta_el barely moved ***")
    if iq_rms < 0.05 and id_rms > 0.5:
        print("*** BAD add: high Id, low Iq ***")
    if not decoupled and best["ratio"] < 0.35:
        print("*** Angle error: offline -47deg class fix greatly improves Id/Iq ***")

    if export is not None:
        id_opt, iq_opt = park_recompute(ia_s, ib_s, ic_s, th_unwrap, math.radians(best["delta_deg"]))
        export.parent.mkdir(parents=True, exist_ok=True)
        with export.open("w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow([
                "t_s", "Ia", "Ib", "Ic", "Iq_fw", "Id_fw", "theta_el",
                "theta_est", "theta_err_deg", "Id_opt", "Iq_opt",
            ])
            for i in range(len(t_s)):
                w.writerow([
                    f"{t_s[i]:.4f}",
                    f"{ia_s[i]:.6f}", f"{ib_s[i]:.6f}", f"{ic_s[i]:.6f}",
                    f"{iq_s[i]:.6f}", f"{id_s[i]:.6f}", f"{th_s[i]:.6f}",
                    f"{th_est[i]:.6f}", f"{math.degrees(err[i]):.4f}",
                    f"{id_opt[i]:.6f}", f"{iq_opt[i]:.6f}",
                ])
        print(f"Exported: {export}")

    return best


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", nargs="?", default="vofa+202606160039.csv")
    parser.add_argument("--skip", type=float, default=1.0)
    parser.add_argument("--fs", type=float, default=None, help="sample rate Hz (auto 20000 if >100k rows else 200)")
    parser.add_argument("--export", type=Path, default=None)
    parser.add_argument("--no-export", action="store_true", help="skip writing *_processed.csv")
    parser.add_argument("--add", type=float, default=5.04, help="nominal encoder add (rad)")
    parser.add_argument("--dir", type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args()

    path = Path(args.csv)
    if not path.is_absolute():
        path = args.dir / path
    if not path.exists():
        print(f"File not found: {path}", file=sys.stderr)
        sys.exit(1)

    n_lines = sum(1 for _ in path.open(encoding="utf-8")) - 1
    fs_hz = args.fs
    if fs_hz is None:
        fs_hz = 20000.0 if n_lines > 100_000 else DEFAULT_FS_HZ

    export = args.export
    if export is None and not args.no_export and n_lines <= 50_000:
        export = path.with_name(path.stem + "_processed.csv")
    elif args.no_export:
        export = None

    analyze(path, args.skip, export, args.add, fs_hz)


if __name__ == "__main__":
    main()
