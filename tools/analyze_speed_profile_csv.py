#!/usr/bin/env python3
"""Analyze speed-loop VOFA CSV (12ch unified layout with profile)."""

import argparse
from pathlib import Path

import numpy as np

FS_HZ = 10000.0  # M1_TELEM_BRINGUP_DECIMATION=2 @ 20kHz FOC tick
CH = ["Ia", "Ib", "Ic", "Id", "Iq", "theta_el", "Ud", "Uq",
      "omega_pll", "omega_ref", "iq_ref", "omega_err"]


def load(path: Path) -> dict[str, np.ndarray]:
    data = np.genfromtxt(path, delimiter=",", skip_header=1, dtype=np.float32)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    if data.shape[1] < 12:
        raise ValueError(f"expected >=12 cols, got {data.shape[1]}")
    return {CH[i]: data[:, i] for i in range(12)}


def plateau_segments(omega_ref: np.ndarray, tol: float = 1.0):
    ref = np.round(omega_ref).astype(int)
    n = len(ref)
    i = 0
    while i < n:
        v = ref[i]
        j = i + 1
        while j < n and abs(ref[j] - v) <= tol:
            j += 1
        yield v, i, j
        i = j


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", type=Path)
    ap.add_argument("--fs", type=float, default=FS_HZ)
    ap.add_argument("--compare", type=Path, default=None)
    args = ap.parse_args()

    cols = load(args.csv)
    n = len(cols["Ia"])
    fs = args.fs
    dur = n / fs

    print(f"File: {args.csv.name}")
    print(f"Rows={n}  fs={fs:.0f}Hz  duration={dur:.1f}s  size={args.csv.stat().st_size/1e6:.1f}MB")
    print()

    print("=== Whole recording ===")
    for k in ["Id", "Iq", "omega_pll", "omega_ref", "iq_ref", "omega_err", "Ud", "Uq"]:
        x = cols[k]
        print(
            f"  {k:12s} min={x.min():8.2f} max={x.max():8.2f} "
            f"mean={x.mean():8.2f} std={x.std():8.3f}"
        )
    iqr = cols["iq_ref"]
    print(
        f"  iq_ref sat ±5A: +{100*np.mean(iqr>=4.99):.1f}%  -{100*np.mean(iqr<=-4.99):.1f}%"
    )
    refs = sorted(set(np.round(cols["omega_ref"]).astype(int)))
    print(f"  omega_ref levels: {refs}")
    print()

    print("=== Per plateau (middle 60%, min 3s) ===")
    print(
        f"{'ref':>5} {'t0':>7} {'dur':>6} {'ω_pll':>8} {'ω_std':>6} "
        f"{'err':>8} {'|e|<30':>7} {'Iqσ':>6} {'Idσ':>6} {'iq_ref':>7} {'sat%':>5}"
    )
    rows = []
    for ref, s, e in plateau_segments(cols["omega_ref"]):
        seg_len = e - s
        if seg_len < int(fs * 3):
            continue
        m0 = s + int(seg_len * 0.2)
        m1 = s + int(seg_len * 0.8)
        sl = slice(m0, m1)
        w = cols["omega_pll"][sl]
        err = cols["omega_err"][sl]
        iq = cols["Iq"][sl]
        id_ = cols["Id"][sl]
        iqr = cols["iq_ref"][sl]
        row = {
            "ref": ref,
            "t0": s / fs,
            "dur": seg_len / fs,
            "omega_mean": float(w.mean()),
            "omega_std": float(w.std()),
            "err_mean": float(err.mean()),
            "err_abs_lt30": float(np.mean(np.abs(err) < 30) * 100),
            "iq_std": float(iq.std()),
            "id_std": float(id_.std()),
            "iq_ref_mean": float(iqr.mean()),
            "iq_ref_sat": float(np.mean(np.abs(iqr) >= 4.99) * 100),
        }
        rows.append(row)
        print(
            f"{row['ref']:5d} {row['t0']:7.1f} {row['dur']:6.1f} "
            f"{row['omega_mean']:8.1f} {row['omega_std']:6.1f} "
            f"{row['err_mean']:+8.1f} {row['err_abs_lt30']:6.1f}% "
            f"{row['iq_std']:6.3f} {row['id_std']:6.3f} "
            f"{row['iq_ref_mean']:+7.2f} {row['iq_ref_sat']:5.1f}"
        )

    for label, t0, t1 in [("startup 0-5s", 0, 5), ("steady last 10s", max(0, dur - 10), dur)]:
        sl = slice(int(t0 * fs), int(t1 * fs))
        print(f"\n=== {label} ===")
        for k in ["omega_pll", "omega_ref", "iq_ref", "Iq", "Id", "omega_err"]:
            x = cols[k][sl]
            print(f"  {k:12s} mean={x.mean():8.2f} std={x.std():8.3f}")

    if args.compare and args.compare.exists():
        c2 = load(args.compare)
        print(f"\n=== Compare vs {args.compare.name} (whole file Iq/Id std) ===")
        for k in ["Iq", "Id", "omega_err"]:
            print(f"  {k:12s} this={cols[k].std():.3f}  ref={c2[k].std():.3f}")


if __name__ == "__main__":
    main()
