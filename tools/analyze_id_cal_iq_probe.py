#!/usr/bin/env python3
"""
Id 标定全流程 VOFA 11 通道分析，重点 Iq 探路段（Pass1 结束 → OFF → LUT ON）。

通道（M1_TELEM_BRINGUP_K=11）:
  ch0-2 Ia,Ib,Ic  ch3 Id  ch4 Iq  ch5 theta_el
  ch6 Ud_pi  ch7 Uq_pi  ch8-10 Ta,Tb,Tc

用法:
  python tools/analyze_id_cal_iq_probe.py VOFA+CSV/20260630/vofa+202607010058.csv
  python tools/analyze_id_cal_iq_probe.py a.csv b.csv --compare
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

TH30 = 0.5235987755982988
FS_DEFAULT = 10000.0
IQ_REF = 0.5
UQ_SAT = 5.9


def unwrap(theta: np.ndarray) -> np.ndarray:
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


def load_csv(path: Path) -> dict[str, np.ndarray]:
    data = np.genfromtxt(path, delimiter=",", names=True, dtype=float)
    names = data.dtype.names
    if names is None or len(names) < 11:
        raise ValueError(f"{path}: need 11 columns, got {names}")
    keys = ["ia", "ib", "ic", "id", "iq", "theta", "ud", "uq", "ta", "tb", "tc"]
    return {k: data[names[i]] for i, k in enumerate(keys)}


def find_pass1_end(c: dict[str, np.ndarray], fs: float) -> int:
    """Last sample still in Pass1: Id>2.5 A, theta pinned ~30 deg."""
    id_, theta = c["id"], c["theta"]
    n = len(id_)
    win = max(1, int(0.2 * fs))
    for i in range(n - win, int(45 * fs), -50):
        sl = slice(i, i + win)
        if (
            np.nanmean(id_[sl]) > 2.5
            and np.std(theta[sl]) < 0.02
            and np.mean(np.abs(theta[sl] - TH30) < 0.02) > 0.95
        ):
            return i + win
    hi = (c["id"] > 1.0) & (np.abs(c["theta"] - TH30) < 0.05)
    if np.any(hi):
        return int(np.where(hi)[0][-1])
    return int(0.75 * n)


def seg_stats(c: dict[str, np.ndarray], a: int, b: int, fs: float) -> dict:
    th = c["theta"][a:b]
    thu = unwrap(th)
    dth = np.diff(thu) if len(thu) > 1 else np.array([0.0])
    om = np.median(np.abs(dth)) * fs if len(dth) else 0.0
    uq = c["uq"][a:b]
    return {
        "n": b - a,
        "dur_s": (b - a) / fs,
        "theta_span": float(th.max() - th.min()) if len(th) else 0.0,
        "theta_rot_rad": float(thu[-1] - thu[0]) if len(thu) > 1 else 0.0,
        "omega_med_rad_s": float(om),
        "spin_pct": float(100 * np.mean(np.abs(dth) * fs > 5)) if len(dth) else 0.0,
        "id_mean": float(np.nanmean(c["id"][a:b])),
        "iq_mean": float(np.nanmean(c["iq"][a:b])),
        "iq_err": float(np.nanmean(c["iq"][a:b]) - IQ_REF),
        "ud_mean": float(np.nanmean(c["ud"][a:b])),
        "uq_mean": float(np.nanmean(uq)),
        "uq_sat_pct": float(100 * np.mean(uq > UQ_SAT)) if len(uq) else 0.0,
        "fix30_pct": float(100 * np.mean(np.abs(th - TH30) < 0.02)) if len(th) else 0.0,
    }


def analyze_one(path: Path, fs: float = FS_DEFAULT) -> dict:
    c = load_csv(path)
    n = len(c["id"])
    t = np.arange(n) / fs
    p1 = find_pass1_end(c, fs)
    t_p1 = t[p1]
    iq_off_end = p1 + int(5.0 * fs)

    lut_idx = np.where(np.abs(c["ia"] + 888888.0) < 1.0)[0]
    t_lut = float(t[lut_idx[0]]) if len(lut_idx) else None

    segs = {
        "pass1_tail_3s": seg_stats(c, max(0, p1 - int(3 * fs)), p1, fs),
        "iq_off_5s": seg_stats(c, p1, min(n, iq_off_end), fs),
        "iq_lut_on": seg_stats(c, min(n, iq_off_end), n, fs),
    }

    # per-second after Pass1 (first 8 s)
    per_sec = []
    for s in range(8):
        a = p1 + s * int(fs)
        b = min(n, a + int(fs))
        if a >= n:
            break
        st = seg_stats(c, a, b, fs)
        st["t_rel_s"] = s
        per_sec.append(st)

    return {
        "path": str(path),
        "name": path.name,
        "duration_s": float(t[-1]),
        "rows": n,
        "t_pass1_end_s": float(t_p1),
        "t_lut_burst_s": t_lut,
        "segments": segs,
        "per_sec_after_pass1": per_sec,
    }


def fmt_seg(label: str, s: dict) -> str:
    return (
        f"| {label} | {s['dur_s']:.1f} | {s['fix30_pct']:.0f}% | "
        f"{s['id_mean']:+.2f} | {s['iq_mean']:+.3f} | {s['iq_err']:+.3f} | "
        f"{s['uq_mean']:+.2f} | {s['uq_sat_pct']:.0f}% | "
        f"{s['omega_med_rad_s']:.0f} | {s['theta_span']:.3f} |"
    )


def print_report(r: dict) -> None:
    print(f"\n{'=' * 72}")
    print(f"File: {r['name']}")
    print(f"Rows: {r['rows']:,}  Duration: {r['duration_s']:.1f} s")
    print(f"Pass1 end: t={r['t_pass1_end_s']:.2f} s")
    if r["t_lut_burst_s"] is not None:
        print(f"LUT burst: t={r['t_lut_burst_s']:.2f} s")

    print("\n| Segment | dur(s) | fix30% | Id | Iq | Iq err | Uq | Uq sat% | omega | th_span |")
    print("|:---|---:|---:|---:|---:|---:|---:|---:|---:|---:|")
    for key, title in [
        ("pass1_tail_3s", "Pass1 tail 3s"),
        ("iq_off_5s", "Iq OFF 5s"),
        ("iq_lut_on", "Iq LUT ON → end"),
    ]:
        print(fmt_seg(title, r["segments"][key]))

    print("\nPer-second after Pass1:")
    print("| +s | Iq | Iq err | Uq | Uq sat% | omega | th_span |")
    print("|---:|---:|---:|---:|---:|---:|---:|")
    for ps in r["per_sec_after_pass1"]:
        print(
            f"| {ps['t_rel_s']} | {ps['iq_mean']:.3f} | {ps['iq_err']:+.3f} | "
            f"{ps['uq_mean']:.2f} | {ps['uq_sat_pct']:.0f}% | "
            f"{ps['omega_med_rad_s']:.0f} | {ps['theta_span']:.3f} |"
        )


def main() -> int:
    ap = argparse.ArgumentParser(description="Analyze Id-cal VOFA CSV with Iq probe focus")
    ap.add_argument("csv", nargs="+", type=Path, help="VOFA CSV path(s)")
    ap.add_argument("--fs", type=float, default=FS_DEFAULT)
    ap.add_argument("--compare", action="store_true")
    args = ap.parse_args()

    results = [analyze_one(p, args.fs) for p in args.csv]
    for r in results:
        print_report(r)

    if args.compare and len(results) >= 2:
        a, b = results[0], results[1]
        print(f"\n{'=' * 72}")
        print(f"COMPARE: {a['name']}  vs  {b['name']}")
        for key, title in [("iq_off_5s", "Iq OFF"), ("iq_lut_on", "Iq LUT ON")]:
            sa, sb = a["segments"][key], b["segments"][key]
            print(f"\n{title}:")
            print(f"  Iq:  {sa['iq_mean']:.3f} -> {sb['iq_mean']:.3f}")
            print(f"  Uq:  {sa['uq_mean']:.2f}V -> {sb['uq_mean']:.2f}V")
            print(f"  omega: {sa['omega_med_rad_s']:.0f} -> {sb['omega_med_rad_s']:.0f} rad/s")
            print(f"  th_span: {sa['theta_span']:.3f} -> {sb['theta_span']:.3f} rad")

    return 0


if __name__ == "__main__":
    sys.exit(main())
