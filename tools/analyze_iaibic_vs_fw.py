#!/usr/bin/env python3
"""
Compare firmware Iq with offline Clarke+Park using the SAME formulas as trans.c / motor_current.c.

Two VOFA modes (auto-detected):
  A) ch0-2 = foc_ia/b/c (A)  — direct Park, must match ch3 if firmware is consistent
  B) ch0-2 = adc_zeroed (LSB) — binding [2,1,0] sign [-1,-1,-1] × scale → ia/ib/ic

Usage:
  python tools/analyze_iaibic_vs_fw.py <csv_or_dir> [--skip N] [--recon]
"""
from __future__ import annotations

import argparse
import csv
import glob
import math
import os
import sys
from dataclasses import dataclass
from typing import List, Optional, Tuple

M1_ADC_SCALE = 3.3 / (4096 * 0.01 * 10)
SKIP_DEFAULT = 400
NEG_A = 0.1
RECON_MIN = 0.05
INV_SQRT3 = 1.0 / math.sqrt(3.0)


@dataclass
class Row:
    c: List[float]


def read_csv(path: str) -> List[Row]:
    rows: List[Row] = []
    with open(path, encoding="utf-8", newline="") as f:
        r = csv.reader(f)
        next(r, None)
        for p in r:
            if len(p) >= 6:
                rows.append(Row([float(x) for x in p[:6]]))
    return rows


def detect_mode(rows: List[Row], skip: int) -> str:
    """Return 'abc' if ch0-2 look like foc_ia/b/c in Amps, else 'raw'."""
    sample = rows[skip : skip + 200] if len(rows) > skip + 200 else rows[skip:]
    if not sample:
        return "raw"
    ints = 0
    amps = 0
    for row in sample:
        for k in range(3):
            v = row.c[k]
            if abs(v - round(v)) < 1e-3 and abs(v) >= 1.0:
                ints += 1
            if abs(v) > 0.02 and abs(v) < 50 and abs(v - round(v)) > 0.01:
                amps += 1
    return "abc" if amps > ints else "raw"


def binding_raw(r0: float, r1: float, r2: float) -> Tuple[float, float, float]:
    ia = -r2 * M1_ADC_SCALE
    ib = -r1 * M1_ADC_SCALE
    ic = -r0 * M1_ADC_SCALE
    return ia, ib, ic


def recon(ia: float, ib: float, ic: float, sec: int) -> float:
    if sec not in (1, 2):
        return ic
    if abs(ia) < RECON_MIN and abs(ib) < RECON_MIN and abs(ic) < RECON_MIN:
        return ic
    return -(ia + ib)


def clarke(ia: float, ib: float, ic: float) -> Tuple[float, float]:
    return ia, (ib - ic) * INV_SQRT3


def park(i_alpha: float, i_beta: float, theta: float) -> Tuple[float, float]:
    c, s = math.cos(theta), math.sin(theta)
    id_ = i_alpha * c + i_beta * s
    iq = -i_alpha * s + i_beta * c
    return id_, iq


def sector_from_ch4(ch4: float) -> int:
    s = int(round(ch4))
    return s if 1 <= s <= 6 else 0


def is_sector_diag(rows: List[Row], skip: int) -> bool:
    sample = rows[skip : skip + 500]
    if not sample:
        return False
    in16 = sum(1 for row in sample if 1 <= int(round(row.c[4])) <= 6)
    return in16 > len(sample) * 0.5


@dataclass
class Result:
    path: str
    mode: str
    sector_diag: bool
    n: int
    rms_iq: float
    corr_iq: float
    agree_sign: float
    neg_fw: float
    neg_off: float
    rms_id: float
    sec_stats: dict


def analyze(path: str, skip: int, do_recon: bool) -> Optional[Result]:
    rows = read_csv(path)
    if len(rows) <= skip + 50:
        return None

    mode = detect_mode(rows, skip)
    sec_diag = is_sector_diag(rows, skip) if mode == "raw" else False

    fw_iq: List[float] = []
    off_iq: List[float] = []
    off_id: List[float] = []
    sectors: dict = {s: {"fw": [], "off": []} for s in range(1, 7)}

    for i, row in enumerate(rows):
        if i < skip:
            continue
        th = row.c[5]
        iq_fw = row.c[3]

        if mode == "abc":
            ia, ib, ic = row.c[0], row.c[1], row.c[2]
        else:
            ia, ib, ic = binding_raw(row.c[0], row.c[1], row.c[2])
            if do_recon and sec_diag:
                sec = sector_from_ch4(row.c[4])
                ic = recon(ia, ib, ic, sec)

        al, be = clarke(ia, ib, ic)
        id_, iq = park(al, be, th)

        fw_iq.append(iq_fw)
        off_iq.append(iq)
        off_id.append(id_)

        if sec_diag:
            sec = sector_from_ch4(row.c[4])
            if sec >= 1:
                sectors[sec]["fw"].append(iq_fw)
                sectors[sec]["off"].append(iq)

    n = len(fw_iq)
    if n == 0:
        return None

    import numpy as np

    fw = np.array(fw_iq)
    oq = np.array(off_iq)
    od = np.array(off_id)
    err = fw - oq
    rms = float(np.sqrt(np.mean(err**2)))
    corr = float(np.corrcoef(fw, oq)[0, 1]) if np.std(oq) > 1e-9 else float("nan")
    agree = float(np.mean((fw >= 0) == (oq >= 0)) * 100)
    neg_fw = float(np.mean(fw < -NEG_A) * 100)
    neg_off = float(np.mean(oq < -NEG_A) * 100)
    rms_id = float(np.sqrt(np.mean((fw - od) ** 2)))

    sec_stats = {}
    for s in range(1, 7):
        if not sectors[s]["fw"]:
            continue
        f = np.array(sectors[s]["fw"])
        o = np.array(sectors[s]["off"])
        sec_stats[s] = {
            "n": len(f),
            "rms": float(np.sqrt(np.mean((f - o) ** 2))),
            "agree": float(np.mean((f >= 0) == (o >= 0)) * 100),
            "neg_fw": float(np.mean(f < -NEG_A) * 100),
            "neg_off": float(np.mean(o < -NEG_A) * 100),
        }

    return Result(
        path=path,
        mode=mode,
        sector_diag=sec_diag,
        n=n,
        rms_iq=rms,
        corr_iq=corr,
        agree_sign=agree,
        neg_fw=neg_fw,
        neg_off=neg_off,
        rms_id=rms_id,
        sec_stats=sec_stats,
    )


def collect(target: str) -> List[str]:
    if os.path.isdir(target):
        ps = sorted(glob.glob(os.path.join(target, "**", "*.csv"), recursive=True))
        out = []
        for p in ps:
            bn = os.path.basename(p).lower()
            if "_processed" in bn or "_offline_recalc" in bn:
                continue
            if "sector_dial" in bn or "uq2" in bn or bn.startswith("vofa+20260616") or bn.startswith("vofa+20260617"):
                out.append(p)
        return out
    return [target]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("target", nargs="?", default="VOFA+CSV")
    ap.add_argument("--skip", type=int, default=SKIP_DEFAULT)
    ap.add_argument("--recon", action="store_true")
    ap.add_argument("--detail", action="store_true")
    args = ap.parse_args()

    root = args.target
    if not os.path.isabs(root):
        root = os.path.join(os.path.dirname(os.path.dirname(__file__)), root)

    paths = collect(root)
    # also root-level vofa+ uq2/binding era
    for p in sorted(glob.glob(os.path.join(os.path.dirname(os.path.dirname(__file__)), "vofa+20260616*.csv"))):
        if "_processed" not in p:
            paths.append(p)

    paths = sorted(set(paths))
    results: List[Result] = []
    for p in paths:
        try:
            r = analyze(p, args.skip, args.recon)
            if r:
                results.append(r)
        except Exception as e:
            print(f"SKIP {p}: {e}", file=sys.stderr)

    print("=" * 100)
    print("Offline Park(Clarke(ia,ib,ic), theta) vs firmware ch3  —  same formulas as trans.c")
    print("=" * 100)
    print(
        f"{'file':<42} {'mode':>4} {'sec?':>4} {'N':>6} {'rms':>7} {'corr':>6} "
        f"{'sgn%':>6} {'fw<0':>6} {'off<0':>6} {'rms|fw-Id|':>10}"
    )
    print("-" * 100)
    for r in results:
        name = os.path.basename(r.path)
        if len(name) > 41:
            name = "..." + name[-38:]
        sd = "Y" if r.sector_diag else "N"
        print(
            f"{name:<42} {r.mode:>4} {sd:>4} {r.n:>6} {r.rms_iq:>7.3f} {r.corr_iq:>6.3f} "
            f"{r.agree_sign:>5.1f}% {r.neg_fw:>5.1f}% {r.neg_off:>5.1f}% {r.rms_id:>10.3f}"
        )

    print()
    print("Interpretation:")
    print("  mode=abc : ch0-2 IS foc_ia/b/c → rms~0 means firmware self-consistent; rms>>0 means firmware bug in Park/ch3")
    print("  mode=raw : ch0-2 IS adc_zeroed → offline ia/ib/ic = binding(raw); mismatch localizes bug before/after Clarke")

    abc_ok = [r for r in results if r.mode == "abc" and r.rms_iq < 0.05]
    abc_bad = [r for r in results if r.mode == "abc" and r.rms_iq >= 0.05]
    raw_m = [r for r in results if r.mode == "raw"]

    print()
    print(f"  foc_ia/b/c CSVs with rms<50mA (Park OK): {len(abc_ok)}")
    print(f"  foc_ia/b/c CSVs with rms>=50mA (Park BUG): {len(abc_bad)}")
    print(f"  adc_zeroed CSVs: {len(raw_m)}")

    if args.detail or len(results) <= 3:
        for r in results:
            if not r.sec_stats:
                continue
            print()
            print(f"  Sectors: {os.path.basename(r.path)}")
            print(f"  {'sec':>3} {'N':>5} {'rms':>7} {'sgn%':>6} {'fw<0':>6} {'off<0':>6}")
            for s in sorted(r.sec_stats):
                st = r.sec_stats[s]
                print(
                    f"  {s:>3} {st['n']:>5} {st['rms']:>7.3f} {st['agree']:>5.1f}% "
                    f"{st['neg_fw']:>5.1f}% {st['neg_off']:>5.1f}%"
                )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
