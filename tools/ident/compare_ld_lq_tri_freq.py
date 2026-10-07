#!/usr/bin/env python3
"""Compare MCU 500Hz / 1kHz / 2kHz Ld-Lq grid vs LCR reference."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))

from parse_ident_vofa import find_header, load_csv, parse_burst  # noqa: E402

LD_LCR = 59.0
LQ_LCR = 87.0


def pct(val: float, ref: float) -> float:
    return 100.0 * (val / ref - 1.0)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("csv", type=Path)
    ap.add_argument("--ld-lcr", type=float, default=LD_LCR)
    ap.add_argument("--lq-lcr", type=float, default=LQ_LCR)
    ap.add_argument("--tol", type=float, default=15.0, help="|err|%% threshold")
    args = ap.parse_args()

    rows = load_csv(args.csv)
    hi = find_header(rows, use_last=True)
    if hi < 0:
        print("no ident burst", file=sys.stderr)
        return 1
    m = parse_burst(rows, hi)
    print(f"{args.csv.name}  proto={m['proto']:.1f}  burst row {hi}")
    print(f"LCR: Ld={args.ld_lcr:.0f} uH  Lq={args.lq_lcr:.0f} uH  tol=±{args.tol:.0f}%")
    print()
    print(f"{'(Id,Iq)':>12}  {'Ld500':>8} {'Ld1k':>8} {'Ld2k':>8}  {'Lq500':>8} {'Lq1k':>8} {'Lq2k':>8}")
    print("-" * 72)
    for g in m["grid"]:
        def cell(val, valid, ref):
            if not valid or val != val:
                return "   —   "
            e = pct(val, ref)
            mark = "OK" if abs(e) <= args.tol else "--"
            return f"{val:6.1f}{mark}"

        line = (
            f"({g['id_bias']:+.1f},{g['iq_bias']:+.1f})  "
            f"{cell(g['ld_coarse_uH'], g['ld_coarse_valid'], args.ld_lcr):>8} "
            f"{cell(g['ld_fine_uH'], g['ld_fine_valid'], args.ld_lcr):>8} "
            f"{cell(g.get('ld_f2_uH', float('nan')), g.get('ld_f2_valid', False), args.ld_lcr):>8}  "
            f"{cell(g['lq_coarse_uH'], g['lq_coarse_valid'], args.lq_lcr):>8} "
            f"{cell(g['lq_fine_uH'], g['lq_fine_valid'], args.lq_lcr):>8} "
            f"{cell(g.get('lq_f2_uH', float('nan')), g.get('lq_f2_valid', False), args.lq_lcr):>8}"
        )
        print(line)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
