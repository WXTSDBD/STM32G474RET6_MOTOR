#!/usr/bin/env python3
"""
Large 20 kHz VOFA CSV: summary / decimate / window export (streaming, low RAM).

Channels (M1_VOFA_FOC_ABC=1): ch0-2=Ia/Ib/Ic, ch3=Iq, ch4=Id, ch5=theta_el

Examples:
  python tools/process_vofa_20k.py VOFA+CSV/20260618/vofa+202606190137.csv --summary
  python tools/process_vofa_20k.py ... --summary --skip 12 --duration 2
  python tools/process_vofa_20k.py ... --decimate 100 --skip 12 --duration 5 -o out_200hz.csv
  python tools/process_vofa_20k.py ... --decimate 20 --skip 12 --duration 1 -o out_1khz.csv
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import sys
from typing import Iterator, List, Optional, Tuple

DEFAULT_FS = 20000.0


def iter_samples(
    path: str,
    skip: int = 0,
    max_src_rows: Optional[int] = None,
    decimate: int = 1,
) -> Iterator[Tuple[int, List[float]]]:
    """Yield (row_index, [6 floats]) with optional skip / decimate."""
    with open(path, encoding="utf-8", newline="") as f:
        reader = csv.reader(f)
        next(reader, None)
        for i, parts in enumerate(reader):
            if len(parts) < 6:
                continue
            if i < skip:
                continue
            if max_src_rows is not None and (i - skip) >= max_src_rows:
                break
            if ((i - skip) % decimate) != 0:
                continue
            row = [float(parts[j]) for j in range(6)]
            yield i, row


def run_summary(
    path: str,
    fs: float,
    skip: int,
    max_rows: Optional[int],
    decimate: int,
) -> None:
    n = 0
    sum_iq = sum_id = 0.0
    sum_iq2 = 0.0
    min_iq = 1e9
    max_iq = -1e9
    ss_park = 0.0
    neg_iq = 0

    for _, row in iter_samples(path, skip, max_rows, decimate):
        ia, ib, ic, iq, id_, th = row
        ibe = (ib - ic) / math.sqrt(3.0)
        c, s = math.cos(th), math.sin(th)
        iq_c = -ia * s + ibe * c
        d = iq - iq_c
        ss_park += d * d
        if iq < -0.1:
            neg_iq += 1
        sum_iq += iq
        sum_id += id_
        sum_iq2 += iq * iq
        if iq < min_iq:
            min_iq = iq
        if iq > max_iq:
            max_iq = iq
        n += 1

    if n == 0:
        print("No samples in range.", file=sys.stderr)
        sys.exit(1)

    eff_fs = fs / decimate
    dur = n / eff_fs
    sz_mb = os.path.getsize(path) / (1024 * 1024)

    print(f"=== VOFA 20k summary: {os.path.basename(path)} ===")
    print(f"File size: {sz_mb:.2f} MB")
    print(f"Segment: skip={skip} rows, decimate={decimate}, N={n}")
    print(f"Effective Fs: {eff_fs:.1f} Hz, duration: {dur:.3f} s")
    print()
    print(f"Iq  mean={sum_iq/n:+.4f} A  rms={math.sqrt(sum_iq2/n):.4f} A")
    print(f"    min={min_iq:.4f}  max={max_iq:.4f}  Iq<-0.1A: {100*neg_iq/n:.2f}%")
    print(f"Id  mean={sum_id/n:+.4f} A")
    print(f"Park self-check rms|Iq_fw-Iq_calc| = {math.sqrt(ss_park/n):.6f} A")
    print()
    print("Tips:")
    print("  - Full file in VOFA for eyeball; use --decimate 100 for offline_recalc / analyze_foc_vofa")
    print("  - PI ripple: --decimate 1 --duration 0.05 (~1k samples) + FFT in Python/MATLAB")
    print("  - Long run stats: --decimate 100 or --decimate 200")


def run_export(
    path: str,
    out_path: str,
    skip: int,
    max_rows: Optional[int],
    decimate: int,
) -> None:
    n = 0
    with open(out_path, "w", encoding="utf-8", newline="") as out:
        w = csv.writer(out)
        w.writerow(["I0", "I1", "I2", "I3", "I4", "I5"])
        for _, row in iter_samples(path, skip, max_rows, decimate):
            w.writerow([f"{v:.6f}" for v in row])
            n += 1
    sz = os.path.getsize(out_path) / 1024
    print(f"Wrote {n} rows -> {out_path} ({sz:.1f} KB)")


def main() -> int:
    ap = argparse.ArgumentParser(description="Stream-process large 20 kHz VOFA CSV")
    ap.add_argument("csv", help="input CSV")
    ap.add_argument("--fs", type=float, default=DEFAULT_FS, help="native sample rate (default 20000)")
    ap.add_argument("--skip", type=float, default=0.0, help="skip first N seconds")
    ap.add_argument("--duration", type=float, default=None, help="max seconds to process (default: all)")
    ap.add_argument("--decimate", type=int, default=1, help="keep every Nth sample (1=full rate)")
    ap.add_argument("-o", "--output", default=None, help="export decimated CSV")
    ap.add_argument("--summary", action="store_true", help="print stats only (default if no -o)")
    args = ap.parse_args()

    if not os.path.isfile(args.csv):
        print(f"Not found: {args.csv}", file=sys.stderr)
        return 1
    if args.decimate < 1:
        print("--decimate must be >= 1", file=sys.stderr)
        return 1

    skip_rows = int(args.skip * args.fs)
    max_rows = None
    if args.duration is not None:
        max_rows = int(args.duration * args.fs)

    if args.output:
        run_export(args.csv, args.output, skip_rows, max_rows, args.decimate)
        if args.summary:
            run_summary(args.csv, args.fs, skip_rows, max_rows, args.decimate)
    else:
        run_summary(args.csv, args.fs, skip_rows, max_rows, args.decimate)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
