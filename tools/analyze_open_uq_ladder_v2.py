#!/usr/bin/env python3
"""Experiment A: legacy 6ch or 12ch (see analyze_open_ladder_12ch.py --axis uq)."""

import csv
import sys
from collections import defaultdict
from pathlib import Path

VBUS = 24.0
RS = 0.115
SAMPLE_HZ_LEGACY = 20000.0


def uq_bin(uq: float) -> float:
    """Snap Uq_out to ladder command for segmentation."""
    for cmd in (0.0, 0.2, 0.5, 1.0, 2.0):
        if abs(uq - cmd) < 0.05:
            return cmd
    return round(uq, 3)


def analyze(path: Path) -> None:
    rows = []
    ncols = 0
    with path.open(newline="", encoding="utf-8") as f:
        reader = csv.reader(f)
        next(reader, None)
        for row in reader:
            if len(row) < 6:
                continue
            try:
                vals = [float(x) for x in row]
            except ValueError:
                continue
            ncols = max(ncols, len(vals))
            rows.append(vals)

    if ncols >= 12:
        print(f"{path.name}: 12ch CSV — run: python tools/analyze_open_ladder_12ch.py --axis uq {path}")
        return

    hz = SAMPLE_HZ_LEGACY
    print(f"\n=== {path.name}  rows={len(rows)}  duration~{len(rows)/hz:.1f}s ===")

    by_uq = defaultdict(list)
    by_phase = defaultdict(list)
    for v in rows:
        ph = round(v[5])
        if 68 <= ph <= 78:
            by_phase[ph].append(v)
        uq = uq_bin(v[4])
        if uq in (0.0, 0.2, 0.5, 1.0, 2.0):
            by_uq[uq].append(v)

    buckets = by_uq if len(by_uq) >= 3 else by_phase
    label = "Uq_out" if buckets is by_uq else "phase"

    if not buckets:
        print("No ladder steps found (Uq_out or phase 70-74).")
        return

    print(
        f"Segment by {label}\n"
        f"{'step':>6} {'Uq_out':>7} {'Uref':>8} {'duty_d':>8} {'duty_ab':>8} "
        f"{'Iq':>8} {'theta':>8} {'Uq/Iq':>8} {'Rs*I mV':>9}"
    )
    keys = sorted(buckets.keys(), key=lambda x: (isinstance(x, str), x))
    for key in keys:
        chunk = buckets[key]
        n = max(1, len(chunk) // 5)
        tail = chunk[-n:]
        uref = sum(x[0] for x in tail) / len(tail)
        duty = sum(x[1] for x in tail) / len(tail)
        dab = sum(x[2] for x in tail) / len(tail)
        iq = sum(x[3] for x in tail) / len(tail)
        uq = sum(x[4] for x in tail) / len(tail)
        th = sum(x[5] for x in tail) / len(tail)
        r_eff = (uq / iq) if abs(iq) > 0.01 else float("nan")
        rs_mv = RS * iq * 1000.0
        print(
            f"{key!s:>6} {uq:7.3f} {uref:8.5f} {duty:8.5f} {dab:8.5f} "
            f"{iq:8.4f} {th:8.4f} {r_eff:8.3f} {rs_mv:9.1f}"
        )


def main() -> None:
    if len(sys.argv) > 1:
        paths = [Path(sys.argv[1])]
    else:
        root = Path(__file__).resolve().parents[1] / "VOFA+CSV"
        paths = sorted(root.rglob("vofa+*.csv"), key=lambda p: p.stat().st_mtime, reverse=True)[:1]
    for p in paths:
        analyze(p)


if __name__ == "__main__":
    main()
