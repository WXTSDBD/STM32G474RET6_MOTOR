#!/usr/bin/env python3
"""Experiment B: legacy 6ch or 12ch (see analyze_open_ladder_12ch.py --axis ud)."""

import csv
import sys
from collections import defaultdict
from pathlib import Path

VBUS = 24.0
RS = 0.115
SAMPLE_HZ_LEGACY = 20000.0
SAMPLE_HZ_12CH = 10000.0


def ud_bin(ud: float) -> float:
    for cmd in (0.0, 0.2, 0.5, 1.0, 2.0):
        if abs(ud - cmd) < 0.05:
            return cmd
    return round(ud, 3)


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
        print(f"{path.name}: 12ch CSV — run: python tools/analyze_open_ladder_12ch.py --axis ud {path}")
        return

    hz = SAMPLE_HZ_LEGACY
    print(f"\n=== {path.name}  rows={len(rows)}  duration~{len(rows)/hz:.1f}s ===")

    by_ud = defaultdict(list)
    for v in rows:
        ud = ud_bin(v[4])
        if ud in (0.0, 0.2, 0.5, 1.0, 2.0):
            by_ud[ud].append(v)

    if not by_ud:
        print("No Ud ladder steps found (Ud_out 0/0.2/0.5/1/2).")
        return

    print(
        f"{'Ud':>6} {'Uref':>8} {'duty_d':>8} {'duty_ab':>8} "
        f"{'Id':>8} {'theta':>8} {'Ud/Id':>8} {'Rs*I mV':>9} {'Id/(Ud/Rs)':>10}"
    )
    for key in sorted(by_ud.keys()):
        chunk = by_ud[key]
        n = max(1, len(chunk) // 5)
        tail = chunk[-n:]
        uref = sum(x[0] for x in tail) / len(tail)
        duty = sum(x[1] for x in tail) / len(tail)
        dab = sum(x[2] for x in tail) / len(tail)
        id_ = sum(x[3] for x in tail) / len(tail)
        ud = sum(x[4] for x in tail) / len(tail)
        th = sum(x[5] for x in tail) / len(tail)
        r_eff = (ud / id_) if abs(id_) > 0.01 else float("nan")
        rs_mv = RS * id_ * 1000.0
        expect = (id_ / (ud / RS)) if ud > 0.01 else float("nan")
        print(
            f"{ud:6.3f} {uref:8.5f} {duty:8.5f} {dab:8.5f} "
            f"{id_:8.4f} {th:8.4f} {r_eff:8.3f} {rs_mv:9.1f} {expect:10.3f}"
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
