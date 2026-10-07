#!/usr/bin/env python3
"""Analyze experiment A open-loop Uq ladder CSV (ch5 = open_seq_phase 70..76)."""

import csv
import sys
from collections import defaultdict
from pathlib import Path


def analyze(path: Path) -> None:
    rows = []
    with path.open(newline="", encoding="utf-8") as f:
        reader = csv.reader(f)
        next(reader, None)
        for row in reader:
            if len(row) < 6:
                continue
            try:
                rows.append([float(x) for x in row[:6]])
            except ValueError:
                continue

    print(f"\n=== {path.name}  rows={len(rows)} ===")

    phases = defaultdict(list)
    for v in rows:
        ph = round(v[5])
        if 68 <= ph <= 78:
            phases[ph].append(v)

    uq_cmd = {70: 0.0, 71: 0.2, 72: 0.5, 73: 1.0, 74: 2.0}
    dmm_mv = {70: 1, 71: 7, 72: 15, 73: 22, 74: 223}

    print(f"{'ph':>4} {'Uq_cmd':>7} {'Uref':>8} {'duty_d':>8} {'Iq':>8} {'Uq_out':>7} {'Uq/Iq':>8} {'n':>8}")
    for ph in sorted(phases):
        chunk = phases[ph]
        n = max(1, len(chunk) // 5)
        tail = chunk[-n:]
        uref = sum(x[0] for x in tail) / len(tail)
        duty = sum(x[1] for x in tail) / len(tail)
        iq = sum(x[3] for x in tail) / len(tail)
        uq = sum(x[4] for x in tail) / len(tail)
        cmd = uq_cmd.get(ph, float("nan"))
        r_vofa = (uq / iq) if abs(iq) > 0.01 else float("nan")
        print(
            f"{ph:4d} {cmd:7.2f} {uref:8.5f} {duty:8.5f} {iq:8.4f} {uq:7.3f} {r_vofa:8.3f} {len(chunk):8d}"
        )

    if dmm_mv:
        print("\nPaste DMM mV per phase into script dmm_mv dict for DMM/Iq column.")

    uniq = sorted({round(v[5]) for v in rows if 60 <= v[5] <= 120})
    print("ch5 phases seen:", uniq)


def main() -> None:
    root = Path(__file__).resolve().parents[1] / "VOFA+CSV" / "20260629"
    paths = sorted(root.glob("vofa+*.csv"), key=lambda p: p.stat().st_mtime, reverse=True)
    if not paths:
        print("No CSV found", file=sys.stderr)
        sys.exit(1)
    for path in paths[:2]:
        analyze(path)


if __name__ == "__main__":
    main()
