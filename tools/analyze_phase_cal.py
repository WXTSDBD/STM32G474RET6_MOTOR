#!/usr/bin/env python3
"""Analyze M1 phase/gain diagnostic VOFA CSV (zeroed LSB + pwm/delta/st tags)."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

FS_HZ = 200.0
DELTA_CCR = (400, 600, 800)
DOM_RANK_DEFAULT = (2, 1, 0)


def load_csv(path: Path):
    d = np.genfromtxt(path, delimiter=",", names=True, dtype=float)
    n = d.dtype.names
    return d[n[0]], d[n[1]], d[n[2]], d[n[3]], d[n[4]], d[n[5]]


def find_cal_regions(st: np.ndarray) -> list[tuple[int, int]]:
    cal = np.round(st) != 4
    regions: list[tuple[int, int]] = []
    i = 0
    n = len(st)
    while i < n:
        if not cal[i]:
            i += 1
            continue
        j = i
        while j < n and cal[j]:
            j += 1
        if j - i > 500:
            regions.append((i, j))
        i = j
    return regions


def segment_means(ch0, ch1, ch2, pwm, delta, st, pwm_i, delta_i, st_i):
    m = (
        (np.round(pwm) == float(pwm_i))
        & (np.round(delta) == float(delta_i))
        & (np.round(st) == float(st_i))
    )
    if not np.any(m):
        return None
    return (
        float(np.mean(ch0[m])),
        float(np.mean(ch1[m])),
        float(np.mean(ch2[m])),
        int(np.sum(m)),
    )


def analyze_run(ch0, ch1, ch2, pwm, delta, st, label: str, dom_rank: tuple[int, ...]) -> list[str]:
    lines = [f"--- {label} ---"]
    dom600 = []
    for p in range(3):
        row = []
        for t, _dccr in enumerate(DELTA_CCR):
            pos = segment_means(ch0, ch1, ch2, pwm, delta, st, p, t, 1)
            neg = segment_means(ch0, ch1, ch2, pwm, delta, st, p, t, 2)
            if pos is None or neg is None:
                row.append(0.0)
                continue
            s0, s1, s2 = pos[0] - neg[0], pos[1] - neg[1], pos[2] - neg[2]
            svals = [s0, s1, s2]
            row.append(abs(svals[dom_rank[p]]))
            if t == 1:
                lines.append(
                    f"  CH{p} S=[{s0:+.0f},{s1:+.0f},{s2:+.0f}] dom=rank{dom_rank[p]}"
                )
        if row[0] > 1:
            lines.append(
                f"  CH{p} |S| d400/600/800={row[0]:.0f}/{row[1]:.0f}/{row[2]:.0f}  "
                f"600/400={row[1]/row[0]:.2f} 800/600={row[2]/row[1]:.2f}"
            )
        dom600.append(row[1] if len(row) > 1 else 0.0)

    if dom600[0] > 1:
        lines.append(
            f"  Gain @600: CH1/CH0={dom600[1]/dom600[0]:.3f}  CH2/CH0={dom600[2]/dom600[0]:.3f}"
        )

    m = (
        (np.round(pwm) == 0)
        & (np.round(delta) == 0)
        & (np.round(st) == 0)
    )
    if np.any(m):
        kcl = float(np.sqrt(np.mean((ch0[m] + ch1[m] + ch2[m]) ** 2)))
        lines.append(f"  neutral KCL rms={kcl:.1f} LSB")

    for p in range(3):
        m = (np.round(pwm) == p) & (np.round(delta) == 1) & (np.round(st) == 1)
        z = [ch0, ch1, ch2][dom_rank[p]][m]
        if len(z):
            lines.append(
                f"  CH{p} d600 pos rank{dom_rank[p]} std={float(z.std()):.1f} mean={float(z.mean()):.1f}"
            )
    lines.append("")
    return lines


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv")
    args = parser.parse_args()
    path = Path(args.csv)
    ch0, ch1, ch2, pwm, delta, st = load_csv(path)

    print(f"=== phase cal analysis: {path.name} ===")
    print(f"samples={len(ch0)}  duration={len(ch0)/FS_HZ:.1f}s")
    print(f"Δ CCR: {DELTA_CCR}")
    print(f"Rank map (default): PWM0->rank2, PWM1->rank1, PWM2->rank0")
    print()

    regions = find_cal_regions(st)
    all_lines = [
        f"# phase cal — {path.name}",
        f"# samples={len(ch0)}  runs={len(regions)}",
        "",
    ]

    if len(regions) <= 1:
        lines = analyze_run(ch0, ch1, ch2, pwm, delta, st, "full", DOM_RANK_DEFAULT)
        print("\n".join(lines))
        all_lines.extend(lines)
    else:
        for i, (s, e) in enumerate(regions):
            lab = f"Run {i+1} t=[{s/FS_HZ:.1f},{e/FS_HZ:.1f})s"
            if i == len(regions) - 1 and "2325" in path.name:
                lab += " (likely hand-lock if last)"
            lines = analyze_run(
                ch0[s:e], ch1[s:e], ch2[s:e], pwm[s:e], delta[s:e], st[s:e], lab, DOM_RANK_DEFAULT
            )
            print("\n".join(lines))
            all_lines.extend(lines)

    hold = int(np.sum(np.round(st) == 4))
    if hold:
        msg = f"hold segment (st=4): {hold} samples"
        print(msg)
        all_lines.append(msg)

    txt = path.with_suffix(".txt")
    txt.write_text("\n".join(all_lines), encoding="utf-8")
    print(f"Report: {txt}")


if __name__ == "__main__":
    main()
