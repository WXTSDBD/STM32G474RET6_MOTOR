#!/usr/bin/env python3
"""Open-loop ladder CSV (12ch @ D=2): K_pwm and K_plant per phase step."""

import csv
import sys
from collections import defaultdict
from pathlib import Path

VBUS = 24.0
RS = 0.115
PWM_PERIOD = 3999.0
SAMPLE_HZ = 10000.0  # 20kHz / M1_TELEM_BRINGUP_DECIMATION(2)

# 12ch layout (phase 68..77)
CH_UD = 0
CH_UQ = 1
CH_VD = 2
CH_VQ = 3
CH_ID = 4
CH_IQ = 5
CH_CCR1 = 6
CH_CCR2 = 7
CH_CCR3 = 8
CH_UREF = 9
CH_SECTOR = 10
CH_PHASE = 11

LADDER_PHASES = (70, 71, 72, 73, 74, 75)
LADDER_UD = (0.0, 0.2, 0.5, 1.0, 2.0, 4.0)
LADDER_UQ = LADDER_UD


def load_rows(path: Path) -> tuple[list[list[float]], int]:
    rows: list[list[float]] = []
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
    return rows, ncols


def tail_median(chunk: list[list[float]], key: int) -> float:
    n = max(1, len(chunk) // 5)
    tail = chunk[-n:]
    return sum(r[key] for r in tail) / len(tail)


def analyze(path: Path, axis: str) -> None:
    rows, ncols = load_rows(path)
    if not rows:
        print(f"{path.name}: no data")
        return

    is_12ch = ncols >= 12
    duration = len(rows) / SAMPLE_HZ
    print(f"\n=== {path.name}  rows={len(rows)}  cols={ncols}  ~{duration:.1f}s @ {SAMPLE_HZ:.0f}Hz ===")

    if not is_12ch:
        print("Legacy 6ch CSV — use analyze_open_ud_ladder.py / analyze_open_uq_ladder_v2.py")
        return

    by_phase: dict[int, list[list[float]]] = defaultdict(list)
    for r in rows:
        ph = int(round(r[CH_PHASE]))
        if 68 <= ph <= 77:
            by_phase[ph].append(r)

    if axis == "ud":
        cmd_ch, est_ch, i_ch, cmd_name, est_name, i_name = (
            CH_UD, CH_VD, CH_ID, "Ud", "Vd_est", "Id"
        )
        ladder_cmds = LADDER_UD
    else:
        cmd_ch, est_ch, i_ch, cmd_name, est_name, i_name = (
            CH_UQ, CH_VQ, CH_IQ, "Uq", "Vq_est", "Iq"
        )
        ladder_cmds = LADDER_UQ

    print(
        f"{'phase':>5} {cmd_name+'_cmd':>8} {est_name:>8} {i_name:>8} "
        f"{'K_pwm':>8} {'K_plant':>8} {'1/Rs':>8} "
        f"{'CCR1':>7} {'CCR2':>7} {'CCR3':>7} {'Uref':>8} {'sec':>4}"
    )

    for ph in sorted(by_phase.keys()):
        chunk = by_phase[ph]
        u_cmd = tail_median(chunk, cmd_ch)
        v_est = tail_median(chunk, est_ch)
        i_val = tail_median(chunk, i_ch)
        ccr1 = tail_median(chunk, CH_CCR1)
        ccr2 = tail_median(chunk, CH_CCR2)
        ccr3 = tail_median(chunk, CH_CCR3)
        uref = tail_median(chunk, CH_UREF)
        sec = tail_median(chunk, CH_SECTOR)

        k_pwm = (v_est / u_cmd) if abs(u_cmd) > 0.01 else float("nan")
        k_plant = (i_val / v_est) if abs(v_est) > 0.01 else float("nan")
        inv_rs = 1.0 / RS

        print(
            f"{ph:5d} {u_cmd:8.3f} {v_est:8.3f} {i_val:8.4f} "
            f"{k_pwm:8.3f} {k_plant:8.3f} {inv_rs:8.2f} "
            f"{ccr1:7.0f} {ccr2:7.0f} {ccr3:7.0f} {uref:8.5f} {sec:4.0f}"
        )

    print("\nLadder steps (phase 70..75):")
    print(
        f"{'step':>5} {cmd_name:>6} {est_name:>8} {i_name:>8} "
        f"{'K_pwm':>8} {'K_plant':>8} {'expect I':>9}"
    )
    for ph, u_nom in zip(LADDER_PHASES, ladder_cmds):
        chunk = by_phase.get(ph, [])
        if not chunk:
            continue
        u_cmd = tail_median(chunk, cmd_ch)
        v_est = tail_median(chunk, est_ch)
        i_val = tail_median(chunk, i_ch)
        k_pwm = (v_est / u_cmd) if abs(u_cmd) > 0.01 else float("nan")
        k_plant = (i_val / v_est) if abs(v_est) > 0.01 else float("nan")
        expect_i = (u_nom / RS) if u_nom > 0.01 else 0.0
        print(
            f"{ph:5d} {u_cmd:6.3f} {v_est:8.3f} {i_val:8.4f} "
            f"{k_pwm:8.3f} {k_plant:8.3f} {expect_i:9.3f}"
        )

    print(
        f"\nInterpret: K_pwm=V_est/U_cmd (~1 => PWM OK); "
        f"K_plant=I/V_est (~{1/RS:.1f} A/V => current OK)"
    )


def main() -> None:
    axis = "ud"
    paths: list[Path] = []
    args = sys.argv[1:]
    if args and args[0] in ("-a", "--axis"):
        axis = args[1]
        args = args[2:]
    if args:
        paths = [Path(args[0])]
    else:
        root = Path(__file__).resolve().parents[1] / "VOFA+CSV"
        paths = sorted(root.rglob("vofa+*.csv"), key=lambda p: p.stat().st_mtime, reverse=True)[:1]
    for p in paths:
        analyze(p, axis)


if __name__ == "__main__":
    main()
