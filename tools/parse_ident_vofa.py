#!/usr/bin/env python3
"""Parse VOFA CSV for MCU Rs/Ld-Lq ident telem burst (JustFloat 6/12 ch)."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

HDR_MAGIC = -777777.0
TAIL_MAGIC = -666666.0
HDR_TOL = 1.0

# VASI 过程 telem（open_seq=57）ch9=proc_code：
#   sub*100 + axis*10 + tier_band*5 + amp_idx
#   sub: 0=SETTLE 1=INJ_LD 2=INJ_LQ
#   axis: 0=d 1=q
#   tier_band: 1=500Hz 0=1kHz 2=2kHz(f2)
PROC_SUB = {0: "SETTLE", 1: "INJ_LD", 2: "INJ_LQ"}


def decode_proc_code(code: float) -> dict:
    v = int(round(code))
    tier_band = (v // 5) % 10
    if tier_band == 1:
        freq = "500Hz"
    elif tier_band == 2:
        freq = "2kHz"
    else:
        freq = "1kHz"
    return {
        "sub": PROC_SUB.get(v // 100, str(v // 100)),
        "axis": "d" if (v // 10) % 10 == 0 else "q",
        "coarse": tier_band == 1,
        "freq": freq,
        "tier_band": tier_band,
        "amp_idx": v % 5,
        "raw": v,
    }


def load_csv(path: Path) -> np.ndarray:
    data = np.genfromtxt(path, delimiter=",", names=True, dtype=float)
    names = data.dtype.names
    if names is None or len(names) < 6:
        raise ValueError(f"need >=6 columns, got {names}")
    ncol = min(len(names), 12)
    return np.column_stack([data[names[i]] for i in range(ncol)])


def find_header_rows(rows: np.ndarray) -> list[int]:
    out = []
    for i, row in enumerate(rows):
        if abs(row[0] - HDR_MAGIC) < HDR_TOL:
            out.append(i)
    return out


def find_header(rows: np.ndarray, use_last: bool = True) -> int:
    hits = find_header_rows(rows)
    if not hits:
        return -1
    return hits[-1] if use_last else hits[0]


def _parse_grid_row(row: np.ndarray, proto: float) -> dict:
    g = {
        "id_bias": float(row[0]),
        "iq_bias": float(row[1]),
        "ld_coarse_uH": float(row[2]),
        "lq_coarse_uH": float(row[3]),
        "ld_fine_uH": float(row[4]),
        "lq_fine_uH": float(row[5]),
        "ld_coarse_valid": int(row[6]) != 0,
        "lq_coarse_valid": int(row[7]) != 0,
        "ld_fine_valid": int(row[8]) != 0,
        "lq_fine_valid": int(row[9]) != 0,
        "ld_uH": float(row[2]),
        "lq_uH": float(row[3]),
        "ld_valid": int(row[6]) != 0,
        "lq_valid": int(row[7]) != 0,
    }
    if proto >= 2.95:
        g["ld_f2_uH"] = float(row[10]) if row.shape[0] > 10 else float("nan")
        g["lq_f2_uH"] = float(row[11]) if row.shape[0] > 11 else float("nan")
        g["ld_f2_valid"] = g["ld_f2_uH"] == g["ld_f2_uH"] and g["ld_f2_uH"] > 0.0
        g["lq_f2_valid"] = g["lq_f2_uH"] == g["lq_f2_uH"] and g["lq_f2_uH"] > 0.0
    else:
        g["ld_f2_uH"] = float("nan")
        g["lq_f2_uH"] = float("nan")
        g["ld_f2_valid"] = False
        g["lq_f2_valid"] = False
    if proto < 1.95:
        g["ld_fine_uH"] = float("nan")
        g["lq_fine_uH"] = float("nan")
        g["ld_fine_valid"] = False
        g["lq_fine_valid"] = False
        g["ld_coarse_valid"] = int(row[4]) != 0
        g["lq_coarse_valid"] = int(row[5]) != 0
        g["ld_uH"] = float(row[2])
        g["lq_uH"] = float(row[3])
        g["ld_valid"] = g["ld_coarse_valid"]
        g["lq_valid"] = g["lq_coarse_valid"]
    return g


def parse_burst(rows: np.ndarray, hdr_i: int) -> dict:
    hdr = rows[hdr_i]
    grid_n = int(hdr[1])
    rs_ohm = float(hdr[2])
    rs_ok = int(hdr[3]) != 0
    proto = float(hdr[4])
    n_ld_ok = int(hdr[5])
    n_lq_ok = int(hdr[6]) if rows.shape[1] > 6 else 0

    if proto >= 1.95:
        n_ld_ok_fine = int(hdr[7]) if rows.shape[1] > 7 else 0
        n_lq_ok_fine = int(hdr[8]) if rows.shape[1] > 8 else 0
        ld_lq_ok = int(hdr[9]) != 0 if rows.shape[1] > 9 else False
        ld_lq_ok_fine = int(hdr[10]) != 0 if rows.shape[1] > 10 else False
    else:
        n_ld_ok_fine = 0
        n_lq_ok_fine = 0
        ld_lq_ok = int(hdr[7]) != 0 if rows.shape[1] > 7 else False
        ld_lq_ok_fine = False

    n_ld_ok_f2 = 0
    n_lq_ok_f2 = 0
    ld_lq_ok_f2 = False
    if proto >= 2.95:
        n_ld_ok_f2 = int(hdr[11]) if rows.shape[1] > 11 else 0

    grid = []
    for j in range(grid_n):
        row = rows[hdr_i + 1 + j]
        grid.append(_parse_grid_row(row, proto))

    tail_i = hdr_i + 1 + grid_n
    tail = rows[tail_i]
    if abs(tail[0] - TAIL_MAGIC) >= HDR_TOL:
        raise ValueError(f"expected tail at row {tail_i}, got I0={tail[0]}")

    if proto >= 2.95 and rows.shape[1] > 4:
        n_lq_ok_f2 = int(tail[4])
        ld_lq_ok_f2 = int(tail[5]) != 0 if rows.shape[1] > 5 else False

    return {
        "hdr_row": hdr_i,
        "tail_row": tail_i,
        "proto": proto,
        "grid_n": grid_n,
        "rs_ohm": rs_ohm,
        "rs_ok": rs_ok,
        "n_ld_ok": n_ld_ok,
        "n_lq_ok": n_lq_ok,
        "n_ld_ok_fine": n_ld_ok_fine,
        "n_lq_ok_fine": n_lq_ok_fine,
        "n_ld_ok_f2": n_ld_ok_f2,
        "n_lq_ok_f2": n_lq_ok_f2,
        "ld_lq_ok": ld_lq_ok,
        "ld_lq_ok_fine": ld_lq_ok_fine,
        "ld_lq_ok_f2": ld_lq_ok_f2,
        "rs_used_ohm": float(tail[3]) if rows.shape[1] > 3 else rs_ohm,
        "checksum": float(tail[2]),
        "grid": grid,
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("csv", type=Path)
    ap.add_argument(
        "--first",
        action="store_true",
        help="use first ident burst (default: last, for Pass0+VASI chain)",
    )
    args = ap.parse_args()

    rows = load_csv(args.csv)
    hdr_i = find_header(rows, use_last=not args.first)
    if hdr_i < 0:
        print("No ident burst (I0≈-777777) found.", file=sys.stderr)
        return 1

    all_hdr = find_header_rows(rows)
    if len(all_hdr) > 1 and not args.first:
        print(f"Note: {len(all_hdr)} bursts found, using last @ row {hdr_i}", file=sys.stderr)

    r = parse_burst(rows, hdr_i)
    print(f"Ident burst @ row {r['hdr_row']} (proto {r['proto']:.1f})")
    print(f"  Rs = {r['rs_ohm']:.4f} Ω  ok={r['rs_ok']}")
    print(
        f"  coarse ok={r['ld_lq_ok']}  n_ld={r['n_ld_ok']}  n_lq={r['n_lq_ok']}"
    )
    if r["proto"] >= 1.95:
        print(
            f"  fine   ok={r['ld_lq_ok_fine']}  n_ld={r['n_ld_ok_fine']}  "
            f"n_lq={r['n_lq_ok_fine']}"
        )
    if r["proto"] >= 2.95:
        print(
            f"  f2@2k ok={r['ld_lq_ok_f2']}  n_ld={r['n_ld_ok_f2']}  "
            f"n_lq={r['n_lq_ok_f2']}"
        )
    print(f"  rs_used={r['rs_used_ohm']:.4f} Ω  checksum={r['checksum']:.2f} uH-sum")
    hdr = "500Hz / @1kHz / @2kHz" if r["proto"] >= 2.95 else "500Hz / @1kHz"
    print(f"  grid (Id, Iq) -> Ld/Lq [uH] @{hdr}:")
    for g in r["grid"]:
        ldc = f"{g['ld_coarse_uH']:.1f}" if g["ld_coarse_valid"] else "—"
        lqc = f"{g['lq_coarse_uH']:.1f}" if g["lq_coarse_valid"] else "—"
        if r["proto"] >= 1.95:
            ldf = f"{g['ld_fine_uH']:.1f}" if g["ld_fine_valid"] else "—"
            lqf = f"{g['lq_fine_uH']:.1f}" if g["lq_fine_valid"] else "—"
            if r["proto"] >= 2.95:
                ld2 = f"{g['ld_f2_uH']:.1f}" if g["ld_f2_valid"] else "—"
                lq2 = f"{g['lq_f2_uH']:.1f}" if g["lq_f2_valid"] else "—"
                print(
                    f"    ({g['id_bias']:+.2f}, {g['iq_bias']:+.2f})  "
                    f"Ld={ldc}/{ldf}/{ld2}  Lq={lqc}/{lqf}/{lq2}"
                )
            else:
                print(
                    f"    ({g['id_bias']:+.2f}, {g['iq_bias']:+.2f})  "
                    f"Ld={ldc}/{ldf}  Lq={lqc}/{lqf}"
                )
        else:
            print(f"    ({g['id_bias']:+.2f}, {g['iq_bias']:+.2f})  Ld={ldc}  Lq={lqc}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
