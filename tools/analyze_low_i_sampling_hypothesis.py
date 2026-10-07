#!/usr/bin/env python3
"""Test hypothesis: problem is low-I sampling/zero-cross, not LUT merge."""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from analyze_id_cal_dual_53 import (  # noqa: E402
    AMP_TABLE,
    CAPTURE_EPS,
    FS,
    RS,
    TH0,
    TH30,
    analyze_steps,
    count_sign_flips,
    find_segment_starts,
    load,
    parse_lut,
    steady_slice,
)

LOW_CUTOFF = 0.40
MID_CUTOFF = 1.50


def band(id_ref: float) -> str:
    if id_ref <= LOW_CUTOFF + 1e-6:
        return "low"
    if id_ref <= MID_CUTOFF:
        return "mid"
    return "high"


def agg(rows: list[dict], key: str) -> float:
    xs = [r[key] for r in rows if not (isinstance(r[key], float) and math.isnan(r[key]))]
    return float(np.mean(xs)) if xs else float("nan")


def summarize_pass(rows: list[dict], label: str) -> None:
    print(f"\n=== {label} ({len(rows)} steps) ===")
    for bname in ("low", "mid", "high"):
        sub = [r for r in rows if band(r["id_ref"]) == bname]
        if not sub:
            continue
        id_err = [abs(r["id_err"]) for r in sub]
        iq_abs = [abs(r["iq"]) for r in sub]
        flips = [r["ib_flips"] + r["ic_flips"] for r in sub]
        cap_ok = sum(1 for r in sub if r["capture_ok"])
        uoi = [r["u_over_i"] for r in sub if not math.isnan(r["u_over_i"])]
        print(f"  [{bname:4s}] Id_ref {sub[0]['id_ref']:.3f}..{sub[-1]['id_ref']:.3f} A  n={len(sub)}")
        print(
            f"         |Id_err| mean={np.mean(id_err)*1000:.1f}mA max={max(id_err)*1000:.1f}mA  "
            f"capture_ok={cap_ok}/{len(sub)}"
        )
        print(
            f"         |Iq| mean={np.mean(iq_abs)*1000:.1f}mA max={max(iq_abs)*1000:.1f}mA  "
            f"Ib+Ic flips/dwell mean={np.mean(flips):.1f} max={max(flips)}"
        )
        if uoi:
            print(
                f"         Pass1 U/I mean={np.mean(uoi):.3f}Ω  "
                f"near Rs(0.115)? fraction in [0.08,0.20]={sum(0.08<=x<=0.20 for x in uoi)/len(uoi)*100:.0f}%"
            )


def pass1_vs_pass0(p0: list[dict], p1: list[dict]) -> None:
    print("\n=== Pass1 vs Pass0 (compensation residual) ===")
    by_ref = {r["id_ref"]: r for r in p0}
    pairs = []
    for r1 in p1:
        r0 = by_ref.get(r1["id_ref"])
        if r0 is None:
            continue
        pairs.append(
            {
                "id_ref": r1["id_ref"],
                "ud_res_p0": r0["ud_res"],
                "ud_res_p1": r1["ud_res"],
                "ud_drop": r0["ud_res"] - r1["ud_res"],
                "ud_drop_pct": (r0["ud_res"] - r1["ud_res"]) / r0["ud_res"] * 100
                if abs(r0["ud_res"]) > 0.05
                else float("nan"),
                "uoi_p1": r1["u_over_i"],
                "flips_p1": r1["ib_flips"] + r1["ic_flips"],
                "iq_p1": abs(r1["iq"]),
                "id_err_p1": abs(r1["id_err"]),
            }
        )
    for bname in ("low", "mid", "high"):
        sub = [p for p in pairs if band(p["id_ref"]) == bname]
        if not sub:
            continue
        drop = [p["ud_drop_pct"] for p in sub if not math.isnan(p["ud_drop_pct"])]
        uoi = [p["uoi_p1"] for p in sub if not math.isnan(p["uoi_p1"])]
        print(f"  [{bname}] n={len(sub)}")
        if drop:
            print(f"       Ud_res drop Pass0→Pass1: mean {np.mean(drop):.0f}%  (compensation reducing Ud)")
        if uoi:
            good = sum(0.08 <= x <= 0.25 for x in uoi)
            print(
                f"       Pass1 U/I: mean={np.mean(uoi):.3f}Ω  "
                f"good≈Rs {good}/{len(uoi)} ({100*good/len(uoi):.0f}%)"
            )
        print(
            f"       Pass1 |Iq| mean={np.mean([p['iq_p1'] for p in sub])*1000:.1f}mA  "
            f"flips mean={np.mean([p['flips_p1'] for p in sub]):.1f}"
        )


def near_zero_frac(c: dict, start: int, dwell_s: float, k: int) -> dict:
    t0 = start + int(k * dwell_s * FS)
    sl = steady_slice(len(c["id"]), t0, dwell_s)
    ib, ic, iq = c["ib"][sl], c["ic"][sl], c["iq"][sl]
    thr = 0.02
    return {
        "ib_near0": float(np.mean(np.abs(ib) < thr)),
        "ic_near0": float(np.mean(np.abs(ic) < thr)),
        "iq_near0": float(np.mean(np.abs(iq) < thr)),
    }


def analyze_file(path: Path) -> None:
    print(f"\n{'='*70}\nFILE: {path.name}\n{'='*70}")
    c = load(path)
    lut = parse_lut(c)
    lut_t = lut["t_idx"] if lut else int(0.5 * len(c["id"]))
    seg = find_segment_starts(c, lut_t)
    p0a = analyze_steps(c, seg["pass0a"], 0.5, "pass0a")
    p0b = analyze_steps(c, seg["pass0b"], 0.5, "pass0b")
    p1 = analyze_steps(c, seg["pass1"], 0.5, "pass1")

    summarize_pass(p0a, "Pass0-A @30°")
    summarize_pass(p0b, "Pass0-B @0°")
    summarize_pass(p1, "Pass1 LUT ON")
    pass1_vs_pass0(p0a, p1)

    print("\n=== Near-zero fraction in steady window (Pass1, |i|<20mA) ===")
    for bname, k_range in [
        ("low", range(0, 113)),
        ("mid", range(113, 127)),
        ("high", range(127, len(AMP_TABLE))),
    ]:
        ks = [k for k in k_range if k < len(AMP_TABLE)]
        if not ks:
            continue
        nz = [near_zero_frac(c, seg["pass1"], 0.5, k) for k in ks[: min(len(ks), 20)]]
        print(
            f"  [{bname}] Ib near0={np.mean([x['ib_near0'] for x in nz])*100:.0f}%  "
            f"Ic near0={np.mean([x['ic_near0'] for x in nz])*100:.0f}%  "
            f"Iq near0={np.mean([x['iq_near0'] for x in nz])*100:.0f}%"
        )


def main() -> None:
    files = [
        Path(r"d:/stm32/STM32G474RET6_MOTOR/VOFA+CSV/20260701/vofa+202607010048.csv"),
        Path(r"d:/stm32/STM32G474RET6_MOTOR/VOFA+CSV/20260701/vofa+202607010025.csv"),
    ]
    for p in files:
        if p.exists():
            analyze_file(p)


if __name__ == "__main__":
    main()
