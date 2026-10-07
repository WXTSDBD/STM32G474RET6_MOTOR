#!/usr/bin/env python3
"""Batch VASI Ld/Lq + Rs report for multiple VOFA CSVs."""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))

import numpy as np

from ident.analyze_ld_lq_vasi import RS_OHM, analyze_csv
from ident.analyze_rs_ramp import EPS_TRACK, FS, I_MIN_FIT, ls_rs
from vofa_io import load_vofa

LD_REF = 59.0
LQ_REF = 87.0
RS_NOM = 0.115


def rs_round(vf, lut_round: int) -> tuple[float, float, int]:
    seq = vf.ch("open_seq").astype(np.int32)
    ch10 = vf.ch("duty_dev")
    idf = vf.ch("id")
    ud = vf.ch("ud_out")
    idr = vf.ch("id_ref")
    if lut_round == 0:
        base = (seq >= 54) & (seq < 57)
    else:
        base = (seq >= 154) & (seq < 157)
    m = base & (np.abs(ch10 - float(lut_round)) < 0.5)
    m &= (np.abs(idf) >= I_MIN_FIT) & (np.abs(idf - idr) <= EPS_TRACK)
    return ls_rs(idf[m], ud[m])


def flow_lines(vf) -> list[str]:
    t = vf.t
    seq = vf.ch("open_seq").astype(int)
    ch10 = vf.ch("duty_dev")
    out: list[str] = []
    for ms in (54, 55, 57, 58, 154, 157, 158, 163, 263):
        hit = np.where(seq >= ms)[0]
        if len(hit):
            out.append(f"open_seq>={ms}: t={t[hit[0]]:.2f}s")
    flip = np.where((ch10[1:] > 0.9) & (ch10[:-1] < 0.1))[0]
    if len(flip):
        out.append(f"ch10 OFF→LUT: t={t[flip[0] + 1]:.2f}s")
    return out


def print_grid(title: str, merged: list[tuple[float, float, float]]) -> None:
    mp = {(a, b): v for a, b, v in merged}
    print(f"\n{title}")
    print("| Id \\ Iq | 0.0 | 0.5 | 1.0 |")
    print("|--------|-----|-----|-----|")
    for id0 in (0.5, 1.0, 1.5):
        cells = [f"{mp.get((id0, iq0), float('nan')):.1f}" for iq0 in (0.0, 0.5, 1.0)]
        print(f"| {id0} | {' | '.join(cells)} |")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("csv", type=Path, nargs="+")
    args = ap.parse_args()

    print("# VASI 电感辨识 + Rs 批量报告")
    print(f"\n方法: `analyze_ld_lq_vasi.py --method psi`（论文 Δψ/ΔI，2 kHz coarse-d/q 子窗 @10 kHz）")
    print(f"参考: LCR Ld={LD_REF} uH  Lq={LQ_REF} uH  Rs={RS_NOM} ohm  脚本 Rs={RS_OHM} ohm\n")

    off_stats: dict[str, list[float]] = {
        "G0_Ld": [],
        "G3_Ld": [],
        "G6_Ld": [],
        "Ld_rms": [],
        "Ld@1A0": [],
        "Lq@1A0": [],
        "Lq@1A05": [],
        "Lq@1A1": [],
        "Rs_OFF": [],
    }

    for fp in args.csv:
        tag = fp.stem.replace("vofa+", "")
        vf = load_vofa(fp, fs=FS)
        r = analyze_csv(fp, method="psi", rs_ohm=RS_OHM)

        print(f"---\n## {tag}\n")
        print(f"| 项 | 值 |")
        print(f"|----|-----|")
        print(f"| 时长 | {vf.t[-1]:.1f} s ({vf.n} samples @10 kHz) |")
        print(f"| open_seq 结束 | {r['open_seq_end']} (58=单轮 OFF VASI 完, 158=旧双轮) |")
        print(f"| flow_done | {r['flow_done']} |")
        print(f"| VASI t0 | {r['t0_l_ident']:.2f} s |")

        print("\n**流程节点:**")
        for ln in flow_lines(vf):
            print(f"- {ln}")

        for rnd, lbl in ((0, "OFF"), (1, "LUT")):
            rs, b, n = rs_round(vf, rnd)
            if np.isfinite(rs):
                pct = 100.0 * (rs - RS_NOM) / RS_NOM
                print(f"\n**Rs {lbl}:** {rs:.6f} ohm  (n={n}, intercept={b:.4f} V, vs nom {pct:+.2f}%)")
                if rnd == 0:
                    off_stats["Rs_OFF"].append(rs)
            else:
                print(f"\n**Rs {lbl}:** 拟合失败 (n={n})")

        rd = r.get("by_round", {}).get(0, {})
        rl = r.get("by_round", {}).get(1, {})

        if rd:
            print("\n### OFF 轮 VASI（可信）")
            print_grid("**Ld (uH)**", rd.get("ld_merged", []))
            print_grid("**Lq (uH)**", rd.get("lq_merged", []))
            ld_m = {(a, b): v for a, b, v in rd.get("ld_merged", [])}
            lq_m = {(a, b): v for a, b, v in rd.get("lq_merged", [])}
            print(
                f"\n- Ld@(Id=1,Iq=0) = **{rd.get('ld_at_1A_uH', float('nan')):.1f} uH**  "
                f"rms = {rd.get('ld_rms_uH', float('nan')):.2f} uH"
            )
            print(
                f"- Lq@(Id=1,Iq=0) = **{lq_m.get((1.0, 0.0), float('nan')):.1f} uH**  "
                f"(PI 零 Iq 偏置)"
            )
            print(
                f"- Lq@(Id=1,Iq=0.5) = **{lq_m.get((1.0, 0.5), float('nan')):.1f} uH**  "
                f"(PI Iq≈0.5 A)"
            )
            print(
                f"- Lq@(Id=1,Iq=1) 曲面 = **{rd.get('lq_at_1A_uH', float('nan')):.1f} uH**  "
                f"rms = {rd.get('lq_rms_uH', float('nan')):.2f} uH  (勿单独签收)"
            )
            off_stats["G0_Ld"].append(ld_m.get((0.5, 0.0), float("nan")))
            off_stats["G3_Ld"].append(ld_m.get((1.0, 0.0), float("nan")))
            off_stats["G6_Ld"].append(ld_m.get((1.5, 0.0), float("nan")))
            off_stats["Ld_rms"].append(rd.get("ld_rms_uH", float("nan")))
            off_stats["Ld@1A0"].append(rd.get("ld_at_1A_uH", float("nan")))
            off_stats["Lq@1A0"].append(lq_m.get((1.0, 0.0), float("nan")))
            off_stats["Lq@1A05"].append(lq_m.get((1.0, 0.5), float("nan")))
            off_stats["Lq@1A1"].append(rd.get("lq_at_1A_uH", float("nan")))

        if rl:
            ld_m = {(a, b): v for a, b, v in rl.get("ld_merged", [])}
            print("\n### LUT 轮 VASI（duty 污染，对照用）")
            print(
                f"- G0 Ld={ld_m.get((0.5, 0.0), float('nan')):.1f}  "
                f"G3 Ld={ld_m.get((1.0, 0.0), float('nan')):.1f}  "
                f"Ld_rms={rl.get('ld_rms_uH', float('nan')):.2f} uH"
            )
            print(
                f"- Lq@(1,1)={rl.get('lq_at_1A_uH', float('nan')):.1f} uH  "
                f"rms={rl.get('lq_rms_uH', float('nan')):.2f} uH"
            )

    if len(args.csv) > 1:
        print("\n---\n## 三次 OFF 轮重复性汇总\n")
        print("| 指标 | " + " | ".join(fp.stem.replace("vofa+", "") for fp in args.csv) + " | spread |")
        print("|------|" + "|".join(["------"] * len(args.csv)) + "|--------|")
        for key, vals in off_stats.items():
            if not vals:
                continue
            spread = max(vals) - min(vals)
            row = " | ".join(f"{v:.2f}" for v in vals)
            print(f"| {key} | {row} | {spread:.2f} |")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
