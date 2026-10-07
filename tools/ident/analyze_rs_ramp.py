#!/usr/bin/env python3
"""Offline Rs LS for Pass0+Rs ramp CSV — same gate/ formula as motor/ident/rs_ident.c."""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import numpy as np

from vofa_io import load_vofa

RS_NOMINAL = 0.115
I_MIN_FIT = 2.0
EPS_TRACK = 0.03
RAMP_A_PER_S = 0.5
I_MAX = 3.0
REPEAT_N = 2
INTER_ROUND_S = 1.0
MIN_SAMPLES = 200
FS = 10000.0


def ls_rs(id_a: np.ndarray, u_a: np.ndarray) -> tuple[float, float, int]:
    m = len(id_a)
    if m < MIN_SAMPLES:
        return float("nan"), float("nan"), m
    sid = float(np.sum(id_a))
    sud = float(np.sum(u_a))
    sid2 = float(np.sum(id_a * id_a))
    sidud = float(np.sum(id_a * u_a))
    den = m * sid2 - sid * sid
    if abs(den) < 1e-6:
        return float("nan"), float("nan"), m
    rs = (m * sidud - sid * sud) / den
    b = (sud - rs * sid) / m
    return rs, b, m


def reconstruct_id_ref(t: np.ndarray, t0: float) -> np.ndarray:
    """Match rs_ident.c: ramp up, down, hold, repeat."""
    ref = np.zeros_like(t)
    rate = RAMP_A_PER_S
    cycle = 2.0 * I_MAX / rate + INTER_ROUND_S
    for i, ti in enumerate(t):
        if ti < t0:
            ref[i] = 0.0
            continue
        local = (ti - t0) % cycle
        up_s = I_MAX / rate
        down_s = up_s
        if local < up_s:
            ref[i] = min(I_MAX, local * rate)
        elif local < up_s + down_s:
            ref[i] = max(0.0, I_MAX - (local - up_s) * rate)
        else:
            ref[i] = 0.0
    return ref


def analyze_csv(path: Path) -> dict:
    vf = load_vofa(path, fs=FS)
    t = vf.t
    id_fb = vf.ch("id")
    if vf.has("ud_out"):
        ud_pi = vf.ch("ud_out")
    else:
        ud_pi = vf.ch("ud_pi")
    id_ref = vf.ch("id_ref")
    open_seq = vf.ch("open_seq").astype(np.int32)

    rs_mask = open_seq >= 54
    if np.any(rs_mask):
        t0 = float(t[np.argmax(rs_mask)])
    else:
        decay = np.where(open_seq == 39)[0]
        t0 = float(t[decay[-1]]) if len(decay) else 0.0

    seg = (t >= t0) & (open_seq >= 54)
    use = (
        seg
        & (np.abs(id_fb) >= I_MIN_FIT)
        & (np.abs(id_fb - id_ref) <= EPS_TRACK)
    )
    rs_off, b_off, n = ls_rs(id_fb[use], ud_pi[use])

    mcu_rs = float("nan")
    if vf.has("rs_ident_ohm"):
        mcu_rs = float(np.nanmax(vf.ch("rs_ident_ohm")))
    elif open_seq[-1] == 55:
        pass

    return {
        "path": path.name,
        "t0_rs": t0,
        "n_fit": n,
        "rs_offline": rs_off,
        "intercept_v": b_off,
        "rs_nominal": RS_NOMINAL,
        "rs_mcu_dbg": mcu_rs,
        "open_seq_end": int(open_seq[-1]),
    }


def format_md(r: dict) -> str:
    rs = r["rs_offline"]
    delta_pct = (
        100.0 * (rs - RS_NOMINAL) / RS_NOMINAL if np.isfinite(rs) else float("nan")
    )
    lines = [
        f"# Rs ramp 分析 — `{r['path']}`",
        "",
        "| 项 | 值 |",
        "|----|-----|",
        f"| Rs 段起点 | t≈{r['t0_rs']:.2f} s |",
        f"| 拟合样本 n | {r['n_fit']} |",
        f"| **Rs_offline** | **{rs:.6f} Ω** |" if np.isfinite(rs) else "| Rs_offline | FAIL |",
        f"| 截距 b | {r['intercept_v']:.4f} V |" if np.isfinite(r["intercept_v"]) else "| 截距 b | — |",
        f"| M1_RS_OHM | {RS_NOMINAL} Ω |",
        f"| vs 宏偏差 | {delta_pct:+.2f}% |" if np.isfinite(delta_pct) else "",
        f"| open_seq 末 | {r['open_seq_end']} (55=OK 56=FAIL) |",
        "",
        "MCU 结果见 Watch：`dbg.rs_ident_ohm` / `rs_ident_ok`（VOFA 12ch 未直出时需仿真器读）。",
        "",
    ]
    return "\n".join(lines)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("csv", type=Path)
    ap.add_argument("-o", "--out", type=Path, default=None)
    args = ap.parse_args()
    r = analyze_csv(args.csv)
    md = format_md(r)
    print(md)
    if args.out:
        args.out.write_text(md, encoding="utf-8")
    return 0 if np.isfinite(r["rs_offline"]) and r["n_fit"] >= MIN_SAMPLES else 1


if __name__ == "__main__":
    raise SystemExit(main())
