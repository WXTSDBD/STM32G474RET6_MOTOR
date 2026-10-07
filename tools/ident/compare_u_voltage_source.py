#!/usr/bin/env python3
"""Offline compare Ld/Lq: duty Vd_est vs command u_inj square (MCU psi path)."""
from __future__ import annotations

import argparse
import sys
from collections import Counter
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "tools" / "ident"))

from parse_ident_vofa import find_header_rows, load_csv, parse_burst  # noqa: E402
from vofa_io import load_vofa  # noqa: E402
from analyze_ld_lq_vasi import (  # noqa: E402
    F_COARSE_HZ,
    F_FINE_HZ,
    FS,
    RS_OHM,
    extract_l_vasi_psi,
    firmware_half_ticks,
    upsample_ctrl_rate,
)

ID_BIAS = [0.5, 0.75, 1.0]
IQ_BIAS = [0.0, 0.25, 0.5, 0.75, 1.0]
AMP_STEPS = 7
CYCLES_PER_AMP = 15
N_INJ_HALVES = AMP_STEPS * CYCLES_PER_AMP * 2
TELEM_STAGE_COARSE = int(round(N_INJ_HALVES * 0.5 / F_COARSE_HZ * FS))
TELEM_STAGE_FINE = int(round(N_INJ_HALVES * 0.5 / F_FINE_HZ * FS))
INJ_TOTAL = 2 * (TELEM_STAGE_COARSE + TELEM_STAGE_FINE)
LD_LCR = 59.0
LQ_LCR = 87.0


def longest_vasi_run(open_seq: np.ndarray) -> tuple[int, int]:
    m = np.isclose(open_seq, 57.0, atol=0.5)
    runs: list[tuple[int, int, int]] = []
    i, n = 0, len(m)
    while i < n:
        if not m[i]:
            i += 1
            continue
        j = i + 1
        while j < n and m[j]:
            j += 1
        runs.append((i, j, j - i))
        i = j
    if not runs:
        return 0, len(open_seq)
    s, e, _ = max(runs, key=lambda x: x[2])
    return s, e


def cluster_grid_cells(
    id_fb: np.ndarray, iq_fb: np.ndarray, min_run: int = 8000
) -> list[tuple[int, int, float, float]]:
    id_r = np.round(id_fb * 4.0) / 4.0
    iq_r = np.round(iq_fb * 4.0) / 4.0
    key = id_r * 10.0 + iq_r
    segs: list[tuple[int, int, float, float]] = []
    i, n = 0, len(key)
    while i < n:
        k0 = key[i]
        j = i + 1
        while j < n and key[j] == k0:
            j += 1
        if j - i >= min_run:
            id_b = float(id_r[i])
            iq_b = float(iq_r[i])
            if id_b in ID_BIAS and iq_b in IQ_BIAS:
                segs.append((i, j, id_b, iq_b))
        i = j
    # keep longest run per (Id,Iq)
    best: dict[tuple[float, float], tuple[int, int, float, float]] = {}
    for s, e, id_b, iq_b in segs:
        k = (id_b, iq_b)
        if k not in best or (e - s) > (best[k][1] - best[k][0]):
            best[k] = (s, e, id_b, iq_b)
    return list(best.values())


def find_inject_start(vd: np.ndarray, *, win: int = 300) -> int:
    """First sample where AC terminal voltage shows inject ripple."""
    n = len(vd)
    if n < INJ_TOTAL + 500:
        return max(0, n - INJ_TOTAL)
    med = float(np.median(vd[: min(2000, n // 4)]))
    ac = vd - med
    for k in range(200, n - INJ_TOTAL):
        if float(np.std(ac[k : k + win])) > 0.04:
            return k
    return max(0, n - INJ_TOTAL)


def inj_slices(n_inj: int, head: int = 0) -> dict[str, slice] | None:
    total = INJ_TOTAL
    if n_inj < total:
        return None
    d0 = head
    d1 = d0 + TELEM_STAGE_COARSE
    df0 = d1
    df1 = df0 + TELEM_STAGE_FINE
    q0 = df1
    q1 = q0 + TELEM_STAGE_COARSE
    qf0 = q1
    qf1 = qf0 + TELEM_STAGE_FINE
    if qf1 > n_inj:
        return None
    return {"c_d": slice(d0, d1), "f_d": slice(df0, df1), "c_q": slice(q0, q1), "f_q": slice(qf0, qf1)}


def u_ac_terminal(u: np.ndarray) -> np.ndarray:
    return u - float(np.median(u))


def u_ac_cmd_square(u_term: np.ndarray, f_hz: float) -> np.ndarray:
    u = np.asarray(u_term, dtype=np.float64)
    med = float(np.median(u))
    ac = u - med
    v_pk = float(np.percentile(np.abs(ac), 92))
    if v_pk < 0.02:
        return ac
    ht = firmware_half_ticks(f_hz)
    u_up = upsample_ctrl_rate(u)
    ac_up = u_up - float(np.median(u_up))
    n = len(ac_up)
    out = np.zeros(n, dtype=np.float64)
    pos = 0
    sign = 1.0 if ac_up[0] >= 0.0 else -1.0
    while pos < n:
        end = min(pos + ht, n)
        out[pos:end] = sign * v_pk
        sign = -sign
        pos += ht
    return out[::2][: len(u)]


def l_from_stage(u: np.ndarray, i: np.ndarray, rs: float, f_hz: float, source: str) -> float | None:
    if source == "cmd":
        u_use = u_ac_cmd_square(u, f_hz)
    else:
        u_use = u_ac_terminal(u)
    return extract_l_vasi_psi(u_use, i, rs, f_hz)


def extract_cell(vd, vq, id_, iq, rs, source) -> dict[str, float | None]:
    sl = inj_slices(len(vd))
    if sl is None:
        return {}
    out: dict[str, float | None] = {}
    for key, slc, u, ii, f in [
        ("ld_c", sl["c_d"], vd, id_, F_COARSE_HZ),
        ("ld_f", sl["f_d"], vd, id_, F_FINE_HZ),
        ("lq_c", sl["c_q"], vq, iq, F_COARSE_HZ),
        ("lq_f", sl["f_q"], vq, iq, F_FINE_HZ),
    ]:
        l_h = l_from_stage(u[slc], ii[slc], rs, f, source)
        out[key] = l_h * 1e6 if l_h is not None else None
    return out


def analyze(path: Path, rs: float = RS_OHM) -> None:
    vf = load_vofa(path, fs=FS)
    raw = np.genfromtxt(path, delimiter=",", names=True, dtype=float)
    open_seq = raw["I11"]
    vs, ve = longest_vasi_run(open_seq)
    id_fb = raw["I3"][vs:ve]
    iq_fb = raw["I4"][vs:ve]
    vd = raw["I6"][vs:ve]
    vq = raw["I7"][vs:ve]

    segs = cluster_grid_cells(id_fb, iq_fb)
    print(f"\n=== {path.name}  VASI [{vs}:{ve}] ({(ve-vs)/FS:.1f}s)  cells={len(segs)} ===")
    print(f"  stage telem: coarse={TELEM_STAGE_COARSE} fine={TELEM_STAGE_FINE} inj_total={INJ_TOTAL}")

    by_src: dict[str, dict[tuple[float, float], dict]] = {"vd_est": {}, "cmd": {}}
    for s, e, id_b, iq_b in segs:
        inj0 = find_inject_start(vd[s:e])
        seg_vd = vd[s + inj0 : e]
        seg_vq = vq[s + inj0 : e]
        seg_id = id_fb[s + inj0 : e]
        seg_iq = iq_fb[s + inj0 : e]
        for src in ("vd_est", "cmd"):
            cell = extract_cell(seg_vd, seg_vq, seg_id, seg_iq, rs, src)
            if cell:
                by_src[src][(id_b, iq_b)] = cell

    rows = load_csv(path)
    bursts = [parse_burst(rows, hi) for hi in find_header_rows(rows)]
    mcu = bursts[-1]
    mcu_map = {(g["id_bias"], g["iq_bias"]): g for g in mcu["grid"]}

    def pick(d: dict | None, k: str) -> float:
        if not d:
            return float("nan")
        v = d.get(k)
        return float("nan") if v is None else v

    print(
        f"\n{'grid':>12}  {'MCU Ld c/f':>12}  {'Vd c/f':>12}  {'Cmd c/f':>12}  "
        f"{'MCU Lq c/f':>12}  {'Vq c/f':>12}  {'Cmd c/f':>12}"
    )
    print("-" * 96)
    vd_ld_f, cmd_ld_f, mcu_ld_f = [], [], []
    vd_lq_f, cmd_lq_f, mcu_lq_f = [], [], []
    for key in sorted(mcu_map.keys()):
        g = mcu_map[key]
        vd_c = by_src["vd_est"].get(key, {})
        cm_c = by_src["cmd"].get(key, {})
        mcu_ldc = g["ld_coarse_uH"] if g.get("ld_coarse_valid") else float("nan")
        mcu_ldf = g["ld_fine_uH"] if g.get("ld_fine_valid") else float("nan")
        mcu_lqc = g["lq_coarse_uH"] if g.get("lq_coarse_valid") else float("nan")
        mcu_lqf = g["lq_fine_uH"] if g.get("lq_fine_valid") else float("nan")
        print(
            f"({key[0]:+.2f},{key[1]:+.2f})"
            f"  {mcu_ldc:5.0f}/{mcu_ldf:5.0f}"
            f"  {pick(vd_c,'ld_c'):5.0f}/{pick(vd_c,'ld_f'):5.0f}"
            f"  {pick(cm_c,'ld_c'):5.0f}/{pick(cm_c,'ld_f'):5.0f}"
            f"  {mcu_lqc:5.0f}/{mcu_lqf:5.0f}"
            f"  {pick(vd_c,'lq_c'):5.0f}/{pick(vd_c,'lq_f'):5.0f}"
            f"  {pick(cm_c,'lq_c'):5.0f}/{pick(cm_c,'lq_f'):5.0f}"
        )
        if key[1] == 0.0 and mcu_ldf == mcu_ldf:
            mcu_ld_f.append(mcu_ldf)
            vf_ = pick(vd_c, "ld_f")
            cf = pick(cm_c, "ld_f")
            if vf_ == vf_:
                vd_ld_f.append(vf_)
            if cf == cf:
                cmd_ld_f.append(cf)
        if abs(key[0] - 1.0) < 0.01 and abs(key[1] - 0.5) < 0.01 and mcu_lqf == mcu_lqf:
            mcu_lq_f.append(mcu_lqf)
            vf_ = pick(vd_c, "lq_f")
            cf = pick(cm_c, "lq_f")
            if vf_ == vf_:
                vd_lq_f.append(vf_)
            if cf == cf:
                cmd_lq_f.append(cf)

    print("\n--- Summary vs LCR (fine @1kHz) ---")
    for label, mcu_a, vd_a, cmd_a, ref in [
        ("Ld @ Iq=0", mcu_ld_f, vd_ld_f, cmd_ld_f, LD_LCR),
        ("Lq @ (1,0.5)", mcu_lq_f, vd_lq_f, cmd_lq_f, LQ_LCR),
    ]:
        if not mcu_a:
            continue
        m = float(np.mean(mcu_a))
        print(f"  {label}:")
        print(f"    MCU burst   {m:.1f} uH  ({100*(m/ref-1):+.0f}% vs LCR {ref:.0f})")
        if vd_a:
            v = float(np.mean(vd_a))
            print(f"    Vd_est off  {v:.1f} uH  ({100*(v/ref-1):+.0f}%)  d vs MCU {100*(v/m-1):+.1f}%")
        if cmd_a:
            c = float(np.mean(cmd_a))
            print(f"    Cmd sq off  {c:.1f} uH  ({100*(c/ref-1):+.0f}%)  d vs MCU {100*(c/m-1):+.1f}%")

    key10 = (1.0, 0.0)
    if key10 in by_src["vd_est"]:
        for s, e, id_b, iq_b in segs:
            if (id_b, iq_b) != key10:
                continue
            inj0 = find_inject_start(vd[s:e])
            u_f = vd[s + inj0 : e]
            sl = inj_slices(len(u_f))
            if sl is None:
                break
            u_f = u_f[sl["f_d"]]
            ac = u_f - float(np.median(u_f))
            v_term = float(np.percentile(np.abs(ac), 92))
            u_cmd = u_ac_cmd_square(u_f, F_FINE_HZ)
            v_cmd = float(np.percentile(np.abs(u_cmd), 92))
            print(
                f"\n  @(1,0) fine d-inject |u_ac|: terminal={v_term:.3f} V  "
                f"cmd_square={v_cmd:.3f} V  ratio={v_term/max(v_cmd,1e-6):.3f}"
            )
            break


def main() -> None:
    ap = argparse.ArgumentParser(description="Compare Vd_est vs command u for offline Ld/Lq")
    ap.add_argument("csv", type=Path, nargs="+")
    ap.add_argument("--rs", type=float, default=RS_OHM)
    args = ap.parse_args()
    for p in args.csv:
        analyze(p, rs=args.rs)


if __name__ == "__main__":
    main()
