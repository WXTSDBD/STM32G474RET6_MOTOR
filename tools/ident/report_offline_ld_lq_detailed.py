#!/usr/bin/env python3
"""
Detailed offline Ld/Lq VASI report: all grid cells, coarse/fine, Ld/Lq vs MCU burst.

Uses ud_out (ch6) + id for d-axis; uq_pi channel is foc_uq_out (ch7) in unified 12ch.
"""
from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import numpy as np

from ident.analyze_ld_lq_vasi import (
    F_COARSE_HZ,
    F_FINE_HZ,
    FS,
    SETTLE_S,
    TELEM_STAGE_COARSE,
    TELEM_STAGE_FINE,
    extract_l_axis,
    extract_l_vasi_psi,
    firmware_half_ticks,
    inj_stage_slices,
    segment_by_grid_targets,
    upsample_ctrl_rate,
)
from parse_ident_vofa import find_header, load_csv, parse_burst
from vofa_io import load_vofa

RS_DEFAULT = 0.1225
L_MIN_H = 20e-6
L_MAX_H = 1e-3
MIN_DI = 0.05
TS_CTRL = 50e-6
AMP_STEPS = 7
CYCLES_PER_AMP = 10
STATIC_LD_UH = 59.0
STATIC_LQ_UH = 87.0


@dataclass
class StageResult:
    label: str
    f_hz: float
    l_seg_uh: float | None
    l_mcu_agg_uh: float | None
    n_cycles: int
    amp_buckets_uh: list[float]
    u_pos_pk: float
    u_neg_pk: float
    u_near_zero_frac: float
    f_dom_hz: float
    u_pk: float
    i_pk: float


def dom_freq(u: np.ndarray) -> float:
    u = np.asarray(u, dtype=float) - float(np.median(u))
    zc = int(np.sum(np.abs(np.diff(np.sign(u))) > 0))
    return zc / (2.0 * len(u) / FS) if len(u) else float("nan")


def waveform_stats(u: np.ndarray) -> tuple[float, float, float, float]:
    u = np.asarray(u, dtype=float)
    pos = u[u > 0.05]
    neg = u[u < -0.05]
    near0 = float(np.mean(np.abs(u) <= 0.05))
    u_pos = float(pos.max()) if len(pos) else 0.0
    u_neg = float(neg.min()) if len(neg) else 0.0
    return u_pos, u_neg, near0, float(u.max() - u.min())


def per_cycle_ls(
    u: np.ndarray,
    i: np.ndarray,
    rs: float,
    half_ticks: int,
    *,
    ts: float = TS_CTRL,
) -> list[float]:
    u_arr = np.asarray(u, dtype=float)
    u_ac = u_arr - float(np.median(u_arr))
    ls: list[float] = []
    n = len(u_ac)
    pos = 0
    while pos + 2 * half_ticks <= n:
        i0 = i[pos]
        psi_p = float(np.sum((u_ac[pos : pos + half_ticks] - rs * i[pos : pos + half_ticks]) * ts))
        di_p = float(i[pos + half_ticks - 1] - i0)
        psi_n = float(
            np.sum(
                (
                    u_ac[pos + half_ticks : pos + 2 * half_ticks]
                    - rs * i[pos + half_ticks : pos + 2 * half_ticks]
                )
                * ts
            )
        )
        di_n = float(i[pos + 2 * half_ticks - 1] - i[pos + half_ticks])
        den = di_p - di_n
        if abs(den) >= MIN_DI:
            l_h = (psi_p - psi_n) / den
            if L_MIN_H <= l_h < L_MAX_H:
                ls.append(l_h)
        pos += 2 * half_ticks
    return ls


def mcu_amp_median(cycles: list[float]) -> tuple[float | None, list[float]]:
    buckets: list[float] = []
    for a in range(AMP_STEPS):
        sl = cycles[a * CYCLES_PER_AMP : (a + 1) * CYCLES_PER_AMP]
        if sl:
            buckets.append(float(np.median(sl)))
    if not buckets:
        return None, buckets
    return float(np.median(buckets)) * 1e6, [b * 1e6 for b in buckets]


def analyze_stage(
    u_seg: np.ndarray,
    i_seg: np.ndarray,
    rs: float,
    f_hz: float,
    label: str,
) -> StageResult:
    u20 = upsample_ctrl_rate(u_seg)
    i20 = upsample_ctrl_rate(i_seg)
    ht = firmware_half_ticks(f_hz)
    cycles = per_cycle_ls(u20, i20, rs, ht, ts=TS_CTRL)
    l_mcu_agg, buckets = mcu_amp_median(cycles)
    l_seg_h = extract_l_vasi_psi(u_seg, i_seg, rs, f_hz)
    l_seg_uh = (l_seg_h * 1e6) if l_seg_h is not None else None
    u_pos, u_neg, near0, u_pk = waveform_stats(u_seg)
    i_pk = float(np.max(i_seg) - np.min(i_seg))
    return StageResult(
        label=label,
        f_hz=f_hz,
        l_seg_uh=l_seg_uh,
        l_mcu_agg_uh=l_mcu_agg,
        n_cycles=len(cycles),
        amp_buckets_uh=buckets,
        u_pos_pk=u_pos,
        u_neg_pk=u_neg,
        u_near_zero_frac=near0,
        f_dom_hz=dom_freq(u_seg),
        u_pk=u_pk,
        i_pk=i_pk,
    )


def pct_err(off: float | None, mcu: float) -> str:
    if off is None or not np.isfinite(mcu) or mcu == 0:
        return "—"
    return f"{100.0 * (off - mcu) / mcu:+.1f}%"


def analyze_csv(path: Path, *, rs: float = RS_DEFAULT) -> None:
    vf = load_vofa(path)
    rows = load_csv(path)
    hdr = find_header(rows)
    if hdr < 0:
        print(f"{path.name}: no ident burst")
        return
    burst = parse_burst(rows, hdr)
    seq = vf.channels["open_seq"]
    m57 = np.isclose(seq, 57, atol=0.5)
    if not np.any(m57):
        print(f"{path.name}: no open_seq=57")
        return
    i0 = int(np.where(m57)[0][0])
    id_ = vf.channels["id"][i0:]
    iq = vf.channels["iq"][i0:]
    ud = vf.channels["ud_out"][i0:]
    uq = vf.channels["uq_pi"][i0:]  # unified12 ch7 = foc_uq_out

    segs = segment_by_grid_targets(id_, iq, tol=0.12)
    print(f"\n{'=' * 78}")
    print(f"FILE: {path.name}")
    print(
        f"  proto={burst['proto']:.1f}  Rs={burst['rs_ohm']:.4f} ohm  "
        f"ok coarse/fine={burst['ld_lq_ok']}/{burst.get('ld_lq_ok_fine')}  "
        f"segments={len(segs)}/9"
    )
    print(
        f"  static ref: Ld={STATIC_LD_UH:.0f} uH  Lq={STATIC_LQ_UH:.0f} uH  "
        f"TELEM SC={TELEM_STAGE_COARSE} SF={TELEM_STAGE_FINE}  SETTLE={SETTLE_S}s"
    )
    print(f"  q-axis voltage: ch7 foc_uq_out (layout name uq_pi)")

    grid_map = {
        (round(g["id_bias"], 3), round(g["iq_bias"], 3)): g for g in burst["grid"]
    }

    for gi, (s, e, tb_id, tb_iq) in enumerate(segs):
        key = (round(tb_id, 3), round(tb_iq, 3))
        g = grid_map.get(key)
        if g is None:
            g = burst["grid"][gi] if gi < len(burst["grid"]) else {}

        inj0 = s + int(SETTLE_S * FS)
        ud_i = ud[inj0:e]
        id_i = id_[inj0:e]
        uq_i = uq[inj0:e]
        iq_i = iq[inj0:e]
        sl = inj_stage_slices(len(ud_i))
        if sl is None:
            print(f"\n  G{gi} ({tb_id},{tb_iq}) inj_len={len(ud_i)} — slice fail")
            continue

        stages = [
            analyze_stage(ud_i[sl["coarse_d"]], id_i[sl["coarse_d"]], rs, F_COARSE_HZ, "Ld@500Hz"),
            analyze_stage(ud_i[sl["fine_d"]], id_i[sl["fine_d"]], rs, F_FINE_HZ, "Ld@1kHz"),
            analyze_stage(uq_i[sl["coarse_q"]], iq_i[sl["coarse_q"]], rs, F_COARSE_HZ, "Lq@500Hz"),
            analyze_stage(uq_i[sl["fine_q"]], iq_i[sl["fine_q"]], rs, F_FINE_HZ, "Lq@1kHz"),
        ]

        print(f"\n  --- G{gi} bias Id={tb_id:.1f}A Iq={tb_iq:.1f}A  inj_len={len(ud_i)} ---")
        print(
            f"  {'stage':10s} {'off_seg':>8s} {'off_agg':>8s} {'MCU':>8s} "
            f"{'err_seg':>8s} {'err_agg':>8s} {'f_dom':>6s} {'u+':>6s} {'u-':>6s} {'~0%':>5s}"
        )
        mcu_keys = [
            ("Ld@500Hz", "ld_coarse_uH"),
            ("Ld@2kHz", "ld_fine_uH"),
            ("Lq@500Hz", "lq_coarse_uH"),
            ("Lq@2kHz", "lq_fine_uH"),
        ]
        for st, mkey in zip(stages, mcu_keys):
            mcu_v = float(g.get(mkey[1], float("nan")))
            print(
                f"  {st.label:10s} "
                f"{st.l_seg_uh if st.l_seg_uh is not None else float('nan'):8.1f} "
                f"{st.l_mcu_agg_uh if st.l_mcu_agg_uh is not None else float('nan'):8.1f} "
                f"{mcu_v:8.1f} "
                f"{pct_err(st.l_seg_uh, mcu_v):>8s} "
                f"{pct_err(st.l_mcu_agg_uh, mcu_v):>8s} "
                f"{st.f_dom_hz:6.0f} "
                f"{st.u_pos_pk:6.3f} "
                f"{st.u_neg_pk:6.3f} "
                f"{100*st.u_near_zero_frac:5.0f}"
            )

        # G0 detailed amp buckets
        if gi == 0:
            print("\n  G0 per-amp bucket medians (uH, MCU 7x10-cycle agg):")
            for st in stages:
                bk = st.amp_buckets_uh
                bk_s = ", ".join(f"{x:.0f}" for x in bk) if bk else "—"
                print(f"    {st.label:10s} n_cyc={st.n_cycles:2d}  buckets=[{bk_s}]")

    # Summary medians across matched grids
    print(f"\n  --- {path.name} summary (median over detected segments) ---")
    cols: dict[str, list[float]] = {k: [] for k in ["ldc_s", "ldf_s", "lqc_s", "lqf_s", "ldc_a", "ldf_a", "lqc_a", "lqf_a"]}
    mcu_cols: dict[str, list[float]] = {k: [] for k in ["ldc", "ldf", "lqc", "lqf"]}

    for gi, (s, e, tb_id, tb_iq) in enumerate(segs):
        key = (round(tb_id, 3), round(tb_iq, 3))
        g = grid_map.get(key, burst["grid"][gi] if gi < len(burst["grid"]) else {})
        inj0 = s + int(SETTLE_S * FS)
        sl = inj_stage_slices(len(ud[inj0:e]))
        if sl is None:
            continue
        ud_i = ud[inj0:e]
        id_i = id_[inj0:e]
        uq_i = uq[inj0:e]
        iq_i = iq[inj0:e]
        pairs = [
            ("ldc_s", "ldc_a", "ldc", ud_i[sl["coarse_d"]], id_i[sl["coarse_d"]], F_COARSE_HZ),
            ("ldf_s", "ldf_a", "ldf", ud_i[sl["fine_d"]], id_i[sl["fine_d"]], F_FINE_HZ),
            ("lqc_s", "lqc_a", "lqc", uq_i[sl["coarse_q"]], iq_i[sl["coarse_q"]], F_COARSE_HZ),
            ("lqf_s", "lqf_a", "lqf", uq_i[sl["fine_q"]], iq_i[sl["fine_q"]], F_FINE_HZ),
        ]
        for sk, ak, mk, uu, ii, ff in pairs:
            st = analyze_stage(uu, ii, rs, ff, "")
            if st.l_seg_uh is not None:
                cols[sk].append(st.l_seg_uh)
            if st.l_mcu_agg_uh is not None:
                cols[ak].append(st.l_mcu_agg_uh)
            mv = g.get({"ldc": "ld_coarse_uH", "ldf": "ld_fine_uH", "lqc": "lq_coarse_uH", "lqf": "lq_fine_uH"}[mk])
            if mv is not None and np.isfinite(mv):
                mcu_cols[mk].append(float(mv))

    def med(xs: list[float]) -> float:
        return float(np.median(xs)) if xs else float("nan")

    print(
        f"  {'':8s} {'seg_med':>8s} {'agg_med':>8s} {'MCU_med':>8s} "
        f"{'fine/coarse seg':>14s} {'fine/coarse MCU':>14s}"
    )
    for name, sk, ak, mk, mck in [
        ("Ld", "ldc_s", "ldc_a", "ld_coarse_uH", "ld_fine_uH"),
        ("", "ldf_s", "ldf_a", "ld_fine_uH", None),
        ("Lq", "lqc_s", "lqc_a", "lq_coarse_uH", "lq_fine_uH"),
        ("", "lqf_s", "lqf_a", "lq_fine_uH", None),
    ]:
        if name == "Ld":
            lc_s, lf_s = med(cols["ldc_s"]), med(cols["ldf_s"])
            lc_a, lf_a = med(cols["ldc_a"]), med(cols["ldf_a"])
            lc_m, lf_m = med(mcu_cols["ldc"]), med(mcu_cols["ldf"])
            print(
                f"  Ld@500   {lc_s:8.1f} {lc_a:8.1f} {lc_m:8.1f}   "
                f"{'':14s} {'':14s}"
            )
            print(
                f"  Ld@1k    {lf_s:8.1f} {lf_a:8.1f} {lf_m:8.1f}   "
                f"{lf_s/lc_s if lc_s else float('nan'):14.2f} "
                f"{lf_m/lc_m if lc_m else float('nan'):14.2f}"
            )
        elif name == "Lq":
            lc_s, lf_s = med(cols["lqc_s"]), med(cols["lqf_s"])
            lc_a, lf_a = med(cols["lqc_a"]), med(cols["lqf_a"])
            lc_m, lf_m = med(mcu_cols["lqc"]), med(mcu_cols["lqf"])
            print(
                f"  Lq@500   {lc_s:8.1f} {lc_a:8.1f} {lc_m:8.1f}   "
                f"{'':14s} {'':14s}"
            )
            print(
                f"  Lq@1k    {lf_s:8.1f} {lf_a:8.1f} {lf_m:8.1f}   "
                f"{lf_s/lc_s if lc_s else float('nan'):14.2f} "
                f"{lf_m/lc_m if lc_m else float('nan'):14.2f}"
            )


def main() -> None:
    ap = argparse.ArgumentParser(description="Detailed offline Ld/Lq vs MCU burst")
    ap.add_argument("csv", nargs="*", help="VOFA CSV paths or tags like 2253")
    ap.add_argument("--rs", type=float, default=RS_DEFAULT)
    ap.add_argument(
        "--dir",
        type=Path,
        default=Path(r"D:\stm32\STM32G474RET6_MOTOR\VOFA+CSV\20260707"),
    )
    args = ap.parse_args()
    paths: list[Path] = []
    for a in args.csv or ["2243", "2252", "2253"]:
        p = Path(a)
        if p.suffix.lower() == ".csv":
            paths.append(p)
        else:
            paths.append(args.dir / f"vofa+20260707{a}.csv")
    for p in paths:
        if p.is_file():
            analyze_csv(p, rs=args.rs)
        else:
            print(f"skip missing {p}")


if __name__ == "__main__":
    main()
