#!/usr/bin/env python3
"""
Offline dual-freq VASI debug: replay ld_lq_ident.c PSI @ 20 kHz vs 10 kHz telem,
compare per-grid / per-stage with MCU ident burst (proto 2.0).

Firmware RS_LD_LQ_ONLY: F_COARSE=500 Hz, F_FINE=1000 Hz.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import numpy as np

from parse_ident_vofa import find_header, load_csv, parse_burst
from vofa_io import load_vofa

F_COARSE_HZ = 500.0
F_FINE_HZ = 1000.0
M1_CTRL_TS_S = 50e-6
TELEM_DECIM = 2
FS_TELEM = 10000.0
AMP_STEPS = 7
CYCLES_PER_AMP = 10
SETTLE_S = 0.5
RS_DEFAULT = 0.1225
L_MIN_H = 20e-6
L_MAX_H = 1e-3
MIN_DI = 0.05
ID_BIAS = [0.5, 1.0, 1.5]
IQ_BIAS = [0.0, 0.5, 1.0]


def half_ticks_ctrl(f_hz: float) -> int:
    half_s = 0.5 / f_hz
    return max(2, int(round(half_s / M1_CTRL_TS_S)))


def stage_samples_telem(f_hz: float) -> int:
    ht = half_ticks_ctrl(f_hz)
    return AMP_STEPS * CYCLES_PER_AMP * 2 * (ht // TELEM_DECIM)


def upsample_linear(x: np.ndarray, decim: int = TELEM_DECIM) -> np.ndarray:
    if decim <= 1 or len(x) < 2:
        return np.asarray(x, dtype=np.float64)
    n = len(x)
    out = np.empty((n - 1) * decim + 1, dtype=np.float64)
    for k in range(n - 1):
        for j in range(decim):
            a = j / decim
            out[k * decim + j] = (1.0 - a) * x[k] + a * x[k + 1]
    out[-1] = float(x[-1])
    return out


def extract_l_firmware(
    u: np.ndarray,
    i: np.ndarray,
    rs: float,
    f_hz: float,
    *,
    use_telem: bool = False,
    fs_telem: float = FS_TELEM,
) -> tuple[float | None, list[float]]:
    """Mirror ld_lq_ident.c: per-cycle L, then median over amp steps (7)."""
    if use_telem:
        half_s = 0.5 / f_hz
        ht = max(2, int(round(half_s * fs_telem)))
        ts = half_s / (2.0 * ht)
    else:
        ht = half_ticks_ctrl(f_hz)
        ts = M1_CTRL_TS_S

    per_cycle: list[float] = []
    pos = 0
    n = len(u)
    while pos + 2 * ht <= n:
        i0 = i[pos]
        psi_p = float(np.sum((u[pos : pos + ht] - rs * i[pos : pos + ht]) * ts))
        di_p = float(i[pos + ht - 1] - i0)
        psi_n = float(
            np.sum(
                (
                    u[pos + ht : pos + 2 * ht]
                    - rs * i[pos + ht : pos + 2 * ht]
                )
                * ts
            )
        )
        di_n = float(i[pos + 2 * ht - 1] - i[pos + ht])
        den = di_p - di_n
        if abs(den) >= MIN_DI:
            l_h = (psi_p - psi_n) / den
            if L_MIN_H <= l_h < L_MAX_H:
                per_cycle.append(l_h)
        pos += 2 * ht

    if not per_cycle:
        return None, per_cycle

    # group by CYCLES_PER_AMP like firmware amp buffer
    amp_medians: list[float] = []
    chunk = CYCLES_PER_AMP
    for a in range(AMP_STEPS):
        sl = per_cycle[a * chunk : (a + 1) * chunk]
        if sl:
            amp_medians.append(float(np.median(sl)))
    if not amp_medians:
        return None, per_cycle
    return float(np.median(amp_medians)), per_cycle


def fft_peak_hz(u: np.ndarray, fs: float) -> float | None:
    if len(u) < 256:
        return None
    u_ac = u - np.mean(u)
    spec = np.abs(np.fft.rfft(u_ac))
    freqs = np.fft.rfftfreq(len(u_ac), d=1.0 / fs)
    k = int(np.argmax(spec[1:]) + 1)
    return float(freqs[k])


def analyze_grid(
    ud: np.ndarray,
    uq: np.ndarray,
    id_: np.ndarray,
    iq: np.ndarray,
    rs: float,
) -> dict:
    settle = int(SETTLE_S * FS_TELEM)
    sc = stage_samples_telem(F_COARSE_HZ)
    sf = stage_samples_telem(F_FINE_HZ)
    off = 0
    out: dict = {}
    for axis, u_ch, i_ch, prefix in [
        (0, ud, id_, "ld"),
        (1, uq, iq, "lq"),
    ]:
        for stage, f_hz, length in [
            ("coarse", F_COARSE_HZ, sc),
            ("fine", F_FINE_HZ, sf),
        ]:
            seg_u = u_ch[off : off + length]
            seg_i = i_ch[off : off + length]
            off += length
            u20 = upsample_linear(seg_u)
            i20 = upsample_linear(seg_i)
            l_ctrl, _ = extract_l_firmware(u20, i20, rs, f_hz, use_telem=False)
            l_telem, _ = extract_l_firmware(seg_u, seg_i, rs, f_hz, use_telem=True)
            f_peak = fft_peak_hz(seg_u, FS_TELEM)
            out[f"{prefix}_{stage}_ctrl_uH"] = None if l_ctrl is None else l_ctrl * 1e6
            out[f"{prefix}_{stage}_telem_uH"] = None if l_telem is None else l_telem * 1e6
            out[f"{prefix}_{stage}_fft_hz"] = f_peak
            out[f"{prefix}_{stage}_u_pk"] = float(np.max(seg_u) - np.min(seg_u))
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("csv", type=Path)
    ap.add_argument("--rs", type=float, default=RS_DEFAULT)
    args = ap.parse_args()

    vf = load_vofa(args.csv)
    seq = vf.channels.get("open_seq")
    if seq is None:
        print("no open_seq", file=sys.stderr)
        return 1

    m57 = np.isclose(seq, 57, atol=0.5)
    i0 = int(np.where(m57)[0][0])
    # only first contiguous ph57 block (full 9-grid run)
    idx = np.where(m57)[0]
    gaps = np.where(np.diff(idx) > 1)[0]
    if len(gaps):
        i1 = int(idx[gaps[0]])
    else:
        i1 = int(idx[-1] + 1)

    ud = vf.channels["ud_out"][i0:i1]
    uq = vf.channels["uq_pi"][i0:i1]
    id_ = vf.channels["id"][i0:i1]
    iq = vf.channels["iq"][i0:i1]

    settle = int(SETTLE_S * FS_TELEM)
    sc = stage_samples_telem(F_COARSE_HZ)
    sf = stage_samples_telem(F_FINE_HZ)
    per_grid = sc + sf
    per_axis = 2 * per_grid
    grid_block = settle + per_axis

    print(f"=== {args.csv.name} ===")
    print(f"  ph57 block: {vf.t[i0]:.2f}s .. {vf.t[i1-1]:.2f}s  ({(i1-i0)/FS_TELEM:.2f}s)")
    print(f"  stage telem samples: coarse={sc} fine={sf}  ht_ctrl 500Hz={half_ticks_ctrl(F_COARSE_HZ)} 1kHz={half_ticks_ctrl(F_FINE_HZ)}")

    mcu = None
    hi = find_header(load_csv(args.csv))
    if hi >= 0:
        mcu = parse_burst(load_csv(args.csv), hi)
        print(f"  MCU burst proto={mcu['proto']:.1f} Rs={mcu['rs_ohm']:.4f} ok c/f={mcu['ld_lq_ok']}/{mcu.get('ld_lq_ok_fine')}")

    print(f"\n  {'grid':>12}  {'Ld c/f MCU':>14}  {'off c/f ctrl':>14}  {'off c/f telem':>16}  {'fft c/f Hz':>14}")
    rows = []
    for gi in range(9):
        id_i = gi // 3
        iq_i = gi % 3
        base = gi * grid_block
        if base + grid_block > len(ud):
            break
        g = analyze_grid(
            ud[base : base + grid_block],
            uq[base : base + grid_block],
            id_[base : base + grid_block],
            iq[base : base + grid_block],
            args.rs,
        )
        rows.append(g)
        id_b, iq_b = ID_BIAS[id_i], IQ_BIAS[iq_i]
        if mcu and gi < len(mcu["grid"]):
            mg = mcu["grid"][gi]
            mcu_ld = f"{mg['ld_coarse_uH']:.0f}/{mg['ld_fine_uH']:.0f}"
            mcu_lq = f"{mg['lq_coarse_uH']:.0f}/{mg['lq_fine_uH']:.0f}"
        else:
            mcu_ld = mcu_lq = "—"
        ldc = g.get("ld_coarse_ctrl_uH")
        ldf = g.get("ld_fine_ctrl_uH")
        lqc = g.get("lq_coarse_ctrl_uH")
        lqf = g.get("lq_fine_ctrl_uH")
        off_ld = (
            f"{ldc:.0f}/{ldf:.0f}"
            if ldc is not None and ldf is not None
            else "—"
        )
        off_lq = (
            f"{lqc:.0f}/{lqf:.0f}"
            if lqc is not None and lqf is not None
            else "—"
        )
        t_ld = (
            f"{g['ld_coarse_telem_uH']:.0f}/{g['ld_fine_telem_uH']:.0f}"
            if g.get("ld_coarse_telem_uH") is not None
            and g.get("ld_fine_telem_uH") is not None
            else "—"
        )
        t_lq = (
            f"{g['lq_coarse_telem_uH']:.0f}/{g['lq_fine_telem_uH']:.0f}"
            if g.get("lq_coarse_telem_uH") is not None
            and g.get("lq_fine_telem_uH") is not None
            else "—"
        )
        fft_d = (
            f"{g['ld_coarse_fft_hz']:.0f}/{g['ld_fine_fft_hz']:.0f}"
            if g.get("ld_coarse_fft_hz") is not None
            and g.get("ld_fine_fft_hz") is not None
            else "—"
        )
        print(
            f"  ({id_b:+.1f},{iq_b:+.1f})"
            f"  Ld {mcu_ld:>14}  {off_ld:>14}  {t_ld:>16}  {fft_d:>14}"
        )
        print(
            f"  {'':>12}  Lq {mcu_lq:>14}  {off_lq:>14}  {t_lq:>16}  "
            f"{g['lq_coarse_fft_hz']:.0f}/{g['lq_fine_fft_hz']:.0f}"
        )

    if rows:
        def med(key: str) -> float:
            v = [r[key] for r in rows if r.get(key) is not None]
            return float(np.median(v)) if v else float("nan")

        print("\n  --- medians ---")
        print(f"  offline ctrl  Ld coarse/fine: {med('ld_coarse_ctrl_uH'):.1f} / {med('ld_fine_ctrl_uH'):.1f} uH")
        print(f"  offline ctrl  Lq coarse/fine: {med('lq_coarse_ctrl_uH'):.1f} / {med('lq_fine_ctrl_uH'):.1f} uH")
        print(f"  offline telem Ld coarse/fine: {med('ld_coarse_telem_uH'):.1f} / {med('ld_fine_telem_uH'):.1f} uH")
        print(f"  offline telem Lq coarse/fine: {med('lq_coarse_telem_uH'):.1f} / {med('lq_fine_telem_uH'):.1f} uH")
        if mcu:
            mcu_ldc = np.median([g["ld_coarse_uH"] for g in mcu["grid"]])
            mcu_ldf = np.median([g["ld_fine_uH"] for g in mcu["grid"]])
            mcu_lqc = np.median([g["lq_coarse_uH"] for g in mcu["grid"]])
            mcu_lqf = np.median([g["lq_fine_uH"] for g in mcu["grid"]])
            print(f"  MCU burst     Ld coarse/fine: {mcu_ldc:.1f} / {mcu_ldf:.1f} uH")
            print(f"  MCU burst     Lq coarse/fine: {mcu_lqc:.1f} / {mcu_lqf:.1f} uH")
            print(
                f"  MCU/off ctrl fine ratio Ld={mcu_ldf/med('ld_fine_ctrl_uH'):.2f}  "
                f"Lq={mcu_lqf/med('lq_fine_ctrl_uH'):.2f}"
            )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
