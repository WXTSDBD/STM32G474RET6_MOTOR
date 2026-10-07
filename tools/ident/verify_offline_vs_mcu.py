#!/usr/bin/env python3
"""Verify offline script alignment vs MCU — correct stage lengths."""
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import numpy as np
from parse_ident_vofa import find_header, load_csv, parse_burst
from vofa_io import load_vofa
from ident.analyze_ld_lq_vasi import (
    F_COARSE_HZ,
    F_FINE_HZ,
    FS,
    SETTLE_S,
    TELEM_STAGE_COARSE,
    TELEM_STAGE_FINE,
    extract_l_axis,
    extract_l_vasi_psi,
    inj_stage_slices,
    segment_by_grid_targets,
    upsample_ctrl_rate,
)

RS = 0.1225
SC = TELEM_STAGE_COARSE
SF = TELEM_STAGE_FINE


def dom_freq(u: np.ndarray) -> float:
    u = u - np.median(u)
    zc = np.sum(np.abs(np.diff(np.sign(u))) > 0)
    return zc / (2.0 * len(u) / FS)


def analyze(path: Path) -> None:
    vf = load_vofa(path)
    seq = vf.channels["open_seq"]
    m57 = np.isclose(seq, 57, atol=0.5)
    idx = np.where(m57)[0]
    i0 = idx[0]
    id_ = vf.channels["id"][i0:]
    iq = vf.channels["iq"][i0:]
    ud = vf.channels["ud_out"][i0:]
    uq = vf.channels["uq_pi"][i0:]
    mcu = parse_burst(load_csv(path), find_header(load_csv(path)))
    segs = segment_by_grid_targets(id_, iq, tol=0.12)
    s, e, id0, iq0 = segs[0]
    inj0 = s + int(SETTLE_S * FS)
    ud_i = ud[inj0:e]
    id_i = id_[inj0:e]
    uq_i = uq[inj0:e]
    iq_i = iq[inj0:e]
    sl = inj_stage_slices(len(ud_i))
    g0 = mcu["grid"][0]

    print(f"\n=== {path.name}  G0 ({id0},{iq0}) ===")
    print(f"  TELEM SC={SC} SF={SF}  inj_len={len(ud_i)}  expect={2*(SC+SF)}")
    if sl:
        for k in sl:
            a, b = sl[k].start, sl[k].stop
            print(f"  slice {k:10s}: [{a}:{b}] n={b-a}")

    print(
        f"  MCU  Ld {g0['ld_coarse_uH']:.1f}/{g0['ld_fine_uH']:.1f}  "
        f"Lq {g0['lq_coarse_uH']:.1f}/{g0['lq_fine_uH']:.1f} uH"
    )

    print("  --- inj_stage_slices + extract_l_vasi_psi ---")
    for label, u, i, key, f in [
        ("Ld coarse", ud_i, id_i, "coarse_d", F_COARSE_HZ),
        ("Ld fine", ud_i, id_i, "fine_d", F_FINE_HZ),
        ("Lq coarse", uq_i, iq_i, "coarse_q", F_COARSE_HZ),
        ("Lq fine", uq_i, iq_i, "fine_q", F_FINE_HZ),
    ]:
        seg_u = u[sl[key]]
        seg_i = i[sl[key]]
        L = extract_l_vasi_psi(seg_u, seg_i, RS, f)
        lu = (L * 1e6) if L else float("nan")
        print(
            f"    {label:10s}  L={lu:6.1f} uH  f~{dom_freq(seg_u):.0f}Hz  "
            f"u_pk={seg_u.max()-seg_u.min():.3f}  n={len(seg_u)}"
        )

    print("  --- fixed head layout [Ld_c][Ld_f][Lq_c][Lq_f] ---")
    for label, axis, off, ln, ht in [
        ("Ld coarse", 0, 0, SC, 20),
        ("Ld fine", 0, SC, SF, 5),
        ("Lq coarse", 1, SC + SF, SC, 20),
        ("Lq fine", 1, 2 * SC + SF, SF, 5),
    ]:
        seg_u = (ud if axis == 0 else uq)[inj0 + off : inj0 + off + ln]
        seg_i = (id_ if axis == 0 else iq)[inj0 + off : inj0 + off + ln]
        u20 = upsample_ctrl_rate(seg_u)
        i20 = upsample_ctrl_rate(seg_i)
        L = extract_l_axis(u20, i20, RS, ht, ts=50e-6)
        lu = (L * 1e6) if L else float("nan")
        print(
            f"    {label:10s}  L={lu:6.1f} uH  f~{dom_freq(seg_u):.0f}Hz  "
            f"u_pk={seg_u.max()-seg_u.min():.3f}  n={len(seg_u)}"
        )

    # swap test: if MCU fine ~= offline coarse
    ldc = extract_l_vasi_psi(ud_i[sl["coarse_d"]], id_i[sl["coarse_d"]], RS, F_COARSE_HZ)
    ldf = extract_l_vasi_psi(ud_i[sl["fine_d"]], id_i[sl["fine_d"]], RS, F_FINE_HZ)
    if ldc and ldf:
        print(
            f"  label swap check: MCU_fine/Ld_off_coarse={g0['ld_fine_uH']/(ldc*1e6):.2f}  "
            f"MCU_fine/Ld_off_fine={g0['ld_fine_uH']/(ldf*1e6):.2f}  "
            f"(~1.0 if labels match)"
        )


if __name__ == "__main__":
    for tag in sys.argv[1:] or ["2243", "2252", "2253"]:
        analyze(
            Path(rf"D:\stm32\STM32G474RET6_MOTOR\VOFA+CSV\20260707\vofa+20260707{tag}.csv")
        )
