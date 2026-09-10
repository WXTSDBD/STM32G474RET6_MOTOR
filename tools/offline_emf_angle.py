#!/usr/bin/env python3
"""
Offline EMF / SMO electrical-angle observation vs encoder theta_el.

Gate for MCU: if offline debiased angle error at ~1000 rpm is poor,
do not port SMO to firmware yet.

Flux-ID VOFA layout (JustFloat x12, D=5 -> 4 kHz):
  Ia Ib Ic Id Iq theta_el Ud Uq omega_pll omega_ref Iq_ref enc_raw

Usage:
  python tools/offline_emf_angle.py VOFA+CSV/20260909/vofa+202609092307.csv
  python tools/offline_emf_angle.py path.csv --fs 4000 --rpm 1000
"""
from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

# Defaults from motor_params_m1.h + flux-ID profile
R_OHM = 0.122
LD_H = 59e-6
LQ_H = 87e-6
POLE_PAIRS = 7
PSI_F_WB = 0.00603  # from 2307 median (Uq equation)
FS_DEFAULT = 4000.0


def wrap_pi(x: np.ndarray | float) -> np.ndarray | float:
    return (x + np.pi) % (2.0 * np.pi) - np.pi


def clarke_ab(ia: np.ndarray, ib: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Amplitude-invariant Clarke (ic unused; consistent with FOC)."""
    return ia, (ia + 2.0 * ib) / np.sqrt(3.0)


def inv_park(ud: np.ndarray, uq: np.ndarray, th: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    c, s = np.cos(th), np.sin(th)
    return ud * c - uq * s, ud * s + uq * c


def lpf_1st(x: np.ndarray, fs: float, fc_hz: float) -> np.ndarray:
    a = 1.0 - np.exp(-2.0 * np.pi * fc_hz / fs)
    y = np.empty_like(x)
    y[0] = x[0]
    for k in range(1, len(x)):
        y[k] = y[k - 1] + a * (x[k] - y[k - 1])
    return y


@dataclass
class AngleStats:
    name: str
    n: int
    bias_deg: float
    abs_p50_deg: float
    abs_p95_deg: float
    rms_deg: float
    abs_max_deg: float
    emag_mean: float
    emag_theory: float


def angle_stats(
    name: str,
    th_hat: np.ndarray,
    th_enc: np.ndarray,
    mask: np.ndarray,
    emag: np.ndarray | None,
    emag_theory: float,
) -> AngleStats:
    d = wrap_pi(th_hat[mask] - th_enc[mask])
    bias = float(np.median(d))
    d0 = wrap_pi(d - bias)
    em = float(np.mean(emag[mask])) if emag is not None else float("nan")
    return AngleStats(
        name=name,
        n=int(np.sum(mask)),
        bias_deg=bias * 180.0 / np.pi,
        abs_p50_deg=float(np.median(np.abs(d0)) * 180.0 / np.pi),
        abs_p95_deg=float(np.percentile(np.abs(d0), 95) * 180.0 / np.pi),
        rms_deg=float(np.sqrt(np.mean(d0 ** 2)) * 180.0 / np.pi),
        abs_max_deg=float(np.max(np.abs(d0)) * 180.0 / np.pi),
        emag_mean=em,
        emag_theory=emag_theory,
    )


def steady_mask(wref: np.ndarray, fs: float, rpm: float, lead_s: float, tail_s: float) -> np.ndarray:
    at = np.abs(wref - rpm) < max(2.0, 0.02 * rpm)
    if not np.any(at):
        raise SystemExit(f"no samples with |omega_ref-{rpm}|<tol")
    # contiguous holds >= 2 s
    n = len(wref)
    mask = np.zeros(n, dtype=bool)
    i = 0
    while i < n:
        if not at[i]:
            i += 1
            continue
        j = i + 1
        while j < n and at[j]:
            j += 1
        if (j - i) >= int(2.0 * fs):
            s = i + int(lead_s * fs)
            e = j - int(tail_s * fs)
            if e > s:
                mask[s:e] = True
        i = j
    if not np.any(mask):
        raise SystemExit("steady mask empty; check --rpm / lead-tail")
    return mask


def emf_voltage_eq(
    ualpha: np.ndarray,
    ubeta: np.ndarray,
    ialpha: np.ndarray,
    ibeta: np.ndarray,
    fs: float,
    r: float,
    l: float,
    fc_hz: float,
) -> tuple[np.ndarray, np.ndarray]:
    ts = 1.0 / fs
    dia = np.gradient(ialpha, ts)
    dib = np.gradient(ibeta, ts)
    ea = ualpha - r * ialpha - l * dia
    eb = ubeta - r * ibeta - l * dib
    return lpf_1st(ea, fs, fc_hz), lpf_1st(eb, fs, fc_hz)


def emf_pll(
    ea: np.ndarray,
    eb: np.ndarray,
    fs: float,
    kp: float,
    ki: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Type-II EMF PLL; e ~ |e|[-sin θ, cos θ]."""
    ts = 1.0 / fs
    th = 0.0
    w = 0.0
    th_out = np.empty(len(ea))
    w_out = np.empty(len(ea))
    for k in range(len(ea)):
        em = np.hypot(ea[k], eb[k]) + 1e-9
        pe = (eb[k] * np.cos(th) - ea[k] * np.sin(th)) / em
        w += ki * pe * ts
        w_inst = w + kp * pe
        th = float(wrap_pi(th + w_inst * ts))
        th_out[k] = th
        w_out[k] = w_inst
    return th_out, w_out


def smo_exact_discrete(
    ualpha: np.ndarray,
    ubeta: np.ndarray,
    ialpha: np.ndarray,
    ibeta: np.ndarray,
    fs: float,
    r: float,
    l: float,
    k_slide: float,
    sat_a: float,
    e_lpf_hz: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Current SMO with exact RL discrete + sat sliding + LPF(z)->e.
    i[k+1] = a*i + ((1-a)/R)*(u - z), a=exp(-R Ts/L)
    """
    ts = 1.0 / fs
    a = float(np.exp(-r * ts / l))
    b = (1.0 - a) / r if r > 1e-9 else ts / l
    al = 1.0 - np.exp(-2.0 * np.pi * e_lpf_hz * ts)

    iha = ihb = 0.0
    ea = eb = 0.0
    ea_o = np.empty(len(ualpha))
    eb_o = np.empty(len(ualpha))
    iha_o = np.empty(len(ualpha))
    ihb_o = np.empty(len(ualpha))

    for k in range(len(ualpha)):
        err_a = iha - ialpha[k]
        err_b = ihb - ibeta[k]
        za = k_slide * float(np.clip(err_a / sat_a, -1.0, 1.0))
        zb = k_slide * float(np.clip(err_b / sat_a, -1.0, 1.0))
        iha = a * iha + b * (ualpha[k] - za)
        ihb = a * ihb + b * (ubeta[k] - zb)
        ea += al * (za - ea)
        eb += al * (zb - eb)
        ea_o[k] = ea
        eb_o[k] = eb
        iha_o[k] = iha
        ihb_o[k] = ihb
    return ea_o, eb_o, iha_o, ihb_o


def print_stats(rows: list[AngleStats], gate_p95_deg: float, gate_rms_deg: float) -> None:
    print()
    print(
        f"{'method':<22} {'n':>7} {'bias°':>8} {'|e|p50°':>8} {'|e|p95°':>8} "
        f"{'rms°':>7} {'|e|max°':>8} {'|E|V':>7} {'ωψ':>7} {'GATE':>6}"
    )
    for s in rows:
        ok = (s.abs_p95_deg <= gate_p95_deg) and (s.rms_deg <= gate_rms_deg)
        print(
            f"{s.name:<22} {s.n:7d} {s.bias_deg:8.2f} {s.abs_p50_deg:8.2f} "
            f"{s.abs_p95_deg:8.2f} {s.rms_deg:7.2f} {s.abs_max_deg:8.2f} "
            f"{s.emag_mean:7.3f} {s.emag_theory:7.3f} "
            f"{'PASS' if ok else 'FAIL':>6}"
        )
    print()
    print(
        f"GATE (debiased vs theta_el): |err|p95 <= {gate_p95_deg:.0f} deg "
        f"AND rms <= {gate_rms_deg:.0f} deg"
    )


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("csv", type=Path, help="VOFA JustFloat x12 CSV")
    ap.add_argument("--fs", type=float, default=FS_DEFAULT)
    ap.add_argument("--rpm", type=float, default=1000.0)
    ap.add_argument("--lead", type=float, default=1.0)
    ap.add_argument("--tail", type=float, default=0.5)
    ap.add_argument("--r", type=float, default=R_OHM)
    ap.add_argument("--ld", type=float, default=LD_H)
    ap.add_argument("--lq", type=float, default=LQ_H)
    ap.add_argument("--psi", type=float, default=PSI_F_WB)
    ap.add_argument("--pp", type=int, default=POLE_PAIRS)
    ap.add_argument("--emf-lpf", type=float, default=200.0)
    ap.add_argument("--pll-kp", type=float, default=120.0)
    ap.add_argument("--pll-ki", type=float, default=3000.0)
    ap.add_argument("--smo-k", type=float, default=12.0)
    ap.add_argument("--smo-sat", type=float, default=0.25)
    ap.add_argument("--gate-p95", type=float, default=15.0)
    ap.add_argument("--gate-rms", type=float, default=10.0)
    ap.add_argument("--npz", type=Path, default=None, help="optional .npz dump")
    args = ap.parse_args()

    raw = np.genfromtxt(args.csv, delimiter=",", skip_header=1)
    if raw.ndim != 2 or raw.shape[1] < 12:
        print(f"need 12 columns, got {None if raw.ndim < 2 else raw.shape[1]}", file=sys.stderr)
        return 2

    fs = args.fs
    ia, ib = raw[:, 0], raw[:, 1]
    id_, iq = raw[:, 3], raw[:, 4]
    th_enc = raw[:, 5]
    ud, uq = raw[:, 6], raw[:, 7]
    wpll, wref = raw[:, 8], raw[:, 9]
    n = len(raw)
    t = np.arange(n) / fs

    ialpha, ibeta = clarke_ab(ia, ib)
    ialpha_dq = id_ * np.cos(th_enc) - iq * np.sin(th_enc)
    ibeta_dq = id_ * np.sin(th_enc) + iq * np.cos(th_enc)
    ualpha, ubeta = inv_park(ud, uq, th_enc)

    mask = steady_mask(wref, fs, args.rpm, args.lead, args.tail)
    we = wpll[mask] * (2.0 * np.pi / 60.0) * args.pp
    emag_th = float(np.mean(np.abs(we) * args.psi))

    print("=" * 78)
    print(f"file: {args.csv}")
    print(f"dur={t[-1]:.2f}s  n={n}  fs={fs:.0f}Hz  steady@{args.rpm:.0f}rpm "
          f"n={mask.sum()} ({mask.sum()/fs:.2f}s)")
    print(f"params: R={args.r}  Ld={args.ld}  Lq={args.lq}  psi={args.psi}  p={args.pp}")
    print(
        f"clarke vs dq-invPark |iα| mae={np.mean(np.abs(ialpha[mask]-ialpha_dq[mask])):.5f} A"
    )
    print(f"theory |e|≈ωe·ψf mean={emag_th:.3f} V")

    # L for Veq: use Ld for isotropic; also try (Ld+Lq)/2
    l_iso = args.ld
    l_avg = 0.5 * (args.ld + args.lq)

    rows: list[AngleStats] = []

    ea, eb = emf_voltage_eq(ualpha, ubeta, ialpha, ibeta, fs, args.r, l_iso, args.emf_lpf)
    th_atan = np.arctan2(-ea, eb)
    rows.append(angle_stats("Veq+LPF+atan(Ld)", th_atan, th_enc, mask, np.hypot(ea, eb), emag_th))

    ea2, eb2 = emf_voltage_eq(ualpha, ubeta, ialpha, ibeta, fs, args.r, l_avg, args.emf_lpf)
    th_atan2 = np.arctan2(-ea2, eb2)
    rows.append(angle_stats("Veq+LPF+atan(Lavg)", th_atan2, th_enc, mask, np.hypot(ea2, eb2), emag_th))

    th_pll, _ = emf_pll(ea, eb, fs, args.pll_kp, args.pll_ki)
    rows.append(angle_stats("Veq+LPF+EMF-PLL", th_pll, th_enc, mask, np.hypot(ea, eb), emag_th))

    # SMO with exact discrete (may still struggle at 4 kHz / small L)
    for k_slide, tag in ((args.smo_k, "SMO+atan"), (args.smo_k * 1.5, "SMO*1.5+atan")):
        esa, esb, _, _ = smo_exact_discrete(
            ualpha, ubeta, ialpha, ibeta, fs, args.r, l_iso,
            k_slide, args.smo_sat, args.emf_lpf,
        )
        th_s = np.arctan2(-esa, esb)
        rows.append(angle_stats(tag, th_s, th_enc, mask, np.hypot(esa, esb), emag_th))

    # SMO + PLL on best-effort SMO EMF
    esa, esb, _, _ = smo_exact_discrete(
        ualpha, ubeta, ialpha, ibeta, fs, args.r, l_iso,
        args.smo_k, args.smo_sat, args.emf_lpf,
    )
    th_sp, _ = emf_pll(esa, esb, fs, args.pll_kp, args.pll_ki)
    rows.append(angle_stats("SMO+EMF-PLL", th_sp, th_enc, mask, np.hypot(esa, esb), emag_th))

    print_stats(rows, args.gate_p95, args.gate_rms)

    # MCU go/no-go: any Veq-family PASS counts as signal-path OK
    veq_pass = any(
        (r.abs_p95_deg <= args.gate_p95 and r.rms_deg <= args.gate_rms)
        for r in rows
        if r.name.startswith("Veq")
    )
    smo_pass = any(
        (r.abs_p95_deg <= args.gate_p95 and r.rms_deg <= args.gate_rms)
        for r in rows
        if r.name.startswith("SMO")
    )
    print("Verdict:")
    if veq_pass:
        print("  Veq path PASS — alpha/beta + Ud/Uq/theta consistent; MCU EMF observer is justified.")
    else:
        print("  Veq path FAIL — do NOT port angle observer to MCU until voltage/current path fixed.")
    if smo_pass:
        print("  Classic SMO PASS at telem rate — SMO firmware candidate OK.")
    else:
        print(
            "  Classic SMO FAIL at 4 kHz telem (expected if Ts/L stiff) — "
            "port Veq/EMF-PLL first, or re-record at 20 kHz before tuning SMO gains."
        )

    if args.npz:
        np.savez_compressed(
            args.npz,
            t=t,
            theta_enc=th_enc,
            theta_veq_atan=th_atan,
            theta_veq_pll=th_pll,
            e_alpha=ea,
            e_beta=eb,
            mask=mask,
            omega_ref=wref,
            omega_pll=wpll,
        )
        print(f"wrote {args.npz}")

    return 0 if veq_pass else 1


if __name__ == "__main__":
    raise SystemExit(main())
