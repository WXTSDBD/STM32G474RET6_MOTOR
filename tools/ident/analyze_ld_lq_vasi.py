#!/usr/bin/env python3
"""Offline VASI Ld/Lq — paper §2.3 Δψ/ΔI + 9-point quadratic RLS (10 kHz telem).

Mirrors ld_lq_ident.c inject layout per grid cell:
  settle → d coarse(500Hz) → d fine(1kHz) → q coarse → q fine

@ 10 kHz (decim=2): fine 1 kHz half ≈ 5 samples → 20 kHz upsample + ht=10 ticks.
"""
from __future__ import annotations

import argparse
import sys
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import numpy as np

from vofa_io import load_vofa

ID_BIAS = np.array([0.5, 1.0, 1.5])
IQ_BIAS = np.array([0.0, 0.5, 1.0])
RS_OHM = 0.122
LD_NOMINAL_UH = 59.0
LQ_NOMINAL_UH = 87.0
L_NOM_H = 70e-6
# RS_LD_LQ_ONLY 固件：500 Hz coarse + 1 kHz fine（与 motor_params_m1.h 一致）
F_COARSE_HZ = 500.0
F_FINE_HZ = 1000.0
DI_TARGET_A = 0.12
I_RIPPLE_MARGIN_A = 0.15
U_INJ_MIN_V = 0.12
U_INJ_MAX_V = 0.50
SETTLE_S = 0.5
BIAS_RAMP_S = 0.12  # leg entry G0 only; G1..G8 step ref in firmware
AMP_STEPS = 7
CYCLES_PER_AMP = 10
VBUS = 24.0
# VOFA unified12 @ M1_TELEM_BRINGUP_DECIMATION=2 → 10 kHz (Nyquist 5 kHz)
FS = 10000.0
TS_TELEM = 1.0 / FS
M1_CTRL_TS_S = 50e-6
TELEM_DECIMATION = 2
# Firmware inject stage lengths @ 20 kHz ctrl → telem sample counts
_CTRL_HALF_COARSE = max(1, int(round(0.5 / F_COARSE_HZ / M1_CTRL_TS_S)))
_CTRL_HALF_FINE = max(1, int(round(0.5 / F_FINE_HZ / M1_CTRL_TS_S)))
_N_INJ_HALVES = AMP_STEPS * CYCLES_PER_AMP * 2


def telem_stage_samples(f_hz: float) -> int:
    """Wall-time telem length for one axis stage (7 amp × 10 cyc × 2 halves)."""
    return int(round(_N_INJ_HALVES * 0.5 / f_hz * FS))


TELEM_STAGE_COARSE = telem_stage_samples(F_COARSE_HZ)
TELEM_STAGE_FINE = telem_stage_samples(F_FINE_HZ)
# PSI：与 ld_lq_ident.c 一致 — 20 kHz 下半周 5 tick；遥测 decim=2 时先插值到控制率
TS_PSI_CTRL = M1_CTRL_TS_S
HALF_TICKS_COARSE_CTRL = _CTRL_HALF_COARSE
HALF_TICKS_FINE_CTRL = _CTRL_HALF_FINE
# legacy telem-native half (deprecated — kept for reference)
HALF_TICKS_COARSE = max(2, int(round(0.5 / F_COARSE_HZ / TS_TELEM)))
MIN_DI = 0.05
L_MAX_H = 0.001
L_MIN_H = 20e-6
MIN_LD_OK = 7
MIN_LQ_OK = 7
VASI_SEQ_BASE = 170
VASI_SEQ_STRIDE = 40
VASI_SEQ_PRE_DECAY = 163
VASI_SEQ_LUT_ROUND_OFFSET = 100
LUT_ROUND_NAMES = {0: "OFF", 1: "LUT"}
BIAS_TOL_A = 0.05
# merge brief ref glitches (samples) when splitting a grid cell
GLITCH_MERGE_SAMPLES = 50
# Square-wave fundamental / sinusoid amplitude (4/π)
SQ_FUNDAMENTAL = 4.0 / np.pi



def clarke_park_id_iq(
    ia: np.ndarray, ib: np.ndarray, ic: np.ndarray, theta_el: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """Match foc_svpwm.c Clarke + Park_sc (same as motor_current.c)."""
    i_alpha = ia
    i_beta = (ib - ic) / np.sqrt(3.0)
    c = np.cos(theta_el)
    s = np.sin(theta_el)
    id_ = i_alpha * c + i_beta * s
    iq = -i_alpha * s + i_beta * c
    return id_, iq


def upsample_ctrl_rate(x: np.ndarray, decim: int = TELEM_DECIMATION) -> np.ndarray:
    """Piecewise-linear 10 kHz telem → 20 kHz control-rate series."""
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


def firmware_half_ticks(f_hz: float) -> int:
    half_s = 0.5 / f_hz
    return max(2, int(round(half_s / M1_CTRL_TS_S)))


def grid_ref_hits(id_a: np.ndarray, iq_a: np.ndarray, *, tol: float = BIAS_TOL_A) -> int:
    """Count how many of 9 bias cells have a sustained plateau in ref/fb."""
    hits = 0
    min_run = 500
    for gi in range(len(ID_BIAS) * len(IQ_BIAS)):
        tb_id, tb_iq = grid_bias(gi)
        m = (np.abs(id_a - tb_id) < tol) & (np.abs(iq_a - tb_iq) < tol)
        if np.count_nonzero(m) >= min_run:
            hits += 1
    return hits


def resolve_grid_refs(
    id_ref: np.ndarray,
    iq_ref: np.ndarray,
    id_fb: np.ndarray,
    iq_fb: np.ndarray,
    vasi_mask: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, str]:
    """
    Pick segmentation driver. VASI 过程 telem 占用 ch8/9 时 id_ref 恒为 0 → fallback 到 Id/Iq 反馈。
    """
    if not np.any(vasi_mask):
        return id_ref, iq_ref, "ref"

    hits_ref = grid_ref_hits(id_ref[vasi_mask], iq_ref[vasi_mask])
    hits_fb = grid_ref_hits(id_fb[vasi_mask], iq_fb[vasi_mask], tol=0.12)
    if hits_ref >= 5:
        return id_ref, iq_ref, "ref"
    if hits_fb > hits_ref:
        return id_fb, iq_fb, "fb"
    if hits_fb >= 3:
        return id_fb, iq_fb, "fb"
    return id_ref, iq_ref, "ref"


def grid_bias(idx: int) -> tuple[float, float]:
    iq_i = idx % len(IQ_BIAS)
    id_i = idx // len(IQ_BIAS)
    return float(ID_BIAS[id_i]), float(IQ_BIAS[iq_i])


def min_segment_samples() -> int:
    per_axis = TELEM_STAGE_COARSE + TELEM_STAGE_FINE
    return max(4000, int(SETTLE_S * FS) + 2 * per_axis - 1500)


def _runs_from_mask(mask: np.ndarray) -> list[tuple[int, int]]:
    runs: list[tuple[int, int]] = []
    i = 0
    n = len(mask)
    while i < n:
        if not mask[i]:
            i += 1
            continue
        j = i + 1
        while j < n and mask[j]:
            j += 1
        runs.append((i, j))
        i = j
    return runs


def _merge_runs(runs: list[tuple[int, int]], max_gap: int) -> list[tuple[int, int]]:
    if not runs:
        return []
    merged = [runs[0]]
    for s, e in runs[1:]:
        ps, pe = merged[-1]
        if s - pe <= max_gap:
            merged[-1] = (ps, e)
        else:
            merged.append((s, e))
    return merged


def segment_by_grid_targets(
    id_ref: np.ndarray,
    iq_ref: np.ndarray,
    *,
    min_stable: int | None = None,
    tol: float = BIAS_TOL_A,
    max_gap: int = GLITCH_MERGE_SAMPLES,
) -> list[tuple[int, int, float, float]]:
    """
    按 9 个已知 (Id,Iq) 目标找注入窗；合并单点/短 glitch（如 G6 的 1 拍 0.17A）。
    兼容 G0 leg 入口 ramp；G1..G8 阶跃 ref（目标附近 ref 平坦）。
    """
    if min_stable is None:
        min_stable = min_segment_samples()

    segs: list[tuple[int, int, float, float]] = []
    for gi in range(len(ID_BIAS) * len(IQ_BIAS)):
        tb_id, tb_iq = grid_bias(gi)
        mask = (np.abs(id_ref - tb_id) < tol) & (np.abs(iq_ref - tb_iq) < tol)
        runs = _merge_runs(_runs_from_mask(mask), max_gap)
        if not runs:
            continue
        s, e = max(runs, key=lambda ab: ab[1] - ab[0])
        if e - s >= min_stable:
            segs.append((s, e, tb_id, tb_iq))
    return segs


def segment_by_bias(id_ref: np.ndarray, iq_ref: np.ndarray) -> list[tuple[int, int, float, float]]:
    """Legacy plateau grouper; kept for reference — use segment_by_grid_targets."""
    return segment_by_grid_targets(id_ref, iq_ref)


def u_inj_cap(i_bias: float, f_hz: float) -> float:
    room = max(0.0, abs(i_bias) - I_RIPPLE_MARGIN_A)
    return min(U_INJ_MAX_V, 2.0 * f_hz * L_NOM_H * room)


def extract_l_axis(
    u: np.ndarray,
    i: np.ndarray,
    rs: float,
    half_ticks: int,
    *,
    ts: float = TS_PSI_CTRL,
    u_bias: float | None = None,
) -> float | None:
    """Paper VASI: median L from symmetric square halves, ψ=∫(U−Rs·I)dt, L=Δψ/ΔI.

    u_bias: subtract from u before ψ (matches M1_LD_LQ_IDENT_PSI_USE_U_AC=1 on MCU).
    Default None → median(u) per segment.
    """
    u_arr = np.asarray(u, dtype=float)
    if u_bias is None:
        u_bias = float(np.median(u_arr))
    u_ac = u_arr - u_bias
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

    if not ls:
        return None
    return float(np.median(ls))


def inj_stage_slices(n_inj: int, *, head: int = 0) -> dict[str, slice] | None:
    """
    Inject sub-stages within [head : head + 2*(coarse+fine)].
    Layout: [d_coarse][d_fine][q_coarse][q_fine] — matches ld_lq_ident.c order.

    Use head=0 relative to inject start (after SETTLE_S).  Do NOT tail-anchor:
    inj_len often exceeds nominal by ~0–160 telem samples; tail anchor shifts
    all windows into wrong sub-stages (e.g. Lq fine u_pk≈4 V glitch).
    """
    total = 2 * (TELEM_STAGE_COARSE + TELEM_STAGE_FINE)
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
    return {
        "coarse_d": slice(d0, d1),
        "fine_d": slice(df0, df1),
        "coarse_q": slice(q0, q1),
        "fine_q": slice(qf0, qf1),
    }


def _median_valid(*vals: float | None) -> float | None:
    ok = [v for v in vals if v is not None]
    if not ok:
        return None
    return float(np.median(ok))


def extract_l_vasi_psi(
    u: np.ndarray,
    i: np.ndarray,
    rs: float,
    f_hz: float,
    *,
    upsample: bool = False,
) -> float | None:
    """
    Δψ/ΔI，对齐 ld_lq_ident.c 签收量级。

    10 kHz 遥测（decim=2）默认：ht≈round(half_s/TS_TELEM)，ts=half_s/(2·ht)。
    在 0014/0022 上与 MCU -777777 突发 Ld 逐格偏差约 1–4%（优于旧版 ht=2,ts=50µs 偏低 ~29%）。
    --upsample-ctrl：线性插值到 20 kHz 后 5×50 µs（实验，一般更偏高）。
    """
    half_s = 0.5 / f_hz
    if upsample and len(u) >= 2:
        u = upsample_ctrl_rate(u)
        i = upsample_ctrl_rate(i)
        ht = firmware_half_ticks(f_hz)
        ts = TS_PSI_CTRL
        return extract_l_axis(u, i, rs, ht, ts=ts)

    # 500 Hz / 2 kHz：默认 10 kHz 线性插值到 20 kHz 后与 ld_lq_ident.c 一致
    if len(u) >= 2:
        u = upsample_ctrl_rate(u)
        i = upsample_ctrl_rate(i)
        ht = firmware_half_ticks(f_hz)
        ts = TS_PSI_CTRL
        return extract_l_axis(u, i, rs, ht, ts=ts)

    if f_hz <= F_COARSE_HZ * 1.25:
        ht = max(2, int(round(half_s / TS_TELEM)))
        ts = half_s / (2.0 * ht)
        return extract_l_axis(u, i, rs, ht, ts=ts)

    # fine @ 2 kHz：10 kHz 遥测仅 ~2 点/半周，用相邻样本差分（易偏高，见 debug_ld_lq_dual_freq.py）
    ht = max(2, int(round(half_s / TS_TELEM)))
    ts = half_s / (2.0 * ht)
    ls: list[float] = []
    for k in range(1, len(u) - 1, 2):
        psi_p = (u[k] - rs * i[k]) * ts
        psi_n = (u[k + 1] - rs * i[k + 1]) * ts
        di_p = i[k] - i[k - 1]
        di_n = i[k + 1] - i[k]
        den = di_p - di_n
        if abs(den) >= MIN_DI:
            l_h = (psi_p - psi_n) / den
            if L_MIN_H <= l_h < L_MAX_H:
                ls.append(l_h)
    if len(ls) < 3:
        return None
    return float(np.median(ls))


def extract_l_d_from_inj(
    u_d: np.ndarray, i_d: np.ndarray, rs: float, method: str, *, upsample: bool = False
) -> float | None:
    sl = inj_stage_slices(len(u_d))
    if sl is None:
        return None
    if method in ("psi", "both"):
        lc = extract_l_vasi_psi(
            u_d[sl["coarse_d"]], i_d[sl["coarse_d"]], rs, F_COARSE_HZ, upsample=upsample
        )
        lf = extract_l_vasi_psi(
            u_d[sl["fine_d"]], i_d[sl["fine_d"]], rs, F_FINE_HZ, upsample=upsample
        )
        ld = _median_valid(lc, lf) if method == "both" else lc
    else:
        ld = extract_l_harmonic_balance(u_d[sl["coarse_d"]], i_d[sl["coarse_d"]], rs, F_COARSE_HZ, FS)
    return ld


def extract_l_q_from_inj(
    u_q: np.ndarray, i_q: np.ndarray, rs: float, method: str, *, upsample: bool = False
) -> float | None:
    sl = inj_stage_slices(len(u_q))
    if sl is None:
        return None
    if method in ("psi", "both"):
        lc = extract_l_vasi_psi(
            u_q[sl["coarse_q"]], i_q[sl["coarse_q"]], rs, F_COARSE_HZ, upsample=upsample
        )
        lf = extract_l_vasi_psi(
            u_q[sl["fine_q"]], i_q[sl["fine_q"]], rs, F_FINE_HZ, upsample=upsample
        )
        lq = _median_valid(lc, lf) if method == "both" else lc
    else:
        lq = extract_l_harmonic_balance(u_q[sl["coarse_q"]], i_q[sl["coarse_q"]], rs, F_COARSE_HZ, FS)
    return lq


def extract_l_harmonic_balance(
    u: np.ndarray, i: np.ndarray, rs: float, f_hz: float, fs: float = FS
) -> float | None:
    """
    Harmonic balance @ injection fundamental: L ≈ |U1| / (ω |I1|).
    u should be terminal voltage minus Rs drop; square-wave U uses 4/π factor.
    """
    n = len(u)
    min_n = int(max(4.0 * fs / f_hz, 200))
    if n < min_n:
        return None

    u_eff = u - rs * i
    u_ac = u_eff - float(np.mean(u_eff))
    i_ac = i - float(np.mean(i))

    spec_u = np.fft.rfft(u_ac)
    spec_i = np.fft.rfft(i_ac)
    freqs = np.fft.rfftfreq(n, d=1.0 / fs)
    k = int(np.argmin(np.abs(freqs - f_hz)))
    if abs(freqs[k] - f_hz) > f_hz * 0.05:
        return None

    u1 = abs(spec_u[k]) * 2.0 / n
    i1 = abs(spec_i[k]) * 2.0 / n
    if i1 < 1e-4:
        return None

    omega = 2.0 * np.pi * f_hz
    l_h = (SQ_FUNDAMENTAL * u1) / (omega * i1)
    if L_MIN_H <= l_h < L_MAX_H:
        return float(l_h)
    return None


def extract_l_harmonic_dual(u: np.ndarray, i: np.ndarray, rs: float) -> float | None:
    vals: list[float] = []
    for f_hz in (F_COARSE_HZ, F_FINE_HZ):
        l_h = extract_l_harmonic_balance(u, i, rs, f_hz)
        if l_h is not None:
            vals.append(l_h)
    if not vals:
        return None
    return float(np.median(vals))


def rls_quad_surface(id_a: np.ndarray, iq_a: np.ndarray, l_h: np.ndarray) -> tuple[np.ndarray, float]:
    """L = a0 + a1*Id + a2*Iq + a3*Id^2 + a4*Iq^2 + a5*Id*Iq"""
    x = np.column_stack(
        [
            np.ones_like(id_a),
            id_a,
            iq_a,
            id_a * id_a,
            iq_a * iq_a,
            id_a * iq_a,
        ]
    )
    coeff, *_ = np.linalg.lstsq(x, l_h, rcond=None)
    pred = x @ coeff
    rms = float(np.sqrt(np.mean((pred - l_h) ** 2)))
    return coeff, rms


def eval_quad_surface(coeff: np.ndarray, id_a: np.ndarray, iq_a: np.ndarray) -> np.ndarray:
    return (
        coeff[0]
        + coeff[1] * id_a
        + coeff[2] * iq_a
        + coeff[3] * id_a * id_a
        + coeff[4] * iq_a * iq_a
        + coeff[5] * id_a * iq_a
    )


def merge_grid_median(
    pts: list[tuple[int, float, float, float]],
) -> list[tuple[float, float, float]]:
    """Per (Id,Iq) cell median across angle legs → [(Id,Iq,L_uH), ...]."""
    bins: dict[tuple[float, float], list[float]] = defaultdict(list)
    for _leg, id0, iq0, l_uh in pts:
        key = (round(id0, 2), round(iq0, 2))
        bins[key].append(l_uh)
    return [(k[0], k[1], float(np.median(v))) for k, v in sorted(bins.items())]


def leg_index_from_open_seq(open_seq_val: int, angle_leg: float | None) -> int:
    if open_seq_val == 57:
        return 0
    if open_seq_val >= VASI_SEQ_BASE:
        return (open_seq_val - VASI_SEQ_BASE) // VASI_SEQ_STRIDE
    if open_seq_val == 14 and angle_leg is not None and abs(angle_leg - 2.0) < 0.5:
        return 2
    return 0


def vasi_open_seq_for_leg(leg_i: int) -> int:
    return VASI_SEQ_BASE + leg_i * VASI_SEQ_STRIDE


def vasi_progress_open_seq(lut_round: int) -> int:
    return 57 + lut_round * VASI_SEQ_LUT_ROUND_OFFSET


def discover_leg_masks(
    open_seq_r: np.ndarray,
    angle_r: np.ndarray | None,
    t_r: np.ndarray,
    t0: float,
    lut_round_r: np.ndarray | None = None,
) -> list[tuple[int, np.ndarray]]:
    """Return [(lut_round, bool_mask), ...] for each VASI leg in the recording."""
    found: list[tuple[int, np.ndarray]] = []
    seen: set[int] = set()

    # Prefer ch10 = rs_l_ident_lut_round (0=OFF, 1=LUT) when present — open_seq 57/157 alone
    # spans the whole multi-grid VASI and must not be used as the only time mask.
    if lut_round_r is not None:
        vasi_any = (open_seq_r >= 57) & (open_seq_r <= 59)
        vasi_any |= (open_seq_r >= 57 + VASI_SEQ_LUT_ROUND_OFFSET) & (
            open_seq_r <= 59 + VASI_SEQ_LUT_ROUND_OFFSET
        )
        vasi_any |= (open_seq_r >= VASI_SEQ_BASE) & (open_seq_r < VASI_SEQ_BASE + 8 * VASI_SEQ_STRIDE)
        for lut_round in range(2):
            m = vasi_any & (np.abs(lut_round_r - float(lut_round)) < 0.5)
            if np.count_nonzero(m) >= min_segment_samples() and lut_round not in seen:
                found.append((lut_round, m))
                seen.add(lut_round)

    for lut_round in range(2):
        if lut_round in seen:
            continue
        seq_v = vasi_progress_open_seq(lut_round)
        m = open_seq_r == seq_v
        if np.count_nonzero(m) >= min_segment_samples() and lut_round not in seen:
            found.append((lut_round, m))
            seen.add(lut_round)

    for seq_v in sorted(set(int(v) for v in open_seq_r)):
        if seq_v >= VASI_SEQ_BASE and (seq_v - VASI_SEQ_BASE) % VASI_SEQ_STRIDE == 0:
            leg_i = (seq_v - VASI_SEQ_BASE) // VASI_SEQ_STRIDE
            m = open_seq_r == seq_v
            if np.count_nonzero(m) >= min_segment_samples() and leg_i not in seen:
                found.append((leg_i, m))
                seen.add(leg_i)

    if angle_r is not None:
        m14 = (open_seq_r == 14) & (np.abs(angle_r - 2.0) < 0.5)
        if np.count_nonzero(m14) >= min_segment_samples() and 2 not in seen:
            found.append((2, m14))
            seen.add(2)

    if not found:
        m = (open_seq_r >= 57) | ((open_seq_r >= 170) & (open_seq_r < 320))
        m |= (open_seq_r >= 57 + VASI_SEQ_LUT_ROUND_OFFSET) & (
            open_seq_r < 60 + VASI_SEQ_LUT_ROUND_OFFSET
        )
        if np.any(m):
            found.append((0, m))
    return sorted(found, key=lambda x: x[0])


def extract_points_from_mask(
    seg_mask: np.ndarray,
    id_r: np.ndarray,
    iq_r: np.ndarray,
    ud: np.ndarray,
    uq: np.ndarray,
    id_fb: np.ndarray,
    iq_fb: np.ndarray,
    leg_i: int,
    method: str,
    rs_ohm: float,
    *,
    seg_tol: float = BIAS_TOL_A,
    upsample: bool = False,
) -> tuple[list[tuple[int, float, float, float]], list[tuple[int, float, float, float]]]:
    ld_pts: list[tuple[int, float, float, float]] = []
    lq_pts: list[tuple[int, float, float, float]] = []

    id_rr = id_r[seg_mask]
    iq_rr = iq_r[seg_mask]
    ud_r = ud[seg_mask]
    uq_r = uq[seg_mask]
    idf_r = id_fb[seg_mask]
    iqf_r = iq_fb[seg_mask]

    for s, e, id0, iq0 in segment_by_grid_targets(id_rr, iq_rr, tol=seg_tol):
        if e - s < min_segment_samples():
            continue
        inj_start = s + int(SETTLE_S * FS)
        inj_end = e
        u_d = ud_r[inj_start:inj_end]
        u_q = uq_r[inj_start:inj_end]
        i_d = idf_r[inj_start:inj_end]
        i_q = iqf_r[inj_start:inj_end]

        ld = extract_l_d_from_inj(u_d, i_d, rs_ohm, method, upsample=upsample)
        lq = extract_l_q_from_inj(u_q, i_q, rs_ohm, method, upsample=upsample)

        if method == "both":
            sl = inj_stage_slices(len(u_d))
            if sl is not None:
                ld_h = extract_l_harmonic_balance(
                    u_d[sl["coarse_d"]], i_d[sl["coarse_d"]], rs_ohm, F_COARSE_HZ, FS
                )
                lq_h = extract_l_harmonic_balance(
                    u_q[sl["coarse_q"]], i_q[sl["coarse_q"]], rs_ohm, F_COARSE_HZ, FS
                )
                ld = _median_valid(ld, ld_h)
                lq = _median_valid(lq, lq_h)

        if ld is not None:
            ld_pts.append((leg_i, id0, iq0, ld * 1e6))
        if lq is not None:
            lq_pts.append((leg_i, id0, iq0, lq * 1e6))

    return ld_pts, lq_pts


def fit_surface(
    pts_uh: list[tuple[float, float, float]], min_pts: int = 6
) -> dict | None:
    if len(pts_uh) < min_pts:
        return None
    id_a = np.array([p[0] for p in pts_uh])
    iq_a = np.array([p[1] for p in pts_uh])
    l_h = np.array([p[2] for p in pts_uh]) * 1e-6
    coeff, rms = rls_quad_surface(id_a, iq_a, l_h)
    return {
        "coeff": coeff.tolist(),
        "rms_uH": rms * 1e6,
        "at_1A0_uH": float(eval_quad_surface(coeff, 1.0, 0.0)) * 1e6,
        "at_1A1_uH": float(eval_quad_surface(coeff, 1.0, 1.0)) * 1e6,
    }


def print_surface_table(coeff: np.ndarray, label: str) -> None:
    print(f"\n  {label} surface (uH) — rows Id, cols Iq:")
    hdr = "        " + "  ".join(f"Iq={q:.1f}" for q in IQ_BIAS)
    print(hdr)
    for id_v in ID_BIAS:
        row = f"  Id={id_v:.1f} "
        for iq_v in IQ_BIAS:
            l_uh = float(eval_quad_surface(coeff, id_v, iq_v)) * 1e6
            row += f"  {l_uh:6.1f}"
        print(row)


def plot_surface(path: Path, ld_coeff: np.ndarray, lq_coeff: np.ndarray | None) -> None:
    try:
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    except ImportError:
        print("  [--plot] matplotlib not installed, skip surface plot")
        return

    id_g = np.linspace(float(ID_BIAS[0]), float(ID_BIAS[-1]), 25)
    iq_g = np.linspace(float(IQ_BIAS[0]), float(IQ_BIAS[-1]), 25)
    ID, IQ = np.meshgrid(id_g, iq_g)

    fig = plt.figure(figsize=(10, 4))
    ax1 = fig.add_subplot(121, projection="3d")
    ld_s = eval_quad_surface(ld_coeff, ID, IQ) * 1e6
    ax1.plot_surface(ID, IQ, ld_s, cmap="viridis", alpha=0.85)
    ax1.set_title("Ld(Id,Iq) uH")
    ax1.set_xlabel("Id (A)")
    ax1.set_ylabel("Iq (A)")

    if lq_coeff is not None:
        ax2 = fig.add_subplot(122, projection="3d")
        lq_s = eval_quad_surface(lq_coeff, ID, IQ) * 1e6
        ax2.plot_surface(ID, IQ, lq_s, cmap="plasma", alpha=0.85)
        ax2.set_title("Lq(Id,Iq) uH")
        ax2.set_xlabel("Id (A)")
        ax2.set_ylabel("Iq (A)")

    out_png = path.with_suffix(".ld_lq_surface.png")
    fig.tight_layout()
    fig.savefig(out_png, dpi=120)
    plt.close(fig)
    print(f"  surface plot → {out_png}")


def analyze_csv(
    path: Path,
    method: str = "psi",
    rs_ohm: float = RS_OHM,
    *,
    from_abc: bool = False,
    upsample: bool = False,
) -> dict:
    vf = load_vofa(path, fs=FS)
    t = vf.t
    id_fb = vf.ch("id")
    iq_fb = vf.ch("iq")
    if from_abc and vf.has("ia") and vf.has("ib") and vf.has("ic") and vf.has("theta"):
        id_fb, iq_fb = clarke_park_id_iq(
            vf.ch("ia"), vf.ch("ib"), vf.ch("ic"), vf.ch("theta")
        )
    id_ref = vf.ch("id_ref")
    iq_ref = vf.ch("iq_ref")
    open_seq = vf.ch("open_seq").astype(np.int32)
    ud = vf.ch("ud_out") if vf.has("ud_out") else vf.ch("ud_pi")
    # unified12 ch7 = foc_uq_out (含 VASI 注入)
    uq = vf.ch("uq_out") if vf.has("uq_out") else vf.ch("uq_pi")
    # ch10 = rs_l_ident_lut_round (0/1) during Rs+VASI when dual-round enabled; else duty_dev noise
    lut_round_ch = vf.ch("duty_dev") if vf.has("duty_dev") else None

    mask = (open_seq >= 57) & (open_seq <= 59)
    mask |= (open_seq >= 57 + VASI_SEQ_LUT_ROUND_OFFSET) & (
        open_seq <= 59 + VASI_SEQ_LUT_ROUND_OFFSET
    )
    mask |= (open_seq >= VASI_SEQ_BASE) & (
        (open_seq.astype(np.int32) - VASI_SEQ_BASE) % VASI_SEQ_STRIDE == 0
    ) & (open_seq < VASI_SEQ_BASE + 8 * VASI_SEQ_STRIDE)
    if lut_round_ch is not None:
        mask |= (open_seq == 14) & (np.abs(lut_round_ch - 2.0) < 0.5) & (t > 30.0)
    if not np.any(mask):
        return {"file": str(path), "error": "no L ident segment (open_seq 57/157/170+/14@leg2)"}

    t0 = float(t[np.argmax(mask)])
    run = t >= t0

    id_ref_full = id_ref[run]
    iq_ref_full = iq_ref[run]
    idf_full = id_fb[run]
    iqf_full = iq_fb[run]
    id_r, iq_r, grid_src = resolve_grid_refs(
        id_ref_full, iq_ref_full, idf_full, iqf_full, open_seq[run] == 57
    )
    seg_tol = 0.12 if grid_src == "fb" else BIAS_TOL_A

    open_seq_r = open_seq[run]
    lut_round_r = lut_round_ch[run] if lut_round_ch is not None else None
    t_r = t[run]
    ud_r = ud[run]
    uq_r = uq[run]
    idf_r = idf_full
    iqf_r = iqf_full

    all_ld: list[tuple[int, float, float, float]] = []
    all_lq: list[tuple[int, float, float, float]] = []
    by_round: dict[int, dict] = {}

    leg_masks = discover_leg_masks(open_seq_r, None, t_r, t0, lut_round_r)
    for leg_i, seg_mask in leg_masks:
        ld_pts, lq_pts = extract_points_from_mask(
            seg_mask,
            id_r,
            iq_r,
            ud_r,
            uq_r,
            idf_r,
            iqf_r,
            leg_i,
            method,
            rs_ohm,
            seg_tol=seg_tol,
            upsample=upsample,
        )
        all_ld.extend(ld_pts)
        all_lq.extend(lq_pts)
        ld_m = merge_grid_median(ld_pts)
        lq_m = merge_grid_median(lq_pts)
        rd: dict = {
            "ld_merged": ld_m,
            "lq_merged": lq_m,
            "n_ld_merged": len(ld_m),
            "n_lq_merged": len(lq_m),
        }
        ld_fit_r = fit_surface(ld_m)
        if ld_fit_r:
            rd["ld_coeff"] = ld_fit_r["coeff"]
            rd["ld_rms_uH"] = ld_fit_r["rms_uH"]
            rd["ld_at_1A_uH"] = ld_fit_r["at_1A0_uH"]
        lq_fit_r = fit_surface(lq_m)
        if lq_fit_r:
            rd["lq_coeff"] = lq_fit_r["coeff"]
            rd["lq_rms_uH"] = lq_fit_r["rms_uH"]
            rd["lq_at_1A_uH"] = lq_fit_r["at_1A1_uH"]
        by_round[leg_i] = rd

    ld_merged = merge_grid_median(all_ld)
    lq_merged = merge_grid_median(all_lq)

    end_seq = int(open_seq[-1])
    flow_done = end_seq in (58, 58 + VASI_SEQ_LUT_ROUND_OFFSET, 158)

    out: dict = {
        "file": str(path),
        "method": method,
        "grid_src": grid_src,
        "from_abc": from_abc,
        "half_ticks_coarse_ctrl": HALF_TICKS_COARSE_CTRL,
        "t0_l_ident": t0,
        "n_legs": len(leg_masks),
        "n_ld": len(all_ld),
        "n_lq": len(all_lq),
        "n_ld_merged": len(ld_merged),
        "n_lq_merged": len(lq_merged),
        "ld_pts": [(p[1], p[2], p[3]) for p in all_ld],
        "lq_pts": [(p[1], p[2], p[3]) for p in all_lq],
        "ld_merged": ld_merged,
        "lq_merged": lq_merged,
        "ld_by_leg": all_ld,
        "lq_by_leg": all_lq,
        "by_round": by_round,
        "open_seq_end": end_seq,
        "flow_done": flow_done,
        "mcu_ok": flow_done,
        "u_inj_cap_0.5A_coarse": u_inj_cap(0.5, F_COARSE_HZ),
    }

    ld_fit = fit_surface(ld_merged)
    if ld_fit:
        out["ld_coeff"] = ld_fit["coeff"]
        out["ld_rms_uH"] = ld_fit["rms_uH"]
        out["ld_at_1A_uH"] = ld_fit["at_1A0_uH"]

    lq_fit = fit_surface(lq_merged)
    if lq_fit:
        out["lq_coeff"] = lq_fit["coeff"]
        out["lq_rms_uH"] = lq_fit["rms_uH"]
        out["lq_at_1A_uH"] = lq_fit["at_1A1_uH"]

    return out


def print_mcu_compare(path: Path, off: dict) -> None:
    try:
        from parse_ident_vofa import find_header, load_csv, parse_burst
    except ImportError:
        return
    rows = load_csv(path)
    hi = find_header(rows)
    if hi < 0:
        return
    mcu = parse_burst(rows, hi)
    off_ld = {(a, b): v for a, b, v in off.get("ld_merged", [])}
    off_lq = {(a, b): v for a, b, v in off.get("lq_merged", [])}
    print(f"\n  --- MCU burst vs offline (Rs={mcu['rs_ohm']:.4f}) ---")
    dual = mcu.get("proto", 1.0) >= 1.95
    if dual:
        print(
            f"  {'grid':>10}  {'MCU Ld c/f':>14}  {'off Ld':>8}  {'d%':>6}  "
            f"{'MCU Lq c/f':>14}  {'off Lq':>8}  {'d%':>6}"
        )
    else:
        print(f"  {'grid':>10}  {'MCU Ld':>8}  {'off Ld':>8}  {'d%':>6}  {'MCU Lq':>8}  {'off Lq':>8}  {'d%':>6}")
    for g in mcu["grid"]:
        key = (g["id_bias"], g["iq_bias"])
        lo = off_ld.get(key, float("nan"))
        lqo = off_lq.get(key, float("nan"))
        if dual:
            ldc = g["ld_coarse_uH"] if g.get("ld_coarse_valid", g.get("ld_valid")) else float("nan")
            lqc = g["lq_coarse_uH"] if g.get("lq_coarse_valid", g.get("lq_valid")) else float("nan")
            ldf = g["ld_fine_uH"] if g.get("ld_fine_valid") else float("nan")
            lqf = g["lq_fine_uH"] if g.get("lq_fine_valid") else float("nan")
            dld = 100 * (ldf / lo - 1) if lo == lo and lo > 0 and ldf == ldf else float("nan")
            dlq = 100 * (lqf / lqo - 1) if lqo == lqo and lqo > 0 and lqf == lqf else float("nan")
            ld_s = f"{ldc:.0f}/{ldf:.0f}" if ldc == ldc and ldf == ldf else f"{ldc:.0f}/—"
            lq_s = f"{lqc:.0f}/{lqf:.0f}" if lqc == lqc and lqf == lqf else f"{lqc:.0f}/—"
            print(
                f"  ({g['id_bias']:+.1f},{g['iq_bias']:+.1f})"
                f"  {ld_s:>14}  {lo:8.1f}  {dld:+6.1f}"
                f"  {lq_s:>14}  {lqo:8.1f}  {dlq:+6.1f}"
            )
        else:
            dld = 100 * (g["ld_uH"] / lo - 1) if lo == lo and lo > 0 else float("nan")
            dlq = 100 * (g["lq_uH"] / lqo - 1) if lqo == lqo and lqo > 0 else float("nan")
            print(
                f"  ({g['id_bias']:+.1f},{g['iq_bias']:+.1f})"
                f"  {g['ld_uH']:8.1f}  {lo:8.1f}  {dld:+6.1f}"
                f"  {g['lq_uH']:8.1f}  {lqo:8.1f}  {dlq:+6.1f}"
            )


def main() -> None:
    ap = argparse.ArgumentParser(description="Offline VASI Ld/Lq surface RLS")
    ap.add_argument("csv", type=Path)
    ap.add_argument(
        "--method",
        choices=("psi", "hbm", "both"),
        default="psi",
        help="L extract: psi=Δψ/ΔI telem half-window (default), "
        "hbm=harmonic balance @2k, both=median(psi,hbm)",
    )
    ap.add_argument("--plot", action="store_true", help="save Ld/Lq 3D surface PNG")
    ap.add_argument("--rs", type=float, default=RS_OHM, help="Rs used in flux subtraction (ohm)")
    ap.add_argument(
        "--upsample-ctrl",
        action="store_true",
        help="upsample telem to 20 kHz before PSI (experimental)",
    )
    ap.add_argument(
        "--from-abc",
        action="store_true",
        help="recompute Id/Iq from Ia,Ib,Ic,theta (cross-check; default uses CSV Id/Iq)",
    )
    ap.add_argument(
        "--compare-mcu",
        action="store_true",
        help="print table vs MCU -777777 ident burst if present",
    )
    args = ap.parse_args()

    r = analyze_csv(
        args.csv,
        method=args.method,
        rs_ohm=args.rs,
        from_abc=args.from_abc,
        upsample=args.upsample_ctrl,
    )
    print(f"\n=== {Path(r.get('file', args.csv)).name} ===")
    if "error" in r:
        print(" ", r["error"])
        return

    print(
        f"  method={r['method']}  grid_src={r.get('grid_src', '?')}  "
        f"half_ticks_ctrl={r.get('half_ticks_coarse_ctrl')}  "
        f"L ident t0={r['t0_l_ident']:.2f}s  legs={r['n_legs']}"
    )
    print(
        f"  raw pts: Ld={r['n_ld']}  Lq={r['n_lq']}  "
        f"merged: Ld={r['n_ld_merged']}  Lq={r['n_lq_merged']}"
    )
    print(f"  open_seq_end={r['open_seq_end']}  flow_done={r['flow_done']}")
    print(f"  U_inj cap @0.5A 2kHz (model): {r.get('u_inj_cap_0.5A_coarse', 0):.3f} V")

    if r.get("by_round"):
        for round_i in sorted(r["by_round"].keys()):
            label = LUT_ROUND_NAMES.get(round_i, f"round{round_i}")
            rd = r["by_round"][round_i]
            print(
                f"\n  --- Ld/Lq round {label} (open_seq {vasi_progress_open_seq(round_i)}/"
                f"{58 + round_i * VASI_SEQ_LUT_ROUND_OFFSET}) "
                f"merged {rd.get('n_ld_merged', 0)}/{rd.get('n_lq_merged', 0)} ---"
            )
            if rd.get("ld_merged"):
                print("  Ld grid (uH):")
                for id0, iq0, l in rd["ld_merged"]:
                    print(f"    ({id0:.1f},{iq0:.1f}) {l:.1f} uH  [LCR ref {LD_NOMINAL_UH:.0f}]")
            if rd.get("lq_merged"):
                print("  Lq grid (uH):")
                for id0, iq0, l in rd["lq_merged"]:
                    print(f"    ({id0:.1f},{iq0:.1f}) {l:.1f} uH  [LCR ref {LQ_NOMINAL_UH:.0f}]")
            if "ld_at_1A_uH" in rd:
                print(
                    f"  Ld@(1,0)={rd['ld_at_1A_uH']:.1f} uH  "
                    f"rms={rd.get('ld_rms_uH', 0):.2f} uH"
                )
            if "lq_at_1A_uH" in rd:
                print(
                    f"  Lq@(1,1)={rd['lq_at_1A_uH']:.1f} uH  "
                    f"rms={rd.get('lq_rms_uH', 0):.2f} uH"
                )

    if r.get("ld_merged") and not r.get("by_round"):
        print("\n  Ld merged grid (uH):")
        for id0, iq0, l in r["ld_merged"]:
            print(f"    ({id0:.1f},{iq0:.1f}) {l:.1f} uH  [LCR ref {LD_NOMINAL_UH:.0f}]")

    if r.get("lq_merged") and not r.get("by_round"):
        print("\n  Lq merged grid (uH):")
        for id0, iq0, l in r["lq_merged"]:
            print(f"    ({id0:.1f},{iq0:.1f}) {l:.1f} uH  [LCR ref {LQ_NOMINAL_UH:.0f}]")

    if "ld_at_1A_uH" in r and not r.get("by_round"):
        print(f"\n  Ld surface @(Id=1,Iq=0)={r['ld_at_1A_uH']:.1f} uH  rms={r.get('ld_rms_uH', 0):.2f} uH")
        print_surface_table(np.array(r["ld_coeff"]), "Ld")

    if "lq_at_1A_uH" in r and not r.get("by_round"):
        print(f"  Lq surface @(Id=1,Iq=1)={r['lq_at_1A_uH']:.1f} uH  rms={r.get('lq_rms_uH', 0):.2f} uH")
        print_surface_table(np.array(r["lq_coeff"]), "Lq")

    if r.get("ld_by_leg"):
        print("\n  Ld by round/leg (ch10: 0=OFF 1=LUT):")
        for leg, id0, iq0, l in r["ld_by_leg"]:
            print(f"    leg{leg} ({id0:.1f},{iq0:.1f}) {l:.1f} uH")

    if args.plot and "ld_coeff" in r:
        lq_c = np.array(r["lq_coeff"]) if "lq_coeff" in r else None
        plot_surface(args.csv, np.array(r["ld_coeff"]), lq_c)

    if args.compare_mcu:
        print_mcu_compare(args.csv, r)


if __name__ == "__main__":
    main()
