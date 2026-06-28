#!/usr/bin/env python3
"""
从 Id 扫表 VOFA CSV 离线重建 d 表 / geo 样本池 / phase LUT merge。

通道：I0-I2=Ia,Ib,Ic | I3=Ud_pi | I4=Id_ref | I5=theta_el(rad)

流程（对齐 motor/deadband/deadband_geo.c + deadband_cal.c）：
  1. Pass0 按 theta 分 Pass0-A(30°) / Pass0-B(0°)
  2. dwell 末提取 (Id_k, Ud_res) → d 表（仅 A 角）
  3. geo_dlut_point → abc 样本池（双角）
  4. 多种 merge 策略 → 32 点 phase 表
  5. 与 CSV 末端固件 LUT 突发对比

用法:
  python tools/offline_deadband_geo_merge.py VOFA+CSV/20260627/vofa+202606272335.csv
  python tools/offline_deadband_geo_merge.py --batch 2304 2321 2335
  python tools/offline_deadband_geo_merge.py file.csv --md out.md
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from analyze_id_cal_sweep_vofa import (  # noqa: E402
    CAPTURE_EPS_A,
    D_TO_PHASE_COS,
    OUTLIER_V,
    RS_OHM,
    detect_id_steps,
    find_lut_header,
    load_csv,
    park,
    segment_steady_slice,
    unwrap,
)
from parse_lut_vofa import parse_lut  # noqa: E402

# --- 与 motor_params_m1.h / motor_current.c 一致 ---
THETA_A = math.pi / 6.0  # 30° Pass0-A
THETA_B = 0.0
I_ZERO_A = 0.05
LUT_LEN = 32
DEFAULT_FS = 20000.0
CAPTURE_EPS = CAPTURE_EPS_A
GEO_MERGE_U_MAX = 6

ID_CAL_AMP_TABLE = np.array(
    [
        0.05, 0.0667, 0.0833, 0.10, 0.1167, 0.1333, 0.15, 0.1667,
        0.1833, 0.20, 0.2167, 0.2333, 0.25, 0.2667, 0.2833, 0.30,
        0.35, 0.4091, 0.4682, 0.5273, 0.5864, 0.6455, 0.7045, 0.7636,
        0.8227, 0.8818, 0.9409, 1.00,
        1.10, 1.2333, 1.3667, 1.50,
    ],
    dtype=float,
)

BATCH_DEFAULT = [
    ROOT / "VOFA+CSV/20260627/vofa+202606272304.csv",
    ROOT / "VOFA+CSV/20260627/vofa+202606272321.csv",
    ROOT / "VOFA+CSV/20260627/vofa+202606272335.csv",
]

# 最近双角 Pass0 录波（含 proposed merge 验收 0126）
RECENT_BATCH = [
    ROOT / "VOFA+CSV/20260627/vofa+202606280126.csv",
    ROOT / "VOFA+CSV/20260627/vofa+202606272335.csv",
    ROOT / "VOFA+CSV/20260627/vofa+202606272321.csv",
    ROOT / "VOFA+CSV/20260627/vofa+202606272304.csv",
]

# 论文 §4.4 离线优先策略（batch 报告主表）
PAPER_STRATEGIES = (
    "paper428",
    "paper_per_angle_max",
    "paper_per_phase_max",
    "paper_per_angle_phase",
    "proposed",
    "geo30",
    "v2335",
)


@dataclass
class CapturePoint:
    id_ref: float
    id_fb: float
    ud_pi: float
    ud_res: float
    theta_el: float
    leg: str  # "A" | "B"
    t_mid: float


@dataclass
class GeoSample:
    i_abs: float
    u_abs: float
    id_capture: float
    theta_el: float
    phase: int
    leg: str


def anti_park(d: float, q: float, theta: float) -> tuple[float, float]:
    c, s = math.cos(theta), math.sin(theta)
    alpha = d * c - q * s
    beta = d * s + q * c
    return alpha, beta


def inv_clarke(alpha: float, beta: float) -> tuple[float, float, float]:
    ua = alpha
    ub = -0.5 * alpha + math.sqrt(3) / 2.0 * beta
    uc = -0.5 * alpha - math.sqrt(3) / 2.0 * beta
    return ua, ub, uc


def remove_u0(ua: float, ub: float, uc: float) -> tuple[float, float, float]:
    u0 = (ua + ub + uc) / 3.0
    return ua - u0, ub - u0, uc - u0


def geo_dlut_point(theta_el: float, id_a: float, ud_res: float) -> list[GeoSample]:
    """对齐 deadband_geo_dlut_point()。"""
    id_abs = abs(id_a)
    ud_abs = abs(ud_res)
    u_alpha, u_beta = anti_park(ud_abs, 0.0, theta_el)
    ua, ub, uc = inv_clarke(u_alpha, u_beta)
    ua, ub, uc = remove_u0(ua, ub, uc)

    i_alpha, i_beta = anti_park(id_abs, 0.0, theta_el)
    ia, ib, ic = inv_clarke(i_alpha, i_beta)

    leg = "A" if abs(theta_el - THETA_A) < abs(theta_el - THETA_B) else "B"
    out: list[GeoSample] = []
    for phase, (i_ph, u_ph) in enumerate(
        [(ia, ua), (ib, ub), (ic, uc)]
    ):
        i_abs = abs(i_ph)
        if i_abs < I_ZERO_A:
            continue
        out.append(
            GeoSample(
                i_abs=i_abs,
                u_abs=abs(u_ph),
                id_capture=id_abs,
                theta_el=theta_el,
                phase=phase,
                leg=leg,
            )
        )
    return out


def classify_leg(theta_med: float) -> str:
    da = abs(_wrap(theta_med - THETA_A))
    db = abs(_wrap(theta_med - THETA_B))
    return "A" if da <= db else "B"


def _wrap(x: float) -> float:
    while x > math.pi:
        x -= 2.0 * math.pi
    while x < -math.pi:
        x += 2.0 * math.pi
    return x


def anchor_index_for(id_ref_val: float, tol: float = 0.025) -> int:
    d = np.abs(ID_CAL_AMP_TABLE - id_ref_val)
    j = int(np.argmin(d))
    if d[j] <= tol:
        return j
    return -1


def split_contiguous(idxs: np.ndarray, gap_samples: int) -> list[np.ndarray]:
    if len(idxs) == 0:
        return []
    segs: list[list[int]] = [[int(idxs[0])]]
    for i in range(1, len(idxs)):
        if idxs[i] - idxs[i - 1] > gap_samples:
            segs.append([int(idxs[i])])
        else:
            segs[-1].append(int(idxs[i]))
    return [np.array(s, dtype=int) for s in segs if len(s) > 0]


def dwell_s_for_anchor(id_k: float) -> float:
    return 1.0 if id_k <= 0.40 else 0.5


def extract_pass0_captures(
    ia: np.ndarray,
    ib: np.ndarray,
    ic: np.ndarray,
    ud_pi: np.ndarray,
    id_ref: np.ndarray,
    theta_raw: np.ndarray,
    fs: float,
    sweep_end: int,
) -> list[CapturePoint]:
    """
    32 点锚表对齐 + dwell 末 20% 单拍 capture（|Id−Id_ref|≤0.03 A）。
    对齐 motor_current.c：末 1/5 dwell 置 capture_pending，ISR 满足门限即 capture。
    """
    n = sweep_end
    theta_u = unwrap(theta_raw[:n])
    id_fb_all, _ = park(ia[:n], ib[:n], ic[:n], theta_u)
    t = np.arange(n) / fs
    gap_samples = int(0.15 * fs)
    min_dwell_samples = int(0.08 * fs)

    legs = np.array([classify_leg(float(theta_u[i])) for i in range(n)])
    anchor_idx = np.full(n, -1, dtype=int)
    for i in range(n):
        anchor_idx[i] = anchor_index_for(float(id_ref[i]))

    captures: list[CapturePoint] = []

    for leg, theta_lock in (("A", THETA_A), ("B", THETA_B)):
        for k, id_k in enumerate(ID_CAL_AMP_TABLE):
            mask = (legs == leg) & (anchor_idx == k)
            idxs = np.where(mask)[0]
            if len(idxs) == 0:
                continue

            segs = split_contiguous(idxs, gap_samples)
            best_seg = None
            for seg in segs:
                if len(seg) >= min_dwell_samples:
                    best_seg = seg
                    break
            if best_seg is None:
                best_seg = segs[0]

            tail_n = max(int(len(best_seg) * 0.20), 1)
            tail = best_seg[-tail_n:]

            cap_idx: int | None = None
            for i in tail:
                if abs(float(id_fb_all[i]) - float(id_ref[i])) <= CAPTURE_EPS:
                    cap_idx = int(i)
                    break
            if cap_idx is None:
                err = np.abs(id_fb_all[tail] - id_ref[tail])
                cap_idx = int(tail[int(np.argmin(err))])

            id_mean = float(id_fb_all[cap_idx])
            ud_val = float(ud_pi[cap_idx])
            ud_res = abs(ud_val - id_mean * RS_OHM)

            captures.append(
                CapturePoint(
                    id_ref=float(id_k),
                    id_fb=id_mean,
                    ud_pi=ud_val,
                    ud_res=ud_res,
                    theta_el=theta_lock,
                    leg=leg,
                    t_mid=float(t[cap_idx]),
                )
            )

    return sorted(captures, key=lambda x: (x.leg, x.id_ref))


def build_dlut(captures: list[CapturePoint]) -> tuple[np.ndarray, np.ndarray]:
    """Pass0-A → s_dlut，按固件 amp 表 32 档对齐。"""
    leg_a = {c.id_ref: c for c in captures if c.leg == "A"}
    amps = ID_CAL_AMP_TABLE.copy()
    vals = np.zeros(LUT_LEN, dtype=float)
    for i, id_k in enumerate(amps):
        c = leg_a.get(float(id_k))
        if c is not None:
            vals[i] = c.ud_res
        elif i > 0:
            vals[i] = vals[i - 1]
    return amps, vals


def build_geo_pool(captures: list[CapturePoint]) -> list[GeoSample]:
    pool: list[GeoSample] = []
    for c in captures:
        # 与固件一致：geo 用 capture 单拍 Id（signed）与 Ud_res
        pool.extend(geo_dlut_point(c.theta_el, c.id_fb, c.ud_res))
    return pool


def median_u(vals: list[float]) -> float:
    if not vals:
        return 0.0
    s = sorted(vals)
    return s[len(s) // 2]


def sort_plut_pairs(amps: np.ndarray, vals: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    order = np.argsort(amps)
    return amps[order].copy(), vals[order].copy()


def enforce_val_monotone(vals: np.ndarray) -> np.ndarray:
    out = vals.copy()
    for i in range(1, len(out)):
        if out[i] < out[i - 1]:
            out[i] = out[i - 1]
    return out


def geo_fallback_30(id_k: float, ud_k: float, outlier_v: float | None) -> tuple[float, float]:
    pts = geo_dlut_point(THETA_A, id_k, ud_k)
    if not pts:
        return 0.0, 0.0
    best = max(pts, key=lambda p: p.i_abs)
    u_pool = [p.u_abs for p in pts if outlier_v is None or p.u_abs <= outlier_v]
    u = median_u(u_pool) if u_pool else best.u_abs
    return best.i_abs, u


def merge_id_bin(
    samples: list[GeoSample],
    id_k: float,
    *,
    amp_theta: str | None,  # None=dual, "A"=30° only
    val_theta: str | None,  # None=dual
    val_mode: str,  # "median" | "max_pair"
    outlier_v: float | None,
) -> tuple[float, float] | None:
    hit: list[GeoSample] = []
    for s in samples:
        if abs(s.id_capture - id_k) > CAPTURE_EPS:
            continue
        hit.append(s)
    if not hit:
        return None

    amp_pool = hit if amp_theta is None else [s for s in hit if s.leg == amp_theta]
    if not amp_pool:
        amp_pool = [s for s in hit if s.leg == "A"]
    if not amp_pool:
        return None

    if val_mode == "max_pair":
        best = max(hit, key=lambda s: s.i_abs)
        return best.i_abs, best.u_abs

    amp_max = max(s.i_abs for s in amp_pool)

    val_pool = hit if val_theta is None else [s for s in hit if s.leg == val_theta]
    u_list = [
        s.u_abs
        for s in val_pool
        if outlier_v is None or s.u_abs <= outlier_v
    ]
    if not u_list:
        u_list = [s.u_abs for s in val_pool]
    if not u_list:
        return None
    return amp_max, median_u(u_list)


def apply_zcz_filter(
    samples: list[GeoSample],
    *,
    i_min: float = I_ZERO_A,
    u_max: float | None = None,
) -> list[GeoSample]:
    """§4.5 简化：剔除 |i|<i_min 或 |u'|>u_max 的 geo 点（建表前）。"""
    out: list[GeoSample] = []
    for s in samples:
        if s.i_abs < i_min:
            continue
        if u_max is not None and s.u_abs > u_max:
            continue
        out.append(s)
    return out


def merge_paper428_max_pair(
    samples: list[GeoSample], id_k: float
) -> tuple[float, float] | None:
    """论文 §3.2(2)：Id_k 档内全池取 |i| 最大的一条，(i,u') 同点绑定。"""
    hit = _samples_in_bin(samples, id_k)
    if not hit:
        return None
    best = max(hit, key=lambda s: s.i_abs)
    return best.i_abs, best.u_abs


def merge_paper_per_angle_max(
    samples: list[GeoSample], id_k: float
) -> tuple[float, float] | None:
    """论文 §2.3：每角（30°/0°）各取 max|i| 相一条，amp=max(i)，val=median(u')。"""
    hit = _samples_in_bin(samples, id_k)
    if not hit:
        return None
    bests: list[GeoSample] = []
    for leg in ("A", "B"):
        leg_hit = [s for s in hit if s.leg == leg]
        if leg_hit:
            bests.append(max(leg_hit, key=lambda s: s.i_abs))
    if not bests:
        return None
    amp = max(s.i_abs for s in bests)
    val = median_u([s.u_abs for s in bests])
    return amp, val


def merge_paper_per_phase_max(
    samples: list[GeoSample], id_k: float
) -> tuple[float, float] | None:
    """论文 §2.3 首版：每相 max|i| 一条（双角合并），val=三相 median(u')，amp=30°max|i|。"""
    hit = _samples_in_bin(samples, id_k)
    if not hit:
        return None
    u_list: list[float] = []
    amp_max = 0.0
    for ph in (0, 1, 2):
        ph_hit = [s for s in hit if s.phase == ph]
        if not ph_hit:
            continue
        best = max(ph_hit, key=lambda s: s.i_abs)
        u_list.append(best.u_abs)
        leg_a = [s for s in ph_hit if s.leg == "A"]
        if leg_a:
            amp_max = max(amp_max, max(s.i_abs for s in leg_a))
    if not u_list or amp_max <= 0.0:
        return None
    return amp_max, median_u(u_list)


def merge_paper_per_angle_phase(
    samples: list[GeoSample], id_k: float
) -> tuple[float, float] | None:
    """论文完整分相：每角×每相 max|i|（≤6 条），val=median(u')，amp=30°各相 max|i| 的最大。"""
    hit = _samples_in_bin(samples, id_k)
    if not hit:
        return None
    u_list: list[float] = []
    amp_max = 0.0
    for leg in ("A", "B"):
        for ph in (0, 1, 2):
            sub = [s for s in hit if s.leg == leg and s.phase == ph]
            if not sub:
                continue
            best = max(sub, key=lambda s: s.i_abs)
            u_list.append(best.u_abs)
            if leg == "A":
                amp_max = max(amp_max, best.i_abs)
    if not u_list or amp_max <= 0.0:
        return None
    return amp_max, median_u(u_list)


MERGE_FN_MAP: dict[str, object] = {
    "paper428": merge_paper428_max_pair,
    "paper_per_angle_max": merge_paper_per_angle_max,
    "paper_per_phase_max": merge_paper_per_phase_max,
    "paper_per_angle_phase": merge_paper_per_angle_phase,
}


def build_plut_fn(
    samples: list[GeoSample],
    id_anchor: np.ndarray,
    ud_anchor: np.ndarray,
    merge_fn,
    *,
    sort_monotone: bool = True,
) -> tuple[np.ndarray, np.ndarray]:
    n = len(id_anchor)
    amps = np.zeros(n, dtype=float)
    vals = np.zeros(n, dtype=float)
    for k in range(n):
        r = merge_fn(samples, float(id_anchor[k]))
        if r is None:
            fb = geo_fallback_30(float(id_anchor[k]), float(ud_anchor[k]), None)
            amps[k], vals[k] = fb
        else:
            amps[k], vals[k] = r
    if sort_monotone:
        amps, vals = sort_plut_pairs(amps, vals)
        vals = enforce_val_monotone(vals)
    return amps, vals


def build_plut(
    samples: list[GeoSample],
    id_anchor: np.ndarray,
    ud_anchor: np.ndarray,
    *,
    amp_theta: str | None,
    val_theta: str | None,
    val_mode: str,
    outlier_v: float | None,
    sort_monotone: bool = True,
) -> tuple[np.ndarray, np.ndarray]:
    n = len(id_anchor)
    amps = np.zeros(n, dtype=float)
    vals = np.zeros(n, dtype=float)
    for k in range(n):
        r = merge_id_bin(
            samples,
            float(id_anchor[k]),
            amp_theta=amp_theta,
            val_theta=val_theta,
            val_mode=val_mode,
            outlier_v=outlier_v,
        )
        if r is None:
            fb = geo_fallback_30(float(id_anchor[k]), float(ud_anchor[k]), outlier_v)
            amps[k], vals[k] = fb
        else:
            amps[k], vals[k] = r
    if sort_monotone:
        amps, vals = sort_plut_pairs(amps, vals)
        vals = enforce_val_monotone(vals)
    return amps, vals


def build_plut_from_dlut_30(dlut_amps: np.ndarray, dlut_vals: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    amps = np.zeros_like(dlut_amps)
    vals = np.zeros_like(dlut_vals)
    for i, (ida, udv) in enumerate(zip(dlut_amps, dlut_vals)):
        pts = geo_dlut_point(THETA_A, float(ida), float(udv))
        if not pts:
            amps[i] = float(ida) * D_TO_PHASE_COS
            vals[i] = float(udv) * D_TO_PHASE_COS
            continue
        best = max(pts, key=lambda p: p.i_abs)
        amps[i] = best.i_abs
        vals[i] = best.u_abs
    return amps, vals


def build_scale_0866(dlut_amps: np.ndarray, dlut_vals: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    return dlut_amps * D_TO_PHASE_COS, dlut_vals * D_TO_PHASE_COS


def _samples_in_bin(samples: list[GeoSample], id_k: float) -> list[GeoSample]:
    return [s for s in samples if abs(s.id_capture - id_k) <= CAPTURE_EPS]


def analyze_per_bin(
    samples: list[GeoSample],
    id_anchor: np.ndarray,
    ud_anchor: np.ndarray,
) -> list[dict]:
    """每 Id_k 档：0°/30° 样本数、各自 median u'、dual merge 结果（排序/单调化前）。"""
    rows: list[dict] = []
    for k, id_k in enumerate(id_anchor):
        hit = _samples_in_bin(samples, float(id_k))
        leg_a = [s for s in hit if s.leg == "A"]
        leg_b = [s for s in hit if s.leg == "B"]
        u_a = [s.u_abs for s in leg_a]
        u_b = [s.u_abs for s in leg_b]
        u_all = [s.u_abs for s in hit]

        r_prop = merge_id_bin(
            samples,
            float(id_k),
            amp_theta="A",
            val_theta=None,
            val_mode="median",
            outlier_v=None,
        )
        r_v30 = merge_id_bin(
            samples,
            float(id_k),
            amp_theta="A",
            val_theta="A",
            val_mode="median",
            outlier_v=None,
        )
        r_v0 = merge_id_bin(
            samples,
            float(id_k),
            amp_theta="A",
            val_theta="B",
            val_mode="median",
            outlier_v=None,
        )
        pts30 = geo_dlut_point(THETA_A, float(id_k), float(ud_anchor[k]))
        geo30_val = max(pts30, key=lambda p: p.i_abs).u_abs if pts30 else float("nan")

        rows.append(
            {
                "bin": k + 1,
                "id_k": float(id_k),
                "n_a": len(leg_a),
                "n_b": len(leg_b),
                "n_total": len(hit),
                "median_u_a": median_u(u_a) if u_a else float("nan"),
                "median_u_b": median_u(u_b) if u_b else float("nan"),
                "median_u_dual": median_u(u_all) if u_all else float("nan"),
                "amp_proposed": r_prop[0] if r_prop else float("nan"),
                "val_proposed": r_prop[1] if r_prop else float("nan"),
                "val_30only": r_v30[1] if r_v30 else float("nan"),
                "val_0only": r_v0[1] if r_v0 else float("nan"),
                "val_geo30": geo30_val,
                "delta_dual_vs_geo30": (
                    (r_prop[1] - geo30_val) if r_prop else float("nan")
                ),
                "delta_dual_vs_30only": (
                    (r_prop[1] - r_v30[1]) if (r_prop and r_v30) else float("nan")
                ),
            }
        )
    return rows


def compare_tables_by_index(
    amps_a: np.ndarray,
    vals_a: np.ndarray,
    amps_b: np.ndarray,
    vals_b: np.ndarray,
) -> dict:
    n = min(len(amps_a), len(amps_b))
    d_val = np.abs(vals_a[:n] - vals_b[:n])
    d_amp = np.abs(amps_a[:n] - amps_b[:n])
    return {
        "n": n,
        "max_d_val": float(np.max(d_val)) if n else 0.0,
        "mean_d_val": float(np.mean(d_val)) if n else 0.0,
        "max_d_amp": float(np.max(d_amp)) if n else 0.0,
        "n_val_diff": int(np.sum(d_val > 1e-4)),
        "n_amp_diff": int(np.sum(d_amp > 1e-4)),
    }


def export_lut_csv(path: Path, amps: np.ndarray, vals: np.ndarray, strategy: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = ["# strategy=" + strategy, "amp_A,val_V"]
    for a, v in zip(amps, vals):
        lines.append(f"{a:.6f},{v:.6f}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def merge_global_resample(samples: list[GeoSample], n: int = LUT_LEN) -> tuple[np.ndarray, np.ndarray]:
    """2304 风格：全池按 |i| 排序均匀重采样（示意性复现）。"""
    if not samples:
        return np.zeros(n), np.zeros(n)
    s = sorted(samples, key=lambda x: x.i_abs)
    i_min, i_max = s[0].i_abs, s[-1].i_abs
    targets = np.linspace(i_min, i_max, n)

    def interp_u(i_tgt: float) -> float:
        if i_tgt <= s[0].i_abs:
            return s[0].u_abs
        if i_tgt >= s[-1].i_abs:
            return s[-1].u_abs
        for j in range(len(s) - 1):
            i0, i1 = s[j].i_abs, s[j + 1].i_abs
            if i_tgt <= i1:
                t = (i_tgt - i0) / (i1 - i0) if i1 > i0 else 0.0
                return s[j].u_abs + t * (s[j + 1].u_abs - s[j].u_abs)
        return s[-1].u_abs

    vals = np.array([interp_u(t) for t in targets])
    return targets, vals


MERGE_VARIANTS: dict[str, dict] = {
    "geo30": {
        "desc": "30° geo 单角（应≈Foundation，金标准参照）",
        "fn": "geo30",
    },
    "scale0866": {
        "desc": "d 表 ×0.866（旧 Foundation）",
        "fn": "scale0866",
    },
    "v2321": {
        "desc": "2321：双角 max|i| 同点绑定 val",
        "amp_theta": None,
        "val_mode": "max_pair",
        "outlier_v": None,
    },
    "v2335": {
        "desc": "2335 固件：双角 max|i| + median + outlier≤1.0V",
        "amp_theta": None,
        "val_theta": None,
        "val_mode": "median",
        "outlier_v": OUTLIER_V,
    },
    "proposed": {
        "desc": "建议：30° 定 amp + 双角 median val（无 outlier 过滤）",
        "amp_theta": "A",
        "val_theta": None,
        "val_mode": "median",
        "outlier_v": None,
    },
    "proposed_median_no_outlier": {
        "desc": "2335 规则但去掉 outlier 过滤",
        "amp_theta": None,
        "val_theta": None,
        "val_mode": "median",
        "outlier_v": None,
    },
    "val30_only": {
        "desc": "30° 定 amp + 仅 30° median val",
        "amp_theta": "A",
        "val_theta": "A",
        "val_mode": "median",
        "outlier_v": None,
    },
    "val0_only": {
        "desc": "30° 定 amp + 仅 0° median val",
        "amp_theta": "A",
        "val_theta": "B",
        "val_mode": "median",
        "outlier_v": None,
    },
    "v2304_global": {
        "desc": "2304：全池 |i| 均匀重采样",
        "fn": "global",
    },
    "paper428": {
        "desc": "论文§3.2：档内全局 max|i| 同点绑定 (|i|,|u'|)",
        "fn": "paper428",
    },
    "paper_per_angle_max": {
        "desc": "论文§2.3：每角 max|i| 相 → median(u')，amp=max(i)",
        "fn": "paper_per_angle_max",
    },
    "paper_per_phase_max": {
        "desc": "论文§2.3：每相 max|i|（双角）→ median(u')，amp=30°max|i|",
        "fn": "paper_per_phase_max",
    },
    "paper_per_angle_phase": {
        "desc": "论文完整：每角×每相 max|i| → median(u')，amp=30°max",
        "fn": "paper_per_angle_phase",
    },
    "proposed_zcz": {
        "desc": "proposed + ZCZ 剔 |i|<0.05 A",
        "fn": "proposed_zcz",
    },
}


def lut_summary(amps: np.ndarray, vals: np.ndarray) -> dict:
    mono = bool(np.all(vals[1:] >= vals[:-1] - 1e-9))
    amp_mono = bool(np.all(amps[1:] >= amps[:-1] - 1e-9))
    from collections import Counter

    rounded = [round(float(v), 4) for v in vals]
    top = Counter(rounded).most_common(1)[0] if vals.size else (0, 0)
    return {
        "sum_vals": float(np.sum(vals)),
        "val_end": float(vals[-1]) if len(vals) else 0.0,
        "amp_end": float(amps[-1]) if len(amps) else 0.0,
        "val_mono": mono,
        "amp_mono": amp_mono,
        "plateau_val": top[0],
        "plateau_count": top[1],
    }


def diff_lut(
    a_amps: np.ndarray, a_vals: np.ndarray, b_amps: np.ndarray, b_vals: np.ndarray
) -> dict:
    """在共有 amp 范围比较 val（线性插值 b 到 a 的 amp 网格）。"""

    def interp(amps: np.ndarray, vals: np.ndarray, x: float) -> float:
        if x <= amps[0]:
            return float(vals[0])
        if x >= amps[-1]:
            return float(vals[-1])
        for i in range(len(amps) - 1):
            if x <= amps[i + 1]:
                t = (x - amps[i + 1 - 1]) / (amps[i + 1] - amps[i])
                return float(vals[i] + t * (vals[i + 1] - vals[i]))
        return float(vals[-1])

    n = min(len(a_amps), len(b_amps))
    dv = []
    for i in range(n):
        v_b = interp(b_amps, b_vals, float(a_amps[i]))
        dv.append(abs(float(a_vals[i]) - v_b))
    return {
        "max_val_diff": max(dv) if dv else 0.0,
        "mean_val_diff": float(np.mean(dv)) if dv else 0.0,
    }


def analyze_csv(path: Path, fs: float = DEFAULT_FS) -> dict:
    ia, ib, ic, ud_pi, id_ref, theta_raw = load_csv(path)
    rows = np.column_stack([ia, ib, ic, ud_pi, id_ref, theta_raw])
    lut_hi = find_lut_header(rows)
    sweep_end = lut_hi if lut_hi >= 0 else len(ia)

    captures = extract_pass0_captures(
        ia, ib, ic, ud_pi, id_ref, theta_raw, fs, sweep_end
    )
    dlut_amps, dlut_vals = build_dlut(captures)
    geo_pool = build_geo_pool(captures)

    id_anchor = ID_CAL_AMP_TABLE.copy()
    ud_anchor = dlut_vals if len(dlut_vals) == LUT_LEN else np.zeros(LUT_LEN)

    fw_lut = None
    if lut_hi >= 0:
        try:
            fw_lut = parse_lut(rows)
        except ValueError:
            fw_lut = None

    results: dict[str, dict] = {}
    for name, spec in MERGE_VARIANTS.items():
        fn = spec.get("fn")
        pool = geo_pool
        if fn == "geo30":
            amps, vals = build_plut_from_dlut_30(id_anchor, ud_anchor)
        elif fn == "scale0866":
            amps, vals = build_scale_0866(id_anchor, ud_anchor)
        elif fn == "global":
            amps, vals = merge_global_resample(geo_pool, LUT_LEN)
        elif fn == "proposed_zcz":
            pool = apply_zcz_filter(geo_pool, i_min=I_ZERO_A)
            amps, vals = build_plut(
                pool,
                id_anchor,
                ud_anchor,
                amp_theta="A",
                val_theta=None,
                val_mode="median",
                outlier_v=None,
            )
        elif fn in MERGE_FN_MAP:
            amps, vals = build_plut_fn(
                geo_pool, id_anchor, ud_anchor, MERGE_FN_MAP[fn]
            )
        else:
            amps, vals = build_plut(
                pool,
                id_anchor,
                ud_anchor,
                amp_theta=spec.get("amp_theta"),
                val_theta=spec.get("val_theta"),
                val_mode=spec["val_mode"],
                outlier_v=spec.get("outlier_v"),
            )
        entry = {
            "desc": spec["desc"],
            "amps": amps,
            "vals": vals,
            "summary": lut_summary(amps, vals),
        }
        if fw_lut is not None:
            entry["vs_firmware"] = diff_lut(
                amps, vals, fw_lut["amps"], fw_lut["vals"]
            )
        results[name] = entry

    leg_a_n = sum(1 for c in captures if c.leg == "A")
    leg_b_n = sum(1 for c in captures if c.leg == "B")
    cap_expected = LUT_LEN * 2
    geo_expected_max = LUT_LEN * 3 * 2

    bin_stats = analyze_per_bin(geo_pool, id_anchor, ud_anchor)

    # anchor 序（排序前）全表对比
    prop_pre_a, prop_pre_v = build_plut(
        geo_pool, id_anchor, ud_anchor,
        amp_theta="A", val_theta=None, val_mode="median", outlier_v=None,
        sort_monotone=False,
    )
    geo30_pre_a, geo30_pre_v = build_plut_from_dlut_30(id_anchor, ud_anchor)
    table_cmp = {
        "proposed_vs_geo30_pre_sort": compare_tables_by_index(
            prop_pre_a, prop_pre_v, geo30_pre_a, geo30_pre_v
        ),
    }
    if "proposed" in results and "geo30" in results:
        table_cmp["proposed_vs_geo30_post_sort"] = compare_tables_by_index(
            results["proposed"]["amps"],
            results["proposed"]["vals"],
            results["geo30"]["amps"],
            results["geo30"]["vals"],
        )

    return {
        "path": path,
        "sweep_end": sweep_end,
        "captures": captures,
        "dlut_amps": dlut_amps,
        "dlut_vals": dlut_vals,
        "geo_pool": geo_pool,
        "id_anchor": id_anchor,
        "fw_lut": fw_lut,
        "results": results,
        "leg_a_segments": leg_a_n,
        "leg_b_segments": leg_b_n,
        "cap_expected": cap_expected,
        "geo_expected_max": geo_expected_max,
        "bin_stats": bin_stats,
        "table_cmp": table_cmp,
        "plut_pre_sort": {
            "proposed": (prop_pre_a, prop_pre_v),
            "geo30": (geo30_pre_a, geo30_pre_v),
        },
    }


def format_report(analysis: dict) -> str:
    p = analysis["path"]
    lines = [
        f"# 离线 geo merge — `{p.name}`",
        "",
        "## 输入重建",
        "",
        f"| 项 | 值 |",
        f"|----|-----|",
        f"| Pass0-A 档数 | {analysis['leg_a_segments']} / {LUT_LEN} |",
        f"| Pass0-B 档数 | {analysis['leg_b_segments']} / {LUT_LEN} |",
        f"| capture 点数 | {analysis['leg_a_segments'] + analysis['leg_b_segments']} / {analysis.get('cap_expected', LUT_LEN * 2)} |",
        f"| geo 样本池 | {len(analysis['geo_pool'])} 条 / ≤{analysis.get('geo_expected_max', LUT_LEN * 6)} |",
        f"| d 表 (Pass0-A) | {len(analysis['dlut_amps'])} 点 |",
    ]
    if analysis["dlut_amps"].size:
        lines.append(
            f"| d 表末档 Ud_res | {analysis['dlut_vals'][-1]:.4f} V @ Id={analysis['dlut_amps'][-1]:.3f} A |"
        )
    if analysis["fw_lut"]:
        fw = analysis["fw_lut"]
        s = lut_summary(fw["amps"], fw["vals"])
        lines += [
            "",
            "## CSV 固件 LUT（commit 产物）",
            "",
            f"- sum(vals)={s['sum_vals']:.4f}, 末档 val={s['val_end']:.4f} V/相, "
            f"amp={s['amp_end']:.4f} A",
            f"- val 平台：{s['plateau_val']} V × {s['plateau_count']} 点",
        ]

    lines += ["", "## 论文 §4.4 merge 策略（离线优先）", ""]
    lines.append(
        "| 策略 | 末档 val (V) | sum(vals) | 平台点数 | vs 固件 maxΔval | 说明 |"
    )
    lines.append("|------|-------------|-----------|----------|-----------------|------|")

    paper_first = [n for n in PAPER_STRATEGIES if n in analysis["results"]]
    rest = [n for n in analysis["results"] if n not in paper_first]
    for name in paper_first + rest:
        r = analysis["results"][name]
        s = r["summary"]
        vs = r.get("vs_firmware", {})
        max_d = vs.get("max_val_diff", float("nan"))
        lines.append(
            f"| **{name}** | {s['val_end']:.4f} | {s['sum_vals']:.2f} | "
            f"{s['plateau_count']} | {max_d:.4f} | {r['desc']} |"
        )

    lines += ["", "## 末档 5 点明细（论文策略 vs 固件）", ""]
    for key in (
        "paper428",
        "paper_per_angle_max",
        "paper_per_phase_max",
        "proposed",
        "v2335",
        "geo30",
    ):
        if key not in analysis["results"]:
            continue
        r = analysis["results"][key]
        amps, vals = r["amps"], r["vals"]
        lines.append(f"### {key}")
        lines.append("")
        lines.append("| # | amp | val |")
        lines.append("|---|-----|-----|")
        for i in range(max(0, len(amps) - 5), len(amps)):
            lines.append(f"| {i + 1} | {amps[i]:.4f} | {vals[i]:.4f} |")
        lines.append("")

    if analysis["fw_lut"] is not None and "v2335" in analysis["results"]:
        d = diff_lut(
            analysis["results"]["v2335"]["amps"],
            analysis["results"]["v2335"]["vals"],
            analysis["fw_lut"]["amps"],
            analysis["fw_lut"]["vals"],
        )
        lines += [
            "## 复现检查",
            "",
            f"- **v2335 离线 vs CSV 固件 LUT**：max|Δval|={d['max_val_diff']:.4f} V "
            f"（<0.05 V 表示离线复现固件 merge）",
        ]

    tc = analysis.get("table_cmp", {})
    if tc:
        lines += ["", "## proposed vs geo30 全表对比", ""]
        for key, c in tc.items():
            lines.append(
                f"- **{key}**：max|Δval|={c['max_d_val']:.6f} V，"
                f"mean|Δval|={c['mean_d_val']:.6f} V，"
                f"val 不同档数={c['n_val_diff']}/{c['n']}，"
                f"max|Δamp|={c['max_d_amp']:.6f} A"
            )

    bins = analysis.get("bin_stats", [])
    if bins:
        n_with_b = sum(1 for b in bins if b["n_b"] > 0)
        n_dual_diff = sum(
            1 for b in bins if abs(b.get("delta_dual_vs_30only", 0.0)) > 1e-4
        )
        n_0_affects = sum(
            1
            for b in bins
            if b["n_b"] > 0
            and abs(b["val_0only"] - b["val_30only"]) > 1e-4
            and not (math.isnan(b["val_0only"]) or math.isnan(b["val_30only"]))
        )
        lines += [
            "",
            "## 每档 0° / 30° 样本与 val（排序/单调化前）",
            "",
            f"- 有 0° 样本的档数：**{n_with_b}/{len(bins)}**",
            f"- dual vs 30°only val 不同档数：**{n_dual_diff}/{len(bins)}**",
            f"- 0°only vs 30°only val 不同档数：**{n_0_affects}/{len(bins)}**（0° 校正潜力）",
            "",
            "| # | Id_k | n30 | n0 | med30 | med0 | val30 | val0 | val_dual | val_geo30 | Δdual−geo30 |",
            "|---|------|-----|-----|-------|------|-------|------|----------|-----------|-------------|",
        ]
        for b in bins:
            def _f(x: float) -> str:
                return f"{x:.4f}" if not math.isnan(x) else "—"

            lines.append(
                f"| {b['bin']} | {b['id_k']:.4f} | {b['n_a']} | {b['n_b']} | "
                f"{_f(b['median_u_a'])} | {_f(b['median_u_b'])} | "
                f"{_f(b['val_30only'])} | {_f(b['val_0only'])} | "
                f"{_f(b['val_proposed'])} | {_f(b['val_geo30'])} | "
                f"{_f(b['delta_dual_vs_geo30'])} |"
            )

    lines += ["", "## 完整 32 点 proposed（anchor 序，pre-sort）", ""]
    prop_a, prop_v = analysis.get("plut_pre_sort", {}).get("proposed", (None, None))
    if prop_a is not None:
        lines.append("| # | Id_k | amp | val |")
        lines.append("|---|------|-----|-----|")
        for i, id_k in enumerate(analysis["id_anchor"]):
            lines.append(
                f"| {i + 1} | {id_k:.4f} | {prop_a[i]:.4f} | {prop_v[i]:.4f} |"
            )

    # 论文策略互相对照
    if "paper428" in analysis["results"] and "proposed" in analysis["results"]:
        d = diff_lut(
            analysis["results"]["proposed"]["amps"],
            analysis["results"]["proposed"]["vals"],
            analysis["results"]["paper428"]["amps"],
            analysis["results"]["paper428"]["vals"],
        )
        lines += [
            "",
            "## 论文策略对照",
            "",
            f"- **proposed vs paper428（§3.2 max|i| 绑定）**：max|Δval|={d['max_val_diff']:.4f} V",
        ]
        for key in ("paper_per_angle_max", "paper_per_phase_max", "paper_per_angle_phase"):
            if key not in analysis["results"]:
                continue
            d2 = diff_lut(
                analysis["results"]["proposed"]["amps"],
                analysis["results"]["proposed"]["vals"],
                analysis["results"][key]["amps"],
                analysis["results"][key]["vals"],
            )
            lines.append(
                f"- **proposed vs {key}**：max|Δval|={d2['max_val_diff']:.4f} V"
            )

    return "\n".join(lines)


def format_batch_report(analyses: list[dict]) -> str:
    """多 CSV 论文 merge 横向对比。"""
    lines = [
        "# 离线论文 §4.4 merge — 批量对比",
        "",
        f"CSV 数量：**{len(analyses)}**",
        "",
        "## 汇总：论文策略末档 val (V/相)",
        "",
        "| CSV | geo池 | 固机末档 | "
        + " | ".join(PAPER_STRATEGIES)
        + " |",
        "|-----|-------|---------|"
        + "|".join(["------"] * len(PAPER_STRATEGIES))
        + "|",
    ]
    for a in analyses:
        fw_end = "—"
        if a["fw_lut"] is not None:
            fw_end = f"{lut_summary(a['fw_lut']['amps'], a['fw_lut']['vals'])['val_end']:.4f}"
        cols = []
        for strat in PAPER_STRATEGIES:
            r = a["results"].get(strat)
            if r is None:
                cols.append("—")
            else:
                cols.append(f"{r['summary']['val_end']:.4f}")
        lines.append(
            f"| `{a['path'].name}` | {len(a['geo_pool'])} | {fw_end} | "
            + " | ".join(cols)
            + " |"
        )

    lines += [
        "",
        "## 汇总：vs 固机 LUT max|Δval| (V)",
        "",
        "| CSV | "
        + " | ".join(PAPER_STRATEGIES)
        + " |",
        "|-----|"
        + "|".join(["------"] * len(PAPER_STRATEGIES))
        + "|",
    ]
    for a in analyses:
        cols = []
        for strat in PAPER_STRATEGIES:
            r = a["results"].get(strat)
            if r is None or a["fw_lut"] is None:
                cols.append("—")
            else:
                vs = r.get("vs_firmware", {})
                cols.append(f"{vs.get('max_val_diff', float('nan')):.4f}")
        lines.append(f"| `{a['path'].name}` | " + " | ".join(cols) + " |")

    lines += [
        "",
        "## 读法",
        "",
        "- **paper428**：论文 §3.2 档内全局 max|i| 同点绑定（≈旧 v2321 规则）。",
        "- **paper_per_angle_max**：每角各取 max|i| 相，再 median(u')。",
        "- **paper_per_phase_max / paper_per_angle_phase**：分相 max|i| 后再 median(u')。",
        "- **proposed**：固机当前 proposed（30°定 amp + 双角 median val）。",
        "- **vs 固机 <0.05 V**：离线复现该 CSV commit 时的 merge。",
        "",
        "## 各 CSV 明细",
        "",
    ]
    for a in analyses:
        lines.append(f"### `{a['path'].name}`")
        lines.append("")
        lines.append(format_report(a))
        lines.append("")

    return "\n".join(lines)


def print_summary(analysis: dict) -> None:
    print(f"\n=== {analysis['path'].name} ===")
    print(
        f"Pass0-A/B captures: {analysis['leg_a_segments']}/{analysis['leg_b_segments']} "
        f"(expect {LUT_LEN}/{LUT_LEN}), geo pool: {len(analysis['geo_pool'])} "
        f"(expect ≤{LUT_LEN * 6}, FW ch5≈152)"
    )
    if analysis["dlut_vals"].size:
        print(
            f"d-table end: Id={analysis['dlut_amps'][-1]:.3f} A, "
            f"Ud_res={analysis['dlut_vals'][-1]:.4f} V"
        )
    if analysis["fw_lut"]:
        s = lut_summary(analysis["fw_lut"]["amps"], analysis["fw_lut"]["vals"])
        print(
            f"FW LUT: val_end={s['val_end']:.4f} V, plateau={s['plateau_count']} @ "
            f"{s['plateau_val']} V"
        )
    print(f"{'策略':<28} {'末档val':>8} {'sum':>8} {'平台':>4} {'~固件Δ':>8}")
    print("-" * 62)
    for name, r in analysis["results"].items():
        s = r["summary"]
        vs = r.get("vs_firmware", {})
        md = vs.get("max_val_diff", float("nan"))
        print(
            f"{name:<28} {s['val_end']:8.4f} {s['sum_vals']:8.2f} "
            f"{s['plateau_count']:4d} {md:8.4f}"
        )

    tc = analysis.get("table_cmp", {})
    if "proposed_vs_geo30_pre_sort" in tc:
        c = tc["proposed_vs_geo30_pre_sort"]
        print(
            f"proposed vs geo30 (pre-sort): maxΔval={c['max_d_val']:.6f} V, "
            f"diff bins={c['n_val_diff']}/{c['n']}"
        )
    bins = analysis.get("bin_stats", [])
    if bins:
        n_b = sum(1 for b in bins if b["n_b"] > 0)
        n_dual = sum(1 for b in bins if abs(b.get("delta_dual_vs_30only", 0)) > 1e-4)
        n_0eff = sum(
            1
            for b in bins
            if b["n_b"] > 0
            and abs(b["val_0only"] - b["val_30only"]) > 1e-4
            and not (math.isnan(b["val_0only"]) or math.isnan(b["val_30only"]))
        )
        print(f"per-bin: legs with 0° samples={n_b}/{len(bins)}, dual≠30° val bins={n_dual}, 0°≠30° val bins={n_0eff}")


def main() -> int:
    ap = argparse.ArgumentParser(description="Offline deadband geo merge from VOFA CSV")
    ap.add_argument("csv", nargs="*", type=Path, help="VOFA CSV path(s)")
    ap.add_argument(
        "--batch",
        nargs="*",
        metavar="TAG",
        help="Run batch: 2304/2321/2335 tags, or 'recent' (0126+2335+2321+2304), or 'paper'",
    )
    ap.add_argument("--fs", type=float, default=DEFAULT_FS)
    ap.add_argument("--md", type=Path, help="Write markdown report (single file mode)")
    ap.add_argument(
        "--md-dir",
        type=Path,
        help="Write markdown per CSV to directory",
    )
    ap.add_argument(
        "--export-dir",
        type=Path,
        help="Export LUT csv per strategy (proposed, geo30, v2321, v2335, ...)",
    )
    ap.add_argument(
        "--export-strategies",
        nargs="*",
        default=[
            "paper428",
            "paper_per_angle_max",
            "paper_per_phase_max",
            "paper_per_angle_phase",
            "proposed",
            "geo30",
            "v2335",
        ],
        help="Strategies to export when --export-dir set",
    )
    args = ap.parse_args()

    paths: list[Path] = list(args.csv)
    if args.batch is not None:
        tag_map = {"2304": 0, "2321": 1, "2335": 2}
        if not args.batch or args.batch == ["recent"] or args.batch == ["paper"]:
            paths.extend(RECENT_BATCH)
        else:
            for t in args.batch:
                if t in tag_map:
                    paths.extend([BATCH_DEFAULT[tag_map[t]]])
                elif t == "recent":
                    paths.extend(RECENT_BATCH)

    if not paths:
        ap.print_help()
        return 1

    # 去重保序
    seen: set[Path] = set()
    unique_paths: list[Path] = []
    for p in paths:
        rp = p.resolve()
        if rp not in seen:
            seen.add(rp)
            unique_paths.append(p)

    analyses: list[dict] = []
    for path in unique_paths:
        if not path.is_file():
            print(f"skip missing: {path}", file=sys.stderr)
            continue
        analysis = analyze_csv(path, fs=args.fs)
        analyses.append(analysis)
        print_summary(analysis)

        if args.md_dir:
            args.md_dir.mkdir(parents=True, exist_ok=True)
            out = args.md_dir / f"离线merge_{path.stem}.md"
            out.write_text(format_report(analysis), encoding="utf-8")
            print(f"wrote {out}")

        if args.export_dir:
            args.export_dir.mkdir(parents=True, exist_ok=True)
            for strat in args.export_strategies:
                if strat not in analysis["results"]:
                    continue
                r = analysis["results"][strat]
                lut_path = args.export_dir / f"lut_{path.stem}_{strat}.csv"
                export_lut_csv(lut_path, r["amps"], r["vals"], strat)
                print(f"exported {lut_path}")

    if args.md and len(analyses) == 1:
        args.md.write_text(format_report(analyses[0]), encoding="utf-8")
        print(f"wrote {args.md}")
    elif args.md and len(analyses) > 1:
        args.md.write_text(format_batch_report(analyses), encoding="utf-8")
        print(f"wrote batch {args.md}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
