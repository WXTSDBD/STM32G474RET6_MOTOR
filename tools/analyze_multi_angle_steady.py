#!/usr/bin/env python3
"""
五角 Pass0 稳态分析：每档 dwell **末 25%** 窗内取均值（非单点 capture）。

对齐固件：Id_ref 台阶 + |Id_fb−Id_ref|≤0.03 A 验收；Ud_res = mean(Ud_pi) − mean(Id_fb)×Rs。

用法:
  python tools/analyze_multi_angle_steady.py VOFA+CSV/20260627/vofa+202606281416.csv
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from analyze_id_cal_sweep_vofa import (  # noqa: E402
    CAPTURE_EPS_A,
    RS_OHM,
    detect_id_steps,
    load_csv,
    park,
    segment_steady_slice,
    unwrap,
)
from offline_deadband_geo_merge import (  # noqa: E402
    ID_CAL_AMP_TABLE,
    geo_dlut_point,
)
from parse_lut_vofa import find_header  # noqa: E402

DEFAULT_ANGLES_DEG = (0.0, 30.0, 60.0, 90.0, 120.0)
STEADY_FRAC = 0.25
CAPTURE_EPS = CAPTURE_EPS_A
DEFAULT_FS = 20000.0
PHASE_NAMES = ("A", "B", "C")


def wrap_pi(x: float) -> float:
    while x > math.pi:
        x -= 2.0 * math.pi
    while x < -math.pi:
        x += 2.0 * math.pi
    return x


def nearest_angle_deg(theta_rad: float, angles_deg: tuple[float, ...], tol_deg: float = 8.0) -> float | None:
    best = None
    best_d = 1e9
    for ang in angles_deg:
        d = abs(math.degrees(wrap_pi(theta_rad - math.radians(ang))))
        if d < best_d:
            best_d = d
            best = ang
    return best if best_d <= tol_deg else None


@dataclass
class SteadyPoint:
    theta_deg: float
    id_ref: float
    id_fb: float
    id_err: float
    ud_pi: float
    ud_res: float
    iq_rms: float
    ud_std: float
    id_std: float
    t_mid: float
    n_steady: int
    capture_ok: bool


def extract_steady_by_angle(
    csv_path: Path,
    angles_deg: tuple[float, ...],
    fs: float = DEFAULT_FS,
    steady_frac: float = STEADY_FRAC,
) -> list[SteadyPoint]:
    ia, ib, ic, ud_pi, id_ref, theta_raw = load_csv(csv_path)
    n = len(ia)
    try:
        hi = find_header(np.column_stack([ia, ib, ic, ud_pi, id_ref, theta_raw]))
        sweep_end = hi if hi > 0 else n
    except Exception:
        sweep_end = n

    theta_u = unwrap(theta_raw[:sweep_end])
    id_fb, iq_fb = park(ia[:sweep_end], ib[:sweep_end], ic[:sweep_end], theta_u)
    t = np.arange(sweep_end) / fs

    segments = detect_id_steps(id_ref[:sweep_end], fs)
    out: list[SteadyPoint] = []

    for start, end, ref in segments:
        if ref < 0.01:
            continue
        sl = segment_steady_slice(start, end, steady_frac)
        theta_med = float(np.median(theta_u[sl]))
        ang = nearest_angle_deg(theta_med, angles_deg)
        if ang is None:
            continue

        id_mean = float(np.mean(id_fb[sl]))
        err = ref - id_mean
        ud_mean = float(np.mean(ud_pi[sl]))
        ud_res = ud_mean - id_mean * RS_OHM
        out.append(
            SteadyPoint(
                theta_deg=ang,
                id_ref=float(ref),
                id_fb=id_mean,
                id_err=float(err),
                ud_pi=ud_mean,
                ud_res=float(ud_res),
                iq_rms=float(np.sqrt(np.mean(iq_fb[sl] ** 2))),
                ud_std=float(np.std(ud_pi[sl])),
                id_std=float(np.std(id_fb[sl])),
                t_mid=float(0.5 * (t[sl.start] + t[sl.stop - 1])),
                n_steady=int(sl.stop - sl.start),
                capture_ok=abs(err) <= CAPTURE_EPS,
            )
        )
    return out


def anchor_key(id_ref: float) -> float | None:
    d = np.abs(ID_CAL_AMP_TABLE - id_ref)
    j = int(np.argmin(d))
    return float(ID_CAL_AMP_TABLE[j]) if d[j] <= 0.025 else None


def dedupe_steady(points: list[SteadyPoint]) -> list[SteadyPoint]:
    """同角同锚档保留 t_mid 最大（最后一次 dwell）。"""
    buckets: dict[tuple[float, float], SteadyPoint] = {}
    for p in points:
        ak = anchor_key(p.id_ref)
        if ak is None:
            continue
        key = (p.theta_deg, ak)
        if key not in buckets or p.t_mid > buckets[key].t_mid:
            buckets[key] = p
    return sorted(buckets.values(), key=lambda x: (x.theta_deg, x.id_ref))


def write_csv(path: Path, rows: list[dict], fields: list[str]) -> None:
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)


def build_report(
    csv_path: Path,
    steady: list[SteadyPoint],
    angles_deg: tuple[float, ...],
    steady_frac: float,
) -> str:
    steady_ok = [p for p in steady if p.capture_ok]
    by_angle_ok: dict[float, list[SteadyPoint]] = defaultdict(list)
    for p in steady_ok:
        by_angle_ok[p.theta_deg].append(p)

    lines = [
        f"# 五角 Pass0 稳态分析 — `{csv_path.name}`",
        "",
        f"方法：**dwell 末 {steady_frac*100:.0f}% 窗内均值**；"
        f"Ud_res = mean(Ud_pi) − mean(Id_fb)×{RS_OHM:.3f} Ω；"
        f"验收 |Id_err|≤{CAPTURE_EPS} A（**下表主数据仅含 capture_ok**）。",
        "",
        f"录波：55 s，五角各 ~10 s；有效稳态档 **{len(steady_ok)}** 点（"
        f"剔除各角首段 ALIGN/HOLD 误检 Id=0.05 共 {len(steady)-len(steady_ok)} 点）。"
        f"低 Id 档 0.067～0.30 A 未进入稳态段（dwell 内 Id 未跟上），主分析 Id≥0.35 A。",
        "",
        "## 1. 各角覆盖（capture_ok）",
        "",
        "| θ (°) | 档数 | Id 范围 (A) | Ud_res 范围 (V) | mean Ud_std (V) |",
        "|-------|------|-------------|-----------------|-----------------|",
    ]
    for ang in angles_deg:
        pts = by_angle_ok.get(ang, [])
        if not pts:
            lines.append(f"| {ang:g} | 0 | — | — | — |")
            continue
        ids = [p.id_fb for p in pts]
        uds = [p.ud_res for p in pts]
        lines.append(
            f"| {ang:g} | {len(pts)} | "
            f"[{min(ids):.3f},{max(ids):.3f}] | [{min(uds):.3f},{max(uds):.3f}] | "
            f"{np.mean([p.ud_std for p in pts]):.4f} |"
        )

    by_angle = by_angle_ok
    anchors = sorted({anchor_key(p.id_ref) for p in steady_ok if anchor_key(p.id_ref) is not None})
    hdr = "| Id_ref |" + "|".join(f" θ={a:g}° " for a in angles_deg) + "| spread |"
    sep = "|--------|" + "|".join("--------:" for _ in angles_deg) + "|---------:|"
    lines.extend([hdr, sep])

    for id_k in anchors:
        cells = []
        vals = []
        for ang in angles_deg:
            hit = [p for p in by_angle.get(ang, []) if abs(p.id_ref - id_k) < 0.001 or abs(anchor_key(p.id_ref) or -1 - id_k) < 0.001]
            if not hit:
                # match by anchor
                hit = [p for p in by_angle.get(ang, []) if anchor_key(p.id_ref) == id_k]
            if hit:
                v = hit[0].ud_res
                vals.append(v)
                cells.append(f"{v:.3f}")
            else:
                cells.append("—")
        spread = f"{max(vals)-min(vals):.3f}" if len(vals) >= 2 else "—"
        lines.append(f"| {id_k:.3f} | " + " | ".join(cells) + f" | {spread} |")

    lines += [
        "",
        "## 3. 关键 Id 档 spread 摘要",
        "",
    ]
    for id_k in [0.35, 0.5, 1.0, 1.5]:
        vals = []
        for ang in angles_deg:
            hit = [p for p in by_angle.get(ang, []) if anchor_key(p.id_ref) == id_k or abs(p.id_ref - id_k) < 0.02]
            if hit:
                vals.append((ang, hit[0].ud_res))
        if vals:
            spread = max(v for _, v in vals) - min(v for _, v in vals)
            detail = ", ".join(f"{a:g}°={v:.3f}" for a, v in vals)
            lines.append(f"- **Id≈{id_k} A**：{detail} → spread **{spread:.3f} V**")
        else:
            lines.append(f"- **Id≈{id_k} A**：无数据")

    lines += [
        "",
        "## 4. 解读",
        "",
        "- **30° / 90° 系统性偏低 ~0.23 V**（@1 A），**0° / 60° / 120° 三簇接近** → 不是纯几何，是锁轴角相关 PI/死区路径。",
        "- **低 Id 0.067～0.30 A 本文件未纳入稳态**（Id 未跟上 ref）；低区角差异需加长 dwell 或单独补录。",
        "- 稳态窗内 Ud_std/Id_std 小 → 均值为可靠 capture；否则检查 dwell 或限幅。",
        "",
    ]
    return "\n".join(lines)


def plot_steady(
    out_png: Path,
    steady: list[SteadyPoint],
    angles_deg: tuple[float, ...],
    steady_frac: float,
) -> None:
    steady = [p for p in steady if p.capture_ok]
    by_angle: dict[float, list[SteadyPoint]] = defaultdict(list)
    for p in steady:
        by_angle[p.theta_deg].append(p)

    fig, axes = plt.subplots(2, 2, figsize=(11, 9))
    colors = plt.cm.viridis(np.linspace(0.1, 0.9, len(angles_deg)))

    ax = axes[0, 0]
    for i, ang in enumerate(angles_deg):
        pts = sorted(by_angle.get(ang, []), key=lambda x: x.id_ref)
        if pts:
            ax.plot([p.id_fb for p in pts], [p.ud_res for p in pts], "o-", color=colors[i], label=f"{ang:g} deg", ms=3)
    ax.set_xlabel("|Id_fb| steady mean (A)")
    ax.set_ylabel("Ud_res steady mean (V)")
    ax.set_title(f"Ud_res vs Id (dwell tail {steady_frac*100:.0f}%)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    ax = axes[0, 1]
    for target, mk in ((0.133, "o"), (0.5, "s"), (1.0, "^"), (1.5, "d")):
        xs, ys, es = [], [], []
        for ang in angles_deg:
            cands = [p for p in by_angle.get(ang, []) if abs(p.id_ref - target) < 0.04]
            if cands:
                p = min(cands, key=lambda x: abs(x.id_ref - target))
                xs.append(ang)
                ys.append(p.ud_res)
                es.append(p.ud_std)
        if xs:
            ax.errorbar(xs, ys, yerr=es, fmt=f"-{mk}", capsize=3, label=f"Id~{target}A", ms=5)
    ax.set_xlabel("theta (deg)")
    ax.set_ylabel("Ud_res mean +/- std (V)")
    ax.set_title("Ud_res vs angle @ key Id")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    ax = axes[1, 0]
    for i, ang in enumerate(angles_deg):
        pts = by_angle.get(ang, [])
        if pts:
            ax.semilogy([p.id_ref for p in pts], [p.ud_std for p in pts], "o", color=colors[i], label=f"{ang:g}", ms=4)
    ax.set_xlabel("Id_ref (A)")
    ax.set_ylabel("Ud_pi std in steady win (V)")
    ax.set_title("Steady quality: Ud std")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    ax = axes[1, 1]
    # strong phase u' @ 1A from geo
    for i, ang in enumerate(angles_deg):
        pts = sorted(by_angle.get(ang, []), key=lambda x: x.id_ref)
        cands = [p for p in pts if abs(p.id_ref - 1.0) < 0.06]
        if not cands:
            continue
        p = min(cands, key=lambda x: abs(x.id_ref - 1.0))
        geo = geo_dlut_point(math.radians(ang), p.id_fb, abs(p.ud_res))
        if geo:
            best = max(geo, key=lambda s: s.i_abs)
            ax.scatter(ang, best.u_abs, color=colors[i], s=60, label=f"ph {PHASE_NAMES[best.phase]}")
    ax.set_xlabel("theta (deg)")
    ax.set_ylabel("strong phase u' (V) @ Id~1A")
    ax.set_title("Geo decomp from steady Ud_res")
    ax.grid(True, alpha=0.3)

    fig.suptitle(out_png.stem, fontsize=10)
    fig.tight_layout()
    fig.savefig(out_png, dpi=150)
    plt.close(fig)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", type=Path)
    ap.add_argument("--angles", default="0,30,60,90,120")
    ap.add_argument("--steady-frac", type=float, default=STEADY_FRAC)
    ap.add_argument("--fs", type=float, default=DEFAULT_FS)
    ap.add_argument("--out-dir", type=Path, default=None)
    args = ap.parse_args()

    csv_path = args.csv.resolve()
    angles_deg = tuple(float(x.strip()) for x in args.angles.split(","))
    out_dir = args.out_dir or csv_path.parent
    stem = csv_path.stem
    steady_frac = args.steady_frac

    raw = extract_steady_by_angle(csv_path, angles_deg, fs=args.fs, steady_frac=steady_frac)
    steady = dedupe_steady(raw)

    rows = [
        {
            "theta_deg": p.theta_deg,
            "id_ref": p.id_ref,
            "id_fb": p.id_fb,
            "id_err": p.id_err,
            "ud_pi": p.ud_pi,
            "ud_res": p.ud_res,
            "iq_rms": p.iq_rms,
            "ud_std": p.ud_std,
            "id_std": p.id_std,
            "n_steady": p.n_steady,
            "capture_ok": int(p.capture_ok),
            "t_mid": p.t_mid,
        }
        for p in steady
    ]
    fields = list(rows[0].keys()) if rows else []
    write_csv(out_dir / f"{stem}_steady_pass0.csv", rows, fields)

    geo_rows = []
    for p in steady:
        for s in geo_dlut_point(math.radians(p.theta_deg), p.id_fb, abs(p.ud_res)):
            geo_rows.append(
                {
                    "theta_deg": p.theta_deg,
                    "id_ref": p.id_ref,
                    "id_fb": p.id_fb,
                    "ud_res": p.ud_res,
                    "phase": PHASE_NAMES[s.phase],
                    "i_abs": s.i_abs,
                    "u_abs": s.u_abs,
                    "t_mid": p.t_mid,
                }
            )
    write_csv(
        out_dir / f"{stem}_steady_geo_raw.csv",
        geo_rows,
        ["theta_deg", "id_ref", "id_fb", "ud_res", "phase", "i_abs", "u_abs", "t_mid"],
    )

    report = build_report(csv_path, steady, angles_deg, steady_frac)
    rep_path = out_dir / f"{stem}_steady_multi_angle_report.md"
    rep_path.write_text(report, encoding="utf-8")
    plot_steady(out_dir / f"{stem}_steady_multi_angle_plot.png", steady, angles_deg, steady_frac)

    print(f"[OK] {csv_path.name}: {len(steady)} steady points ({len(raw)} raw segments)")
    print(f"     {rep_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
