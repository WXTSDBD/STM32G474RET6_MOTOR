#!/usr/bin/env python3
"""
多角 Pass0 死区 geo 分析：0°/30°/60°/90°/120°（可配置）。

从 VOFA CSV 提取 **实测** capture（按 ch5=theta_el 分角），并可选把同一 (Id, Ud_res)
**合成投影**到其它角度（对齐 deadband_geo_dlut_point，非实测）。

输出（默认与 CSV 同目录）：
  *_pass0_captures.csv     — 各角各档 Id/Ud 实测
  *_geo_raw.csv            — abc 样本（phase,i,u,id,theta_deg,source）
  *_ud_res_vs_angle.csv    — 按 Id 锚档 pivot：各角 Ud_res
  *_u_strong_phase.csv     — 各角强相 |u'| vs Id（看角度差异）
  *_multi_angle_report.md
  *_multi_angle_plot.png

用法:
  python tools/multi_angle_geo_analysis.py VOFA+CSV/20260627/vofa+202606281146.csv
  python tools/multi_angle_geo_analysis.py file.csv --synthetic-all
  python tools/multi_angle_geo_analysis.py file.csv --angles 0,30,60,90,120

固件五角实扫：motor_params_m1.h 设 M1_ID_CAL_MULTI_ANGLE_ENABLE=1 后重录。
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from collections import defaultdict
from dataclasses import asdict
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import offline_deadband_geo_merge as og  # noqa: E402
from offline_deadband_geo_merge import (  # noqa: E402
    CAPTURE_EPS,
    ID_CAL_AMP_TABLE,
    I_ZERO_A,
    RS_OHM,
    CapturePoint,
    GeoSample,
    geo_dlut_point,
    unwrap,
)
from analyze_id_cal_sweep_vofa import park, load_csv as load_csv_cols  # noqa: E402
from parse_lut_vofa import load_csv, find_header  # noqa: E402

DEFAULT_ANGLES_DEG = (0.0, 30.0, 60.0, 90.0, 120.0)
PHASE_NAMES = ("A", "B", "C")


def deg_to_rad(d: float) -> float:
    return d * math.pi / 180.0


def rad_to_deg(r: float) -> float:
    return r * 180.0 / math.pi


def wrap_pi(x: float) -> float:
    while x > math.pi:
        x -= 2.0 * math.pi
    while x < -math.pi:
        x += 2.0 * math.pi
    return x


def nearest_angle_index(theta_rad: float, angles_rad: list[float], tol: float = 0.08) -> int | None:
    best_i = 0
    best_d = 1e9
    for i, ta in enumerate(angles_rad):
        d = abs(wrap_pi(theta_rad - ta))
        if d < best_d:
            best_d = d
            best_i = i
    return best_i if best_d <= tol else None


def extract_captures_by_angle(
    csv_path: Path,
    angles_deg: tuple[float, ...],
    fs: float = 20000.0,
) -> dict[float, list[CapturePoint]]:
    """按 theta 最近邻分角，复用 offline 锚档 + dwell 末 capture 逻辑。"""
    angles_rad = [deg_to_rad(d) for d in angles_deg]
    ia, ib, ic, ud_pi, id_ref, theta_raw = load_csv_cols(csv_path)
    data = load_csv(csv_path)

    # Pass0 段：LUT 突发前或 open_seq<40
    try:
        hi = find_header(data)
        sweep_end = hi if hi > 0 else len(data)
    except Exception:
        sweep_end = len(ia)

    n = sweep_end
    theta_u = unwrap(theta_raw[:n])
    id_fb_all, _ = park(ia[:n], ib[:n], ic[:n], theta_u)
    t = np.arange(n) / fs
    gap_samples = int(0.15 * fs)
    min_dwell_samples = int(0.08 * fs)

    angle_idx = np.array(
        [
            nearest_angle_index(float(theta_u[i]), angles_rad)
            for i in range(n)
        ]
    )
    anchor_idx = np.full(n, -1, dtype=int)
    for i in range(n):
        d = np.abs(ID_CAL_AMP_TABLE - float(id_ref[i]))
        j = int(np.argmin(d))
        anchor_idx[i] = j if d[j] <= 0.025 else -1

    out: dict[float, list[CapturePoint]] = {d: [] for d in angles_deg}

    for ai, ang_deg in enumerate(angles_deg):
        for k, id_k in enumerate(ID_CAL_AMP_TABLE):
            mask = (angle_idx == ai) & (anchor_idx == k)
            idxs = np.where(mask)[0]
            if len(idxs) == 0:
                continue

            segs: list[list[int]] = [[int(idxs[0])]]
            for i in range(1, len(idxs)):
                if idxs[i] - idxs[i - 1] > gap_samples:
                    segs.append([int(idxs[i])])
                else:
                    segs[-1].append(int(idxs[i]))
            best_seg = next((np.array(s) for s in segs if len(s) >= min_dwell_samples), segs[0])
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
            leg_tag = f"L{ai}"
            out[ang_deg].append(
                CapturePoint(
                    id_ref=float(id_k),
                    id_fb=id_mean,
                    ud_pi=ud_val,
                    ud_res=ud_res,
                    theta_el=angles_rad[ai],
                    leg=leg_tag,
                    t_mid=float(t[cap_idx]),
                )
            )

    for ang in angles_deg:
        out[ang] = sorted(out[ang], key=lambda c: c.id_ref)
    return out


def captures_to_geo(
    captures: list[CapturePoint],
    source: str,
) -> list[dict]:
    rows: list[dict] = []
    for c in captures:
        for s in geo_dlut_point(c.theta_el, c.id_fb, c.ud_res):
            rows.append(
                {
                    "source": source,
                    "theta_deg": rad_to_deg(c.theta_el),
                    "id_capture": c.id_fb,
                    "id_ref": c.id_ref,
                    "ud_res": c.ud_res,
                    "ud_pi": c.ud_pi,
                    "phase": PHASE_NAMES[s.phase],
                    "i_abs": s.i_abs,
                    "u_abs": s.u_abs,
                    "t_mid": c.t_mid,
                }
            )
    return rows


def synthetic_project_all_angles(
    measured: dict[float, list[CapturePoint]],
    angles_deg: tuple[float, ...],
) -> list[dict]:
    """对每个实测 (Id, Ud_res)，投影到全部角度（几何分解，Ud 不变）。"""
    rows: list[dict] = []
    seen: set[tuple[float, float, float]] = set()
    for caps in measured.values():
        for c in caps:
            key = (round(c.id_fb, 4), round(c.ud_res, 4), round(c.theta_el, 4))
            if key in seen:
                continue
            seen.add(key)
            for ang in angles_deg:
                th = deg_to_rad(ang)
                for s in geo_dlut_point(th, c.id_fb, c.ud_res):
                    rows.append(
                        {
                            "source": "synthetic",
                            "theta_deg": ang,
                            "id_capture": c.id_fb,
                            "id_ref": c.id_ref,
                            "ud_res": c.ud_res,
                            "ud_pi": c.ud_pi,
                            "phase": PHASE_NAMES[s.phase],
                            "i_abs": s.i_abs,
                            "u_abs": s.u_abs,
                            "t_mid": c.t_mid,
                        }
                    )
    return rows


def strong_phase_u(rows: list[dict]) -> list[dict]:
    """每 (theta, id) 取 max|i| 相的 u'。"""
    buckets: dict[tuple[float, float], list[dict]] = defaultdict(list)
    for r in rows:
        buckets[(r["theta_deg"], r["id_capture"])].append(r)
    out: list[dict] = []
    for (td, idi), lst in sorted(buckets.items()):
        best = max(lst, key=lambda x: x["i_abs"])
        out.append(
            {
                "theta_deg": td,
                "id_A": idi,
                "strong_phase": best["phase"],
                "i_abs": best["i_abs"],
                "u_abs": best["u_abs"],
                "ud_res": best["ud_res"],
                "source": best["source"],
            }
        )
    return out


def write_csv(path: Path, rows: list[dict], fieldnames: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)


def build_report(
    csv_path: Path,
    angles_deg: tuple[float, ...],
    measured: dict[float, list[CapturePoint]],
    geo_rows: list[dict],
    strong_rows: list[dict],
) -> str:
    lines = [
        f"# 多角 geo 分析 — `{csv_path.name}`",
        "",
        f"目标角 (°)：{', '.join(f'{a:g}' for a in angles_deg)}",
        "",
        "## 1. 实测 capture 覆盖",
        "",
        "| θ (°) | 档数 | Id 范围 (A) | Ud_res 范围 (V) |",
        "|-------|------|-------------|-----------------|",
    ]
    for ang in angles_deg:
        caps = measured.get(ang, [])
        if not caps:
            lines.append(f"| {ang:g} | 0 | — | — |")
            continue
        ids = [c.id_fb for c in caps]
        uds = [c.ud_res for c in caps]
        lines.append(
            f"| {ang:g} | {len(caps)} | [{min(ids):.3f}, {max(ids):.3f}] | "
            f"[{min(uds):.3f}, {max(uds):.3f}] |"
        )

    lines += [
        "",
        "> **说明**：仅 30°+0° 为当前默认固件双角实扫；60/90/120 需 `M1_ID_CAL_MULTI_ANGLE_ENABLE=1` 录波，"
        "或用 `--synthetic-all` 由实测 Ud_res 几何投影（**同一 Ud_res**，只看分解差异）。",
        "",
        "## 2. 同 Id 各角 Ud_res（实测，差异=物理/PI 行为）",
        "",
    ]

    # pivot measured ud_res by id_ref
    id_set = sorted({c.id_ref for caps in measured.values() for c in caps})
    header = "| Id_ref |" + "|".join(f" θ={a:g}° " for a in angles_deg) + "| max−min |"
    sep = "|--------|" + "|".join("--------:" for _ in angles_deg) + "|---------:|"
    lines.extend([header, sep])
    for id_k in id_set[:16]:
        cells = []
        vals = []
        for ang in angles_deg:
            hit = [c for c in measured.get(ang, []) if abs(c.id_ref - id_k) < 0.001]
            if hit:
                v = hit[0].ud_res
                vals.append(v)
                cells.append(f"{v:.3f}")
            else:
                cells.append("—")
        spread = f"{max(vals) - min(vals):.3f}" if len(vals) >= 2 else "—"
        lines.append(f"| {id_k:.3f} | " + " | ".join(cells) + f" | {spread} |")
    if len(id_set) > 16:
        lines.append(f"| … | （共 {len(id_set)} 档） | | |")

    lines += [
        "",
        "## 3. 强相 |u'| vs 角度（@Id≈0.5 / 1.0 A）",
        "",
    ]
    for target in (0.5, 1.0):
        lines.append(f"### Id ≈ {target} A")
        lines.append("")
        lines.append("| θ (°) | 强相 | |i| (A) | u' (V) | source |")
        lines.append("|-------|------|--------|--------|--------|")
        for ang in angles_deg:
            cands = [r for r in strong_rows if abs(r["id_A"] - target) < 0.08 and abs(r["theta_deg"] - ang) < 0.5]
            if not cands:
                lines.append(f"| {ang:g} | — | — | — | — |")
                continue
            r = min(cands, key=lambda x: abs(x["id_A"] - target))
            lines.append(
                f"| {ang:g} | {r['strong_phase']} | {r['i_abs']:.3f} | {r['u_abs']:.3f} | {r['source']} |"
            )
        lines.append("")

    lines += [
        "## 4. 解读提示",
        "",
        "- **Ud_res 随 θ 变化大** → PI/死区在不同锁轴角表现不同，需多角实扫建表或 θ 相关补偿。",
        "- **Ud_res 随 θ 几乎不变、仅 u' 变** → 主要是 Park/Clarke 几何；共享 d 表 + 角度分解即可。",
        "- **合成行 (synthetic)** 不能代替实扫，只用于看分解形状。",
        "",
    ]
    return "\n".join(lines)


def plot_multi_angle(
    out_png: Path,
    measured: dict[float, list[CapturePoint]],
    strong_rows: list[dict],
    angles_deg: tuple[float, ...],
) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(11, 9))
    colors = plt.cm.viridis(np.linspace(0.1, 0.9, len(angles_deg)))

    ax = axes[0, 0]
    for i, ang in enumerate(angles_deg):
        caps = measured.get(ang, [])
        if not caps:
            continue
        ax.plot([c.id_fb for c in caps], [c.ud_res for c in caps], "o-", color=colors[i], label=f"{ang:g}° meas", ms=3)
    ax.set_xlabel("|Id| @ capture (A)")
    ax.set_ylabel("Ud_res (V)")
    ax.set_title("Pass0 measured Ud_res vs Id")
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    ax = axes[0, 1]
    for ph, name in enumerate(PHASE_NAMES):
        for i, ang in enumerate(angles_deg):
            pts = [r for r in strong_rows if r["source"] == "measured" and abs(r["theta_deg"] - ang) < 0.5 and r["strong_phase"] == name]
            if not pts:
                continue
            pts = sorted(pts, key=lambda x: x["id_A"])
            ax.plot([p["id_A"] for p in pts], [p["u_abs"] for p in pts], "-", color=colors[i], alpha=0.5 if ph else 1.0, lw=1 if ph else 1.5)
        if ph == 0:
            for i, ang in enumerate(angles_deg):
                ax.plot([], [], "-", color=colors[i], label=f"{ang:g}°")
    ax.set_xlabel("|Id| (A)")
    ax.set_ylabel("strong phase u' (V)")
    ax.set_title("Measured strong-phase u' vs Id (color=theta)")
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    ax = axes[1, 0]
    for target, mk in ((0.3, "o"), (0.5, "s"), (1.0, "^")):
        xs, ys = [], []
        for ang in angles_deg:
            cands = [r for r in strong_rows if abs(r["id_A"] - target) < 0.08 and abs(r["theta_deg"] - ang) < 0.5]
            if cands:
                r = min(cands, key=lambda x: abs(x["id_A"] - target))
                xs.append(ang)
                ys.append(r["u_abs"])
        if xs:
            ax.plot(xs, ys, f"-{mk}", label=f"Id≈{target}A", ms=5)
    ax.set_xlabel("θ (deg)")
    ax.set_ylabel("strong phase u' (V)")
    ax.set_title("strong-phase u' vs lock angle")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    ax = axes[1, 1]
    syn = [r for r in strong_rows if r["source"] == "synthetic" and abs(r["id_A"] - 1.0) < 0.1]
    for ph, name in enumerate(PHASE_NAMES):
        pts = sorted([r for r in syn if r["strong_phase"] == name], key=lambda x: x["theta_deg"])
        if pts:
            ax.plot([p["theta_deg"] for p in pts], [p["u_abs"] for p in pts], "o-", label=f"ph {name}", ms=4)
    ax.set_xlabel("θ (deg)")
    ax.set_ylabel("u' (V)")
    ax.set_title("Synthetic: same Ud_res@1A projected to all angles")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    fig.suptitle(out_png.stem, fontsize=10)
    fig.tight_layout()
    out_png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_png, dpi=150)
    plt.close(fig)


def main() -> int:
    ap = argparse.ArgumentParser(description="多角 Pass0 geo / Ud_res 离线分析")
    ap.add_argument("csv", type=Path, nargs="+", help="VOFA CSV")
    ap.add_argument("--angles", default="0,30,60,90,120", help="目标角列表 (deg)")
    ap.add_argument(
        "--synthetic-all",
        action="store_true",
        help="对每个实测 (Id,Ud_res) 额外投影到全部角度",
    )
    ap.add_argument("--out-dir", type=Path, default=None)
    args = ap.parse_args()

    angles_deg = tuple(float(x.strip()) for x in args.angles.split(",") if x.strip())

    for csv_path in args.csv:
        csv_path = csv_path.resolve()
        stem = csv_path.stem
        out_dir = args.out_dir or csv_path.parent

        measured = extract_captures_by_angle(csv_path, angles_deg)
        geo_meas = captures_to_geo(
            [c for caps in measured.values() for c in caps],
            "measured",
        )
        geo_rows = list(geo_meas)
        if args.synthetic_all:
            geo_rows.extend(synthetic_project_all_angles(measured, angles_deg))

        cap_rows = []
        for ang, caps in measured.items():
            for c in caps:
                cap_rows.append(
                    {
                        "theta_deg": ang,
                        "id_ref": c.id_ref,
                        "id_fb": c.id_fb,
                        "ud_pi": c.ud_pi,
                        "ud_res": c.ud_res,
                        "t_mid": c.t_mid,
                    }
                )

        strong = strong_phase_u(geo_rows)

        write_csv(
            out_dir / f"{stem}_pass0_captures.csv",
            cap_rows,
            ["theta_deg", "id_ref", "id_fb", "ud_pi", "ud_res", "t_mid"],
        )
        write_csv(
            out_dir / f"{stem}_geo_raw.csv",
            geo_rows,
            [
                "source", "theta_deg", "id_ref", "id_capture", "ud_res", "ud_pi",
                "phase", "i_abs", "u_abs", "t_mid",
            ],
        )
        write_csv(
            out_dir / f"{stem}_u_strong_phase.csv",
            strong,
            ["theta_deg", "id_A", "strong_phase", "i_abs", "u_abs", "ud_res", "source"],
        )

        report = build_report(csv_path, angles_deg, measured, geo_rows, strong)
        rep_path = out_dir / f"{stem}_multi_angle_report.md"
        rep_path.write_text(report, encoding="utf-8")
        plot_multi_angle(out_dir / f"{stem}_multi_angle_plot.png", measured, strong, angles_deg)

        print(f"[OK] {csv_path.name}")
        print(f"     captures -> {out_dir / f'{stem}_pass0_captures.csv'}")
        print(f"     geo raw  -> {out_dir / f'{stem}_geo_raw.csv'}")
        print(f"     report   -> {rep_path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
