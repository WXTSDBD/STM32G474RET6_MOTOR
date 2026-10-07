#!/usr/bin/env python3
"""
Id 标定 DUAL_FULL 分析：12 通道 Pass0/Pass1 + 离线 LUT 重建（amp 表与固件 deadband_id_cal.c 同步）。

通道（M1_TELEM_BRINGUP_K=12）:
  Ia,Ib,Ic, Id, Iq, theta, Ud_out, Uq_pi, Ta, Tb, Tc, duty_dev

用法:
  python tools/analyze_id_cal_dual_53.py VOFA+CSV/20260701/vofa+202607010010.csv
  python tools/analyze_id_cal_dual_53.py file.csv --md out.md --export-dir lut_out
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

RS = 0.115
TH30 = math.pi / 6.0
TH0 = 0.0
FS = 10000.0
LUT_HDR = -888888.0
LUT_TAIL = -999999.0
CAPTURE_EPS = 0.03
D_TO_PHASE = 0.8660254037844386


def build_amp_table() -> np.ndarray:
    """与 motor/deadband/deadband_id_cal.c s_id_cal_amp_table 一致。"""
    lo_end = 0.40
    lo_step = 0.003125
    lo = np.arange(0.05, lo_end + 1e-9, lo_step)
    mid = np.array(
        [
            0.4500, 0.5250, 0.6000, 0.6750, 0.7500, 0.8250, 0.9000, 0.9750,
            1.0500, 1.1250, 1.2000, 1.2750, 1.3500, 1.5000,
        ],
        dtype=float,
    )
    hi = np.array(
        [
            1.5500, 1.6833, 1.8167, 1.9500, 2.0833, 2.2167, 2.3500, 2.4833,
            2.6167, 2.7500, 3.0000,
        ],
        dtype=float,
    )
    return np.concatenate([lo, mid, hi])


AMP_TABLE = build_amp_table()

N_AMP = len(AMP_TABLE)


def load(path: Path) -> dict[str, np.ndarray]:
    d = np.genfromtxt(path, delimiter=",", names=True, dtype=np.float32)
    n = d.dtype.names
    keys = ["ia", "ib", "ic", "id", "iq", "theta", "ud", "uq", "ta", "tb", "tc", "duty_dev"]
    return {k: d[n[i]] for i, k in enumerate(keys)}


def parse_lut(c: dict[str, np.ndarray]) -> dict | None:
    ia = c["ia"]
    hi = np.where(np.abs(ia - LUT_HDR) < 1.0)[0]
    if len(hi) == 0:
        return None
    hi = int(hi[0])
    length = int(round(c["ib"][hi]))
    amps, vals = [], []
    for i in range(hi + 1, min(hi + 80, len(ia))):
        if abs(ia[i] - LUT_TAIL) < 1.0:
            break
        pairs = [
            (float(c["ia"][i]), float(c["ib"][i])),
            (float(c["ic"][i]), float(c["id"][i])),
            (float(c["iq"][i]), float(c["theta"][i])),
        ]
        for a, v in pairs:
            if len(amps) >= length:
                break
            if a > 0.0 or v > 0.0:
                amps.append(a)
                vals.append(v)
    return {
        "t_idx": hi,
        "t_s": hi / FS,
        "len": length,
        "amps": np.array(amps[:length]),
        "vals": np.array(vals[:length]),
        "geo_n_hdr": float(c["theta"][hi]),
    }


def find_segment_starts(c: dict[str, np.ndarray], lut_t: int) -> dict[str, int]:
    """按 theta 块 + LUT 位置估计 Pass0-A/B、Pass1 起点（样本索引）。"""
    th = c["theta"]
    pass0_dwell_s = 0.5
    pass1_dwell_s = 0.5
    pass0_span = int(N_AMP * pass0_dwell_s * FS)

    def block_start(t_target: float, t_min: int, t_max: int) -> int:
        best = t_min
        best_score = -1.0
        win = int(0.25 * FS)
        for i in range(t_min, max(t_min + 1, t_max - win)):
            sl = slice(i, i + win)
            score = float(np.mean(np.abs(th[sl] - t_target) < 0.05))
            if score > best_score:
                best_score = score
                best = i
        return best

    pass0a = block_start(TH30, int(0.3 * FS), int(1.5 * FS))
    pass0a_end = pass0a + pass0_span
    pass0b = block_start(TH0, pass0a_end, lut_t - int(2 * FS) if lut_t > 0 else pass0a_end + pass0_span)
    pass1_end = lut_t + int((N_AMP * pass1_dwell_s + 15.0) * FS)
    pass1 = block_start(TH30, lut_t + int(0.5 * FS), min(len(th), pass1_end))
    return {"pass0a": pass0a, "pass0b": pass0b, "pass1": pass1, "lut": lut_t}


def nearest_amp(id_ref: float) -> int:
    j = int(np.argmin(np.abs(AMP_TABLE - id_ref)))
    return j if abs(AMP_TABLE[j] - id_ref) < 0.03 else -1


def steady_slice(n: int, t0: int, dwell_s: float, tail_frac: float = 0.2) -> slice:
    dwell = int(dwell_s * FS)
    t1 = min(t0 + dwell, n)
    tail = max(int(dwell * tail_frac), 50)
    return slice(max(t0, t1 - tail), t1)


def count_sign_flips(x: np.ndarray) -> int:
    s = np.sign(x)
    s[s == 0] = np.nan
    flips = 0
    prev = 0.0
    for v in s:
        if math.isnan(v):
            continue
        if prev != 0.0 and v != prev:
            flips += 1
        prev = v
    return flips


def analyze_steps(
    c: dict[str, np.ndarray],
    start: int,
    dwell_s: float,
    label: str,
) -> list[dict]:
    rows = []
    n = len(c["id"])
    for k, id_ref in enumerate(AMP_TABLE):
        t0 = start + int(k * dwell_s * FS)
        if t0 + int(0.1 * FS) >= n:
            break
        sl = steady_slice(n, t0, dwell_s)
        ia, ib, ic = c["ia"][sl], c["ib"][sl], c["ic"][sl]
        id_m = float(np.nanmean(c["id"][sl]))
        iq_m = float(np.nanmean(c["iq"][sl]))
        ud = float(np.nanmean(c["ud"][sl]))
        uq = float(np.nanmean(c["uq"][sl]))
        ta = float(np.nanmean(c["ta"][sl]))
        tb = float(np.nanmean(c["tb"][sl]))
        tc = float(np.nanmean(c["tc"][sl]))
        dd = float(np.nanmean(c["duty_dev"][sl]))
        rows.append(
            {
                "pass": label,
                "step": k,
                "id_ref": float(id_ref),
                "id": id_m,
                "iq": iq_m,
                "id_err": float(id_ref - id_m),
                "ud": ud,
                "uq": uq,
                "ud_res": ud - id_m * RS,
                "u_over_i": ud / id_m if abs(id_m) > 0.04 else float("nan"),
                "ta": ta,
                "tb": tb,
                "tc": tc,
                "duty_dev": dd,
                "ib_flips": count_sign_flips(ib),
                "ic_flips": count_sign_flips(ic),
                "capture_ok": abs(id_ref - id_m) <= CAPTURE_EPS,
                "t_mid": (t0 + int(dwell_s * FS * 0.5)) / FS,
            }
        )
    return rows


@dataclass
class GeoSample:
    i_abs: float
    u_abs: float
    id_capture: float
    leg: str


def anti_park(d: float, q: float, theta: float) -> tuple[float, float]:
    c, s = math.cos(theta), math.sin(theta)
    return d * c - q * s, d * s + q * c


def inv_clarke(a: float, b: float) -> tuple[float, float, float]:
    ua = a
    ub = -0.5 * a + math.sqrt(3) / 2.0 * b
    uc = -0.5 * a - math.sqrt(3) / 2.0 * b
    return ua, ub, uc


def geo_point(theta: float, id_fb: float, ud_res: float) -> list[GeoSample]:
    u_a, u_b = anti_park(abs(ud_res), 0.0, theta)
    ua, ub, uc = inv_clarke(u_a, u_b)
    u0 = (ua + ub + uc) / 3.0
    ua -= u0
    ub -= u0
    uc -= u0
    i_a, i_b = anti_park(abs(id_fb), 0.0, theta)
    ia, ib, ic = inv_clarke(i_a, i_b)
    leg = "A" if abs(theta - TH30) < abs(theta - TH0) else "B"
    out = []
    for i_abs, u_abs in [(abs(ia), abs(ua)), (abs(ib), abs(ub)), (abs(ic), abs(uc))]:
        if i_abs <= 0.0:
            continue
        out.append(GeoSample(i_abs, u_abs, abs(id_fb), leg))
    return out


def median(xs: list[float]) -> float:
    if not xs:
        return float("nan")
    s = sorted(xs)
    return s[len(s) // 2]


def build_plut(
    samples: list[GeoSample],
    dlut_vals: np.ndarray,
    *,
    amp_leg: str | None,
    val_legs: str | None,
) -> tuple[np.ndarray, np.ndarray]:
    amps = np.zeros(N_AMP)
    vals = np.zeros(N_AMP)
    for k, id_k in enumerate(AMP_TABLE):
        hit = [s for s in samples if abs(s.id_capture - id_k) <= CAPTURE_EPS]
        if not hit:
            vals[k] = dlut_vals[k] * D_TO_PHASE if k < len(dlut_vals) else 0.0
            amps[k] = id_k * D_TO_PHASE
            continue
        amp_pool = [s for s in hit if s.leg == amp_leg] if amp_leg else hit
        if not amp_pool:
            amp_pool = [s for s in hit if s.leg == "A"] or hit
        val_pool = [s for s in hit if s.leg == val_legs] if val_legs else hit
        if not val_pool:
            val_pool = hit
        if amp_pool:
            amps[k] = max(s.i_abs for s in amp_pool)
        else:
            amps[k] = id_k * D_TO_PHASE
        if val_pool:
            vals[k] = median([s.u_abs for s in val_pool])
        else:
            vals[k] = dlut_vals[k] * D_TO_PHASE if k < len(dlut_vals) else 0.0
    order = np.argsort(amps)
    amps, vals = amps[order], vals[order]
    for i in range(1, len(vals)):
        if vals[i] < vals[i - 1]:
            vals[i] = vals[i - 1]
    return amps, vals


def lut_interp(amps: np.ndarray, vals: np.ndarray, x: float) -> float:
    if x <= amps[0]:
        return float(vals[0])
    if x >= amps[-1]:
        return float(vals[-1])
    j = int(np.searchsorted(amps, x))
    a0, a1 = amps[j - 1], amps[j]
    t = (x - a0) / (a1 - a0) if a1 > a0 else 0.0
    return float(vals[j - 1] + t * (vals[j] - vals[j - 1]))


def infer_pass0b_dwell(pass0b_start: int, lut_t: int) -> float:
    dt = (lut_t - pass0b_start) / FS
    return dt / N_AMP


def analyze(path: Path) -> dict:
    c = load(path)
    lut = parse_lut(c)
    if lut is None:
        raise ValueError("no LUT burst")
    seg = find_segment_starts(c, lut["t_idx"])
    dwell_b = infer_pass0b_dwell(seg["pass0b"], seg["lut"])
    dwell_a = 0.5
    dwell_p1 = 0.5

    p0a = analyze_steps(c, seg["pass0a"], dwell_a, "Pass0-A@30")
    p0b = analyze_steps(c, seg["pass0b"], dwell_b, "Pass0-B@0")
    p1 = analyze_steps(c, seg["pass1"], dwell_p1, "Pass1@30+abc")

    samples: list[GeoSample] = []
    dlut = np.zeros(N_AMP)
    for row in p0a:
        if not row["capture_ok"]:
            continue
        k = row["step"]
        dlut[k] = abs(row["ud_res"])
        samples.extend(geo_point(TH30, row["id"], row["ud_res"]))
    for row in p0b:
        if not row["capture_ok"]:
            continue
        samples.extend(geo_point(TH0, row["id"], row["ud_res"]))

    strategies = {
        "fw_burst": (lut["amps"], lut["vals"]),
        "proposed": build_plut(samples, dlut, amp_leg="A", val_legs=None),
        "val30_only": build_plut(samples, dlut, amp_leg="A", val_legs="A"),
        "val0_only": build_plut(samples, dlut, amp_leg="A", val_legs="B"),
        "geo30_dlut": build_plut(
            [s for s in samples if s.leg == "A"], dlut, amp_leg="A", val_legs="A"
        ),
        "scale0866": (AMP_TABLE * D_TO_PHASE, dlut * D_TO_PHASE),
    }

    # Pass1 compensation quality: Ud vs Rs*Id
    p1_hi = [r for r in p1 if r["id_ref"] >= 0.5 and r["capture_ok"]]
    p1_lo = [r for r in p1 if r["id_ref"] <= 0.2]

    return {
        "path": path,
        "lut": lut,
        "seg": seg,
        "dwell_b_inferred": dwell_b,
        "pass0a": p0a,
        "pass0b": p0b,
        "pass1": p1,
        "strategies": strategies,
        "samples_n": len(samples),
        "p1_hi": p1_hi,
        "p1_lo": p1_lo,
    }


def diagnosis(r: dict) -> list[str]:
    lines = []
    p0a, p0b, p1 = r["pass0a"], r["pass0b"], r["pass1"]
    lut = r["lut"]

    ok_a = sum(1 for x in p0a if x["capture_ok"])
    ok_b = sum(1 for x in p0b if x["capture_ok"])
    ok_p1 = sum(1 for x in p1 if x["capture_ok"])
    lines.append(
        f"Pass0 capture: A {ok_a}/{len(p0a)}, B {ok_b}/{len(p0b)} "
        f"(Pass0-B 推断 dwell={r['dwell_b_inferred']:.2f}s/档，期望 1.0s)"
    )

    for ref in [0.05, 0.10, 0.15, 0.30, 1.0, 1.5, 3.0]:
        a = next((x for x in p0a if abs(x["id_ref"] - ref) < 0.008), None)
        b = next((x for x in p0b if abs(x["id_ref"] - ref) < 0.008), None)
        p = next((x for x in p1 if abs(x["id_ref"] - ref) < 0.008), None)
        if a and b:
            lines.append(
                f"@Id={ref:.2f}A Pass0 Ud_res: 30°={a['ud_res']:.3f}V 0°={b['ud_res']:.3f}V "
                f"Δ={a['ud_res']-b['ud_res']:+.3f}V"
            )
        if a and p:
            lines.append(
                f"@Id={ref:.2f}A Pass1: Ud={p['ud']:.3f}V (Pass0={a['ud']:.3f}) "
                f"U/I={p['u_over_i']:.2f}Ω Ib_flips={p['ib_flips']} Ic_flips={p['ic_flips']} "
                f"duty_dev={p['duty_dev']:.3f}"
            )

    hi = r["p1_hi"]
    if hi:
        ui = np.array([x["u_over_i"] for x in hi if not math.isnan(x["u_over_i"])])
        lines.append(
            f"Pass1 @Id≥0.5A: U/I min/med/max={ui.min():.2f}/{np.median(ui):.2f}/{ui.max():.2f}Ω "
            f"(目标≈Rs={RS})"
        )

    lo = r["p1_lo"]
    if lo:
        fl = np.mean([x["ib_flips"] + x["ic_flips"] for x in lo])
        lines.append(f"Pass1 小电流 Id≤0.2A: 平均 Ib+Ic 换向 {fl:.0f} 次/dwell")

    if len(lut["amps"]) >= 5:
        dv = np.diff(lut["vals"][-10:])
        lines.append(
            f"固件 plut 末 10 点 dval/dI 均值={np.mean(dv):.4f}V/点，"
            f"末档 amp={lut['amps'][-1]:.3f} val={lut['vals'][-1]:.3f}"
        )

    # strategy compare at 1A phase current ~0.87
    for name, (a, v) in r["strategies"].items():
        if name == "fw_burst":
            continue
        if len(a) < 2:
            continue
        d = np.max(np.abs(np.interp(lut["amps"], a, v) - lut["vals"]))
        lines.append(f"离线 {name} vs 固件 plut max|Δval|={d:.4f}V")

    return lines


def write_md(r: dict, out: Path) -> None:
    lut = r["lut"]
    lines = [
        f"# Id 标定 53 档分析报告 — `{r['path'].name}`",
        "",
        f"- LUT 突发: t={lut['t_s']:.2f}s, len={lut['len']}",
        f"- Pass0-A idx={r['seg']['pass0a']} ({r['seg']['pass0a']/FS:.2f}s)",
        f"- Pass0-B idx={r['seg']['pass0b']} ({r['seg']['pass0b']/FS:.2f}s), "
        f"推断 dwell={r['dwell_b_inferred']:.2f}s",
        f"- Pass1 idx={r['seg']['pass1']} ({r['seg']['pass1']/FS:.2f}s)",
        f"- geo 样本池: {r['samples_n']} 条",
        "",
        "## 诊断摘要",
        "",
    ]
    for ln in diagnosis(r):
        lines.append(f"- {ln}")

    lines += ["", "## 固件 commit plut（VOFA burst）", "", "| amp | val |", "|---:|---:|"]
    for a, v in zip(lut["amps"], lut["vals"]):
        lines.append(f"| {a:.4f} | {v:.4f} |")

    lines += ["", "## 离线 LUT 策略对比（末 8 点）", ""]
    for name, (a, v) in r["strategies"].items():
        lines += [f"### {name}", "", "| amp | val |", "|---:|---:|"]
        for i in range(max(0, len(a) - 8), len(a)):
            lines.append(f"| {a[i]:.4f} | {v[i]:.4f} |")
        lines.append("")

    for title, key in [("Pass0-A", "pass0a"), ("Pass0-B", "pass0b"), ("Pass1 abc", "pass1")]:
        rows = r[key]
        lines += [
            f"## {title}",
            "",
            "| step | Id_ref | Id | err | Ud | Ud_res | U/I | duty_dev | Ib↔ | Ic↔ | ok |",
            "|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|:---:|",
        ]
        for x in rows[: min(len(rows), N_AMP)]:
            ok = "Y" if x["capture_ok"] else "N"
            uoi = f"{x['u_over_i']:.2f}" if not math.isnan(x["u_over_i"]) else "—"
            lines.append(
                f"| {x['step']} | {x['id_ref']:.3f} | {x['id']:.3f} | {x['id_err']:+.3f} | "
                f"{x['ud']:.3f} | {x['ud_res']:.3f} | {uoi} | {x['duty_dev']:.3f} | "
                f"{x['ib_flips']} | {x['ic_flips']} | {ok} |"
            )

    lines += [
        "",
        "## 离线 LUT 建议",
        "",
        "1. **高 Id（≥0.45A）**：Pass1 U/I≈Rs 说明 abc 补偿主链路有效；plut 末档以 Pass0-A Ud_res 经 geo 为准。",
        "2. **低 Id（≤0.2A）**：若 Ib/Ic 换向仍多，优先检查 plut 低 I 点密度与 merge（val30_only vs dual）。",
        "3. **0° vs 30°**：Pass0-B Ud_res 系统性高于 30° 时，dual median val 会抬高 plut；可试 val30_only 或恢复 TWO_CLUSTER。",
        "4. **Pass0-B dwell**：若推断 dwell≈0.5s 而非 1.0s，0° 高 Id capture 可能偏软，应确认固件已烧录 PASS0_B_DWELL=1.0。",
        "",
    ]
    out.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", type=Path)
    ap.add_argument("--md", type=Path)
    ap.add_argument("--export-dir", type=Path)
    args = ap.parse_args()

    r = analyze(args.csv)
    print(f"=== {args.csv.name} ===")
    for ln in diagnosis(r):
        print(ln)

    if args.md:
        args.md.parent.mkdir(parents=True, exist_ok=True)
        write_md(r, args.md)
        print(f"wrote {args.md}")

    if args.export_dir:
        args.export_dir.mkdir(parents=True, exist_ok=True)
        for name, (a, v) in r["strategies"].items():
            p = args.export_dir / f"lut_{args.csv.stem}_{name}.csv"
            lines = ["amp_A,val_V"] + [f"{a[i]:.6f},{v[i]:.6f}" for i in range(len(a))]
            p.write_text("\n".join(lines) + "\n", encoding="utf-8")
        print(f"exported LUTs to {args.export_dir}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
