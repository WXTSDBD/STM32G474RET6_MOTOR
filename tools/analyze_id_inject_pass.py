#!/usr/bin/env python3
"""
Pass0 capture + Pass1 abc duty Id 注入段深度分析（11 通道 VOFA）。

通道: Ia,Ib,Ic, Id, Iq, theta, Ud, Uq, Ta, Tb, Tc

用法:
  python tools/analyze_id_inject_pass.py VOFA+CSV/20260630/vofa+202607010058.csv
  python tools/analyze_id_inject_pass.py file.csv --md docs/out.md
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

RS = 0.115
TH30 = 0.5235987755982988
TH0 = 0.0
FS = 10000.0
DWELL_PASS0_A = 0.5
DWELL_PASS0_B = 1.0  # match M1_ID_CAL_PASS0_B_DWELL_S when set
DWELL_PASS1 = 0.5
INIT_HOLD = 0.5
VERIFY_INIT = 1.0
PASS0_DECAY = 0.8
LUT_HDR = -888888.0
LUT_TAIL = -999999.0
CAPTURE_EPS = 0.03

AMP_TABLE = np.array([
    0.05, 0.0667, 0.0833, 0.10, 0.1167, 0.1333, 0.15, 0.1667,
    0.1833, 0.20, 0.2167, 0.2333, 0.25, 0.2667, 0.2833, 0.30,
    0.35, 0.4091, 0.4682, 0.5273, 0.5864, 0.6455, 0.7045, 0.7636,
    0.8227, 0.8818, 0.9409, 1.00, 1.10, 1.2333, 1.3667, 1.50,
])
VERIFY_EXT = np.array([
    1.6333, 1.7667, 1.9000, 2.0333, 2.1667, 2.3000,
    2.4333, 2.5667, 2.7000, 2.8333, 3.0000,
])
PASS1_TABLE = np.concatenate([AMP_TABLE, VERIFY_EXT])


def load(path: Path) -> dict[str, np.ndarray]:
    d = np.genfromtxt(path, delimiter=",", names=True, dtype=float)
    names = d.dtype.names
    keys = ["ia", "ib", "ic", "id", "iq", "theta", "ud", "uq", "ta", "tb", "tc"]
    return {k: d[names[i]] for i, k in enumerate(keys)}


def parse_lut(c: dict[str, np.ndarray]) -> dict | None:
    ia = c["ia"]
    hi = np.where(np.abs(ia - LUT_HDR) < 1.0)[0]
    if len(hi) == 0:
        return None
    hi = int(hi[0])
    # read rows manually from hi
    rows = []
    for i in range(hi, min(hi + 200, len(ia))):
        rows.append([ia[i], c["ib"][i], c["ic"][i], c["id"][i], c["iq"][i], c["theta"][i]])
    hdr = rows[0]
    length = int(round(hdr[1]))
    amps, vals = [], []
    for row in rows[1:]:
        if abs(row[0] - LUT_TAIL) < 1.0:
            break
        for k in (0, 2, 4):
            if len(amps) >= length:
                break
            a, v = float(row[k]), float(row[k + 1])
            if a > 0 or v > 0:
                amps.append(a)
                vals.append(v)
    return {
        "t_idx": hi,
        "len": length,
        "amps": np.array(amps[:length]),
        "vals": np.array(vals[:length]),
        "rs_hdr": float(hdr[2]),
    }


def repark(ia, ib, ic, theta):
    beta = (ib - ic) / math.sqrt(3.0)
    c, s = np.cos(theta), np.sin(theta)
    return ia * c + beta * s, -ia * s + beta * c


def phase_rms(ia, ib, ic):
    return np.sqrt((ia * ia + ib * ib + ic * ic) / 3.0)


def duty_dev(ta, tb, tc):
    m = (ta + tb + tc) / 3.0
    return np.max(np.abs(np.stack([ta - m, tb - m, tc - m], axis=0)), axis=0)


def steady_stats(c: dict[str, np.ndarray], sl: slice) -> dict:
    ia, ib, ic = c["ia"][sl], c["ib"][sl], c["ic"][sl]
    th = c["theta"][sl]
    id_m, iq_m = c["id"][sl], c["iq"][sl]
    ud, uq = c["ud"][sl], c["uq"][sl]
    ta, tb, tc = c["ta"][sl], c["tb"][sl], c["tc"][sl]
    id_rp, iq_rp = repark(ia, ib, ic, th)
    i_rms = phase_rms(ia, ib, ic)
    dd = duty_dev(ta, tb, tc)
    return {
        "id": float(np.nanmean(id_m)),
        "iq": float(np.nanmean(iq_m)),
        "id_repark": float(np.nanmean(id_rp)),
        "iq_repark": float(np.nanmean(iq_rp)),
        "id_std": float(np.nanstd(id_m)),
        "iq_rms": float(np.sqrt(np.nanmean(iq_m ** 2))),
        "ud": float(np.nanmean(ud)),
        "uq": float(np.nanmean(uq)),
        "u_abs": float(np.sqrt(np.nanmean(ud) ** 2 + np.nanmean(uq) ** 2)),
        "theta_std": float(np.std(th)),
        "theta_span": float(np.max(th) - np.min(th)),
        "i_rms": float(np.nanmean(i_rms)),
        "i_rms_max": float(np.nanmax(i_rms)),
        "duty_dev_mean": float(np.nanmean(dd)),
        "duty_dev_max": float(np.nanmax(dd)),
        "ta": float(np.nanmean(ta)),
        "tb": float(np.nanmean(tb)),
        "tc": float(np.nanmean(tc)),
    }


def find_pass1_start(c: dict[str, np.ndarray], lut: dict, fs: float) -> int:
    """First sample after LUT burst where theta pinned ~30deg and sweep resumes."""
    n = len(c["id"])
    t_lut = lut["t_idx"]
    # skip LUT tail (~few ms) + align; search for Id step pattern after lut
    for i in range(t_lut + int(0.5 * fs), n - int(DWELL_PASS1 * fs)):
        sl = slice(i, i + int(0.3 * fs))
        th = c["theta"][sl]
        if np.std(th) > 0.02:
            continue
        if np.mean(np.abs(th - TH30) < 0.05) < 0.9:
            continue
        # look ahead one dwell for stable id
        sl2 = slice(i + int(0.2 * fs), i + int(0.45 * fs))
        id_mean = np.nanmean(c["id"][sl2])
        if 0.02 < id_mean < 0.12:
            return i
    # fallback: fixed offset from lut end
    return t_lut + int(4.0 * fs)


def analyze_pass_steps(
    c: dict[str, np.ndarray],
    start_idx: int,
    amp_table: np.ndarray,
    dwell: float,
    fs: float,
    label: str,
) -> list[dict]:
    rows = []
    for k, id_ref in enumerate(amp_table):
        t0 = start_idx + int(k * dwell * fs)
        t1 = t0 + int(dwell * fs)
        if t1 >= len(c["id"]):
            break
        # use last 20% of dwell as steady
        tail = int(0.2 * dwell * fs)
        sl = slice(t1 - max(tail, 50), t1)
        st = steady_stats(c, sl)
        st.update({
            "pass": label,
            "step": k,
            "id_ref": float(id_ref),
            "t_start": t0 / fs,
            "t_end": t1 / fs,
            "id_err": float(id_ref - st["id"]),
            "ud_res": st["ud"] - st["id"] * RS,
            "u_over_i": st["ud"] / st["id"] if abs(st["id"]) > 0.05 else float("nan"),
            "capture_ok": abs(id_ref - st["id"]) <= CAPTURE_EPS,
        })
        rows.append(st)
    return rows


def find_pass0_starts(c: dict[str, np.ndarray], fs: float) -> tuple[int, int]:
    """Return (pass0a_start, pass0b_start) by theta cluster after align."""
    n = len(c["id"])
    theta = c["theta"]
    # pass0a: first init_hold ends ~0.5s after align at 30deg
    for i in range(int(0.5 * fs), int(20 * fs)):
        sl = slice(i, i + int(0.3 * fs))
        if np.std(theta[sl]) < 0.01 and np.mean(np.abs(theta[sl] - TH30) < 0.02) > 0.95:
            id0 = np.nanmean(c["id"][slice(i, i + int(0.45 * fs))])
            if 0.02 < id0 < 0.12:
                pass0a = i
                break
    else:
        pass0a = int(1.0 * fs)

    # pass0b: first stretch at theta~0 after mid-file transition
    pass0b = None
    for i in range(int(15 * fs), int(30 * fs)):
        sl = slice(i, i + int(0.3 * fs))
        if np.std(theta[sl]) < 0.01 and np.mean(np.abs(theta[sl]) < 0.05) > 0.95:
            id0 = np.nanmean(c["id"][slice(i, i + int(0.45 * fs))])
            if 0.02 < id0 < 0.12:
                pass0b = i
                break
    if pass0b is None:
        pass0b = int(20 * fs)
    return pass0a, pass0b


def lut_lookup(amps: np.ndarray, vals: np.ndarray, id_a: float) -> float:
    if id_a <= amps[0]:
        return float(vals[0])
    if id_a >= amps[-1]:
        return float(vals[-1])
    j = int(np.searchsorted(amps, id_a))
    a0, a1 = amps[j - 1], amps[j]
    v0, v1 = vals[j - 1], vals[j]
    t = (id_a - a0) / (a1 - a0) if a1 > a0 else 0.0
    return float(v0 + t * (v1 - v0))


def analyze_file(path: Path) -> dict:
    c = load(path)
    n = len(c["id"])
    fs = FS
    lut = parse_lut(c)
    if lut is None:
        raise ValueError("no LUT burst")

    pass0a, pass0b = find_pass0_starts(c, fs)
    pass1_start = find_pass1_start(c, lut, fs)

    p0a = analyze_pass_steps(c, pass0a, AMP_TABLE, DWELL_PASS0_A, fs, "Pass0-A@30")
    p0b = analyze_pass_steps(c, pass0b, AMP_TABLE, DWELL_PASS0_B, fs, "Pass0-B@0")
    p1 = analyze_pass_steps(c, pass1_start, PASS1_TABLE, DWELL_PASS1, fs, "Pass1@30+abc")

    return {
        "path": str(path),
        "name": path.name,
        "duration_s": n / fs,
        "lut": lut,
        "pass0a_start_s": pass0a / fs,
        "pass0b_start_s": pass0b / fs,
        "pass1_start_s": pass1_start / fs,
        "pass0a": p0a,
        "pass0b": p0b,
        "pass1": p1,
    }


def summarize_rows(rows: list[dict], lut: dict | None = None) -> None:
    print(f"\n| step | Id_ref | Id_fb | err | Iq | Ud | Ud_res | U/I | duty_dev | θ_std | ok |")
    print(f"|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|:---:|")
    for r in rows:
        lut_v = ""
        if lut is not None and r["pass"].startswith("Pass0"):
            lv = lut_lookup(lut["amps"], lut["vals"], r["id_ref"])
            lut_v = f" lut={lv:.3f}"
        ok = "Y" if r["capture_ok"] else "**N**"
        print(
            f"| {r['step']} | {r['id_ref']:.3f} | {r['id']:.3f} | {r['id_err']:+.3f} | "
            f"{r['iq']:.3f} | {r['ud']:.3f} | {r['ud_res']:.3f} | "
            f"{r['u_over_i']:.2f} | {r['duty_dev_mean']:.3f} | {r['theta_std']:.4f} | {ok} |"
            + lut_v
        )


def diagnosis(r: dict) -> list[str]:
    lines = []
    p0 = r["pass0a"] + r["pass0b"]
    p1 = r["pass1"]
    lut = r["lut"]

    # Pass0 capture quality
    bad_p0 = [x for x in p0 if not x["capture_ok"]]
    out_p0 = [x for x in p0 if abs(x["ud_res"]) > 1.0]
    lines.append(f"Pass0 capture: {len(p0)-len(bad_p0)}/{len(p0)} within ±{CAPTURE_EPS}A; "
                 f"{len(out_p0)} Ud_res>|1V|")

    # Compare Pass0-A vs Pass0-B Ud_res at same id_ref
    for ref in [0.3, 0.5, 1.0, 1.5]:
        a = next((x for x in r["pass0a"] if abs(x["id_ref"] - ref) < 0.02), None)
        b = next((x for x in r["pass0b"] if abs(x["id_ref"] - ref) < 0.02), None)
        if a and b:
            d = a["ud_res"] - b["ud_res"]
            lines.append(f"  Pass0 Ud_res @Id={ref:.2f}A: 30°={a['ud_res']:.3f}V 0°={b['ud_res']:.3f}V Δ={d:+.3f}V")

    # Pass1 tracking
    bad_p1 = [x for x in p1 if not x["capture_ok"]]
    hi_p1 = [x for x in p1 if x["id_ref"] >= 1.5]
    bad_hi = [x for x in hi_p1 if not x["capture_ok"]]
    lines.append(f"Pass1 abc duty: {len(p1)-len(bad_p1)}/{len(p1)} Id track ok; "
                 f"high Id≥1.5A: {len(hi_p1)-len(bad_hi)}/{len(hi_p1)} ok")

    # Pass1 vs Pass0 at overlap amps
    for ref in [0.5, 1.0, 1.5, 2.0, 2.5, 3.0]:
        c0 = next((x for x in r["pass0a"] if abs(x["id_ref"] - ref) < 0.02), None)
        c1 = next((x for x in p1 if abs(x["id_ref"] - ref) < 0.02), None)
        if c0 and c1:
            lines.append(
                f"  Id={ref:.2f}A Pass0 Ud={c0['ud']:.2f} Ud_res={c0['ud_res']:.2f} | "
                f"Pass1 Ud={c1['ud']:.2f} Id_fb={c1['id']:.2f} err={c1['id_err']:+.2f} "
                f"Iq={c1['iq']:.3f} duty_dev={c1['duty_dev_mean']:.3f}"
            )

    # 3A deep dive
    top = next((x for x in p1 if abs(x["id_ref"] - 3.0) < 0.02), None)
    if top:
        lines.append(
            f"Pass1 @3A: Id_fb={top['id']:.3f} err={top['id_err']:+.3f} "
            f"Ud={top['ud']:.2f} Uq={top['uq']:.2f} Iq={top['iq']:.3f} "
            f"i_rms={top['i_rms']:.3f} i_rms_max={top['i_rms_max']:.3f} "
            f"U/I={top['u_over_i']:.2f}Ω (Rs={RS}) duty_dev={top['duty_dev_mean']:.3f}"
        )
        lv = lut_lookup(lut["amps"], lut["vals"], 3.0)
        lines.append(f"  LUT d-table @3A (extrap): {lv:.3f}V — Pass1 uses phase abc table, not Ud inject")

    # Plant gain trend Pass1
    hi = [x for x in p1 if x["id_ref"] >= 0.5 and x["id"] > 0.1]
    if len(hi) >= 3:
        ui = np.array([x["u_over_i"] for x in hi if not math.isnan(x["u_over_i"])])
        lines.append(f"Pass1 U/I spread (Id≥0.5A): min={ui.min():.2f} med={np.median(ui):.2f} "
                     f"max={ui.max():.2f} Ω vs Rs={RS}")

    iq_p1 = [abs(x["iq"]) for x in p1 if x["id_ref"] >= 1.0]
    if iq_p1:
        lines.append(f"Pass1 |Iq| @Id≥1A: mean={np.mean(iq_p1):.3f} max={max(iq_p1):.3f} "
                     f"(锁轴应≈0，偏大说明 abc duty / 相序 / 表过补)")

    return lines


def write_md(r: dict, out: Path) -> None:
    lut = r["lut"]
    lines = [
        f"# Id 注入段分析报告 — `{r['name']}`",
        "",
        f"- 时长: {r['duration_s']:.1f} s",
        f"- LUT 突发: t={lut['t_idx']/FS:.2f} s, len={lut['len']}, Rs_hdr={lut['rs_hdr']:.3f}",
        f"- Pass0-A start: t={r['pass0a_start_s']:.2f} s",
        f"- Pass0-B start: t={r['pass0b_start_s']:.2f} s",
        f"- Pass1 start: t={r['pass1_start_s']:.2f} s",
        "",
        "## LUT d 表（Pass0 capture 产物）",
        "",
        "| Id | Ud_res |",
        "|---:|---:|",
    ]
    for a, v in zip(lut["amps"], lut["vals"]):
        lines.append(f"| {a:.4f} | {v:.4f} |")

    for title, key in [("Pass0-A @30°", "pass0a"), ("Pass0-B @0°", "pass0b"), ("Pass1 @30° abc duty", "pass1")]:
        lines += ["", f"## {title}", ""]
        rows = r[key]
        lines.append("| step | Id_ref | Id | err | Iq | Ud | Ud_res | U/I | duty_dev | ok |")
        lines.append("|---:|---:|---:|---:|---:|---:|---:|---:|---:|:---:|")
        for x in rows:
            ok = "Y" if x["capture_ok"] else "N"
            lines.append(
                f"| {x['step']} | {x['id_ref']:.3f} | {x['id']:.3f} | {x['id_err']:+.3f} | "
                f"{x['iq']:.3f} | {x['ud']:.3f} | {x['ud_res']:.3f} | {x['u_over_i']:.2f} | "
                f"{x['duty_dev_mean']:.3f} | {ok} |"
            )

    lines += ["", "## 诊断摘要", ""]
    for ln in diagnosis(r):
        lines.append(f"- {ln}")

    out.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", type=Path)
    ap.add_argument("--md", type=Path, default=None)
    args = ap.parse_args()

    r = analyze_file(args.csv)
    lut = r["lut"]
    print(f"File: {r['name']}  LUT @ t={lut['t_idx']/FS:.2f}s  len={lut['len']}")
    print(f"Pass0-A t={r['pass0a_start_s']:.2f}  Pass0-B t={r['pass0b_start_s']:.2f}  Pass1 t={r['pass1_start_s']:.2f}")

    print("\n=== LUT d-table (first/last 5) ===")
    for i in list(range(5)) + list(range(lut["len"] - 5, lut["len"])):
        print(f"  Id={lut['amps'][i]:.4f}  Ud_res={lut['vals'][i]:.4f}")

    for title, key in [("Pass0-A@30", "pass0a"), ("Pass0-B@0", "pass0b"), ("Pass1@30+abc", "pass1")]:
        print(f"\n=== {title} ===")
        summarize_rows(r[key], lut if key.startswith("pass0") else None)

    print("\n=== DIAGNOSIS ===")
    for ln in diagnosis(r):
        print(f"  {ln}")

    if args.md:
        write_md(r, args.md)
        print(f"\nWrote {args.md}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
