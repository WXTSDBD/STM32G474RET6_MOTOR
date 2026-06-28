#!/usr/bin/env python3
"""Iq Bode 闭环频响 + 带宽 / 裕度（由 Gcl=Iq/Iq_ref 反推 L=Gcl/(1-Gcl)）。"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))

from analyze_id_cal_sweep_vofa import load_csv  # noqa: E402

FS = 20000.0
TS = 50e-6
CYCLES = 8.0
RATIO = 1.15
F1 = 800.0


def ticks_from_s(s: float) -> int:
    return int(s / TS + 0.5)


def freq_list() -> list[float]:
    f, out = 10.0, []
    while f <= F1:
        out.append(f)
        f *= RATIO
    return out


FREQS = freq_list()
SWEEP_S = sum(CYCLES / f for f in FREQS)


def find_bode_start(ch4: np.ndarray, after: int) -> int:
    w = int(0.2 * FS)
    for i in range(after + int(0.5 * FS), len(ch4) - w, int(0.02 * FS)):
        seg = ch4[i : i + w]
        if seg.mean() < 0.15 or seg.mean() > 0.35:
            continue
        if np.std(seg) < 0.01:
            continue
        if np.max(np.abs(np.diff(seg))) < 0.001:
            continue
        return i
    raise RuntimeError("Bode start not found")


def analyze_segment(
    ch3: np.ndarray,
    ch4: np.ndarray,
    ch5: np.ndarray,
    ia: np.ndarray,
    ib: np.ndarray,
    ic: np.ndarray,
    a: int,
    b: int,
) -> dict:
    tick = a
    rows: list[dict] = []
    for f in FREQS:
        nw = ticks_from_s(CYCLES / f)
        i0, i1 = tick, tick + nw
        if i1 > b:
            break
        tt = np.arange(i1 - i0) / FS
        w = 2 * np.pi * f
        ref = ch4[i0:i1] - 0.25
        c, s = np.cos(w * tt), np.sin(w * tt)
        R = 2 * np.mean(ref * c) + 1j * 2 * np.mean(ref * s)
        if abs(R) < 1e-8:
            tick = i1
            continue
        I = 2 * np.mean(ch3[i0:i1] * c) + 1j * 2 * np.mean(ch3[i0:i1] * s)
        U = 2 * np.mean(ch5[i0:i1] * c) + 1j * 2 * np.mean(ch5[i0:i1] * s)
        G = I / R
        rows.append(
            {
                "f": f,
                "mag": abs(G),
                "ph": float(np.angle(G, deg=True)),
                "G": G,
                "U_mag": abs(U / R),
                "err": float(np.sqrt(np.mean((ch3[i0:i1] - ch4[i0:i1]) ** 2))),
            }
        )
        tick = i1

    if not rows:
        return {}

    freqs = np.array([r["f"] for r in rows])
    mags = np.array([r["mag"] for r in rows])
    phs = np.array([r["ph"] for r in rows])

    Lmag = np.full(len(rows), np.nan)
    Lph = np.full(len(rows), np.nan)
    for i, r in enumerate(rows):
        g = r["G"]
        if abs(1.0 - g) > 0.08:
            L = g / (1.0 - g)
            Lmag[i] = abs(L)
            Lph[i] = float(np.angle(L, deg=True))

    bw3 = float("nan")
    for i in range(len(mags) - 1):
        if mags[i] >= 0.707 and mags[i + 1] < 0.707:
            t = (0.707 - mags[i]) / (mags[i + 1] - mags[i])
            bw3 = freqs[i] + t * (freqs[i + 1] - freqs[i])
            break
    else:
        if mags[-1] >= 0.707:
            bw3 = float(freqs[-1])

    wc = float("nan")
    pm = float("nan")
    valid = np.isfinite(Lmag)
    idx = np.where(valid)[0]
    for j in range(len(idx) - 1):
        i0, i1 = idx[j], idx[j + 1]
        if Lmag[i0] >= 1.0 and Lmag[i1] < 1.0:
            t = (1.0 - Lmag[i0]) / (Lmag[i1] - Lmag[i0])
            wc = freqs[i0] + t * (freqs[i1] - freqs[i0])
            phx = Lph[i0] + t * (Lph[i1] - Lph[i0])
            pm = 180.0 + phx
            break

    gm = float("nan")
    for i in range(len(Lph) - 1):
        if not (valid[i] and valid[i + 1]):
            continue
        if Lph[i] > -180.0 and Lph[i + 1] <= -180.0:
            t = (-180.0 - Lph[i]) / (Lph[i + 1] - Lph[i])
            gmx = Lmag[i] + t * (Lmag[i + 1] - Lmag[i])
            if gmx > 1e-6:
                gm = 1.0 / gmx
            break

    return {
        "n": len(rows),
        "f_last": rows[-1]["f"],
        "bw3": bw3,
        "wc": wc,
        "pm": pm,
        "gm": gm,
        "err_mean": float(np.mean([r["err"] for r in rows]) * 1000),
        "rows": rows,
    }


def main() -> int:
    path = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "VOFA+CSV/20260627/vofa+202606281851.csv"
    ia, ib, ic, ch3, ch4, ch5 = load_csv(path)
    id_end = int(np.where(ch4 > 2.0)[0][-1])
    start = find_bode_start(ch4, id_end)
    done = next(
        i for i in range(int(25 * FS), len(ch4)) if ch4[i] == 0 and ch4[i - 1] > 0.15
    )

    dur = (done - start) / FS
    print(f"=== {path.name} 扫频验收 ===")
    print(f"Bode {start/FS:.2f} - {done/FS:.2f} s  有效 {dur:.2f} s / 理论 {3*SWEEP_S:.2f} s")
    pct = dur / (3 * SWEEP_S) * 100
    ok = "OK" if pct >= 95 else ("PARTIAL" if pct >= 85 else "FAIL")
    split_mode = "6.063s/轮"
    print(f"完成度 {pct:.1f}%  [{ok}]  DONE 期望 ~{start/FS + 3*SWEEP_S:.1f} s")
    print(f"分段：{split_mode}\n")

    round_ticks = int(SWEEP_S * FS + 0.5)
    splits = [
        (start + k * round_ticks, min(start + (k + 1) * round_ticks, done))
        for k in range(3)
    ]

    print("| 组 | 频点 | f_max | 闭环-3dB | f_c | PM | GM | err |")
    print("|----|------|-------|----------|-----|-----|-----|-----|")
    results: dict[str, dict] = {}
    for lbl, (a, b) in zip(["OFF", "FIXED", "LUT"], splits):
        r = analyze_segment(ch3, ch4, ch5, ia, ib, ic, a, b)
        results[lbl] = r
        pm_s = f"{r['pm']:.0f} deg" if np.isfinite(r.get("pm", np.nan)) else "—"
        gm_s = f"{r['gm']:.2f}" if np.isfinite(r.get("gm", np.nan)) else "—"
        wc_s = f"{r['wc']:.0f} Hz" if np.isfinite(r.get("wc", np.nan)) else "—"
        bw_s = f"{r['bw3']:.0f} Hz" if np.isfinite(r.get("bw3", np.nan)) else "—"
        print(
            f"| {lbl} | {r['n']}/32 | {r['f_last']:.0f} Hz | {bw_s} | {wc_s} | {pm_s} | {gm_s} | {r['err_mean']:.1f} mA |"
        )

    print("\n说明：")
    print("- Gcl=Iq/Iq_ref（闭环跟踪）；L=Gcl/(1-Gcl) 反推开环（单位负反馈近似）")
    print("- PM/GM 由 L 估算，高频/|Gcl|→1 时不可靠；以 -3dB 带宽为主指标")
    print()

    for lbl in ["OFF", "FIXED", "LUT"]:
        r = results[lbl]
        print(f"--- {lbl} 代表频点 |Gcl| / phase ---")
        for row in r["rows"][::5]:
            print(f"  {row['f']:6.0f} Hz  |G|={row['mag']:.3f}  phase={row['ph']:+6.1f} deg")
        last = r["rows"][-1]
        print(f"  (末点 {last['f']:.0f} Hz  |G|={last['mag']:.3f}  phase={last['ph']:+.1f} deg)\n")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
