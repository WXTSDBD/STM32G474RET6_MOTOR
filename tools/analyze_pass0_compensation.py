#!/usr/bin/env python3
"""
Pass0（第一段 Id 扫表，deadband OFF）补偿电压与稳态分析。

公式（与 deadband_cal.c 一致）：
  Ud_res = Ud_pi − Id_fb × Rs
  应补 d 轴电压幅值 = |Ud_res|
  commit 后 phase 表 val ≈ |Ud_res| × 0.866

稳态判据（对齐固件 capture + 分析指南）：
  - dwell 时长 ≥ 0.35 s
  - dwell 末 20%：|Id_fb − Id_ref| < 0.03 A（capture OK）
  - dwell 末 20%：Id_std < 0.04 A，Ud_pi_std < 0.15 V
  - dwell 末 20%：θ 档内漂移 < 5°
  - 可选：dwell 前半 vs 末 20% Ud_pi 变化 < 0.2 V（PI 已收敛）

用法:
  python tools/analyze_pass0_compensation.py --latest
  python tools/analyze_pass0_compensation.py VOFA+CSV/20260625/vofa+202606260819.csv
  python tools/analyze_pass0_compensation.py file.csv --md -o report.md
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
    D_TO_PHASE_COS,
    LUT_HDR,
    LUT_HDR_TOL,
    RS_OHM,
    load_csv,
    park,
    unwrap,
)

FS = 20000.0
VBUS = 24.0
DEADTIME_NS = 591
V_COMP_FIXED = VBUS * DEADTIME_NS * 1e-9 * FS  # M1_DEADBAND_V_COMP_V

CAPTURE_EPS_A = 0.03
ID_STD_MAX_A = 0.04
UD_STD_MAX_V = 0.15
THETA_MAX_DEG = 5.0
DWELL_MIN_S = 0.35
UD_SETTLE_MAX_V = 0.20
TAIL_FRAC = 0.20


def valid_mask(ia: np.ndarray, ib: np.ndarray, ic: np.ndarray) -> np.ndarray:
    return (np.abs(ia) < 20) & (np.abs(ib) < 20) & (np.abs(ic) < 20)


def find_lut_row(ia: np.ndarray) -> int:
    for i, v in enumerate(ia):
        if abs(v - LUT_HDR) < LUT_HDR_TOL:
            return i
    return -1


def find_latest_csv(root: Path) -> Path:
    skip = ("processed", "offline", "deadband_processed")
    cands = [
        p
        for p in root.rglob("vofa+*.csv")
        if not any(s in p.name for s in skip)
    ]
    if not cands:
        raise FileNotFoundError("no vofa+*.csv under repo")
    return max(cands, key=lambda p: p.stat().st_mtime)


def detect_id_dwells(
    id_ref: np.ndarray, t_end: int, min_step: float = 0.01, min_dwell_s: float = 0.2
) -> list[tuple[int, int, float]]:
    """Return (start_idx, end_idx, id_ref_value) for Pass0 plateaus."""
    segments: list[tuple[int, int, float]] = []
    i = 0
    while i < t_end:
        ref = id_ref[i]
        if ref < 0.04:
            i += 1
            continue
        j = i + 1
        while j < t_end and abs(id_ref[j] - ref) < 0.008:
            j += 1
        if (j - i) / FS >= min_dwell_s:
            val = float(np.median(id_ref[i : min(i + int(0.05 * FS), j)]))
            segments.append((i, j, val))
        i = j

    merged: list[tuple[int, int, float]] = []
    for seg in segments:
        if merged and abs(seg[2] - merged[-1][2]) < min_step:
            merged[-1] = (merged[-1][0], seg[1], seg[2])
        else:
            merged.append(seg)
    return merged


@dataclass
class StepResult:
    id_ref: float
    t_start: float
    t_end: float
    dwell_s: float
    id_fb: float
    id_err: float
    id_std: float
    ud_pi: float
    ud_std: float
    ud_res: float
    ud_comp_abs: float
    phase_comp_v: float
    iq_rms: float
    theta_span_deg: float
    abc_pp: float
    capture_ok: bool
    steady_ok: bool
    steady_notes: list[str]
    ud_early: float
    ud_late: float


def analyze_tail(
    ia, ib, ic, ud, id_ref, theta, sl: slice, ref: float
) -> dict:
    th = theta[sl]
    id_, iq = park(ia[sl], ib[sl], ic[sl], th)
    id_mean = float(np.mean(id_))
    ud_s = ud[sl]
    ia_s, ib_s, ic_s = ia[sl], ib[sl], ic[sl]
    return {
        "id_fb": id_mean,
        "id_err": ref - id_mean,
        "id_std": float(np.std(id_)),
        "ud_pi": float(np.mean(ud_s)),
        "ud_std": float(np.std(ud_s)),
        "ud_res": float(np.mean(ud_s) - id_mean * RS_OHM),
        "iq_rms": float(np.sqrt(np.mean(iq**2))),
        "theta_span_deg": math.degrees(float(np.max(th) - np.min(th))),
        "abc_pp": float(np.mean([np.ptp(ia_s), np.ptp(ib_s), np.ptp(ic_s)])),
    }


def assess_steady(
    dwell_s: float,
    id_err: float,
    id_std: float,
    ud_std: float,
    theta_deg: float,
    ud_early: float,
    ud_late: float,
) -> tuple[bool, list[str]]:
    notes: list[str] = []
    ok = True

    if dwell_s < DWELL_MIN_S:
        ok = False
        notes.append(f"dwell={dwell_s:.2f}s<{DWELL_MIN_S}s")
    if abs(id_err) > CAPTURE_EPS_A:
        ok = False
        notes.append(f"|Id_err|={abs(id_err):.3f}>{CAPTURE_EPS_A}A")
    if id_std > ID_STD_MAX_A:
        ok = False
        notes.append(f"Id_std={id_std:.3f}>{ID_STD_MAX_A}A")
    if ud_std > UD_STD_MAX_V:
        ok = False
        notes.append(f"Ud_std={ud_std:.3f}>{UD_STD_MAX_V}V")
    if theta_deg > THETA_MAX_DEG:
        ok = False
        notes.append(f"θ漂移={theta_deg:.1f}°>{THETA_MAX_DEG}°")
    if abs(ud_late - ud_early) > UD_SETTLE_MAX_V:
        ok = False
        notes.append(f"Ud前后差={abs(ud_late-ud_early):.2f}>{UD_SETTLE_MAX_V}V")

    if ok:
        notes.append("稳态 OK")
    return ok, notes


def analyze_pass0(path: Path, rs: float = RS_OHM) -> dict:
    ia, ib, ic, ud, id_ref, theta_raw = load_csv(path)
    theta = unwrap(theta_raw)
    n = len(ia)
    fs = FS if n > 100_000 else 200.0
    t = np.arange(n) / fs
    v = valid_mask(ia, ib, ic)

    lut_i = find_lut_row(ia)
    pass0_end = lut_i if lut_i >= 0 else n
    lut_t = pass0_end / fs

    dwells = detect_id_dwells(id_ref, pass0_end)
    steps: list[StepResult] = []

    for start, end, ref in dwells:
        span = end - start
        tail_n = max(int(span * TAIL_FRAC), 1)
        early_n = max(int(span * 0.5), 1)
        tail_sl = slice(end - tail_n, end)
        early_sl = slice(start, start + early_n)

        m_tail = np.zeros(n, dtype=bool)
        m_tail[tail_sl] = v[tail_sl]
        m_early = np.zeros(n, dtype=bool)
        m_early[early_sl] = v[early_sl]

        if m_tail.sum() < 100:
            continue

        tail = analyze_tail(ia, ib, ic, ud, id_ref, theta, tail_sl, ref)
        ud_early = float(np.mean(ud[early_sl])) if m_early.sum() > 50 else tail["ud_pi"]

        capture_ok = abs(tail["id_err"]) <= CAPTURE_EPS_A
        steady_ok, notes = assess_steady(
            (end - start) / fs,
            tail["id_err"],
            tail["id_std"],
            tail["ud_std"],
            tail["theta_span_deg"],
            ud_early,
            tail["ud_pi"],
        )

        ud_res = tail["ud_res"]
        steps.append(
            StepResult(
                id_ref=ref,
                t_start=t[start],
                t_end=t[end - 1],
                dwell_s=(end - start) / fs,
                id_fb=tail["id_fb"],
                id_err=tail["id_err"],
                id_std=tail["id_std"],
                ud_pi=tail["ud_pi"],
                ud_std=tail["ud_std"],
                ud_res=ud_res,
                ud_comp_abs=abs(ud_res),
                phase_comp_v=abs(ud_res) * D_TO_PHASE_COS,
                iq_rms=tail["iq_rms"],
                theta_span_deg=tail["theta_span_deg"],
                abc_pp=tail["abc_pp"],
                capture_ok=capture_ok,
                steady_ok=steady_ok,
                steady_notes=notes,
                ud_early=ud_early,
                ud_late=tail["ud_pi"],
            )
        )

    n_cap_ok = sum(1 for s in steps if s.capture_ok)
    n_steady = sum(1 for s in steps if s.steady_ok)

    return {
        "path": path,
        "fs": fs,
        "pass0_end_row": pass0_end,
        "lut_t_s": lut_t,
        "n_steps": len(steps),
        "n_capture_ok": n_cap_ok,
        "n_steady_ok": n_steady,
        "rs": rs,
        "v_comp_fixed": V_COMP_FIXED,
        "steps": steps,
    }


def format_text(r: dict) -> str:
    p = r["path"]
    lines = [
        f"=== Pass0 补偿分析 — {p.name} ===",
        f"Pass0 段: t=0 ~ {r['lut_t_s']:.2f}s（LUT 前，行 0~{r['pass0_end_row']}）",
        f"Rs={r['rs']} Ω  固定死区参考={r['v_comp_fixed']:.4f} V/相",
        f"档数={r['n_steps']}  capture_OK={r['n_capture_ok']}  稳态_OK={r['n_steady_ok']}",
        "",
        f"{'Id_ref':>7} {'Id_fb':>7} {'err':>7} {'Ud_pi':>7} {'Ud_res':>7} "
        f"{'|Ud_res|':>7} {'phase':>7} {'abc_pp':>7} {'稳态':>5} {'capture':>7} 备注",
        "-" * 105,
    ]
    for s in r["steps"]:
        st = "OK" if s.steady_ok else "NO"
        cap = "OK" if s.capture_ok else "FAIL"
        note = "; ".join(s.steady_notes)
        lines.append(
            f"{s.id_ref:7.3f} {s.id_fb:7.3f} {s.id_err:+7.3f} {s.ud_pi:7.3f} "
            f"{s.ud_res:7.3f} {s.ud_comp_abs:7.3f} {s.phase_comp_v:7.3f} "
            f"{s.abc_pp:7.3f} {st:>5} {cap:>7}  {note}"
        )

    lines += ["", "说明:"]
    lines.append("  |Ud_res| = 应补 d 轴电压（capture 写入 LUT 的 d 域 val）")
    lines.append("  phase    = |Ud_res|×0.866（commit 后 phase abc 表 val）")
    lines.append(f"  固定死区 M1_DEADBAND_V_COMP_V ≈ {r['v_comp_fixed']:.3f} V（仅大电流区参考）")
    if r["n_steady_ok"] < r["n_steps"]:
        bad = [s for s in r["steps"] if not s.steady_ok]
        lines.append(f"  [!] {len(bad)} 档未达稳态，补偿电压仅供参考")
    return "\n".join(lines)


def format_md(r: dict) -> str:
    p = r["path"]
    lines = [
        f"# Pass0 补偿与稳态 — `{p.name}`",
        "",
        f"| 项 | 值 |",
        f"|----|-----|",
        f"| Pass0 结束 | t≈{r['lut_t_s']:.2f} s（行 {r['pass0_end_row']}） |",
        f"| Rs | {r['rs']} Ω |",
        f"| 固定死区参考 | {r['v_comp_fixed']:.4f} V/相 |",
        f"| 档数 | {r['n_steps']} |",
        f"| capture OK | {r['n_capture_ok']}/{r['n_steps']} |",
        f"| 稳态 OK | {r['n_steady_ok']}/{r['n_steps']} |",
        "",
        "## 应补电压（dwell 末 20%）",
        "",
        "`Ud_res = Ud_pi − Id×Rs`；`|Ud_res|` 为 d 轴 capture；`phase` = ×0.866。",
        "",
        "| Id_ref | Id_fb | err(A) | Ud_pi | Ud_res | **\\|Ud_res\\|** | phase val | abc_pp | 稳态 | capture |",
        "|--------|-------|--------|-------|--------|-------------|-----------|--------|------|---------|",
    ]
    for s in r["steps"]:
        st = "✅" if s.steady_ok else "❌"
        cap = "OK" if s.capture_ok else "FAIL"
        lines.append(
            f"| {s.id_ref:.3f} | {s.id_fb:.3f} | {s.id_err:+.3f} | {s.ud_pi:.3f} | "
            f"{s.ud_res:.3f} | **{s.ud_comp_abs:.3f}** | {s.phase_comp_v:.3f} | "
            f"{s.abc_pp:.3f} | {st} | {cap} |"
        )
    lines += ["", "## 稳态判据", ""]
    lines += [
        f"- dwell ≥ {DWELL_MIN_S} s",
        f"- 末 20% \\|Id−Id_ref\\| ≤ {CAPTURE_EPS_A} A",
        f"- Id_std ≤ {ID_STD_MAX_A} A，Ud_pi_std ≤ {UD_STD_MAX_V} V",
        f"- θ 档内 ≤ {THETA_MAX_DEG}°",
        f"- dwell 前半 vs 末 20% Ud_pi 差 ≤ {UD_SETTLE_MAX_V} V",
        "",
    ]
    for s in r["steps"]:
        if not s.steady_ok:
            lines.append(f"- Id_ref={s.id_ref:.3f} A：{'; '.join(s.steady_notes)}")
    return "\n".join(lines)


def main() -> int:
    ap = argparse.ArgumentParser(description="Pass0 compensation & steady-state analysis")
    ap.add_argument("csv", nargs="?", type=Path, help="VOFA CSV path")
    ap.add_argument("--latest", action="store_true", help="use newest vofa+*.csv in repo")
    ap.add_argument("--rs", type=float, default=RS_OHM)
    ap.add_argument("--md", action="store_true")
    ap.add_argument("-o", "--output", type=Path)
    args = ap.parse_args()

    if args.latest or args.csv is None:
        path = find_latest_csv(ROOT)
        print(f"latest: {path}", file=sys.stderr)
    else:
        path = args.csv.resolve()

    result = analyze_pass0(path, args.rs)
    text = format_md(result) if args.md else format_text(result)
    if args.output:
        args.output.write_text(text, encoding="utf-8")
        print(f"wrote {args.output}", file=sys.stderr)
    elif args.md:
        sys.stdout.reconfigure(encoding="utf-8")  # type: ignore[attr-defined]
        print(text)
    else:
        print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
