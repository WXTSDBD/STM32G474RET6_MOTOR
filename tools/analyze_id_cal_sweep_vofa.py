#!/usr/bin/env python3
"""
Id 锁轴扫表 VOFA CSV 分析（段 1/2/3A 探路等）。

前提：M1_ID_LOCK_CAL_SWEEP=1 时 VOFA 通道
  ch0-2 = Ia, Ib, Ic (A)
  ch3   = Ud_pi (V)
  ch4   = Id_ref (A)
  ch5   = theta_el (rad)

用法:
  python tools/analyze_id_cal_sweep_vofa.py VOFA+CSV/20260624/vofa+202606242320.csv
  python tools/analyze_id_cal_sweep_vofa.py file.csv --md --report out.md
  python tools/analyze_id_cal_sweep_vofa.py file.csv --compare-lut
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

RS_OHM = 0.115
OUTLIER_V = 1.0
CAPTURE_EPS_A = 0.03
DEFAULT_FS = 20000.0
LUT_HDR = -888888.0
LUT_TAIL = -999999.0
LUT_HDR_TOL = 1.0
D_TO_PHASE_COS = 0.8660254037844386


def load_csv(path: Path) -> tuple[np.ndarray, ...]:
    data = np.genfromtxt(path, delimiter=",", names=True, dtype=float)
    names = data.dtype.names
    if names is None or len(names) < 6:
        raise ValueError(f"need 6 columns, got {names}")
    cols = [data[names[i]] for i in range(6)]
    return tuple(cols)


def infer_fs(n: int, fs_arg: float | None) -> float:
    if fs_arg is not None:
        return fs_arg
    return DEFAULT_FS if n > 100_000 else 200.0


def unwrap(theta: np.ndarray) -> np.ndarray:
    out = np.empty_like(theta)
    out[0] = theta[0]
    for i in range(1, len(theta)):
        d = theta[i] - theta[i - 1]
        if d > math.pi:
            d -= 2.0 * math.pi
        elif d < -math.pi:
            d += 2.0 * math.pi
        out[i] = out[i - 1] + d
    return out


def park(ia: np.ndarray, ib: np.ndarray, ic: np.ndarray, theta: np.ndarray):
    i_beta = (ib - ic) / math.sqrt(3.0)
    c, s = np.cos(theta), np.sin(theta)
    id_ = ia * c + i_beta * s
    iq = -ia * s + i_beta * c
    return id_, iq


def find_lut_header(rows: np.ndarray) -> int:
    for i, row in enumerate(rows):
        if abs(row[0] - LUT_HDR) < LUT_HDR_TOL:
            return i
    return -1


def parse_lut_burst(rows: np.ndarray) -> dict | None:
    hi = find_lut_header(rows)
    if hi < 0:
        return None

    hdr = rows[hi]
    length = int(round(hdr[1]))
    rs = float(hdr[2])
    outlier = int(round(hdr[3]))
    proto_ver = float(hdr[4])
    amps: list[float] = []
    vals: list[float] = []
    i = hi + 1
    tail_row = -1
    tail_sum = 0.0

    while i < len(rows):
        row = rows[i]
        if abs(row[0] - LUT_TAIL) < LUT_HDR_TOL:
            tail_row = i
            tail_sum = float(row[2])
            break
        if abs(row[0] - LUT_HDR) < LUT_HDR_TOL:
            raise ValueError(f"duplicate LUT header at row {i}")
        for k in (0, 2, 4):
            if len(amps) >= length:
                break
            a, v = float(row[k]), float(row[k + 1])
            if a > 0.0 or v > 0.0:
                amps.append(a)
                vals.append(v)
        i += 1

    if tail_row < 0:
        raise ValueError("LUT tail not found")
    if len(amps) != length:
        raise ValueError(f"LUT len mismatch: header {length}, got {len(amps)}")

    calc_sum = float(np.sum(vals))
    ch5 = float(hdr[5]) if len(hdr) > 5 else 0.0
    return {
        "header_row": hi,
        "tail_row": tail_row,
        "len": length,
        "rs": rs,
        "outlier": outlier,
        "proto_ver": proto_ver,
        "ch5": ch5,
        "amps": np.array(amps),
        "vals": np.array(vals),
        "tail_sum": tail_sum,
        "calc_sum": calc_sum,
    }


def detect_id_steps(id_ref: np.ndarray, fs: float, min_step_a: float = 0.02):
    """Return list of (start_idx, end_idx, id_ref_value)."""
    n = len(id_ref)
    change = np.abs(np.diff(id_ref, prepend=id_ref[0])) > min_step_a
    edges = np.where(change)[0]
    if len(edges) == 0:
        return [(0, n, float(id_ref[0]))]

    segments: list[tuple[int, int, float]] = []
    for k, start in enumerate(edges):
        end = edges[k + 1] if k + 1 < len(edges) else n
        val = float(np.median(id_ref[start : min(start + int(0.05 * fs), end)]))
        if end - start < int(0.1 * fs):
            continue
        segments.append((start, end, val))

    merged: list[tuple[int, int, float]] = []
    for seg in segments:
        if merged and abs(seg[2] - merged[-1][2]) < min_step_a:
            merged[-1] = (merged[-1][0], seg[1], seg[2])
        else:
            merged.append(seg)
    return merged


def segment_steady_slice(start: int, end: int, frac: float = 0.2) -> slice:
    span = end - start
    tail = max(int(span * frac), 1)
    s0 = end - tail
    return slice(s0, end)


def analyze_sweep(
    ia, ib, ic, ud_pi, id_ref, theta, fs: float, rs: float = RS_OHM
) -> dict:
    theta_u = unwrap(theta)
    id_fb, iq_fb = park(ia, ib, ic, theta_u)
    t = np.arange(len(ia)) / fs

    lut_hi = find_lut_header(np.column_stack([ia, ib, ic, ud_pi, id_ref, theta]))
    sweep_end = lut_hi if lut_hi >= 0 else len(ia)

    segments = detect_id_steps(id_ref[:sweep_end], fs)
    rows = []

    for start, end, ref in segments:
        if ref < 0.01:
            continue
        sl = segment_steady_slice(start, end, 0.2)
        id_mean = float(np.mean(id_fb[sl]))
        iq_rms = float(np.sqrt(np.mean(iq_fb[sl] ** 2)))
        ud_mean = float(np.mean(ud_pi[sl]))
        theta_sl = theta_u[sl]
        theta_span_deg = math.degrees(float(np.max(theta_sl) - np.min(theta_sl)))
        err = ref - id_mean
        ud_res = ud_mean - id_mean * rs
        outlier = abs(ud_res) > OUTLIER_V

        rows.append(
            {
                "t_start": float(t[start]),
                "t_end": float(t[end - 1]),
                "id_ref": ref,
                "id_fb": id_mean,
                "id_err": err,
                "iq_rms": iq_rms,
                "ud_pi": ud_mean,
                "ud_res": ud_res,
                "theta_span_deg": theta_span_deg,
                "outlier": outlier,
                "capture_ok": abs(err) <= CAPTURE_EPS_A,
            }
        )

    return {
        "fs": fs,
        "duration_s": len(ia) / fs,
        "sweep_end_row": sweep_end,
        "sweep_duration_s": sweep_end / fs,
        "segments": rows,
        "iq_global_rms": float(np.sqrt(np.mean(iq_fb[:sweep_end] ** 2))),
        "theta_full_span_deg": math.degrees(
            float(np.max(theta_u[:sweep_end]) - np.min(theta_u[:sweep_end]))
        ),
    }


def format_md_report(path: Path, result: dict, lut: dict | None) -> str:
    lines = [
        f"# Id 扫表自动分析 — `{path.name}`",
        "",
        "> 由 `tools/analyze_id_cal_sweep_vofa.py` 生成；详见 "
        "`VOFA+CSV/20260624/Id扫表与LUT标定_CSV分析指南_给AI.md`",
        "",
        "## 录波概况",
        "",
        f"| 项 | 值 |",
        f"|----|-----|",
        f"| 采样率 | {result['fs']:.0f} Hz |",
        f"| 总时长 | {result['duration_s']:.2f} s |",
        f"| 扫表段时长 | {result['sweep_duration_s']:.2f} s（行 0～{result['sweep_end_row']}） |",
        f"| Iq 全局 RMS（扫表段） | {result['iq_global_rms']:.4f} A |",
        f"| θ 全程跨度 | {result['theta_full_span_deg']:.1f}° |",
        "",
    ]

    if lut:
        try:
            from parse_lut_vofa import decode_proto, format_lut_burst_md

            ch5 = float(lut.get("ch5", 0.0))
            dec = decode_proto(lut.get("proto_ver", 3.0), ch5)
            lines += [
                "## LUT 遥测突发",
                "",
                f"| 项 | 值 |",
                f"|----|-----|",
                f"| 头帧行 | {lut['header_row']} |",
                f"| 尾帧行 | {lut['tail_row']} |",
                f"| len | {lut['len']} |",
                f"| Rs | {lut['rs']:.4f} Ω |",
                f"| outlier | {lut['outlier']} |",
                f"| proto | {lut.get('proto_ver', 2.0):.1f} |",
                f"| ch5 meta | {ch5:.4g} |",
                f"| 表含义 | {dec['proto_desc']} |",
                f"| runtime | {dec['runtime']} |",
                f"| sum(vals) | tail={lut['tail_sum']:.6f} calc={lut['calc_sum']:.6f} |",
                "",
                "表来源：Pass0 `|Ud_pi−Id·Rs|` capture → commit phase×0.866；"
                "突发 amp=|i_phase|、val=V/相。详见 `parse_lut_vofa.py --md-full`。",
                "",
            ]
        except ImportError:
            lines += [
                "## LUT 遥测突发",
                "",
                f"| 头帧行 | {lut['header_row']} |",
                f"| proto | {lut.get('proto_ver', 2.0):.0f} |",
                "",
            ]

    lines += [
        "## 各档稳态（dwell 末 20%）",
        "",
        "| Id_ref (A) | Id_fb (A) | err (A) | Ud_pi (V) | Ud_res (V) | Iq_rms | θ档内(°) | capture |",
        "|------------|-----------|---------|-----------|------------|--------|----------|---------|",
    ]
    for r in result["segments"]:
        cap = "OK" if r["capture_ok"] else "FAIL"
        flag = " **" if r["outlier"] else ""
        lines.append(
            f"| {r['id_ref']:.3f} | {r['id_fb']:.3f} | {r['id_err']:+.4f} | "
            f"{r['ud_pi']:.4f} |{flag} {r['ud_res']:.4f}{flag} | {r['iq_rms']:.4f} | "
            f"{r['theta_span_deg']:.2f} | {cap} |"
        )

    if lut and result["segments"]:
        proto = float(lut.get("proto_ver", 2.0))
        lut_amps = lut["amps"]
        lut_vals = lut["vals"]
        if proto >= 2.5:
            lut_amps = lut_amps / D_TO_PHASE_COS
            lut_vals = lut_vals / D_TO_PHASE_COS
        lines += ["", "## LUT vs CSV Ud_res 对照（d 轴域）", ""]
        lines += ["| Id 参考 | LUT val | CSV Ud_res | Δ (V) |", "|---------|---------|------------|-------|"]
        for ref_pick in [0.05, 0.15, 0.20, 0.50, 1.0, 1.5, 2.0, 3.0]:
            csv_row = min(result["segments"], key=lambda r: abs(r["id_ref"] - ref_pick), default=None)
            if csv_row is None or abs(csv_row["id_ref"] - ref_pick) > 0.08:
                continue
            lut_i = int(np.argmin(np.abs(lut_amps - csv_row["id_fb"])))
            lv = float(lut_vals[lut_i])
            cv = csv_row["ud_res"]
            lines.append(f"| {ref_pick:.2f} | {lv:.4f} | {cv:.4f} | {lv - cv:+.4f} |")

    lines.append("")
    return "\n".join(lines)


# --- Iq 探路时序（与 motor_params_m1.h ID_CAL_DUAL_FULL 默认一致）---
IQ_PROBE_OFF_S = 10.0
IQ_PROBE_FIXED_S = 5.0
IQ_PROBE_PASS0_TO_IQ_S = 1.0  # LUT 突发 + Pass0 decay 后进入 Iq


def _valid_mask(ia: np.ndarray) -> np.ndarray:
    return np.abs(ia) < 20.0


def find_pass1_end(
    t: np.ndarray,
    id_ref: np.ndarray,
    ia: np.ndarray,
    lut_t: float,
    fs: float = DEFAULT_FS,
    id_max: float = 3.0,
) -> float:
    v = _valid_mask(ia)
    idx = np.where(v & (t > lut_t + 1.0) & (np.abs(id_ref - id_max) < 0.02))[0]
    if len(idx) < 500:
        idx15 = np.where(v & (t > lut_t + 1.0) & (np.abs(id_ref - 1.5) < 0.01))[0]
        if len(idx15) < 500:
            return float(t[-1]) - 1.0
        idx = idx15
    br = np.where(np.diff(idx) > 50)[0]
    return float(idx[br[0]] / fs) if len(br) else float(idx[-1] / fs)


def recording_has_pass1(
    t: np.ndarray, id_ref: np.ndarray, ia: np.ndarray, lut_t: float, id_thresh: float = 1.8
) -> bool:
    if lut_t <= 0.0:
        return True
    m = _valid_mask(ia) & (t > lut_t + 1.0)
    return bool(np.any(np.abs(id_ref[m]) > id_thresh))


def find_iq_probe_start(
    t: np.ndarray,
    id_ref: np.ndarray,
    ia: np.ndarray,
    lut_t: float,
    fs: float = DEFAULT_FS,
) -> float:
    """Pass0→Iq 直切：LUT 后 ~1 s；legacy：Pass1 末档结束时刻。"""
    if recording_has_pass1(t, id_ref, ia, lut_t):
        return find_pass1_end(t, id_ref, ia, lut_t, fs)
    if lut_t > 0.0:
        return lut_t + IQ_PROBE_PASS0_TO_IQ_S
    return find_pass1_end(t, id_ref, ia, lut_t, fs)


def iq_probe_segment_times(
    iq_start: float,
    duration_s: float,
    off_s: float = IQ_PROBE_OFF_S,
    fixed_s: float = IQ_PROBE_FIXED_S,
) -> dict[str, float]:
    """返回 Iq 探路各段边界（off/fixed 为 0 时直切 LUT ON）。"""
    iq0 = iq_start + 0.5
    iq_off_end = iq0 + off_s
    iq_fixed_end = iq_off_end + fixed_s
    iq_on_end = duration_s - 0.5
    lut_on_start = iq_fixed_end if fixed_s > 0.0 else (iq_off_end if off_s > 0.0 else iq0)
    return {
        "iq0": iq0,
        "iq_off_end": iq_off_end,
        "iq_fixed_end": iq_fixed_end,
        "iq_on_end": iq_on_end,
        "lut_on_start": lut_on_start,
    }


def main() -> int:
    ap = argparse.ArgumentParser(description="Analyze Id lock cal sweep VOFA CSV")
    ap.add_argument("csv", type=Path)
    ap.add_argument("--fs", type=float, default=None, help="sample rate Hz (default 20k if N>100k)")
    ap.add_argument("--rs", type=float, default=RS_OHM)
    ap.add_argument("--md", action="store_true", help="print markdown")
    ap.add_argument("--report", type=Path, default=None, help="write markdown report path")
    ap.add_argument("--compare-lut", action="store_true", help="parse LUT burst if present")
    args = ap.parse_args()

    ia, ib, ic, ud_pi, id_ref, theta = load_csv(args.csv)
    fs = infer_fs(len(ia), args.fs)
    rows = np.column_stack([ia, ib, ic, ud_pi, id_ref, theta])

    result = analyze_sweep(ia, ib, ic, ud_pi, id_ref, theta, fs, args.rs)
    lut = None
    if args.compare_lut or find_lut_header(rows) >= 0:
        try:
            lut = parse_lut_burst(rows)
        except ValueError as e:
            print(f"LUT parse warn: {e}", file=sys.stderr)

    md = format_md_report(args.csv, result, lut)

    if args.report:
        args.report.write_text(md, encoding="utf-8")
        print(f"wrote {args.report}")
    if args.md or not args.report:
        print(md)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
