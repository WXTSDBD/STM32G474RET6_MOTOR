#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""有感位置短表回归门（架构改动日常用）。

用途：改 PWM / 回调 / 优先级等「声称不改行为」的刀之后，烧录有感签收档，
录一份 VOFA 12ch CSV，跑本脚本看过不过。不是 P3 九项全签收。

通道（M1_VOFA_SIGNOFF_CH=1）：
  ch0..2 Ia Ib Ic | ch3 Iq | ch4 th_fb | ch5 th_enc | ch6 th_ref
  ch7 th_err | ch8 w_pll | ch9 w_des | ch10 Iq_ref | ch11 seq

硬门（抗 JustFloat 毛刺，用分位数，不用全局 max）：
  1) 出现 seq=255，且不出现 seq=252
  2) 出现 MIT 正反切段（230/231），编码器行程 |Δenc| ≥ 20°
  3) MREV |θ_err| p99 ≤ 15°
  4) 全程 |Iq_ref| p99 ≤ 5 A，|ω_pll| p99 ≤ 300 rpm
  5) 保持段 seq=10 存在时，|θ_err| p99 ≤ 3°

用法：
  python tools/regress_sensed_short.py
  python tools/regress_sensed_short.py VOFA+CSV/20261007/vofa+202610071955.csv
  python tools/regress_sensed_short.py --latest
  python tools/regress_sensed_short.py --latest --json out.json

退出码：0=PASS，1=FAIL，2=用法/读文件错误。
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
VOFA_ROOT = REPO / "VOFA+CSV"

DONE_SEQ = 255
GUARD_SEQ = 252
RAD2DEG = 180.0 / np.pi


def find_latest_csv() -> Path | None:
    if not VOFA_ROOT.is_dir():
        return None
    cands = list(VOFA_ROOT.rglob("vofa+*.csv"))
    if not cands:
        return None
    return max(cands, key=lambda p: p.stat().st_mtime)


def _strip_layout_id(data: np.ndarray) -> np.ndarray:
    """兼容旧 13 列 CSV：末列像 open_seq 时剥掉首列 layout_id。"""
    if data.ndim != 2 or data.shape[1] < 13:
        return data
    last = data[:, -1]
    last_i = np.rint(last)
    seq_like = (
        float(np.nanmax(np.abs(last - last_i))) < 0.25
        and float(np.nanmax(last_i)) <= 255.0
        and float(np.nanmin(last_i)) >= 0.0
        and bool(np.any(last_i == 255) or np.any(last_i >= 200))
    )
    c0 = data[:, 0]
    c0_i = np.rint(c0)
    # 多数为小整数 layout_id（允许少量 JustFloat 毛刺）
    frac_ok = float(np.mean(np.abs(c0 - c0_i) < 0.25)) >= 0.99
    in_range = float(np.mean((c0_i >= 0) & (c0_i <= 31))) >= 0.99
    if seq_like and frac_ok and in_range:
        return data[:, 1:13]
    if data.shape[1] == 13 and seq_like:
        return data[:, 1:13]
    return data


def load_csv(path: Path) -> np.ndarray:
    # 首行可能是 I0..I11；T-1 起可能多一列 layout_id
    data = np.loadtxt(path, delimiter=",", skiprows=1)
    if data.ndim != 2 or data.shape[1] < 12:
        raise ValueError(f"need ≥12 columns, got shape {getattr(data, 'shape', None)}")
    data = _strip_layout_id(data)
    if data.shape[1] < 12:
        raise ValueError(f"need ≥12 data columns after layout_id strip, got {data.shape}")
    return data[:, :12]


def pct(a: np.ndarray, q: float) -> float:
    if a.size == 0:
        return float("nan")
    return float(np.percentile(a, q))


def analyze(data: np.ndarray) -> dict:
    th_fb = data[:, 4] * RAD2DEG
    th_enc = data[:, 5] * RAD2DEG
    th_ref = data[:, 6] * RAD2DEG
    th_err = data[:, 7] * RAD2DEG
    w_pll = data[:, 8]
    iq_ref = data[:, 10]
    seq = data[:, 11]

    # seq 偶发浮点毛刺：只认接近整数的标记
    seq_i = np.rint(seq).astype(np.int32)
    seq_ok = np.abs(seq - seq_i) < 0.25
    seq_clean = seq_i.copy()
    seq_clean[~seq_ok] = -999

    has_255 = bool(np.any(seq_clean == DONE_SEQ))
    has_252 = bool(np.any(seq_clean == GUARD_SEQ))
    hold = seq_clean == 10
    mrev = (seq_clean == 230) | (seq_clean == 231)

    mrev_enc_travel = 0.0
    mrev_err_p99 = float("nan")
    if np.any(mrev):
        enc = th_enc[mrev]
        mrev_enc_travel = float(enc[-1] - enc[0])
        # 多段正反切时用峰峰值更稳
        mrev_enc_travel = float(np.max(enc) - np.min(enc))
        mrev_err_p99 = pct(np.abs(th_err[mrev]), 99.0)

    hold_err_p99 = pct(np.abs(th_err[hold]), 99.0) if np.any(hold) else float("nan")
    iq_p99 = pct(np.abs(iq_ref), 99.0)
    w_p99 = pct(np.abs(w_pll), 99.0)
    n_err_gt90 = int(np.sum(np.abs(th_err) > 90.0))
    n_iq_gt11 = int(np.sum(np.abs(iq_ref) > 11.0))

    checks = []
    checks.append(("done_255", has_255, "seq=255 present"))
    checks.append(("no_guard_252", not has_252, "seq=252 absent"))
    checks.append(("mrev_present", bool(np.any(mrev)), "seq 230/231 present"))
    checks.append(
        (
            "mrev_enc_travel",
            abs(mrev_enc_travel) >= 20.0 if np.any(mrev) else False,
            f"|Δenc|_pp on MREV ≥ 20° (got {abs(mrev_enc_travel):.2f})",
        )
    )
    checks.append(
        (
            "mrev_err_p99",
            (mrev_err_p99 <= 15.0) if np.isfinite(mrev_err_p99) else False,
            f"MREV |θ_err| p99 ≤ 15° (got {mrev_err_p99:.2f})",
        )
    )
    checks.append(
        (
            "iq_ref_p99",
            iq_p99 <= 5.0,
            f"|Iq_ref| p99 ≤ 5 A (got {iq_p99:.3f})",
        )
    )
    checks.append(
        (
            "w_pll_p99",
            w_p99 <= 300.0,
            f"|ω_pll| p99 ≤ 300 rpm (got {w_p99:.1f})",
        )
    )
    if np.any(hold):
        checks.append(
            (
                "hold_err_p99",
                hold_err_p99 <= 3.0,
                f"hold |θ_err| p99 ≤ 3° (got {hold_err_p99:.2f})",
            )
        )

    failed = [c for c in checks if not c[1]]
    passed = len(failed) == 0

    return {
        "PASS": passed,
        "n": int(data.shape[0]),
        "duration_s_at_2khz": float(data.shape[0] / 2000.0),
        "has_255": has_255,
        "has_252": has_252,
        "mrev_enc_travel_pp_deg": abs(mrev_enc_travel),
        "mrev_err_p99_deg": mrev_err_p99,
        "hold_err_p99_deg": hold_err_p99,
        "iq_ref_p99_a": iq_p99,
        "w_pll_p99_rpm": w_p99,
        "spike_n_err_gt90": n_err_gt90,
        "spike_n_iq_gt11": n_iq_gt11,
        "final_seq": int(seq_clean[-1]) if seq_clean.size else None,
        "checks": [
            {"id": i, "ok": ok, "detail": detail} for i, ok, detail in checks
        ],
        "failed": [i for i, ok, _ in failed],
    }


def main() -> int:
    ap = argparse.ArgumentParser(description="Sensed-pos short-table regression gate")
    ap.add_argument(
        "csv",
        nargs="?",
        help="VOFA CSV path; default with --latest is newest under VOFA+CSV/",
    )
    ap.add_argument(
        "--latest",
        action="store_true",
        help="use newest vofa+*.csv under VOFA+CSV/",
    )
    ap.add_argument("--json", type=Path, help="write full report JSON here")
    args = ap.parse_args()

    if args.csv:
        path = Path(args.csv)
        if not path.is_file():
            # allow repo-relative
            alt = REPO / args.csv
            path = alt if alt.is_file() else path
    elif args.latest or args.csv is None:
        path = find_latest_csv()
        if path is None:
            print("FAIL: no vofa+*.csv under VOFA+CSV/", file=sys.stderr)
            return 2
    else:
        ap.print_help()
        return 2

    if not path.is_file():
        print(f"FAIL: file not found: {path}", file=sys.stderr)
        return 2

    try:
        data = load_csv(path)
        report = analyze(data)
    except Exception as exc:  # noqa: BLE001 — gate script, show any load/parse error
        print(f"FAIL: {exc}", file=sys.stderr)
        return 2

    report["csv"] = str(path.resolve())
    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(
            json.dumps(report, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )

    verdict = "PASS" if report["PASS"] else "FAIL"
    print(f"{verdict}  {path.name}")
    print(
        f"  n={report['n']}  ~{report['duration_s_at_2khz']:.1f}s@2kHz  "
        f"255={report['has_255']} 252={report['has_252']}  "
        f"MREV|Δenc|pp={report['mrev_enc_travel_pp_deg']:.1f}°  "
        f"MREV|err|p99={report['mrev_err_p99_deg']:.2f}°  "
        f"|Iq|p99={report['iq_ref_p99_a']:.3f}A  "
        f"|ω|p99={report['w_pll_p99_rpm']:.1f}rpm"
    )
    for c in report["checks"]:
        mark = "OK " if c["ok"] else "BAD"
        print(f"  [{mark}] {c['detail']}")
    if args.json:
        print(f"  json → {args.json}")

    return 0 if report["PASS"] else 1


if __name__ == "__main__":
    sys.exit(main())
