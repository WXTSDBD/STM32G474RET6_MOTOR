#!/usr/bin/env python3
"""
VOFA 离线分析统一入口 — 单次读 CSV，复用 vofa_analyze_lib。

子命令:
  iq-steady   Pass0 后 Iq 稳态 OFF/FIXED/LUT 对比（abc_pp、Iq 跟踪、逐秒）
  iq-rotate   Iq 旋转探路：同速配对 OFF vs LUT + 判级（推荐旋转工况）
  iq-step     阶跃响应（委托 tools/ident/analyze_iq_step.py）
  iq-bode     Bode 扫频（委托 tools/ident/analyze_iq_bode.py）

示例:
  python tools/analyze_vofa.py iq-steady VOFA+CSV/.../vofa+xxx.csv
  python tools/analyze_vofa.py iq-steady file.csv --iq-ref 0.3 --off-s 20 --md-out report.md
  python tools/analyze_vofa.py iq-steady file.csv --segment-mode open_seq
  python tools/analyze_vofa.py iq-step file.csv --md report.md
"""
from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"
sys.path.insert(0, str(TOOLS))

from vofa_analyze_lib import (  # noqa: E402
    IqProbeConfig,
    RotateAnalyzeConfig,
    analyze_iq_rotate,
    analyze_iq_steady,
    format_iq_rotate_report,
    format_iq_steady_report,
)


def _cmd_iq_steady(args: argparse.Namespace) -> int:
    cfg = IqProbeConfig(
        iq_ref_a=args.iq_ref,
        off_s=args.off_s,
        fixed_s=args.fixed_s,
        off_trim_head_s=args.off_trim_head,
        off_trim_tail_s=args.off_trim_tail,
        lut_trim_head_s=args.lut_trim_head,
    )
    t0 = time.perf_counter()
    rep = analyze_iq_steady(
        args.csv,
        cfg,
        fs=args.fs,
        segment_mode=args.segment_mode,
        per_second=not args.no_per_second,
    )
    elapsed = time.perf_counter() - t0
    text = format_iq_steady_report(rep, markdown=not args.plain)
    text += f"\n\n<!-- analyzed in {elapsed:.2f}s -->\n" if not args.plain else f"\n(analyzed in {elapsed:.2f}s)\n"

    if args.md_out:
        args.md_out.write_text(text, encoding="utf-8")
        print(f"Wrote {args.md_out} ({elapsed:.2f}s)")
    else:
        print(text)
        if args.verbose:
            print(
                f"[{rep.path.name}] {rep.duration_s:.1f}s @ {rep.fs:.0f}Hz, "
                f"mode={rep.segment_mode}, {elapsed:.2f}s",
                file=sys.stderr,
            )
    return 0


def _cmd_iq_rotate(args: argparse.Namespace) -> int:
    cfg = RotateAnalyzeConfig(
        iq_ref_a=args.iq_ref,
        off_s=args.off_s,
        fixed_s=args.fixed_s,
        rpm_match_tol=args.rpm_match_tol,
        rpm_min=args.rpm_min,
        rpm_max=args.rpm_max,
    )
    t0 = time.perf_counter()
    rep = analyze_iq_rotate(
        args.csv,
        cfg,
        fs=args.fs,
        segment_mode=args.segment_mode,
    )
    elapsed = time.perf_counter() - t0
    text = format_iq_rotate_report(rep, markdown=not args.plain)
    text += f"\n\n<!-- analyzed in {elapsed:.2f}s -->\n" if not args.plain else f"\n(analyzed in {elapsed:.2f}s)\n"

    if args.md_out:
        args.md_out.write_text(text, encoding="utf-8")
        print(f"Wrote {args.md_out} ({elapsed:.2f}s)")
    else:
        print(text)
        if args.verbose:
            v = rep.verdict
            grade = v.grade if v else "?"
            print(
                f"[{rep.path.name}] {grade} pairs={v.n_pairs if v else 0} {elapsed:.2f}s",
                file=sys.stderr,
            )
    return 0


def _delegate(script: Path, csv: Path, extra: list[str]) -> int:
    cmd = [sys.executable, str(script), str(csv), *extra]
    return subprocess.call(cmd, cwd=str(ROOT))


def _cmd_iq_step(args: argparse.Namespace) -> int:
    extra = []
    if args.md:
        extra.extend(["--md", str(args.md)])
    return _delegate(TOOLS / "ident" / "analyze_iq_step.py", args.csv, extra)


def _cmd_iq_bode(args: argparse.Namespace) -> int:
    extra = []
    if args.md:
        extra.extend(["--md", str(args.md)])
    return _delegate(TOOLS / "ident" / "analyze_iq_bode.py", args.csv, extra)


def main() -> int:
    ap = argparse.ArgumentParser(description="VOFA unified offline analysis")
    sub = ap.add_subparsers(dest="command", required=True)

    p_steady = sub.add_parser("iq-steady", help="Iq steady OFF/LUT comparison after Pass0")
    p_steady.add_argument("csv", type=Path)
    p_steady.add_argument("--fs", type=float, default=None, help="sample rate Hz (auto if omitted)")
    p_steady.add_argument("--iq-ref", type=float, default=0.3, help="expected Iq (A)")
    p_steady.add_argument("--off-s", type=float, default=20.0, help="deadband OFF duration (s)")
    p_steady.add_argument("--fixed-s", type=float, default=0.0, help="FIXED segment duration (s)")
    p_steady.add_argument(
        "--segment-mode",
        choices=("auto", "open_seq", "time"),
        default="auto",
        help="auto: open_seq 50/51/53 else time fallback",
    )
    p_steady.add_argument("--off-trim-head", type=float, default=1.0)
    p_steady.add_argument("--off-trim-tail", type=float, default=1.5)
    p_steady.add_argument("--lut-trim-head", type=float, default=0.5)
    p_steady.add_argument("--no-per-second", action="store_true")
    p_steady.add_argument("--md-out", type=Path, default=None)
    p_steady.add_argument("--plain", action="store_true", help="plain text instead of markdown")
    p_steady.add_argument("-v", "--verbose", action="store_true")
    p_steady.set_defaults(func=_cmd_iq_steady)

    p_rot = sub.add_parser("iq-rotate", help="Iq rotating probe: matched-speed OFF vs LUT")
    p_rot.add_argument("csv", type=Path)
    p_rot.add_argument("--fs", type=float, default=None)
    p_rot.add_argument("--iq-ref", type=float, default=0.3)
    p_rot.add_argument("--off-s", type=float, default=20.0)
    p_rot.add_argument("--fixed-s", type=float, default=0.0)
    p_rot.add_argument("--segment-mode", choices=("auto", "open_seq", "time"), default="auto")
    p_rot.add_argument("--rpm-match-tol", type=float, default=15.0)
    p_rot.add_argument("--rpm-min", type=float, default=80.0)
    p_rot.add_argument("--rpm-max", type=float, default=4000.0)
    p_rot.add_argument("--md-out", type=Path, default=None)
    p_rot.add_argument("--plain", action="store_true")
    p_rot.add_argument("-v", "--verbose", action="store_true")
    p_rot.set_defaults(func=_cmd_iq_rotate)

    p_step = sub.add_parser("iq-step", help="Iq step response (legacy ident script)")
    p_step.add_argument("csv", type=Path)
    p_step.add_argument("--md", type=Path, default=None)
    p_step.set_defaults(func=_cmd_iq_step)

    p_bode = sub.add_parser("iq-bode", help="Iq Bode sweep (legacy ident script)")
    p_bode.add_argument("csv", type=Path)
    p_bode.add_argument("--md", type=Path, default=None)
    p_bode.set_defaults(func=_cmd_iq_bode)

    args = ap.parse_args()
    if not args.csv.is_file():
        print(f"error: not found: {args.csv}", file=sys.stderr)
        return 1
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
