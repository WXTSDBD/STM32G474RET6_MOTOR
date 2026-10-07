#!/usr/bin/env python3
"""Batch summary for BODE_OFF_ONLY CSV trio."""
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "tools" / "ident"))

from analyze_iq_bode import analyze  # noqa: E402

FILES = [
    "vofa+202607062340.csv",
    "vofa+202607062344.csv",
    "vofa+202607062345.csv",
]


def main() -> int:
    lines = [
        "# BODE_OFF_ONLY 三联录波汇总 (ch5 对齐)",
        "",
        "| 文件 | 时长 | OFF-lo f−3dB 实测/理论 | f可靠至 | OFF-hi f−3dB | f可靠至 | G@10 lo/hi |",
        "|------|------|----------------------|---------|--------------|---------|------------|",
    ]
    for fn in FILES:
        r = analyze(ROOT / "VOFA+CSV" / "20260706" / fn, fs=20000.0)
        lo = r["segments"].get("OFF-lo", {})
        hi = r["segments"].get("OFF-hi", {})

        def fmax(lbl: str) -> float:
            rows = r["all_rows"].get(lbl, [])
            return max((f for f, _ in rows), default=0.0)

        def f3(s: dict) -> str:
            tag = s.get("f3db_tag", "")
            v = s.get("f3db_hz", float("nan"))
            th = s.get("f3db_theory_hz", float("nan"))
            if tag == "n/a" or v != v:
                return f"n/a/{th:.0f}" if th == th else "n/a"
            if tag == "":
                return f"{v:.0f}/{th:.0f}"
            return f"{tag}{v:.0f}"

        f_rel = lo.get("f_reliable_max_hz", 0)

        rs = " / ".join(f"{x:.1f}" for x in r["round_starts_s"])
        tag = fn.replace("vofa+", "").replace(".csv", "")
        lines.append(
            f"| {tag} | {r['duration_s']:.1f}s | "
            f"{f3(lo)} | {lo.get('f_reliable_max_hz', 0):.0f}Hz | "
            f"{f3(hi)} | {hi.get('f_reliable_max_hz', 0):.0f}Hz | "
            f"{lo.get('G_10', 0):.3f}/{hi.get('G_10', 0):.3f} |"
        )

    lines += [
        "",
        "## 结论",
        "",
        "- 三条均 ch5 扫至 **1515 Hz**（48 点固件正常）",
        "- **f−3dB 实测**：可靠频段（至 ~670–710 Hz）内未过 −3 dB → 报 **n/a**",
        "- **f−3dB 理论**（PI@1000Hz, Lq×ωc）：≈ **1000 Hz**",
        "- **>700 Hz bin** 相对理论偏高 / 非单调 → 脚本标 ⚠，不参与 f−3dB",
        "- **10–600 Hz**：G@10 ≈ 1.0，|G| 多数 0.92–1.03",
        "",
    ]
    out = ROOT / "VOFA+CSV" / "20260706" / "分析报告_BODE_OFF_ONLY_三联汇总.md"
    out.write_text("\n".join(lines), encoding="utf-8")
    print("\n".join(lines))
    print(f"\nWrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
