#!/usr/bin/env python3
"""堵转 Iq 辨识录波离线摘要（open_seq 60–63）。

用法:
  python tools/ident/analyze_iq_ident.py VOFA+CSV/xxx.csv

通道（M1_IDENT_ENABLE）：ch3=Iq_fb ch4=Iq_ref ch5=Uq_pi
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))

from analyze_id_cal_sweep_vofa import load_csv, park, unwrap  # noqa: E402

FS = 20000.0
SEG = {60: "HOLD", 61: "STEP", 62: "BODE", 63: "DONE"}


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    path = Path(sys.argv[1])
    ia, ib, ic, uq, iq_ref_ch, theta = load_csv(path)
    t = np.arange(len(ia)) / FS
    id_p, iq_p = park(ia, ib, ic, theta)

    # open_seq 不在 CSV：用 Iq_ref 阶跃 + Uq 正弦启发式分段
    iq_ref = iq_ref_ch
    print(f"# {path.name} — Iq ident 离线摘要\n")
    print(f"时长 {t[-1]:.2f} s  样本 {len(t)}\n")

    for thr, name in [(0.05, "Iq_ref≈0"), (0.25, "Iq_ref≈0.3"), (0.45, "Iq_ref≈0.5")]:
        m = np.abs(iq_ref - thr) < 0.08
        if int(m.sum()) < 500:
            continue
        print(
            f"## {name} 窗 (n={int(m.sum())})"
            f"\n  Iq mean={np.mean(iq_p[m]):+.3f} std={np.std(iq_p[m]):.4f}"
            f"  |Iq-Iref|={np.mean(np.abs(iq_p[m]-iq_ref[m])):.4f}"
            f"  Uq std={np.std(uq[m]):.4f}\n"
        )

    # Bode 粗看：Uq/Iq 在 ref 正弦段
    m_sin = (np.abs(iq_ref) > 0.15) & (np.abs(iq_ref) < 0.4) & (np.std(np.diff(iq_ref)) > 1e-4)
    if int(m_sin.sum()) > 2000:
        print("## 疑似 BODE 段（Iq_ref 小幅波动）")
        print(f"  样本 {int(m_sin.sum())}  Iq std={np.std(iq_p[m_sin]):.4f}  Uq std={np.std(uq[m_sin]):.4f}")
        print("  → 用 MATLAB/Python tfestimate(iq_ref→iq) 或 iq_ref→uq 求 Bode\n")

    print("提示：Watch `dbg.open_seq_phase` 60=HOLD 61=STEP 62=BODE 63=DONE")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
