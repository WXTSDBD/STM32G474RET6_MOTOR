#!/usr/bin/env python3
"""扇区诊断 CSV 专用统计（ch4=sector）并写 .txt 结论。"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

FS = 200.0
IDLE_S = 10.0
IDLE_SKIP = 1.0
RUN_SKIP = 2.0
I_SIG = 0.05

RUN_NOTES: dict[str, str] = {
    "vofa+202606172341(uq2,sector_dial).csv": (
        "binding [2,1,0]+[-1,-1,-1]；add=2.10；Uq=2V；gain 1/1/1；"
        "recon OFF；deadband ON；M1_VOFA_SECTOR_DIAG=1"
    ),
    "vofa+202606172357(uq2,sector_dial).csv": (
        "binding 同上；gain 1/1.15/1.50；recon ON（扇区1-2 ic）；sector_diag"
    ),
    "vofa+202606180010(uq2,sector_dial,nogain).csv": (
        "binding 同上；gain 1/1/1；recon ON（**仅扇区1-2 ic**）；deadband ON；sector_diag"
    ),
    "vofa+202606180026(uq2,sector_dial,nogain,KCLall).csv": (
        "binding 同上；gain 1/1/1；recon ON（**全6扇区 KCL**）；deadband ON；sector_diag"
    ),
    "vofa+202606180057(uq2,sector_dial,nogain,exangerank02).csv": (
        "**作废**：adc rank 已交换但 binding 仍为 [2,1,0]→ia/ic 对调；勿当 rank A/B"
    ),
    "vofa+202606180104.csv": (
        "M1_ADC_RANK_SWAP_IAIC=1；binding [0,1,2]+[-1,-1,-1]；"
        "JDR1=PC4(ia) JDR3=PC3(ic)；recon OFF；Uq=2V；sector_diag"
    ),
    "vofa+202606180117.csv": (
        "TIM8 CCR4=998；rank0 binding [2,1,0]；recon ON 1～2；"
        "sector_diag；Uq=2V；**无效：run 段 ch0-2≈0 LSB，Iq≈0**"
    ),
    "vofa+202606180121.csv": (
        "TIM8 CCR4=2998；rank0 binding [2,1,0]；recon ON 1～2；"
        "sector_diag；Uq=2V；ADC 有效，Iq<0 与 172341 同量级"
    ),
    "rankswap_b012": (
        "M1_ADC_RANK_SWAP_IAIC=1；binding [0,1,2]+[-1,-1,-1]；"
        "JDR1=PC4(ia) JDR3=PC3(ic)；recon OFF；Uq=2V；sector_diag"
    ),
}

# ch0–2 物理 rank 与逻辑相（VOFA 始终打物理 JDR，不含 binding 重排）
BINDING_RANK012 = (
    "**binding [0,1,2]+[-1,-1,-1]（rank 交换试验）：** "
    "ch0/JDR1→**ia**(PC4)，ch1/JDR2→**ib**(PA0)，ch2/JDR3→**ic**(PC3)"
)
BINDING_RANK210 = (
    "**binding [2,1,0]+[-1,-1,-1]（默认）：** "
    "ch0/JDR1→**ic**(PC3)，ch1/JDR2→**ib**(PA0)，ch2/JDR3→**ia**(PC4)"
)


def binding_note_for(path: Path) -> str:
    name = path.name.lower()
    note = RUN_NOTES.get(path.name, "")
    if "rankswap" in name or "b012" in name or path.name == "vofa+202606180104.csv":
        return BINDING_RANK012
    if "exangerank" in name or "180057" in name:
        return (
            "**误录：adc rank 已交换但 binding 仍为 [2,1,0]** — "
            "ch0/JDR1 物理 PC4 却被标成 ic；ch2/JDR3 物理 PC3 却被标成 ia"
        )
    if note and "RANK_SWAP" in note or "[0,1,2]" in note:
        return BINDING_RANK012
    return BINDING_RANK210


def load(path: Path):
    d = np.genfromtxt(path, delimiter=",", names=True)
    n = d.dtype.names
    return d[n[0]], d[n[1]], d[n[2]], d[n[3]], d[n[4]], d[n[5]]


def wrap_deg(th_rad: np.ndarray) -> np.ndarray:
    return (np.degrees(th_rad) % 360.0 + 360.0) % 360.0


def iq_neg_pct(iq: np.ndarray, mask: np.ndarray) -> float:
    m = mask & (np.abs(iq) > I_SIG)
    if m.sum() == 0:
        return 0.0
    return 100.0 * float((iq[m] < 0).mean())


def rpm_from_theta(theta: np.ndarray) -> float:
    th_u = np.empty_like(theta)
    th_u[0] = theta[0]
    for i in range(1, len(theta)):
        d = theta[i] - theta[i - 1]
        if d > np.pi:
            d -= 2.0 * np.pi
        elif d < -np.pi:
            d += 2.0 * np.pi
        th_u[i] = th_u[i - 1] + d
    dt = (len(theta) - 1) / FS
    if dt <= 0:
        return 0.0
    return abs(th_u[-1] - th_u[0]) / (2.0 * np.pi * 7.0) / (dt / 60.0)


def segment_stats(ia, ib, ic, iq, sector, theta, t_start: float, t_end: float, label: str) -> dict:
    ra = float(np.sqrt(np.mean(ia ** 2)))
    rb = float(np.sqrt(np.mean(ib ** 2)))
    rc = float(np.sqrt(np.mean(ic ** 2)))
    kcl = float(np.sqrt(np.mean((ia + ib + ic) ** 2)))
    avg = (ra + rb + rc) / 3.0
    th_deg = wrap_deg(theta)
    neg_all = iq_neg_pct(iq, np.ones(len(iq), bool))
    neg_sig = iq_neg_pct(iq, np.abs(iq) > I_SIG)
    neg280 = iq_neg_pct(iq, (th_deg >= 280) & (th_deg < 350))
    neg100 = iq_neg_pct(iq, (th_deg >= 100) & (th_deg < 190))
    rpm = rpm_from_theta(theta)
    iq_rms = float(np.sqrt(np.mean(iq ** 2)))
    sec = np.round(sector).astype(int)
    sectors = []
    for s in range(1, 7):
        m = sec == s
        if m.sum() == 0:
            continue
        iq_s = iq[m]
        sectors.append({
            "s": s,
            "n": int(m.sum()),
            "mean": float(iq_s.mean()),
            "rms": float(np.sqrt(np.mean(iq_s ** 2))),
            "neg": iq_neg_pct(iq, m),
        })
    return {
        "label": label,
        "t_start": t_start,
        "t_end": t_end,
        "n": len(iq),
        "ra": ra,
        "rb": rb,
        "rc": rc,
        "kcl": kcl,
        "kcl_pct": 100.0 * kcl / avg if avg > 0 else 0.0,
        "ib_over_ch0": rb / ra if ra > 0 else 0.0,
        "ch2_over_ch0": rc / ra if ra > 0 else 0.0,
        "iq_rms": iq_rms,
        "iq_mean": float(iq.mean()),
        "neg_all": neg_all,
        "neg_sig": neg_sig,
        "neg280": neg280,
        "neg100": neg100,
        "rpm": rpm,
        "sectors": sectors,
    }


def analyze_file(path: Path) -> tuple[dict | None, dict]:
    ia, ib, ic, iq, sector, theta = load(path)
    n = len(iq)
    seg_n = int(IDLE_S * FS)
    idle_start = int(IDLE_SKIP * FS)
    run_start = seg_n + int(RUN_SKIP * FS)
    idle = None
    if seg_n > idle_start + 50 and run_start < n - 100:
        idle = segment_stats(
            ia[idle_start:seg_n], ib[idle_start:seg_n], ic[idle_start:seg_n],
            iq[idle_start:seg_n], sector[idle_start:seg_n], theta[idle_start:seg_n],
            idle_start / FS, seg_n / FS, "IDLE",
        )
        run = segment_stats(
            ia[run_start:], ib[run_start:], ic[run_start:],
            iq[run_start:], sector[run_start:], theta[run_start:],
            run_start / FS, n / FS, "RUN",
        )
    else:
        run = segment_stats(ia, ib, ic, iq, sector, theta, 0.0, n / FS, "RUN")
    return idle, run


def write_txt(path: Path, idle: dict | None, run: dict, compare: list[tuple[str, dict]] | None = None) -> Path:
    txt = path.with_suffix(".txt")
    note = RUN_NOTES.get(path.name, "")
    lines = [
        f"# VOFA 扇区诊断结论 — {path.name}",
        "# 工具：tools/analyze_sector_dial.py",
        "# 分析用 add：**2.10 rad**",
        f"# 双段 split @ {IDLE_S:.1f}s (M1_OPEN_IDLE_S)",
    ]
    if note:
        lines.append(f"# 录制条件：{note}")
    lines.extend([
        "",
        "## VOFA 六通道（扇区诊断模式）",
        "",
        "| ch | 含义 |",
        "|----|------|",
        "| ch0–2 | `adc_zeroed` LSB（raw−offset，**不含 gain**） |",
        "| ch3 | **Iq (A)**（binding + gain + 可选扇区重构后） |",
        "| ch4 | **SVPWM 扇区 1～6** |",
        "| ch5 | θ_el (rad) |",
        "",
        binding_note_for(path),
        "",
    ])
    for seg in ([idle] if idle else []) + [run]:
        if seg is None:
            continue
        lines.extend([
            f"## {seg['label']}  t=[{seg['t_start']:.1f}, {seg['t_end']:.1f}]s  n={seg['n']}",
            f"- RMS ch0/ch1/ch2 = {seg['ra']:.2f} / {seg['rb']:.2f} / {seg['rc']:.2f} LSB",
            f"- ch1/ch0 = {seg['ib_over_ch0']:.3f}   ch2/ch0 = {seg['ch2_over_ch0']:.3f}",
            f"- KCL |sum| RMS = {seg['kcl']:.2f} LSB ({seg['kcl_pct']:.1f}% of avg)",
            f"- **Iq rms = {seg['iq_rms']:.3f} A**  mean = {seg['iq_mean']:.3f} A",
        ])
        if seg["label"] == "RUN":
            lines.extend([
                f"- **Iq<0 占比: {seg['neg_sig']:.1f}%**  (全样本: {seg['neg_all']:.1f}%)",
                f"- θ280–350° Iq<0: {seg['neg280']:.1f}%  θ100–190°: {seg['neg100']:.1f}%",
                f"- rpm ≈ **{seg['rpm']:.0f}**",
                "",
                "## 按 SVPWM 扇区统计 Iq（run 段）",
                "",
                "| 扇区 | n | Iq 均值 (A) | Iq rms (A) | Iq<0 % |",
                "|------|---|-------------|------------|--------|",
            ])
            for row in seg["sectors"]:
                lines.append(
                    f"| {row['s']} | {row['n']} | {row['mean']:+.2f} | "
                    f"{row['rms']:.2f} | {row['neg']:.1f}% |"
                )
            lines.append("")
    if compare:
        lines.extend([
            "## 与历史 run 段对比",
            "",
            "| 文件 | Iq<0% | θ280–350 | 扇区1 Iq<0% | 扇区2 | Iq rms | rpm |",
            "|------|-------|----------|-------------|-------|--------|-----|",
        ])
        for name, r in compare:
            s1 = next((x["neg"] for x in r["sectors"] if x["s"] == 1), 0.0)
            s2 = next((x["neg"] for x in r["sectors"] if x["s"] == 2), 0.0)
            lines.append(
                f"| {name} | {r['neg_sig']:.1f}% | {r['neg280']:.1f}% | "
                f"{s1:.1f}% | {s2:.1f}% | {r['iq_rms']:.2f} A | {r['rpm']:.0f} |"
            )
        lines.append("")
    txt.write_text("\n".join(lines), encoding="utf-8")
    return txt


def print_run(label: str, run: dict) -> None:
    print("=" * 72)
    print(label)
    print(f"RUN t=[{run['t_start']:.1f}, {run['t_end']:.1f}]s  n={run['n']}  rpm~{run['rpm']:.0f}")
    print(f"RMS ch0/1/2 = {run['ra']:.1f}/{run['rb']:.1f}/{run['rc']:.1f} LSB")
    print(f"Iq rms={run['iq_rms']:.3f} A  Iq<0%={run['neg_sig']:.1f}%  θ280-350={run['neg280']:.1f}%")


def main() -> None:
    ap = argparse.ArgumentParser(description="扇区诊断 VOFA CSV 分析")
    ap.add_argument("paths", nargs="*", type=Path, help="CSV 路径（默认 20260617 最新）")
    ap.add_argument("--compare", action="store_true", help="写入与 2341/2357 对比表")
    args = ap.parse_args()
    base = Path(__file__).resolve().parents[1] / "VOFA+CSV" / "20260617"
    if args.paths:
        paths = [p if p.is_absolute() else Path.cwd() / p for p in args.paths]
    else:
        paths = [base / "vofa+202606180010(uq2,sector_dial,nogain).csv"]

    compare_runs: list[tuple[str, dict]] = []
    ref_names = [
        ("172341 reconOFF", "vofa+202606172341(uq2,sector_dial).csv"),
        ("180010 recon12 ic", "vofa+202606180010(uq2,sector_dial,nogain).csv"),
        ("172357 gain115150 recon12", "vofa+202606172357(uq2,sector_dial).csv"),
    ]
    if args.compare:
        for name, fn in ref_names:
            p = base / fn
            if p.exists():
                _, r = analyze_file(p)
                compare_runs.append((name, r))

    for path in paths:
        if not path.exists():
            print(f"Missing: {path}", file=sys.stderr)
            sys.exit(1)
        idle, run = analyze_file(path)
        print_run(path.name, run)
        if args.compare:
            short = path.name.replace("vofa+", "").replace(".csv", "")
            compare_runs.append((short, run))
        txt = write_txt(path, idle, run, compare_runs if args.compare else None)
        print(f"Report: {txt}")


if __name__ == "__main__":
    main()
