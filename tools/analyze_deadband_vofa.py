#!/usr/bin/env python3
"""
Deadband / three-phase VOFA CSV analyzer.

Focus: Ia/Ib/Ic waveform quality (clamp, THD proxy, zero-cross), not only Iq/Id.
Optional A/B compare (OFF vs ON) and auto markdown report next to CSV.

Usage:
  python tools/analyze_deadband_vofa.py VOFA+CSV/20260623/vofa+202606232139.csv
  python tools/analyze_deadband_vofa.py off.csv --compare on.csv
  python tools/analyze_deadband_vofa.py file.csv --export-processed --report
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable

import numpy as np

POLE_PAIRS = 7
DEFAULT_FS_HZ = 200.0
IQ_NEG_THRESH_A = 0.05
CLAMP_I_MIN_A = 0.06
CLAMP_IPEAK_MIN_A = 0.12
NEAR_ZERO_A = 0.08
FLAT_WIN_MS = 2.0
FLAT_STD_A = 0.02


def load_csv(path: Path):
    data = np.genfromtxt(path, delimiter=",", names=True, dtype=float)
    names = data.dtype.names
    if names is None or len(names) < 6:
        raise ValueError(f"Expected >=6 columns, got {names}")
    return (
        data[names[0]],
        data[names[1]],
        data[names[2]],
        data[names[3]],
        data[names[4]],
        data[names[5]],
    )


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


def park_offline(ia, ib, ic, theta):
    i_beta = (ib - ic) / math.sqrt(3.0)
    c, s = np.cos(theta), np.sin(theta)
    id_ = ia * c + i_beta * s
    iq = -ia * s + i_beta * c
    return id_, iq


def infer_fs(n_samples: int, fs_arg: float | None) -> float:
    if fs_arg is not None:
        return fs_arg
    return 20000.0 if n_samples > 100_000 else DEFAULT_FS_HZ


@dataclass
class SegmentMetrics:
    name: str
    t0: float
    t1: float
    fe_hz: float
    rpm: float
    ipeak_rms: float
    ipeak_max: float
    ia_rms: float
    ib_rms: float
    ic_rms: float
    ia_peak: float
    ib_peak: float
    ic_peak: float
    kcl_rms: float
    kcl_peak: float
    iq_rms: float
    id_rms: float
    id_iq_ratio: float
    iq_neg_pct: float
    park_iq_err_rms: float
    thd_ia_pct: float
    clamp_pct: float
    near_zero_pct: float
    flat_pct: float
    di_max_ka_s: float


@dataclass
class IsrProfile:
    """Keil Watch g_telem_dbg 快照（CSV 不含 ISR，需手动或调试传入）。"""
    isr_delta: int | None = None
    enc_dma_cpu_delta: int | None = None
    enc_dma_seq_delta: int | None = None
    enc_total_delta: int | None = None
    enc_dma_seq_delta_max: int | None = None
    cpu_mhz: float = 170.0
    ctrl_ts_us: float = 50.0


@dataclass
class FileReport:
    path: Path
    fs_hz: float
    n_samples: int
    duration_s: float
    segments: list[SegmentMetrics] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)
    extra_notes: list[str] = field(default_factory=list)


def _slice(t0: float, t1: float, fs: float, n: int) -> slice:
    i0 = max(0, int(t0 * fs))
    i1 = min(n, int(t1 * fs))
    if i1 <= i0:
        i1 = min(n, i0 + 1)
    return slice(i0, i1)


def _elec_freq_hz(theta: np.ndarray, fs: float) -> float:
    if len(theta) < 2:
        return 0.0
    th_u = unwrap(theta)
    d = np.diff(th_u)
    if len(d) == 0:
        return 0.0
    omega = float(np.median(d) * fs)
    return omega / (2.0 * math.pi)


def _fundamental_distortion_pct(ia: np.ndarray, theta: np.ndarray) -> float:
    if len(ia) < 8:
        return float("nan")
    th_u = unwrap(theta)
    c, s = np.cos(th_u), np.sin(th_u)
    a1 = 2.0 * float(np.mean(ia * c))
    b1 = 2.0 * float(np.mean(ia * s))
    fund = math.hypot(a1, b1) / math.sqrt(2.0)
    if fund < 1e-9:
        return float("nan")
    recon = a1 * c + b1 * s
    resid = ia - recon
    return float(np.sqrt(np.mean(resid ** 2)) / fund * 100.0)


def _clamp_fraction(ia, ib, ic, ipeak) -> float:
    minabc = np.minimum(np.abs(ia), np.minimum(np.abs(ib), np.abs(ic)))
    mask = (minabc < CLAMP_I_MIN_A) & (ipeak > CLAMP_IPEAK_MIN_A)
    return float(np.mean(mask) * 100.0)


def _near_zero_fraction(ia, ib, ic) -> float:
    nz = (
        (np.abs(ia) < NEAR_ZERO_A)
        | (np.abs(ib) < NEAR_ZERO_A)
        | (np.abs(ic) < NEAR_ZERO_A)
    )
    return float(np.mean(nz) * 100.0)


def _flat_fraction(ia, ib, ic, fs: float) -> float:
    w = max(2, int(FLAT_WIN_MS * 1e-3 * fs))
    total = 0.0
    for x in (ia, ib, ic):
        if len(x) < w + 1:
            continue
        rs = np.array([np.std(x[i : i + w]) for i in range(len(x) - w)])
        total += float(np.mean(rs < FLAT_STD_A) * 100.0)
    return total / 3.0


def _di_max_ka_s(ia, ib, ic, fs: float) -> float:
    m = 0.0
    for x in (ia, ib, ic):
        if len(x) < 2:
            continue
        m = max(m, float(np.max(np.abs(np.diff(x))) * fs / 1000.0))
    return m


def analyze_segment(
    name: str,
    t0: float,
    t1: float,
    ia, ib, ic, iq, id_, th,
    fs: float,
    n: int,
) -> SegmentMetrics:
    sl = _slice(t0, t1, fs, n)
    a, b, c = ia[sl], ib[sl], ic[sl]
    q, d = iq[sl], id_[sl]
    th_s = th[sl]
    isum = a + b + c
    ipk = np.sqrt(a ** 2 + b ** 2 + c ** 2)

    fe = _elec_freq_hz(th_s, fs)
    rpm = fe / POLE_PAIRS * 60.0

    id_o, iq_o = park_offline(a, b, c, th_s)
    iq_err = float(np.sqrt(np.mean((q - iq_o) ** 2)))

    iq_neg = float(np.mean((q < -IQ_NEG_THRESH_A) & (np.abs(q) > IQ_NEG_THRESH_A)) * 100.0)
    iq_rms = float(np.sqrt(np.mean(q ** 2)))
    id_rms = float(np.sqrt(np.mean(d ** 2)))

    return SegmentMetrics(
        name=name,
        t0=t0,
        t1=t1,
        fe_hz=fe,
        rpm=rpm,
        ipeak_rms=float(np.sqrt(np.mean(ipk ** 2))),
        ipeak_max=float(np.max(ipk)),
        ia_rms=float(np.sqrt(np.mean(a ** 2))),
        ib_rms=float(np.sqrt(np.mean(b ** 2))),
        ic_rms=float(np.sqrt(np.mean(c ** 2))),
        ia_peak=float(np.max(np.abs(a))),
        ib_peak=float(np.max(np.abs(b))),
        ic_peak=float(np.max(np.abs(c))),
        kcl_rms=float(np.sqrt(np.mean(isum ** 2))),
        kcl_peak=float(np.max(np.abs(isum))),
        iq_rms=iq_rms,
        id_rms=id_rms,
        id_iq_ratio=id_rms / max(iq_rms, 1e-9),
        iq_neg_pct=iq_neg,
        park_iq_err_rms=iq_err,
        thd_ia_pct=_fundamental_distortion_pct(a, th_s),
        clamp_pct=_clamp_fraction(a, b, c, ipk),
        near_zero_pct=_near_zero_fraction(a, b, c),
        flat_pct=_flat_fraction(a, b, c, fs),
        di_max_ka_s=_di_max_ka_s(a, b, c, fs),
    )


def deadband_ab_sweep_segments(half_s: float = 2.5) -> list[tuple[str, float, float]]:
    """15 s 综合录波：每 Uq 先 OFF 后 ON，各 half_s（与 M1_OPEN_UQ_HALF_STEP_S 一致）。"""
    segs: list[tuple[str, float, float]] = []
    uqs = [2.0, 2.5, 3.0]
    for i, uq in enumerate(uqs):
        t0 = i * 2.0 * half_s
        t_mid = t0 + half_s
        t1 = t0 + 2.0 * half_s
        segs.append((f"Uq~{uq:.1f}V DB-OFF {t0:.1f}-{t_mid:.1f}s", t0, t_mid))
        segs.append((f"Uq~{uq:.1f}V DB-ON  {t_mid:.1f}-{t1:.1f}s", t_mid, t1))
    return segs


def default_uq_segments(duration_s: float, step_s: float = 5.0) -> list[tuple[str, float, float]]:
    segs: list[tuple[str, float, float]] = []
    uqs = [2.0, 2.5, 3.0]
    for i, uq in enumerate(uqs):
        t0 = i * step_s
        t1 = min((i + 1) * step_s, duration_s)
        if t0 >= duration_s:
            break
        segs.append((f"Uq~{uq:.1f}V {t0:.0f}-{t1:.0f}s", t0, t1))
    if len(uqs) * step_s < duration_s:
        segs.append((f"Uq~3.0V hold {len(uqs)*step_s:.0f}-{duration_s:.0f}s", len(uqs) * step_s, duration_s))
    return segs


def _primary_eval_segment(segments: list) -> object | None:
    """Prefer highest-fe rotating segment (typically Uq=3 V hold) for deadband KPI."""
    if not segments:
        return None
    rotating = [s for s in segments if s.fe_hz > 5.0]
    if rotating:
        return max(rotating, key=lambda s: s.fe_hz)
    return segments[-1]


def _extra_conclusions(segments: list) -> list[str]:
    lines: list[str] = []
    if not segments:
        return lines
    primary = _primary_eval_segment(segments)
    if primary is None:
        return lines

    if primary.iq_neg_pct < 1.0 and primary.thd_ia_pct > 40.0:
        lines.append(
            f"**dq 好看 ≠ abc 正常**：{primary.name} 段 Iq rms≈{primary.iq_rms:.2f} A、"
            f"Iq<0%={primary.iq_neg_pct:.1f}，但 Ia THD≈{primary.thd_ia_pct:.0f}%、"
            f"clamp≈{primary.clamp_pct:.0f}% — 死区联调必须看三相波形。"
        )

    rpms = [s.rpm for s in segments if s.fe_hz > 1.0]
    if len(rpms) >= 2:
        lines.append(
            f"Uq 升高 → 转速升高（{'→'.join(f'{r:.0f}' for r in rpms)} rpm）→ "
            f"|I| rms 略降，符合开环固定 Uq 预期；5 s 边界无异常跳变。"
        )

    first = segments[0]
    if first.ipeak_max > 2.0 and first.di_max_ka_s > 10.0:
        lines.append(
            f"{first.name} 段 |I| peak={first.ipeak_max:.2f} A、dI/dt≈{first.di_max_ka_s:.0f} kA/s，"
            f"疑为启动/机械瞬态窄脉冲，不计入稳态 KPI。"
        )
    return lines


def analyze_file(
    path: Path,
    fs_hz: float | None,
    skip_s: float,
    segments: Iterable[tuple[str, float, float]] | None,
    step_s: float = 5.0,
) -> FileReport:
    ia, ib, ic, iq, id_, th = load_csv(path)
    n = len(ia)
    fs = infer_fs(n, fs_hz)
    duration = n / fs
    report = FileReport(path=path, fs_hz=fs, n_samples=n, duration_s=duration)

    if segments is None:
        segments = default_uq_segments(duration, step_s=step_s)

    for name, t0, t1 in segments:
        if t0 >= duration:
            continue
        report.segments.append(
            analyze_segment(name, t0, t1, ia, ib, ic, iq, id_, th, fs, n)
        )

    steady = _primary_eval_segment(report.segments)
    if steady is None:
        steady = analyze_segment("steady(skip)", skip_s, duration, ia, ib, ic, iq, id_, th, fs, n)

    if steady.fe_hz > 1.0:
        t_sector = 1.0 / (6.0 * steady.fe_hz) * 1000.0
        report.notes.append(
            f"主评段 {steady.name}：fe≈{steady.fe_hz:.1f} Hz（{steady.rpm:.0f} rpm）→ "
            f"SVPWM 扇区 T/6≈{t_sector:.1f} ms；VOFA「一相 platform/clamp 轮换」约此尺度，"
            f"不是 Uq 5 s 阶梯。"
        )
    if steady.thd_ia_pct > 40.0:
        report.notes.append(
            f"Ia THD proxy≈{steady.thd_ia_pct:.0f}%：三相相对 θ 基波畸变大；"
            f"死区 A/B 主看 ch0–2 THD/clamp，勿单用 Iq<0%。"
        )
    if steady.clamp_pct > 50.0:
        report.notes.append(
            f"clamp≈{steady.clamp_pct:.0f}%：一相近零且 |I|>0.12 A 占比高；"
            f"典型 SVPWM min 相 + 低电流/死区 OFF，开 deadband 后应对比此项。"
        )
    if steady.park_iq_err_rms > 0.001:
        report.notes.append(
            f"离线 Park 与固件 Iq 误差 {steady.park_iq_err_rms*1000:.2f} mA — 查 binding/θ。"
        )
    elif steady.park_iq_err_rms < 0.0001:
        report.notes.append("离线 Park 与固件 Iq 一致（<0.1 mA rms），dq 计算可信。")

    report.extra_notes = _extra_conclusions(report.segments)

    return report


def compare_reports(a: FileReport, b: FileReport) -> list[str]:
    la, lb = a.path.stem, b.path.stem
    lines = [f"## A/B 对比（{la} vs {lb}）", ""]
    n = min(len(a.segments), len(b.segments))
    if n == 0:
        return lines + ["(no matching segments)"]
    lines.append(
        f"| 段 | {la} THD% | {lb} THD% | {la} clamp% | {lb} clamp% | "
        f"{la} rpm | {lb} rpm | KCL {la} | KCL {lb} |"
    )
    lines.append("|" + "---|" * 8)
    for i in range(n):
        sa, sb = a.segments[i], b.segments[i]
        lines.append(
            f"| {sa.name} | {sa.thd_ia_pct:.1f} | {sb.thd_ia_pct:.1f} | "
            f"{sa.clamp_pct:.1f} | {sb.clamp_pct:.1f} | "
            f"{sa.rpm:.0f} | {sb.rpm:.0f} | "
            f"{sa.kcl_rms:.4f} | {sb.kcl_rms:.4f} |"
        )
    lines.append("")
    return lines


def _ab_interpretation(a: FileReport, b: FileReport) -> list[str]:
    """对比两段录波的主评段差异（a=本文件，b=--compare）。"""
    pa = _primary_eval_segment(a.segments)
    pb = _primary_eval_segment(b.segments)
    if pa is None or pb is None:
        return []
    d_thd = pa.thd_ia_pct - pb.thd_ia_pct
    d_clamp = pa.clamp_pct - pb.clamp_pct
    lines = [
        "### A/B 解读（主评段）",
        "",
        f"- **{a.path.stem}** vs **{b.path.stem}**：Uq≈3 V 段 rpm **{pa.rpm:.0f} vs {pb.rpm:.0f}**（同 Uq 转速不同会直接影响 THD proxy，需结合 rpm 看）。",
        f"- clamp：{pa.clamp_pct:.1f}% vs {pb.clamp_pct:.1f}%（Δ={d_clamp:+.1f} pp）。",
        f"- Ia THD proxy：{pa.thd_ia_pct:.1f}% vs {pb.thd_ia_pct:.1f}%（Δ={d_thd:+.1f} pp）。",
        "",
    ]
    return lines


def format_isr_md(p: IsrProfile | None) -> list[str]:
    if p is None or p.isr_delta is None:
        return []
    cyc = p.isr_delta
    us = cyc / (p.cpu_mhz * 1e6) * 1e6
    pct = us / p.ctrl_ts_us * 100.0
    lines = [
        "## ISR / 编码器耗时（Keil Watch `g_telem_dbg`）",
        "",
        f"| 项 | cycles | @ {p.cpu_mhz:.0f} MHz |",
        "|----|--------|-----------|",
        f"| **isr_delta**（`motor_current_tick` 主体） | **{cyc}** | **{us:.2f} µs**（≈{pct:.0f}% × {p.ctrl_ts_us:.0f} µs 控制周期） |",
    ]
    if p.enc_dma_cpu_delta is not None:
        lines.append(
            f"| enc_dma_cpu_delta | {p.enc_dma_cpu_delta} | "
            f"{p.enc_dma_cpu_delta / (p.cpu_mhz * 1e6) * 1e6:.2f} µs |"
        )
    if p.enc_dma_seq_delta is not None:
        lines.append(
            f"| enc_dma_seq_delta | {p.enc_dma_seq_delta} | "
            f"{p.enc_dma_seq_delta / (p.cpu_mhz * 1e6) * 1e6:.2f} µs |"
        )
    if p.enc_total_delta is not None:
        lines.append(
            f"| enc_total_delta | {p.enc_total_delta} | "
            f"{p.enc_total_delta / (p.cpu_mhz * 1e6) * 1e6:.2f} µs |"
        )
    if p.enc_dma_seq_delta_max is not None:
        lines.append(
            f"| enc_dma_seq_delta_max | {p.enc_dma_seq_delta_max} | "
            f"{p.enc_dma_seq_delta_max / (p.cpu_mhz * 1e6) * 1e6:.2f} µs |"
        )
    lines.extend([
        "",
        "说明：`isr_delta` 为 ADC 回调内 DWT 计数（含 SVPWM + deadband + 遥测 `telem_bringup_tick`）；"
        "编码器 DMA 在 `encoder_kick` 后异步完成，与 `enc_*_delta` 互补。",
        "",
    ])
    return lines


def format_report_md(
    r: FileReport,
    title: str | None = None,
    note: str = "",
    isr: IsrProfile | None = None,
) -> str:
    title = title or f"Deadband 分析 — {r.path.name}"
    lines = [
        f"# VOFA 分析结论 — {r.path.name}",
        "",
        f"**日期：** {r.path.parent.name}  ",
        f"**文件：** `{r.path.as_posix()}`  ",
        f"**工具：** `tools/analyze_deadband_vofa.py`  ",
    ]
    if note:
        lines.append(f"**工况：** {note}  ")
    lines.extend([
        "",
        "**通道：** ch0–2 Ia/Ib/Ic (A)，ch3 Iq，ch4 Id，ch5 θ_el  ",
        "",
        "## 录波",
        "",
        f"| 样本 | Fs | 时长 |",
        f"|------|-----|------|",
        f"| {r.n_samples} | {r.fs_hz:.0f} Hz | {r.duration_s:.2f} s |",
        "",
    ])
    lines.extend(format_isr_md(isr))
    lines.extend([
        "## 分段（三相 + dq）",
        "",
        "| 时段 | fe(Hz) | rpm | \\|I\\| rms | \\|I\\| pk | Ia THD% | clamp% | near0% | flat% | Iq rms | Id/Iq | Iq<0% | KCL | Park err |",
        "|------|--------|-----|---------|--------|---------|--------|--------|-------|--------|-------|-------|-----|----------|",
    ])
    for s in r.segments:
        lines.append(
            f"| {s.name} | {s.fe_hz:.1f} | {s.rpm:.0f} | {s.ipeak_rms:.3f} | {s.ipeak_max:.2f} | "
            f"{s.thd_ia_pct:.1f} | {s.clamp_pct:.1f} | {s.near_zero_pct:.1f} | {s.flat_pct:.1f} | "
            f"{s.iq_rms:.3f} | {s.id_iq_ratio:.2f} | {s.iq_neg_pct:.1f} | {s.kcl_rms:.4f} | "
            f"{s.park_iq_err_rms*1000:.3f}mA |"
        )
    primary = _primary_eval_segment(r.segments)
    if primary:
        lines.extend([
            "",
            "## 死区基线 KPI（主评段）",
            "",
            "| 段 | fe | rpm | Ia THD% | clamp% | near0% | Iq rms | Iq<0% |",
            "|----|-----|-----|---------|--------|--------|--------|-------|",
            f"| {primary.name} | {primary.fe_hz:.1f} Hz | {primary.rpm:.0f} | "
            f"{primary.thd_ia_pct:.1f} | {primary.clamp_pct:.1f} | {primary.near_zero_pct:.1f} | "
            f"{primary.iq_rms:.3f} | {primary.iq_neg_pct:.1f} |",
        ])
    lines.extend(["", "## 结论", ""])
    for n in r.notes:
        lines.append(f"- {n}")
    for n in r.extra_notes:
        lines.append(f"- {n}")
    lines.extend([
        "",
        "### 指标说明",
        "",
        "- **Ia THD%**：相对 θ_el 基波的残差/fund，越大 abc 越非正弦（死区联调主 KPI）。",
        "- **clamp%**：min(\\|Ia\\|,\\|Ib\\|,\\|Ic\\|)<0.06 且 \\|I\\|>0.12 的样本占比（SVPWM min 相 + 死区）。",
        "- **near0% / flat%**：近零与 2 ms 低方差 platform 占比，对应 VOFA 上「一相贴地」。",
        "- **Iq<0%**：dq 半圈为负；死区 A/B 不宜单用此项，应看 THD/clamp。",
        "",
        "## 联调目标对照",
        "",
        "| 目标 | 本文件 |",
        "|------|--------|",
        "| 开环 Uq 扫档 + 看 Iabc | ✅ 三档 + 保持 3 V |",
        "| 死区 OFF 基线 / ON A/B | ⚠️ 本文件为 OFF；录 ON 后用 `--compare` |",
        "| 电流环验收 | ❌ 观测开环，无 PI/startup |",
        "",
        "## 建议下一步",
        "",
        "1. 同条件录 **deadband ON**，`python tools/analyze_deadband_vofa.py off.csv --compare on.csv --report`",
        "2. 长录改 `TELEM_BRINGUP_DECIMATION=100`（200 Hz）减文件体积",
        "3. 堵转 Iq_ref 阶跃另录，勿与开环扫 Uq 混评",
        "",
        f"*处理后 CSV（稳态段）：* `{r.path.stem}_deadband_processed.csv`",
        "",
    ])
    return "\n".join(lines)


def print_console(r: FileReport):
    print(f"=== Deadband VOFA: {r.path.name} ===")
    print(f"Samples={r.n_samples}  Fs={r.fs_hz:.0f}Hz  duration={r.duration_s:.2f}s")
    print()
    for s in r.segments:
        print(f"--- {s.name} ---")
        print(
            f"  fe={s.fe_hz:.1f}Hz rpm={s.rpm:.0f}  |I| rms={s.ipeak_rms:.3f} pk={s.ipeak_max:.2f}"
        )
        print(
            f"  Ia THD~{s.thd_ia_pct:.1f}%  clamp={s.clamp_pct:.1f}%  "
            f"near0={s.near_zero_pct:.1f}%  flat={s.flat_pct:.1f}%  dI_max={s.di_max_ka_s:.1f}kA/s"
        )
        print(
            f"  Iq rms={s.iq_rms:.3f}  Id/Iq={s.id_iq_ratio:.2f}  Iq<0={s.iq_neg_pct:.1f}%  "
            f"KCL={s.kcl_rms:.4f}  Park err={s.park_iq_err_rms*1000:.3f}mA"
        )
        print()
    for note in r.notes:
        print(f"  * {note}")
    print()


def export_processed(path: Path, fs: float, skip_s: float):
    ia, ib, ic, iq, id_, th = load_csv(path)
    n = len(ia)
    sl = slice(int(skip_s * fs), n)
    t = np.arange(n)[sl] / fs - skip_s
    th_u = unwrap(th[sl])
    th_est = np.arctan2((ib[sl] - ic[sl]) / math.sqrt(3.0), ia[sl]) - 0.5 * math.pi
    ipk = np.sqrt(ia[sl] ** 2 + ib[sl] ** 2 + ic[sl] ** 2)
    minabc = np.minimum(np.abs(ia[sl]), np.minimum(np.abs(ib[sl]), np.abs(ic[sl])))
    clamp = ((minabc < CLAMP_I_MIN_A) & (ipk > CLAMP_IPEAK_MIN_A)).astype(float)

    out = path.with_name(path.stem + "_deadband_processed.csv")
    with out.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow([
            "t_s", "Ia", "Ib", "Ic", "Iq", "Id", "theta_el",
            "I_peak", "clamp_flag", "I_sum",
        ])
        for i in range(len(t)):
            w.writerow([
                f"{t[i]:.5f}",
                f"{ia[sl][i]:.6f}", f"{ib[sl][i]:.6f}", f"{ic[sl][i]:.6f}",
                f"{iq[sl][i]:.6f}", f"{id_[sl][i]:.6f}", f"{th[sl][i]:.6f}",
                f"{ipk[i]:.6f}", f"{clamp[i]:.0f}",
                f"{ia[sl][i]+ib[sl][i]+ic[sl][i]:.6f}",
            ])
    print(f"Exported: {out}")
    return out


def main():
    parser = argparse.ArgumentParser(description="Deadband-focused VOFA CSV analyzer")
    parser.add_argument("csv", type=Path, help="primary CSV")
    parser.add_argument("--compare", type=Path, default=None, help="second CSV (e.g. deadband ON)")
    parser.add_argument("--fs", type=float, default=None)
    parser.add_argument("--skip", type=float, default=1.0, help="steady segment start (s)")
    parser.add_argument("--step-s", type=float, default=5.0, help="Uq sweep step duration (s)")
    parser.add_argument(
        "--ab-sweep",
        action="store_true",
        help="6×2.5s segments: each Uq deadband OFF then ON (M1_OPEN_UQ_DEADBAND_AB_SWEEP)",
    )
    parser.add_argument("--half-s", type=float, default=2.5, help="with --ab-sweep: half-step (s)")
    parser.add_argument("--export-processed", action="store_true")
    parser.add_argument("--report", action="store_true", help="write 分析结论_<stem>.md beside CSV")
    parser.add_argument("--note", type=str, default="", help="工况说明写入报告")
    parser.add_argument("--isr-cycles", type=int, default=None, help="g_telem_dbg.isr_delta")
    parser.add_argument("--enc-dma-cycles", type=int, default=None, help="enc_dma_cpu_delta")
    parser.add_argument("--enc-seq-cycles", type=int, default=None, help="enc_dma_seq_delta")
    parser.add_argument("--enc-total-cycles", type=int, default=None, help="enc_total_delta")
    parser.add_argument("--enc-seq-max-cycles", type=int, default=None, help="enc_dma_seq_delta_max")
    parser.add_argument("--cpu-mhz", type=float, default=170.0)
    parser.add_argument("--dir", type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args()

    path = args.csv if args.csv.is_absolute() else args.dir / args.csv
    if not path.exists():
        print(f"Not found: {path}", file=sys.stderr)
        sys.exit(1)

    fs = infer_fs(sum(1 for _ in path.open(encoding="utf-8")) - 1, args.fs)

    segs = deadband_ab_sweep_segments(args.half_s) if args.ab_sweep else None
    report = analyze_file(path, fs, args.skip, segs, step_s=args.step_s)
    print_console(report)

    isr_prof = IsrProfile(
        isr_delta=args.isr_cycles,
        enc_dma_cpu_delta=args.enc_dma_cycles,
        enc_dma_seq_delta=args.enc_seq_cycles,
        enc_total_delta=args.enc_total_cycles,
        enc_dma_seq_delta_max=args.enc_seq_max_cycles,
        cpu_mhz=args.cpu_mhz,
    )

    compare_lines: list[str] = []
    compare_b: FileReport | None = None
    if args.compare is not None:
        p2 = args.compare if args.compare.is_absolute() else args.dir / args.compare
        if p2.exists():
            compare_b = analyze_file(p2, fs, args.skip, segs, step_s=args.step_s)
            print_console(compare_b)
            compare_lines = compare_reports(report, compare_b)
        else:
            print(f"Compare file not found: {p2}", file=sys.stderr)

    if args.export_processed:
        export_processed(path, report.fs_hz, args.skip)

    if args.report:
        md_path = path.with_name(f"分析结论_{path.stem}.md")
        body = format_report_md(report, note=args.note, isr=isr_prof)
        if compare_lines:
            body += "\n".join(compare_lines)
            if compare_b is not None:
                body += "\n".join(_ab_interpretation(report, compare_b))
        md_path.write_text(body, encoding="utf-8")
        print(f"Report: {md_path}")


if __name__ == "__main__":
    main()
