#!/usr/bin/env python3
"""堵转 Iq 阶跃响应离线分析（M1_IDENT_ENABLE，open_seq 60/61/67/74/73）。

默认：18 轮 × (0→0.3→0→0.5→0→1.0→0)，各 0.5 s；6 OFF + 6 FIXED + 6 LUT。

四核心指标：Tr（10-90%）、超调 σ%、稳态误差、调节时间 Ts（±2% 阶跃幅值带内且保持）。

通道：ch0-2=Ia/Ib/Ic  ch3=Iq_fb  ch4=Iq_ref  ch5=Uq_pi

用法:
  python tools/ident/analyze_iq_step.py VOFA+CSV/xxx.csv
  python tools/ident/analyze_iq_step.py file.csv --md report.md
  python tools/ident/analyze_iq_step.py file.csv --fc-hz 1000
"""
from __future__ import annotations

import argparse
import sys
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))

from analyze_id_cal_sweep_vofa import load_csv  # noqa: E402

FS = 20000.0
DEFAULT_ROUNDS = 18
DEFAULT_OFF_ROUNDS = 6
DEFAULT_FIXED_ROUNDS = 6
RISES_PER_ROUND = 3

# 5065 @ fc Hz 带宽参考（保守 / 理想）；默认与 M1_PI_FC_HZ 对齐
DEFAULT_FC_HZ = 1000.0
DEFAULT_TARGETS = {
    "0->0.3": {"tr": (3.0, 1.0), "os": (15.0, 10.0), "err_pct": (2.0, 1.0), "ts": (5.0, 2.0)},
    "0->0.5": {"tr": (3.0, 1.0), "os": (15.0, 10.0), "err_pct": (2.0, 1.0), "ts": (5.0, 2.0)},
    "0->1.0": {"tr": (3.0, 1.0), "os": (15.0, 10.0), "err_pct": (2.0, 1.0), "ts": (5.0, 2.0)},
}


def classify_edge(fv: float, tv: float) -> str:
    if abs(fv) < 0.05 and tv > 0.05:
        return f"0->{tv:.1f}"
    if abs(tv) < 0.05 and fv > 0.05:
        return f"{fv:.1f}->0"
    if tv > fv + 0.05:
        return f"{fv:.1f}->{tv:.1f}"
    if tv < fv - 0.05:
        return f"{fv:.1f}->{tv:.1f}"
    return f"{fv:.2f}->{tv:.2f}"


def is_rise_from_zero(kind: str) -> bool:
    return kind.startswith("0->")


def next_edge_index(edge_idx: list[int], pos: int, n: int) -> int:
    for k in range(pos + 1, len(edge_idx)):
        return int(edge_idx[k])
    return n


def analyze_step(
    ia: np.ndarray,
    ib: np.ndarray,
    ic: np.ndarray,
    iq_fb: np.ndarray,
    uq: np.ndarray,
    t: np.ndarray,
    i0: int,
    from_v: float,
    to_v: float,
    i_end: int,
) -> dict | None:
    step_amp = abs(to_v - from_v)
    if step_amp < 0.01:
        return None

    n = len(iq_fb)
    i_end = min(i_end, n)
    win = min(i_end - i0, int(FS * 1.2))
    if win < 200:
        win = min(int(FS * 0.05), n - i0)
    if win < 200:
        return None

    seg_t = t[i0 : i0 + win] - t[i0]
    y = iq_fb[i0 : i0 + win]
    u = uq[i0 : i0 + win]
    y0 = float(np.mean(iq_fb[max(0, i0 - 400) : i0]))
    u0 = float(np.mean(uq[max(0, i0 - 400) : i0]))

    settle_s = i0 + int(win * 0.7)
    settle_e = i0 + win
    y_ss = float(np.mean(iq_fb[settle_s:settle_e]))
    u_ss = float(np.mean(uq[settle_s:settle_e]))
    err_ss = y_ss - to_v
    err_pct = abs(err_ss) / max(abs(to_v), 0.01) * 100.0

    rising = to_v > from_v
    if rising:
        lv, hv = y0 + 0.1 * step_amp, y0 + 0.9 * step_amp
        t10 = t90 = np.nan
        for j in range(len(y)):
            if np.isnan(t10) and y[j] >= lv:
                t10 = seg_t[j]
            if y[j] >= hv:
                t90 = seg_t[j]
                break
        tr_ms = (t90 - t10) * 1000 if not (np.isnan(t10) or np.isnan(t90)) else np.nan
        peak_i = int(np.argmax(y))
        peak = float(y[peak_i])
        t_peak_ms = float(seg_t[peak_i] * 1000.0)
        overshoot_pct = (peak - y_ss) / max(abs(y_ss), 0.01) * 100.0 if abs(y_ss) > 0.01 else (peak - to_v) / step_amp * 100.0
    else:
        lv, hv = y0 - 0.1 * step_amp, y0 - 0.9 * step_amp
        t10 = t90 = np.nan
        for j in range(len(y)):
            if np.isnan(t10) and y[j] <= lv:
                t10 = seg_t[j]
            if y[j] <= hv:
                t90 = seg_t[j]
                break
        tr_ms = (t90 - t10) * 1000 if not (np.isnan(t10) or np.isnan(t90)) else np.nan
        peak_i = int(np.argmin(y))
        peak = float(y[peak_i])
        t_peak_ms = float(seg_t[peak_i] * 1000.0)
        overshoot_pct = (y_ss - peak) / max(abs(y_ss), 0.01) * 100.0 if abs(y_ss) > 0.01 else (to_v - peak) / step_amp * 100.0

    tol = 0.02 * step_amp
    ts_ms = np.nan
    target = to_v
    for j in range(len(y)):
        if np.all(np.abs(y[j:] - target) <= tol):
            ts_ms = float(seg_t[j] * 1000.0)
            break

    settle_std = float(np.std(iq_fb[settle_s:settle_e]))
    uq_jump = u_ss - u0
    uq_peak = float(np.max(np.abs(u)))

    pre = slice(max(0, i0 - 200), i0)
    post = slice(i0, min(n, i0 + 400))
    abc_pp_step = float(
        (np.ptp(ia[post]) + np.ptp(ib[post]) + np.ptp(ic[post])) / 3.0
    )
    abc_pp_ss = float(
        (np.ptp(ia[settle_s:settle_e]) + np.ptp(ib[settle_s:settle_e]) + np.ptp(ic[settle_s:settle_e])) / 3.0
        if settle_e > settle_s + 50
        else np.nan
    )

    return {
        "t0": float(t[i0]),
        "from_v": from_v,
        "to_v": to_v,
        "y0": y0,
        "y_ss": y_ss,
        "err_ss": err_ss,
        "err_pct": err_pct,
        "tr_ms": tr_ms,
        "ts_ms": ts_ms,
        "t_peak_ms": t_peak_ms,
        "peak": peak,
        "overshoot_pct": overshoot_pct,
        "settle_std": settle_std,
        "uq0": u0,
        "uq_ss": u_ss,
        "uq_jump": uq_jump,
        "uq_peak": uq_peak,
        "abc_pp_step": abc_pp_step,
        "abc_pp_ss": abc_pp_ss,
        "rising": rising,
    }


def find_edges(iq_ref: np.ndarray) -> list[tuple[int, str, float, float]]:
    dref = np.diff(iq_ref)
    edge_idx = np.where(np.abs(dref) > 0.05)[0] + 1
    steps = []
    for i in edge_idx:
        fv, tv = float(iq_ref[i - 1]), float(iq_ref[i])
        steps.append((int(i), classify_edge(fv, tv), fv, tv))
    return steps


def verdict_mark(val: float, conservative: float, ideal: float, lower_is_better: bool = True) -> str:
    if np.isnan(val):
        return "—"
    if lower_is_better:
        if val <= ideal:
            return "✅"
        if val <= conservative:
            return "🟡"
        return "❌"
    if val >= ideal:
        return "✅"
    if val >= conservative:
        return "🟡"
    return "❌"


def run_analysis(
    path: Path,
    fc_hz: float = DEFAULT_FC_HZ,
    rounds: int = DEFAULT_ROUNDS,
    off_rounds: int = DEFAULT_OFF_ROUNDS,
    fixed_rounds: int = DEFAULT_FIXED_ROUNDS,
) -> str:
    ia, ib, ic, iq_fb, iq_ref, uq = load_csv(path)
    n = len(iq_ref)
    t = np.arange(n) / FS
    steps = find_edges(iq_ref)
    edge_idx = [s[0] for s in steps]
    cnt = Counter(s[1] for s in steps)

    lines: list[str] = []
    lines.append(f"# Iq 阶跃响应分析 — `{path.name}`\n")
    lines.append(f"- 时长 **{t[-1]:.2f} s**，Fs={FS:.0f} Hz，参考带宽 **{fc_hz:.0f} Hz**")
    lines.append("- 通道：ch0-2=**Ia/Ib/Ic** ch3=**Iq_fb** ch4=**Iq_ref** ch5=**Uq_pi**")
    lines.append(f"- 边沿统计：{dict(cnt)}")
    lut_rounds = max(0, rounds - off_rounds - fixed_rounds)
    lines.append(
        f"- 预期：**{rounds}** 轮（**{off_rounds}** OFF + **{fixed_rounds}** FIXED + **{lut_rounds}** LUT），"
        f"每轮 **{RISES_PER_ROUND}** 次 0 起点上升\n"
    )

    lines.append("## 四核心指标（时域）\n")
    lines.append("| 参数 | 含义 | 好（5065@1000Hz 线性理想） | 差 |")
    lines.append("|------|------|---------------------------|-----|")
    lines.append("| **Tr** | 10%→90% 上升时间 | < 1 ms（理想）/ < 3 ms | 太大 → Kp/带宽偏低 |")
    lines.append("| **σ%** | 超调 (峰值−稳态)/稳态 | < 10% | 太大 → Kp 过大或 windup |")
    lines.append("| **稳态误差** | 稳态均值 − 目标 | < 1% | 太大 → Ki 不足或死区 |")
    lines.append("| **Ts** | 进入 ±2%×阶跃幅值带并保持 | < 5 ms | 太长 → 积分/阻尼不足 |\n")

    first_up = next((i for i, k, _, _ in steps if is_rise_from_zero(k)), None)
    if first_up is not None:
        lines.append("## HOLD\n")
        lines.append(f"- 首次 0 起点阶跃 @ **{t[first_up]:.3f} s**")
        hold = iq_fb[:first_up]
        lines.append(f"- HOLD Iq：mean={hold.mean():+.4f} A，std={hold.std():.4f} A\n")

    results: list[tuple[str, dict]] = []
    for pos, (i, kind, fv, tv) in enumerate(steps):
        i_end = next_edge_index(edge_idx, pos, n)
        r = analyze_step(ia, ib, ic, iq_fb, uq, t, i, fv, tv, i_end)
        if r:
            results.append((kind, r))

    rise_kinds = sorted({k for k, _ in results if is_rise_from_zero(k)}, key=lambda x: float(x.split("->")[1]))
    lines.append("## 三组 0 起点阶跃对比（验收主表）\n")
    lines.append(
        "| 阶跃 | n | Tr (ms) | Ts (ms) | σ% | 稳态误差 (%) | Iq std (mA) | ΔUq (V) | Uq_ss (V) | abc_pp@阶跃 | 判定 |"
    )
    lines.append("|------|---|---------|---------|-----|--------------|-------------|---------|-----------|-------------|------|")

    overall_pass = True
    for kind in rise_kinds:
        grp = [r for k, r in results if k == kind]
        if not grp:
            continue
        tgt = DEFAULT_TARGETS.get(kind, DEFAULT_TARGETS.get("0->0.5"))
        tr_m = float(np.nanmean([r["tr_ms"] for r in grp]))
        ts_m = float(np.nanmean([r["ts_ms"] for r in grp]))
        os_m = float(np.mean([r["overshoot_pct"] for r in grp]))
        err_m = float(np.mean([r["err_pct"] for r in grp]))
        std_m = float(np.mean([r["settle_std"] for r in grp])) * 1000.0
        duq_m = float(np.mean([r["uq_jump"] for r in grp]))
        uq_m = float(np.mean([r["uq_ss"] for r in grp]))
        abc_m = float(np.nanmean([r["abc_pp_step"] for r in grp]))
        marks = [
            verdict_mark(tr_m, tgt["tr"][0], tgt["tr"][1]),
            verdict_mark(os_m, tgt["os"][0], tgt["os"][1]),
            verdict_mark(err_m, tgt["err_pct"][0], tgt["err_pct"][1]),
            verdict_mark(ts_m, tgt["ts"][0], tgt["ts"][1]),
        ]
        ok = all(m == "✅" for m in marks)
        if not ok and any(m == "❌" for m in marks):
            overall_pass = False
        mark = "✅" if ok else ("🟡" if "🟡" in marks else "❌")
        lines.append(
            f"| **{kind} A** | {len(grp)} | {tr_m:.2f} | {ts_m:.1f} | {os_m:+.1f} | {err_m:.3f} | {std_m:.1f} | {duq_m:+.3f} | {uq_m:+.3f} | {abc_m:.3f} | {mark} |"
        )
    lines.append("")

    off_rise_n = off_rounds * RISES_PER_ROUND
    fix_rise_n = fixed_rounds * RISES_PER_ROUND
    rise_idx = 0
    off_results: list[tuple[str, dict]] = []
    fix_results: list[tuple[str, dict]] = []
    lut_results: list[tuple[str, dict]] = []
    for i, kind, fv, tv in steps:
        if not is_rise_from_zero(kind):
            continue
        rise_idx += 1
        pos = edge_idx.index(i)
        i_end = next_edge_index(edge_idx, pos, n)
        r = analyze_step(ia, ib, ic, iq_fb, uq, t, i, fv, tv, i_end)
        if not r:
            continue
        if rise_idx <= off_rise_n:
            off_results.append((kind, r))
        elif rise_idx <= off_rise_n + fix_rise_n:
            fix_results.append((kind, r))
        else:
            lut_results.append((kind, r))

    seg_buckets: list[tuple[str, list[tuple[str, dict]]]] = [
        ("OFF", off_results),
        ("FIXED", fix_results),
        ("LUT", lut_results),
    ]
    active = [(label, bucket) for label, bucket in seg_buckets if bucket]
    if len(active) >= 2:
        title = " / ".join(label for label, _ in active)
        lines.append(f"## 死区分段：{title}（0 起点上升沿）\n")
        lines.append("| 阶跃 | 组 | n | Tr (ms) | σ% | err (%) | ΔUq (V) | abc_pp |")
        lines.append("|------|-----|---|---------|-----|---------|---------|--------|")
        for label, bucket in active:
            by_kind: dict[str, list] = defaultdict(list)
            for k, r in bucket:
                by_kind[k].append(r)
            for kind in sorted(by_kind, key=lambda x: float(x.split("->")[1])):
                grp = by_kind[kind]
                lines.append(
                    f"| {kind} | **{label}** | {len(grp)} | "
                    f"{np.nanmean([r['tr_ms'] for r in grp]):.2f} | "
                    f"{np.mean([r['overshoot_pct'] for r in grp]):+.1f} | "
                    f"{np.mean([r['err_pct'] for r in grp]):.3f} | "
                    f"{np.mean([r['uq_jump'] for r in grp]):+.3f} | "
                    f"{np.nanmean([r['abc_pp_step'] for r in grp]):.3f} |"
                )
        lines.append("")

    lines.append("## 分次明细（0 起点上升沿）\n")
    lines.append(
        "| # | 阶跃 | t (s) | Tr (ms) | Ts (ms) | t_peak (ms) | σ% | err (%) | Iq_ss | ΔUq | abc_pp |"
    )
    lines.append("|---|------|-------|---------|---------|-------------|-----|---------|-------|-----|--------|")
    idx = 0
    for i, kind, fv, tv in steps:
        if not is_rise_from_zero(kind):
            continue
        idx += 1
        pos = edge_idx.index(i)
        i_end = next_edge_index(edge_idx, pos, n)
        r = analyze_step(ia, ib, ic, iq_fb, uq, t, i, fv, tv, i_end)
        if not r:
            continue
        tr_s = f"{r['tr_ms']:.2f}" if not np.isnan(r["tr_ms"]) else "—"
        ts_s = f"{r['ts_ms']:.1f}" if not np.isnan(r["ts_ms"]) else "—"
        lines.append(
            f"| {idx} | {kind} | {r['t0']:.2f} | {tr_s} | {ts_s} | {r['t_peak_ms']:.1f} | {r['overshoot_pct']:+.1f} | {r['err_pct']:.3f} | {r['y_ss']:+.3f} | {r['uq_jump']:+.3f} | {r['abc_pp_step']:.3f} |"
        )
    lines.append("")

    # Legacy pattern (0.3->0.5 chained) if present
    chain = [r for k, r in results if k == "0.3->0.5"]
    if chain:
        lines.append("## 链式阶跃 0.3→0.5（非 0 起点，仅供参考）\n")
        tr_m = float(np.nanmean([r["tr_ms"] for r in chain]))
        os_m = float(np.mean([r["overshoot_pct"] for r in chain]))
        lines.append(f"- n={len(chain)}，Tr={tr_m:.2f} ms，σ%={os_m:+.1f}%\n")

    lines.append("## 回零沿（下降）\n")
    fall_kinds = sorted({k for k, _ in results if k.endswith("->0")}, key=lambda x: float(x.split("->")[0]))
    for kind in fall_kinds:
        grp = [r for k, r in results if k == kind]
        if not grp:
            continue
        tr_m = float(np.nanmean([r["tr_ms"] for r in grp]))
        ts_m = float(np.nanmean([r["ts_ms"] for r in grp]))
        os_m = float(np.mean([r["overshoot_pct"] for r in grp]))
        lines.append(f"- **{kind} A**：Tr={tr_m:.2f} ms，Ts={ts_m:.1f} ms，σ%={os_m:+.1f}%")
    lines.append("")

    abc_rms = np.sqrt((ia**2 + ib**2 + ic**2) / 3)
    lines.append("## 堵转 / 相电流\n")
    lines.append(f"- abc_rms：mean={abc_rms.mean():.3f} A，max={abc_rms.max():.3f} A")
    if first_up is not None:
        for v in (0.3, 0.5, 1.0):
            m = (np.abs(iq_ref - v) < 0.05) & (t > t[first_up])
            if int(m.sum()) < 500:
                continue
            pp = (np.ptp(ia[m]) + np.ptp(ib[m]) + np.ptp(ic[m])) / 3
            lines.append(
                f"- @Iq_ref={v} A：Ia={ia[m].mean():+.3f} Ib={ib[m].mean():+.3f} Ic={ic[m].mean():+.3f}，abc_pp={pp:.3f} A"
            )
    lines.append("")

    lines.append("## 验收判读\n")
    exp_rises = rounds * RISES_PER_ROUND
    rise_count = sum(1 for _, k, _, _ in steps if is_rise_from_zero(k))
    lines.append(
        f"- 0 起点上升沿：实测 **{rise_count}** / 预期 **{exp_rises}** "
        f"{'✅' if rise_count == exp_rises else '⚠️'}"
    )
    lines.append(
        f"- 分段上升沿：OFF **{off_rise_n}** / FIXED **{fix_rise_n}** / LUT **{lut_rounds * RISES_PER_ROUND}**"
    )
    if first_up is not None:
        hold_ok = abs(t[first_up] - 2.0) < 0.5
        lines.append(f"- HOLD 2 s：实测 {t[first_up]:.2f} s {'✅' if hold_ok else '⚠️'}")
    lines.append(f"- 相对 **{fc_hz:.0f} Hz** 线性参考：{'✅ 达标' if overall_pass else '⚠️ 需再整定或重录'}")
    dwell = 0.5
    step_s = rounds * 6 * dwell
    lines.append(
        f"\n> 录波建议：堵转+制动 → 上电 → 录 **~{2 + step_s:.0f} s**"
        f"（2 s HOLD + {rounds} 轮×6×{dwell}s）→ open_seq=73 后停录。"
    )

    return "\n".join(lines)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("csv", type=Path)
    ap.add_argument("--md", type=Path, help="write markdown report")
    ap.add_argument("--fc-hz", type=float, default=DEFAULT_FC_HZ, help="PI bandwidth reference (Hz)")
    ap.add_argument("--rounds", type=int, default=DEFAULT_ROUNDS)
    ap.add_argument("--off-rounds", type=int, default=DEFAULT_OFF_ROUNDS)
    ap.add_argument("--fixed-rounds", type=int, default=DEFAULT_FIXED_ROUNDS)
    args = ap.parse_args()
    text = run_analysis(
        args.csv,
        fc_hz=args.fc_hz,
        rounds=args.rounds,
        off_rounds=args.off_rounds,
        fixed_rounds=args.fixed_rounds,
    )
    print(text)
    if args.md:
        args.md.write_text(text, encoding="utf-8")
        print(f"\nWrote {args.md}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
