#!/usr/bin/env python3
"""
为 Id 扫表 VOFA CSV 生成「下一 AI 可读」的完整诊断报告。

合并：扫表稳态、Pass0/Pass1 三相纹波、LUT 突发全表、固件配置推断、诊断要点。

用法:
  python tools/generate_daily_diagnosis_report.py VOFA+CSV/20260625/vofa+202606260819.csv
  python tools/generate_daily_diagnosis_report.py --batch VOFA+CSV/20260625/vofa+20260626*.csv
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from analyze_id_cal_sweep_vofa import (  # noqa: E402
    D_TO_PHASE_COS,
    IQ_PROBE_FIXED_S,
    IQ_PROBE_OFF_S,
    LUT_HDR,
    LUT_TAIL,
    RS_OHM,
    analyze_sweep,
    find_iq_probe_start,
    find_pass1_end,
    iq_probe_segment_times,
    load_csv,
    parse_lut_burst,
    park,
    recording_has_pass1,
    unwrap,
)
from parse_lut_vofa import decode_proto, format_lut_burst_md  # noqa: E402

FS = 20000.0
VBUS = 24.0
DEADTIME_NS = 591
V_COMP_THEORY = VBUS * DEADTIME_NS * 1e-9 * FS  # ~0.709 V/相 fixed

# Iq 探路段固件目标 (A)，与 M1_ID_CAL_IQ_PROBE_A / M1_IQ_REF_A 一致
IQ_PROBE_REF_A = 0.5
IQ_PROBE_REF_TOL_A = 0.12
# 与 motor_params ID_CAL_DUAL_FULL 默认一致（Pass0→Iq 直切 LUT ON）
# legacy Pass1→OFF→LUT 时改为 OFF=10、FIXED=5
IQ_PROBE_OFF_CORE_SKIP_START_S = 1.0
IQ_PROBE_OFF_CORE_SKIP_END_S = 1.5
IQ_PROBE_ON_CORE_SKIP_START_S = 0.5

# Pass1 abc 路径：与 1807/1025 一致，Id_ref 低于此视为「小电流失配区」
PASS1_LOW_ID_FAIL_A = 0.42
# phase LUT：amp 低于此的档视为小电流查表区（runtime 弱相会查到这些 val）
LUT_LOW_AMP_MARK_A = 0.35

# proto → 录波日固件配置（与 telem_lut_dump / motor_params 一致）
PROTO_FIRMWARE = {
    3.1: {
        "label": "平坦化 baseline",
        "low_flat": True,
        "i_zero_disable": False,
        "apply_min_a": None,
        "apply_ud": False,
        "pass1_max_a": 1.5,
    },
    3.2: {
        "label": "I_ZERO 关 A/B",
        "low_flat": True,
        "i_zero_disable": True,
        "apply_min_a": None,
        "apply_ud": False,
        "pass1_max_a": 3.0,
    },
    3.3: {
        "label": "LUT_APPLY_MIN=0.4 A",
        "low_flat": True,
        "i_zero_disable": False,
        "apply_min_a": 0.40,
        "apply_ud": False,
        "pass1_max_a": 3.0,
    },
}


def find_lut_row(ia: np.ndarray) -> int:
    for i, v in enumerate(ia):
        if abs(v - LUT_HDR) < 1.0:
            return i
    return -1


def dwell_abc_stats(
    ia, ib, ic, ud, id_ref, theta, t0: float, t1: float
) -> dict | None:
    i0, i1 = int(t0 * FS), int(t1 * FS)
    if i1 - i0 < int(0.08 * FS):
        return None
    sl = slice(i0, i1)
    th = theta[sl]
    id_, iq = park(ia[sl], ib[sl], ic[sl], th)
    ia_s, ib_s, ic_s = ia[sl], ib[sl], ic[sl]
    id_mean = float(np.mean(id_))
    return {
        "id_ref": float(np.median(id_ref[sl])),
        "id_fb": id_mean,
        "ud_pi": float(np.mean(ud[sl])),
        "ud_res": float(np.mean(ud[sl]) - id_mean * RS_OHM),
        "ia_pp": float(np.ptp(ia_s)),
        "ib_pp": float(np.ptp(ib_s)),
        "ic_pp": float(np.ptp(ic_s)),
        "abc_pp_mean": float(np.mean([np.ptp(ia_s), np.ptp(ib_s), np.ptp(ic_s)])),
        "iq_rms": float(np.sqrt(np.mean(iq**2))),
    }


TH_FIX = math.pi / 6.0


def valid_mask(ia, ib, ic):
    return (np.abs(ia) < 20) & (np.abs(ib) < 20) & (np.abs(ic) < 20)


def pass01_by_ref(
    ia, ib, ic, ud, id_ref, theta, lut_t: float, p1_end: float, refs: list[float]
) -> tuple[list[dict], list[dict]]:
    """按 Id_ref 目标值在 Pass0/Pass1 时间窗内取 dwell 末 25% 统计。"""
    t = np.arange(len(ia)) / FS
    v = valid_mask(ia, ib, ic)
    p0_rows: list[dict] = []
    p1_rows: list[dict] = []

    for ref in refs:
        m0 = v & (t < lut_t - 0.2) & (np.abs(id_ref - ref) < 0.025)
        m1 = v & (t > lut_t + 1.0) & (t < p1_end) & (np.abs(id_ref - ref) < 0.025)
        if m0.sum() >= 200:
            idx = np.where(m0)[0]
            sl = slice(int(idx[int(len(idx) * 0.75)]), idx[-1])
            p0_rows.append(_stats_slice(ia, ib, ic, ud, id_ref, theta, sl, ref))
        if m1.sum() >= 200:
            idx = np.where(m1)[0]
            sl = slice(int(idx[int(len(idx) * 0.75)]), idx[-1])
            p1_rows.append(_stats_slice(ia, ib, ic, ud, id_ref, theta, sl, ref))
    return p0_rows, p1_rows


def _stats_slice(ia, ib, ic, ud, id_ref, theta, sl: slice, ref: float) -> dict:
    ia_s, ib_s, ic_s = ia[sl], ib[sl], ic[sl]
    th = theta[sl]
    id_, iq = park(ia_s, ib_s, ic_s, th)
    id_mean = float(np.mean(id_))
    return {
        "id_ref": ref,
        "id_fb": id_mean,
        "ud_pi": float(np.mean(ud[sl])),
        "ud_res": float(np.mean(ud[sl]) - id_mean * RS_OHM),
        "ia_pp": float(np.ptp(ia_s)),
        "ib_pp": float(np.ptp(ib_s)),
        "ic_pp": float(np.ptp(ic_s)),
        "abc_pp_mean": float(np.mean([np.ptp(ia_s), np.ptp(ib_s), np.ptp(ic_s)])),
        "iq_rms": float(np.sqrt(np.mean(iq**2))),
    }


def scan_id_dwells(
    ia, ib, ic, ud, id_ref, theta, t_start: float, t_end: float
) -> list[dict]:
    rows: list[dict] = []
    i = int(t_start * FS)
    i_end = int(t_end * FS)
    while i < i_end:
        ref = id_ref[i]
        if ref < 0.04:
            i += int(0.05 * FS)
            continue
        j = i + 1
        while j < i_end and abs(id_ref[j] - ref) < 0.008:
            j += 1
        dwell = (j - i) / FS
        if dwell > 0.25:
            t0 = i / FS + dwell * 0.75
            t1 = j / FS - 0.05
            s = dwell_abc_stats(ia, ib, ic, ud, id_ref, theta, t0, t1)
            if s:
                rows.append(s)
        i = j
    return rows


def iq_probe_seg_stats(
    ia, ib, ic, ud, theta, t: np.ndarray, t0: float, t1: float
) -> dict | None:
    """单段 Iq 探路统计（Park 量 + abc 纹波 + 转速）。"""
    m = (t >= t0) & (t < t1)
    if int(m.sum()) < 2000:
        return None
    th_u = unwrap(theta[m])
    dt = float(t[m][-1] - t[m][0])
    dth_rate = float(np.degrees(th_u[-1] - th_u[0])) / dt if dt > 0.01 else 0.0
    id_p, iq_p = park(ia[m], ib[m], ic[m], theta[m])
    iq_mean = float(np.mean(iq_p))
    abc_pp = float(np.mean([np.ptp(ia[m]), np.ptp(ib[m]), np.ptp(ic[m])]))
    return {
        "t0": t0,
        "t1": t1,
        "iq_mean": iq_mean,
        "iq_rms": float(np.std(iq_p)),
        "iq_err": abs(iq_mean - IQ_PROBE_REF_A),
        "id_mean": float(np.mean(id_p)),
        "id_rms": float(np.std(id_p)),
        "abc_pp": abc_pp,
        "ud_pp": float(np.ptp(ud[m])),
        "ud_rms": float(np.std(ud[m])),
        "dth_rate": dth_rate,
        "rotating": abs(dth_rate) >= 30.0,
        "stall": abs(dth_rate) < 30.0,
    }


def iq_probe_analyze(
    ia, ib, ic, ud, id_ref, theta, t: np.ndarray, iq_start: float, duration_s: float
) -> dict | None:
    """Iq 探路：Pass0→LUT 直切，或 legacy OFF/FIXED/LUT A/B。"""
    if duration_s <= iq_start + 1.5:
        return None

    seg = iq_probe_segment_times(iq_start, duration_s, IQ_PROBE_OFF_S, IQ_PROBE_FIXED_S)
    iq0 = seg["iq0"]
    iq_off_end = seg["iq_off_end"]
    iq_fixed_end = seg["iq_fixed_end"]
    iq_on_end = seg["iq_on_end"]
    lut_on_start = seg["lut_on_start"]

    if iq_on_end <= lut_on_start + 0.5:
        return None

    off_full = None
    if IQ_PROBE_OFF_S > 0.0 and iq_off_end > iq0 + 0.2:
        off_full = iq_probe_seg_stats(ia, ib, ic, ud, theta, t, iq0, iq_off_end)

    on_full = iq_probe_seg_stats(ia, ib, ic, ud, theta, t, lut_on_start, iq_on_end)

    off_core = None
    if IQ_PROBE_OFF_S > 0.0:
        off_core_t0 = iq0 + IQ_PROBE_OFF_CORE_SKIP_START_S
        off_core_t1 = iq_off_end - IQ_PROBE_OFF_CORE_SKIP_END_S
        if off_core_t1 > off_core_t0 + 1.0:
            off_core = iq_probe_seg_stats(ia, ib, ic, ud, theta, t, off_core_t0, off_core_t1)
        else:
            off_core = off_full

    on_core_t0 = lut_on_start + IQ_PROBE_ON_CORE_SKIP_START_S
    on_core_t1 = iq_on_end
    on_core = (
        iq_probe_seg_stats(ia, ib, ic, ud, theta, t, on_core_t0, on_core_t1)
        if on_core_t1 > on_core_t0 + 1.0
        else on_full
    )

    return {
        "iq0": iq0,
        "iq_off_end": iq_off_end,
        "iq_on_end": iq_on_end,
        "lut_on_start": lut_on_start,
        "direct_lut": IQ_PROBE_OFF_S <= 0.0 and IQ_PROBE_FIXED_S <= 0.0,
        "off_full": off_full,
        "on_full": on_full,
        "off_core": off_core,
        "on_core": on_core,
    }


def iq_probe_format_row(label: str, s: dict | None) -> str:
    if s is None:
        return f"| {label} | — | — | — | — | — | — | — |"
    rot = "旋转" if s["rotating"] else "堵转/未转"
    return (
        f"| {label} | {s['t0']:.1f}～{s['t1']:.1f} | {s['iq_mean']:+.3f} | {s['iq_rms']:.3f} | "
        f"{s['id_mean']:+.3f} | {s['abc_pp']:.3f} | {s['ud_pp']:.3f} | {s['dth_rate']:.0f} | {rot} |"
    )


def iq_probe_diagnosis_hints(iq: dict | None) -> list[str]:
    if iq is None:
        return []

    hints: list[str] = []
    off = iq.get("off_core") or iq.get("off_full")
    on = iq.get("on_core") or iq.get("on_full")
    if on is None:
        return ["Iq 探路段数据不足，未生成 LUT ON 统计。"]

    if iq.get("direct_lut"):
        hints.append(
            "Pass0→Iq 直切 phase LUT ON（无 OFF 基线段）；重点看 LUT 稳态 abc_pp、Id std、是否正反馈。"
        )
        if on["abc_pp"] > 5.0:
            hints.append(
                f"Iq+LUT：abc_pp={on['abc_pp']:.2f} A 过大 — 检查 M1_DEADBAND_LUT_RUNTIME_SCALE、Id PI 是否开启。"
            )
        elif on["abc_pp"] <= 2.5 and on["rotating"]:
            hints.append(f"Iq+LUT 旋转稳态：abc_pp≈{on['abc_pp']:.2f} A ✅（相对 1457 满幅 LUT 有改善嫌疑）")
        if on["id_rms"] > 1.0:
            hints.append(f"Iq+LUT：Id std={on['id_rms']:.2f} A 偏大，确认 M1_ID_CAL_IQ_PROBE_ID_PI_ENABLE=1。")
        if on["iq_rms"] > 0.5:
            hints.append(f"Iq+LUT：Iq RMS={on['iq_rms']:.3f} A 过大，电流环振荡/补偿过强。")
        return hints

    if off is None or on is None:
        return ["Iq 探路段数据不足，未生成补偿前/后对比。"]

    if off["stall"] and not on["stall"]:
        hints.append(
            "Iq 探路：**补偿前堵转、补偿后才转** — 两段工况不同，勿仅用 abc 大小评判 LUT；"
            "应看同处于旋转态的稳态窗（§7.3）。"
        )
    elif off["rotating"] and on["rotating"]:
        ratio = on["abc_pp"] / off["abc_pp"] if off["abc_pp"] > 0.05 else float("nan")
        if ratio > 1.5:
            hints.append(
                f"Iq 旋转探路：**补偿后 abc_pp 劣化** "
                f"OFF={off['abc_pp']:.2f} A → ON={on['abc_pp']:.2f} A（×{ratio:.1f}）— "
                "phase abc duty 注入路径在旋转下可能过补/坐标不对；Pass1 锁轴 Ud 验收与 Iq 旋转验收应分开看。"
            )
        elif ratio <= 1.2 and on["abc_pp"] <= 2.0:
            hints.append(
                f"Iq 旋转探路：补偿后 abc_pp≈{on['abc_pp']:.2f} A，与补偿前 {off['abc_pp']:.2f} A 同级 ✅"
            )
        else:
            hints.append(
                f"Iq 旋转探路：补偿后 abc_pp={on['abc_pp']:.2f} A vs 补偿前 {off['abc_pp']:.2f} A "
                f"（×{ratio:.1f}），需结合 Iq 跟踪与 per-second 波形人工确认。"
            )

    if off["iq_err"] > IQ_PROBE_REF_TOL_A:
        hints.append(
            f"Iq 探路补偿前：iq_fb 均值 {off['iq_mean']:.3f} A，距目标 {IQ_PROBE_REF_A} A "
            f"偏差 {off['iq_err']:.3f} A（转速高时 Park Iq 常偏低，属联调现象）。"
        )
    if on["iq_rms"] > 0.5:
        hints.append(
            f"Iq 探路补偿后：Iq RMS={on['iq_rms']:.3f} A 过大，电流环在旋转+LUT 下振荡/发散。"
        )
    return hints


def iq_probe_summary(
    ia, ib, ic, ud, id_ref, theta, t: np.ndarray, iq_start: float, duration_s: float,
    csv_name: str = "<csv>",
) -> tuple[list[str], dict | None]:
    """Iq 探路 Markdown（Pass0→LUT 或 legacy A/B）。"""
    iq = iq_probe_analyze(ia, ib, ic, ud, id_ref, theta, t, iq_start, duration_s)
    if iq is None:
        return [], None

    off = iq.get("off_core") or iq.get("off_full")
    on = iq.get("on_core") or iq.get("on_full")
    ratio = (
        on["abc_pp"] / off["abc_pp"]
        if off and on and off["abc_pp"] > 0.05
        else float("nan")
    )

    if iq.get("direct_lut"):
        section_title = "## 7. Iq 探路段（Pass0 commit 后 phase LUT ON）"
        timing = "Pass0 decay → commit → open_seq **51** 至停录"
        switch_note = f"| LUT ON 起点 | t ≈ {iq.get('lut_on_start', iq['iq0']):.1f} s |"
        compare_title = "### 7.1 LUT ON 稳态核心窗（推荐判读）"
        core_note = (
            f"> 核心窗：去掉 LUT 切入后首 {IQ_PROBE_ON_CORE_SKIP_START_S:.1f} s 瞬态。"
        )
        off_row_label = None
    else:
        section_title = "## 7. Iq 探路段（Pass1 后，同次录波 A/B）"
        timing = f"**{IQ_PROBE_OFF_S:.0f} s OFF** → LUT ON 至停录"
        switch_note = f"| 补偿前→后切换 | t ≈ {iq['iq_off_end']:.1f} s |"
        compare_title = "### 7.1 补偿前 / 补偿后对比（稳态核心窗，推荐判读）"
        core_note = (
            f"> 核心窗：去掉 OFF 首 {IQ_PROBE_OFF_CORE_SKIP_START_S:.0f} s 起转、"
            f"末 {IQ_PROBE_OFF_CORE_SKIP_END_S:.1f} s 切换前毛刺；"
            f"ON 首 {IQ_PROBE_ON_CORE_SKIP_START_S:.1f} s LUT 切换瞬态。"
        )
        off_row_label = "**补偿前**（LUT OFF）"

    lines = [
        "",
        section_title,
        "",
        "**说明**（与固件 open_seq **51** = phase LUT ON 对应）：",
        "",
        f"| 项 | 值 |",
        f"|----|-----|",
        f"| 固件 Iq 目标 | **{IQ_PROBE_REF_A} A** |",
        f"| 时序 | {timing} |",
        f"| Iq 段起点 | t ≈ {iq['iq0']:.1f} s |",
        switch_note,
        "",
        compare_title,
        "",
        core_note,
        "",
        "| 段 | 时段 (s) | Iq_fb (A) | Iq_rms | Id_fb (A) | abc_pp (A) | Ud_pp (V) | dθ/dt (°/s) | 状态 |",
        "|----|----------|-----------|--------|-----------|------------|-----------|-------------|------|",
    ]
    if off_row_label:
        lines.append(iq_probe_format_row(off_row_label, off))
    lines.append(iq_probe_format_row("**LUT ON**（phase 表）", on))
    lines.append("")

    if off and on and not math.isnan(ratio):
        verdict = (
            "补偿后劣于补偿前（phase 旋转注入 FAIL）"
            if ratio > 1.5
            else "补偿后与补偿前同级或更优"
            if ratio <= 1.2 and on["abc_pp"] <= 2.0
            else "需人工看图"
        )
        lines += [
            f"| 对比 | abc_pp **{off['abc_pp']:.3f} → {on['abc_pp']:.3f} A**（×{ratio:.2f}） | "
            f"Iq 偏差 {off['iq_err']:.3f} → {on['iq_err']:.3f} A | "
            f"Iq RMS {off['iq_rms']:.3f} → {on['iq_rms']:.3f} A | 判定：{verdict} |",
            "",
        ]

    lines += [
        "### 7.2 全段统计（含切换边界，供对照）",
        "",
        "| 段 | 时段 (s) | Iq_fb (A) | Iq_rms | abc_pp (A) | dθ/dt (°/s) | 状态 |",
        "|----|----------|-----------|--------|------------|-------------|------|",
    ]
    for label, key in [("补偿前全段", "off_full"), ("LUT ON 全段", "on_full")]:
        s = iq.get(key)
        if s:
            lines.append(
                f"| {label} | {s['t0']:.1f}～{s['t1']:.1f} | {s['iq_mean']:+.3f} | "
                f"{s['iq_rms']:.3f} | {s['abc_pp']:.3f} | {s['dth_rate']:.0f} | "
                f"{'旋转' if s['rotating'] else '堵转/未转'} |"
            )
    if iq.get("direct_lut"):
        lines += [
            "",
            "> Pass0→LUT 直切：无 OFF 基线；判读看 §7.1 LUT 稳态 abc_pp、Id、Iq RMS。",
            f"> 逐秒波形：`python tools/_analyze_iq_probe.py {csv_name}`。",
        ]
    else:
        lines += [
            "",
            "> 全段 OFF 常因末秒切换前 abc 抬升而 **高估** 补偿前纹波；**§7.1 核心窗** 更可靠。",
            f"> 逐秒波形：`python tools/_analyze_iq_probe.py {csv_name}`。",
        ]
    return lines, iq


def infer_pass1_max(segments: list[dict]) -> float:
    refs = [s["id_ref"] for s in segments if s["id_ref"] > 0.5]
    return max(refs) if refs else 1.5


def firmware_table(proto: float, ch5: float) -> str:
    key = round(proto, 1)
    cfg = PROTO_FIRMWARE.get(key, {})
    apply_min = ch5 if abs(proto - 3.3) < 0.05 else cfg.get("apply_min_a")
    min_txt = f"{apply_min}" if apply_min is not None else "无（全 |i| 注入）"
    lines = [
        "| 项目 | 值 |",
        "|------|-----|",
        f"| proto | **{proto:.1f}** ({cfg.get('label', '见 telem_lut_dump.h')}) |",
        f"| 低区平坦化 commit | {'ON' if cfg.get('low_flat') else '未知'} |",
        f"| I_ZERO_DISABLE | {'1（关过零硬开关）' if cfg.get('i_zero_disable') else '0（默认）'} |",
        f"| LUT_APPLY_MIN_A | {min_txt} |",
        f"| LUT 注入路径 | {'Ud d轴' if cfg.get('apply_ud') else 'phase abc duty'} |",
        f"| Pass1 预期 Id_max | {cfg.get('pass1_max_a', infer_pass1_max([]))} A |",
        f"| Pass0 | deadband OFF，0.05～1.50 A / 32 点，30° 锁轴 |",
        f"| Capture | `Ud_res = \|Ud_pi − Id×Rs\|`，Rs={RS_OHM} Ω |",
        f"| Commit | flatten_low_dlut(0.05～0.30 A) → ×0.866 → phase 表 |",
        f"| 理论固定死区 | {V_COMP_THEORY:.3f} V/相（591 ns @ 24 V） |",
    ]
    return "\n".join(lines)


def format_low_id_lut_section(
    lut: dict | None,
    p0_rows: list[dict],
    p1_rows: list[dict],
) -> list[str]:
    """小电流区：LUT 低 amp 档 + Pass1 失配档标注。"""
    lines = [
        "",
        "## 5b. 小电流区 LUT 与 Pass1 abc 失配（重点）",
        "",
        f"> **分界**：Id_ref < **{PASS1_LOW_ID_FAIL_A} A** 时 Pass1 abc 常 FAIL；"
        f"≥ **{PASS1_LOW_ID_FAIL_A} A** 高区常稳（与 1807/1025 Step1 一致）。",
        f"> LUT **小电流档**：phase amp < **{LUT_LOW_AMP_MARK_A} A** 的表点（runtime 弱相 \|i\\| 会落在此段查表）。",
        "",
    ]
    if lut and lut.get("amps") is not None:
        amps = lut["amps"]
        vals = lut["vals"]
        lines += [
            "### phase LUT 小电流档（amp < {:.2f} A）".format(LUT_LOW_AMP_MARK_A),
            "",
            "| # | amp (A) | val (V/相) | 标记 |",
            "|---|---------|------------|------|",
        ]
        for i, (a, v) in enumerate(zip(amps, vals)):
            if float(a) >= LUT_LOW_AMP_MARK_A:
                continue
            plateau = ""
            if i + 1 < len(vals) and abs(float(v) - float(vals[i + 1])) < 1e-4:
                plateau = "平台/重复"
            lines.append(
                f"| **{i + 1}** | **{float(a):.4f}** | **{float(v):.4f}** | 小电流档 {plateau} |"
            )
        lines.append("")
    else:
        lines.append("*（无 LUT 突发，跳过表档列表）*\n")

    lines += [
        "### Pass1 小电流区各档（LUT ON + apply_duty）",
        "",
        "| Id_ref | Id_fb | P0 abc | P1 abc | P1/P0 | 判定 | 备注 |",
        "|--------|-------|--------|--------|-------|------|------|",
    ]
    p0m = {round(r["id_ref"], 3): r for r in p0_rows}
    for r in sorted(p1_rows, key=lambda x: x["id_ref"]):
        ref = r["id_ref"]
        if ref >= PASS1_LOW_ID_FAIL_A:
            continue
        r0 = min(p0m.keys(), key=lambda k: abs(k - ref), default=None)
        a0 = p0m[r0]["abc_pp_mean"] if r0 is not None and abs(r0 - ref) <= 0.05 else float("nan")
        a1 = r["abc_pp_mean"]
        ratio = f"{a1/a0:.2f}×" if not math.isnan(a0) and a0 > 0.01 else "—"
        fail = a1 > 0.5 and (math.isnan(a0) or a1 > a0 * 1.5)
        verdict = "**FAIL ❌**" if fail else "边缘"
        id_err = abs(r["id_fb"] - ref)
        note = ""
        if id_err > 0.05:
            note = f"Id失锁 fb={r['id_fb']:.3f}"
        elif r["ib_pp"] < 0.35 and r["ia_pp"] > 1.0:
            note = "Ib 弱相仍注入"
        lines.append(
            f"| **{ref:.3f}** | {r['id_fb']:.3f} | "
            f"{a0:.3f} | **{a1:.3f}** | {ratio} | {verdict} | {note} |"
        )

    lines += [
        "",
        "**机理（重复出现的原因）**：锁轴 30° 纯 Id 时 Ib/Ic 很小，"
        "`apply_duty` 仍按 \|i_phase\| 查上表 val 注入 duty → dq 耦合 → Id 失锁 → abc 放大。"
        "高 Id 时三相 \|i\| 均落入表中高区，注入与 Pass0 capture 一致，故 **大电流区好、小电流区差**。"
        "",
    ]
    return lines


def diagnosis_hints(
    proto: float,
    ch5: float,
    p0_rows: list[dict],
    p1_rows: list[dict],
    lut: dict | None,
) -> list[str]:
    hints: list[str] = []
    p0 = {round(r["id_ref"], 2): r for r in p0_rows}
    p1 = {round(r["id_ref"], 2): r for r in p1_rows}

    def abc(ref):
        k = min(p0.keys(), key=lambda x: abs(x - ref), default=None)
        if k is None or abs(k - ref) > 0.04:
            return None, None
        k1 = min(p1.keys(), key=lambda x: abs(x - ref), default=None)
        if k1 is None or abs(k1 - ref) > 0.04:
            return p0[k]["abc_pp_mean"], None
        return p0[k]["abc_pp_mean"], p1[k1]["abc_pp_mean"]

    for ref in [0.05, 0.20, 0.35, 0.47, 1.0, 3.0]:
        a0, a1 = abc(ref)
        if a0 is None:
            continue
        if a1 is None:
            hints.append(f"Id_ref≈{ref:.2f} A：仅 Pass0 abc_pp≈{a0:.2f} A（Pass1 无对应档）")
        elif a1 > a0 * 2.0 and ref < 0.45:
            hints.append(
                f"Id_ref≈{ref:.2f} A：**Pass1 毛刺** P0={a0:.2f} → P1={a1:.2f} A p2p（×{a1/a0:.1f}）"
            )
        elif a1 <= a0 * 1.3 and ref < 0.45:
            hints.append(
                f"Id_ref≈{ref:.2f} A：Pass1 纹波正常 P0={a0:.2f} P1={a1:.2f} A p2p ✅"
            )
        elif ref >= 0.47 and a1 is not None and a1 < 0.35:
            hints.append(f"Id_ref≈{ref:.2f} A：Pass1 高区稳定 abc_pp≈{a1:.2f} A ✅")

    if abs(proto - 3.3) < 0.05:
        hints.append(
            f"proto 3.3：\|i_phase\|<{ch5:.2f} A 时 **runtime 不注入 LUT**；"
            "低 Id Pass1 改善可能来自「少注入」而非 LUT 正确补偿。"
        )
        r35 = p1.get(0.35) or p1.get(0.34) or p1.get(0.36)
        if r35 and r35["abc_pp_mean"] > 2.0:
            hints.append(
                "Id≈0.35 A 附近 Pass1 纹波仍大：可能 \|Ic\| 刚超过 APPLY_MIN 导致单相过补偿 cliff。"
            )

    if abs(proto - 3.2) < 0.05:
        hints.append("proto 3.2：I_ZERO 已关；若低 Id 仍差，问题在 phase abc 注入路径或 capture 量纲。")

    if lut and lut.get("outlier"):
        hints.append("Pass0 outlier=1：曾见 \|Ud_res\|>1 V，首档 0.05 A capture 可能 FAIL。")

    if not hints:
        hints.append("（无自动规则命中；请人工看 Pass0/Pass1 表与 LUT 全表）")
    return hints


def generate_report(csv_path: Path) -> tuple[str, dict]:
    ia, ib, ic, ud_pi, id_ref, theta_raw = load_csv(csv_path)
    theta = unwrap(theta_raw)
    rows = np.column_stack([ia, ib, ic, ud_pi, id_ref, theta_raw])
    n = len(ia)
    fs = FS if n > 100_000 else 200.0
    t = np.arange(n) / fs

    result = analyze_sweep(ia, ib, ic, ud_pi, id_ref, theta_raw, fs)
    lut = None
    lut_row = find_lut_row(ia)
    if lut_row >= 0:
        lut = parse_lut_burst(rows)
        if lut is not None:
            lut["ch5"] = float(rows[lut_row][5])

    lut_t = lut_row / fs if lut_row >= 0 else result["sweep_duration_s"]
    has_pass1 = recording_has_pass1(t, id_ref, ia, lut_t) if lut_row >= 0 else False
    p1_end = find_pass1_end(t, id_ref, ia, lut_t, fs) if has_pass1 else lut_t + 0.5
    iq_start = find_iq_probe_start(t, id_ref, ia, lut_t, fs)

    pass0_refs = [round(x, 4) for x in np.linspace(0.05, 1.5, 32)]
    pass1_ext = [round(x, 3) for x in np.linspace(1.633, 3.0, 11)]
    all_refs = pass0_refs + pass1_ext

    p0, p1 = pass01_by_ref(ia, ib, ic, ud_pi, id_ref, theta_raw, lut_t, p1_end, all_refs)
    max_ref = max((r["id_ref"] for r in p1), default=max((r["id_ref"] for r in p0), default=1.5))

    proto = float(lut["proto_ver"]) if lut else 0.0
    ch5 = float(lut.get("ch5", 0.0)) if lut else 0.0
    dec = decode_proto(proto, ch5) if lut else {}

    lines = [
        f"# 诊断报告 — `{csv_path.name}`",
        "",
        "> **给下一 AI**：本文件由 `tools/generate_daily_diagnosis_report.py` 自动生成。",
        "> 通道：I0-I2=Ia,Ib,Ic | I3=Ud_pi | I4=Id_ref | I5=theta_el(rad)。",
        f"> **Iq 探路目标**：`M1_ID_CAL_IQ_PROBE_A` = **{IQ_PROBE_REF_A} A**（Pass0 后 Iq + phase LUT）。",
        "> 协议详解：`VOFA+CSV/20260624/Id扫表与LUT标定_CSV分析指南_给AI.md` §6。",
        "",
        "## 1. 固件配置（由 LUT 头帧 proto 推断）",
        "",
        firmware_table(proto, ch5),
        "",
        "## 2. 录波概况",
        "",
        f"| 项 | 值 |",
        f"|----|-----|",
        f"| 文件 | `{csv_path.name}` |",
        f"| 采样率 | {fs:.0f} Hz |",
        f"| 总时长 | {result['duration_s']:.2f} s |",
        f"| Pass0 段 | t ≈ 0.5 ～ {lut_t:.2f} s（LUT 突发前） |",
        f"| LUT 突发 | t ≈ {lut_t:.2f} s，行 **{lut_row}** |",
    ]
    if has_pass1:
        lines.append(f"| Pass1 段 | t ≈ {lut_t + 0.8:.2f} ～ {p1_end:.2f} s |")
        lines.append(f"| Pass1 Id_max（扫表段） | {max_ref:.2f} A |")
    else:
        lines.append("| Pass1 段 | **无**（Pass0→Iq 直切 LUT ON） |")
    lines += [
        f"| Iq 探路起点 | t ≈ {iq_start:.2f} s |",
        f"| Iq 探路目标（固件） | **{IQ_PROBE_REF_A} A**（`M1_ID_CAL_IQ_PROBE_A`） |",
        f"| Iq 全局 RMS | {result['iq_global_rms']:.4f} A |",
        f"| θ 扫表段跨度 | {result['theta_full_span_deg']:.1f}° |",
        "",
    ]

    if lut:
        lines += [
            "## 3. LUT 遥测突发摘要",
            "",
            f"| len | Rs | outlier | proto | ch5 | 表域 | runtime |",
            f"|-----|-----|---------|-------|-----|------|---------|",
            f"| {lut['len']} | {lut['rs']:.4f} | {lut['outlier']} | {proto:.1f} | {ch5:.4g} | "
            f"{dec.get('table_domain', '?')} | {dec.get('runtime', '?')} |",
            "",
            f"sum(vals)：tail={lut['tail_sum']:.6f}，calc={lut['calc_sum']:.6f}。",
            "",
            "**表含义**：Pass0 capture 的 d 轴 `( |Id|, |Ud_res| )` 经 commit（平坦化 + ×0.866）"
            "后的 **phase 表**；数据帧 **amp=|i_phase| (A)**，**val=补偿电压 (V/相)**。",
            "",
        ]

    lines += [
        "## 4. Pass0 各档（deadband OFF，dwell 末 25%）",
        "",
        "| Id_ref | Id_fb | Ud_pi | Ud_res | ia_pp | ib_pp | ic_pp | abc_mean | Iq_rms |",
        "|--------|-------|-------|--------|-------|-------|-------|----------|--------|",
    ]
    for r in p0:
        lines.append(
            f"| {r['id_ref']:.3f} | {r['id_fb']:.3f} | {r['ud_pi']:.3f} | {r['ud_res']:.3f} | "
            f"{r['ia_pp']:.3f} | {r['ib_pp']:.3f} | {r['ic_pp']:.3f} | "
            f"{r['abc_pp_mean']:.3f} | {r['iq_rms']:.4f} |"
        )

    lines += [
        "",
        "## 5. Pass1 各档（LUT ON，dwell 末 25%）",
        "",
    ]
    if not p1:
        lines += ["> 本录波 **无 Pass1**（`M1_ID_CAL_LUT_VERIFY_SWEEP=0`，Pass0 commit 后直进 Iq）。", ""]
    lines += [
        "| Id_ref | Id_fb | Ud_pi | Ud_res | ia_pp | ib_pp | ic_pp | abc_mean | Iq_rms |",
        "|--------|-------|-------|--------|-------|-------|-------|----------|--------|",
    ]
    for r in p1:
        low = r["id_ref"] < PASS1_LOW_ID_FAIL_A
        mark = "**" if low else ""
        end = "**" if low else ""
        fail = low and r["abc_pp_mean"] > 0.5
        flag = " ⚠️小电流FAIL" if fail else (" ⚠️小电流区" if low else "")
        lines.append(
            f"| {mark}{r['id_ref']:.3f}{end} | {r['id_fb']:.3f} | {r['ud_pi']:.3f} | {r['ud_res']:.3f} | "
            f"{r['ia_pp']:.3f} | {r['ib_pp']:.3f} | {r['ic_pp']:.3f} | "
            f"{mark}{r['abc_pp_mean']:.3f}{end} | {r['iq_rms']:.4f} |{flag}"
        )

    lines += format_low_id_lut_section(lut, p0, p1)

    lines += ["", "## 6. Pass0 vs Pass1 纹波对比（abc_pp_mean）", ""]
    lines += ["| Id_ref | P0 abc | P1 abc | 比值 P1/P0 | P0 Ud_res | P1 Ud_res |", "|--------|--------|--------|------------|-----------|-----------|"]
    p0m = {round(r["id_ref"], 2): r for r in p0}
    p1m = {round(r["id_ref"], 2): r for r in p1}
    all_refs = sorted(set(p0m.keys()) | set(p1m.keys()))
    for ref in all_refs:
        if ref > 3.05:
            continue
        r0 = min(p0m.keys(), key=lambda k: abs(k - ref), default=None)
        r1 = min(p1m.keys(), key=lambda k: abs(k - ref), default=None)
        if r0 is None or abs(r0 - ref) > 0.05:
            continue
        s0 = p0m[r0]
        s1 = p1m[r1] if r1 is not None and abs(r1 - ref) <= 0.05 else None
        ratio = f"{s1['abc_pp_mean']/s0['abc_pp_mean']:.2f}×" if s1 and s0["abc_pp_mean"] > 0.01 else "—"
        p1abc = f"{s1['abc_pp_mean']:.3f}" if s1 else "—"
        p1ud = f"{s1['ud_res']:.3f}" if s1 else "—"
        low = ref < PASS1_LOW_ID_FAIL_A
        mark = "**" if low else ""
        end = "**" if low else ""
        tag = " ⚠️" if low and s1 and s1["abc_pp_mean"] > 0.5 else ""
        lines.append(
            f"| {mark}{ref:.2f}{end} | {s0['abc_pp_mean']:.3f} | {mark}{p1abc}{end} | {ratio} | "
            f"{s0['ud_res']:.3f} | {p1ud} |{tag}"
        )

    iq_lines, iq_stats = iq_probe_summary(
        ia, ib, ic, ud_pi, id_ref, theta_raw, t, iq_start, result["duration_s"],
        csv_name=csv_path.name,
    )
    lines += iq_lines

    lines += ["", "## 8. 自动诊断要点", ""]
    for h in diagnosis_hints(proto, ch5, p0, p1, lut):
        lines.append(f"- {h}")
    for h in iq_probe_diagnosis_hints(iq_stats):
        lines.append(f"- {h}")
    lines.append("")

    if lut:
        # Re-parse for full md block
        from parse_lut_vofa import parse_lut, format_lut_burst_md as fmt_full

        lut_full = parse_lut(rows)
        lines.append("---")
        lines.append("")
        lines.append(fmt_full(lut_full, csv_path.name))

    lines.append("")
    lines.append("---")
    lines.append("")
    lines.append("*生成：`python tools/generate_daily_diagnosis_report.py* "
                 f"`{csv_path.as_posix()}`*")
    tag = csv_path.stem.replace("vofa+", "")[-4:]
    summary = {
        "tag": tag,
        "proto": proto,
        "ch5": ch5,
        "max_ref": max_ref,
        "p1_abc": {ref: extract_p1_abc(p1, ref) for ref in [0.05, 0.20, 0.35, 0.47, 1.0]},
    }
    return "\n".join(lines), summary


def extract_p1_abc(p1_rows: list[dict], ref: float) -> str:
    r = min(p1_rows, key=lambda x: abs(x["id_ref"] - ref), default=None)
    if r is None or abs(r["id_ref"] - ref) > 0.04:
        return "—"
    return f"{r['abc_pp_mean']:.2f}"


def write_index(reports: list[tuple[Path, Path]], summaries: list[dict], out: Path) -> None:
    lines = [
        "# 2026-06-26 CSV 诊断索引（给下一 AI）",
        "",
        "> 本目录当日共 **4** 条 Id 锁轴扫表录波，按 proto 演进顺序排列。",
        "> 每条均有独立 **`诊断报告_vofa+*.md`**，含 LUT 全表 + Pass0/Pass1 对比。",
        "",
        "## 快速选文件",
        "",
        "| 时间戳 | 文件 | proto | 关键变量 | 报告 |",
        "|--------|------|-------|----------|------|",
    ]
    meta = [
        ("0751", "3.1", "LOW_FLAT，I_ZERO 开，Pass1→1.5 A", "低 Id Pass1 仍 ~1.2 A p2p"),
        ("0807", "3.2", "I_ZERO **关**，Pass1→**3 A**", "关 I_ZERO 仅略改善低 Id"),
        ("0816", "3.3", "APPLY_MIN=0.4 A，Pass1→**1.5 A**（早停）", "MIN 门控首测，未扫 3 A"),
        ("0819", "3.3", "APPLY_MIN=0.4 A，Pass1→**3 A**", "低 Id 改善但 0.35 A cliff"),
    ]
    for tag, proto, var, note in meta:
        csv = f"vofa+20260626{tag}.csv"
        rpt = f"诊断报告_vofa+20260626{tag}.md"
        lines.append(f"| {tag} | `{csv}` | {proto} | {var} | [`{rpt}`]({rpt}) |")
    lines += [
        "",
        "## 读 CSV 必知",
        "",
        "1. **CSV 无文字注释**；LUT 表在文件末尾 `I0=−888888` 突发帧。",
        "2. **amp/val 是 phase 域**（commit 后），不是 Pass0 原始 d 轴。",
        "3. **Capture 公式**：`Ud_res = |Ud_pi − Id×0.115|`；commit ×0.866。",
        "4. **Pass0** = LUT 突发之前；**Pass1** = 突发之后、Iq 探路之前。",
        "5. 理论固定死区 ≈ **0.709 V/相**（591 ns）；LUT 低区 ~0.28 V 合理，高 Id ~1.5 V 是 d 轴整包。",
        "",
        "## 诊断流程建议",
        "",
        "```text",
        "打开对应 诊断报告_*.md",
        "  → §7 自动要点",
        "  → §6 Pass0 vs Pass1（低 Id 看 abc 比值）",
        "  → 附录 LUT 全表（对照 Ud_res）",
        "  → 若需 deep dive：分析结论_vofa+*.md（人工结论）",
        "```",
        "",
        "## 相关人工报告",
        "",
        "| CSV | 深度分析 |",
        "|-----|----------|",
        "| 0807 | [`分析结论_vofa+202606260807_I_ZERO关与Pass1至3A_2026-06-26.md`](分析结论_vofa+202606260807_I_ZERO关与Pass1至3A_2026-06-26.md) |",
        "| 0819 | [`分析结论_vofa+202606260819_应补vs实补与MIN门控_2026-06-26.md`](分析结论_vofa+202606260819_应补vs实补与MIN门控_2026-06-26.md) |",
        "| 0751/0816 | 仅自动 [`诊断报告_*.md`](诊断报告_vofa+202606260751.md) |",
        "",
        "## 四录波横向对比（Pass1 低 Id abc_pp_mean）",
        "",
        "| Id_ref | 0751 proto3.1 | 0807 proto3.2 | 0816 proto3.3 | 0819 proto3.3 |",
        "|--------|---------------|---------------|---------------|---------------|",
    ]
    tags = ["0751", "0807", "0816", "0819"]
    smap = {s["tag"]: s for s in summaries}
    for ref in [0.05, 0.20, 0.35, 0.47, 1.0]:
        row = f"| {ref:.2f} A |"
        for tag in tags:
            row += f" {smap.get(tag, {}).get('p1_abc', {}).get(ref, '—')} |"
        lines.append(row)
    lines += [
        "",
        "（0816 Pass1 仅至 1.5 A；≥0.47 A 四录波应均 ~0.25–0.30 A。）",
        "",
        "## 工具",
        "",
        "```bash",
        "python tools/generate_daily_diagnosis_report.py VOFA+CSV/20260625/vofa+202606260819.csv",
        "python tools/parse_lut_vofa.py <csv> --md-full",
        "python tools/analyze_id_cal_sweep_vofa.py <csv> --compare-lut --md",
        "```",
    ]
    out.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="*", type=Path)
    ap.add_argument("--batch", nargs="+", type=Path, help="glob expanded csv list")
    ap.add_argument("-o", "--out-dir", type=Path, default=None)
    ap.add_argument("--index", type=Path, default=None, help="write master index md")
    args = ap.parse_args()

    paths = list(args.csv)
    if args.batch:
        paths.extend(args.batch)
    if not paths:
        ap.error("need csv path(s)")

    out_dir = args.out_dir or paths[0].parent
    written: list[tuple[Path, Path]] = []
    summaries: list[dict] = []

    for p in paths:
        p = p.resolve()
        text, summary = generate_report(p)
        out = out_dir / f"诊断报告_{p.stem}.md"
        out.write_text(text, encoding="utf-8")
        print(f"wrote {out}")
        written.append((p, out))
        summaries.append(summary)

    if args.index or len(written) > 1:
        idx = args.index or (out_dir / "今日CSV诊断索引_2026-06-26.md")
        write_index(written, summaries, idx)
        print(f"wrote {idx}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
