#!/usr/bin/env python3
"""Parse VOFA CSV for MCU deadband LUT telem burst (JustFloat 6ch)."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

HDR_MAGIC = -888888.0
TAIL_MAGIC = -999999.0
PROTO_VER = 3.0
D_TO_PHASE_COS = 0.8660254037844386
HDR_TOL = 1.0

# 头帧 ch4 proto 版本说明（与 motor/telem_lut_dump.c 一致）
PROTO_TABLE = {
    2.0: "d 轴表（旧）：amp=|Id|, val=|Ud_pi−Id·Rs|",
    3.0: "phase abc 表：amp=|Id|×0.866, val=|Ud_res|×0.866（无平坦化）",
    3.1: "phase abc + commit 低 Id 平坦化（ch5=1）",
    3.2: "phase abc + 平坦化 + I_ZERO 关（ch5=平坦化 flag）",
    3.3: "phase abc + 平坦化 + LUT_APPLY_MIN（ch5=门槛 A）",
    3.4: "geo 双角样本池 merge → 共享 phase abc 表（ch5=geo 样本数）",
    3.5: "geo 分相三表 f_a/f_b/f_c（ch5=geo 样本数）；数据帧 amp_a,val_a..amp_c,val_c",
}

CAPTURE_FORMULA = (
    "Pass0 每档 dwell 末 capture：`lut_d_amp=|Id|`, "
    "`lut_d_val=|Ud_pi − Id×Rs|`（`deadband_cal.c`）"
)
COMMIT_STEPS = (
    "commit：`flatten_low_dlut`（可选）→ `build_phase_lut`（×0.866）→ `deadband_set_lut`"
)
RUNTIME_NOTE = (
    "运行时（Pass1）：`deadband_apply_duty` 用 **|i_phase|** 查 **phase 表** 注入 duty；"
    "表内 val 单位为 **V/相**"
)


def load_csv(path: Path) -> np.ndarray:
    data = np.genfromtxt(path, delimiter=",", names=True, dtype=float)
    names = data.dtype.names
    if names is None or len(names) < 6:
        raise ValueError(f"need 6 columns, got {names}")
    return np.column_stack([data[names[i]] for i in range(6)])


def find_header(rows: np.ndarray) -> int:
    for i, row in enumerate(rows):
        if abs(row[0] - HDR_MAGIC) < HDR_TOL:
            return i
    return -1


def decode_proto(ch4: float, ch5: float) -> dict:
    """解读头帧 ch4/ch5 固件/runtime 含义。"""
    ver = float(ch4)
    meta: dict = {
        "proto_ver": ver,
        "ch5_raw": float(ch5),
        "table_domain": "phase abc" if ver >= 2.5 else "d-axis",
        "proto_desc": PROTO_TABLE.get(round(ver, 1), "未知 proto"),
    }
    if abs(ver - 3.3) < 0.05:
        meta["runtime"] = f"LUT_APPLY_MIN_A={ch5:.4f} A（|i_phase| 低于此不注入）"
    elif abs(ver - 3.2) < 0.05:
        meta["runtime"] = f"I_ZERO_DISABLE=1, LOW_FLAT ch5={ch5:.0f}"
    elif abs(ver - 3.1) < 0.05:
        meta["runtime"] = "LOW_FLAT_ENABLE=1（ch5=1）"
    elif abs(ver - 3.4) < 0.05:
        meta["runtime"] = f"geo merge 共享 phase 表，geo_samples={int(ch5)}"
    elif abs(ver - 3.5) < 0.05:
        meta["runtime"] = f"geo 分相三表 A/B/C，geo_samples={int(ch5)}"
    elif abs(ver - 3.0) < 0.05:
        meta["runtime"] = "phase 表，无平坦化/MIN 门控"
    elif ver < 2.5:
        meta["runtime"] = "d 轴 Ud 注入或旧路径"
    else:
        meta["runtime"] = f"ch5={ch5}"
    return meta


def parse_lut(rows: np.ndarray) -> dict:
    hi = find_header(rows)
    if hi < 0:
        raise ValueError("LUT header magic not found (ch0 ≈ -888888)")

    hdr = rows[hi]
    length = int(round(hdr[1]))
    rs = float(hdr[2])
    outlier = int(round(hdr[3]))
    ver = float(hdr[4])
    ch5 = float(hdr[5])

    if abs(ver - PROTO_VER) > 0.5 and abs(ver - 2.0) > 0.5 and ver < 2.5:
        print(f"warn: proto ver {ver}", file=sys.stderr)

    amps: list[float] = []
    vals: list[float] = []
    amps_ph: list[list[float]] = [[], [], []]
    vals_ph: list[list[float]] = [[], [], []]
    triplet = abs(ver - 3.5) < 0.05
    i = hi + 1
    n_rows = len(rows)

    while i < n_rows:
        row = rows[i]
        if abs(row[0] - TAIL_MAGIC) < HDR_TOL:
            tail_len = int(round(row[1]))
            tail_sum = float(row[2])
            if tail_len != length:
                raise ValueError(f"tail len {tail_len} != header len {length}")
            if triplet:
                if len(amps_ph[0]) != length:
                    raise ValueError(
                        f"triplet got {len(amps_ph[0])} points, header says {length}"
                    )
                calc_sum = float(
                    np.sum(vals_ph[0]) + np.sum(vals_ph[1]) + np.sum(vals_ph[2])
                )
                amps = amps_ph[0]
                vals = vals_ph[0]
            else:
                if len(amps) != length:
                    raise ValueError(f"got {len(amps)} points, header says {length}")
                calc_sum = float(np.sum(vals))
            meta = decode_proto(ver, ch5)
            out = {
                "len": length,
                "rs": rs,
                "outlier": outlier,
                "proto_ver": ver,
                "ch5": ch5,
                "meta": meta,
                "amps": np.array(amps),
                "vals": np.array(vals),
                "tail_sum": tail_sum,
                "calc_sum": calc_sum,
                "header_row": hi,
                "tail_row": i,
                "hdr": hdr,
            }
            if triplet:
                out["triplet"] = True
                out["amps_ph"] = [np.array(x) for x in amps_ph]
                out["vals_ph"] = [np.array(x) for x in vals_ph]
            return out
        if abs(row[0] - HDR_MAGIC) < HDR_TOL:
            raise ValueError(f"duplicate header at row {i}")

        if triplet:
            if len(amps_ph[0]) < length:
                for ph in range(3):
                    amps_ph[ph].append(float(row[ph * 2]))
                    vals_ph[ph].append(float(row[ph * 2 + 1]))
        else:
            for k in (0, 2, 4):
                if len(amps) >= length:
                    break
                a, v = float(row[k]), float(row[k + 1])
                if a > 0.0 or v > 0.0:
                    amps.append(a)
                    vals.append(v)
        i += 1

    raise ValueError("LUT tail magic not found before EOF")


def format_lut_burst_md(lut: dict, csv_name: str = "") -> str:
    """生成可写入分析报告的 LUT 突发说明（Markdown）。"""
    meta = lut["meta"]
    hi, ti = lut["header_row"], lut["tail_row"]
    lines = [
        "## CSV 末端 LUT 遥测突发（表来源与数值含义）",
        "",
    ]
    if csv_name:
        lines.append(f"> 文件：`{csv_name}`")
        lines.append("")
    lines += [
        "### 突发位置",
        "",
        f"| 项 | 值 |",
        f"|----|-----|",
        f"| 头帧 CSV 行号（0-based） | **{hi}** |",
        f"| 尾帧 CSV 行号 | **{ti}** |",
        f"| 数据帧行数 | {ti - hi - 1}（含头 1 + 数据 + 尾 1） |",
        "",
        "### 头帧协议（I0=−888888 那一行）",
        "",
        "| 列 | 字段 | 本文件值 | 含义 |",
        "|----|------|----------|------|",
        f"| I0 | magic | −888888 | LUT 头帧标识 |",
        f"| I1 | len | {lut['len']} | phase 表点数 |",
        f"| I2 | Rs | {lut['rs']:.4f} Ω | capture 用 `Ud_res=Ud_pi−Id×Rs` |",
        f"| I3 | outlier | {lut['outlier']} | Pass0 曾见 \\|Ud_res\\|>1 V |",
        f"| I4 | proto | **{lut['proto_ver']:.1f}** | {meta['proto_desc']} |",
        f"| I5 | meta | **{lut['ch5']:.4g}** | {meta['runtime']} |",
        "",
        "### 表是怎么来的（固件 pipeline）",
        "",
        "1. **Pass0**（deadband OFF）：30° Id 锁轴扫 **0.05～1.50 A**，dwell 末 capture。",
        f"2. **Capture**：{CAPTURE_FORMULA}",
        f"3. **Commit**：{COMMIT_STEPS}",
        "4. **本突发发出的表** = commit 后的 **phase 域** `(amp, val)`，**不是** Pass0 原始 d 表。",
        "5. **Pass1** 起 runtime 查此 phase 表；Pass0 不查表。",
        "",
        f"**运行时**：{RUNTIME_NOTE}",
        "",
        "### 数据帧 / 尾帧",
        "",
        "| 帧 | I0 | I1 | I2 | I3 | I4 | I5 |",
        "|----|----|----|----|----|----|-----|",
        "| 数据帧 | amp0 | val0 | amp1 | val1 | amp2 | val2 |",
        "| 尾帧 −999999 | magic | len | sum(vals) | 0 | 0 | 0 |",
        "",
        f"尾帧 sum 校验：tail={lut['tail_sum']:.6f}，解析累加={lut['calc_sum']:.6f}。",
        "",
        "### phase 表 + 反推 d 轴（与 Pass0 capture 对照）",
        "",
        "| # | phase amp (A) | phase val (V) | Id≈amp/0.866 (A) | Ud_res≈val/0.866 (V) |",
        "|---|---------------|---------------|------------------|----------------------|",
    ]
    for i, (a, v) in enumerate(zip(lut["amps"], lut["vals"])):
        lines.append(
            f"| {i + 1} | {a:.4f} | {v:.4f} | {a / D_TO_PHASE_COS:.4f} | {v / D_TO_PHASE_COS:.4f} |"
        )
    lines.append("")
    return "\n".join(lines)


def main() -> int:
    ap = argparse.ArgumentParser(description="Parse VOFA LUT burst from Id cal CSV")
    ap.add_argument("csv", type=Path, help="VOFA+ exported CSV (I0..I5)")
    ap.add_argument("--md", action="store_true", help="phase 表 markdown")
    ap.add_argument("--md-full", action="store_true", help="完整突发说明（可粘贴进分析报告）")
    ap.add_argument("-o", "--output", type=Path, help="--md-full 写入文件")
    args = ap.parse_args()

    rows = load_csv(args.csv)
    lut = parse_lut(rows)

    print(f"file: {args.csv.name}")
    print(f"header @ row {lut['header_row']}, tail @ row {lut['tail_row']}")
    print(
        f"len={lut['len']}  Rs={lut['rs']:.4f}  outlier={lut['outlier']}  "
        f"proto={lut['proto_ver']:.1f}  ch5={lut['ch5']:.4g}"
    )
    print(f"  → {lut['meta']['proto_desc']}")
    print(f"  → runtime: {lut['meta']['runtime']}")
    print(f"sum(vals) tail={lut['tail_sum']:.6f}  calc={lut['calc_sum']:.6f}")

    if args.md_full:
        text = format_lut_burst_md(lut, args.csv.name)
        if args.output:
            args.output.write_text(text, encoding="utf-8")
            print(f"wrote {args.output}")
        else:
            print()
            print(text)
    elif args.md:
        print("\n| amp (A) | val (V) | Id~ (A) | Ud_res~ (V) |")
        print("|---------|---------|---------|-------------|")
        for a, v in zip(lut["amps"], lut["vals"]):
            print(
                f"| {a:.3f} | {v:.4f} | {a / D_TO_PHASE_COS:.3f} | {v / D_TO_PHASE_COS:.4f} |"
            )
    else:
        print("\namp(A)  val(V)  Id~(A)  Ud_res~(V)")
        for a, v in zip(lut["amps"], lut["vals"]):
            print(
                f"{a:6.3f}  {v:7.4f}  {a / D_TO_PHASE_COS:6.3f}  {v / D_TO_PHASE_COS:7.4f}"
            )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
