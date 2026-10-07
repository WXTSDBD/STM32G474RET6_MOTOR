#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""包 8.2 步 1：只读生成 motor_params_m1.h 内 #undef 分类表。

输入：config/motor_params_m1.h + docs/宏归属表_2026-10-07.json
输出：docs/宏归属表_undef_classify_2026-10-07.{md,csv,json}

用法：
  python tools/classify_motor_params_undef.py
"""

from __future__ import annotations

import csv
import json
import re
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
HEADER = ROOT / "config" / "motor_params_m1.h"
MAP_JSON = ROOT / "docs" / "宏归属表_2026-10-07.json"
OUT_STEM = ROOT / "docs" / "宏归属表_undef_classify_2026-10-07"

RE_UNDEF = re.compile(r"^\s*#\s*undef\s+(M1_\S+)\b")
RE_DEFINE = re.compile(r"^\s*#\s*define\s+(M1_\S+)(?:\s+(.*))?$")
RE_IFNDEF = re.compile(r"^\s*#\s*ifndef\s+(M1_\S+)\b")


def strip_cmt(s: str) -> str:
    if "//" in s:
        s = s.split("//", 1)[0]
    return s.strip()


def norm_val(v: str) -> str:
    v = strip_cmt(v)
    v = re.sub(r"\s+", "", v)
    if v.endswith("f") and re.match(r"^-?[0-9.]+f$", v):
        try:
            return f"{float(v[:-1]):.9g}"
        except ValueError:
            return v
    if v.endswith("u") and v[:-1].isdigit():
        return str(int(v[:-1]))
    return v


def parse_header(text: str) -> tuple[list[dict], dict[str, list[dict]]]:
    """返回 (undef_rows, name->define_sites)。"""
    lines = text.splitlines()
    defines_by_name: dict[str, list[dict]] = defaultdict(list)
    pending_ifndef: str | None = None
    undef_rows: list[dict] = []

    i = 0
    while i < len(lines):
        ln = i + 1
        line = lines[i]
        m_ifn = RE_IFNDEF.match(line)
        if m_ifn:
            pending_ifndef = m_ifn.group(1)
            i += 1
            continue

        m_un = RE_UNDEF.match(line)
        if m_un:
            name = m_un.group(1)
            force_val = ""
            force_line = 0
            # 下一非空行若是同名 #define，记强制值
            j = i + 1
            while j < len(lines) and not lines[j].strip():
                j += 1
            if j < len(lines):
                m_def = RE_DEFINE.match(lines[j])
                if m_def and m_def.group(1) == name:
                    force_val = strip_cmt(m_def.group(2) or "")
                    force_line = j + 1
            undef_rows.append(
                {
                    "undef_line": ln,
                    "name": name,
                    "force_line": force_line,
                    "force_val": force_val,
                    "force_val_norm": norm_val(force_val),
                }
            )
            pending_ifndef = None
            i += 1
            continue

        m_def = RE_DEFINE.match(line)
        if m_def:
            name = m_def.group(1)
            val = strip_cmt(m_def.group(2) or "")
            style = "ifndef_guard" if pending_ifndef == name else "bare"
            if pending_ifndef == name:
                pending_ifndef = None
            defines_by_name[name].append(
                {
                    "line": ln,
                    "val": val,
                    "val_norm": norm_val(val),
                    "style": style,
                }
            )
            i += 1
            continue

        if line.strip().startswith("#") and not line.strip().startswith("#define"):
            pending_ifndef = None
        i += 1

    return undef_rows, defines_by_name


def classify_row(
    row: dict,
    sites: list[dict],
    reader_files: int,
    profile_override: int,
) -> tuple[str, str]:
    """返回 (初判, 理由)。"""
    name = row["name"]
    fv = row["force_val_norm"]
    ul = row["undef_line"]

    priors = [s for s in sites if s["line"] < ul]
    same_prior = [s for s in priors if s["val_norm"] == fv and fv != ""]
    diff_prior = [s for s in priors if s["val_norm"] != fv and s["val_norm"] != ""]

    # 后面还有 ifndef 兜底默认
    later_ifndef = [
        s for s in sites if s["line"] > ul and s["style"] == "ifndef_guard"
    ]

    if reader_files == 0 and not later_ifndef and len(sites) <= 2:
        return "可删", "无外部读取且几乎无后续守卫（须人工复核后再删）"

    if same_prior and not diff_prior:
        return "重复强写", f"与先前定义同值（先前行 {same_prior[-1]['line']}）→ 可改 #ifndef"

    if same_prior and diff_prior:
        return "必留", "分支内曾出现不同值，本处用 #undef 钉死当前分支"

    if diff_prior and fv:
        return "必留", f"覆盖先前不同值（先前 {diff_prior[-1]['line']}={diff_prior[-1]['val']}）"

    if profile_override:
        return "必留", "亦被 profile #undef（本文件内强制与 profile 链交织，先留）"

    if later_ifndef and fv and later_ifndef[0]["val_norm"] != fv:
        return "必留", "后面 #ifndef 默认值不同，本处为分支强制"

    if later_ifndef and fv and later_ifndef[0]["val_norm"] == fv:
        return "重复强写", "与后续 #ifndef 默认同值 → 可删本处 #undef+#define"

    if reader_files >= 1:
        return "必留", "有外部读取，默认先留（未证伪前不改）"

    return "必留", "保守默认"


def main() -> int:
    text = HEADER.read_text(encoding="utf-8", errors="replace")
    undef_rows, defines_by_name = parse_header(text)

    readers: dict[str, dict] = {}
    if MAP_JSON.is_file():
        mp = json.loads(MAP_JSON.read_text(encoding="utf-8"))
        for m in mp.get("macros", []):
            readers[m["name"]] = m

    out_rows = []
    guess_c = defaultdict(int)
    for row in undef_rows:
        name = row["name"]
        info = readers.get(name, {})
        rf = int(info.get("read_file_n", 0))
        pov = int(info.get("profile_override", 0))
        sites = defines_by_name.get(name, [])
        guess, why = classify_row(row, sites, rf, pov)
        guess_c[guess] += 1
        had_ifndef = any(s["style"] == "ifndef_guard" for s in sites)
        out_rows.append(
            {
                "undef_line": row["undef_line"],
                "name": name,
                "force_line": row["force_line"],
                "force_val": row["force_val"],
                "read_file_n": rf,
                "reads_n": int(info.get("reads_n", 0)),
                "hot_path": int(info.get("hot_path", 0)),
                "profile_override": pov,
                "had_ifndef_somewhere": int(had_ifndef),
                "n_define_sites": len(sites),
                "guess": guess,
                "why": why,
                "priority": 1 if rf > 0 else 2,
            }
        )

    # 有读取点优先
    out_rows.sort(key=lambda r: (r["priority"], -r["read_file_n"], r["undef_line"]))

    payload = {
        "source": "config/motor_params_m1.h",
        "undef_lines": len(out_rows),
        "unique_names": len({r["name"] for r in out_rows}),
        "guess_counts": dict(guess_c),
        "with_readers": sum(1 for r in out_rows if r["read_file_n"] > 0),
        "rows": out_rows,
    }

    OUT_STEM.with_suffix(".json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )

    fields = list(out_rows[0].keys()) if out_rows else []
    with OUT_STEM.with_suffix(".csv").open("w", encoding="utf-8-sig", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(out_rows)

    md = []
    md.append("# 宏 `#undef` 分类表（motor_params_m1.h 内，2026-10-07）")
    md.append("")
    md.append("**性质**：只读扫描；**不改固件**。profile 576 处不在本表。")
    md.append("再生：`python tools/classify_motor_params_undef.py`")
    md.append("")
    md.append("## 1. 汇总")
    md.append("")
    md.append("| 项 | 值 |")
    md.append("|---|---:|")
    md.append(f"| `#undef` 行 | {payload['undef_lines']} |")
    md.append(f"| 波及宏名 | {payload['unique_names']} |")
    md.append(f"| 有外部读取的行 | {payload['with_readers']} |")
    for k, n in sorted(guess_c.items(), key=lambda kv: -kv[1]):
        md.append(f"| 初判 `{k}` | {n} |")
    md.append("")
    md.append("## 2. 建议刀序")
    md.append("")
    md.append("1. 只动初判 **`重复强写`** 且人工复核通过的（≤10/批）。")
    md.append("2. **`可删`** 须再确认无 `#if` 依赖后再动。")
    md.append("3. **`必留`** 默认不动。")
    md.append("4. 编完比四数+hex：相同→不烧；不同→烧录窗口停。")
    md.append("")
    md.append("## 3. 重复强写候选（有读取点优先，Top）")
    md.append("")
    md.append("| 行 | 宏 | 强制值 | 读文件 | hot | 理由 |")
    md.append("|---:|---|---|---:|---:|---|")
    cand = [r for r in out_rows if r["guess"] == "重复强写"]
    for r in cand[:40]:
        md.append(
            f"| {r['undef_line']} | `{r['name']}` | `{r['force_val'][:40]}` | "
            f"{r['read_file_n']} | {r['hot_path']} | {r['why']} |"
        )
    md.append(f"\n共 {len(cand)} 行初判为重复强写（全量见 csv/json）。")
    md.append("")
    md.append("## 4. 可删候选（须复核）")
    md.append("")
    md.append("| 行 | 宏 | 强制值 | 理由 |")
    md.append("|---:|---|---|---|")
    for r in [x for x in out_rows if x["guess"] == "可删"][:30]:
        md.append(
            f"| {r['undef_line']} | `{r['name']}` | `{r['force_val'][:40]}` | {r['why']} |"
        )
    md.append("")

    OUT_STEM.with_suffix(".md").write_text("\n".join(md) + "\n", encoding="utf-8")

    print(f"wrote {OUT_STEM}.md/.csv/.json")
    print(
        f"undef_lines={payload['undef_lines']} names={payload['unique_names']} "
        f"with_readers={payload['with_readers']} guesses={dict(guess_c)}"
    )
    print(f"dup_force_candidates={len(cand)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
