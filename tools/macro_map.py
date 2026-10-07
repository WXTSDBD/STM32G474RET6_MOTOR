#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""包 8.1：只读扫描 motor_params_m1.h → 宏归属表（md + json）。

规格见 docs/方案_宏归属表_2026-10-07.md §6。
不改任何固件；可重复跑，json 供搬迁前后 diff。

用法：
  python tools/macro_map.py
  python tools/macro_map.py --out-dir docs
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import re
from collections import Counter, defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
HEADER = ROOT / "config" / "motor_params_m1.h"
PROFILES_DIR = ROOT / "config" / "profiles"

RE_DEFINE = re.compile(r"^\s*#\s*define\s+(\S+)(?:\s+(.*))?$")
RE_UNDEF = re.compile(r"^\s*#\s*undef\s+(\S+)\b")
RE_UNDEF_M = re.compile(r"^\s*#\s*undef\s+(\S+)\b", re.M)
RE_IFNDEF = re.compile(r"^\s*#\s*ifndef\s+(\S+)\b")
RE_IFDEF = re.compile(r"^\s*#\s*ifdef\s+(\S+)\b")
RE_IF = re.compile(r"^\s*#\s*if\b")
RE_ELIF = re.compile(r"^\s*#\s*elif\b")
RE_ELSE = re.compile(r"^\s*#\s*else\b")
RE_ENDIF = re.compile(r"^\s*#\s*endif\b")
RE_ERROR = re.compile(r"^\s*#\s*error\b")
RE_STATIC = re.compile(r"\b_Static_assert\b")
RE_M1 = re.compile(r"\b(M1_[A-Za-z0-9_]+)\b")
RE_COND_DENS = re.compile(r"^\s*#\s*(if|ifdef|ifndef|elif)\b")

HOT_PATH_FILES = {
    "motor/motor_current.c",
    "motor/motor_foc_loop.c",
    "motor/motor_outer_loop.c",
    "motor/foc_svpwm.c",
}

SCAN_DIRS = (
    "motor",
    "config",
    "cal",
    "bringup",
    "board",
    "platform",
    "debug",
    "Core",
    "app",
)

# class 初判（方案 §2：铭牌/参数/能力/策略/实验/工具）
CLASS_RULES: list[tuple[str, re.Pattern[str]]] = [
    (
        "铭牌",
        re.compile(
            r"^M1_(RS_OHM|LD_H|LQ_H|POLE_PAIRS|VBUS_V|DEADTIME_NS|KT_|PSI|FLUX|MOTOR_)"
        ),
    ),
    (
        "策略",
        re.compile(
            r"THETA_FB_SRC|OMEGA_FB_SRC|OBS_SELECT|CTRL_MODE|BINDING_|OUTER_EXPT|"
            r"POS_MODE|MIT_MODE|HANDOFF|CRUISE"
        ),
    ),
    (
        "实验",
        re.compile(
            r"GATE|SIGNOFF|POS_STEP|VOFA_|TELEM_|BRINGUP|SMOKE|PROBE|SWEEP|"
            r"LADDER|FRIC_FF|USE_.*_PROFILE"
        ),
    ),
    (
        "能力",
        re.compile(
            r"_ENABLE$|_ENABLE_|_BOOT$|^M1_IF_ENABLE|^M1_HFI_ENABLE|^M1_PLL_ENABLE|"
            r"^M1_SPEED_LOOP_ENABLE|^M1_POS_LOOP_ENABLE|^M1_IDENT_ENABLE|"
            r"^M1_EMF_|^M1_OBS_"
        ),
    ),
    (
        "参数",
        re.compile(
            r"_KP|_KI|_KD|_FC_|_FN_|_ZETA|_TS_|_LIMIT|_MAX|_MIN|_SCALE|"
            r"_GAIN|_BAND|_HZ|_RPM|_A$|_RAD|_OHM|_H$|PWM_|ADC_|CTRL_TS|SHUNT|AMP_"
        ),
    ),
    ("工具", re.compile(r"^MOTOR_PARAMS_|^M1_BRINGUP_MODE")),
]


def classify(name: str) -> str:
    for label, pat in CLASS_RULES:
        if pat.search(name):
            return label
    return "参数" if name.startswith("M1_") else "工具"


def strip_line_comment(s: str) -> str:
    if "//" in s:
        return s.split("//", 1)[0].rstrip()
    return s.rstrip()


def parse_header(text: str) -> dict:
    lines = text.splitlines()
    defines: dict[str, dict] = {}
    undef_lines_param: list[tuple[int, str]] = []
    define_lines = 0
    ifndef_n = 0
    error_n = 0
    static_n = 0
    cond_dens = 0
    pending_ifndef: str | None = None
    # stack of raw #if/#ifdef/#ifndef lines for context
    if_stack: list[str] = []

    for i, line in enumerate(lines, 1):
        raw = line.rstrip("\n")
        if RE_ERROR.search(raw):
            error_n += 1
        if RE_STATIC.search(raw):
            static_n += 1
        if RE_COND_DENS.match(raw):
            cond_dens += 1

        m_ifn = RE_IFNDEF.match(raw)
        m_ifd = RE_IFDEF.match(raw)
        if m_ifn:
            ifndef_n += 1
            pending_ifndef = m_ifn.group(1)
            if_stack.append(raw.strip()[:80])
            continue
        if m_ifd or RE_IF.match(raw):
            pending_ifndef = None
            if_stack.append(raw.strip()[:80])
            continue
        if RE_ELIF.match(raw):
            pending_ifndef = None
            if if_stack:
                if_stack[-1] = raw.strip()[:80]
            continue
        if RE_ELSE.match(raw):
            pending_ifndef = None
            continue
        if RE_ENDIF.match(raw):
            pending_ifndef = None
            if if_stack:
                if_stack.pop()
            continue

        m_un = RE_UNDEF.match(raw)
        if m_un:
            undef_lines_param.append((i, m_un.group(1)))
            pending_ifndef = None
            continue

        m_def = RE_DEFINE.match(raw)
        if not m_def:
            continue

        define_lines += 1
        name = m_def.group(1)
        value = strip_line_comment(m_def.group(2) or "")
        deps = sorted(set(RE_M1.findall(value)) - {name})

        style = "bare"
        if pending_ifndef == name:
            style = "ifndef_guard"
            pending_ifndef = None
        # undef_force decided later if name appears in undef_lines_param before this line

        ctx = " | ".join(if_stack[-2:]) if if_stack else ""

        info = defines.get(name)
        site = {
            "line": i,
            "style": style,
            "value": value[:120],
            "deps": deps,
            "if_ctx": ctx,
        }
        if info is None:
            defines[name] = {
                "name": name,
                "first_line": i,
                "last_line": i,
                "redef_count": 1,
                "styles": [style],
                "sites": [site],
                "deps_union": set(deps),
            }
        else:
            info["last_line"] = i
            info["redef_count"] += 1
            info["styles"].append(style)
            info["sites"].append(site)
            info["deps_union"].update(deps)

        # after a define that wasn't ifndef-paired, clear pending if mismatched
        if pending_ifndef and pending_ifndef != name:
            pass

    # mark undef_force on sites: if any #undef of name exists in this file before a site
    undef_by_name: dict[str, list[int]] = defaultdict(list)
    for ln, n in undef_lines_param:
        undef_by_name[n].append(ln)

    for name, info in defines.items():
        ulines = undef_by_name.get(name, [])
        for site in info["sites"]:
            if any(u < site["line"] for u in ulines):
                site["style"] = "undef_force"
        # primary style: worst
        styles = {s["style"] for s in info["sites"]}
        if "undef_force" in styles or name in undef_by_name:
            info["def_kind"] = "undef_force"
        elif "ifndef_guard" in styles and "bare" in styles:
            info["def_kind"] = "mixed"
        elif "ifndef_guard" in styles:
            info["def_kind"] = "ifndef_guard"
        else:
            info["def_kind"] = "bare"
        info["undef_in_param"] = name in undef_by_name
        info["undef_param_lines"] = ulines

    return {
        "lines": len(lines),
        "define_lines": define_lines,
        "defines": defines,
        "undef_lines_param": undef_lines_param,
        "ifndef_n": ifndef_n,
        "error_n": error_n,
        "static_n": static_n,
        "cond_dens": cond_dens,
    }


def scan_profile_undefs() -> tuple[dict[str, list[str]], dict[str, int], int]:
    """返回 (name→profiles, per_file 行数, 合计行数)。"""
    out: dict[str, list[str]] = defaultdict(list)
    per_file: dict[str, int] = {}
    total_lines = 0
    if not PROFILES_DIR.is_dir():
        return out, per_file, 0
    for path in sorted(PROFILES_DIR.glob("*.profile.h")):
        text = path.read_text(encoding="utf-8", errors="replace")
        names = RE_UNDEF_M.findall(text)
        n = len(names)
        per_file[path.name] = n
        total_lines += n
        for name in set(names):
            out[name].append(path.name)
    return out, per_file, total_lines


def scan_readers(names: set[str]) -> dict[str, list[str]]:
    """name -> list of 'rel:line' read sites (excluding motor_params_m1.h defines)."""
    hits: dict[str, list[str]] = defaultdict(list)
    for d in SCAN_DIRS:
        base = ROOT / d
        if not base.is_dir():
            continue
        for path in base.rglob("*"):
            if not path.is_file() or path.suffix.lower() not in {".c", ".h"}:
                continue
            rel = str(path.relative_to(ROOT)).replace("\\", "/")
            try:
                text = path.read_text(encoding="utf-8", errors="replace")
            except OSError:
                continue
            for li, line in enumerate(text.splitlines(), 1):
                # skip define/undef/ifndef of the name itself in params header
                if path.name == "motor_params_m1.h":
                    if RE_DEFINE.match(line) or RE_UNDEF.match(line) or RE_IFNDEF.match(line):
                        continue
                for m in RE_M1.finditer(line):
                    name = m.group(1)
                    if name in names:
                        hits[name].append(f"{rel}:{li}")
    return hits


def build_rows(
    parsed: dict, profile_undefs: dict[str, list[str]], readers: dict
) -> list[dict]:
    # 反向依赖：谁的宏值引用了我（搬迁时编译期计算会断）
    rev: dict[str, set[str]] = defaultdict(set)
    for name, info in parsed["defines"].items():
        for d in info["deps_union"]:
            rev[d].add(name)

    rows = []
    for name, info in sorted(parsed["defines"].items(), key=lambda kv: kv[1]["first_line"]):
        reads = readers.get(name, [])
        files = sorted({r.split(":")[0] for r in reads})
        hot = any(f in HOT_PATH_FILES for f in files)
        prof = list(profile_undefs.get(name, []))
        undef_parts: list[str] = []
        if info["undef_in_param"]:
            undef_parts.append("param")
        if prof:
            undef_parts.append("profile")
        undef_in = "+".join(undef_parts) if undef_parts else "无"

        deps = sorted(info["deps_union"])
        dependents = sorted(rev.get(name, ()))
        rows.append(
            {
                "name": name,
                "line": info["first_line"],
                "last_line": info["last_line"],
                "redef_count": info["redef_count"],
                "class": classify(name),
                "def_kind": info["def_kind"],
                "undef_in": undef_in,
                "macro_deps": ",".join(deps),
                "macro_deps_n": len(deps),
                "depended_by": ",".join(dependents),
                "depended_by_n": len(dependents),
                "reads": ";".join(reads[:30]) + (";..." if len(reads) > 30 else ""),
                "reads_n": len(reads),
                "read_file_n": len(files),
                "hot_path": int(hot),
                "profile_override": int(bool(prof)),
                "profile_files": ",".join(prof),
                "if_ctx_first": info["sites"][0].get("if_ctx", ""),
            }
        )
    return rows


def low_risk_subset(rows: list[dict]) -> list[dict]:
    """§7.1：读取少 + 非热路径 + 无 undef + 无宏依赖。"""
    out = []
    for r in rows:
        if r["class"] != "铭牌":
            continue
        if r["hot_path"]:
            continue
        if r["undef_in"] != "无":
            continue
        if r["macro_deps_n"] > 0:
            continue
        if r["read_file_n"] == 0:
            continue
        out.append(r)
    return sorted(out, key=lambda x: (x["read_file_n"], x["reads_n"], x["name"]))


def write_outputs(
    out_dir: Path,
    parsed: dict,
    rows: list[dict],
    per_file: dict[str, int],
    prof_total: int,
    stamp_date: str,
) -> tuple[Path, Path]:
    unique = len(rows)
    define_lines = parsed["define_lines"]
    redefs = define_lines - unique
    dens = 100.0 * parsed["cond_dens"] / parsed["lines"] if parsed["lines"] else 0.0

    class_c = Counter(r["class"] for r in rows)
    kind_c = Counter(r["def_kind"] for r in rows)
    undef_param_names = sum(1 for r in rows if "param" in r["undef_in"])
    low = low_risk_subset(rows)

    payload = {
        "generated": stamp_date,
        "source": "config/motor_params_m1.h",
        "summary": {
            "lines": parsed["lines"],
            "define_lines": define_lines,
            "unique_names": unique,
            "redef_sites": redefs,
            "ifndef_n": parsed["ifndef_n"],
            "undef_lines_param": len(parsed["undef_lines_param"]),
            "undef_names_param": undef_param_names,
            "undef_lines_profile": prof_total,
            "macro_deps_macros": sum(1 for r in rows if r["macro_deps_n"] > 0),
            "error_n": parsed["error_n"],
            "static_assert_n": parsed["static_n"],
            "cond_dens": parsed["cond_dens"],
            "cond_dens_pct": round(dens, 1),
            "m1_only_define_lines": sum(
                r["redef_count"] for r in rows if r["name"].startswith("M1_")
            ),
            "m1_only_unique": sum(1 for r in rows if r["name"].startswith("M1_")),
        },
        "class_counts": dict(class_c),
        "def_kind_counts": dict(kind_c),
        "profile_undef_per_file": per_file,
        "macros": rows,
    }

    json_path = out_dir / f"宏归属表_{stamp_date}.json"
    md_path = out_dir / f"宏归属表_{stamp_date}.md"

    json_path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )

    s = payload["summary"]
    md: list[str] = []
    md.append(f"# 宏归属表（{stamp_date}）")
    md.append("")
    md.append("**性质**：`tools/macro_map.py` 只读扫描产物，**不改固件**。")
    md.append(f"**规格**：[`方案_宏归属表_2026-10-07.md`](./方案_宏归属表_2026-10-07.md) §6。")
    md.append("")
    md.append("## 1. 汇总（验收数字）")
    md.append("")
    md.append("| 指标 | 值 |")
    md.append("|---|---:|")
    md.append(f"| 物理行数（`splitlines`） | {s['lines']} |")
    md.append(f"| `#define` 行数（含 include guard） | {s['define_lines']} |")
    md.append(f"| 唯一宏名 | {s['unique_names']} |")
    md.append(f"| 重定义处（行−唯一） | {s['redef_sites']} |")
    md.append(f"| 其中 `M1_*` 行 / 唯一 | {s['m1_only_define_lines']} / {s['m1_only_unique']} |")
    md.append(f"| `#ifndef` | {s['ifndef_n']} |")
    md.append(f"| `#undef` 本文件行 | {s['undef_lines_param']} |")
    md.append(f"| `#undef` 本文件波及宏名 | {s['undef_names_param']} |")
    md.append(f"| `#undef` profile 合计行 | {s['undef_lines_profile']} |")
    md.append(f"| 宏值含其他 `M1_` 的宏名数 | {s['macro_deps_macros']} |")
    md.append(f"| `#error` | {s['error_n']} |")
    md.append(f"| `_Static_assert`（本文件） | {s['static_assert_n']} |")
    md.append(f"| 条件编译密度口径 | {s['cond_dens']}（{s['cond_dens_pct']}%） |")
    md.append("")
    md.append("> 验收钉死：全量含 guard = **1004 / 592**；业务实体只看 `M1_*` = **1003 / 591**。")
    md.append("> profile `#undef` **全部保留**；本文件内 `#undef` 才是 8.2 分类对象。")
    md.append("")
    md.append("## 2. 定义风格 / 归属初判")
    md.append("")
    md.append("| def_kind | 宏名数 |")
    md.append("|---|---:|")
    for k, n in kind_c.most_common():
        md.append(f"| `{k}` | {n} |")
    md.append("")
    md.append("| class | 宏名数 |")
    md.append("|---|---:|")
    for k, n in class_c.most_common():
        md.append(f"| {k} | {n} |")
    md.append("")
    md.append("## 3. profile `#undef` 分布（合法覆盖，勿砍）")
    md.append("")
    md.append("| profile | `#undef` 行 |")
    md.append("|---|---:|")
    for fn, n in sorted(per_file.items(), key=lambda kv: -kv[1]):
        md.append(f"| `{fn}` | {n} |")
    md.append(f"| **合计** | **{prof_total}** |")
    md.append("")
    md.append("## 4. 8.3 低风险铭牌候选（启发式，须人工复核）")
    md.append("")
    md.append("筛选：`class==铭牌` 且 `hot_path==0` 且 `undef_in==无` 且无 `macro_deps` 且有外部读取。")
    md.append("")
    md.append("| 宏 | 行 | 读文件数 | 读取次数 |")
    md.append("|---|---:|---:|---:|")
    if low:
        for r in low:
            md.append(
                f"| `{r['name']}` | {r['line']} | {r['read_file_n']} | {r['reads_n']} |"
            )
    else:
        md.append("| （空：铭牌多在热路径或有依赖——见全表） | | | |")
    md.append("")
    md.append("## 5. 高触达：`undef_force` 且有读取（Top 40）")
    md.append("")
    risky = [
        r
        for r in rows
        if r["def_kind"] == "undef_force" and r["read_file_n"] > 0
    ]
    risky.sort(key=lambda r: (-r["read_file_n"], -r["reads_n"], r["name"]))
    md.append("| 宏 | 读文件 | 次数 | class | hot |")
    md.append("|---|---:|---:|---|---:|")
    for r in risky[:40]:
        md.append(
            f"| `{r['name']}` | {r['read_file_n']} | {r['reads_n']} | "
            f"{r['class']} | {r['hot_path']} |"
        )
    md.append(f"\n共 {len(risky)} 项（全量见 json）。")
    md.append("")
    md.append("## 6. 全表")
    md.append("")
    md.append(
        "| name | line | redef | class | def_kind | undef_in | deps_n | "
        "read_files | reads | hot | profile_ov |"
    )
    md.append("|---|---:|---:|---|---|---|---:|---:|---:|---:|---:|")
    for r in rows:
        md.append(
            f"| `{r['name']}` | {r['line']} | {r['redef_count']} | {r['class']} | "
            f"{r['def_kind']} | {r['undef_in']} | {r['macro_deps_n']} | "
            f"{r['read_file_n']} | {r['reads_n']} | {r['hot_path']} | "
            f"{r['profile_override']} |"
        )
    md.append("")
    md.append("## 7. 产物")
    md.append("")
    md.append(f"- Markdown：`{md_path.name}`")
    md.append(f"- JSON：`{json_path.name}`")
    md.append("- 再生：`python tools/macro_map.py`")
    md.append("")

    md_path.write_text("\n".join(md) + "\n", encoding="utf-8")
    return md_path, json_path


def main() -> int:
    ap = argparse.ArgumentParser(description="宏归属表扫描（只读）")
    ap.add_argument("--out-dir", type=Path, default=ROOT / "docs")
    ap.add_argument("--no-readers", action="store_true")
    args = ap.parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)

    text = HEADER.read_text(encoding="utf-8", errors="replace")
    print("parse header...", flush=True)
    parsed = parse_header(text)
    print("scan profiles...", flush=True)
    profile_undefs, per_file, prof_total = scan_profile_undefs()
    names = set(parsed["defines"])
    readers: dict[str, list[str]] = {}
    if not args.no_readers:
        print("scan readers...", flush=True)
        readers = scan_readers(names)

    rows = build_rows(parsed, profile_undefs, readers)
    stamp = dt.date.today().isoformat()
    md_path, json_path = write_outputs(
        args.out_dir, parsed, rows, per_file, prof_total, stamp
    )

    s_lines = parsed["lines"]
    s_def = parsed["define_lines"]
    s_u = len(rows)
    print(f"wrote {md_path}")
    print(f"wrote {json_path}")
    print(
        f"summary: lines={s_lines} define_lines={s_def} unique={s_u} "
        f"redefs={s_def - s_u} undef_param={len(parsed['undef_lines_param'])} "
        f"undef_profile={prof_total} "
        f"errors={parsed['error_n']} ifndef={parsed['ifndef_n']}"
    )
    ok = s_def == 1004 and s_u == 592
    print(f"accept_1004_592: {'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
