#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""count_if —— 普查 #if / M1_*（7 步收敛台账用）。

用法：
  python tools/count_if.py
  python tools/count_if.py --json docs/_scratch_count_if.json
"""
from __future__ import annotations

import argparse
import json
import re
from collections import Counter, defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCAN = (
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
SKIP_PARTS = {".git", "MDK-ARM", "Drivers", "Middlewares", "别人的代码", "_pkg", ".zig-cache"}
RE_DIR = re.compile(r"^\s*#\s*(if|ifdef|ifndef|elif)\b")
RE_M1_DEF = re.compile(r"^\s*#\s*define\s+(M1_[A-Za-z0-9_]+)\b")
RE_M1_NAME = re.compile(r"\b(M1_[A-Za-z0-9_]+)\b")
RE_IF_M1 = re.compile(r"\bif\s*\(\s*M1_")


def iter_sources() -> list[Path]:
    out: list[Path] = []
    for top in SCAN:
        base = ROOT / top
        if not base.is_dir():
            continue
        for p in base.rglob("*"):
            if not p.is_file():
                continue
            if p.suffix.lower() not in {".c", ".h"}:
                continue
            if any(part in SKIP_PARTS for part in p.parts):
                continue
            out.append(p)
    return sorted(out)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--json", type=Path, default=None)
    args = ap.parse_args()

    by_dir: dict[str, int] = defaultdict(int)
    by_file: list[tuple[int, str]] = []
    m1_defs: set[str] = set()
    m1_names: set[str] = set()
    total = 0
    if_m1_total = 0
    if_m1_by_file: list[tuple[int, str]] = []
    for p in iter_sources():
        text = p.read_text(encoding="utf-8", errors="replace")
        n = 0
        n_if_m1 = 0
        for line in text.splitlines():
            if RE_DIR.match(line):
                n += 1
            n_if_m1 += len(RE_IF_M1.findall(line))
            m = RE_M1_DEF.match(line)
            if m:
                m1_defs.add(m.group(1))
            for name in RE_M1_NAME.findall(line):
                m1_names.add(name)
        if n:
            rel = p.relative_to(ROOT).as_posix()
            top = rel.split("/", 1)[0]
            by_dir[top] += n
            by_file.append((n, rel))
            total += n
        if n_if_m1:
            if_m1_total += n_if_m1
            if_m1_by_file.append((n_if_m1, p.relative_to(ROOT).as_posix()))

    by_file.sort(reverse=True)
    if_m1_by_file.sort(reverse=True)
    summary = {
        "total_if_elif_ifdef_ifndef": total,
        "if_M1_runtime_total": if_m1_total,
        "by_dir": dict(sorted(by_dir.items(), key=lambda kv: -kv[1])),
        "m1_define_count": len(m1_defs),
        "m1_name_count": len(m1_names),
        "top_files": [{"n": n, "path": p} for n, p in by_file[:30]],
        "top_if_M1_files": [{"n": n, "path": p} for n, p in if_m1_by_file[:20]],
    }
    print(f"#if/#ifdef/#ifndef/#elif total = {total}")
    print(f"if (M1_*) runtime total = {if_m1_total}")
    print("by_dir:")
    for k, v in summary["by_dir"].items():
        print(f"  {k:12s} {v}")
    print(f"M1_* #define names = {len(m1_defs)}")
    print(f"M1_* referenced names = {len(m1_names)}")
    print("top #if files:")
    for n, p in by_file[:15]:
        print(f"  {n:4d}  {p}")
    print("top if (M1_*) files:")
    for n, p in if_m1_by_file[:10]:
        print(f"  {n:4d}  {p}")
    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(json.dumps(summary, indent=2), encoding="utf-8")
        print(f"wrote {args.json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
