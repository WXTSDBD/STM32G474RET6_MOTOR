#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""自动找并删除「强制值 == 最早 #ifndef 默认」的 #undef+#define 对。

用法：
  python tools/_tmp_undef_safe_batch.py --dry-run
  python tools/_tmp_undef_safe_batch.py --apply --max-macros 10
"""
from __future__ import annotations

import argparse
import re
from collections import OrderedDict
from pathlib import Path

HEADER = Path(__file__).resolve().parents[1] / "config" / "motor_params_m1.h"
RE_IFNDEF = re.compile(r"^\s*#\s*ifndef\s+(M1_\S+)\b")
RE_DEFINE = re.compile(r"^\s*#\s*define\s+(M1_\S+)(?:\s+(.*))?$")
RE_UNDEF = re.compile(r"^\s*#\s*undef\s+(M1_\S+)\s*$")


def norm(v: str) -> str:
    v = (v or "").split("//", 1)[0].strip()
    v = re.sub(r"\s+", "", v)
    if re.match(r"^-?[0-9.]+f$", v):
        try:
            return f"{float(v[:-1]):.9g}f"
        except ValueError:
            return v
    if v.endswith("u") and v[:-1].isdigit():
        return str(int(v[:-1]))
    return v


def is_simple_literal(v: str) -> bool:
    """只接受数字字面量默认，拒绝宏别名/表达式。"""
    n = norm(v)
    if not n or "M1_" in n or "(" in n or ")" in n:
        return False
    return bool(re.match(r"^-?[0-9.]+f?$", n) or n.isdigit())


def earliest_ifndef_defaults(lines: list[str]) -> dict[str, str]:
    pending = None
    out: dict[str, str] = {}
    for line in lines:
        m = RE_IFNDEF.match(line)
        if m:
            pending = m.group(1)
            continue
        m = RE_DEFINE.match(line)
        if m and pending == m.group(1):
            if m.group(1) not in out:
                out[m.group(1)] = norm(m.group(2) or "")
            pending = None
        elif line.strip().startswith("#"):
            pending = None
    return out


def find_safe_pairs(lines: list[str], defaults: dict[str, str]) -> list[tuple[int, int, str, str]]:
    """返回 (undef_idx0, define_idx0, name, raw_val)。"""
    pairs = []
    i = 0
    while i < len(lines):
        m = RE_UNDEF.match(lines[i])
        if m and m.group(1) in defaults:
            name = m.group(1)
            j = i + 1
            while j < len(lines) and not lines[j].strip():
                j += 1
            if j < len(lines):
                md = RE_DEFINE.match(lines[j])
                if md and md.group(1) == name:
                    raw = md.group(2) or ""
                    if (
                        is_simple_literal(defaults[name])
                        and is_simple_literal(raw)
                        and norm(raw) == defaults[name]
                    ):
                        pairs.append((i, j, name, raw))
                        i = j + 1
                        continue
        i += 1
    return pairs


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--apply", action="store_true")
    ap.add_argument("--max-macros", type=int, default=10)
    args = ap.parse_args()

    text = HEADER.read_text(encoding="utf-8")
    lines = text.splitlines(keepends=True)
    bare = [ln.rstrip("\n") for ln in lines]
    defaults = earliest_ifndef_defaults(bare)
    pairs = find_safe_pairs(bare, defaults)

    by = OrderedDict()
    for p in pairs:
        by.setdefault(p[2], []).append(p)

    selected_names = list(by.keys())[: args.max_macros]
    selected = [p for p in pairs if p[2] in selected_names]

    print(f"safe_macros_total={len(by)} safe_pairs_total={len(pairs)}")
    print(f"this_batch_macros={len(selected_names)} pairs={len(selected)}")
    for n in selected_names:
        print(f"  {n}: {len(by[n])} pairs default={defaults[n]!r}")

    if args.dry_run or not args.apply:
        return 0

    drop = {p[0] for p in selected} | {p[1] for p in selected}
    # also drop blank lines strictly between undef and define of a selected pair
    for ui, di, _, _ in selected:
        for k in range(ui + 1, di):
            if not bare[k].strip():
                drop.add(k)

    out = [ln for i, ln in enumerate(lines) if i not in drop]
    HEADER.write_text("".join(out), encoding="utf-8")
    print(f"applied: removed_index_count={len(drop)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
