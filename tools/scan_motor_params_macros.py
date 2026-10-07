#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""兼容入口：转发到 tools/macro_map.py（包 8.1 正式扫描器）。"""

from __future__ import annotations

import runpy
import sys
from pathlib import Path

print(
    "note: scan_motor_params_macros.py → macro_map.py "
    "(docs/方案_宏归属表_2026-10-07.md §6)",
    file=sys.stderr,
)
sys.argv[0] = str(Path(__file__).with_name("macro_map.py"))
runpy.run_path(str(Path(__file__).with_name("macro_map.py")), run_name="__main__")
