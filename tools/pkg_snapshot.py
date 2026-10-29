#!/usr/bin/env python3
"""pkg_snapshot —— 冻结一个「可回放」的交付包（`MDK-ARM/_pkg/<日期>_<名>/`）。

**为什么要它**：README §7.4 那条发现 —— 2026-10-09 之后的包**复原不出 bw/lpf**，
因为 `_pkg` 只抄了 `bringup_active.h` 与 `*.profile.h`，而 `bw`/`lpf` 住在
`config/motor_params_m1.h`，那个文件没人抄。**没有它，包就不可回放**，
台账里一切「带参数」的结论都站不住。

本工具把**参数单一源**一次冻结齐（PC 侧一律解析，不手抄）：

    config/  bringup_active.h / bringup_bench.h / motor_params_m1.h /
             irq_priority.h / motor_cfg.h / motor_cfg_m1.c / memory_map.h /
             hal_bridge.h
             profiles/*.profile.h     （全抄，不去猜哪个「活跃」）
    debug/   telem_table.c / telem_table.h
    MDK-ARM/rebuild_log.txt          → 同名；并抽 `Program Size:` 写 size.txt
    <hex>                            → 同名；并写 hex.sha256
    git.txt                          （HEAD + 分支 + `git status --porcelain`）
    README.md                        （空模板，人工补）
    model_params.json                （tools/model/params.py 快照）
    fwcfg.json                       （active profile 的 M1_* 宏求值）

用法：

    python tools/pkg_snapshot.py --name smo_lpf_fix
    python tools/pkg_snapshot.py --name smo_lpf_fix --dry-run
    python tools/pkg_snapshot.py --selftest

纪律：**只新增不覆盖**（除非 `--force`）；**不碰 git**（提交要人工批准）。
"""
from __future__ import annotations

import argparse
import hashlib
import json
import re
import shutil
import subprocess
import sys
from datetime import date
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
PKG_ROOT = ROOT / "MDK-ARM" / "_pkg"
HEX_DEFAULT = ROOT / "MDK-ARM" / "STM32G474RET6_MOTOR" / "STM32G474RET6_MOTOR.hex"
BUILD_LOG = ROOT / "MDK-ARM" / "rebuild_log.txt"

CONFIG_FILES = (
    "config/bringup_active.h", "config/bringup_bench.h",
    "config/motor_params_m1.h", "config/irq_priority.h",
    "config/motor_cfg.h", "config/motor_cfg_m1.c",
    "config/memory_map.h", "config/hal_bridge.h",
)
DEBUG_FILES = ("debug/telem_table.c", "debug/telem_table.h")

SIZE_RE = re.compile(r"Program Size:\s*(.+?)\s*$", re.M)
ACTIVE_RE = re.compile(r"^#define\s+(M1_USE_\w+)\s+1\b", re.M)


def parse_size(build_log: str) -> str:
    """从 rebuild_log 里抽 `Program Size: ...` 那一行（四数）。抽不到 → 空串。"""
    m = SIZE_RE.search(build_log or "")
    return m.group(1).strip() if m else ""


def active_profiles(bringup: str) -> list:
    """`#define M1_USE_XXX_PROFILE 1` 的宏名清单（可能 0/1/多个；**不猜文件名**）。"""
    return ACTIVE_RE.findall(bringup or "")


def sha256_of(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def _git(*args: str) -> str:
    try:
        r = subprocess.run(["git", *args], cwd=str(ROOT), capture_output=True,
                           text=True, encoding="utf-8", errors="replace")
    except OSError:
        return ""
    return (r.stdout or "").rstrip("\n")


def collect(hex_path: Path) -> list:
    """`(源绝对路径, 包内文件名)` 清单。`profiles/` 全抄，不猜「活跃的那一个」。"""
    items = [(ROOT / f, Path(f).name) for f in CONFIG_FILES]
    items += [(ROOT / f, Path(f).name) for f in DEBUG_FILES]
    items += [(q, q.name) for q in sorted((ROOT / "config" / "profiles").glob("*.profile.h"))]
    items += [(ROOT / "tools" / "model" / "params.py", "params.py")]
    items += [(BUILD_LOG, "rebuild_log.txt"), (hex_path, hex_path.name)]
    return items


def readme_stub(day: str, name: str, *, hex_sha: str, size: str, head: str,
                branch: str, dirty_n: int, actives: list) -> str:
    return "\n".join([
        "# %s_%s" % (day, name),
        "",
        "## 改了什么",
        "（人工补：一刀一包，1~2 处）",
        "",
        "## 怎么验",
        "（人工补：主回归门 / 短回归命令 + 结果）",
        "",
        "## 台架结果",
        "（人工补：PASS/FAIL + 主峰 / std / 饱和拍）",
        "",
        "## 冻结信息（自动生成，别手改）",
        "- 日期：%s" % day,
        "- hex sha256：%s" % hex_sha,
        "- Program Size：%s" % (size if size else "(未从 rebuild_log.txt 抽到)"),
        "- git HEAD：%s（分支 %s，dirty 条目 %d）" % (head, branch, dirty_n),
        "- 活跃 profile 宏：%s" % (", ".join(actives) if actives else "(没读到)"),
        "- 模型参数：model_params.json（tools/model/params.py 快照）",
        "- 控制器配置：fwcfg.json（active profile 的 M1_* 宏求值）",
        "",
    ])


def write_model_manifest(dst: Path) -> list:
    """Write params.py + active fwcfg snapshots; never fail the whole package."""
    out = []
    try:
        sys.path.insert(0, str(ROOT / "tools" / "model"))
        from dataclasses import asdict
        from params import M1_N5065
        (dst / "model_params.json").write_text(
            json.dumps(asdict(M1_N5065), indent=2, ensure_ascii=False),
            encoding="utf-8")
        out.append("model_params.json")
    except Exception as exc:
        (dst / "model_params.error.txt").write_text(str(exc), encoding="utf-8")
    try:
        sys.path.insert(0, str(ROOT / "tools" / "simtool"))
        from fwcfg import extract, preprocess_macros
        vals, raw, bad, active = extract(preprocess_macros())
        (dst / "fwcfg.json").write_text(json.dumps({
            "active_profiles": active,
            "values": vals, "macros": raw, "warnings": bad,
        }, indent=2, ensure_ascii=False), encoding="utf-8")
        out.append("fwcfg.json")
    except Exception as exc:
        (dst / "fwcfg.error.txt").write_text(str(exc), encoding="utf-8")
    return out


def freeze(name: str, *, day: str, hex_path: Path, dry: bool, force: bool) -> dict:
    dst = PKG_ROOT / ("%s_%s" % (day, name))
    items = collect(hex_path)
    missing = [str(s) for s, _ in items if not s.exists()]
    if missing:
        return {"ok": False, "why": "缺源文件", "missing": missing, "dst": str(dst)}
    if dst.exists() and any(dst.iterdir()) and not force:
        return {"ok": False, "why": "目标已存在且非空（要覆盖请加 --force）", "dst": str(dst)}
    if dry:
        return {"ok": True, "dry": True, "dst": str(dst), "files": [str(n) for _, n in items]}

    dst.mkdir(parents=True, exist_ok=True)
    for src, dstname in items:
        shutil.copy2(src, dst / dstname)

    size = parse_size(BUILD_LOG.read_text(encoding="utf-8", errors="replace")) \
        if BUILD_LOG.exists() else ""
    size_txt = ("Program Size: %s" % size) if size else "(抽不到 Program Size)"
    (dst / "size.txt").write_text(size_txt + "\n", encoding="utf-8")

    hex_sha = sha256_of(hex_path)
    (dst / "hex.sha256").write_text(hex_sha + "\n", encoding="utf-8")

    head = _git("rev-parse", "HEAD")
    branch = _git("rev-parse", "--abbrev-ref", "HEAD")
    dirty = _git("status", "--porcelain")
    (dst / "git.txt").write_text(
        "HEAD %s\nbranch %s\n\n-- git status --porcelain --\n%s\n" % (head, branch, dirty),
        encoding="utf-8")

    bringup = (ROOT / "config" / "bringup_active.h").read_text(encoding="utf-8", errors="replace")
    (dst / "README.md").write_text(
        readme_stub(day, name, hex_sha=hex_sha, size=size, head=head, branch=branch,
                    dirty_n=len(dirty.splitlines()), actives=active_profiles(bringup)),
        encoding="utf-8")
    generated = write_model_manifest(dst)
    return {"ok": True, "dst": str(dst), "n_files": len(items) + len(generated),
            "size": size}


def selftest() -> int:
    print("=== pkg_snapshot 自检 ===")
    fails = []

    def chk(name, ok, detail=""):
        print("  [%s] %s %s" % ("PASS" if ok else "FAIL", name, detail))
        if not ok:
            fails.append(name)

    log = ("compiling main.c...\nlinking...\n"
           "Program Size: Code=57908 RO-data=2612 RW-data=84 ZI-data=27644  \n"
           "FromELF: creating hex file...\n"
           "a.axf - 0 Error(s), 0 Warning(s).\n")
    chk("parse_size 抽四数", parse_size(log) == "Code=57908 RO-data=2612 RW-data=84 ZI-data=27644",
        parse_size(log))
    chk("parse_size 无该行 → 空串", parse_size("compiling a.c...\n") == "",
        repr(parse_size("compiling a.c...\n")))

    bringup = ("#undef M1_USE_HFI_STANDSTILL_PROFILE\n#define M1_USE_HFI_STANDSTILL_PROFILE 0\n"
               "#undef M1_USE_SPEED_LOW_SWEEP_PROFILE\n#define M1_USE_SPEED_LOW_SWEEP_PROFILE 1\n")
    got = active_profiles(bringup)
    chk("active_profiles 只认 =1、忽略 #undef",
        got == ["M1_USE_SPEED_LOW_SWEEP_PROFILE"], str(got))

    items = collect(HEX_DEFAULT)
    miss = [str(s) for s, _ in items if not s.exists()]
    chk("collect() 的源全都在", not miss,
        ("缺 " + ", ".join(miss)) if miss else "%d 项" % len(items))
    names = [n for _, n in items]
    chk("清单里有 motor_params_m1.h（§7.4 漏的就是它）", "motor_params_m1.h" in names)
    chk("清单里有全部 profile", len([n for n in names if n.endswith(".profile.h")]) >= 5,
        "%d 个 profile" % len([n for n in names if n.endswith(".profile.h")]))
    chk("清单里有 telem_table.h", "telem_table.h" in names)
    chk("清单里有 params.py", "params.py" in names)

    d = readme_stub("20261017", "x", hex_sha="ab", size="Code=1", head="h", branch="b",
                    dirty_n=2, actives=["M1_USE_X_PROFILE"])
    chk("README 模板带冻结信息", "hex sha256" in d and "M1_USE_X_PROFILE" in d)

    print("pkg_snapshot 自检%s" % ("全过" if not fails else "失败：" + ", ".join(fails)))
    return 1 if fails else 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="冻结一个可回放的 _pkg 交付包")
    ap.add_argument("--name", help="包名后缀（如 smo_lpf_fix）")
    ap.add_argument("--day", default=date.today().strftime("%Y%m%d"), help="日期前缀，默认今天")
    ap.add_argument("--hex", default=str(HEX_DEFAULT), help="要冻结的 hex（默认最新构建）")
    ap.add_argument("--dry-run", action="store_true", help="只打印清单，不落盘")
    ap.add_argument("--force", action="store_true", help="目标非空也覆盖")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args(argv)
    if a.selftest:
        return selftest()
    if not a.name:
        ap.error("--name 必给（或用 --selftest）")
    r = freeze(a.name, day=a.day, hex_path=Path(a.hex), dry=a.dry_run, force=a.force)
    if not r.get("ok"):
        print("失败：%s" % r.get("why"))
        for m in r.get("missing", []):
            print("  缺：" + m)
        return 2
    if r.get("dry"):
        print("dry-run：将写入 %s" % r["dst"])
        for f in r["files"]:
            print("  + " + f)
        return 0
    print("已冻结 %s（%d 个源文件）" % (r["dst"], r["n_files"]))
    print("  Program Size: %s" % (r["size"] if r["size"] else "(未抽到)"))
    return 0


if __name__ == "__main__":
    sys.exit(main())