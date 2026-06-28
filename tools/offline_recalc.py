#!/usr/bin/env python3
"""
Offline Iq/Id recalculation from VOFA+ CSV.

Recomputes Clarke+Park from CSV ch0/1/2 + theta_el ch5.
  M1_VOFA_FOC_ABC=1: ch0–2 = foc_ia/b/c (A), ch4 = Id — direct Park self-check vs ch3.
  Legacy: ch0–2 = adc_zeroed LSB + binding [2,1,0] sign [-1,-1,-1].

Usage:
  python tools/offline_recalc.py <csv_or_dir> [--recon] [--skip N] [--out dir]
  python tools/offline_recalc.py VOFA+CSV/20260617/
"""
from __future__ import annotations

import argparse
import csv
import glob
import math
import os
import sys
from dataclasses import dataclass, field
from typing import Iterable, List, Optional, Tuple

M1_ADC_SCALE_A_LSB = 3.3 / (4096 * 0.01 * 10)  # 0.008057 A/LSB
NEG_THRESH_A = 0.1
RECON_MIN_A = 0.05
DEFAULT_SKIP = 400  # ~2 s @ 200 Hz


@dataclass
class Sample:
    r0: float
    r1: float
    r2: float
    iq_fw: float
    ch4: float
    theta: float


@dataclass
class SectorStats:
    n: int = 0
    neg_fw: int = 0
    neg_iq_pos: int = 0
    neg_iq_neg: int = 0
    sum_fw: float = 0.0
    sum_iq_pos: float = 0.0
    sum_id_pos: float = 0.0
    ss_fw_iq: float = 0.0
    ss_fw_id: float = 0.0
    sign_agree_iq: int = 0


@dataclass
class FileResult:
    path: str
    foc_abc: bool
    n: int
    neg_fw: int
    neg_iq_pos: int
    neg_iq_neg: int
    ss_fw_iq: float = 0.0
    sectors: dict = field(default_factory=dict)


def load_csv(path: str) -> List[Sample]:
    rows: List[Sample] = []
    with open(path, encoding="utf-8", newline="") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        if header is None:
            return rows
        for parts in reader:
            if len(parts) < 6:
                continue
            rows.append(
                Sample(
                    r0=float(parts[0]),
                    r1=float(parts[1]),
                    r2=float(parts[2]),
                    iq_fw=float(parts[3]),
                    ch4=float(parts[4]),
                    theta=float(parts[5]),
                )
            )
    return rows


def binding_abc(raw: Sample) -> Tuple[float, float, float]:
    ia = -raw.r2 * M1_ADC_SCALE_A_LSB
    ib = -raw.r1 * M1_ADC_SCALE_A_LSB
    ic = -raw.r0 * M1_ADC_SCALE_A_LSB
    return ia, ib, ic


def maybe_recon(ia: float, ib: float, ic: float, sector: int) -> float:
    if sector not in (1, 2):
        return ic
    a, b, c = abs(ia), abs(ib), abs(ic)
    if a < RECON_MIN_A and b < RECON_MIN_A and c < RECON_MIN_A:
        return ic
    return -(ia + ib)


def clarke_park(ia: float, ib: float, ic: float, theta: float) -> Tuple[float, float]:
    ialpha = ia
    ibeta = (ib - ic) / math.sqrt(3.0)
    c, s = math.cos(theta), math.sin(theta)
    id_ = ialpha * c + ibeta * s
    iq = -ialpha * s + ibeta * c
    return id_, iq


def detect_foc_abc_mode(samples: List[Sample], skip: int) -> bool:
    """True if ch0–2 look like foc_ia/b/c in Amps (not adc_zeroed LSB)."""
    chunk = samples[skip : skip + 200] if len(samples) > skip + 200 else samples[skip:]
    if not chunk:
        return False
    ints = amps = 0
    for s in chunk:
        for k in (s.r0, s.r1, s.r2):
            if abs(k - round(k)) < 1e-3 and abs(k) >= 1.0:
                ints += 1
            if abs(k) > 0.02 and abs(k) < 50.0 and abs(k - round(k)) > 0.01:
                amps += 1
    return amps > ints


def sample_to_abc(raw: Sample, foc_abc: bool) -> Tuple[float, float, float]:
    if foc_abc:
        return raw.r0, raw.r1, raw.r2
    return binding_abc(raw)


def sector_from_ch4(ch4: float) -> int:
    s = int(round(ch4))
    if 1 <= s <= 6:
        return s
    return 0


def analyze_file(
    path: str,
    skip: int,
    recon: bool,
    write_detail: bool,
    out_dir: Optional[str],
) -> FileResult:
    samples = load_csv(path)
    if not samples:
        raise ValueError(f"no data rows in {path}")

    foc_abc = detect_foc_abc_mode(samples, skip)
    sectors = {s: SectorStats() for s in range(1, 7)}
    neg_fw = neg_iq_pos = neg_iq_neg = 0
    ss_fw_iq = 0.0
    detail_path = None
    detail_f = None

    if write_detail:
        base = os.path.splitext(os.path.basename(path))[0]
        detail_dir = out_dir or os.path.dirname(path)
        detail_path = os.path.join(detail_dir, f"{base}_offline_recalc.csv")
        detail_f = open(detail_path, "w", encoding="utf-8", newline="")
        w = csv.writer(detail_f)
        w.writerow(
            [
                "Iq_fw",
                "sector",
                "Iq_offline_pos",
                "Id_offline_pos",
                "Iq_offline_neg",
                "Id_offline_neg",
                "theta_el",
            ]
        )

    n = 0
    for i, raw in enumerate(samples):
        if i < skip:
            continue
        n += 1
        ia, ib, ic = sample_to_abc(raw, foc_abc)
        sec = sector_from_ch4(raw.ch4) if not foc_abc else 0
        if recon and not foc_abc and sec in (1, 2):
            ic = maybe_recon(ia, ib, ic, sec)

        id_pos, iq_pos = clarke_park(ia, ib, ic, raw.theta)
        id_neg, iq_neg = clarke_park(ia, ib, ic, -raw.theta)

        ss_fw_iq += (raw.iq_fw - iq_pos) ** 2

        if raw.iq_fw < -NEG_THRESH_A:
            neg_fw += 1
        if iq_pos < -NEG_THRESH_A:
            neg_iq_pos += 1
        if iq_neg < -NEG_THRESH_A:
            neg_iq_neg += 1

        if sec >= 1:
            st = sectors[sec]
            st.n += 1
            if raw.iq_fw < -NEG_THRESH_A:
                st.neg_fw += 1
            if iq_pos < -NEG_THRESH_A:
                st.neg_iq_pos += 1
            if iq_neg < -NEG_THRESH_A:
                st.neg_iq_neg += 1
            st.sum_fw += raw.iq_fw
            st.sum_iq_pos += iq_pos
            st.sum_id_pos += id_pos
            d_iq = raw.iq_fw - iq_pos
            d_id = raw.iq_fw - id_pos
            st.ss_fw_iq += d_iq * d_iq
            st.ss_fw_id += d_id * d_id
            if (raw.iq_fw >= 0) == (iq_pos >= 0):
                st.sign_agree_iq += 1

        if detail_f is not None:
            w.writerow(
                [
                    f"{raw.iq_fw:.6f}",
                    sec,
                    f"{iq_pos:.6f}",
                    f"{id_pos:.6f}",
                    f"{iq_neg:.6f}",
                    f"{id_neg:.6f}",
                    f"{raw.theta:.6f}",
                ]
            )

    if detail_f is not None:
        detail_f.close()

    res = FileResult(
        path=path,
        foc_abc=foc_abc,
        n=n,
        neg_fw=neg_fw,
        neg_iq_pos=neg_iq_pos,
        neg_iq_neg=neg_iq_neg,
        ss_fw_iq=ss_fw_iq,
        sectors=sectors,
    )
    if detail_path:
        res.detail_path = detail_path  # type: ignore[attr-defined]
    return res


def pct(num: int, den: int) -> float:
    return 100.0 * num / den if den else 0.0


def rms(ss: float, n: int) -> float:
    return math.sqrt(ss / n) if n else float("nan")


def print_summary(results: Iterable[FileResult], recon: bool) -> None:
    rows = list(results)
    if not rows:
        print("No CSV files processed.")
        return

    print("=" * 96)
    print(
        f"Offline recalc  neg threshold={NEG_THRESH_A}A  "
        f"(foc_abc=ch0–2 are Ia/Ib/Ic in A; legacy=adc_zeroed+binding)"
    )
    print("=" * 96)
    print(
        f"{'file':<40} {'in':>4} {'N':>6} {'rms':>7} {'fw%':>6} {'+θ%':>6} {'-θ%':>6}  best"
    )
    print("-" * 96)
    for r in rows:
        name = os.path.basename(r.path)
        if len(name) > 39:
            name = "..." + name[-36:]
        fw_p = pct(r.neg_fw, r.n)
        pos_p = pct(r.neg_iq_pos, r.n)
        neg_p = pct(r.neg_iq_neg, r.n)
        best = "+θ" if pos_p <= neg_p else "-θ"
        rms_err = rms(r.ss_fw_iq, r.n)
        mode = "abc" if r.foc_abc else "raw"
        print(
            f"{name:<40} {mode:>4} {r.n:>6} {rms_err:>7.4f} {fw_p:>5.1f}% {pos_p:>5.1f}% {neg_p:>5.1f}%  {best}"
        )

    # Detailed sector table for the first file (or only file)
    if len(rows) == 1 and not rows[0].foc_abc:
        r = rows[0]
        print()
        print(f"Per-sector ({os.path.basename(r.path)}):")
        print(
            f"{'sec':>3} {'N':>6} {'fw<0%':>7} {'+θ<0%':>7} "
            f"{'sign%':>7} {'rms|fw-Iq|':>10} {'rms|fw-Id|':>10} {'fw_mean':>8} {'+θ_mean':>8}"
        )
        print("-" * 88)
        for sec in range(1, 7):
            st = r.sectors[sec]
            if st.n == 0:
                continue
            print(
                f"{sec:>3} {st.n:>6} {pct(st.neg_fw, st.n):>6.1f}% "
                f"{pct(st.neg_iq_pos, st.n):>6.1f}% {pct(st.sign_agree_iq, st.n):>6.1f}% "
                f"{rms(st.ss_fw_iq, st.n):>10.3f} {rms(st.ss_fw_id, st.n):>10.3f} "
                f"{st.sum_fw / st.n:>8.3f} {st.sum_iq_pos / st.n:>8.3f}"
            )
        print()
        print(
            "Interpret: foc_abc mode → rms|fw-Iq| should be ~0 if Park self-consistent; "
            "neg Iq% on fw vs +θ shows real motor/binding input issue."
        )
        if hasattr(r, "detail_path"):
            print(f"Detail CSV: {r.detail_path}")


def collect_paths(target: str) -> List[str]:
    if os.path.isdir(target):
        paths = sorted(glob.glob(os.path.join(target, "*.csv")))
        return [p for p in paths if "_offline_recalc" not in p]
    return [target]


def main() -> int:
    ap = argparse.ArgumentParser(description="Offline Clarke+Park recalc from VOFA CSV")
    ap.add_argument("target", help="CSV file or directory")
    ap.add_argument("--recon", action="store_true", help="simulate sector 1-2 KCL recon")
    ap.add_argument("--skip", type=int, default=DEFAULT_SKIP, help="skip first N samples")
    ap.add_argument("--out", default=None, help="output dir for detail CSV")
    ap.add_argument("--no-detail", action="store_true", help="skip per-sample output CSV")
    args = ap.parse_args()

    paths = collect_paths(args.target)
    if not paths:
        print(f"No CSV found under {args.target}", file=sys.stderr)
        return 1

    results: List[FileResult] = []
    for path in paths:
        try:
            r = analyze_file(
                path,
                skip=args.skip,
                recon=args.recon,
                write_detail=not args.no_detail and len(paths) == 1,
                out_dir=args.out,
            )
            results.append(r)
        except Exception as e:
            print(f"SKIP {path}: {e}", file=sys.stderr)

    print_summary(results, args.recon)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
