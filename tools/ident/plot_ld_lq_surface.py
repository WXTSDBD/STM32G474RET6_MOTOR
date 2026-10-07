#!/usr/bin/env python3
"""Plot Ld/Lq quadratic surface from MCU ident burst (15-grid fine @1kHz).

Usage:
  python tools/ident/plot_ld_lq_surface.py VOFA+CSV/20260710/vofa+202607102336.csv
  python tools/ident/plot_ld_lq_surface.py a.csv b.csv --qc -o out.png
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

_TOOLS = Path(__file__).resolve().parents[1]
if str(_TOOLS) not in sys.path:
    sys.path.insert(0, str(_TOOLS))
_IDENT = Path(__file__).resolve().parent
if str(_IDENT) not in sys.path:
    sys.path.insert(0, str(_IDENT))

from parse_ident_vofa import find_header_rows, load_csv, parse_burst  # noqa: E402
from analyze_ld_lq_vasi import eval_quad_surface, rls_quad_surface  # noqa: E402

# Scheme A fine grid (M1_LD_LQ_IDENT_FINE_GRID_ENABLE=1)
ID_BIAS_FINE = np.array([0.5, 0.75, 1.0])
IQ_BIAS_FINE = np.array([0.0, 0.25, 0.5, 0.75, 1.0])

# Classic 9-grid fallback
ID_BIAS_9 = np.array([0.5, 1.0, 1.5])
IQ_BIAS_9 = np.array([0.0, 0.5, 1.0])

LD_LCR_UH = 59.0
LQ_LCR_UH = 87.0

# Default QC: down-weight known bad cells from repeat-run analysis
DEFAULT_EXCLUDE_LD = frozenset({(1.0, 0.0)})
DEFAULT_EXCLUDE_LQ = frozenset(
    {
        (0.5, 0.75),
        (0.5, 1.0),
        (0.75, 0.75),
        (0.75, 1.0),
        (1.0, 0.75),
        (1.0, 1.0),
    }
)

TIER_KEYS = {
    "fine": ("ld_fine_uH", "lq_fine_uH", "ld_fine_valid", "lq_fine_valid"),
    "coarse": ("ld_coarse_uH", "lq_coarse_uH", "ld_coarse_valid", "lq_coarse_valid"),
    "f2": ("ld_f2_uH", "lq_f2_uH", "ld_f2_valid", "lq_f2_valid"),
}


def parse_exclude(texts: list[str]) -> set[tuple[float, float]]:
    out: set[tuple[float, float]] = set()
    for t in texts:
        parts = t.replace(",", " ").split()
        if len(parts) != 2:
            raise ValueError(f"exclude pair needs Id Iq, got {t!r}")
        out.add((float(parts[0]), float(parts[1])))
    return out


def load_burst_points(
    path: Path,
    tier: str = "fine",
    *,
    exclude_ld: set[tuple[float, float]] | None = None,
    exclude_lq: set[tuple[float, float]] | None = None,
) -> tuple[dict, list[dict], list[dict]]:
    rows = load_csv(path)
    hdrs = find_header_rows(rows)
    if not hdrs:
        raise ValueError(f"no ident burst in {path}")
    burst = parse_burst(rows, hdrs[-1])

    ld_k, lq_k, ld_vk, lq_vk = TIER_KEYS[tier]
    exclude_ld = exclude_ld or set()
    exclude_lq = exclude_lq or set()

    ld_pts: list[dict] = []
    lq_pts: list[dict] = []
    for g in burst["grid"]:
        key = (round(g["id_bias"], 2), round(g["iq_bias"], 2))
        if g.get(ld_vk) and key not in exclude_ld:
            ld_pts.append(
                {
                    "id": g["id_bias"],
                    "iq": g["iq_bias"],
                    "l_uh": float(g[ld_k]),
                    "file": path.name,
                }
            )
        if g.get(lq_vk) and key not in exclude_lq:
            lq_pts.append(
                {
                    "id": g["id_bias"],
                    "iq": g["iq_bias"],
                    "l_uh": float(g[lq_k]),
                    "file": path.name,
                }
            )
    return burst, ld_pts, lq_pts


def fit_from_points(pts: list[dict], min_pts: int = 6) -> tuple[np.ndarray, float] | None:
    if len(pts) < min_pts:
        return None
    id_a = np.array([p["id"] for p in pts])
    iq_a = np.array([p["iq"] for p in pts])
    l_h = np.array([p["l_uh"] for p in pts]) * 1e-6
    return rls_quad_surface(id_a, iq_a, l_h)


def infer_grid_axes(pts: list[dict]) -> tuple[np.ndarray, np.ndarray]:
    ids = sorted({round(p["id"], 2) for p in pts})
    iqs = sorted({round(p["iq"], 2) for p in pts})
    if len(ids) * len(iqs) == len(pts) and len(ids) == 3 and len(iqs) == 5:
        return ID_BIAS_FINE, IQ_BIAS_FINE
    if len(ids) == 3 and len(iqs) == 3:
        return ID_BIAS_9, IQ_BIAS_9
    id_a = np.array(ids, dtype=float)
    iq_a = np.array(iqs, dtype=float)
    return id_a, iq_a


def plot_surfaces(
    csv_paths: list[Path],
    out_png: Path,
    *,
    tier: str = "fine",
    exclude_ld: set[tuple[float, float]] | None = None,
    exclude_lq: set[tuple[float, float]] | None = None,
    ld_coeff: np.ndarray | None = None,
    lq_coeff: np.ndarray | None = None,
    title_suffix: str = "",
) -> dict:
    try:
        import matplotlib.pyplot as plt
        from matplotlib import cm
    except ImportError as e:
        raise SystemExit("matplotlib required: pip install matplotlib") from e

    all_ld: list[dict] = []
    all_lq: list[dict] = []
    bursts = []
    for p in csv_paths:
        burst, ld_pts, lq_pts = load_burst_points(
            p, tier, exclude_ld=exclude_ld, exclude_lq=exclude_lq
        )
        bursts.append(burst)
        all_ld.extend(ld_pts)
        all_lq.extend(lq_pts)

    if ld_coeff is None:
        fit_ld = fit_from_points(all_ld)
        ld_coeff = fit_ld[0] if fit_ld else None
        ld_rms = fit_ld[1] * 1e6 if fit_ld else float("nan")
    else:
        ld_rms = float("nan")

    if lq_coeff is None:
        fit_lq = fit_from_points(all_lq)
        lq_coeff = fit_lq[0] if fit_lq else None
        lq_rms = fit_lq[1] * 1e6 if fit_lq else float("nan")
    else:
        lq_rms = float("nan")

    if ld_coeff is None and lq_coeff is None:
        raise ValueError("not enough valid points to fit Ld or Lq surface")

    id_axis, iq_axis = infer_grid_axes(all_ld or all_lq)
    pad = 0.05
    id_g = np.linspace(float(id_axis[0]) - pad, float(id_axis[-1]) + pad, 40)
    iq_g = np.linspace(float(iq_axis[0]) - pad, float(iq_axis[-1]) + pad, 40)
    ID, IQ = np.meshgrid(id_g, iq_g)

    n_files = len(csv_paths)
    scatter_cmaps = [cm.tab10(i) for i in range(n_files)]

    fig = plt.figure(figsize=(13, 9))
    fig.suptitle(
        f"Ld/Lq surface RLS ({tier}) — {', '.join(p.name for p in csv_paths)}{title_suffix}",
        fontsize=11,
    )

    def _add_3d(ax, coeff, pts, label, cmap_name, ref_uh: float | None):
        Z = eval_quad_surface(coeff, ID, IQ) * 1e6
        ax.plot_surface(ID, IQ, Z, cmap=cmap_name, alpha=0.55, linewidth=0, antialiased=True)
        for fi, path in enumerate(csv_paths):
            sub = [p for p in pts if p["file"] == path.name]
            if not sub:
                continue
            ax.scatter(
                [p["id"] for p in sub],
                [p["iq"] for p in sub],
                [p["l_uh"] for p in sub],
                c=[scatter_cmaps[fi]],
                s=55,
                edgecolors="k",
                linewidths=0.6,
                depthshade=True,
                label=path.stem if fi == 0 else path.stem,
            )
        ax.scatter([1.0], [0.0], [eval_quad_surface(coeff, 1.0, 0.0) * 1e6], c="red", s=80, marker="*")
        ax.scatter(
            [1.0],
            [0.5],
            [eval_quad_surface(coeff, 1.0, 0.5) * 1e6],
            c="lime",
            s=80,
            marker="*",
        )
        ax.set_xlabel("Id (A)")
        ax.set_ylabel("Iq (A)")
        ax.set_zlabel(f"{label} (uH)")
        if ref_uh is not None:
            ax.plot_surface(ID, IQ, np.full_like(Z, ref_uh), alpha=0.08, color="gray")

    def _add_contour(ax, coeff, pts, label, ref_uh: float | None):
        Z = eval_quad_surface(coeff, ID, IQ) * 1e6
        cf = ax.contourf(ID, IQ, Z, levels=16, cmap="viridis" if label == "Ld" else "plasma")
        ax.contour(ID, IQ, Z, levels=8, colors="k", linewidths=0.35, alpha=0.5)
        fig.colorbar(cf, ax=ax, shrink=0.85, label=f"{label} (uH)")
        for fi, path in enumerate(csv_paths):
            sub = [p for p in pts if p["file"] == path.name]
            if not sub:
                continue
            ax.scatter(
                [p["id"] for p in sub],
                [p["iq"] for p in sub],
                c=[scatter_cmaps[fi]],
                s=45,
                edgecolors="k",
                linewidths=0.5,
                zorder=5,
            )
        ax.scatter([1.0], [0.0], c="red", s=90, marker="*", zorder=6, label="(1,0)")
        ax.scatter([1.0], [0.5], c="lime", s=90, marker="*", zorder=6, label="(1,0.5)")
        if ref_uh is not None:
            ax.contour(ID, IQ, Z, levels=[ref_uh], colors=["w"], linewidths=2.0, linestyles="--")
        ax.set_xlabel("Id (A)")
        ax.set_ylabel("Iq (A)")
        ax.set_title(f"{label} contour")
        ax.set_aspect("equal", adjustable="box")

    if ld_coeff is not None:
        ax_ld3 = fig.add_subplot(221, projection="3d")
        _add_3d(ax_ld3, ld_coeff, all_ld, "Ld", "viridis", LD_LCR_UH)
        ld10 = float(eval_quad_surface(ld_coeff, 1.0, 0.0)) * 1e6
        ld15 = float(eval_quad_surface(ld_coeff, 1.0, 0.5)) * 1e6
        ax_ld3.set_title(f"Ld 3D  rms={ld_rms:.2f} uH  @(1,0)={ld10:.1f}  @(1,0.5)={ld15:.1f}")
        ax_ldc = fig.add_subplot(223)
        _add_contour(ax_ldc, ld_coeff, all_ld, "Ld", LD_LCR_UH)
        ax_ldc.legend(loc="upper left", fontsize=8)

    if lq_coeff is not None:
        ax_lq3 = fig.add_subplot(222, projection="3d")
        _add_3d(ax_lq3, lq_coeff, all_lq, "Lq", "plasma", LQ_LCR_UH)
        lq10 = float(eval_quad_surface(lq_coeff, 1.0, 0.0)) * 1e6
        lq15 = float(eval_quad_surface(lq_coeff, 1.0, 0.5)) * 1e6
        ax_lq3.set_title(f"Lq 3D  rms={lq_rms:.2f} uH  @(1,0)={lq10:.1f}  @(1,0.5)={lq15:.1f}")
        ax_lqc = fig.add_subplot(224)
        _add_contour(ax_lqc, lq_coeff, all_lq, "Lq", LQ_LCR_UH)
        ax_lqc.legend(loc="upper left", fontsize=8)

    fig.tight_layout()
    out_png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_png, dpi=140, bbox_inches="tight")
    plt.close(fig)

    meta = {
        "out_png": str(out_png),
        "n_ld_pts": len(all_ld),
        "n_lq_pts": len(all_lq),
        "ld_rms_uH": ld_rms,
        "lq_rms_uH": lq_rms,
    }
    if ld_coeff is not None:
        meta["ld_at_1_0_uH"] = float(eval_quad_surface(ld_coeff, 1.0, 0.0)) * 1e6
        meta["ld_at_1_0p5_uH"] = float(eval_quad_surface(ld_coeff, 1.0, 0.5)) * 1e6
        meta["ld_coeff"] = ld_coeff.tolist()
    if lq_coeff is not None:
        meta["lq_at_1_0_uH"] = float(eval_quad_surface(lq_coeff, 1.0, 0.0)) * 1e6
        meta["lq_at_1_0p5_uH"] = float(eval_quad_surface(lq_coeff, 1.0, 0.5)) * 1e6
        meta["lq_coeff"] = lq_coeff.tolist()
    return meta


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", type=Path, nargs="+", help="VOFA CSV with MCU ident burst")
    ap.add_argument("-o", "--out", type=Path, default=None, help="output PNG (default: <first_csv>.ld_lq_surface.png)")
    ap.add_argument("--tier", choices=TIER_KEYS, default="fine", help="which tier to fit/plot")
    ap.add_argument(
        "--qc",
        action="store_true",
        help="exclude known bad grid cells (Ld: (1,0); Lq: Iq>=0.75 etc.)",
    )
    ap.add_argument("--exclude-ld", nargs="*", default=[], metavar="Id,Iq", help="extra Ld exclude pairs")
    ap.add_argument("--exclude-lq", nargs="*", default=[], metavar="Id,Iq", help="extra Lq exclude pairs")
    ap.add_argument("--exclude", nargs="*", default=[], metavar="Id,Iq", help="exclude from both Ld and Lq")
    args = ap.parse_args()

    exclude_ld = parse_exclude(args.exclude_ld + args.exclude)
    exclude_lq = parse_exclude(args.exclude_lq + args.exclude)
    if args.qc:
        exclude_ld |= DEFAULT_EXCLUDE_LD
        exclude_lq |= DEFAULT_EXCLUDE_LQ

    out = args.out
    if out is None:
        out = args.csv[0].with_suffix(".ld_lq_surface.png")

    suffix = " [QC exclude]" if args.qc else ""
    meta = plot_surfaces(
        args.csv,
        out,
        tier=args.tier,
        exclude_ld=exclude_ld,
        exclude_lq=exclude_lq,
        title_suffix=suffix,
    )

    print(f"saved: {meta['out_png']}")
    print(f"  fit points: Ld={meta['n_ld_pts']}  Lq={meta['n_lq_pts']}")
    if "ld_at_1_0_uH" in meta:
        print(
            f"  Ld: rms={meta['ld_rms_uH']:.2f} uH  "
            f"@(1,0)={meta['ld_at_1_0_uH']:.1f}  @(1,0.5)={meta['ld_at_1_0p5_uH']:.1f}  "
            f"(LCR {LD_LCR_UH:.0f})"
        )
    if "lq_at_1_0_uH" in meta:
        print(
            f"  Lq: rms={meta['lq_rms_uH']:.2f} uH  "
            f"@(1,0)={meta['lq_at_1_0_uH']:.1f}  @(1,0.5)={meta['lq_at_1_0p5_uH']:.1f}  "
            f"(LCR {LQ_LCR_UH:.0f})"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
