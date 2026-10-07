#!/usr/bin/env python3
"""Offline: d-table vs plut vs VOFA burst — why >0.9A looks flat."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from analyze_id_inject_pass import analyze_file, AMP_TABLE  # noqa: E402
from offline_deadband_geo_merge import (  # noqa: E402
    CAPTURE_EPS,
    ID_CAL_AMP_TABLE,
    CapturePoint,
    GeoSample,
    build_geo_pool,
    build_plut,
    build_plut_from_dlut_30,
    enforce_val_monotone,
    geo_dlut_point,
    geo_fallback_30,
    median_u,
    sort_plut_pairs,
    THETA_A,
    THETA_B,
)

THETA_MATCH = 0.02


def theta_cluster(theta: float, cluster: int) -> bool:
    ref = THETA_A if cluster == 0 else THETA_B
    return abs(theta - ref) < THETA_MATCH


def build_plut_cluster_offline(
    samples: list[GeoSample],
    id_anchor: np.ndarray,
    ud_anchor: np.ndarray,
    cluster: int,
) -> tuple[np.ndarray, np.ndarray]:
    n = len(id_anchor)
    amps = np.zeros(n)
    vals = np.zeros(n)
    for k in range(n):
        id_k = float(id_anchor[k])
        ud_k = float(ud_anchor[k])
        hit = [
            s
            for s in samples
            if abs(s.id_capture - id_k) <= CAPTURE_EPS and theta_cluster(s.theta_el, cluster)
        ]
        if hit:
            amps[k], vals[k] = max(s.i_abs for s in hit), median_u([s.u_abs for s in hit])
        else:
            amps[k], vals[k] = geo_fallback_30(id_k, ud_k, None)
    return sort_plut_pairs(amps, enforce_val_monotone(vals))


def rows_to_caps(rows: list[dict], leg: str) -> list[CapturePoint]:
    out = []
    for r in rows:
        out.append(
            CapturePoint(
                id_ref=r["id_ref"],
                id_fb=r["id"],
                ud_pi=r["ud"],
                ud_res=abs(r["ud_res"]),
                theta_el=THETA_A if "30" in r["pass"] else THETA_B,
                leg=leg,
                t_mid=r["t_end"],
            )
        )
    return out


def main() -> None:
    path = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "VOFA+CSV/20260630/vofa+202607010121.csv"
    a = analyze_file(path)
    lut = a["lut"]
    p0a, p0b = a["pass0a"], a["pass0b"]

    amps = ID_CAL_AMP_TABLE.copy()
    dlut_vals = np.array([r["ud_res"] for r in p0a])

    print(f"File: {a['name']}")
    print("\n=== 1. Raw d-table (Pass0-A @30°, Ud_res) — NOT flat ===")
    for i in [0, 5, 10, 15, 20, 25, 28, 29, 30, 31]:
        r = p0a[i]
        id_g = float(r["id_ref"])
        pts = geo_dlut_point(THETA_A, id_g, abs(r["ud_res"]))
        if not pts:
            continue
        best = max(pts, key=lambda p: p.i_abs)
        print(
            f"  Id={r['id_ref']:.3f}  Ud_res={r['ud_res']:.4f}  "
            f"-> plut_amp={best.i_abs:.4f}  u_abs={best.u_abs:.4f}"
        )

    plut_a, plut_v = build_plut_from_dlut_30(amps, dlut_vals)
    print("\n=== 2. plut_from_dlut_30 (30° only, ×geo transform) ===")
    for i in [0, 5, 10, 15, 20, 25, 28, 29, 30, 31]:
        print(f"  anchor={amps[i]:.3f}  amp={plut_a[i]:.4f}  val={plut_v[i]:.4f}")

    captures = rows_to_caps(p0a, "A") + rows_to_caps(p0b, "B")
    pool = build_geo_pool(captures)
    la, lv = lut["amps"], lut["vals"]

    print("\n=== 3b. plut[0] dual-angle merge (VOFA dump source) ===")
    pa_m, pv_m = build_plut(
        pool, amps, dlut_vals, amp_theta="A", val_theta=None, val_mode="median", outlier_v=None
    )
    for i in [0, 5, 10, 15, 20, 25, 28, 29, 30, 31]:
        print(
            f"  k={i} merge amp={pa_m[i]:.4f} val={pv_m[i]:.4f}  "
            f"burst amp={la[i]:.4f} val={lv[i]:.4f}"
        )
    err_m = np.max(np.abs(pv_m[:32] - lv[:32]))
    print(f"  dual-merge vs burst max|Δval| = {err_m:.4f} V")

    print("\n=== 3c. Cluster plut (runtime TWO_CLUSTER select) ===")
    clusters = {}
    for cl in (0, 1):
        pa, pv = build_plut_cluster_offline(pool, amps, dlut_vals, cl)
        clusters[cl] = (pa, pv)
        print(f"  cluster {cl} ({'30°' if cl == 0 else '0°'}):")
        for i in [0, 5, 10, 15, 20, 25, 28, 29, 30, 31]:
            print(f"    anchor={amps[i]:.3f}  amp={pa[i]:.4f}  val={pv[i]:.4f}")

    print("\n=== 4. VOFA burst (= plut[0] dual-merge, NOT s_dlut) ===")
    for i in list(range(5)) + list(range(max(0, len(la) - 5), len(la))):
        print(f"  amp={la[i]:.4f}  val={lv[i]:.4f}")

    print("\n=== 5. 30° vs 0° Ud_res (Pass0 capture) ===")
    for id_k in (0.3, 0.5, 0.9, 1.0, 1.5):
        ia = int(np.argmin(np.abs(AMP_TABLE - id_k)))
        u30, u0 = p0a[ia]["ud_res"], p0b[ia]["ud_res"]
        print(f"  Id={id_k:.1f}A: 30°={u30:.3f}V  0°={u0:.3f}V  Δ(30-0)={u30 - u0:+.3f}V")

    print("\n=== 6. High-I slope: dUd_res vs dplut_val (Pass0-A) ===")
    for i in range(25, 32):
        did = p0a[i]["id_ref"] - p0a[i - 1]["id_ref"]
        dud = p0a[i]["ud_res"] - p0a[i - 1]["ud_res"]
        du_plut = plut_v[i] - plut_v[i - 1]
        print(
            f"  {p0a[i-1]['id_ref']:.3f}->{p0a[i]['id_ref']:.3f}A: "
            f"dUd_res={dud:.4f}V  dplut={du_plut:.4f}V  ({dud/did:.3f} V/A)"
        )

    pa0, pv0 = clusters[0]
    err = np.max(np.abs(la[:32] - pv_m[:32]))
    print(f"\n=== 7. burst vs offline dual-merge max|Δval| = {err:.4f} V ===")

    # plateau count
    flat = sum(1 for i in range(1, len(lv)) if abs(lv[i] - lv[i - 1]) < 0.002)
    print(f"=== 8. burst plateau: {flat} bins with |Δval|<2mV; val@0.9A={lv[26]:.3f} val@1.5A={lv[-1]:.3f} ===")


if __name__ == "__main__":
    main()
