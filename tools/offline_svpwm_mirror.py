#!/usr/bin/env python3
"""Offline mirror of foc_svpwm.c setPhaseVoltage_core (Ud=0 path)."""

import csv
import math
from pathlib import Path

PI = math.pi
PI_2 = PI / 2
PI_3 = PI / 3
TWOPI = 2 * PI
SQRT3 = math.sqrt(3)
SIN_PI3 = math.sin(PI_3)
VBUS = 24.0
INV_VBUS = 1.0 / VBUS
INV_PI3 = 1.0 / PI_3
HALF = 0.5
RS = 0.115


def sincos(x):
    return math.cos(x), math.sin(x)


def svpwm_t1_t2(theta, uref):
    c, s = sincos(theta)
    sin_pi3_m_theta = SIN_PI3 * c - HALF * s
    scale = SQRT3 * uref
    t1 = scale * sin_pi3_m_theta
    t2 = scale * s
    return t1, t2


def sector_from_angle_ref(angle_ref):
    return int(angle_ref * INV_PI3) % 6 + 1


def angle_ref_ud0(uq, angle_el):
    ar = angle_el + PI_2
    if uq < 0:
        ar += PI
    if ar >= TWOPI:
        ar -= TWOPI
    return ar


def duties_from_t1_t2(sector, t1, t2, t0):
    th = t0 * HALF
    if sector == 1:
        return t1 + t2 + th, t2 + th, th
    if sector == 2:
        return t1 + th, t1 + t2 + th, th
    if sector == 3:
        return th, t1 + t2 + th, t2 + th
    if sector == 4:
        return th, t1 + th, t1 + t2 + th
    if sector == 5:
        return t2 + th, th, t1 + t2 + th
    if sector == 6:
        return t1 + t2 + th, th, t1 + th
    return 0.5, 0.5, 0.5


def duty_dev(ta, tb, tc):
    return max(abs(ta - HALF), abs(tb - HALF), abs(tc - HALF))


def set_phase_voltage_ud0(uq, angle_el):
    uq_abs = abs(uq)
    uref = uq_abs * INV_VBUS
    if uref > 0.577:
        uref = 0.577
    ar = angle_ref_ud0(uq, angle_el)
    sec = sector_from_angle_ref(ar)
    theta = ar - (sec - 1) * PI_3
    t1, t2 = svpwm_t1_t2(theta, uref)
    t0 = max(0.0, 1.0 - t1 - t2)
    ta, tb, tc = duties_from_t1_t2(sec, t1, t2, t0)
    return {
        "Uref": uref,
        "angle_ref_deg": math.degrees(ar),
        "sector": sec,
        "theta_loc_deg": math.degrees(theta),
        "T1": t1,
        "T2": t2,
        "T0": t0,
        "Ta": ta,
        "Tb": tb,
        "Tc": tc,
        "duty_dev": duty_dev(ta, tb, tc),
        "duty_ab": abs(ta - tb),
    }


def compare_csv(path: Path, theta_el: float) -> None:
    print(f"\n=== Firmware mirror vs CSV: {path.name} ===")
    print(f"theta_el = {theta_el:.6f} rad ({math.degrees(theta_el):.2f} deg)\n")
    print(
        f"{'Uq':>5} {'Uref_csv':>9} {'Uref_sim':>9} {'dd_csv':>9} {'dd_sim':>9} "
        f"{'dab_csv':>9} {'dab_sim':>9} {'match':>7} {'sec':>4} {'th_loc':>7}"
    )
    buckets = {0.0: [], 0.2: [], 0.5: [], 1.0: [], 2.0: []}
    with path.open(newline="", encoding="utf-8") as f:
        r = csv.reader(f)
        next(r)
        for row in r:
            v = [float(x) for x in row[:6]]
            for cmd in buckets:
                if abs(v[4] - cmd) < 0.05:
                    buckets[cmd].append(v)
                    break
    for uq in sorted(buckets):
        if not buckets[uq]:
            continue
        tail = buckets[uq][-5000:]
        n = len(tail)
        uref_c = sum(x[0] for x in tail) / n
        dd_c = sum(x[1] for x in tail) / n
        dab_c = sum(x[2] for x in tail) / n
        sim = set_phase_voltage_ud0(uq, theta_el)
        match = dab_c / sim["duty_ab"] if sim["duty_ab"] > 1e-9 else 0.0
        print(
            f"{uq:5.1f} {uref_c:9.5f} {sim['Uref']:9.5f} {dd_c:9.5f} {sim['duty_dev']:9.5f} "
            f"{dab_c:9.5f} {sim['duty_ab']:9.5f} {match:7.3f} "
            f"{sim['sector']:4d} {sim['theta_loc_deg']:7.2f}"
        )


def theta_sweep(uq=0.2):
    print(f"\n=== Theta sweep @ Uq={uq} V ===")
    print(f"{'theta_deg':>10} {'sector':>7} {'th_loc':>8} {'T1':>9} {'T2':>9} {'duty_ab':>9} {'duty_dev':>9}")
    best = (0.0, 0)
    for deg in range(0, 360, 5):
        th = math.radians(deg)
        s = set_phase_voltage_ud0(uq, th)
        if s["duty_ab"] > best[0]:
            best = (s["duty_ab"], deg)
        if deg % 30 == 0 or abs(deg - 330) <= 5:
            print(
                f"{deg:10d} {s['sector']:7d} {s['theta_loc_deg']:8.2f} {s['T1']:9.5f} {s['T2']:9.5f} "
                f"{s['duty_ab']:9.5f} {s['duty_dev']:9.5f}"
            )
    print(f"Max duty_ab @ theta={best[1]} deg -> {best[0]:.5f}")


def main():
    theta = 5.750107
    csv_path = Path(__file__).resolve().parents[1] / "VOFA+CSV" / "20260629" / "vofa+202606300812.csv"
    compare_csv(csv_path, theta)

    s = set_phase_voltage_ud0(0.2, theta)
    print("\n=== Geometry @ theta=329.5 deg, Uq=0.2 V ===")
    print(f"angle_ref = {s['angle_ref_deg']:.2f} deg (encoder theta + 90 deg)")
    print(f"sector={s['sector']}  theta_local={s['theta_loc_deg']:.2f} deg")
    print(f"T1={s['T1']:.6f}  T2={s['T2']:.6f}  T0={s['T0']:.6f}")
    print(f"Ta={s['Ta']:.6f}  Tb={s['Tb']:.6f}  Tc={s['Tc']:.6f}")
    print(f"duty_ab={s['duty_ab']:.6f}  duty_dev={s['duty_dev']:.6f}")

    theta_sweep(0.2)


if __name__ == "__main__":
    main()
