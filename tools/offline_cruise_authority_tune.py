#!/usr/bin/env python3
"""Offline: IF→OBS cruise metrics + Bd/iq_min guidance (no firmware flash)."""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(r"D:\stm32\STM32G474RET6_MOTOR")
CSV_DIR = ROOT / "VOFA+CSV" / "20260912"
OUT = ROOT / "MDK-ARM" / "_offline_cruise_tune_20260912.json"

# Motor / loop (from project docs + active IF profile)
KT = 0.068  # Nm/A
POLE_PAIRS = 7
KP = 0.0008  # A/rpm (handoff weak PI)
KI = 0.00005  # A/rpm / sample-ish (discrete; treat lightly)
BD_FW = 0.002  # A/rpm HP damp currently
FS = 10000.0
CTRL_TS = 50e-6
SPEED_TS = CTRL_TS * 10  # SPEED_DECIM=10

# 5065 outrunner J is small; bracket if unknown
J_CANDIDATES = [5e-6, 1e-5, 2e-5, 5e-5, 1e-4]  # kg·m^2

PACKS = {
    "1433_HP_lock1": "vofa+202609121433.csv",  # gold-ish HP only
    "1453_iqmin_0p6": "vofa+202609121453.csv",  # pass stage -0.6
    "1503_iqmin_1p2": "vofa+202609121503.csv",  # fail stage -1.2
}


def load_csv(name: str) -> pd.DataFrame:
    path = CSV_DIR / name
    df = pd.read_csv(
        path,
        skiprows=1,
        header=None,
        names=[
            "w_enc",
            "w_ref",
            "iq_ref",
            "th_err",
            "w_obs",
            "spd_fb",
            "iq",
            "emag",
            "uq",
            "th_park",
            "spd_on",
            "ss",
        ],
        low_memory=False,
    )
    return df.apply(pd.to_numeric, errors="coerce").dropna().reset_index(drop=True)


def find_cruise(df: pd.DataFrame):
    ss = df.ss.to_numpy()
    wref = df.w_ref.to_numpy()
    spdon = df.spd_on.to_numpy()
    n = len(df)
    cand = np.where((ss >= 2.95) & (ss < 3.5))[0]
    o0 = int(cand[0]) if len(cand) else 0
    spd = np.where((spdon > 0.5) & (np.arange(n) >= o0))[0]
    i_spd = int(spd[0]) if len(spd) else o0
    s1 = None
    for i in range(max(o0, i_spd), n):
        if wref[i] > 960:
            s1 = i
            break
    return o0, s1, n


def window_metrics(df, a, b) -> dict:
    spdfb = df.spd_fb.to_numpy()
    wref = df.w_ref.to_numpy()
    iqr = df.iq_ref.to_numpy()
    wenc = df.w_enc.to_numpy()
    m = (
        (spdfb[a:b] > 200)
        & (spdfb[a:b] < 2000)
        & (iqr[a:b] > -5)
        & (iqr[a:b] < 10)
    )
    if m.sum() < 500:
        return {"n": int(m.sum()), "ok": False}
    fb = spdfb[a:b][m]
    ir = iqr[a:b][m]
    ew = np.abs(wref[a:b][m] - fb)
    enc = wenc[a:b][m]
    x = fb - fb.mean()
    hunt_hz = None
    if len(x) > int(2 * FS):
        X = np.fft.rfft(x * np.hanning(len(x)))
        f = np.fft.rfftfreq(len(x), 1 / FS)
        band = (f >= 0.2) & (f <= 5.0)
        if band.any():
            hunt_hz = float(f[band][np.argmax(np.abs(X[band]))])
    return {
        "ok": True,
        "n": int(m.sum()),
        "dur_s": float((b - a) / FS),
        "mean": float(fb.mean()),
        "std": float(fb.std()),
        "p5_p95": float(np.percentile(fb, 95) - np.percentile(fb, 5)),
        "ew_mean": float(ew.mean()),
        "ew_p95": float(np.percentile(ew, 95)),
        "iqr_min": float(ir.min()),
        "iqr_max": float(ir.max()),
        "iqr_mean": float(ir.mean()),
        "hit_m025": float((ir <= -0.24).mean()),
        "hit_m055": float((ir <= -0.55).mean()),
        "hit_m115": float((ir <= -1.15).mean()),
        "enc_std": float(enc.std()),
        "hunt_hz": hunt_hz,
        "ew_gt100_frac": float((ew > 100).mean()),
        "ew_gt200_frac": float((ew > 200).mean()),
    }


def analyze_pack(label: str, fname: str) -> dict:
    df = load_csv(fname)
    o0, s1, n = find_cruise(df)
    if s1 is None:
        return {"label": label, "error": "no cruise"}
    # late soak: last 8s before end-1s, but after cruise+8s
    a = max(s1 + int(8 * FS), n - int(9 * FS))
    b = n - int(1 * FS)
    late = window_metrics(df, a, b)
    # mid after cruise+5 for 8s
    mid = window_metrics(df, s1 + int(5 * FS), s1 + int(13 * FS))
    # first evidence of deeper brake
    iqr = df.iq_ref.to_numpy()
    first_m028 = next((i for i in range(s1, n) if iqr[i] < -0.28), None)
    first_m065 = next((i for i in range(s1, n) if iqr[i] <= -0.65), None)
    first_m115 = next((i for i in range(s1, n) if iqr[i] <= -1.15), None)
    return {
        "label": label,
        "file": fname,
        "cruise_t": float((s1 - o0) / FS),
        "file_dur_s": float(n / FS),
        "iqr_global_min_after_cruise": float(iqr[s1:].min()),
        "first_iqr_lt_m028_s": None
        if first_m028 is None
        else float((first_m028 - o0) / FS),
        "first_iqr_le_m065_s": None
        if first_m065 is None
        else float((first_m065 - o0) / FS),
        "first_iqr_le_m115_s": None
        if first_m115 is None
        else float((first_m115 - o0) / FS),
        "mid_cruise_plus5_to13s": mid,
        "late_soak": late,
    }


def rpm_to_rads(rpm: float) -> float:
    return rpm * (2.0 * np.pi / 60.0)


def bd_from_j(J: float, tau_s: float = 0.25) -> dict:
    """Virtual viscous: B = J/tau [Nm/(rad/s)] → Bd [A/rpm]."""
    B = J / tau_s  # Nm/(rad/s)
    bd_per_rads = B / KT  # A/(rad/s)
    bd_per_rpm = bd_per_rads * rpm_to_rads(1.0)  # A/rpm
    # damping torque at 100 rpm AC amplitude
    t_at_100rpm = B * rpm_to_rads(100.0)
    iq_at_100rpm = t_at_100rpm / KT
    return {
        "J": J,
        "tau_s": tau_s,
        "B_Nm_per_rads": B,
        "Bd_A_per_rpm": bd_per_rpm,
        "Iq_damp_at_100rpm_ac": iq_at_100rpm,
        "vs_firmware_Bd": BD_FW,
        "ratio_fw_over_model": BD_FW / bd_per_rpm if bd_per_rpm > 0 else None,
    }


def estimate_j_from_csv(fname: str) -> dict | None:
    """Rough J from segments with |iq| large and measurable alpha."""
    df = load_csv(fname)
    o0, s1, n = find_cruise(df)
    if s1 is None:
        return None
    spdfb = df.spd_fb.to_numpy()
    iqr = df.iq_ref.to_numpy()
    # 50ms windows
    win = int(0.05 * FS)
    js = []
    for i in range(s1, n - win, win):
        w0, w1 = spdfb[i], spdfb[i + win]
        if not (400 < w0 < 1600 and 400 < w1 < 1600):
            continue
        iq = np.median(iqr[i : i + win])
        if abs(iq) < 0.4 or abs(iq) > 3.5:
            continue
        alpha = rpm_to_rads(w1 - w0) / 0.05  # rad/s^2
        if abs(alpha) < 50:
            continue
        # J * alpha ≈ Kt * iq  (ignore friction)
        J_est = (KT * iq) / alpha
        if 1e-7 < J_est < 5e-4:
            js.append(J_est)
    if len(js) < 20:
        return {"n": len(js), "ok": False}
    js = np.array(js)
    # keep positive and within plausible
    js = js[(js > 0) & (js < 5e-4)]
    return {
        "ok": True,
        "n": int(len(js)),
        "J_median": float(np.median(js)),
        "J_p25": float(np.percentile(js, 25)),
        "J_p75": float(np.percentile(js, 75)),
        "file": fname,
    }


def simulate_limit_cycle(
    iq_min: float,
    Bd: float,
    J: float,
    t_end: float = 10.0,
    w0: float = 1000.0,
    dist_nm: float = 0.02,
    dist_hz: float = 1.4,
) -> dict:
    """Plant + weak PI + HP + iq clip, with sinusoidal disturbance torque."""
    dt = SPEED_TS
    n = int(t_end / dt)
    w = w0
    w_lpf = w0
    integ = 0.3  # start near cruise Iq
    wref = 1000.0
    wc = 2 * np.pi * 0.35
    a_lpf = (wc * dt) / (1.0 + wc * dt)
    ws = np.empty(n)
    iqs = np.empty(n)
    iq_max = 3.5
    for k in range(n):
        t = k * dt
        ew = wref - w
        integ += KI * ew
        integ = float(np.clip(integ, iq_min, iq_max))
        iq_pi = KP * ew + integ
        w_lpf += a_lpf * (w - w_lpf)
        iq_hp = -Bd * (w - w_lpf)
        iq = float(np.clip(iq_pi + iq_hp, iq_min, iq_max))
        t_dist = dist_nm * np.sin(2 * np.pi * dist_hz * t)
        torque = KT * iq - t_dist
        alpha = torque / max(J, 1e-9)
        w = w + (alpha * dt) * (60.0 / (2 * np.pi))
        ws[k] = w
        iqs[k] = iq
    sl = slice(int(3.0 / dt), None)
    return {
        "iq_min": iq_min,
        "Bd": Bd,
        "J": J,
        "dist_nm": dist_nm,
        "std": float(ws[sl].std()),
        "p5_p95": float(np.percentile(ws[sl], 95) - np.percentile(ws[sl], 5)),
        "mean": float(ws[sl].mean()),
        "iq_min_hit_frac": float((iqs[sl] <= iq_min + 1e-3).mean()),
    }


def propose_gates(gold: dict, fail: dict) -> dict:
    g = gold.get("late_soak") or gold.get("mid_cruise_plus5_to13s")
    f = fail.get("late_soak") or fail.get("mid_cruise_plus5_to13s")
    if not g or not g.get("ok") or not f or not f.get("ok"):
        return {}
    # pass if below midpoint on log scale between gold and fail for std
    std_gate = float(np.sqrt(g["std"] * min(f["std"], g["std"] * 8)))
    # more conservative: gold*2.5 but less than fail*0.35
    std_gate = float(min(max(g["std"] * 2.5, 25.0), f["std"] * 0.25))
    ptp_gate = float(min(max(g["p5_p95"] * 2.0, 60.0), f["p5_p95"] * 0.25))
    ew95_gate = float(min(max(g["ew_p95"] * 2.0, 40.0), f["ew_p95"] * 0.35))
    return {
        "method": "between gold(1453-like) and fail(1503); conservative under fail",
        "gold_ref": {"std": g["std"], "p5_p95": g["p5_p95"], "ew_p95": g["ew_p95"]},
        "fail_ref": {"std": f["std"], "p5_p95": f["p5_p95"], "ew_p95": f["ew_p95"]},
        "propose_promote_if": {
            "std_max": std_gate,
            "p5_p95_max": ptp_gate,
            "ew_p95_max": ew95_gate,
            "hold_s": 1.0,
            "note": "All must hold continuously before raising iq_min",
        },
        "propose_freeze_or_pullback_if": {
            "std_above": std_gate * 1.5,
            "p5_p95_above": ptp_gate * 1.5,
        },
    }


def main():
    packs = {k: analyze_pack(k, v) for k, v in PACKS.items()}
    j_est = estimate_j_from_csv(PACKS["1453_iqmin_0p6"])
    bd_table = [bd_from_j(J, tau_s=t) for J in J_CANDIDATES for t in (0.15, 0.25, 0.4)]
    # pick J for sim
    J_use = j_est["J_median"] if j_est and j_est.get("ok") else 2e-5
    sims = []
    for iq_min in (-0.25, -0.6, -0.8, -1.0, -1.2, -2.0):
        for Bd in (0.002, 0.003, 0.004, 0.006, 0.008, 0.010):
            sims.append(
                simulate_limit_cycle(iq_min, Bd, J_use, dist_nm=0.025, dist_hz=1.4)
            )

    best_by_iq = {}
    for iq_min in (-0.25, -0.6, -0.8, -1.0, -1.2, -2.0):
        rows = [s for s in sims if s["iq_min"] == iq_min]
        best = min(rows, key=lambda s: s["std"])
        at_fw = next(s for s in rows if abs(s["Bd"] - BD_FW) < 1e-9)
        best_by_iq[str(iq_min)] = {"best": best, "at_firmware_Bd": at_fw}

    # compare -0.6 vs -1.2 at same Bd=0.002
    cmp_same_bd = {
        "Bd": BD_FW,
        "iq_min_-0.6": next(s for s in sims if s["iq_min"] == -0.6 and s["Bd"] == BD_FW),
        "iq_min_-1.2": next(s for s in sims if s["iq_min"] == -1.2 and s["Bd"] == BD_FW),
    }

    gates = propose_gates(packs["1453_iqmin_0p6"], packs["1503_iqmin_1p2"])

    # recommendation narrative numbers
    gold_late = packs["1453_iqmin_0p6"]["late_soak"]
    fail_late = packs["1503_iqmin_1p2"]["late_soak"]
    rec = {
        "do_not_flash_yet": True,
        "stage": "S4a pass (-0.6); S4b fail (-1.2 empty-shaft pump)",
        "firmware_next_only_after_agreeing": [
            "Lock authority at -0.6 (LOCK_STAGE2=1) until amplitude gate exists",
            "Add promote gate: std/p5-p95/ew_p95 from propose_promote_if",
            "iq_min slew (A/s) instead of step -0.6→-1.2",
            "When raising |iq_min|, raise Bd toward sim best_by_iq (or J/tau model)",
            "Do NOT raise CRUISE_PI_KP/KI until amplitude gate passes at target iq_min",
        ],
        "offline_Bd_note": (
            f"Firmware Bd={BD_FW} A/rpm. Model Bd=J/(tau*Kt)*2pi/60. "
            "If J~2e-5 and tau=0.25s → Bd~0.0012; current HP is same order. "
            "Sim suggests needing larger Bd when iq_min deeper."
        ),
        "gold_vs_fail": {
            "1453_late_std": gold_late.get("std") if gold_late else None,
            "1503_late_std": fail_late.get("std") if fail_late else None,
            "std_ratio_fail_over_gold": (
                fail_late["std"] / gold_late["std"]
                if gold_late and fail_late and gold_late.get("ok") and fail_late.get("ok")
                else None
            ),
        },
    }

    out = {
        "constants": {
            "Kt": KT,
            "Kp_A_per_rpm": KP,
            "Bd_firmware": BD_FW,
            "FS_telem": FS,
        },
        "packs": packs,
        "J_estimate_from_1453": j_est,
        "Bd_model_table": bd_table,
        "sim_J_used": J_use,
        "sim_grid": sims,
        "sim_best_Bd_per_iq_min": best_by_iq,
        "sim_same_Bd_compare": cmp_same_bd,
        "proposed_amplitude_gates": gates,
        "recommendation": rec,
    }
    OUT.write_text(json.dumps(out, indent=2), encoding="utf-8")

    # human summary print
    print("=== CSV late-soak ===")
    for k, p in packs.items():
        late = p.get("late_soak") or {}
        mid = p.get("mid_cruise_plus5_to13s") or {}
        print(
            f"{k}: iqr_min={p.get('iqr_global_min_after_cruise'):.3f} "
            f"late std={late.get('std')} p5-p95={late.get('p5_p95')} "
            f"ew_p95={late.get('ew_p95')} hunt={late.get('hunt_hz')} "
            f"| mid std={mid.get('std')} p5-p95={mid.get('p5_p95')}"
        )
    print("\n=== J estimate ===")
    print(j_est)
    print("\n=== Bd model (tau=0.25) ===")
    for row in bd_table:
        if row["tau_s"] == 0.25:
            print(
                f"J={row['J']:.1e} → Bd={row['Bd_A_per_rpm']:.4f} A/rpm "
                f"(fw={BD_FW}, ratio={row['ratio_fw_over_model']:.2f})"
            )
    print("\n=== Sim same Bd=0.002: -0.6 vs -1.2 ===")
    print(cmp_same_bd["iq_min_-0.6"])
    print(cmp_same_bd["iq_min_-1.2"])
    print("\n=== Sim best Bd by iq_min ===")
    for k, v in best_by_iq.items():
        b, f = v["best"], v["at_firmware_Bd"]
        print(
            f"iq_min={k}: fwBd std={f['std']:.1f} ptp={f['p5_p95']:.1f} | "
            f"best Bd={b['Bd']} std={b['std']:.1f} ptp={b['p5_p95']:.1f}"
        )
    print("\n=== Proposed promote gates ===")
    print(json.dumps(gates, indent=2))
    print("\n=== Recommendation ===")
    print(json.dumps(rec, indent=2))
    print(f"\nWrote {OUT}")


if __name__ == "__main__":
    main()
