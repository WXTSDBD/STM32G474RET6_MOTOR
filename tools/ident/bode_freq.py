"""Bode sweep frequency plans (match ident_module.c / gen_bode_freq_table.py)."""
from __future__ import annotations

F0, F1 = 10.0, 5000.0
RATIO = 1.15
F_SPLIT = 500.0
RATIO_HI = 1.06
DEFAULT_CYCLES = 20.0
DEFAULT_CYCLES_HI = 50.0
DEFAULT_T_OBS_HI_S = 0.1  # 100 ms for f >= F_SPLIT; 0 = use fixed CYCLES_HI
DEFAULT_FS = 20000.0
CTRL_TS_S = 50e-6


def ticks_from_s(s: float) -> int:
    """Match ident_module.c ident_ticks_from_s()."""
    return max(1, int(s / CTRL_TS_S + 0.5))


def freq_list_uniform(f0: float = F0, f1: float = F1, ratio: float = RATIO) -> list[float]:
    f, out = f0, []
    while True:
        out.append(f)
        if f >= f1:
            break
        f *= ratio
    return out


def freq_list_d(
    f0: float = F0,
    f1: float = F1,
    *,
    split: float = F_SPLIT,
    ratio_lo: float = RATIO,
    ratio_hi: float = RATIO_HI,
) -> list[float]:
    freqs = [f0]
    while freqs[-1] < f1:
        r = ratio_lo if freqs[-1] < split else ratio_hi
        freqs.append(freqs[-1] * r)
    return freqs


def cycles_for_freq(
    f_hz: float,
    *,
    split: float = F_SPLIT,
    cycles_lo: float = DEFAULT_CYCLES,
    cycles_hi: float = DEFAULT_CYCLES_HI,
    t_obs_hi_s: float = DEFAULT_T_OBS_HI_S,
) -> float:
    if f_hz >= split:
        if t_obs_hi_s > 0.0:
            return t_obs_hi_s * f_hz
        return cycles_hi
    return cycles_lo


FREQS_D = freq_list_d()
FREQS_UNIFORM37 = freq_list_uniform()
FREQS_LEGACY800 = FREQS_UNIFORM37[:32]

BODE_FREQ_PLANS: list[tuple[str, list[float], float]] = [
    ("D-48-split", FREQS_D, DEFAULT_CYCLES),
    ("D-57", freq_list_d(split=200.0), 20.0),
    ("uni-37", FREQS_UNIFORM37, 16.0),
    ("uni-32", FREQS_LEGACY800, 16.0),
    ("uni-32-8cyc", FREQS_LEGACY800, 8.0),
]

FREQS_FULL = FREQS_D
FREQS_LEGACY = FREQS_LEGACY800


def sweep_duration_s(
    freqs: list[float],
    cycles: float | None = None,
    *,
    split: float = F_SPLIT,
    cycles_lo: float = DEFAULT_CYCLES,
    cycles_hi: float = DEFAULT_CYCLES_HI,
    t_obs_hi_s: float = DEFAULT_T_OBS_HI_S,
) -> float:
    if cycles is not None:
        return sum(cycles / f for f in freqs)
    return sum(
        cycles_for_freq(
            f, split=split, cycles_lo=cycles_lo, cycles_hi=cycles_hi, t_obs_hi_s=t_obs_hi_s
        )
        / f
        for f in freqs
    )


def sweep_ticks(
    freqs: list[float],
    fs: float = DEFAULT_FS,
    cycles: float | None = None,
    *,
    split: float = F_SPLIT,
    cycles_lo: float = DEFAULT_CYCLES,
    cycles_hi: float = DEFAULT_CYCLES_HI,
    t_obs_hi_s: float = DEFAULT_T_OBS_HI_S,
) -> int:
    """Exact sample count for one full Bode sweep (matches ident_module.c)."""
    total = 0
    for f in freqs:
        cyc = cycles if cycles is not None else cycles_for_freq(
            f, split=split, cycles_lo=cycles_lo, cycles_hi=cycles_hi, t_obs_hi_s=t_obs_hi_s
        )
        total += ticks_from_s(cyc / f)
    return total


def ticks_for_freq(
    f_hz: float,
    fs: float = DEFAULT_FS,
    cycles: float | None = None,
    *,
    split: float = F_SPLIT,
    cycles_lo: float = DEFAULT_CYCLES,
    cycles_hi: float = DEFAULT_CYCLES_HI,
    t_obs_hi_s: float = DEFAULT_T_OBS_HI_S,
) -> int:
    """Sample count for one frequency bin (ident_module ident_bode_ticks_per_freq)."""
    if f_hz < 1.0:
        return ticks_from_s(0.5)
    cyc = cycles if cycles is not None else cycles_for_freq(
        f_hz, split=split, cycles_lo=cycles_lo, cycles_hi=cycles_hi, t_obs_hi_s=t_obs_hi_s
    )
    return ticks_from_s(cyc / f_hz)


def infer_bode_plan(
    avail_s: float,
    *,
    cycles: float | None = None,
    bode_rounds: int,
    split: float = F_SPLIT,
    cycles_lo: float = DEFAULT_CYCLES,
    cycles_hi: float = DEFAULT_CYCLES_HI,
) -> tuple[int, list[float], float | None]:
    def fits(nr: int, freqs: list[float], uniform_cycles: float | None) -> bool:
        sw = sweep_duration_s(
            freqs,
            uniform_cycles,
            split=split,
            cycles_lo=cycles_lo,
            cycles_hi=cycles_hi,
        )
        return nr * sw <= avail_s + 0.15

    candidates: list[tuple[list[float], float | None]] = []
    seen: set[tuple[int, float | None]] = set()
    for _name, freqs, plan_cycles in BODE_FREQ_PLANS:
        cyc = cycles if _name.startswith("D-48") else plan_cycles
        key = (len(freqs), cyc)
        if key in seen:
            continue
        seen.add(key)
        candidates.append((freqs, cyc if cycles is None else cycles))
    if (len(FREQS_D), cycles) not in seen:
        candidates.insert(0, (FREQS_D, cycles))

    for freqs, cyc in candidates:
        if fits(bode_rounds, freqs, cyc):
            return bode_rounds, freqs, cyc
    for nr in (2, 1):
        for freqs, cyc in candidates:
            if fits(nr, freqs, cyc):
                return nr, freqs, cyc
    freqs, cyc = candidates[0]
    return 1, freqs, cyc


# --- PI 电流环理论频响（motor_params_m1.h N5065 默认） ---
THEORY_LQ_H = 87e-6
THEORY_LD_H = 59e-6
THEORY_RS_OHM = 0.115
THEORY_FC_HZ = 1000.0


def theory_pi_cl_gain(
    f_hz: float,
    *,
    Lq: float = THEORY_LQ_H,
    Rs: float = THEORY_RS_OHM,
    fc_hz: float = THEORY_FC_HZ,
    Ts: float = CTRL_TS_S,
    zoh: bool = False,
) -> float:
    """|Iq/Iq_ref| for PI + R-L plant, Ki_cont = Rs*ωc."""
    import math

    if f_hz < 1e-6:
        return 1.0
    wc = 2.0 * math.pi * fc_hz
    Kp = Lq * wc
    Ki = Rs * wc
    w = 2.0 * math.pi * f_hz
    s = 1j * w
    Gc = Kp + Ki / s
    if zoh:
        Gc *= complex(math.cos(-w * Ts), math.sin(-w * Ts))
    Gp = 1.0 / (Lq * s + Rs)
    H = Gc * Gp / (1.0 + Gc * Gp)
    return float(abs(H))


def theory_f3db_hz(
    *,
    Lq: float = THEORY_LQ_H,
    Rs: float = THEORY_RS_OHM,
    fc_hz: float = THEORY_FC_HZ,
    Ts: float = CTRL_TS_S,
    f_lo: float = 10.0,
    f_hi: float = 5000.0,
) -> float:
    """−3 dB frequency of theory PI closed loop (log scan)."""
    import math

    g0 = theory_pi_cl_gain(f_lo, Lq=Lq, Rs=Rs, fc_hz=fc_hz, Ts=Ts)
    target = g0 * 0.707
    freqs = [f_lo * (f_hi / f_lo) ** (i / 799.0) for i in range(800)]
    for f in freqs:
        if theory_pi_cl_gain(f, Lq=Lq, Rs=Rs, fc_hz=fc_hz, Ts=Ts) < target:
            return f
    return f_hi
