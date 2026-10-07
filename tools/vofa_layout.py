"""VOFA JustFloat 通道布局：统一 12ch（M1_VOFA_UNIFIED_12CH=1）与 legacy 6ch 自动识别。"""
from __future__ import annotations

from pathlib import Path

import numpy as np

FS_DEFAULT = 10000.0

# M1_VOFA_PLL_CH8_11=1（bringup + PLL 试用）
UNIFIED_12_PLL = {
    "ia": 0,
    "ib": 1,
    "ic": 2,
    "id": 3,
    "iq": 4,
    "theta": 5,
    "ud_out": 6,
    "uq_pi": 7,
    "pll_omega_rpm": 8,
    "pll_omega_diff_rpm": 9,
    "pll_theta_err": 10,
    "pll_omega_err_rpm": 11,
    "id_ref": None,
    "iq_ref": None,
    "duty_dev": None,
    "open_seq": None,
}

# T-1 签收档（M1_VOFA_SIGNOFF_CH / TELEM_LAYOUT_SIGNOFF=1）
# 数据列在剥掉帧头 layout_id 之后：ia ib ic iq θ_fb θ_mech θ_ref θ_err ω_pll ω_ref iq_ref mark
UNIFIED_12_SIGNOFF = {
    "ia": 0,
    "ib": 1,
    "ic": 2,
    "iq": 3,
    "theta_fb": 4,
    "theta_mech": 5,
    "theta_ref": 6,
    "theta_err": 7,
    "outer_theta_err_rad": 7,
    "pll_omega_rpm": 8,
    "omega_ref": 9,
    "iq_ref": 10,
    "outer_iq_ref": 10,
    "open_seq": 11,
    "id": None,
    "id_ref": None,
    "duty_dev": None,
}

# M1_VOFA_MIT_CH8_11=1（NORMAL_MIT_ONLY 阻抗验收）
UNIFIED_12_MIT = {
    "ia": 0,
    "ib": 1,
    "ic": 2,
    "id": 3,
    "iq": 4,
    "theta": 5,
    "ud_out": 6,
    "uq_out": 7,
    "uq_pi": 7,
    "pll_omega_rpm": 8,
    "theta_err_rad": 9,
    "outer_theta_err_rad": 9,
    "iq_ref": 10,
    "outer_iq_ref": 10,
    "open_seq": 11,
    "id_ref": None,
    "duty_dev": None,
}

# M1_VOFA_IDENT_DUTY_12CH=1（辨识/Pass0/VASI，2026-07 起默认）
UNIFIED_12_IDENT_DUTY = {
    "ia": 0,
    "ib": 1,
    "ic": 2,
    "id": 3,
    "iq": 4,
    "theta": 5,
    "vd_est": 6,
    "vq_est": 7,
    "duty_ta": 8,
    "duty_tb": 9,
    "duty_tc": 10,
    "open_seq": 11,
    "ud_out": 6,
    "uq_out": 7,
    "id_ref": None,
    "iq_ref": None,
    "duty_dev": None,
}

# M1_VOFA_UNIFIED_12CH=1（bringup 旧布局 / legacy）
UNIFIED_12 = {
    "ia": 0,
    "ib": 1,
    "ic": 2,
    "id": 3,
    "iq": 4,
    "theta": 5,
    "ud_out": 6,
    "uq_pi": 7,
    "id_ref": 8,
    "iq_ref": 9,
    "duty_dev": 10,
    "open_seq": 11,
}

# legacy ident 6ch：ch3=Iq ch4=Iq_ref；ch5=Uq_pi 或 BODE_OFF_ONLY 遥测 bode_f_hz
LEGACY_IDENT_6 = {
    "ia": 0,
    "ib": 1,
    "ic": 2,
    "id": None,
    "iq": 3,
    "theta": 5,
    "ud_out": None,
    "uq_pi": 5,
    "bode_f_hz": None,
    "id_ref": None,
    "iq_ref": 4,
    "duty_dev": None,
    "open_seq": None,
}


def is_bode_f_hz_ch5(ch5: np.ndarray) -> bool:
    """True when ch5 carries ident_module_bode_freq_hz() (stepwise 10..1500+ Hz)."""
    if ch5 is None or len(ch5) < 1000:
        return False
    mx = float(np.nanmax(ch5))
    if mx < 200.0:
        return False
    active = ch5[ch5 > 5.0]
    if active.size < 500:
        return False
    uniq = np.unique(np.round(active, 0))
    return uniq.size >= 20 and mx > 500.0

# legacy Id cal 6ch：ch3=Ud_pi ch4=Id_ref
LEGACY_IDCAL_6 = {
    "ia": 0,
    "ib": 1,
    "ic": 2,
    "id": None,
    "iq": None,
    "theta": 5,
    "ud_out": None,
    "uq_pi": 3,
    "id_ref": 4,
    "iq_ref": None,
    "duty_dev": None,
    "open_seq": None,
}


def load_csv_raw(path: Path | str) -> tuple[np.ndarray, ...]:
    """Read all CSV columns as float arrays (delegates to vofa_io)."""
    from vofa_io import load_csv_raw as _load

    return _load(path)


def strip_layout_id_cols(cols: tuple[np.ndarray, ...]) -> tuple[np.ndarray, ...]:
    """若首列是 T-1 layout_id（0..31 整数），剥掉后返回数据列。"""
    if len(cols) < 13:
        return cols
    c0 = cols[0]
    lid = np.rint(c0)
    if (
        np.all(np.abs(c0 - lid) < 0.25)
        and np.all((lid >= 0) & (lid <= 31))
    ):
        return cols[1:13]
    return cols


def detect_layout(cols: tuple[np.ndarray, ...]) -> str:
    """返回 'unified12_signoff' | 'unified12_mit' | ...。"""
    # T-1：帧头 layout_id==1 → 签收（优先于启发式）
    if len(cols) >= 13:
        c0 = cols[0]
        lid = np.rint(c0)
        if (
            np.all(np.abs(c0 - lid) < 0.25)
            and np.all((lid >= 0) & (lid <= 31))
            and float(np.nanmedian(lid)) == 1.0
        ):
            return "unified12_signoff"

    cols = strip_layout_id_cols(cols)
    n_ch = len(cols)
    if n_ch >= 12:
        ch6 = cols[6]
        ch7 = cols[7]
        ch8 = cols[8]
        ch9 = cols[9]
        ch10 = cols[10]
        ch11 = cols[11]
        # ident duty：Ta/Tb/Tc ∈ [0,1]，Vd_est 在 ±Vbus/√3；open_seq 为 0–300 整数
        ta_ok = float(np.nanmax(ch8)) <= 1.05 and float(np.nanmin(ch8)) >= -0.05
        tb_ok = float(np.nanmax(ch9)) <= 1.05 and float(np.nanmin(ch9)) >= -0.05
        tc_ok = float(np.nanmax(ch10)) <= 1.05 and float(np.nanmin(ch10)) >= -0.05
        vd_rng = float(np.nanmax(np.abs(ch6)))
        open_seq_like = (
            float(np.nanmax(ch11)) <= 320.0
            and float(np.nanmean(np.abs(ch11 - np.round(ch11))) < 0.02)
            and float(np.nanmax(ch11)) >= 1.0
        )
        if ta_ok and tb_ok and (tc_ok or open_seq_like) and vd_rng < 20.0 and open_seq_like:
            return "unified12_ident_duty"
        # MIT 验收：ch11=open_seq 130..156，ch8=ω_pll[rpm]（|ω| 通常 < 400）
        seq_round = np.round(ch11).astype(int)
        mit_seq = open_seq_like and float(np.nanmin(seq_round)) >= 128.0 and float(
            np.nanmax(seq_round)
        ) <= 158.0
        if mit_seq and float(np.nanmax(np.abs(ch8))) < 500.0:
            return "unified12_mit"
        # PLL ch8–11：ω 在 rpm 量级，θ_err 在 ±π，open_seq 不在 ch11
        if (
            float(np.nanmax(np.abs(ch8))) > 5.0
            and float(np.nanmax(np.abs(ch9))) > 5.0
            and float(np.nanmax(np.abs(ch10))) < 4.0
            and float(np.nanmax(np.abs(ch11))) < 2000.0
            and float(np.nanmax(np.abs(ch11))) > 0.01
            and float(np.nanstd(ch8)) > 1.0
            and float(np.nanmax(np.abs(cols[9] if len(cols) > 9 else ch8))) < 5000.0
        ):
            # ch11=open_seq 时多为 0–300 整数阶跃；PLL Δω 连续且一般 |Δω|≪rpm
            open_seq_like = float(np.nanmax(ch11)) > 40.0 and float(np.nanstd(ch11)) > 5.0
            pll_like = float(np.nanstd(ch11)) < float(np.nanstd(ch8)) * 0.5
            if pll_like and not open_seq_like:
                return "unified12_pll"
        # 仅前 6 路有数据（12ch 容器里的 legacy ident）
        if float(np.nanmax(np.abs(cols[9]))) < 0.05 and float(np.nanmax(np.abs(ch8))) < 0.05:
            if float(np.nanmax(np.abs(cols[4]))) > 0.08:
                return "legacy_ident6"
        if float(np.nanmax(np.abs(cols[9]))) > 0.08:
            return "unified12"
        ch8_active = float(np.mean(np.abs(ch8) > 0.05))
        if float(np.nanmax(ch8)) > 1.0 and ch8_active > 0.002:
            return "unified12"
    if n_ch >= 5:
        ch3, ch4 = cols[3], cols[4]
        if float(np.nanmax(np.abs(ch4))) > 0.08:
            if n_ch < 10 or float(np.nanmax(np.abs(cols[9]))) < 0.05:
                return "legacy_ident6"
        if float(np.nanmax(ch4)) > 0.5 and float(np.nanstd(ch3)) < float(np.nanstd(ch4)) * 0.5:
            return "legacy_idcal6"
    return "legacy_idcal6"


def pick(cols: tuple[np.ndarray, ...], layout: str, key: str) -> np.ndarray:
    cols = strip_layout_id_cols(cols)
    maps = {
        "unified12_signoff": UNIFIED_12_SIGNOFF,
        "unified12_mit": UNIFIED_12_MIT,
        "unified12_ident_duty": UNIFIED_12_IDENT_DUTY,
        "unified12_pll": UNIFIED_12_PLL,
        "unified12": UNIFIED_12,
        "legacy_ident6": LEGACY_IDENT_6,
        "legacy_idcal6": LEGACY_IDCAL_6,
    }
    if layout == "legacy_ident6" and len(cols) >= 6:
        ch5_bode = is_bode_f_hz_ch5(cols[5])
        if key == "bode_f_hz":
            return cols[5] if ch5_bode else np.zeros(len(cols[0]))
        if key == "uq_pi":
            return np.zeros(len(cols[0])) if ch5_bode else cols[5]
    idx = maps[layout].get(key)
    if idx is None:
        if layout == "legacy_ident6" and key == "id_ref":
            return cols[4]
        return np.zeros(len(cols[0]))
    return cols[idx]


def load_csv(
    path: Path | str,
    *,
    layout: str = "auto",
    fs: float | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    兼容旧接口：返回 (ia, ib, ic, iq_fb, iq_ref, uq_pi)。
    unified12：iq=ch4, iq_ref=ch9, uq_pi=ch7。
    """
    cols = load_csv_raw(path)
    if layout == "auto":
        layout = detect_layout(cols)
    ia = pick(cols, layout, "ia")
    ib = pick(cols, layout, "ib")
    ic = pick(cols, layout, "ic")
    iq_fb = pick(cols, layout, "iq")
    iq_ref = pick(cols, layout, "iq_ref")
    uq_pi = pick(cols, layout, "uq_pi")
    return ia, ib, ic, iq_fb, iq_ref, uq_pi


def load_csv_full(
    path: Path | str,
    *,
    layout: str = "auto",
) -> dict[str, np.ndarray]:
    cols = load_csv_raw(path)
    if layout == "auto":
        layout = detect_layout(cols)
    keys = set(UNIFIED_12_IDENT_DUTY) | set(UNIFIED_12_PLL) | set(UNIFIED_12) | set(LEGACY_IDENT_6) | set(LEGACY_IDCAL_6)
    return {k: pick(cols, layout, k) for k in keys if pick(cols, layout, k) is not None}
