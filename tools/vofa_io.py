"""Fast VOFA CSV I/O and shared signal helpers for offline analysis."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from vofa_layout import (
    FS_DEFAULT,
    LEGACY_IDCAL_6,
    LEGACY_IDENT_6,
    UNIFIED_12,
    UNIFIED_12_MIT,
    UNIFIED_12_SIGNOFF,
    detect_layout,
    pick,
)

# Column name order when CSV has no header (rare)
_FALLBACK_NAMES = tuple(UNIFIED_12.keys())


@dataclass(frozen=True)
class VofaFrame:
    """One VOFA recording with named channels and layout metadata."""

    path: Path
    layout: str
    fs: float
    channels: dict[str, np.ndarray]

    @property
    def n(self) -> int:
        return len(next(iter(self.channels.values())))

    @property
    def t(self) -> np.ndarray:
        return np.arange(self.n, dtype=np.float64) / self.fs

    def ch(self, key: str) -> np.ndarray:
        if key not in self.channels:
            raise KeyError(key)
        return self.channels[key]

    def has(self, key: str) -> bool:
        ch = self.channels.get(key)
        if ch is None:
            return False
        return bool(np.nanmax(np.abs(ch)) > 1e-9)


def read_csv_matrix(path: Path | str) -> tuple[list[str], np.ndarray]:
    """
    Read CSV → (column_names, float matrix shape (N, C)).
    Prefer pandas; fall back to numpy.loadtxt.
    """
    path = Path(path)
    try:
        import pandas as pd

        df = pd.read_csv(path, sep=",", engine="c")
        names = [str(c) for c in df.columns]
        arr = df.to_numpy(dtype=np.float64, copy=False)
        if arr.ndim == 1:
            arr = arr.reshape(-1, 1)
        return names, arr
    except ImportError:
        pass

    try:
        arr = np.loadtxt(path, delimiter=",", skiprows=1, dtype=np.float64)
    except ValueError:
        arr = np.loadtxt(path, delimiter=",", dtype=np.float64)
    if arr.ndim == 1:
        arr = arr.reshape(-1, 1)
    with path.open(encoding="utf-8", errors="replace") as f:
        header = f.readline().strip()
    names = [c.strip() for c in header.split(",")]
    if len(names) != arr.shape[1]:
        names = [f"ch{i}" for i in range(arr.shape[1])]
    return names, arr


def cols_from_matrix(names: list[str], arr: np.ndarray) -> tuple[np.ndarray, ...]:
    """Matrix columns → tuple of 1-D arrays (vofa_layout.detect_layout 接口)."""
    return tuple(arr[:, i] for i in range(arr.shape[1]))


def load_vofa(path: Path | str, *, fs: float = FS_DEFAULT) -> VofaFrame:
    """Load CSV once; return named channels + auto layout."""
    path = Path(path)
    names, mat = read_csv_matrix(path)
    cols = cols_from_matrix(names, mat)
    layout = detect_layout(cols)
    keys = (
        set(UNIFIED_12)
        | set(UNIFIED_12_MIT)
        | set(UNIFIED_12_SIGNOFF)
        | set(LEGACY_IDENT_6)
        | set(LEGACY_IDCAL_6)
        | {"bode_f_hz"}
    )
    channels: dict[str, np.ndarray] = {}
    for key in keys:
        arr = pick(cols, layout, key)
        if arr is not None and len(arr) == len(cols[0]):
            channels[key] = np.asarray(arr, dtype=np.float64)
    return VofaFrame(path=path, layout=layout, fs=fs, channels=channels)


def load_csv_raw(path: Path | str) -> tuple[np.ndarray, ...]:
    """Backward-compatible tuple API for vofa_layout.load_csv_raw."""
    _, mat = read_csv_matrix(path)
    return cols_from_matrix([f"ch{i}" for i in range(mat.shape[1])], mat)


def unwrap(theta: np.ndarray) -> np.ndarray:
    """Phase unwrap (rad); vectorized."""
    return np.unwrap(np.asarray(theta, dtype=np.float64))


def fundamental(
    x: np.ndarray,
    f_hz: float,
    fs: float,
    *,
    bias: float | None = None,
) -> complex:
    """Single-frequency DFT coefficient of the AC component at f_hz."""
    x = np.asarray(x, dtype=np.float64)
    n = len(x)
    if n < 4:
        return 0.0 + 0.0j
    dc = float(np.mean(x)) if bias is None else bias
    ac = x - dc
    tt = np.arange(n, dtype=np.float64) / fs
    w = 2.0 * np.pi * f_hz
    c = np.cos(w * tt)
    s = np.sin(w * tt)
    return complex(np.dot(ac, c), -np.dot(ac, s)) * (2.0 / n)


def find_edges(
    ref: np.ndarray,
    fs: float,
    *,
    threshold: float = 0.05,
) -> list[tuple[int, str, float, float]]:
    """Vectorized step-edge finder on reference signal."""
    ref = np.asarray(ref, dtype=np.float64)
    dref = np.diff(ref)
    idx = np.where(np.abs(dref) > threshold)[0] + 1
    out: list[tuple[int, str, float, float]] = []
    for i in idx:
        fv, tv = float(ref[i - 1]), float(ref[i])
        out.append((int(i), _classify_edge(fv, tv), fv, tv))
    return out


def _classify_edge(fv: float, tv: float) -> str:
    if abs(fv) < 0.05 and tv > 0.05:
        return f"0->{tv:.1f}"
    if abs(tv) < 0.05 and fv > 0.05:
        return f"{fv:.1f}->0"
    if tv > fv + 0.05:
        return f"{fv:.1f}->{tv:.1f}"
    if tv < fv - 0.05:
        return f"{fv:.1f}->{tv:.1f}"
    return f"{fv:.2f}->{tv:.2f}"


def open_seq_runs(
    open_seq: np.ndarray,
    codes: tuple[int, ...],
    *,
    min_len: int = 100,
) -> list[tuple[int, int, int]]:
    """Contiguous index runs where round(open_seq) equals each code (in order)."""
    seq = np.round(np.asarray(open_seq, dtype=np.float64)).astype(np.int32)
    runs: list[tuple[int, int, int]] = []
    for code in codes:
        mask = seq == code
        if not np.any(mask):
            continue
        changes = np.diff(mask.astype(np.int8))
        starts = np.where(changes == 1)[0] + 1
        ends = np.where(changes == -1)[0] + 1
        if mask[0]:
            starts = np.concatenate([[0], starts])
        if mask[-1]:
            ends = np.concatenate([ends, [len(seq)]])
        for a, b in zip(starts, ends):
            if b - a >= min_len:
                runs.append((int(a), int(b), code))
    runs.sort(key=lambda x: x[0])
    return runs


def ident_bode_rounds(
    open_seq: np.ndarray,
    *,
    n_rounds: int = 4,
    base_code: int = 62,
) -> list[tuple[int, int, int]]:
    """Return [(start, end, round_idx), ...] for Bode open_seq 62..65."""
    codes = tuple(base_code + r for r in range(n_rounds))
    raw = open_seq_runs(open_seq, codes, min_len=int(0.5 * FS_DEFAULT))
    by_code: dict[int, tuple[int, int]] = {}
    for a, b, code in raw:
        if code not in by_code or (b - a) > (by_code[code][1] - by_code[code][0]):
            by_code[code] = (a, b)
    out: list[tuple[int, int, int]] = []
    for r in range(n_rounds):
        code = base_code + r
        if code in by_code:
            a, b = by_code[code]
            out.append((a, b, r))
    return out


def ident_step_rounds(
    open_seq: np.ndarray,
    *,
    n_rounds: int = 8,
    base_code: int = 61,
) -> list[tuple[int, int, int]]:
    """Return [(start, end, round_idx), ...] for step open_seq 61..68."""
    codes = tuple(base_code + r for r in range(n_rounds))
    raw = open_seq_runs(open_seq, codes, min_len=int(0.2 * FS_DEFAULT))
    by_code: dict[int, tuple[int, int]] = {}
    for a, b, code in raw:
        if code not in by_code or (b - a) > (by_code[code][1] - by_code[code][0]):
            by_code[code] = (a, b)
    out: list[tuple[int, int, int]] = []
    for r in range(n_rounds):
        code = base_code + r
        if code in by_code:
            a, b = by_code[code]
            out.append((a, b, r))
    return out


def ident_step_profile(round_idx: int, *, rounds_per_profile: int = 2) -> str:
    """Match ident_flow: pair=round//rounds_per_profile; OFF if pair%2==0 else LUT."""
    pair = round_idx // max(1, rounds_per_profile)
    return "OFF" if (pair % 2 == 0) else "LUT"


def align_bin_start(
    ref: np.ndarray,
    seg_start: int,
    seg_end: int,
    f_hz: float,
    fs: float,
    *,
    bias: float,
    cycles: float,
    search_s: float = 0.35,
) -> int:
    """Vectorized search for first-bin phase alignment within the segment head."""
    from numpy.lib.stride_tricks import sliding_window_view

    ref = np.asarray(ref, dtype=np.float64)
    nw = max(4, int((cycles / f_hz) * fs + 0.5))
    span = min(int(search_s * fs), max(0, seg_end - seg_start - nw))
    if span <= 0:
        return seg_start
    i_lo = seg_start
    i_hi = seg_start + span
    tt = np.arange(nw, dtype=np.float64) / fs
    w = 2.0 * np.pi * f_hz
    c = np.cos(w * tt)
    s = np.sin(w * tt)
    win = sliding_window_view(ref[i_lo : i_hi + nw], nw)
    ac = win - bias
    mag = np.abs(np.dot(ac, c) - 1j * np.dot(ac, s)) * (2.0 / nw)
    return i_lo + int(np.argmax(mag))
