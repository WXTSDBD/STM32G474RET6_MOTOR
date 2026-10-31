#!/usr/bin/env python3
"""发 OBS SET，看 JustFloat 里 ch2 是否变成 marker=3。"""
from __future__ import annotations

import argparse
import struct
import sys
import time

import serial

TAIL = bytes((0x00, 0x00, 0x80, 0x7F))
CMD = b"OBS SET ia,iq,marker\r\n"


def frames(buf: bytes, k: int = 12):
    fl = 4 + 4 * k + 4
    out = []
    i = 0
    while True:
        j = buf.find(TAIL, i)
        if j < 0:
            break
        start = j - (fl - 4)
        if start >= 0 and j - start == fl - 4:
            fs = struct.unpack("<" + "f" * (1 + k), buf[start:j])
            out.append((fs[0], list(fs[1:])))
        i = j + 4
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("port", default="COM11", nargs="?")
    ap.add_argument("--baud", type=int, default=6_000_000)
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.05, write_timeout=0.5)
    time.sleep(0.5)

    def grab(sec=0.3):
        while ser.in_waiting:
            ser.read(ser.in_waiting)
        time.sleep(sec)
        return frames(ser.read(16384))

    f0 = grab()
    if not f0:
        print("FAIL: no telem")
        return 3
    print("boot", f"seq={f0[-1][0]:.0f}", f"ch0..3={f0[-1][1][:4]}")

    t_end = time.time() + 1.2
    while time.time() < t_end:
        try:
            if ser.in_waiting:
                ser.read(ser.in_waiting)
            ser.write(CMD)
            ser.flush()
        except serial.SerialTimeoutException:
            pass
        time.sleep(0.02)

    time.sleep(0.4)
    f1 = grab(0.35)
    if not f1:
        print("FAIL: no telem after SET")
        return 3
    print("after", f"ch={f1[-1][1][:4]}")
    hits = sum(1 for _, c in f1[-80:] if abs(c[2] - 3.0) < 0.05)
    print(f"marker==3 hits={hits}")
    ser.close()
    ok = hits >= 5
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
