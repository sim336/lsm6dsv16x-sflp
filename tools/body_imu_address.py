#!/usr/bin/env python3
"""Is the LSM6DSV16X answering at two addresses?

Reads WHO_AM_I once at 0x6A and once at 0x6B, back to back, thousands of times, and classifies
each round trip: both answered, only 0x6A, only 0x6B, or neither.

Only-0x6A plus only-0x6B, with almost nothing in the "both" bucket, is the signature of a single
device whose SA0 strap is moving: SA0 high answers 0x6B, SA0 low answers 0x6A, and one device
cannot do both at the same instant. Anything else — both answering at the same instant — would
mean two chips.

Historical note: this was written while SA0 was floating, and it is what identified that fault.
With SA0 tied to a rail it is history: the address is fixed at 0x6B. The method is kept because
it is still the right one for the question — if only_6a and only_6b ever fill up again with
"both" near zero, SA0 is moving again, and it is one device, not two.

Usage (on the board): python3 tools/body_imu_address.py [/dev/i2c-4] [rounds, default 3000]
"""
import ctypes
import fcntl
import os
import sys
import time

I2C_RDWR = 0x0707
I2C_M_RD = 0x0001


class I2cMsg(ctypes.Structure):
    _fields_ = [
        ("addr", ctypes.c_uint16),
        ("flags", ctypes.c_uint16),
        ("len", ctypes.c_uint16),
        ("buf", ctypes.POINTER(ctypes.c_uint8)),
    ]


class I2cRdwrIoctlData(ctypes.Structure):
    _fields_ = [
        ("msgs", ctypes.POINTER(I2cMsg)),
        ("nmsgs", ctypes.c_uint32),
    ]


def who(fd, addr, reg=0x0F):
    """One transaction, one try. Returns the byte, or None if it NACKed."""
    abuf = (ctypes.c_uint8 * 1)(reg)
    dbuf = (ctypes.c_uint8 * 1)()
    msgs = (I2cMsg * 2)()
    msgs[0].addr, msgs[0].flags, msgs[0].len, msgs[0].buf = addr, 0, 1, abuf
    msgs[1].addr, msgs[1].flags, msgs[1].len, msgs[1].buf = addr, I2C_M_RD, 1, dbuf
    io = I2cRdwrIoctlData(ctypes.cast(msgs, ctypes.POINTER(I2cMsg)), 2)
    try:
        fcntl.ioctl(fd, I2C_RDWR, io)
    except OSError:
        return None
    return dbuf[0]


def main():
    dev = sys.argv[1] if len(sys.argv) > 1 else "/dev/i2c-4"
    rounds = int(sys.argv[2]) if len(sys.argv) > 2 else 3000
    fd = os.open(dev, os.O_RDWR)

    tally = {"both": 0, "only_6a": 0, "only_6b": 0, "neither": 0}
    states = []          # (t, symbol)
    a_vals, b_vals = set(), set()

    t0 = time.perf_counter()
    for _ in range(rounds):
        t = time.perf_counter() - t0
        a = who(fd, 0x6A)
        b = who(fd, 0x6B)
        if a is not None:
            a_vals.add(a)
        if b is not None:
            b_vals.add(b)
        if a is not None and b is not None:
            key, sym = "both", "X"
        elif a is not None:
            key, sym = "only_6a", "A"
        elif b is not None:
            key, sym = "only_6b", "B"
        else:
            key, sym = "neither", "."
        tally[key] += 1
        if not states or states[-1][1] != sym:
            states.append((t, sym))
    total = time.perf_counter() - t0

    print("== %d rounds in %.2f s (%d transactions) ==" % (rounds, total, rounds * 2))
    for key in ("only_6a", "only_6b", "both", "neither"):
        print("  %-8s %6d  %5.1f%%" % (key, tally[key], 100.0 * tally[key] / rounds))
    print("  WHO_AM_I seen at 0x6A: %s" % sorted("0x%02X" % v for v in a_vals))
    print("  WHO_AM_I seen at 0x6B: %s" % sorted("0x%02X" % v for v in b_vals))

    print("== transitions (first 60), A=only 0x6A  B=only 0x6B  X=both  .=neither ==")
    line = "".join(s for _, s in states[:120])
    print("  " + line)
    times = [t for t, _ in states]
    if len(times) > 2:
        gaps = [times[i + 1] - times[i] for i in range(len(times) - 1)]
        gaps.sort()
        mid = gaps[len(gaps) // 2]
        print("  transitions seen: %d over %.2f s -> %d flips/s" % (len(times), total, len(times) / total))
        print("  median time between transitions: %.2f ms (=> half-period %.2f ms, ~%.0f Hz)"
              % (mid * 1e3, mid * 1e3, 1.0 / (2 * mid) if mid else 0))
        print("  min %.2f ms / max %.2f ms" % (gaps[0] * 1e3, gaps[-1] * 1e3))

    os.close(fd)


main()