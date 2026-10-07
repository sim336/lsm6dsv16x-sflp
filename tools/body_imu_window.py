#!/usr/bin/env python3
"""Do register writes actually land inside one stable SA0 window?

This records the contradiction the bring-up hit: the write was **ACKed, looked like it landed,
and the readback had not changed.** The method was to poll both addresses until one of them
answered 0x70 twice in a row, then read, write, and read again inside that same window, and
finally re-check that the address had not moved mid-experiment.

That window existed because the SA0 strap was flipping about every 9 ms, so the device answered
0x6A for a while and then 0x6B for a while — long enough to fit a write plus a readback. Today's
answer to the contradiction is "the write must be a single joined transaction" (see
`body_imu_write_shape.py`), and the window itself is gone now that SA0 is tied to a rail:
`stable_addr` returns 0x6B immediately.

The script is kept because its method is worth having for any I2C device — "read back
immediately after writing, and confirm nothing else changed the register in between" — not
because the condition it was written for still exists.

Usage (on the board): python3 tools/body_imu_window.py [/dev/i2c-4] [rounds, default 40]
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


def xfer(fd, addr, reg, length, is_read, payload=None):
    abuf = (ctypes.c_uint8 * 1)(reg)
    if is_read:
        dbuf = (ctypes.c_uint8 * length)()
    else:
        dbuf = (ctypes.c_uint8 * length)(*payload)
    msgs = (I2cMsg * 2)()
    msgs[0].addr, msgs[0].flags, msgs[0].len, msgs[0].buf = addr, 0, 1, abuf
    msgs[1].addr = addr
    msgs[1].flags = I2C_M_RD if is_read else 0
    msgs[1].len = length
    msgs[1].buf = dbuf
    io = I2cRdwrIoctlData(ctypes.cast(msgs, ctypes.POINTER(I2cMsg)), 2)
    try:
        fcntl.ioctl(fd, I2C_RDWR, io)
    except OSError:
        return None
    return bytes(dbuf)


def read1(fd, addr, reg):
    d = xfer(fd, addr, reg, 1, True)
    return d[0] if d else None


def write1(fd, addr, reg, val):
    return xfer(fd, addr, reg, 1, False, [val]) is not None


def stable_addr(fd):
    """Poll both addresses until the same one answers twice in a row."""
    for _ in range(400):
        a = read1(fd, 0x6A, 0x0F)
        if a == 0x70:
            if read1(fd, 0x6A, 0x0F) == 0x70:
                return 0x6A
            continue
        b = read1(fd, 0x6B, 0x0F)
        if b == 0x70 and read1(fd, 0x6B, 0x0F) == 0x70:
            return 0x6B
    return None


def main():
    dev = sys.argv[1] if len(sys.argv) > 1 else "/dev/i2c-4"
    rounds = int(sys.argv[2]) if len(sys.argv) > 2 else 40
    fd = os.open(dev, os.O_RDWR)

    landed = corrupted = no_window = 0
    examples = []
    for i in range(rounds):
        addr = stable_addr(fd)
        if addr is None:
            no_window += 1
            continue
        # Everything below must fit in the ~9 ms the strap held still.
        before = read1(fd, addr, 0x10)
        ok_w = write1(fd, addr, 0x10, 0x05)
        after = read1(fd, addr, 0x10)
        held = read1(fd, addr, 0x0F) == 0x70
        if not held or before is None or after is None:
            corrupted += 1
            note = "address moved mid-window"
        elif after == 0x05:
            landed += 1
            note = "write landed"
        else:
            corrupted += 1
            note = "write lost (CTRL1 read %s)" % ("0x%02X" % after)
        if len(examples) < 12:
            examples.append(
                "  0x%02X: before=%s write=%s after=%s  %s"
                % (addr,
                   "0x%02X" % before if before is not None else "NACK",
                   "ok" if ok_w else "NACK",
                   "0x%02X" % after if after is not None else "NACK",
                   note)
            )

    print("== %d attempts at a write + read-back inside one stable window ==" % rounds)
    for line in examples:
        print(line)
    print("  write landed     : %d" % landed)
    print("  write lost/raced : %d" % corrupted)
    print("  no stable window : %d" % no_window)

    print("== CTRL3 (reset default 0x44), 20 samples: a reset loop shows it never holding a "
          "written value ==")
    seen = {}
    for _ in range(20):
        d = xfer(fd, 0x6B, 0x0F, 1, True)
        if d is None:
            continue
        v = read1(fd, 0x6B, 0x12)
        if v is not None:
            seen["0x%02X" % v] = seen.get("0x%02X" % v, 0) + 1
    print("  CTRL3 values seen at 0x6B: %s" % seen)

    os.close(fd)


main()