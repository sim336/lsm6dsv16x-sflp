#!/usr/bin/env python3
"""Where does an ACKed write actually go?

Dumps registers 0x00..0x7F, writes four configuration registers, then dumps the whole map again
and prints the difference. A write that lands in a different register bank, or at a different
address than the one asked for, shows up as a diff somewhere unexpected; a write that is accepted
on the wire and then dropped shows up as no diff at all.

Note the values written below are **this probe's own** choices (CTRL6 = 0x04 is ±2000 dps), not
the driver's (which must write ±500 dps — see `body_imu_probe.py`). This is asking "does a write
land anywhere", not configuring a set of parameters that could be used.

Usage (on the board): python3 tools/body_imu_regmap.py [/dev/i2c-4] [0x6B]
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


class Bus:
    def __init__(self, dev, addr):
        self.fd = os.open(dev, os.O_RDWR)
        self.addr = addr
        self.errs = 0

    def xfer(self, reg, length, is_read, payload=None):
        abuf = (ctypes.c_uint8 * 1)(reg)
        if is_read:
            dbuf = (ctypes.c_uint8 * length)()
        else:
            dbuf = (ctypes.c_uint8 * length)(*payload)
        msgs = (I2cMsg * 2)()
        msgs[0].addr, msgs[0].flags, msgs[0].len, msgs[0].buf = self.addr, 0, 1, abuf
        msgs[1].addr = self.addr
        msgs[1].flags = I2C_M_RD if is_read else 0
        msgs[1].len = length
        msgs[1].buf = dbuf
        io = I2cRdwrIoctlData(ctypes.cast(msgs, ctypes.POINTER(I2cMsg)), 2)
        try:
            fcntl.ioctl(self.fd, I2C_RDWR, io)
        except OSError:
            self.errs += 1
            return None
        return bytes(dbuf)

    def rd(self, reg, try_n=3):
        for _ in range(try_n):
            d = self.xfer(reg, 1, True)
            if d is not None:
                return d[0]
        return None

    def wr(self, reg, val):
        return self.xfer(reg, 1, False, [val]) is not None

    def map(self):
        return {r: self.rd(r) for r in range(0x00, 0x80)}


def show(m, from_r=0x00, to_r=0x7F):
    for base in range(from_r, to_r + 1, 16):
        cells = []
        for r in range(base, min(base + 16, to_r + 1)):
            v = m.get(r)
            cells.append("--" if v is None else "%02X" % v)
        print("  0x%02X: %s" % (base, " ".join(cells)))


def main():
    dev = sys.argv[1] if len(sys.argv) > 1 else "/dev/i2c-4"
    addr = int(sys.argv[2], 0) if len(sys.argv) > 2 else 0x6B
    bus = Bus(dev, addr)

    print("== map before ==")
    before = bus.map()
    show(before)

    print("== writes ==")
    plan = [(0x10, 0x05, "CTRL1 XL 60 Hz"), (0x11, 0x05, "CTRL2 GY 60 Hz"),
            (0x15, 0x04, "CTRL6 FS_G 2000 dps"), (0x17, 0x02, "CTRL8 FS_XL 8 g")]
    for reg, val, what in plan:
        ok = bus.wr(reg, val)
        back = bus.rd(reg)
        print("  0x%02X <- 0x%02X  %-22s ack=%-3s read back immediately: %s"
              % (reg, val, what, "yes" if ok else "NO",
                 "0x%02X" % back if back is not None else "N/A"))

    time.sleep(0.05)
    print("== map after ==")
    after = bus.map()
    show(after)

    print("== diff ==")
    changed = [(r, before[r], after[r]) for r in range(0x80) if before[r] != after[r]]
    if not changed:
        print("  nothing changed anywhere in 0x00..0x7F")
    for r, b, a in changed:
        print("  0x%02X: %s -> %s"
              % (r, "N/A" if b is None else "0x%02X" % b, "N/A" if a is None else "0x%02X" % a))

    print("== STATUS 0x1E and CTRL1 0x10, five reads each ==")
    print("  STATUS: " + " ".join(
        (lambda v: "0x%02X" % v if v is not None else "N/A")(bus.rd(0x1E)) for _ in range(5)))
    print("  CTRL1 : " + " ".join(
        (lambda v: "0x%02X" % v if v is not None else "N/A")(bus.rd(0x10)) for _ in range(5)))

    print("bus errors (NACKs) during the whole run: %d" % bus.errs)
    os.close(bus.fd)


main()