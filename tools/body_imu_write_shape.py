#!/usr/bin/env python3
"""The controlled comparison behind "a register write has to be one joined transaction".

Every write in the first bring-up attempt used two i2c messages (a register byte, then a data
byte), which the controller puts on the bus as START/addr/reg/RESTART/addr/data/STOP. A slave
that only accepts one continuous write ACKs every byte and stores nothing — which is exactly
what this chip does: the register keeps its old value, the readback agrees with itself, and a
bring-up built on it reports success.

This repeats the same write in both shapes, side by side, so they can be compared directly, and
reads OUT_TEMP as a liveness check on the core: a running core reports a temperature near 25 C,
one held in reset reports zero.

Historical note: this script's timestamps come from a period when the SA0 strap was floating and
the part chimed in at 0x6A and 0x6B alternately at about 50 Hz. With SA0 tied to a rail the
address is fixed at 0x6B (1500/1500 probes, zero NACKs) and that history is over. The conclusion
below is not history — it is how this chip behaves today, and it is why the Rust driver's
`write_reg` makes a single `I2c::write` call and why `every_register_write_is_a_single_i2c_message`
exists as a test rather than as a comment.

For routine confirmation use `body_imu_probe.py`; this one is the experiment that establishes
the rule.

Usage (on the board): python3 tools/body_imu_write_shape.py [/dev/i2c-4] [0x6B]
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
    _fields_ = [("msgs", ctypes.POINTER(I2cMsg)), ("nmsgs", ctypes.c_uint32)]


class Bus:
    def __init__(self, dev, addr):
        self.fd = os.open(dev, os.O_RDWR)
        self.addr = addr
        self.errs = 0

    def _io(self, msgs):
        io = I2cRdwrIoctlData(ctypes.cast(msgs, ctypes.POINTER(I2cMsg)), len(msgs))
        try:
            fcntl.ioctl(self.fd, I2C_RDWR, io)
        except OSError:
            self.errs += 1
            return False
        return True

    def write_split(self, reg, vals):
        """Two messages: the register byte, then the data (a repeated start between them)."""
        abuf = (ctypes.c_uint8 * 1)(reg)
        dbuf = (ctypes.c_uint8 * len(vals))(*vals)
        msgs = (I2cMsg * 2)()
        msgs[0].addr, msgs[0].flags, msgs[0].len, msgs[0].buf = self.addr, 0, 1, abuf
        msgs[1].addr, msgs[1].flags, msgs[1].len, msgs[1].buf = self.addr, 0, len(vals), dbuf
        return self._io(msgs)

    def write_joined(self, reg, vals):
        """One message: the register byte and the data in a single continuous write."""
        buf = (ctypes.c_uint8 * (1 + len(vals)))(reg, *vals)
        msgs = (I2cMsg * 1)()
        msgs[0].addr, msgs[0].flags, msgs[0].len, msgs[0].buf = self.addr, 0, 1 + len(vals), buf
        return self._io(msgs)

    def read(self, reg, length=1):
        abuf = (ctypes.c_uint8 * 1)(reg)
        dbuf = (ctypes.c_uint8 * length)()
        msgs = (I2cMsg * 2)()
        msgs[0].addr, msgs[0].flags, msgs[0].len, msgs[0].buf = self.addr, 0, 1, abuf
        msgs[1].addr, msgs[1].flags, msgs[1].len, msgs[1].buf = self.addr, I2C_M_RD, length, dbuf
        if not self._io(msgs):
            return None
        return bytes(dbuf)


def main():
    bus = Bus(sys.argv[1] if len(sys.argv) > 1 else "/dev/i2c-4",
              int(sys.argv[2], 0) if len(sys.argv) > 2 else 0x6B)
    hex1 = lambda d: "0x%02X" % d[0] if d else "NACK"

    print("== write as two messages (repeated start), the shape used at first ==")
    ok = bus.write_split(0x10, [0x05])
    print("  CTRL1 <- 0x05 ack=%-5s read back %s" % (ok, hex1(bus.read(0x10))))

    print("== write as one continuous message ==")
    ok = bus.write_joined(0x10, [0x05])
    print("  CTRL1 <- 0x05 ack=%-5s read back %s" % (ok, hex1(bus.read(0x10))))
    ok = bus.write_joined(0x10, [0x05, 0x05])
    print("  CTRL1,CTRL2 <- 05 05 ack=%-5s read back %s %s"
          % (ok, hex1(bus.read(0x10)), hex1(bus.read(0x11))))
    ok = bus.write_joined(0x17, [0x02, 0x00])
    print("  CTRL8 <- 0x02 ack=%-5s read back %s" % (ok, hex1(bus.read(0x17))))

    print("== core liveness: OUT_TEMP 0x20 ==")
    for _ in range(3):
        d = bus.read(0x20, 2)
        if d:
            raw = d[0] | (d[1] << 8)
            if raw & 0x8000:
                raw -= 0x10000
            print("  0x20..0x21 = %02X %02X  -> %.1f C" % (d[0], d[1], 25.0 + raw / 256.0))
        else:
            print("  NACK")
        time.sleep(0.05)

    print("== after all that ==")
    print("  CTRL1 %s  CTRL2 %s  CTRL3 %s  CTRL6 %s  CTRL8 %s  STATUS %s"
          % (hex1(bus.read(0x10)), hex1(bus.read(0x11)), hex1(bus.read(0x12)),
             hex1(bus.read(0x15)), hex1(bus.read(0x17)), hex1(bus.read(0x1E))))
    print("  bus errors: %d" % bus.errs)
    os.close(bus.fd)


main()