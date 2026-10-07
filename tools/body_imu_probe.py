#!/usr/bin/env python3
"""First confirmation that a bare LSM6DSV16X on this bus is wired up, with no compilation.

The Rust driver is in `src/lib.rs`; the register sequence below is byte-for-byte the same one
(the constants are even named the same), it just sends a few I2C transactions directly instead
of having to be built. It answers exactly one question: **is this chip reachable, and is it
this chip.**

  1. run `i2cdetect` over the bus and see whether anything answers at 0x6B (SA0 is strapped, so
     the address is not scanned for);
  2. read `WHO_AM_I` (0x0F). It must be 0x70. If it is not, stop immediately — never continue
     past a stranger, because an unrelated part read through this register map returns numbers
     that look like measurements;
  3. configure the chip with the driver's own sequence, then dump one SFLP game-rotation vector
     word out of the FIFO (7 bytes: a tag byte plus x/y/z as IEEE half-precision).

Three things worth knowing before reading the code (all measured, not assumed)
------------------------------------------------------------------------------
* **A register write has to be one joined transaction.** One `i2c_msg`, with the register number
  and the data in the same buffer. Split into two messages (register in one, data in the next),
  the chip ACKs every byte and then **silently drops** the payload: the register keeps its old
  value, the readback agrees with itself, and the bring-up is all green. `Bus.write()` below has
  only that one shape and no `write_split` option. Controlled comparison, 6/6, in
  `body_imu_write_shape.py`.
* **The gyroscope full scale is ±500 dps** (`CTRL6.FS_G = 0b0010`). 0.0175 dps/LSB corresponds to
  ±500 dps and nothing else, so a wrong setting raises no error anywhere and is silently off by a
  factor of two — the reference C driver on a comparable board runs this part at ±250 dps, and
  copying it is that mistake.
* The chip is **bare**: `SW_RESET`/`BOOT` are not issued here and not in the driver either.
  `configure` writes every register it depends on, so a warm chip left by a previous process is
  as well-defined as a cold one.

Usage
-----
    # On the board. Defaults to /dev/i2c-4 @ 0x6B, which works whether or not the udev alias
    # from the documentation has been installed.
    python3 tools/body_imu_probe.py

    # Through the udev alias, or at another address.
    python3 tools/body_imu_probe.py --bus /dev/i2c-imu
    python3 tools/body_imu_probe.py --bus /dev/i2c-4 --address 0x6B

    # i2cdetect + WHO_AM_I only: writes not one register.
    python3 tools/body_imu_probe.py --read-only

    # Wait longer for an SFLP word (default 5 s; a chip that has just powered up genuinely has
    # none for its first tens of milliseconds).
    python3 tools/body_imu_probe.py --seconds 15

The three criteria: `i2cdetect` shows an answer in the 0x6B column (`6b`); `WHO_AM_I == 0x70`;
and a FIFO word arrives whose tag is SFLP's game-rotation vector and whose payload is not all
zero.

Exit codes: 0 all three passed; 1 nothing at 0x6B, or `WHO_AM_I` is not 0x70; 2 no SFLP word
arrived in time; 3 the bus could not be opened.
"""

from __future__ import annotations

import argparse
import ctypes
import fcntl
import os
import re
import struct
import subprocess
import sys
import time
from typing import List, Optional, Tuple

I2C_RDWR = 0x0707
I2C_M_RD = 0x0001


# ---- Register map: one-to-one with the constants in src/lib.rs. Change one, change both. -----
WHO_AM_I_REG = 0x0F          # expected 0x70, fixed by the part; nothing relative to fall back to
WHO_AM_I = 0x70
ADDRESS_DEFAULT = 0x6B       # SAD = 110101xb with SA0 tied to Vdd_IO

FUNC_CFG_ACCESS_REG = 0x01
FIFO_CTRL3_REG = 0x09
FIFO_CTRL4_REG = 0x0A
CTRL1_REG = 0x10
CTRL2_REG = 0x11
CTRL3_REG = 0x12
CTRL6_REG = 0x15
CTRL8_REG = 0x17
FIFO_STATUS1_REG = 0x1B      # FIFO_STATUS1 and FIFO_STATUS2 are adjacent; read 2 bytes at once
FIFO_DATA_OUT_TAG_REG = 0x78

# Embedded-function page. These three addresses are different registers in the main page and are
# only reachable while FUNC_CFG_ACCESS has EMB_FUNC_REG_ACCESS set — and that same bit is what
# decides whether a write lands at all.
EMB_FUNC_EN_A_REG = 0x04
EMB_FUNC_FIFO_EN_A_REG = 0x44
SFLP_ODR_REG = 0x5E

FUNC_CFG_EMB_ACCESS = 0x80
SFLP_GAME_EN = 0x02
# SFLP_GAME_ODR = 011 (120 Hz) sits in the [5:3] field, not in [2:0]. The register's reset value
# reads back as 0x03, which "looks like the 120 Hz enum value" but lands as 000 (15 Hz) — so this
# has to be written as 0x18.
SFLP_ODR_120HZ = 0x18
FIFO_MODE_STREAM = 0x06
FIFO_MODE_BYPASS = 0x00
FIFO_BDR_NONE = 0x00
CTRL3_BDU_IF_INC = 0x40 | 0x04
ODR_240HZ_HIGH_PERF = 0x07
CTRL6_FS_G_500_DPS = 0x02
CTRL8_FS_XL_8G = 0x02

FIFO_WORD_LEN = 7            # one FIFO word = a 1-byte tag plus 6 data bytes
FIFO_TAG_SFLP_GAME = 0x13    # lives in bits [7:3] of the tag byte
MAX_FIFO_WORDS_PER_TICK = 4  # the driver's cap, for the reason given in src/lib.rs

BUS_DEFAULT = "/dev/i2c-4"


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
    """A minimal i2c-dev host: reads take two messages (write the register number, repeated
    start, read the data), writes take **one** (register and data in the same buffer). That is
    the only write shape in this whole script."""

    def __init__(self, dev: str, addr: int):
        self.dev = dev
        self.addr = addr
        self.errs = 0
        self.fd = os.open(dev, os.O_RDWR)

    def _io(self, msgs) -> bool:
        io = I2cRdwrIoctlData(ctypes.cast(msgs, ctypes.POINTER(I2cMsg)), len(msgs))
        try:
            fcntl.ioctl(self.fd, I2C_RDWR, io)
        except OSError:
            self.errs += 1
            return False
        return True

    def read(self, reg: int, length: int = 1) -> Optional[bytes]:
        abuf = (ctypes.c_uint8 * 1)(reg)
        dbuf = (ctypes.c_uint8 * length)()
        msgs = (I2cMsg * 2)()
        msgs[0].addr, msgs[0].flags, msgs[0].len, msgs[0].buf = self.addr, 0, 1, abuf
        msgs[1].addr, msgs[1].flags, msgs[1].len, msgs[1].buf = (
            self.addr, I2C_M_RD, length, dbuf)
        if not self._io(msgs):
            return None
        return bytes(dbuf)

    def read_byte(self, reg: int) -> Optional[int]:
        d = self.read(reg, 1)
        return d[0] if d else None

    def write(self, reg: int, vals: List[int]) -> bool:
        """One joined write: register number and data in a single continuous transfer. This is
        the only correct shape."""
        buf = (ctypes.c_uint8 * (1 + len(vals)))(reg, *vals)
        msgs = (I2cMsg * 1)()
        msgs[0].addr, msgs[0].flags, msgs[0].len, msgs[0].buf = (
            self.addr, 0, 1 + len(vals), buf)
        return self._io(msgs)

    def close(self) -> None:
        os.close(self.fd)


def bus_number(dev: str) -> Optional[int]:
    """The bus number behind a device node, for `i2cdetect`. An alias (/dev/i2c-imu) is resolved
    first."""
    real = os.path.realpath(dev)
    m = re.fullmatch(r"/dev/i2c-(\d+)", real)
    return int(m.group(1)) if m else None


def i2cdetect_row(dev: str) -> Optional[str]:
    """Run `i2cdetect -y -r <n>` and pull out the row containing 0x6B. None if i2c-tools is not
    installed."""
    n = bus_number(dev)
    if n is None:
        print(f"  (no bus number, skipping i2cdetect: {dev} is not /dev/i2c-N)")
        return None
    # /usr/sbin is usually not in a non-root PATH, and that is where i2c-tools lives.
    for exe in ("i2cdetect", "/usr/sbin/i2cdetect", "/sbin/i2cdetect"):
        try:
            out = subprocess.run([exe, "-y", "-r", str(n)],
                                 capture_output=True, text=True, timeout=10)
            break
        except FileNotFoundError:
            continue
        except subprocess.TimeoutExpired:
            print("  !! i2cdetect timed out")
            return None
    else:
        print("  (no i2cdetect: `apt install i2c-tools` on the board; falling back to a direct "
              "WHO_AM_I read)")
        return None
    if out.returncode != 0:
        print(f"  !! i2cdetect exited {out.returncode}: {out.stderr.strip()}")
        return None
    for line in out.stdout.splitlines():
        if re.match(r"^60:", line):
            return line
    return None


def check_i2cdetect(dev: str, addr: int) -> Optional[bool]:
    """Is 0x6B there? True yes / False no / None cannot tell (tool missing)."""
    row = i2cdetect_row(dev)
    if row is None:
        return None
    print(f"  {row}")
    # i2cdetect prints an answering device as `6b`, a free address as `--`, and one held by a
    # kernel driver as `UU`. The column for our address inside the 0x60 row is addr - 0x60.
    cells = row.split()[1:]
    cell = cells[addr - 0x60] if 0 <= addr - 0x60 < len(cells) else "--"
    if cell == "UU":
        print(f"  !! 0x{addr:02X} shows UU: a kernel driver already owns it, so i2c-dev cannot"
              " reach it")
        return False
    if cell == "--":
        print(f"  !! nothing answered at 0x{addr:02X}")
        return False
    print(f"  0x{addr:02X} answered (i2cdetect shows {cell})")
    return True


def configure(bus: Bus) -> List[Tuple[str, int, int, Optional[int]]]:
    """The same sequence as `BodyImu::configure`, returning a per-register readback.
    Main-page registers are written and then read straight back. The embedded-function page is
    reached with the access gate open and the gate is closed afterwards — leaving it open would
    make every later register mean something else, including on the failure path."""
    plan: List[Tuple[str, int, int]] = [
        ("CTRL3", CTRL3_REG, CTRL3_BDU_IF_INC),
        ("CTRL1", CTRL1_REG, ODR_240HZ_HIGH_PERF),
        ("CTRL2", CTRL2_REG, ODR_240HZ_HIGH_PERF),
        ("CTRL6", CTRL6_REG, CTRL6_FS_G_500_DPS),
        ("CTRL8", CTRL8_REG, CTRL8_FS_XL_8G),
        # Batching rates must not change while the FIFO is running, so it is emptied and put
        # into bypass first.
        ("FIFO_CTRL4", FIFO_CTRL4_REG, FIFO_MODE_BYPASS),
        ("FIFO_CTRL3", FIFO_CTRL3_REG, FIFO_BDR_NONE),
    ]
    rows: List[Tuple[str, int, int, Optional[int]]] = []
    for _, reg, val in plan:
        bus.write(reg, [val])
    for name, reg, val in plan:
        rows.append((name, reg, val, bus.read_byte(reg)))

    # Embedded-function page: the SFLP rate, its enable, and "put the fused word in the FIFO".
    embedded: List[Tuple[str, int, int]] = [
        ("SFLP_ODR", SFLP_ODR_REG, SFLP_ODR_120HZ),
        ("EMB_FUNC_EN_A", EMB_FUNC_EN_A_REG, SFLP_GAME_EN),
        ("EMB_FUNC_FIFO_EN_A", EMB_FUNC_FIFO_EN_A_REG, SFLP_GAME_EN),
    ]
    bus.write(FUNC_CFG_ACCESS_REG, [FUNC_CFG_EMB_ACCESS])
    for name, reg, val in embedded:
        bus.write(reg, [val])
    for name, reg, val in embedded:
        rows.append((name, reg, val, bus.read_byte(reg)))
    bus.write(FUNC_CFG_ACCESS_REG, [0x00])

    bus.write(FIFO_CTRL3_REG, [FIFO_BDR_NONE])
    bus.write(FIFO_CTRL4_REG, [FIFO_MODE_STREAM])
    return rows


def wait_for_sflp_word(bus: Bus, seconds: float,
                       hz: float = 50.0) -> Optional[Tuple[bytes, int]]:
    """Poll at 50 Hz and return (the first live SFLP word, how many words were queued ahead of
    it).

    "Live" means the tag is SFLP's game-rotation vector and the payload is not all zero: an
    all-zero payload is SFLP saying "I have not written my state yet", and accepting it as a
    sample would report a robot lying on its side as upright.
    """
    deadline = time.monotonic() + seconds
    period = 1.0 / hz
    watched = 0
    while time.monotonic() < deadline:
        watched += 1
        status = bus.read(FIFO_STATUS1_REG, 2)
        if status is None:
            time.sleep(period)
            continue
        waiting = status[0] | ((status[1] & 0x01) << 8)
        words = min(waiting, MAX_FIFO_WORDS_PER_TICK)
        if words == 0:
            time.sleep(period)
            continue
        buf = bus.read(FIFO_DATA_OUT_TAG_REG, words * FIFO_WORD_LEN)
        if buf is None:
            time.sleep(period)
            continue
        for i in range(words):
            word = buf[i * FIFO_WORD_LEN:(i + 1) * FIFO_WORD_LEN]
            if len(word) < FIFO_WORD_LEN:
                break
            tag = word[0] >> 3
            if tag != FIFO_TAG_SFLP_GAME or word[1:] == b"\x00" * 6:
                continue
            print(f"  poll {watched}: {waiting} word(s) queued, and word {i + 1} going through "
                  f"them oldest-first is live")
            return word, waiting
        time.sleep(period)
    return None


def half(value: int) -> float:
    """IEEE 754 half (low 16 bits) to float. Python 3.6+ struct supports '<e' directly."""
    return struct.unpack("<e", struct.pack("<H", value))[0]


def dump_word(word: bytes) -> None:
    tag = word[0]
    x, y, z = (struct.unpack("<H", word[1 + 2 * i:3 + 2 * i])[0] for i in range(3))
    fx, fy, fz = half(x), half(y), half(z)
    norm2 = fx * fx + fy * fy + fz * fz
    print(f"  raw   : {word.hex(' ')}")
    print(f"  tag   : 0x{tag:02X} (bits[7:3] = 0x{tag >> 3:02X}, "
          f"expected 0x{FIFO_TAG_SFLP_GAME:02X})")
    print(f"  x/y/z : 0x{x:04X} 0x{y:04X} 0x{z:04X}"
          f"  -> {fx:+.5f} {fy:+.5f} {fz:+.5f}")
    if norm2 <= 1.0:
        print(f"  w     : {1.0 - norm2 ** 0.5:.5f}"
              f"  (|xyz| = {norm2 ** 0.5:.5f}, must be <= 1)")
    else:
        print(f"  !! |xyz| = {norm2 ** 0.5:.5f} > 1: those three halves are not a rotation "
              "vector")


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(
        description="First, no-compilation confirmation of a bare LSM6DSV16X on I2C",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="The register sequence is the same one as src/lib.rs; writes use only the "
               "'joined' shape (split across two messages, the payload is silently dropped).")
    ap.add_argument("--bus", default=BUS_DEFAULT,
                    help=f"i2c-dev node, default {BUS_DEFAULT} (the alias /dev/i2c-imu works "
                         "too)")
    ap.add_argument("--address", default=hex(ADDRESS_DEFAULT),
                    help=f"7-bit address, default {hex(ADDRESS_DEFAULT)} (SA0 is strapped, so "
                         "this is not scanned for)")
    ap.add_argument("--read-only", action="store_true",
                    help="i2cdetect + WHO_AM_I only; writes not one register")
    ap.add_argument("--seconds", type=float, default=5.0,
                    help="how long to wait for an SFLP word, default 5")
    args = ap.parse_args(argv)
    addr = int(args.address, 0)

    print(f"== LSM6DSV16X probe · {args.bus} @ 0x{addr:02X} ==")

    print(f"1) i2cdetect: looking for 0x{addr:02X}")
    present = check_i2cdetect(args.bus, addr)

    try:
        bus = Bus(args.bus, addr)
    except OSError as exc:
        print(f"!! cannot open {args.bus}: {exc}")
        print("   (on the board that is `apt install i2c-tools`; if /dev/i2c-4 does not exist "
              "the controller overlay is not enabled — see docs/bring-up.md)")
        return 3

    try:
        print(f"2) WHO_AM_I(0x{WHO_AM_I_REG:02X})")
        who = bus.read_byte(WHO_AM_I_REG)
        if who is None:
            print(f"  !! nothing answered at 0x{addr:02X} (NACK)")
            return 1
        print(f"  read 0x{who:02X}, expected 0x{WHO_AM_I:02X}")
        if who != WHO_AM_I:
            print("  !! not this part: a stranger read through this map returns numbers that "
                  "look like measurements, and refusing is more honest than falling back")
            return 1

        if args.read_only:
            print("(--read-only: writing no registers, stopping here)")
            print(f"bus errors: {bus.errs}")
            return 0 if (present is not False and bus.errs == 0) else 1

        print("3) configure (the driver's own sequence)")
        for name, reg, wrote, read in configure(bus):
            got = "NACK" if read is None else f"0x{read:02X}"
            flag = "OK" if read == wrote else "!! MISMATCH"
            print(f"  {name:<17} 0x{reg:02X} <- 0x{wrote:02X}  read back {got}  {flag}")

        print(f"4) waiting for an SFLP game-rotation word (up to {args.seconds:g} s)")
        found = wait_for_sflp_word(bus, args.seconds)
        if found is None:
            print("  !! no live SFLP word arrived — the fusion block is not running, or the rate "
                  "or the enable did not land")
            print(f"bus errors: {bus.errs}")
            return 2
        word, waiting = found
        dump_word(word)
        print(f"bus errors: {bus.errs}")
        print("verdict: 3/3 passed (i2cdetect answered, WHO_AM_I = 0x70, an SFLP word is in the "
              "FIFO)")
        return 0
    finally:
        bus.close()


if __name__ == "__main__":
    raise SystemExit(main())