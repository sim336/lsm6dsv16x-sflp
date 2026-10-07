# Bring-up

From a bare LSM6DSV16X wired to an I²C header to a live attitude, in four checks that fail
loudly.

The order matters. Each step answers one question and stops if it is wrong, so a failure
never looks like the step after it.

- [0. What you need](#0-what-you-need)
- [1. Wiring](#1-wiring)
- [2. Give the bus a stable name (optional)](#2-give-the-bus-a-stable-name-optional)
- [3. Check the chip answers](#3-check-the-chip-answers)
- [4. Check the configuration lands](#4-check-the-configuration-lands)
- [5. Check the fusion block is running](#5-check-the-fusion-block-is-running)
- [6. Measure what a tick costs](#6-measure-what-a-tick-costs)
- [7. Measure your mounting rotation](#7-measure-your-mounting-rotation)
- [8. The tools](#8-the-tools)
- [9. When it does not work](#9-when-it-does-not-work)

## 0. What you need

- A **bare LSM6DSV16X** (the chip itself, not a carrier board with an MCU on it).
- An I²C controller on your host that you can enable. On the board this was developed on
  (Radxa Zero 3, RK3566) that is `i2c4-M0` on the 40-pin header, exposed as `/dev/i2c-4`.
- `i2c-tools` on the host (`apt install i2c-tools`) — used by the Python probe, optional
  otherwise.
- Nothing else. The Rust driver needs no kernel module beyond `i2c-dev`.

## 1. Wiring

| chip pin | connects to | note |
|---|---|---|
| `VDD` | 3.3 V | |
| `VDD_IO` | 3.3 V | |
| `GND` | GND | |
| `SCL` | the controller's SCL | 40-pin pin **28** on the board this was built on |
| `SDA` | the controller's SDA | 40-pin pin **27** |
| `SA0` | **3.3 V** (strap high) | sets the address; see below |
| `CS` | 3.3 V (select I²C) | leave floating only if the datasheet's I²C default applies |

Pull-ups: most 40-pin headers already carry 4.7 kΩ on SDA/SCL. Check before adding your
own — doubling up is not fatal but the waveform gets ugly fast on a long jumper.

### SA0, and why the address is not scanned

`SAD = 110101xb`, and `SA0` selects the `b`:

- **`SA0` tied to `VDD_IO` → `0x6B`** (this is the configuration everything here assumes);
- `SA0` tied low → `0x6A`.

> **Do not scan the bus for this part.** The address is strappable only between `0x6A` and
> `0x6B`, and `ADDRESS` in the driver is a constant rather than a scan result.
>
> **Tie `SA0` to a rail, do not leave it floating.** Floating is *not* a third state — it
> picks up whatever the trace is near. On one board that was the 50 Hz around it, and it
> showed up as `ENXIO` from the kernel with no pattern in the data. Before `SA0` was tied
> down, the part chimed in at `0x6A` and `0x6B` alternately at about 50 Hz — which is why
> two of the tools in §8 exist at all, and why they still matter for a board you wired
> yourself.

## 2. Give the bus a stable name (optional)

`/dev/i2c-N` numbers can move when the device tree changes. A udev rule gives the bus a name
that does not:

```udev
# /etc/udev/rules.d/70-i2c-imu.rules
SUBSYSTEM=="i2c-dev", KERNELS=="fe5d0000.i2c", SYMLINK+="i2c-imu"
```

`fe5d0000.i2c` is `i2c4` on the RK3566. Find yours with:

```sh
readlink -f /sys/bus/i2c/devices/i2c-4
```

Reload and check:

```sh
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/i2c-imu
```

The controller itself is enabled by an overlay. On the board this was built on the overlay
list has to include **both** `uart2-m0` and `i2c4-m0` — the serial console and the IMU's
controller share the overlay set. If `/dev/i2c-4` does not exist at all, that is the
overlay, not this crate:

```sh
grep -o 'overlays=[^ ]*' /boot/armbianEnv.txt   # or your distro's equivalent
```

`/dev/i2c-4` is owned `root:i2c`. Either run the bench as root or add yourself to the `i2c`
group and re-login:

```sh
sudo usermod -aG i2c "$USER"    # then log out and back in
```

`/dev/i2c-imu` is only an alias — `/dev/i2c-4` always works, whether or not §2 was done.

## 3. Check the chip answers

No compilation. This is the fastest way to a yes/no, and it writes nothing unless you ask it
to:

```sh
python3 tools/body_imu_probe.py --bus /dev/i2c-4 --address 0x6B
```

It runs three checks and prints a verdict:

1. **`i2cdetect`** sees something in the `0x6B` column (`6b`). `--` means nothing answered;
   `UU` means a kernel driver already owns the address, and `i2c-dev` cannot reach it.
2. **`WHO_AM_I`** (`0x0F`) reads `0x70`. If it is anything else, **stop** — a stranger read
   through this register map returns numbers that look like measurements, and refusing is
   more honest than falling back.
3. **configure + one SFLP word** out of the FIFO (see §4–5).

Exit codes: `0` all three passed; `1` nothing at `0x6B` or `WHO_AM_I` is wrong; `2` no SFLP
word arrived; `3` the bus could not be opened.

A real run on the board this crate was developed on:

```text
== LSM6DSV16X probe · /dev/i2c-4 @ 0x6B ==
1) i2cdetect: looking for 0x6B
  60: -- -- -- -- -- -- -- -- -- -- -- 6b -- -- -- --
  0x6B answered (i2cdetect shows 6b)
2) WHO_AM_I(0x0F)
  read 0x70, expected 0x70
3) configure (the driver's own sequence)
  CTRL3             0x12 <- 0x44  read back 0x44  OK
  CTRL1             0x10 <- 0x07  read back 0x07  OK
  CTRL2             0x11 <- 0x07  read back 0x07  OK
  CTRL6             0x15 <- 0x02  read back 0x02  OK
  CTRL8             0x17 <- 0x02  read back 0x02  OK
  FIFO_CTRL4        0x0A <- 0x00  read back 0x00  OK
  FIFO_CTRL3        0x09 <- 0x00  read back 0x00  OK
  SFLP_ODR          0x5E <- 0x18  read back 0x18  OK
  EMB_FUNC_EN_A     0x04 <- 0x02  read back 0x02  OK
  EMB_FUNC_FIFO_EN_A 0x44 <- 0x02  read back 0x02  OK
4) waiting for an SFLP game-rotation word (up to 5 s)
  poll 2: 3 word(s) queued, and word 1 going through them oldest-first is live
  raw   : 9f ea 31 73 3a 75 af
  tag   : 0x9F (bits[7:3] = 0x13, expected 0x13)
  x/y/z : 0x31EA 0x3A73 0xAF75  -> +0.18481 +0.80615 -0.11652
  w     : 0.16477  (|xyz| = 0.83523, must be <= 1)
bus errors: 0
verdict: 3/3 passed (i2cdetect answered, WHO_AM_I = 0x70, an SFLP word is in the FIFO)
```

`--read-only` stops after step 2 and writes no register at all — the safe thing to run
before you trust the wiring.

## 4. Check the configuration lands

This is the step that catches §3 of [design.md](design.md#3-a-register-write-is-one-i2c_msg-and-it-fails-silently)
and §4. **A register write the chip ACKs and then drops is invisible on the bus** — it
leaves the register in its reset state and every readback agreeing with itself. The only way
to see it is to read the map back afterwards, which both the Python probe (above) and the
Rust bench do.

Ten registers, in bring-up order: the main page first (`CTRL3`, `CTRL1`, `CTRL2`, `CTRL6`,
`CTRL8`, `FIFO_CTRL3`, `FIFO_CTRL4`), then the embedded-function page (`SFLP_ODR`,
`EMB_FUNC_EN_A`, `EMB_FUNC_FIFO_EN_A`). Every "read back" must equal "wrote".

If **`EMB_FUNC_EN_A`** specifically does not read back with the SFLP enable bit set, the
driver refuses with `BodyImuError::SflpNotEnabled` rather than letting you discover it from
an empty FIFO a second later. That is the one register `configure` checks itself.

Values and the reasoning behind each are in [register-map.md](register-map.md).

## 5. Check the fusion block is running

A FIFO word with the SFLP tag (`0x13` in bits `[7:3]`) and a payload that is **not all
zero**. An all-zero payload is SFLP saying "I have not written my state yet" — accepting it
as a sample would report a robot lying on its side as upright.

This is the only step that tells a working fusion block from a chip that ACKed the enable
and did nothing: both produce byte-identical 12-byte blocks, because an empty tick's
quaternion bytes are zero and zero is the "hold the last attitude" sentinel. See §6 of
[design.md](design.md#6-the-fifo-and-why-the-block-cannot-tell-you-it-is-empty).

## 6. Measure what a tick costs

The question that decides the design of your consumer: can a read fit inside the control
tick you need it in?

```sh
# Build for the board. On the board itself, or cross-compiled.
cargo build --release

# 10 s at 50 Hz through the alias.
./target/release/bench_imu --bus /dev/i2c-imu --hz 50 --seconds 10

# Before the udev alias exists, or at another address.
./target/release/bench_imu --bus /dev/i2c-4 --address 0x6B --hz 50 --seconds 30
```

It prints `WHO_AM_I`, the per-register readback, a line per second
(`t / rate / read mean,p95,max / jitter p95,max / fifo% / sflp% / errors / |g| / |q|`), a
total, and one `attitude` line. Exit code is non-zero if `WHO_AM_I` is wrong, if **not one**
SFLP word arrived, or if any tick's read failed — the three ways this can look like it works
while it does not.

The `attitude` line is also the only place `MOUNT` is visible (§7 of
[design.md](design.md#7-the-mounting-rotation)), which is how it was measured.

Measured on the board this crate was developed on is in the
[README](../README.md#measured).

## 7. Measure your mounting rotation

**Do this.** The mounting rotation is a property of how your chip is bolted on, not of the
chip, and the failure is a robot that reports itself upside down while every register
readback agrees. `MOUNT` in this crate describes *one* board.

1. Hold the board in a known upright pose and read the `attitude` line from `bench_imu`.
2. **Upright must print `gravity=[+0.000, +0.000, -1.000]`.** Anything else is the mounting
   rotation.
3. Pitch it 90° forward (belly down) and read again: gravity must move to `[+1, 0, 0]`.
4. Fit the rotation that satisfies both — see the method in §7 of [design.md](design.md#7-the-mounting-rotation).
   Pass it to `SflpDecoder::new(your_mount)`.

## 8. The tools

Everything in `tools/` is Python, needs only `i2c-dev` and `i2c-tools`, and is meant to be
run **on the target** — no compilation, so a wiring question is answered in seconds.

| tool | question it answers | default |
|---|---|---|
| [`body_imu_probe.py`](../tools/body_imu_probe.py) | *Is the chip there, is it this part, is it fusing?* — the four steps above | 0 / 1 / 2 / 3 exit codes |
| [`body_imu_write_shape.py`](../tools/body_imu_write_shape.py) | *Does a joined write land where a split one does not?* — the controlled comparison | side-by-side |
| [`body_imu_address.py`](../tools/body_imu_address.py) | *`0x6A` or `0x6B`?* — stability classification of the two strappable addresses | 3000 rounds |
| [`body_imu_window.py`](../tools/body_imu_window.py) | *Does a write land inside one stable address window?* read → write → read-back | 40 rounds |
| [`body_imu_regmap.py`](../tools/body_imu_regmap.py) | *Where does an ACKed write actually go?* — full `0x00`–`0x7F` dump, diffed before and after | — |

The last two were written while `SA0` was floating, when the part chimed in at `0x6A` and
`0x6B` alternately. With `SA0` tied to a rail that is history — the address is fixed, and
`body_imu_address.py` now returns `0x6B` immediately. They are kept because their *method*
is still the right one for those two questions on a board you wired yourself, not because
the condition still exists.

> The register map is transcribed in two places: the Rust constants in
> [`src/lib.rs`](../src/lib.rs) and the Python constants at the top of
> `tools/body_imu_probe.py`. They are kept one-to-one on purpose — **change one, change
> both.**

## 9. When it does not work

| symptom | likely cause | check |
|---|---|---|
| `/dev/i2c-4` does not exist | controller overlay not enabled | §2, `armbianEnv.txt` |
| `i2cdetect` shows `UU` at `0x6B` | a kernel driver owns the address | `lsmod`; unbind it or use a different bus |
| `i2cdetect` shows `--` at `0x6B` | wiring, `SA0`, or a wrong bus | §1, then `body_imu_address.py` |
| `WHO_AM_I` reads anything but `0x70` | wrong part, or the chip is not the one you think | stop — do not continue past a stranger |
| configure says OK but no SFLP word ever arrives | the write did not land, or the fusion block is off | `body_imu_regmap.py` (does the write land anywhere?), `body_imu_write_shape.py` |
| SFLP words arrive at about 15/s, not about 120/s | `SFLP_ODR` written as the enum `0x03` instead of the field `0x18` | §6 of [design.md](design.md#6-the-fifo-and-why-the-block-cannot-tell-you-it-is-empty) |
| attitude is frozen but no error | the chip answers without refreshing | `StaleImuTracker`; §8 of [design.md](design.md#8-the-stale-imu-fault) |
| upright reports gravity pointing the wrong way | mounting rotation | §7 |
| gyro rates are wrong by a factor of two | full scale at ±250 instead of ±500 dps | §4 of [design.md](design.md#4-500-dps-is-a-contract-with-the-decoder-not-a-preference) |

---

Next: [register-map.md](register-map.md) for the registers written and why.