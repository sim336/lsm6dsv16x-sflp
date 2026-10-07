# Design

Why this crate exists, what it deliberately does *not* do, and which parts of it are
measurements rather than choices.

- [1. The problem](#1-the-problem)
- [2. Two routes to one part, and the seam between them](#2-two-routes-to-one-part-and-the-seam-between-them)
- [3. A register write is one `i2c_msg` (and it fails silently)](#3-a-register-write-is-one-i2c_msg-and-it-fails-silently)
- [4. ±500 dps is a contract with the decoder, not a preference](#4-500-dps-is-a-contract-with-the-decoder-not-a-preference)
- [5. The 12-byte block](#5-the-12-byte-block)
- [6. The FIFO, and why the block cannot tell you it is empty](#6-the-fifo-and-why-the-block-cannot-tell-you-it-is-empty)
- [7. The mounting rotation](#7-the-mounting-rotation)
- [8. The stale-IMU fault](#8-the-stale-imu-fault)
- [9. Honest unknowns](#9-honest-unknowns)

## 1. The problem

ST's **LSM6DSV16X** is a 6-axis IMU with an on-die sensor-fusion block, **SFLP** (sensor
fusion low power). SFLP fuses the accelerometer and the gyroscope and emits a
**game-rotation vector**: a quaternion the host can turn into an attitude without writing a
madgwick or a complementary filter of its own.

The usual way to reach it is a small MCU on a carrier board that polls the chip and
republishes the fused result on another bus. That is a good design — the MCU owns the
timing, the host reads one register — but it puts a second processor between the chip and
the thing that needs the attitude, and it means the register-level behaviour of the chip
is never seen by the host at all.

This crate is the other route: wire the bare chip to the SoC's own I²C controller, read
the registers directly, and **assemble the same bytes the MCU-based board serves**, so the
choice between the two disappears at the seam. Nothing downstream can tell which one it is
talking to.

The hard part is not the register map. It is the three ways this fails **silently** — no
error on the bus, every readback agreeing with itself, and a robot that behaves as if the
IMU were fine. §3, §4 and §7 are those three. Each is pinned by a test rather than a
comment, because a comment cannot fail a build.

## 2. Two routes to one part, and the seam between them

```text
  path A — an MCU carrier board                path B — the bare chip (this crate)
  ┌──────────────────────────────┐             ┌──────────────────────────────┐
  │ some serial bus              │             │ /dev/i2c-4 (i2c4-M0)         │
  │ an on-board MCU polls SFLP   │             │ 40-pin header, SDA / SCL     │
  │ the host reads one register  │             │ 7-bit address 0x6B           │
  │                              │             │ read by the SoC, no MCU      │
  └──────────────┬───────────────┘             └──────────────┬───────────────┘
                 │                                            │
                 │ 12-byte block                      12 bytes, assembled
                 │ (the MCU ships it)                 by `assemble_block` (§5)
                 ▼                                            ▼
                        ┌───────────────────────────────┐
                        │      SflpDecoder (§5)          │
                        └───────────────┬───────────────┘
                                        ▼
                                   ImuData: gyro, gravity, quat
```

| | path A (MCU board) | path B (this crate) |
|---|---|---|
| transport | a serial bus shared with other devices | its own I²C controller |
| who talks to the chip | the carrier board's MCU | the SoC, directly |
| who builds the block | the MCU | [`assemble_block`](../src/lib.rs) |
| fused attitude | SFLP, on the chip | SFLP, on the same chip |
| what is on the wire | 12 of the board's 20 diagnostic bytes | the same 12 bytes, byte for byte |

The seam is the **12-byte block** (§5). If the two paths produce the same bytes, the
decoder, the fall detection, the policy observation and the logging are all one code path.
`assembled_block_decodes_identically_to_the_mcu_board` is the test that makes that claim
mean something: it writes the MCU board's bytes out literally, builds the same sample with
`assemble_block`, and asserts that two independent decoders return equal `ImuData` for four
ticks.

## 3. A register write is one `i2c_msg` (and it fails silently)

This is the constraint the whole bring-up turned on.

| shape | bytes on the wire | result |
|---|---|---|
| joined (**correct**) | `START addr reg data… STOP` | the register takes the value |
| split (**broken**) | `START addr reg RESTART addr data… STOP` | the chip ACKs every byte, then **drops** the payload |

The broken shape leaves the register at its old value. A readback agrees with itself
because nothing ever changed. The bring-up reports success. There is no NACK, no error
flag, nothing to notice — a chip configured by the split shape simply does not fuse, and
the only symptom is an attitude that never arrives.

It was found by **controlled comparison, 6/6 runs**: an `i2c_msg` with `len = 1 + n`
(register and payload in one message) landed; the same write as two messages did not. The
probe tools that settle it are
[`tools/body_imu_write_shape.py`](../tools/body_imu_write_shape.py) (joined vs split, side
by side) and [`tools/body_imu_regmap.py`](../tools/body_imu_regmap.py) (a full 0x00–0x7F
dump, diffed before and after a write).

The Rust path is correct **by construction**, which is exactly why there is a test rather
than a comment. `embedded-hal` 1.0's default `I2c::write` implementation is a single
`Operation::Write`, and `linux-embedded-hal`'s `I2cdev` turns one operation into one
`i2c_msg`. So `self.i2c.write(addr, &[reg, value])` is the joined shape, and there is no
other shape anywhere in this crate.

Pinned by:

- **`every_register_write_is_a_single_i2c_message`** — no transaction may contain two
  consecutive `Write` operations, and a payload write to `CTRL6` really did happen. The
  assertion is on the *shape*, not the bytes: two operations is two `i2c_msg`s.
- **`register_read_is_write_then_read`** — a read is `[Write(reg), Read(1)]` against a
  recording `I2c` mock.
- **`configuration_reads_back_as_written`** — every register `configure` writes appears in
  the readback with the value written, and `FUNC_CFG_ACCESS` is opened *and closed* on both
  paths.

The mock models a dropped write faithfully — `payload_writes()` is empty for a chip that
was never identified — so `who_am_i_must_be_0x70` can assert that a stranger at the address
is refused **before a single register is written**. Reading an unrelated part through this
register map returns numbers that look like measurements; a fallback would be worse than a
refusal.

## 4. ±500 dps is a contract with the decoder, not a preference

`SflpDecoder` scales gyro counts by `GYRO_RAD_PER_LSB = 0.0175 × π/180` rad/s per count,
which is the LSM6DSV16X's **±500 dps** sensitivity and nothing else. So `configure` writes
`CTRL6.FS_G = 0b0010`.

Nothing on the bus states what the full scale is, and getting it wrong raises no error
anywhere — it just decodes to a wrong rate. The easy mistake is specific and nearby: the
reference C driver that ships with a comparable project runs this same part at **±250 dps**
(twice as sensitive), so copying its sequence halves nothing and doubles every rate.

Pinned by **`gyro_full_scale_matches_the_decoder`**, which does *not* restate the constant:
it reads the `FS_G` field out of the byte that was actually written, maps it through the
datasheet's mdps/LSB table, and compares that against the sensitivity measured by decoding a
1000-count block through a real `SflpDecoder`. A ±250 setting therefore fails the test
instead of silently halving every rate.

## 5. The 12-byte block

This is the wire format, and it is the whole interface:

| bytes | contents |
|---|---|
| `0..6` | `OUTX_L_G`…`OUTZ_H_G`, `i16` LE raw counts, ±500 dps |
| `6..12` | SFLP game-rotation vector `x`/`y`/`z` as IEEE **half**; `w = √(1 − x² − y² − z²)` |

Note what is *not* there: no timestamp, no sequence number, no validity flag. That is the
point — a consumer cannot tell which path a block came from, which is what makes the two
paths interchangeable.

`SflpDecoder::decode` turns those bytes into `ImuData`:

- it reads the three `i16` counts, scales them by the ±500 dps constant, and rotates them
  by the mounting quaternion (§7) into the trunk frame;
- it decodes the three halves and reconstructs `w`, rejecting anything that is not a
  rotation vector (`|xyz|² > 1.02`, non-finite, or an all-zero payload);
- it takes a **component-wise median over three** samples for gyro and gravity — single-
  sample spike rejection that discards one wild block without the lag of an average;
- `ready()` is false until 25 live quaternion samples have arrived (about 0.25 s at
  100 Hz). Until then the orientation is a *default*, not a measurement. Fall detection
  must not run before it.

A subtlety preserved from the original: gravity is **normalised before** the median, not
after. A component-wise median across three unit vectors is not itself unit-norm, so during
a transient gravity is slightly short of 1.0. Steady state is exact.
`gravity_stays_close_to_unit_through_a_transient` pins the bound so it is known rather than
assumed.

## 6. The FIFO, and why the block cannot tell you it is empty

A tick with no fused word still produces **12 bytes**, with the quaternion slot zeroed.
That is not a hack: all-zero quaternion bytes are already the sentinel `SflpDecoder` reads
as *"hold the last good attitude"*, because that is what the MCU board emits before SFLP has
written its table. So the empty case costs nothing on either side
(`no_sflp_word_holds_the_last_quaternion`).

But it also means **the block cannot say whether a word arrived at all** — a live tick and an
empty tick are byte-identical, deliberately. That is why `FifoReport` exists
(`fifo_report_tells_a_live_word_from_an_empty_fifo`), and why the FIFO has its own tag and
payload rules:

- one FIFO word is **7 bytes**: a tag byte plus six data bytes, from `FIFO_DATA_OUT_TAG`;
- the SFLP game vector's tag sits in **bits `[7:3]`** and equals `0x13`; a word whose tag
  says otherwise is skipped;
- an **all-zero payload** is SFLP saying "not written yet" and is skipped too — passing it
  through as an identity quaternion would report a robot lying on its side as upright;
- `MAX_FIFO_WORDS_PER_TICK = 4` bounds the cost inside a tick. The FIFO is read
  oldest-first, so dropping the surplus loses the *newest* words, not the oldest, and the
  next tick collects them. Reaching the cap means the loop has already fallen behind.

The gyroscope is **not** batched: it is read straight from `OUTX_L_G` every tick, so the
FIFO carries only fused words. One word per fused sample keeps the newest quaternion at most
one poll old.

Two measured facts about the fused rate, both of which look wrong before you measure them:

- `SFLP_ODR = 0x18` places `SFLP_GAME_ODR = 011` (120 Hz) in bits **`[5:3]`**. The field is
  not `[2:0]`, and the register's reset value reads back as `0x03`, which as an enum value
  looks like the right choice but lands as `000` (15 Hz). The reference C driver on a
  comparable board writes exactly that, so its comment's "120 Hz" is 15 Hz in practice.
- Measured on this board: **118 words/s** with the field at `011`, **14.8 words/s** with it
  at `000`. A FIFO that is merely non-empty cannot tell those apart, which is why the
  bring-up tool reports a rate rather than a yes/no.

## 7. The mounting rotation

The single most likely thing to get wrong when you wire up your own chip. The mounting
rotation is **not a property of the chip** — it is a property of how the chip is bolted on.
Two boards with the same part can be 180° apart, and the failure is a robot that reports
itself upside down while every register readback agrees.

`MOUNT` is a **−90° rotation about Y** (`trunk = [−raw_z, +raw_y, +raw_x]`), measured on the
board this crate was developed on. `SflpDecoder::V2_BOARD_MOUNT` is the **+90°** rotation
(`trunk = [+raw_z, +raw_y, −raw_x]`) of a different board. They differ by 180° about Y and
are **not** interchangeable — which is why `V2_BOARD_MOUNT` is named for the board it
describes rather than being called `DEFAULT`.

The decoder applies the mount to its *inputs*, not to its output:

- gyro: `rotate(mount, gyro_sensor)`;
- quaternion: `mul(q, mount⁻¹)`, so the reported attitude is trunk→world.

**Measure your own.** It takes two known attitudes, because a mounted gyroscope is the one
sensor whose resting output is the same no matter how it is turned — only gravity says
anything:

| pose | gravity, trunk frame | gravity, chip frame | residual |
|---|---|---|---|
| upright | `[-0.132, +0.102, +0.986]` | `[-0.986, +0.102, -0.132]` | 9.6° |
| pitched 90° forward, belly down | `[-0.995, -0.007, -0.103]` | `[+0.103, -0.007, -0.995]` | 5.9° |

The chip-frame column is the same two readings with `V2_BOARD_MOUNT` undone — a pure
property of the chip, independent of what is being fitted to it. Scoring all 24 axis-aligned
rotations against "upright → `(0,0,-1)`, pitched → `(1,0,0)`" picks this one at 15.5° total;
the runner-up is 90.1° away, so the solution is unique and the residuals are the chip's own
tilt on a duck standing on a bench, not ambiguity.

`bench_imu`'s `attitude` line is how it was measured: **a robot standing upright has to print
`gravity=[+0.000, +0.000, -1.000]`**. Anything else there is the mounting rotation, not the
chip. `mount_reports_the_upright_calibration_as_upright` pins the value, and asserts that
`V2_BOARD_MOUNT` gives the *mirror* image — so the two chips cannot accidentally agree.

## 8. The stale-IMU fault

The fault with no other symptom: the bus read succeeds, the chip ACKs, the block decodes,
and the attitude is simply **frozen**. Nothing in a gravity vector says how old it is, so a
control loop will happily balance on a reading from ten seconds ago.

`StaleImuTracker` compares each block with its predecessor. This works because the chip's
fused output changes in the last bit on every sample at rest — so a **byte-identical** block
means the stream stopped, not that the robot stopped moving. It reports two counters:

- `total` — stale reads since startup, cumulative and never reset ("has this ever happened");
- `run` — length of the current unbroken run of stale reads; any fresh block resets it to
  zero ("how bad is it right now").

`STALE_RUN_WARN = 25` (half a second at 50 Hz) is the point at which a caller should say
something rather than merely count. Below it, stay quiet on purpose: warning on the very
first repeated block is what teaches everyone to ignore the message.

The first block can never be stale (there is no predecessor), which
`the_first_block_is_never_stale` pins — getting that wrong puts a `1` in a counter rendered
as an alarm on the first tick of every boot.

It is split out from the read path so it can be tested without hardware: the fault it
describes would otherwise be verifiable only against a broken chip.

## 9. Honest unknowns

Nothing in this crate is a filter or a calibration beyond the one mounting rotation. The
things that are deliberately left open:

| # | unknown | how it would get settled | state |
|---|---|---|---|
| 1 | Does *your* chip's mounting match `MOUNT`? | static-tilt calibration (§7) | **you must measure it** |
| 2 | The chip's internal filters | not configured here: gyro LPF1 / accel LPF2 are **off**. A comparable board's filter settings are generally not published, so no filter choice here could be checked against one. The raw counts go to the decoder, which rejects single-sample spikes itself | deliberate |
| 3 | Does a warm chip left by another process behave? | `configure` writes every register it depends on; `SW_RESET`/`BOOT` are deliberately **not** issued, so a warm chip is as well-defined as a cold one | by construction, not by measurement |
| 4 | Gyro-noise character across temperature | not characterised here; the median filter and the fused quaternion cover the same ground a temperature sweep would | open |

The measured numbers for the board this was developed on — read time, jitter, achieved rate,
FIFO and SFLP word ratios — are in the [README](../README.md#measured). They are what one
board did, not a guarantee about yours; run `bench_imu` and read your own.

---

Next: [bring-up.md](bring-up.md) for wiring and the four-step bench procedure, and
[register-map.md](register-map.md) for the registers written and why.