# Register map

Every register this crate touches, the value written, and the reason. Nothing here is copied
from a vendor driver: the map is transcribed from the LSM6DSV16X datasheet, and the sequences
are the datasheet's, with the gyroscope full scale changed on purpose (§`CTRL6`) and the
filters left out.

Two pages are involved, and the same address means different registers on each:

- the **main page**, always reachable;
- the **embedded-function page**, reachable **only** while `FUNC_CFG_ACCESS` has its
  `EMB_FUNC_REG_ACCESS` bit set. The same bit is what makes a write land at all rather than
  silently go nowhere.

The constants live in two places and are kept one-to-one: [`src/lib.rs`](../src/lib.rs) (Rust)
and the top of [`tools/body_imu_probe.py`](../tools/body_imu_probe.py) (Python). **Change
one, change both.**

## What `configure` writes

In bring-up order. "Value" is the byte put on the wire; "read back" is what the same
register must answer afterwards.

### Main page

| register | addr | value | bits | why |
|---|---|---|---|---|
| `CTRL3` | `0x12` | `0x44` | `BDU=1`, `IF_INC=1` | **Block data update**: a burst read returns one coherent sample, not a mix of two. **Address auto-increment**: a burst read walks the map. The reset value already has both; it is written anyway because `configure` may run against a chip another process left in another state. |
| `CTRL1` | `0x10` | `0x07` | `OP_MODE=000` (high perf), `ODR=0111` (240 Hz) | The accelerometer has to run: SFLP fuses both sensors. 240 Hz is comfortably above the 120 Hz fused rate. |
| `CTRL2` | `0x11` | `0x07` | same as `CTRL1` | The gyroscope, same reasoning. This is also the sensor read **unbuffered** every tick from `OUTX_L_G`. |
| `CTRL6` | `0x15` | `0x02` | `FS_G=0010` | **±500 dps — a contract with the decoder, not a preference.** `SflpDecoder` scales counts by `0.0175 × π/180` rad/s, which is this sensitivity and nothing else. See §4 of [design.md](design.md#4-500-dps-is-a-contract-with-the-decoder-not-a-preference). |
| `CTRL8` | `0x17` | `0x02` | `FS_XL=0010` | ±8 g. Nothing downstream reads the accelerometer directly — gravity comes from the quaternion — but SFLP fuses it, so it has to run. ±8 g is the range that survives a small robot landing hard. |
| `FIFO_CTRL3` | `0x09` | `0x00` | `BDR_GY=000`, `BDR_XL=000` | **Neither raw sensor is batched.** They are read straight from their registers every tick, so the FIFO carries only fused words — one word per fused sample, which keeps the newest quaternion at most one poll old. |
| `FIFO_CTRL4` | `0x0A` | `0x06` (running) / `0x00` (while batching changes) | `FIFO_MODE=110` stream / `000` bypass | **Stream**: continuous, oldest words dropped when full, so a stalled host loses resolution instead of wedging the FIFO. **Bypass** is held while the batching rates change — they are not meant to change while the FIFO is running, and on a re-configure they are. |

### Embedded-function page

Reached with `FUNC_CFG_ACCESS` (`0x01`) = `0x80`, and the gate is **closed again on every
path**, including the failure path: left open, every later register in the sequence would
mean something else.

| register | addr | value | bits | why |
|---|---|---|---|---|
| `SFLP_ODR` | `0x5E` | `0x18` | `SFLP_GAME_ODR=011` in bits **`[5:3]`** | 120 Hz fused output. **The field is `[5:3]`, not `[2:0]`** — see below. |
| `EMB_FUNC_EN_A` | `0x04` | `0x02` | SFLP game-rotation enable | Turns the fusion block on. Read straight back (with the gate still open) and **refused** if the bit did not land: a write the chip ACKs and drops leaves the block off and the FIFO empty, and the only other symptom is "no attitude, forever". |
| `EMB_FUNC_FIFO_EN_A` | `0x44` | `0x02` | SFLP game-rotation → FIFO | Puts the fused word *in the FIFO* rather than discarding it. Enabling the block without this produces a fusing chip and an empty FIFO. |
| `FUNC_CFG_ACCESS` | `0x01` | `0x80` then `0x00` | `EMB_FUNC_REG_ACCESS` | Opens the embedded-function page, then shuts it. Opened in `configure`, and opened and shut again in the readback — four writes in total across a busy run. |

## `SFLP_ODR = 0x18`, and why `0x03` is wrong

The mistake is specific and looks right:

- `SFLP_GAME_ODR = 011` (120 Hz) sits in bits **`[5:3]`** of `SFLP_ODR`. Not `[2:0]`.
- The register's **reset value reads back as `0x03`**, which as an enum value looks like the
  right choice — but `0x03` puts `011` in `[2:0]`, i.e. `000` in the real field, i.e. 15 Hz.
- A comparable project's reference C driver writes exactly that, so its comment's "120 Hz"
  is **15 Hz in practice**.

Measured on the board this crate was developed on: **118 words/s** with the field at `011`,
**14.8 words/s** with it at `000`. A FIFO that is merely non-empty cannot tell those apart —
which is why the bring-up tool reports a *rate*, not a yes/no.

## Registers read but not written

| register | addr | what it is | read by |
|---|---|---|---|
| `WHO_AM_I` | `0x0F` | fixed `0x70`; there is no relative to fall back to | `configure` (and it refuses if wrong) |
| `OUTX_L_G` | `0x22` | gyro x/y/z, three `i16` LE, six bytes | `read_block`, every tick |
| `FIFO_STATUS1` | `0x1B` | `DIFF_FIFO[7:0]`, with `FIFO_STATUS2`'s bit 0 as `DIFF_FIFO[8]` | `read_block_with_fifo`. The two bytes are adjacent so one transaction answers "is there anything"; the rest of `FIFO_STATUS2` is flags, not count, and is masked down to the one bit. |
| `FIFO_DATA_OUT_TAG` | `0x78` | the FIFO: one **7-byte** word per entry (tag + six data bytes) | `read_block_with_fifo` |

### The FIFO word

```text
byte 0    tag          ── the SFLP game vector lives in bits [7:3] and equals 0x13
byte 1..3 x, IEEE half (u16 LE)
byte 3..5 y, IEEE half (u16 LE)
byte 5..7 z, IEEE half (u16 LE)
```

Two rules that are not obvious:

- the tag is in **bits `[7:3]`**, so the comparison is `word[0] >> 3 == 0x13`. A word whose
  tag says otherwise is **skipped**, not guessed at.
- an **all-zero payload** is SFLP saying "not written yet" and is skipped too. Passing it
  through as an identity quaternion would report a robot lying on its side as upright.

`MAX_FIFO_WORDS_PER_TICK = 4` bounds the drain inside a tick. The FIFO is read
oldest-first, so the surplus that is dropped is the *newest* words — the next tick collects
them.

## The write shape

Every write in this crate is **one joined I²C message**: register number and data in the
same buffer.

| shape | bytes on the wire | result |
|---|---|---|
| joined (**correct**) | `START addr reg data… STOP` | the register takes the value |
| split (**broken**) | `START addr reg RESTART addr data… STOP` | the chip ACKs every byte, then **drops** the payload |

The split shape is the trap this whole crate is arranged around: it leaves the register in
its reset state, every readback agreeing with itself, and the bring-up green. It is correct
here **by construction** — `embedded-hal` 1.0's default `I2c::write` is one
`Operation::Write`, which `linux-embedded-hal` turns into one `i2c_msg` — and pinned by
tests. See §3 of [design.md](design.md#3-a-register-write-is-one-i2c_msg-and-it-fails-silently).

## Not configured here

- **`SW_RESET` / `BOOT`**: deliberately not issued. `configure` writes every register it
  depends on, so a warm chip left by a previous process is as well-defined as a cold one.
- **The filters** (gyro LPF1, accel LPF2): left off. A comparable board's filter settings
  are generally not published, so no filter choice here could be checked against one; the
  raw counts go to the decoder, which rejects single-sample spikes itself.

---

Back to [design.md](design.md) · [bring-up.md](bring-up.md) · [README](../README.md).