//! A bare LSM6DSV16X on I²C, read by the SoC directly.
//!
//! ST's LSM6DSV16X has an on-die sensor-fusion block (SFLP) that fuses the accelerometer and the
//! gyroscope and emits a game-rotation vector. The usual way to get at it is a small MCU that
//! polls the chip and republishes the result on some other bus. This crate is the other route:
//! wire the chip to the SoC's own I²C controller, read the registers yourself, and assemble the
//! **same 12 bytes** the MCU-based board serves, so the choice between the two is invisible to
//! everything downstream.
//!
//! ```no_run
//! # #[cfg(target_os = "linux")]
//! # fn main() -> Result<(), Box<dyn std::error::Error>> {
//! use lsm6dsv16x_sflp::{BoardBodyImu, SflpDecoder, MOUNT};
//!
//! let bus = linux_embedded_hal::I2cdev::new("/dev/i2c-imu")?;
//! let mut imu = BoardBodyImu::new(bus, lsm6dsv16x_sflp::ADDRESS);
//! imu.configure()?;
//!
//! let mut decoder = SflpDecoder::new(MOUNT);
//! loop {
//!     let block = imu.read_block()?;              // 12 bytes, same as the MCU board's
//!     let attitude = decoder.decode(&block);
//!     if decoder.ready() {
//!         println!("gravity = {:?}", attitude.gravity);
//!         break;
//!     }
//! }
//! # Ok(())
//! # }
//! # #[cfg(not(target_os = "linux"))]
//! # fn main() {}
//! ```
//!
//! # The block
//!
//! | bytes | contents |
//! |---|---|
//! | 0..6  | `OUTX_L_G`..`OUTZ_H_G`, `i16` LE raw counts, ±500 dps |
//! | 6..12 | SFLP game-rotation vector x/y/z as IEEE half; `w = √(1 − x² − y² − z²)` |
//!
//! This is the layout [`crate::SflpDecoder`] decodes, and the layout the reference board puts
//! on the wire. Nothing outside the 12 bytes carries a timestamp, a sequence number or a
//! validity flag, which is the point: a consumer cannot tell where they came from.
//!
//! # Three things decide whether that works, and all three fail silently
//!
//! All three are pinned by tests below, because none of them raises an error anywhere:
//!
//! * **The mounting rotation is a property of how the chip is bolted on, not of the chip.**
//!   See [`MOUNT`]. A wrong mount is a robot that reports itself upside down while standing
//!   still, with every register readback agreeing.
//! * **A register write is one `i2c_msg`.** `BodyImu::write_reg` makes one `I2c::write` call,
//!   whose default implementation is a single `Operation::Write`, which `linux-embedded-hal`
//!   turns into one `i2c_msg` whose buffer is the register number followed by the payload. Split
//!   into two messages, this chip ACKs every byte and then drops the payload: the register keeps
//!   its old value, the readback agrees with itself, and the bring-up reports success. That was
//!   a real bug in the first C attempt at this, which is why it is a test and not a comment.
//! * **The gyroscope full scale is ±500 dps.** [`crate::SflpDecoder`] scales counts by
//!   `0.0175 * π/180` rad/s, which is the ±500 dps sensitivity and nothing else. A different
//!   full scale decodes to a wrong rate with no error raised anywhere — and the nearby reference
//!   C driver runs this chip at ±250 dps, twice as sensitive, so copying it is the easy mistake.
//!
//! # Layout
//!
//! * [`BodyImu`] — the driver. Generic over `embedded-hal`'s `I2c`, with no platform dependency.
//! * [`assemble_block`] — the wire format, as a free function so it can be tested in isolation.
//! * [`sflp`] — the decoder, if you do not already have one for this block.
//! * [`stale`] — the "the chip answers but stopped refreshing" fault.
//! * `bench_imu` — the bring-up binary, on Linux only.
//!
//! The register map is written out here rather than taken from a crate. ST publishes one
//! (`lsm6dsv16x-rs`) and it is the first thing to reach for, but every release of it depends on
//! `bisync`, and every release of `bisync` is yanked from crates.io, so it cannot be resolved.
//! What is left is the datasheet and a working reference on real hardware — the sequences below
//! are the datasheet's, with the full scale changed deliberately (see the third bullet) and the
//! filters left out.
//!
//! Deliberately not done here: the chip's `SW_RESET`/`BOOT`, and the gyro LPF1 / accel LPF2 a
//! reference driver may turn on. [`BodyImu::configure`] writes every register it depends on, so
//! a warm chip left over from a previous process is as well-defined as a cold one. And the
//! filter settings of a comparable board are generally not published, so no filter choice made
//! here could be checked against one; the raw counts go to the decoder, which rejects
//! single-sample spikes itself.

#![warn(missing_docs)]

pub mod sflp;
pub mod stale;

pub use sflp::{IMU_BLOCK_LEN, ImuData, SflpDecoder};
pub use stale::{ImuStale, STALE_RUN_WARN, StaleImuTracker};

use embedded_hal::i2c::I2c;

/// `WHO_AM_I` for this part, fixed by it. There is no relative to fall back to.
pub const WHO_AM_I: u8 = 0x70;

/// `WHO_AM_I`'s address, named for the callers that have to seed a bus with it.
pub const WHO_AM_I_REG: u8 = 0x0F;

/// `CTRL6`'s address, which holds `FS_G`. The gyroscope full scale is a contract with
/// [`SflpDecoder`] and nothing else on the bus states it, so the test that reads it back starts
/// from this.
pub const CTRL6_REG: u8 = 0x15;

/// The chip's address, with `SAD = 110101xb` and SA0 tied to `Vdd_IO`.
///
/// SA0 floating is not a third state — it picks up whatever the trace is near, which on one
/// board was the 50 Hz around it and showed up as `ENXIO` from the kernel with no pattern in the
/// data. Tying it to a rail is what makes the address a fact. **Do not scan the bus for this
/// part**: the address is strappable only between `0x6A` and `0x6B`, and a scan can only find
/// the wrong thing.
pub const ADDRESS: u8 = 0x6B;

/// Sensor→trunk mounting rotation for **one specific** bare-chip board, scalar-first. It is not
/// [`SflpDecoder::V2_BOARD_MOUNT`], which describes a different board's chip.
///
/// Trunk = `[−raw_z, +raw_y, +raw_x]`, a **−90° rotation about Y**; `V2_BOARD_MOUNT` is the +90°
/// one, so the two differ by 180° about Y and are not interchangeable. Measuring it takes two
/// known attitudes, because a mounted gyroscope is the one sensor whose resting output is the
/// same no matter how it is turned:
///
/// | pose | gravity, trunk frame | gravity, chip frame | residual |
/// |---|---|---|---|
/// | upright | `[-0.132, +0.102, +0.986]` | `[-0.986, +0.102, -0.132]` | 9.6° |
/// | pitched 90° forward, belly down | `[-0.995, -0.007, -0.103]` | `[+0.103, -0.007, -0.995]` | 5.9° |
///
/// The chip-frame column is the same two readings with `V2_BOARD_MOUNT` undone — a pure property
/// of the chip, independent of what is being fitted to it. Scoring all 24 axis-aligned rotations
/// against "upright → `(0,0,-1)`, pitched → `(1,0,0)`" picks this one at 15.5° total; the
/// runner-up is 90.1° away, so the solution is unique and the residuals are the chip's own tilt
/// on a duck standing on a bench, not ambiguity.
///
/// **Measure your own.** Written out as the solved constant rather than re-derived at runtime:
/// the calibration is a measurement, and a boot that recomputed it would be averaging over
/// whatever the robot happened to be doing. `bench_imu`'s `attitude` line is how it was measured
/// — a robot standing upright has to print `gravity=[+0.000, +0.000, -1.000]`, and anything else
/// there is the mounting rotation. The regression test below pins this value.
pub const MOUNT: [f64; 4] = [
    std::f64::consts::FRAC_1_SQRT_2,
    0.0,
    -std::f64::consts::FRAC_1_SQRT_2,
    0.0,
];

// Main page. Reset/interface, the two ODR registers, the two full scales, the FIFO's batching and
// mode, and the two places a sample is read from.
const FUNC_CFG_ACCESS_REG: u8 = 0x01;
const FIFO_CTRL3_REG: u8 = 0x09;
const FIFO_CTRL4_REG: u8 = 0x0A;
const CTRL1_REG: u8 = 0x10;
const CTRL2_REG: u8 = 0x11;
const CTRL3_REG: u8 = 0x12;
const CTRL8_REG: u8 = 0x17;
const FIFO_STATUS1_REG: u8 = 0x1B;
const OUTX_L_G_REG: u8 = 0x22;
const FIFO_DATA_OUT_TAG_REG: u8 = 0x78;

// Embedded-function page. These three addresses mean something else in the main page, so they are
// only reachable while `FUNC_CFG_ACCESS` has its EMB_FUNC_REG_ACCESS bit set — and the same bit is
// what makes the write land at all rather than silently go nowhere.
const EMB_FUNC_EN_A_REG: u8 = 0x04;
const EMB_FUNC_FIFO_EN_A_REG: u8 = 0x44;
const SFLP_ODR_REG: u8 = 0x5E;

const FUNC_CFG_EMB_ACCESS: u8 = 0x80;
/// `EMB_FUNC_EN_A` / `EMB_FUNC_FIFO_EN_A` bit for the game rotation vector.
const SFLP_GAME_EN: u8 = 0x02;
/// `SFLP_ODR = 0x18`: `SFLP_GAME_ODR = 011` (120 Hz) placed in the field it belongs to, which is
/// bits [5:3] — **not** [2:0], and the register's own reset value reads back as `0x03`, which is
/// what makes the mistake look right.
///
/// `0x03` is the enum value, not the field, and it lands as `SFLP_GAME_ODR = 000` (15 Hz). That
/// is what the nearby reference C driver writes, so its comment's "120 Hz" is 15 Hz in practice;
/// measured on one board, the block delivers 14.8 words/s with the field at `000` and 118 with it
/// at `011`. Nothing about the configured rate is observable from a FIFO that is merely
/// non-empty, which is why this is a constant with its reason attached.
pub const SFLP_ODR_120HZ: u8 = 0x18;
/// `FIFO_MODE[2:0] = 110`: stream/continuous, oldest words dropped when full, so a stalled host
/// loses resolution instead of wedging the FIFO.
const FIFO_MODE_STREAM: u8 = 0x06;
/// `FIFO_MODE[2:0] = 000`, held while the batching rates change.
const FIFO_MODE_BYPASS: u8 = 0x00;
/// `FIFO_CTRL3` with neither raw sensor batched — they are read straight from their registers.
const FIFO_BDR_NONE: u8 = 0x00;
/// `CTRL3`: block data update, and address auto-increment so a burst read walks the map. The
/// reset value already has both, and it is written anyway because [`BodyImu::configure`] may run
/// against a chip another process left in another state.
const CTRL3_BDU_IF_INC: u8 = 0x40 | 0x04;
/// `CTRL1`/`CTRL2`: `OP_MODE = 000` (high performance), `ODR = 0111` (240 Hz).
const ODR_240HZ_HIGH_PERF: u8 = 0x07;
/// `CTRL6`'s `FS_G[3:0] = 0010` — ±500 dps. See the crate comment: this is a contract with
/// [`SflpDecoder`]'s scaling, not a preference.
pub const CTRL6_FS_G_500_DPS: u8 = 0x02;
/// `CTRL8`'s `FS_XL[3:0] = 0010` — ±8 g. Nothing downstream reads the accelerometer; gravity
/// comes from the quaternion, but SFLP fuses both sensors so it has to run.
const CTRL8_FS_XL_8G: u8 = 0x02;
/// One FIFO word: a tag byte plus six data bytes.
const FIFO_WORD_LEN: usize = 7;
/// `FIFO_DATA_OUT_TAG`'s value for the SFLP game rotation vector, in bits [7:3] of the tag.
const FIFO_TAG_SFLP_GAME: u8 = 0x13;

/// How many FIFO words one tick will drain.
///
/// Not a sample-rate question. The gyroscope is read straight from `OUTX_L_G`, so the FIFO carries
/// only fused words, and it delivers them at the rate `SFLP_ODR` is set to — **118 words per
/// second measured** with that field at `011` (120 Hz), against a 50 Hz control tick. So a tick
/// normally finds two or three waiting and this cap is only reached by a loop that has already
/// fallen behind. Draining fewer than are waiting loses nothing: the FIFO is read oldest-first, so
/// the surplus is the freshest words and the next tick collects them.
///
/// What it is for is the stall: the FIFO holds 4.5 KB, and draining an unbounded FIFO inside a
/// control tick would turn a hitch into a longer one. The oldest words are the ones worth
/// dropping.
const MAX_FIFO_WORDS_PER_TICK: u16 = 4;

/// The main-page registers [`BodyImu::configure`] leaves set, in bring-up order: name, address,
/// value.
///
/// The values are the very constants `configure` writes, so a change to one moves both; what is
/// written out here is only the pairing of a register with a name a human can read in a bring-up
/// dump. [`BodyImu::configuration_readback`] exists because the failure this driver guards
/// against — a write the chip ACKs and then drops — is invisible on the bus and leaves every
/// readback agreeing with itself.
const MAIN_PAGE_CONFIG: [(&str, u8, u8); 7] = [
    ("CTRL3", CTRL3_REG, CTRL3_BDU_IF_INC),
    ("CTRL1", CTRL1_REG, ODR_240HZ_HIGH_PERF),
    ("CTRL2", CTRL2_REG, ODR_240HZ_HIGH_PERF),
    ("CTRL6", CTRL6_REG, CTRL6_FS_G_500_DPS),
    ("CTRL8", CTRL8_REG, CTRL8_FS_XL_8G),
    ("FIFO_CTRL3", FIFO_CTRL3_REG, FIFO_BDR_NONE),
    ("FIFO_CTRL4", FIFO_CTRL4_REG, FIFO_MODE_STREAM),
];

/// The embedded-function page registers, kept apart because they are only reachable with
/// `FUNC_CFG_ACCESS.EMB_FUNC_REG_ACCESS` set — at the same addresses in the main page they are
/// different registers, and reading them without the gate would be reading a stranger.
const EMB_PAGE_CONFIG: [(&str, u8, u8); 3] = [
    ("SFLP_ODR", SFLP_ODR_REG, SFLP_ODR_120HZ),
    ("EMB_FUNC_EN_A", EMB_FUNC_EN_A_REG, SFLP_GAME_EN),
    ("EMB_FUNC_FIFO_EN_A", EMB_FUNC_FIFO_EN_A_REG, SFLP_GAME_EN),
];

/// One register's configuration: what `configure` wrote, and what the chip reads back.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ConfigRow {
    /// The register's name in the datasheet's map.
    pub name: &'static str,
    /// Its address, so a dump can be held against the datasheet.
    pub address: u8,
    /// The byte `configure` wrote.
    pub wrote: u8,
    /// The byte the chip answered with afterwards.
    pub read: u8,
}

impl ConfigRow {
    /// Whether the write landed. A `false` here is the silent failure this crate is built
    /// around: the chip ACKed and kept its old value.
    pub fn landed(&self) -> bool {
        self.read == self.wrote
    }
}

/// The FIFO, as one [`BodyImu::read_block`] found it. `None` of this is in the 12-byte block, and
/// a bring-up tool is the only thing that wants it: "no fused word ever arrived" and "the fused
/// word arrives every tick" are the same block, and telling them apart is how a stalled SFLP block
/// is caught before the robot is.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct FifoReport {
    /// Words the FIFO was holding when its status was read, before the per-tick cap.
    pub waiting: u16,
    /// Words drained this tick. Less than `waiting` only when the loop had fallen more than
    /// `MAX_FIFO_WORDS_PER_TICK` words behind.
    pub drained: u16,
    /// Whether one of them carried a live game-rotation vector — the tag was SFLP's and the
    /// payload was not the all-zero "not written yet" sentinel.
    pub sflp: bool,
}

/// Assembles the reference board's 12-byte block from what the chip's registers and FIFO give.
///
/// `sflp_xyz` is `None` on a tick where no fused word was waiting. At 120 Hz of fused words
/// against a 50 Hz tick that is the exception rather than the rule (see
/// `MAX_FIFO_WORDS_PER_TICK`), but a chip still starting up has none for its first tens of
/// milliseconds, so the empty case is not hypothetical. The all-zero quaternion bytes that leaves
/// are exactly the sentinel [`SflpDecoder`] already understands ("hold the last good attitude"),
/// which is how the two ends meet without either knowing about the other. The gyroscope is read
/// fresh every tick either way.
pub fn assemble_block(gyro_raw: [i16; 3], sflp_xyz: Option<[u16; 3]>) -> [u8; IMU_BLOCK_LEN] {
    let mut block = [0u8; IMU_BLOCK_LEN];
    for (axis, counts) in gyro_raw.iter().enumerate() {
        block[2 * axis..2 * axis + 2].copy_from_slice(&counts.to_le_bytes());
    }
    if let Some([x, y, z]) = sflp_xyz {
        block[6..8].copy_from_slice(&x.to_le_bytes());
        block[8..10].copy_from_slice(&y.to_le_bytes());
        block[10..12].copy_from_slice(&z.to_le_bytes());
    }
    block
}

/// Why the chip did not come up.
#[derive(Debug, thiserror::Error)]
pub enum BodyImuError {
    /// The chip at that address is not this part. An unconfigured stranger read through this
    /// register map returns numbers that look like measurements, so refusing is the only honest
    /// answer — there is no default to fall back to.
    #[error("who_am_i read {who_am_i:#04x}, expected {WHO_AM_I:#04x}")]
    NotAnLsm6dsv16x {
        /// The part number the chip actually answered with.
        who_am_i: u8,
    },
    /// `EMB_FUNC_EN_A` did not read back with the SFLP enable bit set. The same failure the
    /// joined-write test guards against, caught on the one register where it is checkable: a
    /// write the chip ACKed and then dropped leaves the fusion block off and the FIFO empty,
    /// which otherwise only shows up as "no attitude, forever".
    #[error(
        "EMB_FUNC_EN_A read back {read_back:#04x} with the SFLP game bit clear after it was set"
    )]
    SflpNotEnabled {
        /// The byte the chip answered with.
        read_back: u8,
    },
    /// The bus, or this driver's view of it. Stringly-typed on purpose: nothing downstream
    /// branches on which bus error it was, and `embedded-hal`'s associated error type would
    /// otherwise have to be threaded through every signature.
    #[error("i2c: {0}")]
    Bus(String),
}

/// A bare LSM6DSV16X on I²C, producing the reference board's 12-byte block.
pub struct BodyImu<P: I2c> {
    i2c: P,
    address: u8,
}

impl<P: I2c> BodyImu<P> {
    /// Wraps an already-open bus. Nothing is written until [`BodyImu::configure`] runs.
    pub fn new(i2c: P, address: u8) -> Self {
        Self { i2c, address }
    }

    /// The bus back, for a caller that shares it with something else.
    pub fn release(self) -> P {
        self.i2c
    }

    /// Writes one register with its value, in **one** I²C message. See the crate comment: this is
    /// the whole reason the bring-up is not a series of `HAL_I2C_Mem_Write`-shaped two-message
    /// writes.
    fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), BodyImuError> {
        self.i2c
            .write(self.address, &[reg, value])
            .map_err(|e| BodyImuError::Bus(format!("{e:?}")))
    }

    /// Reads one register: the register number, a repeated start, then the data. The chip is not
    /// addressed in between, so whatever answers is the register that was asked for.
    fn read_reg(&mut self, reg: u8) -> Result<u8, BodyImuError> {
        let mut byte = [0u8; 1];
        self.read_regs(reg, &mut byte)?;
        Ok(byte[0])
    }

    /// A burst read of consecutive registers, `IF_INC` walking the map.
    fn read_regs(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), BodyImuError> {
        self.i2c
            .write_read(self.address, &[reg], buf)
            .map_err(|e| BodyImuError::Bus(format!("{e:?}")))
    }

    /// The raw part number, as read. Reported rather than checked, so a caller can quote what it
    /// actually saw in the error it raises.
    pub fn who_am_i(&mut self) -> Result<u8, BodyImuError> {
        self.read_reg(WHO_AM_I_REG)
    }

    /// Brings the chip to the configuration [`assemble_block`] expects. Idempotent, and safe on a
    /// chip another process already configured.
    pub fn configure(&mut self) -> Result<(), BodyImuError> {
        let who_am_i = self.who_am_i()?;
        if who_am_i != WHO_AM_I {
            return Err(BodyImuError::NotAnLsm6dsv16x { who_am_i });
        }

        self.write_reg(CTRL3_REG, CTRL3_BDU_IF_INC)?;
        self.write_reg(CTRL1_REG, ODR_240HZ_HIGH_PERF)?;
        self.write_reg(CTRL2_REG, ODR_240HZ_HIGH_PERF)?;
        // The gyroscope scale is not a preference — see the crate comment. Nothing else on this
        // bus says what it is, and getting it wrong is a silent factor of two.
        self.write_reg(CTRL6_REG, CTRL6_FS_G_500_DPS)?;
        self.write_reg(CTRL8_REG, CTRL8_FS_XL_8G)?;

        // The FIFO is emptied before its mode is set: batching rates are not meant to change
        // while the FIFO is running, and on a re-configure it is.
        self.write_reg(FIFO_CTRL4_REG, FIFO_MODE_BYPASS)?;
        self.write_reg(FIFO_CTRL3_REG, FIFO_BDR_NONE)?;

        // SFLP: the fused word is batched, the raw sensors are not. One FIFO word per fused
        // sample keeps the newest quaternion at most one poll old while the raw gyroscope stays
        // unbuffered for the loop to read straight from its registers.
        self.write_reg(FUNC_CFG_ACCESS_REG, FUNC_CFG_EMB_ACCESS)?;
        self.write_reg(SFLP_ODR_REG, SFLP_ODR_120HZ)?;
        self.write_reg(EMB_FUNC_EN_A_REG, SFLP_GAME_EN)?;
        self.write_reg(EMB_FUNC_FIFO_EN_A_REG, SFLP_GAME_EN)?;
        // Read the enable bit back with the gate still open: a write that did not land, or a part
        // that does not fuse, is worth knowing now rather than from an empty FIFO a second later.
        let read_back = self.read_reg(EMB_FUNC_EN_A_REG);
        // Closed on every path, including the failure: a gate left open would make every later
        // register in this sequence mean something else.
        self.write_reg(FUNC_CFG_ACCESS_REG, 0x00)?;
        let read_back = read_back?;
        if read_back & SFLP_GAME_EN == 0 {
            return Err(BodyImuError::SflpNotEnabled { read_back });
        }

        self.write_reg(FIFO_CTRL3_REG, FIFO_BDR_NONE)?;
        self.write_reg(FIFO_CTRL4_REG, FIFO_MODE_STREAM)?;

        Ok(())
    }

    /// Every register [`BodyImu::configure`] writes, read back. For a bring-up tool: the
    /// sequence's whole failure mode is a write the chip ACKs and drops, and reading the map
    /// afterwards is the only way to see it — a readback of one register the sequence happens to
    /// check is already inside `configure` (`EMB_FUNC_EN_A`), and this is the rest.
    ///
    /// The embedded-function page is reached with the access gate open and the gate is closed
    /// again on every path, including one where a read failed: left open, every later register in
    /// this module would mean something else.
    pub fn configuration_readback(&mut self) -> Result<Vec<ConfigRow>, BodyImuError> {
        let mut rows = Vec::with_capacity(MAIN_PAGE_CONFIG.len() + EMB_PAGE_CONFIG.len());
        for (name, address, wrote) in MAIN_PAGE_CONFIG {
            let read = self.read_reg(address)?;
            rows.push(ConfigRow {
                name,
                address,
                wrote,
                read,
            });
        }

        self.write_reg(FUNC_CFG_ACCESS_REG, FUNC_CFG_EMB_ACCESS)?;
        let mut embedded = Vec::with_capacity(EMB_PAGE_CONFIG.len());
        let mut failure = None;
        for (name, address, wrote) in EMB_PAGE_CONFIG {
            match self.read_reg(address) {
                Ok(read) => embedded.push(ConfigRow {
                    name,
                    address,
                    wrote,
                    read,
                }),
                Err(e) => {
                    failure = Some(e);
                    break;
                }
            }
        }
        self.write_reg(FUNC_CFG_ACCESS_REG, 0x00)?;
        if let Some(e) = failure {
            return Err(e);
        }

        rows.extend(embedded);
        Ok(rows)
    }

    /// One tick's 12 bytes: the gyroscope as it stands, and the newest fused quaternion the FIFO
    /// had waiting — zero bytes if none did.
    pub fn read_block(&mut self) -> Result<[u8; IMU_BLOCK_LEN], BodyImuError> {
        self.read_block_with_fifo(&mut FifoReport::default())
    }

    /// [`BodyImu::read_block`], and what the FIFO looked like while it ran.
    ///
    /// Nothing in the block distinguishes "no fused word this tick" from "the fusion block never
    /// started" — both are zero quaternion bytes, deliberately, so that [`SflpDecoder`] holds the
    /// last attitude in either case. That is right for a consumer and useless for a bring-up
    /// tool, which is what this variant is for.
    pub fn read_block_with_fifo(
        &mut self,
        fifo: &mut FifoReport,
    ) -> Result<[u8; IMU_BLOCK_LEN], BodyImuError> {
        *fifo = FifoReport::default();

        let mut gyro_bytes = [0u8; 6];
        self.read_regs(OUTX_L_G_REG, &mut gyro_bytes)?;
        let gyro = [
            i16::from_le_bytes([gyro_bytes[0], gyro_bytes[1]]),
            i16::from_le_bytes([gyro_bytes[2], gyro_bytes[3]]),
            i16::from_le_bytes([gyro_bytes[4], gyro_bytes[5]]),
        ];

        // `FIFO_STATUS1` is `DIFF_FIFO[7:0]` and `FIFO_STATUS2`'s bit 0 is `DIFF_FIFO[8]`; the
        // rest of that byte is flags rather than count, so it is masked down to the one bit. They
        // are adjacent, so one transaction answers "is there anything".
        let mut status = [0u8; 2];
        self.read_regs(FIFO_STATUS1_REG, &mut status)?;
        let waiting = u16::from(status[0]) | (u16::from(status[1] & 0x01) << 8);
        let words = usize::from(waiting.min(MAX_FIFO_WORDS_PER_TICK));
        fifo.waiting = waiting;
        fifo.drained = words as u16;

        let mut newest = None;
        if words > 0 {
            // Exactly as many words as are waiting, up to the cap: the FIFO is the one read on
            // this bus whose cost scales with how far behind the loop is.
            let mut fifo_buf = [0u8; MAX_FIFO_WORDS_PER_TICK as usize * FIFO_WORD_LEN];
            let fifo_buf = &mut fifo_buf[..words * FIFO_WORD_LEN];
            self.read_regs(FIFO_DATA_OUT_TAG_REG, fifo_buf)?;

            for word in fifo_buf.as_chunks::<FIFO_WORD_LEN>().0 {
                // The tag sits in bits [7:3] of the tag byte; the low bits are reserved.
                if word[0] >> 3 != FIFO_TAG_SFLP_GAME {
                    continue;
                }
                // An all-zero payload is SFLP saying it has not written its state yet. An
                // identity quaternion there would report a robot lying on its side as upright,
                // so it is treated as "no sample" and the search goes on. Letting it through as
                // zeros reaches [`SflpDecoder`] as the same sentinel it holds the last good
                // attitude across, so the two agree without either knowing.
                if word[1..FIFO_WORD_LEN] == [0u8; 6] {
                    continue;
                }
                newest = Some([
                    u16::from_le_bytes([word[1], word[2]]),
                    u16::from_le_bytes([word[3], word[4]]),
                    u16::from_le_bytes([word[5], word[6]]),
                ]);
            }
        }
        fifo.sflp = newest.is_some();

        Ok(assemble_block(gyro, newest))
    }
}

/// The bus node type of the board this was developed on, on the only platform that has one.
///
/// `/dev/i2c-imu` is a udev alias for `i2c-4` (see `docs/bring-up.md`); `/dev/i2c-4` works before
/// that rule is installed.
#[cfg(target_os = "linux")]
pub type BoardBodyImu = BodyImu<linux_embedded_hal::I2cdev>;

#[cfg(test)]
mod tests {
    use super::*;
    use core::cell::{Cell, RefCell};
    use embedded_hal::i2c::Operation;
    use std::collections::HashMap;
    use std::rc::Rc;

    /// One `I2c::transaction`, which is what `linux-embedded-hal` turns into `i2c_msg`s — one per
    /// operation, in order. Recording that shape rather than the bytes is the point: a register
    /// write that arrives as two operations is two `i2c_msg`s on the wire, and that is the shape
    /// this chip silently discards.
    #[derive(Debug, Clone, PartialEq)]
    enum Op {
        Write(Vec<u8>),
        Read(usize),
    }

    #[derive(Debug, Clone, PartialEq)]
    struct Xfer {
        address: u8,
        ops: Vec<Op>,
    }

    /// A recording `embedded_hal::i2c::I2c` with as much of the chip's addressing model as the
    /// driver uses: writing `[reg, ..]` moves the address pointer, reading returns that register's
    /// canned bytes.
    ///
    /// Registers nobody seeded answer `0xFF`, not `0x00`, so a read that only the failure paths
    /// depend on terminates on its first answer instead of spinning for the rest of the test run.
    #[derive(Clone, Default)]
    struct Recorder {
        log: Rc<RefCell<Vec<Xfer>>>,
        regs: Rc<RefCell<HashMap<u8, Vec<u8>>>>,
        pointer: Rc<Cell<u8>>,
    }

    impl Recorder {
        fn with_reg(&self, reg: u8, bytes: &[u8]) {
            self.regs.borrow_mut().insert(reg, bytes.to_vec());
        }

        fn xfers(&self) -> Vec<Xfer> {
            self.log.borrow().clone()
        }

        /// Every write of `reg` that carried a payload, in order.
        fn writes_to(&self, reg: u8) -> Vec<u8> {
            self.xfers()
                .iter()
                .filter_map(|x| match x.ops.as_slice() {
                    [Op::Write(bytes)] if bytes.len() == 2 && bytes[0] == reg => Some(bytes[1]),
                    _ => None,
                })
                .collect()
        }

        /// Every write of anything, payload or not — the shape a stranger must never see.
        fn payload_writes(&self) -> Vec<Vec<u8>> {
            self.xfers()
                .iter()
                .flat_map(|x| x.ops.iter())
                .filter_map(|op| match op {
                    Op::Write(bytes) if bytes.len() > 1 => Some(bytes.clone()),
                    _ => None,
                })
                .collect()
        }
    }

    impl embedded_hal::i2c::ErrorType for Recorder {
        type Error = core::convert::Infallible;
    }

    impl I2c for Recorder {
        fn transaction(
            &mut self,
            address: u8,
            operations: &mut [Operation<'_>],
        ) -> Result<(), Self::Error> {
            let mut ops = Vec::with_capacity(operations.len());
            for op in operations.iter_mut() {
                match op {
                    Operation::Write(bytes) => {
                        ops.push(Op::Write(bytes.to_vec()));
                        self.pointer.set(bytes[0]);
                        // A payload write is `[register, value]` and the chip keeps it; a
                        // one-byte write is a register-select and changes no value. Modelling
                        // that is what makes a readback test mean anything at all.
                        if bytes.len() == 2 {
                            self.regs.borrow_mut().insert(bytes[0], vec![bytes[1]]);
                        }
                    }
                    Operation::Read(buf) => {
                        let buf: &mut [u8] = buf;
                        ops.push(Op::Read(buf.len()));
                        let seeded = self.regs.borrow().get(&self.pointer.get()).cloned();
                        for (i, byte) in buf.iter_mut().enumerate() {
                            *byte = match &seeded {
                                Some(src) => src.get(i).copied().unwrap_or(0xFF),
                                None => 0xFF,
                            };
                        }
                    }
                }
            }
            self.log.borrow_mut().push(Xfer { address, ops });
            Ok(())
        }
    }

    fn configured() -> (Recorder, BodyImu<Recorder>) {
        let rec = Recorder::default();
        rec.with_reg(WHO_AM_I_REG, &[WHO_AM_I]);
        let mut imu = BodyImu::new(rec.clone(), ADDRESS);
        imu.configure().expect("a seeded bus must configure");
        (rec, imu)
    }

    /// The sensitivity [`SflpDecoder`] scales a gyroscope count by, measured by decoding a count
    /// rather than by restating its constant: 1000 counts on one axis, read back in mdps/LSB.
    fn decoder_mdps_per_lsb() -> f64 {
        const COUNTS: f64 = 1000.0;
        let mut decoder = SflpDecoder::default();
        let mut block = [0u8; IMU_BLOCK_LEN];
        block[0..2].copy_from_slice(&(COUNTS as i16).to_le_bytes());
        // Three blocks, because the decoder takes a median over three and its history starts at
        // zero — the first two outputs are still settling.
        let mut gyro = [0.0f64; 3];
        for _ in 0..3 {
            gyro = decoder.decode(&block).gyro;
        }
        // The norm, not one axis: the decoder rotates the sample by its mounting quaternion, and
        // a single-axis input does not survive that rotation. The length does.
        let rad_s = (gyro[0] * gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2]).sqrt();
        rad_s.to_degrees() * 1000.0 / COUNTS
    }

    /// The bug the first C attempt at this had, pinned. A register write split across two messages
    /// is ACKed byte by byte and then discarded, so the register keeps its old value and every
    /// readback agrees with it. One transaction, one message, register first.
    #[test]
    fn every_register_write_is_a_single_i2c_message() {
        let (rec, _imu) = configured();
        let xfers = rec.xfers();

        assert!(
            xfers.len() > 8,
            "bring-up wrote almost nothing, so the shape assertion means nothing: {xfers:?}"
        );
        for xfer in &xfers {
            assert_eq!(xfer.address, ADDRESS, "the chip is at SA0 = 1: {xfer:?}");
            assert!(
                !matches!(xfer.ops.as_slice(), [Op::Write(_), Op::Write(_)]),
                "register write split into two messages: {xfer:?}"
            );
        }

        // And a write with a payload really did happen, register byte in front.
        let ctrl6 = rec.writes_to(CTRL6_REG);
        assert_eq!(ctrl6.len(), 1, "CTRL6 writes: {ctrl6:?}");
    }

    /// Reads are the other shape, and the other half of the same constraint: the register address,
    /// a repeated start, then the data.
    #[test]
    fn register_read_is_write_then_read() {
        let rec = Recorder::default();
        rec.with_reg(WHO_AM_I_REG, &[WHO_AM_I]);
        let mut imu = BodyImu::new(rec.clone(), ADDRESS);

        assert_eq!(imu.who_am_i().unwrap(), WHO_AM_I);

        assert_eq!(
            rec.xfers(),
            vec![Xfer {
                address: ADDRESS,
                ops: vec![Op::Write(vec![WHO_AM_I_REG]), Op::Read(1)],
            }]
        );
    }

    /// The one that matters most, because nothing on the wire would say it went wrong:
    /// [`SflpDecoder`]'s scaling is the ±500 dps sensitivity, so the byte written to `FS_G` has to
    /// select ±500 dps — not the ±250 some reference drivers use.
    #[test]
    fn gyro_full_scale_matches_the_decoder() {
        let (rec, _imu) = configured();

        let ctrl6 = rec.writes_to(CTRL6_REG)[0];
        // The datasheet's mdps/LSB for each `FS_G` field, so the expected value comes from the
        // byte that was written rather than from a constant restated here. A ±250 setting
        // therefore fails this test instead of silently halving every rate.
        let fs_g = ctrl6 & 0x0F;
        let expected = match fs_g {
            0x0 => 4.375,
            0x1 => 8.75,
            0x2 => 17.50,
            0x3 => 35.0,
            other => panic!(
                "CTRL6 was written {ctrl6:#04x}; FS_G = {other:#x} is not a defined full scale"
            ),
        };

        let actual = decoder_mdps_per_lsb();
        assert!(
            (actual - expected).abs() < 1e-9,
            "decoder scales gyro at {actual} mdps/LSB, but FS_G = {fs_g:#x} is {expected}"
        );
    }

    /// A tick with no fused word: the gyroscope is fresh and the quaternion bytes are zero, which
    /// the decoder already reads as "hold the last good attitude". Getting this wrong would snap
    /// the robot to upright on every wordless tick — the worst thing to tell fall detection.
    #[test]
    fn no_sflp_word_holds_the_last_quaternion() {
        let mut decoder = SflpDecoder::default();

        // An attitude the chip could have produced: x = 0.125 in half precision.
        let mut live = [0u8; IMU_BLOCK_LEN];
        live[6..8].copy_from_slice(&0x3000u16.to_le_bytes());
        decoder.decode(&live);
        let attitude = decoder.decode(&live).quat;
        assert_ne!(
            attitude,
            [1.0, 0.0, 0.0, 0.0],
            "the block must move something"
        );

        let block = assemble_block([1234, -234, 45], None);
        assert_eq!(
            &block[6..12],
            &[0u8; 6],
            "no fused word must leave the quaternion bytes zero"
        );
        assert_eq!(i16::from_le_bytes([block[0], block[1]]), 1234);

        assert_eq!(
            decoder.decode(&block).quat,
            attitude,
            "a wordless tick must not move the attitude"
        );
    }

    /// The seam this crate is. The same sample has to arrive as the same bytes whether it came off
    /// an MCU board's bus or was assembled from the chip's registers — "one IMU, one code path" is
    /// only true if the layout matches byte for byte.
    ///
    /// The board's bytes are written out literally, the way its layout reads on the wire, so this
    /// cannot pass by both sides sharing a mistake.
    #[test]
    fn assembled_block_decodes_identically_to_the_mcu_board() {
        let board: [u8; IMU_BLOCK_LEN] = [
            0xC9, 0xFE, // gyro x = -311 counts
            0x4D, 0x00, // gyro y = 77
            0x00, 0xF8, // gyro z = -2048
            0x00, 0x30, // quat x = 0.125
            0x00, 0xB0, // quat y = -0.5
            0x00, 0x38, // quat z = 0.5
        ];
        let here = assemble_block([-311, 77, -2048], Some([0x3000, 0xB000, 0x3800]));
        assert_eq!(
            here, board,
            "the two paths must put the same bytes on the wire"
        );

        let mut on_the_bus = SflpDecoder::default();
        let mut over_i2c = SflpDecoder::default();
        for tick in 0..4 {
            let a = on_the_bus.decode(&board);
            let b = over_i2c.decode(&here);
            assert_eq!(a, b, "tick {tick}");
        }
    }

    /// A stranger at the address is an error, not a default. An unrelated part read through this
    /// register map returns numbers that look like measurements, so a fallback would be worse than
    /// a refusal — and it is refused before a single register is written.
    #[test]
    fn who_am_i_must_be_0x70() {
        let rec = Recorder::default();
        rec.with_reg(WHO_AM_I_REG, &[0x6A]); // a plausible-looking neighbour
        let mut imu = BodyImu::new(rec.clone(), ADDRESS);

        assert_eq!(
            imu.who_am_i().unwrap(),
            0x6A,
            "the raw read is reported as-is"
        );

        match imu.configure() {
            Err(BodyImuError::NotAnLsm6dsv16x { who_am_i }) => assert_eq!(who_am_i, 0x6A),
            Ok(()) => panic!("configure accepted a chip it should have refused"),
            Err(other) => panic!("expected a refusal, got {other}"),
        }

        assert!(
            rec.payload_writes().is_empty(),
            "a chip that was not identified was written to: {:?}",
            rec.payload_writes()
        );
    }

    /// What a bring-up tool prints, checked against what `configure` wrote. The mock keeps payload
    /// writes, so this is the real question — "did the write land" — which the wire cannot answer:
    /// a dropped write leaves every readback agreeing with its own old value.
    #[test]
    fn configuration_reads_back_as_written() {
        let (rec, mut imu) = configured();

        let rows = imu
            .configuration_readback()
            .expect("a seeded bus must read back");

        assert_eq!(
            rows.len(),
            MAIN_PAGE_CONFIG.len() + EMB_PAGE_CONFIG.len(),
            "every register configure writes must appear: {rows:?}"
        );
        for row in &rows {
            assert_eq!(row.read, row.wrote, "mismatch in {row:?}");
            assert!(row.landed());
        }
        assert_eq!(rec.writes_to(CTRL6_REG), vec![CTRL6_FS_G_500_DPS]);

        // The embedded-function page was reached with the gate open, and the gate is shut again
        // afterwards — left open, `CTRL1` and the rest would mean something else.
        assert_eq!(
            rec.writes_to(FUNC_CFG_ACCESS_REG),
            vec![FUNC_CFG_EMB_ACCESS, 0x00, FUNC_CFG_EMB_ACCESS, 0x00],
            "configure opens and closes it, and so must the readback"
        );
    }

    /// The FIFO report distinguishes two ticks whose 12-byte blocks are identical: a live fused
    /// word, and a FIFO with nothing in it. Everything downstream is deliberately blind to the
    /// difference, so this is the only place it is visible.
    #[test]
    fn fifo_report_tells_a_live_word_from_an_empty_fifo() {
        let rec = Recorder::default();
        rec.with_reg(WHO_AM_I_REG, &[WHO_AM_I]);
        let mut imu = BodyImu::new(rec.clone(), ADDRESS);
        imu.configure().expect("a seeded bus must configure");

        // One word waiting: the tag byte for SFLP's game vector — the tag sits in bits [7:3] —
        // then three half-precision components.
        rec.with_reg(FIFO_STATUS1_REG, &[1, 0]);
        rec.with_reg(
            FIFO_DATA_OUT_TAG_REG,
            &[FIFO_TAG_SFLP_GAME << 3, 0x00, 0x30, 0x00, 0xB0, 0x00, 0x38],
        );

        let mut report = FifoReport::default();
        let block = imu.read_block_with_fifo(&mut report).unwrap();
        assert_eq!(
            report,
            FifoReport {
                waiting: 1,
                drained: 1,
                sflp: true
            }
        );
        assert_eq!(&block[6..12], &[0x00, 0x30, 0x00, 0xB0, 0x00, 0x38]);

        // An empty FIFO: no words read, no fused sample, quaternion bytes zero — which
        // `SflpDecoder` reads as "hold the last attitude".
        rec.with_reg(FIFO_STATUS1_REG, &[0, 0]);
        let block = imu.read_block_with_fifo(&mut report).unwrap();
        assert_eq!(report, FifoReport::default());
        assert_eq!(&block[6..12], &[0u8; 6]);

        // A word that is not SFLP's — the raw sensors are not batched here, but a tag the driver
        // does not know must not be mistaken for an attitude.
        rec.with_reg(FIFO_STATUS1_REG, &[1, 0]);
        rec.with_reg(
            FIFO_DATA_OUT_TAG_REG,
            &[0x01 << 3, 0x00, 0x30, 0x00, 0xB0, 0x00, 0x38],
        );
        let block = imu.read_block_with_fifo(&mut report).unwrap();
        assert_eq!(
            report,
            FifoReport {
                waiting: 1,
                drained: 1,
                sflp: false
            }
        );
        assert_eq!(&block[6..12], &[0u8; 6]);
    }

    /// The cap, which is not reachable by a loop that is keeping up and is the whole reason it
    /// exists: a stalled host has to drain a bounded number of words, not 4.5 KB worth.
    #[test]
    fn a_backlog_is_drained_up_to_the_cap() {
        let rec = Recorder::default();
        rec.with_reg(WHO_AM_I_REG, &[WHO_AM_I]);
        let mut imu = BodyImu::new(rec.clone(), ADDRESS);
        imu.configure().expect("a seeded bus must configure");

        // 40 words waiting, 4 drained. The oldest are the ones dropped, so the newest word in the
        // buffer is the one that wins.
        rec.with_reg(FIFO_STATUS1_REG, &[40, 0]);
        let mut words = Vec::new();
        for word in 0..MAX_FIFO_WORDS_PER_TICK {
            let x = 0x3000u16 + word;
            words.push(FIFO_TAG_SFLP_GAME << 3);
            words.extend_from_slice(&x.to_le_bytes());
            words.extend_from_slice(&0u16.to_le_bytes());
            words.extend_from_slice(&0u16.to_le_bytes());
        }
        rec.with_reg(FIFO_DATA_OUT_TAG_REG, &words);

        let mut report = FifoReport::default();
        let block = imu.read_block_with_fifo(&mut report).unwrap();
        assert_eq!(report.waiting, 40);
        assert_eq!(report.drained, MAX_FIFO_WORDS_PER_TICK);
        let last = (0x3000u16 + MAX_FIFO_WORDS_PER_TICK - 1).to_le_bytes();
        assert_eq!(&block[6..8], &last, "the newest word must win");
    }

    /// [`MOUNT`], pinned. The failure it guards against is a robot that reports itself upside down
    /// while every readback on the bus agrees, so both ends of the calibration are asserted: the
    /// attitude the measurement saw, and the trunk-frame gravity the decoder owes it.
    ///
    /// The block is the upright pose reduced to what was measured — chip-frame gravity along −X —
    /// which is the quaternion `[√½, 0, −√½, 0]`, a −90° rotation about Y. Feeding it through both
    /// mounts is the whole point: this crate's returns upright, the MCU board's returns the mirror
    /// image, and neither raises an error anywhere.
    #[test]
    fn mount_reports_the_upright_calibration_as_upright() {
        // y = −0.70703125, the half-precision nearest −√½; x and z stay zero.
        let mut block = [0u8; IMU_BLOCK_LEN];
        block[8..10].copy_from_slice(&0xB9A8u16.to_le_bytes());

        // Three blocks: the decoder takes a median over three and seeds its history upright, so
        // the first two outputs are still settling.
        let gravity_under = |mount: [f64; 4]| {
            let mut decoder = SflpDecoder::new(mount);
            let mut gravity = [0.0f64; 3];
            for _ in 0..3 {
                gravity = decoder.decode(&block).gravity;
            }
            gravity
        };

        let ours = gravity_under(MOUNT);
        assert!(
            ours[2] < -0.99 && ours[0].abs() < 0.05 && ours[1].abs() < 0.05,
            "this chip's mount must call the upright calibration upright: {ours:?}"
        );

        // The MCU board's mount is 180° about Y away from this one, which is exactly the mistake
        // this constant exists to prevent — and it is a sign flip, not an error.
        let mcu_board = gravity_under(SflpDecoder::V2_BOARD_MOUNT);
        assert!(
            mcu_board[2] > 0.99,
            "V2_BOARD_MOUNT describes the other chip's mounting; if it agreed here, the two chips \
             would be at the same angle and this crate would need no constant: {mcu_board:?}"
        );
    }
}
