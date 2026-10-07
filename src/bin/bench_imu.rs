//! Is the chip on its own I²C bus producing the block the decoder expects — and can reading it
//! fit inside a 50 Hz control tick?
//!
//! ```text
//! bench_imu --bus /dev/i2c-4 --hz 50 --seconds 10
//! ```
//!
//! **`--bus /dev/i2c-4`, not the udev alias, on a fresh board.** `/dev/i2c-imu` is a udev rule
//! (see `docs/bring-up.md`); until it is installed the chip is reached through the bus node
//! itself. The alias is the default because it is the name that survives a device-tree change,
//! and this is the tool to run when it does not exist yet.
//!
//! Three questions, in the order they matter:
//!
//! 1. **Is the chip there?** `WHO_AM_I` = `0x70`, or nothing else in this output is worth
//!    reading. A stranger at the address answers with plausible-looking numbers, so a mismatch
//!    exits before a single register is written.
//! 2. **Did the configuration land?** Every register `configure` writes is read back and printed.
//!    A register write the chip ACKs and then drops is invisible on the bus and leaves the chip in
//!    its reset state; the readback is the whole reason this tool exists, and the reason the
//!    write-shape tests in `src/lib.rs` exist.
//! 3. **What does one tick cost, and is the SFLP block actually fusing?** Read time and period
//!    jitter, against the tick they have to fit in — and the two ratios that separate "the fusion
//!    block is producing" from "it ACKed the enable and did nothing", which produce byte-identical
//!    12-byte blocks.
//!
//! That last number decides the design of the consumer: a read that fits in the tick can be done
//! inside a control loop; one that does not needs its own thread publishing the newest
//! [`ImuData`]. Nothing here guesses which.
//!
//! The `attitude` line is also the one place `MOUNT` is visible, and it is how that constant was
//! measured: a robot standing upright has to print `gravity=[+0.000, +0.000, -1.000]`. Anything
//! else there is the mounting rotation, not the chip.
//!
//! **Exit codes.** Non-zero when `WHO_AM_I` is wrong, when not one SFLP word arrived, or when any
//! tick's read failed — the three ways this can look like it works while it does not.

use std::time::{Duration, Instant};

use anyhow::{Context, Result, bail};
use clap::Parser;
use lsm6dsv16x_sflp::{BoardBodyImu, ConfigRow, FifoReport, ImuData, MOUNT, SflpDecoder, WHO_AM_I};

#[derive(Parser)]
#[command(about = "Read an LSM6DSV16X over I²C and report what it costs")]
struct Args {
    /// The bus the chip is on. `/dev/i2c-imu` once the udev rule is installed; `/dev/i2c-4`
    /// before that.
    #[arg(long, default_value = "/dev/i2c-imu")]
    bus: String,

    /// The chip's 7-bit address — `0x6B` while SA0 is strapped to `Vdd_IO`, `0x6A` if it is tied
    /// low. Hex or decimal.
    #[arg(long, default_value = "0x6B", value_parser = parse_u8)]
    address: u8,

    /// Ticks per second. 50 is a typical control rate, so the number this reports is the number
    /// that has to fit.
    #[arg(long, default_value_t = 50.0)]
    hz: f64,

    /// How long to run. One line per second, so this is also the line count.
    #[arg(long, default_value_t = 10)]
    seconds: u64,
}

/// `--address` takes `0x6B` or `107` alike. The datasheet writes the two addresses in hex, so a
/// bare decimal is the surprising form here rather than the natural one.
fn parse_u8(text: &str) -> Result<u8, String> {
    let parsed = match text.strip_prefix("0x").or_else(|| text.strip_prefix("0X")) {
        Some(hex) => u8::from_str_radix(hex, 16),
        None => text.parse(),
    };
    parsed.map_err(|e| format!("{text:?} is not a byte: {e}"))
}

/// The percentile of an already-sorted slice. `p95` over a one-second window is thirty samples at
/// 50 Hz, which is enough to say whether a hitch is intermittent or constant.
#[cfg(target_os = "linux")]
fn percentile(sorted: &[Duration], fraction: f64) -> Duration {
    if sorted.is_empty() {
        return Duration::ZERO;
    }
    sorted[((sorted.len() - 1) as f64 * fraction).round() as usize]
}

#[cfg(target_os = "linux")]
fn mean(samples: &[Duration]) -> Duration {
    if samples.is_empty() {
        return Duration::ZERO;
    }
    samples.iter().sum::<Duration>() / samples.len() as u32
}

#[cfg(target_os = "linux")]
fn magnitude(v: &[f64]) -> f64 {
    v.iter().map(|x| x * x).sum::<f64>().sqrt()
}

fn main() -> Result<()> {
    run(&Args::parse())
}

#[cfg(target_os = "linux")]
fn run(args: &Args) -> Result<()> {
    use linux_embedded_hal::I2cdev;

    if !args.hz.is_finite() || args.hz <= 0.0 {
        bail!("--hz must be positive, got {}", args.hz);
    }
    if args.seconds == 0 {
        bail!("--seconds 0 would measure nothing");
    }
    let period = Duration::from_secs_f64(1.0 / args.hz);

    let bus = I2cdev::new(&args.bus).with_context(|| {
        format!(
            "cannot open {} — the chip is on /dev/i2c-4 until a udev rule installs the \
             /dev/i2c-imu alias, and the node is root:i2c, so this needs root or the i2c group",
            args.bus
        )
    })?;
    let mut imu = BoardBodyImu::new(bus, args.address);

    let who_am_i = imu.who_am_i().context("WHO_AM_I could not be read")?;
    println!("bus       {} at {:#04x}", args.bus, args.address);
    println!("WHO_AM_I  {who_am_i:#04x} (this part is {WHO_AM_I:#04x})");
    if who_am_i != WHO_AM_I {
        bail!(
            "nothing that is an LSM6DSV16X answered at {:#04x}",
            args.address
        );
    }

    imu.configure().context("the chip did not come up")?;

    println!("\nregister        addr  wrote  read");
    for ConfigRow {
        name,
        address,
        wrote,
        read,
    } in imu.configuration_readback()?
    {
        let flag = if read == wrote {
            ""
        } else {
            "  <- did not land"
        };
        println!("{name:<14} {address:#04x}  {wrote:#04x}   {read:#04x}{flag}");
    }

    // All the µs columns, and what the two ratios count. Printed once rather than in every line,
    // which is where a reader looks for it exactly never.
    println!("\ntimes in µs · fifo = ticks with no word waiting · sflp = ticks with a fused word");
    println!(
        "\n{:>4} {:>7} {:>7} {:>7} {:>7} {:>8} {:>7} {:>9} {:>7} {:>6} {:>5} {:>5}",
        "t",
        "rate",
        "mean",
        "p95",
        "max",
        "jit p95",
        "jit max",
        "fifo",
        "sflp",
        "errors",
        "|g|",
        "|q|"
    );

    let mut decoder = SflpDecoder::new(MOUNT);
    let mut attitude = ImuData::default();

    let start = Instant::now();
    let deadline = start + Duration::from_secs(args.seconds);
    let mut window_start = start;
    let mut window_end = start + Duration::from_secs(1);
    let mut next = start;
    let mut last_tick: Option<Instant> = None;

    // Per window, cleared as each line is printed. The whole run's totals are kept separately: the
    // per-second lines say whether it is steady, the totals say whether it works.
    let mut ticks = 0u32;
    let mut empty = 0u32;
    let mut fused = 0u32;
    let mut errors = 0u32;
    let mut reads: Vec<Duration> = Vec::new();
    let mut jitter: Vec<Duration> = Vec::new();
    let mut total_ticks = 0u64;
    let mut total_fused = 0u64;
    let mut total_errors = 0u64;

    while Instant::now() < deadline {
        // Paced before the work, not after, so a slow read eats into its own slot rather than
        // pushing the whole run later — the same shape as a control loop's tick.
        let now = Instant::now();
        if next > now {
            std::thread::sleep(next - now);
        }
        next += period;

        let tick_start = Instant::now();
        if let Some(previous) = last_tick {
            let delta = tick_start - previous;
            jitter.push(delta.abs_diff(period));
        }
        last_tick = Some(tick_start);

        // The read is what has to fit in the tick: everything else here is arithmetic.
        let mut fifo = FifoReport::default();
        let read_started = Instant::now();
        let block = imu.read_block_with_fifo(&mut fifo);
        reads.push(read_started.elapsed());
        ticks += 1;
        total_ticks += 1;

        match block {
            Ok(block) => {
                attitude = decoder.decode(&block);
                if fifo.waiting == 0 {
                    empty += 1;
                }
                if fifo.sflp {
                    fused += 1;
                    total_fused += 1;
                }
            }
            Err(e) => {
                errors += 1;
                total_errors += 1;
                // Not retried: a tick that failed is a tick the loop would skip, and the next one
                // re-reads everything anyway.
                eprintln!("tick error: {e}");
            }
        }

        if Instant::now() >= window_end {
            let elapsed = window_start.elapsed();
            reads.sort();
            jitter.sort();
            let micros = |d: Duration| d.as_micros();
            let fraction = |n: u32| 100.0 * f64::from(n) / f64::from(ticks.max(1));
            println!(
                "{:>3}s {:>6.1} {:>7} {:>7} {:>7} {:>8} {:>7} {:>8.0}% {:>6.0}% {:>6} {:>5.3} {:>5.3}",
                (window_start - start).as_secs() + 1,
                f64::from(ticks) / elapsed.as_secs_f64(),
                micros(mean(&reads)),
                micros(percentile(&reads, 0.95)),
                micros(*reads.last().unwrap_or(&Duration::ZERO)),
                micros(percentile(&jitter, 0.95)),
                micros(*jitter.last().unwrap_or(&Duration::ZERO)),
                fraction(empty),
                fraction(fused),
                errors,
                magnitude(&attitude.gravity),
                magnitude(&attitude.quat),
            );

            window_start = window_end;
            window_end += Duration::from_secs(1);
            ticks = 0;
            empty = 0;
            fused = 0;
            errors = 0;
            reads.clear();
            jitter.clear();
        }
    }

    let run = start.elapsed();
    println!(
        "\n{} ticks in {:.1}s — {:.1} Hz, {} SFLP words, {} errors, attitude {}",
        total_ticks,
        run.as_secs_f64(),
        total_ticks as f64 / run.as_secs_f64(),
        total_fused,
        total_errors,
        if decoder.ready() {
            "live"
        } else {
            "still the default (fewer than 25 fused samples)"
        },
    );
    // The components, not just the magnitudes above: those say the block decodes to a unit vector,
    // these say which way it points — gravity is `[0, 0, -1]` upright, and the quaternion is
    // scalar-first.
    println!(
        "attitude  gravity=[{:+.3}, {:+.3}, {:+.3}]  quat(wxyz)=[{:+.3}, {:+.3}, {:+.3}, {:+.3}]",
        attitude.gravity[0],
        attitude.gravity[1],
        attitude.gravity[2],
        attitude.quat[0],
        attitude.quat[1],
        attitude.quat[2],
        attitude.quat[3],
    );

    if total_fused == 0 {
        bail!(
            "not one SFLP word in {}s. The enable was accepted — `configure` checks that the bit \
             reads back — so what is left is that the fusion block wrote nothing: the FIFO word \
             the decoder needs carries tag 0x13, and none did",
            args.seconds
        );
    }
    if total_errors > 0 {
        bail!("{total_errors} of {total_ticks} reads failed");
    }

    println!(
        "clean: {} ticks, {total_fused} SFLP words, no errors",
        total_ticks
    );
    Ok(())
}

#[cfg(not(target_os = "linux"))]
fn run(_args: &Args) -> Result<()> {
    bail!("the chip is read over /dev/i2c-*, which exists only on Linux")
}
