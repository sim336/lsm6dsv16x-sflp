//! An IMU that answers without refreshing.
//!
//! Derived from `pollen-robotics/microduck`'s `duck-control/src/bus.rs`; see the NOTICE file.
//!
//! This is the fault that has no other symptom: the bus read succeeds, the chip ACKs, the block
//! decodes, and the attitude is simply frozen. Nothing in a gravity vector says how old it is,
//! so a control loop consuming one will happily balance on a reading from ten seconds ago.
//! Comparing each block with its predecessor is the whole detector — the chip's fused output
//! changes in the last bit on every sample at rest, so a byte-identical block means the stream
//! stopped, not that the robot stopped moving.
//!
//! It is split out from the read path so it can be tested without hardware: the fault it
//! describes would otherwise be verifiable only against a broken chip.

use crate::sflp::IMU_BLOCK_LEN;

/// Run of consecutive stale reads at which a caller should say something, rather than merely
/// count.
///
/// 25 reads is half a second at 50 Hz — the same span [`crate::SflpDecoder::ready`] waits for
/// before it will call the chip's output a measurement, and far longer than any ordinary hiccup.
/// Below it a caller should stay quiet on purpose: warning on the very first repeated block is
/// what teaches everyone to ignore the message.
pub const STALE_RUN_WARN: u64 = 25;

/// How stale the stream is, as two counters.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct ImuStale {
    /// Stale reads since startup, cumulative and never reset.
    pub total: u64,
    /// Length of the current unbroken run of stale reads. Any fresh block resets it to zero.
    pub run: u64,
}

/// Detects an IMU that answers without refreshing, by remembering the last block.
#[derive(Debug, Default)]
pub struct StaleImuTracker {
    /// `None` until the first block. A fixed initial value cannot work here: it would have to be
    /// all zeros, and an all-zero block is exactly what a chip whose SFLP table is still empty
    /// sends — scoring a stale read against a predecessor that never existed.
    last: Option<[u8; IMU_BLOCK_LEN]>,
    stale: ImuStale,
}

impl StaleImuTracker {
    /// Records one block and returns the length of the run it belongs to — 0 when the block is
    /// fresh, which is the overwhelmingly common answer.
    pub fn observe(&mut self, block: &[u8; IMU_BLOCK_LEN]) -> u64 {
        if self.last.replace(*block) == Some(*block) {
            self.stale.total = self.stale.total.saturating_add(1);
            self.stale.run = self.stale.run.saturating_add(1);
        } else {
            self.stale.run = 0;
        }
        self.stale.run
    }

    /// The counters as they stand. A method rather than a public field, so the two numbers
    /// cannot drift apart from the one observation that maintains both.
    pub fn stale(&self) -> ImuStale {
        self.stale
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A block that is unique per `n`, so "fresh" is unambiguous.
    fn block(n: u8) -> [u8; IMU_BLOCK_LEN] {
        [n; IMU_BLOCK_LEN]
    }

    /// The very first block has no predecessor to repeat, so it can never be stale. Getting this
    /// wrong puts a 1 in a counter rendered as an alarm on the first tick of every boot.
    #[test]
    fn the_first_block_is_never_stale() {
        let mut t = StaleImuTracker::default();
        assert_eq!(t.observe(&block(0)), 0);
        assert_eq!(t.stale.total, 0);
    }

    /// A healthy chip changes its block every tick, and that has to cost nothing.
    #[test]
    fn fresh_blocks_count_for_nothing() {
        let mut t = StaleImuTracker::default();
        for n in 0..10 {
            assert_eq!(t.observe(&block(n)), 0);
        }
        assert_eq!(t.stale, ImuStale::default());
    }

    /// A single repeated block is remembered forever but is not reported "right now", so
    /// anything that does not repeat has to clear the run.
    #[test]
    fn a_hiccup_is_remembered_in_the_total_but_not_the_run() {
        let mut t = StaleImuTracker::default();
        t.observe(&block(1));
        assert_eq!(t.observe(&block(1)), 1, "the repeat is the first of a run");
        assert_eq!(t.observe(&block(2)), 0, "a fresh block ends the run");
        assert_eq!(t.stale.total, 1);
        assert_eq!(t.stale.run, 0, "the run is 'now', the total is 'ever'");
    }

    /// The threshold has to be reachable, or a genuinely dead IMU is never reported at all.
    #[test]
    fn a_dead_chip_runs_past_the_warning_threshold() {
        let mut t = StaleImuTracker::default();
        t.observe(&block(7));
        let mut run = 0;
        for _ in 0..STALE_RUN_WARN {
            run = t.observe(&block(7));
        }
        assert_eq!(run, STALE_RUN_WARN);
        assert_eq!(t.stale.total, STALE_RUN_WARN);
    }

    /// `total` answers "has this ever happened", not "how bad is it now", so separate episodes
    /// have to add up.
    #[test]
    fn separate_episodes_add_up() {
        let mut t = StaleImuTracker::default();
        for n in 0..3u8 {
            t.observe(&block(n));
            t.observe(&block(n));
        }
        assert_eq!(t.stale.total, 3);
        assert_eq!(t.stale.run, 1, "only the last episode is still current");
    }
}
