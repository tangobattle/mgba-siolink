//! Anchors MAX_SLICES_PER_TICK to reality: measure the peak run_loop
//! slices a real tick takes on a maximally chatty link (the test ROM
//! exchanges over MULTI every frame), across boot and steady state, and
//! assert the cap keeps a wide margin above it. If a legitimate workload
//! ever approaches the cap, this fails before any user's session does.

use mgba_rollback::{testrom, Link, MAX_SLICES_PER_TICK};

#[test]
fn real_ticks_fit_far_under_the_slice_cap() {
    mgba::log::install_default_logger();
    for num_players in [2, 4] {
        let rom = testrom::build();
        let mut link = Link::new(vec![rom; num_players]).unwrap();
        let mut peak = 0;
        for t in 0..600u32 {
            let keys: Vec<u32> = (0..num_players as u32)
                .map(|p| (t * (0x11 + p * 6)) & 0x3ff)
                .collect();
            let slices = link.try_tick(&keys).unwrap();
            peak = peak.max(slices);
        }
        eprintln!("{num_players}p peak slices/tick over 600 ticks: {peak}");
        // 20x headroom over the chattiest thing we can produce keeps the
        // cap an honest livelock detector, not a latent workload limit.
        assert!(
            peak * 20 < MAX_SLICES_PER_TICK,
            "{num_players}p peak {peak} is within 20x of the {MAX_SLICES_PER_TICK} cap"
        );
    }
}
