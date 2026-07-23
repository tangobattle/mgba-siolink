//! Experimental generic rollback netplay over emulated SIO (link cable).
//!
//! Instead of per-game traps that replace a game's link protocol with
//! memory-level input exchange, all of the GBAs on the cable (two to four)
//! run locally as a *link* connected through mgba's lockstep SIO driver,
//! and the link is the rollback unit: the only true inputs are the
//! joypads, everything on the wire is derived deterministically. A netplay
//! session runs the same `Link` on every peer, feeds confirmed local +
//! predicted remote keys into `tick`, and restores a `Snapshot` to
//! re-simulate when a prediction turns out wrong.
//!
//! A link of ONE is also valid: for a cable, a single core with no SIO
//! driver — mgba's model of a GBA with nothing plugged in. Together with
//! [`Link::capture_boot_state`] and [`Link::from_states`] that makes the
//! cable itself dynamic — a solo machine runs until peers appear, the
//! captures are exchanged, and every peer rebuilds the full link mid-game
//! (the cable plugs in); when the session ends, the local capture continues
//! as a solo link again (the cable unplugs).
//!
//! A link may instead be built around the wireless adapter
//! ([`Peripheral::Wireless`]): every core gets its own emulated AGB-015
//! plugged into its link port, and the adapters share one airwaves
//! coordinator that synchronizes the cores only at a coarse RF tick. A
//! solo wireless link still installs the driver — a lone wireless game
//! talks to its adapter (login, broadcast, scan) even with nobody else on
//! the airwaves. A wireless capture carries its adapter session AND its
//! seat ([`Link::capture_adapter_state`]), so a link rebuilt mid-game
//! resumes every adapter where it was, under the identity it had:
//! membership changes never renumber a surviving adapter, so the
//! connections among survivors ride through a rebuild and only pairings
//! with a departed player drop (as an in-range disconnect games handle).
//!
//! The cores are interleaved cooperatively on ONE thread (see
//! `mgba::sio`): a tick runs whichever cores the lockstep protocol has not
//! parked, one `run_loop` timing slice at a time, until the reference core
//! (index 0) finishes one video frame. The other cores float inside the
//! lockstep drift window and may be mid-frame or parked at a tick boundary;
//! that partial progress is exactly captured by the link snapshot, so the
//! interleave replays identically after a restore.

pub mod session;
pub mod testrom;
pub mod throttler;

/// The most players a cable link supports — mgba's `MAX_GBAS`, the size
/// of a real multi-cable chain (the SIO multi protocol has two player-id
/// bits; four is physical law).
pub const MAX_CABLE_PLAYERS: usize = 4;

/// The most players a wireless link supports. The airwaves themselves
/// are uncapped — any number of ≤5-player RFU host groups can share
/// spectrum, union-room style — but the coordinator parks players on a
/// 64-bit bitmask (`WL_MAX_ATTACHED` in wireless.c), so one link holds
/// at most 63 GBAs.
pub const MAX_WIRELESS_PLAYERS: usize = 63;

/// The most players any link supports (the wireless bound; cable links
/// cap lower — see [`Peripheral::max_players`]).
pub const MAX_PLAYERS: usize = MAX_WIRELESS_PLAYERS;

// GBA io block indices (register address >> 1), for the boot-capture
// cable neutralization.
const REG_SIOCNT: usize = 0x128 >> 1;
const REG_RCNT: usize = 0x134 >> 1;

/// Which core a tick treats as the frame-boundary reference. Player 0 is
/// also the lockstep clock owner (primary).
const REFERENCE: usize = 0;

/// Upper bound on run_loop slices per tick, turning a lockstep livelock
/// (which would otherwise spin forever) into a [`Link::try_tick`] error.
/// Measured peaks on a maximally chatty link (MULTI exchange every frame,
/// boot included — see the slice_budget test): ~1.6K slices for 2 players,
/// ~3.2K for 4. 100K keeps ~30x headroom over that, while a *grinding*
/// corrupt state errors out in well under a second instead of pegging a
/// core for minutes below the cap (the old 2M cap read as a silent
/// freeze: near-no-op slices ground for ages and the panic rarely fired).
pub const MAX_SLICES_PER_TICK: usize = 100_000;

pub struct Link {
    // Declaration order is drop order, and it matters: a core's deinit
    // calls back into its SIO driver, and detaching a driver touches the
    // coordinator.
    cores: Vec<mgba::core::OwnedCore>,
    drivers: Vec<Driver>,
    #[allow(dead_code)]
    coordinator: Coordinator,
}

/// Which link hardware a [`Link`]'s cores are wired to.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum Peripheral {
    /// The multi-cable, through mgba's lockstep SIO driver.
    #[default]
    Cable,
    /// The wireless adapter (AGB-015), through mgba's wireless SIO
    /// driver: one emulated adapter per core, one shared airwaves.
    Wireless,
}

impl Peripheral {
    /// The most players a link built on this peripheral holds.
    pub fn max_players(self) -> usize {
        match self {
            Peripheral::Cable => MAX_CABLE_PLAYERS,
            Peripheral::Wireless => MAX_WIRELESS_PLAYERS,
        }
    }
}

// Held for ownership only (the drivers reference it until they drop).
#[allow(dead_code)]
enum Coordinator {
    Cable(mgba::sio::Coordinator),
    Wireless(mgba::sio::wireless::Coordinator),
}

enum Driver {
    Cable(mgba::sio::Driver),
    Wireless(mgba::sio::wireless::Driver),
}

impl Driver {
    fn install(&mut self, core: &mut mgba::core::Core) {
        match self {
            Driver::Cable(d) => d.install(core),
            Driver::Wireless(d) => d.install(core),
        }
    }

    fn asleep(&self) -> bool {
        match self {
            Driver::Cable(d) => d.asleep(),
            Driver::Wireless(d) => d.asleep(),
        }
    }

    fn player_id(&self) -> i32 {
        match self {
            Driver::Cable(d) => d.player_id(),
            Driver::Wireless(d) => d.player_id(),
        }
    }

    fn save_state(&mut self) -> Vec<u8> {
        match self {
            Driver::Cable(d) => d.save_state(),
            Driver::Wireless(d) => d.save_state(),
        }
    }

    fn load_state(&mut self, blob: &[u8]) -> bool {
        match self {
            Driver::Cable(d) => d.load_state(blob),
            Driver::Wireless(d) => d.load_state(blob),
        }
    }

    fn save_adapter_state(&mut self) -> Option<Vec<u8>> {
        match self {
            Driver::Cable(_) => None,
            Driver::Wireless(d) => Some(d.save_adapter_state()),
        }
    }

    fn load_adapter_state(&mut self, blob: &[u8]) -> bool {
        match self {
            Driver::Cable(_) => false,
            Driver::Wireless(d) => d.load_adapter_state(blob),
        }
    }
}

/// Build the coordinator plus one driver per player for the chosen
/// peripheral. A solo cable installs no driver at all (a GBA with nothing
/// plugged in); a solo wireless link still gets its adapter.
///
/// `seats` has one entry per player: the wireless seat (playerId) each
/// adapter asks for, `-1` for any free one. A fresh link passes
/// positions; a rebuild passes each capture's recorded seat so every
/// adapter keeps its airwaves identity (see [`Link::from_states`]).
/// Cable player ids are positional regardless.
fn build_drivers(peripheral: Peripheral, seats: &[i32]) -> (Coordinator, Vec<Driver>) {
    match peripheral {
        Peripheral::Cable => {
            let mut coordinator = mgba::sio::Coordinator::new();
            let drivers = if seats.len() > 1 {
                (0..seats.len())
                    .map(|i| Driver::Cable(mgba::sio::Driver::new(&mut coordinator, i as i32)))
                    .collect()
            } else {
                Vec::new()
            };
            (Coordinator::Cable(coordinator), drivers)
        }
        Peripheral::Wireless => {
            let mut coordinator = mgba::sio::wireless::Coordinator::new();
            let drivers = seats
                .iter()
                .map(|&seat| Driver::Wireless(mgba::sio::wireless::Driver::new(&mut coordinator, seat)))
                .collect();
            (Coordinator::Wireless(coordinator), drivers)
        }
    }
}

/// A consistent snapshot of the whole linked system: every core plus every
/// lockstep driver blob (the coordinator's shared state rides in player
/// 0's blob). Core savestates alone are NOT sufficient — the lockstep
/// event queues, sleep flags, and in-flight transfer data live outside
/// them.
pub struct Snapshot {
    cores: Vec<Box<mgba::state::State>>,
    drivers: Vec<Vec<u8>>,
    /// Each core's direct-sound FIFO channels (A, B), captured verbatim
    /// because the core savestate's encoding is lossy; see
    /// [`audio_fifo_state`].
    audio_fifos: Vec<[FifoLane; 2]>,
    /// Each core's internal DMA control state (all four channels), captured
    /// verbatim because the core savestate reconstructs it from the io
    /// block, which diverges from the truth mid-FIFO-refill; see
    /// [`dma_state`].
    dmas: Vec<[DmaLane; 4]>,
}

/// Raw image of one direct-sound FIFO channel (`GBAAudioFIFO`): ring
/// contents, absolute ring pointers, and the internal sample countdown.
#[derive(Clone, Copy, PartialEq, Eq)]
struct FifoLane {
    fifo: [u32; 8],
    write: i32,
    read: i32,
    internal_remaining: i32,
}

/// Raw image of one DMA channel's control state (`GBADMA`): the internal
/// control register plus the values derived from it.
#[derive(Clone, Copy, PartialEq, Eq)]
struct DmaLane {
    reg: u16,
    cycles: i32,
    source_offset: i32,
    dest_offset: i32,
}

impl Snapshot {
    /// Number of players (cores) this snapshot covers.
    pub fn num_players(&self) -> usize {
        self.cores.len()
    }

    pub fn core_state(&self, i: usize) -> &mgba::state::State {
        &self.cores[i]
    }

    pub fn driver_blob(&self, i: usize) -> &[u8] {
        &self.drivers[i]
    }

    /// Mutable view of a driver blob, for tests that manufacture
    /// hard-to-reach lockstep states through the serialized form (the
    /// same bytes a rollback restore replays); see the wedge repros in
    /// this file's tests and the clock-wrap test in tests/wireless.rs.
    pub fn driver_blob_mut(&mut self, i: usize) -> &mut Vec<u8> {
        &mut self.drivers[i]
    }

    /// Digest of the rollback-relevant state, comparable across peers
    /// simulating the same link (the desync canary). Deliberately built
    /// from discrete savestate fields rather than raw state bytes: mgba
    /// serializes into an uninitialized buffer without touching reserved
    /// regions, so whole-struct bytes are not comparable. CPU registers
    /// plus both RAMs plus the lockstep blobs expose any trajectory
    /// divergence within a tick or two.
    pub fn digest(&self) -> u32 {
        let mut h = crc32fast::Hasher::new();
        for i in 0..self.cores.len() {
            let s = self.core_state(i);
            for r in 0..16 {
                h.update(&s.gpr(r).to_le_bytes());
            }
            h.update(&s.cpsr().to_le_bytes());
            h.update(s.wram());
            h.update(s.iwram());
            if let Some(blob) = self.drivers.get(i) {
                h.update(blob);
            }
            for lane in &self.audio_fifos[i] {
                for w in &lane.fifo {
                    h.update(&w.to_le_bytes());
                }
                h.update(&lane.write.to_le_bytes());
                h.update(&lane.read.to_le_bytes());
                h.update(&lane.internal_remaining.to_le_bytes());
            }
            for dma in &self.dmas[i] {
                h.update(&dma.reg.to_le_bytes());
                h.update(&dma.cycles.to_le_bytes());
                h.update(&dma.source_offset.to_le_bytes());
                h.update(&dma.dest_offset.to_le_bytes());
            }
        }
        h.finalize()
    }
}

/// Per-core boot configuration beyond the ROM itself.
#[derive(Default)]
pub struct SideOptions {
    pub rom: Vec<u8>,
    /// SRAM/flash image, if resuming from an existing save.
    pub save: Option<Vec<u8>>,
}

#[derive(Default)]
pub struct LinkOptions {
    /// One entry per player, 1 to [`MAX_PLAYERS`]. Core `i` runs `sides[i]`
    /// and requests player `i`. A single CABLE side boots with no SIO
    /// driver at all — mgba's model of a GBA with nothing plugged in; a
    /// single wireless side still gets its adapter.
    pub sides: Vec<SideOptions>,
    /// Pin every cart's RTC to a fixed clock. Mandatory for netplay/replay
    /// of RTC-bearing games (e.g. BN4.5): all peers must negotiate the
    /// same match clock or the link diverges on the first RTC read.
    pub rtc: Option<std::time::SystemTime>,
    /// The link hardware on every core's port. All sides share one
    /// peripheral; a mixed link is not a thing that exists.
    pub peripheral: Peripheral,
}

/// One side of a link booted from a live capture instead of power-on: the
/// ROM, the SRAM/flash image at capture time, the serialized core state
/// from [`Link::capture_boot_state`], and — for a wireless link — the
/// adapter session from [`Link::capture_adapter_state`], so the machine's
/// adapter resumes where it was instead of power-cycling.
pub struct BootSide {
    pub rom: Vec<u8>,
    pub save: Option<Vec<u8>>,
    pub state: Vec<u8>,
    pub adapter: Option<Vec<u8>>,
}

impl Link {
    /// Boot a link from ROM images, one per player (1 to [`MAX_PLAYERS`]).
    /// Core 0 requests lockstep player 0 (primary/master side), core `i`
    /// requests player `i`.
    pub fn new(roms: Vec<Vec<u8>>) -> Result<Self, mgba::Error> {
        Self::with_options(LinkOptions {
            sides: roms.into_iter().map(|rom| SideOptions { rom, save: None }).collect(),
            rtc: None,
            peripheral: Peripheral::Cable,
        })
    }

    pub fn with_options(options: LinkOptions) -> Result<Self, mgba::Error> {
        let num_players = options.sides.len();
        let max_players = options.peripheral.max_players();
        assert!(
            (1..=max_players).contains(&num_players),
            "a {:?} link takes 1 to {max_players} players, got {num_players}",
            options.peripheral,
        );

        let seats = (0..num_players as i32).collect::<Vec<_>>();
        let (coordinator, mut drivers) = build_drivers(options.peripheral, &seats);
        let core_options = mgba::core::Options::default();

        let mut cores = (0..num_players)
            .map(|_| mgba::core::OwnedCore::new_gba("mgba-rollback", &core_options))
            .collect::<Result<Vec<_>, _>>()?;

        for (i, (core, side)) in cores.iter_mut().zip(options.sides).enumerate() {
            core.enable_video_buffer();
            core.load_rom(mgba::vfile::VFile::from_vec(side.rom))?;
            if let Some(save) = side.save {
                core.load_save(mgba::vfile::VFile::from_vec(save))?;
            }
            if let Some(rtc) = options.rtc {
                core.set_rtc_fixed(rtc);
            }
            if let Some(driver) = drivers.get_mut(i) {
                driver.install(core);
            }
            core.reset();
        }

        Ok(Link {
            cores,
            drivers,
            coordinator,
        })
    }

    /// Boot a link mid-game from per-side captures — the emulated
    /// equivalent of plugging a link cable into machines that are already
    /// running. Each core boots fresh and loads its captured state while no
    /// SIO driver is installed (so the load cannot fire lockstep callbacks
    /// into a half-built link); only then does every core attach to the
    /// coordinator, the same mid-run attach path mgba's own multi-window
    /// multiplayer uses, which reads the core's current link mode and clock
    /// at registration. Peers that build a link from identical captures get
    /// bit-identical machines, which is what rollback needs — including the
    /// building peer itself, whose own side must load from its serialized
    /// capture rather than continue its live core (core savestates are
    /// deliberately lossy in a few corners, and everyone must agree on the
    /// reconstruction).
    ///
    /// A single CABLE side is the unplugged continuation of a capture: no
    /// driver, no coordinator registration. A single wireless side keeps
    /// its adapter and its adapter session — the unplug is the other
    /// players leaving RF range, nothing more.
    pub fn from_states(
        sides: Vec<BootSide>,
        rtc: Option<std::time::SystemTime>,
        peripheral: Peripheral,
    ) -> Result<Self, mgba::Error> {
        let num_players = sides.len();
        let max_players = peripheral.max_players();
        assert!(
            (1..=max_players).contains(&num_players),
            "a {peripheral:?} link takes 1 to {max_players} players, got {num_players}",
        );

        let core_options = mgba::core::Options::default();

        let mut cores = (0..num_players)
            .map(|_| mgba::core::OwnedCore::new_gba("mgba-rollback", &core_options))
            .collect::<Result<Vec<_>, _>>()?;
        let mut adapters = Vec::with_capacity(num_players);
        for (core, side) in cores.iter_mut().zip(sides) {
            core.enable_video_buffer();
            core.load_rom(mgba::vfile::VFile::from_vec(side.rom))?;
            if let Some(save) = side.save {
                core.load_save(mgba::vfile::VFile::from_vec(save))?;
            }
            if let Some(rtc) = rtc {
                core.set_rtc_fixed(rtc);
            }
            core.reset();
            match peripheral {
                // The capture describes the OLD cable; neutralize the
                // cable-dependent SIO state so the game re-announces its
                // link mode to the new one.
                Peripheral::Cable => load_boot_state(core, &side.state)?,
                // The adapter is the SAME adapter: the core state (its
                // half of any in-flight exchange included) and the
                // adapter blob form a consistent local pair, so nothing
                // is neutralized and nothing is descheduled.
                Peripheral::Wireless => load_boot_state_verbatim(core, &side.state)?,
            }
            adapters.push(side.adapter);
        }

        // The cable plugs in (or the adapters come into range): attach in
        // player order, deterministically. Each wireless side asks for
        // the seat recorded in its capture — seats are sticky in the
        // coordinator, so a rebuild after a departure leaves every
        // survivor's airwaves identity (device id, connection
        // references, its own game's cached ids) untouched, and only
        // pairings with the departed actually drop. Blob-less sides
        // (fresh joiners, pre-seat captures) take any free seat.
        let seats = adapters
            .iter()
            .map(|blob| {
                blob.as_deref()
                    .and_then(mgba::sio::wireless::adapter_state_player_id)
                    .unwrap_or(-1)
            })
            .collect::<Vec<_>>();
        let (coordinator, mut drivers) = build_drivers(peripheral, &seats);
        for (core, driver) in cores.iter_mut().zip(drivers.iter_mut()) {
            driver.install(core);
        }
        // Attach powered the adapters on fresh; put each captured session
        // back. Every peer injects the same blobs in the same order, so
        // the merged airwaves is bit-identical everywhere.
        for (driver, blob) in drivers.iter_mut().zip(adapters.iter()) {
            if let Some(blob) = blob {
                if !driver.load_adapter_state(blob) {
                    return Err(mgba::Error::CallFailed("GBASIOWirelessDriverLoadAdapterState"));
                }
            }
        }

        Ok(Link {
            cores,
            drivers,
            coordinator,
        })
    }

    /// Serialize core `i` for [`Link::from_states`]: a plain core
    /// savestate. Valid at any tick boundary. The capture is only as exact
    /// as mgba's savestate encoding — the deliberately-lossy corners the
    /// rollback [`Snapshot`] carries out-of-band (FIFO sample countdowns,
    /// mid-refill DMA control) reconstruct approximately, which is fine
    /// here: every peer reconstructs from the same bytes, and a cable
    /// plug-in has no prior trajectory to stay faithful to.
    pub fn capture_boot_state(&mut self, i: usize) -> Result<Vec<u8>, mgba::Error> {
        let state = self.cores[i].save_state()?;
        Ok(state.as_slice().to_vec())
    }

    /// Core `i`'s live adapter session, the wireless pair to
    /// [`Link::capture_boot_state`] (`None` on a cable link, which has no
    /// adapter). Valid at any tick boundary; feed it to
    /// [`Link::from_states`] as [`BootSide::adapter`] so the machine and
    /// its adapter resume as one.
    pub fn capture_adapter_state(&mut self, i: usize) -> Option<Vec<u8>> {
        self.drivers.get_mut(i).and_then(|d| d.save_adapter_state())
    }

    /// Core `i`'s current SRAM/flash/EEPROM image, or `None` if the game
    /// has no savedata (type never detected). Read straight from the live
    /// savedata buffer — the pair to [`Link::capture_boot_state`], since
    /// core savestates do not carry savedata.
    pub fn export_save(&mut self, i: usize) -> Option<Vec<u8>> {
        unsafe {
            let gba = gba_ptr(&mut self.cores[i]);
            let savedata = std::ptr::addr_of!((*gba).memory.savedata);
            let size = mgba_sys::GBASavedataSize(savedata);
            let data = (*savedata).data;
            if size == 0 || data.is_null() {
                return None;
            }
            Some(std::slice::from_raw_parts(data as *const u8, size).to_vec())
        }
    }

    /// Number of players (cores) on this link.
    pub fn num_players(&self) -> usize {
        self.cores.len()
    }

    pub fn core(&self, i: usize) -> &mgba::core::Core {
        &self.cores[i]
    }

    pub fn core_mut(&mut self, i: usize) -> &mut mgba::core::Core {
        &mut self.cores[i]
    }

    pub fn player_id(&self, i: usize) -> i32 {
        self.drivers.get(i).map(|d| d.player_id()).unwrap_or(0)
    }

    /// Core `i`'s rendered frame (240x160, mgba's native 16-bit XBGR1555),
    /// for frontends.
    pub fn video_buffer(&self, i: usize) -> Option<&[u8]> {
        self.cores[i].video_buffer()
    }

    /// Install instruction traps on core `i` (see `mgba::core::Core::set_traps`).
    /// The core owns the trapper, which is the only sound ownership: the
    /// trapper splices itself into the core's CPU component table and has
    /// no uninstall, so the core dereferences it right up through its own
    /// deinit — a trapper held anywhere else can be freed first and turn
    /// core teardown into a jump through reclaimed memory. `Core`'s drop
    /// order (deinit, then fields) keeps the trapper alive exactly long
    /// enough.
    pub fn set_traps(&mut self, i: usize, traps: Vec<(u32, Box<dyn Fn(&mut mgba::core::Core)>)>) {
        self.cores[i].set_traps(traps);
    }

    /// Set core `i`'s video frameskip: `i32::MAX` never renders, `0`
    /// renders every frame. Rendering is invisible to the emulated machine
    /// and frameskip is not serialized, so this is rollback-safe — it
    /// survives `load` and cannot perturb snapshot digests. Skip whichever
    /// cores nobody is watching: the remote sides during live play, every
    /// side while re-simulating.
    pub fn set_frameskip(&mut self, i: usize, frameskip: i32) {
        self.cores[i].gba_mut().set_frameskip(frameskip);
    }

    /// Advance the link by one frame of the reference core, interleaving
    /// run_loop slices between whichever cores the lockstep protocol
    /// currently allows to run. `keys[i]` is latched for core `i` at the
    /// start of the tick — the fixed sequence point that makes the key
    /// schedule (and therefore the whole link) deterministic and
    /// replayable.
    ///
    /// Returns the number of slices run (diagnostic only). Panics on a
    /// corrupt lockstep state; drivers that must survive one (netplay,
    /// where a bad restore is survivable by unplugging the cable) use
    /// [`try_tick`](Link::try_tick) instead.
    pub fn tick(&mut self, keys: &[u32]) -> usize {
        self.try_tick(keys).unwrap()
    }

    /// [`tick`](Link::tick), but a corrupt lockstep state — every core
    /// asleep, or [`MAX_SLICES_PER_TICK`] exceeded without finishing a
    /// reference frame — comes back as an error instead of a panic. A
    /// grind *below* any panic threshold reads as a frozen emulator with
    /// a clean console; the cap is set low enough that the error fires
    /// while the tick is still fast enough to deliver it.
    pub fn try_tick(&mut self, keys: &[u32]) -> Result<usize, mgba::Error> {
        assert_eq!(keys.len(), self.cores.len(), "one key set per player");
        for (core, &k) in self.cores.iter_mut().zip(keys.iter()) {
            core.set_keys(k);
        }

        let target = self.cores[REFERENCE].frame_counter().wrapping_add(1);
        let mut slices = 0;
        while self.cores[REFERENCE].frame_counter() != target {
            let mut progressed = false;
            for i in 0..self.cores.len() {
                if self.drivers.get(i).is_some_and(|d| d.asleep()) {
                    continue;
                }
                if i == REFERENCE && self.cores[REFERENCE].frame_counter() == target {
                    continue;
                }
                self.cores[i].run_loop();
                progressed = true;
                slices += 1;
            }
            if !progressed {
                // _verifyAwake on the C side guarantees not everyone sleeps;
                // reaching this means the link state is corrupt.
                return Err(mgba::Error::CallFailed(
                    "Link::tick (lockstep deadlock: all cores asleep)",
                ));
            }
            if slices > MAX_SLICES_PER_TICK {
                return Err(mgba::Error::CallFailed(
                    "Link::tick (lockstep livelock: slice cap exceeded)",
                ));
            }
        }
        Ok(slices)
    }

    /// Snapshot the full link. Valid at any tick boundary, including with a
    /// transfer in flight or any core parked by the lockstep protocol.
    pub fn save(&mut self) -> Result<Snapshot, mgba::Error> {
        let cores = self
            .cores
            .iter_mut()
            .map(|core| core.save_state())
            .collect::<Result<Vec<_>, _>>()?;
        Ok(Snapshot {
            cores,
            drivers: self.drivers.iter_mut().map(|d| d.save_state()).collect(),
            audio_fifos: self.cores.iter_mut().map(|core| audio_fifo_state(core)).collect(),
            dmas: self.cores.iter_mut().map(|core| dma_state(core)).collect(),
        })
    }

    /// Restore a snapshot taken from THIS link (same attach configuration).
    /// Core states load first — a core load rebuilds its timing list, the
    /// SIO completion event included — and the driver blobs then
    /// re-schedule each lockstep event into it, so an exact-timestamp tie
    /// keeps the completion first, matching the live scheduling order.
    pub fn load(&mut self, snapshot: &Snapshot) -> Result<(), mgba::Error> {
        assert_eq!(
            snapshot.cores.len(),
            self.cores.len(),
            "snapshot is from a link with a different player count"
        );
        for (i, (core, state)) in self.cores.iter_mut().zip(snapshot.cores.iter()).enumerate() {
            core.load_state(state)?;
            restore_audio_fifos(core, &snapshot.audio_fifos[i]);
            restore_dmas(core, &snapshot.dmas[i]);
        }
        for (driver, blob) in self.drivers.iter_mut().zip(snapshot.drivers.iter()) {
            if !driver.load_state(blob) {
                return Err(mgba::Error::CallFailed("GBASIOLockstepDriver::loadState"));
            }
        }
        Ok(())
    }
}

/// Load a serialized boot state (from [`Link::capture_boot_state`]) into a
/// freshly reset core with NO SIO driver installed, then neutralize the
/// cable-dependent SIO state: the capture describes the OLD cable (or the
/// lack of one), and this core is about to be on a new one.
///
/// - The SIOCNT mode bits are stripped from the shadow register, so the
///   game's next SIOCNT write reads as a mode switch and re-announces the
///   game's link mode to whatever driver is attached by then. Hot-attach
///   discovery depends on this: the lockstep protocol propagates a player's
///   mode via `setMode` events, and a core loaded already-in-mode would
///   otherwise never fire one — no peer ever reports ready. (Real link
///   menus re-assert their mode constantly; that re-assert is the plug-in
///   handshake, same as on hardware.)
/// - The SIOCNT line/status bits (slave, ready, multi-ID, busy) are
///   stripped too: `GBASIOWriteSIOCNT` ORs the previous shadow's bits 2-7
///   back into every write, so a stale 1 (e.g. the slave bit every
///   unplugged GBA reads, or the busy bit of a transfer the unplug killed)
///   would survive re-derivation forever.
/// - Any pending transfer-completion event is descheduled: it belongs to a
///   transfer on the old cable.
fn load_boot_state(core: &mut mgba::core::Core, blob: &[u8]) -> Result<(), mgba::Error> {
    if blob.len() != std::mem::size_of::<mgba::state::State>() {
        return Err(mgba::Error::CallFailed("boot state has the wrong length"));
    }
    // Sound per State::from_slice's contract: exact size, and the bytes came
    // from capture_boot_state on a compatible core.
    let state = unsafe { mgba::state::State::from_slice(blob) };
    core.load_state(&state)?;
    deschedule_sio_complete(core);
    unsafe {
        let gba = gba_ptr(core);
        let io = &mut (*gba).memory.io;
        (*gba).sio.siocnt = io[REG_SIOCNT] & !0x30fc;
        (*gba).sio.rcnt = io[REG_RCNT];
        // Mode derivation per sio.c's _switchMode, over the neutralized
        // shadow. A driver attached after this reads the core's mode from
        // here at registration.
        let mode = (((*gba).sio.rcnt & 0xc000) | ((*gba).sio.siocnt & 0x3000)) >> 12;
        let mode = if mode < 8 { mode & 0x3 } else { mode & 0xc };
        (*gba).sio.mode = mode as mgba_sys::GBASIOMode;
    }
    Ok(())
}

/// Load a serialized boot state exactly as captured, with no SIO surgery:
/// the wireless path, where the peripheral on the port is the same
/// adapter the capture was talking to (its session travels separately in
/// the adapter blob) and any pending transfer completion belongs to a
/// local exchange that is being restored consistently.
fn load_boot_state_verbatim(core: &mut mgba::core::Core, blob: &[u8]) -> Result<(), mgba::Error> {
    if blob.len() != std::mem::size_of::<mgba::state::State>() {
        return Err(mgba::Error::CallFailed("boot state has the wrong length"));
    }
    // Sound per State::from_slice's contract: exact size, and the bytes came
    // from capture_boot_state on a compatible core.
    let state = unsafe { mgba::state::State::from_slice(blob) };
    core.load_state(&state)
}

/// Raw C-side view of a core's GBA, for the state surgery in this module
/// (the `mgba-sys` dependency comes from the same git source as `mgba`
/// itself, so the types are the same crate's).
fn gba_ptr(core: &mut mgba::core::Core) -> *mut mgba_sys::GBA {
    core.gba_mut().as_raw()
}

/// Deschedule a core's SIO transfer-completion event
/// (`GBASIO::completeEvent`). The core savestate restores the completion
/// exactly, but a boot capture is loaded onto a NEW cable: a pending
/// completion belongs to a transfer on the old one and must not fire here.
fn deschedule_sio_complete(core: &mut mgba::core::Core) {
    unsafe {
        let gba = gba_ptr(core);
        let timing = std::ptr::addr_of_mut!((*gba).timing);
        let event = std::ptr::addr_of_mut!((*gba).sio.completeEvent);
        mgba_sys::mTimingDeschedule(timing, event);
    }
}

/// Capture a core's direct-sound FIFO channels verbatim.
///
/// This must ride in the link snapshot because the core savestate's
/// encoding is lossy: `GBAAudioSerialize` packs each channel's
/// `internalRemaining` — which counts 4..0 samples left in the popped
/// word — into a TWO-bit legacy field (`FIFOInternalSamplesA/B`), so the
/// common value 4 aliases to 0. A restored core then pops its next FIFO
/// word up to 4 sample-events early, drains the FIFO faster than the live
/// machine did, and crosses the DMA refill threshold (fill < 4) at a
/// different timer overflow — the refill DMA steals ~10 bus cycles at a
/// point in time where the live run had none, and the whole link's
/// interleave forks from there. (The serializer also normalizes the ring
/// to `read == 0`, which is behaviorally invisible except for the
/// open-bus-ish value `GBAAudioWriteFIFO` returns from the next slot;
/// carrying the raw ring makes the round trip exact rather than merely
/// equivalent.)
fn audio_fifo_state(core: &mut mgba::core::Core) -> [FifoLane; 2] {
    unsafe {
        let gba = gba_ptr(core);
        let lane = |ch: *const mgba_sys::GBAAudioFIFO| FifoLane {
            fifo: (*ch).fifo,
            write: (*ch).fifoWrite,
            read: (*ch).fifoRead,
            internal_remaining: (*ch).internalRemaining,
        };
        [
            lane(std::ptr::addr_of!((*gba).audio.chA)),
            lane(std::ptr::addr_of!((*gba).audio.chB)),
        ]
    }
}

/// Force a core's direct-sound FIFO channels back to the recorded truth,
/// after the core's `load_state` applied the lossy serialized version.
fn restore_audio_fifos(core: &mut mgba::core::Core, lanes: &[FifoLane; 2]) {
    unsafe {
        let gba = gba_ptr(core);
        for (ch, lane) in [
            std::ptr::addr_of_mut!((*gba).audio.chA),
            std::ptr::addr_of_mut!((*gba).audio.chB),
        ]
        .into_iter()
        .zip(lanes.iter())
        {
            (*ch).fifo = lane.fifo;
            (*ch).fifoWrite = lane.write;
            (*ch).fifoRead = lane.read;
            (*ch).internalRemaining = lane.internal_remaining;
        }
    }
}

/// Capture a core's internal DMA control state verbatim.
///
/// This must ride in the link snapshot because the core savestate
/// reconstructs `GBADMA::reg` (and the `sourceOffset`/`destOffset`/`cycles`
/// values derived from it) from the io block's DMAxCNT_HI — but
/// `GBAAudioScheduleFifoDma` rewrites `reg` in place (dest control forced
/// to FIXED, width forced to 32-bit) WITHOUT updating the io block when a
/// FIFO refill is dispatched. A snapshot that lands while a refill is
/// pending or mid-block (routine here: `GBASIOLockstepPlayerSleep` parks a
/// core mid-event-batch, freezing an in-flight refill across the tick
/// boundary) restores the channel with the game's raw control instead: the
/// destination increments off the FIFO register, the width may be wrong,
/// and the re-simulated audio stream — and every bus cycle it steals —
/// forks from the original run.
fn dma_state(core: &mut mgba::core::Core) -> [DmaLane; 4] {
    unsafe {
        let gba = gba_ptr(core);
        std::array::from_fn(|i| {
            let dma = std::ptr::addr_of!((*gba).memory.dma[i]);
            DmaLane {
                reg: (*dma).reg,
                cycles: (*dma).cycles,
                source_offset: (*dma).sourceOffset,
                dest_offset: (*dma).destOffset,
            }
        })
    }
}

/// Force a core's internal DMA control state back to the recorded truth,
/// after the core's `load_state` reconstructed it from the io block.
fn restore_dmas(core: &mut mgba::core::Core, lanes: &[DmaLane; 4]) {
    unsafe {
        let gba = gba_ptr(core);
        for (i, lane) in lanes.iter().enumerate() {
            let dma = std::ptr::addr_of_mut!((*gba).memory.dma[i]);
            (*dma).reg = lane.reg;
            (*dma).cycles = lane.cycles;
            (*dma).sourceOffset = lane.source_offset;
            (*dma).destOffset = lane.dest_offset;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Reproduces the netplay hard-freeze: a secondary player whose local
    /// clock has caught the coordinator's shared cycle while its queue
    /// head is still in the future. mgba's `_lockstepEvent` then computes
    /// a non-positive reschedule; without the parking fix in the mgba
    /// fork it refires forever at the same timestamp — with every core on
    /// one thread, nothing can ever advance the shared clock, so a single
    /// `Link::tick` never returns. The state is manufactured through the
    /// driver-blob serialization (layout: GBASIOLockstepSerializedState
    /// in mgba's gba/sio/lockstep.c), which is exactly how a rollback
    /// restore would replay it.
    #[test]
    fn caught_up_secondary_with_future_event_does_not_wedge() {
        mgba::log::install_default_logger();
        let rom = testrom::build();
        let mut link = Link::new(vec![rom.clone(), rom]).unwrap();
        let keys = |t: u32| [(t * 0x11) & 0x3ff, (t * 0x17) & 0x3ff];
        for t in 0..600u32 {
            link.tick(&keys(t));
        }
        let mut snap = link.save().unwrap();

        // GBASIOLockstepSerializedState field offsets, checked against the
        // static_asserts in lockstep.c.
        const OFF_FLAGS: usize = 0x04;
        const OFF_DRIVER_NEXT_EVENT: usize = 0x10;
        const OFF_EVENTS: usize = 0x40;
        const EVENT_SIZE: usize = 0x30;
        const OFF_COORD_CYCLE: usize = 0x1c0;
        const FLAG_ASLEEP: u32 = 1 << 7;
        const FLAG_EVENT_SCHEDULED: u32 = 1 << 9;
        const NUM_EVENTS_SHIFT: u32 = 3;
        const NUM_EVENTS_MASK: u32 = 0xf << NUM_EVENTS_SHIFT;
        const SIO_EV_HARD_SYNC: i32 = 2;

        let rd = |b: &[u8], off: usize| i32::from_le_bytes(b[off..off + 4].try_into().unwrap());
        let wr =
            |b: &mut [u8], off: usize, v: i32| b[off..off + 4].copy_from_slice(&v.to_le_bytes());

        // The shared clock rides player 0's blob: pull it well behind
        // player 1's local time, and push player 0's next lockstep firing
        // out so player 1 fires first.
        let cycle = rd(&snap.drivers[0], OFF_COORD_CYCLE);
        wr(&mut snap.drivers[0], OFF_COORD_CYCLE, cycle - 100_000);
        wr(&mut snap.drivers[0], OFF_DRIVER_NEXT_EVENT, 100_000);

        // Player 1: awake, lockstep event due immediately, and a queued
        // event stamped far in the future (as the clock owner routinely
        // produces when it enqueues mid-interval).
        let b = &mut snap.drivers[1];
        let mut flags = rd(b, OFF_FLAGS) as u32;
        flags &= !FLAG_ASLEEP;
        flags |= FLAG_EVENT_SCHEDULED;
        let n = ((flags & NUM_EVENTS_MASK) >> NUM_EVENTS_SHIFT) as usize;
        assert!(n < 8, "lockstep queue unexpectedly full");
        flags = (flags & !NUM_EVENTS_MASK) | ((n as u32 + 1) << NUM_EVENTS_SHIFT);
        wr(b, OFF_FLAGS, flags as i32);
        wr(b, OFF_DRIVER_NEXT_EVENT, 4);
        let ev = OFF_EVENTS + n * EVENT_SIZE;
        wr(b, ev, cycle + 1_000_000); // timestamp: far future
        wr(b, ev + 4, 0); // sender: player 0
        wr(b, ev + 8, SIO_EV_HARD_SYNC);

        link.load(&snap).unwrap();

        // An unfixed driver never returns from the first tick, so run on
        // a watchdog'd thread: the wedge becomes a reported failure, not
        // a hung test run.
        let (tx, rx) = std::sync::mpsc::channel();
        std::thread::spawn(move || {
            for t in 600..840u32 {
                link.tick(&keys(t));
            }
            let _ = tx.send(());
        });
        rx.recv_timeout(std::time::Duration::from_secs(20)).expect(
            "link wedged or died: the lockstep driver busy-waited on the shared \
             clock instead of parking the caught-up player (mgba parking fix missing?)",
        );
    }

    /// Reproduces the freeze found after the parking fix landed: a player
    /// promoted to clock owner by a detach still holds a TRANSFER_START in
    /// its queue. Only secondaries are ever sent that event, and the detach
    /// that promoted the player also tore down the transfer it announced —
    /// but the queue survives promotion. Draining it as player 0 executes
    /// the finishCycle reschedule override; with the transfer window long
    /// past, nextEvent goes non-positive, and player 0 has no parking
    /// branch to catch it: `mASSERT_DEBUG(nextEvent > 0)` (lockstep.c)
    /// aborts a debug build, and a release build flags a phantom transfer
    /// busy. Manufactured through the driver blob exactly like the wedge
    /// repro above: this is the promoted player's persisted state as a
    /// rollback restore would replay it. Note the abort tripwire only arms
    /// in debug builds (mASSERT_DEBUG compiles out in release).
    #[test]
    fn promoted_clock_owner_with_stale_transfer_start_does_not_die() {
        mgba::log::install_default_logger();
        let rom = testrom::build();
        let mut link = Link::new(vec![rom.clone(), rom]).unwrap();
        let keys = |t: u32| [(t * 0x11) & 0x3ff, (t * 0x17) & 0x3ff];
        for t in 0..600u32 {
            link.tick(&keys(t));
        }
        let mut snap = link.save().unwrap();

        // GBASIOLockstepSerializedState field offsets, checked against the
        // static_asserts in lockstep.c.
        const OFF_FLAGS: usize = 0x04;
        const OFF_DRIVER_NEXT_EVENT: usize = 0x10;
        const OFF_EVENTS: usize = 0x40;
        const OFF_EVENT_FINISH_CYCLE: usize = 0x20;
        const OFF_COORD_CYCLE: usize = 0x1c0;
        const FLAG_ASLEEP: u32 = 1 << 7;
        const FLAG_EVENT_SCHEDULED: u32 = 1 << 9;
        const NUM_EVENTS_SHIFT: u32 = 3;
        const NUM_EVENTS_MASK: u32 = 0xf << NUM_EVENTS_SHIFT;
        const SIO_EV_TRANSFER_START: i32 = 4;

        let rd = |b: &[u8], off: usize| i32::from_le_bytes(b[off..off + 4].try_into().unwrap());
        let wr =
            |b: &mut [u8], off: usize, v: i32| b[off..off + 4].copy_from_slice(&v.to_le_bytes());

        // Player 0's own blob: awake, lockstep event due immediately, and
        // the queue replaced by a single TRANSFER_START whose transfer
        // window is long gone — the leftover a detach-promotion carries.
        let cycle = rd(&snap.drivers[0], OFF_COORD_CYCLE);
        let b = &mut snap.drivers[0];
        let mut flags = rd(b, OFF_FLAGS) as u32;
        flags &= !FLAG_ASLEEP;
        flags |= FLAG_EVENT_SCHEDULED;
        flags = (flags & !NUM_EVENTS_MASK) | (1 << NUM_EVENTS_SHIFT);
        wr(b, OFF_FLAGS, flags as i32);
        wr(b, OFF_DRIVER_NEXT_EVENT, 4);
        wr(b, OFF_EVENTS, cycle - 10_000); // timestamp: already due
        wr(b, OFF_EVENTS + 4, 0); // sender: the departed clock owner
        wr(b, OFF_EVENTS + 8, SIO_EV_TRANSFER_START);
        wr(b, OFF_EVENTS + OFF_EVENT_FINISH_CYCLE, cycle - 100_000); // finished long ago

        link.load(&snap).unwrap();

        // An unguarded debug driver aborts on the reschedule assert; run
        // on a watchdog'd thread so a residual wedge is also a reported
        // failure rather than a hung test run.
        let (tx, rx) = std::sync::mpsc::channel();
        std::thread::spawn(move || {
            for t in 600..840u32 {
                link.tick(&keys(t));
            }
            let _ = tx.send(());
        });
        rx.recv_timeout(std::time::Duration::from_secs(20)).expect(
            "link wedged or died: player 0 drained a stale TRANSFER_START \
             (mgba stale-transfer guard missing?)",
        );
    }

    /// The cable twin of the wireless clock-wrap test (see
    /// tests/wireless.rs): every lockstep clock is an int32 cycle
    /// counter that crosses the sign boundary ~128 emulated seconds
    /// after boot, and the sign-unsafe comparisons in lockstep.c froze
    /// the clock owner's guard there. A busy MULTI link limps across
    /// (the transfer path advances the shared clock outside the broken
    /// guard), but a QUIET link — two players idling outside a link
    /// scene — parks its secondaries on the sync cadence, and only the
    /// guard ever wakes them: at the boundary the secondary freezes for
    /// the next 128 emulated seconds. Shift every clock so local time
    /// sits just below 2^31 and idle across the boundary: the secondary
    /// must keep making frames, and the shared clock must come out the
    /// other side wrapped negative.
    #[test]
    fn cable_link_survives_the_cycle_counter_sign_wrap() {
        mgba::log::install_default_logger();
        let rom = testrom::build_idle();
        let mut link = Link::new(vec![rom.clone(), rom]).unwrap();
        let keys = |t: u32| [(t * 0x11) & 0x3ff, (t * 0x17) & 0x3ff];
        for t in 0..600u32 {
            link.tick(&keys(t));
        }
        let mut snap = link.save().unwrap();

        // GBASIOLockstepSerializedState field offsets, checked against
        // the static_asserts in lockstep.c.
        const OFF_FLAGS: usize = 0x04;
        const OFF_PLAYER_CYCLE_OFFSET: usize = 0x34;
        const OFF_EVENTS: usize = 0x40;
        const EVENT_SIZE: usize = 0x30;
        const OFF_EVENT_FLAGS: usize = 0x08;
        const OFF_EVENT_FINISH_CYCLE: usize = 0x20;
        const OFF_COORD_CYCLE: usize = 0x1c0;
        const NUM_EVENTS_SHIFT: u32 = 3;
        const NUM_EVENTS_MASK: u32 = 0xf << NUM_EVENTS_SHIFT;
        const EVENT_TYPE_MASK: u32 = 0x7;
        const SIO_EV_TRANSFER_START: u32 = 4;

        let rd = |b: &[u8], off: usize| i32::from_le_bytes(b[off..off + 4].try_into().unwrap());
        let wr =
            |b: &mut [u8], off: usize, v: i32| b[off..off + 4].copy_from_slice(&v.to_le_bytes());

        // Land the shared clock a quarter-frame short of 2^31. Shifting
        // every cycleOffset down moves every local time up in lockstep;
        // queued event timestamps (and TRANSFER_START finish cycles —
        // the union holds a mode for MODE_SET, which must not shift)
        // ride along.
        let delta = 0x7FFF_0000u32.wrapping_sub(rd(&snap.drivers[0], OFF_COORD_CYCLE) as u32) as i32;
        for b in snap.drivers.iter_mut() {
            let n = ((rd(b, OFF_FLAGS) as u32 & NUM_EVENTS_MASK) >> NUM_EVENTS_SHIFT) as usize;
            let off = rd(b, OFF_PLAYER_CYCLE_OFFSET).wrapping_sub(delta);
            wr(b, OFF_PLAYER_CYCLE_OFFSET, off);
            for e in 0..n {
                let ev = OFF_EVENTS + e * EVENT_SIZE;
                let ts = rd(b, ev).wrapping_add(delta);
                wr(b, ev, ts);
                if rd(b, ev + OFF_EVENT_FLAGS) as u32 & EVENT_TYPE_MASK == SIO_EV_TRANSFER_START {
                    let finish = rd(b, ev + OFF_EVENT_FINISH_CYCLE).wrapping_add(delta);
                    wr(b, ev + OFF_EVENT_FINISH_CYCLE, finish);
                }
            }
        }
        let b = &mut snap.drivers[0];
        let cycle = rd(b, OFF_COORD_CYCLE).wrapping_add(delta);
        wr(b, OFF_COORD_CYCLE, cycle);
        link.load(&snap).unwrap();

        // Cross the boundary with the link under load, watchdog'd so a
        // wedge is a reported failure rather than a hung test run.
        let (tx, rx) = std::sync::mpsc::channel();
        std::thread::spawn(move || {
            let before = link.core(1).frame_counter();
            for t in 600..840u32 {
                link.tick(&keys(t));
            }
            let frames = link.core(1).frame_counter().wrapping_sub(before);
            let _ = tx.send((link.save().unwrap(), frames));
        });
        let (snap, frames) = rx
            .recv_timeout(std::time::Duration::from_secs(20))
            .expect("link wedged or died crossing the int32 cycle boundary");
        assert!(
            rd(snap.driver_blob(0), OFF_COORD_CYCLE) < 0,
            "shared clock stalled instead of wrapping"
        );
        assert!(
            frames >= 200,
            "secondary froze crossing the boundary: only {frames} frames"
        );
    }
}
