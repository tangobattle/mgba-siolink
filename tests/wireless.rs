//! Wireless adapter (RFU) protocol tests.
//!
//! The cores run an idle ROM and the tests puppet each GBA's SIO
//! registers from outside the emulated CPU — the same writes librfu
//! would perform, including its SO/SI handshake between words (whose
//! line levels are asserted at every step). Register pokes follow a
//! fixed script, so the whole run stays deterministic and the rollback
//! test can replay it after a restore.

use mgba_siolink::{testrom, BootSide, Link, LinkOptions, Peripheral, SideOptions};

const IDLE: u32 = 0x8000_0000;

// SIOCNT values for NORMAL-32: internal 2MHz clock (master) and external
// clock (slave), each with and without the start bit.
const CNT_MASTER: u16 = 0x1003;
const CNT_MASTER_GO: u16 = 0x1083;
const CNT_SLAVE: u16 = 0x1000;
const CNT_SLAVE_GO: u16 = 0x1080;

// io halfword indices.
const IO_SIODATA32_LO: usize = 0x120 >> 1;
const IO_SIODATA32_HI: usize = 0x122 >> 1;

fn wireless_link(players: usize) -> Link {
    mgba::log::install_default_logger();
    let rom = testrom::build_idle();
    Link::with_options(LinkOptions {
        sides: (0..players)
            .map(|_| SideOptions {
                rom: rom.clone(),
                save: None,
            })
            .collect(),
        rtc: None,
        peripheral: Peripheral::Wireless,
    })
    .unwrap()
}

fn sio_ptr(link: &mut Link, i: usize) -> *mut mgba_sys::GBASIO {
    unsafe { std::ptr::addr_of_mut!((*link.core_mut(i).gba_mut().as_raw()).sio) }
}

fn write_rcnt(link: &mut Link, i: usize, v: u16) {
    unsafe { mgba_sys::GBASIOWriteRCNT(sio_ptr(link, i), v) }
}

fn write_siocnt(link: &mut Link, i: usize, v: u16) {
    unsafe { mgba_sys::GBASIOWriteSIOCNT(sio_ptr(link, i), v) }
}

fn siocnt(link: &mut Link, i: usize) -> u16 {
    unsafe { (*sio_ptr(link, i)).siocnt }
}

fn set_data32(link: &mut Link, i: usize, w: u32) {
    unsafe {
        let gba = link.core_mut(i).gba_mut().as_raw();
        (*gba).memory.io[IO_SIODATA32_LO] = w as u16;
        (*gba).memory.io[IO_SIODATA32_HI] = (w >> 16) as u16;
    }
}

fn data32(link: &mut Link, i: usize) -> u32 {
    unsafe {
        let gba = link.core_mut(i).gba_mut().as_raw();
        (*gba).memory.io[IO_SIODATA32_LO] as u32 | ((*gba).memory.io[IO_SIODATA32_HI] as u32) << 16
    }
}

fn tick(link: &mut Link) {
    let keys = vec![0u32; link.num_players()];
    link.tick(&keys);
}

/// One GBA-clocked 32-bit exchange at `cnt`: send `word`, return the
/// adapter's.
fn xfer_at(link: &mut Link, i: usize, cnt: u16, word: u32) -> u32 {
    set_data32(link, i, word);
    write_siocnt(link, i, cnt);
    write_siocnt(link, i, cnt | 0x80);
    for _ in 0..4 {
        if siocnt(link, i) & 0x80 == 0 {
            break;
        }
        tick(link);
    }
    assert_eq!(siocnt(link, i) & 0x80, 0, "transfer on core {i} never completed");
    data32(link, i)
}

/// The SI line as the GBA reads it (SIOCNT bit 2).
fn si(link: &mut Link, i: usize) -> bool {
    siocnt(link, i) & 0x4 != 0
}

/// librfu's inter-word handshake after a GBA-clocked word: the adapter
/// holds SI high (word consumed); the GBA answers SO high and the
/// adapter drops SI; the GBA returns SO low and clocks the next word.
fn dance_master(link: &mut Link, i: usize) {
    assert!(si(link, i), "adapter must raise SI after consuming a word (core {i})");
    write_siocnt(link, i, CNT_MASTER | 0x8);
    assert!(!si(link, i), "adapter must drop SI on SO high (core {i})");
    write_siocnt(link, i, CNT_MASTER);
}

/// librfu's handshake between adapter-clocked words: the adapter drops
/// SI after clocking; the GBA answers SO high and the adapter raises SI
/// (ready for the next exchange); the GBA returns SO low before
/// re-arming. Runs once more after the final ack transfer (the slave
/// ISR's state-8 pass).
fn dance_slave(link: &mut Link, i: usize) {
    assert!(!si(link, i), "adapter must drop SI after clocking a word (core {i})");
    write_siocnt(link, i, CNT_SLAVE | 0x8);
    assert!(si(link, i), "adapter must answer SO high with SI high (core {i})");
    write_siocnt(link, i, CNT_SLAVE);
}

/// One GBA-clocked 32-bit exchange at the post-login 2MHz rate, with
/// the full inter-word handshake.
fn xfer(link: &mut Link, i: usize, word: u32) -> u32 {
    let reply = xfer_at(link, i, CNT_MASTER, word);
    dance_master(link, i);
    reply
}

/// The full reset + login bring-up, validated against the canonical
/// NI/NT/EN/DO trace (see librfu's sio32id and the gba-link-connection
/// docs; every word below matches real-hardware captures).
fn login(link: &mut Link, i: usize) {
    // The SD-high pulse in GPIO mode resets the adapter.
    write_rcnt(link, i, 0x8000);
    write_rcnt(link, i, 0x80A0);
    write_rcnt(link, i, 0x80A2);
    tick(link);
    write_rcnt(link, i, 0x80A0);
    write_rcnt(link, i, 0x0000);

    let trace = [
        (0x7FFF494Eu32, 0x00000000u32),
        (0xFFFF494E, 0x494EB6B1),
        (0xB6B1494E, 0x494EB6B1),
        (0xB6B1544E, 0x544EB6B1),
        (0xABB1544E, 0x544EABB1),
        (0xABB14E45, 0x4E45ABB1),
        (0xB1BA4E45, 0x4E45B1BA),
        (0xB1BA4F44, 0x4F44B1BA),
        (0xB0BB4F44, 0x4F44B0BB),
        (0xB0BB8001, 0x8001B0BB),
    ];
    for (t, (send, expect)) in trace.iter().enumerate() {
        let got = if t == 0 {
            // Sio32IDInit arms the start bit as an external-clock slave,
            // then flips the clock source to internal (SIO_38400_BPS) —
            // the transfer must begin on the clock-source edge, with no
            // start-bit edge ever occurring.
            set_data32(link, i, *send);
            write_siocnt(link, i, 0x1000);
            write_siocnt(link, i, 0x1080);
            write_siocnt(link, i, 0x1081);
            for _ in 0..4 {
                if siocnt(link, i) & 0x80 == 0 {
                    break;
                }
                tick(link);
            }
            assert_eq!(siocnt(link, i) & 0x80, 0, "sio32id-armed transfer never completed");
            data32(link, i)
        } else {
            // Later words re-arm with a plain start edge at the 256kHz
            // internal clock (SIOCNT bit 0 set, bit 1 clear — the
            // combination that distinguishes clock SOURCE from SPEED).
            xfer_at(link, i, 0x1001, *send)
        };
        assert_eq!(
            got, *expect,
            "login word {t} on core {i}: sent {send:08X}, expected {expect:08X}, got {got:08X}"
        );
    }
}

/// Issue one command and collect the ack id plus response words. Asserts
/// the adapter's load-bearing 0x80000000 replies along the way.
fn command(link: &mut Link, i: usize, cmd: u8, params: &[u32]) -> (u8, Vec<u32>) {
    let header = 0x9966_0000 | ((params.len() as u32) << 8) | cmd as u32;
    assert_eq!(xfer(link, i, header), IDLE, "adapter must idle during the header");
    for &p in params {
        assert_eq!(xfer(link, i, p), IDLE, "adapter must idle during params");
    }
    let ack = xfer(link, i, IDLE);
    assert_eq!(ack >> 16, 0x9966, "bad ack header {ack:08X} for command {cmd:02X}");
    let count = (ack >> 8) & 0xFF;
    let words = (0..count).map(|_| xfer(link, i, IDLE)).collect();
    ((ack & 0xFF) as u8, words)
}

/// `command` with the ordinary success ack asserted.
fn ok(link: &mut Link, i: usize, cmd: u8, params: &[u32]) -> Vec<u32> {
    let (ack, words) = command(link, i, cmd, params);
    assert_eq!(ack, 0x80 | cmd, "command {cmd:02X} was not acked");
    words
}

#[test]
fn login_matches_the_canonical_trace() {
    let mut link = wireless_link(1);
    login(&mut link, 0);
    // Post-login the adapter answers commands: 0x10 reset acks empty.
    assert_eq!(ok(&mut link, 0, 0x10, &[]), vec![]);
    // Version status reports the real-hardware firmware word.
    assert_eq!(ok(&mut link, 0, 0x12, &[]), vec![0x0083_0117]);
}

#[test]
fn bad_commands_error_ack() {
    let mut link = wireless_link(1);
    login(&mut link, 0);
    // Below the valid range: unknown (code 2).
    let (ack, words) = command(&mut link, 0, 0x0F, &[]);
    assert_eq!(ack, 0xEE);
    assert_eq!(words, vec![2]);
    // In-range but unimplemented: wrong state (code 1).
    let (ack, words) = command(&mut link, 0, 0x18, &[]);
    assert_eq!(ack, 0xEE);
    assert_eq!(words, vec![1]);
    // The adapter must remain addressable afterwards.
    assert_eq!(ok(&mut link, 0, 0x10, &[]), vec![]);
}

const BROADCAST: [u32; 6] = [
    0x0001_7FFF, // gameId 0x7FFF | first game-name bytes
    0x0403_0201,
    0x0807_0605,
    0x0C0B_0A09,
    0x1211_100F,
    0x1615_1413,
];

/// Bring core `h` up as an open host.
fn host(link: &mut Link, h: usize) {
    ok(link, h, 0x16, &BROADCAST);
    ok(link, h, 0x17, &[0x003C_0420]);
    ok(link, h, 0x19, &[]);
}

/// Bring core `c` up as a client of `server_id`, through scan + connect.
fn join(link: &mut Link, c: usize, server_id: u32) -> u32 {
    ok(link, c, 0x1C, &[]);
    tick(link); // let an RF tick populate the scan
    let servers = ok(link, c, 0x1D, &[]);
    assert!(!servers.is_empty(), "scan found nothing");
    assert_eq!(servers.len() % 7, 0);
    assert_eq!(servers[0] & 0xFFFF, server_id, "wrong server id in {servers:08X?}");
    assert_eq!(&servers[1..7], &BROADCAST, "broadcast data did not travel");
    ok(link, c, 0x1E, &[]);
    ok(link, c, 0x1F, &[server_id]);
    // In progress until an RF tick resolves it.
    let mut status = ok(link, c, 0x20, &[])[0];
    for _ in 0..4 {
        if status != 0x0100_0000 {
            break;
        }
        tick(link);
        status = ok(link, c, 0x20, &[])[0];
    }
    assert_eq!(status >> 24, 0, "connect did not complete: {status:08X}");
    let finish = ok(link, c, 0x21, &[])[0];
    assert_eq!(finish, status, "0x21 and 0x20 disagree");
    status
}

#[test]
fn host_and_client_connect_and_exchange() {
    let mut link = wireless_link(2);
    login(&mut link, 0);
    login(&mut link, 1);

    host(&mut link, 0);
    // Host status: serving, open, own id.
    assert_eq!(ok(&mut link, 0, 0x13, &[]), vec![0x0200_61F0]);

    let status = join(&mut link, 1, 0x61F0);
    // Client 1's own id with slot 0.
    assert_eq!(status, 0x0000_61F1);
    // Client status: connected, one-hot slot 0.
    assert_eq!(ok(&mut link, 1, 0x13, &[]), vec![0x0501_61F1]);

    // The host's slot view: next slot 1, then {slot 0, client id}.
    assert_eq!(ok(&mut link, 0, 0x14, &[]), vec![0x0000_0001, 0x0000_61F1]);
    assert_eq!(ok(&mut link, 0, 0x1A, &[]), vec![0x0000_61F1]);
    // Signal: host sees client 0's byte, client sees its own slot byte.
    assert_eq!(ok(&mut link, 0, 0x11, &[]), vec![0x0000_00FF]);
    assert_eq!(ok(&mut link, 1, 0x11, &[]), vec![0x0000_00FF]);

    // Client schedules a 3-byte upload; the host's 4-byte frame collects
    // it and broadcasts its own payload.
    ok(&mut link, 1, 0x24, &[3 << 8, 0x00CC_BBAA]);
    ok(&mut link, 0, 0x24, &[4, 0xDDCC_BBAA]);
    tick(&mut link);

    // Host receives the client's bytes in the client-0 lane.
    assert_eq!(ok(&mut link, 0, 0x26, &[]), vec![3 << 8, 0x00CC_BBAA]);
    // A drained buffer reads back empty.
    assert_eq!(ok(&mut link, 0, 0x26, &[]), vec![]);
    // Client receives the host's bytes in the host lane.
    assert_eq!(ok(&mut link, 1, 0x26, &[]), vec![4, 0xDDCC_BBAA]);
    assert_eq!(ok(&mut link, 1, 0x26, &[]), vec![]);
}

#[test]
fn send_data_and_wait_delivers_an_event_frame() {
    let mut link = wireless_link(2);
    login(&mut link, 0);
    login(&mut link, 1);
    host(&mut link, 0);
    join(&mut link, 1, 0x61F0);

    // Client: sendDataAndWait, then hand the adapter the bus.
    ok(&mut link, 1, 0x25, &[2 << 8, 0x0000_BEEF]);
    set_data32(&mut link, 1, IDLE);
    write_siocnt(&mut link, 1, CNT_SLAVE);
    write_siocnt(&mut link, 1, CNT_SLAVE_GO);

    // Host transmits; the RF frame both collects the client's upload and
    // wakes the client's wait with a data-available event.
    ok(&mut link, 0, 0x24, &[1, 0x0000_0042]);
    for _ in 0..8 {
        if siocnt(&mut link, 1) & 0x80 == 0 {
            break;
        }
        tick(&mut link);
    }
    assert_eq!(siocnt(&mut link, 1) & 0x80, 0, "event frame never arrived");
    assert_eq!(data32(&mut link, 1), 0x9966_0028);
    dance_slave(&mut link, 1);

    // Acknowledge the event; the adapter answers 0x80000000 in the ack
    // transfer (librfu checks this) and hands the bus back.
    set_data32(&mut link, 1, 0x9966_00A8);
    write_siocnt(&mut link, 1, CNT_SLAVE_GO);
    for _ in 0..4 {
        if siocnt(&mut link, 1) & 0x80 == 0 {
            break;
        }
        tick(&mut link);
    }
    assert_eq!(siocnt(&mut link, 1) & 0x80, 0, "ack transfer never completed");
    assert_eq!(data32(&mut link, 1), IDLE);
    // The slave ISR's final state-8 handshake pass.
    dance_slave(&mut link, 1);

    // Back as master: the host's payload is waiting.
    assert_eq!(ok(&mut link, 1, 0x26, &[]), vec![1, 0x0000_0042]);
    // And the host got the client's upload.
    assert_eq!(ok(&mut link, 0, 0x26, &[]), vec![2 << 8, 0x0000_BEEF]);
}

#[test]
fn wait_times_out_per_the_configured_deadline() {
    let mut link = wireless_link(2);
    login(&mut link, 0);
    login(&mut link, 1);
    host(&mut link, 0);
    // waitTimeout = 2 frames.
    ok(&mut link, 0, 0x17, &[0x003C_0402]);
    join(&mut link, 1, 0x61F0);

    ok(&mut link, 1, 0x17, &[0x003C_0402]);
    ok(&mut link, 1, 0x27, &[]);
    set_data32(&mut link, 1, IDLE);
    write_siocnt(&mut link, 1, CNT_SLAVE);
    write_siocnt(&mut link, 1, CNT_SLAVE_GO);

    // Nobody transmits; the adapter must return the bus by itself.
    for _ in 0..8 {
        if siocnt(&mut link, 1) & 0x80 == 0 {
            break;
        }
        tick(&mut link);
    }
    assert_eq!(siocnt(&mut link, 1) & 0x80, 0, "timeout event never arrived");
    assert_eq!(data32(&mut link, 1), 0x9966_0027);
    dance_slave(&mut link, 1);
}

#[test]
fn disconnect_reaches_a_waiting_client() {
    let mut link = wireless_link(2);
    login(&mut link, 0);
    login(&mut link, 1);
    host(&mut link, 0);
    join(&mut link, 1, 0x61F0);

    // Client waits on the airwaves.
    ok(&mut link, 1, 0x27, &[]);
    set_data32(&mut link, 1, IDLE);
    write_siocnt(&mut link, 1, CNT_SLAVE);
    write_siocnt(&mut link, 1, CNT_SLAVE_GO);

    // Host kicks slot 0.
    ok(&mut link, 0, 0x30, &[1]);
    for _ in 0..8 {
        if siocnt(&mut link, 1) & 0x80 == 0 {
            break;
        }
        tick(&mut link);
    }
    assert_eq!(siocnt(&mut link, 1) & 0x80, 0, "disconnect event never arrived");
    // 0x129: one param word carrying the slot bitmask + reason.
    assert_eq!(data32(&mut link, 1), 0x9966_0129);
    dance_slave(&mut link, 1);
    set_data32(&mut link, 1, IDLE);
    write_siocnt(&mut link, 1, CNT_SLAVE_GO);
    for _ in 0..4 {
        if siocnt(&mut link, 1) & 0x80 == 0 {
            break;
        }
        tick(&mut link);
    }
    let param = data32(&mut link, 1);
    assert_eq!(param & 0xFF, 1, "slot bitmask should name slot 0: {param:08X}");
    dance_slave(&mut link, 1);

    // The host's roster frees the slot at the same tick.
    assert_eq!(ok(&mut link, 0, 0x1A, &[]), vec![]);
}

#[test]
fn four_players_share_one_host() {
    let mut link = wireless_link(4);
    for i in 0..4 {
        login(&mut link, i);
    }
    host(&mut link, 0);
    for c in 1..4 {
        let status = join(&mut link, c, 0x61F0);
        // Client c gets slot c-1 and its own device id.
        assert_eq!(status, ((c as u32 - 1) << 16) | (0x61F0 + c as u32));
    }
    // The host's roster: next slot 3, then all three clients in order.
    assert_eq!(
        ok(&mut link, 0, 0x14, &[]),
        vec![0x0000_0003, 0x0000_61F1, 0x0001_61F2, 0x0002_61F3]
    );

    // Everyone schedules an upload; one host frame collects them all.
    for c in 1..4 {
        ok(&mut link, c, 0x24, &[2 << (8 + 5 * (c - 1)), 0x1100 * c as u32]);
    }
    ok(&mut link, 0, 0x24, &[4, 0xF00D_F00D]);
    tick(&mut link);

    // Host: three 2-byte payloads, byte-concatenated in slot order.
    let host_rx = ok(&mut link, 0, 0x26, &[]);
    assert_eq!(host_rx[0], (2 << 8) | (2 << 13) | (2 << 18));
    // Bytes 00 11 | 00 22 | 00 33, concatenated then read as LE words.
    assert_eq!(host_rx[1..], [0x2200_1100, 0x0000_3300]);
    // Every client got the host's word.
    for c in 1..4 {
        assert_eq!(ok(&mut link, c, 0x26, &[]), vec![4, 0xF00D_F00D]);
    }
}

#[test]
fn wireless_snapshots_restore_exactly() {
    // Drive the same scripted exchange twice across a snapshot restore;
    // every digest and every read must repeat. This is the property
    // netplay rollback leans on.
    let mut link = wireless_link(2);
    login(&mut link, 0);
    login(&mut link, 1);
    host(&mut link, 0);
    join(&mut link, 1, 0x61F0);

    let snap = link.save().unwrap();
    let script = |link: &mut Link| -> (Vec<u32>, Vec<u32>, Vec<u32>) {
        ok(link, 1, 0x24, &[3 << 8, 0x0033_2211]);
        ok(link, 0, 0x24, &[4, 0x4444_3333]);
        tick(link);
        let host_rx = ok(link, 0, 0x26, &[]);
        let client_rx = ok(link, 1, 0x26, &[]);
        tick(link);
        let mut digests = Vec::new();
        for _ in 0..8 {
            tick(link);
            digests.push(link.save().unwrap().digest());
        }
        (host_rx, client_rx, digests)
    };

    let first = script(&mut link);
    link.load(&snap).unwrap();
    let second = script(&mut link);
    assert_eq!(first, second, "the wireless link did not restore exactly");
}

/// The RF-range merge: solo wireless machines keep their live adapter
/// sessions when a netplay link is built from their captures. The host
/// stays hosting, the scanner stays scanning, and the merge is nothing
/// but the other player appearing on the next poll — no adapter reset,
/// no re-login. The session-end continuation is the same event in
/// reverse: the peer walks out of range.
#[test]
fn merging_solo_links_brings_players_into_range() {
    // Machine A: logs in and hosts, alone on its airwaves.
    let mut a = wireless_link(1);
    login(&mut a, 0);
    host(&mut a, 0);
    for _ in 0..4 {
        tick(&mut a);
    }
    // Machine B: logs in and scans, seeing nobody.
    let mut b = wireless_link(1);
    login(&mut b, 0);
    ok(&mut b, 0, 0x1C, &[]);
    tick(&mut b);
    assert_eq!(ok(&mut b, 0, 0x1D, &[]), vec![], "nobody should be in range yet");

    // The room starts: capture both machines, adapter sessions included.
    let side = |link: &mut Link| BootSide {
        rom: testrom::build_idle(),
        save: link.export_save(0),
        state: link.capture_boot_state(0).unwrap(),
        adapter: link.capture_adapter_state(0),
    };
    let (side_a, side_b) = (side(&mut a), side(&mut b));
    assert!(side_a.adapter.is_some(), "wireless captures carry the adapter session");
    let build = || {
        Link::from_states(
            vec![
                BootSide {
                    rom: side_a.rom.clone(),
                    save: side_a.save.clone(),
                    state: side_a.state.clone(),
                    adapter: side_a.adapter.clone(),
                },
                BootSide {
                    rom: side_b.rom.clone(),
                    save: side_b.save.clone(),
                    state: side_b.state.clone(),
                    adapter: side_b.adapter.clone(),
                },
            ],
            None,
            Peripheral::Wireless,
        )
        .unwrap()
    };

    // Every peer builds the identical merged machine.
    let mut peer_a = build();
    let mut peer_b = build();
    for t in 0..60 {
        tick(&mut peer_a);
        tick(&mut peer_b);
        if t % 20 == 19 {
            let d_a = peer_a.save().unwrap().digest();
            let d_b = peer_b.save().unwrap().digest();
            assert_eq!(d_a, d_b, "merged links diverged at tick {t}");
        }
    }

    let mut link = peer_a;
    // A is still hosting (no re-login, no reconfiguration): its status
    // reports serving/open with its id, and B's ongoing scan — started
    // before the merge — now sees A's pre-merge broadcast.
    assert_eq!(ok(&mut link, 0, 0x13, &[]), vec![0x0200_61F0]);
    let servers = ok(&mut link, 1, 0x1D, &[]);
    assert_eq!(servers.len(), 7, "the host should appear in range: {servers:08X?}");
    assert_eq!(servers[0] & 0xFFFF, 0x61F0);
    assert_eq!(&servers[1..7], &BROADCAST, "the pre-merge broadcast data must survive");

    // The game-level flow continues normally from here.
    ok(&mut link, 1, 0x1E, &[]);
    ok(&mut link, 1, 0x1F, &[0x61F0]);
    tick(&mut link);
    assert_eq!(ok(&mut link, 1, 0x20, &[]), vec![0x0000_61F1]);
    ok(&mut link, 1, 0x21, &[]);
    ok(&mut link, 1, 0x24, &[3 << 8, 0x00CC_BBAA]);
    ok(&mut link, 0, 0x24, &[4, 0xDDCC_BBAA]);
    tick(&mut link);
    assert_eq!(ok(&mut link, 0, 0x26, &[]), vec![3 << 8, 0x00CC_BBAA]);
    assert_eq!(ok(&mut link, 1, 0x26, &[]), vec![4, 0xDDCC_BBAA]);

    // The session ends: B continues solo with its adapter session, and
    // the first RF tick resolves the dangling connection as the host
    // walking out of range.
    let solo_side = BootSide {
        rom: testrom::build_idle(),
        save: link.export_save(1),
        state: link.capture_boot_state(1).unwrap(),
        adapter: link.capture_adapter_state(1),
    };
    let mut alone = Link::from_states(vec![solo_side], None, Peripheral::Wireless).unwrap();
    // The first tick finishes the captured mid-frame remainder; the next
    // crosses RF ticks, whose liveness sweep resolves the dangling
    // connection.
    tick(&mut alone);
    tick(&mut alone);
    // Disconnected and idle — but still logged in and addressable.
    assert_eq!(ok(&mut alone, 0, 0x13, &[]), vec![0]);
}
