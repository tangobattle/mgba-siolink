//! Headless BN3 cut-in patch probe (local debugging tool, not for commit).
//!
//! Reads a script of lines:
//!   TICKS KEYS0 [KEYS1]     advance (keys as decimal mgba bitmasks:
//!                           A=1 B=2 SEL=4 START=8 R=16 L=32 U=64 D=128
//!                           Rb=256 Lb=512; `=` repeats previous column)
//!   !poke8 ADDR VAL         write byte on BOTH cores (keeps lockstep)
//!   !poke16 ADDR VAL        write halfword on both cores
//!   !dump TAG               dump both frames as PPMs now
//!   !state TAG              save a Link snapshot to <outdir>/TAG.snap? (no)
//! and prints cut-in telemetry at every script step boundary.

use mgba_siolink::{Link, LinkOptions, Peripheral, SideOptions};

const BATTLECTL: u32 = 0x02006ca0;
const CUTSCENE: u32 = 0x0200a810;
const UNIT0: u32 = 0x02037270;
const UNIT1: u32 = 0x02037344;
const CHIPBLOCKS: u32 = 0x02034060;
const STATE: u32 = 0x0203fe80;
const BDREQ: u32 = 0x020380c0;
const JUMPFLAG: u32 = 0x02036a50;

fn dump_frame(link: &mut Link, i: usize, dir: &str, tag: &str) {
    let Some(buf) = link.video_buffer(i) else { return };
    let mut out = Vec::with_capacity(240 * 160 * 3 + 20);
    out.extend_from_slice(b"P6\n240 160\n255\n");
    for px in buf.chunks_exact(4) {
        out.push(px[0]);
        out.push(px[1]);
        out.push(px[2]);
    }
    if out.len() != 240 * 160 * 3 + 15 {
        // 16-bit video buffer fallback
        out.truncate(15);
        for px in buf.chunks_exact(2) {
            let v = u16::from_le_bytes([px[0], px[1]]);
            out.push(((v & 0x1F) << 3) as u8);
            out.push((((v >> 5) & 0x1F) << 3) as u8);
            out.push((((v >> 10) & 0x1F) << 3) as u8);
        }
    }
    std::fs::write(format!("{dir}/{tag}-p{i}.ppm"), out).unwrap();
}

fn r8(link: &mut Link, i: usize, a: u32) -> u8 {
    link.core_mut(i).raw_read_8(a, -1)
}
fn r16(link: &mut Link, i: usize, a: u32) -> u16 {
    link.core_mut(i).raw_read_16(a, -1)
}
fn r32(link: &mut Link, i: usize, a: u32) -> u32 {
    link.core_mut(i).raw_read_32(a, -1)
}

fn telemetry(link: &mut Link, tick: u32) -> String {
    let mut cols = Vec::new();
    let mut stable: Vec<String> = Vec::new();
    for i in 0..link.num_players() {
        let kind = r8(link, i, BATTLECTL + 0x19);
        let mode = r16(link, i, BATTLECTL + 2);
        let frz = r8(link, i, BATTLECTL + 0x16);
        let ctype = r8(link, i, CUTSCENE + 4);
        let cph = r32(link, i, CUTSCENE);
        let u0f = r8(link, i, UNIT0 + 6);
        let u0s = r8(link, i, UNIT0 + 7);
        let u0p = r8(link, i, UNIT0 + 0x61);
        let u1f = r8(link, i, UNIT1 + 6);
        let u1s = r8(link, i, UNIT1 + 7);
        let u1p = r8(link, i, UNIT1 + 0x61);
        let magic = r32(link, i, STATE);
        let armed = r8(link, i, STATE + 5);
        let hold = r8(link, i, STATE + 0x1e);
        let pend = r8(link, i, STATE + 0x0d);
        let bridge = r8(link, i, STATE + 0x50);
        let handoff = r8(link, i, STATE + 0x1c);
        let standw = r32(link, i, STATE + 0x48);
        let u0hp = r16(link, i, UNIT0 + 0x24);
        let u1hp = r16(link, i, UNIT1 + 0x24);
        let u0p9 = r8(link, i, UNIT0 + 9);
        let u1p9 = r8(link, i, UNIT1 + 9);
        let u0ab = r16(link, i, UNIT0 + 0x2e);
        let u1ab = r16(link, i, UNIT1 + 0x2e);
        let u0ctx = r32(link, i, UNIT0 + 0x68);
        let u1ctx = r32(link, i, UNIT1 + 0x68);
        let u0acc = if u0ctx >> 24 == 2 { r32(link, i, u0ctx + 0x54) } else { 0 };
        let u1acc = if u1ctx >> 24 == 2 { r32(link, i, u1ctx + 0x54) } else { 0 };
        let localp = r8(link, i, BATTLECTL + 0x15);
        let u0pl = r8(link, i, UNIT0 + 0x16);
        let bldcnt = r16(link, i, 0x04000050);
        let bldy = r16(link, i, 0x04000054);
        let busy = r8(link, i, BDREQ + 2);
        let jf = r8(link, i, JUMPFLAG);
        cols.push(format!(
            "c{i}[L{localp}u{u0pl} kind={kind:02x} mode={mode:x} frz={frz:x} ct={ctype:02x} cph={cph:08x} \
             u0={u0f:02x}.{u0s:02x}p{u0p:02x}h{u0hp}s{u0p9:02x}b{u0ab:03x}x{u0acc:x}              u1={u1f:02x}.{u1s:02x}p{u1p:02x}h{u1hp}s{u1p9:02x}b{u1ab:03x}x{u1acc:x} \
             m={} a={armed} h={hold} p={pend} br={bridge} ho={handoff} sw={standw} \
             bld={bldcnt:04x}/{bldy:x} bd={busy}{jf}]",
            if magic == 0x21213343 { "M" } else { "-" }
        ));
        // phase-stable gameplay vector for the cross-core desync check (no
        // per-core identity, no PPU, no stall-phase counters)
        stable.push(format!(
            "{kind:02x}{mode:x}{frz:x}{ctype:02x}{cph:08x}{u0f:02x}{u0s:02x}{u0p:02x}{u0hp}{u0p9:02x}{u0ab:03x}{u1f:02x}{u1s:02x}{u1p:02x}{u1hp}{u1p9:02x}{u1ab:03x}{magic:08x}{armed}{pend}{busy}{jf}"
        ));
    }
    let mut digs = Vec::new();
    let mut regdigs: Vec<Vec<u32>> = Vec::new();
    let mut allbufs: Vec<Vec<Vec<u8>>> = Vec::new();
    for i in 0..link.num_players() {
        let mut h = crc32fast::Hasher::new();
        let mut regs = Vec::new();
        let mut regbufs: Vec<Vec<u8>> = Vec::new();
        for (a, len, mask) in [
            // 0xb4/0xb5: the link payload's session-tick counter is local-phase-relative
            (BATTLECTL, 0x100u32, &[0x15usize, 0x22, 0x24, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b, 0xb2, 0xb4, 0xb5][..]),
            (CUTSCENE, 0x40, &[][..]),
            (UNIT0, 0xd4, &[0x20usize, 0x9c, 0xa3][..]),
            (UNIT1, 0xd4, &[0x20usize, 0x9c, 0xa3][..]),
            // 0x04 S_FRESH (tick-phase), 0x13 S_BURST / 0x50 S_BRIDGE (frame-domain visuals)
            (STATE, 0x60, &[0x04usize, 0x13, 0x50][..]),
        ] {
            let mut buf = vec![0u8; len as usize];
            link.core_mut(i).raw_read_range(a, -1, &mut buf);
            for &m in mask {
                buf[m] = 0;
            }
            h.update(&buf);
            let mut rh = crc32fast::Hasher::new();
            rh.update(&buf);
            regs.push(rh.finalize());
            regbufs.push(buf);
        }
        digs.push(format!("{:08x}", h.finalize()));
        regdigs.push(regs);
        allbufs.push(regbufs);
    }
    let mark = if stable.windows(2).all(|w| w[0] == w[1]) {
        String::new()
    } else {
        let names = ["BCTL", "CUT", "U0", "U1", "STATE"];
        let mut bad = Vec::new();
        for r in 0..regdigs[0].len() {
            if regdigs.iter().any(|c| c[r] != regdigs[0][r]) {
                let detail: Vec<String> = allbufs[0][r]
                    .iter()
                    .zip(allbufs[1][r].iter())
                    .enumerate()
                    .filter(|(_, (x, y))| x != y)
                    .take(8)
                    .map(|(o, (x, y))| format!("+{o:02x}({x:02x}/{y:02x})"))
                    .collect();
                bad.push(format!("{}:{}", names[r], detail.join(",")));
            }
        }
        format!(" *** DIVERGED[{}] ***", bad.join(" "))
    };
    let mark = &mark;
    format!("t{tick:06} dig=[{}]{} {}", digs.join(","), mark, cols.join(" "))
}

fn main() {
    mgba::log::install_default_logger();
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 5 || args.len() % 2 == 0 {
        panic!("usage: bn3_probe <script> <outdir> <rom> <save|-> [<rom> <save|-> ...]");
    }
    let (script_path, outdir) = (&args[1], &args[2]);
    std::fs::create_dir_all(outdir).unwrap();

    let sides: Vec<SideOptions> = args[3..]
        .chunks(2)
        .map(|pair| SideOptions {
            rom: std::fs::read(&pair[0]).unwrap(),
            save: (pair[1] != "-").then(|| std::fs::read(&pair[1]).unwrap()),
        })
        .collect();
    let n_players = sides.len();
    let mut link = Link::with_options(LinkOptions {
        sides,
        rtc: Some(std::time::UNIX_EPOCH + std::time::Duration::from_secs(1_752_000_000)),
        peripheral: Peripheral::Cable,
    })
    .unwrap();

    let script = std::fs::read_to_string(script_path).unwrap();
    let mut tick_no = 0u32;
    for line in script.lines() {
        let line = line.split('#').next().unwrap().trim();
        if line.is_empty() {
            continue;
        }
        if let Some(rest) = line.strip_prefix('!') {
            let mut it = rest.split_whitespace();
            match it.next().unwrap() {
                "poke8" => {
                    let a = u32::from_str_radix(it.next().unwrap().trim_start_matches("0x"), 16).unwrap();
                    let v = u32::from_str_radix(it.next().unwrap().trim_start_matches("0x"), 16).unwrap();
                    for i in 0..n_players {
                        link.core_mut(i).raw_write_8(a, -1, v as u8);
                    }
                    println!("POKE8 {a:08x} = {v:02x}");
                }
                "poke16" => {
                    let a = u32::from_str_radix(it.next().unwrap().trim_start_matches("0x"), 16).unwrap();
                    let v = u32::from_str_radix(it.next().unwrap().trim_start_matches("0x"), 16).unwrap();
                    for i in 0..n_players {
                        link.core_mut(i).raw_write_16(a, -1, v as u16);
                    }
                    println!("POKE16 {a:08x} = {v:04x}");
                }
                "pokeu" => {
                    // !pokeu P OFF VAL: write byte at (unit of player P)+OFF
                    // on each core, resolving the unit slot per core.
                    let pl: u8 = it.next().unwrap().parse().unwrap();
                    let off = u32::from_str_radix(it.next().unwrap().trim_start_matches("0x"), 16).unwrap();
                    let v = u32::from_str_radix(it.next().unwrap().trim_start_matches("0x"), 16).unwrap();
                    for i in 0..n_players {
                        let unit = if link.core_mut(i).raw_read_8(UNIT0 + 0x16, -1) == pl {
                            UNIT0
                        } else {
                            UNIT1
                        };
                        link.core_mut(i).raw_write_8(unit + off, -1, v as u8);
                    }
                    println!("POKEU p{pl}+{off:02x} = {v:02x}");
                }
                "cmp" => {
                    // !cmp: byte-compare the digest blocks across cores,
                    // print differing offsets (block+off) to whitelist
                    // console-relative fields.
                    for (name, a, len) in [
                        ("BCTL", BATTLECTL, 0x100u32),
                        ("CUTS", CUTSCENE, 0x40),
                        ("U0", UNIT0, 0xd4),
                        ("U1", UNIT1, 0xd4),
                        ("ST", STATE, 0x60),
                    ] {
                        let mut b0 = vec![0u8; len as usize];
                        let mut b1 = vec![0u8; len as usize];
                        link.core_mut(0).raw_read_range(a, -1, &mut b0);
                        link.core_mut(1).raw_read_range(a, -1, &mut b1);
                        let diffs: Vec<String> = (0..len as usize)
                            .filter(|&i| b0[i] != b1[i])
                            .map(|i| format!("{name}+{i:02x}({:02x}/{:02x})", b0[i], b1[i]))
                            .collect();
                        if !diffs.is_empty() {
                            println!("CMP {}", diffs.join(" "));
                        }
                    }
                }
                "skew" => {
                    // !skew I N: run N frames on core I alone — the other
                    // core stands still, so core I renders stall frames
                    // with its battle logic waiting on the link, like a
                    // real-world link stall.
                    let i: usize = it.next().unwrap().parse().unwrap();
                    let n: u32 = it.next().unwrap().parse().unwrap();
                    for _ in 0..n {
                        link.core_mut(i).run_frame();
                    }
                    println!("SKEW c{i} +{n} frames");
                }
                "save" => {
                    // !save TAG ADDR LEN: dump core0 memory range to a file
                    let tag = it.next().unwrap().to_string();
                    let a = u32::from_str_radix(it.next().unwrap().trim_start_matches("0x"), 16).unwrap();
                    let n: usize = usize::from_str_radix(it.next().unwrap().trim_start_matches("0x"), 16).unwrap();
                    let mut buf = vec![0u8; n];
                    link.core_mut(0).raw_read_range(a, -1, &mut buf);
                    std::fs::write(format!("{outdir}/{tag}.bin"), &buf).unwrap();
                    println!("SAVE {tag} {a:08x}+{n:x}");
                }
                "md" => {
                    let a = u32::from_str_radix(it.next().unwrap().trim_start_matches("0x"), 16).unwrap();
                    let n: usize = it.next().unwrap().parse().unwrap();
                    for i in 0..n_players {
                        let mut bytes = Vec::new();
                        for k in 0..n {
                            bytes.push(format!("{:02x}", link.core_mut(i).raw_read_8(a + k as u32, -1)));
                        }
                        println!("MD c{i} {a:08x}: {}", bytes.join(""));
                    }
                }
                "dump" => {
                    let tag = it.next().unwrap_or("dump").to_string();
                    for i in 0..n_players {
                        dump_frame(&mut link, i, outdir, &format!("{tag}-{tick_no}"));
                    }
                    println!("DUMP {tag} at {tick_no}");
                }
                other => panic!("unknown directive {other}"),
            }
            continue;
        }
        let mut it = line.split_whitespace();
        let n: u32 = it.next().unwrap().parse().unwrap();
        let mut keys = vec![0u32; n_players];
        let mut prev = 0u32;
        for k in keys.iter_mut() {
            match it.next() {
                Some("=") => *k = prev,
                Some(v) => *k = v.parse().unwrap(),
                None => *k = 0,
            }
            prev = *k;
        }
        for _ in 0..n {
            if let Err(e) = link.try_tick(&keys) {
                println!("!! tick {tick_no}: link error: {e}");
                for i in 0..n_players {
                    dump_frame(&mut link, i, outdir, &format!("err-{tick_no}"));
                }
                return;
            }
            tick_no += 1;
        }
        println!("{}", telemetry(&mut link, tick_no));
    }
    for i in 0..n_players {
        dump_frame(&mut link, i, outdir, "final");
    }
    println!("done at {tick_no}");
}
