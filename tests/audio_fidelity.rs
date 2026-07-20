//! Audio savestate fidelity: the mixed PSG output must be indifferent to
//! the machinery rollback leans on. A per-tick `save`, a `save`+`load` of
//! the very same state, and frameskip on an unwatched core must each
//! leave the sample stream bit-identical to a plain run — the testrom's
//! tone makes any divergence (detune, phase shift, stale output latch)
//! visible where silence could hide it. These caught the write-only
//! frequency readback clobber, the stale square output latches, and the
//! absolute sampling-grid anchor in the mgba fork's deserialize path.

use mgba_siolink::{testrom, Link};

fn drain(link: &mut Link, out: &mut Vec<i16>) {
    let buf = link.core_mut(0).audio_buffer();
    let n = buf.available();
    let mut tmp = vec![0i16; n * 2];
    buf.read(&mut tmp, n);
    out.extend_from_slice(&tmp);
}

fn assert_streams_match(plain: &[i16], other: &[i16], what: &str) {
    assert_eq!(plain.len(), other.len(), "{what}: sample counts diverge");
    if let Some(at) = plain.iter().zip(other).position(|(a, b)| a != b) {
        panic!(
            "{what} perturbed the mix at interleaved sample {at} (tick ~{}): {} vs {}",
            at / 2 / 549,
            plain[at],
            other[at]
        );
    }
}

fn plain_run(rom: &[u8], ticks: u32) -> Vec<i16> {
    let mut out = Vec::new();
    let mut link = Link::new(vec![rom.to_vec(), rom.to_vec()]).unwrap();
    link.core_mut(0).audio_buffer().clear();
    for _ in 0..ticks {
        link.tick(&[0, 0]);
        drain(&mut link, &mut out);
    }
    out
}

#[test]
fn per_tick_save_does_not_perturb_the_mix() {
    mgba::log::install_default_logger();
    let rom = testrom::build();
    const TICKS: u32 = 20;
    let plain = plain_run(&rom, TICKS);

    let mut saved = Vec::new();
    let mut link = Link::new(vec![rom.clone(), rom]).unwrap();
    link.core_mut(0).audio_buffer().clear();
    for _ in 0..TICKS {
        link.tick(&[0, 0]);
        let _snap = link.save().unwrap();
        drain(&mut link, &mut saved);
    }

    assert_streams_match(&plain, &saved, "per-tick save");
}

#[test]
fn save_load_roundtrip_does_not_perturb_the_mix() {
    mgba::log::install_default_logger();
    let rom = testrom::build();
    const TICKS: u32 = 10;
    let plain = plain_run(&rom, TICKS * 2);

    let mut reloaded = Vec::new();
    let mut link = Link::new(vec![rom.clone(), rom]).unwrap();
    link.core_mut(0).audio_buffer().clear();
    for _ in 0..TICKS {
        link.tick(&[0, 0]);
        drain(&mut link, &mut reloaded);
    }
    // Save and immediately restore the very same state: the continuation
    // must be indistinguishable from never pausing.
    let snap = link.save().unwrap();
    link.load(&snap).unwrap();
    for _ in 0..TICKS {
        link.tick(&[0, 0]);
        drain(&mut link, &mut reloaded);
    }

    assert_streams_match(&plain, &reloaded, "save+load roundtrip");
}

#[test]
fn frameskip_on_the_other_core_does_not_perturb_the_mix() {
    mgba::log::install_default_logger();
    let rom = testrom::build();
    const TICKS: u32 = 20;
    let plain = plain_run(&rom, TICKS);

    let mut skipped = Vec::new();
    let mut link = Link::new(vec![rom.clone(), rom]).unwrap();
    link.set_frameskip(1, i32::MAX);
    link.core_mut(0).audio_buffer().clear();
    for _ in 0..TICKS {
        link.tick(&[0, 0]);
        drain(&mut link, &mut skipped);
    }

    assert_streams_match(&plain, &skipped, "frameskip");
}
