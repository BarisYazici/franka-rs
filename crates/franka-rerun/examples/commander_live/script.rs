//! The commander side of `commander_live.rs`, taken from franka-rs's
//! `examples/nonrealtime_commander.rs`: the scripted sequence of steps, stalls and a burst,
//! and the stdin commander. Each hands its (clamped, relative) targets to a `publish`
//! closure -- `set_position` on a target control in bridged mode, a slot in raw mode -- and
//! what the original printed to stderr goes through a [`Sink`], so the live example can log
//! it into the recording.

use std::io::BufRead;
use std::time::{Duration, Instant};

/// Size of one scripted step, m.
pub const STEP: f64 = 0.05;
/// Targets are clamped into +-`BOX` m around the start position, never more than `MAX_DROP` m
/// below it.
pub const BOX: f64 = 0.12;
pub const MAX_DROP: f64 = 0.05;

/// A step of `(dx, dy, dz)` metres held for some seconds; a stall of some seconds sending
/// nothing; a burst toggling x by [`STEP`] `count` times `spacing` seconds apart, then holding.
pub enum Event {
    Step(f64, f64, f64, f64),
    Stall(f64),
    Burst(usize, f64, f64),
}
use Event::{Burst, Stall, Step};

/// About 21 s of steps that never leave the box: `x` and `y` stay within +-0.10 m (the burst
/// reaches 0.10), `z` within `[-0.05, +0.05]`, and the target ends back at the start.
#[rustfmt::skip]
pub const SCRIPT: &[Event] = &[
    Stall(0.5),
    Step(STEP, 0.0, 0.0, 0.8), Step(0.0, STEP, 0.0, 0.3), Step(0.0, 0.0, STEP, 1.2),
    Step(-STEP, 0.0, 0.0, 0.5), Step(-STEP, 0.0, 0.0, 1.5), Step(0.0, -STEP, 0.0, 0.2),
    Step(0.0, -STEP, 0.0, 0.9), Step(0.0, 0.0, -STEP, 0.4), Step(0.0, 0.0, -STEP, 1.1),
    Step(STEP, 0.0, 0.0, 0.6), Step(0.0, 0.0, STEP, 1.3),
    Stall(2.0),
    Step(STEP, 0.0, 0.0, 0.7), Step(0.0, STEP, 0.0, 0.25), Step(0.0, STEP, 0.0, 1.4),
    Step(0.0, 0.0, STEP, 0.35),
    Burst(20, 0.005, 1.0),
    Step(-STEP, 0.0, 0.0, 0.9), Step(STEP, 0.0, 0.0, 0.55), Step(-STEP, 0.0, 0.0, 0.6),
    Step(0.0, -STEP, 0.0, 0.45), Step(0.0, 0.0, -STEP, 1.0),
];

/// Clamps a relative target into the box.
pub fn clamp(t: [f64; 3]) -> [f64; 3] {
    [
        t[0].clamp(-BOX, BOX),
        t[1].clamp(-BOX, BOX),
        t[2].clamp(-MAX_DROP, BOX),
    ]
}

/// Where the commander reports: every target it published (clamped, relative to the start)
/// and its events (`warning` for stalls and bursts).
pub trait Sink {
    fn published(&mut self, target: [f64; 3]);
    fn event(&mut self, warning: bool, text: String);
}

/// Where the targets go; `false` ends the commander early.
pub type Publish<'a> = &'a mut dyn FnMut([f64; 3]) -> bool;

/// The scripted commander: runs [`SCRIPT`] against the clock.
pub fn run_script(publish: Publish<'_>, sink: &mut impl Sink) {
    let started = Instant::now();
    let mut target = [0.0f64; 3];
    for event in SCRIPT {
        let elapsed = started.elapsed().as_secs_f64();
        match *event {
            Step(dx, dy, dz, hold) => {
                target = clamp([target[0] + dx, target[1] + dy, target[2] + dz]);
                if !publish(target) {
                    return;
                }
                sink.published(target);
                let text = format!("{elapsed:.2} s: step to {target:.3?}, hold {hold} s");
                sink.event(false, text);
                pause(hold);
            }
            Stall(seconds) => {
                sink.event(
                    true,
                    format!("{elapsed:.2} s: stall, nothing for {seconds} s"),
                );
                pause(seconds);
            }
            Burst(count, spacing, hold) => {
                let text = format!("{elapsed:.2} s: burst of {count} targets {spacing} s apart");
                sink.event(true, text);
                for i in 0..count {
                    let mut toggled = target;
                    toggled[0] += STEP * f64::from(i % 2 == 0);
                    let toggled = clamp(toggled);
                    if !publish(toggled) {
                        return;
                    }
                    sink.published(toggled);
                    pause(spacing);
                }
                pause(hold);
            }
        }
    }
}

fn pause(seconds: f64) {
    std::thread::sleep(Duration::from_secs_f64(seconds));
}

/// The interactive commander: one `x y z` line per target until stdin closes.
pub fn run_stdin(publish: Publish<'_>, sink: &mut impl Sink) {
    for line in std::io::stdin().lock().lines().map_while(Result::ok) {
        let mut fields = line.split_whitespace().map(str::parse::<f64>);
        match (fields.next(), fields.next(), fields.next()) {
            (Some(Ok(x)), Some(Ok(y)), Some(Ok(z))) => {
                let target = clamp([x, y, z]);
                if !publish(target) {
                    return;
                }
                sink.published(target);
                sink.event(false, format!("stdin: step to {target:.3?}"));
            }
            _ => sink.event(
                true,
                format!("stdin: ignoring {line:?}, want `x y z` in metres"),
            ),
        }
    }
}
