//! The scripted, non-realtime commander of `nonrealtime_commander.rs`: steps of 5 cm with
//! irregular holds, a 2 s stall and a burst of 20 targets inside 100 ms, all inside a box
//! around the start, or lines of `x y z` from stdin. Every target is handed to a `publish`
//! closure; the example decides where it goes.

use std::io::BufRead;
use std::time::{Duration, Instant};

/// One scripted step, m; targets stay within +-`BOX` m of the start, at most `MAX_DROP` below.
pub const STEP: f64 = 0.05;
pub const BOX: f64 = 0.12;
pub const MAX_DROP: f64 = 0.05;

/// A step of `(dx, dy, dz)` metres held for some seconds; a stall of some seconds sending
/// nothing; a burst toggling x by [`STEP`] `count` times `spacing` seconds apart, then holding.
enum Event {
    Step(f64, f64, f64, f64),
    Stall(f64),
    Burst(usize, f64, f64),
}
use Event::{Burst, Stall, Step};

/// About 21 s of steps that never leave the box; the target ends back at the start.
#[rustfmt::skip]
const SCRIPT: &[Event] = &[
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

pub fn clamp(t: [f64; 3]) -> [f64; 3] {
    [
        t[0].clamp(-BOX, BOX),
        t[1].clamp(-BOX, BOX),
        t[2].clamp(-MAX_DROP, BOX),
    ]
}

/// Runs the script, or stdin, handing every clamped relative target to `publish`, which
/// returns `false` to end the commander early.
pub fn run_commander(from_stdin: bool, publish: &mut dyn FnMut([f64; 3]) -> bool) {
    let sleep = |s: f64| std::thread::sleep(Duration::from_secs_f64(s));
    if from_stdin {
        for line in std::io::stdin().lock().lines().map_while(Result::ok) {
            let v: Vec<f64> = line
                .split_whitespace()
                .filter_map(|f| f.parse().ok())
                .collect();
            match v[..] {
                [x, y, z] if publish(clamp([x, y, z])) => eprintln!("commander: -> {v:.3?}"),
                [_, _, _] => return,
                _ => eprintln!("commander: ignoring {line:?}, want `x y z` in metres"),
            }
        }
        return;
    }
    let (started, mut target) = (Instant::now(), [0.0f64; 3]);
    for event in SCRIPT {
        let t = started.elapsed().as_secs_f64();
        match *event {
            Step(dx, dy, dz, hold) => {
                target = clamp([target[0] + dx, target[1] + dy, target[2] + dz]);
                eprintln!("{t:7.3}s  commander: step to {target:.3?}, hold {hold} s");
                if !publish(target) {
                    return;
                }
                sleep(hold);
            }
            Stall(seconds) => {
                eprintln!("{t:7.3}s  commander: stall, nothing for {seconds} s");
                sleep(seconds);
            }
            Burst(count, spacing, hold) => {
                eprintln!("{t:7.3}s  commander: burst of {count} targets {spacing} s apart");
                for i in 0..count {
                    let mut toggled = target;
                    toggled[0] += STEP * f64::from(i % 2 == 0);
                    if !publish(clamp(toggled)) {
                        return;
                    }
                    sleep(spacing);
                }
                sleep(hold);
            }
        }
    }
}
