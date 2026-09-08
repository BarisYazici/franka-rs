//! The command line of `commander_live.rs`.

use std::path::PathBuf;

use franka::{ControllerMode, OtgLimits, RealtimeConfig, TargetControlOptions};

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    Bridged,
    Raw,
}

pub struct Args {
    pub hostname: String,
    pub mode: Mode,
    pub from_stdin: bool,
    pub yes: bool,
    pub limits: OtgLimits,
    pub controller: ControllerMode,
    pub live: Option<String>,
    pub out: Option<PathBuf>,
    pub meshes: Option<PathBuf>,
}

pub fn parse() -> Result<Args, String> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mut parsed = Args {
        hostname: String::new(),
        mode: Mode::Bridged,
        from_stdin: false,
        yes: false,
        limits: TargetControlOptions::default().limits,
        controller: ControllerMode::CartesianImpedance,
        live: None,
        out: None,
        meshes: None,
    };
    let mut i = 0;
    let value = |i: &mut usize, flag: &str| {
        *i += 1;
        args.get(*i)
            .cloned()
            .ok_or_else(|| format!("{flag} needs a value"))
    };
    while i < args.len() {
        match args[i].as_str() {
            "--bridged" => parsed.mode = Mode::Bridged,
            "--raw" => parsed.mode = Mode::Raw,
            "--stdin" => parsed.from_stdin = true,
            "--yes" => parsed.yes = true,
            "--live" => parsed.live = Some(value(&mut i, "--live")?),
            "--out" => parsed.out = Some(PathBuf::from(value(&mut i, "--out")?)),
            "--meshes" => parsed.meshes = Some(PathBuf::from(value(&mut i, "--meshes")?)),
            "--budget" => {
                let text = value(&mut i, "--budget")?;
                let budget: Vec<f64> = text.split(',').filter_map(|v| v.parse().ok()).collect();
                let [max_velocity, max_acceleration, max_jerk] = budget[..] else {
                    return Err(format!("--budget {text:?}: want V,A,J"));
                };
                parsed.limits = OtgLimits {
                    max_velocity,
                    max_acceleration,
                    max_jerk,
                };
            }
            "--controller" => {
                parsed.controller = match value(&mut i, "--controller")?.as_str() {
                    "joint" => ControllerMode::JointImpedance,
                    "cartesian" => ControllerMode::CartesianImpedance,
                    other => return Err(format!("--controller {other:?}: want joint|cartesian")),
                }
            }
            other if parsed.hostname.is_empty() && !other.starts_with('-') => {
                parsed.hostname = other.to_string()
            }
            other => return Err(format!("unexpected argument {other:?}")),
        }
        i += 1;
    }
    if parsed.hostname.is_empty() {
        return Err("no robot hostname given".into());
    }
    if parsed.live.is_none() && parsed.out.is_none() {
        return Err("give --live ADDR (a listening viewer) and/or --out FILE (an .rrd)".into());
    }
    Ok(parsed)
}

/// `RealtimeConfig` from `FRANKA_REALTIME`, like the franka-rs examples.
pub fn realtime_config_from_env() -> RealtimeConfig {
    match std::env::var("FRANKA_REALTIME").as_deref() {
        Ok("ignore") => RealtimeConfig::Ignore,
        _ => RealtimeConfig::Enforce,
    }
}
