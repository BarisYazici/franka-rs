//! The client's command line.

pub const USAGE: &str = "\
usage: client <arm> [--connect tcp/host:7447] [--hz 20] [--dz 0.05] [--seconds 4] [--mode joints]
                    [--episode NAME]
       client <arm> home [--connect tcp/host:7447] [--speed 0.2] [--episode NAME]
       client <arm> recover|stop|release [--connect tcp/host:7447] [--client-id N]
       client <arm> gripper <width_m> | gripper grasp <width_m> <force_n> | gripper home
       (release needs --client-id, the holder's id; stop and recover need no lease;
        --episode NAME, [A-Za-z0-9_-]{1,128}, names the session: its recording id is that
        name and its file is NAME-<arm>.rrd, so every arm enabled with one name writes
        into one recording)";

/// One gripper command, sent with a lease of its own.
pub enum GripperCmd {
    Width(f64),
    Grasp(f64, f64),
    Home,
}

pub struct Sine {
    pub hz: f64,
    pub dz: f64,
    pub seconds: f64,
    pub joints: bool,
}

/// One command; `client_id` is the holder's for `release`, else this process's id is sent.
pub struct Verb {
    pub name: String,
    pub client_id: Option<u32>,
}

pub enum Mode {
    Sine(Sine),
    Verb(Verb),
    Home(Option<f64>),
    Gripper(GripperCmd),
}

pub struct Args {
    pub arm: String,
    pub connect: Option<String>,
    pub mode: Mode,
    /// `--episode NAME`: the session's name, sent with `enable` or `home`.
    pub episode: Option<String>,
}

/// `<width_m>`, `grasp <width_m> <force_n>` or `home` after `gripper`.
fn parse_gripper(args: &mut impl Iterator<Item = String>) -> Result<GripperCmd, String> {
    let metres = |what: &str, text: Option<String>| {
        text.and_then(|t| t.parse::<f64>().ok())
            .filter(|x| x.is_finite() && *x >= 0.0)
            .ok_or_else(|| format!("gripper: {what} must be a non-negative number"))
    };
    match args.next().as_deref() {
        Some("home") => Ok(GripperCmd::Home),
        Some("grasp") => {
            let width = metres("<width_m>", args.next())?;
            let force = metres("<force_n>", args.next())?;
            Ok(GripperCmd::Grasp(width, force))
        }
        other => Ok(GripperCmd::Width(metres(
            "<width_m>",
            other.map(String::from),
        )?)),
    }
}

pub fn parse_args() -> Result<Args, String> {
    let mut args = std::env::args().skip(1).peekable();
    let arm = args.next().ok_or(USAGE)?;
    let gripper = args
        .next_if(|a| a == "gripper")
        .map(|_| parse_gripper(&mut args))
        .transpose()?;
    let verb = args.next_if(|a| matches!(a.as_str(), "recover" | "stop" | "release" | "home"));
    let (mut connect, mut client_id, mut speed, mut joints) = (None, None, None, false);
    let mut episode = None;
    let (mut hz, mut dz, mut seconds) = (20.0, 0.05, 4.0);
    while let Some(flag) = args.next() {
        let value = args.next().ok_or_else(|| format!("{flag} needs a value"))?;
        let number = |what: &str| {
            value
                .parse::<f64>()
                .map_err(|_| format!("{what}: not a number: {value}"))
        };
        match (flag.as_str(), verb.as_deref()) {
            ("--connect", _) => connect = Some(value.clone()),
            (_, _) if gripper.is_some() => return Err(USAGE.into()),
            ("--client-id", Some("recover" | "stop" | "release")) => {
                client_id = Some(
                    value
                        .parse::<u32>()
                        .ok()
                        .filter(|id| *id != 0)
                        .ok_or_else(|| format!("--client-id: not a non-zero u32: {value}"))?,
                )
            }
            ("--speed", Some("home")) => speed = Some(number("--speed")?),
            ("--episode", None | Some("home")) => episode = Some(value.clone()),
            ("--hz", None) => hz = number("--hz")?,
            ("--dz", None) => dz = number("--dz")?,
            ("--seconds", None) => seconds = number("--seconds")?,
            ("--mode", None) if value == "joints" => joints = true,
            ("--mode", None) if value == "cartesian" => joints = false,
            _ => return Err(USAGE.into()),
        }
    }
    let mode = match (gripper, verb) {
        (Some(cmd), None) => Mode::Gripper(cmd),
        (Some(_), Some(_)) => return Err(USAGE.into()),
        (None, Some(name)) if name == "home" => Mode::Home(speed),
        (None, Some(name)) if name == "release" && client_id.is_none() => {
            return Err("release needs --client-id N, the holder's id".into());
        }
        (None, Some(name)) => Mode::Verb(Verb { name, client_id }),
        (None, None) if hz > 0.0 && dz.is_finite() && dz.abs() <= 0.2 && seconds > 0.0 => {
            Mode::Sine(Sine {
                hz,
                dz,
                seconds,
                joints,
            })
        }
        (None, None) => {
            return Err("--hz and --seconds must be positive, |--dz| at most 0.2".into())
        }
    };
    Ok(Args {
        arm,
        connect,
        mode,
        episode,
    })
}
