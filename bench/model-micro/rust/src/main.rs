//! Offline microbenchmark of `franka::Model` (franka-rs native backend).
//!
//! No robot and no simulator are involved. The C++ counterpart
//! (`../../cpp/main.cpp`) draws the random `(q, dq)` inputs and writes them to `states.json`
//! together with `reference.bin`, its own outputs; this program **replays exactly those
//! inputs**, times the same five calls with the same code shape, and cross-checks every
//! output against the reference.
//!
//! Timing uses `CLOCK_MONOTONIC` around each individual call and the same nearest-rank
//! percentile function as `bench/rust/src/main.rs`.

use std::io::Read;

use franka::model::{Frame, Model};

/// Doubles per sample in `reference.bin`, in this order:
/// `mass(49), coriolis(7), coriolis_rnea(7), gravity(7), zero_jacobian(42), pose(16)`.
const REF_DOUBLES_PER_SAMPLE: usize = 49 + 7 + 7 + 7 + 42 + 16;

fn monotonic_ns() -> i64 {
    let mut ts = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    // SAFETY: `ts` is a valid, initialised `timespec` and `CLOCK_MONOTONIC` always exists.
    unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts) };
    ts.tv_sec * 1_000_000_000 + ts.tv_nsec
}

#[derive(Default)]
struct Stats {
    n: usize,
    min: f64,
    p50: f64,
    p99: f64,
    p999: f64,
    max: f64,
    mean: f64,
}

/// Nearest-rank percentile over an already-sorted slice.
fn percentile(sorted: &[f64], q: f64) -> f64 {
    if sorted.is_empty() {
        return 0.0;
    }
    let rank = (q * sorted.len() as f64).ceil() as usize;
    sorted[rank.clamp(1, sorted.len()) - 1]
}

fn summarize(values: &mut [f64]) -> Stats {
    let mut stats = Stats {
        n: values.len(),
        ..Stats::default()
    };
    if values.is_empty() {
        return stats;
    }
    stats.mean = values.iter().sum::<f64>() / values.len() as f64;
    values.sort_by(|a, b| a.partial_cmp(b).unwrap());
    stats.min = values[0];
    stats.max = values[values.len() - 1];
    stats.p50 = percentile(values, 0.50);
    stats.p99 = percentile(values, 0.99);
    stats.p999 = percentile(values, 0.999);
    stats
}

fn stats_json(stats: &Stats) -> String {
    format!(
        "{{\"n\": {}, \"min\": {:.4}, \"p50\": {:.4}, \"p99\": {:.4}, \"p999\": {:.4}, \
         \"max\": {:.4}, \"mean\": {:.4}}}",
        stats.n, stats.min, stats.p50, stats.p99, stats.p999, stats.max, stats.mean
    )
}

fn json_f64_array<const N: usize>(value: &serde_json::Value, key: &str) -> [f64; N] {
    let array = value
        .get(key)
        .and_then(|v| v.as_array())
        .unwrap_or_else(|| panic!("states.json: missing array {key}"));
    assert_eq!(array.len(), N, "states.json: {key} has the wrong length");
    let mut out = [0.0; N];
    for (slot, item) in out.iter_mut().zip(array) {
        *slot = item.as_f64().expect("states.json: non-numeric entry");
    }
    out
}

struct Args {
    dir: String,
    urdf: String,
}

fn parse_args() -> Args {
    let argv: Vec<String> = std::env::args().collect();
    let mut args = Args {
        dir: ".".to_owned(),
        urdf: String::new(),
    };
    let mut i = 1;
    while i < argv.len() {
        let flag = argv[i].clone();
        match flag.as_str() {
            "--dir" => {
                i += 1;
                args.dir = argv
                    .get(i)
                    .cloned()
                    .unwrap_or_else(|| usage("missing --dir"));
            }
            other if other.starts_with('-') => usage(&format!("unknown flag {other}")),
            other => args.urdf = other.to_owned(),
        }
        i += 1;
    }
    if args.urdf.is_empty() {
        usage("a URDF path is required");
    }
    args
}

fn usage(message: &str) -> ! {
    eprintln!("error: {message}");
    eprintln!("usage: model_micro_rust <fr3.urdf> [--dir DIR]");
    eprintln!("  DIR must already hold states.json and reference.bin written by the C++ side");
    std::process::exit(2);
}

#[allow(non_snake_case)]
fn main() {
    let args = parse_args();

    let urdf = std::fs::read_to_string(&args.urdf)
        .unwrap_or_else(|e| panic!("cannot read {}: {e}", args.urdf));
    let model = Model::from_urdf(&urdf).expect("Model::from_urdf");

    let states_path = format!("{}/states.json", args.dir);
    let states: serde_json::Value = serde_json::from_str(
        &std::fs::read_to_string(&states_path)
            .unwrap_or_else(|e| panic!("cannot read {states_path}: {e}")),
    )
    .expect("states.json is not valid JSON");

    let warmup = states["meta"]["warmup"].as_u64().expect("meta.warmup") as usize;
    let count = states["meta"]["count"].as_u64().expect("meta.count") as usize;
    let per_sample = states["meta"]["ref_doubles_per_sample"]
        .as_u64()
        .expect("meta.ref_doubles_per_sample") as usize;
    assert_eq!(
        per_sample, REF_DOUBLES_PER_SAMPLE,
        "reference layout changed on the C++ side"
    );

    let config = &states["config"];
    let F_T_EE: [f64; 16] = json_f64_array(config, "F_T_EE");
    let EE_T_K: [f64; 16] = json_f64_array(config, "EE_T_K");
    let I_total: [f64; 9] = json_f64_array(config, "I_total");
    let F_x_Ctotal: [f64; 3] = json_f64_array(config, "F_x_Ctotal");
    let gravity_earth: [f64; 3] = json_f64_array(config, "gravity_earth");
    let m_total = config["m_total"].as_f64().expect("config.m_total");

    let samples = states["samples"].as_array().expect("samples");
    assert_eq!(samples.len(), warmup + count, "unexpected sample count");
    let qs: Vec<[f64; 7]> = samples.iter().map(|s| json_f64_array(s, "q")).collect();
    let dqs: Vec<[f64; 7]> = samples.iter().map(|s| json_f64_array(s, "dq")).collect();

    // Preallocated: nothing is allocated or printed between the first and the last timestamp.
    let mut t_mass = vec![0.0f64; count];
    let mut t_coriolis = vec![0.0f64; count];
    let mut t_gravity = vec![0.0f64; count];
    let mut t_jacobian = vec![0.0f64; count];
    let mut t_pose = vec![0.0f64; count];
    let mut t_total = vec![0.0f64; count];
    let mut outputs = vec![0.0f64; count * REF_DOUBLES_PER_SAMPLE];
    let mut checksum = 0.0f64;

    for i in 0..warmup + count {
        let q = &qs[i];
        let dq = &dqs[i];

        // Same order as the `model` control variant: mass, coriolis, gravity, Jacobian, pose.
        let t0 = monotonic_ns();
        let mass = model.mass_q(q, &I_total, m_total, &F_x_Ctotal);
        let t1 = monotonic_ns();
        let coriolis = model.coriolis_q(q, dq, &I_total, m_total, &F_x_Ctotal, &gravity_earth);
        let t2 = monotonic_ns();
        let gravity = model.gravity_q(q, m_total, &F_x_Ctotal, &gravity_earth);
        let t3 = monotonic_ns();
        let jacobian = model.zero_jacobian_q(Frame::EndEffector, q, &F_T_EE, &EE_T_K);
        let t4 = monotonic_ns();
        let pose = model.pose_q(Frame::EndEffector, q, &F_T_EE, &EE_T_K);
        let t5 = monotonic_ns();

        if i < warmup {
            checksum += mass[0] + coriolis[0] + gravity[0] + jacobian[0] + pose[0];
            continue;
        }
        let k = i - warmup;
        t_mass[k] = (t1 - t0) as f64 * 1e-3;
        t_coriolis[k] = (t2 - t1) as f64 * 1e-3;
        t_gravity[k] = (t3 - t2) as f64 * 1e-3;
        t_jacobian[k] = (t4 - t3) as f64 * 1e-3;
        t_pose[k] = (t5 - t4) as f64 * 1e-3;
        t_total[k] = (t5 - t0) as f64 * 1e-3;

        let out = &mut outputs[k * REF_DOUBLES_PER_SAMPLE..(k + 1) * REF_DOUBLES_PER_SAMPLE];
        out[..49].copy_from_slice(&mass);
        // franka-rs evaluates a single coriolis algorithm (rnea - gravity), so the same
        // vector is compared against both C++ overloads.
        out[49..56].copy_from_slice(&coriolis);
        out[56..63].copy_from_slice(&coriolis);
        out[63..70].copy_from_slice(&gravity);
        out[70..112].copy_from_slice(&jacobian);
        out[112..128].copy_from_slice(&pose);
    }

    // --- cross-check against the C++ reference ---------------------------------------
    let reference_path = format!("{}/reference.bin", args.dir);
    let mut reference_bytes = Vec::new();
    std::fs::File::open(&reference_path)
        .unwrap_or_else(|e| panic!("cannot read {reference_path}: {e}"))
        .read_to_end(&mut reference_bytes)
        .expect("read reference.bin");
    assert_eq!(
        reference_bytes.len(),
        count * REF_DOUBLES_PER_SAMPLE * 8,
        "reference.bin has the wrong size"
    );

    // (label, offset, length) of every block in one sample's reference record.
    let blocks: [(&str, usize, usize); 6] = [
        ("mass", 0, 49),
        ("coriolis", 49, 7),
        ("coriolis_rnea", 56, 7),
        ("gravity", 63, 7),
        ("zero_jacobian", 70, 42),
        ("pose", 112, 16),
    ];
    let mut max_diff = [0.0f64; 6];
    let mut worst_sample = [0usize; 6];
    for k in 0..count {
        for (b, (_, offset, len)) in blocks.iter().enumerate() {
            for j in 0..*len {
                let index = k * REF_DOUBLES_PER_SAMPLE + offset + j;
                let mut buf = [0u8; 8];
                buf.copy_from_slice(&reference_bytes[index * 8..index * 8 + 8]);
                let diff = (outputs[index] - f64::from_le_bytes(buf)).abs();
                if diff > max_diff[b] {
                    max_diff[b] = diff;
                    worst_sample[b] = k;
                }
            }
        }
    }

    let s_mass = summarize(&mut t_mass);
    let s_coriolis = summarize(&mut t_coriolis);
    let s_gravity = summarize(&mut t_gravity);
    let s_jacobian = summarize(&mut t_jacobian);
    let s_pose = summarize(&mut t_pose);
    let s_total = summarize(&mut t_total);

    let agreement = blocks
        .iter()
        .enumerate()
        .map(|(b, (label, _, _))| {
            format!(
                "    \"{label}\": {{\"max_abs_diff\": {:.3e}, \"worst_sample\": {}}}",
                max_diff[b], worst_sample[b]
            )
        })
        .collect::<Vec<_>>()
        .join(",\n");

    let json = format!(
        "{{\n  \"lang\": \"rust\",\n  \"library\": \"franka-rs 0.1.0 (NativeBackend)\",\n  \
         \"count\": {count},\n  \"warmup\": {warmup},\n  \"urdf\": \"{urdf_path}\",\n  \
         \"checksum\": {checksum},\n  \"calls\": {{\n    \
         \"mass\": {mass},\n    \"coriolis\": {coriolis},\n    \"gravity\": {gravity},\n    \
         \"zero_jacobian\": {jacobian},\n    \"pose\": {pose}\n  }},\n  \
         \"total\": {total},\n  \"agreement_vs_cpp\": {{\n{agreement}\n  }}\n}}\n",
        count = count,
        warmup = warmup,
        urdf_path = args.urdf,
        checksum = checksum,
        mass = stats_json(&s_mass),
        coriolis = stats_json(&s_coriolis),
        gravity = stats_json(&s_gravity),
        jacobian = stats_json(&s_jacobian),
        pose = stats_json(&s_pose),
        total = stats_json(&s_total),
        agreement = agreement,
    );

    print!("{json}");
    let out_path = format!("{}/rust.json", args.dir);
    std::fs::write(&out_path, &json).unwrap_or_else(|e| panic!("cannot write {out_path}: {e}"));
}
