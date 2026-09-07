//! Percentile statistics over a batch of per-cycle samples, and their JSON rendering.

#[derive(Default)]
pub struct Stats {
    pub n: usize,
    pub min: f64,
    pub p50: f64,
    pub p99: f64,
    pub p999: f64,
    pub max: f64,
    pub mean: f64,
    /// Index of the worst sample, before sorting.
    pub max_at: usize,
}

/// Nearest-rank percentile over an already-sorted slice.
pub fn percentile(sorted: &[f64], q: f64) -> f64 {
    if sorted.is_empty() {
        return 0.0;
    }
    let rank = (q * sorted.len() as f64).ceil() as usize;
    sorted[rank.clamp(1, sorted.len()) - 1]
}

pub fn summarize(values: &mut [f64]) -> Stats {
    let mut stats = Stats {
        n: values.len(),
        ..Stats::default()
    };
    if values.is_empty() {
        return stats;
    }
    stats.mean = values.iter().sum::<f64>() / values.len() as f64;
    stats.max_at = values
        .iter()
        .enumerate()
        .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
        .map(|(i, _)| i)
        .unwrap_or(0);
    values.sort_by(|a, b| a.partial_cmp(b).unwrap());
    stats.min = values[0];
    stats.max = values[values.len() - 1];
    stats.p50 = percentile(values, 0.50);
    stats.p99 = percentile(values, 0.99);
    stats.p999 = percentile(values, 0.999);
    stats
}

pub fn stats_json(stats: &Stats) -> String {
    format!(
        "{{\"n\": {}, \"min\": {:.3}, \"p50\": {:.3}, \"p99\": {:.3}, \"p999\": {:.3}, \
         \"max\": {:.3}, \"mean\": {:.3}, \"max_at_cycle\": {}}}",
        stats.n, stats.min, stats.p50, stats.p99, stats.p999, stats.max, stats.mean, stats.max_at
    )
}
