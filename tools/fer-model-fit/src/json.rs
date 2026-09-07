//! Minimal JSON number/array formatting for the fixture writer.

/// Shortest round-tripping decimal for a JSON number.
pub fn number(value: f64) -> String {
    format!("{value:?}")
}

pub fn json_array(values: &[f64]) -> String {
    let mut out = String::with_capacity(values.len() * 12 + 2);
    out.push('[');
    for (i, value) in values.iter().enumerate() {
        if i > 0 {
            out.push(',');
        }
        out.push_str(&number(*value));
    }
    out.push(']');
    out
}
