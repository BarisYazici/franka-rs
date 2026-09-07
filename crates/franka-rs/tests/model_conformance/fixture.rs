//! The fixture `tools/model-reference` produces, and the small array
//! conversions the suite reads it with.

use std::path::{Path, PathBuf};

#[allow(non_snake_case)]
#[derive(serde::Deserialize)]
pub struct Fixture {
    pub meta: Meta,
    pub load_configs: Vec<LoadConfig>,
    pub samples: Vec<Sample>,
}

#[derive(serde::Deserialize)]
pub struct Meta {
    pub libfranka_version: String,
    pub urdf_sha256: String,
    pub rng_seed: u64,
    pub random_sample_count: usize,
    pub gravity_earth: Vec<f64>,
    pub gravity_earth_alt: Vec<f64>,
    pub frames: Vec<String>,
}

#[allow(non_snake_case)]
#[derive(serde::Deserialize)]
pub struct LoadConfig {
    pub name: String,
    pub F_T_EE: Vec<f64>,
    pub EE_T_K: Vec<f64>,
    pub m_total: f64,
    pub F_x_Ctotal: Vec<f64>,
    pub I_total: Vec<f64>,
}

#[derive(serde::Deserialize)]
pub struct Sample {
    pub index: usize,
    pub kind: String,
    pub q: Vec<f64>,
    pub dq: Vec<f64>,
    pub cases: Vec<Case>,
}

#[derive(serde::Deserialize)]
pub struct Case {
    pub config: usize,
    pub pose: Vec<f64>,
    pub body_jacobian: Vec<f64>,
    pub zero_jacobian: Vec<f64>,
    pub mass: Vec<f64>,
    pub coriolis: Vec<f64>,
    pub gravity: Vec<f64>,
    pub gravity_alt: Vec<f64>,
}

pub fn data_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/data")
}

pub fn load_fixture() -> Fixture {
    let path = data_dir().join("model_reference_fr3.json");
    let text = std::fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("cannot read {}: {e}", path.display()));
    serde_json::from_str(&text).expect("fixture parses")
}

pub fn load_urdf() -> String {
    let path = data_dir().join("fr3.urdf");
    std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("cannot read {}: {e}", path.display()))
}

pub fn array7(v: &[f64]) -> [f64; 7] {
    v.try_into().expect("7 elements")
}

pub fn array3(v: &[f64]) -> [f64; 3] {
    v.try_into().expect("3 elements")
}

pub fn array9(v: &[f64]) -> [f64; 9] {
    v.try_into().expect("9 elements")
}

pub fn array16(v: &[f64]) -> [f64; 16] {
    v.try_into().expect("16 elements")
}
