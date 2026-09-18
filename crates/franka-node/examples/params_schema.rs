//! Prints an arm's `params/schema` reply as JSON, without a robot or a Zenoh session.
//!
//! It exists so that nothing outside the node has to describe the node's tunable parameters:
//! the panel's mock owner reads the dump this writes instead of carrying a table of its own,
//! and a lab script can see the bounds without a running arm. `schema/params-schema.json` is
//! that dump, and a unit test fails when it no longer matches what the node would serve.
//!
//! ```sh
//! cargo run -p franka-node --example params_schema > crates/franka-node/schema/params-schema.json
//! ```
//!
//! With no arguments it dumps the defaults every key of `[[arm]]` has, under the arm name and
//! boot id the committed dump uses. `--config <file> --arm <name>` dumps a real configuration's
//! instead, and `--fci 5` the schema of an arm speaking FCI v5 (an FER or Panda; the default is
//! v10, an FR3) -- the FCI version decides `derived.dq_limit` and nothing else here.

use franka::FciVersion;
use franka_node::config::{ArmConfig, NodeConfig};
use franka_node::msg::params::{SchemaMsg, DUMP_ARM, DUMP_BOOT_ID};

fn main() -> Result<(), String> {
    let mut args = std::env::args().skip(1);
    let (mut config, mut arm, mut version) = (None, None, FciVersion::V10);
    while let Some(flag) = args.next() {
        let mut value = || args.next().ok_or(format!("{flag} takes a value"));
        match flag.as_str() {
            "--config" => config = Some(value()?),
            "--arm" => arm = Some(value()?),
            "--fci" => {
                version = match value()?.as_str() {
                    "5" => FciVersion::V5,
                    "10" => FciVersion::V10,
                    other => return Err(format!("--fci is 5 or 10, not {other}")),
                }
            }
            other => return Err(format!("unknown argument {other}")),
        }
    }
    let config = match config {
        Some(path) => {
            let text = std::fs::read_to_string(&path).map_err(|e| format!("{path}: {e}"))?;
            let node: NodeConfig = text
                .parse()
                .map_err(|e: franka_node::config::ConfigError| e.to_string())?;
            let name = arm.as_deref();
            node.arms
                .into_iter()
                .find(|config| name.is_none_or(|name| config.name == name))
                .ok_or_else(|| format!("{path} has no arm {}", name.unwrap_or("at all")))?
        }
        None => defaults(arm.as_deref().unwrap_or(DUMP_ARM))?,
    };
    let schema = SchemaMsg::new(&config, version, DUMP_BOOT_ID);
    println!(
        "{}",
        serde_json::to_string_pretty(&schema).map_err(|e| e.to_string())?
    );
    Ok(())
}

/// An `[[arm]]` with nothing set but the two required keys, so every bound and every `derived`
/// number is the default the node ships.
fn defaults(name: &str) -> Result<ArmConfig, String> {
    let toml = format!("[[arm]]\nname = \"{name}\"\nhost = \"robot\"\n");
    let mut node: NodeConfig = toml
        .parse()
        .map_err(|e: franka_node::config::ConfigError| e.to_string())?;
    Ok(node.arms.remove(0))
}
