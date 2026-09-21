//! [`EpisodeMsg`], the JSON of `franka/<arm>/episode`: one sample when a session starts and
//! one when it ends, so a recorder in another process (a camera node) can open and close a
//! file of its own under the arm episode's `recording_id`.
//!
//! Both samples are best effort, as every publisher here drops rather than blocks, and a
//! killed node sends no end at all; `franka/<arm>/state` is what says whether a session runs.

use serde::{Deserialize, Serialize};

/// Longest episode token `enable` takes.
pub const EPISODE_TOKEN_MAX: usize = 128;

/// Checks an episode token a collector chose: `[A-Za-z0-9_-]{1,128}`.
///
/// It becomes the session's `recording_id` and part of every file name of the episode, here and
/// in every process that follows it, so anything else is refused rather than reaching a path.
pub fn check_episode_token(token: &str) -> Result<(), String> {
    let shape = || format!("episode {token:?} is not [A-Za-z0-9_-]{{1,{EPISODE_TOKEN_MAX}}}");
    if token.is_empty() || token.len() > EPISODE_TOKEN_MAX {
        return Err(shape());
    }
    let ok = |c: char| c.is_ascii_alphanumeric() || c == '_' || c == '-';
    if !token.chars().all(ok) {
        return Err(shape());
    }
    Ok(())
}

/// Start or end of an episode.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum EpisodePhase {
    Start,
    End,
}

/// The JSON of `franka/<arm>/episode`.
///
/// Unknown fields are accepted and `file` may be absent, so a subscriber built against this
/// version still reads a later one's extra fields. A phase it does not know is still an error,
/// so a new [`EpisodePhase`] would be a breaking change.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct EpisodeMsg {
    pub arm: String,
    /// The Rerun `RecordingId` of the session: with the `record` feature and a file open, the
    /// `.rrd`'s file stem, otherwise an id generated in the same shape. A file written under
    /// this id and `franka_rerun::APPLICATION_ID` loads as part of the same recording.
    pub recording_id: String,
    /// The arm's `.rrd` file name; `null` without one.
    #[serde(default)]
    pub file: Option<String>,
    /// The host's `CLOCK_MONOTONIC` at publish, the clock of `StateMsg::t_node_ns`.
    pub t_node_ns: u64,
    pub phase: EpisodePhase,
}

impl EpisodeMsg {
    pub fn to_json(&self) -> String {
        serde_json::to_string(self).expect("strings, an integer and an enum serialise")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_token_is_key_safe_and_bounded() {
        for token in ["pick-0042", "a", "A_b-9", &"x".repeat(EPISODE_TOKEN_MAX)] {
            assert_eq!(check_episode_token(token), Ok(()), "{token}");
        }
        for token in [
            "",
            "../etc",
            "a/b",
            "pick 42",
            "pick.0042",
            "naïve",
            &"x".repeat(129),
        ] {
            let error = check_episode_token(token).expect_err(token);
            assert!(error.contains("[A-Za-z0-9_-]{1,128}"), "{error}");
        }
    }

    #[test]
    fn episode_json_round_trips() {
        let msg = EpisodeMsg {
            arm: "L".into(),
            recording_id: "L-20260101T120000Z".into(),
            file: Some("L-20260101T120000Z.rrd".into()),
            t_node_ns: 12,
            phase: EpisodePhase::Start,
        };
        assert_eq!(
            msg.to_json(),
            r#"{"arm":"L","recording_id":"L-20260101T120000Z","file":"L-20260101T120000Z.rrd","t_node_ns":12,"phase":"start"}"#
        );
        let end = EpisodeMsg {
            file: None,
            phase: EpisodePhase::End,
            ..msg.clone()
        };
        assert!(end.to_json().contains(r#""file":null"#));
        assert!(end.to_json().ends_with(r#""phase":"end"}"#));
        assert_eq!(
            serde_json::from_str::<EpisodeMsg>(&msg.to_json()).unwrap(),
            msg
        );
        // A later version may add fields and leave `file` out; this one still reads it.
        let newer = r#"{"arm":"L","recording_id":"i","t_node_ns":1,"phase":"end","extra":7}"#;
        let decoded: EpisodeMsg = serde_json::from_str(newer).unwrap();
        assert_eq!(decoded.file, None);
        assert_eq!(decoded.phase, EpisodePhase::End);
    }
}
