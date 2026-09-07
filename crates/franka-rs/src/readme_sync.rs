//! Keeps README.md's "Quick example" block and `examples/readme_joint_move.rs` in lockstep.
//!
//! The block must be the byte-identical body of the example's `main`, and at most 20 lines of
//! code -- the README's first example is meant to stay a 20-line joint move. Living in the
//! library
//! rather than in `tests/` means this runs as part of the Docker-free
//! `cargo test -p franka-rs --lib` that CI's `check` job executes.

/// The README lives at the workspace root, one level above this crate.
const README: &str = include_str!("../../../README.md");
const EXAMPLE: &str = include_str!("../examples/readme_joint_move.rs");

/// The body of `main` in `examples/readme_joint_move.rs`, verbatim.
fn example_main_body() -> &'static str {
    const HEAD: &str = "fn main() -> franka::FrankaResult<()> {\n";
    const TAIL: &str = "\n    Ok(())\n";
    let start = EXAMPLE.find(HEAD).expect("example has no `fn main`") + HEAD.len();
    let end = EXAMPLE[start..]
        .find(TAIL)
        .expect("example has no `Ok(())`")
        + start
        + 1;
    &EXAMPLE[start..end]
}

/// The contents of the first ```` ```rust ```` block after the "Quick example" heading.
fn readme_quick_example() -> &'static str {
    let section = README
        .find("## Quick example")
        .expect("README has no \"Quick example\" section");
    let start = README[section..]
        .find("```rust\n")
        .expect("no fenced block")
        + section
        + 8;
    let end = README[start..].find("```\n").expect("unterminated fence") + start;
    &README[start..end]
}

#[test]
fn readme_quick_example_matches_this_file() {
    assert_eq!(
        readme_quick_example(),
        example_main_body(),
        "README.md's \"Quick example\" block must be byte-identical to the body of `main` \
         in crates/franka-rs/examples/readme_joint_move.rs"
    );
}

#[test]
fn readme_quick_example_is_at_most_twenty_lines_of_code() {
    let code_lines = readme_quick_example()
        .lines()
        .filter(|line| !line.trim().is_empty() && !line.trim_start().starts_with("//"))
        .count();
    assert!(
        code_lines <= 20,
        "the README's joint-move example is {code_lines} lines of code, at most 20 allowed"
    );
}
