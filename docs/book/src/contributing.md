# Contributing

## The checks

Everything CI's `check` job runs, in order — none of it needs Docker, a network or a robot:

```sh
cargo fmt --all -- --check
cargo clippy --workspace --all-targets -- -D warnings
cargo clippy --workspace --all-targets --no-default-features -- -D warnings
cargo clippy -p franka-rs --all-targets --no-default-features -- -D warnings
cargo clippy -p franka-rs --all-targets --features serde -- -D warnings
cargo clippy -p franka-rs --all-targets --features model-library -- -D warnings
cargo test --workspace --lib
cargo test -p franka-rs --lib --features serde
cargo test -p franka-rs --lib --features model-library
cargo test -p franka-rs \
  --test wire_sizes --test model_conformance \
  --test wire_sizes_v5 --test fer_native_conformance \
  --test example_motion_generator
cargo test -p franka-rs --features model-library --test fer_model_conformance
cargo test --workspace --doc
cargo test -p franka-rs --doc
cargo test -p franka-rerun
cargo build -p franka-rerun --examples
RUSTDOCFLAGS="-D warnings" cargo doc --workspace --no-deps
RUSTDOCFLAGS="-D warnings" cargo doc -p franka-rs --no-deps
```

The standalone `-p franka-rs` checks cover the default native model without the optional
`model-library` loader. Workspace builds can enable that feature through the
`fer-model-fit` development tool, so CI also checks each path explicitly.

`cargo test --workspace --lib` includes the README-sync unit tests: the README's "Quick
example" block must stay byte-identical to the body of `main` in
`crates/franka-rs/examples/readme_joint_move.rs`, and at most 20 lines of code. Edit the
example and the README together, or the test fails.

Do **not** run `cargo test --tests`: it selects every integration binary in the workspace,
including the `sim_*.rs` files, which start a Docker container.

## The simulator lock

Only one franka-sim may run per host — the FCI ports (1337 robot, 1338 gripper) are fixed
by the protocol and only one client may hold them at a time. Take the machine-wide lock for
anything that touches the simulator:

```sh
flock .sim.lock env FRANKA_SIM_IMAGE=franka-sim:dev cargo test --release -p franka-rs \
  --test sim_handshake --test sim_commands --test sim_motions \
  --test sim_gripper --test sim_stop_and_reflex --test sim_target_control -- --test-threads=1
```

`--release` because the loops under test answer a 1 kHz state stream; unoptimised, the
impedance loop of target control needs about a millisecond per cycle.

The container harness has its own test, which starts and tears down a real container and
asserts on `docker ps`. No CI job runs it, because it cannot share a server with anything
else; run it by hand:

```sh
flock .sim.lock env FRANKA_SIM_IMAGE=franka-sim:dev cargo test -p franka-sim-test --test harness
```

Always `--test-threads=1`. Before starting anything, check that ports 1337 and 1338 are
free and no `franka-sim*` container is running. `SimServer::start` refuses to start a second
container, but two concurrent `cargo test` invocations will still fight over the ports of
an already-running server. See [Test against the simulator](./howto/simulator-tests.md).

## No robot in CI

**CI never talks to a robot, and neither should any test.** Every automated job runs
against the simulator or against nothing at all. Hardware runs are deliberate and manual: they
go through `bench/`'s `--hardware` harness, with a read-only `Idle` precondition probe, a
return-to-ready move before each run, and torque and end-effector deviation guards checked
outside the timed region. See `bench/README.md`.

If you are adding a test that would need an arm, add a simulator test and a
characterisation assertion for the gap instead.

## Working on the book

```sh
cargo install mdbook --locked      # no root needed
mdbook build docs/book             # output goes to target/book
mdbook serve docs/book             # live reload at http://localhost:3000
```

The build output is `target/book/`, which is already gitignored, so a local build leaves
nothing untracked behind.

The book has four parts, and a page belongs to exactly one of them. **Getting started** is
the shortest path to a moving arm; **Things to keep in mind** is what every user must know
before the first motion; **How-to** is one task per page, code first; **Reference** holds
the protocol tables, constants, measurements and the reasoning behind design decisions.
Wire-format and version differences, benchmark numbers and the long explanations go to
Reference and are linked from the other parts, not repeated there. Every claim must be
traceable to the code, a test or a measurement, with no marketing adjectives. `book.toml` keeps redirects from the 0.2.0 page
names, so a renamed page gets a redirect entry.

Every Rust snippet in the book is compiled by `mdbook test`. It needs the crate on
rustdoc's search path, and rustdoc fails with `E0464` if that path holds more than one
`libfranka-<hash>.rlib` — which an incrementally used `./target` usually does. So build
into a scratch directory:

```sh
export CARGO_TARGET_DIR=/tmp/franka-booktest
cargo build -p franka-rs -p franka-sim-test
mdbook test docs/book -L "$CARGO_TARGET_DIR/debug,$CARGO_TARGET_DIR/debug/deps"
unset CARGO_TARGET_DIR
```

`.github/workflows/docs.yml` does exactly that. Snippets carry a hidden
`# extern crate franka;` line for the same reason rustdoc needs it: `mdbook test` runs
them at edition 2015.

The changelog page is an mdBook `include` of the repository's `CHANGELOG.md`, so it is
single-sourced — edit the changelog, not the page. The [benchmarks
page](./reference/benchmarks.md) summarises the measurements and should stay in sync with
the harness's results.

Snippets are marked `no_run`: they are type-checked but never executed, because every one
of them would otherwise try to open a socket to a robot.

## Publishing

`.github/workflows/docs.yml` builds the book and `cargo doc --no-deps -p franka-rs`
(with `RUSTDOCFLAGS="-D warnings"`, so a broken intra-doc link fails the build), places the
rustdoc under `book/api/`, and deploys the result to GitHub Pages on every push to `main`.
It can also be run by hand from the Actions tab (`workflow_dispatch`).

> **Repository setting required.** The workflow uses `actions/deploy-pages`, which only
> works when **Settings → Pages → Build and deployment → Source** is set to **"GitHub
> Actions"** (not "Deploy from a branch"). This has to be set once per repository; until
> then the deploy step fails with a "Pages site not found" style error even though the
> build succeeds.

The site lands at <https://barisyazici.github.io/franka-rs/> and the API reference at
<https://barisyazici.github.io/franka-rs/api/franka/index.html>.

## Releasing

`.github/workflows/release.yml` runs on a `v*` tag: it builds the `franka-rs` wheels
(x86_64 and aarch64 manylinux, plus the sdist) with `PyO3/maturin-action`, uploads them to
PyPI through trusted publishing (`pypa/gh-action-pypi-publish`), attaches the `franka-node`
and `franka-cam` tarballs (cargo-zigbuild, aarch64 gnu and musl, x86_64 gnu) to the tag's
GitHub release, and publishes those of `franka-rs`, `franka-description`, `franka-rerun`,
`franka-cam` and `franka-node` whose version crates.io does not have yet, with a short-lived token from
crates.io's trusted publishing (`rust-lang/crates-io-auth-action`). No secret is stored
anywhere. From the Actions tab (`workflow_dispatch`) it builds the wheels and tarballs and
dry-runs the crate publish, uploading nothing.

Trusted publishing is configured on both registries for the GitHub repository
`BarisYazici/franka-rs`, workflow `release.yml`, environment `pypi`; no API token is stored
in the repository or its secrets. Trusted publishing only updates a crate that exists, so a
release that brings a new crate (0.4.0: `franka-description`, `franka-rerun`, `franka-node`,
`franka-cam`) is published by hand from the release commit with an API token and a current
stable cargo (multi-package publish needs 1.90), `cargo publish -p franka-rs -p
franka-description -p franka-rerun -p franka-cam -p franka-node` (cargo publishes them in
that dependency order), before the tag is pushed; then each new crate gets
the same trusted publisher on crates.io, and the tag's run skips the versions crates.io has.
The Python client `franka-node-client` (`crates/franka-node/python`, job `node-client`) goes to
PyPI in the same upload. Before the first tag that ships it, register a *pending* trusted
publisher for `franka-node-client` on PyPI with the same repository, workflow `release.yml` and
environment `pypi`, since trusted publishing cannot create the project otherwise.

A release is: bump `version` in the root `Cargo.toml`'s `[workspace.package]` (all five crates
and the wheel share it, and `crates/franka-node/python/pyproject.toml` repeats it, which a test
checks; on a minor bump also the `version = "0.x"` of `franka`,
`franka-description` and `franka-rerun` in `[workspace.dependencies]`), date the section in `CHANGELOG.md`, and push a
`v*` tag matching it (the tarball job refuses another, since `cargo binstall` looks for
`v<version>`).

## Commit conventions

Plain commit messages, no trailers: a subject line in the imperative mood saying what the
commit does, and a body explaining why when that is not obvious from the diff.
