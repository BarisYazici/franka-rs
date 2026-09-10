# Build for another machine

At the end of this page you have an `aarch64` build of the crate and its examples for a
Raspberry Pi 4 or 5 or another 64-bit ARM Linux machine, made and smoke-tested on an x86-64 box.
The target needs a 64-bit system (Raspberry Pi OS 64-bit or Ubuntu arm64); 32-bit `armhf`
systems are not supported.

Validation status: the `aarch64` binaries of both targets below have run the crate's test
suite under `qemu-aarch64-static` against the simulator, with byte-identical model numbers.
They have not yet run on a physical Raspberry Pi and not against a robot.

## The two targets

| target | libc | linking | `model-library` |
|---|---|---|---|
| `aarch64-unknown-linux-gnu` | glibc | dynamic | compiles |
| `aarch64-unknown-linux-musl` | musl | fully static | off (`--no-default-features`) |

The default `model-library` feature compiles `Robot::load_model_from_robot`, which `dlopen`s
the model library an FER serves. That library is an x86-64 build (`libfcimodels_x64.so`) and
of no use on `aarch64` either way; `load_model()` evaluates both robots' models natively and
is the path to use. Turning the feature off drops `libloading`, which a static musl binary
cannot use, and changes nothing else.

## Option A: `cargo-zigbuild`, no root

```sh
rustup target add aarch64-unknown-linux-gnu
python3 -m venv ~/.venvs/zigbuild && source ~/.venvs/zigbuild/bin/activate
pip install ziglang
cargo install cargo-zigbuild

cargo zigbuild --release --target aarch64-unknown-linux-gnu -p franka-rs --examples
```

The bundled `zig cc` is linker and sysroot; the result is an ordinary dynamically linked
`aarch64` binary with the default features.

## Option B: a system cross compiler

```sh
sudo apt install gcc-aarch64-linux-gnu   # Debian/Ubuntu
rustup target add aarch64-unknown-linux-gnu
CARGO_TARGET_AARCH64_UNKNOWN_LINUX_GNU_LINKER=aarch64-linux-gnu-gcc \
  cargo build --release --target aarch64-unknown-linux-gnu -p franka-rs --examples
```

This is what CI's `aarch64-cross-build` job does; the linker can also go in `.cargo/config.toml`.

## Option C: a static musl binary, no cross linker

Rust's musl target ships a self-contained linker (`rust-lld`) with `rustup`:

```sh
rustup target add aarch64-unknown-linux-musl
cargo build --release --target aarch64-unknown-linux-musl -p franka-rs \
  --no-default-features --examples
```

The repository's `.cargo/config.toml` sets `linker = "rust-lld"` for this target only;
outside the checkout pass `RUSTFLAGS="-C linker=rust-lld"`. The result is one statically
linked binary per example with no runtime dependencies.

## Smoke test without hardware

`qemu-aarch64-static` (package `qemu-user-static`) runs the binaries on the x86-64 host. A
static musl binary needs nothing else:

```sh
qemu-aarch64-static target/aarch64-unknown-linux-musl/release/examples/communication_test
# Usage: .../communication_test <robot-hostname>
```

A dynamically linked glibc binary also needs an `aarch64` sysroot, for example the `/lib` and
`/usr` of an `arm64` base image:

```sh
cid=$(docker create --platform linux/arm64 arm64v8/ubuntu:24.04)
mkdir sysroot && docker cp $cid:/lib sysroot/ && docker cp $cid:/usr sysroot/
docker rm $cid
qemu-aarch64-static -L sysroot target/aarch64-unknown-linux-gnu/release/examples/communication_test
```

The crate's test suite has been run this way, one binary at a time with `--test-threads=1`
and `FRANKA_SIM_ADDR=127.0.0.1` pointing at a container started on the host (a binary under
QEMU cannot exec the host's `docker`, so the harness must [attach](./simulator-tests.md)).
Everything that passes natively passed on `aarch64` with byte-identical model numbers, except
the tests calling `load_model_from_robot`: compiled out under `--no-default-features`, and
failing with the feature on because the x86-64 library cannot be loaded.

## Deploy and run

```sh
scp target/aarch64-unknown-linux-gnu/release/examples/communication_test pi@<pi-host>:~/
ssh pi@<pi-host> chmod +x ~/communication_test

# on the target
FRANKA_REALTIME=ignore ./communication_test 172.16.0.2
```

The same [realtime prerequisites](../getting-started/realtime-machine.md) apply on the
target: a `PREEMPT_RT` kernel (`cat /sys/kernel/realtime` reads `1`) and the right to
`SCHED_FIFO`, or `FRANKA_REALTIME=ignore` at the cost described there. Raspberry Pi OS 64-bit
has a `PREEMPT_RT` kernel package (`sudo apt install linux-image-rpi-v8-rt`, selected with
`kernel=kernel8_rt.img` in `/boot/firmware/config.txt`) and Ubuntu 24.04 for Raspberry Pi has
one through Ubuntu Pro (`sudo pro enable realtime-kernel --variant=raspi`); neither has been
exercised with this crate yet.

Building natively on the target instead (`rustup`, then `cargo build --release -p franka-rs
--examples`) needs none of the above; it has not been tried on a Pi yet.
