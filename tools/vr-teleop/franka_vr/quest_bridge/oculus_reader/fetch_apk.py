"""Fetch the Quest teleop APK from upstream and refuse anything but the pinned build.

The APK is a build of upstream's `app_source/` and it embeds Meta's Oculus Mobile SDK
(`libvrapi.so`, `libvrinputstandard.so`), which is not Apache-2.0 and is not ours to
redistribute -- so it is not in this repository. This module downloads it from the
upstream commit the bridge was verified against and checks the sha256 before writing
anything. Upstream replaced the APK later with a different (7.5 MB) build and a fork
carries a third; neither has been tested here, which is why the hash is pinned and a
mismatch is a hard failure, never a warning.

The APK is kept per user, in `$XDG_DATA_HOME/franka-vr/` (default `~/.local/share/franka-vr/`),
never in the installed package, and is verified again whenever it is used from there.

    franka-vr-fetch-apk [--force]
"""
import argparse
import hashlib
import os
import sys
import urllib.request

COMMIT = "1168953a0f4fe9ac761a0ef6139a511bb24c67a2"
URL = ("https://media.githubusercontent.com/media/rail-berkeley/oculus_reader/"
       f"{COMMIT}/oculus_reader/APK/teleop-debug.apk")
SHA256 = "97b49f94682a732e14d131d50bcc7885e183d73b977e19a1752c28502519cd4b"
SIZE = 4865715
APK_NAME = "teleop-debug.apk"


def apk_path() -> str:
    """Where the APK is kept: the user's data directory, as XDG defines it."""
    base = os.environ.get("XDG_DATA_HOME", "")
    if not os.path.isabs(base):                   # unset, empty or relative: the spec's default
        base = os.path.join(os.path.expanduser("~"), ".local", "share")
    return os.path.join(base, "franka-vr", APK_NAME)


class ApkHashMismatch(ValueError):
    """The bytes fetched are not the build this bridge was verified against. Not a
    RuntimeError: the reader's install() turns those into an adb-permissions message."""


def verify(blob: bytes, *, source: str = URL) -> bytes:
    """Return `blob` if it is the pinned APK; raise `ApkHashMismatch` otherwise."""
    got = hashlib.sha256(blob).hexdigest()
    if got == SHA256 and len(blob) == SIZE:
        return blob
    raise ApkHashMismatch(
        f"refusing the APK from {source}: expected sha256 {SHA256} ({SIZE} bytes), "
        f"got {got} ({len(blob)} bytes). This is NOT the build this bridge was "
        f"verified against; do not install it. Upstream's current APK and the "
        f"community fork's are different builds -- fetch the pinned commit {COMMIT}.")


def fetch(dest: str = None, *, url: str = URL, force: bool = False,
          opener=None) -> str:
    """Download and verify the APK into `dest` (default `apk_path()`), or verify the copy
    already there. Nothing is written unless it verifies."""
    dest = apk_path() if dest is None else dest
    if os.path.exists(dest) and not force:
        with open(dest, "rb") as fh:
            verify(fh.read(), source=dest)
        return dest
    with (opener or urllib.request.urlopen)(url, timeout=60) as response:
        blob = verify(response.read(), source=url)
    os.makedirs(os.path.dirname(dest) or ".", exist_ok=True)
    tmp = dest + ".part"
    with open(tmp, "wb") as fh:
        fh.write(blob)
    os.replace(tmp, dest)
    return dest


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        prog="franka-vr-fetch-apk",
        description=f"Download the Quest teleop APK and refuse anything but sha256 {SHA256[:8]}...")
    parser.add_argument("--force", action="store_true", help="download again even if present")
    force = parser.parse_args(argv).force
    try:
        print(f"[apk] {fetch(force=force)}  sha256 {SHA256[:8]}... verified")
    except ApkHashMismatch as exc:
        print(f"FATAL: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
