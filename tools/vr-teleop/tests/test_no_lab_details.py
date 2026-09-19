"""The merge gate: no host, user or private path of ours may enter this tree.

"Public docs name no hosts, users or paths" is a project rule, and this is what enforces
it. The patterns here are deliberately GENERIC -- any absolute home directory, any
private-range address, a private notes or agent directory -- so a detail nobody has thought
of yet is caught the first time it is committed. Each is written with a one-character
character class (`/home[/]`) so that this file does not match itself.

Names that are only private because they are ours (accounts, hosts, directories) cannot be
listed in a public file without publishing them. They are read at test time from outside the
tree: `FRANKA_PRIVATE_NEEDLES` (newline- or comma-separated) and the repository's never
committed `public-needles.txt` in its private notes directory, one per line. Without either,
that one check is skipped and says so; the generic ones always run.

`PENDING` is the parcelling escape hatch, not a suppression list. Each entry names a file
another parcel of the same cleanup still owns, and `test_pending_is_not_stale` fails as soon
as one of them is clean -- so an entry cannot outlive the violation it excuses. It is empty,
as it must be for a published tree.
"""
import os
import re

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO = os.path.dirname(os.path.dirname(ROOT))
NEEDLES_ENV = "FRANKA_PRIVATE_NEEDLES"
NEEDLES_FILE = os.path.join(REPO, "." + "private", "public-needles.txt")

SKIP_DIRS = {"__pycache__", "venv", "node_modules", "APK"}   # plus every dotted directory
SKIP_NAMES = {"LICENSE"}          # upstream's Apache-2.0 text, carried verbatim
MAX_BYTES = 2_000_000

# The one address that may appear: Franka's own documented default for a control box.
# Matched with a boundary, so that address plus another digit is still a leak.
ALLOWED = (re.compile(r"\b172\.16\.0\.2\b"),)

# 10.x is a documented private range that synthetic test data uses freely; a real one in
# shipping code is still a leak, so it is only tolerated in the test files themselves.
TEST_FILES = re.compile(r"(^tests/|/tests/|selftest.*\.py$|run_tests\.sh$|test_.*\.py$)")

PATTERNS = [
    ("absolute home path", re.compile(r"/home[/]|/Users[/]")),
    ("private LAN address", re.compile(r"\b192\.168\.\d")),
    ("robot network address", re.compile(r"\b172\.16\.\d")),
    ("private notes tree", re.compile(r"[.]private[/\w]")),
    ("agent working directory", re.compile(r"[.]claude[/]")),
]
TEST_ONLY = [("private-range address", re.compile(r"\b10\.\d+\.\d+\.\d+\b"))]

# Owned by another parcel of this cleanup; each is deleted as that parcel lands, which
# `test_pending_is_not_stale` makes unmissable. The list must be empty before publication.
PENDING = {}


def private_needles(env=None, path=NEEDLES_FILE):
    """The private names to hunt for, from the environment and the private needle file."""
    env = os.environ if env is None else env
    items = re.split(r"[\n,]", env.get(NEEDLES_ENV, ""))
    if os.path.isfile(path):
        with open(path, encoding="utf-8") as fh:
            items += fh.read().splitlines()
    return sorted({i.strip() for i in items if i.strip() and not i.strip().startswith("#")})


def _candidates():
    for dirpath, dirnames, filenames in os.walk(ROOT):
        dirnames[:] = sorted(d for d in dirnames
                             if d not in SKIP_DIRS and not d.startswith("."))
        for name in sorted(filenames):
            if name in SKIP_NAMES or name.endswith((".pyc", ".gz", ".apk", ".png")):
                continue
            path = os.path.join(dirpath, name)
            if os.path.getsize(path) > MAX_BYTES:
                continue
            yield os.path.relpath(path, ROOT).replace(os.sep, "/"), path


def scan_text(rel, text, needles=()):
    """Every (line number, label, line) this file would be rejected for; `needles` are
    private names, matched case-insensitively."""
    hits = []
    patterns = PATTERNS if TEST_FILES.search(rel) else PATTERNS + TEST_ONLY
    lowered = [n.lower() for n in needles]
    for lineno, line in enumerate(text.splitlines(), 1):
        stripped = line
        for allowed in ALLOWED:
            stripped = allowed.sub("", stripped)
        for label, pattern in patterns:
            if pattern.search(stripped):
                hits.append((lineno, label, line.strip()[:110]))
        if any(n in line.lower() for n in lowered):
            hits.append((lineno, "private name", "(not echoed)"))
    return hits


def scan_tree(needles=()):
    found = {}
    for rel, path in _candidates():
        try:
            with open(path, "r", encoding="utf-8") as fh:
                text = fh.read()
        except (UnicodeDecodeError, OSError):
            continue
        hits = scan_text(rel, text, needles)
        if hits:
            found[rel] = hits
    return found


def _report(found):
    return "\n".join(f"  {rel}:{lineno}: {label}: {line}"
                     for rel, hits in sorted(found.items()) for lineno, label, line in hits)


def test_no_lab_details():
    """No file outside PENDING names a host, a user or a private path."""
    found = {rel: hits for rel, hits in scan_tree().items() if rel not in PENDING}
    if found:
        pytest.fail(
            "lab details must not be published. Remove these, or -- only if another "
            f"parcel owns the file -- add it to PENDING with the owner:\n{_report(found)}")


def test_no_private_name():
    """No file names one of the private needles; skipped where none are available."""
    needles = private_needles()
    if not needles:
        pytest.skip(f"no private needles: set {NEEDLES_ENV} or create {NEEDLES_FILE}")
    found = scan_tree(needles)
    assert not found, "private names must not be published:\n" + _report(found)


def test_pending_is_not_stale():
    """A PENDING entry that is clean, or gone, must be deleted from the list."""
    found = scan_tree()
    clean = [f"  {rel}: {why}" for rel, why in sorted(PENDING.items())
             if rel not in found]
    assert not clean, ("these files are clean now: delete their PENDING entries so the "
                       "gate guards them again.\n" + "\n".join(clean))


def _s(text):
    """A sample with one `|` dropped, so this file does not contain the needle itself."""
    return text.replace("|", "")


# Strings this tree has NEVER held. They are the point of the gate: it is the pattern that
# is guarded, not a list of the leaks we already know about.
@pytest.mark.parametrize("sample", [
    "QUEST=${QUEST_IP:-192|.168.1.50}",           # a headset IP nobody has used yet
    "robot_ip = '172|.16.4.9'",                   # an arm that is not the documented default
    "PY=/ho|me/someone-else/.venvs/rt/bin/python",
    "cache = '/Us|ers/someone/Library/vr'",
    "# see .pri|vate/docs/vr/NEW-MEMO.md",
    "fixture at ~/.cla|ude/jobs/1234/tmp/x.jsonl",
])
def test_gate_catches_a_new_detail(sample):
    assert scan_text("franka_vr/quest_bridge/newfile.py", _s(sample)), sample


@pytest.mark.parametrize("sample", [
    "connect('172.16.0.2')",                    # Franka's documented default
    "endpoint tcp://127.0.0.1:5560",
    "ZENOH=tcp/127.0.0.1:7447",
    "os.path.expanduser('~/.venv')",
    "home = os.environ['HOME']",
])
def test_gate_passes_what_it_must(sample):
    assert not scan_text("franka_vr/quest_bridge/newfile.py", sample), sample


def test_ten_dot_is_a_leak_outside_tests():
    line = "--bridge-host " + ".".join(("10", "0", "0", "9"))
    assert scan_text("franka_vr/teleop/sinks.py", line)
    assert not scan_text("tests/teleop/selftest.py", line)


def test_private_needles_come_from_the_env_and_the_file(tmp_path):
    """Made-up names here: the real ones never enter the tree."""
    needle_file = tmp_path / "public-needles.txt"
    needle_file.write_text("# a comment\nexample-host\n\n")
    env = {NEEDLES_ENV: "Someone-Else, other-dir\nthird"}
    needles = private_needles(env, str(needle_file))
    assert needles == ["Someone-Else", "example-host", "other-dir", "third"]
    assert private_needles({}, str(tmp_path / "missing")) == []
    assert scan_text("README.md", "ssh someone-else@example-host", needles)
    assert not scan_text("README.md", "ssh operator@robot", needles)
