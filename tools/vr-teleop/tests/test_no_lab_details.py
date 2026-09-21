"""The merge gate: no host, user or private path of ours may enter this tree.

"Public docs name no hosts, users or paths" is a project rule, and this is what enforces
it. The gate reads every file git tracks in the repository -- crates, docs, tools and the
root -- not only this package, because a detail is published wherever it sits, and a gate
that covers one directory says nothing about the rest of the tree. Where there is no
repository to ask -- an export, a vendored copy -- it falls back to
this package, which is then all there is to guard. The published sdist is not such a case: it
carries `franka_vr` alone by design, and the gate runs here, on the tree, before anything is
built from it.

The patterns are deliberately GENERIC -- any absolute home directory, any private-range
address, a private notes or agent directory -- so a detail nobody has thought of yet is
caught the first time it is committed. Each is written with a one-character character class
(`/home[/]`) so that this file does not match itself. They hold for test files exactly as
for shipping code: a fixture is published like anything else, and synthetic data has RFC
5737 (192.0.2.0/24, 198.51.100.0/24, 203.0.113.0/24) to draw on.

Text is not the whole surface. A PNG carries its authoring tool's render stamp, and an
export path in one of those chunks is a home directory published past every gate that only
reads source, so the text chunks are scanned too.

Names that are only private because they are ours (accounts, hosts, directories) cannot be
listed in a public file without publishing them. They are read at test time from outside the
tree: `FRANKA_PRIVATE_NEEDLES` (newline- or comma-separated) and the repository's never
committed `public-needles.txt` in its private notes directory, one per line. Without either,
that one check is skipped and says so; the generic ones always run.

`PENDING` is the parcelling escape hatch, not a suppression list. Each entry names a file
another parcel of the same cleanup still owns, and `test_pending_is_not_stale` fails as soon
as one of them is clean -- so an entry cannot outlive the violation it excuses. It must be
empty before the tree is published.
"""
import os
import re
import subprocess
import zlib

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO = os.path.dirname(os.path.dirname(ROOT))
NEEDLES_ENV = "FRANKA_PRIVATE_NEEDLES"
NEEDLES_FILE = os.path.join(REPO, "." + "private", "public-needles.txt")

SKIP_DIRS = {"__pycache__", "venv", "node_modules", "APK"}
SKIP_NAMES = {"LICENSE"}          # upstream's Apache-2.0 text, carried verbatim
SKIP_EXTS = (".pyc", ".gz", ".apk")
MAX_BYTES = 2_000_000

# Cut out of a line before it is matched. An FCI arm answers on the control-box network, so a
# two-arm page cannot demonstrate a second one from a documentation range: the four addresses
# below are the whole vocabulary the book uses. 172.16.0.2 and 172.16.0.1 are Franka's
# documented arm and host interface; 172.16.2.2 and 172.16.2.1 are the invented second pair.
# Deliberately narrow, and the narrowness is the point: every other address on the robot
# network is still a leak, because a reader cannot tell a documented example from the one a
# machine answers on. Plus the one ignore entry that has to name the private notes directory
# in order to ignore it. Matched with a boundary, so one of these plus a digit is still a leak.
ALLOWED = (re.compile(r"\b172\.16\.[02]\.[12]\b"),
           re.compile(r"^/?[.]private/$"))

# A recording id is `<arm>-20YYMMDDTHHMMSSZ`. A test writes a placeholder date -- the first of
# a month, or 29 February for the leap-day case -- at a whole hour; a session carries the day
# it ran and a reading off the clock. Only the second shape records when a robot was moved.
SESSION_ID = re.compile(r"\b20\d\d(\d\d)(\d\d)T\d\d(\d\d)(\d\d)Z\b")

PATTERNS = [
    ("absolute home path", re.compile(r"/home[/]|/Users[/]")),
    ("private LAN address", re.compile(r"\b192\.168\.\d")),
    ("private-range address", re.compile(r"\b10\.\d+\.\d+\.\d+\b")),
    ("robot network address", re.compile(r"\b172\.16\.\d")),
    ("private notes tree", re.compile(r"[.]private[/\w]")),
    # The last alternative is a home directory flattened into a scratch path, which is what an
    # agent runtime writes: an absolute path with the user in it, but under /tmp, so no
    # home-directory pattern sees it.
    ("agent working directory", re.compile(r"[.]claude[/]|/tmp/cl[a]ude|/scratchpad[/]")),
]

# Owned by another parcel of this cleanup; each is deleted as that parcel lands, which
# `test_pending_is_not_stale` makes unmissable. The list must be empty before publication.
# It is empty: the second arm's example subnet was ruled documentation and named in ALLOWED
# above rather than excused per file, so nothing is outstanding.
PENDING = {}


def private_needles(env=None, path=NEEDLES_FILE):
    """The private names to hunt for, from the environment and the private needle file."""
    env = os.environ if env is None else env
    items = re.split(r"[\n,]", env.get(NEEDLES_ENV, ""))
    if os.path.isfile(path):
        with open(path, encoding="utf-8") as fh:
            items += fh.read().splitlines()
    return sorted({i.strip() for i in items if i.strip() and not i.strip().startswith("#")})


def _tracked():
    """Every path git tracks in the repository, or None when there is no repository to ask:
    this package installed on its own, or a source tree git does not know about. Tracked is
    the set that gets published -- it leaves out the private notes and every build output
    without a list of them."""
    if os.path.join(REPO, "tools", "vr-teleop") != ROOT:
        return None
    if not os.path.exists(os.path.join(REPO, ".git")):  # a worktree's .git is a file
        return None
    try:
        done = subprocess.run(["git", "-C", REPO, "ls-files", "-z"], stdout=subprocess.PIPE,
                              stderr=subprocess.DEVNULL, timeout=120)
    except (OSError, subprocess.SubprocessError):
        return None
    if done.returncode != 0:
        return None
    return [p for p in done.stdout.decode("utf-8", "replace").split("\0") if p]


TRACKED = _tracked()
BASE = REPO if TRACKED is not None else ROOT


def _skipped(rel, tracked=True):
    """`tracked` is git's list, where a dotted directory is published like any other:
    `.github/workflows` is where a runner label or an account name would sit. The walk
    fallback has no such list, so there it still drops dotted directories -- `.git` above all
    -- along with everything git would have excluded."""
    parts = rel.split("/")
    if any(p in SKIP_DIRS for p in parts[:-1]):
        return True
    if not tracked and any(p.startswith(".") for p in parts[:-1]):
        return True
    return parts[-1] in SKIP_NAMES or parts[-1].endswith(SKIP_EXTS)


def _walk(root):
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames[:] = sorted(d for d in dirnames
                             if d not in SKIP_DIRS and not d.startswith("."))
        for name in sorted(filenames):
            path = os.path.join(dirpath, name)
            yield os.path.relpath(path, root).replace(os.sep, "/")


def _candidates():
    """(the path the report names, the path on disk) for every file the gate reads."""
    for rel in sorted(TRACKED) if TRACKED is not None else _walk(ROOT):
        if _skipped(rel, TRACKED is not None):
            continue
        path = os.path.join(BASE, rel)
        if not os.path.isfile(path):
            continue
        # A PNG is read chunk header by chunk header, so its size does not matter.
        if os.path.getsize(path) > MAX_BYTES and not rel.endswith(".png"):
            continue
        yield rel, path


def _session_id(line):
    """True when the line carries a timestamp id that reads as a record of a real run."""
    for month, day, minute, second in SESSION_ID.findall(line):
        if day == "01" or (month, day) == ("02", "29"):
            continue
        if (minute, second) == ("00", "00"):
            continue
        return True
    return False


def png_text(path):
    """Every tEXt/zTXt/iTXt chunk of a PNG, decompressed, as `keyword: value`. Render stamps,
    export paths and authoring tools live here; no chunk but the header of each is read."""
    out = []
    with open(path, "rb") as fh:
        if fh.read(8) != b"\x89PNG\r\n\x1a\n":
            return out
        while True:
            head = fh.read(8)
            if len(head) < 8:
                break
            size, kind = int.from_bytes(head[:4], "big"), head[4:]
            if kind == b"IEND":
                break
            if kind in (b"tEXt", b"zTXt", b"iTXt") and size <= MAX_BYTES:
                out.append(_png_chunk(kind, fh.read(size)))
                fh.read(4)                                # the chunk's CRC
            else:                                         # the image data, and anything absurd
                fh.seek(size + 4, os.SEEK_CUR)
    return out


def _png_chunk(kind, body):
    key, _, rest = body.partition(b"\0")
    if kind == b"zTXt":
        rest = _inflate(rest[1:])
    elif kind == b"iTXt":
        compressed, rest = rest[:1] != b"\0", rest[2:]
        for _ in range(2):                                # language tag, translated keyword
            _, _, rest = rest.partition(b"\0")
        rest = _inflate(rest) if compressed else rest
    return (key + b": " + rest).decode("utf-8", "replace")


def _inflate(data):
    try:
        return zlib.decompress(data)
    except zlib.error:
        return b""


def scan_text(rel, text, needles=()):
    """Every (line number, label, line) `text` would be rejected for; `needles` are private
    names, matched case-insensitively. `rel` only names the file in what a caller prints:
    every path is held to the same rules, tests included."""
    hits = []
    lowered = [n.lower() for n in needles]
    for lineno, line in enumerate(text.splitlines(), 1):
        stripped = line
        for allowed in ALLOWED:
            stripped = allowed.sub("", stripped)
        for label, pattern in PATTERNS:
            if pattern.search(stripped):
                hits.append((lineno, label, line.strip()[:110]))
        if _session_id(stripped):
            hits.append((lineno, "session timestamp", line.strip()[:110]))
        if any(n in line.lower() for n in lowered):
            hits.append((lineno, "private name", "(not echoed)"))
    return hits


def scan_file(rel, path, needles=()):
    """A PNG is its text chunks -- the line numbers are then the chunks, in file order;
    anything else is its text, and what does not decode as text is not read."""
    try:
        if rel.endswith(".png"):
            return scan_text(rel, "\n".join(png_text(path)), needles)
        with open(path, "r", encoding="utf-8") as fh:
            return scan_text(rel, fh.read(), needles)
    except (UnicodeDecodeError, OSError):
        return []


def scan_tree(needles=()):
    found = {}
    for rel, path in _candidates():
        hits = scan_file(rel, path, needles)
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
    """No file names one of the private needles. PENDING does not excuse one -- it parcels
    the generic patterns, and a name that must never be published is nobody's to defer."""
    needles = private_needles()
    if not needles:
        pytest.skip(f"no private needles: set {NEEDLES_ENV} or create {NEEDLES_FILE}")
    found = {rel: named for rel, hits in scan_tree(needles).items()
             if (named := [h for h in hits if h[1] == "private name"])}
    assert not found, "private names must not be published:\n" + _report(found)


def test_pending_is_not_stale():
    """A PENDING entry that is clean, or gone, must be deleted from the list."""
    found = scan_tree()
    clean = [f"  {rel}: {why}" for rel, why in sorted(PENDING.items())
             if rel not in found]
    assert not clean, ("these files are clean now: delete their PENDING entries so the "
                       "gate guards them again.\n" + "\n".join(clean))


def test_the_gate_reads_the_whole_repository():
    """The scope, which is the part that failed: every top level of the repository is read,
    not just the package this file sits in."""
    if TRACKED is None:
        pytest.skip("not inside the repository: the gate covers this package alone")
    seen = {rel for rel, _ in _candidates()}
    assert os.path.relpath(os.path.abspath(__file__), REPO).replace(os.sep, "/") in seen
    for top in ("crates/", "docs/", "tools/"):
        assert any(rel.startswith(top) for rel in seen), top
    assert not any(rel.startswith(("." + "private/", "target/")) for rel in seen)


def test_the_gate_reads_dotted_directories():
    """A dotted directory is published like any other, and `.github/workflows` is where a
    runner label, a lab host or an account name would sit. They were invisible: the skip
    tested every parent component for a leading dot, on git's list as well as the walk."""
    assert not _skipped(".github/workflows/ci.yml")
    assert not _skipped(".cargo/config.toml")
    assert _skipped(".github/workflows/ci.yml", tracked=False)
    assert _skipped(".git/config", tracked=False)
    if TRACKED is not None and ".github/workflows/ci.yml" in TRACKED:
        assert ".github/workflows/ci.yml" in {rel for rel, _ in _candidates()}


def test_a_session_timestamp_is_a_leak():
    """A recording id dated the day it ran, at a time read off the clock, records a session.
    A placeholder date -- the first of a month, or the leap day -- at a whole hour does not,
    and the crates' own timestamp tests are built out of those."""
    assert scan_text("crates/x/src/record.rs", 'let id = "L-2026' + '0917T084512Z";')
    assert not scan_text("crates/x/src/record.rs", 'let id = "L-20260101T120000Z";')
    assert not scan_text("crates/x/src/record.rs", 'let id = "L-20260917T120000Z";')
    assert not scan_text("crates/x/src/record.rs", 'let id = "a-20240229T235959Z";')


def _s(text):
    """A sample with one `|` dropped, so this file does not contain the needle itself."""
    return text.replace("|", "")


# Strings this tree has NEVER held. They are the point of the gate: it is the pattern that
# is guarded, not a list of the leaks we already know about.
CAUGHT = [
    "QUEST=${QUEST_IP:-192|.168.1.50}",           # a headset IP nobody has used yet
    "robot_ip = '172|.16.4.9'",                   # an arm that is not the documented default
    "robot_ip = '172|.16.0.7'",                   # nor a documented address on its network
    "host = '172|.16.3.5'  # the other one",      # nor a third subnet on the robot network
    "PY=/ho|me/someone-else/.venvs/rt/bin/python",
    "cache = '/Us|ers/someone/Library/vr'",
    "# see .pri|vate/docs/vr/NEW-MEMO.md",
    "fixture at ~/.cla|ude/jobs/1234/tmp/x.jsonl",
    "out = /t|mp/claude-1001/-home-someone-repo/7b99/scratch|pad/scene.blend",
    "episode = 'L-2026|0917T084512Z'",            # a run, dated and clocked
]


@pytest.mark.parametrize("sample", CAUGHT)
def test_gate_catches_a_new_detail(sample):
    assert scan_text("franka_vr/quest_bridge/newfile.py", _s(sample)), sample


PASSED = [
    "connect('172.16.0.2')",                    # Franka's documented default
    "addresses: [172.16.0.1/24]",               # and the host interface on its network
    "host = '172.16.2.2'  # the second arm",    # and the book's invented second pair
    "addresses: [172.16.2.1/24]",
    "let id = 'L-20260101T120000Z'",            # a placeholder date at a whole hour
    "assert_eq!(name, 'a-20240229T235959Z')",   # and the leap-day case a formatter must test
    "endpoint tcp://127.0.0.1:5560",
    "ZENOH=tcp/127.0.0.1:7447",
    "os.path.expanduser('~/.venv')",
    "home = os.environ['HOME']",
]


@pytest.mark.parametrize("sample", PASSED)
def test_gate_passes_what_it_must(sample):
    assert not scan_text("franka_vr/quest_bridge/newfile.py", sample), sample


def test_no_sample_is_a_real_one():
    """The `|` that keeps a sample from matching this file's own scan would hide a real
    value just as well, and a near-miss list is exactly where one gets reached for. Every
    sample is checked against the needles with the break taken back out, so a fixture can
    never quietly become the thing it is meant to stand in for."""
    needles = private_needles()
    if not needles:
        pytest.skip("no private needle file to check against")
    for sample in CAUGHT + PASSED:
        bare = _s(sample).lower()
        for needle in needles:
            assert needle.lower() not in bare, f"{sample!r} carries a private name"


def test_a_private_range_address_is_a_leak_in_tests_too():
    """A test file is published like any other, so exempting one from the address rule on
    the grounds that synthetic data uses 10.x freely would leave the largest private range
    unguarded in the files most likely to carry a literal. The rule holds everywhere, and
    the documentation ranges are what synthetic data draws on."""
    assert scan_text("tests/teleop/selftest.py", "--bridge-host " + ".".join(("10", "0", "0", "9")))
    assert not scan_text("tests/teleop/selftest.py",
                         "--bridge-host " + ".".join(("198", "51", "100", "9")))


def _png(tmp_path, chunks):
    """A minimal PNG: the signature, a header, the given text chunks and an end. Nothing
    decodes it; the gate reads chunk headers only."""
    def chunk(kind, body):
        return len(body).to_bytes(4, "big") + kind + body + b"\0\0\0\0"

    data = b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", b"\0" * 13)
    data += b"".join(chunk(kind, body) for kind, body in chunks) + chunk(b"IEND", b"")
    path = tmp_path / "render.png"
    path.write_bytes(data)
    return str(path)


def test_the_gate_reads_png_text_chunks(tmp_path):
    """A render stamp carries the authoring tool's own output path. That is how a home
    directory reaches a published asset past a gate that only reads source."""
    stamp = b"File: /ho" + b"me/someone/work/out/_scene.blend"
    plain = _png(tmp_path, [(b"tEXt", stamp)])
    packed = _png(tmp_path, [(b"zTXt", b"File\0\0" + zlib.compress(stamp))])
    labelled = _png(tmp_path, [(b"iTXt", b"File\0\0\0en\0File\0" + stamp)])
    for path in (plain, packed, labelled):
        assert scan_file("docs/assets/render.png", path), path
    # The shape the renders actually carried: an agent's scratch directory, not `/home`.
    scratch = b"File: /t" + b"mp/claude-1001/-home-someone/7b99/scratch" + b"pad/x.blend"
    assert scan_file("docs/assets/render.png", _png(tmp_path, [(b"tEXt", scratch)]))
    assert not scan_file("docs/assets/render.png",
                         _png(tmp_path, [(b"tEXt", b"Software: an offline renderer")]))


def test_png_text_chunks_are_searched_for_private_names(tmp_path):
    """Made-up names here: the real ones never enter the tree."""
    path = _png(tmp_path, [(b"tEXt", b"File: /var/example-host/scene.blend")])
    assert scan_file("docs/assets/render.png", path, ["example-host"])
    assert not scan_file("docs/assets/render.png", path, ["other-host"])


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
