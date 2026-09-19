"""The APK fetch must refuse anything but the pinned build, and refuse loudly, and the APK
lives in the user's data directory, never in the installed package.

No network here: `fetch()` takes its opener, which is the whole point of that argument, and
no headset: the installer's reader is a stand-in.
"""
import contextlib
import hashlib
import io
import os

import pytest

from franka_vr.quest_bridge.oculus_reader import fetch_apk, install, reader

GOOD = b"PK\x03\x04 pretend this is the verified build"
BAD = b"PK\x03\x04 a different build entirely"


@pytest.fixture
def pinned(monkeypatch):
    """Pin the module to GOOD, so the test needs no 4.8 MB of bytes."""
    monkeypatch.setattr(fetch_apk, "SHA256", hashlib.sha256(GOOD).hexdigest())
    monkeypatch.setattr(fetch_apk, "SIZE", len(GOOD))
    return fetch_apk


def opener_for(blob):
    @contextlib.contextmanager
    def _open(url, timeout=None):
        yield io.BytesIO(blob)
    return _open


def test_the_pin_is_the_verified_build():
    """The constants are the contract; a silent edit of them is the failure to catch."""
    assert fetch_apk.COMMIT == "1168953a0f4fe9ac761a0ef6139a511bb24c67a2"
    assert fetch_apk.SHA256 == (
        "97b49f94682a732e14d131d50bcc7885e183d73b977e19a1752c28502519cd4b")
    assert fetch_apk.SIZE == 4865715
    assert fetch_apk.COMMIT in fetch_apk.URL
    assert fetch_apk.URL.startswith("https://")


def test_fetch_writes_the_verified_build(pinned, tmp_path):
    dest = str(tmp_path / "APK" / "teleop-debug.apk")
    assert pinned.fetch(dest, opener=opener_for(GOOD)) == dest
    with open(dest, "rb") as fh:
        assert fh.read() == GOOD


def test_a_mismatch_refuses_and_writes_nothing(pinned, tmp_path):
    dest = str(tmp_path / "APK" / "teleop-debug.apk")
    with pytest.raises(pinned.ApkHashMismatch) as exc:
        pinned.fetch(dest, opener=opener_for(BAD))
    assert not os.path.exists(dest)
    assert not os.path.exists(dest + ".part")
    message = str(exc.value)
    assert pinned.SHA256 in message and hashlib.sha256(BAD).hexdigest() in message
    assert str(len(BAD)) in message


def test_a_wrong_file_on_disk_is_refused_not_reused(pinned, tmp_path):
    dest = tmp_path / "teleop-debug.apk"
    dest.write_bytes(BAD)
    with pytest.raises(pinned.ApkHashMismatch):
        pinned.fetch(str(dest), opener=opener_for(GOOD))
    assert dest.read_bytes() == BAD          # and nothing was downloaded over it


def test_a_verified_file_on_disk_is_not_refetched(pinned, tmp_path):
    dest = tmp_path / "teleop-debug.apk"
    dest.write_bytes(GOOD)

    def refuse(url):                         # a fetch here would be a bug
        raise AssertionError("re-downloaded an APK that was already verified")

    assert pinned.fetch(str(dest), opener=refuse) == str(dest)


def test_force_refetches_and_still_verifies(pinned, tmp_path):
    dest = tmp_path / "teleop-debug.apk"
    dest.write_bytes(GOOD)
    with pytest.raises(pinned.ApkHashMismatch):
        pinned.fetch(str(dest), opener=opener_for(BAD), force=True)
    assert dest.read_bytes() == GOOD


def test_the_apk_lives_in_the_users_data_dir(pinned, monkeypatch, tmp_path):
    monkeypatch.setenv("XDG_DATA_HOME", str(tmp_path))
    dest = str(tmp_path / "franka-vr" / "teleop-debug.apk")
    assert pinned.apk_path() == dest
    assert pinned.fetch(opener=opener_for(GOOD)) == dest
    assert open(dest, "rb").read() == GOOD
    package = os.path.dirname(os.path.abspath(pinned.__file__))
    for value in ("", "relative/dir"):       # the XDG default, never the package directory
        monkeypatch.setenv("XDG_DATA_HOME", value)
        assert pinned.apk_path() == os.path.join(
            os.path.expanduser("~"), ".local", "share", "franka-vr", "teleop-debug.apk")
        assert not pinned.apk_path().startswith(package)


class _Reader:
    """The installer's view of an OculusReader: which APK it was asked to install."""

    installed = []

    def __init__(self, run=True):
        pass

    def install(self, APK_path=None, verbose=True, reinstall=False):
        self.installed.append((APK_path, reinstall))


def test_the_installer_installs_the_verified_apk_from_the_data_dir(pinned, monkeypatch,
                                                                   tmp_path):
    monkeypatch.setenv("XDG_DATA_HOME", str(tmp_path))
    monkeypatch.setattr(install, "OculusReader", _Reader)
    monkeypatch.setattr(pinned.urllib.request, "urlopen", opener_for(BAD))
    dest = tmp_path / "franka-vr" / "teleop-debug.apk"
    dest.parent.mkdir()
    dest.write_bytes(GOOD)
    _Reader.installed = []
    install.main([])
    install.main(["--reinstall"])
    assert _Reader.installed == [(str(dest), False), (str(dest), True)]
    dest.write_bytes(BAD)                    # a tampered copy is refused, not installed
    with pytest.raises(pinned.ApkHashMismatch):
        install.main([])
    assert len(_Reader.installed) == 2 and dest.read_bytes() == BAD


class _Device:
    """A headset without the app, as ppadb shows it: what was installed, from where."""

    def __init__(self):
        self.paths = []

    def is_installed(self, name):
        return bool(self.paths)

    def install(self, path, test=True, reinstall=False):
        self.paths.append(path)
        return True


def test_the_readers_own_install_verifies_the_data_dir_apk(pinned, monkeypatch, tmp_path):
    """`OculusReader.__init__` installs a missing app itself, so a bridge start is an
    install path too: a tampered file there is refused and never reaches the headset."""
    monkeypatch.setenv("XDG_DATA_HOME", str(tmp_path))
    monkeypatch.setattr(pinned.urllib.request, "urlopen", opener_for(BAD))
    dest = tmp_path / "franka-vr" / "teleop-debug.apk"
    dest.parent.mkdir()
    dest.write_bytes(BAD)
    oculus = reader.OculusReader.__new__(reader.OculusReader)   # no adb, no headset
    oculus.APK_name, oculus.device = "com.rail.oculus.teleop", _Device()
    with pytest.raises(pinned.ApkHashMismatch):
        oculus.install(verbose=False)
    assert oculus.device.paths == []
    dest.write_bytes(GOOD)
    oculus.install(verbose=False)
    assert oculus.device.paths == [str(dest)]


def test_main_fails_loudly(pinned, monkeypatch, tmp_path, capsys):
    monkeypatch.setenv("XDG_DATA_HOME", str(tmp_path))
    monkeypatch.setattr(pinned.urllib.request, "urlopen", opener_for(BAD))
    assert pinned.main([]) == 2
    err = capsys.readouterr().err
    assert err.startswith("FATAL:") and "refusing the APK" in err
