"""so101.urdf: the `hf buckets sync` command and which bucket it syncs from."""

import subprocess

import pytest

from so101 import urdf
from so101.urdf import DEFAULT_BUCKET, fetch_urdfs, urdfs_present


@pytest.fixture
def hf(monkeypatch):
    """Replace the hf CLI and subprocess.run; returns the commands that would have run."""
    calls = []
    monkeypatch.setattr(urdf, "hf_executable", lambda: "/usr/bin/hf")
    monkeypatch.setattr(urdf.subprocess, "run", lambda cmd, check: calls.append((cmd, check)))
    monkeypatch.delenv("URDF_BUCKET", raising=False)
    return calls


def test_fetch_builds_the_sync_command(hf, tmp_path):
    dest = tmp_path / "urdf"
    fetch_urdfs(dest=dest)
    assert hf == [(["/usr/bin/hf", "buckets", "sync", DEFAULT_BUCKET, str(dest)], True)]
    assert dest.is_dir()


def test_bucket_from_the_environment(hf, tmp_path, monkeypatch):
    monkeypatch.setenv("URDF_BUCKET", "hf://buckets/env/bucket")
    fetch_urdfs(dest=tmp_path)
    assert hf[0][0][3] == "hf://buckets/env/bucket"


def test_bucket_argument_beats_the_environment(hf, tmp_path, monkeypatch):
    monkeypatch.setenv("URDF_BUCKET", "hf://buckets/env/bucket")
    fetch_urdfs("hf://buckets/arg/bucket", dest=tmp_path)
    assert hf[0][0][3] == "hf://buckets/arg/bucket"


def test_empty_environment_variable_uses_the_default(hf, tmp_path, monkeypatch):
    monkeypatch.setenv("URDF_BUCKET", "")
    fetch_urdfs(dest=tmp_path)
    assert hf[0][0][3] == DEFAULT_BUCKET


def test_hf_next_to_python_is_preferred(tmp_path, monkeypatch):
    (tmp_path / "hf").touch()
    monkeypatch.setattr(urdf.sys, "executable", str(tmp_path / "python"))
    assert urdf.hf_executable() == str(tmp_path / "hf")


def test_hf_from_path(tmp_path, monkeypatch):
    monkeypatch.setattr(urdf.sys, "executable", str(tmp_path / "python"))
    monkeypatch.setattr(urdf.shutil, "which", lambda name: f"/opt/bin/{name}")
    assert urdf.hf_executable() == "/opt/bin/hf"


def test_hf_missing(tmp_path, monkeypatch):
    monkeypatch.setattr(urdf.sys, "executable", str(tmp_path / "python"))
    monkeypatch.setattr(urdf.shutil, "which", lambda name: None)
    with pytest.raises(FileNotFoundError, match="hf"):
        urdf.hf_executable()


def test_urdfs_present_needs_both(tmp_path, monkeypatch):
    follower, leader = tmp_path / "follower.urdf", tmp_path / "leader.urdf"
    monkeypatch.setattr(urdf, "FOLLOWER_URDF", follower)
    monkeypatch.setattr(urdf, "LEADER_URDF", leader)
    follower.touch()
    assert not urdfs_present()
    leader.touch()
    assert urdfs_present()


def test_cli_turns_a_failed_sync_into_an_exit_message(monkeypatch):
    from so101.cli import fetch_urdfs as cli

    def fail(bucket):
        raise subprocess.CalledProcessError(1, ["hf"])

    monkeypatch.setattr(cli, "fetch_urdfs", fail)
    monkeypatch.setattr(cli.sys, "argv", ["so101-fetch-urdfs", "--bucket", "hf://buckets/x/y"])
    with pytest.raises(SystemExit, match="Fetching the URDFs failed"):
        cli.main()
