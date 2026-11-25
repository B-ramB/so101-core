"""so101.paths: finding the project root, creating the config folder, and printing paths."""

from pathlib import Path

import pytest

from so101 import paths
from so101.paths import GITIGNORE, ensure_home, find_project_root, pretty


@pytest.mark.parametrize("marker", ["pyproject.toml", ".git"])
def test_project_root_is_the_nearest_folder_with_a_marker(tmp_path, marker):
    (tmp_path / marker).touch()
    nested = tmp_path / "a" / "b"
    nested.mkdir(parents=True)
    assert find_project_root(nested) == tmp_path.resolve()


def test_project_root_prefers_the_nearest_marker(tmp_path):
    (tmp_path / ".git").mkdir()
    inner = tmp_path / "inner"
    (inner / "sub").mkdir(parents=True)
    (inner / "pyproject.toml").touch()
    assert find_project_root(inner / "sub") == inner.resolve()


def test_project_root_falls_back_to_start(tmp_path, monkeypatch):
    # tmp_path itself has no marker, but a parent might (e.g. a temp dir inside a repo); make every check miss.
    monkeypatch.setattr(Path, "exists", lambda self: False)
    assert find_project_root(tmp_path) == tmp_path.resolve()


def test_project_root_defaults_to_the_current_directory(tmp_path, monkeypatch):
    (tmp_path / "pyproject.toml").touch()
    monkeypatch.chdir(tmp_path)
    assert find_project_root() == tmp_path.resolve()


def test_ensure_home_creates_the_folder_with_a_gitignore(tmp_path, monkeypatch):
    home = tmp_path / "so101_config"
    monkeypatch.setattr(paths, "SO101_HOME", home)
    assert ensure_home() == home
    assert (home / ".gitignore").read_text() == GITIGNORE


def test_ensure_home_keeps_an_existing_gitignore(tmp_path, monkeypatch):
    home = tmp_path / "so101_config"
    home.mkdir()
    (home / ".gitignore").write_text("custom\n")
    monkeypatch.setattr(paths, "SO101_HOME", home)
    ensure_home()
    assert (home / ".gitignore").read_text() == "custom\n"


@pytest.fixture
def cwd_and_home(tmp_path, monkeypatch):
    """A current directory and a home directory next to each other in tmp_path."""
    cwd, home = tmp_path / "project", tmp_path / "home"
    cwd.mkdir()
    home.mkdir()
    monkeypatch.chdir(cwd)
    monkeypatch.setenv("HOME", str(home))
    return cwd, home


def test_pretty_inside_the_current_directory_is_relative(cwd_and_home):
    cwd, _ = cwd_and_home
    assert pretty(cwd / "so101_config" / "arm_config.json") == "so101_config/arm_config.json"


def test_pretty_inside_home_uses_a_tilde(cwd_and_home):
    _, home = cwd_and_home
    assert pretty(home / "other" / "file.json") == "~/other/file.json"


def test_pretty_elsewhere_is_absolute(cwd_and_home):
    assert pretty("/dev/ttyACM0") == "/dev/ttyACM0"


def test_pretty_accepts_strings(cwd_and_home):
    cwd, _ = cwd_and_home
    assert pretty(str(cwd / "x.json")) == "x.json"
