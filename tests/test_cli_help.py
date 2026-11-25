"""Every console script in pyproject.toml loads and answers --help without touching hardware."""

from importlib.metadata import distribution

import pytest

ENTRY_POINTS = sorted(
    (ep for ep in distribution("so101-core").entry_points if ep.group == "console_scripts"), key=lambda ep: ep.name
)


def test_entry_points_are_installed():
    assert {ep.name for ep in ENTRY_POINTS} >= {
        "so101-setup",
        "so101-teleop",
        "so101-view-urdf",
        "so101-fetch-urdfs",
        "so101-sim",
    }


@pytest.mark.parametrize("entry_point", ENTRY_POINTS, ids=lambda ep: ep.name)
def test_help_exits_0(entry_point, monkeypatch, capsys):
    main = entry_point.load()
    monkeypatch.setattr("sys.argv", [entry_point.name, "--help"])
    with pytest.raises(SystemExit) as exit_info:
        main()
    assert exit_info.value.code == 0
    out = capsys.readouterr().out
    assert out.startswith(f"usage: {entry_point.name}")
