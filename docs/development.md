# Development

## Setup

```bash
uv sync
```

This installs the package plus the `dev` group (pytest, ruff). The first install is slow because lerobot pulls in torch.

## Running the checks

```bash
uv run ruff check     # lint (line length 120)
uv run pytest         # tests
```

CI (`.github/workflows/ci.yml`) runs the same two commands on every push to `main` and every pull request.

## Test isolation

`tests/conftest.py` sets `SO101_HOME` to a fresh temp dir before any `so101` import. `so101.paths` reads that
variable at import time, so the tests never read or write your real `so101_config/`. Keep it that way: don't
import `so101` from anything that runs before `conftest.py`.

An autouse fixture (`empty_so101_home`) empties that folder after every test, so each test starts without an
`arm_config.json`, calibration files or USB configs. The folder is removed when the session ends.

Code that needs the arms, a camera, the network or a terminal is replaced with `monkeypatch`: `input` / `ask_yes_no`
for the prompts, `subprocess.run` for lerobot's calibration and `hf buckets sync`, and `detect_arm_ports`,
`detect_camera` and `ensure_port_access` for the USB setup. Ports that must exist are temp files.

## Markers

| Marker     | Meaning                                                  | Runs by default |
|------------|----------------------------------------------------------|-----------------|
| (none)     | Pure logic, no hardware or downloads needed              | yes             |
| `sim`      | Needs the `sim` extra and the URDFs (skipped otherwise)  | yes             |
| `hardware` | Needs the real arms connected                            | no              |

`hardware` tests are deselected via `addopts = "-m 'not hardware'"` in `pyproject.toml`. Useful selections:

```bash
uv run --extra sim pytest -m sim     # only sim tests
uv run pytest -m "not sim"           # skip sim tests
uv run pytest -m hardware            # run hardware tests (a later -m overrides the default)
```
