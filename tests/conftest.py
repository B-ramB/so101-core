"""Shared pytest setup.

SO101_HOME must point at a temp dir before anything imports so101: so101.paths binds CALIBRATION_DIR,
CONFIG_PATH, etc. at import time, and the tests must never touch the real so101_config/.
"""

import os
import shutil
import tempfile
from pathlib import Path

import pytest

TEST_HOME = Path(tempfile.mkdtemp(prefix="so101-test-home-"))
os.environ["SO101_HOME"] = str(TEST_HOME)


@pytest.fixture(autouse=True)
def empty_so101_home():
    """Every test starts with an empty SO101_HOME, so files written by one test never leak into the next."""
    yield TEST_HOME
    for child in TEST_HOME.iterdir():
        if child.is_dir():
            shutil.rmtree(child)
        else:
            child.unlink()


def pytest_sessionfinish(session, exitstatus):
    shutil.rmtree(TEST_HOME, ignore_errors=True)
