"""Shared pytest setup.

SO101_HOME must point at a temp dir before anything imports so101: so101.paths binds CALIBRATION_DIR,
CONFIG_PATH, etc. at import time, and the tests must never touch the real so101_config/.
"""

import os
import tempfile

os.environ["SO101_HOME"] = tempfile.mkdtemp(prefix="so101-test-home-")
