"""SO-101 URDFs and meshes in so101_config/urdf, downloaded from the lerobot Hugging Face bucket."""

import os
import shutil
import subprocess
import sys
from pathlib import Path

from so101.paths import FOLLOWER_URDF, LEADER_URDF, URDF_DIR, ensure_home, pretty

DEFAULT_BUCKET = "hf://buckets/lerobot/robot-urdfs/so101"


def urdfs_present() -> bool:
    return FOLLOWER_URDF.exists() and LEADER_URDF.exists()


def hf_executable() -> str:
    """The `hf` CLI installed with huggingface-hub next to this Python, or the one on PATH."""
    local = Path(sys.executable).parent / "hf"
    if local.exists():
        return str(local)
    found = shutil.which("hf")
    if found is None:
        raise FileNotFoundError("The `hf` command (from huggingface-hub) was not found.")
    return found


def fetch_urdfs(bucket: str | None = None, dest: Path = URDF_DIR) -> None:
    """Sync the URDFs and meshes from the bucket into `dest` (only changed files are downloaded)."""
    bucket = bucket or os.environ.get("URDF_BUCKET") or DEFAULT_BUCKET
    ensure_home()
    dest.mkdir(parents=True, exist_ok=True)
    print(f"Syncing {bucket} -> {pretty(dest)}", flush=True)
    subprocess.run([hf_executable(), "buckets", "sync", bucket, str(dest)], check=True)
