"""Download (or update) the SO-101 URDFs and meshes into so101_config/urdf of this project.

Run with:
    uv run so101-fetch-urdfs
    uv run so101-fetch-urdfs --bucket hf://buckets/<namespace>/<bucket>/<prefix>
"""

import argparse
import subprocess
import sys

from so101.urdf import DEFAULT_BUCKET, fetch_urdfs


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--bucket", default=None, help=f"source bucket (default: $URDF_BUCKET or {DEFAULT_BUCKET})"
    )
    args = parser.parse_args()
    try:
        fetch_urdfs(args.bucket)
    except (FileNotFoundError, subprocess.CalledProcessError) as e:
        sys.exit(f"Fetching the URDFs failed: {e}")


if __name__ == "__main__":
    main()
