#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

BUCKET="${URDF_BUCKET:-hf://buckets/lerobot/robot-urdfs/so101}"
DEST="urdf"

uvx --python 3.12 --from "huggingface_hub>=1.5.0" hf buckets sync "$BUCKET" "$DEST"

