#!/usr/bin/env bash
# Download and prepare generic interceptor missile mesh for UE5.
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ASSET_DIR="${ROOT}/assets/ue5/models/generic_missile/source"
ICOSA_ASSET_ID="1Xid2Qhqn2s"
GLTF_URL="https://s3.us-east-005.backblazeb2.com/icosa-gallery/poly/${ICOSA_ASSET_ID}/model_(GLTFupdated).gltf"
BIN_URL="https://s3.us-east-005.backblazeb2.com/icosa-gallery/poly/${ICOSA_ASSET_ID}/model.bin"
TARGET_LENGTH_M="3.66"

usage() {
  cat <<EOF
Usage: $(basename "$0")

Downloads Jarlan Perez "Missile" (Google Poly / CC BY 3.0) via Icosa Gallery
and builds generic_missile.obj + generic_missile.glb scaled to ${TARGET_LENGTH_M} m (+X forward).

Output: ${ASSET_DIR}
EOF
}

ensure_trimesh() {
  python3 -c "import trimesh, numpy" 2>/dev/null || pip install trimesh numpy -q
}

fetch_and_build() {
  ensure_trimesh
  mkdir -p "${ASSET_DIR}"
  tmp_dir="$(mktemp -d)"
  trap 'rm -rf "${tmp_dir}"' EXIT

  echo "==> Downloading CC BY missile model (${ICOSA_ASSET_ID})"
  curl -sSL "${GLTF_URL}" -o "${tmp_dir}/model_(GLTFupdated).gltf"
  curl -sSL "${BIN_URL}" -o "${tmp_dir}/model.bin"

  python3 <<PY
import trimesh
import trimesh.transformations as tf
import numpy as np
from pathlib import Path

tmp = Path("${tmp_dir}")
out = Path("${ASSET_DIR}")
target_length = float("${TARGET_LENGTH_M}")

mesh = trimesh.load(tmp / "model_(GLTFupdated).gltf", force="mesh")
mesh.apply_translation(-mesh.centroid)
# Source length axis is +Y; align nose to FlightSim body +X.
mesh.apply_transform(tf.rotation_matrix(-np.pi / 2.0, [0.0, 0.0, 1.0]))
longest = max(mesh.extents)
mesh.apply_scale(target_length / longest)
mesh.apply_translation(-mesh.centroid)

mesh.export(out / "generic_missile.obj")
mesh.export(out / "generic_missile.glb")
print(f"Built {out / 'generic_missile.glb'} extents={mesh.extents}")
PY
}

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  usage
  exit 0
fi

fetch_and_build
echo "Done."
