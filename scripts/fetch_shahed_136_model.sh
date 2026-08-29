#!/usr/bin/env bash
# Download and prepare Shahed-136 3D assets for UE5.
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ASSET_DIR="${ROOT}/assets/ue5/models/shahed_136/source"
WIKIMEDIA_URL="https://upload.wikimedia.org/wikipedia/commons/f/f8/Shahed-136.stl"
HARRY_UID="bfc7a02b26814f51a265e57fcf2babc6"

usage() {
  cat <<EOF
Usage: $(basename "$0") [OPTIONS]

Options:
  (default)            Download Wikimedia STL and build shahed_136.obj + shahed_136.glb
  --sketchfab-harry    Download alternate Sketchfab GLB (CC BY-NC-ND, needs SKETCHFAB_API_TOKEN)
  --help               Show this help

Output directory: ${ASSET_DIR}
EOF
}

ensure_trimesh() {
  python3 -c "import trimesh, numpy" 2>/dev/null || pip install trimesh numpy -q
}

build_converted_meshes() {
  ensure_trimesh
  python3 <<PY
import trimesh
from pathlib import Path

src = Path("${ASSET_DIR}/shahed_136_wikimedia.stl")
if not src.exists():
    raise SystemExit(f"Missing source STL: {src}")

mesh = trimesh.load(src, force="mesh")
longest = max(mesh.extents[0], mesh.extents[2])
mesh.apply_scale(3.5 / longest)
mesh.apply_translation(-mesh.centroid)
if mesh.extents[2] > mesh.extents[0]:
    mesh.apply_transform(trimesh.transformations.rotation_matrix(1.57079632679, [0, 1, 0]))

out = Path("${ASSET_DIR}")
mesh.export(out / "shahed_136.obj")
mesh.export(out / "shahed_136.glb")
print(f"Built {out / 'shahed_136.glb'} extents={mesh.extents}")
PY
}

fetch_wikimedia() {
  mkdir -p "${ASSET_DIR}"
  echo "==> Downloading Wikimedia Shahed-136.stl"
  curl -sSL "${WIKIMEDIA_URL}" -o "${ASSET_DIR}/shahed_136_wikimedia.stl"
  build_converted_meshes
}

fetch_sketchfab_harry() {
  if [[ -z "${SKETCHFAB_API_TOKEN:-}" ]]; then
    echo "ERROR: Set SKETCHFAB_API_TOKEN (https://sketchfab.com/settings/password)" >&2
    exit 1
  fi
  mkdir -p "${ASSET_DIR}"
  echo "==> Downloading Sketchfab Shahed 136 (${HARRY_UID})"
  local archive_url
  archive_url="$(curl -sS -H "Authorization: Token ${SKETCHFAB_API_TOKEN}" \
    "https://api.sketchfab.com/v3/models/${HARRY_UID}/download" \
    | python3 -c "import sys,json; d=json.load(sys.stdin); print(d['glb']['url'])")"
  curl -sSL "${archive_url}" -o "${ASSET_DIR}/shahed_136_sketchfab_harry.glb"
  echo "Saved ${ASSET_DIR}/shahed_136_sketchfab_harry.glb (CC BY-NC-ND)"
}

main() {
  local action="wikimedia"
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --sketchfab-harry) action="sketchfab" ;;
      --help|-h) usage; exit 0 ;;
      *) echo "Unknown option: $1" >&2; usage; exit 1 ;;
    esac
    shift
  done

  case "${action}" in
    wikimedia) fetch_wikimedia ;;
    sketchfab) fetch_sketchfab_harry ;;
  esac
}

main "$@"
