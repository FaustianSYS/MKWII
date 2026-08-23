#!/usr/bin/env bash
# Download AccuCities TQ3280 London sample and build UE5 map mesh cache.
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MAP_DIR="${ROOT}/assets/gui/maps/accucities_tq3280"
RAW_DIR="${MAP_DIR}/raw"
OBJ_ZIP_URL="https://www.accucities.com/wp-content/uploads/samples/AccuCities-OBJ-sample-3D%20Model-of-London-TQ3280.zip"

mkdir -p "${RAW_DIR}"

echo "==> Downloading AccuCities TQ3280 OBJ sample"
curl -sSL "${OBJ_ZIP_URL}" -o "${RAW_DIR}/london_obj.zip"

echo "==> Extracting Level 2 (No Logo) tile"
unzip -o "${RAW_DIR}/london_obj.zip" \
  "OBJ/Level_2/No_Logo/TQ3280_LD_SAMPLE_ZERO.obj" \
  "OBJ/Level_2/No_Logo/TQ3280_LD_SAMPLE_ZERO.mtl" \
  -d "${RAW_DIR}"

python3 -m pip install trimesh fast_simplification numpy -q

python3 <<PY
import struct
from pathlib import Path
import trimesh
import trimesh.transformations as tf
import numpy as np

src = Path("${RAW_DIR}/OBJ/Level_2/No_Logo/TQ3280_LD_SAMPLE_ZERO.obj")
loaded = trimesh.load(src, force="mesh", process=True)
if isinstance(loaded, trimesh.Scene):
    mesh = loaded.dump(concatenate=True)
else:
    mesh = loaded

def clean_mesh(m):
    m.remove_infinite_values()
    m.update_faces(m.nondegenerate_faces())
    m.merge_vertices(digits_vertex=4)
    m.update_faces(m.unique_faces())
    m.remove_unreferenced_vertices()
    m.fix_normals()
    trimesh.repair.fix_winding(m)
    return m

if len(mesh.faces) > 90000:
    mesh = mesh.simplify_quadric_decimation(face_count=90000)
mesh = clean_mesh(mesh)

# AccuCities OBJ is Y-up; horizontal plane is X-Z.
cx = 0.5 * (mesh.bounds[0][0] + mesh.bounds[1][0])
cz = 0.5 * (mesh.bounds[0][2] + mesh.bounds[1][2])
min_y = mesh.bounds[0][1]
mesh.apply_translation([-cx, -min_y, -cz])

# Y-up -> Z-up for OpenGL/NED visual (+X north ground plane, +Z up).
mesh.apply_transform(tf.rotation_matrix(np.pi / 2.0, [1.0, 0.0, 0.0]))
mesh.apply_translation([0.0, 0.0, -mesh.bounds[0][2]])

# Align tile long axis to North (+X); corrects 90 deg clockwise misalignment.
mesh.apply_transform(tf.rotation_matrix(-np.pi / 2.0, [0.0, 0.0, 1.0]))

horiz = max(mesh.extents[0], mesh.extents[1])
scale = 500.0 / horiz if horiz > 1.0 else 1.0
mesh.apply_scale(scale)
mesh.apply_translation([0.0, 0.0, -mesh.bounds[0][2]])
mesh = clean_mesh(mesh)

out_dir = Path("${MAP_DIR}")
mesh.export(out_dir / "london_tq3280_lod2.obj")
cache = out_dir / "london_tq3280_lod2.bin"
with cache.open("wb") as f:
    f.write(struct.pack("<IIIf", 1, len(mesh.vertices), len(mesh.faces), float(scale)))
    for v in mesh.vertices:
        f.write(struct.pack("<fff", float(v[0]), float(v[1]), float(v[2])))
    for tri in mesh.faces:
        f.write(struct.pack("<III", int(tri[0]), int(tri[1]), int(tri[2])))
print("Wrote", cache, "verts", len(mesh.vertices), "faces", len(mesh.faces), "scale", scale)
print("Extents (m):", mesh.extents)
print("Winding consistent:", mesh.is_winding_consistent)
PY

echo "Done. UE5 loads ${MAP_DIR}/london_tq3280_lod2.obj"
