# Shahed-136 — UE5 drone model

Primary visual for the air-defense **depot** target actor in UE5 (`SceneEntity` `id=depot`, `model=shahed_136`).

## Included mesh (ready for UE5)

| File | Format | Notes |
|------|--------|--------|
| `source/shahed_136.glb` | glTF binary | **Recommended UE5 import** |
| `source/shahed_136.obj` | Wavefront OBJ | Alternate import |
| `source/shahed_136_wikimedia.stl` | STL (original) | Unscaled source from Wikimedia |

Meshes are scaled to **~3.5 m** longest axis (Shahed-136 length) and centered at the origin. Forward axis is aligned to **+X** for FlightSim NED / missile body conventions.

## Source & license

- **Original 3D model:** [Sketchfab — Shahed-136 by Idmenthal](https://sketchfab.com/3d-models/shahed-136-b604d1f1f8d04996a2959d34515bcf7d)
- **Redistribution:** [Wikimedia Commons — Shahed-136.stl](https://commons.wikimedia.org/wiki/File:Shahed-136.stl)
- **License:** [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/) — attribution required

See `source/ATTRIBUTION.txt`.

## UE5 import

1. Import `source/shahed_136.glb` into `Content/FlightSim/Actors/BP_Shahed136/Meshes/`.
2. Import settings:
   - **Scale:** 1.0 (already in meters)
   - **Forward / front:** +X
   - **Combine meshes:** on
   - **Generate lightmap UVs:** as needed
3. Create **`BP_Shahed136`**:
   - Static mesh component → imported mesh
   - Subscribe to `/flightsim/scene_state`, update from entity `type=depot` / `model=shahed_136`
   - Apply NED → UE transform per [ros2_bridge_contract.md](../../docs/ue5/ros2_bridge_contract.md)

## Sim alignment

| Real Shahed-136 | FlightSim `DroneAttributes` |
|-----------------|----------------------------|
| Length ~3.5 m | scaled mesh length ~3.5 m |
| Wingspan ~2.5 m | `wingspan_m = 2.5` |
| Cruise ~50 m/s | `max_speed_mps = 50` (default) |

## Refresh / optional downloads

```bash
# Re-download Wikimedia STL and rebuild OBJ/GLB (needs: pip install trimesh numpy)
./scripts/fetch_shahed_136_model.sh

# Higher-detail Sketchfab GLB (CC BY-NC-ND, non-commercial) — requires API token
export SKETCHFAB_API_TOKEN=your_token
./scripts/fetch_shahed_136_model.sh --sketchfab-harry
```

## Alternate free model

[Sketchfab — Shahed 136 by harry](https://sketchfab.com/3d-models/shahed-136-bfc7a02b26814f51a265e57fcf2babc6) (CC BY-NC-ND) via `--sketchfab-harry`.
