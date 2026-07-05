# AccuCities TQ3280 — London tactical map (GUI)

3D city backdrop for the Qt **Tactical Overview** map tab.

## Source

- Product: [TQ3280 – Free 3D London Sample](https://www.accucities.com/product/tq3280-free-3d-london-samples/)
- Tile: **TQ3280** (London Bridge / Southwark / The Shard area, 1 km² sample)
- License: [CC BY-SA 4.0](https://creativecommons.org/licenses/by-sa/4.0/) — AccuCities / 3D London

## Files in repo

| File | Description |
|------|-------------|
| `london_tq3280_lod2.bin` | Simplified mesh cache for real-time Qt GL (~60k faces) |
| `london_tq3280_lod2.obj` | Same mesh, Wavefront OBJ |
| `raw/london_obj.zip` | Official AccuCities OBJ sample (not committed if large) |

The GUI loads `london_tq3280_lod2.bin` automatically from this folder.

## Rebuild map cache

```bash
./scripts/fetch_london_map_accucities.sh
```

Requires `python3`, `trimesh`, and `fast_simplification`.

## Placement in sim

The city mesh is fixed at the world origin in **meters** (500 m tile). The default air-defense scenario uses the same meter scale so the Shahed drone (~3.5 m) and missile (~3.7 m) render at true size over the buildings. A cyan boundary ring marks the map extent.

## Attribution

> 3D London map sample TQ3280 by AccuCities, CC BY-SA 4.0
