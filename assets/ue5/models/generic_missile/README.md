# Generic interceptor missile — UE5 model

Low-poly missile mesh for UE5 `SceneEntity` `id=missile`.

## Included mesh

| File | Format | Notes |
|------|--------|--------|
| `source/generic_missile.glb` | glTF binary | **Recommended UE5 import** |
| `source/generic_missile.obj` | Wavefront OBJ | Alternate mesh export |

Meshes are scaled to **3.66 m** length (`MissileAttributes.length_m`), centered at the origin, nose **+X**.

## Source & license

- **Original:** [Missile by Jarlan Perez](https://poly.pizza/m/1Xid2Qhqn2s) (Google Poly)
- **Archive:** [Icosa Gallery](https://api.icosa.gallery/v1/assets/1Xid2Qhqn2s)
- **License:** [CC BY 3.0](https://creativecommons.org/licenses/by/3.0/) — attribution required

See `source/ATTRIBUTION.txt`.

## Refresh

```bash
./scripts/fetch_generic_missile_model.sh
```

Requires `python3`, `trimesh`, `numpy`.

## Sim alignment

| FlightSim | Model |
|-----------|--------|
| `MissileAttributes.length_m = 3.66` | mesh length ~3.66 m |
| Body +X forward | mesh +X nose |
| `SceneEntity type=missile` | UE5 actor |
