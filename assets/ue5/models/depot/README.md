# Depot — ground objective target

Static **ground depot** that the inbound **Shahed-136** drone is steering toward. The interceptor **missile** targets the Shahed, not the depot.

## ROS / scene mapping

| Field | Value |
|-------|--------|
| `SceneEntity.id` | `depot` |
| `SceneEntity.type` | `depot` |
| `SceneEntity.model` | `depot` |

Published on `/flightsim/scene_state` only (static position, zero velocity). Map to **`BP_Depot`** in UE5 — use a building, pad, or warehouse static mesh at terrain height.

## Engagement chain

```
Depot (static ground objective)
  ↑ inbound
Shahed-136 (moving threat, model shahed_136)
  ↑ intercept
Missile (generic_missile)
```

## Related assets

- Shahed mesh: [`../shahed_136/`](../shahed_136/README.md)
- Missile mesh: [`../generic_missile/`](../generic_missile/README.md)
- Bridge contract: [`../../../docs/ue5/ros2_bridge_contract.md`](../../../docs/ue5/ros2_bridge_contract.md)
