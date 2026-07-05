# Isaac Sim integration

Isaac Sim is an **offline / parallel sidecar** for synthetic computer-vision datasets. FlightSim C++ remains the sole physics truth source.

## Phases

| Phase | Name | Status | Doc |
|-------|------|--------|-----|
| 0 Mode A | Dataset sidecar (truth export + render scaffold) | ✅ Done | [dataset_sidecar.md](dataset_sidecar.md) |
| 1 | Live Isaac seeker (USD prims, PNG export) | ⬜ Planned | [../PHASE_PLAN.md](../PHASE_PLAN.md) |
| 2 | Namespaced co-sim with UE5 | ⬜ Planned | [../PHASE_PLAN.md](../PHASE_PLAN.md) |

See the full roadmap in [docs/PHASE_PLAN.md](../PHASE_PLAN.md).

## Tools

| Script | Purpose |
|--------|---------|
| `tools/isaac/ned_transform.py` | NED ↔ Isaac coordinate helpers |
| `tools/isaac/truth_logger.py` | Record `/flightsim/*` truth to JSON frames |
| `tools/isaac/isaac_dataset_sidecar.py` | Isaac render + label export (or `--dry-run`) |

## Quick start (Phase 0 Mode A)

```bash
./scripts/build_ros2.sh
./scripts/launch_dataset_sidecar.sh
```
