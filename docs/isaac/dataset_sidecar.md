# Isaac Sim — Phase 0 Mode A (Dataset Sidecar)

FlightSim C++ remains the **single source of truth**. Isaac Sim is an **offline sidecar** for synthetic CV dataset generation. It does not close the guidance loop and does not replace UE5 for live monitoring.

## Architecture

```text
flightsim_engagement_node          Isaac Sim (sidecar)
  │                                  │
  ├─ /flightsim/scene_state ───────► subscribe · sync USD prims
  ├─ /flightsim/missile_state ─────► seeker camera mount
  ├─ /flightsim/target_state ──────► ground-truth labels
  └─ /flightsim/engagement_status    (optional episode metadata)

Isaac publishes (dataset output only — not consumed by engagement):
  /flightsim/isaac/seeker_camera/image
  /flightsim/isaac/labels/*.json  (local disk export recommended)
```

## Coordinate frame

Same contract as [UE5 bridge](../ue5/ros2_bridge_contract.md):

| NED | Isaac / Omniverse |
|-----|-------------------|
| North (+X) | +X |
| East (+Y) | +Y |
| Down (+Z) | −Z (Isaac Z-up) |

Attitudes: body→NED quaternion `[w, x, y, z]`.

Use `tools/isaac/ned_transform.py` for conversions.

## Quick start

### 1. Build FlightSim ROS stack

```bash
./scripts/build_ros2.sh
source ros2/install/debug/setup.bash
```

### 2. Start truth publisher + frame logger

```bash
./scripts/launch_dataset_sidecar.sh
```

This launches:

- `air_defense_dataset.launch.py` — engagement only, geometric seeker (no vision loop)
- `truth_logger.py` — writes per-step JSON frames to `datasets/<timestamp>/`

### 3. Run Isaac sidecar (when Isaac Sim is installed)

Terminal 2:

```bash
source /opt/ros/jazzy/setup.bash
source ros2/install/debug/setup.bash
export FLIGHTSIM_ROOT=~/FlightSim

# Dry-run: verify ROS subscription without Isaac
/usr/bin/python3 tools/isaac/isaac_dataset_sidecar.py --dry-run

# Live Isaac render + PNG export
/usr/bin/python3 tools/isaac/isaac_dataset_sidecar.py \
  --output datasets/isaac_run_001 \
  --headless
```

Requires Isaac Sim 4.x with ROS 2 bridge enabled and `ISAACSIM_PATH` set.

## Dataset layout

```
datasets/run_20260704_154500/
  metadata.json          # scenario, dt, topic names, frame count
  frames/
    000000.json          # truth snapshot (entities, missile, target)
    000001.json
    ...
  images/                # Isaac sidecar output (when running)
    000000.png
    labels/
      000000.json        # bbox, los_truth, entity poses in camera frame
```

## Domain randomization (Isaac-side)

Configure in `isaac_dataset_sidecar.py` or your USD scene:

- lighting intensity / color temperature
- fog / haze
- sensor noise (gain, Gaussian blur)
- background clutter
- Shahed appearance variants

FlightSim truth trajectories stay deterministic; randomization applies only to rendering.

## Topic reference

| Topic | Direction | Notes |
|-------|-----------|-------|
| `/flightsim/scene_state` | FlightSim → Isaac | Primary sync input |
| `/flightsim/missile_state` | FlightSim → Isaac | Camera mount + fin angles |
| `/flightsim/target_state` | FlightSim → Isaac | Ground-truth target |
| `/flightsim/engagement_status` | FlightSim → Isaac | Episode end / intercept flag |
| `/flightsim/isaac/seeker_camera/image` | Isaac → disk | Not fed back to engagement in Mode A |

## What Mode A does NOT include

- Closed-loop `/flightsim/seeker_track` feedback
- Concurrent UE5 operator view (run separately if needed)
- Isaac physics integration (actors are kinematic)

See [Phase 1+ in the phase plan](../PHASE_PLAN.md#phase-1--live-isaac-seeker--planned) for live Isaac seeker and namespaced co-simulation with UE5.
