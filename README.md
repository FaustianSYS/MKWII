# FlightSim

Air-defense engagement simulator with 6-DOF missile physics, ROS 2 integration, UE5 co-simulation, and Isaac Sim dataset export.

## Architecture

Two independent domains coupled via ROS 2 topics:

| Domain | Libraries | Role |
|--------|-----------|------|
| **Simulation** | `libflightsim_engagement`, `libflightsim_fdm` | Truth physics, scenario, guidance |
| **Computer Vision** | `libflightsim_vision` | Seeker image processing, track output |

See [docs/PHASE_PLAN.md](docs/PHASE_PLAN.md) for the full roadmap (simulation, CV, UE5, Isaac tracks).

See [docs/sim/missile_core.md](docs/sim/missile_core.md) for the missile 6-DOF C++ core (guidance, aero, EOM).

See [docs/sim/drone_core.md](docs/sim/drone_core.md) for the drone / Shahed target kinematics core.

See [docs/isaac/dataset_sidecar.md](docs/isaac/dataset_sidecar.md) for Isaac Sim Phase 0 integration.

## Quick start

```bash
./scripts/build_debug.sh          # C++ core + unit tests
./scripts/build_ros2.sh           # ROS 2 nodes + UE5 bridge lib
./scripts/build_ue5_plugin.sh     # UE5 plugin (requires UE5 + Epic GitHub access)

# Simulation + vision (software bridge, no Unreal)
./scripts/launch_ue5_bridge.sh
ros2 launch flightsim_ros air_defense_ue5.launch.py use_vision_seeker:=true

# Full UE5 co-sim (requires linked Epic/GitHub account)
./scripts/launch_ue5_ros2.sh
ros2 launch flightsim_ros air_defense_vision.launch.py

# Isaac dataset sidecar (sim only, no vision loop)
./scripts/launch_dataset_sidecar.sh
```

## UE5 / Epic GitHub

UE5 source access requires linking your GitHub account to Epic Games:

1. [Link GitHub to Epic](https://www.unrealengine.com/en-US/ue-on-github)
2. Add your SSH key to GitHub ([Settings → SSH keys](https://github.com/settings/keys))
3. Run `./scripts/setup_ue5_user.sh` to clone and build Unreal Engine 5.5

See [ue5/README.md](ue5/README.md) for the ROS 2 bridge contract.

## Requirements

- Ubuntu 24.04, ROS 2 Jazzy
- CMake 3.24+, C++20 compiler
- Python 3.12 (`/usr/bin/python3` for ROS nodes)
- Optional: UE5 5.5, Isaac Sim 4.x, NVIDIA GPU
