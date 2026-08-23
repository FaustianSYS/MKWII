# FlightSim Phase Plan

Roadmap for simulation physics, computer vision, UE5 monitoring, and Isaac Sim integration.

**Repository:** [github.com/FaustianSYS/FlightSim](https://github.com/FaustianSYS/FlightSim)

## Domain split

| Domain | Libraries | ROS nodes | Coupling |
|--------|-----------|-----------|----------|
| **Simulation** | `libflightsim_engagement`, `libflightsim_fdm`, `libflightsim_sim` | `flightsim_engagement_node`, UE5 scene bridge | Publishes truth topics |
| **Computer Vision** | `libflightsim_vision` | `flightsim_vision_node`, `flightsim_ue5_bridge_node` | Consumes seeker images, publishes `seeker_track` |

Interface topics only — no cross-domain C++ linkage.

---

## Simulation — Missile 6-DOF

Reference: [Missile C++ core](sim/missile_core.md).

### Phase 1 — Control allocation & mass properties ✅ Done

**Goal:** Replace virtual fin shortcut with physically structured actuator and propellant pipeline.

| Deliverable | Status | Location |
|-------------|--------|----------|
| 4-fin cruciform control allocation | ✅ | `missile_control_allocation.hpp/.cpp` |
| 1st-order servo dynamics (rate/deflection limits) | ✅ | same |
| Linear propellant burn model | ✅ | `missile_mass_properties.hpp/.cpp` |
| CG shift + inertia interpolation over burn | ✅ | same |
| CG-offset moment in wrench | ✅ | `missile.cpp` → `step_missile()` |
| Unit tests (servo saturation, burn mass/CG) | ✅ | `tests/unit/test_engagement.cpp` |

**Verified:** air_defense intercept ~88 steps, miss distance ~4.8 m.

### Phase 2 — Aerodynamics & autopilot 🔄 Next

**Goal:** Replace linear aero with coefficient tables; add inner-loop autopilot.

| Deliverable | Status | Notes |
|-------------|--------|-------|
| Mach / α / β lookup tables (Cd, Cn, Cm) | ⬜ | Extend `missile.cpp` aero block |
| Fin effectiveness vs Mach / α | ⬜ | Per-fin moment tables |
| 3-axis rate damper autopilot | ⬜ | Between guidance and allocation |
| Acceleration command limiting | ⬜ | Structural / actuator envelope |
| Expanded unit tests (table interpolation, trim) | ⬜ | Golden aero cases |

**Exit criteria:** Stable intercept across Mach 0.5–2.5 envelope; autopilot holds commanded accel within 10% at 100 Hz.

### Phase 3 — Full engagement fidelity ⬜ Planned

| Deliverable | Status | Notes |
|-------------|--------|-------|
| Full inertia tensor (Ixy, Ixz, Iyz) | ⬜ | Off-diagonal terms in EOM |
| Seeker gimbal / FOV model in sim | ⬜ | Complement geometric seeker |
| Multi-target scenario support | ⬜ | Extend `EngagementScenario` |
| Monte Carlo batch runner | ⬜ | Pk / miss-distance statistics |

---

## Computer Vision — Seeker pipeline

### Phase CV-1 — Baseline blob tracker ✅ Done

| Deliverable | Status | Location |
|-------------|--------|----------|
| Pinhole camera model | ✅ | `libs/vision/camera_model.cpp` |
| Blob detection + centroid | ✅ | `seeker_image_processor.cpp` |
| Seeker track output (az/el, LOS, lock) | ✅ | `SeekerTrack` msg |
| ROS vision node | ✅ | `flightsim_vision_node` |
| Software bridge (headless seeker) | ✅ | `flightsim_ue5_bridge_node` |
| Closed-loop launch files | ✅ | `air_defense_vision.launch.py` |

### Phase CV-2 — Robust detection 🔄 Next

| Deliverable | Status | Notes |
|-------------|--------|-------|
| Adaptive threshold / background subtraction | ⬜ | Clutter rejection |
| Track filtering (Kalman / α-β) | ⬜ | Reduce jitter on `seeker_track` |
| Multi-hypothesis blob association | ⬜ | Decoy / flare rejection |
| Isaac-generated training data ingest | ⬜ | Feed from Phase 0 Mode A datasets |

### Phase CV-3 — ML seeker ⬜ Planned

| Deliverable | Status | Notes |
|-------------|--------|-------|
| ONNX / TensorRT detector integration | ⬜ | Replace blob detect |
| Sim-to-real domain adaptation pipeline | ⬜ | Isaac randomization → fine-tune |
| Seeker performance metrics (Pd, Pfa) | ⬜ | Offline eval harness |

---

## UE5 — Monitoring & visualization

### Phase UE-1 — Scene sync & London map ✅ Done

| Deliverable | Status | Location |
|-------------|--------|----------|
| ROS 2 C bridge (`fsros2`) | ✅ | `flightsim_ue5_ros2_bridge` |
| Scene actor teleport from truth | ✅ | `FlightSimROS2Subsystem` |
| AccuCities London LOD2 map | ✅ | `FlightSimLondonMapActor` |
| Seeker SceneCapture (640×480 mono8) | ✅ | UE5 plugin |
| Telemetry Blueprint struct | ✅ | `FFlightSimTelemetry` |

### Phase UE-2 — Operator HUD 🔄 Next

| Deliverable | Status | Notes |
|-------------|--------|-------|
| UMG HUD overlay (range, lock, fins, thrust) | ⬜ | Bind to `FFlightSimTelemetry` |
| Camera director (tactical / missile / target / free) | ⬜ | Planned in architecture canvas |
| Seeker picture-in-picture on HUD | ⬜ | Subscribes seeker image topic |
| Intercept / miss-distance event display | ⬜ | From `engagement_status` |

---

## Isaac Sim — Synthetic CV datasets

### Phase 0 Mode A — Dataset sidecar ✅ Done

**Goal:** Offline truth export + Isaac render scaffold. No closed-loop vision.

| Deliverable | Status | Location |
|-------------|--------|----------|
| `air_defense_dataset.launch.py` (vision off) | ✅ | `ros2/flightsim_ros/launch/` |
| `truth_logger.py` → JSON frames | ✅ | `tools/isaac/` |
| `isaac_dataset_sidecar.py` (dry-run + scaffold) | ✅ | same |
| NED ↔ Isaac transforms | ✅ | `ned_transform.py` |
| Launch script | ✅ | `scripts/launch_dataset_sidecar.sh` |
| Documentation | ✅ | `docs/isaac/dataset_sidecar.md` |

**Verified:** 78 JSON frames per air_defense run; script exits cleanly on engagement complete.

Details: [docs/isaac/dataset_sidecar.md](isaac/dataset_sidecar.md)

### Phase 1 — Live Isaac seeker ⬜ Planned

| Deliverable | Status | Notes |
|-------------|--------|-------|
| USD asset import (missile, Shahed, depot, London) | ⬜ | Kinematic prims |
| RTX seeker camera on missile nose | ⬜ | Match UE5 640×480 contract |
| PNG + label export at sim rate | ⬜ | Complete `isaac_dataset_sidecar.py` TODOs |
| Domain randomization presets | ⬜ | Lighting, fog, sensor noise |

### Phase 2 — Namespaced co-simulation ⬜ Planned

| Deliverable | Status | Notes |
|-------------|--------|-------|
| Run Isaac + UE5 concurrently (namespaced topics) | ⬜ | `/flightsim/isaac/*` vs `/flightsim/ue5/*` |
| Isaac seeker → `seeker_track` closed loop | ⬜ | Optional `use_isaac_seeker` param |
| Side-by-side operator view (UE5) + dataset capture (Isaac) | ⬜ | Same truth publisher |

---

## Launch matrix

| Mode | Script / launch file | Domains | Vision loop |
|------|---------------------|---------|-------------|
| UE5 co-sim | `launch_ue5_ros2.sh` + `air_defense_vision.launch.py` | Sim + CV + UE5 | Closed (UE5 capture) |
| Software bridge | `launch_ue5_bridge.sh` + `air_defense_ue5.launch.py` | Sim + CV | Closed (synthetic) |
| Isaac dataset | `launch_dataset_sidecar.sh` | Sim only | Off (geometric seeker) |
| Sim-only headless | `air_defense_dataset.launch.py` | Sim only | Off |

---

## Current priority order

1. **Phase 2 missile** — Mach/α aero tables + rate-damper autopilot
2. **Phase UE-2** — UMG HUD + camera director
3. **Isaac Phase 1** — USD scene + live PNG export
4. **Phase CV-2** — Track filtering + Isaac dataset training pipeline

---

## Build & test gates (all phases)

```bash
./scripts/build_debug.sh          # unit + integration — must pass before merge
./scripts/build_ros2.sh           # ROS 2 overlay
./scripts/build_ue5_plugin.sh     # UE5 plugin (optional, requires UE5)
```

Regression target: air_defense intercept completes; vision loop publishes `seeker_track` at ≥50 Hz when enabled.
