# Missile C++ core simulation

Truth physics and guidance for the interceptor live in **`libflightsim_engagement`**. This document describes the missile 6-DOF core: state, per-tick pipeline, modules, frames, and how ROS / scenarios call it.

FlightSim is the **single source of truth**. UE5 and Isaac consume published state; they do not integrate missile EOMs.

## Role in the stack

```text
EngagementScenario / engagement_node
        │
        ▼
  initialize_missile() / step_missile()     ← this document
        │
        ├─ geometric seeker  (default)
        └─ optional SeekerTrack from vision  (use_vision_seeker)
        │
        ▼
  /flightsim/missile_state · scene_state · engagement_status
```

| Layer | Library / path | Responsibility |
|-------|----------------|----------------|
| Scenario orchestration | `libs/engagement/src/scenario/` | Launch, tick order, hit/miss |
| Missile core | `libs/engagement/src/missile/` | Guidance, aero wrench, 6-DOF, seeker fallback |
| Generic FDM primitives | `libs/fdm/` | Shared aero/EOM helpers (aircraft path; missile has its own EOM) |
| Vision seeker (optional) | `libs/vision/` | Image → `SeekerTrack` for closed-loop guidance |

## Source map

| Header | Implementation | Purpose |
|--------|----------------|---------|
| [`missile.hpp`](../../include/flightsim/engagement/missile/missile.hpp) | [`missile.cpp`](../../libs/engagement/src/missile/missile.cpp) | Public API: init, step, range, nose position |
| [`missile_object.hpp`](../../include/flightsim/engagement/missile/missile_object.hpp) | [`missile_object.cpp`](../../libs/engagement/src/missile/missile_object.cpp) | State + attributes + defaults |
| [`missile_eom.hpp`](../../include/flightsim/engagement/missile/missile_eom.hpp) | [`missile_eom.cpp`](../../libs/engagement/src/missile/missile_eom.cpp) | Semi-implicit Euler 6-DOF + attitude helpers |
| [`missile_control_allocation.hpp`](../../include/flightsim/engagement/missile/missile_control_allocation.hpp) | [`missile_control_allocation.cpp`](../../libs/engagement/src/missile/missile_control_allocation.cpp) | Cruciform fin mix + 1st-order servos |
| [`missile_mass_properties.hpp`](../../include/flightsim/engagement/missile/missile_mass_properties.hpp) | [`missile_mass_properties.cpp`](../../libs/engagement/src/missile/missile_mass_properties.cpp) | Burn-linked mass / CG / inertia |
| [`geometric_seeker_source.hpp`](../../include/flightsim/engagement/missile/geometric_seeker_source.hpp) | [`geometric_seeker_source.cpp`](../../libs/engagement/src/missile/geometric_seeker_source.cpp) | Truth LOS seeker when vision is off |

Unit coverage: [`tests/unit/test_engagement.cpp`](../../tests/unit/test_engagement.cpp).

## Frames and conventions

- **World / truth:** NED meters — North (+X), East (+Y), Down (+Z). Altitude = `−z`.
- **Body:** +X forward (nose), +Y right, +Z down (aircraft-style).
- **Attitude:** body→NED quaternion `[w, x, y, z]` (`core::Quaternion`).
- **Nose tip:** `position_ned_m + body_x_ned(attitude) * (length_m / 2)` — used for kill-radius and range.

Same NED contract as the [UE5 ROS bridge](../ue5/ros2_bridge_contract.md).

## Public API

```cpp
namespace flightsim::engagement {

void initialize_missile(MissileObject& missile, const MissileAttributes& attributes,
                        const core::Vec3& launch_position_ned_m,
                        const core::Vec3& launch_velocity_ned_mps) noexcept;

void step_missile(MissileObject& missile,
                  const core::Vec3& target_position_ned_m,
                  const core::Vec3& target_velocity_ned_mps,
                  float dt,
                  const vision::SeekerTrack* seeker_track = nullptr) noexcept;

float range_to_target(const MissileObject& missile,
                      const core::Vec3& target_position_ned_m) noexcept;

core::Vec3 missile_nose_position(const MissileObject& missile) noexcept;

MissileAttributes default_missile_attributes() noexcept;

}
```

Callers:

- `EngagementScenario::initialize` / `step` — [`scenario.cpp`](../../libs/engagement/src/scenario/scenario.cpp)
- `apps/engagement_runner`, `apps/air_defense_runner`
- `ros2/flightsim_ros/src/engagement_node.cpp`

Typical fixed step: **`dt = 0.01 s` (100 Hz)**.

## State (`MissileObject`)

| Field | Meaning |
|-------|---------|
| `position_ned_m` / `velocity_ned_mps` | CG translational state in NED |
| `attitude` / `angular_rate_body_rps` | Orientation and body rates `p,q,r` |
| `surfaces` / `commanded_surfaces` | Virtual pitch/yaw/roll fin angles (active path in `step_missile`) |
| `fins` / `commanded_fins` | Four individual actuators (allocation module; see below) |
| `mass_properties` | Time-varying mass / CG / inertia from burn model |
| `wrench` | Last body force `N` + moment `N·m` applied to EOM |
| `thrust_n` | Axial motor force while burning |
| `flight_time_sec` | Time since launch |
| `active` / `hit` | Flight and intercept flags |
| `seeker_locked` / `seeker_los_angle_rad` / `seeker_range_m` | Seeker telemetry snapshot |

Attributes (`MissileAttributes`) hold geometry, limits, aero coefficients, propulsion endpoints, and allocation config. Defaults come from `default_missile_attributes()` (~3.66 m interceptor class).

### Default interceptor (summary)

| Parameter | Default |
|-----------|---------|
| Length / diameter | 3.66 m / 0.34 m |
| Mass | 152 kg (90 dry + 62 propellant) |
| Max thrust / burn | 12 kN / 8 s |
| Max speed | 700 m/s |
| Max lateral accel | 60 m/s² |
| PN navigation gain | 5.5 |
| Kill radius | 5 m |
| Seeker FOV (az/el) | ±0.52 rad (~30°) half-angle used vs LOS |
| Acquisition range | 8 km |

## Per-tick pipeline (`step_missile`)

Early exits: inactive, already hit, or `dt ≤ 0`.

```text
1. flight_time += dt
2. If nose-to-target range ≤ kill_radius → hit, deactivate, clear thrust/wrench
3. Resolve seeker track
      · if seeker_track == null or !valid → geometric truth track
      · else use external vision track
4. Apply track → seeker_locked / range / LOS angle
5. Thrust = max while flight_time ≤ burn_time else 0
6. Proportional navigation → commanded lateral accel (NED)
      · gravity bias (−g on Z)
      · boost along LOS while motor burning
      · clamp to accel envelope
7. Map accel → commanded virtual surfaces; rate-limit → surfaces
8. Build body wrench (thrust, guidance force, drag, aero moments, damping)
9. integrate_missile_eom(...)
10. Clamp speed ≤ max_speed_mps
```

### Guidance (proportional navigation)

With LOS unit `λ̂` and LOS rate `λ̇` (from track or relative kinematics):

```text
a_cmd = N · V_c · (λ̇ × v̂)
```

- `N` = `navigation_gain` when seeker locked, else `0.25 × navigation_gain`
- `V_c` = closing speed along LOS (clamped ≥ 0)
- Lateral magnitude capped by `max_lateral_accel_mps2`
- Gravity compensation: `a_z -= g`
- During burn, an axial boost term along LOS is added (portion of `T/m`)

### Control surfaces (active path)

`step_missile` currently commands **virtual** pitch/yaw/roll angles:

1. Body-frame desired lateral force from commanded accel
2. Divide by `fin_force_per_rad_n` → deflection command
3. Rate + deflection limits from `ControlSurfaceLimits`

Those angles enter moments in `compute_wrench` (`fin_*_moment_per_rad_nm`).

### Control allocation module (available)

`allocate_virtual_axes_to_fin_commands` / `step_fin_servos` / `effective_virtual_axes_from_fins` implement a **cruciform +** mixer:

| Fin index | Location | Pitch | Yaw | Roll |
|-----------|----------|-------|-----|------|
| 0 | top | +1 | 0 | +0.25 |
| 1 | right | 0 | −1 | +0.25 |
| 2 | bottom | −1 | 0 | +0.25 |
| 3 | left | 0 | +1 | +0.25 |

Servos are first-order with rate and deflection limits (`time_constant_sec ≈ 0.03 s`). Unit-tested; **not yet called from `step_missile`** — wiring them into the wrench path is the next fidelity step (see [PHASE_PLAN](../PHASE_PLAN.md) Phase 1 done / Phase 2 next).

### Mass / propellant (available)

`update_mass_properties_from_burn` linearly interpolates mass, CG, and principal inertias from full → dry over `burn_time_sec`. Unit-tested; **`step_missile` still integrates with `attributes.mass_kg` / `attributes.inertia`**. Burn currently only gates thrust on/off via `flight_time_sec`.

## Equations of motion

[`integrate_missile_eom`](../../libs/engagement/src/missile/missile_eom.cpp):

1. Rotate body force to NED; add weight `m·g` on +Down
2. Translational: `a = F / m`
3. Angular (diagonal inertia `Ixx, Iyy, Izz`):

```text
I · ω̇ = M − ω × (I · ω)
```

4. Quaternion kinematics from body rates
5. **Semi-implicit Euler:** update `v`, then `p` with new `v`; update `ω` and `q`; renormalize quaternion

Helpers: `body_x_ned`, `ned_to_body`, `quaternion_from_body_x_ned` (launch attitude from velocity).

### Wrench model (in `missile.cpp`)

Body force / moment includes:

- Axial thrust while burning
- Guidance force (commanded accel × mass, rotated to body)
- Parasite drag `∝ q∞ · S · Cd` along body −X
- Static aero moments from `α`, `β`
- Velocity / LOS alignment moments (stronger during boost)
- Fin deflection moments + rate damping

## Seeker interface

```text
                    ┌─────────────────────────┐
  truth target ────►│ geometric seeker        │──► SeekerTrack
                    └─────────────────────────┘
                              ▲ fallback
  vision node ── SeekerTrack ─┴──► step_missile(..., seeker_track)
```

- **Geometric:** LOS = target − missile position; lock if range ≤ acquisition and angle to boresight ≤ half FOV.
- **External:** When `use_vision_seeker` and `ISeekerTrackSource::poll` returns a valid track, that track drives PN; invalid/missing falls back to geometric.

`SeekerTrack` fields used for guidance: `valid`, `locked`, `los_unit_ned`, `los_rate_ned`, `range_m`.

## ROS / telemetry

Published as `flightsim_msgs/MissileState` ([`MissileState.msg`](../../ros2/flightsim_msgs/msg/MissileState.msg)):

- Position, velocity, attitude, body rates
- `active` / `hit`, `thrust_n`, length
- Virtual fin pitch/yaw/roll
- Seeker lock, FOV, range

Scene entities for UE5 come from `vision::build_scene_state` (missile + target + depot).

## How to exercise the core

```bash
# Build C++ libs + unit tests
./scripts/build_debug.sh

# Headless engagement (default air-defense scenario)
# binary under build/<profile>/apps/engagement_runner/

# ROS engagement node (publishes missile_state)
./scripts/build_ros2.sh
source ros2/install/debug/setup.bash
ros2 launch flightsim_ros engagement.launch.py
```

Focused tests:

```bash
ctest --preset debug -R engagement --output-on-failure
```

## Current fidelity vs roadmap

| Capability | In tree | Used by `step_missile` |
|------------|---------|------------------------|
| PN guidance + geometric/external seeker | ✅ | ✅ |
| Virtual surface rate limits + aero wrench | ✅ | ✅ |
| Diagonal-inertia 6-DOF SI Euler | ✅ | ✅ |
| 4-fin allocation + servo dynamics | ✅ | ❌ (unit tests only) |
| Burn-varying mass / CG / inertia | ✅ | ❌ (unit tests only; thrust schedule yes) |
| Table aero / rate-damper autopilot | ⬜ | — ([Phase 2](../PHASE_PLAN.md)) |
| Full inertia tensor products | ⬜ | — (Phase 3) |

## Related docs

- [Phase plan — Missile 6-DOF](../PHASE_PLAN.md#simulation--missile-6-dof)
- [UE5 ROS 2 bridge contract](../ue5/ros2_bridge_contract.md)
- [Isaac dataset sidecar](../isaac/dataset_sidecar.md) (truth-only runs; geometric seeker)
- Mesh asset: [generic interceptor model](../../assets/ue5/models/generic_missile/README.md)
