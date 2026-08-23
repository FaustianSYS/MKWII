# Drone / target C++ core simulation

The air-defense **drone (Shahed-class target)** is a kinematic mover inside **`libflightsim_engagement`**. It is not a full 6-DOF aircraft FDM: each tick samples Hermite heading/climb splines, applies mission and evasion corrections, then integrates position with a commanded groundspeed.

FlightSim remains the **single source of truth**. UE5 / Isaac consume published target state; they do not fly the drone.

## Role in the stack

```text
EngagementScenario / engagement_node
        │
        ▼
  initialize_target() / step_target()     ← this document
        │
        ├─ inbound  → fly toward depot
        ├─ outbound → fly away from reference
        └─ optional missile evasion (MissileThreat)
        │
        ▼
  /flightsim/target_state · scene_state (entity id=shahed)
        │
        ▼
  step_missile(..., target position/velocity)   guidance uses this truth
```

| Layer | Path | Responsibility |
|-------|------|----------------|
| Scenario orchestration | `libs/engagement/src/scenario/` | Tick order: target then missile; build scene |
| Drone preset | `include/flightsim/engagement/drone/` | Shahed-like attributes → `TargetConfig` |
| Target kinematics | `libs/engagement/src/target/` | Splines, inbound/outbound, evasion |
| Scene publish | `libs/engagement/src/scenario/scene_entity.cpp` | `id=shahed`, `model=shahed_136` |

Companion interceptor doc: [Missile C++ core](missile_core.md).

## Source map

| Header | Implementation | Purpose |
|--------|----------------|---------|
| [`drone.hpp`](../../include/flightsim/engagement/drone/drone.hpp) | (header-only) | `DroneAttributes`, `drone_target_config()`, defaults |
| [`target.hpp`](../../include/flightsim/engagement/target/target.hpp) | [`target.cpp`](../../libs/engagement/src/target/target.cpp) | State, config, runtime, init/step, headings |
| [`air_defense_scenario.hpp`](../../include/flightsim/engagement/scenario/air_defense_scenario.hpp) | — | Default inbound Shahed vs interceptor layout |

Unit coverage: [`tests/unit/test_engagement.cpp`](../../tests/unit/test_engagement.cpp) (outbound cruise).

## Frames and conventions

- **World / truth:** NED meters — North (+X), East (+Y), Down (+Z). Altitude = `−z`.
- **Heading:** yaw about Down; `0` = North, positive toward East (`atan2(East, North)`).
- **Climb rate:** NED `velocity.z` — positive = descending; negative = ascending.
- **Attitude for rendering:** derived from velocity direction via `quaternion_from_body_x_ned` (nose +X along track). There is no independent attitude dynamics.

Same NED contract as the [UE5 ROS bridge](../ue5/ros2_bridge_contract.md).

## Public API

```cpp
namespace flightsim::engagement {

void initialize_target(TargetState& state, TargetConfig& config, TargetRuntime& runtime,
                       const core::Vec3& start_position_ned_m, float initial_heading_rad) noexcept;

void step_target(TargetState& state, const TargetConfig& config, TargetRuntime& runtime, float dt,
                 const MissileThreat* missile_threat = nullptr) noexcept;

float inbound_heading_rad(const core::Vec3& aim_ned_m, const core::Vec3& position_ned_m) noexcept;
float outbound_heading_rad(const core::Vec3& reference_ned_m, const core::Vec3& position_ned_m) noexcept;

DroneAttributes default_drone_attributes() noexcept;
TargetConfig drone_target_config(const DroneAttributes& drone) noexcept;

}
```

Callers:

- `EngagementScenario::initialize` / `step` — [`scenario.cpp`](../../libs/engagement/src/scenario/scenario.cpp)
- Air-defense preset — [`air_defense_scenario.hpp`](../../include/flightsim/engagement/scenario/air_defense_scenario.hpp)
- `apps/air_defense_runner`, `ros2/.../engagement_node.cpp`

Typical fixed step: **`dt = 0.01 s` (100 Hz)** (same as missile).

## Drone attributes vs target config

`DroneAttributes` is a thin vehicle preset. Motion parameters live on `TargetConfig`.

| `DroneAttributes` | Default | Maps to |
|-------------------|---------|---------|
| `wingspan_m` | 2.5 | Visual / RCS context (not used in kinematics) |
| `length_m` | 3.5 | Aligns with Shahed-136 mesh (~3.5 m) |
| `rcs_m2` | 0.05 | Metadata for future RF / CV |
| `max_speed_mps` | 50 | → `TargetConfig::speed_mps` (air-defense overrides to **20**) |
| `max_climb_mps` | 4 | → `max_climb_rate_mps` |

`drone_target_config()` also sets:

| Field | Value | Role |
|-------|-------|------|
| `max_yaw_rate_rps` | 0.18 | Turn-rate limit |
| `spline_segment_sec` | 4.0 | Nominal Hermite segment length |
| `max_heading_change_rad` | 0.42 | Max random heading delta per splice |
| `outbound` | true (preset) | Soft “fly away” bias |
| `evade_missile` | true | Enable threat weave |
| `evasion_range_m` | 4800 | Start reacting inside this range |
| `evasion_gain` | 0.48 | Blend weight toward beam heading |
| `evasion_weave_period_sec` | 8.0 | Lateral / vertical weave period |
| `curved_initial_spline` | true | First segment is an intentional curve |
| `initial_spline_curve_rad` | 0.48 | Initial heading offset magnitude |
| `initial_spline_duration_scale` | 2.4 | Longer first segment |

**Air-defense scenario** then flips mission mode to **inbound** toward the depot and slows cruise to 20 m/s:

```text
target_inbound = true
target_outbound = false
speed_mps = 20
rng_seed = 2024
start ≈ (-120, -60, -70) NED  →  depot ≈ (170, 120, 0)
```

## State structs

### `TargetState` (published kinematics)

| Field | Meaning |
|-------|---------|
| `position_ned_m` | CG position |
| `velocity_ned_mps` | Ground velocity (horizontal speed fixed to `speed_mps`) |

### `TargetRuntime` (internal)

| Field | Meaning |
|-------|---------|
| `heading_rad` / `heading_rate_rps` | Current heading and rate |
| `climb_rate_mps` / `climb_accel_mps2` | Vertical channel |
| `segment` / `segment_elapsed_sec` | Active Hermite segment |
| `evasion_phase_sec` | Phase clock for weave `sin` |
| `rng` | Deterministic RNG (`rng_seed`) |

### `MissileThreat` (from scenario)

Filled each tick from the live missile when `missile.active` — position, velocity, `active` flag — and passed into `step_target`.

## Per-tick pipeline (`step_target`)

```text
1. evasion_phase += dt
2. Advance / splice Hermite segments as needed
3. Sample heading, heading_rate, climb, climb_accel from current segment
4. Clamp yaw rate, climb rate, climb accel
5. Soft mission correction (inbound and/or outbound)
6. Missile evasion weave (if enabled + threat closing)
7. velocity = [speed·cos(ψ), speed·sin(ψ), climb]
8. position += velocity · dt
```

### Hermite spline segments

Each segment stores start/end heading and climb, plus start/end rates (heading rate, climb accel). Sampling uses cubic Hermite:

```text
s = clamp(u / duration, 0, 1)

h00 =  2s³ − 3s² + 1
h10 =    s³ − 2s² + s
h01 = −2s³ + 3s²
h11 =    s³ −   s²

value(u) = h00·p0 + h10·T·v0 + h01·p1 + h11·T·v1
```

(`T` = segment duration; derivatives come from the Hermite derivative form.)

When a segment expires, `splice_spline_segment`:

1. Keeps continuity of heading / climb / rates at the splice
2. Draws a random heading delta in `±max_heading_change_rad`
3. Blends **55% random / 45% mission pull** toward inbound or outbound heading
4. Randomizes end climb and end rates within limits

Optional **curved initial spline** offsets the first heading left/right of the mission heading so the drone does not fly a straight line from t = 0.

### Mission modes

**Inbound** — aim point `inbound_reference_ned_m` (depot in air-defense):

```text
ψ_mission = atan2(aim_E − pos_E, aim_N − pos_N)
```

Soft correction: if horizontal velocity points away from the aim, blend heading toward `ψ_mission`.

**Outbound** — fly away from `outbound_reference_ned_m` (often launch site):

```text
ψ_outbound = atan2(pos_E − ref_E, pos_N − ref_N)
```

Soft correction if velocity points back toward the reference.

Inbound and outbound soft corrections can both be configured; air-defense uses inbound only.

### Missile evasion

Active when `evade_missile`, threat `active`, range in `(8 m, evasion_range_m]`, and closing speed `> 4 m/s`:

```text
closing = − (v_rel · λ̂)
threat  = (1 − range/R_max)² · clamp(closing/80, 0, 1)
weight  = threat · evasion_gain

beam heading = ±90° from horizontal LOS  (sign from weave sin)
ψ ← ψ + shortest_delta(ψ, ψ_beam) · weight
```

Also adds a vertical weave on climb and, when inbound, a light pull back toward the depot weighted by `(1 − weight)`.

## Kinematic model (not 6-DOF)

```text
v_N = speed_mps · cos(ψ)
v_E = speed_mps · sin(ψ)
v_D = climb_rate_mps

p ← p + v · dt
```

- No mass, thrust, aero, or attitude EOM
- Horizontal speed is **constant** (`speed_mps`); turns change direction only
- Attitude for UE5 is reconstructed from `v` each publish

This keeps the target cheap and deterministic for intercept scoring while still providing curved, evasive paths for seeker / PN stress.

## ROS / telemetry

Published as `flightsim_msgs/TargetState` ([`TargetState.msg`](../../ros2/flightsim_msgs/msg/TargetState.msg)):

| Field | Source |
|-------|--------|
| `id` / `model` | Typically `shahed` / `shahed_136` |
| `position_ned_m` / `velocity_ned_mps` | `TargetState` |
| `attitude_wxyz` | From velocity → body +X (scene builder) |

Scene entity: `type=shahed`, `id=shahed`, `model=shahed_136` — see [UE5 bridge contract](../ue5/ros2_bridge_contract.md). Mesh: [Shahed-136 asset](../../assets/ue5/models/shahed_136/README.md).

## How to exercise

```bash
./scripts/build_debug.sh

# Air-defense scenario (inbound drone + interceptor)
# build/<profile>/apps/air_defense_runner/

./scripts/build_ros2.sh
source ros2/install/debug/setup.bash
ros2 launch flightsim_ros engagement.launch.py
# or air_defense_* launch files
```

Focused tests:

```bash
ctest --preset debug -R engagement --output-on-failure
```

## Current fidelity vs gaps

| Capability | Status |
|------------|--------|
| Deterministic Hermite path + RNG seed | ✅ |
| Inbound / outbound mission bias | ✅ |
| Missile-relative evasion weave | ✅ |
| Constant-speed kinematics | ✅ |
| Full 6-DOF drone aero / autopilot | ❌ (not in scope yet) |
| Multi-target scenarios | ⬜ ([PHASE_PLAN](../PHASE_PLAN.md)) |
| RF / RCS usage of `rcs_m2` | ⬜ metadata only |

## Related docs

- [Missile C++ core](missile_core.md)
- [Phase plan](../PHASE_PLAN.md)
- [UE5 ROS 2 bridge contract](../ue5/ros2_bridge_contract.md)
- [Shahed-136 UE5 model](../../assets/ue5/models/shahed_136/README.md)
