#pragma once

#include "flightsim/core/types.hpp"
#include "flightsim/engagement/common/rng.hpp"

namespace flightsim {
namespace engagement {

struct TargetState {
    core::Vec3 position_ned_m{};
    core::Vec3 velocity_ned_mps{};
};

struct TargetConfig {
    float speed_mps{40.0F};
    float max_yaw_rate_rps{0.15F};
    float min_turn_radius_m{0.0F};  // >0 enforces ω ≤ V/R (realistic coordinated turn)
    float max_climb_rate_mps{5.0F};
    float max_climb_accel_mps2{1.5F};
    float spline_segment_sec{4.0F};
    float spline_segment_jitter{0.45F};  // randomize duration ± fraction
    float max_heading_change_rad{0.55F};
    float spline_mission_blend{0.45F};  // inbound/outbound pull vs random (0 = pure random)
    bool smooth_path_only{false};  // constant-rate arcs, no jerky hermite endpoints
    std::uint32_t rng_seed{42U};
    bool outbound{true};
    bool inbound{false};
    bool evade_missile{false};
    float evasion_range_m{4800.0F};
    float evasion_gain{0.72F};
    float evasion_weave_period_sec{3.0F};
    bool curved_initial_spline{false};
    float initial_spline_curve_rad{0.55F};
    float initial_spline_duration_scale{1.45F};
    bool constrain_play_area{false};
    float play_area_half_m{500.0F};
    float play_area_margin_m{60.0F};
    core::Vec3 outbound_reference_ned_m{};
    core::Vec3 inbound_reference_ned_m{};
};

struct MissileThreat {
    core::Vec3 position_ned_m{};
    core::Vec3 velocity_ned_mps{};
    bool active{false};
};

float outbound_heading_rad(const core::Vec3& reference_ned_m, const core::Vec3& position_ned_m) noexcept;
float inbound_heading_rad(const core::Vec3& aim_ned_m, const core::Vec3& position_ned_m) noexcept;

struct TargetSplineSegment {
    float duration_sec{4.0F};
    float start_heading_rad{0.0F};
    float end_heading_rad{0.0F};
    float start_climb_mps{0.0F};
    float end_climb_mps{0.0F};
    float start_heading_rate_rps{0.0F};
    float end_heading_rate_rps{0.0F};
    float start_climb_accel_mps2{0.0F};
    float end_climb_accel_mps2{0.0F};
};

struct TargetRuntime {
    float segment_elapsed_sec{0.0F};
    float heading_rad{0.0F};
    float heading_rate_rps{0.0F};
    float climb_rate_mps{0.0F};
    float climb_accel_mps2{0.0F};
    float evasion_phase_sec{0.0F};
    TargetSplineSegment segment{};
    DeterministicRng rng{42U};
};

void initialize_target(TargetState& state, TargetConfig& config, TargetRuntime& runtime,
                       const core::Vec3& start_position_ned_m, float initial_heading_rad) noexcept;

void step_target(TargetState& state, const TargetConfig& config, TargetRuntime& runtime, float dt,
                 const MissileThreat* missile_threat = nullptr) noexcept;

}  // namespace engagement
}  // namespace flightsim
