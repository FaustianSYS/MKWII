#pragma once

#include "flightsim/engagement/target/target.hpp"

namespace flightsim {
namespace engagement {

struct DroneAttributes {
    float wingspan_m{2.5F};
    float length_m{3.5F};
    float rcs_m2{0.05F};
    float max_speed_mps{50.0F};
    float max_climb_mps{5.0F};
    // Coordinated-turn radius at cruise (~25–30° bank at 50 m/s ≈ 400–500 m).
    float min_turn_radius_m{450.0F};
};

inline TargetConfig drone_target_config(const DroneAttributes& drone) noexcept {
    TargetConfig config{};
    config.speed_mps = drone.max_speed_mps;
    config.max_climb_rate_mps = drone.max_climb_mps;
    config.max_climb_accel_mps2 = 1.0F;
    config.min_turn_radius_m = drone.min_turn_radius_m;
    config.smooth_path_only = true;
    // Cap yaw from radius: ω = V/R (e.g. 50/450 ≈ 0.111 rad/s).
    config.max_yaw_rate_rps = drone.max_speed_mps / (drone.min_turn_radius_m > 1.0F ? drone.min_turn_radius_m : 1.0F);
    config.spline_segment_sec = 8.0F;
    config.spline_segment_jitter = 0.20F;
    // Max heading per segment ≈ ω * duration (filled in by splice from radius).
    config.max_heading_change_rad = 1.2F;
    config.spline_mission_blend = 0.0F;
    config.outbound = false;
    config.inbound = false;
    config.evade_missile = true;
    config.evasion_range_m = 700.0F;
    config.evasion_gain = 0.18F;  // gentle nudge only — still radius-limited
    config.evasion_weave_period_sec = 10.0F;
    config.curved_initial_spline = true;
    config.initial_spline_curve_rad = 0.18F;
    config.initial_spline_duration_scale = 1.5F;
    config.constrain_play_area = true;
    config.play_area_half_m = 500.0F;
    config.play_area_margin_m = 100.0F;
    return config;
}

inline DroneAttributes default_drone_attributes() noexcept {
    return DroneAttributes{};
}

}  // namespace engagement
}  // namespace flightsim
