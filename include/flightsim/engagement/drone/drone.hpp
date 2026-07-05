#pragma once

#include "flightsim/engagement/target/target.hpp"

namespace flightsim {
namespace engagement {

struct DroneAttributes {
    float wingspan_m{2.5F};
    float length_m{3.5F};
    float rcs_m2{0.05F};
    float max_speed_mps{50.0F};
    float max_climb_mps{4.0F};
};

inline TargetConfig drone_target_config(const DroneAttributes& drone) noexcept {
    TargetConfig config{};
    config.speed_mps = drone.max_speed_mps;
    config.max_climb_rate_mps = drone.max_climb_mps;
    config.max_yaw_rate_rps = 0.18F;
    config.spline_segment_sec = 4.0F;
    config.max_heading_change_rad = 0.42F;
    config.outbound = true;
    config.evade_missile = true;
    config.evasion_range_m = 4800.0F;
    config.evasion_gain = 0.48F;
    config.evasion_weave_period_sec = 8.0F;
    config.curved_initial_spline = true;
    config.initial_spline_curve_rad = 0.48F;
    config.initial_spline_duration_scale = 2.4F;
    return config;
}

inline DroneAttributes default_drone_attributes() noexcept {
    return DroneAttributes{};
}

}  // namespace engagement
}  // namespace flightsim
