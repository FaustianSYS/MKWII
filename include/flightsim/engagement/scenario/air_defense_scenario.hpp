#pragma once

#include "flightsim/engagement/drone/drone.hpp"
#include "flightsim/engagement/missile/missile_object.hpp"
#include "flightsim/engagement/scenario/scenario.hpp"

namespace flightsim {
namespace engagement {

inline ScenarioConfig default_air_defense_config() noexcept {
    ScenarioConfig config{};
    config.dt_sec = 0.01F;
    config.missile = default_missile_attributes();
    config.missile.navigation_gain = 6.5F;
    config.missile.max_lateral_accel_mps2 = 60.0F;
    config.missile.optical_window.acquisition_range_m = 1200.0F;

    const DroneAttributes drone = default_drone_attributes();
    config.target = drone_target_config(drone);
    config.target.rng_seed = 2024U;
    config.target.speed_mps = drone.max_speed_mps;
    config.target.min_turn_radius_m = drone.min_turn_radius_m;  // 450 m realistic banked turn
    config.target.smooth_path_only = true;
    config.target.max_yaw_rate_rps = drone.max_speed_mps / drone.min_turn_radius_m;
    config.target.spline_segment_sec = 8.0F;
    config.target.spline_segment_jitter = 0.20F;
    config.target.constrain_play_area = true;
    config.target.play_area_half_m = 500.0F;
    config.target.play_area_margin_m = 100.0F;

    // 1 km × 1 km play area (±500 m N/E). Spawns randomized each initialize().
    config.play_area_half_m = 500.0F;
    config.randomize_initial_positions = true;
    config.spawn_rng_seed = 0U;
    config.missile_launch_position_ned_m = core::Vec3{-170.0F, -110.0F, 0.0F};
    config.depot_position_ned_m = core::Vec3{170.0F, 120.0F, 0.0F};
    config.target_start_position_ned_m = core::Vec3{-120.0F, -60.0F, -70.0F};
    config.target_initial_heading_rad = 0.0F;
    // Free randomized spline (not depot-bound inbound).
    config.target_outbound = false;
    config.target_inbound = false;
    config.missile_launch_speed_mps = 80.0F;
    config.missile.navigation_gain = 7.0F;
    config.missile.max_lateral_accel_mps2 = 70.0F;
    config.use_vision_seeker = false;
    return config;
}

}  // namespace engagement
}  // namespace flightsim
