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
    config.target.speed_mps = 20.0F;

    // Scenario envelope matches the ~500 m London map tile (1 sim meter = 1 map meter).
    config.missile_launch_position_ned_m = core::Vec3{-170.0F, -110.0F, 0.0F};
    config.depot_position_ned_m = core::Vec3{170.0F, 120.0F, 0.0F};
    config.target_start_position_ned_m = core::Vec3{-120.0F, -60.0F, -70.0F};
    config.target_initial_heading_rad = 0.0F;
    config.target_outbound = false;
    config.target_inbound = true;
    config.missile_launch_speed_mps = 80.0F;
    config.use_vision_seeker = false;
    return config;
}

}  // namespace engagement
}  // namespace flightsim
