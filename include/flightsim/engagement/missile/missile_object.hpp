#pragma once

#include "flightsim/core/types.hpp"
#include "flightsim/engagement/missile/missile_autopilot.hpp"
#include "flightsim/engagement/missile/missile_control_allocation.hpp"
#include "flightsim/engagement/missile/missile_eom.hpp"
#include "flightsim/engagement/missile/missile_mass_properties.hpp"

namespace flightsim {
namespace engagement {

// Fin/canard control surface state (body frame: pitch/yaw/roll).
struct ControlSurfaces {
    float fin_pitch_rad{0.0F};
    float fin_yaw_rad{0.0F};
    float fin_roll_rad{0.0F};
};

struct ControlSurfaceLimits {
    float max_deflection_rad{0.35F};
    float max_rate_rps{5.0F};
    float lateral_accel_per_rad{120.0F};
};

// Seeker optical window / FOV envelope.
struct OpticalWindow {
    float fov_azimuth_rad{0.52F};
    float fov_elevation_rad{0.52F};
    float max_track_range_m{15000.0F};
    float acquisition_range_m{8000.0F};
};

struct MissileAero {
    float reference_area_m2{0.091F};
    float drag_coefficient{0.35F};
    float fin_force_per_rad_n{18000.0F};
    float fin_pitch_moment_per_rad_nm{1200.0F};
    float fin_yaw_moment_per_rad_nm{1200.0F};
    float fin_roll_moment_per_rad_nm{300.0F};
    float damping_moment_per_rps_nm{80.0F};
    float static_pitch_moment_per_rad_nm{2500.0F};
    float static_yaw_moment_per_rad_nm{2500.0F};
    float velocity_alignment_moment_nm{4000.0F};
};

struct MissileAttributes {
    float length_m{3.66F};
    float diameter_m{0.34F};
    float mass_kg{152.0F};
    float max_thrust_n{12000.0F};
    float burn_time_sec{8.0F};
    float max_speed_mps{700.0F};
    float max_lateral_accel_mps2{50.0F};
    float navigation_gain{4.0F};
    float kill_radius_m{5.0F};
    ControlSurfaceLimits surface_limits{};
    OpticalWindow optical_window{};
    MissileInertia inertia{};
    MissileAero aero{};
    MissilePropulsionProperties propulsion{};
    ControlAllocationConfig allocation{};
    MissileAutopilotConfig autopilot{};
};

struct MissileObject {
    MissileAttributes attributes{};

    core::Vec3 position_ned_m{};
    core::Vec3 velocity_ned_mps{};
    core::Quaternion attitude{core::Quaternion::identity()};
    core::Vec3 angular_rate_body_rps{};
    ControlSurfaces surfaces{};
    ControlSurfaces commanded_surfaces{};
    IndividualFins fins{};
    IndividualFins commanded_fins{};
    MissileMassProperties mass_properties{};
    MissileAutopilotState autopilot{};
    MissileWrench wrench{};

    float thrust_n{0.0F};
    float flight_time_sec{0.0F};
    bool active{false};
    bool hit{false};
    bool seeker_locked{false};
    float seeker_los_angle_rad{0.0F};
    float seeker_range_m{0.0F};
};

MissileAttributes default_missile_attributes() noexcept;

}  // namespace engagement
}  // namespace flightsim
