#include "flightsim/engagement/missile/missile_object.hpp"

#include <cmath>

namespace flightsim {
namespace engagement {

namespace {

constexpr float kPi = 3.14159265F;

}  // namespace

MissileAttributes default_missile_attributes() noexcept {
    MissileAttributes attrs{};
    attrs.length_m = 3.66F;
    attrs.diameter_m = 0.34F;
    attrs.mass_kg = 152.0F;
    attrs.max_thrust_n = 12000.0F;
    attrs.burn_time_sec = 8.0F;
    attrs.max_speed_mps = 700.0F;
    attrs.max_lateral_accel_mps2 = 60.0F;
    attrs.navigation_gain = 5.5F;
    attrs.kill_radius_m = 5.0F;
    attrs.surface_limits.max_deflection_rad = 0.35F;
    attrs.surface_limits.max_rate_rps = 8.7F;
    attrs.surface_limits.lateral_accel_per_rad = 120.0F;
    attrs.optical_window.fov_azimuth_rad = 0.52F;
    attrs.optical_window.fov_elevation_rad = 0.52F;
    attrs.optical_window.max_track_range_m = 15000.0F;
    attrs.optical_window.acquisition_range_m = 8000.0F;

    const float radius = attrs.diameter_m * 0.5F;
    attrs.inertia.ixx_kgm2 = 5.0F;
    attrs.inertia.iyy_kgm2 = (attrs.mass_kg * attrs.length_m * attrs.length_m) / 12.0F;
    attrs.inertia.izz_kgm2 = attrs.inertia.iyy_kgm2;

    attrs.aero.reference_area_m2 = kPi * radius * radius;
    attrs.aero.drag_coefficient = 0.35F;
    attrs.aero.fin_force_per_rad_n = attrs.surface_limits.lateral_accel_per_rad * attrs.mass_kg;
    attrs.aero.fin_pitch_moment_per_rad_nm = 180.0F;
    attrs.aero.fin_yaw_moment_per_rad_nm = 180.0F;
    attrs.aero.fin_roll_moment_per_rad_nm = 120.0F;
    attrs.aero.damping_moment_per_rps_nm = 120.0F;
    attrs.aero.static_pitch_moment_per_rad_nm = 3000.0F;
    attrs.aero.static_yaw_moment_per_rad_nm = 3000.0F;
    attrs.aero.velocity_alignment_moment_nm = 5000.0F;

    attrs.propulsion.dry_mass_kg = 90.0F;
    attrs.propulsion.propellant_mass_kg = 62.0F;
    attrs.propulsion.burn_time_sec = attrs.burn_time_sec;
    attrs.propulsion.cg_full_body_m = core::Vec3{0.48F * attrs.length_m, 0.0F, 0.0F};
    attrs.propulsion.cg_dry_body_m = core::Vec3{0.55F * attrs.length_m, 0.0F, 0.0F};
    attrs.propulsion.inertia_full.ixx_kgm2 = 5.0F;
    attrs.propulsion.inertia_full.iyy_kgm2 = attrs.inertia.iyy_kgm2;
    attrs.propulsion.inertia_full.izz_kgm2 = attrs.inertia.izz_kgm2;
    attrs.propulsion.inertia_dry.ixx_kgm2 = 4.0F;
    attrs.propulsion.inertia_dry.iyy_kgm2 = (attrs.propulsion.dry_mass_kg * attrs.length_m * attrs.length_m) / 12.0F;
    attrs.propulsion.inertia_dry.izz_kgm2 = attrs.propulsion.inertia_dry.iyy_kgm2;

    attrs.allocation.servo_limits.max_deflection_rad = attrs.surface_limits.max_deflection_rad;
    attrs.allocation.servo_limits.max_rate_rps = attrs.surface_limits.max_rate_rps;
    attrs.allocation.servo_limits.time_constant_sec = 0.03F;

    return attrs;
}

}  // namespace engagement
}  // namespace flightsim
