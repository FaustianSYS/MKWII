#include "flightsim/fdm/aero.hpp"

#include <cmath>

namespace flightsim {
namespace fdm {

namespace {

constexpr float kDegToRad = 0.0174532925F;

float lookup_cl(float alpha_rad) noexcept {
    const float alpha_deg = alpha_rad / kDegToRad;
    const float clamped = core::clamp(alpha_deg, -10.0F, 15.0F);
    return 0.08F * clamped + 0.25F;
}

float lookup_cd(float alpha_rad) noexcept {
    const float alpha_deg = alpha_rad / kDegToRad;
    const float clamped = core::clamp(alpha_deg, -10.0F, 15.0F);
    return 0.02F + (0.001F * clamped * clamped);
}

float lookup_cm(float alpha_rad, float elevator_rad) noexcept {
    const float alpha_deg = alpha_rad / kDegToRad;
    const float clamped = core::clamp(alpha_deg, -10.0F, 15.0F);
    return (-0.02F * clamped) + (-0.5F * elevator_rad);
}

}  // namespace

void compute_aero_forces(const AircraftState& state, const AircraftConfig& config, ForceMoment& wrench) noexcept {
    const core::Mat3 dcm = state.attitude.to_rotation_matrix();
    const core::Vec3 wind_ned{state.environment.wind_ned_mps, 0.0F, 0.0F};
    const core::Vec3 airspeed_ned = state.velocity_ned_mps - wind_ned;
    const core::Vec3 airspeed_body = dcm.multiply(airspeed_ned);

    const float u = airspeed_body.x;
    const float v = airspeed_body.y;
    const float w = airspeed_body.z;
    const float speed = std::sqrt((u * u) + (v * v) + (w * w));

    if (speed < 1.0F) {
        return;
    }

    const float alpha = std::atan2(w, u);
    const float beta = std::asin(core::clamp(v / speed, -1.0F, 1.0F));
    (void)beta;

    const float q_bar = 0.5F * state.environment.density_kgm3 * speed * speed;
    const float elevator_rad = state.controls.elevator * 0.35F;

    const float cl = lookup_cl(alpha);
    const float cd = lookup_cd(alpha);
    const float cm = lookup_cm(alpha, elevator_rad);

    const float lift = q_bar * config.wing_area_m2 * cl;
    const float drag = q_bar * config.wing_area_m2 * cd;
    const float pitch_moment = q_bar * config.wing_area_m2 * config.reference_chord_m * cm;

    const float ca = std::cos(alpha);
    const float sa = std::sin(alpha);

    const core::Vec3 aero_force_body{
        (-drag * ca) + (lift * sa),
        0.0F,
        (-drag * sa) - (lift * ca)};

    wrench.force_body_n = wrench.force_body_n + aero_force_body;
    wrench.moment_body_nm = wrench.moment_body_nm + core::Vec3{0.0F, pitch_moment, 0.0F};
}

}  // namespace fdm
}  // namespace flightsim
