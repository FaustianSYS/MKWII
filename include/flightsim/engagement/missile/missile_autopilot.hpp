#pragma once

#include "flightsim/core/types.hpp"
#include "flightsim/engagement/missile/missile_control_allocation.hpp"

namespace flightsim {
namespace engagement {

// Per-axis gains modeled after PX4 RateControl (P / I / D on angular accel / FF).
struct AxisRateGains {
    float p{0.0F};
    float i{0.0F};
    float d{0.0F};
    float ff{0.0F};
    float lim_int{0.0F};
};

struct MissileAutopilotConfig {
    bool enabled{true};
    // Gains produce virtual fin deflection (rad) directly — PX4 RateControl topology.
    AxisRateGains roll{0.08F, 0.02F, 0.002F, 0.01F, 0.05F};
    AxisRateGains pitch{0.12F, 0.03F, 0.003F, 0.02F, 0.08F};
    AxisRateGains yaw{0.12F, 0.03F, 0.003F, 0.02F, 0.08F};
    float max_rate_rps{4.0F};
    // Maps lateral body accel command → body rate setpoint: ω ≈ a / (V * k).
    float accel_to_rate_gain{1.0F};
    float min_speed_mps{20.0F};
    // Blend: virtual = accel_ff + rate_loop_blend * rate_pid  (PX4 cascade with FF).
    float rate_loop_blend{0.35F};
    float max_deflection_rad{0.35F};
};

struct MissileAutopilotState {
    core::Vec3 rate_integral{};
    core::Vec3 prev_rate_body_rps{};
    bool has_prev_rate{false};
    bool sat_pos[3]{false, false, false};
    bool sat_neg[3]{false, false, false};
};

void reset_missile_autopilot(MissileAutopilotState& state) noexcept;

// Outer map: commanded body accel → body rate setpoint (PX4-like cascade).
core::Vec3 accel_command_to_rate_setpoint(const core::Vec3& accel_cmd_body_mps2,
                                          const core::Vec3& velocity_body_mps,
                                          const MissileAutopilotConfig& config) noexcept;

// Inner PX4-style rate PID: torque_cmd then scaled to virtual fin axes.
VirtualAxisCommand update_missile_rate_autopilot(MissileAutopilotState& state,
                                                 const core::Vec3& rate_body_rps,
                                                 const core::Vec3& rate_setpoint_rps,
                                                 float dt,
                                                 const MissileAutopilotConfig& config) noexcept;

}  // namespace engagement
}  // namespace flightsim
