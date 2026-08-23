#include "flightsim/engagement/missile/missile_autopilot.hpp"

#include <cmath>

namespace flightsim {
namespace engagement {

namespace {

void update_integral_axis(float& integral, float& rate_error, float gain_i, float lim_int, float dt, bool sat_pos,
                          bool sat_neg) noexcept {
    // PX4 anti-windup: freeze integral in the saturated direction.
    if (sat_pos) {
        rate_error = std::min(rate_error, 0.0F);
    }
    if (sat_neg) {
        rate_error = std::max(rate_error, 0.0F);
    }

    // PX4 i_factor: reduce I gain for large rate errors (~400 deg ≈ 7 rad).
    constexpr float kIFactorRefRad = 7.0F;
    float i_factor = rate_error / kIFactorRefRad;
    i_factor = std::max(0.0F, 1.0F - (i_factor * i_factor));

    const float next = integral + (i_factor * gain_i * rate_error * dt);
    if (std::isfinite(next)) {
        integral = core::clamp(next, -lim_int, lim_int);
    }
}

}  // namespace

void reset_missile_autopilot(MissileAutopilotState& state) noexcept {
    state.rate_integral = core::Vec3{};
    state.prev_rate_body_rps = core::Vec3{};
    state.has_prev_rate = false;
    state.sat_pos[0] = state.sat_pos[1] = state.sat_pos[2] = false;
    state.sat_neg[0] = state.sat_neg[1] = state.sat_neg[2] = false;
}

core::Vec3 accel_command_to_rate_setpoint(const core::Vec3& accel_cmd_body_mps2,
                                          const core::Vec3& velocity_body_mps,
                                          const MissileAutopilotConfig& config) noexcept {
    const float speed = std::max(std::fabs(velocity_body_mps.x), config.min_speed_mps);
    const float scale = config.accel_to_rate_gain / speed;

    // Skid-to-turn: lateral accel about body Y/Z → pitch/yaw rate setpoints.
    // Roll rate damps to zero (bank-to-turn not used).
    core::Vec3 rate_sp{};
    rate_sp.x = 0.0F;
    rate_sp.y = core::clamp((-accel_cmd_body_mps2.z) * scale, -config.max_rate_rps, config.max_rate_rps);
    rate_sp.z = core::clamp(accel_cmd_body_mps2.y * scale, -config.max_rate_rps, config.max_rate_rps);
    return rate_sp;
}

VirtualAxisCommand update_missile_rate_autopilot(MissileAutopilotState& state,
                                                 const core::Vec3& rate_body_rps,
                                                 const core::Vec3& rate_setpoint_rps, float dt,
                                                 const MissileAutopilotConfig& config) noexcept {
    VirtualAxisCommand command{};
    if (!config.enabled || dt <= 0.0F) {
        return command;
    }

    core::Vec3 angular_accel{};
    if (state.has_prev_rate && dt > 1.0e-6F) {
        angular_accel = (rate_body_rps - state.prev_rate_body_rps) * (1.0F / dt);
    }
    state.prev_rate_body_rps = rate_body_rps;
    state.has_prev_rate = true;

    const core::Vec3 rate_error = rate_setpoint_rps - rate_body_rps;
    float err_x = rate_error.x;
    float err_y = rate_error.y;
    float err_z = rate_error.z;

    // PX4: u = P*e + I - D*ang_accel + FF*rate_sp  (here u is virtual fin rad)
    const float u_x = (config.roll.p * err_x) + state.rate_integral.x - (config.roll.d * angular_accel.x) +
                      (config.roll.ff * rate_setpoint_rps.x);
    const float u_y = (config.pitch.p * err_y) + state.rate_integral.y - (config.pitch.d * angular_accel.y) +
                      (config.pitch.ff * rate_setpoint_rps.y);
    const float u_z = (config.yaw.p * err_z) + state.rate_integral.z - (config.yaw.d * angular_accel.z) +
                      (config.yaw.ff * rate_setpoint_rps.z);

    update_integral_axis(state.rate_integral.x, err_x, config.roll.i, config.roll.lim_int, dt, state.sat_pos[0],
                         state.sat_neg[0]);
    update_integral_axis(state.rate_integral.y, err_y, config.pitch.i, config.pitch.lim_int, dt, state.sat_pos[1],
                         state.sat_neg[1]);
    update_integral_axis(state.rate_integral.z, err_z, config.yaw.i, config.yaw.lim_int, dt, state.sat_pos[2],
                         state.sat_neg[2]);

    const float max_def = config.max_deflection_rad;
    command.roll_rad = core::clamp(u_x, -max_def, max_def);
    command.pitch_rad = core::clamp(u_y, -max_def, max_def);
    command.yaw_rad = core::clamp(u_z, -max_def, max_def);

    state.sat_pos[0] = command.roll_rad >= max_def - 1.0e-6F;
    state.sat_neg[0] = command.roll_rad <= -max_def + 1.0e-6F;
    state.sat_pos[1] = command.pitch_rad >= max_def - 1.0e-6F;
    state.sat_neg[1] = command.pitch_rad <= -max_def + 1.0e-6F;
    state.sat_pos[2] = command.yaw_rad >= max_def - 1.0e-6F;
    state.sat_neg[2] = command.yaw_rad <= -max_def + 1.0e-6F;

    return command;
}

}  // namespace engagement
}  // namespace flightsim
