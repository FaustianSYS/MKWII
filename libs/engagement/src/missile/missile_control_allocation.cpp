#include "flightsim/engagement/missile/missile_control_allocation.hpp"

namespace flightsim {
namespace engagement {

IndividualFins allocate_virtual_axes_to_fin_commands(const VirtualAxisCommand& command,
                                                     const ControlAllocationConfig& config) noexcept {
    IndividualFins commanded{};
    const float max_deflection = config.servo_limits.max_deflection_rad;

    for (int i = 0; i < IndividualFins::kCount; ++i) {
        const float fin_cmd = (config.pitch_weight[i] * command.pitch_rad) +
                              (config.yaw_weight[i] * command.yaw_rad) +
                              (config.roll_weight[i] * command.roll_rad);
        commanded.fin[i].command_rad = core::clamp(fin_cmd, -max_deflection, max_deflection);
    }

    return commanded;
}

void step_fin_servos(IndividualFins& fins, const IndividualFins& commanded, const FinServoLimits& limits,
                     float dt) noexcept {
    if (dt <= 0.0F) {
        return;
    }

    for (int i = 0; i < IndividualFins::kCount; ++i) {
        FinActuatorState& actuator = fins.fin[i];
        const float cmd =
            core::clamp(commanded.fin[i].command_rad, -limits.max_deflection_rad, limits.max_deflection_rad);

        float desired_rate = (cmd - actuator.position_rad) / limits.time_constant_sec;
        desired_rate = core::clamp(desired_rate, -limits.max_rate_rps, limits.max_rate_rps);

        actuator.rate_rps = desired_rate;
        actuator.position_rad += actuator.rate_rps * dt;
        actuator.position_rad =
            core::clamp(actuator.position_rad, -limits.max_deflection_rad, limits.max_deflection_rad);
        actuator.command_rad = cmd;
    }
}

VirtualAxisCommand effective_virtual_axes_from_fins(const IndividualFins& fins,
                                                    const ControlAllocationConfig& config) noexcept {
    VirtualAxisCommand effective{};

    for (int i = 0; i < IndividualFins::kCount; ++i) {
        const float position = fins.fin[i].position_rad;
        effective.pitch_rad += config.pitch_weight[i] * position;
        effective.yaw_rad += config.yaw_weight[i] * position;
        effective.roll_rad += config.roll_weight[i] * position;
    }

    return effective;
}

}  // namespace engagement
}  // namespace flightsim
