#pragma once

#include "flightsim/core/types.hpp"

namespace flightsim {
namespace engagement {

struct FinServoLimits {
    float max_deflection_rad{0.35F};
    float max_rate_rps{8.7F};
    float time_constant_sec{0.03F};
};

struct FinActuatorState {
    float position_rad{0.0F};
    float rate_rps{0.0F};
    float command_rad{0.0F};
};

struct IndividualFins {
    static constexpr int kCount = 4;
    FinActuatorState fin[kCount]{};
};

struct VirtualAxisCommand {
    float pitch_rad{0.0F};
    float yaw_rad{0.0F};
    float roll_rad{0.0F};
};

// Cruciform + layout: fin0=top, fin1=right, fin2=bottom, fin3=left.
struct ControlAllocationConfig {
    float pitch_weight[IndividualFins::kCount]{1.0F, 0.0F, -1.0F, 0.0F};
    float yaw_weight[IndividualFins::kCount]{0.0F, -1.0F, 0.0F, 1.0F};
    float roll_weight[IndividualFins::kCount]{0.25F, 0.25F, 0.25F, 0.25F};
    FinServoLimits servo_limits{};
};

IndividualFins allocate_virtual_axes_to_fin_commands(const VirtualAxisCommand& command,
                                                     const ControlAllocationConfig& config) noexcept;

void step_fin_servos(IndividualFins& fins, const IndividualFins& commanded, const FinServoLimits& limits,
                     float dt) noexcept;

VirtualAxisCommand effective_virtual_axes_from_fins(const IndividualFins& fins,
                                                    const ControlAllocationConfig& config) noexcept;

}  // namespace engagement
}  // namespace flightsim
