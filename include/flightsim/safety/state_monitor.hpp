#pragma once

#include <cmath>

#include "flightsim/core/types.hpp"
#include "flightsim/fdm/state.hpp"
#include "flightsim/safety/fault_register.hpp"

namespace flightsim {
namespace safety {

struct StateLimits {
    float max_speed_mps{300.0F};
    float max_angular_rate_rps{15.0F};
    float max_altitude_m{20000.0F};
};

// @req LLR-SAF-006
// @mcdc LLR-SAF-006
class StateMonitor {
public:
    explicit StateMonitor(StateLimits limits) noexcept : limits_(limits) {}

    // @req LLR-SAF-007
    bool validate(const fdm::AircraftState& state, FaultRegister& faults) noexcept {
        bool valid = true;

        if (!state.position_ned_m.is_valid() || !state.velocity_ned_mps.is_valid() ||
            !state.angular_rate_body_rps.is_valid() || !state.attitude.is_valid()) {
            faults.set(FaultFlag::NaNDetected);
            valid = false;
        }

        const float speed = state.velocity_ned_mps.magnitude();
        if (speed > limits_.max_speed_mps) {
            faults.set(FaultFlag::IntegratorDiverged);
            valid = false;
        }

        const float p_rate = std::fabs(state.angular_rate_body_rps.x);
        const float q_rate = std::fabs(state.angular_rate_body_rps.y);
        const float r_rate = std::fabs(state.angular_rate_body_rps.z);
        if (p_rate > limits_.max_angular_rate_rps ||
            q_rate > limits_.max_angular_rate_rps ||
            r_rate > limits_.max_angular_rate_rps) {
            faults.set(FaultFlag::RateLimitExceeded);
            valid = false;
        }

        if (state.position_ned_m.z < -limits_.max_altitude_m ||
            state.position_ned_m.z > limits_.max_altitude_m) {
            faults.set(FaultFlag::OutOfBounds);
            valid = false;
        }

        return valid;
    }

private:
    StateLimits limits_;
};

}  // namespace safety
}  // namespace flightsim
