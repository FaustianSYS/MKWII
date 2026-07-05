#pragma once

#include <cmath>

#include "flightsim/core/types.hpp"
#include "flightsim/safety/fault_register.hpp"

namespace flightsim {
namespace safety {

struct InputLimits {
    float min_value{0.0F};
    float max_value{0.0F};
    float max_rate_per_sec{0.0F};
};

// @req LLR-SAF-004
// @mcdc LLR-SAF-004
class InputGuard {
public:
    InputGuard(float min_val, float max_val, float max_rate) noexcept
        : limits_{min_val, max_val, max_rate} {}

    // @req LLR-SAF-005
    // @mcdc LLR-SAF-005
    core::Result<float> apply(float commanded, float previous, float dt, FaultRegister& faults) noexcept {
        if (!core::is_finite(commanded) || !core::is_finite(previous) || dt <= 0.0F) {
            faults.set(FaultFlag::InvalidInput);
            return core::Result<float>::error(core::ResultCode::InvalidInput);
        }

        float saturated = core::clamp(commanded, limits_.min_value, limits_.max_value);
        if (saturated != commanded) {
            faults.set(FaultFlag::InvalidInput);
        }

        const float delta = saturated - previous;
        const float max_delta = limits_.max_rate_per_sec * dt;
        if (std::fabs(delta) > max_delta) {
            saturated = previous + core::clamp(delta, -max_delta, max_delta);
        }

        return core::Result<float>::ok(saturated);
    }

private:
    InputLimits limits_;
};

}  // namespace safety
}  // namespace flightsim
