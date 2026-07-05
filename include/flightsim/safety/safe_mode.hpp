#pragma once

#include "flightsim/safety/fault_register.hpp"

namespace flightsim {
namespace safety {

enum class SafeModeState : std::uint8_t {
    Normal = 0U,
    Hold = 1U,
    LatchedFault = 2U,
};

// @req LLR-SAF-002
// @mcdc LLR-SAF-002
class SafeModeFsm {
public:
    SafeModeState state() const noexcept { return state_; }

    // @req LLR-SAF-003
    // @mcdc LLR-SAF-003
    void evaluate(FaultRegister& faults) noexcept {
        if (faults.is_set(FaultFlag::NaNDetected) ||
            faults.is_set(FaultFlag::IntegratorDiverged) ||
            faults.is_set(FaultFlag::WatchdogFault)) {
            state_ = SafeModeState::LatchedFault;
            faults.set(FaultFlag::SafeModeActive);
            return;
        }

        if (faults.is_set(FaultFlag::InvalidInput)) {
            state_ = SafeModeState::Hold;
            faults.set(FaultFlag::SafeModeActive);
            return;
        }

        if (state_ == SafeModeState::LatchedFault) {
            faults.set(FaultFlag::SafeModeActive);
            return;
        }

        state_ = SafeModeState::Normal;
        faults.clear(FaultFlag::SafeModeActive);
    }

    void reset() noexcept { state_ = SafeModeState::Normal; }

private:
    SafeModeState state_{SafeModeState::Normal};
};

}  // namespace safety
}  // namespace flightsim
