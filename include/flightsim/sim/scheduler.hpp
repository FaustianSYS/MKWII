#pragma once

#include <cstdint>

#include "flightsim/fdm/eom.hpp"
#include "flightsim/fdm/init.hpp"
#include "flightsim/fdm/state.hpp"
#include "flightsim/safety/fault_register.hpp"
#include "flightsim/safety/input_guard.hpp"
#include "flightsim/safety/safe_mode.hpp"
#include "flightsim/safety/state_monitor.hpp"

namespace flightsim {
namespace sim {

struct SimConfig {
    fdm::AircraftConfig aircraft{};
    float dt_sec{0.01F};
    fdm::IntegratorMethod integrator{fdm::IntegratorMethod::SemiImplicitEuler};
    safety::StateLimits state_limits{};
};

struct SimOutputs {
    fdm::AircraftState state{};
    safety::FaultRegister faults{};
    safety::SafeModeState safe_mode{safety::SafeModeState::Normal};
    std::uint64_t step_count{0U};
};

    // @req LLR-SIM-001
    class Scheduler {
public:
    explicit Scheduler(SimConfig config) noexcept;

    // @req LLR-SIM-002
    void initialize(SimOutputs& outputs) noexcept;

    // @req LLR-SIM-003
    void step(SimOutputs& outputs, const fdm::ControlInputs& commanded) noexcept;

    // @req LLR-SIM-004
    void run(SimOutputs& outputs, const fdm::ControlInputs& commanded, std::uint64_t steps) noexcept;

private:
    SimConfig config_;
    safety::SafeModeFsm safe_mode_fsm_{};
    safety::StateMonitor state_monitor_;
    safety::InputGuard elevator_guard_{-1.0F, 1.0F, 2.0F};
    safety::InputGuard aileron_guard_{-1.0F, 1.0F, 2.0F};
    safety::InputGuard rudder_guard_{-1.0F, 1.0F, 2.0F};
    safety::InputGuard throttle_guard_{0.0F, 1.0F, 1.0F};
    fdm::AircraftState last_valid_state_{};
    bool has_last_valid_{false};
    std::uint64_t expected_step_{0U};
};

}  // namespace sim
}  // namespace flightsim
