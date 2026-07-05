#include "flightsim/sim/scheduler.hpp"

#include "flightsim/fdm/aero.hpp"
#include "flightsim/fdm/environment.hpp"
#include "flightsim/fdm/propulsion.hpp"

namespace flightsim {
namespace sim {

Scheduler::Scheduler(SimConfig config) noexcept
    : config_(config), state_monitor_(config.state_limits) {}

void Scheduler::initialize(SimOutputs& outputs) noexcept {
    outputs.faults.reset();
    safe_mode_fsm_.reset();
    fdm::initialize_state(outputs.state, config_.aircraft);
    last_valid_state_ = outputs.state;
    has_last_valid_ = true;
    expected_step_ = 0U;
    outputs.step_count = 0U;
    outputs.safe_mode = safe_mode_fsm_.state();
}

void Scheduler::step(SimOutputs& outputs, const fdm::ControlInputs& commanded) noexcept {
    if (outputs.step_count != expected_step_) {
        outputs.faults.set(safety::FaultFlag::WatchdogFault);
    }
    ++expected_step_;

    if (safe_mode_fsm_.state() != safety::SafeModeState::LatchedFault) {
        outputs.faults.clear(safety::FaultFlag::InvalidInput);
        outputs.faults.clear(safety::FaultFlag::RateLimitExceeded);
        outputs.faults.clear(safety::FaultFlag::OutOfBounds);
    }

    fdm::ControlInputs guarded = commanded;
    const float dt = config_.dt_sec;

    const auto elev = elevator_guard_.apply(commanded.elevator, outputs.state.controls.elevator, dt, outputs.faults);
    if (elev.ok()) {
        guarded.elevator = elev.value;
    }
    const auto ail = aileron_guard_.apply(commanded.aileron, outputs.state.controls.aileron, dt, outputs.faults);
    if (ail.ok()) {
        guarded.aileron = ail.value;
    }
    const auto rud = rudder_guard_.apply(commanded.rudder, outputs.state.controls.rudder, dt, outputs.faults);
    if (rud.ok()) {
        guarded.rudder = rud.value;
    }
    const auto thr = throttle_guard_.apply(commanded.throttle, outputs.state.controls.throttle, dt, outputs.faults);
    if (thr.ok()) {
        guarded.throttle = thr.value;
    }

    outputs.state.controls = guarded;

    safe_mode_fsm_.evaluate(outputs.faults);
    outputs.safe_mode = safe_mode_fsm_.state();

    if (outputs.safe_mode == safety::SafeModeState::Hold ||
        outputs.safe_mode == safety::SafeModeState::LatchedFault) {
        if (has_last_valid_) {
            outputs.state = last_valid_state_;
        }
        ++outputs.step_count;
        return;
    }

    fdm::update_environment(outputs.state);
    outputs.state.body_wrench = fdm::ForceMoment{};
    fdm::compute_aero_forces(outputs.state, config_.aircraft, outputs.state.body_wrench);
    fdm::compute_propulsion_forces(outputs.state, config_.aircraft, outputs.state.body_wrench);
    fdm::integrate_eom(outputs.state, dt, config_.integrator);

    if (!state_monitor_.validate(outputs.state, outputs.faults)) {
        safe_mode_fsm_.evaluate(outputs.faults);
        outputs.safe_mode = safe_mode_fsm_.state();
        if (has_last_valid_) {
            outputs.state = last_valid_state_;
        }
    } else {
        last_valid_state_ = outputs.state;
        has_last_valid_ = true;
        outputs.faults.clear(safety::FaultFlag::RateLimitExceeded);
    }

    ++outputs.step_count;
}

void Scheduler::run(SimOutputs& outputs, const fdm::ControlInputs& commanded, std::uint64_t steps) noexcept {
    for (std::uint64_t i = 0U; i < steps; ++i) {
        step(outputs, commanded);
    }
}

}  // namespace sim
}  // namespace flightsim
