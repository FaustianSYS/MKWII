#include "minimal_test.hpp"

#include "flightsim/safety/fault_register.hpp"
#include "flightsim/safety/input_guard.hpp"
#include "flightsim/safety/safe_mode.hpp"

int run_safety_tests() {
    using flightsim::safety::FaultFlag;
    using flightsim::safety::FaultRegister;
    using flightsim::safety::InputGuard;
    using flightsim::safety::SafeModeFsm;
    using flightsim::safety::SafeModeState;

    {
        FaultRegister faults{};
        faults.set(FaultFlag::InvalidInput);
        REQUIRE(faults.is_set(FaultFlag::InvalidInput));
        faults.clear(FaultFlag::InvalidInput);
        REQUIRE_FALSE(faults.any());
    }

    {
        FaultRegister faults{};
        SafeModeFsm fsm{};
        faults.set(FaultFlag::NaNDetected);
        fsm.evaluate(faults);
        REQUIRE(fsm.state() == SafeModeState::LatchedFault);
        REQUIRE(faults.is_set(FaultFlag::SafeModeActive));
    }

    {
        FaultRegister faults{};
        InputGuard guard{-1.0F, 1.0F, 100.0F};
        const auto result = guard.apply(2.0F, 0.0F, 0.01F, faults);
        REQUIRE(result.ok());
        REQUIRE_APPROX(result.value, 1.0F, 0.001F);
        REQUIRE(faults.is_set(FaultFlag::InvalidInput));
    }

    return 0;
}
