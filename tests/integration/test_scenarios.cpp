#include "minimal_test.hpp"

#include "flightsim/fdm/state.hpp"
#include "flightsim/sim/scheduler.hpp"

int run_integration_tests() {
    using flightsim::fdm::ControlInputs;
    using flightsim::sim::Scheduler;
    using flightsim::sim::SimConfig;
    using flightsim::sim::SimOutputs;

    {
        SimConfig config{};
        config.dt_sec = 0.01F;
        config.aircraft.max_thrust_n = 9000.0F;

        Scheduler scheduler(config);
        SimOutputs outputs{};
        scheduler.initialize(outputs);

        ControlInputs controls{};
        controls.throttle = 0.65F;

        scheduler.run(outputs, controls, 500U);

        const float altitude_m = -outputs.state.position_ned_m.z;
        REQUIRE_APPROX(altitude_m, 1000.0F, 300.0F);
        REQUIRE_FALSE(outputs.faults.is_set(flightsim::safety::FaultFlag::SafeModeActive));
        REQUIRE_FALSE(outputs.faults.is_set(flightsim::safety::FaultFlag::NaNDetected));
    }

    {
        SimConfig config{};
        Scheduler scheduler(config);
        SimOutputs outputs{};
        scheduler.initialize(outputs);

        ControlInputs bad{};
        bad.elevator = 999.0F;

        scheduler.run(outputs, bad, 10U);

        REQUIRE(outputs.faults.is_set(flightsim::safety::FaultFlag::InvalidInput));
    }

    return 0;
}
