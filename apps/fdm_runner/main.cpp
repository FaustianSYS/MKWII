#include <cstdio>
#include <cstdlib>

#include "flightsim/fdm/state.hpp"
#include "flightsim/sim/scheduler.hpp"

int main() {
    flightsim::sim::SimConfig config{};
    config.dt_sec = 0.01F;
    config.aircraft.wing_area_m2 = 16.0F;
    config.aircraft.max_thrust_n = 8000.0F;

    flightsim::sim::Scheduler scheduler(config);
    flightsim::sim::SimOutputs outputs{};

    scheduler.initialize(outputs);

    flightsim::fdm::ControlInputs controls{};
    controls.throttle = 0.6F;

    const std::uint64_t steps = 1000U;
    scheduler.run(outputs, controls, steps);

    const float altitude_m = -outputs.state.position_ned_m.z;
    const float speed_mps = outputs.state.velocity_ned_mps.magnitude();

    std::printf("steps=%llu altitude_m=%.2f speed_mps=%.2f faults=0x%08X safe_mode=%u\n",
                static_cast<unsigned long long>(outputs.step_count),
                altitude_m,
                speed_mps,
                outputs.faults.raw(),
                static_cast<unsigned>(outputs.safe_mode));

    return outputs.safe_mode == flightsim::safety::SafeModeState::LatchedFault ? EXIT_FAILURE : EXIT_SUCCESS;
}
