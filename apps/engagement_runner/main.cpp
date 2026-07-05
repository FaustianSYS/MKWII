#include <cstdio>
#include <cstdlib>

#include "flightsim/engagement/missile/missile_object.hpp"
#include "flightsim/engagement/scenario/scenario.hpp"

int main() {
    flightsim::engagement::ScenarioConfig config{};
    config.dt_sec = 0.01F;
    config.missile = flightsim::engagement::default_missile_attributes();
    config.missile.navigation_gain = 5.0F;
    config.missile.max_speed_mps = 700.0F;
    config.target.speed_mps = 45.0F;
    config.target.rng_seed = 12345U;

    flightsim::engagement::EngagementScenario scenario(config);
    scenario.initialize();
    scenario.run(20000U);

    const auto& state = scenario.state();
    std::printf(
        "steps=%llu intercept=%d miss_distance_m=%.2f thrust_n=%.0f length_m=%.2f seeker_locked=%d\n",
        static_cast<unsigned long long>(state.step_count),
        state.intercept ? 1 : 0,
        state.miss_distance_m,
        state.missile.thrust_n,
        state.missile.attributes.length_m,
        state.missile.seeker_locked ? 1 : 0);

    return state.intercept ? EXIT_SUCCESS : EXIT_FAILURE;
}
