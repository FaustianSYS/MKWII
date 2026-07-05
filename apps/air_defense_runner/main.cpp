#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "flightsim/engagement/scenario/air_defense_scenario.hpp"
#include "flightsim/engagement/scenario/scenario.hpp"

int main(int argc, char* argv[]) {
    bool use_vision = false;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--vision") == 0) {
            use_vision = true;
        }
    }

    flightsim::engagement::ScenarioConfig config = flightsim::engagement::default_air_defense_config();
    config.use_vision_seeker = use_vision;

    flightsim::engagement::EngagementScenario scenario(config);
    scenario.initialize();
    scenario.run(35000U);

    const auto& state = scenario.state();
    std::printf(
        "air_defense steps=%llu intercept=%d miss_distance_m=%.2f vision=%d\n",
        static_cast<unsigned long long>(state.step_count),
        state.intercept ? 1 : 0,
        state.miss_distance_m,
        use_vision ? 1 : 0);

    return state.intercept ? EXIT_SUCCESS : EXIT_FAILURE;
}
