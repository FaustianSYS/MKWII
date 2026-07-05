#include "minimal_test.hpp"

#include "flightsim/fdm/eom.hpp"
#include "flightsim/fdm/init.hpp"
#include "flightsim/fdm/propulsion.hpp"
#include "flightsim/fdm/state.hpp"

int run_fdm_tests() {
    using flightsim::fdm::AircraftConfig;
    using flightsim::fdm::AircraftState;
    using flightsim::fdm::compute_propulsion_forces;
    using flightsim::fdm::ForceMoment;
    using flightsim::fdm::initialize_state;
    using flightsim::fdm::IntegratorMethod;
    using flightsim::fdm::integrate_eom;

    {
        AircraftState state{};
        AircraftConfig config{};
        initialize_state(state, config);
        state.velocity_ned_mps = flightsim::core::Vec3{};
        state.body_wrench = ForceMoment{};
        state.mass_props.mass_kg = 100.0F;

        integrate_eom(state, 0.1F, IntegratorMethod::SemiImplicitEuler);

        REQUIRE_APPROX(state.velocity_ned_mps.z, flightsim::core::kGravity * 0.1F, 0.01F);
    }

    {
        AircraftState state{};
        AircraftConfig config{};
        config.max_thrust_n = 1000.0F;
        initialize_state(state, config);
        state.controls.throttle = 1.0F;
        state.body_wrench = ForceMoment{};
        compute_propulsion_forces(state, config, state.body_wrench);

        integrate_eom(state, 0.1F, IntegratorMethod::SemiImplicitEuler);

        REQUIRE(state.velocity_ned_mps.x > 0.0F);
    }

    return 0;
}
