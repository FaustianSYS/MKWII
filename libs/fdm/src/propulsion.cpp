#include "flightsim/fdm/propulsion.hpp"

#include <cmath>

#include "flightsim/core/atmosphere.hpp"

namespace flightsim {
namespace fdm {

void compute_propulsion_forces(const AircraftState& state, const AircraftConfig& config, ForceMoment& wrench) noexcept {
    const float altitude_m = -state.position_ned_m.z;
    const float density_ratio = core::isa_density(altitude_m) / core::kSeaLevelDensity;
    const float throttle = core::clamp(state.controls.throttle, 0.0F, 1.0F);
    const float thrust = config.max_thrust_n * throttle * std::sqrt(density_ratio);

    wrench.force_body_n = wrench.force_body_n + core::Vec3{thrust, 0.0F, 0.0F};
}

}  // namespace fdm
}  // namespace flightsim
