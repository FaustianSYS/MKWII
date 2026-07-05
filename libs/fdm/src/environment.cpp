#include "flightsim/fdm/environment.hpp"

#include "flightsim/core/atmosphere.hpp"

namespace flightsim {
namespace fdm {

void update_environment(AircraftState& state) noexcept {
    const float altitude_m = -state.position_ned_m.z;
    state.environment.density_kgm3 = core::isa_density(altitude_m);
    state.environment.wind_ned_mps = 0.0F;
}

}  // namespace fdm
}  // namespace flightsim
