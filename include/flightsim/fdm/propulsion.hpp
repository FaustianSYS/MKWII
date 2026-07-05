#pragma once

#include "flightsim/fdm/config.hpp"
#include "flightsim/fdm/state.hpp"

namespace flightsim {
namespace fdm {

// @req LLR-FDM-004
void compute_propulsion_forces(const AircraftState& state, const AircraftConfig& config, ForceMoment& wrench) noexcept;

}  // namespace fdm
}  // namespace flightsim
