#pragma once

#include "flightsim/fdm/config.hpp"
#include "flightsim/fdm/state.hpp"

namespace flightsim {
namespace fdm {

// @req LLR-FDM-002
void initialize_state(AircraftState& state, const AircraftConfig& config) noexcept;

}  // namespace fdm
}  // namespace flightsim
