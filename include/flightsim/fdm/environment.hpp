#pragma once

#include "flightsim/fdm/state.hpp"

namespace flightsim {
namespace fdm {

// @req LLR-FDM-006
void update_environment(AircraftState& state) noexcept;

}  // namespace fdm
}  // namespace flightsim
