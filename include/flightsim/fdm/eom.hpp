#pragma once

#include "flightsim/fdm/state.hpp"

namespace flightsim {
namespace fdm {

enum class IntegratorMethod : std::uint8_t {
    SemiImplicitEuler = 0U,
    Rk4 = 1U,
};

// @req LLR-FDM-005
void integrate_eom(AircraftState& state, float dt, IntegratorMethod method) noexcept;

}  // namespace fdm
}  // namespace flightsim
