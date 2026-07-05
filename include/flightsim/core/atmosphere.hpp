#include "flightsim/core/constants.hpp"
#include "flightsim/core/types.hpp"

namespace flightsim {
namespace core {

// @req LLR-CORE-023
float isa_density(float altitude_m) noexcept;

// @req LLR-CORE-024
float isa_temperature(float altitude_m) noexcept;

}  // namespace core
}  // namespace flightsim
