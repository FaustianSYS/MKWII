#include "flightsim/core/atmosphere.hpp"

#include <cmath>

namespace flightsim {
namespace core {

float isa_density(float altitude_m) noexcept {
    const float h = clamp(altitude_m, -1000.0F, 20000.0F);
    const float temp = isa_temperature(h);
    const float pressure = kSeaLevelPressure * std::pow(temp / kSeaLevelTemperature, kGravity / (kGasConstantAir * kLapseRate));
    return pressure / (kGasConstantAir * temp);
}

float isa_temperature(float altitude_m) noexcept {
    const float h = clamp(altitude_m, -1000.0F, 11000.0F);
    return kSeaLevelTemperature - (kLapseRate * h);
}

}  // namespace core
}  // namespace flightsim
