#pragma once

namespace flightsim {
namespace core {

// @req LLR-CORE-017
constexpr float kGravity = 9.80665F;

// @req LLR-CORE-018
constexpr float kSeaLevelDensity = 1.225F;

// @req LLR-CORE-019
constexpr float kSeaLevelPressure = 101325.0F;

// @req LLR-CORE-020
constexpr float kSeaLevelTemperature = 288.15F;

// @req LLR-CORE-021
constexpr float kGasConstantAir = 287.05F;

// @req LLR-CORE-022
constexpr float kLapseRate = 0.0065F;

}  // namespace core
}  // namespace flightsim
