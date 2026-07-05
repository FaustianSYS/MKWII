#pragma once

#include <cstdint>

namespace flightsim {
namespace engagement {

// Deterministic PRNG for reproducible target maneuvers (non-safety partition).
class DeterministicRng {
public:
    explicit DeterministicRng(std::uint32_t seed) noexcept : state_(seed != 0U ? seed : 1U) {}

    std::uint32_t next_u32() noexcept {
        state_ ^= state_ << 13U;
        state_ ^= state_ >> 17U;
        state_ ^= state_ << 5U;
        return state_;
    }

    float uniform(float min_val, float max_val) noexcept {
        const float unit = static_cast<float>(next_u32()) / static_cast<float>(0xFFFFFFFFU);
        return min_val + (unit * (max_val - min_val));
    }

private:
    std::uint32_t state_;
};

}  // namespace engagement
}  // namespace flightsim
