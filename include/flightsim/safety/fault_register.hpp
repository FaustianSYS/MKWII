#pragma once

#include <cstdint>

namespace flightsim {
namespace safety {

enum class FaultFlag : std::uint32_t {
    None = 0U,
    InvalidInput = 1U << 0U,
    NaNDetected = 1U << 1U,
    RateLimitExceeded = 1U << 2U,
    IntegratorDiverged = 1U << 3U,
    SafeModeActive = 1U << 4U,
    WatchdogFault = 1U << 5U,
    OutOfBounds = 1U << 6U,
};

// @req LLR-SAF-001
class FaultRegister {
public:
    void set(FaultFlag flag) noexcept {
        flags_ |= static_cast<std::uint32_t>(flag);
    }

    void clear(FaultFlag flag) noexcept {
        flags_ &= ~static_cast<std::uint32_t>(flag);
    }

    bool is_set(FaultFlag flag) const noexcept {
        return (flags_ & static_cast<std::uint32_t>(flag)) != 0U;
    }

    bool any() const noexcept { return flags_ != 0U; }

    std::uint32_t raw() const noexcept { return flags_; }

    void reset() noexcept { flags_ = 0U; }

private:
    std::uint32_t flags_{0U};
};

}  // namespace safety
}  // namespace flightsim
