#pragma once

#include "flightsim/core/types.hpp"

namespace flightsim {
namespace vision {

struct SeekerTrack {
    bool valid{false};
    bool locked{false};
    float range_m{0.0F};
    core::Vec3 los_unit_ned{};
    core::Vec3 los_rate_ned{};
    float confidence{0.0F};
    double stamp_sec{0.0};
};

}  // namespace vision
}  // namespace flightsim
