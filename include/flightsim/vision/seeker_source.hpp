#pragma once

#include "flightsim/vision/seeker_track.hpp"

namespace flightsim {
namespace vision {

class ISeekerTrackSource {
public:
    virtual ~ISeekerTrackSource() = default;

    virtual bool poll(SeekerTrack& track) noexcept = 0;
    virtual bool is_live() const noexcept { return true; }
};

}  // namespace vision
}  // namespace flightsim
