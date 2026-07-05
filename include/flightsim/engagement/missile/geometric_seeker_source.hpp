#pragma once

#include "flightsim/engagement/missile/missile_object.hpp"
#include "flightsim/vision/seeker_source.hpp"
#include "flightsim/vision/seeker_track.hpp"

namespace flightsim {
namespace engagement {

vision::SeekerTrack compute_geometric_seeker_track(const MissileObject& missile,
                                                   const core::Vec3& target_position_ned_m) noexcept;

void apply_seeker_track_to_missile(MissileObject& missile, const vision::SeekerTrack& track) noexcept;

class GeometricSeekerSource final : public vision::ISeekerTrackSource {
public:
    GeometricSeekerSource(const MissileObject* missile, const core::Vec3* target_position_ned_m) noexcept;

    bool poll(vision::SeekerTrack& track) noexcept override;

private:
    const MissileObject* missile_{nullptr};
    const core::Vec3* target_position_ned_m_{nullptr};
};

}  // namespace engagement
}  // namespace flightsim
