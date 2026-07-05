#pragma once

#include "flightsim/engagement/missile/missile_object.hpp"
#include "flightsim/vision/seeker_track.hpp"

namespace flightsim {
namespace engagement {

void initialize_missile(MissileObject& missile, const MissileAttributes& attributes,
                        const core::Vec3& launch_position_ned_m, const core::Vec3& launch_velocity_ned_mps) noexcept;

void step_missile(MissileObject& missile, const core::Vec3& target_position_ned_m,
                  const core::Vec3& target_velocity_ned_mps, float dt,
                  const vision::SeekerTrack* seeker_track = nullptr) noexcept;

float range_to_target(const MissileObject& missile, const core::Vec3& target_position_ned_m) noexcept;

core::Vec3 missile_nose_position(const MissileObject& missile) noexcept;

}  // namespace engagement
}  // namespace flightsim
