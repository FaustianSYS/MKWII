#include "flightsim/engagement/missile/geometric_seeker_source.hpp"

#include "flightsim/engagement/missile/missile_eom.hpp"

#include <cmath>

namespace flightsim {
namespace engagement {

namespace {

core::Vec3 normalize_vec(const core::Vec3& v) noexcept {
    const float mag = v.magnitude();
    if (mag < 1.0e-6F) {
        return core::Vec3{};
    }
    return v * (1.0F / mag);
}

}  // namespace

vision::SeekerTrack compute_geometric_seeker_track(const MissileObject& missile,
                                                   const core::Vec3& target_position_ned_m) noexcept {
    vision::SeekerTrack track{};
    const core::Vec3 relative = target_position_ned_m - missile.position_ned_m;
    track.range_m = relative.magnitude();
    track.los_unit_ned = normalize_vec(relative);

    const core::Vec3 boresight = body_x_ned(missile.attitude);
    if (track.range_m < 1.0e-3F || boresight.magnitude() < 1.0e-6F || track.los_unit_ned.magnitude() < 1.0e-6F) {
        return track;
    }

    const float cos_angle = core::clamp(boresight.dot(track.los_unit_ned), -1.0F, 1.0F);
    const float los_angle = std::acos(cos_angle);
    const float half_fov = missile.attributes.optical_window.fov_azimuth_rad * 0.5F;

    track.valid = true;
    track.confidence = 1.0F;
    track.locked = track.range_m <= missile.attributes.optical_window.acquisition_range_m && los_angle <= half_fov;
    track.los_rate_ned = core::Vec3{};
    return track;
}

void apply_seeker_track_to_missile(MissileObject& missile, const vision::SeekerTrack& track) noexcept {
    if (!track.valid) {
        missile.seeker_locked = false;
        return;
    }

    missile.seeker_locked = track.locked;
    missile.seeker_range_m = track.range_m;

    const core::Vec3 boresight = body_x_ned(missile.attitude);
    if (boresight.magnitude() > 1.0e-6F && track.los_unit_ned.magnitude() > 1.0e-6F) {
        const float cos_angle = core::clamp(boresight.dot(track.los_unit_ned), -1.0F, 1.0F);
        missile.seeker_los_angle_rad = std::acos(cos_angle);
    }
}

GeometricSeekerSource::GeometricSeekerSource(const MissileObject* missile,
                                             const core::Vec3* target_position_ned_m) noexcept
    : missile_(missile), target_position_ned_m_(target_position_ned_m) {}

bool GeometricSeekerSource::poll(vision::SeekerTrack& track) noexcept {
    if (missile_ == nullptr || target_position_ned_m_ == nullptr) {
        return false;
    }
    track = compute_geometric_seeker_track(*missile_, *target_position_ned_m_);
    return track.valid;
}

}  // namespace engagement
}  // namespace flightsim
