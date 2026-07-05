#include "flightsim/vision/camera_model.hpp"

#include <cmath>

namespace flightsim {
namespace vision {

void CameraModel::pixel_to_bearing_rad(float pixel_x, float pixel_y, float& azimuth_rad,
                                       float& elevation_rad) const noexcept {
    if (width_px <= 0 || height_px <= 0) {
        azimuth_rad = 0.0F;
        elevation_rad = 0.0F;
        return;
    }

    const float cx = static_cast<float>(width_px) * 0.5F;
    const float cy = static_cast<float>(height_px) * 0.5F;
    const float nx = (pixel_x - cx) / cx;
    const float ny = (pixel_y - cy) / cy;

    azimuth_rad = nx * (fov_azimuth_rad * 0.5F);
    elevation_rad = -ny * (fov_elevation_rad * 0.5F);
}

bool CameraModel::bearing_rad_to_pixel(float azimuth_rad, float elevation_rad, float& pixel_x,
                                       float& pixel_y) const noexcept {
    if (width_px <= 0 || height_px <= 0) {
        return false;
    }

    const float half_az = fov_azimuth_rad * 0.5F;
    const float half_el = fov_elevation_rad * 0.5F;
    if (std::abs(azimuth_rad) > half_az || std::abs(elevation_rad) > half_el) {
        return false;
    }

    const float cx = static_cast<float>(width_px) * 0.5F;
    const float cy = static_cast<float>(height_px) * 0.5F;
    const float nx = azimuth_rad / half_az;
    const float ny = -elevation_rad / half_el;
    pixel_x = (nx * cx) + cx;
    pixel_y = (ny * cy) + cy;
    return true;
}

bool CameraModel::target_to_pixel(const core::Vec3& missile_pos_ned_m, const core::Quaternion& missile_attitude,
                                  const core::Vec3& target_pos_ned_m, float& pixel_x, float& pixel_y) const noexcept {
    const core::Vec3 delta = target_pos_ned_m - missile_pos_ned_m;
    const float range = delta.magnitude();
    if (range < 1.0e-3F) {
        return false;
    }

    const core::Vec3 los_ned = delta * (1.0F / range);
    const core::Quaternion attitude_inv{missile_attitude.w, -missile_attitude.x, -missile_attitude.y,
                                        -missile_attitude.z};
    const core::Vec3 los_body = attitude_inv.rotate(los_ned);
    if (los_body.x <= 1.0e-4F) {
        return false;
    }

    const float azimuth_rad = std::atan2(los_body.y, los_body.x);
    const float elevation_rad = std::asin(core::clamp(los_body.z, -1.0F, 1.0F));
    return bearing_rad_to_pixel(azimuth_rad, elevation_rad, pixel_x, pixel_y);
}

core::Vec3 CameraModel::los_unit_body(float azimuth_rad, float elevation_rad) const noexcept {
    const float ca = std::cos(azimuth_rad);
    const float sa = std::sin(azimuth_rad);
    const float ce = std::cos(elevation_rad);
    const float se = std::sin(elevation_rad);
    return core::Vec3{ce * ca, ce * sa, se};
}

core::Vec3 CameraModel::los_unit_ned(float azimuth_rad, float elevation_rad,
                                     const core::Quaternion& attitude) const noexcept {
    return attitude.rotate(los_unit_body(azimuth_rad, elevation_rad));
}

}  // namespace vision
}  // namespace flightsim
