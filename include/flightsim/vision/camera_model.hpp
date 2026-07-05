#pragma once

#include "flightsim/core/types.hpp"

namespace flightsim {
namespace vision {

struct CameraModel {
    int width_px{640};
    int height_px{480};
    float fov_azimuth_rad{0.52F};
    float fov_elevation_rad{0.52F};

    void pixel_to_bearing_rad(float pixel_x, float pixel_y, float& azimuth_rad, float& elevation_rad) const noexcept;

    bool bearing_rad_to_pixel(float azimuth_rad, float elevation_rad, float& pixel_x, float& pixel_y) const noexcept;

    bool target_to_pixel(const core::Vec3& missile_pos_ned_m, const core::Quaternion& missile_attitude,
                         const core::Vec3& target_pos_ned_m, float& pixel_x, float& pixel_y) const noexcept;

    core::Vec3 los_unit_body(float azimuth_rad, float elevation_rad) const noexcept;

    core::Vec3 los_unit_ned(float azimuth_rad, float elevation_rad, const core::Quaternion& attitude) const noexcept;
};

}  // namespace vision
}  // namespace flightsim
