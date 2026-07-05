#pragma once

#include <cstdint>

#include "flightsim/vision/camera_model.hpp"
#include "flightsim/vision/seeker_track.hpp"

namespace flightsim {
namespace vision {

struct SeekerImageProcessorConfig {
    int threshold_value{180};
    float min_blob_area_px{20.0F};
    float default_range_m{5000.0F};
    float lock_confidence_threshold{0.45F};
};

struct SeekerDetection {
    bool detected{false};
    float centroid_x_px{0.0F};
    float centroid_y_px{0.0F};
    float area_px{0.0F};
    float confidence{0.0F};
};

class SeekerImageProcessor {
public:
    explicit SeekerImageProcessor(CameraModel camera, SeekerImageProcessorConfig config = {}) noexcept;

    SeekerDetection detect(const std::uint8_t* grayscale, int width, int height, int stride_bytes) const noexcept;

    SeekerTrack track_from_detection(const SeekerDetection& detection, const core::Quaternion& attitude,
                                     float range_m, double stamp_sec) const noexcept;

    const CameraModel& camera() const noexcept { return camera_; }

private:
    CameraModel camera_{};
    SeekerImageProcessorConfig config_{};
};

void render_synthetic_drone_blob(std::uint8_t* grayscale, int width, int height, int stride_bytes, float pixel_x,
                                 float pixel_y, int radius_px, std::uint8_t intensity = 255U) noexcept;

void render_synthetic_seeker_frame(std::uint8_t* grayscale, int width, int height, int stride_bytes,
                                   const CameraModel& camera, const core::Vec3& missile_pos_ned_m,
                                   const core::Quaternion& missile_attitude, const core::Vec3& target_pos_ned_m,
                                   std::uint8_t background = 24U, int blob_radius_px = 10,
                                   std::uint8_t blob_intensity = 255U) noexcept;

}  // namespace vision
}  // namespace flightsim
