#include "flightsim/vision/seeker_image_processor.hpp"

#include <cmath>

namespace flightsim {
namespace vision {

namespace {

core::Vec3 normalize_vec(const core::Vec3& v) noexcept {
    const float mag = v.magnitude();
    if (mag < 1.0e-6F) {
        return core::Vec3{};
    }
    return v * (1.0F / mag);
}

}  // namespace

SeekerImageProcessor::SeekerImageProcessor(CameraModel camera, SeekerImageProcessorConfig config) noexcept
    : camera_(camera), config_(config) {}

SeekerDetection SeekerImageProcessor::detect(const std::uint8_t* grayscale, int width, int height,
                                             int stride_bytes) const noexcept {
    SeekerDetection result{};
    if (grayscale == nullptr || width <= 0 || height <= 0 || stride_bytes < width) {
        return result;
    }

    float sum_x = 0.0F;
    float sum_y = 0.0F;
    float count = 0.0F;

    for (int y = 0; y < height; ++y) {
        const std::uint8_t* row = grayscale + (y * stride_bytes);
        for (int x = 0; x < width; ++x) {
            if (row[x] >= static_cast<std::uint8_t>(config_.threshold_value)) {
                sum_x += static_cast<float>(x);
                sum_y += static_cast<float>(y);
                count += 1.0F;
            }
        }
    }

    if (count < config_.min_blob_area_px) {
        return result;
    }

    result.detected = true;
    result.centroid_x_px = sum_x / count;
    result.centroid_y_px = sum_y / count;
    result.area_px = count;
    result.confidence = core::clamp(count / (config_.min_blob_area_px * 4.0F), 0.0F, 1.0F);
    return result;
}

SeekerTrack SeekerImageProcessor::track_from_detection(const SeekerDetection& detection,
                                                     const core::Quaternion& attitude, float range_m,
                                                     double stamp_sec) const noexcept {
    SeekerTrack track{};
    if (!detection.detected) {
        return track;
    }

    float azimuth_rad = 0.0F;
    float elevation_rad = 0.0F;
    camera_.pixel_to_bearing_rad(detection.centroid_x_px, detection.centroid_y_px, azimuth_rad, elevation_rad);

    const core::Vec3 los_body = camera_.los_unit_body(azimuth_rad, elevation_rad);
    const core::Vec3 los_ned = attitude.rotate(los_body);

    track.valid = true;
    track.locked = detection.confidence >= config_.lock_confidence_threshold;
    track.range_m = range_m > 0.0F ? range_m : config_.default_range_m;
    track.los_unit_ned = normalize_vec(los_ned);
    track.los_rate_ned = core::Vec3{};
    track.confidence = detection.confidence;
    track.stamp_sec = stamp_sec;
    return track;
}

void render_synthetic_drone_blob(std::uint8_t* grayscale, int width, int height, int stride_bytes, float pixel_x,
                                 float pixel_y, int radius_px, std::uint8_t intensity) noexcept {
    if (grayscale == nullptr || width <= 0 || height <= 0 || stride_bytes < width || radius_px <= 0) {
        return;
    }

    const int cx = static_cast<int>(pixel_x);
    const int cy = static_cast<int>(pixel_y);
    const int r_sq = radius_px * radius_px;

    for (int y = -radius_px; y <= radius_px; ++y) {
        const int py = cy + y;
        if (py < 0 || py >= height) {
            continue;
        }
        for (int x = -radius_px; x <= radius_px; ++x) {
            if ((x * x) + (y * y) > r_sq) {
                continue;
            }
            const int px = cx + x;
            if (px < 0 || px >= width) {
                continue;
            }
            grayscale[(py * stride_bytes) + px] = intensity;
        }
    }
}

void render_synthetic_seeker_frame(std::uint8_t* grayscale, int width, int height, int stride_bytes,
                                   const CameraModel& camera, const core::Vec3& missile_pos_ned_m,
                                   const core::Quaternion& missile_attitude, const core::Vec3& target_pos_ned_m,
                                   std::uint8_t background, int blob_radius_px, std::uint8_t blob_intensity) noexcept {
    if (grayscale == nullptr || width <= 0 || height <= 0 || stride_bytes < width) {
        return;
    }

    for (int y = 0; y < height; ++y) {
        std::uint8_t* row = grayscale + (y * stride_bytes);
        const std::uint8_t shade = static_cast<std::uint8_t>(
            core::clamp(static_cast<float>(background) + (static_cast<float>(y) / static_cast<float>(height)) * 18.0F,
                        0.0F, 255.0F));
        for (int x = 0; x < width; ++x) {
            row[x] = shade;
        }
    }

    float pixel_x = 0.0F;
    float pixel_y = 0.0F;
    if (!camera.target_to_pixel(missile_pos_ned_m, missile_attitude, target_pos_ned_m, pixel_x, pixel_y)) {
        return;
    }

    const core::Vec3 delta = target_pos_ned_m - missile_pos_ned_m;
    const float range_m = delta.magnitude();
    const float denom = range_m < 50.0F ? 50.0F : range_m;
    const int radius_px = core::clamp(static_cast<int>(8000.0F / denom), 4, blob_radius_px);
    render_synthetic_drone_blob(grayscale, width, height, stride_bytes, pixel_x, pixel_y, radius_px, blob_intensity);
}

}  // namespace vision
}  // namespace flightsim
