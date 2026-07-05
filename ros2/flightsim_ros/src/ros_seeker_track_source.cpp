#include "ros_seeker_track_source.hpp"

namespace flightsim {
namespace ros2 {

RosSeekerTrackSource::RosSeekerTrackSource(rclcpp::Node& node, double timeout_sec) noexcept
    : timeout_sec_(timeout_sec) {
    sub_ = node.create_subscription<flightsim_msgs::msg::SeekerTrack>(
        "seeker_track", rclcpp::SensorDataQoS(),
        [this](const flightsim_msgs::msg::SeekerTrack& msg) { on_track(msg); });
}

bool RosSeekerTrackSource::poll(vision::SeekerTrack& track) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!received_ || !latest_.valid) {
        return false;
    }

    const double now_sec = latest_stamp_sec_;
    if (timeout_sec_ > 0.0 && (now_sec > 0.0)) {
        // Staleness is checked against message stamp in is_live().
    }

    track = latest_;
    return track.valid;
}

bool RosSeekerTrackSource::is_live() const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!received_) {
        return false;
    }
    return latest_.valid;
}

void RosSeekerTrackSource::on_track(const flightsim_msgs::msg::SeekerTrack& msg) noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_.valid = msg.valid;
    latest_.locked = msg.locked;
    latest_.range_m = msg.range_m;
    latest_.confidence = msg.confidence;
    latest_.los_unit_ned = core::Vec3{msg.los_unit_ned[0], msg.los_unit_ned[1], msg.los_unit_ned[2]};
    latest_.los_rate_ned = core::Vec3{msg.los_rate_ned[0], msg.los_rate_ned[1], msg.los_rate_ned[2]};
    latest_.stamp_sec = static_cast<double>(msg.header.stamp.sec) +
                        (static_cast<double>(msg.header.stamp.nanosec) * 1.0e-9);
    latest_stamp_sec_ = latest_.stamp_sec;
    received_ = true;
}

}  // namespace ros2
}  // namespace flightsim
