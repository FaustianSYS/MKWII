#pragma once

#include <mutex>

#include "flightsim/vision/seeker_source.hpp"
#include "flightsim/vision/seeker_track.hpp"
#include "flightsim_msgs/msg/seeker_track.hpp"
#include "rclcpp/rclcpp.hpp"

namespace flightsim {
namespace ros2 {

class RosSeekerTrackSource final : public vision::ISeekerTrackSource {
public:
    RosSeekerTrackSource(rclcpp::Node& node, double timeout_sec) noexcept;

    bool poll(vision::SeekerTrack& track) noexcept override;
    bool is_live() const noexcept override;

private:
    void on_track(const flightsim_msgs::msg::SeekerTrack& msg) noexcept;

    rclcpp::Subscription<flightsim_msgs::msg::SeekerTrack>::SharedPtr sub_;
    mutable std::mutex mutex_;
    vision::SeekerTrack latest_{};
    double latest_stamp_sec_{0.0};
    double timeout_sec_{0.25};
    bool received_{false};
};

}  // namespace ros2
}  // namespace flightsim
