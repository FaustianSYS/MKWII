#include <chrono>
#include <memory>
#include <vector>

#include "flightsim/engagement/missile/missile_object.hpp"
#include "flightsim/engagement/scenario/air_defense_scenario.hpp"
#include "flightsim/engagement/scenario/scenario.hpp"
#include "flightsim/vision/camera_model.hpp"
#include "flightsim/vision/seeker_image_processor.hpp"
#include "flightsim_msgs/msg/seeker_track.hpp"
#include "flightsim_msgs/msg/missile_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace flightsim {
namespace ros2 {

class VisionNode : public rclcpp::Node {
public:
    VisionNode()
        : Node("flightsim_vision") {
        camera_width_px_ = declare_parameter<int>("camera_width_px", 640);
        camera_height_px_ = declare_parameter<int>("camera_height_px", 480);
        camera_fov_rad_ = declare_parameter<double>("camera_fov_rad", 0.52);
        detection_threshold_ = declare_parameter<int>("detection_threshold", 180);
        default_range_m_ = declare_parameter<double>("default_range_m", 5000.0);

        vision::CameraModel camera{};
        camera.width_px = camera_width_px_;
        camera.height_px = camera_height_px_;
        camera.fov_azimuth_rad = static_cast<float>(camera_fov_rad_);
        camera.fov_elevation_rad = static_cast<float>(camera_fov_rad_);

        vision::SeekerImageProcessorConfig config{};
        config.threshold_value = detection_threshold_;
        config.default_range_m = static_cast<float>(default_range_m_);
        processor_ = std::make_unique<vision::SeekerImageProcessor>(camera, config);

        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "seeker_camera/image", rclcpp::SensorDataQoS(),
            std::bind(&VisionNode::on_image, this, std::placeholders::_1));
        missile_sub_ = create_subscription<flightsim_msgs::msg::MissileState>(
            "missile_state", rclcpp::SensorDataQoS(),
            std::bind(&VisionNode::on_missile, this, std::placeholders::_1));
        track_pub_ = create_publisher<flightsim_msgs::msg::SeekerTrack>("seeker_track", 10);

        RCLCPP_INFO(get_logger(), "Vision node ready (camera=%dx%d fov=%.2f rad)", camera_width_px_, camera_height_px_,
                    camera_fov_rad_);
    }

private:
    void on_missile(const flightsim_msgs::msg::MissileState& msg) {
        missile_attitude_ = core::Quaternion{msg.attitude_w, msg.attitude_x, msg.attitude_y, msg.attitude_z};
        missile_seeker_range_m_ = msg.seeker_range_m;
        has_missile_state_ = true;
    }

    void on_image(const sensor_msgs::msg::Image& msg) {
        if (msg.encoding != "mono8" && msg.encoding != "8UC1") {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Unsupported image encoding: %s",
                                 msg.encoding.c_str());
            return;
        }

        const int width = static_cast<int>(msg.width);
        const int height = static_cast<int>(msg.height);
        const int stride = static_cast<int>(msg.step);
        if (msg.data.empty()) {
            return;
        }

        const auto detection = processor_->detect(msg.data.data(), width, height, stride);
        const double stamp_sec = static_cast<double>(msg.header.stamp.sec) +
                                 (static_cast<double>(msg.header.stamp.nanosec) * 1.0e-9);

        core::Quaternion attitude = core::Quaternion::identity();
        float range_m = static_cast<float>(default_range_m_);
        if (has_missile_state_) {
            attitude = missile_attitude_;
            if (missile_seeker_range_m_ > 0.0F) {
                range_m = missile_seeker_range_m_;
            }
        }

        const vision::SeekerTrack track =
            processor_->track_from_detection(detection, attitude, range_m, stamp_sec);

        core::Vec3 los_rate = core::Vec3{};
        if (detection.detected && prev_detected_) {
            const float dt = static_cast<float>(stamp_sec - prev_stamp_sec_);
            if (dt > 1.0e-4F) {
                float az = 0.0F;
                float el = 0.0F;
                float prev_az = 0.0F;
                float prev_el = 0.0F;
                processor_->camera().pixel_to_bearing_rad(detection.centroid_x_px, detection.centroid_y_px, az, el);
                processor_->camera().pixel_to_bearing_rad(prev_centroid_x_px_, prev_centroid_y_px_, prev_az, prev_el);
                const core::Vec3 los_now = attitude.rotate(processor_->camera().los_unit_body(az, el));
                const core::Vec3 los_prev = attitude.rotate(processor_->camera().los_unit_body(prev_az, prev_el));
                los_rate = (los_now - los_prev) * (1.0F / dt);
            }
        }

        prev_detected_ = detection.detected;
        prev_centroid_x_px_ = detection.centroid_x_px;
        prev_centroid_y_px_ = detection.centroid_y_px;
        prev_stamp_sec_ = stamp_sec;

        flightsim_msgs::msg::SeekerTrack out{};
        out.header.stamp = msg.header.stamp;
        out.header.frame_id = "ned";
        out.valid = track.valid;
        out.locked = track.locked;
        out.range_m = track.range_m;
        out.confidence = track.confidence;
        out.los_unit_ned = {track.los_unit_ned.x, track.los_unit_ned.y, track.los_unit_ned.z};
        out.los_rate_ned = {los_rate.x, los_rate.y, los_rate.z};
        track_pub_->publish(out);
    }

    int camera_width_px_{640};
    int camera_height_px_{480};
    double camera_fov_rad_{0.52};
    int detection_threshold_{180};
    double default_range_m_{5000.0};

    std::unique_ptr<vision::SeekerImageProcessor> processor_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<flightsim_msgs::msg::MissileState>::SharedPtr missile_sub_;
    rclcpp::Publisher<flightsim_msgs::msg::SeekerTrack>::SharedPtr track_pub_;

    core::Quaternion missile_attitude_{core::Quaternion::identity()};
    float missile_seeker_range_m_{0.0F};
    bool has_missile_state_{false};

    bool prev_detected_{false};
    float prev_centroid_x_px_{0.0F};
    float prev_centroid_y_px_{0.0F};
    double prev_stamp_sec_{0.0};
};

}  // namespace ros2
}  // namespace flightsim

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<flightsim::ros2::VisionNode>());
    rclcpp::shutdown();
    return 0;
}
