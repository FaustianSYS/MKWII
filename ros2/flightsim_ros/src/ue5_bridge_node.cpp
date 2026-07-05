#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "flightsim/core/types.hpp"
#include "flightsim/vision/camera_model.hpp"
#include "flightsim/vision/seeker_image_processor.hpp"
#include "flightsim_msgs/msg/scene_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace flightsim {
namespace ros2 {

namespace {

core::Vec3 from_msg(const std::array<float, 3>& arr) {
    return core::Vec3{arr[0], arr[1], arr[2]};
}

core::Quaternion attitude_from_msg(const std::array<float, 4>& arr) {
    return core::Quaternion{arr[0], arr[1], arr[2], arr[3]};
}

sensor_msgs::msg::CameraInfo make_camera_info(int width_px, int height_px, float fov_rad) {
    sensor_msgs::msg::CameraInfo info{};
    info.width = static_cast<uint32_t>(width_px);
    info.height = static_cast<uint32_t>(height_px);
    info.distortion_model = "plumb_bob";

    const double cx = static_cast<double>(width_px) * 0.5;
    const double cy = static_cast<double>(height_px) * 0.5;
    const double fx = cx / std::tan(static_cast<double>(fov_rad) * 0.5);
    const double fy = cy / std::tan(static_cast<double>(fov_rad) * 0.5);

    info.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
    info.p = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
    info.d = {0.0, 0.0, 0.0, 0.0, 0.0};
    info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    return info;
}

}  // namespace

class Ue5BridgeNode : public rclcpp::Node {
public:
    Ue5BridgeNode()
        : Node("flightsim_ue5_bridge") {
        camera_width_px_ = declare_parameter<int>("camera_width_px", 640);
        camera_height_px_ = declare_parameter<int>("camera_height_px", 480);
        camera_fov_rad_ = declare_parameter<double>("camera_fov_rad", 0.52);
        blob_radius_px_ = declare_parameter<int>("blob_radius_px", 12);
        background_value_ = declare_parameter<int>("background_value", 24);

        camera_.width_px = camera_width_px_;
        camera_.height_px = camera_height_px_;
        camera_.fov_azimuth_rad = static_cast<float>(camera_fov_rad_);
        camera_.fov_elevation_rad = static_cast<float>(camera_fov_rad_);

        image_buffer_.assign(static_cast<size_t>(camera_width_px_) * static_cast<size_t>(camera_height_px_), 0U);
        camera_info_ = make_camera_info(camera_width_px_, camera_height_px_, static_cast<float>(camera_fov_rad_));

        scene_sub_ = create_subscription<flightsim_msgs::msg::SceneState>(
            "scene_state", rclcpp::SensorDataQoS(),
            std::bind(&Ue5BridgeNode::on_scene_state, this, std::placeholders::_1));
        image_pub_ = create_publisher<sensor_msgs::msg::Image>("seeker_camera/image", rclcpp::SensorDataQoS());
        camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("seeker_camera/camera_info", 10);

        RCLCPP_INFO(get_logger(),
                    "UE5 bridge ready (scene sync + seeker camera %dx%d fov=%.2f rad). "
                    "Replace with Unreal when the UE5 ROS plugin is available.",
                    camera_width_px_, camera_height_px_, camera_fov_rad_);
    }

private:
    void on_scene_state(const flightsim_msgs::msg::SceneState& msg) {
        core::Vec3 missile_pos{};
        core::Quaternion missile_attitude = core::Quaternion::identity();
        core::Vec3 shahed_pos{};
        bool has_missile = false;
        bool has_shahed = false;
        std::size_t entity_count = 0U;

        for (const auto& entity : msg.entities) {
            ++entity_count;
            if (entity.type == "missile") {
                missile_pos = from_msg(entity.position_ned_m);
                if (entity.attitude_wxyz.size() >= 4U) {
                    missile_attitude = attitude_from_msg(entity.attitude_wxyz);
                }
                has_missile = true;
            } else if (entity.type == "shahed" || entity.type == "drone") {
                shahed_pos = from_msg(entity.position_ned_m);
                has_shahed = true;
            }
        }

        if (!has_missile) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "scene_state missing missile entity");
            return;
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            sim_step_ = msg.sim_step;
            entity_count_ = entity_count;
            has_scene_ = true;
        }

        if (has_shahed) {
            vision::render_synthetic_seeker_frame(
                image_buffer_.data(), camera_width_px_, camera_height_px_, camera_width_px_, camera_, missile_pos,
                missile_attitude, shahed_pos, static_cast<std::uint8_t>(background_value_), blob_radius_px_, 255U);
        } else {
            const std::uint8_t background = static_cast<std::uint8_t>(background_value_);
            for (auto& value : image_buffer_) {
                value = background;
            }
        }

        sensor_msgs::msg::Image image{};
        image.header = msg.header;
        image.header.frame_id = "missile_seeker";
        image.height = static_cast<uint32_t>(camera_height_px_);
        image.width = static_cast<uint32_t>(camera_width_px_);
        image.encoding = "mono8";
        image.is_bigendian = 0U;
        image.step = static_cast<uint32_t>(camera_width_px_);
        image.data = image_buffer_;

        image_pub_->publish(image);

        camera_info_.header = image.header;
        camera_info_pub_->publish(camera_info_);

        RCLCPP_DEBUG(get_logger(), "Published seeker frame step=%llu entities=%zu missile=(%.1f, %.1f, %.1f)",
                     static_cast<unsigned long long>(msg.sim_step), entity_count, missile_pos.x, missile_pos.y,
                     missile_pos.z);
    }

    int camera_width_px_{640};
    int camera_height_px_{480};
    double camera_fov_rad_{0.52};
    int blob_radius_px_{12};
    int background_value_{24};

    vision::CameraModel camera_{};
    std::vector<std::uint8_t> image_buffer_{};
    sensor_msgs::msg::CameraInfo camera_info_{};

    std::mutex mutex_{};
    bool has_scene_{false};
    std::uint64_t sim_step_{0U};
    std::size_t entity_count_{0U};

    rclcpp::Subscription<flightsim_msgs::msg::SceneState>::SharedPtr scene_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
};

}  // namespace ros2
}  // namespace flightsim

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<flightsim::ros2::Ue5BridgeNode>());
    rclcpp::shutdown();
    return 0;
}
