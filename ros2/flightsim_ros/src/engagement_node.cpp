#include <chrono>
#include <memory>

#include "flightsim/engagement/scenario/air_defense_scenario.hpp"
#include "flightsim/engagement/scenario/scenario.hpp"
#include "flightsim/vision/scene_entity.hpp"
#include "flightsim_msgs/msg/engagement_status.hpp"
#include "flightsim_msgs/msg/missile_state.hpp"
#include "flightsim_msgs/msg/scene_entity.hpp"
#include "flightsim_msgs/msg/scene_state.hpp"
#include "flightsim_msgs/msg/target_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros_seeker_track_source.hpp"

namespace flightsim {
namespace ros2 {

namespace {

flightsim_msgs::msg::SceneEntity to_scene_entity_msg(const vision::SceneEntity& entity) {
    flightsim_msgs::msg::SceneEntity msg{};
    msg.id = entity.id;
    msg.model = entity.model;
    if (entity.type == vision::SceneEntityType::Missile) {
        msg.type = "missile";
    } else if (entity.type == vision::SceneEntityType::Shahed) {
        msg.type = "shahed";
    } else {
        msg.type = "depot";
    }
    msg.position_ned_m = {entity.position_ned_m.x, entity.position_ned_m.y, entity.position_ned_m.z};
    msg.velocity_ned_mps = {entity.velocity_ned_mps.x, entity.velocity_ned_mps.y, entity.velocity_ned_mps.z};
    msg.attitude_wxyz = {entity.attitude.w, entity.attitude.x, entity.attitude.y, entity.attitude.z};
    return msg;
}

}  // namespace

class EngagementNode : public rclcpp::Node {
public:
    EngagementNode()
        : Node("flightsim_engagement") {
        dt_sec_ = declare_parameter<double>("dt_sec", 0.01);
        use_vision_seeker_ = declare_parameter<bool>("use_vision_seeker", false);
        seeker_track_timeout_sec_ = declare_parameter<double>("seeker_track_timeout_sec", 0.25);
        const std::string scenario_name = declare_parameter<std::string>("scenario", "default");

        engagement::ScenarioConfig config{};
        if (scenario_name == "air_defense") {
            config = engagement::default_air_defense_config();
            config.dt_sec = static_cast<float>(dt_sec_);
        } else {
            config.dt_sec = static_cast<float>(dt_sec_);
            config.target.speed_mps = static_cast<float>(declare_parameter<double>("target_speed_mps", 45.0));
            config.target.rng_seed = static_cast<std::uint32_t>(declare_parameter<int>("target_rng_seed", 12345));
            config.missile = engagement::default_missile_attributes();
            config.missile.navigation_gain =
                static_cast<float>(declare_parameter<double>("missile_navigation_gain", 5.0));
            config.missile.max_speed_mps =
                static_cast<float>(declare_parameter<double>("missile_max_speed_mps", 700.0));
            config.missile.max_thrust_n =
                static_cast<float>(declare_parameter<double>("missile_max_thrust_n", 12000.0));
            config.missile.length_m = static_cast<float>(declare_parameter<double>("missile_length_m", 3.66));
            config.missile.burn_time_sec =
                static_cast<float>(declare_parameter<double>("missile_burn_time_sec", 8.0));
            const double seeker_fov = declare_parameter<double>("missile_seeker_fov_azimuth_rad", 0.52);
            config.missile.optical_window.fov_azimuth_rad = static_cast<float>(seeker_fov);
            config.missile.optical_window.fov_elevation_rad = static_cast<float>(seeker_fov);
        }

        config.use_vision_seeker = use_vision_seeker_;

        scenario_ = std::make_unique<engagement::EngagementScenario>(config);
        if (use_vision_seeker_) {
            ros_seeker_source_ = std::make_unique<RosSeekerTrackSource>(*this, seeker_track_timeout_sec_);
            scenario_->set_seeker_source(ros_seeker_source_.get());
        }
        scenario_->initialize();

        target_pub_ = create_publisher<flightsim_msgs::msg::TargetState>("target_state", 10);
        missile_pub_ = create_publisher<flightsim_msgs::msg::MissileState>("missile_state", 10);
        status_pub_ = create_publisher<flightsim_msgs::msg::EngagementStatus>("engagement_status", 10);
        scene_pub_ = create_publisher<flightsim_msgs::msg::SceneState>("scene_state", 10);

        const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(dt_sec_));
        timer_ = create_wall_timer(period, std::bind(&EngagementNode::on_timer, this));

        publish_state();
        RCLCPP_INFO(get_logger(), "Engagement sim started (scenario=%s, vision=%s, dt=%.4f s)",
                    scenario_name.c_str(), use_vision_seeker_ ? "on" : "off", dt_sec_);
    }

private:
    void on_timer() {
        scenario_->step();
        publish_state();

        if (scenario_->state().complete) {
            if (scenario_->state().intercept) {
                RCLCPP_INFO(get_logger(), "Intercept at step %llu, miss_distance=%.2f m",
                            static_cast<unsigned long long>(scenario_->state().step_count),
                            scenario_->state().miss_distance_m);
            } else {
                RCLCPP_WARN(get_logger(), "Engagement ended without intercept (miss=%.2f m)",
                            scenario_->state().miss_distance_m);
            }
            timer_->cancel();
        }
    }

    void publish_state() {
        const auto stamp = get_clock()->now();
        const auto& state = scenario_->state();

        flightsim_msgs::msg::TargetState target_msg{};
        target_msg.header.stamp = stamp;
        target_msg.header.frame_id = "ned";
        target_msg.id = "shahed";
        target_msg.model = "shahed_136";
        target_msg.position_ned_m = {
            state.target.position_ned_m.x,
            state.target.position_ned_m.y,
            state.target.position_ned_m.z};
        target_msg.velocity_ned_mps = {
            state.target.velocity_ned_mps.x,
            state.target.velocity_ned_mps.y,
            state.target.velocity_ned_mps.z};
        target_msg.attitude_wxyz = {
            state.scene.shahed.attitude.w,
            state.scene.shahed.attitude.x,
            state.scene.shahed.attitude.y,
            state.scene.shahed.attitude.z};
        target_pub_->publish(target_msg);

        flightsim_msgs::msg::MissileState missile_msg{};
        missile_msg.header.stamp = stamp;
        missile_msg.header.frame_id = "ned";
        missile_msg.position_ned_m = {
            state.missile.position_ned_m.x,
            state.missile.position_ned_m.y,
            state.missile.position_ned_m.z};
        missile_msg.velocity_ned_mps = {
            state.missile.velocity_ned_mps.x,
            state.missile.velocity_ned_mps.y,
            state.missile.velocity_ned_mps.z};
        missile_msg.active = state.missile.active;
        missile_msg.hit = state.missile.hit;
        missile_msg.thrust_n = state.missile.thrust_n;
        missile_msg.length_m = state.missile.attributes.length_m;
        missile_msg.fin_pitch_rad = state.missile.surfaces.fin_pitch_rad;
        missile_msg.fin_yaw_rad = state.missile.surfaces.fin_yaw_rad;
        missile_msg.fin_roll_rad = state.missile.surfaces.fin_roll_rad;
        missile_msg.seeker_locked = state.missile.seeker_locked;
        missile_msg.seeker_fov_azimuth_rad = state.missile.attributes.optical_window.fov_azimuth_rad;
        missile_msg.seeker_range_m = state.missile.seeker_range_m;
        missile_msg.attitude_w = state.missile.attitude.w;
        missile_msg.attitude_x = state.missile.attitude.x;
        missile_msg.attitude_y = state.missile.attitude.y;
        missile_msg.attitude_z = state.missile.attitude.z;
        missile_msg.pitch_rate_rps = state.missile.angular_rate_body_rps.y;
        missile_msg.yaw_rate_rps = state.missile.angular_rate_body_rps.z;
        missile_msg.roll_rate_rps = state.missile.angular_rate_body_rps.x;
        missile_pub_->publish(missile_msg);

        flightsim_msgs::msg::EngagementStatus status_msg{};
        status_msg.header.stamp = stamp;
        status_msg.range_m = state.miss_distance_m;
        status_msg.miss_distance_m = state.miss_distance_m;
        status_msg.intercept = state.intercept;
        status_msg.complete = state.complete;
        status_msg.step_count = state.step_count;
        status_pub_->publish(status_msg);

        flightsim_msgs::msg::SceneState scene_msg{};
        scene_msg.header.stamp = stamp;
        scene_msg.header.frame_id = "ned";
        scene_msg.sim_step = state.scene.sim_step;
        scene_msg.entities.push_back(to_scene_entity_msg(state.scene.missile));
        scene_msg.entities.push_back(to_scene_entity_msg(state.scene.shahed));
        scene_msg.entities.push_back(to_scene_entity_msg(state.scene.depot));
        scene_pub_->publish(scene_msg);
    }

    double dt_sec_{0.01};
    bool use_vision_seeker_{false};
    double seeker_track_timeout_sec_{0.25};

    std::unique_ptr<engagement::EngagementScenario> scenario_;
    std::unique_ptr<RosSeekerTrackSource> ros_seeker_source_;
    rclcpp::Publisher<flightsim_msgs::msg::TargetState>::SharedPtr target_pub_;
    rclcpp::Publisher<flightsim_msgs::msg::MissileState>::SharedPtr missile_pub_;
    rclcpp::Publisher<flightsim_msgs::msg::EngagementStatus>::SharedPtr status_pub_;
    rclcpp::Publisher<flightsim_msgs::msg::SceneState>::SharedPtr scene_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ros2
}  // namespace flightsim

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<flightsim::ros2::EngagementNode>());
    rclcpp::shutdown();
    return 0;
}
