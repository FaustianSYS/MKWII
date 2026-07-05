#include <chrono>
#include <memory>
#include <mutex>

#include "flightsim/fdm/state.hpp"
#include "flightsim/sim/io_adapter.hpp"
#include "flightsim/sim/scheduler.hpp"
#include "flightsim_msgs/msg/aircraft_state.hpp"
#include "flightsim_msgs/msg/control_inputs.hpp"
#include "flightsim_msgs/msg/sim_status.hpp"
#include "rclcpp/rclcpp.hpp"

namespace flightsim {
namespace ros2 {

class Ros2IoAdapter : public sim::IoAdapter {
public:
    Ros2IoAdapter(rclcpp::Node& node, const fdm::ControlInputs& defaults)
        : node_(node), controls_(defaults) {
        control_sub_ = node_.create_subscription<flightsim_msgs::msg::ControlInputs>(
            "control_inputs", rclcpp::QoS(10),
            [this](const flightsim_msgs::msg::ControlInputs::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                controls_.elevator = msg->elevator;
                controls_.aileron = msg->aileron;
                controls_.rudder = msg->rudder;
                controls_.throttle = msg->throttle;
            });

        state_pub_ = node_.create_publisher<flightsim_msgs::msg::AircraftState>("aircraft_state", 10);
        status_pub_ = node_.create_publisher<flightsim_msgs::msg::SimStatus>("sim_status", 10);
    }

    void on_initialize(const sim::SimOutputs& outputs) override { write_outputs(outputs); }

    fdm::ControlInputs read_controls() override {
        std::lock_guard<std::mutex> lock(mutex_);
        return controls_;
    }

    void write_outputs(const sim::SimOutputs& outputs) override {
        flightsim_msgs::msg::AircraftState state_msg{};
        state_msg.header.stamp = node_.get_clock()->now();
        state_msg.header.frame_id = "ned";

        state_msg.position_ned_m = {
            outputs.state.position_ned_m.x,
            outputs.state.position_ned_m.y,
            outputs.state.position_ned_m.z};
        state_msg.velocity_ned_mps = {
            outputs.state.velocity_ned_mps.x,
            outputs.state.velocity_ned_mps.y,
            outputs.state.velocity_ned_mps.z};
        state_msg.attitude_quaternion = {
            outputs.state.attitude.w,
            outputs.state.attitude.x,
            outputs.state.attitude.y,
            outputs.state.attitude.z};
        state_msg.angular_rate_body_rps = {
            outputs.state.angular_rate_body_rps.x,
            outputs.state.angular_rate_body_rps.y,
            outputs.state.angular_rate_body_rps.z};
        state_msg.density_kgm3 = outputs.state.environment.density_kgm3;
        state_msg.throttle = outputs.state.controls.throttle;
        state_msg.step_count = outputs.step_count;

        flightsim_msgs::msg::SimStatus status_msg{};
        status_msg.header.stamp = state_msg.header.stamp;
        status_msg.fault_flags = outputs.faults.raw();
        status_msg.safe_mode = static_cast<std::uint8_t>(outputs.safe_mode);
        status_msg.step_count = outputs.step_count;

        state_pub_->publish(state_msg);
        status_pub_->publish(status_msg);
    }

private:
    rclcpp::Node& node_;
    fdm::ControlInputs controls_;
    std::mutex mutex_;
    rclcpp::Subscription<flightsim_msgs::msg::ControlInputs>::SharedPtr control_sub_;
    rclcpp::Publisher<flightsim_msgs::msg::AircraftState>::SharedPtr state_pub_;
    rclcpp::Publisher<flightsim_msgs::msg::SimStatus>::SharedPtr status_pub_;
};

class FdmNode : public rclcpp::Node {
public:
    FdmNode()
        : Node("flightsim_fdm") {
        dt_sec_ = declare_parameter<double>("dt_sec", 0.01);
        max_thrust_n_ = declare_parameter<double>("max_thrust_n", 8000.0);
        wing_area_m2_ = declare_parameter<double>("wing_area_m2", 16.0);
        default_throttle_ = declare_parameter<double>("default_throttle", 0.6);

        sim::SimConfig config{};
        config.dt_sec = static_cast<float>(dt_sec_);
        config.aircraft.max_thrust_n = static_cast<float>(max_thrust_n_);
        config.aircraft.wing_area_m2 = static_cast<float>(wing_area_m2_);

        scheduler_ = std::make_unique<sim::Scheduler>(config);
        scheduler_->initialize(outputs_);

        fdm::ControlInputs defaults{};
        defaults.throttle = static_cast<float>(default_throttle_);
        io_adapter_ = std::make_unique<Ros2IoAdapter>(*this, defaults);
        io_adapter_->on_initialize(outputs_);

        const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(dt_sec_));
        timer_ = create_wall_timer(period, std::bind(&FdmNode::on_timer, this));

        RCLCPP_INFO(get_logger(), "FlightSim FDM node started (dt=%.4f s)", dt_sec_);
    }

private:
    void on_timer() {
        const fdm::ControlInputs controls = io_adapter_->read_controls();
        scheduler_->step(outputs_, controls);
        io_adapter_->write_outputs(outputs_);
    }

    double dt_sec_{0.01};
    double max_thrust_n_{8000.0};
    double wing_area_m2_{16.0};
    double default_throttle_{0.6};

    sim::SimOutputs outputs_{};
    std::unique_ptr<sim::Scheduler> scheduler_;
    std::unique_ptr<Ros2IoAdapter> io_adapter_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ros2
}  // namespace flightsim

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<flightsim::ros2::FdmNode>());
    rclcpp::shutdown();
    return 0;
}
